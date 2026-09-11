#include "mavros_xyz_position_offboard/communication/ground_station_link.hpp"

#include "mavros_xyz_position_offboard/common/artifact_log.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <json/json.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace mavros_xyz_position_offboard::communication
{
namespace
{

bool valid_port(int value) {return value >= 1 && value <= 65535;}

bool valid_ipv4(const std::string & value)
{
  in_addr address{};
  return ::inet_pton(AF_INET, value.c_str(), &address) == 1;
}

bool valid_utf8(const std::string & text)
{
  const auto * bytes = reinterpret_cast<const unsigned char *>(text.data());
  for (std::size_t i = 0; i < text.size();) {
    const unsigned char first = bytes[i];
    if (first <= 0x7fU) {++i; continue;}
    std::size_t length = 0;
    std::uint32_t codepoint = 0;
    if ((first & 0xe0U) == 0xc0U) {length = 2; codepoint = first & 0x1fU;}
    else if ((first & 0xf0U) == 0xe0U) {length = 3; codepoint = first & 0x0fU;}
    else if ((first & 0xf8U) == 0xf0U) {length = 4; codepoint = first & 0x07U;}
    else {return false;}
    if (i + length > text.size()) {return false;}
    for (std::size_t offset = 1; offset < length; ++offset) {
      if ((bytes[i + offset] & 0xc0U) != 0x80U) {return false;}
      codepoint = (codepoint << 6U) | (bytes[i + offset] & 0x3fU);
    }
    if ((length == 2 && codepoint < 0x80U) || (length == 3 && codepoint < 0x800U) ||
      (length == 4 && codepoint < 0x10000U) || codepoint > 0x10ffffU ||
      (codepoint >= 0xd800U && codepoint <= 0xdfffU)) {return false;}
    i += length;
  }
  return true;
}

bool only_members(const Json::Value & value, const std::vector<std::string> & allowed)
{
  for (const auto & name : value.getMemberNames()) {
    bool found = false;
    for (const auto & candidate : allowed) {
      if (name == candidate) {found = true; break;}
    }
    if (!found) {return false;}
  }
  return true;
}

std::string number(double value)
{
  if (!std::isfinite(value)) {throw std::invalid_argument("outgoing JSON number is non-finite");}
  std::ostringstream stream;
  stream << std::setprecision(12) << value;
  return stream.str();
}

std::string optional_number(const std::optional<double> & value)
{
  return value ? number(*value) : "null";
}

std::string ros_header_json(const RosHeader & header)
{
  std::ostringstream stream;
  stream << "{\"stamp\":{\"sec\":" << header.stamp_sec << ",\"nanosec\":" <<
    header.stamp_nanosec << "},\"frame_id\":\"" << common::json_escape(header.frame_id) << "\"}";
  return stream.str();
}

std::string optional_stamp_json(const std::optional<common::RosTimestamp> & stamp)
{
  if (!stamp) {return "null";}
  return "{\"sec\":" + std::to_string(stamp->sec) + ",\"nanosec\":" +
         std::to_string(stamp->nanosec) + "}";
}

bool finite_xyz_status(const XyzStatus & value)
{
  const double numbers[] = {
    value.position_x_m, value.position_y_m, value.yaw_rad, value.front_distance_m,
    value.rear_distance_m, value.left_distance_m, value.right_distance_m,
    value.map_size_x_m, value.map_size_y_m};
  for (const double number_value : numbers) {
    if (!std::isfinite(number_value)) {return false;}
  }
  return !value.position_z_m || std::isfinite(*value.position_z_m);
}

}  // namespace

void GroundStationConfig::validate() const
{
  if (!valid_ipv4(bind_ip) || !valid_ipv4(remote_ip) || !valid_ipv4(whitelist_ip)) {
    throw std::invalid_argument("UDP addresses must be IPv4 literals");
  }
  if (!valid_port(bind_port) || !valid_port(remote_port) || !valid_port(whitelist_port)) {
    throw std::invalid_argument("UDP ports must be within 1..65535");
  }
  if (inbox_capacity == 0U || inbox_capacity > kMaxUdpInboxCapacity) {
    throw std::invalid_argument("UDP inbox capacity must be within 1..4096");
  }
  if (!std::isfinite(message_ttl_s) || message_ttl_s <= 0.0) {
    throw std::invalid_argument("UDP message TTL must be finite and positive");
  }
}

GroundStationLink::GroundStationLink(const GroundStationConfig & config) : config_(config)
{
  config_.validate();
  if (!config_.enabled) {return;}
  socket_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (socket_fd_ < 0) {bind_error_ = std::strerror(errno); return;}
  const int flags = ::fcntl(socket_fd_, F_GETFL, 0);
  if (flags < 0 || ::fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK) < 0) {
    bind_error_ = std::strerror(errno); ::close(socket_fd_); socket_fd_ = -1; return;
  }
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port = htons(static_cast<std::uint16_t>(config_.bind_port));
  ::inet_pton(AF_INET, config_.bind_ip.c_str(), &address.sin_addr);
  if (::bind(socket_fd_, reinterpret_cast<const sockaddr *>(&address), sizeof(address)) < 0) {
    bind_error_ = std::strerror(errno); ::close(socket_fd_); socket_fd_ = -1;
  }
}

GroundStationLink::~GroundStationLink()
{
  if (socket_fd_ >= 0) {::close(socket_fd_);}
}

UdpReceiveResult GroundStationLink::reject(const std::string & reason, double now)
{
  last_rejection_ = reason;
  return UdpReceiveResult{now, std::nullopt, reason};
}

UdpReceiveResult GroundStationLink::decode_datagram(
  const std::string & payload, const std::string & source_ip, int source_port, double now)
{
  if (source_ip != config_.whitelist_ip || source_port != config_.whitelist_port) {
    return reject("source_not_whitelisted", now);
  }
  if (!std::isfinite(now)) {return reject("invalid_receive_time", now);}
  if (!valid_utf8(payload)) {return reject("invalid_utf8", now);}
  Json::CharReaderBuilder builder;
  builder["collectComments"] = false;
  builder["failIfExtra"] = true;
  builder["rejectDupKeys"] = true;
  Json::Value root;
  std::string errors;
  std::istringstream input(payload);
  if (!Json::parseFromStream(builder, input, &root, &errors) || !root.isObject()) {
    return reject("invalid_json", now);
  }
  if (!only_members(root, {"header", "data"}) || !root.isMember("header") ||
    !root["header"].isString() || !root.isMember("data") || !root["data"].isObject()) {
    return reject("invalid_envelope", now);
  }
  const std::string header = root["header"].asString();
  if (header.empty() || header.size() > 64U) {return reject("invalid_header", now);}
  Json::StreamWriterBuilder writer;
  writer["indentation"] = "";
  last_rejection_.clear();
  return UdpReceiveResult{now, UdpMessage{header, Json::writeString(writer, root["data"]), now}, {}};
}

std::vector<UdpReceiveResult> GroundStationLink::poll(double now)
{
  std::vector<UdpReceiveResult> result;
  if (socket_fd_ < 0) {return result;}
  for (int count = 0; count < 64; ++count) {
    char buffer[65536];
    sockaddr_in source{};
    socklen_t source_length = sizeof(source);
    const auto received = ::recvfrom(socket_fd_, buffer, sizeof(buffer), 0,
      reinterpret_cast<sockaddr *>(&source), &source_length);
    if (received < 0) {
      if (errno != EAGAIN && errno != EWOULDBLOCK) {last_rejection_ = std::string("recv_error:") + std::strerror(errno);}
      break;
    }
    char address[INET_ADDRSTRLEN]{};
    ::inet_ntop(AF_INET, &source.sin_addr, address, sizeof(address));
    result.push_back(decode_datagram(
      std::string(buffer, static_cast<std::size_t>(received)), address,
      ntohs(source.sin_port), now));
  }
  return result;
}

std::string GroundStationLink::encode(const UdpMessage & message) const
{
  if (message.header.empty() || message.header.size() > 64U) {
    throw std::invalid_argument("UDP message header must contain 1..64 characters");
  }
  Json::CharReaderBuilder builder;
  builder["collectComments"] = false;
  builder["failIfExtra"] = true;
  builder["rejectDupKeys"] = true;
  Json::Value data;
  std::string errors;
  std::istringstream input(message.data_json);
  if (!Json::parseFromStream(builder, input, &data, &errors) || !data.isObject()) {
    throw std::invalid_argument("UDP message data must be a JSON object");
  }
  Json::StreamWriterBuilder writer;
  writer["indentation"] = "";
  return "{\"header\":\"" + common::json_escape(message.header) +
    "\",\"data\":" + Json::writeString(writer, data) + "}";
}

std::string GroundStationLink::encode_xyzstatus(const XyzStatus & status) const
{
  if (!finite_xyz_status(status)) {throw std::invalid_argument("xyzstatus contains a non-finite LCP field");}
  if (!status.z_valid && (status.position_z_m || status.z_source != "none" || status.z_source_stamp || status.z_quality)) {
    throw std::invalid_argument("invalid Z must use the required null/none representation");
  }
  if (status.z_valid && (!status.position_z_m || status.z_source == "none" || !status.z_source_stamp)) {
    throw std::invalid_argument("valid Z requires value, source, and source stamp");
  }
  std::ostringstream stream;
  stream << "{\"header\":\"xyzstatus\",\"data\":{";
  stream << "\"header\":" << ros_header_json(status.header)
         << ",\"status\":" << static_cast<unsigned int>(status.status)
         << ",\"map_locked\":" << (status.map_locked ? "true" : "false")
         << ",\"pose_valid\":" << (status.pose_valid ? "true" : "false")
         << ",\"position_x_m\":" << number(status.position_x_m)
         << ",\"position_y_m\":" << number(status.position_y_m)
         << ",\"position_z_m\":" << optional_number(status.position_z_m)
         << ",\"z_source\":\"" << common::json_escape(status.z_source) << "\""
         << ",\"z_source_stamp\":" << optional_stamp_json(status.z_source_stamp)
         << ",\"z_quality\":" << optional_number(status.z_quality)
         << ",\"z_valid\":" << (status.z_valid ? "true" : "false")
         << ",\"yaw_rad\":" << number(status.yaw_rad)
         << ",\"front_distance_m\":" << number(status.front_distance_m)
         << ",\"rear_distance_m\":" << number(status.rear_distance_m)
         << ",\"left_distance_m\":" << number(status.left_distance_m)
         << ",\"right_distance_m\":" << number(status.right_distance_m)
         << ",\"map_size_x_m\":" << number(status.map_size_x_m)
         << ",\"map_size_y_m\":" << number(status.map_size_y_m) << "}}";
  return stream.str();
}

bool GroundStationLink::send_json(const std::string & json)
{
  if (socket_fd_ < 0 || json.empty()) {return false;}
  sockaddr_in remote{};
  remote.sin_family = AF_INET;
  remote.sin_port = htons(static_cast<std::uint16_t>(config_.remote_port));
  ::inet_pton(AF_INET, config_.remote_ip.c_str(), &remote.sin_addr);
  const auto sent = ::sendto(socket_fd_, json.data(), json.size(), MSG_DONTWAIT,
    reinterpret_cast<const sockaddr *>(&remote), sizeof(remote));
  return sent == static_cast<ssize_t>(json.size());
}

bool GroundStationLink::send(const UdpMessage & message)
{
  return send_json(encode(message));
}

bool GroundStationLink::send_xyzstatus(const XyzStatus & status)
{
  return send_json(encode_xyzstatus(status));
}

}  // namespace mavros_xyz_position_offboard::communication
