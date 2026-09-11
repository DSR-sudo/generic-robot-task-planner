#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include "mavros_xyz_position_offboard/communication/ground_station_link.hpp"

namespace
{
using mavros_xyz_position_offboard::communication::GroundStationConfig;
using mavros_xyz_position_offboard::communication::GroundStationLink;
using mavros_xyz_position_offboard::communication::UdpMessage;
using mavros_xyz_position_offboard::communication::kMaxUdpInboxCapacity;

GroundStationConfig test_config()
{
  GroundStationConfig config;
  config.enabled = false;
  config.bind_ip = "127.0.0.1";
  config.remote_ip = "127.0.0.1";
  config.whitelist_ip = "127.0.0.1";
  config.inbox_capacity = 2;
  config.message_ttl_s = 0.5;
  return config;
}

TEST(GenericUdpTest, StrictEnvelopeAndWhitelist)
{
  GroundStationLink link(test_config());
  const auto accepted = link.decode_datagram(
    R"({"header":"sample","data":{"value":1,"text":"ok"}})",
    "127.0.0.1", 5005, 10.0);
  ASSERT_TRUE(accepted.message.has_value());
  EXPECT_EQ(accepted.message->header, "sample");
  EXPECT_NE(accepted.message->data_json.find("\"value\""), std::string::npos);

  EXPECT_FALSE(link.decode_datagram(
    R"({"header":"sample","data":[]})", "127.0.0.1", 5005, 10.0).message);
  EXPECT_FALSE(link.decode_datagram(
    R"({"header":"sample","data":{},"extra":1})", "127.0.0.1", 5005, 10.0).message);
  EXPECT_FALSE(link.decode_datagram(
    R"({"header":"sample","data":{}})", "10.0.0.2", 5005, 10.0).message);
  EXPECT_EQ(link.last_rejection(), "source_not_whitelisted");
}

TEST(GenericUdpTest, GenericEncodeRejectsInvalidDataAndRetainsArbitraryHeaders)
{
  GroundStationLink link(test_config());
  const auto encoded = link.encode({"future_action", R"({"enabled":true,"count":2})", 0.0});
  EXPECT_NE(encoded.find(R"("header":"future_action")"), std::string::npos);
  EXPECT_NE(encoded.find(R"("enabled":true)"), std::string::npos);
  EXPECT_THROW(link.encode(UdpMessage{"", "{}", 0.0}), std::invalid_argument);
  EXPECT_THROW(link.encode(UdpMessage{"bad", "[]", 0.0}), std::invalid_argument);
  EXPECT_FALSE(link.send({"future_action", "{}", 0.0}));
}

TEST(GenericUdpTest, EndpointAndInboxConfigurationAreBounded)
{
  auto config = test_config();
  config.bind_port = 0;
  EXPECT_THROW({ GroundStationLink link(config); }, std::invalid_argument);
  config = test_config();
  config.inbox_capacity = 0;
  EXPECT_THROW({ GroundStationLink link(config); }, std::invalid_argument);
  config = test_config();
  config.inbox_capacity = kMaxUdpInboxCapacity + 1U;
  EXPECT_THROW({ GroundStationLink link(config); }, std::invalid_argument);
  config = test_config();
  config.message_ttl_s = NAN;
  EXPECT_THROW({ GroundStationLink link(config); }, std::invalid_argument);
}

}  // namespace
