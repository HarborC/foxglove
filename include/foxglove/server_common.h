#pragma once

#include <string>

#define LOG_CLIENT 0

// server common
namespace foxglove {
namespace websocketpp {
using ChannelId = uint32_t;
struct ChannelWithoutId {
  std::string topic;
  std::string encoding;
  std::string schemaName;
  std::string schema;

  inline bool operator==(const ChannelWithoutId& other) const {
    return topic == other.topic && encoding == other.encoding &&
           schemaName == other.schemaName && schema == other.schema;
  }
};

}  // namespace websocketpp
}  // namespace foxglove