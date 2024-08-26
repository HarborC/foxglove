// taken from https://github.com/foxglove/ws-protocol/blob/main/cpp/examples/example_server.cpp
// for message schema encoding
#pragma once

#include <string>
#include <google/protobuf/descriptor.h>

namespace foxglove_viz {
namespace foxglove {
std::string Base64Encode(const std::string& input);

// Writes the FileDescriptor of this descriptor and all transitive dependencies
// to a string, for use as a channel schema.
std::string SerializeFdSet(const google::protobuf::Descriptor* toplevelDescriptor);

}  // namespace foxglove

}  // namespace foxglove_viz