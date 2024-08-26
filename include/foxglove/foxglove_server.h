#pragma once

#include <string>
#include <memory>
#include <thread>
#include <atomic>
#include <map>
#include <future>
#include <chrono>
#include <shared_mutex>

#include "foxglove/server_common.h"
#include "foxglove/proto_utility.h"
#include "foxglove/utility.h"

namespace foxglove {
namespace websocketpp {
class Server;
using ServerPtr = std::unique_ptr<Server>;
}  // namespace websocketpp
}  // namespace foxglove

namespace foxglove_viz {
namespace foxglove {
using ChannelId = ::foxglove::websocketpp::ChannelId;
using ChannelInfo = ::foxglove::websocketpp::ChannelWithoutId;
class FoxgloveServer {
 public:
  FoxgloveServer(const int port_num = 8766,
                 const std::string& server_name = std::string("foxglove_server"));
  FoxgloveServer(const FoxgloveServer&) = delete;
  ~FoxgloveServer();

  void Run();
  void Stop();

  inline bool IsRunning() const { return server_running_.load(); }
  inline size_t NumChannels() const {
    std::shared_lock lock(channel_mutex_);
    return ch_nm_to_id_.size();
  }
  inline bool HasChannel(const std::string& ch_nm) const {
    std::shared_lock lock(channel_mutex_);
    return ch_nm_to_id_.find(ch_nm) != ch_nm_to_id_.end();
  }
  inline ChannelId GetChannelId(const std::string& ch_nm) const {
    std::shared_lock lock(channel_mutex_);
    return ch_nm_to_id_.at(ch_nm);
  }

  template <typename MsgT>
  bool AddChannel(const std::string& topic);

  // sending messages
  template <typename MsgT>
  void SendMessage(const std::string& topic_nm, const int64_t& usec, const MsgT& msg);
  bool SendTransform(const std::string& topic_nm, const int64_t& usec,
                     const Eigen::Matrix4f& pose, const std::string& parent_frm,
                     const std::string& cur_frm);
  bool SendPosesInFrame(const std::string& topic_nm, const int64_t& usec,
                        ::foxglove::PosesInFrame& posesInFrame_msg,
                        const Eigen::Matrix4f& pose,
                        const std::string& parent_frm);
  template <typename PointContainerT>
  bool SendPointCloud(const std::string& topic_nm, const int64_t& usec, const size_t skip_n,
                      const std::vector<std::array<uchar, 4>>& colors,
                      const PointContainerT& pc_viz, const std::string& parent_frm);
  void DeleteAllSceneEntity(const std::string& topic_nm, const int wait_ms = 50);

 private:
  // avoid expose server.hpp
  ChannelId AddChannelInternal(const ChannelInfo& info);
  void SendMsgInternal(const ChannelId ch_id, const int64_t nsec, const std::string& str);

  ::foxglove::websocketpp::ServerPtr server_internal_;

  // internal status
  std::map<std::string, ChannelId> ch_nm_to_id_{};
  std::thread thread_;
  std::atomic_bool server_running_{false};
  mutable std::shared_mutex channel_mutex_;
};

template <typename MsgT>
bool FoxgloveServer::AddChannel(const std::string& topic) {
  if (HasChannel(topic)) {
    return false;
  }
  ChannelInfo channel{topic, "protobuf", MsgT::descriptor()->full_name(),
                      Base64Encode(SerializeFdSet(MsgT::descriptor()))};
  AddChannelInternal(channel);
  return true;
}

template <typename MsgT>
void FoxgloveServer::SendMessage(const std::string& topic_nm, const int64_t& usec,
                                 const MsgT& msg) {
  AddChannel<MsgT>(topic_nm);
  const ChannelId ch_id = GetChannelId(topic_nm);
  auto str = msg.SerializeAsString();
  this->SendMsgInternal(ch_id, usec * 1000l, str);
}

template <typename PointContainerT>
bool FoxgloveServer::SendPointCloud(const std::string& topic_nm, const int64_t& usec,
                                    const size_t skip_n,
                                    const std::vector<std::array<uchar, 4>>& colors,
                                    const PointContainerT& pc_viz,
                                    const std::string& parent_frm) {
  ::foxglove::PointCloud pc_msg;

  utility::SetPointCloudMsgProperties(&pc_msg);
  bool res = utility::AddPointsToMsg(pc_viz, skip_n, colors, &pc_msg);
  if (res) {
    pc_msg.set_frame_id(parent_frm);
    utility::SetMsgTimeStamp(usec, &pc_msg);
    SendMessage(topic_nm, usec, pc_msg);
  }

  return true;
}

}  // namespace foxglove
}  // namespace foxglove_viz