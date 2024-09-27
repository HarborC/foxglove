#include "foxglove/foxglove_server.h"
#include "foxglove/server.hpp"
#include "foxglove/utility.h"

#include <thread>

#include "google/protobuf/util/time_util.h"
#include "foxglove/FrameTransform.pb.h"
#include "foxglove/SceneUpdate.pb.h"

using namespace foxglove::websocketpp;

namespace foxglove_viz {
namespace foxglove {
FoxgloveServer::FoxgloveServer(const int port_num, const std::string& server_name)
    : server_internal_(std::make_unique<Server>(port_num, server_name)) {
#if LOG_CLIENT
#ifdef ASIO_STANDALONE
  std::cout << "FoxgloveServer use standalone ASIO" << std::endl;
#else
  std::cout << "FoxgloveServer use Boost ASIO" << std::endl;
#endif
#endif
  server_internal_->setSubscribeHandler(
      [&](ChannelId chanId) { 
#if LOG_CLIENT
        std::cout << "first client subscribed to " << chanId << std::endl; 
#endif
        });
  server_internal_->setUnsubscribeHandler(
      [&](ChannelId chanId) { 
#if LOG_CLIENT
      std::cout << "last client unsubscribed from " << chanId << std::endl; 
#endif
});
}

FoxgloveServer::~FoxgloveServer() { this->Stop(); };

void FoxgloveServer::Run() {
  if (server_running_.load()) {
#if LOG_CLIENT
    std::cout << "Server is already running." << std::endl;
#endif
    return;
  }
  thread_ = std::thread(&Server::run, server_internal_.get());
  server_running_.store(true);
}

void FoxgloveServer::Stop() {
  if (!server_running_.load()) {
    std::cout << "Server is not running." << std::endl;
    return;
  }
  server_internal_->stop();
  thread_.join();
  server_running_.store(false);
}

ChannelId FoxgloveServer::AddChannelInternal(const ChannelInfo& info) {
  std::unique_lock lock(channel_mutex_);
  ChannelId res = server_internal_->addChannel(info);
  ch_nm_to_id_.emplace(std::make_pair(info.topic, res));
  return res;
}

void FoxgloveServer::SendMsgInternal(const ChannelId ch_id, const int64_t nsec,
                                     const std::string& msg_str) {
  this->server_internal_->sendMessage(ch_id, static_cast<uint64_t>(nsec), msg_str);
}

bool FoxgloveServer::SendTransform(const std::string& topic_nm, const int64_t& usec,
                                   const Eigen::Matrix4f& pose,
                                   const std::string& parent_frm, const std::string& cur_frm) {
  ::foxglove::FrameTransform tf_msg;

  utility::SetMsgTimeStamp(usec, &tf_msg);

  tf_msg.set_parent_frame_id(parent_frm);
  tf_msg.set_child_frame_id(cur_frm);

  utility::Transformation3ToPosOri(pose, tf_msg.mutable_translation(),
                                   tf_msg.mutable_rotation());

  SendMessage(topic_nm, usec, tf_msg);

  return true;
}

bool FoxgloveServer::SendPosesInFrame(const std::string& topic_nm, const int64_t& usec,
                                      ::foxglove::PosesInFrame& posesInFrame_msg,
                                      const Eigen::Matrix4f& pose,
                                      const std::string& parent_frm) {
  utility::SetMsgTimeStamp(usec, &posesInFrame_msg);
  posesInFrame_msg.set_frame_id(parent_frm);
  auto* cur_pose = posesInFrame_msg.add_poses();
  utility::Transformation3ToPosOri(pose, cur_pose->mutable_position(),
                                   cur_pose->mutable_orientation());
  SendMessage(topic_nm, usec, posesInFrame_msg);

  return true;
}

void FoxgloveServer::DeleteAllSceneEntity(const std::string& topic_nm, const int wait_ms) {
  this->AddChannel<::foxglove::SceneUpdate>(topic_nm);
  std::this_thread::sleep_for(std::chrono::milliseconds(wait_ms));
  ::foxglove::SceneUpdate del_msg;
  auto* del = del_msg.add_deletions();
  utility::SetMsgTimeStamp(0l, del);
  del->set_type(::foxglove::SceneEntityDeletion::ALL);
  this->SendMessage(topic_nm, 0l, del_msg);
}

}  // namespace foxglove

}  // namespace foxglove_viz