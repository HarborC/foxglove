#include "foxglove/foxglove_utils.h"

namespace foxglove_viz {
Visualizer_fg::Visualizer_fg(const int port_num, const int sleep_time_s) {
  server_.reset(new FoxgloveServer(port_num));
  server_->Run();
  std::cout << "wait " << sleep_time_s << " seconds to run server ..."
            << std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(sleep_time_s));
}

Visualizer_fg::~Visualizer_fg() {
  if (server_)
    server_->Stop();
  std::cout << "visualizer release~" << std::endl;
}

// show Image
void Visualizer_fg::showImage(const std::string &topic_nm, const int64_t &usec,
                              const cv::Mat &viz_img,
                              bool b_send_compressedimg) {
  if (b_send_compressedimg) {
    fg_msg::CompressedImage compressed_img;
    fg::utility::SetMsgTimeStamp(usec, &compressed_img);
    compressed_img.set_frame_id("cam");
    fg::utility::SetImgMsg(viz_img, ".jpeg", &compressed_img);
    server_->SendMessage(topic_nm, usec, compressed_img);
  } else {
    fg_msg::RawImage raw_msg;
    fg::utility::SetMsgTimeStamp(usec, &raw_msg);
    raw_msg.set_frame_id("cam");
    if (fg::utility::SetImgMsg(viz_img, &raw_msg)) {
      server_->SendMessage(topic_nm, usec, raw_msg);
    }
  }
}

// show pose
void Visualizer_fg::showPose(const std::string &topic_nm, const int64_t &usec,
                             const Eigen::Matrix4f &pose,
                             const std::string &parent_frm,
                             const std::string &cur_frm) {
  server_->SendTransform(topic_nm, usec, pose, parent_frm, cur_frm);
}

// show path
void Visualizer_fg::showPath(const std::string &topic_nm, const int64_t &usec,
                             const Eigen::Matrix4f &pose,
                             const std::string &parent_frm) {
  if (path_map_.find(topic_nm) == path_map_.end()) {
    fg_msg::PosesInFrame path_msg;
    path_map_[topic_nm] = path_msg;
  }
  utility::SetMsgTimeStamp(usec, &path_map_[topic_nm]);
  path_map_[topic_nm].set_frame_id(parent_frm);
  auto *cur_pose = path_map_[topic_nm].add_poses();
  utility::Transformation3ToPosOri(pose, cur_pose->mutable_position(),
                                   cur_pose->mutable_orientation());
  server_->SendMessage(topic_nm, usec, path_map_[topic_nm]);
}

void Visualizer_fg::clearPath(const std::string &topic_nm) {
  if (path_map_.find(topic_nm) != path_map_.end()) {
    fg_msg::PosesInFrame path_msg;
    path_map_[topic_nm] = path_msg;
  }
}

} // namespace foxglove_viz
