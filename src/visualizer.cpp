#include "foxglove/visualizer.h"

namespace foxglove_viz {

Visualizer::Visualizer(const int port_num, const int sleep_time_s) {
  server_.reset(new FoxgloveServer(port_num));
  server_->Run();
  std::cout << "wait " << sleep_time_s << " seconds to run server ..."
            << std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(sleep_time_s));
}

Visualizer::~Visualizer() {
  if (server_)
    server_->Stop();
  std::cout << "visualizer release~" << std::endl;
}

// show Image
void Visualizer::showImage(const std::string &topic_nm, const int64_t &usec,
                           const cv::Mat &viz_img, bool b_send_compressedimg) {
  if (viz_img.empty()) {
    std::cerr << "Empty image" << std::endl;
    return;
  }

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

// show pointcloud
void Visualizer::showPointCloud(const std::string &topic_nm,
                                const int64_t &usec,
                                const std::vector<std::vector<float>> &pcd,
                                const std::vector<std::vector<uint8_t>> &colors,
                                const std::string &parent_frm,
                                const size_t &pc_skip) {
  pcl::PointCloud<pcl::PointXYZ> pcd_new;
  for (size_t i = 0; i < pcd.size(); i++) {
    pcl::PointXYZ pt;
    pt.x = pcd[i][0];
    pt.y = pcd[i][1];
    pt.z = pcd[i][2];
    pcd_new.push_back(pt);
  }

  std::vector<std::vector<uint8_t>> new_colors;
  if (colors.size() != pcd.size()) {
    new_colors.resize(pcd.size());
    for (size_t i = 0; i < pcd.size(); i++) {
      new_colors[i] = {0, 255, 0, 255};
    }
  } else {
    new_colors = colors;
    for (size_t i = 0; i < colors.size(); i++) {
      new_colors[i].push_back(255);
    }
  }

  fg_msg::PointCloud pc_msg;
  fg::utility::SetPointCloudMsgProperties(&pc_msg);
  if (fg::utility::AddPointsToMsg(pcd_new, pc_skip, new_colors, &pc_msg)) {
    pc_msg.set_frame_id(parent_frm);
    fg::utility::SetMsgTimeStamp(usec, &pc_msg);
    server_->SendMessage(topic_nm, usec, pc_msg);
  }
}

// show pose
void Visualizer::showPose(const std::string &topic_nm, const int64_t &usec,
                          const Eigen::Matrix4f &pose,
                          const std::string &parent_frm,
                          const std::string &cur_frm) {
  server_->SendTransform(topic_nm, usec, pose, parent_frm, cur_frm);
}

// show path
void Visualizer::showPath(const std::string &topic_nm, const int64_t &usec,
                          const std::vector<Eigen::Matrix4f> &poses,
                          const std::string &parent_frm) {
  fg_msg::PosesInFrame path_msg;
  utility::SetMsgTimeStamp(usec, &path_msg);
  path_msg.set_frame_id(parent_frm);

  for (const auto &pose : poses) {
    // add pose to path_msg (PosesInFrame)
    auto *cur_pose = path_msg.add_poses();
    utility::Transformation3ToPosOri(pose, cur_pose->mutable_position(),
                                     cur_pose->mutable_orientation());
  }
  server_->SendMessage(topic_nm, usec, path_msg);
}
} // namespace foxglove_viz