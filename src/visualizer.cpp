#include "foxglove/visualizer.h"
#include <omp.h>

namespace foxglove_viz {

Visualizer::Visualizer(const int port_num, const int sleep_time_s) {
  server_.reset(new FoxgloveServer(port_num));
  server_->Run();

  if (sleep_time_s >= 0) {
    std::cout << "wait " << sleep_time_s << " seconds to run server ..."
            << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(sleep_time_s));
  }
}

Visualizer::~Visualizer() {
  if (server_)
    server_->Stop();
}

// show Image
void Visualizer::showImage(const std::string &topic_nm, const int64_t &usec,
                           const cv::Mat &viz_img, const std::string &parent_frm, 
                           bool b_send_compressedimg) {
  if (viz_img.empty()) {
    std::cerr << "Empty image" << std::endl;
    return;
  }

  if (b_send_compressedimg) {
    fg_msg::CompressedImage compressed_img;
    fg::utility::SetMsgTimeStamp(usec, &compressed_img);
    compressed_img.set_frame_id(parent_frm);
    fg::utility::SetImgMsg(viz_img, ".jpeg", &compressed_img);
    server_->SendMessage(topic_nm, usec, compressed_img);
  } else {
    fg_msg::RawImage raw_msg;
    fg::utility::SetMsgTimeStamp(usec, &raw_msg);
    raw_msg.set_frame_id(parent_frm);
    if (fg::utility::SetImgMsg(viz_img, &raw_msg)) {
      server_->SendMessage(topic_nm, usec, raw_msg);
    }
  }
}

void Visualizer::showCameraCalibration(const std::string &topic_nm, const int64_t &usec,
                                       const std::string &frame_id, const Eigen::Matrix3f &K, 
                                       const int width, const int height, const Eigen::Matrix<float, 3, 4> &P) {

  fg_msg::CameraCalibration cam_calib_msg;
  fg::utility::SetMsgTimeStamp(usec, &cam_calib_msg);
  cam_calib_msg.set_frame_id(frame_id);
  cam_calib_msg.set_width(width);
  cam_calib_msg.set_height(height);
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      cam_calib_msg.add_k(K(i,j));
    }
  }

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 4; j++) {
      cam_calib_msg.add_p(P(i,j));
    }
  }

  server_->SendMessage(topic_nm, usec, cam_calib_msg);
}

void Visualizer::show3DModel(const std::string &topic_nm, const int64_t &usec,
                             const std::string &frame_id, const std::string url,
                             const Eigen::Matrix4f &pose, const Eigen::Vector3f &scale) {
  fg_msg::SceneUpdate scene_update_msg;
  auto* scene_entity_msg = scene_update_msg.add_entities();
  fg::utility::SetMsgTimeStamp(usec, scene_entity_msg);
  scene_entity_msg->set_frame_id(frame_id);

  std::string id = "body";
  scene_entity_msg->set_frame_locked(true);

  auto *model_msg = scene_entity_msg->add_models();
  auto *model_pose = model_msg->mutable_pose();
  utility::Transformation3ToPosOri(pose, model_pose->mutable_position(),
                                   model_pose->mutable_orientation());

  auto *model_scale = model_msg->mutable_scale();
  model_scale->set_x(scale.x());
  model_scale->set_y(scale.y());
  model_scale->set_z(scale.z());

  model_msg->set_url(url);
  model_msg->set_override_color(false);

  server_->SendMessage(topic_nm, usec, scene_update_msg);
}

// show pointcloud
void Visualizer::showPointCloud(const std::string &topic_nm,
                                const int64_t &usec,
                                const std::vector<std::vector<float>> &pcd,
                                const std::vector<std::vector<uint8_t>> &colors,
                                const std::string &parent_frm,
                                const size_t &pc_skip) {
  pcl::PointCloud<pcl::PointXYZ> pcd_new;
  pcd_new.resize(pcd.size());

  #pragma omp parallel for
  for (size_t i = 0; i < pcd.size(); i++) {
    pcl::PointXYZ& pt = pcd_new.points[i];
    pt.x = pcd[i][0];
    pt.y = pcd[i][1];
    pt.z = pcd[i][2];
  }

  std::vector<std::vector<uint8_t>> new_colors;
  if (colors.size() != pcd.size()) {
    new_colors.resize(pcd.size(), {0, 255, 0, 255});
  } else {
    new_colors = colors;
    #pragma omp parallel for
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

void Visualizer::showPointCloudRGBA(const std::string &topic_nm, const int64_t &usec,
                                    const pcl::PointCloud<pcl::PointXYZRGBA> &pcd,
                                    const std::string &parent_frm, const size_t &pc_skip,
                                    const int new_a) {
  fg_msg::PointCloud pc_msg;
  fg::utility::SetPointCloudMsgProperties(&pc_msg);
  if (fg::utility::AddColorPointsToMsg(pcd, pc_skip, &pc_msg, new_a)) {
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