#include "foxglove/visualizer.h"
#include "foxglove/FrameTransform.pb.h"
#include "foxglove/PointCloud.pb.h"
#include "foxglove/PoseInFrame.pb.h"
#include "foxglove/PosesInFrame.pb.h"
#include "foxglove/CameraCalibration.pb.h"
#include "foxglove/SceneUpdate.pb.h"
#include "foxglove/IMU.pb.h"
#include "foxglove/IMUState.pb.h"
#include "foxglove/Vector3.pb.h"
#include "foxglove/TriangleListPrimitive.pb.h"

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

void Visualizer::showTriangles(const std::string &topic_nm, const int64_t &usec,
                               const std::string &frame_id, const std::vector<Eigen::Vector3f> &points,
                               const std::vector<std::vector<float>> &colors, const std::vector<uint32_t> &indices) {
  fg_msg::SceneUpdate scene_update_msg;
  auto* scene_entity_msg = scene_update_msg.add_entities();
  fg::utility::SetMsgTimeStamp(usec, scene_entity_msg);
  scene_entity_msg->set_frame_id(frame_id);

  scene_entity_msg->set_frame_locked(true);

  auto *triangles_msg = scene_entity_msg->add_triangles();

  auto *triangles_pose = triangles_msg->mutable_pose();
  Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
  utility::Transformation3ToPosOri(pose, triangles_pose->mutable_position(),
                                   triangles_pose->mutable_orientation());

  for (int i = 0; i < points.size(); i++) {
    auto point = points[i];
    auto *point_msg = triangles_msg->add_points();
    point_msg->set_x(point.x());
    point_msg->set_y(point.y());
    point_msg->set_z(point.z());

    auto *color_msg = triangles_msg->add_colors();
    color_msg->set_r(colors[i][0]);
    color_msg->set_g(colors[i][1]);
    color_msg->set_b(colors[i][2]);
    color_msg->set_a(colors[i][3]);
  }

  if (indices.size()) {
    for (const auto &index : indices) {
      triangles_msg->add_indices(index);
    }
  }
  
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

void Visualizer::publishIMU(const std::string &topic_nm, const int64_t &usec,
                            const std::string &frame_id, 
                            const Eigen::Vector3d& acceleration, 
                            const Eigen::Vector3d& angular_velocity, 
                            const Eigen::Quaterniond& orientation, 
                            const Eigen::Vector3d& magnetic_field) {
  fg_msg::IMU imu_msg;
  fg::utility::SetMsgTimeStamp(usec, &imu_msg);
  imu_msg.set_frame_id(frame_id);

  auto *accel = imu_msg.mutable_acceleration();
  accel->set_x(acceleration.x());
  accel->set_y(acceleration.y());
  accel->set_z(acceleration.z());

  auto *gyro = imu_msg.mutable_angular_velocity();
  gyro->set_x(angular_velocity.x());
  gyro->set_y(angular_velocity.y());
  gyro->set_z(angular_velocity.z());

  auto *orien = imu_msg.mutable_orientation();
  orien->set_x(orientation.x());
  orien->set_y(orientation.y());
  orien->set_z(orientation.z());
  orien->set_w(orientation.w());

  auto *magn = imu_msg.mutable_magnetic_field();
  magn->set_x(magnetic_field.x());
  magn->set_y(magnetic_field.y());
  magn->set_z(magnetic_field.z());

  server_->SendMessage(topic_nm, usec, imu_msg);
}

void Visualizer::publishVector3(const std::string &topic_nm, const int64_t &usec, const Eigen::Vector3f& vec) {
  fg_msg::Vector3 vec_msg;

  vec_msg.set_x(vec.x());
  vec_msg.set_y(vec.y());
  vec_msg.set_z(vec.z());

  server_->SendMessage(topic_nm, usec, vec_msg);
}

void Visualizer::publishIMUState(const std::string &topic_nm, const int64_t &usec,
                       const std::string &frame_id, const Eigen::Matrix4f& pose, 
                       const Eigen::Vector3f& velocity, const Eigen::Vector3f& accel_bias, 
                       const Eigen::Vector3f& gyro_bias) {
  fg_msg::IMUState imu_state_msg;
  utility::SetMsgTimeStamp(usec, &imu_state_msg);
                       
  imu_state_msg.set_frame_id(frame_id);
  auto *imu_pose = imu_state_msg.mutable_pose();
  utility::Transformation3ToPosOri(pose, imu_pose->mutable_position(),
                                   imu_pose->mutable_orientation());
                                   
  auto *imu_velocity = imu_state_msg.mutable_velocity();
  imu_velocity->set_x(velocity.x());
  imu_velocity->set_y(velocity.y());
  imu_velocity->set_z(velocity.z());
  
  auto *imu_accel_bias = imu_state_msg.mutable_accel_bias();
  imu_accel_bias->set_x(accel_bias.x());
  imu_accel_bias->set_y(accel_bias.y());
  imu_accel_bias->set_z(accel_bias.z());
  
  auto *imu_gyro_bias = imu_state_msg.mutable_gyro_bias();
  imu_gyro_bias->set_x(gyro_bias.x());
  imu_gyro_bias->set_y(gyro_bias.y());
  imu_gyro_bias->set_z(gyro_bias.z());
  
  server_->SendMessage(topic_nm, usec, imu_state_msg);
}

} // namespace foxglove_viz