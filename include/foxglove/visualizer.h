#pragma once

#include "foxglove/foxglove_server.h"
#include "foxglove/FrameTransform.pb.h"
#include "foxglove/PointCloud.pb.h"
#include "foxglove/PoseInFrame.pb.h"
#include "foxglove/PosesInFrame.pb.h"
#include "foxglove/CameraCalibration.pb.h"
#include "foxglove/utility.h"

#include <pcl/point_types.h>

#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>

namespace fg_msg = ::foxglove;
namespace fg = ::foxglove_viz::foxglove;
using namespace foxglove_viz::foxglove;

namespace foxglove_viz {
class Visualizer {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef std::shared_ptr<Visualizer> Ptr;

public:
  Visualizer() = delete;
  Visualizer(const int port_num = 8088, const int sleep_time_s = -1);

  ~Visualizer();

  // show Image
  void showImage(const std::string &topic_nm, const int64_t &usec,
                 const cv::Mat &viz_img, const std::string &parent_frm, 
                 bool b_send_compressedimg = true);

  void showCameraCalibration(const std::string &topic_nm, const int64_t &usec,
                             const std::string &frame_id, const Eigen::Matrix3f &K, 
                             const int width, const int height, const Eigen::Matrix<float, 3, 4> &P);

  // show pointcloud
  void showPointCloud(const std::string &topic_nm, const int64_t &usec,
                      const std::vector<std::vector<float>> &pcd,
                      const std::vector<std::vector<uint8_t>> &colors,
                      const std::string &parent_frm, const size_t &pc_skip = 1);

  // show pointcloud
  void showPointCloudRGBA(const std::string &topic_nm, const int64_t &usec,
                          const pcl::PointCloud<pcl::PointXYZRGBA> &pcd,
                          const std::string &parent_frm, const size_t &pc_skip = 1, 
                          const int new_a = -1);

  // show pose
  void showPose(const std::string &topic_nm, const int64_t &usec,
                const Eigen::Matrix4f &pose, const std::string &parent_frm,
                const std::string &cur_frm);

  // show path
  void showPath(const std::string &topic_nm, const int64_t &usec,
                const std::vector<Eigen::Matrix4f> &poses,
                const std::string &parent_frm);

private:
  std::shared_ptr<FoxgloveServer> server_;
};
} // namespace foxglove_viz