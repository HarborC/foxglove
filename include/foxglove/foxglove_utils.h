#pragma once

#include "foxglove/foxglove_server.h"
#include "foxglove/proto/FrameTransform.pb.h"
#include "foxglove/proto/PointCloud.pb.h"
#include "foxglove/proto/PoseInFrame.pb.h"
#include "foxglove/proto/PosesInFrame.pb.h"
#include "foxglove/utility.h"

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

class Visualizer_fg {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef std::shared_ptr<Visualizer_fg> Ptr;

public:
  Visualizer_fg(const Visualizer_fg &) = delete;
  Visualizer_fg(const int port_num = 8088, const int sleep_time_s = 5);
  ~Visualizer_fg();

  // show Image
  void showImage(const std::string &topic_nm, const int64_t &usec,
                 const cv::Mat &viz_img, bool b_send_compressedimg = true);

  // show pointcloud
  template <typename PointT>
  void showPointCloud(const std::string &topic_nm, const int64_t &usec,
                      const pcl::PointCloud<PointT> &pcd,
                      const std::vector<std::vector<uint8_t>> &colors,
                      const std::string &parent_frm,
                      const size_t &pc_skip = 1) {
    fg_msg::PointCloud pc_msg;
    fg::utility::SetPointCloudMsgProperties(&pc_msg);
    if (fg::utility::AddPointsToMsg(pcd, pc_skip, colors, &pc_msg)) {
      pc_msg.set_frame_id(parent_frm);
      fg::utility::SetMsgTimeStamp(usec, &pc_msg);
      server_->SendMessage(topic_nm, usec, pc_msg);
    }
  }

  // show pose
  void showPose(const std::string &topic_nm, const int64_t &usec,
                const Eigen::Matrix4f &pose, const std::string &parent_frm,
                const std::string &cur_frm);

  // show path
  void showPath(const std::string &topic_nm, const int64_t &usec,
                const Eigen::Matrix4f &pose, const std::string &parent_frm);

  void clearPath(const std::string &topic_nm);

private:
  std::unordered_map<std::string, fg_msg::PosesInFrame> path_map_;
  std::shared_ptr<FoxgloveServer> server_;
};
} // namespace foxglove_viz
