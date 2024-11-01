#pragma once

#include "google/protobuf/util/time_util.h"

#include <opencv2/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "foxglove/FrameTransform.pb.h"
#include "foxglove/PosesInFrame.pb.h"
#include "foxglove/Color.pb.h"
#include "foxglove/Point3.pb.h"
#include "foxglove/RawImage.pb.h"
#include "foxglove/CompressedImage.pb.h"
#include "foxglove/PointCloud.pb.h"
#include "foxglove/LinePrimitive.pb.h"

#include <chrono>

namespace foxglove_viz {
namespace foxglove {
namespace utility {
inline void Transformation3ToPosOri(const Eigen::Matrix4f& pose,
                                    ::foxglove::Vector3* pos, ::foxglove::Quaternion* rot) {
  const Eigen::Quaternionf quat_in = Eigen::Quaternionf(pose.block<3, 3>(0, 0)).normalized();
  const Eigen::Vector3f& pos_in = pose.block<3, 1>(0, 3);
  pos->set_x(pos_in.x());
  pos->set_y(pos_in.y());
  pos->set_z(pos_in.z());
  rot->set_x(quat_in.x());
  rot->set_y(quat_in.y());
  rot->set_z(quat_in.z());
  rot->set_w(quat_in.w());
}

inline int64_t UsecSinceEpoch() {
  return static_cast<int64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
                               std::chrono::system_clock::now().time_since_epoch())
                               .count());
}

template <typename MsgT>
void SetMsgTimeStamp(const int64_t usec, MsgT* msg) {
  *(msg->mutable_timestamp()) =
      google::protobuf::util::TimeUtil::NanosecondsToTimestamp(usec * 1000l);
}

inline void SetColor(const std::array<float, 4>& rgba, ::foxglove::Color* out) {
  out->set_r(rgba[0]);
  out->set_g(rgba[1]);
  out->set_b(rgba[2]);
  out->set_a(rgba[3]);
}

template <typename ValT>
void SetPoint(const Eigen::Matrix<ValT, 3, 1>& pt, ::foxglove::Point3* out) {
  out->set_x(pt.x());
  out->set_y(pt.y());
  out->set_z(pt.z());
}

template <typename PointT>
void SetPoint(const PointT& pt, ::foxglove::Point3* out) {
  out->set_x(pt.x());
  out->set_y(pt.y());
  out->set_z(pt.z());
}

std::string CompressAsStr(const cv::Mat& img, const std::string& fmt);

void SetImgMsg(const cv::Mat& img, const std::string& fmt, ::foxglove::CompressedImage* msg);

bool SetImgMsg(const cv::Mat& img, ::foxglove::RawImage* msg);

void SetPointCloudMsgProperties(::foxglove::PointCloud* pc_msg, bool no_color = false);

template <typename PointT>
inline float getPtElem(const PointT& pt, const size_t idx) {
  if (idx == 0) {
    return pt.x;
  } else if (idx == 1) {
    return pt.y;
  } else if (idx == 2) {
    return pt.z;
  }
}
template <>
inline float getPtElem<Eigen::Vector3f>(const Eigen::Vector3f& pt, const size_t idx) {
  return pt(static_cast<int>(idx));
}

bool AddColorPointsToMsg(const pcl::PointCloud<pcl::PointXYZRGBA>& raw_pc, const size_t skip_n,
                         ::foxglove::PointCloud* pc_msg, const int new_a = -1);

template <typename PointContainerT>
bool AddPointsToMsg(const PointContainerT& raw_pc, const size_t skip_n,
                    const std::vector<std::vector<uint8_t>>& colors,
                    ::foxglove::PointCloud* pc_msg) {
  if (pc_msg->point_stride() != 16 && pc_msg->point_stride() != 12) {
    return false;
  }
  const bool use_color = (pc_msg->point_stride() == 16);
  if (use_color && (colors.size() != 1 && colors.size() != raw_pc.size())) {
    return false;
  }
  if (!use_color && colors.size() != 0) {
    return false;
  }

  const size_t pt_to_add_num = raw_pc.size();
  const size_t actual_pt_to_add_num = pt_to_add_num / skip_n;
  auto msg_data = pc_msg->mutable_data();
  const size_t offset_start = msg_data->size();
  msg_data->resize(msg_data->size() + actual_pt_to_add_num * pc_msg->point_stride());

  size_t offset_i = offset_start;
  std::vector<char> temp_buf(4);
  auto cp_float_to_msg_data = [&temp_buf, msg_data](const float val, const size_t offset) {
    std::memcpy(temp_buf.data(), &(val), 4);
    (*msg_data)[offset + 0] = temp_buf[0];
    (*msg_data)[offset + 1] = temp_buf[1];
    (*msg_data)[offset + 2] = temp_buf[2];
    (*msg_data)[offset + 3] = temp_buf[3];
  };
  for (size_t pi = 0; pi < raw_pc.size(); pi += skip_n) {
    if (offset_i >= msg_data->size()) {
      break;
    }
    const auto& pt_i = raw_pc.at(pi);
    cp_float_to_msg_data(getPtElem(pt_i, 0), offset_i);
    offset_i += 4;
    cp_float_to_msg_data(getPtElem(pt_i, 1), offset_i);
    offset_i += 4;
    cp_float_to_msg_data(getPtElem(pt_i, 2), offset_i);
    offset_i += 4;

    if (!use_color) {
      continue;
    }
    const auto& color = colors.size() == 1 ? colors.front() : colors[pi];
    (*msg_data)[offset_i++] = static_cast<char>(color[0]);
    (*msg_data)[offset_i++] = static_cast<char>(color[1]);
    (*msg_data)[offset_i++] = static_cast<char>(color[2]);
    (*msg_data)[offset_i++] = static_cast<char>(color[3]);
  }
  return true;
}

template <typename PointContainerT>
void addPointsToLine(const PointContainerT& points, ::foxglove::LinePrimitive* line) {
  for (const auto& pt : points) {
    foxglove_viz::foxglove::utility::SetPoint(pt, line->add_points());
  }
}

template <typename MarkerT>
void setMarkerProps(const ::foxglove::Vector3& size, const ::foxglove::Color& color,
                    const Eigen::Matrix4f& marker_pose, MarkerT* marker) {
  (*marker->mutable_size()) = size;
  (*marker->mutable_color()) = color;
  auto* mpose = marker->mutable_pose();
  Transformation3ToPosOri(marker_pose, mpose->mutable_position(),
                          mpose->mutable_orientation());
}

inline ::foxglove::Vector3 fgVec3(const double s) {
  ::foxglove::Vector3 result;
  result.set_x(s);
  result.set_y(s);
  result.set_z(s);
  return result;
}

inline ::foxglove::Vector3 fgVec3(const double x, const double y, const double z) {
  ::foxglove::Vector3 result;
  result.set_x(x);
  result.set_y(y);
  result.set_z(z);
  return result;
}

inline ::foxglove::Quaternion fgQuat(const double w, const double x, const double y,
                                     const double z) {
  ::foxglove::Quaternion result;
  result.set_w(w);
  result.set_x(x);
  result.set_y(y);
  result.set_z(z);
  return result;
}

void setLineProps(const ::foxglove::LinePrimitive::Type& line_type,
                  const ::foxglove::Color& color, const double thickness,
                  ::foxglove::LinePrimitive* line);

}  // namespace utility

}  // namespace foxglove
}  // namespace foxglove_viz