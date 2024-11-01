#include "foxglove/utility.h"

#include "opencv2/imgcodecs.hpp"

namespace foxglove_viz {
namespace foxglove {
namespace utility {

std::string CompressAsStr(const cv::Mat& img, const std::string& fmt) {
  std::vector<uchar> enc_buf;
  cv::imencode(fmt, img, enc_buf);
  return std::string(std::make_move_iterator(enc_buf.begin()),
                     std::make_move_iterator(enc_buf.end()));
}

void SetImgMsg(const cv::Mat& img, const std::string& fmt, ::foxglove::CompressedImage* msg) {
  (*msg->mutable_data()) = CompressAsStr(img, fmt);
  msg->set_format(fmt);
}

bool SetImgMsg(const cv::Mat& img, ::foxglove::RawImage* msg) {
  if (img.type() != CV_8UC1 && img.type() != CV_8UC3) {
    std::cout <<"SetImgMsg: image type is not supported" << std::endl;
    return false;
  }
  msg->set_width(img.cols);
  msg->set_height(img.rows);
  if (img.type() == CV_8UC3) {
    msg->set_encoding("8UC3");
  } else if (img.type() == CV_8UC1) {
    msg->set_encoding("8UC1");
  }
  msg->set_step(img.elemSize() * img.cols);
  if (img.isContinuous()) {
    msg->set_data(img.data, img.total() * img.elemSize());
  } else {
    std::cout <<"SetImgMsg: uncontinuous image is not supported" << std::endl;
    return false;
  }

  return true;
}

void SetPointCloudMsgProperties(::foxglove::PointCloud* pc_msg, bool no_color) {
  pc_msg->set_point_stride(no_color ? 12 : 12 + 4);
  auto add_single_field = [&pc_msg](const std::string& name, const int offset,
                                    ::foxglove::PackedElementField::NumericType num_type) {
    auto p = pc_msg->mutable_fields()->Add();
    p->set_name(name);
    p->set_offset(offset);
    p->set_type(num_type);
  };
  add_single_field("x", 0, ::foxglove::PackedElementField::FLOAT32);
  add_single_field("y", 4, ::foxglove::PackedElementField::FLOAT32);
  add_single_field("z", 8, ::foxglove::PackedElementField::FLOAT32);
  if (no_color) {
    return;
  }

  add_single_field("red", 12, ::foxglove::PackedElementField::UINT8);
  add_single_field("green", 13, ::foxglove::PackedElementField::UINT8);
  add_single_field("blue", 14, ::foxglove::PackedElementField::UINT8);
  add_single_field("alpha", 15, ::foxglove::PackedElementField::UINT8);
}

void setLineProps(const ::foxglove::LinePrimitive::Type& line_type,
                  const ::foxglove::Color& color, const double thickness,
                  ::foxglove::LinePrimitive* line) {
  line->set_type(line_type);
  (*line->mutable_color()) = color;
  line->set_thickness(thickness);
}

bool AddColorPointsToMsg(const pcl::PointCloud<pcl::PointXYZRGBA>& raw_pc, const size_t skip_n,
                         ::foxglove::PointCloud* pc_msg, const int new_a) {
  if (pc_msg->point_stride() != 16 && pc_msg->point_stride() != 12) {
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

  if (new_a >= 0) {
    char new_a_char = static_cast<char>(new_a);
    for (size_t pi = 0; pi < raw_pc.size(); pi += skip_n) {
      if (offset_i >= msg_data->size()) {
        break;
      }
      const auto& pt_i = raw_pc.at(pi);
      cp_float_to_msg_data(pt_i.x, offset_i);
      offset_i += 4;
      cp_float_to_msg_data(pt_i.y, offset_i);
      offset_i += 4;
      cp_float_to_msg_data(pt_i.z, offset_i);
      offset_i += 4;

      (*msg_data)[offset_i++] = static_cast<char>(pt_i.r);
      (*msg_data)[offset_i++] = static_cast<char>(pt_i.g);
      (*msg_data)[offset_i++] = static_cast<char>(pt_i.b);
      (*msg_data)[offset_i++] = new_a_char;
    }
  } else {
    for (size_t pi = 0; pi < raw_pc.size(); pi += skip_n) {
      if (offset_i >= msg_data->size()) {
        break;
      }
      const auto& pt_i = raw_pc.at(pi);
      cp_float_to_msg_data(pt_i.x, offset_i);
      offset_i += 4;
      cp_float_to_msg_data(pt_i.y, offset_i);
      offset_i += 4;
      cp_float_to_msg_data(pt_i.z, offset_i);
      offset_i += 4;

      (*msg_data)[offset_i++] = static_cast<char>(pt_i.r);
      (*msg_data)[offset_i++] = static_cast<char>(pt_i.g);
      (*msg_data)[offset_i++] = static_cast<char>(pt_i.b);
      (*msg_data)[offset_i++] = static_cast<char>(pt_i.a);
    }
  }
  return true;
}

}  // namespace utility

}  // namespace foxglove

}  // namespace foxglove_viz