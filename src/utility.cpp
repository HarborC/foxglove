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

}  // namespace utility

}  // namespace foxglove

}  // namespace foxglove_viz