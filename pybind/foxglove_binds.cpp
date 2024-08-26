#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
#include <pybind11/eigen.h>

#include <pcl/point_types.h>

#include "foxglove/visualizer.h"
#include "ndarray_converter.h"

namespace py = pybind11;

PYBIND11_MODULE(pyfoxglove, m) {
  NDArrayConverter::init_numpy();

  using namespace pybind11::literals;
  m.doc() = "python interface for foxglove";

  using namespace foxglove_viz;

  py::class_<Visualizer>(m, "Visualizer")
      .def(py::init<int, int>())
      .def("show_image", &Visualizer::showImage)
      .def("show_pointcloud", &Visualizer::showPointCloud)
      .def("show_pose", &Visualizer::showPose)
      .def("show_path", &Visualizer::showPath);
}