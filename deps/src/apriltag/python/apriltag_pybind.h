#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <opencv2/opencv.hpp>

namespace py = pybind11;

void cpp_imshow(const std::string& title, const py::array_t<uint8_t>& img) {
  py::buffer_info buf = img.request();
  cv::Mat mat(buf.shape[0], buf.shape[1], CV_8UC3, (unsigned char*)buf.ptr);
  cv::imshow(title, mat);
}

PYBIND11_MODULE(apriltag, m) {
  m.doc() = "AprilTag";
  m.def("cpp_imshow", &cpp_imshow);
}
