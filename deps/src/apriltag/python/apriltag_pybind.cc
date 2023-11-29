#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <AprilTags/TagDetector.h>
#include <AprilTags/Tag36h11.h>

namespace py = pybind11;

py::list detect(py::array_t<uint8_t> &image_nparray) {
  // Form Image - np.array to cv::Mat
  py::buffer_info buf = image_nparray.request();
  const cv::Mat image(buf.shape[0],
                      buf.shape[1],
                      CV_8UC1,
                      (unsigned char *) buf.ptr);

  // Extract tags
  AprilTags::AprilGridDetector detector;
  std::vector<AprilTags::TagDetection> tags = detector.extractTags(image);

  // Sort by tag_id (inorder)
  std::sort(tags.begin(),
            tags.end(),
            [](const AprilTags::TagDetection &a,
               const AprilTags::TagDetection &b) { return (a.id < b.id); });

  // Setup return data
  py::list detections;
  for (const auto &tag : tags) {
    if (!tag.good) {
      continue;
    }

    for (int corner_idx = 0; corner_idx < 4; corner_idx++) {
      py::list tag_data;
      tag_data.append(tag.id);
      tag_data.append(corner_idx);
      tag_data.append(tag.p[corner_idx].first);
      tag_data.append(tag.p[corner_idx].second);
      detections.append(tag_data);
    }
  }

  return detections; // List[List[TagId, CornerIndex, KeypointX, KeypointY]]
}

PYBIND11_MODULE(apriltag_pybind, m) {
  m.doc() = "AprilTag Python Bindings";
  m.def("detect", &detect);
}
