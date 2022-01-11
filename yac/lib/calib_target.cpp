#include "calib_target.hpp"

namespace yac {

calib_target_t::calib_target_t(const std::string target_type_,
                               const int tag_rows_,
                               const int tag_cols_,
                               const real_t tag_size_,
                               const real_t tag_spacing_)
    : target_type{target_type_}, tag_rows{tag_rows_}, tag_cols{tag_cols_},
      tag_size{tag_size_}, tag_spacing{tag_spacing_} {
}

int calib_target_t::load(const std::string &target_file,
                         const std::string &prefix) {
  config_t config{target_file};
  if (config.ok == false) {
    LOG_ERROR("Failed to load target file [%s]!", target_file.c_str());
    return -1;
  }
  const auto parent = (prefix == "") ? "" : prefix + ".";
  parse(config, parent + "target_type", target_type);
  parse(config, parent + "tag_rows", tag_rows);
  parse(config, parent + "tag_cols", tag_cols);
  parse(config, parent + "tag_size", tag_size);
  parse(config, parent + "tag_spacing", tag_spacing);

  return 0;
}

void calib_target_t::print() const {
  printf("target_type: %s\n", target_type.c_str());
  printf("tag_rows: %d\n", tag_rows);
  printf("tag_cols: %d\n", tag_cols);
  printf("tag_size: %f\n", tag_size);
  printf("tag_spacing: %f\n", tag_spacing);
}

} // namespace yac
