function [retval, aprilgrid] = load_aprilgrid(data_path)
  % Format string
  % -- Target properties
  fmt_str = "%d %d %d %f %f ";  % ts, tag_rows, tag_cols, tag_size, tag_spacing
  fmt_str = strcat(fmt_str, "%f %f ");    % Keypoints
  fmt_str = strcat(fmt_str, "%f %f %f "); % Object points
  fmt_str = strcat(fmt_str, "%d %d");     % Tag id, corner idx

  % Parse file
  fid = fopen(data_path, "r");
  % -- Check number of lines
  nb_lines = fskipl(fid, Inf);
  if nb_lines < 2
    retval = -1;
    aprilgrid = {};
    return;
  endif
  % -- Parse file
  frewind(fid);
  csv_data = textscan(fid, fmt_str, "delimiter", ",", "headerlines", 1);
  fclose(fid);

  # AprilGrid
  # -- Target properties
  aprilgrid.ts = csv_data{1}(1);
  aprilgrid.tag_rows = csv_data{2}(1);
  aprilgrid.tag_cols = csv_data{3}(1);
  aprilgrid.tag_size = csv_data{4}(1);
  aprilgrid.tag_spacing = csv_data{5}(1);
  # -- Keypoint
  kp_x = csv_data{6}(1:end);
  kp_y = csv_data{7}(1:end);
  aprilgrid.keypoints = transpose([kp_x, kp_y]);
  # -- Object points
  p_x = csv_data{8}(1:end);
  p_y = csv_data{9}(1:end);
  p_z = csv_data{10}(1:end);
  aprilgrid.object_points = transpose([p_x, p_y, p_z]);
  # -- Tag ids
  aprilgrid.tag_ids = csv_data{11}(1:end)';
  # -- Corner indicies
  aprilgrid.corner_indicies = csv_data{12}(1:end)';

  retval = 0;
endfunction
