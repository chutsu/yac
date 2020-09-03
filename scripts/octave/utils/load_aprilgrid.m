function [retval, aprilgrid] = load_aprilgrid(data_path)
  % Format string
  % -- Target properties
  fmt_str = "%d %d %d %f %f ";
  % -- Observed keypoints
  fmt_str = strcat(fmt_str, "%f %d %f %f ");
  % -- Estimated fiducial pose
  fmt_str = strcat(fmt_str, "%d ");           % Estimated
  fmt_str = strcat(fmt_str, "%f %f %f ");     % Object points
  fmt_str = strcat(fmt_str, "%f %f %f %f ");  % q_WF
  fmt_str = strcat(fmt_str, "%f %f %f");      % r_WF

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

  # Target properties
  aprilgrid.configured = csv_data{1}(1);
  aprilgrid.tag_rows = csv_data{2}(1);
  aprilgrid.tag_cols = csv_data{3}(1);
  aprilgrid.tag_size = csv_data{4}(1);
  aprilgrid.tag_spacing = csv_data{5}(1);

  # Observed keypoints
  % aprilgrid.ts = str2uint64(csv_data{6}{1});
  aprilgrid.ts = csv_data{6}(1);
  aprilgrid.id = csv_data{7}(1:4:end);
  kp_x = csv_data{8}(1:end);
  kp_y = csv_data{9}(1:end);
  aprilgrid.keypoints = transpose([kp_x, kp_y]);

  # Estimated fiducuial pose
  aprilgrid.estimated = csv_data{10}(1);
  # -- AprilTag corners
  p_x = csv_data{11}(1:end);
  p_y = csv_data{12}(1:end);
  p_z = csv_data{13}(1:end);
  aprilgrid.points_CF = transpose([p_x, p_y, p_z]);
  # -- Quaternion q_WF
  q_w = csv_data{14}(1);
  q_x = csv_data{15}(1);
  q_y = csv_data{16}(1);
  q_z = csv_data{17}(1);
  aprilgrid.q_CF = [q_w; q_x; q_y; q_z];
  # -- Translation r_WF
  r_x = csv_data{18}(1);
  r_y = csv_data{19}(1);
  r_z = csv_data{20}(1);
  aprilgrid.r_CF = [r_x; r_y; r_z];
  # -- Form transform T_WF
  aprilgrid.T_CF = tf(quat2rot(aprilgrid.q_CF), aprilgrid.r_CF);

  retval = 0;
endfunction
