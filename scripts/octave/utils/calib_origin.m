function T_FO = calib_origin(aprilgrid, resolution, cam_params)
  % Calculate target center
  tag_rows = aprilgrid.tag_rows;
  tag_cols = aprilgrid.tag_cols;
  tag_size = aprilgrid.tag_size;
  tag_spacing = aprilgrid.tag_spacing;

  spacing_x = (tag_cols - 1) * tag_spacing * tag_size;
  spacing_y = (tag_rows - 1) * tag_spacing * tag_size;
  target_width = tag_cols * tag_size + spacing_x;
  target_height = tag_rows * tag_size + spacing_y;
  center = [target_width / 2.0; target_height / 2.0];
  % retry = 5;

  % Calculate distance away from target center
  target_scale = 0.5;
  image_width = resolution(1);
  target_half_width =  target_width / 2.0;
  target_half_resolution_x = image_width / 2.0;
  fx = cam_params(1);
  z_FO = fx * target_half_width / (target_half_resolution_x * target_scale);

  % Form transform of calibration origin (O) wrt fiducial target (F) T_FO
  C_FO = eye(3);
  r_FO = [center(1); center(2); z_FO];
  T_FO = tf(C_FO, r_FO);
endfunction
