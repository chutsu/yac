function draw_camera(T_WC, scale=0.1, style="b-")
  fov = deg2rad(60.0);

  # Form the camera fov frame
  fov_hwidth = scale;
  fov_corners = zeros(3, 4);
  fov_corners(1:3, 1) = [-fov_hwidth; fov_hwidth; 0.0];  # Bottom left
  fov_corners(1:3, 2) = [-fov_hwidth; -fov_hwidth; 0.0]; # Top left
  fov_corners(1:3, 3) = [fov_hwidth; -fov_hwidth; 0.0];  # Top right
  fov_corners(1:3, 4) = [fov_hwidth; fov_hwidth; 0.0];   # Bottom right

  # Calculate the distance from camera origin to fov frame given fov
  dist = fov_hwidth / tan(fov / 2.0);
  fov_corners(3, :) = dist;

  # Transform fov_corners to world frame
  fov_corners = T_WC * [fov_corners; ones(1, 4)];
  fov_corners = fov_corners(1:3, :);

  # Transform camera_origin to world frame
  cam_origin = [0; 0; 0];
  cam_origin = T_WC * [cam_origin; 1.0];
  cam_origin = cam_origin(1:3, :);

  # Draw fov frame
  frame_x = [fov_corners(1, :), fov_corners(1, 1)];
  frame_y = [fov_corners(2, :), fov_corners(2, 1)];
  frame_z = [fov_corners(3, :), fov_corners(3, 1)];
  plot3(frame_x, frame_y, frame_z, style);

  # Draw from camera origin to fov frame
  for i = 1:4
    x = [cam_origin(1), fov_corners(1, i)];
    y = [cam_origin(2), fov_corners(2, i)];
    z = [cam_origin(3), fov_corners(3, i)];
    plot3(x, y, z, style);
  endfor
endfunction
