#!/usr/bin/octave-cli
poses_csv = "/tmp/poses.csv";

function hp = homogeneous(p)
  hp = [p; 1];
endfunction

function R = quat2rot(q)
  qw = q(1);
  qx = q(2);
  qy = q(3);
  qz = q(4);

  qx2 = qx**2;
  qy2 = qy**2;
  qz2 = qz**2;
  qw2 = qw**2;

  R11 = qw2 + qx2 - qy2 - qz2;
  R12 = 2 * (qx * qy - qw * qz);
  R13 = 2 * (qx * qz + qw * qy);

  R21 = 2 * (qx * qy + qw * qz);
  R22 = qw2 - qx2 + qy2 - qz2;
  R23 = 2 * (qy * qz - qw * qx);

  R31 = 2 * (qx * qz - qw * qy);
  R32 = 2 * (qy * qz + qw * qx);
  R33 = qw2 - qx2 - qy2 + qz2;

  R = [R11, R12, R13; R21, R22, R23; R31, R32, R33;];
endfunction

function T = tf(rot, trans)
  assert(size(rot) == [3, 3] || size(rot) == [4, 1]);
  assert(size(trans) == [3, 1]);

  C = rot;
  if size(rot) == [4, 1]
    C = quat2rot(rot);
  endif

  T = eye(4, 4);
  T(1:3, 1:3) = C;
  T(1:3, 4) = trans;
endfunction

function poses = load_poses(poses_path)
  data = csvread(poses_path);

  poses = {};
  for i = 1:rows(data)
    q = [data(i, 1); data(i, 2); data(i, 3); data(i, 4)];
    r = [data(i, 5); data(i, 6); data(i, 7)];
    poses(i) = tf(q, r);
  endfor
endfunction

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

function draw_frame(T_WS, scale=1.0)
  R_WS = T_WS(1:3, 1:3);
  t_WS = T_WS(1:3, 4);
  origin = t_WS;

  x_axis = T_WS * homogeneous(scale * [1; 0; 0]);
  y_axis = T_WS * homogeneous(scale * [0; 1; 0]);
  z_axis = T_WS * homogeneous(scale * [0; 0; 1]);

  % Draw x-axis
  plot3([origin(1), x_axis(1)], ...
        [origin(2), x_axis(2)], ...
        [origin(3), x_axis(3)], 'r',
        "linewidth", 5)

  % Draw y-axis
  plot3([origin(1), y_axis(1)], ...
        [origin(2), y_axis(2)], ...
        [origin(3), y_axis(3)], 'g',
        "linewidth", 5)

  % Draw z-axis
  plot3([origin(1), z_axis(1)], ...
        [origin(2), z_axis(2)], ...
        [origin(3), z_axis(3)], 'b',
        "linewidth", 5)
endfunction

function [i, j] = aprilgrid_grid_index(grid, id)
  assert(id < (grid.rows * grid.cols) && id >= 0);
  i = floor(id / grid.cols);
  j = floor(rem(id, grid.cols));
endfunction

function object_points = aprilgrid_object_points(grid)
  object_points = {};

  nb_tags = grid.rows * grid.cols;
  for id = 0:nb_tags-1;
    % Calculate the AprilGrid index using tag id
    [i, j] = aprilgrid_grid_index(grid, id);

    % Calculate the x and y of the tag origin (bottom left corner of tag)
    % relative to grid origin (bottom left corner of entire grid)
    x = j * (grid.size + grid.size * grid.spacing);
    y = i * (grid.size + grid.size * grid.spacing);
    pt_bl = [x; y; 0];                          % Bottom left
    pt_br = [x + grid.size; y; 0];              % Bottom right
    pt_tr = [x + grid.size; y + grid.size; 0];  % Top right
    pt_tl = [x; y + grid.size; 0];              % Top left

    % Tag object points
    tag_points = [pt_bl, pt_br, pt_tr, pt_tl];

    % Add to total object points
    object_points{id + 1} = tag_points;
  endfor
endfunction

function grid = aprilgrid_init(rows=6, cols=6, size=0.088, spacing=0.3)
  grid = {};
  grid.rows = rows;
  grid.cols = cols;
  grid.size = size;
  grid.spacing = spacing;
  grid.object_points = aprilgrid_object_points(grid);
  grid.keypoints = {};
  grid.T_CF = eye(4);
endfunction

function draw_aprilgrid(aprilgrid)
  for i = 1:length(aprilgrid.object_points)
    pts = aprilgrid.object_points{i};
    for j = 1:4
      plot3(pts(1, j), pts(2, j), pts(3, j), 'ro', 'markersize', 5.0);
    endfor
  endfor
endfunction

% MAIN
aprilgrid = aprilgrid_init(rows=6, cols=6, size=0.088, spacing=0.3);
cam_poses = load_poses(poses_csv);

figure(1);
hold on;

draw_aprilgrid(aprilgrid)

for i = 1:length(cam_poses)
  T_WC = cam_poses{i};
  % draw_camera(T_WC);
  draw_frame(T_WC, 0.05);
endfor

xlabel("x");
ylabel("y");
zlabel("z");
view(3);
axis 'equal';
ginput();
