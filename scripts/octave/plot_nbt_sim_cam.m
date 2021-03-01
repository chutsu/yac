#!/usr/bin/octave -qf
addpath("utils");
graphics_toolkit("fltk");

traj = csvread("/tmp/nbt/traj_0/traj.csv");
cam0_grids_path = "/tmp/nbt/traj_0/cam0";

function result = list_dir(target_dir)
  listing = dir(target_dir);
  result = [];

  for i = 1:length(listing)
    if any(strcmp(listing(i).name, {'.', '..'})) == 0
      target.name = listing(i).name;
      target.date = listing(i).date;
      target.bytes = listing(i).bytes;
      target.isdir = listing(i).isdir;

      result = [result; target];
    end
  end
endfunction

function hp = homogeneous(p)
  hp = [p; ones(1, columns(p))];
endfunction

function p = dehomogeneous(hp)
  p = hp(1:3, :);
endfunction

function R = euler321(rpy)
  phi = rpy(1);
  theta = rpy(2);
  psi = rpy(3);

  R11 = cos(psi) * cos(theta);
  R21 = sin(psi) * cos(theta);
  R31 = -sin(theta);

  R12 = cos(psi) * sin(theta) * sin(phi) - sin(psi) * cos(phi);
  R22 = sin(psi) * sin(theta) * sin(phi) + cos(psi) * cos(phi);
  R32 = cos(theta) * sin(phi);

  R13 = cos(psi) * sin(theta) * cos(phi) + sin(psi) * sin(phi);
  R23 = sin(psi) * sin(theta) * cos(phi) - cos(psi) * sin(phi);
  R33 = cos(theta) * cos(phi);

  R = [R11, R12, R13; R21, R22, R23; R31, R32, R33;];
endfunction

function calib_target = calib_target_init(nb_rows=6, nb_cols=7, tag_size=0.2)
  # Create calib_target grid
  nb_corners = nb_rows * nb_cols;
  object_points = zeros(3, nb_corners);
  idx = 1;
  for i = 1:nb_rows
    for j = 1:nb_cols
      object_points(1:2, idx) = [j - 1; i - 1];
      idx += 1;
    endfor
  endfor

  # Chessboard struct
  calib_target.nb_rows = nb_rows;
  calib_target.nb_cols = nb_cols;
  calib_target.nb_corners = nb_corners;
  calib_target.tag_size = tag_size;
  calib_target.width = calib_target.nb_cols * calib_target.tag_size;
  calib_target.height = calib_target.nb_rows * calib_target.tag_size;
  calib_target.center = [((nb_cols - 1.0) / 2.0) * tag_size,
                         ((nb_rows - 1.0) / 2.0) * tag_size];
  calib_target.object_points = tag_size * object_points;
endfunction

function calib_target_draw(calib_target, T_WT)
  hp_T = homogeneous(calib_target.object_points);
  hp_W = T_WT * hp_T;
  scatter3(hp_W(1, :), hp_W(2, :), hp_W(3, :), "r", "filled");
  draw_frame(T_WT, calib_target.tag_size);
endfunction

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

function p_distorted = radtan4_distort(k1, k2, p1, p2, p)
  % Point
  x = p(1);
  y = p(2);

  % Apply radial distortion
  x2 = x * x;
  y2 = y * y;
  r2 = x2 + y2;
  r4 = r2 * r2;
  radial_factor = 1.0 + (k1 * r2) + (k2 * r4);
  x_dash = x * radial_factor;
  y_dash = y * radial_factor;

  % Apply tangential distortion
  xy = x * y;
  x_ddash = x_dash + (2.0 * p1 * xy + p2 * (r2 + 2.0 * x2));
  y_ddash = y_dash + (p1 * (r2 + 2.0 * y2) + 2.0 * p2 * xy);
  p_distorted = [x_ddash; y_ddash];
endfunction

function draw_frame(T_WS, scale=0.1)
  r_WS = T_WS(1:3, 4);
  origin = r_WS;

  x_axis = T_WS * [(scale * [1; 0; 0]); 1.0];
  y_axis = T_WS * [(scale * [0; 1; 0]); 1.0];
  z_axis = T_WS * [(scale * [0; 0; 1]); 1.0];

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

function plot_traj(traj)
  for j = 1:rows(traj)
    % q_WC = traj(j, 2:5);
    % r_WC = traj(j, 6:8);
    r_WC = traj(j, 2:4);
    q_WC = traj(j, 5:8);
    C_WC = quat2rot(q_WC);

    T_WC = eye(4);
    T_WC(1:3, 1:3) = C_WC;
    T_WC(1:3, 4) = r_WC;

    draw_frame(T_WC, 0.05);
  endfor
  r_WC = traj(1, 2:4);
  text(r_WC(1), r_WC(2), r_WC(3) + 0.025,
        "start",
        "fontsize", 18.0,
        "fontweight", "bold");
  r_WC = traj(end, 2:4);
  text(r_WC(1), r_WC(2), r_WC(3) + 0.025,
        "end",
        "fontsize", 18.0,
        "fontweight", "bold");
endfunction

function plot_scene(traj, aprilgrid, T_WF)
  % Trajectories
  plot_traj(traj);

  % AprilGrid
  p_WF = dehomogeneous(T_WF * homogeneous(aprilgrid.object_points));
  scatter3(p_WF(1, :), p_WF(2, :), p_WF(3, :), "filled");
endfunction

function plot_camera_view(cam, T_WC, calib_target, T_WT)
  hp_T = homogeneous(calib_target.object_points);
  hp_C = inv(T_WC) * T_WT * hp_T;
  p_C = dehomogeneous(hp_C);

  points = [];
  for i = 1:length(p_C)
    p = p_C(1:3, i);

    % Project
    x = p(1) / p(3);
    y = p(2) / p(3);
    p = [x; y];

    % Distort, scale and center
    p_d = radtan4_distort(cam.k1, cam.k2, cam.p1, cam.p2, p);
    pixel_x = cam.fx * p_d(1) + cam.cx;
    pixel_y = cam.fy * p_d(2) + cam.cy;

    if pixel_x < cam.resolution(1) && pixel_y < cam.resolution(2)
      plot(pixel_x, pixel_y, "rx", "linewidth", 10.0);
    end
  endfor

  % axis("equal");
  xlabel("x [px]");
  ylabel("y [px]");
  xlim([0, cam.resolution(1)]);
  ylim([0, cam.resolution(2)]);
  pause(0.001);
endfunction


% Calibration target
rpy = deg2rad([90.0, 0.0, -90.0]);
C_WF = euler321(rpy);
T_WF = eye(4);
T_WF(1:3, 1:3) = C_WF;
T_WF(1:3, 4) = zeros(3, 1);
% calib_target = calib_target_init(6, 6, 0.088);

% AprilGrids
grids0 = dir(cam0_grids_path);
grids0 = natsortfiles({grids0.name})(3:end);
grid_path = strcat(cam0_grids_path, "/", grids0{1});
[retval, grid_data] = load_aprilgrid(grid_path);


figure();
hold on;
#plot_traj(traj)
plot_scene(traj, grid_data, T_WF)
view(3);
xlabel("x [m]");
ylabel("y [m]");
zlabel("z [m]");
title("Scene");
axis "equal";
ginput();


% graphics_toolkit("gnuplot");
% figure("visible", "off");
% % figure();
%
% for i = 1:length(grids0)
%   progress = i / length(grids0);
%   if (mod(floor(progress * 100), 10) == 0)
%     printf("progress: %.2f%%\n", floor(progress * 100));
%   endif
%
%   grid_path = strcat(cam0_grids_path, "/", grids0{i});
%   [retval, grid_data] = load_aprilgrid(grid_path);
%
%   cam.resolution = [640; 480];
%
%   cam.fx = 462.139;
%   cam.fy = 617.159;
%   cam.cx = 640 / 2;
%   cam.cy = 380 / 2;
%
%   cam.k1 = 0.01;
%   cam.k2 = 0.001;
%   cam.p1 = 0.001;
%   cam.p2 = 0.001;
%
%   % T_FW = inv(T_WF);
%   % T_CW = grid_data.T_CF * T_FW;
%   % T_WC = inv(T_CW);
%
%   clf;
%   hold on;
%   scatter(grid_data.keypoints(1, :), grid_data.keypoints(2, :), "r");
%   xlim([0, 640]);
%   ylim([0, 480]);
%
%   fname = sprintf("/tmp/cam0_traj-%02d.png", i);
%   print("-dpng", "-r100", fname);
% endfor
% cmd = sprintf("ffmpeg -y -framerate 10 -i /%s/cam0_traj-%%02d.png -vf scale=1080:-1 cam_traj.mp4", "tmp");
% system(cmd)

% figure(2);
% % subplot(5, 1, [1, 2, 3]);
% hold on;
% plot_scene(traj, calib_target, T_WF);
% ginput();
