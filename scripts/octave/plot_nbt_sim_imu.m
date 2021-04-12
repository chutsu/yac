#!/usr/bin/octave -qf
addpath("utils");
graphics_toolkit("fltk");

traj_dir = "/tmp/nbt/traj_3/";
cam0_grids_path = strcat(traj_dir, "cam0");
accel = csvread(strcat(traj_dir, "imu/accel.csv"));
gyro = csvread(strcat(traj_dir, "imu/gyro.csv"));
traj = csvread(strcat(traj_dir, "imu/poses.csv"));

ts = accel(:, 1) * 1e-9;
accel = accel(:, 2:4);;
gyro = gyro(:, 2:4);;

% figure(1);
% hold on;
%
% for i = 0:3
%   traj_dir = strcat("/tmp/nbt/traj_", num2str(i), "/");
%   cam0_grids_path = strcat(traj_dir, "cam0");
%   accel = csvread(strcat(traj_dir, "imu/accel.csv"));
%   gyro = csvread(strcat(traj_dir, "imu/gyro.csv"));
%   traj = csvread(strcat(traj_dir, "imu/poses.csv"));
%
%   ts = accel(:, 1) * 1e-9;
%   accel = accel(:, 2:4);;
%   gyro = gyro(:, 2:4);;
%
%   rows(accel)
%   a_norm = [];
%   for i = 1:rows(accel)
%     a = accel(i, 1:3);
%     a_norm = [a_norm; abs(9.81 - norm(a))];
%   endfor
%   plot(ts, a_norm, 'linewidth', 2.0);
%
% endfor
% legend("0", "1", "2", "3");
% % legend("0", "1", "2", "3", "4", "5", "6", "7");
% ginput();

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

function R = quat2rot(q)
  qw = q(1);
  qx = q(2);
  qy = q(3);
  qz = q(4);

  qx2 = qx**2;
  qy2 = qy**2;
  qz2 = qz**2;
  qw2 = qw**2;

  # Homogeneous form
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
  for j = 1:100:rows(traj)
    % q_WS = traj(j, 2:5);
    % r_WS = traj(j, 6:8);
    r_WS = traj(j, 2:4);
    q_WS = traj(j, 5:8);
    C_WS = quat2rot(q_WS);

    T_WS = eye(4);
    T_WS(1:3, 1:3) = C_WS;
    T_WS(1:3, 4) = r_WS;

    draw_frame(T_WS, 0.05);
  endfor
  r_WS = traj(1, 6:8);
  text(r_WS(1), r_WS(2), r_WS(3) + 0.025,
        "start",
        "fontsize", 18.0,
        "fontweight", "bold");
  r_WS = traj(end, 6:8);
  text(r_WS(1), r_WS(2), r_WS(3) + 0.025,
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

% Calibration target
% traj = csvread("/tmp/nbt/traj/traj_0.csv");
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

figure(1);
subplot(5, 1, [1, 2, 3]);
hold on;
plot_scene(traj, grid_data, T_WF)

view(3);
xlabel("x [m]");
ylabel("y [m]");
zlabel("z [m]");
title("Scene");
axis "equal";

subplot(5, 1, [4]);
hold on;
plot(ts, accel(:, 1), "r-", "linewidth", 2.0);
plot(ts, accel(:, 2), "g-", "linewidth", 2.0);
plot(ts, accel(:, 3), "b-", "linewidth", 2.0);
xlabel("Time [s]");
ylabel("Acceleration [ms^-2]");
title("Accelerometer");
legend("x", "y", "z");

subplot(5, 1, [5]);
hold on;
plot(ts, gyro(:, 1), "r-", "linewidth", 2.0);
plot(ts, gyro(:, 2), "g-", "linewidth", 2.0);
plot(ts, gyro(:, 3), "b-", "linewidth", 2.0);
xlabel("Time [s]");
ylabel("Angular Velocity [rad s^-1]");
title("Gyroscope");
legend("x", "y", "z");

ginput();
