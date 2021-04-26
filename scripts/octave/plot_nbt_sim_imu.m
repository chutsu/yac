#!/usr/bin/octave -qf
addpath("utils");
graphics_toolkit("fltk");

% traj_dir = "/tmp/nbt/traj_3/";
% cam0_grids_path = strcat(traj_dir, "cam0");
% accel = csvread(strcat(traj_dir, "imu/accel.csv"));
% gyro = csvread(strcat(traj_dir, "imu/gyro.csv"));
% traj = csvread(strcat(traj_dir, "imu/poses.csv"));
%
% ts = accel(:, 1) * 1e-9;
% accel = accel(:, 2:4);;
% gyro = gyro(:, 2:4);;

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

function plot_imu(ts, accel, gyro)
  figure()
  subplot(211);
  hold on;
  plot(ts, accel(:, 1), "r-", "linewidth", 2.0);
  plot(ts, accel(:, 2), "g-", "linewidth", 2.0);
  plot(ts, accel(:, 3), "b-", "linewidth", 2.0);
  xlabel("Time [s]");
  ylabel("Acceleration [ms^-2]");
  title("Accelerometer");
  legend("x", "y", "z");

  subplot(212);
  hold on;
  plot(ts, gyro(:, 1), "r-", "linewidth", 2.0);
  plot(ts, gyro(:, 2), "g-", "linewidth", 2.0);
  plot(ts, gyro(:, 3), "b-", "linewidth", 2.0);
  xlabel("Time [s]");
  ylabel("Angular Velocity [rad s^-1]");
  title("Gyroscope");
  legend("x", "y", "z");
endfunction

for i = 0:3
  traj_dir = strcat("/tmp/nbt/traj_", num2str(i), "/");
  accel = csvread(strcat(traj_dir, "imu/accel.csv"));
  gyro = csvread(strcat(traj_dir, "imu/gyro.csv"));
  traj = csvread(strcat(traj_dir, "imu/poses.csv"));

  ts = accel(:, 1) * 1e-9;
  accel = accel(:, 2:4);
  gyro = gyro(:, 2:4);
  plot_imu(ts, accel, gyro)
endfor

% % Calibration target
% % traj = csvread("/tmp/nbt/traj/traj_0.csv");
% rpy = deg2rad([90.0, 0.0, -90.0]);
% C_WF = euler321(rpy);
% T_WF = eye(4);
% T_WF(1:3, 1:3) = C_WF;
% T_WF(1:3, 4) = zeros(3, 1);
% % calib_target = calib_target_init(6, 6, 0.088);
%
% % AprilGrids
% grids0 = dir(cam0_grids_path);
% grids0 = natsortfiles({grids0.name})(3:end);
% grid_path = strcat(cam0_grids_path, "/", grids0{1});
% [retval, grid_data] = load_aprilgrid(grid_path);

% figure(1);
% hold on;
% subplot(5, 1, [1, 2, 3]);
% plot_scene(traj, grid_data, T_WF)

% view(3);
% xlabel("x [m]");
% ylabel("y [m]");
% zlabel("z [m]");
% title("Scene");
% axis "equal";

ginput();
