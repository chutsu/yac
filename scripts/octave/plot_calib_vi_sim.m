sensor_poses = "/tmp/sensor_poses.csv";
camera_poses = "/tmp/camera_poses.csv";
extrinsics = "/tmp/extrinsics.csv";
fiducial = "/tmp/fiducial.csv";
imu_data = "/tmp/imu.csv";
imu_vel_data = "/tmp/imu_vel.csv";

function hp = homogeneous(p)
  hp = [p; 1];
endfunction

function y = skew(x)
  y = [0, -x(3), x(2);
       x(3), 0, -x(1);
       -x(2), x(1), 0];
endfunction

function n = quat_norm(q)
  n = sqrt(q(1)**2 + q(2)**2 + q(3)**2 + q(4)**2);
endfunction

function q_out = quat_normalize(q)
  n = quat_norm(q);
  q_out = [q(1) / n; q(2) / n; q(3) / n; q(4) / n];
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

function q = rot2quat(R)
  m00 = R(1, 1);
  m01 = R(1, 2);
  m02 = R(1, 3);

  m10 = R(2, 1);
  m11 = R(2, 2);
  m12 = R(2, 3);

  m20 = R(3, 1);
  m21 = R(3, 2);
  m22 = R(3, 3);

  tr = m00 + m11 + m22;

  if (tr > 0)
    S = sqrt(tr+1.0) * 2; % S=4*qw
    qw = 0.25 * S;
    qx = (m21 - m12) / S;
    qy = (m02 - m20) / S;
    qz = (m10 - m01) / S;
  elseif ((m00 > m11) && (m00 > m22))
    S = sqrt(1.0 + m00 - m11 - m22) * 2; % S=4*qx
    qw = (m21 - m12) / S;
    qx = 0.25 * S;
    qy = (m01 + m10) / S;
    qz = (m02 + m20) / S;
  elseif (m11 > m22)
    S = sqrt(1.0 + m11 - m00 - m22) * 2; % S=4*qy
    qw = (m02 - m20) / S;
    qx = (m01 + m10) / S;
    qy = 0.25 * S;
    qz = (m12 + m21) / S;
  else
    S = sqrt(1.0 + m22 - m00 - m11) * 2; % S=4*qz
    qw = (m10 - m01) / S;
    qx = (m02 + m20) / S;
    qy = (m12 + m21) / S;
    qz = 0.25 * S;
  endif

  q = quat_normalize([qw; qx; qy; qz]);
endfunction

function euler = quat2euler(q)
  qw = q(1);
  qx = q(2);
  qy = q(3);
  qz = q(4);

  qw2 = qw**2;
  qx2 = qx**2;
  qy2 = qy**2;
  qz2 = qz**2;

  t1 = atan2(2 * (qx * qw + qz * qy), (qw2 - qx2 - qy2 + qz2));
  t2 = asin(2 * (qy * qw - qx * qz));
  t3 = atan2(2 * (qx * qy + qz * qw), (qw2 + qx2 - qy2 - qz2));
  euler = [t1; t2; t3];
endfunction

function euler = rot2euler(R)
  q = rot2quat(R);
  euler = quat2euler(q);
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

function draw_frame(T_WS, scale=0.1)
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

function [ts, poses] = load_poses(csv_path)
  data = csvread(csv_path, 0, 0);

  ts = {};
  poses = {};
  for i = 1:rows(data)
    ts{i} = data(i, 1);
    q = [data(i, 2); data(i, 3); data(i, 4); data(i, 5)];
    r = [data(i, 6); data(i, 7); data(i, 8)];
    poses{i} = tf(q, r);
  endfor
endfunction

function pose = load_pose(csv_path)
  data = csvread(csv_path, 0, 0);
  q = [data(1, 1); data(1, 2); data(1, 3); data(1, 4)];
  r = [data(1, 5); data(1, 6); data(1, 7)];
  pose = tf(q, r);
endfunction

function [ts, acc, gyr] = load_imu_data(csv_path)
  data = csvread(csv_path, 0, 0);

  ts = {};
  acc = {};
  gyr = {};
  for i = 1:rows(data)
    ts{i} = data(i, 1);
    acc{i} = [data(i, 2); data(i, 3); data(i, 4)];
    gyr{i} = [data(i, 5); data(i, 6); data(i, 7)];
  endfor
endfunction

function [ts, vel] = load_imu_vel_data(csv_path)
  data = csvread(csv_path, 0, 0);

  ts = {};
  vel = {};
  for i = 1:rows(data)
    ts{i} = data(i, 1);
    vel{i} = [data(i, 2); data(i, 3); data(i, 4)];
  endfor
endfunction

function C = so3_exp(phi)
  if (phi < 1e-3)
    C = eye(3) + skew(phi);
  else
    C = eye(3);
    C += (sin(norm(phi)) / norm(phi)) * skew(phi);
    C += ((1 - cos(norm(phi))) / norm(phi)^2) * skew(phi)^2;
  endif
endfunction

function plot_imu_data(imu_ts, imu_acc, imu_gyr)
  t = [];
  acc_x = [];
  acc_y = [];
  acc_z = [];
  gyr_x = [];
  gyr_y = [];
  gyr_z = [];

  for i = 1:length(imu_ts)
    t = [t; imu_ts{i} * 1e-9];
    acc_x = [acc_x; imu_acc{i}(1)];
    acc_y = [acc_y; imu_acc{i}(2)];
    acc_z = [acc_z; imu_acc{i}(3)];
    gyr_x = [gyr_x; imu_gyr{i}(1)];
    gyr_y = [gyr_y; imu_gyr{i}(2)];
    gyr_z = [gyr_z; imu_gyr{i}(3)];
  endfor

  figure(1);
  subplot(211);
  hold on;
  plot(t, acc_x, 'r-');
  plot(t, acc_y, 'g-');
  plot(t, acc_z, 'b-');
  xlabel("Time [s]");
  ylabel("Acceleration [ms^-2]");
  xlim([0, max(t)]);

  subplot(212);
  hold on;
  plot(t, gyr_x, 'r-');
  plot(t, gyr_y, 'g-');
  plot(t, gyr_z, 'b-');
  xlabel("Time [s]");
  ylabel("Angular Velocity [rad s^-1]");
  xlim([0, max(t)]);

  ginput();
endfunction

function plot_sensor_poses(ts, T_WS)
  t = [];
  x = [];
  y = [];
  z = [];
  for i = 1:100:length(T_WS)
    r_WS = T_WS{i}(1:3, 4);
    t = [t; ts{i} * 1e-9];
    x = [x; r_WS(1)];
    y = [y; r_WS(2)];
    z = [z; r_WS(3)];
  endfor

  figure(1);
  hold on;
  plot(t, x, 'r-');
  plot(t, y, 'g-');
  plot(t, z, 'b-');
  xlabel("Time [s]");
  ylabel("Displacement [m]");
  ginput();
endfunction

function plot_scene(T_WS, T_WC, T_WF)
  figure(1);
  hold on;

  % C_SC = euler321(deg2rad([-90, 0, -90]))
  % r_SC = [0.0; 0.0; 0.1];
  % T_SC = tf(C_SC, r_SC);
  % T_WS = tf(eye(3), zeros(3, 1));
  % draw_frame(T_WS);
  % draw_frame(T_WS * T_SC);

  % C_WF = euler321(deg2rad([90, 0, -90]))
  % T_WF = tf(C_WF, zeros(3, 1));
  % draw_frame(T_WF);

  draw_frame(T_WF);

  % % for i = 1:100:length(T_WS)
  for i = 1:10:1000
    draw_frame(T_WS{i});
  endfor

  % for i = 1:10:length(T_WC)
  %   draw_frame(T_WC{i});
  % endfor

  xlabel "x";
  ylabel "y";
  zlabel "z";
  axis 'equal';
  view(3);
  ginput();
endfunction

% Load data
[imu_ts, T_WS] = load_poses(sensor_poses);
% [cam_ts, T_WC] = load_poses(camera_poses);
% T_SC = load_pose(extrinsics);
% T_WF = load_pose(fiducial);
[imu_ts, imu_acc, imu_gyr] = load_imu_data(imu_data);
[imu_ts, v_WS] = load_imu_vel_data(imu_vel_data);

function x_imu = imu_state_init()
  x_imu.p_WS = zeros(3, 1);
  x_imu.v_WS = zeros(3, 1);
  x_imu.C_WS = eye(3);
  x_imu.b_a = zeros(3, 1);
  x_imu.b_g = zeros(3, 1);
  x_imu.g = [0; 0; -9.81];
endfunction

function x_imu = imu_update(x_imu, a_B, w_B, dt)
  g = x_imu.g;
  b_a = x_imu.b_a;
  b_g = x_imu.b_g;
  n_a = zeros(3, 1);
  n_g = zeros(3, 1);

  C_WS_i = x_imu.C_WS;
  v_WS_i = x_imu.v_WS;

  w = (w_B - b_g - n_g);
  a = (a_B - b_a - n_a);
  dt_sq = dt * dt;

  x_imu.C_WS *= so3_exp(w * dt);
  x_imu.v_WS += (C_WS_i * a * dt) + (g * dt);
  x_imu.p_WS += (v_WS_i * dt) + (0.5 * C_WS_i * a * dt_sq) + (0.5 * g * dt_sq);
endfunction

% Initialize IMU state
x_imu = imu_state_init();
x_imu.p_WS = T_WS{1}(1:3, 4);
x_imu.v_WS = v_WS{1};
x_imu.C_WS = T_WS{1}(1:3, 1:3);

x_imu

% Batch integrate IMU measurements
% N = 8000;
N = length(imu_ts);
traj_time = [0];
traj_pos = [x_imu.p_WS];
traj_vel = [x_imu.v_WS];
traj_att = [rot2euler(x_imu.C_WS)];
% ts_km1 = imu_ts{1} * 1e-9;
ts_km1 = 0;
ts_k = 0;
dt = (1.0 / 200.0);
for k = 2:length(imu_ts)
% for k = 2:N
  % Calculate dt
  % ts_k = imu_ts{k} * 1e-9;
  % dt = ts_k - ts_km1;

  % Propagate IMU state
  acc = imu_acc{k};
  gyr = imu_gyr{k};
  x_imu = imu_update(x_imu, acc, gyr, dt);

  % Update
  traj_time = [traj_time; ts_k];
  traj_pos = [traj_pos, x_imu.p_WS];
  traj_vel = [traj_vel, x_imu.v_WS];
  traj_att = [traj_att, rot2euler(x_imu.C_WS)];
  ts_km1 = ts_k;
  ts_k += dt;
endfor

%
% x = [];
% y = [];
% z = [];
% for i = 1:length(T_WS)
%   r_WS = T_WS{i}(1:3, 4);
%   x = [x; r_WS(1)];
%   y = [y; r_WS(2)];
%   z = [z; r_WS(3)];
% endfor


% figure(2);

% subplot(311);
% hold on;
% plot(traj_time, traj_pos(1, 1:N), 'r--')
% plot(traj_time, x(1:N), 'r-')
%
% subplot(312);
% hold on;
% plot(traj_time, traj_pos(2, 1:N), 'g--')
% plot(traj_time, y(1:N), 'g-')
%
% subplot(313);
% hold on;
% plot(traj_time, traj_pos(3, 1:N), 'b--')
% plot(traj_time, z(1:N), 'b-')
%
% xlabel("Time [s]");
% ylabel("Displacement [m]");

figure();
plot(traj_pos(1, :), traj_pos(2, :), 'r-')
xlabel("Displacement [m]");
ylabel("Displacement [m]");
axis("equal");
ginput();

% plot_imu_data(imu_ts, imu_acc, imu_gyr);
% plot_sensor_poses(imu_ts, T_WS);
% plot_scene(T_WS, T_WC, T_WF);
