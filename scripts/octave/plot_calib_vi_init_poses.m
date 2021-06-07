cam0_poses = "/tmp/calib_vi_init_poses.csv"

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

function r = tf_trans(T)
  r = T(1:3, 4);
endfunction

function r_prime = tf_point(T, r)
  r_prime = (T * [r; 1.0])(1:3);
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

function poses = load_poses(csv_path)
  data = csvread(csv_path, 0, 0);

  poses = {};
  for i = 1:rows(data)
    r = [data(i, 1); data(i, 2); data(i, 3)];
    q = [data(i, 4); data(i, 5); data(i, 6); data(i, 7)];
    poses{i} = tf(q, r);
  endfor
endfunction

function pose = load_pose(csv_path)
  poses = load_poses(csv_path);
  pose = poses{1};
endfunction

% Calibration target
rpy = deg2rad([90.0, 0.0, -90.0]);
C_WF = euler321(rpy);
T_WF = eye(4);
T_WF(1:3, 1:3) = C_WF;
T_WF(1:3, 4) = zeros(3, 1);

% Camera poses
cam0_poses = load_poses(cam0_poses);

figure(1);
hold on;
draw_frame(T_WF * cam0_poses{1});
draw_frame(T_WF * cam0_poses{2});
draw_frame(T_WF * cam0_poses{3});
xlabel("x");
ylabel("y");
zlabel("z");
axis 'equal';
view(3);
ginput();
