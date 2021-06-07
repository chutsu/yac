sensor_poses = "/tmp/sensor_poses.csv";
extrinsics = "/tmp/extrinsics.csv";
fiducial_pose = "/tmp/fiducial_pose.csv";

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
  data = csvread(csv_path, 1, 0);

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

% Load data
T_WS = load_poses(sensor_poses);
T_SC = load_poses(extrinsics);
T_WF = load_pose(fiducial_pose);

% Visualize
figure(1);
hold on;

T_WC0 = T_WS{1} * T_SC{1};
T_WC1 = T_WS{1} * T_SC{2};

draw_frame(T_WF, 0.5);
draw_frame(T_WS{1});
draw_frame(T_WC0);
draw_frame(T_WC1);

% for i = 1:10:length(T_WS)
%   draw_frame(T_WS{i});
% endfor
xlabel "x";
ylabel "y";
zlabel "z";
axis 'equal';
view(3);
ginput();
