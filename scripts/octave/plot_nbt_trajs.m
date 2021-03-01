#!/usr/bin/octave -qf
addpath("utils");
graphics_toolkit("fltk");

function hp = homogeneous(p)
  hp = [p; ones(1, columns(p))];
endfunction

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

function aprilgrid_draw(grid, T_WF)
  p_F = [];
  for id=0:(grid.rows*grid.cols)-1
      points = grid.object_points{id + 1};
        p_F = [p_F, points];
  endfor
  hp_F = homogeneous(p_F);

  hp_WF = T_WF * hp_F;
  scatter3(hp_WF(1, :), hp_WF(2, :), hp_WF(3, :), "r", "filled");
  draw_frame(T_WF, grid.size);
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

results_path = "/tmp/nbt/traj/";
result = list_dir(results_path);
trajs = {};
for i = 1:length(result)
  f = strcat(results_path, result(i).name);
  trajs{i} = csvread(f);
endfor

rpy = deg2rad([90.0, 0.0, -90.0]);
C_WF = euler321(rpy);
T_WF = eye(4);
T_WF(1:3, 1:3) = C_WF;
T_WF(1:3, 4) = zeros(3, 1);
% calib_target = calib_target_init(6, 6, 0.088);
grid = aprilgrid_init();

figure();
hold on;
for i = 1:length(trajs)
  traj = trajs{i};
  for j = 1:rows(traj)
    r_WC = traj(j, 2:4);
    q_WC = traj(j, 5:8);
    C_WC = quat2rot(q_WC);

    T_WC = eye(4);
    T_WC(1:3, 1:3) = C_WC;
    T_WC(1:3, 4) = r_WC;

    draw_frame(T_WC, 0.05);
  endfor
  r_WC = traj(end, 2:4);
  text(r_WC(1), r_WC(2), r_WC(3) + 0.025,
       strcat("T", num2str(i - 1)),
       "fontsize", 18.0,
       "fontweight", "bold");
endfor
aprilgrid_draw(grid, T_WF);
view(3);
xlabel("x [m]");
ylabel("y [m]");
zlabel("z [m]");
axis "equal";
ginput();
