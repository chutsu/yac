#!/usr/bin/octave -qf
addpath("utils");
graphics_toolkit("fltk");

results_path = "/tmp/nbt/traj/";
result = list_dir(results_path);
trajs = {};
for i = 1:length(result)
  f = strcat(results_path, result(i).name);
  trajs{i} = csvread(f);
endfor

function plot_trajs(trajs)
  grid = aprilgrid_init();
  rpy = deg2rad([90.0, 0.0, -90.0]);
  C_WF = euler321(rpy);
  T_WF = eye(4);
  T_WF(1:3, 1:3) = C_WF;
  T_WF(1:3, 4) = zeros(3, 1);

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
endfunction

function plot_cam_vels(traj)
  figure();
  hold on;
  cam_ts = [];
  cam_vel = [];

  for j = 1:rows(traj)
    r_WC = traj(j, 2:4);
    q_WC = traj(j, 5:8);
    C_WC = quat2rot(q_WC);

    ts = traj(j, 1);
    v_WC = traj(j, 9:11);
    cam_ts = [cam_ts; ts];
    cam_vel = [cam_vel; v_WC];
  endfor
  plot(cam_ts * 1e-9, cam_vel(1:end, 1), 'r-', 'linewidth', 2.0);
  plot(cam_ts * 1e-9, cam_vel(1:end, 2), 'g-', 'linewidth', 2.0);
  plot(cam_ts * 1e-9, cam_vel(1:end, 3), 'b-', 'linewidth', 2.0);
  xlabel("Time [s]");
  ylabel("Velocity [ms^{-1}]");
  legend('x', 'y', 'z');
endfunction

% plot_trajs(trajs);
plot_cam_vels(trajs{1});
ginput();
