#!/usr/bin/octave -qf
addpath("utils");
graphics_toolkit("fltk");

function plot_traj(traj)
  for j = 1:100:rows(traj)
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

function t = twist(w, v)
	t = [w; v];
endfunction

function T_adj = tf_adjoint(T)
	C = tf_rot(T);
	r = tf_trans(T);
	T_adj = [C, zeros(3, 3);
					 skew(r) * C, C];
endfunction

function [traj, data] = calib_orbit_traj(direction, aprilgrid, T_WF, T_FO, cam_rate, t_end)
  % Calculate target width and height
  tag_rows = aprilgrid.tag_rows;
  tag_cols = aprilgrid.tag_cols;
  tag_spacing = aprilgrid.tag_spacing;
  tag_size = aprilgrid.tag_size;
  spacing_x = (tag_cols - 1) * tag_spacing * tag_size;
  spacing_y = (tag_rows - 1) * tag_spacing * tag_size;
  calib_width = tag_cols * tag_size + spacing_x;
  calib_height = tag_rows * tag_size + spacing_y;

  % Target center (Fc) w.r.t. Target origin (F)
  r_FFc = [calib_width / 2.0; calib_height / 2.0; 0.0];
	T_FFc = tf(euler321([0, pi/2, pi/2]), r_FFc);

	calib_radius = tf_trans(T_FO)(3);
  nb_frames = cam_rate * t_end;
	C_WF = tf_rot(T_WF);

	% cirle_circum = (2 * pi * calib_radius) * (deg2rad(45) / deg2rad(360))

	is_horiz = 0;
  theta = 0;
  theta_end = 0;
	if strcmp(direction, "LEFT")
		% Orbit left
		theta = deg2rad(180.0);
		theta_end = deg2rad(135.0);
		is_horiz = 1;
	elseif strcmp(direction, "RIGHT")
		% Orbit right
		theta = deg2rad(-180.0);
		theta_end = deg2rad(-135.0);
		is_horiz = 1;
	elseif strcmp(direction, "UP")
		% Orbit up
		theta = deg2rad(180.0);
		theta_end = deg2rad(135.0);
	elseif strcmp(direction, "DOWN")
		% Orbit down
		theta = deg2rad(-180.0);
		theta_end = deg2rad(-135.0);
	endif

	w = (theta_end - theta) / t_end;
	rpy = [0; 0; 0];

	t = 0.0;
	dt = t_end / nb_frames;
  traj = {};

	time = [];
	pos = [];
	att = [];
	poses = {};
	vel = [];
	imu_acc = [];
	imu_angvel = [];

  rpy_BC0 = deg2rad([-90.0, 0.0, -90.0]);
  % rpy_BC0 = deg2rad([0.0, 0.0, 0.0]);
  C_BC0 = euler321(rpy_BC0);
  r_BC0 = zeros(3, 1);
  % r_BC0 = [0; 0.0; 0.1];
	T_BC0 = tf(C_BC0, r_BC0);

  T_WFc = T_WF * T_FFc;
  C_WFc = tf_rot(T_WFc);

  for i = 1:nb_frames
		time = [time; t];

    % Position
		x = 0;
		y = 0;
		z = 0;
		if is_horiz
			x = calib_radius * cos(theta);
			y = calib_radius * sin(theta);
			z = 0;
		else
			x = calib_radius * cos(theta);
			y = 0;
			z = calib_radius * sin(theta);
		endif

		% Orientation
		if is_horiz
			rpy(1) = 0;
			rpy(2) = 0;
			rpy(3) += w * dt;
		else
			rpy(1) = 0;
			rpy(2) -= w * dt;
			rpy(3) = 0;
		endif

    % Pose
		T_FcB = tf(euler321(rpy), [x; y; z]);
		T_WB = T_WF * T_FFc * T_FcB;
		C_WB = tf_rot(T_WB);

		T_WC0 = T_WB * T_BC0;
		C_WC0 = tf_rot(T_WC0);
		r_WC0 = tf_trans(T_WC0);
    rpy_WC0 = rot2euler(tf_rot(T_WC0));
    traj{i} = T_WC0;

		pos = [pos, r_WC0];
		att = [att, rpy_WC0];
		poses{i} = T_WC0;

    % Angular Velocity
    w_FcB = [];
		if is_horiz
			w_FcB = [0; 0; w];
		else
			w_FcB = [0; -w; 0];
		endif
    % w_FcB = zeros(3, 1);
		w_W_WB = C_WFc * w_FcB;
		w_B_WB = C_WB' * w_W_WB;
		w_C0_WC0 = C_BC0' * w_B_WB;
    imu_angvel = [imu_angvel, w_C0_WC0];

    % Velocity
		v_FcB = [];
		if is_horiz
			vx = -calib_radius * w * sin(theta);
			vy = calib_radius * w * cos(theta);
			vz = 0;
			v_FcB = [vx; vy; vz];
		else
			vx = -calib_radius * w * sin(theta);
			vy = 0;
			vz = calib_radius * w * cos(theta);
			v_FcB = [vx; vy; vz];
		endif
		% v_WB = C_WFc * v_FcB;
		% vel = [vel, v_WB];

		v_WB = C_WFc * v_FcB;
		v_WC0 = v_WB + C_WB * cross(w_B_WB, r_BC0);
		vel = [vel, v_WC0];

    % Acceleration
		a_FcB = [];
		if is_horiz
			ax = -calib_radius * w * w * cos(theta);
			ay = -calib_radius * w * w * sin(theta);
			az = 0.0;
			a_FcB = [ax; ay; az];
		else
			ax = -calib_radius * w * w * cos(theta);
			ay = 0.0;
			az = -calib_radius * w * w * sin(theta);
			a_FcB = [ax; ay; az];
		endif
    % g = [0; 0; 9.81];
		% a_W_WB = C_WFc * a_FcB + g;
		% a_B_WB = C_WB' * a_W_WB;
		% a_C0_WC0 = C_BC0' * a_B_WB;
		% imu_acc = [imu_acc, a_C0_WC0];

    g = [0;0;0];
    % g = [0;0;9.81];
		a_W_WB = C_WFc * a_FcB + g;
		a_W_WC0 = a_W_WB + C_WB * cross(w_B_WB, cross(w_B_WB, r_BC0));
		a_C0_WC0 = C_WC0' * a_W_WC0;
		imu_acc = [imu_acc, a_C0_WC0];

    theta += w * dt;
		t += dt;
  endfor

	data = {};
	data.time = time;
	data.pos = pos;
	data.att = att;
	data.poses = poses;
	data.vel = vel;
	data.imu_acc = imu_acc;
	data.imu_gyr = imu_angvel;
	% data.angvel = zeros(3, 200);
endfunction

function [traj, data] = calib_fig8_traj(aprilgrid, T_WF, T_FO, cam_rate, t_end)
  % Calculate target width and height
  tag_rows = aprilgrid.tag_rows;
  tag_cols = aprilgrid.tag_cols;
  tag_spacing = aprilgrid.tag_spacing;
  tag_size = aprilgrid.tag_size;
  spacing_x = (tag_cols - 1) * tag_spacing * tag_size;
  spacing_y = (tag_rows - 1) * tag_spacing * tag_size;
  calib_width = tag_cols * tag_size + spacing_x;
  calib_height = tag_rows * tag_size + spacing_y;

  traj = {};
  nb_frames = cam_rate * t_end;
  nb_hframes = nb_frames / 2;
  % calib_radius = calib_width * 0.5;
	T_WO = T_WF * T_FO;
	C_WO = tf_rot(T_WO);

	circle_circum = 0.62302;
	% cirlce_circum = 2 * pi * r
	% (cirlce_circum / 2 * pi) = r

	calib_radius = circle_circum / (2 * pi);

  t = 0;
	time = [];
	pos = [];
	att = [];
	vel = [];
	acc = [];
	angvel = [];

  % Clock-wise
  theta = deg2rad(0);
  theta_end = deg2rad(360);
	w = (theta_end - theta) / (t_end / 2);
	dt = t_end / nb_frames;

  for i = 1:nb_hframes
		time = [time; t];
    x = 0;
    y = calib_radius * cos(theta) - calib_radius;
    z = calib_radius * sin(theta);
    r_OB = [x; y; z];

		T_OB = tf(eye(3), r_OB);
		T_BC = tf(euler321([-pi/2, 0, -pi/2]), [0; 0; 0]);
		T_WC0 = T_WO * T_OB * T_BC;
		r_WC0 = tf_trans(T_WC0);
		rpy_WC0 = rot2euler(tf_rot(T_WC0));
    traj{i} = T_WC0;
		pos = [pos, r_WC0];
		att = [att, rpy_WC0];

    % Velocity
    vx = 0;
    vy = -calib_radius * w * sin(theta);
    vz = calib_radius * w * cos(theta);
    v_OB = [vx; vy; vz];
    v_WB = C_WO * v_OB;
		vel = [vel, v_WB];

    % Acceleration
    ax = 0;
    ay = -calib_radius * w * w * cos(theta);
    az = -calib_radius * w * w * sin(theta);
    a_OB = [ax; ay; az];
    a_WB = C_WO * a_OB;
		acc = [acc, C_WO * a_OB];

    % Angular Velocity
    w_OB = [0; 0; 0];
    w_WB = C_WO * w_OB;
    angvel = [angvel, w_WB];

    theta += w * dt;
		t += dt;
  end

  % Anti clock-wise
  theta = deg2rad(180);
  theta_end = deg2rad(-180);
	w = (theta_end - theta) / (t_end / 2);
	dt = t_end / nb_frames;

  for i = length(traj)+1:length(traj)+nb_hframes
		time = [time; t];
    x = 0;
    y = calib_radius * cos(theta) + calib_radius;
    z = calib_radius * sin(theta);

		T_OB = tf(eye(3), [x; y; z]);
		T_BC = tf(euler321([-pi/2, 0, -pi/2]), [0; 0; 0]);
		T_WC0 = T_WO * T_OB * T_BC;
		r_WC0 = tf_trans(T_WC0);
		rpy_WC0 = rot2euler(tf_rot(T_WC0));
    traj{i} = T_WC0;
		pos = [pos, r_WC0];
		att = [att, rpy_WC0];

    % Velocity
    vx = 0;
    vy = -calib_radius * w * sin(theta);
    vz = calib_radius * w * cos(theta);
    v_OB = [vx; vy; vz];
    v_WB = C_WO * v_OB;
		vel = [vel, v_WB];

    % Acceleration
    ax = 0;
    ay = -calib_radius * w * w * cos(theta);
    az = -calib_radius * w * w * sin(theta);
    a_OB = [ax; ay; az];
    a_WB = C_WO * a_OB;
		acc = [acc, C_WO * a_OB];

    % Angular Velocity
    w_OB = [0; 0; 0];
    w_WB = C_WO * w_OB;
    angvel = [angvel, w_WB];

    theta += w * dt;
		t += dt;
  end

	data = {};
	data.time = time;
	data.pos = pos;
	data.att = att;
	data.vel = vel;
	data.acc = acc;
	data.angvel = angvel;


	% figure();
	% hold on;
  %
	% % Poses
	% % for i = 1:10:nb_hframes
	% % for i = 1:nb_hframes
	% for i = 1:length(traj)
	%   draw_frame(traj{i}, 0.05);
	% endfor
	% r = tf_trans(traj{1});
	% text(r(1), r(2), r(3) + 0.025,
	% 			"START",
	% 			"fontsize", 18.0,
	% 			"fontweight", "bold");
	% r = tf_trans(traj{end});
	% text(r(1), r(2), r(3) + 0.025,
	% 			"END",
	% 			"fontsize", 18.0,
	% 			"fontweight", "bold");
  %
	% % AprilGrid
	% aprilgrid_draw(aprilgrid, T_WF);
  %
	% % Calibration origin
	% draw_frame(T_WO, 0.1);
	% r = tf_trans(T_WO);
	% text(r(1), r(2), r(3) + 0.025,
	% 			"T_{WO}",
	% 			"fontsize", 18.0,
	% 			"fontweight", "bold");
  %
	% % Plot settings
	% view(3);
	% xlabel("x [m]");
	% ylabel("y [m]");
	% zlabel("z [m]");
	% title("Scene");
	% axis "equal";
endfunction

function plot_scene(T_WF, T_FO, cam_rate, t_end, trajs, aprilgrid)
	% Setup
	figure();
	hold on;

	% AprilGrid
	aprilgrid_draw(aprilgrid, T_WF);

	% Calibration origin
	draw_frame(T_WF * T_FO, 0.1);
	r = tf_trans(T_WF * T_FO);
	text(r(1), r(2), r(3) + 0.025,
				"T_{WO}",
				"fontsize", 18.0,
				"fontweight", "bold");

	% Poses
	for traj_idx = 1:length(trajs)
	  traj = trajs{traj_idx};

    skip = length(traj) * 0.01;
    for i = 1:skip:length(traj)
      draw_frame(traj{i}, 0.05);
    endfor

    % r = tf_trans(traj{1});
    % text(r(1), r(2), r(3) + 0.025,
    % 			"START",
    % 			"fontsize", 18.0,
    % 			"fontweight", "bold");
    % r = tf_trans(traj{cam_rate * t_end});
    % text(r(1), r(2), r(3) + 0.025,
    % 			"END",
    % 			"fontsize", 18.0,
    % 			"fontweight", "bold");
  endfor

	% Plot settings
	view(3);
	xlabel("x [m]");
	ylabel("y [m]");
	zlabel("z [m]");
	title("Scene");
	axis "equal";
endfunction

function plot_sim_est(aprilgrid, T_WF, sim, est)
	% Setup
	figure();
	hold on;

	% AprilGrid
	aprilgrid_draw(aprilgrid, T_WF);

	% Sim poses
	skip = length(sim.poses) * 0.1;
	for i = 1:skip:length(sim.poses)
		draw_frame(sim.poses{i}, 0.05);
	endfor
  % r = tf_trans(sim.poses{1});
  % text(r(1), r(2), r(3) + 0.025,
  % 			"START",
  % 			"fontsize", 18.0,
  % 			"fontweight", "bold");
  % r = tf_trans(sim.poses{end});
  % text(r(1), r(2), r(3) + 0.025,
  % 			"END",
  % 			"fontsize", 18.0,
  % 			"fontweight", "bold");

	% Estimation poses
	for i = 1:skip:length(est.poses)
		draw_frame(est.poses{i}, 0.05);
	endfor

	% Plot settings
	view(3);
	xlabel("x [m]");
	ylabel("y [m]");
	zlabel("z [m]");
	title("Scene");
	axis "equal";
endfunction

function plot_measurements(data)
	figure();
	subplot(311);
	hold on;
	plot(data.time, data.pos(1, :), 'r-', 'linewidth', 2.0);
	plot(data.time, data.pos(2, :), 'g-', 'linewidth', 2.0);
	plot(data.time, data.pos(3, :), 'b-', 'linewidth', 2.0);
	xlabel('Time [s]');
	ylabel('Displacement [m]');

	subplot(312);
	hold on;
	plot(data.time, data.vel(1, :), 'r-', 'linewidth', 2.0);
	plot(data.time, data.vel(2, :), 'g-', 'linewidth', 2.0);
	plot(data.time, data.vel(3, :), 'b-', 'linewidth', 2.0);
	xlabel('Time [s]');
	ylabel('Velocity [ms^{-1}]');

	subplot(313);
	hold on;
	plot(data.time, data.imu_acc(1, :), 'r-', 'linewidth', 2.0);
	plot(data.time, data.imu_acc(2, :), 'g-', 'linewidth', 2.0);
	plot(data.time, data.imu_acc(3, :), 'b-', 'linewidth', 2.0);
	xlabel('Time [s]');
	ylabel('Body Acceleration [ms^{-2}]');

	figure();
	hold on;
	plot(data.time, data.imu_gyr(1, :), 'r-', 'linewidth', 2.0);
	plot(data.time, data.imu_gyr(2, :), 'g-', 'linewidth', 2.0);
	plot(data.time, data.imu_gyr(3, :), 'b-', 'linewidth', 2.0);
	xlabel('Time [s]');
	ylabel('Body Angular Velocity [rad/s]');
endfunction

function x_imu = imu_state_init()
  x_imu.p_WS = zeros(3, 1);
  x_imu.v_WS = zeros(3, 1);
  x_imu.C_WS = eye(3);
  x_imu.ba = zeros(3, 1);
  x_imu.bg = zeros(3, 1);
  % x_imu.g = [0; 0; 9.81];
  x_imu.g = [0; 0; 0];
endfunction

function x_imu = imu_rk4_update(x_imu, acc, gyr, dt)
  g = x_imu.g;
  ba = x_imu.ba;
  bg = x_imu.bg;
  na = zeros(3, 1);
  ng = zeros(3, 1);

  C_WS_k = x_imu.C_WS;
  q_WS_k = rot2quat(C_WS_k);
  v_WS_k = x_imu.v_WS;
  r_k = x_imu.p_WS;

  w = gyr - bg;
  a = acc - ba;

  % Runge-Kutta 4th Order
  % -- Integrate orientation at time k + dt (kpdt: k plus dt)
  % q_WS_kpdt = quat_integrate(q_WS_k, w, dt);
  % C_WS_kpdt = quat2rot(q_WS_kpdt);
  C_WS_kpdt = C_WS_k * so3_exp(w * dt);
  % -- Integrate orientation at time k + dt / 2 (kphdt: k plus half dt)
  % q_WS_kphdt = quat_integrate(q_WS_k, w, dt / 2);
  % C_WS_kphdt = quat2rot(q_WS_kphdt);
  C_WS_kphdt = C_WS_k * so3_exp(w * (dt / 2));
  % -- k1 = f(tn, yn)
  k1_v_dot = C_WS_k * a - g;
  k1_p_dot = v_WS_k;
  % -- k2 = f(tn + dt / 2, yn + k1 * dt / 2)
  k2_v_dot = C_WS_kphdt * acc - g;
  k2_p_dot = v_WS_k + k1_v_dot * dt / 2;
  % -- k3 = f(tn + dt / 2, yn + k2 * dt / 2)
  k3_v_dot = C_WS_kphdt * acc - g;
  k3_p_dot = v_WS_k + k2_v_dot * dt / 2;
  % -- k4 = f(tn + dt, tn + k3 * dt)
  k4_v_dot = C_WS_kpdt * acc - g;
  k4_p_dot = v_WS_k + k3_v_dot * dt;

  x_imu.C_WS = C_WS_kpdt;
  x_imu.v_WS += dt / 6 * (k1_v_dot + 2 * k2_v_dot + 2 * k3_v_dot + k4_v_dot);
  x_imu.p_WS += dt / 6 * (k1_p_dot + 2 * k2_p_dot + 2 * k3_p_dot + k4_p_dot);
endfunction

function x_imu = imu_euler_update(x_imu, acc, gyr, dt)
  g = x_imu.g;
  ba = x_imu.ba;
  bg = x_imu.bg;

  C_WS_i = x_imu.C_WS;
  v_WS_i = x_imu.v_WS;

  w = gyr - bg;
  a = acc - ba;
  dt_sq = dt * dt;
  a_WS_i = ((C_WS_i * a) - g);

  x_imu.C_WS *= so3_exp(w * dt);
  x_imu.v_WS += a_WS_i * dt;
  x_imu.p_WS += (v_WS_i * dt) + (0.5 * a_WS_i * dt_sq);
endfunction

function est = imu_batch_integrate(data)
  % Initialize IMU state
  x_imu = imu_state_init();
  x_imu.p_WS = data.pos(:, 1);
  x_imu.v_WS = data.vel(:, 1);
  x_imu.C_WS = tf_rot(data.poses{1});

  % Batch integrate IMU measurements
  est_pos = [x_imu.p_WS];
  est_vel = [x_imu.v_WS];
  est_att = [rot2euler(x_imu.C_WS)];
  est_poses{1} = tf(x_imu.C_WS, x_imu.p_WS);
  t_prev = data.time(1);
  for k = 2:length(data.time)
    % Calculate dt
    t = data.time(k);
    dt = t - t_prev;

    % Propagate IMU state
    acc = data.imu_acc(:, k);
    gyr = data.imu_gyr(:, k);
		% x_imu = imu_rk4_update(x_imu, acc, gyr, dt);
		x_imu = imu_euler_update(x_imu, acc, gyr, dt);

    % Keep track of t_prev
    est_pos = [est_pos, x_imu.p_WS];
    est_vel = [est_vel, x_imu.v_WS];
    est_att = [est_att, rot2euler(x_imu.C_WS)];
		est_poses{k} = tf(x_imu.C_WS, x_imu.p_WS);
    t_prev = t;
  endfor

  est = {};
  est.time = data.time;
  est.pos = est_pos;
  est.vel = est_vel;
  est.att = est_att;
  est.poses = est_poses;
endfunction

function plot_translation(sim, est)
	% DISPLACEMENT
	figure();

	subplot(311);
	hold on;
	plot(sim.time, sim.pos(1, :), 'r--', 'linewidth', 2.0);
	plot(est.time, est.pos(1, :), 'r-', 'linewidth', 2.0);
	xlabel("Time [s]");
	ylabel("Displacement [m]");
	legend('Target', 'Actual');

	subplot(312);
	hold on;
	plot(sim.time, sim.pos(2, :), 'g--', 'linewidth', 2.0);
	plot(est.time, est.pos(2, :), 'g-', 'linewidth', 2.0);
	xlabel("Time [s]");
	ylabel("Displacement [m]");

	subplot(313);
	hold on;
	plot(sim.time, sim.pos(3, :), 'b--', 'linewidth', 2.0);
	plot(est.time, est.pos(3, :), 'b-', 'linewidth', 2.0);
	xlabel("Time [s]");
	ylabel("Displacement [m]");
endfunction

function plot_velocity(sim, est)
	% VELOCITY
	figure();
	subplot(311);
	hold on;
	plot(sim.time, sim.vel(1, :), 'r--', 'linewidth', 2.0);
	plot(est.time, est.vel(1, :), 'r-', 'linewidth', 2.0);
	xlabel("Time [s]");
	ylabel("Velocity [m/s]");
	legend('Target', 'Actual');

	subplot(312);
	hold on;
	plot(sim.time, sim.vel(2, :), 'g--', 'linewidth', 2.0);
	plot(est.time, est.vel(2, :), 'g-', 'linewidth', 2.0);
	xlabel("Time [s]");
	ylabel("Velocity [m/s]");

	subplot(313);
	hold on;
	plot(sim.time, sim.vel(3, :), 'b--', 'linewidth', 2.0);
	plot(est.time, est.vel(3, :), 'b-', 'linewidth', 2.0);
	xlabel("Time [s]");
	ylabel("Velocity [m/s]");
endfunction

function plot_attitude(sim, est)
	% ATTITUDE
	figure();
	subplot(311);
	hold on;
	plot(sim.time, sim.att(1, :), 'r--', 'linewidth', 2.0);
	plot(est.time, est.att(1, :), 'r-', 'linewidth', 2.0);
	xlabel("Time [s]");
	ylabel("Attitude [rad]");
	legend('Target', 'Actual');

	subplot(312);
	hold on;
	plot(sim.time, sim.att(2, :), 'g--', 'linewidth', 2.0);
	plot(est.time, est.att(2, :), 'g-', 'linewidth', 2.0);
	xlabel("Time [s]");
	ylabel("Attitude [rad]");

	subplot(313);
	hold on;
	plot(sim.time, sim.att(3, :), 'b--', 'linewidth', 2.0);
	plot(est.time, est.att(3, :), 'b-', 'linewidth', 2.0);
	xlabel("Time [s]");
	ylabel("Attitude [rad]");
endfunction

% AprilGrids
rpy = deg2rad([90, 0.0, -90.0]);
% rpy = deg2rad([90, -15.0, -90.0]);
C_WF = euler321(rpy);
T_WF = eye(4);
T_WF(1:3, 1:3) = C_WF;
% T_WF(1:3, 4) = zeros(3, 1);
T_WF(1:3, 4) = [1;2;3];

aprilgrid = aprilgrid_init();
resolution = [640, 480];
cam_params = [384.607657; 385.020013; 324.260255; 237.897895];
T_FO = calib_origin(aprilgrid, resolution, cam_params);

T_offset = tf(euler321(deg2rad([0, 90, 90])), zeros(3, 1));
T_FO = T_FO * T_offset;

t_end = 2.0;
cam_rate = 100.0;
[traj0, data0] = calib_orbit_traj("UP", aprilgrid, T_WF, T_FO, cam_rate, t_end);
[traj1, data1] = calib_orbit_traj("DOWN", aprilgrid, T_WF, T_FO, cam_rate, t_end);
[traj2, data2] = calib_orbit_traj("LEFT", aprilgrid, T_WF, T_FO, cam_rate, t_end);
[traj3, data3] = calib_orbit_traj("RIGHT", aprilgrid, T_WF, T_FO, cam_rate, t_end);
[traj4, data4] = calib_fig8_traj(aprilgrid, T_WF, T_FO, cam_rate, t_end);


data = data0;
sim = {};
sim.poses = data.poses;
sim.pos = data.pos;
sim.att = data.att;
sim.vel = data.vel;
sim.time = data.time;
sim.imu_acc = data.imu_acc;
sim.imu_gyr = data.imu_gyr;
est = imu_batch_integrate(sim);

trajs = {traj0, traj1, traj2, traj3, traj4};
% trajs = {traj0};
% trajs = {traj1};
% trajs = {traj2};
% trajs = {traj3};
% trajs = {traj4};

plot_translation(sim, est);
plot_velocity(sim, est);
plot_attitude(sim, est);

% plot_sim_est(aprilgrid, T_WF, sim, est);
% plot_measurements(data);
% plot_scene(T_WF, T_FO, cam_rate, t_end, trajs, aprilgrid);
% plot_measurements(data1);
% plot_measurements(data2);
% plot_measurements(data3);
% plot_measurements(data4);
ginput();
