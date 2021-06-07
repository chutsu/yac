calib_poses = "/tmp/calib_poses.csv";

% Load data
T_FC = load_poses(calib_poses);

C_WF = euler321(deg2rad([90.0, 0.0, -90.0]));
T_WF = [C_WF, zeros(3, 1);
        zeros(1, 3), 1.0];

figure(1);
hold on;

draw_frame(T_WF);

tag_rows = 6;
tag_cols = 6;
tag_size = 0.088;
tag_spacing = 0.3;

spacing_x = (tag_cols - 1) * tag_spacing * tag_size;
spacing_y = (tag_rows - 1) * tag_spacing * tag_size;
target_width = tag_cols * tag_size + spacing_x;
target_height = tag_rows * tag_size + spacing_y;

origin = tf_trans(T_WF);
r_FF10 = [target_width; 0.0; 0.0];
r_FF01 = [0.0; target_height; 0.0];
r_FF11 = [target_width; target_height; 0.0];

r_WF10 = tf_point(T_WF, r_FF10);
r_WF01 = tf_point(T_WF, r_FF01);
r_WF11 = tf_point(T_WF, r_FF11);

plot3([origin(1), r_WF10(1)], ...
      [origin(2), r_WF10(2)], ...
      [origin(3), r_WF10(3)], 'r',
      "linewidth", 5)

plot3([origin(1), r_WF01(1)], ...
      [origin(2), r_WF01(2)], ...
      [origin(3), r_WF01(3)], 'r',
      "linewidth", 5)

plot3([r_WF01(1), r_WF11(1)], ...
      [r_WF01(2), r_WF11(2)], ...
      [r_WF01(3), r_WF11(3)], 'r',
      "linewidth", 5)

plot3([r_WF11(1), r_WF10(1)], ...
      [r_WF11(2), r_WF10(2)], ...
      [r_WF11(3), r_WF10(3)], 'r',
      "linewidth", 5)

for i = 1:length(T_FC)
  draw_frame(T_WF * T_FC{i});
endfor
xlabel "x";
ylabel "y";
zlabel "z";
axis 'equal';
view(3);
ginput();
