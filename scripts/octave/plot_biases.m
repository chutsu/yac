sb_file = "/tmp/sensor_speed_biases.csv";

data = csvread(sb_file, 1, 0);

ts = [];
vel = [];
biases_acc = [];
biases_gyr = [];
for i = 1:rows(data)
  ts = [ts, data(i, 1)];
  vel = [vel, [data(i, 2); data(i, 3); data(i, 4)]];
  biases_acc = [biases_acc, [data(i, 8); data(i, 9); data(i, 10)]];
  biases_gyr = [biases_gyr, [data(i, 5); data(i, 6); data(i, 7)]];
endfor

t0 = ts(1)
t = [];
for i = 1:length(ts)
  t = [t, (ts(i) - t0) * 1e-9];
endfor

% figure(1);
% hold on;
% plot(t, vel(1, :), 'r-');
% plot(t, vel(2, :), 'g-');
% plot(t, vel(3, :), 'b-');
% xlim([0, t(end)]);
% title("Velocity");
% xlabel("Time [s]");
% ylabel("Velocity [ms^-1]");
% ginput();

% figure(2);
% hold on;
% subplot(311);
% plot(t, biases_acc(1, :), 'r-');
% xlim([0, t(end)]);
% xlabel("Time [s]");
% subplot(312);
% plot(t, biases_acc(2, :), 'g-');
% xlim([0, t(end)]);
% xlabel("Time [s]");
% subplot(313);
% plot(t, biases_acc(3, :), 'b-');
% xlim([0, t(end)]);
% xlabel("Time [s]");
% title("Accelerometer Biases");

figure(3);
hold on;
subplot(311);
plot(t, biases_gyr(1, :), 'r-');
xlim([0, t(end)]);
xlabel("Time [s]");
subplot(312);
plot(t, biases_gyr(2, :), 'g-');
xlim([0, t(end)]);
xlabel("Time [s]");
subplot(313);
plot(t, biases_gyr(3, :), 'b-');
xlim([0, t(end)]);
xlabel("Time [s]");

% figure(3);
% hold on;
% plot(t, biases_gyr(1, :), 'r-');
% plot(t, biases_gyr(2, :), 'g-');
% plot(t, biases_gyr(3, :), 'b-');
% xlim([0, t(end)]);
% title("Gyroscope Biases");
% xlabel("Time [s]");

ginput();
