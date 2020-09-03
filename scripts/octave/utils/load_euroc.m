function data = load_euroc(data_path)
  cam0_data = strcat(data_path, "/mav0/cam0/data");
  cam1_data = strcat(data_path, "/mav0/cam1/data");
  cam0_csv = strcat(data_path, "/mav0/cam0/data.csv");
  cam1_csv = strcat(data_path, "/mav0/cam1/data.csv");
  imu_csv = strcat(data_path, "/mav0/imu0/data.csv");

  % Load camera data
  [cam0_ts, cam0_images] = textread(cam0_csv, ...
                                    "%f %s", ...
                                    "delimiter", ",", ...
                                    "headerlines", 1);
  [cam1_ts, cam1_images] = textread(cam1_csv, ...
                                    "%f %s", ...
                                    "delimiter", ",", ...
                                    "headerlines", 1);

  nb_cam0_images = rows(cam0_images);
  for i = 1:nb_cam0_images
    cam0_images(i, 1) = strcat(cam0_data, "/", cam0_images{i});
  end

  nb_cam1_images = rows(cam1_images);
  for i = 1:nb_cam1_images
    cam1_images(i, 1) = strcat(cam1_data, "/", cam1_images{i});
  end

  % Load IMU data
  [imu_ts, ...
   w_RS_S_x, w_RS_S_y, w_RS_S_z, ...
   a_RS_S_x, a_RS_S_y, a_RS_S_z] = textread( ...
    imu_csv, ...
    "%f %f %f %f %f %f %f", ...
    "delimiter", ",", ...
    "headerlines", 1 ...
  );
  w_RS_S = [w_RS_S_x'; w_RS_S_y'; w_RS_S_z'];
  a_RS_S = [a_RS_S_x'; a_RS_S_y'; a_RS_S_z'];
  nb_imu_measurements = rows(imu_ts);

  # Find the first timestamp
  cam0_ts_min = min(cam0_ts);
  cam1_ts_min = min(cam1_ts);
  imu_ts_min = min(imu_ts);
  t0 = min(min(cam0_ts_min, cam1_ts_min), imu_ts_min);

  % Form struct
  data.t0 = t0 * 1e-9;
  data.cam0.data_path = cam0_data;
  data.cam1.data_path = cam1_data;
  data.cam0.image_paths = cam0_images;
  data.cam1.image_paths = cam1_images;
  data.cam0.ts = cam0_ts;
  data.cam1.ts = cam1_ts;
  cam0_ts_start = data.cam0.ts(1) * ones(nb_cam0_images);
  cam1_ts_start = data.cam1.ts(1) * ones(nb_cam1_images);
  data.cam0.time = (data.cam0.ts - t0) * 1.0e-9;
  data.cam1.time = (data.cam1.ts - t0) * 1.0e-9;

  data.imu.ts = imu_ts;
  imu_ts_start = data.imu.ts(1) * ones(nb_imu_measurements, 1);
  data.imu.time = (data.imu.ts - t0) * 1.0e-9;
  data.imu.w_RS_S = w_RS_S;
  data.imu.a_RS_S = a_RS_S;
endfunction
