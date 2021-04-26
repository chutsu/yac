function T_target_camera = lookat(cam_pos, target, up_axis=[0; -1; 0])
  % Note: If we were using OpenGL the cam_dir would be the opposite direction,
  % since in OpenGL the camera forward is -z. In robotics however our camera
  % is +z forward.
  cam_dir = normalize(target - cam_pos);
  cam_right = cross(up_axis, cam_dir);
  cam_up = cross(cam_dir, cam_right);

  A = [cam_right(1), cam_right(2), cam_right(3), 0.0;
       cam_up(1), cam_up(2), cam_up(3), 0.0;
       cam_dir(1), cam_dir(2), cam_dir(3), 0.0;
       0.0, 0.0, 0.0, 1.0];

  B = [1.0, 0.0, 0.0, -cam_pos(1);
       0.0, 1.0, 0.0, -cam_pos(2);
       0.0, 0.0, 1.0, -cam_pos(3);
       0.0, 0.0, 0.0, 1.0];

  T_camera_target = A * B;
  T_target_camera = inv(T_camera_target);
endfunction
