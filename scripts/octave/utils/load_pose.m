function pose = load_pose(csv_path)
  poses = load_poses(csv_path);
  pose = poses{1};
endfunction
