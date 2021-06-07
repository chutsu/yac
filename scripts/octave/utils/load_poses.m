function poses = load_poses(csv_path)
  data = csvread(csv_path, 0, 0);

  poses = {};
  for i = 1:rows(data)
    r = [data(i, 1); data(i, 2); data(i, 3)];
    q = [data(i, 4); data(i, 5); data(i, 6); data(i, 7)];
    poses{i} = tf(q, r);
  endfor
endfunction
