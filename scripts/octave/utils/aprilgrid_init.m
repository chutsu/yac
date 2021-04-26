function grid = aprilgrid_init(rows=6, cols=6, size=0.088, spacing=0.3)
  grid = {};
  grid.tag_rows = rows;
  grid.tag_cols = cols;
  grid.tag_size = size;
  grid.tag_spacing = spacing;
  grid.object_points = aprilgrid_object_points(grid);
  grid.keypoints = {};
  grid.T_CF = eye(4);
endfunction
