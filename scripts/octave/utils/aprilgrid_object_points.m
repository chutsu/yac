function object_points = aprilgrid_object_points(grid)
  object_points = {};

  nb_tags = grid.rows * grid.cols;
  for id = 0:nb_tags-1;
    % Calculate the AprilGrid index using tag id
    [i, j] = aprilgrid_grid_index(grid, id);

    % Calculate the x and y of the tag origin (bottom left corner of tag)
    % relative to grid origin (bottom left corner of entire grid)
    x = j * (grid.size + grid.size * grid.spacing);
    y = i * (grid.size + grid.size * grid.spacing);

    % Bottom left
    pt_bl = [x; y; 0];
    % Bottom right
    pt_br = [x + grid.size; y; 0];
    % Top right
    pt_tr = [x + grid.size; y + grid.size; 0];
    % Top left
    pt_tl = [x; y + grid.size; 0];

    % Tag object points
    tag_points = [pt_bl, pt_br, pt_tr, pt_tl];

    % Add to total object points
    object_points{id + 1} = tag_points;
  endfor
endfunction
