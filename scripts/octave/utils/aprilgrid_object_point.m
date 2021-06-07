function p = aprilgrid_object_point(aprilgrid, tag_id, corner_idx)
  tag_rows = aprilgrid.tag_rows;
  tag_cols = aprilgrid.tag_cols;
  tag_size = aprilgrid.tag_size;
  tag_spacing = aprilgrid.tag_spacing;

  % Calculate the AprilGrid index using tag id and tag_cols
  row = floor(tag_id / tag_cols);
  col = floor(rem(tag_id, tag_cols));
  % ^ row and col are the i-th and j-th row and col from bottom left

  % Caculate the x and y of the tag origin (bottom left corner of tag)
  % relative to grid origin (bottom left corner of entire grid)
  x = col * (tag_size + tag_size * tag_spacing);
  y = row * (tag_size + tag_size * tag_spacing);

  % Calculate the x and y of each corner
  if corner_idx == 0
    p = [x; y; 0];  % Bottom left
  elseif corner_idx == 1
    p = [x + tag_size; y; 0];  % Bottom right
  elseif corner_idx == 2
    p = [x + tag_size; y + tag_size; 0];  % Top right
  elseif corner_idx == 3
    p = [x; y + tag_size; 0];  % Top left
  end
endfunction
