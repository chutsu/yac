function object_points = aprilgrid_object_points(aprilgrid)
  tag_id = 0;
  object_points = [];

  for i = 1:aprilgrid.tag_rows
    for j = 1:aprilgrid.tag_cols
      for corner_idx = 0:3
        p = aprilgrid_object_point(aprilgrid, tag_id, corner_idx);
        object_points = [object_points, p];
      endfor
      tag_id += 1;
    endfor
  endfor
endfunction
