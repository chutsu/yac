function draw_aprilgrid(aprilgrid)
  for i = 1:length(aprilgrid.object_points)
    pts = aprilgrid.object_points{i};
    for j = 1:4
      plot3(pts(1, j), pts(2, j), pts(3, j), 'ro', 'markersize', 5.0);
    endfor
  endfor
endfunction
