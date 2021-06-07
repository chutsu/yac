function aprilgrid_draw(aprilgrid, T_WF)
  p_WF = dehomogeneous(T_WF * homogeneous(aprilgrid.object_points));
  scatter3(p_WF(1, :), p_WF(2, :), p_WF(3, :), "filled");
  draw_frame(T_WF, aprilgrid.tag_size);
endfunction
