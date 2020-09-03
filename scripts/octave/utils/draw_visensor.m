function draw_visensor(T_WS, T_SC0, T_SC1)
  draw_frame(T_WS, 0.1);
  draw_camera(T_WS * T_SC0, scale=0.05, 'r');
  draw_camera(T_WS * T_SC1, scale=0.05, 'g');
endfunction
