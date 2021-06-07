function euler = rot2euler(R)
  q = rot2quat(R);
  euler = quat2euler(q);
endfunction
