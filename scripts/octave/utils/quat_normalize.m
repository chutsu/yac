function q_out = quat_normalize(q)
  n = quat_norm(q);
  q_out = [q(1) / n; q(2) / n; q(3) / n; q(4) / n];
endfunction
