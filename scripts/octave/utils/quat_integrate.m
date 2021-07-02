function q_kp1 = quat_integrate(q_k, w, dt)
  % "Quaternion kinematics for the error-state Kalman filter" (2017)
  % By Joan Sola
  % [Section 4.6.1 Zeroth-order integration, p.47]
  w_norm = norm(w);
  q_scalar = 0.0;
  q_vec = [0.0; 0.0; 0.0];

  if (w_norm > 1e-5)
    q_scalar = cos(w_norm * dt * 0.5);
    q_vec = w / w_norm * sin(w_norm * dt * 0.5);
  else
    q_scalar = 1.0;
    q_vec = zeros(3, 1);
  endif

  q_kp1 = quat_mul(q_k, [q_scalar; q_vec]);
endfunction
