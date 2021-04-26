function q = vecs2quat(u, v)
  cos_theta = transpose(normalize(u)) * normalize(v);
  half_cos = sqrt(0.5 * (1.0 + cos_theta));
  half_sin = sqrt(0.5 * (1.0 - cos_theta));
  w = normalize(cross(u, v));
  q = [half_cos; half_sin * w(1); half_sin * w(2); half_sin * w(3)];
endfunction
