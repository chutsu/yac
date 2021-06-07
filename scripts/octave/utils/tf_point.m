function y = tf_point(T, x)
  y = (T * [x; 1])(1:3);
endfunction
