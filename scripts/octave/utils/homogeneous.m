function hp = homogeneous(p)
  hp = [p; ones(1, columns(p))];
endfunction
