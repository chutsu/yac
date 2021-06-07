function r = quat_lmul(p, q)
  assert(size(p) == [4, 1]);
  assert(size(q) == [4, 1]);

  pw = p(1);
  px = p(2);
  py = p(3);
  pz = p(4);

  lprod = [
    pw, -px, -py, -pz;
    px, pw, -pz, py;
    py, pz, pw, -px;
    pz, -py, px, pw;
  ];

  r = lprod * q;
endfunction
