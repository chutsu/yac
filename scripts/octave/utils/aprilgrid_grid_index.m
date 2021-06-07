function [i, j] = aprilgrid_grid_index(grid, id)
  assert(id < (grid.rows * grid.cols) && id >= 0);
  i = floor(id / grid.cols);
  j = floor(rem(id, grid.cols));
endfunction
