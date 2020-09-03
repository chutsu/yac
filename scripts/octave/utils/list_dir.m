function result = list_dir(target_dir)
  listing = dir(target_dir);
  result = [];

  for i = 1:length(listing)
    if any(strcmp(listing(i).name, {'.', '..'})) == 0
      target.name = listing(i).name;
      target.date = listing(i).date;
      target.bytes = listing(i).bytes;
      target.isdir = listing(i).isdir;

      result = [result; target];
    end
  end
endfunction
