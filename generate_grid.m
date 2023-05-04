function grid_map = generate_grid(size, obstacle)
%%
    grid_map = ones(size(1), size(2));
    grid_map(obstacle) = 2;
end
