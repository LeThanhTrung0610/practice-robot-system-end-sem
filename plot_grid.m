function plot_grid(grid_map)
    cmap = [1 1 1; ...    % white for free space
            0 0 0; ...    % black for obstacles
            1 0 0; ...    % red for obstacle value 3
            0 1 0; ...    % green for obstacle value 4
            0 0 1; ...    % blue for obstacle value 5
            1 1 0; ...    % yellow for obstacle value 6
            1 0 1];       % magenta for obstacle value 7
    colormap(cmap);
    %%
    [rows, cols] = size(grid_map);
    image(1.5, 1.5, grid_map);
    grid on
    set(gca,'xtick', 1:cols, 'ytick', 1:rows);
    axis image;
    
    for row = 1:rows
        line([1, cols + 1], [row, row], 'Color','#4DBEEE');
    end
    for col = 1:cols
        line([col, col], [1, rows + 1], 'Color','#4DBEEE');
    end
end