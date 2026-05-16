function [map] = loadMap(map_name)

    image_name = sprintf('maps/%s_grid5.png', map_name);
    image = imread(image_name);
    map = 1 - double(image(:, :, 1))./255;
end