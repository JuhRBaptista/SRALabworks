function [map] = loadMap(map_name)

    try
        image_name = sprintf('../maps/%s_grid5.png', map_name);
        image = imread(image_name);

    catch
        image = imread(map_name);
    end

    map = flipud(1 - double(image(:, :, 1)) ./ 255);
end