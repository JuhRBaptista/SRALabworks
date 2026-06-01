function [map] = loadMap(map_name, flip)
    
    if nargin < 2
        flip = false;
    end
    try
        image_name = sprintf('../maps/%s_grid5.png', map_name);
        image = imread(image_name);

    catch
        image = imread(map_name);
    end

    map = 1 - double(image(:, :, 1)) ./ 255;
    
    if flip
        map = flipud(map);
    end
end