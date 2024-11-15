clear;
clc;

% Input Parameters
ship_speed_calm = 14; % Ship speed in calm sea conditions (knots)
ship_heading = 90; % Initial ship heading (degrees, 90 = East)
lat_min = 36.399374008; % Minimum latitude
lat_max = 46.985193253; % Maximum latitude
lon_min = 6.294375240; % Minimum longitude
lon_max = 20.885496319; % Maximum longitude
start_lat = 38.3889447975; % Starting latitude
start_lon = 7.65; % Starting longitude
end_lat = 42.1205435649; % Destination latitude
end_lon = 11.7677567687; % Destination longitude
total_time_with_astar = 0; % Initialize total travel time

% Define path to files
file_path = "C:\Users\Univ ' Parthenope '\Documents\MATLAB";

% Helper Functions
function [lat, lon] = pixel_to_wgs84(row, col, lat_min, lat_max, lon_min, lon_max, res_lat, res_lon)
    lat = lat_max - (row * res_lat);
    lon = lon_min + (col * res_lon);
end

function pixel_coord = wgs84_to_pixel(lat, lon, lat_min, lat_max, lon_min, lon_max, res_lat, res_lon, rows, cols)
    row = round((lat_max - lat) / res_lat);
    col = round((lon - lon_min) / res_lon);
    row = max(1, min(row, rows));
    col = max(1, min(col, cols));
    pixel_coord = [row, col];
end

function speed = calculate_speed(wave_height, wave_direction, wind_speed, current_speed, ship_heading, ship_speed_calm)
    % Calcola la differenza angolare
    angle_diff = mod(abs(wave_direction - ship_heading), 360);
    if angle_diff > 180
        angle_diff = 360 - angle_diff;
    end
    
    % Riduzione della velocità basata sull'angolo
    if angle_diff < 45
        speed_reduction = 0.7;
    elseif angle_diff >= 45 && angle_diff <= 135
        speed_reduction = 0.85;
    else
        speed_reduction = 0.9;
    end
    
    % Riduzione aggiuntiva basata sull'altezza delle onde
    if wave_height > 2.5
        speed_reduction = speed_reduction * 0.7;
    elseif wave_height > 1.5
        speed_reduction = speed_reduction * 0.85;
    end
    
    % Considera la velocità del vento e delle correnti
    wind_effect = wind_speed * 0.1; % Placeholder per il calcolo dell'effetto del vento
    current_effect = current_speed * 0.2; % Placeholder per il calcolo dell'effetto delle correnti
    
    % Calcola la velocità finale
    speed = ship_speed_calm * speed_reduction - wind_effect - current_effect;
    speed = max(speed, 0); % Assicurarsi che la velocità non sia negativa
end

function [route, total_time] = astar(speed_grid, start_point, end_point)
    [rows, cols] = size(speed_grid);
    
    % Verifica che i punti di partenza e arrivo siano validi
    if start_point(1) < 1 || start_point(1) > rows || start_point(2) < 1 || start_point(2) > cols
        error('Start point is out of bounds');
    end
    if end_point(1) < 1 || end_point(1) > rows || end_point(2) < 1 || end_point(2) > cols
        error('End point is out of bounds');
    end
    
    cost = inf(rows, cols);
    heuristic = inf(rows, cols);
    visited = false(rows, cols);
    cost(start_point(1), start_point(2)) = 0;
    heuristic(start_point(1), start_point(2)) = haversine(start_point(1), start_point(2), end_point(1), end_point(2));
    prev = cell(rows, cols);
    pq = [heuristic(start_point(1), start_point(2)), start_point];
    directions = [0, 1; 1, 0; 0, -1; -1, 0; 1, 1; -1, -1; 1, -1; -1, 1];

    while ~isempty(pq)
        pq = sortrows(pq, 1);
        current = pq(1, :);
        pq(1, :) = [];
        cur_heuristic = current(1);
        cur_point = current(2:3);

        if visited(cur_point(1), cur_point(2))
            continue;
        end

        visited(cur_point(1), cur_point(2)) = true;
        if all(cur_point == end_point)
            break;
        end

        for d = 1:size(directions, 1)
            new_point = cur_point + directions(d, :);
            if new_point(1) > 0 && new_point(1) <= rows && new_point(2) > 0 && new_point(2) <= cols
                new_cost = cost(cur_point(1), cur_point(2)) + (1 / speed_grid(new_point(1), new_point(2)));
                new_heuristic = new_cost + haversine(new_point(1), new_point(2), end_point(1), end_point(2));
                if new_heuristic < heuristic(new_point(1), new_point(2))
                    cost(new_point(1), new_point(2)) = new_cost;
                    heuristic(new_point(1), new_point(2)) = new_heuristic;
                    prev{new_point(1), new_point(2)} = cur_point;
                    pq = [pq; new_heuristic, new_point];
                end
            end
        end
    end

    route = [];
    total_time = cost(end_point(1), end_point(2));
    point = end_point;

    while ~isempty(point)
        route = [point; route];
        point = prev{point(1), point(2)};
    end
end

function [lat, lon] = loxodromic_interpolation(start_lat, start_lon, end_lat, end_lon, num_points)
    % Interpolazione loxodromica tra due punti
    lat = linspace(start_lat, end_lat, num_points);
    lon = linspace(start_lon, end_lon, num_points);
end

% Initial Plot
figure;
hold on;
xlabel('Longitude');
ylabel('Latitude');
title('Ship Route Comparison');

% Plot starting and ending points
plot(start_lon, start_lat, 'go', 'MarkerSize', 10, 'DisplayName', 'Start');
plot(end_lon, end_lat, 'ro', 'MarkerSize', 10, 'DisplayName', 'End');

% Generate loxodromic route
num_points = 100; % Define how many points to interpolate
[loxodromic_lat, loxodromic_lon] = loxodromic_interpolation(start_lat, start_lon, end_lat, end_lon, num_points);
plot(loxodromic_lon, loxodromic_lat, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Loxodromic Route (OR)');

% Calculate the time for the loxodromic route
total_distance = haversine(start_lat, start_lon, end_lat, end_lon);
loxodromic_time = total_distance / ship_speed_calm;

% Time steps
time_stamps = [9, 12, 15, 18, 21];
previous_waypoint = [];
all_route_latlon = [];

for i = 1:length(time_stamps)
    time_stamp = sprintf('%02d', time_stamps(i));
    wave_height_file = fullfile(file_path, sprintf('HS_21NOV22_%s.tif', time_stamp));
    wave_direction_file = fullfile(file_path, sprintf('DIR_21NOV22_%s.tif', time_stamp));

    % Read wave height and direction
    if ~isfile(wave_height_file) || ~isfile(wave_direction_file)
        error('Wave file does not exist: %s', time_stamp);
    end
    wave_height = imread(wave_height_file);
    wave_direction = imread(wave_direction_file);

    % Read wind speed and current speed if available
    wind_speed = 0; % Placeholder, replace with actual data if available
    current_speed = 0; % Placeholder, replace with actual data if available

    [rows, cols] = size(wave_height);
    res_lat = (lat_max - lat_min) / (rows / 4); % Riduci ulteriormente la risoluzione
    res_lon = (lon_max - lon_min) / (cols / 4); % Riduci ulteriormente la risoluzione

    if i == 1
        start_point = wgs84_to_pixel(start_lat, start_lon, lat_min, lat_max, lon_min, lon_max, res_lat, res_lon, rows, cols);
    else
        start_point = previous_waypoint;
    end

    end_point = wgs84_to_pixel(end_lat, end_lon, lat_min, lat_max, lon_min, lon_max, res_lat, res_lon, rows, cols);

    % Verifica che i punti di partenza e arrivo siano validi
    if start_point(1) < 1 || start_point(1) > rows || start_point(2) < 1 || start_point(2) > cols
        error('Start point is out of bounds');
    end
    if end_point(1) < 1 || end_point(1) > rows || end_point(2) < 1 || end_point(2) > cols
        error('End point is out of bounds');
    end

    speed_grid = zeros(rows, cols);
    for r = 1:rows
        for c = 1:cols
            speed_grid(r, c) = calculate_speed(wave_height(r, c), wave_direction(r, c), wind_speed, current_speed, ship_heading, ship_speed_calm);
        end
    end

    [optimal_route, segment_time] = astar(speed_grid, start_point, end_point);
    total_time_with_astar = total_time_with_astar + segment_time;

    % Store the optimal route
    if ~isempty(optimal_route)
        for k = 1:size(optimal_route, 1)
            [route_lat, route_lon] = pixel_to_wgs84(optimal_route(k, 1), optimal_route(k, 2), lat_min, lat_max, lon_min, lon_max, res_lat, res_lon);
            all_route_latlon = [all_route_latlon; route_lat, route_lon];
        end
        previous_waypoint = optimal_route(end, :);
    end
end

% Plot the weather route
if ~isempty(all_route_latlon)
    plot(all_route_latlon(:, 2), all_route_latlon(:, 1), '-m', 'LineWidth', 1.5, 'DisplayName', 'Weather Route (WR)');
end

% Display the total times
fprintf('Total route time (Loxodromic Route): %.2f hours\n', loxodromic_time);
fprintf('Total route time (Weather Route): %.2f hours\n', total_time_with_astar);

legend;
hold off;

% Create and save shapefiles
loxodromic_shapefile = struct('Geometry', 'Line', 'X', loxodromic_lon, 'Y', loxodromic_lat);
shapewrite(loxodromic_shapefile, 'loxodromic_route.shp');

if ~isempty(all_route_latlon)
    weather_shapefile = struct('Geometry', 'Line', 'X', all_route_latlon(:, 2), 'Y', all_route_latlon(:, 1));
    shapewrite(weather_shapefile, 'weather_route.shp');
end

% Haversine function to calculate distance
function distance = haversine(lat1, lon1, lat2, lon2)
    R = 3440.065; % Earth radius in nautical miles
    d_lat = deg2rad(lat2 - lat1);
    d_lon = deg2rad(lon2 - lon1);
    a = sin(d_lat / 2) ^ 2 + cos(deg2rad(lat1)) * cos(deg2rad(lat2)) * sin(d_lon / 2) ^ 2;
    c = 2 * atan2(sqrt(a), sqrt(1 - a));
    distance = R * c; % Distance in nautical miles
end
