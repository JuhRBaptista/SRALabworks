close all;
clear all;

cell_size = 0.05;     % 5 cm
map_size_m = 4;       % 4 meters

n_cells = map_size_m / cell_size;  % 80

obstacle_probability1 = 0.1;  % 10% of the cells are obstacles
map1 = zeros(n_cells);              % initialize as empty
map1 = rand(n_cells) < obstacle_probability1;
map1 = double(map1);           % make sure it's 0/1

obstacle_probability2 = 0.25;  % 20% of the cells are obstacles
map2 = zeros(n_cells);              % initialize as empty
map2 = rand(n_cells) < obstacle_probability2;
map2 = double(map2);           % make sure it's 0/1

obstacle_probability3 = 0.4;  % 40% of the cells are obstacles
map3 = zeros(n_cells);              % initialize as empty
map3 = rand(n_cells) < obstacle_probability3;
map3 = double(map3);           % make sure it's 0/1


start1 = [1, 1];
goal1 = [80, 80];

start2 = [1, 80];
goal2 = [80, 1];

start3 = [80, 80];
goal3 = [1, 1];

path1 = aStar(map1, start1, goal1);
path2 = aStar(map2, start2, goal2);
path3 = aStar(map3, start3, goal3);


%% MAP 1
subplot(1,3,1)
imagesc(map1)
colormap(flipud(gray))
axis equal tight
hold on

if ~isempty(path1)
    plot(path1(:,2), path1(:,1), 'r-o','LineWidth',2)
end

plot(start1(2), start1(1),'go','MarkerFaceColor','g')
plot(goal1(2), goal1(1),'bo','MarkerFaceColor','b')

title('Map 1')
set(gca,'YDir','reverse')


%% MAP 2
subplot(1,3,2)
imagesc(map2)
colormap(flipud(gray))
axis equal tight
hold on

if ~isempty(path2)
    plot(path2(:,2), path2(:,1), 'r-o','LineWidth',2)
end

plot(start2(2), start2(1),'go','MarkerFaceColor','g')
plot(goal2(2), goal2(1),'bo','MarkerFaceColor','b')

title('Map 2')
set(gca,'YDir','reverse')


%% MAP 3
subplot(1,3,3)
imagesc(map3)
colormap(flipud(gray))
axis equal tight
hold on

if ~isempty(path3)
    plot(path3(:,2), path3(:,1), 'r-o','LineWidth',2)
end

plot(start3(2), start3(1),'go','MarkerFaceColor','g')
plot(goal3(2), goal3(1),'bo','MarkerFaceColor','b')

title('Map 3')
set(gca,'YDir','reverse')