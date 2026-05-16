% Map 1: Reference path
map1 = [0 1 0 0;
        0 1 0 1;
        0 0 0 1;
        0 1 1 0];

% Map 2: U-turn path
map2 = [0 1 1 1 0;
        0 0 1 0 0;
        0 1 1 1 0;
        0 1 0 1 0;
        0 0 0 0 0];

% Map 3: Dead-end
map3 = [0 0 0 0 0;
        0 1 0 1 0;
        0 1 0 1 0;
        1 1 1 1 0;
        0 0 0 0 0];

start1 = [1, 1];
goal1 = [4, 4];

path1 = aStar(map1, start1, goal1);

start2 = [1, 1];
goal2= [1, 5];

path2 = aStar(map2, start2, goal2);

start3 = [1, 1];
goal3= [5, 1];

path3 = aStar(map3, start3, goal3);

figure;

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

title('Reference Map')
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

title('U Shapped')
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

title('Dead End Scenarios')
set(gca,'YDir','reverse')