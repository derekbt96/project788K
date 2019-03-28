figure
start = [0 0];
start_theta = rand()*40-20;
goal = [rand()*5+20 rand()*-5]; 
goal_theta = -80;
goal_region = goal+[rand*3-1.5 rand*3-1.5];

obstacles = [6 0 1.5;
            10 -2 1;
            6 4 1.5
            15 1 2;
            19 -2 1.5;
            12 6 2];
        
% Plot Path information
% plot(final_path(:,1),final_path(:,2),'g')
hold on
% plot(Vehicle_pos.Data(:,1),Vehicle_pos.Data(:,2),'b')

% Plot obstacles
t = -.1:.1:2*pi;
for i = 1:size(obstacles,1)
    x = obstacles(i,1)+obstacles(i,3)*cos(t);
    y = obstacles(i,2)+obstacles(i,3)*sin(t);
    plot(x,y,'r')
end

% Plot start/goal
scatter(start(1),start(2),70,'g','filled');
scatter(start(1),start(2),70,'b');
plot([start(1) start(1)+cosd(start_theta)],[start(2) start(2)+sind(start_theta)],'g','LineWidth',2)

scatter(goal(1),goal(2),70,'r','filled');
scatter(goal(1),goal(2),70,'o');
plot([goal(1) goal(1)+cosd(goal_theta)],[goal(2) goal(2)+sind(goal_theta)],'r','LineWidth',2)

x = goal_region(1)+2*cos(t);
y = goal_region(2)+2*sin(t);
plot(x,y,'b')

axis equal
