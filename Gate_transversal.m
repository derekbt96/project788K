goal_location = [1 1];
theta = rand()*.7-.35;
while norm(goal_location) > 1
    goal_location = [rand() rand()];
end

target_pos = [4 0];
final_pos = [0 0];
final_theta = rand()*.5-.25;
while norm(final_pos-target_pos) > 1
    final_pos = [rand()-.5 rand()-.5]+target_pos;
end

goal_width = 1;
hold on
plot([goal_location(1)-goal_width/2*cos(theta-pi/2) goal_location(1)+goal_width/2*cos(theta-pi/2)],[goal_location(2)-goal_width/2*sin(theta-pi/2) goal_location(2)+goal_width/2*sin(theta-pi/2)],'Color',[1 .5 0],'Linewidth',2)
scatter([final_pos(1) goal_location(1)],[final_pos(2) goal_location(2)])

v1 = 1;
v2 = 2;

p1 = final_pos;
p2 = final_pos-v1*[cos(final_theta) sin(final_theta)];
p3 = goal_location+v2*[cos(theta) sin(theta)];
p4 = goal_location;
pts = [p1; p2; p3; p4];
scatter(pts(1:2,1),pts(1:2,2),'g')
scatter(pts(3:4,1),pts(3:4,2),'r')
t = 0:.01:1;
s1 = (1-t).^3*p1(1)+3*(1-t).^2.*t*p2(1)+3*(1-t).*t.^2*p3(1)+t.^3*p4(1);
s2 = (1-t).^3*p1(2)+3*(1-t).^2.*t*p2(2)+3*(1-t).*t.^2*p3(2)+t.^3*p4(2);
plot(s1,s2)
axis equal

