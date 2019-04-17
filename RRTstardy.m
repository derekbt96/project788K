%% Initialize
clc
clear all

tic

start = [0 0];
goal = [9 9];
goal_vel = [0 -1.5];

accel_bound = .75;
vel_bound = 3;

epsilon = .8;
dt = .7;

goal_rad = 1;
goal_vel_rad = .75;
greed = .00; 

mu = 3;
max_init_interations = 1000;
G = [];

for k = 1:max_init_interations
    G = [G Node()];
end

finish_id = max_init_interations;

G(1).parent = 0;
G(1).Index = 1;
G(1).X = start(1);
G(1).Y = start(2);
G(1).dx = 1;
G(1).dy = 0;
G(1).cost = 0;
G(1).px = 0;
G(1).py = 0;

obstacles = [5 5 2;
            7 2 1;
            4 9 1.5];

path_cost = 10000000;


% find path to goal
tic
for k = 2:max_init_interations
    
% 		compute points until valid point is found
    while 1

% 			get random point in bounds
        greedtest = rand();
        if (finish_id == max_init_interations && greedtest < greed)
            rand_x = goal(1);
            rand_y = goal(2);
            rand_vx = goal_vel(1);
            rand_vy = goal_vel(2);
            fprintf('greed')
        else
            rand_x = rand() * 10.0;
            rand_y = rand() * 10.0;
            rand_vx = rand() * vel_bound;
            rand_vy = rand() * vel_bound;
            rand_vx = rand_vx/hypot(rand_vy,rand_vx);
            rand_vy = rand_vy/hypot(rand_vy,rand_vx);
        end
       
        parent_id = 0;
        min_accel = [0 0 Inf];
        cost = Inf;
        for j = 1:k-1
            temp_ax = (rand_x - G(j).X - G(j).dx*dt)*2/(dt^2);
            temp_ay = (rand_y - G(j).Y - G(j).dy*dt)*2/(dt^2);
            temp_accel = hypot(temp_ax,temp_ay);
            temp_vel = hypot(rand_vx-(G(j).dx),rand_vy-(G(j).dy));
            temp_cost = .5*temp_accel+1.5*temp_vel;
            if (temp_cost < cost)
                parent_id = j;
                min_accel = [temp_ax,temp_ay,temp_accel];
                cost = temp_cost;
            end
        end
        
        
        if (min_accel(3) > accel_bound)
            min_accel = min_accel*(accel_bound/min_accel(3));
        end
        bounded = 1;
        final_x = G(parent_id).X + G(parent_id).dx*dt+.5*min_accel(1)*dt^2;
        final_y = G(parent_id).Y + G(parent_id).dy*dt+.5*min_accel(2)*dt^2;
        final_dx = G(parent_id).dx + min_accel(1)*dt;
        final_dy = G(parent_id).dy + min_accel(2)*dt;
        if hypot(final_dx, final_dy) > vel_bound
            bounded = 0;
        end
        
        
        parentloc = [G(parent_id).X G(parent_id).Y];
        parentvel = [G(parent_id).dx G(parent_id).dy];
        final_location = [final_x final_y];
        if (~collisionDy(parentloc, parentvel,min_accel(1:2),dt,obstacles) && bounded)
			fprintf("Iteration: %d added at: %f %f \n", k, final_x, final_y);
            G(k).Index = k;
            G(k).X = final_x;
            G(k).Y = final_y;
            G(k).dx = final_dx;
            G(k).dy = final_dy;
            G(k).ax = min_accel(1);
            G(k).ay = min_accel(2);
            G(k).a = min_accel(3);
            G(k).vel = hypot(final_dx,final_dy);
            G(k).parent = parent_id;
            G(k).cost = G(parent_id).cost+dt*min_accel(3);
            
            
            
%             Rewire
            for j = 1:k-1
                temp_ax = (G(j).X - G(k).X - G(k).dx*dt)*2/(dt^2);
                temp_ay = (G(j).Y - G(k).Y - G(k).dy*dt)*2/(dt^2);
                temp_dx = G(j).dx + temp_ax*dt;
                temp_dy = G(j).dy + temp_ay*dt;
                temp_accel = hypot(temp_ax,temp_ay);
                vel_error = hypot(G(j).dx-temp_dx,G(j).dy-temp_dy);
                if temp_accel < accel_bound && vel_error < .5
                    temp_cost = G(k).cost + dt*temp_accel;
                    if (temp_cost < G(j).cost)
                        G(j).parent = k;
                        G(j).ax = temp_ax;
                        G(j).ay = temp_ay;
                        G(j).a = temp_accel;
                        fprintf("Rewired %d to %d\n", j, k);
                    end
                end
            end
            break;
        end
    end 
    goal_dist = hypot(G(k).X - goal(1),G(k).Y - goal(2));
    vel_dist = hypot(G(k).dx - goal_vel(1),G(k).dy - goal_vel(2));
    if (goal_dist <= goal_rad && vel_dist < goal_vel_rad)
        fprintf("Found last, id: %d\n",k); 
        finish_id = k;
        break;
    end
end
toc
%% Plotting stuff

hold on
plot_cost = 1;
if plot_cost == 0
    for k = 2:max_init_interations
        plot([G(k).X G(G(k).parent).X],[G(k).Y G(G(k).parent).Y],'b')
    end
else
    max_cost = 0;
    for k = 2:max_init_interations
        if max_cost < G(k).vel
            max_cost = G(k).vel;
        end
    end
    for k = 2:max_init_interations
        plot([G(k).X G(G(k).parent).X],[G(k).Y G(G(k).parent).Y],'Color',[G(k).vel/max_cost 1-G(k).vel/max_cost 0])
    end
end
t = -.1:.1:2*pi;
for i = 1:size(obstacles,1)
    x = obstacles(i,1)+obstacles(i,3)*cos(t);
    y = obstacles(i,2)+obstacles(i,3)*sin(t);
    plot(x,y,'r')
end

if finish_id ~= max_init_interations
    final_path = [G(finish_id).X G(finish_id).Y finish_id];
    id_temp = finish_id;
else
    id_temp = 1;
    temp = hypot(G(1).X-goal(1),G(1).Y-goal(2))+hypot(G(1).dx-goal_vel(1),G(1).dy-goal_vel(2));
    for k = 2:max_init_interations
        temp2 = hypot(G(k).X-goal(1),G(k).Y-goal(2))+hypot(G(k).dx-goal_vel(1),G(k).dy-goal_vel(2));
        if max_cost < temp2
            id_temp = k;
            temp = temp2;
        end
    end
end
final_path = [];
while id_temp ~= 1
    id_temp = G(id_temp).parent;
    final_path = [final_path; G(id_temp).X G(id_temp).Y id_temp];
end
final_path = [flipud(final_path); goal 0]


% Plot path taken
plot(final_path(:,1),final_path(:,2),'b','Linewidth',2)
scatter(final_path(:,1),final_path(:,2),'b')

axis equal
%%
% for k = 2:max_init_interations
%     if G(k).state == 1
%         plot([G(k).X G(G(k).parent).X],[G(k).Y G(G(k).parent).Y],'m')
%     end
% end
% 
% % Plot path taken
% plot(final_path(:,1),final_path(:,2),'g')
% 
% % Plot path taken


