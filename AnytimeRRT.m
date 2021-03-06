% Anytime RRT*
%% Initialize
clc
clear all

tic

start = [0 0];
goal = [10 10];
speed_bound = 1.5;

epsilon = .8;
goal_rad = epsilon;
greed = .05; 

mu = epsilon;
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
G(1).cost = 0;

obstacles = [5 5 2;
            7 2 1;
            4 9 1.5];

path_cost = 10000000;
tic
%% find path to goal
for k = 2:max_init_interations
    
% 		compute points until valid point is found
    while 1

% 			get random point in bounds
        greedtest = rand();
        if (finish_id == max_init_interations && greedtest > greed)
            rand_x = goal(1);
            rand_y = goal(2);
        else
            rand_x = rand() * 10.0;
            rand_y = rand() * 10.0;
            rand_v = rand() * speed_bound;
        end

% 			find closest node in array to random point
        parent_id = 0;
        closest_dist = 1000.0;
        for j = 1:k-1
            temp_dist = hypot(G(j).X - rand_x,G(j).Y - rand_y);
            if (temp_dist < closest_dist)
                parent_id = j;
                closest_dist = temp_dist;
            end   
        end

        
        
% 			move random point to epsilon distance if further than that
        if (closest_dist < epsilon)
            final_x = rand_x;
            final_y = rand_y;
        else
            final_x = G(parent_id).X + (epsilon / closest_dist) * (rand_x - G(parent_id).X);
            final_y = G(parent_id).Y + (epsilon / closest_dist) * (rand_y - G(parent_id).Y);
        end

        
% 			if new node is not colliding with obstacles add it to array and break while loop to continue to next node
        parentloc = [G(parent_id).X G(parent_id).Y];
        final_location = [final_x final_y];
        if (~collision(parentloc, final_location,obstacles))
% 			fprintf("Iteration: %d added at: %f %f \n", k, final_x, final_y);
            G(k).Index = k;
            G(k).X = final_x;
            G(k).Y = final_y;
            
%           get IDs of nodes within epsilon distance
            rlist = [];
            for j = 1:k-1
                temp_dist = hypot(G(j).X - final_x,G(j).Y - final_y);
                if (temp_dist < epsilon)
                    rlist = [rlist; j temp_dist];
                end
            end
            
            
%           find best parent, if any
            current_cost = G(parent_id).cost+hypot(G(parent_id).X - final_x,G(parent_id).Y - final_y);
            for j = 1:size(rlist,1)
%               Find best parent
                if (G(rlist(j,1)).cost+rlist(j,2) < current_cost)
                    parent_id = rlist(j,1);
                    current_cost = G(rlist(j,1)).cost+rlist(j,2);
                end
            end
            
%           Set parent to node
            G(k).parent = parent_id;
            G(k).cost = G(parent_id).cost+hypot(G(parent_id).X - final_x,G(parent_id).Y - final_y);
            G(parent_id).add_child(k);
            
%           Rewire neighbors
            
            for j = 1:size(rlist,1)
%               Rewire to find best parent
                if (G(k).cost+rlist(j,2) < G(rlist(j,1)).cost)
                    G(G(rlist(j,1)).parent).remove_child(rlist(j,1))
                    G(rlist(j,1)).parent = k;
                    G(rlist(j,1)).cost = G(k).cost+rlist(j,2);
                    G(k).add_child(rlist(j,1));
                end
            end
                
            break
        else
%             fprintf("collision detected\n");
        end
    end
    
% 		if node is close enough to goal, mark node id and break loop
    goal_dist = hypot(G(k).X - goal(1),G(k).Y - goal(2));
    if (goal_dist <= goal_rad)
        fprintf("Found last node, id: %d\n",k);
        finish_id = k;
        break;
    end
end
toc


%% Calc final path
final_path = [G(finish_id).X G(finish_id).Y finish_id];
id_temp = finish_id;
while id_temp ~= 1
    id_temp = G(id_temp).parent;
    final_path = [final_path; G(id_temp).X G(id_temp).Y id_temp];
end
final_path = [flipud(final_path); goal 0]


% G(final_path(2,3)).mark_branch(G);


%% Plotting stuff
% Plot all
hold on
for k = 2:max_init_interations
    plot([G(k).X G(G(k).parent).X],[G(k).Y G(G(k).parent).Y],'b')
end

for k = 2:max_init_interations
    if G(k).state == 1
        plot([G(k).X G(G(k).parent).X],[G(k).Y G(G(k).parent).Y],'m')
    end
end

% Plot path taken
plot(final_path(:,1),final_path(:,2),'g')

% Plot path taken
t = -.1:.1:2*pi;
for i = 1:size(obstacles,1)
    x = obstacles(i,1)+obstacles(i,3)*cos(t);
    y = obstacles(i,2)+obstacles(i,3)*sin(t);
    plot(x,y,'r')
end
axis equal

%% refine for each committed trajectory




max_init_interations2 = 1000;
G2 = [];
for k = 1:max_init_interations2
    G2 = [G2 Node()];
end
finish_id2 = max_init_interations2;

G2(1).parent = 0;
G2(1).Index = 1;
G2(1).X = final_path(2,1);
G2(1).Y = final_path(2,2);
G2(1).cost = G(final_path(2,3)).cost;


