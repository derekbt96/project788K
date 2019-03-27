% Anytime RRT*

clc
clear all

tic

start = [0 0];
goal = [10 10];
speed_bound = 1.5;

epsilon = .41;
goal_rad = epsilon;
greed = .5; 

mu = epsilon;

max_init_interations = 1000;
G(1:max_init_interations) = Node();
finish_id = max_init_interations;

G(1).parent = 0;
G(1).Index = 1;
G(1).X = start(1);
G(1).Y = start(2);

obstacles = [5 5 2;
            7 2 1;
            4 9 1.5];

%% find path to goal
for k = 2:max_init_interations
    
% 		compute points until valid point is found
    while 1

% 			get random point in bounds
        greedtest = rand();
        if (greedtest > greed)
            rand_x = rand() * 10.0;
            rand_y = rand() * 10.0;
            rand_v = rand() * speed_bound;
        else
            rand_x = goal(1);
            rand_y = goal(2);
        end

% 			find closest node in array to random point
        parent_id = 0;
        closest_dist = 1000.0;
        
        for j = 1:k
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
            temp_parent_x = G(parent_id).X;
            temp_parent_y = G(parent_id).Y;
            final_x = temp_parent_x + (epsilon / closest_dist) * (rand_x - temp_parent_x);
            final_y = temp_parent_y + (epsilon / closest_dist) * (rand_y - temp_parent_y);
        end

% 			if new node is not colliding with obstacles add it to array and break while loop to continue to next node
        parentloc = [G(parent_id).X G(parent_id).Y];
        final_location = [final_x final_y];
        if (~collision(parentloc, final_location,obstacles))
			fprintf("Iteration: %d added at: %f %f \n", k, final_x, final_y);
            G(k).Index = k;
            G(k).parent = parent_id;
            G(k).X = final_x;
            G(k).Y = final_y;
            G(k).cost = G(parent_id).cost+hypot(G(parent_id).X - final_x,G(parent_id).Y - final_y);
            
            
% %             rewire nodes around it
%             for j = 1:k
%                 temp_dist = hypot(G(j).X - final_x,G(j).Y - final_y);
%                 if (temp_dist < closest_dist)
%                     parent_id = j;
%                     closest_dist = temp_dist;
%                 end
%             end
            
            
            break
        else
            fprintf("collision detected\n");
        end
    end
% 		if node is close enough to goal, mark node id and break loop
    goal_dist = hypot(G(k).X - goal(1),G(k).Y - goal(2));
    if (goal_dist <= goal_rad)
        fprintf("Found last node, id: %d\n",k);
        finish_id = k;
        break
    end
end
toc

hold on
for k = 2:finish_id
    plot([G(k).X G(G(k).parent).X],[G(k).Y G(G(k).parent).Y],'b')
end

while finish_id ~= 1
    plot([G(finish_id).X G(G(finish_id).parent).X],[G(finish_id).Y G(G(finish_id).parent).Y],'g')
    finish_id = G(finish_id).parent;
end

t = -.1:.1:2*pi;
for i = 1:size(obstacles,1)
    x = obstacles(i,1)+obstacles(i,3)*cos(t);
    y = obstacles(i,2)+obstacles(i,3)*sin(t);
    plot(x,y,'r')
end
axis equal

%% refine for each committed trajectory



