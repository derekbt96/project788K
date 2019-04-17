classdef Node < handle
    
    properties
        X 
        Y
        
        dx
        dy
        vel
        
        ax
        ay
        a
        
        px
        py
        
        Index % id
        
        parent % id
        
        cost % computed cost
        
        children % id of children nodes
        
        state %0 0:state 1:keep
    end
    
    methods    
        function obj = Node()
            obj.children = [];
        end
        
%         function obj = Node(loc,indx,parent)
%             obj.X = loc(1);
%             obj.Y = loc(2);
%             obj.Index = indx;
%             obj.parent = parent;
%         end
        
        function dist = distance(obj,loc)
            dist = ((loc(1)-obj.Location(1))^2+(loc(2)-obj.Location(2))^2+(loc(3)-obj.Location(3))^2)^.5;
        end
        
        function dist = find_travel_dist(obj,graph)
            temp_dist = 0;
            dist = temp_dist;
        end
        
        function remove_child(obj,num)
%             G(G(j).parent).children(G(G(j).parent).children) = [];
            obj.children = obj.children(obj.children~=num);
        end
        
        function add_child(obj,k)
            obj.children = [obj.children k];
        end
        
        function mark_branch(obj,G)
            obj.state = 1;
            for k = 1:length(obj.children)
                G(obj.children(k)).mark_branch(G)
            end
        end
    end
end
