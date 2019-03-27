classdef Node
    
    properties
        X 
        Y
        
        Index % id
        
        parent % id
        
        cost % computed cost
        
        children
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
        
    end
end
