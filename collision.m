function collide = collision(ln1,ln2,obs)
col = 0;

for k = 1:size(obs,1)
    v2 = ln2(1:2)-ln1(1:2);
    t = dot(v2,obs(k,1:2)-ln1(1:2))/(norm(v2)^2);
    
    cpl = ln1+t*v2;
    if hypot(cpl(1)-obs(k,1),cpl(2)-obs(k,2)) < obs(k,3)
        col = 1;
    end
end

collide = col;
end
