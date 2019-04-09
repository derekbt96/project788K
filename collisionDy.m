function collide = collisionDy(pnt,v,a,dt,obs)
col = 0;
vmax = max(norm(v),norm(v+a*dt));
% Tstep = 2*dt/(vmax*dt);
Tstep = .1;
for k = 1:size(obs,1)
    for l = 0:Tstep:dt
        pos = pnt+v*l+a*(.5*l^2);
        if norm(pos-obs(k,1:2)) < obs(k,3)
            col = 1;
        end
    end
end

collide = col;
end
