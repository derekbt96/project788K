p1 = [0 0];
v1 = [1 1];
p2 = [rand()*4 rand()*4];
v2 = [rand()*2-1 rand()*2-1];
dt = 1;
a2 = [(p1(1)-p2(1)-2*v1(1)*dt-1.5*dt*(v2(1)-v1(1)))/(2*dt^2);
    (p1(2)-p2(2)-2*v1(2)*dt-1.5*dt*(v2(2)-v1(2)))/(2*dt^2)];
vc = v2-a2*dt;
a1 = (vc-v1)/dt;
xc = p1+v1*dt+.5*dt^2*a1;

t = 0:.01:dt;
lin1 = [p1(1)+v1(1)*t+.5*t.^2*a1(1);
    p1(2)+v1(2)*t+.5*t.^2*a1(2)];
lin2 = [xc(1)+vc(1)*t+.5*t.^2*a2(1);
    xc(2)+vc(2)*t+.5*t.^2*a2(2)];
hold on
plot(lin1(1,:),lin1(2,:),'r')
plot(lin2(1,:),lin2(2,:),'b')
scatter([p1(1) p2(1) xc(1)],[p1(2) p2(2) xc(2)])

%%
syms x1 x2 v1 v2 a1 a2 xc vc t
a1 = (vc-v1)/t;
