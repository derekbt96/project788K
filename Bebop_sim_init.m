
clear all
clc

I = [1 0 0;
    0 1 0;
    0 0 1];

mass = 1.5;
max_theta_X = (40)*pi/180; % radians
max_theta_Y = (40)*pi/180; % radians
max_v_Z = 1; % m/s
max_rot_Z = (90)*pi/180; % rad/s
goal_alt = 2;

positional_pid = [.07 0 .12];

filename = 'testDyXY.csv';
data = csvread(filename);

Initial_pos = [data(1,2:3) goal_alt];
Initial_vel_b =  [data(1,4:5) 0];

% data(:,1:3) = .1*data(:,1:3);

% Path = data;
start = data(1,2:3); 
goal = data(end,2:3);
tfinal = data(end,1)+2;
% times = data(:,1);
% Path = data(:,2:3);
% vel_path = data(:,4:5);
% accel_path = data(:,6:7);
% t, x, y, dx, dy, ax ,ay

dt = data(2,1);

times = [];
pos = [];
vel = [];
accel = [];
for k = 1:(size(data,1)-1)
    for t = 0:dt/10:dt-dt/10
        times = [times; data(k,1)+t];
        pos = [pos; data(k,2:3)+data(k,4:5)*t+.5*t^2*data(k+1,6:7)];
        vel = [vel; data(k,4:5)+t*data(k+1,6:7)];
        accel = [accel; data(k,6:7)];
    end 
end
for k = 1:2
    times = [times; times(end)+dt/10];
    pos = [pos; goal];
    vel = [vel; data(end,4:5)];
    accel = [accel; 0 0];
end
%% Plotting

hold on
plot(Vehicle_pos.Data(:,1),Vehicle_pos.Data(:,2),'b')
plot(Desired_pos.Data(:,1),Desired_pos.Data(:,2),'r--')
