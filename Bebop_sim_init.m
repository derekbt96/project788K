I = [1 0 0;
    0 1 0;
    0 0 1];

mass = 2;
max_theta_X = (40)*pi/180; % radians
max_theta_Y = (40)*pi/180; % radians
max_v_Z = 1; % m/s
max_rot_Z = (90)*pi/180; % rad/s

Initial_pos = [0 0 0];
Initial_vel_b =  [0 0 0];


%%
filename = 'high_fidelity_path_5.csv';
data = flipud(csvread(filename));

data(:,1:3) = .1*data(:,1:3);

start = data(1,2:3); 
goal = data(end,2:3);

theta_path = data(:,3);
vel_path = data(:,4);
accel_path = data(:,5:6);
gamma_path = data(:,7);
% t, x, y, theta, v, w, a, gamma