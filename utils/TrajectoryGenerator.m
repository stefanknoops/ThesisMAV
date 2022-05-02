frequency = 200;
dt = 10^9/frequency; %in ns
time = 10;
n_timesteps = time*10^9/dt;
z_start = 3;
z_end = 1;


filename = 'FastSmallRollDown.csv';




traj = zeros(n_timesteps,8);

traj(:,1) = linspace(0,time*10^9,n_timesteps);


x = zeros(n_timesteps,1);
y = zeros(n_timesteps,1);
z = linspace(z_start,z_end,n_timesteps).';

traj(:,2:4) = [x,y,z];

%in degrees
yaw = zeros(n_timesteps,1);
roll = sin(2*traj(:,1)/10^9)/8;
pitch = zeros(n_timesteps,1);


quat = eul2quat([yaw,roll,pitch]);

traj(:,5:8) = [quat(:,2),quat(:,3),quat(:,4),quat(:,1)];

delay = 2; %s

n_delay_steps = delay*10^9/dt;

delay_line = traj(1,:);

for i = 1:n_delay_steps
    traj = [delay_line;traj];
end

traj(:,1) = linspace(0,(time+delay)*10^9,size(traj,1));

traj = round(traj,3); %round to avoid engineering notation, splines still expected to look the same


T = array2table(traj);


T.Properties.VariableNames(1:8) = ["# timestamp"," x"," y"," z"," qx"," qy"," qz"," qw"];
%dlmwrite(filename,traj,'precision','%.5f');
writetable(T,filename);

figure(1)
plot(traj(:,1),traj(:,2:4));

figure(2)
plot(traj(:,1),traj(:,5:8));

%%%tools
% sin(traj(:,1)/10^9)
% linspace(z_start,z_end,n_timesteps).'

