%rng(0) %seeded

frequency = 500;
dt = 10^9/frequency; %in ns
time = 5;
n_timesteps = time*10^9/dt;
z_start = 6;
z_end = 1;

FOV_X = 61.7164;
FOV_Y = 48.2168;


filename = 'Test';




traj = zeros(n_timesteps,8);

t = linspace(0,time*10^9,n_timesteps);
t_sim = t/10^9;

traj(:,1) = t;

n_components = 100;
max_freq = 2;
max_amp = 0.05;


frequencies = max_freq * rand(1,n_components);
amplitudes = 2*max_amp * rand(1,n_components) - max_amp;
phase = pi/2 * rand(1,n_components);

disp(frequencies)
disp(amplitudes)

x = sum(amplitudes.*sin(frequencies.*(traj(:,1)./10^9)+phase*pi/2),2);

frequencies = max_freq * rand(1,n_components);
amplitudes = 2*max_amp * rand(1,n_components) - max_amp;
phase = pi/2 * rand(1,n_components);

y = sum(amplitudes.*sin(frequencies.*(traj(:,1)./10^9)+phase*pi/2),2);


z = linspace(z_start,z_end,n_timesteps).';

vx_world = diff(x);
vy_world = diff(y);
vz_world = diff(z);


traj(:,2:4) = [x,y,z];


n_components = 100;
max_freq = 5;
max_amp = 1 *pi /180;

frequencies = max_freq * rand(1,n_components);
amplitudes = 2*max_amp * rand(1,n_components) - max_amp;
phase = pi/2 * rand(1,n_components);
yaw = sum(amplitudes.*sin(frequencies.*(traj(:,1)./10^9)+phase),2);

frequencies = max_freq * rand(1,n_components);
amplitudes = 2*max_amp * rand(1,n_components) - max_amp;
phase = pi/2 * rand(1,n_components);
roll = sum(amplitudes.*sin(frequencies.*(traj(:,1)./10^9)+phase),2);

frequencies = max_freq * rand(1,n_components);
amplitudes = 2*max_amp * rand(1,n_components) - max_amp;
phase = pi/2 * rand(1,n_components);
pitch = sum(amplitudes.*sin(frequencies.*(traj(:,1)./10^9)+phase),2);

%%%EXAMPLES
% sin(traj(:,1)/10^9)
% linspace(z_start,z_end,n_timesteps).'
% sin(traj(:,1)/10^9)+cos(2*traj(:,1)/10^9)/3

% FUNCTIONS FOR RANDOM TRAJECTORIES. 
%b = 0.01;
%x = cumsum(smooth( -b + (b+b)*rand(1,n_timesteps),500));
%y = cumsum(smooth( -b + (b+b)*rand(1,n_timesteps),500));



quat = eul2quat([yaw,roll,pitch]);

traj(:,5:8) = [quat(:,2),quat(:,3),quat(:,4),quat(:,1)];

delay = 1; %s

n_delay_steps = delay*10^9/dt;

delay_line = traj(1,:);

for i = 1:n_delay_steps
    traj = [delay_line;traj];
end

traj(:,1) = linspace(0,(time+delay)*10^9,size(traj,1));

%traj = round(traj,3); %round to avoid engineering notation, splines still expected to look the same

mkdir('../Experiments/'+string(filename));
T = array2table(traj);

varnames = {"# timestamp"," x"," y"," z"," qx"," qy"," qz"," qw"};
T.Properties.VariableNames(1:8) = ["# timestamp"," x"," y"," z"," qx"," qy"," qz"," qw"];
%dlmwrite(filename,traj,'precision','%.5f');
filepath = append('../Experiments/',filename,'/trajectory.csv');

fname=filepath;
writetable(cell2table(varnames),fname,'writevariablenames',0)

dlmwrite(filepath,traj,'-append','precision','%.9f','-append')
%writetable(T,filepath);

[SimulatedFoE_X, SimulatedFoE_Y,vx,vy,vz] = OptiTrackFoE(t_sim,x,y,z,quat(:,1),quat(:,2),quat(:,3),quat(:,4));
clear T


T = array2table([t_sim(2:end).', SimulatedFoE_X.', SimulatedFoE_Y.']);
T.Properties.VariableNames(1:3) = ["t"," FoE_x"," FoE_y"];

filepath = append('../Experiments/',filename,'/Expected_FoE.csv');
writetable(T,filepath);



figure(1)
subplot(2,2,1);
plot(t,[x,y,z]);
legend('x','y','z');
set(gca,'FontName','Arial','FontSize',12);
ylabel('position [m]') 
xlabel('time [ns]') 


subplot(2,2,2);
plot(t,[pitch,roll,yaw]);
legend('pitch','roll','yaw')
set(gca,'FontName','Arial','FontSize',12);
xlabel('time [ns]') 
ylabel('rotation in body frame [rad/s]') 

subplot(2,2,3);
plot(t(2:end),[vx.',vy.',vz.']);
legend('vx','vy','vz')
set(gca,'FontName','Arial','FontSize',12);
ylabel('velocity in body frame [m/s]') 
xlabel('time [ns]') 
%set(gcf,'Position',[100 100 700 800])

subplot(2,2,4);
plot(t(2:end),[vx_world,vy_world,vz_world]);
legend('vx world','vy world','vz world')
set(gca,'FontName','Arial','FontSize',12);
ylabel('velocity in world frame [m/s]') 
xlabel('time [ns]') 
%set(gcf,'Position',[100 100 700 800])

filepath = append('../Experiments/',filename,'/TrajectoryPlot.png');
saveas(gcf,filepath)

figure(2);
subplot(2,1,1)
plot(t(2:end),SimulatedFoE_X)
title('Simulated FoE_x')
yline([-FOV_X/2,FOV_X/2],'--')
yline([0],':')
set(gca,'FontName','Arial','FontSize',12);


subplot(2,1,2)
plot(t(2:end),SimulatedFoE_Y)
title('Simulated FoE_y')
yline([-FOV_Y/2,FOV_Y/2],'--')
yline([0],':')
set(gca,'FontName','Arial','FontSize',12);


%set(gcf,'Position',[800 100 700 800])

filepath = append('../Experiments/',filename,'/PlotFoE.png');
saveas(gcf,filepath)
hold off

figure(3);
plot3(x,y,z)
title('3D trajectory')
set(gca,'FontName','Arial','FontSize',12);




