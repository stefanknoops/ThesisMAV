%rng(0) %seeded

clear variables;


frequency = 1000;
dt = 10^9/frequency; %in ns
time = 10;
n_timesteps = time*10^9/dt;
z_start =0;
z_end = -20;

FOV_X = 61.7164;
FOV_Y = 48.2168;

filename = '6Dflight_1';

traj = zeros(n_timesteps,8);

t = linspace(0,time*10^9,n_timesteps);
t_sim = t/10^9;

traj(:,1) = t;


ttime = t_sim.';

%% TRAJECTORY INPUTS
y = linspace(z_start,z_end,n_timesteps).';

valid = false;
n=0;
while ~valid
    

    l = 1;

    x = RandomNumber(1)*sin(0.5*ttime)    +RandomNumber(0.5)*sin(1*ttime)   +RandomNumber(0.2)*sin(2*ttime)   +RandomNumber(0.1)*sin(3*ttime);
    z = RandomNumber(1)*sin(0.5*ttime)    +RandomNumber(0.5)*sin(1*ttime)   +RandomNumber(0.2)*sin(2*ttime)   +RandomNumber(0.1)*sin(3*ttime);

    roll =  randi([-30,30])*sin(ttime) + randi([-30,30])*sin(2*ttime); %0.8*sin(ttime); %sin(3*ttime)./10.*(sin(0.5*ttime)+1*cos(2*ttime)-0.3*sin(ttime));
    yaw =   randi([-30,30])*sin(ttime) + randi([-30,30])*sin(2*ttime);%zeros(n_timesteps,1); %pi/2*sin(2*ttime); %
    pitch = randi([-30,30])*sin(ttime) + randi([-30,30])*sin(2*ttime);%zeros(n_timesteps,1); %ones(n_timesteps,1)*15*pi/180; %zeros(n_timesteps,1); %cos(3*ttime)./10.*(sin(ttime)+0.5*cos(0.5*ttime)+0.05*sin(2*ttime));


    z = z+ 2.5*ones(n_timesteps,1);
    %% 

    roll = roll.*pi/180;
    pitch = pitch.*pi/180;
    yaw = yaw.*pi/180;

    quat = eul2quat([yaw,roll,pitch]);


    [SimulatedFoE_X, SimulatedFoE_Y,vx,vz,vy] = ForwardOptiTrackFoE(t_sim,x,y,z,quat(:,1),quat(:,2),quat(:,3),quat(:,4));

    if any(abs(SimulatedFoE_X) > FOV_X/2) || any(abs(SimulatedFoE_Y) > FOV_Y/2)
        n = n+1;
    else
        valid = true;
        disp("took "+string(n)+" tries")
    end
end

vx_world = diff(x)/dt*10^9;
vy_world = diff(y)/dt*10^9;
vz_world = diff(z)/dt*10^9;


traj(:,2:4) = [x,y,z];


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

clear T

T = array2table([t_sim(2:end).', SimulatedFoE_X.', SimulatedFoE_Y.']);
T.Properties.VariableNames(1:3) = ["t"," FoE_x"," FoE_y"];

filepath = append('../Experiments/',filename,'/Expected_FoE.csv');
writetable(T,filepath);



figure(1)



subplot(3,1,1);
plot(t,[pitch,roll,yaw]);
legend('\theta','\phi','\psi','Location','eastoutside')
set(gca,'FontName','Arial','FontSize',12);
xlabel('time [ns]') 
ylabel('attitude [rad]') 
grid on

subplot(3,1,2);
plot(t(2:end),[vx.',vy.',vz.']);
legend('v_x','v_y','v_z','Location','eastoutside')
set(gca,'FontName','Arial','FontSize',12);
ylabel('v_{body} [m/s]') 
xlabel('time [ns]') 
%set(gcf,'Position',[100 100 700 800])
grid on

subplot(3,1,3);
plot(t(2:end),[vx_world,vy_world,vz_world]);
legend('v_x','v_y','v_z','Location','eastoutside')
set(gca,'FontName','Arial','FontSize',12);
ylabel('v_{world} [m/s]') 
xlabel('time [ns]') 
grid on
set(gcf,'Position',[100 100 560 800])

filepath = append('../Experiments/',filename,'/TrajectoryPlot.pdf');
exportgraphics(gcf,filepath,'ContentType','vector')

figure(2);
subplot(2,1,1)
plot(t(2:end),SimulatedFoE_X)
title('Simulated FoE_x')
yline([-FOV_X/2,FOV_X/2],'--')
yline([0],':')
ylim(1.05*[-FOV_X/2,FOV_X/2])
set(gca,'FontName','Arial','FontSize',12);
xlabel('time [ns]') 
ylabel("FoE angle [deg]")


subplot(2,1,2)
plot(t(2:end),SimulatedFoE_Y)
title('Simulated FoE_y')
yline([-FOV_Y/2,FOV_Y/2],'--')
yline([0],':')
ylim(1.05*[-FOV_Y/2,FOV_Y/2])
set(gca,'FontName','Arial','FontSize',12);
xlabel('time [ns]') 
ylabel("FoE angle [deg]")

%set(gcf,'Position',[800 100 700 800])

filepath = append('../Experiments/',filename,'/PlotFoE.pdf');
exportgraphics(gcf,filepath,'ContentType','vector')
hold off

figure(3);

plot3(x,y,z)

set(gca,'FontName','Arial','FontSize',12);
xlabel("x [m]");
ylabel("y [m]");
zlabel("z [m]");

filepath = append('../Experiments/',filename,'/TrajectoryProjection.pdf');
grid on

exportgraphics(gcf,filepath,'ContentType','vector')


filepath = append('../Experiments/',filename,'/workspace.mat');
save(filepath);

