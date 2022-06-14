ExperimentName = 'StraightDown';
filepath1 = append('../Experiments/',ExperimentName,'/trajectory.csv');
filepath2 = append('../Experiments/',ExperimentName,'/FoE_recording.txt');
filepath3 = append('../Experiments/',ExperimentName,'/Expected_FoE.csv');


traj = csvread(filepath1,1);
traj(:,1) = (traj(:,1)-traj(1,1))/10^9;

EstimatedFoE = csvread(filepath2,1);
EstimatedFoE(:,1) = (EstimatedFoE(:,1)-EstimatedFoE(1,1))/10^6;

TrueFoE = csvread(filepath3,1);

filepath4 = append('../Experiments/',ExperimentName,'/SimulatedPose.csv');
SimulatedTraj = csvread(filepath4,1);

sim_x = SimulatedTraj(:,3);
sim_y = SimulatedTraj(:,4);
sim_z = SimulatedTraj(:,5);

a = SimulatedTraj(:,6);
b = SimulatedTraj(:,7);
c = SimulatedTraj(:,8);
d = SimulatedTraj(:,9);
angles = quat2eul([a,b,c,d]);

sim_yaw = angles(:,1);
sim_roll = angles(:,2);
sim_pitch = angles(:,3);

FOV_X = 61.7164*pi/180;
FOV_Y = 48.2168*pi/180;

EstimatedFoE(:,2) =     atan(      (120-   EstimatedFoE(:,2))/120 * tan(FOV_X/2)   )   *180/pi;
EstimatedFoE(:,3) = -   atan(      (90-    EstimatedFoE(:,3))/90  * tan(FOV_Y/2)   )   *180/pi;

%EstimatedFoE(:,2) = EstimatedFoE(:,2) - 120;
%EstimatedFoE(:,3) = -EstimatedFoE(:,3) + 90;


diff = abs(EstimatedFoE(:,1).' - TrueFoE(:,1));
minn = min((diff));

indexlookup = mod(find(diff == minn),size(TrueFoE,1));
indexlookup(indexlookup == 0) = size(TrueFoE,1);


err_x = EstimatedFoE(:,2) - TrueFoE(indexlookup,2);
err_y = EstimatedFoE(:,3) - TrueFoE(indexlookup,3);



figure(1);
subplot(2,1,1)
hold off

plot(EstimatedFoE(:,1),EstimatedFoE(:,2))
hold on
plot(TrueFoE(:,1),TrueFoE(:,2));
ylim([-FOV_X*1.05,FOV_X*1.05]*180/pi)
yline([-FOV_X/2,FOV_X/2]*180/pi,'--')
legend('Estimated FoE_x','True FoE_x','Lower bound','Upper bound')
ylabel('FoE position [deg]') 
xlabel('time [s]') 

subplot(2,1,2)
hold off
plot(EstimatedFoE(:,1),EstimatedFoE(:,3))
hold on
plot(TrueFoE(:,1),TrueFoE(:,3));
ylim([-FOV_Y*1.05,FOV_Y*1.05]*180/pi)
yline([-FOV_Y/2,FOV_Y/2]*180/pi,'--')
legend('Estimated FoE_y','True FoE_y','Lower bound','Upper bound')
ylabel('FoE position [deg]') 
xlabel('time [s]') 


set(gcf,'Position',[100 100 700 800])

filepath = append('../Experiments/',ExperimentName,'/FoE_validation.png');
saveas(gcf,filepath)

figure(2)
subplot(2,1,1)
plot(SimulatedTraj(:,2),[sim_x,sim_y,sim_z])
hold on
plot(traj(:,1),[traj(:,2),traj(:,3),traj(:,4)],'--')
legend('sim_x','sim_y','sim_z','planned_x','planned_y','planned_z')
hold off

subplot(2,1,2)
plot(SimulatedTraj(:,2),[sim_roll,sim_pitch,sim_yaw])
hold on
plot(traj(:,1),quat2eul([traj(:,5),traj(:,6),traj(:,7),traj(:,8)]),'--')

legend('sim_roll','sim_pitch','sim_yaw','planned_roll','planned_pitch','planned_yaw')

hold off 

figure(3)
subplot(2,2,1)
plot(EstimatedFoE(:,1),err_x)
yline(mean(err_x),'--')
subplot(2,2,3)
plot(EstimatedFoE(:,1),err_y)
yline(mean(err_y),'--')

subplot(2,2,2)
histogram(err_x,30);
subplot(2,2,4);
histogram(err_y,30);

disp("Mean absolute error x: "+string(mean(abs(err_x))))
disp("Mean absolute error y: "+string(mean(abs(err_y))))
disp("Median amount of flow vectors: "+string(median(EstimatedFoE(:,5))))


% err_x = TrueFoE(1,2) - EstimatedFoE(:,2);
% err_y = TrueFoE(1,3) - EstimatedFoE(:,3);
% 
% figure(3)
% subplot(2,1,1)
% histogram(err_x,100);
% subplot(2,1,2);
% histogram(err_y,100);



