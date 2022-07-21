ExperimentName = 'Shaking2D_translation_3';
close all
filepath1 = append('../Experiments/',ExperimentName,'/trajectory.csv');
filepath2 = append('../Experiments/',ExperimentName,'/FoE_recording.txt');
filepath3 = append('../Experiments/',ExperimentName,'/Expected_FoE.csv');
filepath4 = append('../Experiments/',ExperimentName,'/SimulatedPose.csv');
filepath5 = append('../Experiments/',ExperimentName,'/OF_LOGFILE_.txt');


OF = csvread(filepath5,1);

p = OF(:,7);
q = OF(:,8);
r = OF(:,9);
totalrotation = sqrt(p.^2+q.^2+r.^2);

traj = csvread(filepath1,1);
traj(:,1) = (traj(:,1)-traj(1,1))/10^9;

EstimatedFoE = csvread(filepath2,1);
EstimatedFoE(:,1) = (EstimatedFoE(:,1)-EstimatedFoE(1,1))/10^6;

TrueFoE = csvread(filepath3,1);

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

EstimatedFoE(:,2) =    atan(      (120-   EstimatedFoE(:,2))/120 * tan(FOV_X/2)   )   *180/pi;
EstimatedFoE(:,3) =    atan(      (90-    EstimatedFoE(:,3))/90  * tan(FOV_Y/2)   )   *180/pi;

EstimatedFoE(:,4) =    atan(      (120-   EstimatedFoE(:,4))/120 * tan(FOV_X/2)   )   *180/pi;
EstimatedFoE(:,5) =    atan(      (90-    EstimatedFoE(:,5))/90  * tan(FOV_Y/2)   )   *180/pi;

%EstimatedFoE(:,2) =    EstimatedFoE(:,2) - 120;
%EstimatedFoE(:,3) = -  EstimatedFoE(:,3) + 90;


dif = abs(EstimatedFoE(:,1).' - TrueFoE(:,1));
minn = min((dif));

indexlookup = mod(find(dif == minn),size(TrueFoE,1));
indexlookup(indexlookup == 0) = size(TrueFoE,1);


err_x = EstimatedFoE(:,2) - TrueFoE(indexlookup,2);
err_y = EstimatedFoE(:,3) - TrueFoE(indexlookup,3);



figure(1);
subplot(3,1,1)
hold off

plot(EstimatedFoE(:,1),EstimatedFoE(:,2))
hold on
scatter(EstimatedFoE(:,1),EstimatedFoE(:,4),"Marker","+","SizeData",5)
plot(TrueFoE(:,1),TrueFoE(:,2));
ylim([-FOV_X*1.05/2,FOV_X*1.05/2]*180/pi)
yline([-FOV_X/2,FOV_X/2]*180/pi,'--')
legend({'Estimated FoE_x','Current prediction','True FoE_x','Lower bound','Upper bound'},'Location','eastoutside')
ylabel('FoE position [deg]') 
xlabel('time [s]') 

subplot(3,1,2)
hold off
plot(EstimatedFoE(:,1),EstimatedFoE(:,3))
hold on
scatter(EstimatedFoE(:,1),EstimatedFoE(:,5),"Marker","+","SizeData",5)
plot(TrueFoE(:,1),TrueFoE(:,3));
ylim([-FOV_Y*1.05/2,FOV_Y*1.05/2]*180/pi)
yline([-FOV_Y/2,FOV_Y/2]*180/pi,'--')
legend({'Estimated FoE_y','Current prediction','True FoE_y','Lower bound','Upper bound'},'Location','eastoutside')
ylabel('FoE position [deg]') 
xlabel('time [s]') 


subplot(3,1,3)
plot(EstimatedFoE(:,1),EstimatedFoE(:,7))
xlabel('time [s]')
ylabel('#vectors');

legend({'Amount of used vectors'},'Location','eastoutside')


set(gcf,'Position',[100 100 560 800])

filepath = append('../Experiments/',ExperimentName,'/FoE_validation.png');
saveas(gcf,filepath)

figure(2)
subplot(2,1,1)
plot(SimulatedTraj(:,2),[sim_x,sim_y,sim_z])
hold on
plot(traj(:,1),[traj(:,2),traj(:,3),traj(:,4)],'--')
legend('x_{sim}','y_{sim}','z_{sim}','x_{planned}','y_{planned}','z_{planned}')
hold off

subplot(2,1,2)
plot(SimulatedTraj(:,2),[sim_roll,sim_pitch,sim_yaw])
hold on
plot(traj(:,1),quat2eul([traj(:,5),traj(:,6),traj(:,7),traj(:,8)]),'--')

legend('\phi_{sim}','\theta_{sim}','\psi_{sim}','\phi_{planned}','\theta_{planned}','\psi_{planned}')

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

disp("Mean absolute error x: "+string(mean(abs(err_x)))+". Standard deviation error: "+string(std(err_x)))
disp("Mean absolute error y: "+string(mean(abs(err_y)))+". Standard deviation error: "+string(std(err_y)))
disp("Median amount of flow vectors: "+string(median(EstimatedFoE(:,7))))
disp("Average frequency: "+ string(size(EstimatedFoE(:,1),1)/max(EstimatedFoE(:,1))))

figure(4)
smoothfreq = smooth(EstimatedFoE(2:end,1),1./diff(EstimatedFoE(:,1)),30);
plot(EstimatedFoE(2:end,1),smoothfreq)
hold on
scatter(EstimatedFoE(2:end,1),1./diff(EstimatedFoE(:,1)),'Marker','+','SizeData',5)
hold off


figure(5)
subplot(2,1,1)
plot(OF(:,1),totalrotation);
subplot(2,1,2)
plot(OF(:,1),[p,q,r])
legend("p","q","r")

% err_x = TrueFoE(1,2) - EstimatedFoE(:,2);
% err_y = TrueFoE(1,3) - EstimatedFoE(:,3);
% 
% figure(3)
% subplot(2,1,1)
% histogram(err_x,100);
% subplot(2,1,2);
% histogram(err_y,100);



