SimData = csvread("SpiralDownSmall.csv",1);
%OptiTrackData = OptiTrackData(:,1:8);
%OptiTrackData = OptiTrackData(:,2:9); %for when using export from pandas which has index column added

EstimatedFoE = csvread("FoE_recording_2022_04_28-11_50_47.txt",1);

close all

CPPFoEXtab = EstimatedFoE(:,2);
CPPFoEYtab = EstimatedFoE(:,3);
CPPtime = (EstimatedFoE(:,1)-EstimatedFoE(1,1))/10^9;


CPPtime = [-2;CPPtime];
CPPFoEXtab = [120;CPPFoEXtab];
CPPFoEYtab = [90;CPPFoEYtab];

t = SimData(:,2);
x = SimData(:,3);
y = SimData(:,4);
z = SimData(:,5);
W = SimData(:,6);
X = SimData(:,7);
Y = SimData(:,8);
Z = SimData(:,9);

figure(1);
plot(t,W)
hold on
plot(t,X)
plot(t,Y)
plot(t,Z)
legend('W','X','Y','Z')
hold off

figure(2);
plot(t,x)
hold on
plot(t,y)
plot(t,z)
legend('x','y','z')
hold off

[SimulatedFoE_X, SimulatedFoE_Y] = OptiTrackFoE(t,x,y,z,X,Y,Z,W);

figure(3);
subplot(2,2,1)
plot(t(2:end),SimulatedFoE_X)
title('Simulated FoE_x')
subplot(2,2,2)
plot(t(2:end),SimulatedFoE_Y)
title('Simulated FoE_y')

subplot(2,2,3)
plot(CPPtime,CPPFoEXtab);
hold on
title('Predicted FoE_x')

subplot(2,2,4)
plot(CPPtime,CPPFoEYtab);
title('Predicted FoE_y')

hold off

