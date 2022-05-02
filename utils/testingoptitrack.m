M = csvread("optitrack_10.csv",1);
close all


time = M(:,1)/(10^9);
x = M(:,2);
y = M(:,3);
z = M(:,4);
a = M(:,5);
b = M(:,6);
c = M(:,7);
d = M(:,8);


FoEXtab = [];
FoEYtab = [];

xtab = [];
ytab = [];
ztab = [];

vxtab = [];
vytab = [];
vztab = [];

for i = 2:length(time)
    %% moment in time

    dt = (time(i)-time(i-1));

    %% velocity in global x y z frame

    vx = (x(i)-x(i-1))/dt;
    vy = (y(i)-y(i-1))/dt;
    vz = (z(i)-z(i-1))/dt;
    
    % normalize
%     
%     vx = vx / sqrt(vx^2 + vy^2 + vz^2);
%     vy = vy / sqrt(vx^2 + vy^2 + vz^2);
%     vz = vz / sqrt(vx^2 + vy^2 + vz^2);

    vxtab = [vxtab vx];
    vytab = [vytab vy];
    vztab = [vztab vz];
    
    %% transform velocity vector to local frame
    
    quat = quaternion(a(i), b(i), c(i), d(i));
        
    RotationMatrix = quat2rotm(quat);
    
    XYZ = [vx vy vz] * RotationMatrix ; %in optitrack frame
    
    xtab = [xtab XYZ(3)]; %in body frame defined by me (x forward)
    ytab = [ytab XYZ(1)];
    ztab = [ztab XYZ(2)];
    
    FoEX = XYZ(1)/XYZ(3);
    FoEY = XYZ(2)/XYZ(3);
    
    FoEXtab = [FoEXtab FoEX];
    FoEYtab = [FoEYtab FoEY];
    
    
end

%% smooth

FoEXtab = smooth(FoEXtab,30);
FoEYtab = smooth(FoEYtab,30);

xtab = smooth(xtab,30);
ytab = smooth(ytab,30);
ztab = smooth(ztab,30);

figure(1)
hold on
plot(time(2:end),FoEXtab)
plot(time(2:end),FoEYtab)
legend('FoEx','FoEY');

hold off

figure(2)
hold on
plot(time(2:end),xtab)
plot(time(2:end),ytab)
plot(time(2:end),ztab)
legend('vx','vy','vz');
hold off

% figure(3)
% hold on
% plot(vxtab)
% plot(vytab)
% plot(vztab)
% legend('x','y','z');
% hold off