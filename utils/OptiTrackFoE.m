
function [FoEXtab, FoEYtab,vxtab,vytab,vztab] = OptiTrackFoE(time,x,y,z,a,b,c,d)





FoEXtab = [];
FoEYtab = [];

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
    
    
    
    %% transform velocity vector to local frame
    
    quat = quaternion(a(i), b(i), c(i), d(i));
    
    RotationMatrix = quat2rotm(quat);
    
    XYZ = [vx vy vz] * RotationMatrix ; %in optitrack frame
    
    
    
    vxtab = [vxtab XYZ(1)]; %in body frame defined by me (x forward)
    vytab = [vytab XYZ(2)];
    vztab = [vztab XYZ(3)];
    
    
    FoEX = atan(XYZ(1)/XYZ(3))*180/pi;
    FoEY = atan(XYZ(2)/XYZ(3))*180/pi;
    
    FoEXtab = [FoEXtab FoEX];
    FoEYtab = [FoEYtab FoEY];
    
    
    
    
end


% figure(3)
%     plot(time(2:end),xtab)
%         hold on
% 
%     plot(time(2:end),ytab)
%     plot(time(2:end),ztab)
%     hold off
end