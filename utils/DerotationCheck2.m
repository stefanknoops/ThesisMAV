M = csvread("../Experiments/Yawing/OF_LOGFILE_.txt");

time = M(:,1);
x= M(:,2);
y = M(:,3);
pol = M(:,4);
u = M(:,5);
v = M(:,6);
p = M(:,7);
q = M(:,8);
r = M(:,9);
du = M(:,10);
dv = M(:,11);
rot_u = M(:,12);
rot_v = M(:,13);
foe = M(:,14);

norm_x = (x-120)/120;
norm_y = (y-90)/90;

 mag_of = u.^2 + v.^2;

for i=1:100:size(time,1)
    figure(1)

    clf;
    hold on;
    
    axis equal
       xlim([0,240])
    ylim([0,180])
        set(gca, 'YDir','reverse')
  
    quiver(x(i:i+99),y(i:i+99),u(i:i+99),v(i:i+99),"Color","Black");
    quiver(x(i:i+99),y(i:i+99),rot_u(i:i+99),rot_v(i:i+99),"Color","Red")
    quiver(x(i:i+99),y(i:i+99),du(i:i+99),dv(i:i+99),"Color","Blue");
    
    disp([x(i:i+99),y(i:i+99),u(i:i+99),v(i:i+99),rot_u(i:i+99),rot_v(i:i+99),du(i:i+99),dv(i:i+99)])
    axis equal
       xlim([0,240])
    ylim([0,180])
        set(gca, 'YDir','reverse')
grid on
    legend("Unprocessed","Rotational component","Derotated");
    
    figure(2)
    
    
    bubblechart(norm_x(i:i+99),norm_y(i:i+99),mag_of(i:i+99))
    
    pause();
end