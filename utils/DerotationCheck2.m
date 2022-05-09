M = csvread("../Experiments/SlowRollCarpet/OF_LOGFILE_.txt");

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
figure(1)

for i=1:100:size(time,1)
    clf;
    hold on;
    
    axis equal
       xlim([0,240])
    ylim([0,180])
        set(gca, 'YDir','reverse')
  
    quiver(x(i:i+99),y(i:i+99),u(i:i+99),v(i:i+99),1,"Color","Black");
    quiver(x(i:i+99),y(i:i+99),rot_u(i:i+99),rot_v(i:i+99),1,"Color","Red")
    quiver(x(i:i+99),y(i:i+99),du(i:i+99),dv(i:i+99),10,"Color","Blue");
 
    axis equal
       xlim([0,240])
    ylim([0,180])
        set(gca, 'YDir','reverse')
grid on
    legend("Unprocessed","Rotational component","Derotated");

    pause();
end