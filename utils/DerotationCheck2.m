M = csvread("../Experiments/test_rot/OF_LOGFILE.txt");

time = M(:,1);
x= M(:,2);
y = M(:,3);
pol = M(:,4);
unpro_u = M(:,5);
unpro_v = M(:,6);
p = M(:,7);
q = M(:,8);
r = M(:,9);
derot_u = M(:,10);
derot_v = M(:,11);
rot_u = M(:,12);
rot_v = M(:,13);
foe = M(:,14);

norm_x = (x-120)/120;
norm_y = (y-90)/90;

mag_of = sqrt((unpro_u).^2 + (unpro_v).^2);
mag_rot = sqrt((rot_u).^2 + (rot_v).^2);
mag_derot = (sqrt(((derot_u)).^2 + ((derot_v)).^2));

figure(2)

subplot(2,1,1);
scatter(norm_x,[mag_rot,mag_of]);
legend('Magnitude rotation','Magnitude optic flow')
xlabel('Normalized x coordinate')
ylabel('Vector magnitude')
subplot(2,1,2);
scatter(norm_y,[mag_rot,mag_of]);
legend('Magnitude rotation','Magnitude optic flow')
xlabel('Normalized y coordinate')
ylabel('Vector magnitude')

disp("$"+norm(mean([unpro_u,unpro_v]))+"$ & $"+norm(std([unpro_u,unpro_v]))+"$ & $"+norm(mean([derot_u,derot_v]))+"$ & $"+norm(std([derot_u,derot_v]))+"$")

    
for i=1:100:size(time,1)
    figure(1)

    clf;
    hold on;
    
    axis equal
       xlim([0,240])
    ylim([0,180])
        set(gca, 'YDir','reverse')
  
    scaling = 40;
    quiver(x(i:i+99),y(i:i+99),scaling*unpro_u(i:i+99),scaling*unpro_v(i:i+99),"Color","Black","AutoScale","off");
    quiver(x(i:i+99),y(i:i+99),scaling*rot_u(i:i+99),scaling*rot_v(i:i+99),"Color","Red","AutoScale","off")

    quiver(x(i:i+99),y(i:i+99),scaling*derot_u(i:i+99),scaling*derot_v(i:i+99),"Color","Blue","AutoScale","off");
    
    disp([x(i:i+99),y(i:i+99),unpro_u(i:i+99),unpro_v(i:i+99),rot_u(i:i+99),rot_v(i:i+99),derot_u(i:i+99),derot_v(i:i+99)])
    axis equal
       xlim([0,240])
    ylim([0,180])
        set(gca, 'YDir','reverse')
grid on
    legend("Unprocessed","Rotational component","Derotated");
    

    pause();
end