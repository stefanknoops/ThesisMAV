M = csvread("../Experiments/Yawing/OF_LOGFILE_.txt");


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
ratetime = M(:,14);



ang_OF = zeros(size(time)); %hoek die OF maakt
ang_OF_rot = zeros(size(time)); %hoek van expected rotation
mag_OF = zeros(size(time)); % magnitude OF vector
mag_OF_rot = zeros(size(time)); % magnitude rot vector
pos_ang_OF_rot = zeros(size(time)); % positive angle OF rot
pos_ang_OF = zeros(size(time)); %positive angle OF 
OF_proj = zeros(size(time)); %projected rotation on OF vector
OF_der_pre = zeros(size(time)); %OF vector pre min zero
comp =  zeros(size(time)); % angle between
u_der =  zeros(size(time));
v_der =  zeros(size(time));
OF_der =  zeros(size(time));
x_nor =  zeros(size(time)); % angle between
y_nor =  zeros(size(time));
rotational_u =  zeros(size(time));
rotational_v =  zeros(size(time));



my_mag_of = sqrt((unpro_u).^2 + (unpro_v).^2);
my_mag_rot = sqrt((rot_u).^2 + (rot_v).^2);
my_mag_derot = sqrt((derot_u).^2 + (derot_v).^2);

for i = 1:size(time)
    x_nor(i) = 1.2*((x(i) / 120) - 1);
    y_nor(i) = 1.2*((y(i) / 90) - 1); 

    rotational_u(i) = -(-q(i) + r(i) * y_nor(i) + p(i) * x_nor(i) * y_nor(i) - q(i) * x_nor(i) * x_nor(i));
    rotational_v(i) = -( p(i) - r(i) * x_nor(i) - q(i) * x_nor(i) * y_nor(i) + p(i) * y_nor(i) * y_nor(i));
    ang_OF(i) = atan2(unpro_v(i), unpro_u(i));
    ang_OF_rot(i) = atan2(rotational_v(i), rotational_u(i));
    mag_OF(i) = hypot(unpro_u(i), unpro_v(i));
    mag_OF_rot(i) = hypot(rotational_u(i), rotational_v(i));
    if (ang_OF_rot(i) < 0)
        pos_ang_OF_rot(i) = abs(2 * pi - abs(ang_OF_rot(i)));
    else
       pos_ang_OF_rot(i) = abs(ang_OF_rot(i));
    end
    if (ang_OF(i) < 0)
        pos_ang_OF(i) = abs(2 * pi - abs(ang_OF(i)));
    else
        pos_ang_OF(i) = abs(ang_OF(i));
    end

    comp(i) =  cos(abs(pos_ang_OF_rot(i) - pos_ang_OF(i)));
    OF_proj(i) = mag_OF_rot(i) * comp(i);
    OF_der_pre(i) = mag_OF(i) - OF_proj(i);
    
    if (OF_der_pre(i) > 0)
        
            OF_der(i) = OF_der_pre(i);
        
        else
        
            OF_der(i) = 0;
        
    end
        u_der(i) = OF_der(i) * cos(pos_ang_OF(i));
        v_der(i) = OF_der(i) * sin(pos_ang_OF(i));
    
    
end

err = mag_OF./OF_proj;

% for i = 1:size(time)
% 
%     figure(1)  
%     clf;   
%     axis equal
% 
%     hold on
%     quiver(x(i),y(i),u_der(i),v_der(i),"AutoScale","off")
%     quiver(x(i),y(i),unpro_u(i),unpro_v(i),"AutoScale","off")
%     quiver(x(i),y(i),rot_u(i),rot_v(i),"AutoScale","off")
%     quiver(x(i),y(i),OF_proj(i)* cos(pos_ang_OF(i)),OF_proj(i)* sin(pos_ang_OF(i)),"AutoScale","off")
%     legend("derotated","unprocessed","rotational","projection")
% 
%     pause()
%     
% end




