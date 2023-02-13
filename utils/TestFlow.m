ExperimentName = 'Downward_6Dflight_1';
Variation = '50_average';

filepath1 = append('../Experiments/',ExperimentName,'/trajectory.csv');
filepath2 = append('../Experiments/',ExperimentName,'/',Variation,'/FoE_recording.txt');
filepath3 = append('../Experiments/',ExperimentName,'/',Variation,'/FoE_flow_recording.txt');

filepath4 = append('../Experiments/',ExperimentName,'/',Variation,'/HL_recording.txt');
filepath5 = append('../Experiments/',ExperimentName,'/',Variation,'/HP_recording.txt');

M = csvread("../Experiments/"+ExperimentName+"/"+Variation+"/OF_LOGFILE.txt");

p = M(:,7);
q = M(:,8);
r = M(:,9);
time = (M(:,1)-M(1,1))/1e6;

%clear M

figure(21)
clf;
plot(time,[p,q,r])

traj = csvread(filepath1,8);




FoE = csvread(filepath2,1);

Flow = csvread(filepath3,1);
HL = csvread(filepath4,1);

HP = csvread(filepath5,1);

time = (FoE(:,1)-FoE(1,1))/1e6;

flowindex = 1;
HPindex = 1;
HLindex = 1;

figure(22)
clf;

disp('figure plotted')
for i = 200:size(FoE,1)
    %pause
    clf;
    xlim([0,240])
    ylim([0,180])
    axis equal
    %viscircles([FoE(i,2),FoE(i,3)],10);
    viscircles([FoE(i,4),FoE(i,5)],5);
    
%     scatter(FoE(i,4),FoE(i,5),2000,'Marker','+','MarkerEdgeColor','Red');
%     scatter(FoE(i,4),FoE(i,5),'Marker','+','SizeData',100);

    %disp([FoE(i,2),FoE(i,3)])
    hold on
    %disp(flowindex)
    Indeces = (Flow(:,1) == FoE(i,1));
    CorrespondingFlow = Flow(Indeces,:);
    
    scaling = 70;

    quiver(CorrespondingFlow(:,2),CorrespondingFlow(:,3),scaling*CorrespondingFlow(:,4),scaling*CorrespondingFlow(:,5),'Color','Red',"AutoScale","off")
   % quiver(CorrespondingFlow(:,2),CorrespondingFlow(:,3),scaling*CorrespondingFlow(:,6),scaling*CorrespondingFlow(:,7),'Color','Blue',"AutoScale","off")
    %quiver(CorrespondingFlow(:,2),CorrespondingFlow(:,3),scaling*(CorrespondingFlow(:,6)-CorrespondingFlow(:,4)),scaling*(CorrespondingFlow(:,7)-CorrespondingFlow(:,5)),'Color','Black',"AutoScale","off")
    %scatter(CorrespondingFlow(:,2),CorrespondingFlow(:,3),'+')
    

    %disp([CorrespondingFlow(:,4),CorrespondingFlow(:,5),CorrespondingFlow(:,6),CorrespondingFlow(:,7)])
    
    Indeces = (HP(:,1) == FoE(i,1));
   CorrespondingHP = HP(Indeces,:);

    %disp(flowindex)
    viscircles([CorrespondingHP(:,2),CorrespondingHP(:,3)],3*ones(size(CorrespondingHP,1),1),'Color','Black');
    %disp([HP(HPindex,2),HP(HPindex,3)])
    
    
    Indeces = (HL(:,1) == FoE(i,1));
    
       CorrespondingHL = HL(Indeces,:);
    
       disp(CorrespondingHL(:,2:4));
    %disp(flowindex)
    y1 = -CorrespondingHL(:,4)./CorrespondingHL(:,3);
    y2 = -(CorrespondingHL(:,2).*240+CorrespondingHL(:,4))./CorrespondingHL(:,3);
    line([0,240],[y1,y2],'LineStyle',':','LineWidth',1,'Color','Black');
%     
    
    axis equal
    set(gca, 'YDir','reverse')
    xlim([-5,245])
    ylim([-5,185])
    disp('new, i = '+string(i)+', size = '+string(size(CorrespondingFlow,1))+', time = '+string((FoE(i,1)-FoE(1,1))/10^6))
    legend('FoE estimation','Derotated vectors');
%     legend('FoE estimation','Derotated vectors','Not derotated vectors','Rot component');

    title('')
    title(time(i))
    pause();
    
end