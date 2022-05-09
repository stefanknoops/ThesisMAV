ExperimentName = 'SlowRollCarpet';

filepath1 = append('../Experiments/',ExperimentName,'.csv');
traj = csvread(filepath1,1);

filepath2 = append('../Experiments/',ExperimentName,'/FoE_recording.txt');

filepath3 = append('../Experiments/',ExperimentName,'/FoE_flow_recording.txt');

filepath4 = append('../Experiments/',ExperimentName,'/HL_recording.txt');

filepath5 = append('../Experiments/',ExperimentName,'/HP_recording.txt');

FoE = csvread(filepath2,1);

Flow = csvread(filepath3,1);
HL = csvread(filepath4,1);

HP = csvread(filepath5,1);


flowindex = 1;
HPindex = 1;
HLindex = 1;

figure(2)


disp('figure plotted')
for i = 1:size(FoE,1)
    %pause
    clf;
    xlim([0,240])
    ylim([0,180])
    axis equal
    viscircles([FoE(i,2),FoE(i,3)],5);
    %disp([FoE(i,2),FoE(i,3)])
    hold on
    %disp(flowindex)
    Indeces = (Flow(:,1) == FoE(i,1));
    CorrespondingFlow = Flow(Indeces,:);
    quiver(CorrespondingFlow(:,2),CorrespondingFlow(:,3),CorrespondingFlow(:,4),CorrespondingFlow(:,5),1,'Color','Red')

    
    
    Indeces = (HP(:,1) == FoE(i,1));
   CorrespondingHP = HP(Indeces,:);

    %disp(flowindex)
    viscircles([CorrespondingHP(:,2),CorrespondingHP(:,3)],3*ones(size(CorrespondingHP,1),1),'Color','Yellow');
    %disp([HP(HPindex,2),HP(HPindex,3)])
    
    
    Indeces = (HL(:,1) == FoE(i,1));
    
       CorrespondingHL = HL(Indeces,:);

    %disp(flowindex)
    y1 = -CorrespondingHL(:,4)./CorrespondingHL(:,3);
    y2 = -(CorrespondingHL(:,2).*240+CorrespondingHL(:,4))./CorrespondingHL(:,3);
    line([0,240],[y1,y2],'LineStyle',':','LineWidth',1);
    
    
    axis equal
    set(gca, 'YDir','reverse')
    xlim([0,240])
    ylim([0,180])
    disp('new, i = '+string(i)+', size = '+string(size(CorrespondingFlow,1)))
    
    pause();
    
end