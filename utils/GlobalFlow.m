
ExperimentName = 'Pitching';


filepath1 = append('../Experiments/',ExperimentName,'/trajectory.csv');
filepath2 = append('../Experiments/',ExperimentName,'/FoE_recording.txt');
filepath3 = append('../Experiments/',ExperimentName,'/FoE_flow_recording.txt');

filepath4 = append('../Experiments/',ExperimentName,'/HL_recording.txt');
filepath5 = append('../Experiments/',ExperimentName,'/HP_recording.txt');


OF_log = csvread("../Experiments/"+ExperimentName+"/OF_LOGFILE_.txt");

traj = csvread(filepath1,1);
FoE = csvread(filepath2,1);
Flow = csvread(filepath3,1);
HL = csvread(filepath4,1);
HP = csvread(filepath5,1);

time = OF_log(:,1);
x= OF_log(:,2);
y = OF_log(:,3);
pol = OF_log(:,4);
u = OF_log(:,5);
v = OF_log(:,6);
p = OF_log(:,7);
q = OF_log(:,8);
r = OF_log(:,9);
du = OF_log(:,10);
dv = OF_log(:,11);
rot_u = OF_log(:,12);
rot_v = OF_log(:,13);
foe = OF_log(:,14);


figure(1)
plot(time,[p,q,r]);
legend('p','q','r');
%% integrating the flow

TotalFlowU_rot = [];
TotalFlowV_rot = [];
TotalFlowU = [];
TotalFlowV = [];

for i = 1:size(FoE,1)
    Indeces = (Flow(:,1) == FoE(i,1));
    CorrespondingFlow = Flow(Indeces,:);
    
    TotalFlowU_rot = [TotalFlowU_rot sum((CorrespondingFlow(:,6)))/size(Indeces,1)];
    TotalFlowV_rot = [TotalFlowV_rot sum((CorrespondingFlow(:,7)))/size(Indeces,1)];
    
    TotalFlowU = [TotalFlowU sum((CorrespondingFlow(:,4)))./size(Indeces,1)];
    TotalFlowV = [TotalFlowV sum((CorrespondingFlow(:,5)))./size(Indeces,1)];
    
end

figure(2)

plot(FoE(:,1)/10^9,[TotalFlowU_rot.',TotalFlowV_rot.',TotalFlowU.',TotalFlowV.']);
legend('u including rotational component','v including rotational component','u derotated','v derotated');
    
    