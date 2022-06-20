%ExperimentName = 'StraightDown';
function Experiment = DataReader(ListOfExperiments)

    %nameFolds = {'Test'}

    for i = 1:size(ListOfExperiments,2)
        filepath1 = append('../Experiments/',ListOfExperiments(i),'/trajectory.csv');
        filepath2 = append('../Experiments/',ListOfExperiments(i),'/FoE_recording.txt');
        filepath3 = append('../Experiments/',ListOfExperiments(i),'/Expected_FoE.csv');
        filepath31 = append('../Experiments/',ListOfExperiments(i),'/FoE_flow_recording.txt');
        filepath4 = append('../Experiments/',ListOfExperiments(i),'/SimulatedPose.csv');
        filepath5 = append('../Experiments/',ListOfExperiments(i),'/HL_recording.txt');
        filepath6 = append('../Experiments/',ListOfExperiments(i),'/HP_recording.txt');

        ExperimentName = string(ListOfExperiments(i));
        Experiment.(ExperimentName).ExpectedTrajectory = csvread(string(filepath1),1);    
        Experiment.(ExperimentName).FoE = csvread(string(filepath2),1);
        Experiment.(ExperimentName).TrueFoE = csvread(string(filepath3),1);
        Experiment.(ExperimentName).Flow = csvread(string(filepath31),1);
        Experiment.(ExperimentName).SimulatedTrajectory = csvread(string(filepath4),1);
        Experiment.(ExperimentName).HullLines = csvread(string(filepath5),1);
        Experiment.(ExperimentName).HullPoints = csvread(string(filepath6),1);
        
        
        %% convert to deg
        FOV_X = 61.7164*pi/180;
        FOV_Y = 48.2168*pi/180;
        
        Experiment.(ExperimentName).FoE(:,1) =  (Experiment.(ExperimentName).FoE(:,1)-Experiment.(ExperimentName).FoE(1,1))/10^6;
        Experiment.(ExperimentName).FoE(:,2) =     atan(      (120-    Experiment.(ExperimentName).FoE(:,2))/120 * tan(FOV_X/2)   )   *180/pi;
        Experiment.(ExperimentName).FoE(:,3) = -   atan(      (90-     Experiment.(ExperimentName).FoE(:,3))/90  * tan(FOV_Y/2)   )   *180/pi;
        
        Experiment.(ExperimentName).ExpectedTrajectory(:,1) = Experiment.(ExperimentName).ExpectedTrajectory(:,1)/10^9;
        
        
    end
         Experiment = orderfields(Experiment);

end