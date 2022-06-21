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
        Experiment.(ExperimentName).FOV_X = 61.7164;
        Experiment.(ExperimentName).FOV_Y = 48.2168;
        
        diff = abs(Experiment.(ExperimentName).FoE(:,1).' - Experiment.(ExperimentName).TrueFoE(:,1));
        minn = min((diff));

        indexlookup = mod(find(diff == minn),size(Experiment.(ExperimentName).TrueFoE,1));
        indexlookup(indexlookup == 0) = size(Experiment.(ExperimentName).TrueFoE,1);


        Experiment.(ExperimentName).Actual_X = Experiment.(ExperimentName).TrueFoE(indexlookup,2);
        Experiment.(ExperimentName).Error_X = Experiment.(ExperimentName).FoE(:,2) - Experiment.(ExperimentName).TrueFoE(indexlookup,2);
        
        Experiment.(ExperimentName).Actual_Y = Experiment.(ExperimentName).TrueFoE(indexlookup,3);
        Experiment.(ExperimentName).Error_Y = Experiment.(ExperimentName).FoE(:,3) - Experiment.(ExperimentName).TrueFoE(indexlookup,3);
        
        
        %% convert to deg

        
        Experiment.(ExperimentName).FoE(:,1) =  (Experiment.(ExperimentName).FoE(:,1)-Experiment.(ExperimentName).FoE(1,1))/10^6;
        Experiment.(ExperimentName).FoE(:,2) =     atan(      (120-    Experiment.(ExperimentName).FoE(:,2))/120 * tan( Experiment.(ExperimentName).FOV_X/180*pi/2)   )   *180/pi;
        Experiment.(ExperimentName).FoE(:,3) = -   atan(      (90-     Experiment.(ExperimentName).FoE(:,3))/90  * tan( Experiment.(ExperimentName).FOV_Y/180*pi/2)   )   *180/pi;
        
        Experiment.(ExperimentName).ExpectedTrajectory(:,1) = Experiment.(ExperimentName).ExpectedTrajectory(:,1)/10^9;
        

        
    end
         Experiment = orderfields(Experiment);


end