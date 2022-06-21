function PerformancePlot(ListOfExperiments)
        close all
        FOV_X = 61.7164;
        FOV_Y = 48.2168;
        

        ExperimentData = DataReader(ListOfExperiments);

        Experiments = string(fieldnames(ExperimentData));
    
    
    
    for i = 1:size(Experiments)
        Experiment = Experiments(i);
        
        diff = abs(ExperimentData.(Experiment).FoE(:,1).' - ExperimentData.(Experiment).TrueFoE(:,1));
        minn = min((diff));
        indexlookup = mod(find(diff == minn),size(ExperimentData.(Experiment).TrueFoE,1));
        indexlookup(indexlookup == 0) = size(ExperimentData.(Experiment).TrueFoE,1);
        actual_x = ExperimentData.(Experiment).TrueFoE(indexlookup,2);
        err_x = ExperimentData.(Experiment).FoE(:,2) - ExperimentData.(Experiment).TrueFoE(indexlookup,2);
        actual_y = ExperimentData.(Experiment).TrueFoE(indexlookup,3);
        err_y = ExperimentData.(Experiment).FoE(:,3) - ExperimentData.(Experiment).TrueFoE(indexlookup,3);
        
        
        
        figure();        
        subplot(2,1,1)
        hold on
        plot(ExperimentData.(Experiment).FoE(:,1),ExperimentData.(Experiment).FoE(:,2))
        plot(ExperimentData.(Experiment).TrueFoE(:,1),ExperimentData.(Experiment).TrueFoE(:,2));
        ylim([-FOV_X/2*1.05,FOV_X/2*1.05])
        yline([-FOV_X/2,FOV_X/2],'--')
        legend('Estimated FoE_x','True FoE_x','Lower bound','Upper bound')
        ylabel('FoE position [deg]') 
        xlabel('time [s]') 

        subplot(2,1,2)
        hold on
        plot(ExperimentData.(Experiment).FoE(:,1),ExperimentData.(Experiment).FoE(:,3))
        plot(ExperimentData.(Experiment).TrueFoE(:,1),ExperimentData.(Experiment).TrueFoE(:,3));
        ylim([-FOV_Y/2*1.05,FOV_Y/2*1.05])
        yline([-FOV_Y/2,FOV_Y/2],'--')
        legend('Estimated FoE_y','True FoE_y','Lower bound','Upper bound')
        ylabel('FoE position [deg]') 
        xlabel('time [s]') 


        set(gcf,'Position',[100 100 700 800])

        filepath = append('../Experiments/',Experiment,'/FoE_validation.png');
        saveas(gcf,filepath)

        figure()
        subplot(2,1,1)
        hold on
        plot(ExperimentData.(Experiment).SimulatedTrajectory(:,2),[ExperimentData.(Experiment).SimulatedTrajectory(:,3),ExperimentData.(Experiment).SimulatedTrajectory(:,4),ExperimentData.(Experiment).SimulatedTrajectory(:,5)])
        plot(ExperimentData.(Experiment).ExpectedTrajectory(:,1),[ExperimentData.(Experiment).ExpectedTrajectory(:,2),ExperimentData.(Experiment).ExpectedTrajectory(:,3),ExperimentData.(Experiment).ExpectedTrajectory(:,4)],'--')
        legend('simulated_x','simulated_y','simulated_z','planned_x','planned_y','planned_z')

        subplot(2,1,2)
        hold on
        plot(ExperimentData.(Experiment).SimulatedTrajectory(:,2),[ExperimentData.(Experiment).SimulatedTrajectory(:,3),ExperimentData.(Experiment).SimulatedTrajectory(:,3),ExperimentData.(Experiment).SimulatedTrajectory(:,3)])
        plot(ExperimentData.(Experiment).ExpectedTrajectory(:,1),quat2eul([ExperimentData.(Experiment).ExpectedTrajectory(:,5),ExperimentData.(Experiment).ExpectedTrajectory(:,6),ExperimentData.(Experiment).ExpectedTrajectory(:,7),ExperimentData.(Experiment).ExpectedTrajectory(:,8)]),'--')
        legend('sim_roll','sim_pitch','sim_yaw','planned_roll','planned_pitch','planned_yaw')

        figure()
        subplot(2,2,1)
        plot(ExperimentData.(Experiment).FoE(:,1),ExperimentData.(Experiment).Error_X)
        yline(mean(ExperimentData.(Experiment).Error_X),'--')
        subplot(2,2,3)
        plot(ExperimentData.(Experiment).FoE(:,1),ExperimentData.(Experiment).Error_Y)
        yline(mean(ExperimentData.(Experiment).Error_Y),'--')

        subplot(2,2,2)
        histogram(ExperimentData.(Experiment).Error_X,30);
        subplot(2,2,4);
        histogram(ExperimentData.(Experiment).Error_Y,30);

        disp("Mean absolute error x: "+string(mean(abs(ExperimentData.(Experiment).Error_X))))
        disp("Mean absolute error y: "+string(mean(abs(ExperimentData.(Experiment).Error_Y))))
        disp("Median amount of flow vectors: "+string(median(ExperimentData.(Experiment).FoE(:,5))))
        
    end
end
