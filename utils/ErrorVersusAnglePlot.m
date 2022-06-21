function ErrorVersusAnglePlot(ListOfExperiments)
    
    ExperimentData = DataReader(ListOfExperiments);

    Experiments = string(fieldnames(ExperimentData));
    
    figure(2)
    clf;

    legendlist = [];
    
    for i = 1:size(Experiments)
        Experiment = Experiments(i);
%         diff = abs(ExperimentData.(Experiment).FoE(:,1).' - ExperimentData.(Experiment).TrueFoE(:,1));
%         minn = min((diff));
% 
%         indexlookup = mod(find(diff == minn),size(ExperimentData.(Experiment).TrueFoE,1));
%         indexlookup(indexlookup == 0) = size(ExperimentData.(Experiment).TrueFoE,1);
% 
% 
%         actual_x = ExperimentData.(Experiment).Actual_X;
%         err_x = ExperimentData.(Experiment).Error_X;
%         
%         actual_y = ExperimentData.(Experiment).Actual_Y;
%         err_y = ExperimentData.(Experiment).Error_Y;
        
        subplot(2,1,1)
            hold on

        scatter(ExperimentData.(Experiment).Actual_X,ExperimentData.(Experiment).Error_X)
        
        subplot(2,1,2)
            hold on

        scatter(ExperimentData.(Experiment).Actual_Y,ExperimentData.(Experiment).Error_Y)
        %ErrListX.(Experiment) = ExperimentData.(Experiment).FoE(:,2) - ExperimentData.(Experiment).TrueFoE(indexlookup,2);
        %ErrListY.(Experiment) = ExperimentData.(Experiment).FoE(:,3) - ExperimentData.(Experiment).TrueFoE(indexlookup,3);


        legendlist = [legendlist string(Experiment)];


    end
    
    subplot(2,1,1)
    legend(legendlist);
   subplot(2,1,2)
    legend(legendlist);
    
    