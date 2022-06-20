function ErrorVersusAnglePlot(ListOfExperiments)
    
    ExperimentData = DataReader(ListOfExperiments);

    Experiments = string(fieldnames(ExperimentData));
    
    figure(2)
    clf;

    legendlist = [];
    
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
        
        subplot(2,1,1)
            hold on

        scatter(actual_x,err_x)
        
        subplot(2,1,2)
            hold on

        scatter(actual_y,err_y)
        %ErrListX.(Experiment) = ExperimentData.(Experiment).FoE(:,2) - ExperimentData.(Experiment).TrueFoE(indexlookup,2);
        %ErrListY.(Experiment) = ExperimentData.(Experiment).FoE(:,3) - ExperimentData.(Experiment).TrueFoE(indexlookup,3);


        legendlist = [legendlist string(Experiment)];


    end
    
    subplot(2,1,1)
    legend(legendlist);
   subplot(2,1,2)
    legend(legendlist);
    
    