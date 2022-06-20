%ExperimentData = data

function PlotErrorViolins(ListOfExperiments)
    
    ExperimentData = DataReader(ListOfExperiments);

    Experiments = string(fieldnames(ExperimentData));


    for i = 1:size(Experiments)
        Experiment = Experiments(i);
        diff = abs(ExperimentData.(Experiment).FoE(:,1).' - ExperimentData.(Experiment).TrueFoE(:,1));
        minn = min((diff));

        indexlookup = mod(find(diff == minn),size(ExperimentData.(Experiment).TrueFoE,1));
        indexlookup(indexlookup == 0) = size(ExperimentData.(Experiment).TrueFoE,1);


        ErrListX.(Experiment)= ExperimentData.(Experiment).FoE(:,2) - ExperimentData.(Experiment).TrueFoE(indexlookup,2);
        ErrListY.(Experiment) = ExperimentData.(Experiment).FoE(:,3) - ExperimentData.(Experiment).TrueFoE(indexlookup,3);





    end
    
    ErrListX = orderfields(ErrListX,ListOfExperiments);
    
    figure(1)
    subplot(2,1,1)
    violinplot(ErrListX,[],'DataStyle','none','GroupOrder',num2cell(ListOfExperiments),'HalfViolin','right','Width',0.8);
        xlim([0.5, size(ListOfExperiments,2)+1])
    xlabel('Experiment name')
    ylabel('Error in X')
    subplot(2,1,2)
    violinplot(ErrListY,[],'DataStyle','none','GroupOrder',num2cell(ListOfExperiments),'HalfViolin','right','Width',0.8);
        xlim([0.5, size(ListOfExperiments,2)+1])

    xlabel('Experiment name')
    ylabel('Error in Y')



