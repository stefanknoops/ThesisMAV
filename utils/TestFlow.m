datetime = "2022_04_28-11_50_47";

FoE = csvread("FoE_recording_"+datetime+".txt");

Flow = csvread("FoE_flow_recording_"+datetime+".txt");

HP = csvread("HP_recording"+datetime+".txt");
HL = csvread("HL_recording"+datetime+".txt");

flowindex = 1;
HPindex = 1;
HLindex = 1;

figure(1)  


disp('figure plotted')
for i = 1:size(FoE,1)
    %pause
    clf;
    disp('new, i = '+string(i))
    xlim([0,240])
    ylim([0,180])
    axis equal
    viscircles([FoE(i,2),FoE(i,3)],5);
    %disp([FoE(i,2),FoE(i,3)])
    hold on
    while FoE(i,1) == Flow(flowindex,1) 
        %disp(flowindex)
        if Flow(flowindex,2) == 0
            quiver(Flow(flowindex,3),Flow(flowindex,4),Flow(flowindex,5),Flow(flowindex,6),1000,'Color','Red')
        end
%         if Flow(flowindex,2) == 1
%             quiver(Flow(flowindex,3),Flow(flowindex,4),Flow(flowindex,5),Flow(flowindex,6),300,'Color','Black')
%         end
        flowindex = flowindex + 1;
        
    end
    
    while FoE(i,1) == HP(HPindex,1) 
        %disp(flowindex)
        viscircles([HP(HPindex,2),HP(HPindex,3)],3,'Color','Yellow');
        %disp([HP(HPindex,2),HP(HPindex,3)])
        HPindex = HPindex + 1;
        
    end
    
    while FoE(i,1) == HL(HLindex,1) 
        %disp(flowindex)
        y1 = -HL(HLindex,4)/HL(HLindex,3);
        y2 = -(HL(HLindex,2)*240+HL(HLindex,4))/HL(HLindex,3);
        line([0,240],[y1,y2],'LineStyle',':','LineWidth',1);
        HLindex = HLindex + 1;
        
    end
    
    axis equal
    set(gca, 'YDir','reverse')
    xlim([0,240])
    ylim([0,180])
    pause();

end