
function plot_ex(X, Y)
    set(gcf,'defaultAxesColorOrder','remove', 'DefaultAxesLineStyleOrder', {'--^','--s','-.+','--x','-.v','-*'});
    ax = gca;
    %ax.LineStyleOrder = {'--^','--s','-.+','--x','-.v'};
    ax.LineStyleOrder = {'-^','-s','-+','-x','-v','-*'};
    row = size(Y, 1);
    
    hold on;
    for k=1:row
        plot(X, Y(k,:),'LineWidth',2);
        ax.LineStyleOrderIndex = ax.ColorOrderIndex;
    end
end