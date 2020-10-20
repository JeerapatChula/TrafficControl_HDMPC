function plot_for_area(Fixed_data, HDMPC_data, int)
    fontsize= 16;
    num_np = 3;
    max_gamma = 12000/1000;
    gamma = [0:3:max_gamma];
    controllers = {'Fixed', 'HDC', 'HDMPC Np2', 'HDMPC Np3'};

    figure;
    for a=1:length(int)
        int_list = int(a).list;
        Fixed_total_flow = sum([Fixed_data(int_list).flow]);
        HDMPC_total_flow = 0;
        for k=1:length(int_list)
            HDMPC_total_flow = HDMPC_total_flow + HDMPC_data(int_list(k)).flow;
        end
        
        tmp_flow = [ones(num_np,1)*Fixed_total_flow, HDMPC_total_flow]/1000;
        max_flow = max(max(tmp_flow))*1.01;
        min_flow = min(min(tmp_flow))*0.99;
    
        %figure('Name',['Area ', int2str(area)]);
        subplot(3,2,(a-1)*2 + 1);
        plot([0 max_gamma], ones(1,2)*Fixed_total_flow/1000, '--k','LineWidth',2);
        plot_ex(gamma, HDMPC_total_flow/1000); grid on;
        ylim([min_flow max_flow]);       
        yticks(min_flow:(max_flow-min_flow)/3:max_flow);
        ytickformat('%.1f');
        xlim([0 max_gamma]);
        xticks(gamma);
        %ylabel('Flow rate (veh/h)');
        %legend(controllers);
        set(gca, 'fontsize', fontsize);
        %title(['(a) Total traffic flow on area ', int2str(area)]);
        title(['Area ', int2str(a)]);
        if a==3
            legend(controllers, 'Orientation','horizontal');
        end  

        if a==2
            ylabel('{Traffic flow {(veh x {10^3}/h)}}');
        end

        if a==3
            xlabel('{Gamma \gamma x {10^3}}');
        end
    end
    
 
    for a=1:length(int)
        int_list = int(a).list;
        Fixed_total_travel = sum([Fixed_data(int_list).travel_time])/length(int_list);
        HDMPC_total_travel = 0;
        for k=1:length(int_list)
            HDMPC_total_travel = HDMPC_total_travel + HDMPC_data(int_list(k)).travel_time;
        end

        HDMPC_total_travel = HDMPC_total_travel/length(int_list);
        tmp_travel = [ones(num_np,1)*Fixed_total_travel, HDMPC_total_travel]/1000;
        max_travel = max(max(tmp_travel))*1.01;
        min_travel = min(min(tmp_travel))*0.99;

        subplot(3,2,(a-1)*2 + 2);
        plot([0 max_gamma], ones(1,2)*Fixed_total_travel/1000, '--k','LineWidth',2);
        plot_ex(gamma, HDMPC_total_travel/1000); grid on;
        ylim([min_travel max_travel]);
        yticks(min_travel:(max_travel-min_travel)/3:max_travel);
        ytickformat('%.1f');
        xlim([0 max_gamma]);
        xticks(gamma);
        %ylabel('Travel time (s)');
        %xlabel('Gamma');
        set(gca, 'fontsize', fontsize);
        %title(['(b) Average of estimated travel time on area ', int2str(area)]);
        title(['Area ', int2str(a)]);
        if a==3
            %legend(controllers, 'Orientation','horizontal');
        end  

        if a==2
            ylabel('{Travel time {(sec x {10^3})}}');
        end

        if a==3
            xlabel('{Gamma \gamma x {10^3}}');
        end
    end
    %fig = gcf;
    %fig.PaperPositionMode = 'manual';
    %orient(fig,'landscape');
    %print('-fillpage','area','-dpdf')
end