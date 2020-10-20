clear; close all;
fontsize= 16;
intersection = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6', 'J7', 'J8', 'J9', 'J10'];
controllers = {'Fixed', 'HDMPC Np1', 'HDMPC Np2', 'HDMPC Np3'};

scenarios = 'sathorn_full_junctions_morning_100';
case_compare = 'case_1';

num_np = 3;
num_junct = 10;
max_gamma = 20000;
gamma = 0:1000:max_gamma;

Fixed = load_empty();
HDMPC = load_empty();

data = load([scenarios, '/fixed.mat']);
Fixed = put_junction_performance(Fixed, data.average_flow.junctions, data.average_travel_time.junctions);
Fixed_total.flow = data.average_flow.total;
Fixed_total.travel = data.average_travel_time.total;

HDMPC_total.flow = [];
HDMPC_total.travel = [];
for np = 1:num_np
    data = load([scenarios, '/', case_compare, '/HDMPC_Np', int2str(np),'.mat']);
    HDMPC = put_junction_performance(HDMPC, data.average_flow.junctions, data.average_travel_time.junctions);
    HDMPC_total.flow = [HDMPC_total.flow; data.average_flow.total];
    HDMPC_total.travel = [HDMPC_total.travel; data.average_travel_time.total];
end

figure;
for i = 4:5
    tmp_flow = [ones(num_np,1)*Fixed(i).flow, HDMPC(i).flow]/1000;
    max_flow = max(max(tmp_flow))*1.01;
    min_flow = min(min(tmp_flow))*0.99;

    subplot(2,2,(i-4)*2 + 1);
    %axes('ylim', [min_flow, max_flow], 'YTick', min_flow:(max_flow-min_flow)/5:max_flow, 'NextPlot', 'add');
    plot([0 max_gamma/1000], ones(1,2)*Fixed(i).flow/1000, '--k','LineWidth',2);
    plot_ex(gamma/1000, HDMPC(i).flow/1000); grid on;
    ylim([min_flow max_flow]);
    %yticks(min_flow:(max_flow-min_flow)/3:max_flow);
    %ytickformat('%.1f');
    %xlim([0 max_gamma/1000]);
    %xticks(0:3000/1000:max_gamma/1000);
    
    %yticks(min_flow:(max_flow-min_flow)/3:max_flow);
    %ytickformat('%.1f');
    
    %ylabel('Traffic flow (veh/h)');
    %legend(controllers);
    set(gca, 'fontsize', fontsize);
    title(['Intersection ', int2str(i)]);
 
    if i==5
        legend(controllers, 'Orientation','horizontal');
    end  
    
    if i==4 || i==5
        ylabel('{Traffic flow {(veh x {10^3}/h)}}');
    end
    
    if i==5
        xlabel('{Gamma \gamma x {10^3}}');
    end
end


for i = 4:5  
    tmp_travel = [ones(num_np,1)*Fixed(i).travel_time, HDMPC(i).travel_time]/1000;
    max_travel = max(max(tmp_travel))*1.01;
    min_travel = min(min(tmp_travel))*0.99;
    
    subplot(2,2,(i-4)*2 + 2);
    plot([0 max_gamma/1000], ones(1,2)*Fixed(i).travel_time/1000, '--k','LineWidth',2);
    plot_ex(gamma/1000, HDMPC(i).travel_time/1000); grid on;
    ylim([min_travel max_travel]);
    %yticks(min_travel:(max_travel-min_travel)/3:max_travel);
    %ytickformat('%.1f');
    xlim([0 max_gamma/1000]);
    %xticks(0:3000/1000:max_gamma/1000);
    set(gca, 'fontsize', fontsize);
    title(['Intersection ', int2str(i)]);
    %title(['(b) Estimated travel time on intersection ', int2str(i)]);
    
   
    if i==4 || i==5
        ylabel('{Travel time {(sec x {10^3})}}');
    end
    
    if i == 5
        xlabel('{Gamma \gamma x {10^3}}');
    end
end
%fig = gcf;
%fig.PaperPositionMode = 'auto';
%print('-fillpage','int_1_5','-dpdf')


figure;
for i = 6:10
    tmp_flow = [ones(num_np,1)*Fixed(i).flow, HDMPC(i).flow]/1000;
    max_flow = max(max(tmp_flow))*1.01;
    min_flow = min(min(tmp_flow))*0.99;

    subplot(5,2,(i-6)*2 + 1);
    plot([0 max_gamma/1000], ones(1,2)*Fixed(i).flow/1000, '--k','LineWidth',2);
    plot_ex(gamma/1000, HDMPC(i).flow/1000); grid on;
    ylim([min_flow max_flow]);
    %yticks(min_flow:(max_flow-min_flow)/3:max_flow);
    %ytickformat('%.1f');
    %xlim([0 max_gamma/1000]);
    %xticks(0:3000/1000:max_gamma/1000);
    %ylabel('Traffic flow (veh/h)');
    %legend(controllers);
    set(gca, 'fontsize', fontsize);
    title(['Intersection ', int2str(i)]);
 
    if i==10
        legend(controllers, 'Orientation','horizontal');
    end  
    
    if i==8
        ylabel('{Traffic flow {(veh x {10^3}/h)}}');
    end
    
    if i==10
        xlabel('{Gamma \gamma x {10^3}}');
    end
end


for i = 6:10  
    tmp_travel = [ones(num_np,1)*Fixed(i).travel_time, HDMPC(i).travel_time]/1000;
    max_travel = max(max(tmp_travel))*1.01;
    min_travel = min(min(tmp_travel))*0.99;
    
    subplot(5,2,(i-6)*2 + 2);
    plot([0 max_gamma/1000], ones(1,2)*Fixed(i).travel_time/1000, '--k','LineWidth',2);
    plot_ex(gamma/1000, HDMPC(i).travel_time/1000); grid on;
    ylim([min_travel max_travel]);
    %yticks(min_travel:(max_travel-min_travel)/3:max_travel);
    %ytickformat('%.1f');
    xlim([0 max_gamma/1000]);
    %xticks(0:3000/1000:max_gamma/1000);
    set(gca, 'fontsize', fontsize);
    title(['Intersection ', int2str(i)]);
    %title(['(b) Estimated travel time on intersection ', int2str(i)]);
    
   
    if i==8
        ylabel('{Travel time {(sec x {10^3})}}');
    end
    
    if i==10
        xlabel('{Gamma \gamma x {10^3}}');
    end
end
fig = gcf;
fig.PaperPositionMode = 'auto';
%print('-fillpage','int_6_10','-dpdf')

close all;
int_list(1).list = 6:10;
int_list(2).list = 1:3;
int_list(3).list = 4:5;
plot_for_area(Fixed, HDMPC, int_list);


close all;
tmp_flow = [ones(num_np,1)*Fixed_total.flow, HDMPC_total.flow]/1000;
max_flow = max(max(tmp_flow))*1.01;
min_flow = min(min(tmp_flow))*0.99;

tmp_travel = [ones(num_np,1)*Fixed_total.travel, HDMPC_total.travel]/1000;
max_travel = max(max(tmp_travel))*1.01;
min_travel = min(min(tmp_travel))*0.99;

figure;
subplot(2,2,1);
plot([0 max_gamma]/1000, ones(1,2)*Fixed_total.flow/1000, '--k','LineWidth',2);
plot_ex(gamma/1000, HDMPC_total.flow/1000); grid on;
ylim([min_flow max_flow]);
%yticks(min_flow:(max_flow-min_flow)/3:max_flow);
%ytickformat('%.1f');
xlim([0 max_gamma/1000]);
%xticks(0:3000/1000:max_gamma/1000);
ylabel('{Traffic flow {(veh x {10^3}/h)}}');
xlabel('{Gamma \gamma x {10^3}}');
set(gca, 'fontsize', fontsize);
%title('(a) Total traffic flow on network');

subplot(2,2,2);
plot([0 max_gamma]/1000, ones(1,2)*Fixed_total.travel/1000, '--k','LineWidth',2);
plot_ex(gamma/1000, HDMPC_total.travel/1000); grid on;
ylim([min_travel max_travel]);
%yticks(min_travel:(max_travel-min_travel)/3:max_travel);
%ytickformat('%.1f');
xlim([0 max_gamma/1000]);
%xticks(0:3000/1000:max_gamma/1000);
ylabel('{Travel time {(sec x {10^3})}}');
%xlabel('{Gamma \gamma x {10^3}}');
set(gca, 'fontsize', fontsize);
legend(controllers,'Orientation','horizontal');
%title('(b) Average of estimated travel time on network');

fig = gcf;
fig.PaperPositionMode = 'manual';
orient(fig,'landscape');
%print('-fillpage','network','-dpdf');
