close all;
x0      = 100; y0       = 100;
width   = 800; height   = 600;
fontsize= 16;

cycle_length = [150, 135, 150, 150, 150, 165, 150, 150, (3600*3)/171, 150];

tmp(1).roads = {'R2', 'R3', 'R6', 'R7', 'R1', 'R4', 'R5', 'R8'};
tmp(2).roads = {'R8', 'R9', 'R12', 'R13', 'R7', 'R10', 'R11', 'R14'};
tmp(3).roads = {'R10', 'R15', 'R49', 'R9', 'R16', 'R48'};
tmp(4).roads = {'R5', 'R18', 'R20', 'R21', 'R6', 'R17', 'R19', 'R22'};
tmp(5).roads = {'R11', 'R22', 'R24', 'R25', 'R12', 'R21', 'R23', 'R26'};
tmp(6).roads = {'R28', 'R30', 'R31', 'R27', 'R29', 'R32'};
tmp(7).roads = {'R29', 'R34', 'R36', 'R37', 'R30', 'R33', 'R35', 'R38'};
tmp(8).roads = {'R35', 'R40', 'R4', 'R36', 'R39', 'R41', 'R3'};
tmp(9).roads = {'R32', 'R43', 'R44', 'R31', 'R42', 'R45'};
tmp(10).roads = {'R16', 'R41', 'R42', 'R46', 'R15', 'R43', 'R47'};


for k=1:2:10
    
    dt = cycle_length(k);
    t = 0:dt:(3600*3);

    load(['logs/controllers/intersections/J', int2str(k),'.mat']);
    
    num_phase = size(control_signal_log);
    num_phase = num_phase(2);
    phase = 1:num_phase;    
    legendCell = cellstr(num2str(phase', 'u_%-d'));
    max_t = length(mean_output_log);
    
    figure;
    subplot(2,2,1);
    plot_ex(t(1:max_t), mean_output_log');
    legend(tmp(k).roads);
    grid on;
    ylabel('Mean output (veh)');
    xlim([0 3600*3]);
    title(['Intersection ', int2str(k)]);
    %title('(a) Mean vehicles queue length of each road');
    set(gca, 'fontsize', fontsize);

    subplot(2,2,3);
    plot_ex(t(1:max_t), control_signal_log');
    legend(legendCell);
    grid on;
    ylabel('Control signal (s)');
    xlabel('Time (s)');
    xlim([0 3600*3]);
    %title('(b) Traffic signal');
    set(gca, 'fontsize', fontsize);
    
    
    
    dt = cycle_length(k+1);
    t = 0:dt:(3600*3);

    load(['logs/controllers/intersections/J', int2str(k+1),'.mat']);
    
    num_phase = size(control_signal_log);
    num_phase = num_phase(2);
    phase = 1:num_phase;    
    legendCell = cellstr(num2str(phase', 'u_%-d'));
    max_t = length(mean_output_log);
    
    subplot(2,2,2);
    plot_ex(t(1:max_t), mean_output_log');
    legend(tmp(k+1).roads);
    grid on;
    ylabel('Mean output (veh)');
    xlim([0 3600*3]);
    title(['Intersection ', int2str(k+1)]);
    %title('(a) Mean vehicles queue length of each road');
    set(gca, 'fontsize', fontsize);

    subplot(2,2,4);
    plot_ex(t(1:max_t), control_signal_log');
    legend(legendCell);
    grid on;
    ylabel('Control signal (s)');
    xlabel('Time (s)');
    xlim([0 3600*3]);
    %title('(b) Traffic signal');
    set(gca, 'fontsize', fontsize);
    
%     fig = gcf;
%     surf(peaks);
%     fig.PaperPositionMode = 'manual';
%     orient(fig,'landscape');
%     print('-bestfit', ['J', int2str(k), '_J', int2str(k+1)],'-dpdf');
%     close all;
end