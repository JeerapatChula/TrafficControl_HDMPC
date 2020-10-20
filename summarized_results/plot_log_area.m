close all;
x0      = 100; y0       = 100;
width   = 800; height   = 600;
fontsize= 16;

dt = 5*60;
t = 0:dt:(3600*3);

tmp(1).links = {'R15', 'R16', 'R27', 'R28', 'R29', 'R3', 'R30', 'R31', 'R32', 'R33', 'R34', 'R35', 'R36', 'R37', 'R38', 'R39', 'R4', 'R40', 'R41', 'R42', 'R43', 'R44', 'R45', 'R46', 'R47'};
tmp(1).nodes = {'B1:in', 'B1:out', 'B2:in', 'B2:out', 'B3:in', 'B3:out', 'B4:in', 'B4:out', 'B5:in', 'B5:out', 'B6:in', 'B6:out', 'J10', 'J6', 'J7', 'J8', 'J9'};

tmp(2).links = {'R1', 'R10', 'R11', 'R12', 'R13', 'R14', 'R15', 'R16', 'R2', 'R3', 'R4', 'R48', 'R49', 'R5', 'R6', 'R7', 'R8', 'R9'};
tmp(2).nodes = {'B7:in', 'B7:out', 'B8:in', 'B8:out', 'B9:in', 'B9:out', 'J1', 'J2', 'J3'};

tmp(3).links = {'R11', 'R12', 'R17', 'R18', 'R19', 'R20', 'R21', 'R22', 'R23', 'R24', 'R25', 'R26', 'R5', 'R6'};
tmp(3).nodes = {'B10:in', 'B10:out', 'B11:in', 'B11:out', 'B12:in', 'B12:out', 'B13:in', 'B13:out', 'J4', 'J5'};


for k=1:3
    load(['logs/controllers/area/A', int2str(k), '.mat']);
    
    figure;
    subplot(2,1,1);
    plot_ex(t, state_log');
    columnlegend(2, tmp(k).nodes);
%     legend(tmp(k).nodes);
    grid on;
    ylabel('State (veh)');
    %title(['(a) Demand of each node of A', int2str(k)]);
    set(gca, 'fontsize', fontsize);

    subplot(2,1,2);
    plot_ex(t, 3600*control_signal_log'/dt);
    columnlegend(2, tmp(k).links);
%     legend(tmp(k).links);
    grid on;
    ylabel('Control signal (veh/h)');
    %title('(b) Control signal');
    xlabel('Time (s)');
    set(gca, 'fontsize', fontsize);
end