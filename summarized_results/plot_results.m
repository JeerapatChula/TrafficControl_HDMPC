clear; close all;
fontsize= 16;
profiles = {'Fixed', 'case_1'};
intersection = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6', 'J7', 'J8', 'J9', 'J10'];
controllers = ['HDMPC Np1', 'HDMPC Np2', 'HDMPC Np3'];

num_case = 2;
num_junct = 10;
max_gamma = 20000;
gamma = 0:1000:20000;

Fixed = load_empty();
HDC = load_empty();
HDMPC_Np2 = load_empty();
HDMPC_Np3 = load_empty();

RMSE_HDC = [];
RMSE_HDMPC_Np2 = [];
RMSE_HDMPC_Np3 = [];

offset = 0;

data = load('sathorn_full_junctions_morning_100/fixed.mat');
Fixed = put_junction_performance(Fixed, data.intersection_performance);

for c = 1:num_case
    data = load(['sathorn_full_junctions_morning_100/case_', int2str(c + offset), '/HDMPC_PROOF_Np1.mat']);
    HDC = put_junction_performance(HDC, data.intersection_performance);
    %RMSE_HDC = [RMSE_HDC; data.flow_rmse];
    
    data = load(['sathorn_full_junctions_morning_100/case_', int2str(c + offset), '/HDMPC_PROOF_Np2.mat']);
    HDMPC_Np2 = put_junction_performance(HDMPC_Np2, data.intersection_performance);
    %RMSE_HDMPC_Np2 = [RMSE_HDMPC_Np2; data.flow_rmse];
    
    data = load(['sathorn_full_junctions_morning_100/case_', int2str(c + offset), '/HDMPC_PROOF_Np3.mat']);
    HDMPC_Np3 = put_junction_performance(HDMPC_Np3, data.intersection_performance);
    %RMSE_HDMPC_Np3 = [RMSE_HDMPC_Np3; data.flow_rmse];
end


for i = 1:num_junct
    
    max_y = max(max([ones(num_case,1)*Fixed(i).data, HDC(i).data, HDMPC_Np2(i).data, HDMPC_Np3(i).data]))*1.01;
    min_y = min(min([ones(num_case,1)*Fixed(i).data, HDC(i).data, HDMPC_Np2(i).data, HDMPC_Np3(i).data]))*0.99;
    
    figure;
    subplot(3,1,1);
    plot([0 max_gamma], ones(1,2)*Fixed(i).data, '--k','LineWidth',2);
    plot_ex(gamma, HDC(i).data); grid on;
    ylim([min_y max_y]);
    ylabel('Flow rate (veh/h)');
    legend(profiles);
    set(gca, 'fontsize', fontsize);
    title(['(a) Performance of HDC on intersection ', int2str(i)]);
    
    subplot(3,1,2);
    plot([0 max_gamma], ones(1,2)*Fixed(i).data, '--k','LineWidth',2);
    plot_ex(gamma, HDMPC_Np2(i).data); grid on;
    ylim([min_y max_y]);
    ylabel('Flow rate (veh/h)');
    set(gca, 'fontsize', fontsize);
%     legend(profiles);
    title(['(b) Performance of HDMPC Np2 on intersection ', int2str(i)]);
    
    subplot(3,1,3);
    plot([0 max_gamma], ones(1,2)*Fixed(i).data, '--k','LineWidth',2);
    plot_ex(gamma, HDMPC_Np3(i).data); grid on;
    ylim([min_y max_y]);
    xlabel('gamma'); ylabel('Flow rate (veh/h)');
    set(gca, 'fontsize', fontsize);
%     legend(profiles);
    title(['(c) Performance of HDMPC Np3 on intersection ', int2str(i)]);
end

close all;
%------------------------
%-------- Area 1 --------
int_a2 = [4,5];
A2_Fixed = ones(1,2)*sum([Fixed(int_a2).data]);
A2_HDC = HDC(4).data + HDC(5).data;
A2_HDMPC_Np2 = HDMPC_Np2(4).data + HDMPC_Np2(5).data;
A2_HDMPC_Np3 = HDMPC_Np3(4).data + HDMPC_Np3(5).data;

max_y = max(max([ones(num_case,1)*A2_Fixed, A2_HDC, A2_HDMPC_Np2, A2_HDMPC_Np3]))*1.01;
min_y = min(min([ones(num_case,1)*A2_Fixed, A2_HDC, A2_HDMPC_Np2, A2_HDMPC_Np3]))*0.99;
    
figure;
subplot(3,1,1);
plot([0 max_gamma], A2_Fixed, '--k', 'LineWidth', 2);
plot_ex(gamma, A2_HDC); grid on;
ylim([min_y max_y]);
ylabel('Flow rate (veh/h)');
legend(profiles);
set(gca, 'fontsize', fontsize);
title('(a) Perforemance of 3rd area (HDC)');

subplot(3,1,2);
plot([0 max_gamma], A2_Fixed, '--k', 'LineWidth', 2);
plot_ex(gamma, A2_HDMPC_Np2); grid on;
ylim([min_y max_y]);
ylabel('Flow rate (veh/h)');
set(gca, 'fontsize', fontsize);
% legend(profiles);
title('(b) Perforemance of 3rd area (HDMPC Np2)');

subplot(3,1,3);
plot([0 max_gamma], A2_Fixed, '--k', 'LineWidth', 2);
plot_ex(gamma, A2_HDMPC_Np3); grid on;
ylim([min_y max_y]);
xlabel('gamma'); ylabel('Flow rate (veh/h)');
set(gca, 'fontsize', fontsize);
% legend(profiles);
title('(c) Perforemance of 3rd area (HDMPC Np3)');

%------------------------
%-------- Area 2 --------
int_a1 = [1,2,3];
A1_Fixed = ones(1,2)*sum([Fixed(int_a1).data]);
A1_HDC = HDC(1).data + HDC(2).data + HDC(3).data;
A1_HDMPC_Np2 = HDMPC_Np2(1).data + HDMPC_Np2(2).data + HDMPC_Np2(3).data;
A1_HDMPC_Np3 = HDMPC_Np3(1).data + HDMPC_Np3(2).data + HDMPC_Np3(3).data;

max_y = max(max([ones(num_case,1)*A1_Fixed, A1_HDC, A1_HDMPC_Np2, A1_HDMPC_Np3]))*1.01;
min_y = min(min([ones(num_case,1)*A1_Fixed, A1_HDC, A1_HDMPC_Np2, A1_HDMPC_Np3]))*0.99;

figure;
subplot(3,1,1);
plot([0 max_gamma], A1_Fixed, '--k', 'LineWidth',2);
plot_ex(gamma, A1_HDC); grid on;
ylim([min_y max_y]);
ylabel('Flow rate (veh/h)');
legend(profiles);
set(gca, 'fontsize', fontsize);
title('(a) Perforemance of 2nd area (HDC)');

subplot(3,1,2);
plot([0 max_gamma], A1_Fixed, '--k','LineWidth',2);
plot_ex(gamma, A1_HDMPC_Np2); grid on;
ylim([min_y max_y]);
ylabel('Flow rate (veh/h)');
set(gca, 'fontsize', fontsize);
% legend(profiles);
title('(b) Perforemance of 2nd area (HDMPC Np2)');

subplot(3,1,3);
plot([0 max_gamma], A1_Fixed, '--k','LineWidth',2);
plot_ex(gamma, A1_HDMPC_Np3); grid on;
ylim([min_y max_y]);
xlabel('gamma'); ylabel('Flow rate (veh/h)');
set(gca, 'fontsize', fontsize);
% legend(profiles);
title('(c) Perforemance of 2nd area (HDMPC Np3)');



%------------------------
%-------- Area 3 --------

int_a3 = 6:10;
A3_Fixed = ones(1,2)*sum([Fixed(int_a3).data]);
A3_HDC = HDC(6).data + HDC(7).data + HDC(8).data + HDC(9).data + HDC(10).data;
A3_HDMPC_Np2 = HDMPC_Np2(6).data + HDMPC_Np2(7).data + HDMPC_Np2(8).data + HDMPC_Np2(9).data + HDMPC_Np2(10).data;
A3_HDMPC_Np3 = HDMPC_Np3(6).data + HDMPC_Np3(7).data + HDMPC_Np3(8).data + HDMPC_Np3(9).data + HDMPC_Np3(10).data;

max_y = max(max([ones(num_case,1)*A3_Fixed, A3_HDC, A3_HDMPC_Np2, A3_HDMPC_Np3]))*1.01;
min_y = min(min([ones(num_case,1)*A3_Fixed, A3_HDC, A3_HDMPC_Np2, A3_HDMPC_Np3]))*0.99;

figure;
subplot(3,1,1);
plot([0 max_gamma], A3_Fixed, '--k', 'LineWidth', 2);
plot_ex(gamma, A3_HDC); grid on;
ylim([min_y max_y]);
ylabel('Flow rate (veh/h)');
legend(profiles);
set(gca, 'fontsize', fontsize);
title('(a) Perforemance of 1st area (HDC)');

subplot(3,1,2);
plot([0 max_gamma], A3_Fixed, '--k', 'LineWidth', 2);
plot_ex(gamma, A3_HDMPC_Np2); grid on;
ylim([min_y max_y]);
ylabel('Flow rate (veh/h)');
set(gca, 'fontsize', fontsize);
% legend(profiles);
title('(b) Perforemance of 1st area (HDMPC Np2)');

subplot(3,1,3);
plot([0 max_gamma], A3_Fixed, '--k', 'LineWidth', 2);
plot_ex(gamma, A3_HDMPC_Np3); grid on;
ylim([min_y max_y]);
xlabel('gamma'); ylabel('Flow rate (veh/h)');
set(gca, 'fontsize', fontsize);
% legend(profiles);
title('(c) Perforemance of 1st area (HDMPC Np3)');

%------------------------
%-------- Network --------

N_Fixed = A1_Fixed + A2_Fixed + A3_Fixed;
N_HDC = A1_HDC + A2_HDC + A3_HDC;
N_HDMPC_Np2 = A1_HDMPC_Np2 + A2_HDMPC_Np2 + A3_HDMPC_Np2;
N_HDMPC_Np3 = A1_HDMPC_Np3 + A2_HDMPC_Np3 + A3_HDMPC_Np3;

max_y = max(max([N_HDC, N_HDMPC_Np2, N_HDMPC_Np3]))*1.01;
min_y = min(min([N_HDC, N_HDMPC_Np2, N_HDMPC_Np3]))*0.99;

figure;
subplot(3,1,1);
plot([0 max_gamma], N_Fixed, '--k', 'LineWidth', 2);
plot_ex(gamma, N_HDC); grid on;
ylim([min_y max_y]);
ylabel('Flow rate (veh/h)');
legend(profiles);
set(gca, 'fontsize', fontsize);
title('(a) Perforemance of network (HDC)');

subplot(3,1,2);
plot([0 max_gamma], N_Fixed, '--k', 'LineWidth', 2);
plot_ex(gamma, N_HDMPC_Np2); grid on;
ylim([min_y max_y]);
ylabel('Flow rate (veh/h)');
set(gca, 'fontsize', fontsize);
% legend(profiles);
title('(b) Perforemance of network (HDCMPC Np2)');

subplot(3,1,3);
plot([0 max_gamma], N_Fixed, '--k', 'LineWidth', 2);
plot_ex(gamma, N_HDMPC_Np3); grid on;
ylim([min_y max_y]);
xlabel('gamma'); ylabel('Flow rate (veh/h)');
set(gca, 'fontsize', fontsize);
% legend(profiles);
title('(c) Perforemance of network (HDCMPC Np3)');



% max_y = max(max([RMSE_HDC, RMSE_HDMPC_Np2, RMSE_HDMPC_Np3]))*1.01;
% min_y = min(min([RMSE_HDC, RMSE_HDMPC_Np2, RMSE_HDMPC_Np3]))*0.99;
% figure;
% subplot(3,1,1);
% plot_ex(gamma, RMSE_HDC); grid on;
% ylim([min_y max_y]);
% ylabel('RMSE (veh/h)');
% legend(profiles{2:end});
% set(gca, 'fontsize', fontsize);
% title('(a) Total average RMSE of flow reference (HDC)');
% 
% subplot(3,1,2);
% plot_ex(gamma, RMSE_HDMPC_Np2); grid on;
% ylim([min_y max_y]);
% ylabel('RMSE (veh/h)');
% set(gca, 'fontsize', fontsize);
% % legend(profiles);
% title('(b) Total average RMSE of flow reference (HDCMPC Np2)');
% 
% subplot(3,1,3);
% plot_ex(gamma, RMSE_HDMPC_Np3); grid on;
% ylim([min_y max_y]);
% xlabel('gamma'); ylabel('RMSE (veh/h)');
% set(gca, 'fontsize', fontsize);
% % legend(profiles);
% title('(c) Total average RMSE of flow reference (HDCMPC Np3)');
% 
