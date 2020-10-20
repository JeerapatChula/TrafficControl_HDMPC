close all;
x0      = 100; y0       = 100;
width   = 800; height   = 600;
fontsize= 16;

dt = 10*60;
t = 0:dt:(3600*3);
links = {'L1', 'L2', 'L3', 'L4'};
nodes = {'A1', 'A2', 'A3'};


load('logs/controllers/network_layer.mat');
figure;
subplot(2,1,1);
plot_ex(t, state_log');
legend(nodes); grid on;
ylabel('State (veh)');
%title('(a) Demand of each area');
set(gca, 'fontsize', fontsize);

subplot(2,1,2);
plot_ex(t, 3600*control_signal_log'/dt);
legend(links); grid on;
ylabel('Control signal (veh/h)');
xlabel('Time (s)');
%title('(b) Control signal');
set(gca, 'fontsize', fontsize);
