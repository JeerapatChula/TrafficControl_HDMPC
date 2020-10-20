clear; close all;
fontsize= 16;

HDC = [1.53, 1.59, 1.98];
HDMPC2 = [1.44, 1.24, 1.71];
HDMPC3 = [1.04, 1.05, 1.74];

figure;
data = [HDC' HDMPC2' HDMPC3'];
hb = bar([1 2 3], data);
grid on;
legend('HDMPC Np=1', 'HDMPC Np=2', 'HDMPC Np=3');
set(gca, 'fontsize', fontsize);
ylabel('Improvement (%)');
xlabel('Case');