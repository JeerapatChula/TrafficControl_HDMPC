clear all;
tmp = {'R15', 'R16', 'R27', 'R28', 'R29', 'R3', 'R30', 'R31', 'R32', 'R33', 'R34', 'R35', 'R36', 'R37', 'R38', 'R39', 'R4', 'R40', 'R41', 'R42', 'R43', 'R44', 'R45', 'R46', 'R47', 'R1', 'R10', 'R11', 'R12', 'R13', 'R14', 'R15', 'R16', 'R2', 'R3', 'R4', 'R48', 'R49', 'R5', 'R6', 'R7', 'R8', 'R9', 'R11', 'R12', 'R17', 'R18', 'R19', 'R20', 'R21', 'R22', 'R23', 'R24', 'R25', 'R26', 'R5', 'R6'};

load('A1.mat');
level_1 = mean(control_signal_log, 1);
load('A2.mat');
level_2 = mean(control_signal_log, 1);
load('A3.mat');
level_3 = mean(control_signal_log, 1);

level = [level_1 level_2 level_3];
level = level/max(level);

% cm = colormap; % returns the current color map
cm = autumn(100);

for k=1:length(level)
    colorID = max(1, sum(1-level(k) > [0:1/length(cm(:,1)):1]));
    results(k).road = tmp(k); 
    results(k).level = level(k);
    results(k).colorID = colorID;
    results(k).hex = rgb2hex(cm(colorID, :));
end

