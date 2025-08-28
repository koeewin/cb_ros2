clear; clc; close all;

%% Setup
F_s = 50; % sample frequency [Hz]
T_s = 10; % sampling time [s]
marker_amount = 20;
Phi_u = [];
D_u = [];
X_u = [];
Y_u = [];
Phi_v = [];
D_v = [];
X_v = [];
Y_v = [];
Phi_f = [];
D_f = [];
X_f = [];
Y_f = [];

% Messungen mit neuen Parametern
filename = 'measurements/Messung_Positioning_20230825.txt';

%% Define target values 
% For 20 markers
X_target = [repmat(0, 1, 4), repmat(-0.5, 1, 4), repmat(-1, 1, 4), repmat(0.5, 1, 4), repmat(1, 1, 4)];
Y_target = [repmat([0.5, 1, 1.5, 2], 1, 5)];
Phi_target = atan2(-X_target, Y_target);
d_target = sqrt(X_target.^2 + Y_target.^2);

%% Extract measurement data from log files
samples_per_marker = zeros(1, marker_amount);
marker = 1;
marker_counter = 0;
file = fopen(filename);
while ~feof(file)
    line = fgetl(file);
    values = str2double(strsplit(line));
    if isnan(values(1)) % Skip lines that mark end of measurement
        samples_per_marker(marker) = marker_counter;
        marker = marker + 1;
        marker_counter = 0;
        continue
    end
    marker_counter = marker_counter + 1;

    Phi_u(end+1) = values(1);
    D_u(end+1) = values(2);
    X_u(end+1) = values(3);
    Y_u(end+1) = values(4);
    Phi_v(end+1) = values(5);
    D_v(end+1) = values(6);
    X_v(end+1) = values(7);
    Y_v(end+1) = values(8);
    Phi_f(end+1) = values(9);
    D_f(end+1) = values(10);
    X_f(end+1) = values(11);
    Y_f(end+1) = values(12);
end
fclose(file);

% Test data
assert(all(samples_per_marker == F_s * T_s), 'Amount of samples per marker does not match selected sampling time and frequency.')
samples_per_marker_global = F_s * T_s;

%% Visualize measurement data
%% Plot deviation from target points in 2d
% Plot marker grid
col = turbo(marker_amount*3);

%% UWB
ri = 1; % rolling index
f_uwb = figure();
hold on
for i = 1:marker_amount
    plot(X_target(i), Y_target(i), 'LineStyle', 'none', 'Marker', 'o', 'MarkerFaceColor', col(i*3,:), 'MarkerEdgeColor', '#5A5A5A')
    % plot(X((i-1)*samples_per_marker(1)+1:(i-1)*samples_per_marker(1)+samples_per_marker(1)), Y((i-1)*samples_per_marker(1)+1:(i-1)*samples_per_marker(1)+samples_per_marker(1)), 'LineStyle', 'none', 'Color', col(i*3,:), 'Marker', '*')
    plot(X_u(ri:ri+samples_per_marker(i)-1), Y_u(ri:ri+samples_per_marker(i)-1), 'LineStyle', 'none', 'Color', col(i*3,:), 'Marker', '*', 'MarkerSize', 10)
    % polarplot(Phi_target(i), d_target(i), 'LineStyle', 'none', 'Marker', 'o', 'MarkerFaceColor', col(i*3,:), 'MarkerEdgeColor', '#5A5A5A')
    % hold on
    % polarplot((Phi_u(ri:ri+samples_per_marker(i)-1)), D_u(ri:ri+samples_per_marker(i)-1), 'LineStyle', 'none', 'Color', col(i*3,:), 'Marker', '*')
    ri = ri + samples_per_marker(i);
end
ax = gca;
ax.FontName = 'Times New Roman';
ax.FontSize = 14;
axis equal
xlim([-1.7 1.7])
ylim([-0.1 2.5])
xlabel("$x_r$ [m]", 'Interpreter','latex', 'FontSize', 16, 'FontName', 'Times New Roman')
ylabel("$y_r$ [m]", 'Interpreter','latex', 'FontSize', 16, 'FontName', 'Times New Roman')
grid on

pbaspect([1 1 1])
f_uwb.Position = [0 0 700 700];
% exportgraphics(f_uwb, 'exp1_uwb.pdf', 'ContentType', 'vector');
% exportgraphics(f_uwb, 'exp1_uwb.png');

% % [t, s] = title('Grid Measurements for UWB Positioning', 'n = 500 samples per marker');
% % t.FontSize = 14;
% % s.FontAngle = 'italic';
% ax = gca;
% ax.ThetaLim = [-90 90];
% ax.ThetaDir = 'counterclockwise';
% ax.ThetaZeroLocation = 'top';

%% Vision
ri = 1; % rolling index
f_vis = figure();
hold on
for i = 1:marker_amount
    plot(X_target(i), Y_target(i), 'LineStyle', 'none', 'Marker', 'o', 'MarkerFaceColor', col(i*3,:), 'MarkerEdgeColor', '#5A5A5A')
    % plot(X((i-1)*samples_per_marker(1)+1:(i-1)*samples_per_marker(1)+samples_per_marker(1)), Y((i-1)*samples_per_marker(1)+1:(i-1)*samples_per_marker(1)+samples_per_marker(1)), 'LineStyle', 'none', 'Color', col(i*3,:), 'Marker', '*')
    plot(X_v(ri:ri+samples_per_marker(i)-1), Y_v(ri:ri+samples_per_marker(i)-1), 'LineStyle', 'none', 'Color', col(i*3,:), 'Marker', '*', 'MarkerSize', 10)
    % polarplot(Phi_target(i), d_target(i), 'LineStyle', 'none', 'Marker', 'o', 'MarkerFaceColor', col(i*3,:), 'MarkerEdgeColor', '#5A5A5A')
    % hold on
    % polarplot((Phi_v(ri:ri+samples_per_marker(i)-1)), D_v(ri:ri+samples_per_marker(i)-1), 'LineStyle', 'none', 'Color', col(i*3,:), 'Marker', '*')
    ri = ri + samples_per_marker(i);
end

ax = gca;
ax.FontName = 'Times New Roman';
ax.FontSize = 14;
axis equal
xlim([-1.7 1.7])
ylim([-0.1 2.5])
xlabel("$x_r$ [m]", 'Interpreter','latex', 'FontSize', 16, 'FontName', 'Times New Roman')
ylabel("$y_r$ [m]", 'Interpreter','latex', 'FontSize', 16, 'FontName', 'Times New Roman')
grid on

pbaspect([1 1 1])
f_vis.Position = [0 0 700 700];
% exportgraphics(f_vis, 'exp1_vis.pdf', 'ContentType', 'vector');
% exportgraphics(f_vis, 'exp1_vis.png');

% title('Grid Measurements for Visual Positioning (n=500 samples per marker)')
% ax = gca;
% ax.ThetaLim = [-90 90];
% ax.ThetaDir = 'counterclockwise';
% ax.ThetaZeroLocation = 'top';

%% Fused
ri = 1; % rolling index
f_fus = figure();
hold on
for i = 1:marker_amount
    plot(X_target(i), Y_target(i), 'LineStyle', 'none', 'Marker', 'o', 'MarkerFaceColor', col(i*3,:), 'MarkerEdgeColor', '#5A5A5A')
    % plot(X((i-1)*samples_per_marker(1)+1:(i-1)*samples_per_marker(1)+samples_per_marker(1)), Y((i-1)*samples_per_marker(1)+1:(i-1)*samples_per_marker(1)+samples_per_marker(1)), 'LineStyle', 'none', 'Color', col(i*3,:), 'Marker', '*')
    plot(X_f(ri:ri+samples_per_marker(i)-1), Y_f(ri:ri+samples_per_marker(i)-1), 'LineStyle', 'none', 'Color', col(i*3,:), 'Marker', '*', 'MarkerSize', 10)
    % polarplot(Phi_target(i), d_target(i), 'LineStyle', 'none', 'Marker', 'o', 'MarkerFaceColor', col(i*3,:), 'MarkerEdgeColor', '#5A5A5A')
    % hold on
    % polarplot((Phi_f(ri:ri+samples_per_marker(i)-1)), D_f(ri:ri+samples_per_marker(i)-1), 'LineStyle', 'none', 'Color', col(i*3,:), 'Marker', '*')
    ri = ri + samples_per_marker(i);
end
ax = gca;
ax.FontName = 'Times New Roman';
ax.FontSize = 14;
axis equal
xlim([-1.7 1.7])
ylim([-0.1 2.5])
xlabel("$x_r$ [m]", 'Interpreter','latex', 'FontSize', 16, 'FontName', 'Times New Roman')
ylabel("$y_r$ [m]", 'Interpreter','latex', 'FontSize', 16, 'FontName', 'Times New Roman')
grid on

pbaspect([1 1 1])
f_fus.Position = [0 0 700 700];
% exportgraphics(f_fus, 'exp1_fus.pdf', 'ContentType', 'vector');
% exportgraphics(f_fus, 'exp1_fus.png');

% ax = gca;
% ax.ThetaLim = [-90 90];
% ax.ThetaDir = 'counterclockwise';
% ax.ThetaZeroLocation = 'top';