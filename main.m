%--------------------------------------------------------------------------
% Matlab M-file Project: Inverse-Optimal Safety Control for Hybrid Systems @  Hybrid Systems Laboratory (HSL),
% Filename: main.m
%--------------------------------------------------------------------------
% Project: Example - Bouncing Ball with Disturbances as a
%           Two-Player Zero-Sum Hybrid Game - Saddle Point Behavior
% Author(s): Carlos Montenegro and Santiago Jimenez Leudo
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
%   Make sure to install HyEQ Toolbox (Beta) v3.0.0.22 from
%   https://www.mathworks.com/matlabcentral/fileexchange/102239-hybrid-equations-toolbox-beta
%   (View Version History)
%   Copyright @ Hybrid Systems Laboratory (HSL),
%   Revision: 0.0.0.2 Date: 01/24/2022 9:45:00

%% Main
clear all; clc;
rng("default");

% Class-K functions
rho = @(r) r^3;
inv_rho = @(r) r^(1/3);
gamma = @(r) r^2;
dgamma = @(r) 2*r;
inv_dgamma = @(r) 0.5*r;

% Define CBF object with parameters: a, b, alphaC, alphaD, rho(\bar w)
B = BBControlBarrierFunction(1, 2, 1, 0.5, rho(1)); 

% Define bouncing ball hybrid system with parameters: B, rho, inv_rho, gamma, inv_dgamma 
bb = BouncingBall(B, rho, inv_rho, gamma, inv_dgamma);

sys = bb;

x0 = [B.a, 0, 0]';   % Included the extra 0 for cost evaluation
% x0 = [B.a, 0]';
tspan = [0, 50];
jspan = [0, 30];
config = HybridSolverConfig('refine', 100); % Improves plot smoothness for demo.
sol = sys.solve(x0, tspan, jspan, config);

% Include terminal cost
xf = sol.x(end, 1:end);
q = B.B(xf(1:2)');

% Create a new hybrid arc
sol = HybridArc(sol.t, sol.j, [sol.x(:,1:2), [sol.x(1:end-1, 3); sol.x(end) + q]]);

%% Plots
close all;

% Get maximum and minimum state values for plotting
max_x = max(sol.x);
min_x = min(sol.x);
delta = 1.2;

% Plot the hybrid arc with no filter
ax1 = subplot(1, 2, 1);
hold on;
% Plot the safe set
B.plot()
% Plot hybrid arc
pb = HybridPlotBuilder()...
        .flowColor(1/255*[0, 0, 205])...
        .jumpColor(1/255*[178, 34, 34])...
        .flowLineWidth(3)...
        .plotPhase(sol);

xlim([-0.15 max_x(1, 1)*delta])
ylim([min_x(1, 2) max_x(1, 2)])

% Set font size for tick labels
ax1.FontSize = 12;
xlabel('$x_1$','interpreter','latex', 'FontSize', 16)
ylabel('$x_2$','interpreter','latex', 'FontSize', 16)

% Plot the hybrid arc with filter
ax2 = subplot(1, 2, 2);
hold on;
% Plot the safe set
B.plot()
% Plot hybrid arc
pb_filter = HybridPlotBuilder()...
            .flowColor(1/255*[0, 0, 205])...
            .jumpColor(1/255*[178, 34, 34])...
            .flowLineWidth(3)...
            .legend({'$\phi(t, j)$'}, 'FontSize', 12, 'NumColumns', 2, 'box', 'off')...
            .plotPhase(sol_filter);

xlim([-0.15 max_x(1, 1)*delta])
ylim([min_x(1, 2) max_x(1, 2)])

% Link the y-axes of the subplots
linkaxes([ax1, ax2], 'y');

% Set font size for tick labels
ax2.FontSize = 14;
xlabel('$x_1$','interpreter','latex', 'FontSize', 18)
ylabel('$x_2$','interpreter','latex', 'FontSize', 18)


% Artifically create legend
patch1 = patch([0 0 0 0], [0 0 0 0], ...
                1/255*[169, 169, 169], ...
                'FaceAlpha', 0.3, ...
                'EdgeColor', 1/255*[169, 169, 169] ...
            ); % Safe set
patch2 = patch([0 0 0 0], [0 0 0 0], ...
                1/255*[218, 165, 32], ...
                'FaceAlpha', 0.15, ...
                'EdgeColor', 1/255*[218, 165, 32] ...
            ); % Neighborhood of boundary of K

pb.addLegendEntry(patch1, '$K$');
pb.addLegendEntry(patch2, '$\mathcal{V}$');

sdf(1,'PowerPoint')
% exportgraphics(gcf, 'QP_filter_disturbances.pdf', 'Resolution', 1000)

%% Plots
close all;

set(0,'defaulttextinterpreter','latex')

hpb = HybridPlotBuilder()...
        .slice(1:2)...
        .flowColor(1/255*[0, 0, 205])...
        .jumpColor(1/255*[178, 34, 34])...
        .flowLineWidth(3)...
        .tickLabelSize(10);

hold on

grid on
colormap abyss

% Draw a square at the initial condition
plot(x0(1), x0(2), 'sblack')
text(x0(1), x0(2), "$\>\>\> \xi$", 'FontSize', 14)

% Plot the safe set
B.plot()

hpb.plotPhase(sol)

xlabel("$x_1$", 'FontSize', 14)
ylabel("$x_2$", 'FontSize', 14)

sdf(1,'PowerPoint')

hold off

% exportgraphics(gcf, 'QP_filter_disturbances.pdf', 'Resolution', 1000)

%% Run the following cell to reproduce Fig. 2

close all
set(0, 'defaulttextinterpreter', 'latex')

% Initiate hybrid plot builder
hpb = HybridPlotBuilder()...
    .flowColor([0, 0, 205] / 255)...
    .jumpColor([178, 34, 34] / 255)...
    .flowLineWidth(3)...
    .tickLabelSize(19.6); % Unified tick label size

grid on
colormap abyss

% Generate grid
[X, Y] = meshgrid( ...
    linspace(0, 1.5, 700), ...
    linspace(-5, 5, 700) ...
);

% Compute level set
B0 = B.B(x0(1:2)');
Bgrid = (X / B.a).^2 + (Y / B.b).^2 + ...
        X .* Y / (B.a * B.b) - 1;

hold on

% Draw initial condition
plot(x0(1), x0(2), 'sblack')
text(x0(1), x0(2), "$\>\>\> \xi$", 'FontSize', 19.6)

% Plot surface and contour
surf(X, Y, Bgrid, ...
    'FaceAlpha', 0.3, 'EdgeColor', 'none')

contour3(X, Y, Bgrid, [B0 B0], 'LineWidth', 2)

% Plot phase and boundary
hpb.plotPhase(sol)
B.plot()

hold off

% Configure axes
ax = gca;
set(ax, 'YDir', 'reverse', 'FontSize', 19.6) % Set tick font size

xlabel("$x_1$", 'FontSize', 27.44)
ylabel("$x_2$", 'FontSize', 27.44)
zlabel("$\mathcal{J}$", 'FontSize', 27.44, ...
    'VerticalAlignment', 'top')

% Adjust Z-axis rotation
ax.ZLabel.Rotation = 0;

% Set axis limits
zlim(1.1 * [min(sol.x(:, 3)), max(sol.x(:, 3))])

% Artificially create legend patches
patchProps = {'FaceAlpha', 0.15, 'EdgeColor'};

patch1 = patch( ...
    [0 0 0 0], [0 0 0 0], ...
    [169, 169, 169] / 255, ...
    patchProps{:}, [103, 49, 71] / 255 ...
);

patch2 = patch( ...
    [0 0 0 0], [0 0 0 0], ...
    [218, 165, 32] / 255, ...
    patchProps{:}, [218, 165, 32] / 255 ...
);

hpb.addLegendEntry(patch1, '$K$');
hpb.addLegendEntry(patch2, '$K_d(\bar{w})$');

% Apply styling
sdf(1, 'PowerPoint')

% Set camera view
view(-110.6133, 17.2998)

% Set figure size
set(gcf, 'Position', [148, 210, 484, 484]);

% Set label positions
set( ...
    [ax.XLabel, ax.YLabel, ax.ZLabel], {'Position'}, ...
    { ...
        [0.9902, -6.189, -0.203]; ...
        [-0.0454, 1.0348, -0.151]; ...
        [0.384, -8.882, 1.537] ...
    } ...
);

% Change only the tick label font size without affecting axis labels
set(ax, 'FontSize', 19.6); % Sets both tick and label font sizes
set([ax.XLabel, ax.YLabel, ax.ZLabel], 'FontSize', 27.44); % Restore only label font size

% Set legend properties
set(legend, ...
    'Position', [0.718, 0.826, 0.198, 0.113], ...
    'FontSize', 19.6 ...
);
%% Phase Portrait

close all

set(0,'defaulttextinterpreter','latex')

hpb = HybridPlotBuilder()...
        .select(1:2)...
        .flowColor(1/255*[0, 0, 205])...
        .jumpColor(1/255*[178, 34, 34])...
        .flowLineWidth(3)...
        .tickLabelSize(10);

hold on
colormap abyss

B0 = B.B(x0(1:2)');

% Draw a square at the initial condition
plot(x0(1), x0(2), 'sblack')
text(x0(1), x0(2), "$\>\>\> \xi$", 'FontSize', 14)

hpb.plotPhase(sol)
B.plot()

hold off

xlabel("$x_1$", 'FontSize', 14)
ylabel("$x_2$", 'FontSize', 14)

% Artificially create legend
patch1 = patch([0 0 0 0], [0 0 0 0], ...
                1/255*[169, 169, 169], ...
                'FaceAlpha', 0.15, ...
                'EdgeColor', 1/255*[103, 49, 71] ...
            ); % K
patch2 = patch([0 0 0 0], [0 0 0 0], ...
                1/255*[218, 165, 32], ...
                'FaceAlpha', 0.15, ...
                'EdgeColor', 1/255*[218, 165, 32] ...
            ); % Kd(\bar u)

hpb.addLegendEntry(patch1, '$K$');
hpb.addLegendEntry(patch2, '$K_d(\bar{u})$');

sdf(1,'PowerPoint')


%% 

set(0,'defaulttextinterpreter','latex')

% Open each figure and store the axes explicitly
h1 = openfig('Figures/PP_NoD_NoF.fig', 'reuse');
ax1 = findobj(h1, 'Type', 'Axes'); % Get handle to the axes of the figure

h2 = openfig('Figures/PP_NoD_F.fig', 'reuse');
ax2 = findobj(h2, 'Type', 'Axes');

h3 = openfig('Figures/PP_D_NoF.fig', 'reuse');
ax3 = findobj(h3, 'Type', 'Axes');

h4 = openfig('Figures/PP_all.fig', 'reuse');
ax4 = findobj(h4, 'Type', 'Axes');

%%
figure;

% First subplot
ax_new1 = subplot(2, 2, 1); % Create the first subplot
copyobj(get(ax1, 'Children'), ax_new1); % Copy all children objects from ax1 to the new axes
text(0.8, 0.9, 'a)', 'Units', 'normalized', 'FontSize', 12, 'HorizontalAlignment', 'center');
xlabel("$x_1$", 'FontSize', 14)
ylabel("$x_2$", 'FontSize', 14)
set(ax_new1, 'TickLabelInterpreter', 'latex'); % Set LaTeX font for tick labels
set(get(gca,'ylabel'),'rotation',0)

% Second subplot
ax_new2 = subplot(2, 2, 2); % Create the second subplot
copyobj(get(ax2, 'Children'), ax_new2); % Copy all children objects from ax2 to the new axes
text(0.8, 0.9, 'b)', 'Units', 'normalized', 'FontSize', 12, 'HorizontalAlignment', 'center');
xlabel("$x_1$", 'FontSize', 14)
ylabel("$x_2$", 'FontSize', 14)
set(ax_new2, 'TickLabelInterpreter', 'latex'); % Set LaTeX font for tick labels
set(get(gca,'ylabel'),'rotation',0)

% Add the patches (with visible vertices for legend purposes)
patch1 = patch([0 0 0 0], [0 0 1 1], ...
                1/255*[169, 169, 169], ...
                'FaceAlpha', 0.15, ...
                'EdgeColor', 1/255*[103, 49, 71], ...
                'DisplayName', '$K$'); % Name for legend

patch2 = patch([0 0 0 0], [0 0 1 1], ...
                1/255*[218, 165, 32], ...
                'FaceAlpha', 0.15, ...
                'EdgeColor', 1/255*[218, 165, 32], ...
                'DisplayName', '$K_d(\bar{u})$'); % Name for legend
% Create the legend
legend([patch1, patch2], 'Location', 'southeast', ...
                                    'Interpreter', 'latex', ...
                                    'FontSize', 12);

% Third subplot
ax_new3 = subplot(2, 2, 3); % Create the third subplot
copyobj(get(ax3, 'Children'), ax_new3); % Copy all children objects from ax3 to the new axes
text(0.8, 0.9, 'c)', 'Units', 'normalized', 'FontSize', 12, 'HorizontalAlignment', 'center');
xlabel("$x_1$", 'FontSize', 14)
ylabel("$x_2$", 'FontSize', 14); ylim([-4 4])
set(ax_new3, 'TickLabelInterpreter', 'latex'); % Set LaTeX font for tick labels
set(get(gca,'ylabel'),'rotation',0)

% Fourth subplot
ax_new4 = subplot(2, 2, 4); % Create the fourth subplot
copyobj(get(ax4, 'Children'), ax_new4); % Copy all children objects from ax4 to the new axes
text(0.8, 0.9, 'd)', 'Units', 'normalized', 'FontSize', 12, 'HorizontalAlignment', 'center');
xlabel("$x_1$", 'FontSize', 14)
ylabel("$x_2$", 'FontSize', 14)
set(ax_new4, 'TickLabelInterpreter', 'latex'); % Set LaTeX font for tick labels
set(get(gca,'ylabel'),'rotation',0)

sdf(5,'PowerPoint')