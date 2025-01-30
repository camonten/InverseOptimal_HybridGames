%--------------------------------------------------------------------------
% Matlab M-file Project: Inverse-Optimal Safety Control for Hybrid Systems @  Hybrid Systems Laboratory (HSL),
% Filename: CostEvaluation.m
%--------------------------------------------------------------------------
% Project: Example - Bouncing Ball with Disturbances as a
%           Two-Player Zero-Sum Hybrid Game - Saddle Point Behavior
% Author(s): Carlos Montenegro and Santiago Jimenez Leudo
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
%   Make sure to install HyEQ Toolbox v3.0.0.76 from
%   https://www.mathworks.com/matlabcentral/fileexchange/41372-hybrid-equations-toolbox
%   (View Version History)
%   Copyright @ Hybrid Systems Laboratory (HSL),
%   Revision: 0.0.0.3 Date: 01/29/2025

clear all; clc;
rng("default");

% Class-K functions
rho = @(r) r^3;
inv_rho = @(r) r^(1/3);
gamma = @(r) r^2;
dgamma = @(r) 2*r;
inv_dgamma = @(r) 0.5*r;

% Define CBF object with parameters: a, b, alphaC, alphaD, rho(\bar w)
B = ControlBarrierFunction(1, 2, 1, 0.5, rho(1)); 

% Define bouncing ball hybrid system with parameters: 
%                      B, rho, inv_rho, gamma, inv_dgamma, actFilt, actDist
bb = BouncingBall(B, rho, inv_rho, gamma, inv_dgamma, ...
                                            true, true);

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