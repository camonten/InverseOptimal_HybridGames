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

% Define combinations of actFilt and actDist
combs = {
    struct('actFilt', false, 'actDist', false),...
    struct('actFilt', true, 'actDist', false),...
    struct('actFilt', false, 'actDist', true),...
    struct('actFilt', true, 'actDist', true)
};

% Preallocate an array to store solutions
sols = cell(1, length(combs)); 

x0 = [B.a, 0, 0]';   % Included the extra 0 for cost evaluation
tspan = [0, 50];
jspan = [0, 30];
config = HybridSolverConfig('refine', 100); 

% Iterate through combinations, create bb objects, and solve
for i = 1:length(combs)
    bb = BouncingBall(B, rho, inv_rho, gamma, inv_dgamma,...
                         combs{i}.actFilt, combs{i}.actDist); 
    sols{i} = bb.solve(x0, tspan, jspan, config);
end

%% Run the following cell to reproduce Fig. 1

close all
set(0,'defaulttextinterpreter','latex')

% Determine the grid dimensions based on the number of solutions
nSols = length(sols);
nCols = ceil(sqrt(nSols)); % Number of columns
nRows = ceil(nSols / nCols); % Number of rows

% Create the figure and axes for the grid plot
figure;
labels = {'a)', 'b)', 'c)', 'd)'}; % Define the labels

for i = 1:nSols
    % Create subplot for the current solution
    subplot(nRows, nCols, i); 
    hold on

    % Initialize HybridPlotBuilder for the current subplot
    hpb = HybridPlotBuilder()...
            .select(1:2)...
            .flowColor(1/255*[0, 0, 205])...
            .jumpColor(1/255*[178, 34, 34])...
            .flowLineWidth(3)...
            .tickLabelSize(10);

    colormap abyss
    B0 = B.B(x0(1:2)');
    % Draw a square at the initial condition
    plot(x0(1), x0(2), 'sblack')
    text(x0(1), x0(2), "$\>\>\> \xi$", 'FontSize', 14)

    % Plot the phase for the current solution
    hpb.plotPhase(sols{i}); % This line is modified

    B.plot()
    hold off
    xlabel("$x_1$", 'FontSize', 16)
    ylabel("$x_2$", 'FontSize', 16, 'Rotation', 0)

    % Add the label to the current subplot
    text(0.8, 0.9, labels{i}, 'Units', 'normalized', 'FontSize', 13,...
         'HorizontalAlignment', 'center'); 
    
    % Change the size of tick labels
    ax = gca;
    set(ax, 'FontSize', 12); % Sets both tick and label font sizes
    set([ax.XLabel, ax.YLabel], 'FontSize', 16); % Restore only label font size

    % Artificially create legend (only for the first subplot)
    if i == 2
        patch1 = patch([0 0 0 0], [0 0 0 0],...
                        1/255*[169, 169, 169],...
                        'FaceAlpha', 0.15,...
                        'EdgeColor', 1/255*[103, 49, 71], ...
                        'DisplayName', '$K$'); % K
        patch2 = patch([0 0 0 0], [0 0 0 0],...
                        1/255*[218, 165, 32],...
                        'FaceAlpha', 0.15,...
                        'EdgeColor', 1/255*[218, 165, 32], ...
                        'DisplayName', '$K_d(\bar{w})$'); % Kd(\bar w)
        hpb.addLegendEntry(patch1, '$K$');
        hpb.addLegendEntry(patch2, '$K_d(\bar{u})$');

        % Set legend properties
        set(legend, ...
            'Position', [0.662, 0.838, 0.128, 0.0771], ...
            'FontSize', 12 ...
        );
    end
end

% Apply styling
sdf(1,'PowerPoint')

% Set figure size
set(gcf, 'Position', [106, 170, 682, 587]);