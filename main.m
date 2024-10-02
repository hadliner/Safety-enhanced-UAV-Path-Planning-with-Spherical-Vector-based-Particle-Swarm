clc;
clear;
close all;

%% Problem Definition

model = CreateModel(); % Create search map and parameters

% Run SPSO Algorithm
[BestPosition, BestCost] = SPSO(model);

%% Plot results
% Best solution
disp("Best solution...");
disp(BestPosition);
smooth = 0.95;
PlotSolution(BestPosition, model, smooth);

% Best cost  
figure;
plot(BestCost,'LineWidth',2);
xlabel('Iteration');
ylabel('Best Cost');
grid on;