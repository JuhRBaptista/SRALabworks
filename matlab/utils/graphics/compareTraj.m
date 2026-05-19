clear; clc; close all;

%% ===== Load data =====
corr = load('../data/corrected_trajs.mat');
gt1 = corr.gtTraj;
est1 = corr.traj;

noCorr = load('../data/noCorrected_trajs.mat');
gt2 = noCorr.gtTraj;
est2 = noCorr.traj;

%% ===== Error computation =====
err1 = est1(:,1:2) - gt1(:,1:2);
err2 = est2(:,1:2) - gt2(:,1:2);

en1 = sqrt(err1(:,1).^2 + err1(:,2).^2);
en2 = sqrt(err2(:,1).^2 + err2(:,2).^2);

%% ===== Metrics =====
ATE1 = mean(en1);
RMSE1 = sqrt(mean(en1.^2));

ATE2 = mean(en2);
RMSE2 = sqrt(mean(en2.^2));

%% ===== Print metrics =====
fprintf('===== METRICS =====\n');

fprintf('Com correção:\n');
fprintf('ATE  = %.4f\n', ATE1);
fprintf('RMSE = %.4f\n\n', RMSE1);

fprintf('Sem correção:\n');
fprintf('ATE  = %.4f\n', ATE2);
fprintf('RMSE = %.4f\n\n', RMSE2);

%% ===== Time vectors =====
t1 = 1:length(en1);
t2 = 1:length(en2);

%% ===== Plot =====
figure;
tiledlayout(2,1);

% ===== WITH correction =====
nexttile;
plot(t1, en1, 'r', 'LineWidth', 1.5);
grid on;
title(sprintf('Com correção | ATE=%.3f | RMSE=%.3f', ATE1, RMSE1));
xlabel('Tempo (steps)');
ylabel('||erro||');

% ===== WITHOUT correction =====
nexttile;
plot(t2, en2, 'b', 'LineWidth', 1.5);
grid on;
title(sprintf('Sem correção | ATE=%.3f | RMSE=%.3f', ATE2, RMSE2));
xlabel('Tempo (steps)');
ylabel('||erro||');