clear
close all
clc
%% Sensors data
load("pt3_data_fixed.mat");
dataset = data1;
imu_f = dataset.imu_f.data;
imu_w = dataset.imu_w.data;
t_imu = dataset.imu_f.t;
gnss_data = dataset.gnss.data;
gnss_t = dataset.gnss.t;
num_step = size(imu_f , 1);
gt_p = dataset.gt.p;
gt_t = dataset.gt.t;
%% Intialization primary parameters
p = [0 ; 0 ; 0];
v = [0 ; 0 ; 0];
q = [1 ; 0 ; 0 ; 0];
P = eye(9) * 0.1;
var_imu_f = 0.01;
var_imu_w = 0.01;
g = [0; 0; -9.81];
R_gnss = diag([2, 2, 5]);
est_postation = zeros(num_step , 3);
gnss_index = 1;
%% EKF Loop
for k = 1 : (num_step-1)
  dt = t_imu(k + 1) - t_imu(k);
  [p , v , q , P] = es_ekf.perdict(p , v , q , P, imu_f(k , :)', imu_w(k , :)', var_imu_f, var_imu_w, g, dt);
  if ((gnss_index < length(gnss_t)) && ((abs(t_imu(k) - gnss_t(gnss_index))) < 1e-4))
      [p , v , q , P] = es_ekf.update(p , v , q , P , gnss_data(gnss_index , :)' , R_gnss);
      gnss_index = gnss_index + 1;
  end
  est_postation(k , :) = p';
end
%% Align ground truth to IMU timestamps
gt_t_matched = gt_t(1:size(gt_p,1));
gt_p_aligned = dataset.gt.p;
%% توحيد الأطوال
n = min(size(est_postation,1), size(gt_p_aligned,1));
est_postation = est_postation(1:n, :);
gt_p_aligned  = gt_p_aligned(1:n, :);
%% 3D Trajectory Plot
figure('Name', 'Trajectory Comparison', 'Color', 'white');
plot3(est_postation(:,1), est_postation(:,2), est_postation(:,3), ...
      'b', 'LineWidth', 1.2, 'DisplayName', 'Estimated');
hold on
plot3(gt_p_aligned(:,1), gt_p_aligned(:,2), gt_p_aligned(:,3), ...
      'Color', [0.85 0.33 0.10], 'LineWidth', 1.5, 'DisplayName', 'Ground Truth');

% علامة نقطة البداية (Start) - دايرة خضرا
plot3(gt_p_aligned(1,1), gt_p_aligned(1,2), gt_p_aligned(1,3), ...
      'o', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'MarkerEdgeColor', 'k', ...
      'DisplayName', 'Start');

% علامة نقطة النهاية (End) - مربع أحمر
plot3(gt_p_aligned(end,1), gt_p_aligned(end,2), gt_p_aligned(end,3), ...
      's', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'k', ...
      'DisplayName', 'End');

hold off
grid on
set(gca, 'Color', 'white');
set(gca, 'GridColor', [0.85 0.85 0.85]);
xlabel('Easting [m]');
ylabel('Northing [m]');
zlabel('Up [m]');
title('Ground Truth and Estimated Trajectory');
grid on;
legend('show');
view(-60, 20);
axis equal
rotate3d on