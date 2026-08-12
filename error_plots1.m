clear
close all
clc
%% Sensors data
load("pt1_data_fixed.mat");
dataset = data1;
imu_f = dataset.imu_f.data;
imu_w = dataset.imu_w.data;
t_imu = dataset.imu_f.t;
gnss_data = dataset.gnss.data;
gnss_t = dataset.gnss.t;
num_step = size(imu_f , 1);

gt_p = dataset.gt.p;
gt_r = dataset.gt.r;
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
est_angles = zeros(num_step , 3);
P_diag_history = zeros(num_step , 9);
gnss_index = 1;

%% ES-EKF Loop
for k = 1 : (num_step-1)
  dt = t_imu(k + 1) - t_imu(k);
  [p , v , q , P] = es_ekf.perdict(p , v , q , P, imu_f(k , :)', imu_w(k , :)', var_imu_f, var_imu_w, g, dt);
  if ((gnss_index < length(gnss_t)) && ((abs(t_imu(k) - gnss_t(gnss_index))) < 1e-4))
      [p , v , q , P] = es_ekf.update(p , v , q , P , gnss_data(gnss_index , :)' , R_gnss);
      gnss_index = gnss_index + 1;
  end
  est_postation(k , :) = p';
  [Roll , Pitch , Yaw] = rotation.quat2euler(q);
  est_angles(k , :) = [Roll , Pitch , Yaw];
  P_diag_history(k , :) = diag(P)';
end

%% Align ground truth to IMU timestamps
gt_t_matched = gt_t(1:size(gt_p,1));
gt_p_aligned = dataset.gt.p;
gt_r_aligned = dataset.gt.r;

%% توحيد الأطوال قبل الطرح (بعض المصفوفات بيطلع فيها فرق بسيط في الحجم)
n = min([size(est_postation,1), size(gt_p_aligned,1), size(est_angles,1), size(gt_r_aligned,1), size(P_diag_history,1)]);

est_postation  = est_postation(1:n, :);
est_angles     = est_angles(1:n, :);
gt_p_aligned   = gt_p_aligned(1:n, :);
gt_r_aligned   = gt_r_aligned(1:n, :);
P_diag_history = P_diag_history(1:n, :);

%% Real error = estimate - ground truth
pos_error = est_postation - gt_p_aligned;
angle_error = est_angles - gt_r_aligned;


%% Error Plots
figure(1);
titles = {"Easting" , "Northing" , "Up" , "Roll" , "Pitch" , "Yaw"};
y_labels = {"Meters" , "Meters" , "Meters" , "Radian" , "Radian" , "Radian"};
plot_limit = min(n , 8500);
x_vals = 1 : plot_limit;
for i = 1 : 6
    subplot(2 , 3 , i);
    if i <= 3
        p_idx = i;              % position states: 1,2,3
        err_data = pos_error(x_vals, i);
    else
        p_idx = i + 3;           % attitude states: 7,8,9 (مش 4,5,6)
        angle_idx = i - 3;
        err_data = angle_error(x_vals, angle_idx);
    end
    sigma_bounds = 7 * sqrt(P_diag_history(x_vals , p_idx));
    hold on
    plot(x_vals , sigma_bounds , 'Color', 'r' , 'LineWidth', 1.2);
    plot(x_vals , -sigma_bounds , 'Color', 'r' , 'LineWidth', 1.2);
    plot(x_vals , err_data , 'Color', 'b' , 'LineWidth', 1.1);
    hold off;
    title(titles{i});
    ylabel(y_labels{i});
    grid on;
    box on;
end
sgtitle("Error State (Estimate - Ground Truth)");