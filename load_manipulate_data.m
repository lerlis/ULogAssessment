% 清空完成初始化
clear;
close all;

% 数据读取
% 执行器故障 第一个螺旋桨推力减少至原来的0.4且持续时间60秒
name = 'log_16_2022-3-8-19-55-06';
% 文件存储路径
file_store_location = '.\data\';
% 着陆检测
file_land_name = [file_store_location, name, '\', name, '_vehicle_land_detected_0.csv'];
vehicle_land_detected = readmatrix(file_land_name);
% 着陆检测标志位
% 注意第一列是时间戳，后同不再赘述
ground_contact = [vehicle_land_detected(:, 1), vehicle_land_detected(:, 4)];
has_low_throttle = [vehicle_land_detected(:, 1), vehicle_land_detected(:, 9)];

% 位置速度信息
file_local_posi_name = [file_store_location, name, '\', name, '_vehicle_local_position_0.csv'];
vehicle_local_position = readmatrix(file_local_posi_name);
x = [vehicle_local_position(:, 1), vehicle_local_position(:, 6)];
y = [vehicle_local_position(:, 1), vehicle_local_position(:, 7)];
z = [vehicle_local_position(:, 1), vehicle_local_position(:, 8)];
ax = [vehicle_local_position(:, 1), vehicle_local_position(:, 19)];
ay = [vehicle_local_position(:, 1), vehicle_local_position(:, 20)];
az = [vehicle_local_position(:, 1), vehicle_local_position(:, 21)];
vx = [vehicle_local_position(:, 1), vehicle_local_position(:, 12)];
vy = [vehicle_local_position(:, 1), vehicle_local_position(:, 13)];
vz = [vehicle_local_position(:, 1), vehicle_local_position(:, 14)];

% 姿态信息
file_attitude_name = [file_store_location, name, '\', name, '_vehicle_attitude_0.csv'];
vehicle_attitude = readmatrix(file_attitude_name);
att_size = size(vehicle_attitude);
len = att_size(1);
roll = zeros(len, 2);
pitch = zeros(len, 2);
yaw = zeros(len, 2);

quat = [vehicle_attitude(:, 3), vehicle_attitude(:, 4), vehicle_attitude(:, 5), vehicle_attitude(:, 6)];
% 默认为ZYX旋转
[yaw(:, 2), pitch(:, 2), roll(:, 2)] = quat2angle(quat);
roll(:, 1) = vehicle_attitude(:, 1);
pitch(:, 1) = vehicle_attitude(:, 1);
yaw(:, 1) = vehicle_attitude(:, 1);

% 执行器输出
file_actuator_name = [file_store_location, name, '\', name, '_actuator_outputs_0.csv'];
actuator_output = readmatrix(file_actuator_name);
opt1 = [actuator_output(:, 1), actuator_output(:, 3)];
opt2 = [actuator_output(:, 1), actuator_output(:, 4)];
opt3 = [actuator_output(:, 1), actuator_output(:, 5)];
opt4 = [actuator_output(:, 1), actuator_output(:, 6)];
sigma1 = pwm2sigma(opt1);
sigma2 = pwm2sigma(opt2);
sigma3 = pwm2sigma(opt3);
sigma4 = pwm2sigma(opt4);

% 机体旋转角速度
file_sensor_combined_name = [file_store_location, name, '\', name, '_sensor_combined_0.csv'];
sensor_combined = readmatrix(file_sensor_combined_name);
x_rate = [sensor_combined(:, 1), sensor_combined(:, 2)];
y_rate = [sensor_combined(:, 1), sensor_combined(:, 3)];
z_rate = [sensor_combined(:, 1), sensor_combined(:, 4)];

%重要参数和时间节点
fly_timestamp = find_land_changed_time(ground_contact, false, 0);
land_timestamp = find_land_changed_time(ground_contact, false, fly_timestamp);

new_has_low_throttle = cut_data(fly_timestamp, land_timestamp, has_low_throttle);
begin_land_time = find_land_changed_time(new_has_low_throttle, true);

% 裁切数据
new_z = cut_data(fly_timestamp, begin_land_time, z);
new_vz = cut_data(fly_timestamp, begin_land_time, vz);
new_roll = cut_data(fly_timestamp, begin_land_time, roll);
new_pitch = cut_data(fly_timestamp, begin_land_time, pitch);
new_yaw = cut_data(fly_timestamp, begin_land_time, yaw);
new_x_rate = cut_data(fly_timestamp, begin_land_time, x_rate);
new_y_rate = cut_data(fly_timestamp, begin_land_time, y_rate);
new_z_rate = cut_data(fly_timestamp, begin_land_time, z_rate);
new_sigma1 = cut_data(fly_timestamp, begin_land_time, sigma1);
new_sigma2 = cut_data(fly_timestamp, begin_land_time, sigma2);
new_sigma3 = cut_data(fly_timestamp, begin_land_time, sigma3);
new_sigma4 = cut_data(fly_timestamp, begin_land_time, sigma4);

% 对长度进行规整
% 计划新的采样时间
onesec = 1e6;
ts = 0.05;
sample_period = ts * onesec;
target_timevec = fly_timestamp + 2 * onesec:sample_period:begin_land_time - 2 * onesec;
% 重采样
roll = resample(timeseries(new_roll(:, 2), new_roll(:, 1), 'Name', 'Angle of Roll: phi'), target_timevec);
pitch = resample(timeseries(new_pitch(:, 2), new_pitch(:, 1), 'Name', 'Angle of Pitch: theta'), target_timevec);
yaw = resample(timeseries(new_yaw(:, 2), new_yaw(:, 1), 'Name', 'Angle of Yaw: psi'), target_timevec);
z = resample(timeseries(new_z(:, 2), new_z(:, 1), 'Name', 'Position of Z: z'), target_timevec);
vz = resample(timeseries(new_vz(:, 2), new_vz(:, 1), 'Name', 'Velocity of Z: vz'), target_timevec);
x_rate = resample(timeseries(new_x_rate(:, 2), new_x_rate(:, 1), 'Name', 'Velocity of X: p'), target_timevec);
y_rate = resample(timeseries(new_y_rate(:, 2), new_y_rate(:, 1), 'Name', 'Velocity of Y: q'), target_timevec);
z_rate = resample(timeseries(new_z_rate(:, 2), new_z_rate(:, 1), 'Name', 'Velocity of Z: r'), target_timevec);
sigma1 = resample(timeseries(new_sigma1(:, 2), new_sigma1(:, 1), 'Name', 'throttle of bldc #1'), target_timevec);
sigma2 = resample(timeseries(new_sigma2(:, 2), new_sigma2(:, 1), 'Name', 'throttle of bldc #2'), target_timevec);
sigma3 = resample(timeseries(new_sigma3(:, 2), new_sigma3(:, 1), 'Name', 'throttle of bldc #3'), target_timevec);
sigma4 = resample(timeseries(new_sigma4(:, 2), new_sigma4(:, 1), 'Name', 'throttle of bldc #4'), target_timevec);

% 转为矩阵
euler_re = [roll.Data pitch.Data yaw.Data]';
z_re = z.Data;
vz_re = vz.Data;
omega_re = [x_rate.Data y_rate.Data z_rate.Data]';
sigma_re = [sigma1.Data sigma2.Data sigma3.Data sigma4.Data]';

% 计算欧拉角速度
len = length(target_timevec);
dot_euler_re = zeros(3, len);

for j = 1:1:len
    W = mat_pqr2euler(euler_re(1, j), euler_re(2, j));
    dot_euler_re(:, j) = W * omega_re(:, j);
end

clearvars -except *_re target_timevec ts onesec
