% 主程序：激光扫描数据生成与可视化
% 生成包含圆柱面和平面的三维点云数据，包括真值数据和带噪声的模拟数据
% 输出：
% - 3D点云可视化图
% - 可选的数据文件保存

clc;
clear;

% 生成随机参数
params = generateRandomParams();

% 生成模拟数据
[sim_data, truth_data] = generateSimulatedData(num_lines, params);

% 可视化点云数据
figure('Color', 'white');
sim_cloud = pointCloud(vertcat(sim_data{:}));
pcshow(sim_cloud, 'MarkerSize', 20);
title('Simulated Data with Noise');
view([-45, 30]);

% % 可选：保存数据到文件
% writePointCloudToFile(sim_data, truth_data);

%% 主要函数
% 参数随机生成函数
function params = generateRandomParams()
    % 功能：生成模拟数据所需的所有参数
    % 输出：params结构体，包含以下字段：
    % - cylinder_radius: 圆柱体半径(mm)，固定65mm
    % - points_per_line: 每条激光线的点数，固定3840
    % - x_step: x方向扫描步长(mm)，固定0.2mm
    % - noise_amplitude: 周期噪声幅值(mm)，~0.001±15%
    % - noise_freq: 噪声频率(Hz)，~0.0002±15%
    % - noise_phase: 噪声相位(rad)，±π/4
    % - white_noise_std: 白噪声标准差(mm)，~0.02±15%
    % - plane_angle: 平面倾角(rad)，~1°±30%

    % 基础参数设置
    params.cylinder_radius = 65;     % 圆柱体半径65mm
    params.points_per_line = 3840;   % 每条线3840个点
    params.x_step = 0.2;             % x方向步进0.2mm
    
    % 随机生成噪声参数（在原始值附近波动）
    variation = 0.15;  % 降低波动范围到±15%
    
    % 周期噪声幅值在0.001左右波动（±15%）
    base_amplitude = 0.001;
    params.noise_amplitude = base_amplitude * (1 - variation + 2 * variation * rand());
    
    % 噪声频率在0.0002左右波动（±15%）
    base_freq = 0.0002;
    params.noise_freq = base_freq * (1 - variation + 2 * variation * rand());
    
    % 初始相位在0附近随机（±π/4）
    params.noise_phase = (rand() - 0.5) * pi/2;
    
    % 白噪声标准差在0.02左右波动（±15%）
    base_std = 0.02;
    params.white_noise_std = base_std * (1 - variation + 2 * variation * rand());
    
    % 平面倾角更小范围随机（±1度左右）
    base_angle = pi/180;  % 基准角度1度
    angle_variation = 0.3;  % ±30%波动
    params.plane_angle = base_angle * (1 - angle_variation + 2 * angle_variation * rand());
    
    % 打印生成的参数
    fprintf('Generated Parameters:\n');
    fprintf('Noise Amplitude: %.4f mm\n', params.noise_amplitude);
    fprintf('Noise Frequency: %.4f Hz\n', params.noise_freq);
    fprintf('Noise Phase: %.4f rad\n', params.noise_phase);
    fprintf('White Noise STD: %.4f mm\n', params.white_noise_std);
    fprintf('Plane Angle: %.4f rad (%.2f degrees)\n', params.plane_angle, params.plane_angle*180/pi);
end


function [sim_data, truth_data] = generateSimulatedData(num_lines, params)
    % 功能：生成模拟激光扫描数据
    % 输入：
    % - num_lines: ROI区域的激光线数量
    % - params: 参数结构体
    % 输出：
    % - sim_data: {num_lines x 1} cell数组，每个cell为 [3840 x 3] 的点云数据
    % - truth_data: 与sim_data格式相同，存储真值数据

    % 初始化数据存储
    sim_data = cell(num_lines, 1);
    truth_data = cell(num_lines, 1);
    
    % 基本参数设置
    points_per_line = params.points_per_line;
    y = linspace(0, 40, points_per_line)';  % Y范围0-40mm
    x_start = round(3 + rand() * 2, 1) / 0.2 * 0.2;  % 3-5mm范围内取0.2的倍数
    
    % 过渡段参数设置
    transition_start_idx = randi([1800, 2200]); 
    transition_points = randi([10, 12]);
    transition_end_idx = transition_start_idx + transition_points - 1;
    z_increments = 0.02 + 0.015 * rand(transition_points, 1);  % 0.02-0.035增量
    
    % 圆弧段角度范围
    theta_range = pi/24;
    theta_start = pi/2 - theta_range;
    theta_end = pi/2 + theta_range;
    
    % 生成每条线的数据
    for i = 1:num_lines
        % 生成X坐标
        x = x_start + (i-1) * params.x_step;
        x_vec = repmat(x, points_per_line, 1);
        
        % 生成Z坐标
        z = zeros(points_per_line, 1);
        
        % 圆柱段
        cylinder_indices = 1:transition_start_idx;
        theta = linspace(theta_start, theta_end, length(cylinder_indices))';
        z_cylinder = params.cylinder_radius * sin(theta);
        z(cylinder_indices) = z_cylinder;
        
        % 过渡段
        z_start = z(transition_start_idx);
        z_trans = z_start + cumsum(z_increments);
        z(transition_start_idx:transition_end_idx) = z_trans;
        
        % 平面段
        plane_indices = (transition_end_idx + 1):points_per_line;
        z_plane_start = z(transition_end_idx);
        y_plane = y(plane_indices) - y(transition_end_idx);
        z(plane_indices) = z_plane_start + y_plane * tan(params.plane_angle);
        
        % 创建真值数据
        truth_line = [x_vec, y, z];
        truth_data{i} = truth_line;
        
        % 生成噪声
        t = (1:points_per_line)' * params.noise_freq;
        periodic_noise = params.noise_amplitude * sin(2*pi*t + params.noise_phase);
        white_noise = params.white_noise_std * randn(points_per_line, 1);
        
        % 设置噪声系数
        noise_scale = ones(points_per_line, 1);
        cylinder_noise = 0.35 * (0.75 + 0.5 * rand());  % 0.26-0.44
        plane_noise = 0.22 * (0.75 + 0.5 * rand());     % 0.165-0.275
        
        noise_scale(1:transition_start_idx) = cylinder_noise;
        noise_scale((transition_end_idx+1):end) = plane_noise;
        
        % 添加噪声
        noise = (periodic_noise + white_noise) .* noise_scale;
        sim_line = truth_line;
        sim_line(:, 3) = sim_line(:, 3) + noise;
        
        sim_data{i} = sim_line;
    end
end


function writePointCloudToFile(sim_data, truth_data)
    % 功能：将生成的点云数据保存到文件
    % 输入：
    % - sim_data: cell数组，带噪声的模拟数据
    % - truth_data: cell数组，真值数据
    % 输出：
    % - simulated_data_[timestamp].txt: 模拟数据文件
    % - truth_data_[timestamp].txt: 真值数据文件
    % - simulation_info_[timestamp].txt: 参数信息文件

    % 获取时间戳以命名文件
    timestamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
    
    % 写入模拟数据
    sim_filename = sprintf('simulated_data_%s.txt', timestamp);
    sim_points = vertcat(sim_data{:});
    writematrix(sim_points, sim_filename, 'Delimiter', ',');
    
    % 写入真值数据
    truth_filename = sprintf('truth_data_%s.txt', timestamp);
    truth_points = vertcat(truth_data{:});
    writematrix(truth_points, truth_filename, 'Delimiter', ',');
    
    % 写入信息文件
    info_filename = sprintf('simulation_info_%s.txt', timestamp);
    fid = fopen(info_filename, 'w');
    fprintf(fid, '生成的点云数据信息：\n');
    fprintf(fid, '总行数：%d\n', size(sim_points, 1));
    fprintf(fid, '每行激光点数：%d\n', size(sim_data{1}, 1));
    fprintf(fid, '激光线数量：%d\n', length(sim_data));
    fclose(fid);
    
    fprintf('\n文件已保存：\n');
    fprintf('1. 模拟数据：%s\n', sim_filename);
    fprintf('2. 真值数据：%s\n', truth_filename);
    fprintf('3. 信息文件：%s\n', info_filename);
end