% 主程序：生成模拟激光扫描数据
% 生成一个包含圆柱面和平面的三维点云数据，包括真值数据和带噪声的模拟数据
% 输出：
% - sim_data: cell数组，每个cell包含一条激光线的模拟数据 [x,y,z]
% - truth_data: cell数组，每个cell包含一条激光线的真值数据 [x,y,z]

clc;
clear;

% 生成随机参数
params = generateRandomParams();

% 生成模拟数据
[sim_data, truth_data] = generateSimulatedData(params);

% % 可视化点云数据
% figure('Color', 'white');
% sim_cloud = pointCloud(vertcat(sim_data{:}));
% pcshow(sim_cloud, 'BackgroundColor', [1,1,1]);
% title('Simulated Data with Noise');
% view([-45, 30]);

% 保存数据到文件
writePointCloudToFile(sim_data, truth_data);

%% 主要函数
% 参数生成函数
function params = generateRandomParams()

    % 基础参数设置
    params.cylinder_radius = 65;     
    params.points_per_line = 3840;   
    params.x_step = 0.2;            
    params.base_height = 60 + randi([0, 3]);  
    
    % 禁用周期噪声
    params.num_waves = 0;  % 设置为0表示没有周期噪声
    params.wave_params = struct('amplitude', [], 'freq', [], 'phase', []);
    
    % 白噪声参数
    params.white_noise_std = 0.05;  % 白噪声幅值为0.05mm
    
    % 设置平面为水平面
    params.plane_angle = 0;  % 设置为0，表示水平面
    
    % 打印参数信息
    fprintf('Generated Parameters:\n');
    fprintf('Base Height: %.2f mm\n', params.base_height);
    fprintf('White Noise STD: %.4f mm\n', params.white_noise_std);
    fprintf('Plane Angle: %.4f rad (%.2f degrees)\n', ...
        params.plane_angle, params.plane_angle*180/pi);
end


% 主数据生成函数
function [sim_data, truth_data] = generateSimulatedData(params)
    % 功能：生成完整的模拟数据，包括ROI区域前、中、后三段数据
    % 输入：params - 参数结构体
    % 输出：
    % - sim_data: cell数组，每个cell包含一条带噪声的激光线数据 [x,y,z]
    % - truth_data: cell数组，每个cell包含一条真值激光线数据 [x,y,z]

    % 初始化参数
    points_per_line = params.points_per_line;
    y = linspace(0, 40, points_per_line)';
    
    % 生成ROI区域参数
    num_lines = randi([10, 15]);  % ROI区域的激光线数量
    x_start = floor((3 + rand() * 2) / params.x_step) * params.x_step;
    if x_start < 3
        x_start = x_start + params.x_step;
    end
    m = round(x_start / params.x_step);  % 前段线数
    n = m + randi([-5, 5]);  % 后段线数
    
    % 过渡段参数
    transition_start_idx = randi([1800, 2200]); 
    transition_points = randi([10, 12]);
    transition_end_idx = transition_start_idx + transition_points - 1;
    z_increments = 0.02 + 0.015 * rand(transition_points, 1);
    
    % 圆弧段角度范围
    theta_range = pi/24;
    theta_start = pi/2 - theta_range;
    theta_end = pi/2 + theta_range;
    
    % 生成基准Z坐标
    z_plane_start = generateBaseZ(params, transition_start_idx, ...
        transition_end_idx, z_increments, theta_start, theta_end);
    
    % 内存预分配
    total_lines = m + num_lines + n;
    sim_data = cell(total_lines, 1);
    truth_data = cell(total_lines, 1);
    
    % 生成前段数据
    [pre_sim, pre_truth] = generatePreData(m, params, y, ...
        transition_end_idx, z_plane_start);
    sim_data(1:m) = pre_sim;
    truth_data(1:m) = pre_truth;
    
    % 生成ROI区域数据
    [roi_sim, roi_truth] = generateROIData(num_lines, params, x_start, y, ...
        transition_start_idx, transition_end_idx, z_increments, ...
        theta_start, theta_end);
    sim_data(m+1:m+num_lines) = roi_sim;
    truth_data(m+1:m+num_lines) = roi_truth;
    
    % 生成后段数据
    x_end = x_start + (num_lines - 1) * params.x_step;
    [post_sim, post_truth] = generatePostData(n, params, x_end, y, ...
        transition_end_idx, z_plane_start);
    sim_data(m+num_lines+1:end) = post_sim;
    truth_data(m+num_lines+1:end) = post_truth;
    
    % 打印生成信息
    fprintf('\n生成的点云数据信息：\n');
    fprintf('ROI前的线数：%d\n', m);
    fprintf('ROI区域的线数：%d\n', num_lines);
    fprintf('ROI后的线数：%d\n', n);
    fprintf('总线数：%d\n', total_lines);
end

%% 辅助函数
% 基准Z坐标生成函数
function z_plane_start = generateBaseZ(params, transition_start_idx, ...
    transition_end_idx, z_increments, theta_start, theta_end)
    % 功能：生成基准Z坐标，计算平面段起始高度
    % 输入：
    % - params: 参数结构体
    % - transition_start_idx: 过渡段起始索引
    % - transition_end_idx: 过渡段结束索引
    % - z_increments: 过渡段Z增量
    % - theta_start, theta_end: 圆弧段角度范围
    % 输出：
    % - z_plane_start: 平面段起始Z坐标（标量值）

    points_per_line = params.points_per_line;
    base_z = zeros(points_per_line, 1);
    
    % 生成圆柱段坐标
    cylinder_indices = 1:transition_start_idx;
    theta = linspace(theta_start, theta_end, length(cylinder_indices))';
    base_z(cylinder_indices) = params.cylinder_radius * sin(theta) - params.base_height;
    
    % 计算过渡段坐标
    z_start = base_z(transition_start_idx);
    z_trans = z_start + cumsum(z_increments);
    base_z(transition_start_idx:transition_end_idx) = z_trans;
    
    z_plane_start = base_z(transition_end_idx);  % 获取平面段起始高度
end

function [sim_data, truth_data] = generatePreData(...
    m, params, y, transition_end_idx, z_plane_start)
    % 功能：生成ROI区域前段的激光扫描数据
    % 输入：
    % - m: 前段激光线数量
    % - params: 参数结构体
    % - y: Y坐标向量 [points_per_line x 1]
    % - transition_end_idx: 过渡段结束索引
    % - z_plane_start: 平面段起始高度
    % 输出：
    % - sim_data: cell数组 [m x 1]，每个cell包含一条带噪声的激光线数据 [points_per_line x 3]
    % - truth_data: cell数组 [m x 1]，每个cell包含一条真值激光线数据 [points_per_line x 3]
    
    sim_data = cell(m, 1);
    truth_data = cell(m, 1);
    
    for i = 1:m
        % 生成X坐标
        x = (i-1) * params.x_step;
        x_vec = repmat(x, params.points_per_line, 1);
    
        % 生成Z坐标（水平平面）
        z = zeros(params.points_per_line, 1);
        plane_indices = (transition_end_idx + 1):params.points_per_line;
        z(plane_indices) = z_plane_start;  % 平面段保持在固定高度
    
        % 添加噪声
        [sim_data{i}, truth_data{i}] = addNoise(x_vec, y, z, params, ...
            plane_indices, [], false);
    end
end

function [sim_data, truth_data] = generateROIData(num_lines, params, x_start, y, ...
    transition_start_idx, transition_end_idx, z_increments, theta_start, theta_end)
    % 功能：生成ROI区域的激光扫描数据
    % 输入：
    % - num_lines: ROI区域激光线数量
    % - params: 参数结构体
    % - x_start: ROI区域开始X坐标
    % - y: Y坐标向量 [points_per_line x 1]
    % - transition_start_idx: 过渡段开始索引
    % - transition_end_idx: 过渡段结束索引
    % - z_increments: 扫描平面高度
    % - theta_start: 开始角度
    % - theta_end: 结束角度
    % 输出：
    % - sim_data: cell数组 [n x 1]，每个cell包含一条带噪声的激光线数据
    % - truth_data: cell数组 [n x 1]，每个cell包含一条真值激光线数据    

    sim_data = cell(num_lines, 1);      % 预分配模拟数据存储空间
    truth_data = cell(num_lines, 1);    % 预分配真值数据存储空间   
    
    for i = 1:num_lines
        % 生成X坐标
        x = x_start + (i-1) * params.x_step;    
        x_vec = repmat(x, params.points_per_line, 1);  
    
        % 生成完整的Z坐标和更新的Y坐标
        [z, y_updated] = generateFullZ(params, transition_start_idx, transition_end_idx, ...
            z_increments, theta_start, theta_end, y);
    
        % 定义圆柱段和平面段的索引范围
        cylinder_indices = 1:transition_start_idx;    
        plane_indices = (transition_end_idx + 1):params.points_per_line;  
    
        % 使用更新后的y坐标生成点云
        [sim_data{i}, truth_data{i}] = addNoise(x_vec, y_updated, z, params, ...
            plane_indices, cylinder_indices, true);
    end
end

function [sim_data, truth_data] = generatePostData(n, params, x_end, y, ...
    transition_end_idx, z_plane_start)
    % 功能：生成ROI区域后段的激光扫描数据
    % 输入：
    % - n: 后段激光线数量
    % - params: 参数结构体
    % - x_end: ROI区域结束X坐标
    % - y: Y坐标向量 [points_per_line x 1]
    % - transition_end_idx: 过渡段结束索引
    % - z_plane_start: 平面段起始高度
    % 输出：
    % - sim_data: cell数组 [n x 1]，每个cell包含一条带噪声的激光线数据
    % - truth_data: cell数组 [n x 1]，每个cell包含一条真值激光线数据
 
    sim_data = cell(n, 1);      % 预分配模拟数据存储空间
    truth_data = cell(n, 1);    % 预分配真值数据存储空间
    
    for i = 1:n
        % 生成X坐标
        x = x_end + i * params.x_step;
        x_vec = repmat(x, params.points_per_line, 1);
    
        % 生成Z坐标（水平平面）
        z = zeros(params.points_per_line, 1);
        plane_indices = (transition_end_idx + 1):params.points_per_line;
        z(plane_indices) = z_plane_start;  % 平面段保持在固定高度
    
        % 添加噪声
        [sim_data{i}, truth_data{i}] = addNoise(x_vec, y, z, params, ...
            plane_indices, [], false);
    end
end

function [z, y_new] = generateFullZ(params, transition_start_idx, transition_end_idx, ...
    z_increments, theta_start, theta_end, y)
    % 功能：生成Z值数据
    % 输入：
    % - params: 参数结构体
    % - transition_start_idx: 过渡段开始索引
    % - transition_end_idx: 过渡段结束索引
    % - z_increments: 扫描平面高度
    % - theta_start: 开始角度
    % - theta_end: 结束角度
    % 输出：
    % - y: Y坐标向量 [points_per_line x 1]
    % - y_new: 更新后的Y坐标向量(连续) [points_per_line x 1]
    
    % 初始化Z坐标数组
    z = zeros(params.points_per_line, 1);
    y_new = y;
    
    % 生成圆柱段坐标
    cylinder_indices = 1:transition_start_idx;
    theta = linspace(theta_start, theta_end, length(cylinder_indices))';
    
    % 计算圆柱段的原始y坐标
    y_cylinder = params.cylinder_radius * cos(theta);
    y_offset = y(transition_start_idx) - y_cylinder(1);
    y_new(cylinder_indices) = y_cylinder + y_offset;
    
    % 计算圆柱段的z坐标
    z(cylinder_indices) = params.cylinder_radius * sin(theta) - params.base_height;
    
    % 生成过渡段坐标
    z_start = z(transition_start_idx);
    z_trans = z_start + cumsum(z_increments);
    z(transition_start_idx:transition_end_idx) = z_trans;
    
    % 生成平面段坐标（水平面）
    plane_indices = (transition_end_idx + 1):params.points_per_line;
    z(plane_indices) = z(transition_end_idx);  % 平面段保持在过渡段结束的高度
end

function [sim_line, truth_line] = addNoise(x_vec, y, z, params, plane_indices, cylinder_indices, is_roi)
    % 功能：生成复合噪声（周期噪声 + 白噪声）
    % 输入：
    % - n_points: 需要生成噪声的点数
    % - params: 参数结构体，包含噪声参数
    % - start_index: 起始索引，用于保持噪声连续性
    % 输出：
    % - noise: 生成的复合噪声 [n_points x 1]
    
    % 构建真值点云数据 [x,y,z]
    truth_line = [x_vec, y, z];    
    % 初始化模拟数据为真值数据
    sim_line = truth_line;         
    
    % 为平面段添加噪声
    if ~isempty(plane_indices)
        % 使用全局索引生成噪声
        plane_noise = generateNoise(length(plane_indices), params, plane_indices(1));  
        % 噪声系数设为1
        plane_noise_coef = 1;  
        % 将噪声添加到Z坐标
        sim_line(plane_indices, 3) = sim_line(plane_indices, 3) + ...
            plane_noise * plane_noise_coef;  
    end
    
    % 为ROI区域的圆柱段添加噪声
    if is_roi && ~isempty(cylinder_indices)
        % 使用全局索引生成噪声
        cylinder_noise = generateNoise(length(cylinder_indices), params, cylinder_indices(1));  
        % 噪声系数设为1
        cylinder_noise_coef = 1;  
        % 将噪声添加到Z坐标
        sim_line(cylinder_indices, 3) = sim_line(cylinder_indices, 3) + ...
            cylinder_noise * cylinder_noise_coef;  
    end
end

function noise = generateNoise(n_points, params, start_index)
    % 功能：生成带连续性的白噪声
    % 输入：
    % - n_points: 需要生成噪声的点数
    % - params: 参数结构体，包含噪声参数
    % - start_index: 起始索引
    % 输出：
    % - noise: 生成的白噪声 [n_points x 1]
    
    % 生成原始白噪声
    raw_noise = params.white_noise_std * randn(n_points, 1);
    
    % 使用移动平均来增加相邻元素的连续性
    window_size = 50;  % 移动平均窗口大小
    kernel = ones(1, window_size) / window_size;
    noise = conv(raw_noise, kernel, 'same');
end

function writePointCloudToFile(sim_data, truth_data)
    % 功能：将生成的点云数据保存到文件
    % 输入：
    % - sim_data: cell数组，带噪声的模拟数据
    % - truth_data: cell数组，真值数据
    
    sim_filename = '0_white_0.txt';

    sim_points = vertcat(sim_data{:});
    writematrix(sim_points, sim_filename, 'Delimiter', ',');    
  
    fprintf('%s 模拟数据文件已保存\n', sim_filename);
end