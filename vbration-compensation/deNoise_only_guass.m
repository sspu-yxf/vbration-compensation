function rebuilt_plane_line_data = deNoise_only_guass(plane_line_data)
    % 简单的高斯平滑实现，用于对比实验
    % 输入：plane_line_data - 元胞数组，每个元素是一条激光线的点云数据
    % 输出：rebuilt_plane_line_data - 平滑处理后的点云数据

    % 定义平滑参数结构体
    params = struct();
    params.window_size = 300;  % 固定窗口大小
    params.sigma = params.window_size/6;  % 高斯核标准差

    % 初始化输出
    num_lines = length(plane_line_data);
    rebuilt_plane_line_data = cell(num_lines, 1);
    
    % 拟合平面，获取法向量
    all_points = vertcat(plane_line_data{:});
    [plane_model, ~] = pcfitplane(pointCloud(all_points), 0.1);
    plane_normal = (-1) * plane_model.Normal;
    if(plane_model.Parameters(4) < 0)
        plane_normal = (-1) * plane_normal;
    end
    plane_point = plane_model.Parameters(4) * plane_normal;
    
    % 逐行处理
    for line_idx = 1:num_lines
        current_line = plane_line_data{line_idx};
        
        if size(current_line, 1) > 1
            % 计算点到平面的距离
            vec_to_plane = current_line - repmat(plane_point, size(current_line, 1), 1);
            distances = dot(vec_to_plane, repmat(plane_normal, size(current_line, 1), 1), 2);
            
            % 高斯平滑
            smoothed_distances = gaussian_smooth(distances, params);
            
            % 重建点云
            projected_points = current_line - distances * plane_normal;
            rebuilt_points = projected_points + smoothed_distances * plane_normal;
            rebuilt_plane_line_data{line_idx} = rebuilt_points;
        else
            rebuilt_plane_line_data{line_idx} = current_line;
        end
    end
    
    % 可视化结果
    visualize_smoothing_result(plane_line_data, rebuilt_plane_line_data, plane_normal, plane_point, params);
end

function smoothed = gaussian_smooth(signal, params)
    % 高斯平滑实现
    % 输入：
    %   signal - 输入信号
    %   params - 包含window_size和sigma的参数结构体
    
    % 确保窗口大小为奇数
    window_size = params.window_size;
    sigma = params.sigma;
    window_size = 2 * floor(window_size/2) + 1;
    
    % 创建高斯核
    half_win = floor(window_size/2);
    x = -half_win:half_win;
    gaussian_kernel = exp(-x.^2 / (2*sigma^2));
    gaussian_kernel = gaussian_kernel / sum(gaussian_kernel);  % 归一化
    
    % 应用高斯平滑
    L = length(signal);
    smoothed = zeros(size(signal));
    
    % 处理信号主体
    for i = 1:L
        % 计算当前窗口范围
        start_idx = max(1, i-half_win);
        end_idx = min(L, i+half_win);
        win_start = half_win - (i-start_idx) + 1;
        win_end = win_start + (end_idx-start_idx);
        
        % 应用高斯权重
        current_kernel = gaussian_kernel(win_start:win_end);
        current_kernel = current_kernel / sum(current_kernel);  % 重新归一化
        smoothed(i) = sum(signal(start_idx:end_idx) .* current_kernel');
    end
end

function visualize_smoothing_result(original_data, smoothed_data, plane_normal, plane_point, params)
    % 可视化平滑结果
    % 随机选择一行数据进行展示
    
    % 随机选择一个有效的行索引
    valid_indices = find(cellfun(@(x) size(x,1) > 1, original_data));
    if isempty(valid_indices)
        warning('No valid data lines found for visualization');
        return;
    end
    rand_idx = valid_indices(randi(length(valid_indices)));
    
    % 获取原始和平滑后的距离值
    original_line = original_data{rand_idx};
    smoothed_line = smoothed_data{rand_idx};
    
    % 计算到平面的距离
    original_vec = original_line - repmat(plane_point, size(original_line, 1), 1);
    smoothed_vec = smoothed_line - repmat(plane_point, size(smoothed_line, 1), 1);
    
    original_distances = sum(original_vec .* repmat(plane_normal, size(original_line, 1), 1), 2);
    smoothed_distances = sum(smoothed_vec .* repmat(plane_normal, size(smoothed_line, 1), 1), 2);
    
    % 创建图形
    figure('Color', 'white', 'Position', [100, 100, 800, 400]);
    

    % 绘制对比图
    subplot(2,1,1);
    plot(original_distances, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Original');
    hold on;
    plot(smoothed_distances, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Smoothed');
    
    % 添加图形元素
    title(sprintf('Gaussian Smoothing Result (Line %d)', rand_idx));
    legend('Location', 'best');
    xlabel('Point Index');
    ylabel('Distance (mm)');
    grid on;
    
    % 计算并显示统计信息
    rmse = sqrt(mean((original_distances - smoothed_distances).^2));
    info_str = sprintf('Line Index: %d\nWindow Size: %d\nRMS Error: %.4f mm', ...
        rand_idx, params.window_size, rmse);
    
    annotation('textbox', [0.15, 0.02, 0.3, 0.1], ...
        'String', info_str, ...
        'FitBoxToText', 'on', ...
        'BackgroundColor', 'white');
end