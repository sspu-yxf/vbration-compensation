function rebuilt_plane_line_data = deNoise_only_laplacian(plane_line_data)
    % 简单的拉普拉斯平滑实现，用于对比实验
    % 输入：plane_line_data - 元胞数组，每个元素是一条激光线的点云数据
    % 输出：rebuilt_plane_line_data - 平滑处理后的点云数据

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
    
    % 平滑参数
    num_iterations = 5;  % 迭代次数
    alpha = 0.5;        % 平滑系数

    % 逐行处理
    for line_idx = 1:num_lines
        current_line = plane_line_data{line_idx};
        
        if size(current_line, 1) > 1
            % 计算点到平面的距离
            vec_to_plane = current_line - repmat(plane_point, size(current_line, 1), 1);
            distances = dot(vec_to_plane, repmat(plane_normal, size(current_line, 1), 1), 2);
            
            
            % 拉普拉斯平滑
            smoothed_distances = laplacian_smooth(distances, num_iterations, alpha);
            
            % 重建点云
            projected_points = current_line - distances * plane_normal;
            rebuilt_points = projected_points + smoothed_distances * plane_normal;
            rebuilt_plane_line_data{line_idx} = rebuilt_points;
        else
            rebuilt_plane_line_data{line_idx} = current_line;
        end
    end
    
    % 可视化结果（随机选择一行进行展示）
    visualize_smoothing_result(plane_line_data, rebuilt_plane_line_data, plane_normal, plane_point);
end

function smoothed = laplacian_smooth(signal, num_iterations, alpha)
    % 简单的拉普拉斯平滑实现
    % 输入：
    %   signal - 输入信号
    %   num_iterations - 迭代次数
    %   alpha - 平滑系数 (0-1)
    
    smoothed = signal;
    L = length(signal);
    
    for iter = 1:num_iterations
        temp = smoothed;
        for i = 2:L-1
            % 拉普拉斯算子
            laplacian = temp(i+1) - 2*temp(i) + temp(i-1);
            smoothed(i) = temp(i) + alpha * laplacian;
        end
        
        % 处理边界点
        smoothed(1) = smoothed(2);
        smoothed(L) = smoothed(L-1);
    end
end

function visualize_smoothing_result(original_data, smoothed_data, plane_normal, plane_point)
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

    subplot(2,1,1)
    % 绘制对比图
    plot(original_distances, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Original');
    hold on;
    plot(smoothed_distances, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Smoothed');
    
    % 添加图形元素
    title(sprintf('Laplacian Smoothing Result (Line %d)', rand_idx));
    legend('Location', 'best');
    xlabel('Point Index');
    ylabel('Distance (mm)');
    
    % 计算并显示统计信息
    rmse = sqrt(mean((original_distances - smoothed_distances).^2));
    info_str = sprintf('Line Index: %d\nRMS Error: %.4f mm', rand_idx, rmse);
    
    annotation('textbox', [0.15, 0.02, 0.3, 0.1], ...
        'String', info_str, ...
        'FitBoxToText', 'on', ...
        'BackgroundColor', 'white');
end