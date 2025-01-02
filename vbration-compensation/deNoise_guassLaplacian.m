function rebuilt_plane_line_data = deNoise_guassLaplacian(plane_line_data)
    % 构建平面点云的噪声模型
    num_lines = length(plane_line_data);
    rebuilt_plane_line_data = cell(num_lines, 1);
    
    % 合并所有点云数据用于平面拟合
    all_points = vertcat(plane_line_data{:});
    
    % 使用RANSAC进行平面拟合
    [plane_model, ~] = pcfitplane(pointCloud(all_points), 0.1);
    plane_normal = (-1) * plane_model.Normal;
    if(plane_model.Parameters(4) < 0)
        plane_normal = (-1) * plane_normal;  % 翻转法向
    end
    plane_point = plane_model.Parameters(4) * plane_normal;    
    
    % 设置固定窗口大小和参数
    params = struct();
    params.window_size = 51;          % 固定窗口大小(确保是奇数)
    params.gaussian_sigma = 10;       % 高斯核标准差
    params.laplacian_alpha = 0.5;     % 拉普拉斯增强因子
    params.laplacian_iterations = 3;  % 拉普拉斯迭代次数
    
    % 逐行处理
    for line_idx = 1:num_lines
        current_line = plane_line_data{line_idx};
        
        if size(current_line, 1) > 1
            % 计算相对于平面的有符号距离
            distances = calculate_plane_distances(current_line, plane_normal, plane_point);
            
            % 高斯平滑
            smoothed = gaussian_smooth(distances, params);
            
            % 拉普拉斯增强
            enhanced = laplacian_enhance(smoothed, params);
            
            % 重建点云时保持原始点到平面的投影位置不变
            projected_points = current_line - distances * plane_normal;
            rebuilt_points = projected_points + enhanced * plane_normal;
            rebuilt_plane_line_data{line_idx} = rebuilt_points;
        else
            rebuilt_plane_line_data{line_idx} = current_line;
        end
    end
    
    % 可视化处理结果
    visualize_random_line(plane_line_data, rebuilt_plane_line_data, plane_normal, plane_point);
end

function distances = calculate_plane_distances(points, plane_normal, plane_point)
    % 计算点到平面的有符号距离
    % points: N×3 matrix
    % plane_normal: 1×3 vector
    % plane_point: 1×3 vector
    vec_to_plane = points - repmat(plane_point, size(points, 1), 1);
    distances = sum(vec_to_plane .* repmat(plane_normal, size(points, 1), 1), 2);
end

function smoothed = gaussian_smooth(signal, params)
    % 固定窗口高斯平滑
    L = length(signal);
    smoothed = zeros(L, 1);
    half_win = floor(params.window_size/2);
    
    % 生成高斯核
    x = -half_win:half_win;
    gaussian_kernel = exp(-x.^2 / (2*params.gaussian_sigma^2));
    gaussian_kernel = gaussian_kernel / sum(gaussian_kernel);
    
    % 边界处理：镜像填充
    padded_signal = [flipud(signal(1:half_win)); signal; flipud(signal(end-half_win+1:end))];
    
    % 滑动窗口实现平滑
    for i = 1:L
        start_idx = i;
        end_idx = i + params.window_size - 1;
        window_data = padded_signal(start_idx:end_idx);
        smoothed(i) = sum(window_data .* gaussian_kernel');
    end
end

function enhanced = laplacian_enhance(smoothed, params)
    % 拉普拉斯增强处理
    L = length(smoothed);
    enhanced = smoothed;
    
    % 拉普拉斯迭代增强
    for iter = 1:params.laplacian_iterations
        temp_data = enhanced;
        for i = 2:L-1
            % 计算拉普拉斯算子
            laplacian = temp_data(i+1) - 2*temp_data(i) + temp_data(i-1);
            
            % 应用拉普拉斯修正
            enhanced(i) = temp_data(i) + params.laplacian_alpha * laplacian;
        end
        
        % 处理边界
        enhanced([1,L]) = enhanced([2,L-1]);
    end
end

function visualize_random_line(original_data, rebuilt_data, plane_normal, plane_point)
    % 随机选择一行数据进行可视化
    valid_lines = find(~cellfun(@isempty, original_data));
    rand_idx = valid_lines(randi(length(valid_lines)));
    
    % 获取原始和重建后的距离
    original_line = original_data{rand_idx};
    rebuilt_line = rebuilt_data{rand_idx};
    
    % 计算到平面的距离
    orig_distances = calculate_plane_distances(original_line, plane_normal, plane_point);
    rebuilt_distances = calculate_plane_distances(rebuilt_line, plane_normal, plane_point);
    
    % 创建图形窗口
    figure('Color', 'white', 'Position', [100, 100, 800, 300]);
    
    % 绘制距离对比图
    plot(orig_distances, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Original');
    hold on;
    plot(rebuilt_distances, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Smoothed');
    title(sprintf('Original vs Smoothed Distance (Line %d)', rand_idx));
    legend('Location', 'best');
    grid on;
    xlabel('Point Index');
    ylabel('Distance (mm)');
    
    % 添加统计信息
    rmse = sqrt(mean((orig_distances - rebuilt_distances).^2));
    info_str = sprintf(['Line Index: %d\n' ...
        'RMS Error: %.4f mm'], ...
        rand_idx, rmse);
    
    annotation('textbox', [0.15, 0.02, 0.3, 0.1], ...
        'String', info_str, ...
        'FitBoxToText', 'on', ...
        'BackgroundColor', 'white');
end