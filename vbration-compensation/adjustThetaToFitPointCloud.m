function best_ideal_cylinder_line_data = adjustThetaToFitPointCloud( ...
    bearing_line_data)
    % 自适应调整函数，寻找最佳的 theta，使生成的标准点云的 Y 轴范围略大于目标值

    num_roi_lines = length(bearing_line_data);

    cylinder_radius = 130.0 / 2; % 半径 mm
    cylinder_height = 0.2 * num_roi_lines; % 高度 mm
    
    % 1. 计算 bearing_line_data 的参考 Y 轴量程范围
    y_ranges = zeros(num_roi_lines, 1);
    for i = 1:num_roi_lines
        line_y_data = bearing_line_data{i}(:, 2);
        y_ranges(i) = max(line_y_data) - min(line_y_data);
    end
    target_y_range = max(y_ranges); % 参考 Y 轴量程范围
    fprintf('参考的目标 Y 轴量程：%f \n', target_y_range);

    % 2. 初始化 theta 和步进
    theta = pi/30;       % 初始 theta 值
    theta_step = pi/360; % 步进值
    best_theta = theta;

    best_y_range_diff = inf; % 初始化最佳 Y 轴范围差异

    % 3. 自适应调整 theta 直到 ideal_cylinder_ptCloud 的 Y 轴量程略大于目标值
    for theta = theta : theta_step : pi/2
        % 生成理想点云
        ideal_cylinder_line_data = generateIdealCylinder( ...
            num_roi_lines, cylinder_radius, cylinder_height, theta);
        ideal_cylinder_data = vertcat(ideal_cylinder_line_data{:});

        % 计算生成点云的 Y 轴量程范围
        ideal_y_data = ideal_cylinder_data(:, 2);
        ideal_y_range = max(ideal_y_data) - min(ideal_y_data);

        % 检查 Y 轴量程是否略大于目标范围(多1+)
        y_range_diff = ideal_y_range - target_y_range - 0.15 ;

        if y_range_diff > 0
            best_ideal_cylinder_line_data = ideal_cylinder_line_data;
            fprintf('找到略大于目标的 Y 轴量程：%f，此时 theta：%f \n' ...
                , ideal_y_range, theta);
            return ;
        end

        % 如果当前 Y 轴量程接近目标，则更新最佳 theta
        if abs(y_range_diff) < best_y_range_diff
            best_y_range_diff = abs(y_range_diff);
            best_theta = theta;
            best_ideal_cylinder_line_data = ideal_cylinder_line_data;
        end
        
        % 增加 theta 值进行下一步迭代
        theta = theta + theta_step;
    end

    fprintf('未找到略大于目标的 Y 轴量程，但最优 theta：%f \n', best_theta);
end