function [filtered_line_data, num_valid_lines, zero_indices, zero_points] ...
    = autoROIAndFilterLines(ptCloud)
    % % 自动识别有效激光条并进行筛选
    %
    % % 点云数据，格式为 Nx3的矩阵
    % data = ptCloud.Location;
    %
    % % 根据X轴坐标的唯一值，划分每条激光线
    % x_roi_data = data(:, 1);
    % [unique_x_values, ~, ~] = unique(x_roi_data);
    % num_orignal = length(unique_x_values);
    %
    % % 初始化有效线条数据
    % line_data = cell(num_orignal, 1);
    % valid_line_indices = false(num_orignal, 1);
    % 
    % % 存储Z=0点的索引
    % zero_indices = cell(num_orignal, 4);
    % 
    % for i = 1:num_orignal
    %     % 获取当前激光线的点云数据
    %     line_indices = x_roi_data == unique_x_values(i);
    %     current_line_points = data(line_indices, :);
    % 
    %     % 提取Z轴数据
    %     current_z_data = current_line_points(:, 3);
    % 
    %     % 计算当前激光线的无效点数量（辨别Z=0的点）
    %     invalid_points = sum(current_z_data == 0);
    % 
    % 
    %     % 如果无效点数小于188，则标记为有效线
    %     if invalid_points < 188
    %         valid_line_indices(i) = true;
    % 
    %         % 提取每条线激光的 Z=0 的实际索引坐标
    %         zero_indices{i} (1,:)= [find(current_z_data == 0),];
    %     end
    % 
    %     % 暂存激光线点云数据
    %     line_data{i} = current_line_points;
    % end
    % 
    % % 获取有效激光线条的起始和结束索引（即 Idx1 和 Idx2）
    % valid_indices = find(valid_line_indices);
    % Idx1 = valid_indices(1);
    % Idx2 = valid_indices(end);
    % 
    % % 按要求调整有效激光线的范围
    % Idx1 = Idx1 + 1;
    % Idx2 = Idx2 - 1;
    % 
    % % 保留范围内的有效激光线条
    % filtered_line_data = line_data(Idx1:Idx2, :);
    % num_valid_lines = length(filtered_line_data);
    % % 仅保留有效线的Z=0索引
    % zero_indices = zero_indices(Idx1:Idx2); 



    % 自动识别有效激光条并筛选出Z=0的点并暂时删除

    % 点云数据
    data = ptCloud.Location;

    % 根据X轴坐标唯一值划分激光线
    x_roi_data = data(:, 1);
    [unique_x_values, ~, ~] = unique(x_roi_data);
    num_orignal = length(unique_x_values);

    % 初始化变量
    line_data = cell(num_orignal, 1);
    valid_line_indices = false(num_orignal, 1);

    % 初始化Z=0点的索引和坐标
    zero_indices = cell(num_orignal, 1);
    zero_points = cell(num_orignal, 1);

    for i = 1:num_orignal
        % 获取当前激光线数据
        line_indices = x_roi_data == unique_x_values(i);
        current_line_points = data(line_indices, :);

        % 提取Z轴数据并查找Z=0的点索引
        current_z_data = current_line_points(:, 3);
        z_zero_idx = find(current_z_data == 0);

        % 如果无效点数小于480则标记为有效线
        if length(z_zero_idx) < 480
            valid_line_indices(i) = true;

            % 保存 Z=0 的点的坐标
            zero_indices{i} = z_zero_idx;
            zero_points{i} = current_line_points(z_zero_idx, :);

            % 删除 Z=0 的点并保存处理后的激光线数据
            current_line_points(z_zero_idx, :) = [];
            line_data{i} = current_line_points;
        end
    end

    % 获取有效激光线的范围并调整
    valid_indices = find(valid_line_indices);
    Idx1 = valid_indices(1);
    Idx2 = valid_indices(end);
    Idx1 = Idx1 + 2;
    Idx2 = Idx2 - 2;

    % 返回有效范围内的激光线
    filtered_line_data = line_data(Idx1:Idx2);
    num_valid_lines = length(filtered_line_data);
    zero_indices = zero_indices(Idx1:Idx2);
    zero_points = zero_points(Idx1:Idx2);
end
