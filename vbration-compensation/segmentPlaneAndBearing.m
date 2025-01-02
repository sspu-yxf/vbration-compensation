function [plane_line_data, bearing_line_data] = segmentPlaneAndBearing( ...
    line_roi_data, circle_fit_threshold, line_fit_threshold, consecutive_limit)

    % % 获取线数
    % num_lines = length(line_roi_data);
    % 
    % % 初始化两部分点云
    % plane_line_data = cell(num_lines, 1);
    % bearing_line_data = cell(num_lines, 1);
    % 
    % for i = 1:num_lines
    %     % 当前线
    %     line = line_roi_data{i};
    %     y_data = line(:, 2);
    %     z_data = line(:, 3);
    % 
    %     % 去除 Z=0 的点
    %     valid_indices = ~ismember(1:length(z_data), zero_indices{i});
    %     y_valid = y_data(valid_indices);
    %     z_valid = z_data(valid_indices);
    % 
    %     % 初始化平面和轴承的点数据
    %     bearing_points = [];
    %     plane_points = [];
    % 
    %     % 曲线拟合检测
    %     consecutive_fails = 0;
    %     for j = 3:length(z_valid)-2
    %         if j < consecutive_limit, continue; end
    %         p = polyfit(y_valid(1:j), z_valid(1:j), 2);
    %         z_fit = polyval(p, y_valid(1:j));
    %         error = abs(z_fit - z_valid(1:j));
    % 
    %         if all(error <= circle_fit_threshold)
    %             bearing_points = line(valid_indices(1:j), :);
    %             consecutive_fails = 0;
    %         else
    %             consecutive_fails = consecutive_fails + 1;
    %             if consecutive_fails >= consecutive_limit, break; end
    %         end
    %     end
    % 
    %     % 直线拟合检测
    %     consecutive_fails = 0;
    %     for k = length(z_valid):-1:3
    %         if length(z_valid(k:end)) < consecutive_limit-1, continue; end
    %         p = polyfit(y_valid(k:end), z_valid(k:end), 1);
    %         z_fit = polyval(p, y_valid(k:end));
    %         error = abs(z_fit - z_valid(k:end));
    % 
    %         if all(error <= line_fit_threshold)
    %             plane_points = line(valid_indices(k:end), :);
    %             consecutive_fails = 0;
    %         else
    %             consecutive_fails = consecutive_fails + 1;
    %             if consecutive_fails >= consecutive_limit, break; end
    %         end
    %     end
    % 
    %     % 存储数据
    %     plane_line_data{i} = plane_points;
    %     bearing_line_data{i} = bearing_points(1:end-12, :);
    % end
    % 
    % 
    % 
    % % 获取线数
    % num_lines = length(line_roi_data);
    % 
    % % 初始化两部分点云
    % plane_line_data = cell(num_lines, 1);
    % bearing_line_data = cell(num_lines, 1);
    % 
    % for i = 1:num_lines
    %     % 当前线
    %     line = line_roi_data{i};
    %     y_data = line(:, 2);
    %     z_data = line(:, 3);
    % 
    %     % 去除 Z=0 的点，仅用于拟合
    %     valid_indices = ~ismember(1:length(z_data), zero_indices{i});
    %     y_valid = y_data(valid_indices);
    %     z_valid = z_data(valid_indices);
    % 
    %     % 初始化平面和轴承的点数据
    %     bearing_points = [];
    %     plane_points = [];
    % 
    %     % 曲线拟合检测
    %     consecutive_fails = 0;
    %     % 从小索引到大索引
    %     for j = 3:length(z_valid)-2
    %         if j < consecutive_limit, continue; end
    %         p = polyfit(y_valid(1:j), z_valid(1:j), 2);
    %         z_fit = polyval(p, y_valid(1:j));
    %         error = abs(z_fit - z_valid(1:j));
    % 
    %         if all(error <= circle_fit_threshold)
    %             % 选取 bearing_points 排除 Z=0 的点，但保留其在 line 中的结构
    %             bearing_points = line(valid_indices(1:j), :);
    %             consecutive_fails = 0;
    %         else
    %             consecutive_fails = consecutive_fails + 1;
    %             if consecutive_fails >= consecutive_limit, break; end
    %         end
    %     end
    % 
    %     % 直线拟合检测
    %     consecutive_fails = 0;
    %     % 从大索引到小索引
    %     for k = length(z_valid):-1:3
    %         if length(z_valid(k:end)) < consecutive_limit-1, continue; end
    %         p = polyfit(y_valid(k:end), z_valid(k:end), 1);
    %         z_fit = polyval(p, y_valid(k:end));
    %         error = abs(z_fit - z_valid(k:end));
    % 
    %         if all(error <= line_fit_threshold)
    %             % 选取 plane_points 时排除 Z=0 的点，保留有效的平面点
    %             plane_points = line(valid_indices(k:end), :);
    %             consecutive_fails = 0;
    %         else
    %             consecutive_fails = consecutive_fails + 1;
    %             if consecutive_fails >= consecutive_limit, break; end
    %         end
    %     end
    % 
    %     % 保留 Z=0 点的索引，仅在 bearing_line_data 中存在
    %     bearing_points_full = line;
    %     bearing_points_full(z_data == 0, :) = line(z_data == 0, :);
    % 
    %     % 存储结果， plane_points 中不存在 Z=0
    %     plane_line_data{i} = plane_points;
    %     bearing_line_data{i} = bearing_points_full(1:end-12, :);
    % end


    % 获取线数
    num_lines = length(line_roi_data);

    % 初始化两部分点云
    plane_line_data = cell(num_lines, 1);
    bearing_line_data = cell(num_lines, 1);

    for i = 1:num_lines
        % 当前线
        line = line_roi_data{i};
        y_data = line(:, 2);
        z_data = line(:, 3);

        % 初始化两点云
        bearing_points = [];
        plane_points = [];

        % 圆曲线拟合（从头到尾）
        consecutive_fails = 0;
        for j = 3:length(z_data)-2
            if j < consecutive_limit, continue; end
            p = polyfit(y_data(1:j), z_data(1:j), 2);
            z_fit = polyval(p, y_data(1:j));
            error = abs(z_fit - z_data(1:j));

            if all(error <= circle_fit_threshold)
                bearing_points = line(1:j, :);
                consecutive_fails = 0;
            else
                consecutive_fails = consecutive_fails + 1;
                if consecutive_fails >= consecutive_limit, break; end
            end
        end

        % 直线拟合（从尾到头）
        consecutive_fails = 0;
        for k = length(z_data):-1:3
            if length(z_data(k:end)) < consecutive_limit-1, continue; end
            p = polyfit(y_data(k:end), z_data(k:end), 1);
            z_fit = polyval(p, y_data(k:end));
            error = abs(z_fit - z_data(k:end));

            if all(error <= line_fit_threshold)
                plane_points = line(k:end, :);
                consecutive_fails = 0;
            else
                consecutive_fails = consecutive_fails + 1;
                if consecutive_fails >= consecutive_limit, break; end
            end
        end

        % 存储数据(剔除中间部分的冗余数据)
        bearing_line_data{i} = bearing_points(1:end-30, :);
        plane_line_data{i} = plane_points(8:end-8, :);

    end
end