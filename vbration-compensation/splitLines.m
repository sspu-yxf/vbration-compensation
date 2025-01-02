function [line_roi_data, num_roi_lines] = splitLines(data_roi)
    % 根据X数据特征划分激光线
    x_roi_data = data_roi(:, 1);
    [unique_x_values, ~, ~] = unique(x_roi_data);
    % 计算条数
    num_roi_lines = length(unique_x_values);    

    % 分割每条激光数据
    line_roi_data = cell(num_roi_lines, 1);
    for i = 1:num_roi_lines
        line_indices = x_roi_data == unique_x_values(i);
        line_roi_data{i} = data_roi(line_indices, :);
    end
end