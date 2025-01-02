function distances = calculatePointToCylinderDistances(bearing_line_data, R, t)
    % 圆柱体参数
    radius = 65;
    num_lines = length(bearing_line_data);
    distances = cell(num_lines, 1);

    % 计算参考圆柱曲面方程
    a = [1; 1; -radius^2];
    a_transformed = R' * a;  % 修改: 只选取a的前三个元素与R相乘
    y0 = 0;
    z0 = 0;
    r = sqrt(-a_transformed(3));

    for i = 1:num_lines
        line_points = bearing_line_data{i}; % 当前线数据
        num_points = size(line_points, 1);
        line_distances = zeros(num_points, 1); % 存储每个点到圆柱面距离

        for j = 1:num_points
            point = line_points(j, :)'; % 提取点坐标
            
            % 将点坐标乘以旋转矩阵R并加上平移向量t
            point_transformed = R * point + t;
            
            % 计算变换后的点到圆柱面的距离
            y = point_transformed(2);
            z = point_transformed(3);
            distance = abs(sqrt((y - y0)^2 + (z - z0)^2) - r);
            
            line_distances(j) = distance;
        end

        distances{i} = line_distances; % 存储当前线的距离
    end
end