function ideal_cylinder_line_data = generateIdealCylinder( ...
    num_cylinder_lines, cylinder_radius, cylinder_height, theta)

    % 生成理想的圆柱体数据，每条激光线生成弧度 [pi/2 - theta, pi/2 + theta]
    ideal_cylinder_line_data = cell(num_cylinder_lines, 1);
    
    % 定义圆周点数（2theta对应约2000个点）
    ideal_num_points_per_circle = round(2000);

    % 使用theta作为对称弧度范围
    ideal_circle_theta = linspace(pi/2 - theta, pi/2 + theta, ...
        ideal_num_points_per_circle);
    ideal_circle_y = cylinder_radius * cos(ideal_circle_theta);
    ideal_circle_z = cylinder_radius * sin(ideal_circle_theta);

    % 沿 X 轴生成每条线
    for i = 1:num_cylinder_lines
        ideal_x = ones(1, ideal_num_points_per_circle) * (i * 0.2);
        ideal_circle_points = [ideal_x', ideal_circle_y', ideal_circle_z'];
        ideal_cylinder_line_data{i} = ideal_circle_points;
    end

end