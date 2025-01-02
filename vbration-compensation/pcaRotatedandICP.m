function [aligned_cylinder_line_data, tform, rmse] = ...
    pcaRotatedandICP(ideal_cylinder_line_data, bearing_ptCloud)

    % 转化数据合并点云
    ideal_cylinder_data = vertcat(ideal_cylinder_line_data{:});
    ideal_cylinder_ptCloud = pointCloud(ideal_cylinder_data);

    % 计算条数
    num_lines = length(ideal_cylinder_line_data);

    % ICP对齐
    [tform, ~, rmse] = pcregistericp( ...
        ideal_cylinder_ptCloud, bearing_ptCloud, ...
        'MaxIterations', 500, ...        % 增加迭代次数
        'Tolerance', [0.0001, 0.001]);   % 设置更小的收敛容差以提高精度

    % 获取 tform 的旋转矩阵和位移向量
    R = tform.R;
    T = tform.Translation;

    % 初始化
    aligned_cylinder_line_data = cell(num_lines, 1);

    % 应用 tform 矩阵到每个点
    for i = 1:num_lines
        % 获取当前线条的数据
        current_line = ideal_cylinder_line_data{i};

        % 逐点应用旋转矩阵和位移向量
        transformed_line = (R * current_line')' + T;

        % 存储转换后的数据
        aligned_cylinder_line_data{i} = transformed_line;
    end
    



    % % Step 1: 使用PCA计算主轴方向
    % % [coeff_ideal, ~, ~] = pca(ideal_cylinder_ptCloud.Location);
    % [coeff_bearing, ~, ~] = pca(bearing_ptCloud.Location);
    % 
    % % Step 2: 确保主轴方向一致性
    % for i = 1:3
    %     if dot(coeff_ideal(:, i), coeff_bearing(:, i)) < 0
    %         coeff_ideal(:, i) = -coeff_ideal(:, i);
    %     end
    % end
    % 
    % % Step 3: 计算旋转矩阵并应用到点云
    % rotation_matrix = coeff_ideal * coeff_bearing';
    % rotated_ideal_cylinder_points = (rotation_matrix ...
    %     * ideal_cylinder_ptCloud.Location')';
    % ideal_cylinder_ptCloud = pointCloud( ...
    %     rotated_ideal_cylinder_points);
    % 
    % 
    % % ICP对齐
    % [tform, ~, rmse] = pcregistericp( ...
    %     ideal_cylinder_ptCloud, bearing_ptCloud, ...
    %     'MaxIterations', 500, ...        % 增加迭代次数
    %     'Tolerance', [0.0001, 0.001]);   % 设置更小的收敛容差以提高精度




    % % 转化并合并理想圆柱点云
    % ideal_cylinder_data = vertcat(ideal_cylinder_line_data{:});
    % ideal_cylinder_ptCloud = pointCloud(ideal_cylinder_data);
    % 
    % % 计算条数
    % num_lines = length(ideal_cylinder_line_data);
    % 
    % % --- PCA初步对齐 ---
    % % 1. 对理想圆柱点云应用PCA，找到其主轴方向
    % [coeff_ideal, ~, ~] = pca(ideal_cylinder_data);
    % axis_ideal = coeff_ideal(:, 1);  % 主轴方向
    % 
    % % 2. 对轴承点云应用PCA，找到其主轴方向
    % bearing_data = bearing_ptCloud.Location;
    % [coeff_bearing, ~, ~] = pca(bearing_data);
    % axis_bearing = coeff_bearing(:, 1);  % 主轴方向
    % 
    % % 3. 计算两个主轴之间的旋转矩阵
    % v = cross(axis_ideal, axis_bearing);
    % s = norm(v);
    % c = dot(axis_ideal, axis_bearing);
    % vx = [  0   -v(3)  v(2);
    %        v(3)   0   -v(1);
    %       -v(2)  v(1)   0];
    % R_initial = eye(3) + vx + vx^2 * ((1 - c) / s^2);
    % 
    % % 4. 初步旋转理想圆柱点云
    % rotated_ideal_cylinder_data = (R_initial * ideal_cylinder_data')';
    % rotated_ideal_cylinder_ptCloud = pointCloud(rotated_ideal_cylinder_data);
    % 
    % % --- ICP精细对齐 ---
    % [tform, ~, rmse] = pcregistericp( ...
    %     rotated_ideal_cylinder_ptCloud, bearing_ptCloud, ...
    %     "InitialTransform", affine3d([R_initial [0;0;0]; 0 0 0 1]), ...
    %     'MaxIterations', 50, ...
    %     'Tolerance', [0.0001, 0.001]);
    % 
    % % 最终旋转矩阵和位移向量
    % R = tform.R;
    % T = tform.Translation;
    % 
    % % 初始化输出
    % aligned_cylinder_line_data = cell(num_lines, 1);
    % 
    % % 将转换应用到每条线段
    % for i = 1:num_lines
    %     current_line = ideal_cylinder_line_data{i};
    %     transformed_line = (R * current_line')' + T;
    %     aligned_cylinder_line_data{i} = transformed_line;
    % end

end
