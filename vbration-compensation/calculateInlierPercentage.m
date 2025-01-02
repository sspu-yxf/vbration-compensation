function inlierPercentage = calculateInlierPercentage(pointCloud,dist_leastSquares_threshold)
    
    % 初始化计数
    inlierCount = 0;

    % 提取数据
    data = pointCloud.Location;
    totalPoints = size(data, 1);
    x_data = data(:,1);
    y_data = data(:,2);
    z_data = data(:,3);

    % 计算平面拟合值
    l = 0*x_data + 1;
    A = [x_data, y_data, l];
    param = pinv(A)*z_data;
    Z_fit = param(1)*x_data + param(2)*y_data + param(3);

    % 计算距离
    distances = abs(z_data - Z_fit);

    % 判断是否为内点
    inliers = find(distances <= dist_leastSquares_threshold);

    % 计算内点个数
    inlierCount = inlierCount + numel(inliers);

    % 计算内点百分比
    inlierPercentage = inlierCount / totalPoints * 100;





    
    % % 1. 平面拟合
    % [plane_model, ~] = pcfitplane(pointCloud,dist_leastSquares_threshold);
    % if isempty(plane_model)
    %     warning('平面拟合失败');
    %     inlierPercentage = 0;
    %     return;
    % end
    % 
    % % 2. 获取平面参数
    % normal = plane_model.Normal;
    % point = plane_model.Parameters(4) * normal;
    % 
    % % 3. 计算点到平面的距离
    % points = pointCloud.Location;
    % distances = abs((points - point) * normal');
    % 
    % % 4. 统计内点
    % num_inliers = sum(distances <= dist_leastSquares_threshold);
    % inlierPercentage = (num_inliers / pointCloud.Count) * 100;

end