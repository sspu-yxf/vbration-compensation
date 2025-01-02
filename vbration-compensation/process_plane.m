% 处理plane数据

clear;
clc;

% ****** 读取文件 ****** 
% % 实际数据
filename = 'nums01_times1.txt';
% % 单周期 + 随机白噪声
% filename = 'data/single_white_horizontal.txt';
% filename = 'data/single_white_slope.txt';
% % 多周期 + 随机白噪声
% filename = 'data/multi_white_horizontal.txt';
% filename = 'data/multi_white_slope.txt';
% filename = 'data/sim10.txt';
% % 随机白噪声
% filename = 'data/0_white_0.txt';
% filename = 'data/0_white_slope.txt';


data = readData(filename);
ptCloud_captured = pointCloud(data);
% figure;  pcshow(ptCloud_captured, "BackgroundColor", [1,1,1]); 


% ****** 根据数据特征 自动ROI ****** 
[data_roi_lines, num_vaild_lines, zero_indices, zero_points] = ...
    autoROIAndFilterLines(ptCloud_captured);

% 将线数据合并成矩阵
data_lines_roi = vertcat(data_roi_lines{:});
% 转换为点云
ptCloud_roi = pointCloud(data_lines_roi);

% % 可视化ROI点云
% figure; pcshow(pointCloud(vertcat(data_roi_lines{:})), ...
%    "BackgroundColor", [1,1,1]);
% xlabel("x axis"); ylabel("y axis");


% 定义拟合误差阈值
circle_fit_threshold = 0.2;  % 圆曲线拟合误差阈值
line_fit_threshold = 0.15;   % 斜直线拟合误差阈值
consecutive_limit = 5;       % 连续不符合条件的点的最大数量

% ******  根据曲线拟合结果分割平面和轴承部分 ****** 
[plane_line_data, bearing_line_data] = segmentPlaneAndBearing( ...
    data_roi_lines, ...
    circle_fit_threshold, line_fit_threshold, consecutive_limit);

% 合并数据 转换为点云
plane_data_original = vertcat(plane_line_data{:});
plane_ptCloud_Original = pointCloud(plane_data_original);
% 合并数据 转换为点云
bearing_data_original = vertcat(bearing_line_data{:});
bearing_ptCloud = pointCloud(bearing_data_original);

% 可视化平面和轴承点云
% figure;
% subplot(1, 2, 1);
% pcshow(plane_ptCloud_Original,  "BackgroundColor", [1,1,1]);
% title('Plane Point Cloud');
% subplot(1, 2, 2);
% pcshow(bearing_ptCloud_Original,  "BackgroundColor", [1,1,1]);
% title('Bearing Point Cloud');


% ******  执行自适应调整函数 ****** 
ideal_cylinder_line_data = adjustThetaToFitPointCloud( ...
    bearing_line_data);

% % 转化数据合并点云
% ideal_cylinder_data = vertcat(ideal_cylinder_line_data{:});
% ideal_cylinder_ptCloud = pointCloud(ideal_cylinder_data);
% 
% % 可视化理想圆柱体点云
% figure; pcshow(ideal_cylinder_ptCloud, "BackgroundColor", [1,1,1]); 


% ******  将理想圆柱体与轴承点云对齐（ICP） ****** 
[aligned_ideal_cylinder_line_data, tform, rmse] = ...
    pcaRotatedandICP(ideal_cylinder_line_data, bearing_ptCloud);
disp("rmse = "); disp(rmse);

% 将线数据合并成矩阵
aligned_ideal_cylinder_data = vertcat(aligned_ideal_cylinder_line_data{:});
% 转换为点云
aligned_ideal_cylinder_ptCloud = pointCloud(aligned_ideal_cylinder_data);

% % 可视化对齐后的点云
% figure;
% aligned_ideal_cylinder_ptCloud.Color = [0,0,0];
% bearing_ptCloud.Color = [0,1,0];
% pcshow(aligned_ideal_cylinder_ptCloud,"BackgroundColor", [1,1,1]);
% hold on;
% pcshow(bearing_ptCloud, "BackgroundColor", [1,1,1]); 
% hold off;
% title('Aligned Ideal Cylinder and Bearing Point Clouds');


% % 重建plane数据
rebuilt_plane_line_data = deNoise_only_guass(plane_line_data);
% rebuilt_plane_line_data = deNoise_only_laplacian(plane_line_data);
% rebuilt_plane_line_data = deNoise_guassLaplacian(plane_line_data);
% rebuilt_plane_line_data = deNoise_rebuild_plane(plane_line_data);



% 合并数据
rebuilt_plane_data = vertcat(rebuilt_plane_line_data{:});
% 转化为点云
rebuilt_plane_ptCloud = pointCloud(rebuilt_plane_data);

% % 可视化
% figure('Color', 'white');
% subplot(1,2,1);
% pcshow(plane_ptCloud_Original, 'BackgroundColor', [1,1,1]);
% title('Original Plane Point Cloud');
% subplot(1,2,2);
% pcshow(rebuilt_plane_ptCloud, 'BackgroundColor', [1,1,1]);
% title('Denoised Plane Point Cloud');

% % 可视化对比
% figure('Color', 'white');
% plane_ptCloud_Original.Color = [1,0,0];
% rebuilt_plane_ptCloud.Color = [0,0,1];
% pcshow(plane_ptCloud_Original,"BackgroundColor", [1,1,1]);
% hold on;
% pcshow(rebuilt_plane_ptCloud, "BackgroundColor", [1,1,1]); 
% legend('Original', 'Rebuilt');
% hold off;
% 

% 11. 计算并比较局部变异系数
cv_original = calcCV(plane_data_original(:, 3));
cv_rebuilt = calcCV(rebuilt_plane_data(:, 3));
disp('CV (原始): '); disp(num2str(cv_original));
disp('CV (重建): '); disp(num2str(cv_rebuilt));

 
% % 12. 计算平面点云和重建平面点云的内点百分比
% 
dist_leastSquares_threshold = 0.1;
% 
% % percentage_plane = calculateInlierPercentage_pcfitplane( ...
% %     plane_ptCloud, dist_leastSquares_threshold);
% % percentage_rebuilt = calculateInlierPercentage_pcfitplane( ...
% %     rebuilt_plane_ptCloud, dist_leastSquares_threshold);
percentage_plane = calculateInlierPercentage( ...
    plane_ptCloud_Original, dist_leastSquares_threshold);
percentage_rebuilt = calculateInlierPercentage( ...
    rebuilt_plane_ptCloud, dist_leastSquares_threshold);
disp('plane_ptCloud 内点百分比为：'); 
disp([num2str(percentage_plane), '%']);
disp('rebuilt_plane_ptCloud 内点百分比为：');
disp([num2str(percentage_rebuilt), '%']);


% % 设置不同的距离阈值范围
% threshold_values = 0.02:0.01:0.1;
% num_thresholds = length(threshold_values);
% 
% % 初始化数组存储每个阈值下的内点百分比
% percentage_plane_results = zeros(1, num_thresholds);
% percentage_rebuilt_results = zeros(1, num_thresholds);
% 
% % 循环计算不同阈值下的内点百分比
% for i = 1:num_thresholds
%     dist_leastSquares_threshold = threshold_values(i);
% 
%     % 计算平面点云和重建平面点云的内点百分比
%     percentage_plane_results(i) = calculateInlierPercentage( ...
%         pointCloud(vertcat(plane_line_data{:})), dist_leastSquares_threshold);
%     percentage_rebuilt_results(i) = calculateInlierPercentage( ...
%         rebuilt_plane_ptCloud, dist_leastSquares_threshold);
% end
% 
% % 打印结果
% disp('不同距离阈值下的内点百分比结果：');
% disp('dist_leastSquares_threshold     percentage_plane     percentage_rebuilt');
% for i = 1:num_thresholds
%     fprintf('%0.3f                        %0.2f%%                %0.2f%%\n', ...
%         threshold_values(i), percentage_plane_results(i), percentage_rebuilt_results(i));
% end
% 
% % 绘制图形
% figure('Color', 'white');
% plot(threshold_values, percentage_plane_results, '-o', ...
%     'LineWidth', 1.5, 'DisplayName', 'Plane Percentage');
% hold on;
% plot(threshold_values, percentage_rebuilt_results, '-o', ...
%     'LineWidth', 1.5, 'DisplayName', 'Rebuilt Plane Percentage');
% hold off;
% xlabel('dist\_leastSquares\_threshold');
% ylabel('Inlier Percentage (%)');
% title('Inlier Percentage Before and After Compensation');
% legend('Location', 'best');



% % 评估
% evaluate4(plane_line_data, rebuilt_plane_line_data);

% 可视化第3条激光线
% visualizeSpectrumComparison(plane_line_data, rebuilt_plane_line_data, 3);  

