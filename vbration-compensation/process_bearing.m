% 处理bearing数据

clear;
clc;

% ****** 读取文件 ****** 
% % 实际数据
filename = 'data/nums01_times1.txt';
% % 单周期 + 随机白噪声
% filename = 'data/single_white_horizontal.txt';
% filename = 'data/single_white_slope.txt';
% % 多周期 + 随机白噪声
% filename = 'data/multi_white_horizontal.txt';
% filename = 'data/multi_white_slope.txt';
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

% 转化数据合并点云
ideal_cylinder_data = vertcat(ideal_cylinder_line_data{:});
ideal_cylinder_ptCloud = pointCloud(ideal_cylinder_data);

% % 可视化理想圆柱体点云
% figure; 
% pcshow(ideal_cylinder_ptCloud, "BackgroundColor", [1,1,1]); 
% axis off;


% ******  将理想圆柱体与轴承点云对齐（ICP） ****** 
[aligned_ideal_cylinder_line_data, tform, rmse] = ...
    pcaRotatedandICP(ideal_cylinder_line_data, bearing_ptCloud);
disp("rmse = "); disp(rmse);

% 将线数据合并成矩阵
aligned_ideal_cylinder_data = vertcat(aligned_ideal_cylinder_line_data{:});
% 转换为点云
aligned_ideal_cylinder_ptCloud = pointCloud(aligned_ideal_cylinder_data);

% % 可视化对齐后的点云
% figure('Color', 'white');
% aligned_ideal_cylinder_ptCloud.Color = [1,0,0];
% bearing_ptCloud.Color = [0,0,1];
% pcshow(aligned_ideal_cylinder_ptCloud,"BackgroundColor", [1,1,1]);
% hold on;
% pcshow(bearing_ptCloud, "BackgroundColor", [1,1,1]); 
% hold off;
% legend('bearing ptCloud', 'ideal cylinder ptCloud');
% title('Aligned Ideal Cylinder and Bearing Point Clouds');


% ****** 重建bearing数据 ******  
rebuilt_bearing_line_data = deNoise_rebuild_bearing( ...
    bearing_line_data, tform);

% 合并数据
rebuilt_bearing_data = vertcat(rebuilt_bearing_line_data{:});
% 转化为点云
rebuilt_bearing_ptCloud = pointCloud(rebuilt_bearing_data);

% % 可视化对比
% figure('Color', 'white');
% plane_ptCloud_Original.Color = [1,0,0];
% rebuilt_bearing_ptCloud.Color = [0,1,0];
% pcshow(plane_ptCloud_Original,"BackgroundColor", [1,1,1]);
% hold on;
% pcshow(rebuilt_bearing_ptCloud, "BackgroundColor", [1,1,1]); 
% hold off;


% % 11. 计算并比较局部变异系数
% cv_original = calcCV(bearing_data_original(:, 3));
% cv_rebuilt = calcCV(rebuilt_bearing_data(:, 3));
% disp('CV (原始): '); disp(num2str(cv_original));
% disp('CV (重建): '); disp(num2str(cv_rebuilt));


% % 评估
% evaluate4(bearing_line_data, rebuilt_bearing_line_data);


% % 可视化第3条激光线
% visualizeSpectrumComparison(bearing_line_data, rebuilt_bearing_line_data, 3);  
