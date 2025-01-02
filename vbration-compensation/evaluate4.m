function evaluate4(plane_line_data, rebuilt_plane_line_data)
    % 初始化评估指标
    num_lines = length(plane_line_data);
    results = initializeResults(num_lines);
    
    % 计算所有指标
    results = calculateMetrics(plane_line_data, rebuilt_plane_line_data, results);
    
    % 显示结果（分为两部分）
    displayTableResults(results);  % 数值结果
    % visualizeResults(plane_line_data, rebuilt_plane_line_data);  % 图形分析
end

function results = initializeResults(num_lines)
    results = struct();
    % results.rmse_values = zeros(num_lines, 1);
    results.snr_values = zeros(num_lines, 2);
    % results.feature_scores = zeros(num_lines, 2);
    % results.smoothness_scores = zeros(num_lines, 2);
end

function results = calculateMetrics(plane_line_data, rebuilt_plane_line_data, results)
    num_lines = length(plane_line_data);
    
    for i = 1:num_lines
        z_orig = plane_line_data{i}(:,3);
        z_rebuilt = rebuilt_plane_line_data{i}(:,3);
        
        % RMSE计算
        % results.rmse_values(i) = sqrt(mean((z_orig - z_rebuilt).^2));
        
        % 使用小波分解计算SNR
        [snr_orig, snr_rebuild] = calculateWaveletSNR(z_orig, z_rebuilt);
        results.snr_values(i,:) = [snr_orig, snr_rebuild];
        
        % 特征保持评估
        % [feature_orig, feature_rebuilt] = evaluateFeaturePreservation(z_orig, z_rebuilt);
        % results.feature_scores(i,:) = [feature_orig, feature_rebuilt];
        
        % 平滑度评估
        % [smooth_orig, smooth_rebuilt] = evaluateSmoothness(z_orig, z_rebuilt);
        % results.smoothness_scores(i,:) = [smooth_orig, smooth_rebuilt];
    end
    
    % 计算最终指标
    results.final = struct( ...
    'snr', struct('original', mean(results.snr_values(:,1)) ,...
                     'rebuilt', mean(results.snr_values(:,2)) ));
end

function [snr_orig, snr_rebuild] = calculateWaveletSNR(z_orig, z_rebuilt)
    wname = 'db4';  % 选择小波基函数
    level = 4;      % 分解层数
    
    % 计算原始信号的SNR
    [c_orig, l_orig] = wavedec(z_orig, level, wname);
    clean_signal_orig = wrcoef('a', c_orig, l_orig, wname, level);
    noise_orig = z_orig - clean_signal_orig;
    snr_orig = 20 * log10(rms(clean_signal_orig)/rms(noise_orig));
    
    % 计算重建信号的SNR
    [c_rebuilt, l_rebuilt] = wavedec(z_rebuilt, level, wname);
    clean_signal_rebuilt = wrcoef('a', c_rebuilt, l_rebuilt, wname, level);
    noise_rebuilt = z_rebuilt - clean_signal_rebuilt;
    snr_rebuild = 20 * log10(rms(clean_signal_rebuilt)/rms(noise_rebuilt));
end

function [score_orig, score_rebuilt] = evaluateFeaturePreservation(z_orig, z_rebuilt)
    scales = [5, 15, 25];  % 多尺度分析的窗口大小
    score_orig = 0;
    score_rebuilt = 0;
    
    for scale = scales
        curv_orig = calculateLocalCurvature(z_orig, scale);
        curv_rebuilt = calculateLocalCurvature(z_rebuilt, scale);
        
        threshold = std(curv_orig) * 0.5;  % 自适应阈值
        features_orig = abs(curv_orig) > threshold;
        features_rebuilt = abs(curv_rebuilt) > threshold;
        
        score_orig = score_orig + sum(features_orig)/length(features_orig);
        score_rebuilt = score_rebuilt + sum(features_rebuilt)/length(features_rebuilt);
    end
    
    % 归一化得分
    score_orig = score_orig / length(scales);
    score_rebuilt = score_rebuilt / length(scales);
end

function curv = calculateLocalCurvature(z, window_size)
    % 计算局部曲率
    n = length(z);
    curv = zeros(n, 1);
    half_win = floor(window_size/2);
    
    for i = (half_win+1):(n-half_win)
        window = z((i-half_win):(i+half_win));
        p = polyfit((1:window_size)', window, 3);
        curv(i) = 2*p(1);  % 二阶导数系数
    end
    
    % 处理边界
    curv(1:half_win) = curv(half_win+1);
    curv(end-half_win+1:end) = curv(end-half_win);
end

function [smooth_orig, smooth_rebuilt] = evaluateSmoothness(z_orig, z_rebuilt)
    % 计算总变差
    tv_orig = sum(abs(diff(z_orig)));
    tv_rebuilt = sum(abs(diff(z_rebuilt)));
    
    % 计算局部方差
    window_size = 15;
    var_orig = movvar(z_orig, window_size);
    var_rebuilt = movvar(z_rebuilt, window_size);
    
    % 归一化组合得分
    smooth_orig = (tv_orig/length(z_orig) + mean(var_orig))/2;
    smooth_rebuilt = (tv_rebuilt/length(z_rebuilt) + mean(var_rebuilt))/2;
end

function displayTableResults(results)
    fprintf('\n=== 点云重建质量评估结果 ===\n\n');
    
    % 创建格式化的表格标题
    headers = {'指标', '原始数据', '重建数据', '改善比例(%)'};
    fprintf('%-20s %-15s %-15s %-15s\n', headers{:});
    fprintf(repmat('-', 1, 65));
    fprintf('\n');
    
    % SNR
    snr_imp = (results.final.snr.rebuilt - results.final.snr.original) / abs(results.final.snr.original) * 100;
    fprintf('%-20s %-15.2f %-15.2f %-15.1f\n', 'SNR (dB)', ...
        results.final.snr.original, results.final.snr.rebuilt, snr_imp);
    
    fprintf('\n');
end

function visualizeResults(plane_line_data, rebuilt_line_data)
    line_idx = ceil(length(plane_line_data)/2);
    z_orig = plane_line_data{line_idx}(:,3);
    z_rebuilt = rebuilt_line_data{line_idx}(:,3);
    
    % 创建图形窗口
    figure('Position', [100, 100, 1200, 800], 'Color', 'white');
    
    % 1. 相邻点差分分析
    subplot(2,2,1);
    plotDifferential(z_orig, z_rebuilt);
    
    % 2. 局部方差分析
    subplot(2,2,2);
    plotLocalVariance(z_orig, z_rebuilt);
    
    % 3. 局部曲率分析
    subplot(2,2,3);
    plotLocalCurvature(z_orig, z_rebuilt);
    
    % 4. 特征检测分析
    subplot(2,2,4);
    plotFeatureDetection(z_orig, z_rebuilt);
    
    % 添加总标题
    sgtitle('Original Data VS Reconstructed Data', 'FontSize', 14, 'FontWeight', 'bold');
end


function plotDifferential(z_orig, z_rebuilt)
    % 绘制相邻点差分

    plot(abs(diff(z_orig)), 'b-', 'LineWidth', 1.5);
    hold on;
    plot(abs(diff(z_rebuilt)), 'r--', 'LineWidth', 1.5);
    grid on;
    xlabel('Point Index', 'FontSize', 10);
    ylabel('|\DeltaZ|', 'FontSize', 10);
    title('Point-wise Differential Analysis', 'FontSize', 12);
    legend('Original', 'Reconstructed');
end

function plotLocalVariance(z_orig, z_rebuilt)
    % 计算并绘制局部方差

    window_size = 15;
    var_orig = movvar(z_orig, window_size);
    var_rebuilt = movvar(z_rebuilt, window_size);
    
    plot(var_orig, 'b-', 'LineWidth', 1.5);
    hold on;
    plot(var_rebuilt, 'r--', 'LineWidth', 1.5);
    grid on;
    xlabel('Point Index', 'FontSize', 10);
    ylabel('Local Variance', 'FontSize', 10);
    title('Local Variance Distribution', 'FontSize', 12);
    legend('Original', 'Reconstructed');
end

function plotLocalCurvature(z_orig, z_rebuilt)
    % 计算并绘制局部曲率

    window_size = 15;
    curv_orig = calculateLocalCurvature(z_orig, window_size);
    curv_rebuilt = calculateLocalCurvature(z_rebuilt, window_size);
    
    plot(curv_orig, 'b-', 'LineWidth', 1.5);
    hold on;
    plot(curv_rebuilt, 'r--', 'LineWidth', 1.5);
    grid on;
    xlabel('Point Index', 'FontSize', 10);
    ylabel('Curvature', 'FontSize', 10);
    title('Local Curvature Analysis', 'FontSize', 12);
    legend('Original', 'Reconstructed');
end

function plotFeatureDetection(z_orig, z_rebuilt)
    % 特征检测分析
    
    curv_orig = calculateLocalCurvature(z_orig, 15);
    curv_rebuilt = calculateLocalCurvature(z_rebuilt, 15);
    
    plot(abs(curv_orig), 'b-', 'LineWidth', 1.5);
    hold on;
    plot(abs(curv_rebuilt), 'r--', 'LineWidth', 1.5);
    grid on;
    xlabel('Point Index', 'FontSize', 10);
    ylabel('Feature Magnitude', 'FontSize', 10);
    title('Feature Detection Analysis', 'FontSize', 12);
    legend('Original', 'Reconstructed');
end