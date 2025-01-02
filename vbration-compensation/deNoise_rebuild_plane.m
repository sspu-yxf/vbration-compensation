function rebuilt_plane_line_data = deNoise_rebuild_plane(plane_line_data)
    
    % 构建平面点云的噪声模型
    num_lines = length(plane_line_data);
    rebuilt_plane_line_data = cell(num_lines, 1);
    
    % 合并所有点云数据用于平面拟合
    all_points = vertcat(plane_line_data{:});
    
    % 使用RANSAC进行平面拟合
    [plane_model, ~] = pcfitplane(pointCloud(all_points), 0.1);
    plane_normal = (-1) * plane_model.Normal;
    if(plane_model.Parameters(4) < 0)
        plane_normal = (-1) * plane_normal;  % 翻转法向
    end
    plane_point = plane_model.Parameters(4) * plane_normal;    
    
    % % 输出拟合信息
    % fprintf('Plane normal: [%.4f, %.4f, %.4f]\n', plane_normal(1), plane_normal(2), plane_normal(3));
    % fprintf('Plane point: [%.4f, %.4f, %.4f]\n', plane_point(1), plane_point(2), plane_point(3));
    
    % 初始化噪声模型结构体
    noise_model = initialize_noise_model(num_lines);
    
    % 逐行处理
    for line_idx = 1:num_lines
        current_line = plane_line_data{line_idx};
        
        if size(current_line, 1) > 1
            % 计算相对于平面的有符号距离
            vec_to_plane = current_line - repmat(plane_point, size(current_line, 1), 1);
            distances = dot(vec_to_plane, repmat(plane_normal, size(current_line, 1), 1), 2);
            
            % % 计算并打印距离统计信息
            % if line_idx == 1
            %     fprintf('Line %d distances - Mean: %.4f, Std: %.4f, Range: [%.4f, %.4f]\n', ...
            %         line_idx, mean(distances), std(distances), min(distances), max(distances));
            % end
            
            % 提取周期特征和平滑处理
            periodic_features = extract_periodic_features(distances);
            smooth_features = perform_enhanced_smoothing(distances, periodic_features);
            
            % 保存特征
            noise_model.periodic{line_idx} = periodic_features;
            noise_model.smooth{line_idx} = smooth_features;
            
            % 重建点云时保持原始点到平面的投影位置
            projected_points = current_line - distances * plane_normal;
            rebuilt_points = projected_points + smooth_features.smoothed * plane_normal;
            rebuilt_plane_line_data{line_idx} = rebuilt_points;
        else
            rebuilt_plane_line_data{line_idx} = current_line;
        end
    end
    
    % 可视化分析
    visualize_random_line_analysis(noise_model, plane_line_data, plane_normal, plane_point);
end

function distances = calculate_plane_distances(points, plane_normal, plane_point)
    % 计算点到平面的有符号距离, 确保距离符号一致: 平面上方为正，下方为负
    distances = (points - plane_point) * plane_normal';
end

function visualize_random_line_analysis(noise_model, plane_line_data, plane_normal, plane_point)
    % 随机选择一行激光数据进行可视化分析
    
    % 获取有效行数
    valid_lines = find(~cellfun(@isempty, noise_model.periodic));
    
    % 随机选择一行
    rand_idx = valid_lines(randi(length(valid_lines)));
    
    % 获取该行的数据
    current_line = plane_line_data{rand_idx};
    distances = calculate_plane_distances(current_line, plane_normal, plane_point);
    periodic_features = noise_model.periodic{rand_idx};
    smooth_features = noise_model.smooth{rand_idx};
    
    % 创建图形窗口
    figure('Color', 'white', 'Position', [100, 100, 800, 600]);
    
    % 绘制周期分析结果
    subplot(2,1,1);
    plot(distances, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Original');
    hold on;
    plot(periodic_features.signal, 'r--', 'LineWidth', 1.5, ...
        'DisplayName', 'Periodic');
    title(sprintf('Original vs Periodic Signal (Line %d)', rand_idx));
    legend('Location', 'best');
    grid on;
    xlabel('Point Index');
    ylabel('Distance (mm)');
    
    % 绘制平滑结果
    subplot(2,1,2);
    plot(distances, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Original');
    hold on;
    plot(smooth_features.smoothed, 'r-', 'LineWidth', 1.5, ...
        'DisplayName', 'Smoothed');
    title(sprintf('Original vs Smoothed Signal (Line %d)', rand_idx));
    legend('Location', 'best');
    grid on;
    xlabel('Point Index');
    ylabel('Distance (mm)');
    
    % 添加统计信息
    rmse = sqrt(mean((distances - smooth_features.smoothed).^2));
    info_str = sprintf(['Line Index: %d\n' ...
        'Periodic Components: %d\n' ...
        'RMS Error: %.4f mm'], ...
        rand_idx, ...
        length(periodic_features.frequencies), ...
        rmse);
    
    annotation('textbox', [0.15, 0.02, 0.3, 0.1], ...
        'String', info_str, ...
        'FitBoxToText', 'on', ...
        'BackgroundColor', 'white');
end

% 主要函数实现
function model = initialize_noise_model(num_lines)
    model = struct();
    model.periodic = cell(num_lines, 1);
    model.smooth = cell(num_lines, 1);
end

function periodic_features = extract_periodic_features(distances)
    % 从距离序列中提取周期特征

    params = get_model_parameters();
    
    % FFT分析
    [freq_spectrum, freq_info] = analyze_frequency_spectrum(distances);
    
    % 选择主要频率分量
    [main_freqs, main_amps, main_phases] = select_main_components(...
        freq_spectrum, freq_info, distances, params);
    
    % 重建周期信号
    L = length(distances);
    periodic_signal = reconstruct_periodic_signal(...
        L, main_freqs, main_amps, main_phases);
    
    % 构建特征结构体
    periodic_features = struct(...
        'frequencies', main_freqs, ...
        'amplitudes', main_amps, ...
        'phases', main_phases, ...
        'signal', periodic_signal ...
    );
end

function smooth_features = perform_enhanced_smoothing(distances, periodic_features)
    % 增强平滑处理
    
    params = get_model_parameters();
    
    % 计算局部特征
    local_features = analyze_local_features(distances, periodic_features.signal);
    
    % 自适应窗口平滑
    smoothed_data = adaptive_window_smoothing(...
        distances, periodic_features.signal, local_features, params);
    
    % 拉普拉斯增强
    enhanced_data = laplacian_enhancement(...
        smoothed_data, periodic_features.signal, params);

    % 找到NaN值并用原始数据替换
    nan_indices = isnan(enhanced_data);
    if any(nan_indices)
        enhanced_data(nan_indices) = distances(nan_indices);
    end
    
    % 构建特征结构体
    smooth_features = struct(...
        'smoothed', enhanced_data, ...
        'reference', periodic_features.signal, ...
        'local_features', local_features ...
    );
end


%% ===== 配置参数定义 =====
function model_params = get_model_parameters()
    % 周期特征提取参数
    model_params.target_similarity = 0.85;    % 目标相似度
    model_params.max_components = 15;         % 最大频率分量数
    model_params.min_improvement = 0.02;      % 最小改进阈值
    model_params.max_no_improvement = 3;      % 最大无改进次数
    
    % 平滑处理参数
    model_params.window_sizes = struct(...
        'small', [21, 61], ...         % 变化剧烈区域窗口范围
        'medium', [61, 120], ...       % 过渡区域窗口范围
        'large', [121, 301] ...        % 平稳区域窗口范围
    );
    
    model_params.variation_thresholds = [0.3, 0.7];  % 区域划分阈值
    model_params.laplacian_iterations = 5;           % 拉普拉斯平滑迭代次数
    model_params.global_smoothing_alpha = 0.5;      % 全局平滑因子
    
    % 相似度评估参数
    model_params.similarity_weights = struct(...
        'corr', 0.4, ...              % 相关性权重
        'amplitude', 0.3, ...         % 幅值权重
        'trend', 0.3 ...             % 趋势权重
    );
end

%% ===== 频谱分析实现函数 =====
function [freq_spectrum, freq_info] = analyze_frequency_spectrum(signal)
    % 对信号进行频谱分析，获取频率信息

    L = length(signal);
    Y = fft(signal);
    
    % 计算双边频谱
    P2 = abs(Y/L);
    % 转换为单边频谱
    freq_spectrum = P2(1:floor(L/2+1));
    freq_spectrum(2:end-1) = 2*freq_spectrum(2:end-1);
    
    % 保存频率信息
    freq_info = struct();
    freq_info.freqs = (0:(L/2))/L;           % 频率值
    freq_info.phases = angle(Y(1:floor(L/2+1))); % 相位信息
end

function [main_freqs, main_amps, main_phases] = select_main_components(...
    freq_spectrum, freq_info, original_signal, params)
    % 选择主要频率分量
    
    % 按幅值排序
    [sorted_amps, idx] = sort(freq_spectrum, 'descend');
    sorted_freqs = freq_info.freqs(idx);
    sorted_phases = freq_info.phases(idx);
    
    % 迭代选择频率分量
    L = length(original_signal);
    prev_similarity = 0;
    no_improvement_count = 0;
    
    % 预分配数组
    main_freqs = zeros(params.max_components, 1);
    main_amps = zeros(params.max_components, 1);
    main_phases = zeros(params.max_components, 1);
    
    for i = 1:params.max_components
        % 添加新分量
        main_freqs(i) = sorted_freqs(i);
        main_amps(i) = sorted_amps(i);
        main_phases(i) = sorted_phases(i);
        
        % 重建信号并评估
        reconstructed = reconstruct_periodic_signal(...
            L, main_freqs(1:i), main_amps(1:i), main_phases(1:i));
        
        current_similarity = evaluate_signal_similarity(...
            original_signal, reconstructed, params.similarity_weights);
        
        % 检查改进程度
        improvement = current_similarity - prev_similarity;
        if improvement < params.min_improvement
            no_improvement_count = no_improvement_count + 1;
        else
            no_improvement_count = 0;
        end
        
        % 判断是否达到目标
        if (current_similarity >= params.target_similarity) || ...
           (no_improvement_count >= params.max_no_improvement)
            % 只保留有效分量
            main_freqs = main_freqs(1:i);
            main_amps = main_amps(1:i);
            main_phases = main_phases(1:i);
            break;
        end
        
        prev_similarity = current_similarity;
    end
end

%% ===== 信号重建和评估函数 =====
function reconstructed = reconstruct_periodic_signal(L, freqs, amps, phases)
    % 重建周期信号

    time_points = (0:L-1)';
    reconstructed = zeros(L, 1);
    
    % 叠加各频率分量
    for k = 1:length(freqs)
        reconstructed = reconstructed + ...
            amps(k) * cos(2*pi*freqs(k)*time_points + phases(k));
    end
end

function similarity = evaluate_signal_similarity(original, reconstructed, weights)
    % 评估信号相似度
    
    % 计算相关性
    correlation = corrcoef(original, reconstructed);
    corr_score = abs(correlation(1,2));
    
    % 计算幅值差异
    amplitude_diff = 1 - mean(abs(original - reconstructed)) / ...
        (max(original) - min(original));
    
    % 计算趋势一致性
    trend_score = calculate_trend_consistency(original, reconstructed);
    
    % 加权组合
    similarity = weights.corr * corr_score + ...
                weights.amplitude * amplitude_diff + ...
                weights.trend * trend_score;
end

%% ===== 平滑处理实现函数 =====
function local_features = analyze_local_features(signal, reference)
    % 分析信号的局部特征
    
    window_size = 10;  % 局部分析窗口大小
    
    local_features = struct();
    local_features.diff = abs(signal - reference);
    local_features.gradient = gradient(signal);
    local_features.variance = movvar(signal, window_size);
    local_features.variation_score = calculate_variation_score(local_features);
end

function smoothed = adaptive_window_smoothing(...
    signal, reference, local_features, params)
    % 自适应窗口平滑处理
    
    L = length(signal);
    smoothed = zeros(L, 1);
    
    for i = 1:L
        % 确定窗口大小
        window_size = determine_window_size(...
            local_features.variation_score(i), params.window_sizes, ...
            params.variation_thresholds);
        
        % 计算窗口范围
        half_win = floor(window_size/2);
        start_idx = max(1, i-half_win);
        end_idx = min(L, i+half_win);
        indices = start_idx:end_idx;
        
        % 计算权重并平滑
        weights = calculate_weights(...
            signal(indices), reference(indices), i-start_idx+1, length(indices));
        
        smoothed(i) = sum(signal(indices) .* weights);
    end
end

function enhanced = laplacian_enhancement(smoothed, reference, params)
    % 拉普拉斯增强处理
    
    L = length(smoothed);
    enhanced = smoothed;
    alpha = params.global_smoothing_alpha;
    
    % 迭代增强
    for iter = 1:params.laplacian_iterations
        temp_data = enhanced;
        for i = 2:L-1
            % 计算拉普拉斯修正
            laplacian = temp_data(i+1) - 2*temp_data(i) + temp_data(i-1);
            ref_trend = (reference(i+1) - 2*reference(i) + reference(i-1)) / 2;
            
            % 应用修正
            enhanced(i) = temp_data(i) + alpha * (laplacian - ref_trend);
        end
        
        % 处理边界
        enhanced([1,L]) = enhanced([2,L-1]);
    end
end

%% ===== 辅助函数 =====
function trend_score = calculate_trend_consistency(original, reconstructed)
    % 计算趋势一致性得分

    diff_orig = diff(original);
    diff_recon = diff(reconstructed);
    trend_score = mean(sign(diff_orig) == sign(diff_recon));
end

function variation_score = calculate_variation_score(features)
    % 计算变化程度得分

    norm_diff = features.diff / mean(features.diff);
    norm_grad = abs(features.gradient) / mean(abs(features.gradient));
    variation_score = (norm_diff + norm_grad) / 2;
end

function window_size = determine_window_size(...
    variation_score, window_sizes, thresholds)
    % 确定自适应窗口大小
    
    if variation_score > thresholds(2)
        % 变化剧烈区域
        range = window_sizes.small;
    elseif variation_score > thresholds(1)
        % 过渡区域
        range = window_sizes.medium;
    else
        % 平稳区域
        range = window_sizes.large;
    end
    
    % 计算具体窗口大小
    window_size = range(1) + round((range(2) - range(1)) * ...
        (1 - min(1, variation_score)));
    
    % 确保为奇数
    window_size = 2 * floor(window_size/2) + 1;
end

function weights = calculate_weights(window_data, reference, center_pos, window_length)
    % 计算加权平均的权重
    
    % 距离权重
    sigma = window_length/6;
    x = 1:window_length;
    dist_weights = exp(-(x - center_pos).^2 / (2*sigma^2));
    
    % 参考信号偏差权重
    dev_weights = exp(-abs(window_data - reference).^2 / ...
        (2 * std(window_data - reference)^2));
    
    % 组合权重
    weights = dist_weights' .* dev_weights;
    weights = weights / sum(weights);
end