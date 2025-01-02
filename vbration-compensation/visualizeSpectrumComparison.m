function visualizeSpectrumComparison(original_data, rebuilt_data, line_index)
    % 获取指定行的数据
    original_line = original_data{line_index};
    rebuilt_line = rebuilt_data{line_index};

    % 提取Z值并去除趋势
    z_original = detrend(original_line(:, 3));
    z_rebuilt = detrend(rebuilt_line(:, 3));

    % 设置频谱分析参数
    fs = 1;                     % 采样频率
    window_size = 256;          % 窗口长度
    noverlap = window_size/2;   % 重叠长度
    nfft = max(256, 2^nextpow2(window_size));  % FFT点数
    window = hann(window_size); % Hanning窗

    % 使用Welch方法计算功率谱密度
    [pxx_original, f] = pwelch(z_original, window, noverlap, nfft, fs);
    [pxx_rebuilt, ~] = pwelch(z_rebuilt, window, noverlap, nfft, fs);

    % 创建图形窗口
    figure('Position', [100, 100, 800, 800], 'Color', 'white');

    % 绘制功率谱密度对比（对数刻度）
    semilogy(f, pxx_original, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Original');
    hold on;
    semilogy(f, pxx_rebuilt, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Rebuilt');
    hold off;

    % 设置图形属性
    title(sprintf('Power Spectral Density Comparison (Line %d)', line_index), ...
          'FontSize', 12, 'FontWeight', 'bold');
    xlabel('Frequency (Hz)', 'FontSize', 11);
    ylabel('Power/Frequency (dB/Hz)', 'FontSize', 11);
    legend('Location', 'best', 'FontSize', 10);

    % 设置坐标轴范围和刻度
    xlim([0, 0.3]);  % 限制频率范围在0-0.3Hz
    ax = gca;
    ax.XGrid = 'on';
    ax.YGrid = 'on';
    ax.GridLineStyle = ':';
    ax.GridAlpha = 0.3;


   % % 幅度衰减计算
   % attenuation = 10 * log10(mean(pxx_original) ./ mean(pxx_rebuilt));
   % disp('improved : '); disp(attenuation);
end