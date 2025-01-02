function cv = calcCV(data)

    % % 检查数据中是否包含 NaN
    % nan_indices = find(isnan(data));
    % 
    % % 检查数据中是否包含 Inf
    % inf_indices = find(isinf(data));
    % 
    % % 检查有效数值的数量
    % valid_count = sum(~isnan(data) & ~isinf(data));
    % 
    % % 打印基本统计信息
    % disp(['NaN的数量: ' num2str(length(nan_indices))]);
    % disp(['Inf的数量: ' num2str(length(inf_indices))]);
    % disp(['有效数值的数量: ' num2str(valid_count)]);


    mu = mean(data);
    sigma = std(data);
    cv = sigma / mu;
end