function data = readData(filename)
    
    %     % 获取当前文件夹下的data目录
    %     dataPath = fullfile(pwd, 'data');
    % 
    %     % 确保data文件夹存在
    %     if ~exist(dataPath, 'dir')
    %         error('找不到data文件夹');
    %     end
    % 
    %     % 获取所有文件
    %     filePattern = fullfile(dataPath, '*_i_*_j*');
    %     files = dir(filePattern);
    % 
    %     % 检查是否找到文件
    %     if isempty(files)
    %         warning('没有找到符合格式的文件');
    %         return;
    %     end
    % 
    %     % 遍历处理每个文件
    %     for k = 1:length(files)
    %         fileName = files(k).name;
    %         filePath = fullfile(dataPath, fileName);
    % 
    %         % 从文件名中提取i和j的值
    %         % 假设文件名格式为: xxx_i_xxx_j
    %         tokens = regexp(fileName, '.*_(\d+)_.*_(\d+)', 'tokens');
    % 
    %         if ~isempty(tokens)
    %             i = str2double(tokens{1}{1});
    %             j = str2double(tokens{1}{2});
    % 
    %             try
    %                 % 读取文件内容
    %                 % 这里假设文件是文本文件，根据实际情况修改读取方式
    %                 fileID = fopen(filePath, 'r');
    %                 if fileID == -1
    %                     warning('无法打开文件: %s', fileName);
    %                     continue;
    %                 end
    % 
    %                 data = textscan(fileID, '%s');
    %                 fclose(fileID);
    % 
    %                 % 处理文件内容
    %                 processFile(data, i, j);
    % 
    %             catch ME
    %                 warning('处理文件时出错: %s\n错误信息: %s', fileName, ME.message);
    %                 if exist('fileID', 'var') && fileID ~= -1
    %                     fclose(fileID);
    %                 end
    %             end
    %         else
    %             warning('文件名格式不正确: %s', fileName);
    %         end
    %     end
    % end
    

    fid = fopen(filename, 'r');

    raw_data = textscan(fid, '%f,%f,%f');
    fclose(fid);
    data = cell2mat(raw_data); % 转换为矩阵
end


% function processFile(data, i, j)
%     % 这里添加具体的文件处理逻辑
%     fprintf('正在处理文件，i=%d, j=%d\n', i, j);
%     % ... 处理数据的代码 ...
% end