function [smoothness, curvature] = calculatePathSmoothness(path)
    curvature = []; % 存储曲率
    for i = 2:size(path, 1) - 1
        v1 = path(i, :) - path(i - 1, :);
        v2 = path(i + 1, :) - path(i, :);
        angle = acos(dot(v1, v2) / (norm(v1) * norm(v2))); % 计算夹角
        curvature = [curvature; abs(angle)]; %#ok<AGROW>
    end
    smoothness = sum(curvature); % 曲率总和
end








% function [smoothness, curvature] = calculatePathSmoothness(path)
%     curvature = []; % 存储每段路径的曲率
%     for i = 2:size(path, 1) - 1
%         v1 = path(i, :) - path(i - 1, :);
%         v2 = path(i + 1, :) - path(i, :);
%         angle = acos(dot(v1, v2) / (norm(v1) * norm(v2))); % 计算夹角
%         curvature = [curvature; abs(angle)]; %#ok<AGROW>
%     end
%     smoothness = sum(curvature); % 曲率总和衡量平滑度
% end
