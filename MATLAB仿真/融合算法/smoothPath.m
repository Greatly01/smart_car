function smoothedPath = smoothPath(path, obstacles, robotRadius)
    smoothedPath = path;
    if size(path, 1) > 3
        t = 1:size(path, 1);
        tFine = linspace(1, size(path, 1), 10 * size(path, 1));
        smoothedPath = interp1(t, path, tFine, 'spline'); % 样条插值
    end
end






% function smoothedPath = smoothPath(path, obstacles, robotRadius)
%     smoothedPath = path;
%     i = 1;
%     while i < size(smoothedPath, 1) - 1
%         % 直接跳过中间点的优化
%         if isCollisionFree(smoothedPath(i, :), smoothedPath(i + 2, :), obstacles, robotRadius)
%             smoothedPath(i + 1, :) = []; % 移除中间点
%         else
%             i = i + 1;
%         end
%     end
% end
