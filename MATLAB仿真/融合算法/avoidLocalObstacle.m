function nextPos = avoidLocalObstacle(currentPos, nextPos, obstacles, robotRadius)
    % 简单调整方向避开障碍物
    direction = nextPos - currentPos;
    for theta = linspace(-pi / 4, pi / 4, 10)
        rotatedDir = [cos(theta), -sin(theta); sin(theta), cos(theta)] * direction';
        candidatePos = currentPos + rotatedDir';
        if isPathSafe(currentPos, candidatePos, obstacles, robotRadius)
            nextPos = candidatePos;
            return;
        end
    end
end

function safe = isPathSafe(point1, point2, obstacles, robotRadius)
    % 路径碰撞检测
    safe = true;
    for t = 0:0.1:1
        interpPoint = point1 + t * (point2 - point1);
        if min(vecnorm(obstacles - interpPoint, 2, 2)) < robotRadius
            safe = false;
            return;
        end
    end
end
