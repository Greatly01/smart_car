function turnCount = calculateTurnCount(path)
    turnCount = 0;
    for i = 2:size(path, 1) - 1
        v1 = path(i, :) - path(i - 1, :);
        v2 = path(i + 1, :) - path(i, :);
        angle = acos(dot(v1, v2) / (norm(v1) * norm(v2)));
        if abs(angle) > pi / 6 % 超过 30 度视为一次转向
            turnCount = turnCount + 1;
        end
    end
end










% function turnCount = calculateTurnCount(path)
%     turnCount = 0;
%     for i = 2:size(path, 1) - 1
%         v1 = path(i, :) - path(i - 1, :);
%         v2 = path(i + 1, :) - path(i, :);
%         angle = acos(dot(v1, v2) / (norm(v1) * norm(v2)));
%         if abs(angle) > pi / 6 % 超过30度算一次转向
%             turnCount = turnCount + 1;
%         end
%     end
% end
