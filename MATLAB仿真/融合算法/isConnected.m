% isConnected 函数定义
function connected = isConnected(tree1, tree2, robotRadius)
    connected = false;
    for i = 1:size(tree1, 1)
        for j = 1:size(tree2, 1)
            if norm(tree1(i, :) - tree2(j, :)) < robotRadius
                connected = true;
                return;
            end
        end
    end
end
