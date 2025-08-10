% MATLAB代码：动态避障（DWA）
function [v_opt, omega_opt] = dwa_obstacle_avoidance(current_pose, goal, map, config)
    % 初始化速度和角速度范围
    v_range = config.v_min:config.v_step:config.v_max;
    omega_range = config.omega_min:config.omega_step:config.omega_max;
    max_score = -inf;
    v_opt = 0;
    omega_opt = 0;
    for v = v_range
        for omega = omega_range
            % 预测轨迹
            trajectory = predict_trajectory(current_pose, v, omega, config);
            % 计算评分
            heading = compute_heading(trajectory, goal);
            clearance = compute_clearance(trajectory, map);
            velocity = v;
            score = config.alpha * heading + config.beta * clearance - config.gamma * velocity;
            % 更新最优速度和角速度
            if score > max_score
                max_score = score;
                v_opt = v;
                omega_opt = omega;
            end
        end
    end
end