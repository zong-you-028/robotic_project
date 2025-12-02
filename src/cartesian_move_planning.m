% cartesian_move_planning.m
% ========================================
% 笛卡爾空間軌跡規劃 (Cartesian Move)
% ========================================

function cartesian_move_planning(A, B, C, t_AB, t_BC, t_acc, dt)
    
    %% 提取位置和姿態
    p_A = A(1:3, 4); p_B = B(1:3, 4); p_C = C(1:3, 4);
    R_A = A(1:3, 1:3); R_B = B(1:3, 1:3); R_C = C(1:3, 1:3);
    
    %% 笛卡爾空間軌跡規劃
    fprintf('執行笛卡爾空間軌跡規劃...\n');
    [cart_AB, cart_vel_AB, cart_acc_AB, t_AB_vec] = plan_cartesian_trajectory(p_A, p_B, R_A, R_B, t_AB, t_acc, dt);
    [cart_BC, cart_vel_BC, cart_acc_BC, t_BC_vec] = plan_cartesian_trajectory(p_B, p_C, R_B, R_C, t_BC, t_acc, dt);
    
    % 合併軌跡
    cart_pos = [cart_AB.pos; cart_BC.pos(2:end, :)];
    cart_vel = [cart_vel_AB; cart_vel_BC(2:end, :)];
    cart_acc = [cart_acc_AB; cart_acc_BC(2:end, :)];
    cart_ori = [cart_AB.ori; cart_BC.ori(2:end, :, :)];
    
    t_AB_vec = t_AB_vec(:);
    t_BC_vec = t_BC_vec(:);
    t_all = [t_AB_vec; t_BC_vec(2:end) + t_AB];
    
    fprintf('軌跡點總數: %d\n', size(cart_pos, 1));
    
    %% 計算對應的關節軌跡 (使用IK)
    fprintf('計算關節空間軌跡...\n');
    n_points = size(cart_pos, 1);
    q_all = zeros(n_points, 6);
    qd_all = zeros(n_points, 6);
    qdd_all = zeros(n_points, 6);
    
    for i = 1:n_points
        % squeeze 確保維度為 3x3，避免 horzcat 錯誤
        R_curr = squeeze(cart_ori(i, :, :));
        T_target = [R_curr, cart_pos(i, :)' / 2.54; 0 0 0 1];
        
        sols = inverse_kinematics(T_target);
        if ~isempty(sols)
            if i == 1, q_all(i, :) = sols(1, :);
            else, q_all(i, :) = select_closest_solution(sols, q_all(i-1, :)); end
        else
            if i > 1, q_all(i, :) = q_all(i-1, :); end
        end
        
        if i > 1, qd_all(i, :) = (q_all(i, :) - q_all(i-1, :)) / dt; end
        if i > 2, qdd_all(i, :) = (qd_all(i, :) - qd_all(i-1, :)) / dt; end
    end
    
    %% 繪圖
    fprintf('繪製結果...\n');
    plot_cartesian_move_results(t_all, cart_pos, cart_vel, cart_acc, q_all, qd_all, qdd_all, A, B, C);
    
    %% 動畫
    fprintf('\n自動播放 3D 姿態動畫...\n');
    animate_robot_3d(q_all, t_all, dt, 'Cartesian Move - 3D Animation');
    
    fprintf('笛卡爾空間軌跡規劃完成！\n');
end

function [cart, vel, acc, t_vec] = plan_cartesian_trajectory(p_start, p_end, R_start, R_end, t_total, t_acc, dt)
    t_vec = (0:dt:t_total)'; n_points = length(t_vec);
    cart_pos = zeros(n_points, 3); cart_vel = zeros(n_points, 3);
    cart_acc = zeros(n_points, 3); cart_ori = zeros(n_points, 3, 3);
    for i = 1:n_points
        ti = t_vec(i);
        for j = 1:3
            [cart_pos(i, j), cart_vel(i, j), cart_acc(i, j)] = lspb_trajectory(p_start(j), p_end(j), t_total, t_acc, ti);
        end
        s = ti / t_total; cart_ori(i, :, :) = slerp_rotation(R_start, R_end, s);
    end
    cart.pos = cart_pos; cart.ori = cart_ori; vel = cart_vel; acc = cart_acc;
end

function [pos, vel, acc] = lspb_trajectory(q0, qf, tf, tacc, t)
    h = qf - q0; if tacc > tf/2, tacc = tf/2; end
    V = h / (tf - tacc); a = V / tacc;
    if t <= tacc
        pos = q0 + 0.5 * a * t^2; vel = a * t; acc = a;
    elseif t <= (tf - tacc)
        pos = q0 + V * (t - tacc/2); vel = V; acc = 0;
    else
        t_dec = t - (tf - tacc); pos = qf - 0.5 * a * (tacc - t_dec)^2; vel = V - a * t_dec; acc = -a;
    end
end

function R = slerp_rotation(R1, R2, t)
    R_rel = R1' * R2; [axis, angle] = rotation_to_axis_angle(R_rel);
    angle_interp = angle * t; R_interp = axis_angle_to_rotation(axis, angle_interp); R = R1 * R_interp;
end

function [axis, angle] = rotation_to_axis_angle(R)
    angle = acos((trace(R) - 1) / 2);
    if abs(angle) < 1e-6, axis = [0; 0; 1]; angle = 0;
    else, axis = 1/(2*sin(angle)) * [R(3,2)-R(2,3); R(1,3)-R(3,1); R(2,1)-R(1,2)]; end
end

function R = axis_angle_to_rotation(axis, angle)
    axis = axis / norm(axis); K = [0, -axis(3), axis(2); axis(3), 0, -axis(1); -axis(2), axis(1), 0];
    R = eye(3) + sin(angle) * K + (1 - cos(angle)) * K^2;
end

function q_best = select_closest_solution(solutions, q_prev)
    n_sols = size(solutions, 1); min_dist = inf; q_best = solutions(1, :);
    for i = 1:n_sols
        dist = norm(solutions(i, :) - q_prev);
        if dist < min_dist, min_dist = dist; q_best = solutions(i, :); end
    end
end