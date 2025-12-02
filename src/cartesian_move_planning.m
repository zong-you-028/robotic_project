% cartesian_move_planning.m
% ========================================
% 笛卡爾空間軌跡規劃 (Cartesian Move)
% ========================================
% 輸入:
%   A, B, C: 目標姿態矩陣 (4x4, 單位: cm)
%   t_AB, t_BC: 移動時間 (sec)
%   t_acc: 過渡段加速時間 (sec)
%   dt: 採樣時間 (sec)
% ========================================

function cartesian_move_planning(A, B, C, t_AB, t_BC, t_acc, dt)
    
    %% 提取位置和姿態
    % 位置 (cm)
    p_A = A(1:3, 4);
    p_B = B(1:3, 4);
    p_C = C(1:3, 4);
    
    % 旋轉矩陣
    R_A = A(1:3, 1:3);
    R_B = B(1:3, 1:3);
    R_C = C(1:3, 1:3);
    
    %% 笛卡爾空間軌跡規劃
    fprintf('執行笛卡爾空間軌跡規劃...\n');
    
    % 規劃 A -> B 段
    [cart_AB, cart_vel_AB, cart_acc_AB, t_AB_vec] = ...
        plan_cartesian_trajectory(p_A, p_B, R_A, R_B, t_AB, t_acc, dt);
    
    % 規劃 B -> C 段
    [cart_BC, cart_vel_BC, cart_acc_BC, t_BC_vec] = ...
        plan_cartesian_trajectory(p_B, p_C, R_B, R_C, t_BC, t_acc, dt);
    
    % 合併軌跡
    cart_pos = [cart_AB.pos; cart_BC.pos(2:end, :)];
    cart_vel = [cart_vel_AB; cart_vel_BC(2:end, :)];
    cart_acc = [cart_acc_AB; cart_acc_BC(2:end, :)];
    cart_ori = [cart_AB.ori; cart_BC.ori(2:end, :, :)];
    
    % 確保時間向量是列向量
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
    
    % 單位轉換: cm -> inch
    for i = 1:n_points
        T_target = [cart_ori(i, :, :), cart_pos(i, :)' / 2.54;
                    0 0 0 1];
        T_target = squeeze(T_target);
        
        % 求解IK
        sols = inverse_kinematics(T_target);
        
        if ~isempty(sols)
            % 選擇與前一點最接近的解（軌跡連續性）
            if i == 1
                q_all(i, :) = sols(1, :);
            else
                q_all(i, :) = select_closest_solution(sols, q_all(i-1, :));
            end
        else
            warning('點 %d 的IK無解，使用前一點。', i);
            if i > 1
                q_all(i, :) = q_all(i-1, :);
            end
        end
        
        % 計算關節速度和加速度（數值微分）
        if i > 1
            qd_all(i, :) = (q_all(i, :) - q_all(i-1, :)) / dt;
        end
        if i > 2
            qdd_all(i, :) = (qd_all(i, :) - qd_all(i-1, :)) / dt;
        end
    end
    
    %% 繪圖
    fprintf('繪製結果...\n');
    plot_cartesian_move_results(t_all, cart_pos, cart_vel, cart_acc, ...
                                 q_all, qd_all, qdd_all, A, B, C);
    
    %% 動畫
    fprintf('\n是否要播放動畫? (1=基本動畫, 2=3D姿態動畫, 0=不播放): ');
    user_input = input('');
    if user_input == 1
        fprintf('準備基本動畫...\n');
        animate_robot(q_all, t_all, dt, 'Cartesian Move Animation');
    elseif user_input == 2
        fprintf('準備3D姿態動畫...\n');
        animate_robot_3d(q_all, t_all, dt, 'Cartesian Move - 3D Animation');
    end
    
    fprintf('笛卡爾空間軌跡規劃完成！\n');
end


%% ========================================
%% 子函式: 單段笛卡爾軌跡規劃
%% ========================================
function [cart, vel, acc, t_vec] = plan_cartesian_trajectory(p_start, p_end, R_start, R_end, t_total, t_acc, dt)
    
    t_vec = (0:dt:t_total)';  % 生成列向量
    n_points = length(t_vec);
    
    % 位置軌跡 (x, y, z)
    cart_pos = zeros(n_points, 3);
    cart_vel = zeros(n_points, 3);
    cart_acc = zeros(n_points, 3);
    cart_ori = zeros(n_points, 3, 3);
    
    for i = 1:n_points
        ti = t_vec(i);
        
        % 位置插值 (使用 LSPB)
        for j = 1:3
            [cart_pos(i, j), cart_vel(i, j), cart_acc(i, j)] = ...
                lspb_trajectory(p_start(j), p_end(j), t_total, t_acc, ti);
        end
        
        % 姿態插值 (使用 SLERP - 球面線性插值)
        s = ti / t_total;  % 插值參數 [0, 1]
        cart_ori(i, :, :) = slerp_rotation(R_start, R_end, s);
    end
    
    cart.pos = cart_pos;
    cart.ori = cart_ori;
    vel = cart_vel;
    acc = cart_acc;
end


%% ========================================
%% LSPB 軌跡規劃函式 (單點版本)
%% ========================================
function [pos, vel, acc] = lspb_trajectory(q0, qf, tf, tacc, t)
    % 返回單個時間點的值
    
    h = qf - q0;
    
    if tacc > tf/2
        tacc = tf/2;
    end
    
    V = h / (tf - tacc);
    a = V / tacc;
    
    if t <= tacc
        % 加速段
        pos = q0 + 0.5 * a * t^2;
        vel = a * t;
        acc = a;
        
    elseif t <= (tf - tacc)
        % 等速段
        pos = q0 + V * (t - tacc/2);
        vel = V;
        acc = 0;
        
    else
        % 減速段
        t_dec = t - (tf - tacc);
        pos = qf - 0.5 * a * (tacc - t_dec)^2;
        vel = V - a * t_dec;
        acc = -a;
    end
end


%% ========================================
%% 球面線性插值 (SLERP)
%% ========================================
function R = slerp_rotation(R1, R2, t)
    % R1, R2: 起始和結束旋轉矩陣
    % t: 插值參數 [0, 1]
    
    % 計算相對旋轉
    R_rel = R1' * R2;
    
    % 轉換為軸角表示
    [axis, angle] = rotation_to_axis_angle(R_rel);
    
    % 插值角度
    angle_interp = angle * t;
    
    % 轉回旋轉矩陣
    R_interp = axis_angle_to_rotation(axis, angle_interp);
    
    % 最終旋轉
    R = R1 * R_interp;
end


%% ========================================
%% 旋轉矩陣轉軸角
%% ========================================
function [axis, angle] = rotation_to_axis_angle(R)
    angle = acos((trace(R) - 1) / 2);
    
    if abs(angle) < 1e-6
        axis = [0; 0; 1];  % 任意軸
        angle = 0;
    else
        axis = 1/(2*sin(angle)) * [R(3,2) - R(2,3);
                                     R(1,3) - R(3,1);
                                     R(2,1) - R(1,2)];
    end
end


%% ========================================
%% 軸角轉旋轉矩陣
%% ========================================
function R = axis_angle_to_rotation(axis, angle)
    % Rodrigues' formula
    axis = axis / norm(axis);  % 歸一化
    
    K = [0, -axis(3), axis(2);
         axis(3), 0, -axis(1);
         -axis(2), axis(1), 0];
    
    R = eye(3) + sin(angle) * K + (1 - cos(angle)) * K^2;
end


%% ========================================
%% 選擇最接近的IK解
%% ========================================
function q_best = select_closest_solution(solutions, q_prev)
    % 選擇與前一個關節角度最接近的解
    
    n_sols = size(solutions, 1);
    min_dist = inf;
    q_best = solutions(1, :);
    
    for i = 1:n_sols
        dist = norm(solutions(i, :) - q_prev);
        if dist < min_dist
            min_dist = dist;
            q_best = solutions(i, :);
        end
    end
end