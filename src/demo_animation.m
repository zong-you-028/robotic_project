% demo_animation.m
% ========================================
% Stanford Manipulator 動畫展示
% ========================================
% 直接播放動畫，無需執行完整的軌跡規劃
% ========================================

clc; clear; close all;

fprintf('========================================\n');
fprintf('  Stanford Manipulator 動畫展示\n');
fprintf('========================================\n\n');

%% 定義目標點
A = [0  0  1  15;
     1  0  0  25;
     0  1  0  35;
     0  0  0  1];

B = [0   1  0  10;
    -1   0  0  15;
     0   0  1  20;
     0   0  0  1];

C = [0  -1  0 -10;
     1   0  0  30;
     0   0  1 -15;
     0   0  0  1];

%% 運動參數
t_AB = 0.5;
t_BC = 0.5;
t_acc = 0.2;
dt = 0.002;

%% 選擇動畫類型
fprintf('選擇動畫類型:\n');
fprintf('  1 - Joint Move 動畫\n');
fprintf('  2 - Cartesian Move 動畫\n');
fprintf('  3 - 簡單測試動畫（快速）\n');
fprintf('========================================\n');
choice = input('請輸入選項 (1/2/3): ');

switch choice
    case 1
        fprintf('\n執行 Joint Move 規劃...\n');
        
        % 單位轉換
        A_inch = A; A_inch(1:3, 4) = A_inch(1:3, 4) / 2.54;
        B_inch = B; B_inch(1:3, 4) = B_inch(1:3, 4) / 2.54;
        C_inch = C; C_inch(1:3, 4) = C_inch(1:3, 4) / 2.54;
        
        % IK求解
        q_A = inverse_kinematics(A_inch); q_A = q_A(1, :);
        q_B = inverse_kinematics(B_inch); q_B = q_B(1, :);
        q_C = inverse_kinematics(C_inch); q_C = q_C(1, :);
        
        % 軌跡規劃
        [q_AB, ~, ~, t_AB_vec] = plan_joint_trajectory(q_A, q_B, t_AB, t_acc, dt);
        [q_BC, ~, ~, t_BC_vec] = plan_joint_trajectory(q_B, q_C, t_BC, t_acc, dt);
        
        q_all = [q_AB; q_BC(2:end, :)];
        t_AB_vec = t_AB_vec(:);
        t_BC_vec = t_BC_vec(:);
        t_all = [t_AB_vec; t_BC_vec(2:end) + t_AB];
        
        % 播放 3D 動畫
        animate_robot_3d(q_all, t_all, dt, 'Joint Move - 3D Animation');
        
    case 2
        fprintf('\n執行 Cartesian Move 規劃...\n');
        
        % 提取位置和旋轉
        p_A = A(1:3, 4); R_A = A(1:3, 1:3);
        p_B = B(1:3, 4); R_B = B(1:3, 1:3);
        p_C = C(1:3, 4); R_C = C(1:3, 1:3);
        
        % 軌跡規劃
        [cart_AB, ~, ~, t_AB_vec] = plan_cartesian_trajectory(p_A, p_B, R_A, R_B, t_AB, t_acc, dt);
        [cart_BC, ~, ~, t_BC_vec] = plan_cartesian_trajectory(p_B, p_C, R_B, R_C, t_BC, t_acc, dt);
        
        cart_pos = [cart_AB.pos; cart_BC.pos(2:end, :)];
        cart_ori = [cart_AB.ori; cart_BC.ori(2:end, :, :)];
        
        t_AB_vec = t_AB_vec(:);
        t_BC_vec = t_BC_vec(:);
        t_all = [t_AB_vec; t_BC_vec(2:end) + t_AB];
        
        % 計算關節軌跡
        n_points = size(cart_pos, 1);
        q_all = zeros(n_points, 6);
        
        fprintf('計算關節角度...\n');
        for i = 1:n_points
            T_target = [squeeze(cart_ori(i, :, :)), cart_pos(i, :)' / 2.54; 0 0 0 1];
            sols = inverse_kinematics(T_target);
            if ~isempty(sols)
                if i == 1
                    q_all(i, :) = sols(1, :);
                else
                    q_all(i, :) = select_closest_solution(sols, q_all(i-1, :));
                end
            else
                if i > 1
                    q_all(i, :) = q_all(i-1, :);
                end
            end
            
            if mod(i, 50) == 0
                fprintf('  進度: %d/%d\n', i, n_points);
            end
        end
        
        % 播放 3D 動畫
        animate_robot_3d(q_all, t_all, dt, 'Cartesian Move - 3D Animation');
        
    case 3
        fprintf('\n執行簡單測試動畫...\n');
        
        % 簡單的關節空間插值
        q_start = [0, 0, 15, 0, 0, 0];
        q_end = [90, 45, 25, 45, 45, 90];
        
        t_test = 2.0;  % 2秒
        dt_test = 0.01;  % 10ms
        t_vec = (0:dt_test:t_test)';
        n = length(t_vec);
        
        q_traj = zeros(n, 6);
        for i = 1:6
            q_traj(:, i) = linspace(q_start(i), q_end(i), n);
        end
        
        % 播放 3D 動畫
        animate_robot_3d(q_traj, t_vec, dt_test, 'Simple Test - 3D Animation');
        
    otherwise
        fprintf('無效的選項。\n');
        return;
end

fprintf('\n動畫展示完成！\n');


%% ========================================
%% 輔助函式
%% ========================================

function [q, qd, qdd, t_vec] = plan_joint_trajectory(q_start, q_end, t_total, t_acc, dt)
    n_joints = 6;
    t_vec = (0:dt:t_total)';
    n_points = length(t_vec);
    
    q = zeros(n_points, n_joints);
    qd = zeros(n_points, n_joints);
    qdd = zeros(n_points, n_joints);
    
    for j = 1:n_joints
        theta_0 = q_start(j);
        theta_f = q_end(j);
        [q(:, j), qd(:, j), qdd(:, j)] = lspb_trajectory(theta_0, theta_f, t_total, t_acc, t_vec);
    end
end

function [pos, vel, acc] = lspb_trajectory(q0, qf, tf, tacc, t)
    h = qf - q0;
    if tacc > tf/2
        tacc = tf/2;
    end
    V = h / (tf - tacc);
    a = V / tacc;
    
    n = length(t);
    pos = zeros(n, 1);
    vel = zeros(n, 1);
    acc = zeros(n, 1);
    
    for i = 1:n
        ti = t(i);
        if ti <= tacc
            pos(i) = q0 + 0.5 * a * ti^2;
            vel(i) = a * ti;
            acc(i) = a;
        elseif ti <= (tf - tacc)
            pos(i) = q0 + V * (ti - tacc/2);
            vel(i) = V;
            acc(i) = 0;
        else
            t_dec = ti - (tf - tacc);
            pos(i) = qf - 0.5 * a * (tacc - t_dec)^2;
            vel(i) = V - a * t_dec;
            acc(i) = -a;
        end
    end
end

function [cart, vel, acc, t_vec] = plan_cartesian_trajectory(p_start, p_end, R_start, R_end, t_total, t_acc, dt)
    t_vec = (0:dt:t_total)';
    n_points = length(t_vec);
    
    cart_pos = zeros(n_points, 3);
    cart_vel = zeros(n_points, 3);
    cart_acc = zeros(n_points, 3);
    cart_ori = zeros(n_points, 3, 3);
    
    for i = 1:n_points
        ti = t_vec(i);
        for j = 1:3
            [cart_pos(i, j), cart_vel(i, j), cart_acc(i, j)] = ...
                lspb_trajectory_single(p_start(j), p_end(j), t_total, t_acc, ti);
        end
        s = ti / t_total;
        cart_ori(i, :, :) = slerp_rotation(R_start, R_end, s);
    end
    
    cart.pos = cart_pos;
    cart.ori = cart_ori;
    vel = cart_vel;
    acc = cart_acc;
end

function [pos, vel, acc] = lspb_trajectory_single(q0, qf, tf, tacc, t)
    h = qf - q0;
    if tacc > tf/2
        tacc = tf/2;
    end
    V = h / (tf - tacc);
    a = V / tacc;
    
    if t <= tacc
        pos = q0 + 0.5 * a * t^2;
        vel = a * t;
        acc = a;
    elseif t <= (tf - tacc)
        pos = q0 + V * (t - tacc/2);
        vel = V;
        acc = 0;
    else
        t_dec = t - (tf - tacc);
        pos = qf - 0.5 * a * (tacc - t_dec)^2;
        vel = V - a * t_dec;
        acc = -a;
    end
end

function R = slerp_rotation(R1, R2, t)
    R_rel = R1' * R2;
    [axis, angle] = rotation_to_axis_angle(R_rel);
    angle_interp = angle * t;
    R_interp = axis_angle_to_rotation(axis, angle_interp);
    R = R1 * R_interp;
end

function [axis, angle] = rotation_to_axis_angle(R)
    angle = acos((trace(R) - 1) / 2);
    if abs(angle) < 1e-6
        axis = [0; 0; 1];
        angle = 0;
    else
        axis = 1/(2*sin(angle)) * [R(3,2) - R(2,3); R(1,3) - R(3,1); R(2,1) - R(1,2)];
    end
end

function R = axis_angle_to_rotation(axis, angle)
    axis = axis / norm(axis);
    K = [0, -axis(3), axis(2); axis(3), 0, -axis(1); -axis(2), axis(1), 0];
    R = eye(3) + sin(angle) * K + (1 - cos(angle)) * K^2;
end

function q_best = select_closest_solution(solutions, q_prev)
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