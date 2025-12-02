% joint_move_planning.m
% ========================================
% 關節空間軌跡規劃 (Joint Move)
% ========================================
% 輸入:
%   A, B, C: 目標姿態矩陣 (4x4, 單位: cm)
%   t_AB, t_BC: 移動時間 (sec)
%   t_acc: 過渡段加速時間 (sec)
%   dt: 採樣時間 (sec)
% ========================================

function joint_move_planning(A, B, C, t_AB, t_BC, t_acc, dt)
    
    %% 單位轉換: cm -> inch (因為FK/IK使用inch)
    A_inch = A; A_inch(1:3, 4) = A_inch(1:3, 4) / 2.54;
    B_inch = B; B_inch(1:3, 4) = B_inch(1:3, 4) / 2.54;
    C_inch = C; C_inch(1:3, 4) = C_inch(1:3, 4) / 2.54;
    
    %% 使用逆向運動學求解關節角度
    fprintf('計算目標點的關節角度...\n');
    
    % 求解各點的IK（選擇第一組解）
    sols_A = inverse_kinematics(A_inch);
    sols_B = inverse_kinematics(B_inch);
    sols_C = inverse_kinematics(C_inch);
    
    if isempty(sols_A) || isempty(sols_B) || isempty(sols_C)
        error('逆向運動學無解，請檢查目標點位置。');
    end
    
    % 選擇第一組解
    q_A = sols_A(1, :);
    q_B = sols_B(1, :);
    q_C = sols_C(1, :);
    
    fprintf('點A關節角: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n', q_A);
    fprintf('點B關節角: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n', q_B);
    fprintf('點C關節角: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n', q_C);
    
    %% 軌跡規劃
    fprintf('\n執行關節空間軌跡規劃...\n');
    
    % 規劃 A -> B 段
    [q_AB, qd_AB, qdd_AB, t_AB_vec] = plan_joint_trajectory(q_A, q_B, t_AB, t_acc, dt);
    
    % 規劃 B -> C 段
    [q_BC, qd_BC, qdd_BC, t_BC_vec] = plan_joint_trajectory(q_B, q_C, t_BC, t_acc, dt);
    
    % 合併軌跡
    q_all = [q_AB; q_BC(2:end, :)];     % 移除B點的重複
    qd_all = [qd_AB; qd_BC(2:end, :)];
    qdd_all = [qdd_AB; qdd_BC(2:end, :)];
    
    % 確保時間向量是列向量
    t_AB_vec = t_AB_vec(:);
    t_BC_vec = t_BC_vec(:);
    t_all = [t_AB_vec; t_BC_vec(2:end) + t_AB];
    
    fprintf('軌跡點總數: %d\n', size(q_all, 1));
    
    %% 計算對應的笛卡爾軌跡
    fprintf('計算笛卡爾空間軌跡...\n');
    n_points = size(q_all, 1);
    cart_pos = zeros(n_points, 3);  % [x, y, z]
    cart_vel = zeros(n_points, 3);
    cart_acc = zeros(n_points, 3);
    
    for i = 1:n_points
        [T, ~] = forward_kinematics(q_all(i, :));
        cart_pos(i, :) = T(1:3, 4)' * 2.54;  % inch -> cm
        
        % 使用雅可比矩陣計算速度和加速度（簡化版）
        if i > 1
            cart_vel(i, :) = (cart_pos(i, :) - cart_pos(i-1, :)) / dt;
        end
        if i > 2
            cart_acc(i, :) = (cart_vel(i, :) - cart_vel(i-1, :)) / dt;
        end
    end
    
    %% 繪圖
    fprintf('繪製結果...\n');
    plot_joint_move_results(t_all, q_all, qd_all, qdd_all, cart_pos, cart_vel, cart_acc, A, B, C);
    
    %% 動畫
    fprintf('\n是否要播放動畫? (1=基本動畫, 2=3D姿態動畫, 0=不播放): ');
    user_input = input('');
    if user_input == 1
        fprintf('準備基本動畫...\n');
        animate_robot(q_all, t_all, dt, 'Joint Move Animation');
    elseif user_input == 2
        fprintf('準備3D姿態動畫...\n');
        animate_robot_3d(q_all, t_all, dt, 'Joint Move - 3D Animation');
    end
    
    fprintf('關節空間軌跡規劃完成！\n');
end


%% ========================================
%% 子函式: 單段軌跡規劃 (含直線段與過渡段)
%% ========================================
function [q, qd, qdd, t_vec] = plan_joint_trajectory(q_start, q_end, t_total, t_acc, dt)
    % 使用線性插值 + 拋物線混合 (LSPB - Linear Segment with Parabolic Blends)
    
    n_joints = 6;
    t_vec = (0:dt:t_total)';  % 生成列向量
    n_points = length(t_vec);
    
    q = zeros(n_points, n_joints);
    qd = zeros(n_points, n_joints);
    qdd = zeros(n_points, n_joints);
    
    for j = 1:n_joints
        % 對每個關節進行規劃
        theta_0 = q_start(j);
        theta_f = q_end(j);
        
        % 使用 LSPB 軌跡
        [q(:, j), qd(:, j), qdd(:, j)] = lspb_trajectory(theta_0, theta_f, t_total, t_acc, t_vec);
    end
end


%% ========================================
%% LSPB 軌跡規劃函式
%% ========================================
function [pos, vel, acc] = lspb_trajectory(q0, qf, tf, tacc, t)
    % Linear Segment with Parabolic Blends
    % q0: 起始位置
    % qf: 結束位置
    % tf: 總時間
    % tacc: 加速時間
    % t: 時間向量
    
    % 計算參數
    h = qf - q0;  % 總位移
    
    % 確保 tacc 不超過 tf/2
    if tacc > tf/2
        tacc = tf/2;
    end
    
    % 計算直線段速度
    V = h / (tf - tacc);
    
    % 計算加速度
    a = V / tacc;
    
    n = length(t);
    pos = zeros(n, 1);
    vel = zeros(n, 1);
    acc = zeros(n, 1);
    
    for i = 1:n
        ti = t(i);
        
        if ti <= tacc
            % 加速段 (拋物線)
            pos(i) = q0 + 0.5 * a * ti^2;
            vel(i) = a * ti;
            acc(i) = a;
            
        elseif ti <= (tf - tacc)
            % 等速段 (直線)
            pos(i) = q0 + V * (ti - tacc/2);
            vel(i) = V;
            acc(i) = 0;
            
        else
            % 減速段 (拋物線)
            t_dec = ti - (tf - tacc);
            pos(i) = qf - 0.5 * a * (tacc - t_dec)^2;
            vel(i) = V - a * t_dec;
            acc(i) = -a;
        end
    end
end