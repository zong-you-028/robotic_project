% main_project.m (改善版)
% ========================================
% Stanford Manipulator 正逆向運動學主程式
% ========================================
% 功能:
%   (1) Forward Kinematics (正向運動學)
%   (2) Inverse Kinematics (逆向運動學)
% ========================================

clc; clear; close all;

%% 定義關節限制
% [min, max] for each joint
% θ1, θ2, d3, θ4, θ5, θ6
joint_limits = [-160, 160;   % θ1 (deg)
                -125, 125;   % θ2 (deg)
                -30, 30;     % d3 (in)
                -140, 140;   % θ4 (deg)
                -100, 100;   % θ5 (deg)
                -260, 260];  % θ6 (deg)

%% 選擇功能
fprintf('========================================\n');
fprintf('  Stanford Manipulator 運動學計算\n');
fprintf('========================================\n');
choice = input('請選擇功能: (1) 正向運動學 (2) 逆向運動學: ');

if choice == 1
    %% ========================================
    %% (1) Forward Kinematics 正向運動學
    %% ========================================
    fprintf('\n--- 正向運動學 ---\n');
    
    % 逐一輸入並驗證關節變數
    theta1 = get_validated_joint_input('θ1', joint_limits(1,:), '度');
    theta2 = get_validated_joint_input('θ2', joint_limits(2,:), '度');
    d3     = get_validated_joint_input('d3', joint_limits(3,:), '英吋');
    theta4 = get_validated_joint_input('θ4', joint_limits(4,:), '度');
    theta5 = get_validated_joint_input('θ5', joint_limits(5,:), '度');
    theta6 = get_validated_joint_input('θ6', joint_limits(6,:), '度');
    
    joint_vars = [theta1, theta2, d3, theta4, theta5, theta6];
    
    % 呼叫正向運動學函式
    [T, pose] = forward_kinematics(joint_vars);
    
    % 輸出結果
    fprintf('\n========================================\n');
    fprintf('--- 正向運動學結果 ---\n');
    fprintf('========================================\n');
    fprintf('\n齊次轉換矩陣 (n, o, a, p):\n');
    fprintf('%.8f  %.8f  %.8f  %.8f\n', T(1,:));
    fprintf('%.8f  %.8f  %.8f  %.8f\n', T(2,:));
    fprintf('%.8f  %.8f  %.8f  %.8f\n', T(3,:));
    fprintf('%.8f  %.8f  %.8f  %.8f\n', T(4,:));
    
    fprintf('\n笛卡爾座標 (x, y, z, φ, θ, ψ):\n');
    fprintf('  x = %.6f,  y = %.6f,  z = %.6f\n', pose(1), pose(2), pose(3));
    fprintf('  φ = %.6f°, θ = %.6f°, ψ = %.6f°\n', pose(4), pose(5), pose(6));
    fprintf('========================================\n');

elseif choice == 2
    %% ========================================
    %% (2) Inverse Kinematics 逆向運動學
    %% ========================================
    fprintf('\n--- 逆向運動學 ---\n');
    
    T_input = [];
    
    % 輸入 4x4 矩陣
    while isempty(T_input) || ~isequal(size(T_input), [4, 4])
        
        fprintf('\n請輸入 4x4 齊次轉換矩陣 T:\n');
        fprintf('(貼上 4 行後，在空白行按 Enter 完成輸入)\n\n');
        
        lines = cell(0);
        line_idx = 1;
        
        try
            while true
                line_str = input('', 's');
                
                if isempty(strtrim(line_str))
                    if line_idx == 1
                        continue;
                    else
                        break;
                    end
                end
                
                lines{line_idx} = line_str;
                line_idx = line_idx + 1;
            end
            
            if numel(lines) ~= 4
                fprintf('錯誤: 預期 4 行，但收到 %d 行。請重新輸入。\n', numel(lines));
                T_input = [];
                continue;
            end
            
            matrix_str = strjoin(lines, ';');
            T_input = str2num(matrix_str); %#ok<ST2NM>
            
            if ~isequal(size(T_input), [4, 4])
                fprintf('錯誤: 輸入未產生 4x4 矩陣。請重新輸入。\n');
                T_input = [];
            end
        catch ME
            fprintf('錯誤: %s\n請重新輸入。\n', ME.message);
            T_input = [];
        end
    end
    
    % 呼叫逆向運動學函式
    solutions = inverse_kinematics(T_input);
    
    % 輸出結果
    fprintf('\n========================================\n');
    fprintf('--- 逆向運動學結果 ---\n');
    fprintf('========================================\n');
    
    if isempty(solutions)
        fprintf('無有效解。目標位置可能超出工作範圍。\n');
    else
        fprintf('找到 %d 組解:\n\n', size(solutions, 1));
        
        for i = 1:size(solutions, 1)
            sol = solutions(i, :);
            
            % 檢查關節限制
            range_check = check_joint_limits_silent(sol, joint_limits);
            
            % 顯示解的標題
            fprintf('Corresponding variables (θ1, θ2, d3, θ4, θ5, θ6):\n');
            
            % 先檢查並標示超出範圍的關節
            out_of_range = find(~range_check);
            if ~isempty(out_of_range)
                for j = 1:length(out_of_range)
                    joint_idx = out_of_range(j);
                    if joint_idx == 3
                        fprintf('d%d is out of range!\n', joint_idx);
                    else
                        fprintf('θ%d is out of range!\n', joint_idx);
                    end
                end
            end
            
            % 顯示解的數值
            fprintf('%10.6f  %10.6f  %10.6f  %11.6f  %11.6f  %11.6f\n', ...
                    sol(1), sol(2), sol(3), sol(4), sol(5), sol(6));
            
            fprintf('\n');
        end
        
        fprintf('========================================\n');
    end

else
    fprintf('無效的選擇。\n');
end


%% ========================================
%% 輔助函式: 驗證輸入
%% ========================================
function value = get_validated_joint_input(joint_name, limits, unit)
    % 提示並驗證關節輸入值
    % joint_name: 關節名稱 (字串)
    % limits: [min, max]
    % unit: 單位 (字串)
    
    prompt = sprintf('請輸入 %s (%.0f ~ %.0f %s): ', ...
                     joint_name, limits(1), limits(2), unit);
    
    while true
        value = input(prompt);
        if value >= limits(1) && value <= limits(2)
            break;
        else
            fprintf('錯誤: %s 超出範圍。請輸入 %.0f 到 %.0f 之間的值。\n', ...
                    joint_name, limits(1), limits(2));
        end
    end
end