% animate_robot.m
% ========================================
% Stanford Manipulator 動畫展示
% ========================================
% 輸入:
%   q_trajectory: Nx6 關節角度軌跡 [theta1, theta2, d3, theta4, theta5, theta6]
%   t: Nx1 時間向量
%   dt: 採樣時間
%   title_str: 動畫標題
% ========================================

function animate_robot(q_trajectory, t, dt, title_str)
    
    if nargin < 4
        title_str = 'Stanford Manipulator Animation';
    end
    
    %% DH 參數 (固定部分)
    d2 = 6.375;  % Link 2 偏移 (inch)
    
    % Alpha 角度
    alpha = deg2rad([-90, 90, 0, -90, 90, 0]);
    
    %% 準備動畫視窗
    fig = figure('Name', title_str, 'NumberTitle', 'off', ...
                 'Position', [100, 100, 1200, 800]);
    
    % 設定播放速度（可調整）
    playback_speed = 1.0;  % 1.0 = 實時, 2.0 = 2倍速, 0.5 = 0.5倍速
    frame_skip = max(1, round(0.02 / dt));  % 大約每0.02秒一幀
    
    n_points = size(q_trajectory, 1);
    
    %% 計算所有點的位置（用於設定軸範圍）
    fprintf('計算動畫範圍...\n');
    all_points = [];
    for i = 1:10:n_points  % 抽樣計算
        [positions, ~] = compute_robot_geometry(q_trajectory(i, :), d2, alpha);
        all_points = [all_points; positions'];
    end
    
    % 設定軸範圍（轉換為cm）
    x_range = [min(all_points(:,1)), max(all_points(:,1))] * 2.54;
    y_range = [min(all_points(:,2)), max(all_points(:,2))] * 2.54;
    z_range = [min(all_points(:,3)), max(all_points(:,3))] * 2.54;
    
    % 擴大範圍10%
    x_margin = (x_range(2) - x_range(1)) * 0.1;
    y_margin = (y_range(2) - y_range(1)) * 0.1;
    z_margin = (z_range(2) - z_range(1)) * 0.1;
    
    x_range = x_range + [-x_margin, x_margin];
    y_range = y_range + [-y_margin, y_margin];
    z_range = z_range + [-z_margin, z_margin];
    
    %% 動畫循環
    fprintf('開始動畫...\n');
    
    for i = 1:frame_skip:n_points
        
        % 計算當前姿態
        q_current = q_trajectory(i, :);
        [positions, frames] = compute_robot_geometry(q_current, d2, alpha);
        
        % 轉換為 cm
        positions = positions * 2.54;
        
        % 清除並重繪
        clf;
        
        %% 繪製機械手臂
        subplot(1, 2, 1);
        hold on; grid on; axis equal;
        
        % 繪製連桿
        plot3(positions(1, :), positions(2, :), positions(3, :), ...
              'b-', 'LineWidth', 3);
        
        % 繪製關節
        plot3(positions(1, :), positions(2, :), positions(3, :), ...
              'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
        
        % 繪製基座
        plot3(0, 0, 0, 'ks', 'MarkerSize', 20, 'MarkerFaceColor', 'k');
        
        % 繪製末端執行器座標系
        scale = 5;  % cm
        origin = positions(:, end);
        
        % 從最後一個frame取得方向
        R_end = frames{end}(1:3, 1:3);
        
        % X軸 (紅色)
        quiver3(origin(1), origin(2), origin(3), ...
                R_end(1,1)*scale, R_end(2,1)*scale, R_end(3,1)*scale, ...
                'r', 'LineWidth', 2, 'MaxHeadSize', 0.5);
        
        % Y軸 (綠色)
        quiver3(origin(1), origin(2), origin(3), ...
                R_end(1,2)*scale, R_end(2,2)*scale, R_end(3,2)*scale, ...
                'g', 'LineWidth', 2, 'MaxHeadSize', 0.5);
        
        % Z軸 (藍色)
        quiver3(origin(1), origin(2), origin(3), ...
                R_end(1,3)*scale, R_end(2,3)*scale, R_end(3,3)*scale, ...
                'b', 'LineWidth', 2, 'MaxHeadSize', 0.5);
        
        % 繪製軌跡（已經過的路徑）
        if i > 1
            past_positions = zeros(3, min(i, n_points));
            for j = 1:frame_skip:i
                [pos_temp, ~] = compute_robot_geometry(q_trajectory(j, :), d2, alpha);
                past_positions(:, j) = pos_temp(:, end) * 2.54;
            end
            plot3(past_positions(1, 1:i), past_positions(2, 1:i), ...
                  past_positions(3, 1:i), 'c--', 'LineWidth', 1);
        end
        
        % 設定視角和範圍
        xlabel('X (cm)', 'FontSize', 12);
        ylabel('Y (cm)', 'FontSize', 12);
        zlabel('Z (cm)', 'FontSize', 12);
        title(sprintf('%s\nTime: %.3f s', title_str, t(i)), 'FontSize', 14);
        
        xlim(x_range);
        ylim(y_range);
        zlim(z_range);
        view(45, 30);
        
        %% 繪製關節角度圖
        subplot(1, 2, 2);
        
        for j = 1:6
            subplot(3, 4, j+6);
            plot(t(1:i), q_trajectory(1:i, j), 'b-', 'LineWidth', 1.5);
            hold on;
            plot(t(i), q_trajectory(i, j), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
            grid on;
            
            if j == 3
                ylabel('d3 (in)', 'FontSize', 9);
                title(sprintf('Joint %d: %.2f in', j, q_trajectory(i, j)), 'FontSize', 10);
            else
                ylabel('Angle (deg)', 'FontSize', 9);
                title(sprintf('Joint %d: %.2f°', j, q_trajectory(i, j)), 'FontSize', 10);
            end
            
            if j > 3
                xlabel('Time (s)', 'FontSize', 9);
            end
            
            xlim([0, t(end)]);
        end
        
        drawnow;
        
        % 控制播放速度
        if i < n_points
            pause(dt * frame_skip / playback_speed);
        end
    end
    
    fprintf('動畫完成！\n');
end


%% ========================================
%% 子函式: 計算機器人幾何形狀
%% ========================================
function [positions, frames] = compute_robot_geometry(q, d2, alpha)
    % 計算各關節的位置和座標系
    % q: [theta1, theta2, d3, theta4, theta5, theta6]
    % 返回:
    %   positions: 3x7 矩陣，各關節位置 [base, joint1, ..., joint6]
    %   frames: cell array，各關節的齊次轉換矩陣
    
    % 轉換為弧度
    th1 = deg2rad(q(1));
    th2 = deg2rad(q(2));
    d3 = q(3);
    th4 = deg2rad(q(4));
    th5 = deg2rad(q(5));
    th6 = deg2rad(q(6));
    
    % DH 參數
    d = [0, d2, d3, 0, 0, 0];
    a = [0, 0, 0, 0, 0, 0];
    theta = [th1, th2, 0, th4, th5, th6];
    
    % 計算各關節的轉換矩陣
    frames = cell(1, 7);
    frames{1} = eye(4);  % 基座
    
    for i = 1:6
        A = dh_matrix(d(i), a(i), alpha(i), theta(i));
        frames{i+1} = frames{i} * A;
    end
    
    % 提取位置
    positions = zeros(3, 7);
    for i = 1:7
        positions(:, i) = frames{i}(1:3, 4);
    end
end


%% ========================================
%% DH 轉換矩陣
%% ========================================
function A = dh_matrix(d, a, alpha, theta)
    A = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
         sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0,           sin(alpha),             cos(alpha),            d;
         0,           0,                      0,                     1];
end