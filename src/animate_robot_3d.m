% animate_robot_3d.m
% ========================================
% Stanford Manipulator 3D 姿態動畫展示
% ========================================
% 輸入:
%   q_trajectory: Nx6 關節角度軌跡 [theta1, theta2, d3, theta4, theta5, theta6]
%   t: Nx1 時間向量
%   dt: 採樣時間
%   title_str: 動畫標題
% ========================================

function animate_robot_3d(q_trajectory, t, dt, title_str)
    
    if nargin < 4
        title_str = 'Stanford Manipulator 3D Animation';
    end
    
    %% DH 參數 (固定部分)
    d2 = 6.375;  % Link 2 偏移 (inch)
    
    % Alpha 角度
    alpha = deg2rad([-90, 90, 0, -90, 90, 0]);
    
    %% 準備動畫視窗 (螢幕置中)
    % 獲取螢幕尺寸
    scrsz = get(0, 'ScreenSize'); % [left, bottom, width, height]
    fig_w = 1400;  % 視窗寬度
    fig_h = 900;   % 視窗高度
    
    % 計算置中位置
    pos_left = (scrsz(3) - fig_w) / 2;
    pos_bottom = (scrsz(4) - fig_h) / 2;
    
    fig = figure('Name', title_str, 'NumberTitle', 'off', ...
                 'Position', [pos_left, pos_bottom, fig_w, fig_h]);
    
    % 設定播放速度
    playback_speed = 1.0;  % 1.0 = 實時
    frame_skip = max(1, round(0.02 / dt));  % 約50 FPS
    
    n_points = size(q_trajectory, 1);
    
    %% 計算所有軌跡點（用於繪製完整路徑）
    fprintf('計算軌跡路徑...\n');
    all_positions = zeros(3, n_points);
    for i = 1:n_points
        [positions, ~] = compute_robot_geometry(q_trajectory(i, :), d2, alpha);
        all_positions(:, i) = positions(:, end) * 2.54;  % 末端位置 (cm)
    end
    
    % 計算軸範圍
    x_range = [min(all_positions(1,:)), max(all_positions(1,:))];
    y_range = [min(all_positions(2,:)), max(all_positions(2,:))];
    z_range = [min(all_positions(3,:)), max(all_positions(3,:))];
    
    % 擴大範圍15%
    expand_range = @(r) r + [-1, 1] * (r(2) - r(1)) * 0.15;
    x_range = expand_range(x_range);
    y_range = expand_range(y_range);
    z_range = expand_range(z_range);
    
    % 定義右側子圖的索引位置 (3x4 grid)
    plot_indices = [3, 4, 7, 8, 11, 12];
    
    %% 動畫循環
    fprintf('開始 3D 姿態動畫...\n');
    
    for i = 1:frame_skip:n_points
        
        % 檢查視窗是否存在
        if ~ishandle(fig)
            break;
        end
        figure(fig); % 強制鎖定動畫視窗
        
        % 計算當前姿態
        q_current = q_trajectory(i, :);
        [positions, frames] = compute_robot_geometry(q_current, d2, alpha);
        
        % 轉換為 cm
        positions = positions * 2.54;
        
        % 清除並重繪
        clf;
        
        %% ==================================================
        %% 左側: 3D 機械手臂姿態視圖（主視圖）
        %% ==================================================
        subplot(1, 2, 1);
        hold on; grid on; axis equal;
        
        % 繪製路徑
        plot3(all_positions(1, 1:i), all_positions(2, 1:i), ...
              all_positions(3, 1:i), 'c--', 'LineWidth', 1.5);
        
        if i < n_points
            plot3(all_positions(1, i:end), all_positions(2, i:end), ...
                  all_positions(3, i:end), 'Color', [0.8, 0.8, 0.8], ...
                  'LineWidth', 1, 'LineStyle', ':');
        end
        
        % 繪製標記點
        plot3(all_positions(1, 1), all_positions(2, 1), all_positions(3, 1), ...
              'go', 'MarkerSize', 15, 'MarkerFaceColor', 'g', 'DisplayName', 'A');
        
        if n_points > 250
            mid_idx = round(n_points / 2);
            plot3(all_positions(1, mid_idx), all_positions(2, mid_idx), ...
                  all_positions(3, mid_idx), 'ro', 'MarkerSize', 15, ...
                  'MarkerFaceColor', 'r', 'DisplayName', 'B');
        end
        
        plot3(all_positions(1, end), all_positions(2, end), all_positions(3, end), ...
              'mo', 'MarkerSize', 15, 'MarkerFaceColor', 'm', 'DisplayName', 'C');
        
        % 繪製手臂與關節
        plot3(positions(1, :), positions(2, :), positions(3, :), ...
              'b-', 'LineWidth', 4);
        plot3(positions(1, :), positions(2, :), positions(3, :), ...
              'ro', 'MarkerSize', 12, 'MarkerFaceColor', 'r');
        plot3(0, 0, 0, 'ks', 'MarkerSize', 20, 'MarkerFaceColor', 'k');
        
        % 繪製座標系
        coord_scale = 3;
        for j = 1:length(frames)
            origin = frames{j}(1:3, 4) * 2.54;
            R = frames{j}(1:3, 1:3);
            quiver3(origin(1), origin(2), origin(3), R(1,1)*coord_scale, R(2,1)*coord_scale, R(3,1)*coord_scale, 'r', 'LineWidth', 1, 'MaxHeadSize', 0.3, 'AutoScale', 'off');
            quiver3(origin(1), origin(2), origin(3), R(1,2)*coord_scale, R(2,2)*coord_scale, R(3,2)*coord_scale, 'g', 'LineWidth', 1, 'MaxHeadSize', 0.3, 'AutoScale', 'off');
            quiver3(origin(1), origin(2), origin(3), R(1,3)*coord_scale, R(2,3)*coord_scale, R(3,3)*coord_scale, 'b', 'LineWidth', 1, 'MaxHeadSize', 0.3, 'AutoScale', 'off');
        end
        
        end_scale = 8;
        origin = positions(:, end);
        R_end = frames{end}(1:3, 1:3);
        quiver3(origin(1), origin(2), origin(3), R_end(1,1)*end_scale, R_end(2,1)*end_scale, R_end(3,1)*end_scale, 'r', 'LineWidth', 3, 'MaxHeadSize', 0.5, 'AutoScale', 'off');
        quiver3(origin(1), origin(2), origin(3), R_end(1,2)*end_scale, R_end(2,2)*end_scale, R_end(3,2)*end_scale, 'g', 'LineWidth', 3, 'MaxHeadSize', 0.5, 'AutoScale', 'off');
        quiver3(origin(1), origin(2), origin(3), R_end(1,3)*end_scale, R_end(2,3)*end_scale, R_end(3,3)*end_scale, 'b', 'LineWidth', 3, 'MaxHeadSize', 0.5, 'AutoScale', 'off');
        
        text(all_positions(1, 1)+3, all_positions(2, 1), all_positions(3, 1)+3, 'A', 'FontSize', 14, 'Color', 'g', 'FontWeight', 'bold');
        if n_points > 250
            mid_idx = round(n_points / 2);
            text(all_positions(1, mid_idx)+3, all_positions(2, mid_idx), all_positions(3, mid_idx)+3, 'B', 'FontSize', 14, 'Color', 'r', 'FontWeight', 'bold');
        end
        text(all_positions(1, end)+3, all_positions(2, end), all_positions(3, end)+3, 'C', 'FontSize', 14, 'Color', 'm', 'FontWeight', 'bold');
        
        xlabel('x (cm)', 'FontSize', 12, 'FontWeight', 'bold');
        ylabel('y (cm)', 'FontSize', 12, 'FontWeight', 'bold');
        zlabel('z (cm)', 'FontSize', 12, 'FontWeight', 'bold');
        title(sprintf('%s\nTime: %.3f s / %.3f s', title_str, t(i), t(end)), 'FontSize', 14, 'FontWeight', 'bold');
        
        xlim(x_range); ylim(y_range); zlim(z_range);
        view(45, 30);
        legend('Location', 'northeast', 'FontSize', 9);
        
        %% ==================================================
        %% 右側: 關節角度即時圖表
        %% ==================================================
        for j = 1:6
            subplot(3, 4, plot_indices(j));
            plot(t, q_trajectory(:, j), 'Color', [0.7, 0.7, 0.7], 'LineWidth', 1);
            hold on;
            plot(t(1:i), q_trajectory(1:i, j), 'b-', 'LineWidth', 2);
            plot(t(i), q_trajectory(i, j), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
            grid on; hold off;
            
            if j == 3
                ylabel('d3 (in)', 'FontSize', 10, 'FontWeight', 'bold');
                title(sprintf('Joint %d: %.2f in', j, q_trajectory(i, j)), 'FontSize', 11, 'FontWeight', 'bold');
            else
                ylabel('Angle (deg)', 'FontSize', 10, 'FontWeight', 'bold');
                title(sprintf('Joint %d: %.2f°', j, q_trajectory(i, j)), 'FontSize', 11, 'FontWeight', 'bold');
            end
            if j > 3, xlabel('Time (s)', 'FontSize', 10); end
            
            xlim([0, t(end)]);
            y_min = min(q_trajectory(:, j)); y_max = max(q_trajectory(:, j));
            y_margin = (y_max - y_min) * 0.1;
            if y_margin < 1, y_margin = 1; end
            ylim([y_min - y_margin, y_max + y_margin]);
        end
        
        drawnow;
        if i < n_points
            pause(dt * frame_skip / playback_speed);
        end
    end
    fprintf('動畫完成！\n');
end

function [positions, frames] = compute_robot_geometry(q, d2, alpha)
    th1 = deg2rad(q(1)); th2 = deg2rad(q(2)); d3 = q(3);
    th4 = deg2rad(q(4)); th5 = deg2rad(q(5)); th6 = deg2rad(q(6));
    d = [0, d2, d3, 0, 0, 0]; a = zeros(1,6); theta = [th1, th2, 0, th4, th5, th6];
    frames = cell(1, 7); frames{1} = eye(4);
    for i = 1:6
        A = dh_matrix(d(i), a(i), alpha(i), theta(i));
        frames{i+1} = frames{i} * A;
    end
    positions = zeros(3, 7);
    for i = 1:7, positions(:, i) = frames{i}(1:3, 4); end
end

function A = dh_matrix(d, a, alpha, theta)
    A = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
         sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0,           sin(alpha),             cos(alpha),            d;
         0,           0,                      0,                     1];
end