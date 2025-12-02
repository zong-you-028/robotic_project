% animate_robot_3d.m
% ========================================
% Stanford Manipulator 3D 姿態動畫展示
% ========================================

function animate_robot_3d(q_trajectory, t, dt, title_str)
    
    if nargin < 4, title_str = 'Stanford Manipulator 3D Animation'; end
    d2 = 6.375; alpha = deg2rad([-90, 90, 0, -90, 90, 0]);
    
    % 獲取螢幕尺寸並置中 (動畫視窗 1400x900)
    scrsz = get(0, 'ScreenSize');
    fig_w = 1400; fig_h = 900;
    pos = [(scrsz(3)-fig_w)/2, (scrsz(4)-fig_h)/2, fig_w, fig_h];
    
    fig = figure('Name', title_str, 'NumberTitle', 'off', 'Position', pos);
    
    playback_speed = 1.0; frame_skip = max(1, round(0.02 / dt));
    n_points = size(q_trajectory, 1);
    
    fprintf('計算軌跡路徑...\n');
    all_positions = zeros(3, n_points);
    for i = 1:n_points
        [positions, ~] = compute_robot_geometry(q_trajectory(i, :), d2, alpha);
        all_positions(:, i) = positions(:, end) * 2.54;
    end
    
    % 計算範圍
    x_range = [min(all_positions(1,:)), max(all_positions(1,:))];
    y_range = [min(all_positions(2,:)), max(all_positions(2,:))];
    z_range = [min(all_positions(3,:)), max(all_positions(3,:))];
    % 安全範圍擴充，避免 xlim 報錯
    x_range = ensure_valid_range(x_range);
    y_range = ensure_valid_range(y_range);
    z_range = ensure_valid_range(z_range);
    
    plot_indices = [3, 4, 7, 8, 11, 12];
    
    fprintf('開始 3D 姿態動畫...\n');
    for i = 1:frame_skip:n_points
        if ~ishandle(fig), break; end
        figure(fig); % 強制鎖定動畫視窗
        
        q_current = q_trajectory(i, :);
        [positions, frames] = compute_robot_geometry(q_current, d2, alpha);
        positions = positions * 2.54;
        
        clf;
        
        % 左側 3D 圖
        subplot(1, 2, 1); hold on; grid on; axis equal;
        plot3(all_positions(1, 1:i), all_positions(2, 1:i), all_positions(3, 1:i), 'c--', 'LineWidth', 1.5);
        if i < n_points
            plot3(all_positions(1, i:end), all_positions(2, i:end), all_positions(3, i:end), 'Color', [0.8, 0.8, 0.8], 'LineWidth', 1, 'LineStyle', ':');
        end
        plot3(all_positions(1, 1), all_positions(2, 1), all_positions(3, 1), 'go', 'MarkerSize', 15, 'MarkerFaceColor', 'g');
        plot3(all_positions(1, end), all_positions(2, end), all_positions(3, end), 'mo', 'MarkerSize', 15, 'MarkerFaceColor', 'm');
        
        plot3(positions(1, :), positions(2, :), positions(3, :), 'b-', 'LineWidth', 4);
        plot3(positions(1, :), positions(2, :), positions(3, :), 'ro', 'MarkerSize', 12, 'MarkerFaceColor', 'r');
        plot3(0, 0, 0, 'ks', 'MarkerSize', 20, 'MarkerFaceColor', 'k');
        
        % 座標系
        coord_scale = 3; end_scale = 8;
        for j = 1:length(frames)
            o = frames{j}(1:3, 4) * 2.54; R = frames{j}(1:3, 1:3);
            quiver3(o(1), o(2), o(3), R(1,1)*coord_scale, R(2,1)*coord_scale, R(3,1)*coord_scale, 'r', 'LineWidth', 1, 'AutoScale', 'off');
            quiver3(o(1), o(2), o(3), R(1,2)*coord_scale, R(2,2)*coord_scale, R(3,2)*coord_scale, 'g', 'LineWidth', 1, 'AutoScale', 'off');
            quiver3(o(1), o(2), o(3), R(1,3)*coord_scale, R(2,3)*coord_scale, R(3,3)*coord_scale, 'b', 'LineWidth', 1, 'AutoScale', 'off');
        end
        o = positions(:, end); R = frames{end}(1:3, 1:3);
        quiver3(o(1), o(2), o(3), R(1,1)*end_scale, R(2,1)*end_scale, R(3,1)*end_scale, 'r', 'LineWidth', 3, 'AutoScale', 'off');
        quiver3(o(1), o(2), o(3), R(1,2)*end_scale, R(2,2)*end_scale, R(3,2)*end_scale, 'g', 'LineWidth', 3, 'AutoScale', 'off');
        quiver3(o(1), o(2), o(3), R(1,3)*end_scale, R(2,3)*end_scale, R(3,3)*end_scale, 'b', 'LineWidth', 3, 'AutoScale', 'off');

        xlabel('x (cm)'); ylabel('y (cm)'); zlabel('z (cm)');
        title(sprintf('%s\nTime: %.2f s', title_str, t(i)));
        xlim(x_range); ylim(y_range); zlim(z_range); view(45, 30);
        
        % 右側圖表
        for j = 1:6
            subplot(3, 4, plot_indices(j));
            plot(t, q_trajectory(:, j), 'Color', [0.7, 0.7, 0.7]); hold on;
            plot(t(1:i), q_trajectory(1:i, j), 'b-', 'LineWidth', 2);
            plot(t(i), q_trajectory(i, j), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
            grid on; hold off;
            if j == 3, title(sprintf('d3: %.2f', q_trajectory(i,j))); else, title(sprintf('J%d: %.1f', j, q_trajectory(i,j))); end
            xlim([0, t(end)]);
        end
        drawnow;
    end
    fprintf('動畫完成！\n');
end

function r = ensure_valid_range(r)
    w = r(2) - r(1);
    if w < 1e-3, m = mean(r); r = m + [-10, 10]; else, r = r + [-w, w]*0.15; end
end

function [positions, frames] = compute_robot_geometry(q, d2, alpha)
    th = deg2rad(q([1,2,4,5,6])); d3 = q(3);
    d = [0, d2, d3, 0, 0, 0]; a = zeros(1,6); theta = [th(1), th(2), 0, th(3), th(4), th(5)];
    frames = cell(1, 7); frames{1} = eye(4);
    for i = 1:6
        frames{i+1} = frames{i} * dh_matrix(d(i), a(i), alpha(i), theta(i));
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