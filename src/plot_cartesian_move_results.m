% plot_cartesian_move_results.m
% ========================================
% 繪製笛卡爾空間軌跡規劃結果 (分離視窗 + 螢幕置中)
% ========================================

function plot_cartesian_move_results(t, cart_pos, cart_vel, cart_acc, q, qd, qdd, A, B, C)
    
    % 獲取螢幕尺寸
    scrsz = get(0, 'ScreenSize');
    % 定義計算置中位置的輔助函式
    get_pos = @(w, h) [(scrsz(3)-w)/2, (scrsz(4)-h)/2, w, h];
    
    %% 視窗 1: 末端執行器 (End-effector)
    figure('Name', 'Cartesian Move - Cartesian Analysis', 'NumberTitle', 'off', ...
           'Position', get_pos(800, 800));
    
    % Position
    titles_pos = {'X Position', 'Y Position', 'Z Position'};
    for i = 1:3
        subplot(3, 3, i);
        plot(t, cart_pos(:, i), 'LineWidth', 1.5, 'Color', 'b');
        grid on; title(titles_pos{i}); ylabel('cm'); xlim([0, t(end)]);
    end
    
    % Velocity
    titles_vel = {'X Velocity', 'Y Velocity', 'Z Velocity'};
    for i = 1:3
        subplot(3, 3, i+3);
        plot(t, cart_vel(:, i), 'LineWidth', 1.5, 'Color', 'r');
        grid on; title(titles_vel{i}); ylabel('cm/s'); xlim([0, t(end)]);
    end
    
    % Acceleration
    titles_acc = {'X Accel', 'Y Accel', 'Z Accel'};
    for i = 1:3
        subplot(3, 3, i+6);
        plot(t, cart_acc(:, i), 'LineWidth', 1.5, 'Color', 'm');
        grid on; title(titles_acc{i}); ylabel('cm/s^2'); xlabel('Time (s)'); xlim([0, t(end)]);
    end

    %% 視窗 2: 關節角度 (Joint Angles)
    figure('Name', 'Cartesian Move - Joint Angles', 'NumberTitle', 'off', ...
           'Position', get_pos(1000, 700));
    for i = 1:6
        subplot(3, 2, i);
        plot(t, q(:, i), 'LineWidth', 1.5);
        grid on;
        if i == 3, ylabel('d3 (inch)'); else, ylabel(sprintf('theta%d (deg)', i)); end
        xlabel('time (s)'); title(sprintf('Joint %d Position', i)); xlim([0, t(end)]);
    end
    
    %% 視窗 3: 關節速度 (Joint Velocities)
    figure('Name', 'Cartesian Move - Joint Velocities', 'NumberTitle', 'off', ...
           'Position', get_pos(1000, 700));
    for i = 1:6
        subplot(3, 2, i);
        plot(t, qd(:, i), 'LineWidth', 1.5, 'Color', 'r');
        grid on;
        if i == 3, ylabel('vel (inch/s)'); else, ylabel('vel (deg/s)'); end
        xlabel('time (s)'); title(sprintf('Joint %d Velocity', i)); xlim([0, t(end)]);
    end
    
    %% 視窗 4: 關節加速度 (Joint Accelerations)
    figure('Name', 'Cartesian Move - Joint Accelerations', 'NumberTitle', 'off', ...
           'Position', get_pos(1000, 700));
    for i = 1:6
        subplot(3, 2, i);
        plot(t, qdd(:, i), 'LineWidth', 1.5, 'Color', 'm');
        grid on;
        if i == 3, ylabel('acc (inch/s^2)'); else, ylabel('acc (deg/s^2)'); end
        xlabel('time (s)'); title(sprintf('Joint %d Acceleration', i)); xlim([0, t(end)]);
    end
    
end