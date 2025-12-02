% plot_cartesian_move_results.m
% ========================================
% 繪製笛卡爾空間軌跡規劃結果 (符合PDF範例: 3個視窗，置中)
% ========================================

function plot_cartesian_move_results(t, cart_pos, cart_vel, cart_acc, q, qd, qdd, A, B, C)
    
    % 獲取螢幕尺寸
    scrsz = get(0, 'ScreenSize');
    % 定義視窗大小 (800x800)
    get_pos = @() [(scrsz(3)-800)/2, (scrsz(4)-800)/2, 800, 800];
    
    %% 視窗 1: 末端位置 (End-effector Position)
    figure('Name', 'Cartesian Move - End-effector Position', 'NumberTitle', 'off', 'Position', get_pos());
    titles = {'Position of x', 'Position of y', 'Position of z'};
    for i = 1:3
        subplot(3, 1, i);
        plot(t, cart_pos(:, i), 'LineWidth', 1.5);
        grid on;
        title(titles{i});
        ylabel('cm');
        if i == 3, xlabel('Time (s)'); end
        xlim([0, t(end)]);
    end
    
    %% 視窗 2: 末端速度 (End-effector Velocity)
    figure('Name', 'Cartesian Move - End-effector Velocity', 'NumberTitle', 'off', 'Position', get_pos());
    titles = {'Velocity of x', 'Velocity of y', 'Velocity of z'};
    for i = 1:3
        subplot(3, 1, i);
        plot(t, cart_vel(:, i), 'LineWidth', 1.5);
        grid on;
        title(titles{i});
        ylabel('cm/s');
        if i == 3, xlabel('Time (s)'); end
        xlim([0, t(end)]);
    end
    
    %% 視窗 3: 末端加速度 (End-effector Acceleration)
    figure('Name', 'Cartesian Move - End-effector Acceleration', 'NumberTitle', 'off', 'Position', get_pos());
    titles = {'Acceleration of x', 'Acceleration of y', 'Acceleration of z'};
    for i = 1:3
        subplot(3, 1, i);
        plot(t, cart_acc(:, i), 'LineWidth', 1.5);
        grid on;
        title(titles{i});
        ylabel('cm/s^2');
        if i == 3, xlabel('Time (s)'); end
        xlim([0, t(end)]);
    end
    
end