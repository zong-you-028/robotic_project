% plot_cartesian_move_results.m
% ========================================
% 繪製笛卡爾空間軌跡規劃結果 (還原 6 個獨立視窗 + 分開繪製)
% ========================================

function plot_cartesian_move_results(t, cart_pos, cart_vel, cart_acc, q, qd, qdd, A, B, C)
    
    % 獲取螢幕尺寸
    scrsz = get(0, 'ScreenSize');
    get_pos_joint = @() [(scrsz(3)-1000)/2, (scrsz(4)-700)/2, 1000, 700];
    get_pos_cart = @() [(scrsz(3)-800)/2, (scrsz(4)-800)/2, 800, 800];
    
    %% 視窗 1: 末端位置 (End-effector Position)
    figure('Name', 'Cartesian Move - End-effector Position', 'NumberTitle', 'off', 'Position', get_pos_cart());
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
    figure('Name', 'Cartesian Move - End-effector Velocity', 'NumberTitle', 'off', 'Position', get_pos_cart());
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
    figure('Name', 'Cartesian Move - End-effector Acceleration', 'NumberTitle', 'off', 'Position', get_pos_cart());
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

    %% 視窗 4: 關節角度 (Joint Angles)
    figure('Name', 'Cartesian Move - Joint Angles', 'NumberTitle', 'off', 'Position', get_pos_joint());
    for i = 1:6
        subplot(3, 2, i);
        plot(t, q(:, i), 'LineWidth', 1.5);
        grid on;
        if i == 3, ylabel('d3 (inch)'); else, ylabel('deg'); end
        xlabel('time (s)');
        title(sprintf('Joint %d', i));
        xlim([0, t(end)]);
    end
    
    %% 視窗 5: 關節速度 (Joint Velocities)
    figure('Name', 'Cartesian Move - Joint Velocities', 'NumberTitle', 'off', 'Position', get_pos_joint());
    for i = 1:6
        subplot(3, 2, i);
        plot(t, qd(:, i), 'LineWidth', 1.5);
        grid on;
        if i == 3, ylabel('inch/s'); else, ylabel('deg/s'); end
        xlabel('time (s)');
        title(sprintf('Joint %d', i));
        xlim([0, t(end)]);
    end
    
    %% 視窗 6: 關節加速度 (Joint Accelerations)
    figure('Name', 'Cartesian Move - Joint Accelerations', 'NumberTitle', 'off', 'Position', get_pos_joint());
    for i = 1:6
        subplot(3, 2, i);
        plot(t, qdd(:, i), 'LineWidth', 1.5);
        grid on;
        if i == 3, ylabel('inch/s^2'); else, ylabel('deg/s^2'); end
        xlabel('time (s)');
        title(sprintf('Joint %d', i));
        xlim([0, t(end)]);
    end
    
end