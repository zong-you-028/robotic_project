% plot_joint_move_results.m
% ========================================
% 繪製關節空間軌跡規劃結果 (還原 6 個獨立視窗 + 分開繪製)
% ========================================

function plot_joint_move_results(t, q, qd, qdd, cart_pos, cart_vel, cart_acc, A, B, C)
    
    % 獲取螢幕尺寸
    scrsz = get(0, 'ScreenSize');
    
    % 定義視窗大小與置中函數
    % 關節圖表 (3x2) 稍微寬一點
    get_pos_joint = @() [(scrsz(3)-1000)/2, (scrsz(4)-700)/2, 1000, 700];
    % 笛卡爾圖表 (3x1) 稍微窄高一點
    get_pos_cart = @() [(scrsz(3)-800)/2, (scrsz(4)-800)/2, 800, 800];
    
    %% 視窗 1: 關節角度 (Joint Angles)
    figure('Name', 'Joint Move - Joint Angles', 'NumberTitle', 'off', 'Position', get_pos_joint());
    for i = 1:6
        subplot(3, 2, i);
        plot(t, q(:, i), 'LineWidth', 1.5);
        grid on;
        if i == 3, ylabel('d3 (inch)'); else, ylabel('deg'); end
        xlabel('time (s)');
        title(sprintf('Joint %d', i));
        xlim([0, t(end)]);
    end
    
    %% 視窗 2: 關節速度 (Joint Velocities)
    figure('Name', 'Joint Move - Joint Velocities', 'NumberTitle', 'off', 'Position', get_pos_joint());
    for i = 1:6
        subplot(3, 2, i);
        plot(t, qd(:, i), 'LineWidth', 1.5);
        grid on;
        if i == 3, ylabel('inch/s'); else, ylabel('deg/s'); end
        xlabel('time (s)');
        title(sprintf('Joint %d', i));
        xlim([0, t(end)]);
    end
    
    %% 視窗 3: 關節加速度 (Joint Accelerations)
    figure('Name', 'Joint Move - Joint Accelerations', 'NumberTitle', 'off', 'Position', get_pos_joint());
    for i = 1:6
        subplot(3, 2, i);
        plot(t, qdd(:, i), 'LineWidth', 1.5);
        grid on;
        if i == 3, ylabel('inch/s^2'); else, ylabel('deg/s^2'); end
        xlabel('time (s)');
        title(sprintf('Joint %d', i));
        xlim([0, t(end)]);
    end
    
    %% 視窗 4: 末端位置 (End-effector Position)
    figure('Name', 'Joint Move - End-effector Position', 'NumberTitle', 'off', 'Position', get_pos_cart());
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
    
    %% 視窗 5: 末端速度 (End-effector Velocity)
    figure('Name', 'Joint Move - End-effector Velocity', 'NumberTitle', 'off', 'Position', get_pos_cart());
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
    
    %% 視窗 6: 末端加速度 (End-effector Acceleration)
    figure('Name', 'Joint Move - End-effector Acceleration', 'NumberTitle', 'off', 'Position', get_pos_cart());
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