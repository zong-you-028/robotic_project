% plot_joint_move_results.m
% ========================================
% 繪製關節空間軌跡規劃結果 (符合PDF範例: 3個視窗，置中)
% ========================================

function plot_joint_move_results(t, q, qd, qdd, cart_pos, cart_vel, cart_acc, A, B, C)
    
    % 獲取螢幕尺寸
    scrsz = get(0, 'ScreenSize');
    
    % 定義視窗大小與置中函數 (1000x700)
    get_pos = @() [(scrsz(3)-1000)/2, (scrsz(4)-700)/2, 1000, 700];
    
    %% 視窗 1: 關節角度 (Joint Angles)
    figure('Name', 'Joint Move - Joint Angles', 'NumberTitle', 'off', 'Position', get_pos());
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
    figure('Name', 'Joint Move - Joint Velocities', 'NumberTitle', 'off', 'Position', get_pos());
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
    figure('Name', 'Joint Move - Joint Accelerations', 'NumberTitle', 'off', 'Position', get_pos());
    for i = 1:6
        subplot(3, 2, i);
        plot(t, qdd(:, i), 'LineWidth', 1.5);
        grid on;
        if i == 3, ylabel('inch/s^2'); else, ylabel('deg/s^2'); end
        xlabel('time (s)');
        title(sprintf('Joint %d', i));
        xlim([0, t(end)]);
    end
    
    % (依據PDF範例，Joint Move 主要是看關節變數，若需要末端也可以加，但為保持整潔先以上述為主)
    % 若您也希望看到末端軌跡的數據圖，可以再增加。
end