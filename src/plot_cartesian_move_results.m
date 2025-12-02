% plot_cartesian_move_results.m
% ========================================
% 繪製笛卡爾空間軌跡規劃結果
% ========================================

function plot_cartesian_move_results(t, cart_pos, cart_vel, cart_acc, q, qd, qdd, A, B, C)
    
    %% 圖1: 3D 笛卡爾空間軌跡
    figure('Name', 'Cartesian Move - 3D Path', 'NumberTitle', 'off');
    plot3(cart_pos(:,1), cart_pos(:,2), cart_pos(:,3), 'b-', 'LineWidth', 2);
    hold on;
    grid on;
    
    % 標記起點、中間點、終點
    plot3(A(1,4), A(2,4), A(3,4), 'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g');
    plot3(B(1,4), B(2,4), B(3,4), 'ro', 'MarkerSize', 12, 'MarkerFaceColor', 'r');
    plot3(C(1,4), C(2,4), C(3,4), 'mo', 'MarkerSize', 12, 'MarkerFaceColor', 'm');
    
    % 添加標籤
    text(A(1,4), A(2,4), A(3,4), '  A', 'FontSize', 14, 'Color', 'g');
    text(B(1,4), B(2,4), B(3,4), '  B', 'FontSize', 14, 'Color', 'r');
    text(C(1,4), C(2,4), C(3,4), '  C', 'FontSize', 14, 'Color', 'm');
    
    xlabel('x (cm)', 'FontSize', 12);
    ylabel('y (cm)', 'FontSize', 12);
    zlabel('z (cm)', 'FontSize', 12);
    title('3D path of Cartesian motion', 'FontSize', 14);
    view(45, 30);
    axis equal;
    
    %% 圖2: 末端位置 (x, y, z)
    figure('Name', 'Cartesian Move - Position', 'NumberTitle', 'off');
    
    subplot(3, 1, 1);
    plot(t, cart_pos(:, 1), 'b-', 'LineWidth', 1.5);
    grid on;
    ylabel('Position (cm)', 'FontSize', 10);
    title('Position of x', 'FontSize', 12);
    
    subplot(3, 1, 2);
    plot(t, cart_pos(:, 2), 'b-', 'LineWidth', 1.5);
    grid on;
    ylabel('Position (cm)', 'FontSize', 10);
    title('Position of y', 'FontSize', 12);
    
    subplot(3, 1, 3);
    plot(t, cart_pos(:, 3), 'b-', 'LineWidth', 1.5);
    grid on;
    xlabel('Time (s)', 'FontSize', 10);
    ylabel('Position (cm)', 'FontSize', 10);
    title('Position of z', 'FontSize', 12);
    
    %% 圖3: 末端速度
    figure('Name', 'Cartesian Move - Velocity', 'NumberTitle', 'off');
    
    subplot(3, 1, 1);
    plot(t, cart_vel(:, 1), 'b-', 'LineWidth', 1.5);
    grid on;
    ylabel('Velocity (cm/s)', 'FontSize', 10);
    title('Velocity of x', 'FontSize', 12);
    
    subplot(3, 1, 2);
    plot(t, cart_vel(:, 2), 'b-', 'LineWidth', 1.5);
    grid on;
    ylabel('Velocity (cm/s)', 'FontSize', 10);
    title('Velocity of y', 'FontSize', 12);
    
    subplot(3, 1, 3);
    plot(t, cart_vel(:, 3), 'b-', 'LineWidth', 1.5);
    grid on;
    xlabel('Time (s)', 'FontSize', 10);
    ylabel('Velocity (cm/s)', 'FontSize', 10);
    title('Velocity of z', 'FontSize', 12);
    
    %% 圖4: 末端加速度
    figure('Name', 'Cartesian Move - Acceleration', 'NumberTitle', 'off');
    
    subplot(3, 1, 1);
    plot(t, cart_acc(:, 1), 'b-', 'LineWidth', 1.5);
    grid on;
    ylabel('Acceleration (cm/s^2)', 'FontSize', 10);
    title('Acceleration of x', 'FontSize', 12);
    
    subplot(3, 1, 2);
    plot(t, cart_acc(:, 2), 'b-', 'LineWidth', 1.5);
    grid on;
    ylabel('Acceleration (cm/s^2)', 'FontSize', 10);
    title('Acceleration of y', 'FontSize', 12);
    
    subplot(3, 1, 3);
    plot(t, cart_acc(:, 3), 'b-', 'LineWidth', 1.5);
    grid on;
    xlabel('Time (s)', 'FontSize', 10);
    ylabel('Acceleration (cm/s^2)', 'FontSize', 10);
    title('Acceleration of z', 'FontSize', 12);
    
    %% 圖5: 關節角度變化
    figure('Name', 'Cartesian Move - Joint Angles', 'NumberTitle', 'off');
    for i = 1:6
        subplot(3, 2, i);
        plot(t, q(:, i), 'b-', 'LineWidth', 1.5);
        grid on;
        xlabel('time (s)', 'FontSize', 10);
        ylabel('angle (degree)', 'FontSize', 10);
        title(sprintf('Joint %d', i), 'FontSize', 12);
    end
    
    %% 圖6: 關節角速度
    figure('Name', 'Cartesian Move - Angular Velocity', 'NumberTitle', 'off');
    for i = 1:6
        subplot(3, 2, i);
        plot(t, qd(:, i), 'b-', 'LineWidth', 1.5);
        grid on;
        xlabel('time (s)', 'FontSize', 10);
        ylabel('angular velocity (degree/s)', 'FontSize', 10);
        title(sprintf('Joint %d', i), 'FontSize', 12);
    end
    
    %% 圖7: 關節角加速度
    figure('Name', 'Cartesian Move - Angular Acceleration', 'NumberTitle', 'off');
    for i = 1:6
        subplot(3, 2, i);
        plot(t, qdd(:, i), 'b-', 'LineWidth', 1.5);
        grid on;
        xlabel('time (s)', 'FontSize', 10);
        ylabel('angular acceleration (degree/s^2)', 'FontSize', 10);
        title(sprintf('Joint %d', i), 'FontSize', 12);
    end
    
end