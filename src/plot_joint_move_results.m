% plot_joint_move_results.m
% ========================================
% 繪製關節空間軌跡規劃結果 (符合範例排版: 4張圖)
% 1. Angle (6 subplots)
% 2. Angular Velocity (6 subplots)
% 3. Angular Acceleration (6 subplots)
% 4. 3D Plot (Path with A, B, C points)
% ========================================

function plot_joint_move_results(t, q, qd, qdd, cart_pos, cart_vel, cart_acc, A, B, C)
    
    % 獲取螢幕尺寸以利視窗置中
    scrsz = get(0, 'ScreenSize');
    
    % 定義 2D 曲線視窗大小 (寬 x 高)
    fig_w_2d = 1000; fig_h_2d = 700;
    get_pos_2d = @() [(scrsz(3)-fig_w_2d)/2, (scrsz(4)-fig_h_2d)/2, fig_w_2d, fig_h_2d];
    
    % 定義 3D 圖視窗大小
    fig_w_3d = 800; fig_h_3d = 600;
    get_pos_3d = @() [(scrsz(3)-fig_w_3d)/2, (scrsz(4)-fig_h_3d)/2, fig_w_3d, fig_h_3d];

    %% ========================================
    %% Figure 1: Joint Angles (符合 image_54fed6.png)
    %% ========================================
    figure('Name', 'Joint move - angle', 'NumberTitle', 'off', 'Position', get_pos_2d());
    % 整體標題 (若 Matlab 版本較舊不支援 sgtitle 可註解掉)
    try sgtitle('呈現結果範例(a) Joint move – angle', 'FontSize', 16); catch, end
    
    for i = 1:6
        subplot(3, 2, i);
        plot(t, q(:, i), 'LineWidth', 1.2);
        grid on;
        
        % 標題加粗
        title(sprintf('Joint %d', i), 'FontWeight', 'bold');
        
        % Y 軸標籤 (左側欄位顯示，J3 顯示 inch，其餘 deg)
        if mod(i, 2) == 1
            if i == 3
                ylabel('d3 (inch)'); % Joint 3 是移動關節
            else
                ylabel('angle (degree)');
            end
        end
        
        % X 軸標籤 (僅最下列顯示)
        if i >= 5
            xlabel('time (s)');
        end
        
        xlim([0, t(end)]);
    end
    
    %% ========================================
    %% Figure 2: Joint Velocities (符合 image_54feb7.png)
    %% ========================================
    figure('Name', 'Joint move - angular velocity', 'NumberTitle', 'off', 'Position', get_pos_2d());
    try sgtitle('呈現結果範例(a) Joint move – angular velocity', 'FontSize', 16); catch, end
    
    for i = 1:6
        subplot(3, 2, i);
        plot(t, qd(:, i), 'LineWidth', 1.2);
        grid on;
        
        title(sprintf('Joint %d', i), 'FontWeight', 'bold');
        
        if mod(i, 2) == 1
            if i == 3
                ylabel('velocity (inch/s)');
            else
                ylabel('angular velocity(degree/s)');
            end
        end
        
        if i >= 5
            xlabel('time (s)');
        end
        xlim([0, t(end)]);
    end
    
    %% ========================================
    %% Figure 3: Joint Accelerations (符合 image_54fe9a.png)
    %% ========================================
    figure('Name', 'Joint move - angular acceleration', 'NumberTitle', 'off', 'Position', get_pos_2d());
    try sgtitle('呈現結果範例(a) Joint move – angular acceleration', 'FontSize', 16); catch, end
    
    for i = 1:6
        subplot(3, 2, i);
        plot(t, qdd(:, i), 'LineWidth', 1.2);
        grid on;
        
        title(sprintf('Joint %d', i), 'FontWeight', 'bold');
        
        if mod(i, 2) == 1
            if i == 3
                ylabel('acceleration (inch/s^2)');
            else
                ylabel('angular acceleration(degree/s^2)');
            end
        end
        
        if i >= 5
            xlabel('time (s)');
        end
        xlim([0, t(end)]);
    end
    
    %% ========================================
    %% Figure 4: 3D Plot (符合 image_54fe59.png)
    %% ========================================
    figure('Name', 'Joint move - 3D plot', 'NumberTitle', 'off', 'Position', get_pos_3d());
    
    % 繪製路徑 (Cyan 色線條)
    plot3(cart_pos(:, 1), cart_pos(:, 2), cart_pos(:, 3), 'c-', 'LineWidth', 2.5);
    hold on;
    
    % 提取 A, B, C 座標 (單位: cm)
    p_A = A(1:3, 4);
    p_B = B(1:3, 4);
    p_C = C(1:3, 4);
    
    % 標示點 (紅色星號)
    plot3(p_A(1), p_A(2), p_A(3), 'r*', 'MarkerSize', 8, 'LineWidth', 1.5);
    plot3(p_B(1), p_B(2), p_B(3), 'r*', 'MarkerSize', 8, 'LineWidth', 1.5);
    plot3(p_C(1), p_C(2), p_C(3), 'r*', 'MarkerSize', 8, 'LineWidth', 1.5);
    
    % 標示文字座標
    text(p_A(1), p_A(2), p_A(3), sprintf('  A(%.0f, %.0f, %.0f)', p_A), 'FontSize', 10);
    text(p_B(1), p_B(2), p_B(3), sprintf('  B(%.0f, %.0f, %.0f)', p_B), 'FontSize', 10);
    text(p_C(1), p_C(2), p_C(3), sprintf('  C(%.0f, %.0f, %.0f)', p_C), 'FontSize', 10);
    
    grid on; axis equal;
    xlabel('x(cm)'); ylabel('y(cm)'); zlabel('z(cm)');
    title('3D path of Joint motion', 'FontWeight', 'bold', 'FontSize', 12);
    try sgtitle('呈現結果範例(a) Joint move – 3D plot', 'FontSize', 16); catch, end
    
    view(45, 30); % 調整視角
    hold off;
    
end