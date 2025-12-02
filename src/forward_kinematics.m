% forward_kinematics.m (修正版)
% 輸入: 1x6 的關節變數向量 [th1, th2, d3, th4, th5, th6] (角度單位)
% 輸出: 4x4 姿態矩陣 T, 以及 1x6 姿態向量 [x, y, z, phi, theta, psi]
% 歐拉角使用 ZYZ 順序

function [T, pose] = forward_kinematics(joint_vars)
    % 將角度轉換為弧度
    th1_rad = deg2rad(joint_vars(1));
    th2_rad = deg2rad(joint_vars(2));
    d3 = joint_vars(3);
    th4_rad = deg2rad(joint_vars(4));
    th5_rad = deg2rad(joint_vars(5));
    th6_rad = deg2rad(joint_vars(6));
    
    % DH 參數
    d = [0, 6.375, d3, 0, 0, 0];
    a = [0, 0, 0, 0, 0, 0];
    alpha = deg2rad([-90, 90, 0, -90, 90, 0]);
    
    % 建立各軸的轉換矩陣
    A1 = dh_matrix(d(1), a(1), alpha(1), th1_rad);
    A2 = dh_matrix(d(2), a(2), alpha(2), th2_rad);
    A3 = dh_matrix(d(3), a(3), alpha(3), 0); % theta3 is fixed at 0
    A4 = dh_matrix(d(4), a(4), alpha(4), th4_rad);
    A5 = dh_matrix(d(5), a(5), alpha(5), th5_rad);
    A6 = dh_matrix(d(6), a(6), alpha(6), th6_rad);
    
    % 計算最終的姿態矩陣
    T = A1 * A2 * A3 * A4 * A5 * A6;
    
    % 提取 x, y, z
    x = T(1, 4);
    y = T(2, 4);
    z = T(3, 4);
    
    % 從旋轉矩陣計算 ZYZ 尤拉角 (phi, theta, psi)
    R = T(1:3, 1:3);
    
    % ZYZ 歐拉角標準公式
    % R = Rz(φ) * Ry(θ) * Rz(ψ)
    phi = atan2(R(2,3), R(1,3));
    theta = atan2(sqrt(R(1,3)^2 + R(2,3)^2), R(3,3));
    psi = atan2(R(3,2), -R(3,1));

    pose = [x, y, z, rad2deg(phi), rad2deg(theta), rad2deg(psi)];
end

function A = dh_matrix(d, a, alpha, theta)
    % DH 轉換矩陣的輔助函式
    A = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
         sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0,           sin(alpha),             cos(alpha),            d;
         0,           0,                      0,                     1];
end