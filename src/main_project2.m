% project2_main.m
% ========================================
% Stanford Manipulator 軌跡規劃主程式
% ========================================
% 功能:
%   (a) Joint move (關節空間軌跡規劃)
%   (b) Cartesian move (笛卡爾空間軌跡規劃)
% ========================================

clc; clear; close all;

%% 定義目標點 A, B, C (單位: cm)

A = [0  0  1  15;
     1  0  0  25;
     0  1  0  35;
     0  0  0  1];

B = [0   1  0  10;
    -1   0  0  15;
     0   0  1  20;
     0   0  0  1];

C = [0  -1  0 -10;
     1   0  0  30;
     0   0  1 -15;
     0   0  0  1];

%% 運動參數設定
t_AB = 0.5;      % A到B的時間 (sec)
t_BC = 0.5;      % B到C的時間 (sec)
t_acc = 0.2;     % 過渡段加速時間 (sec)
dt = 0.002;      % 採樣時間 (sec)

%% 選擇功能
fprintf('========================================\n');
fprintf('  Stanford Manipulator 軌跡規劃\n');
fprintf('========================================\n');
fprintf('(a) Joint move - 關節空間軌跡規劃\n');
fprintf('(b) Cartesian move - 笛卡爾空間軌跡規劃\n');
fprintf('========================================\n');
choice = input('請選擇功能 (a/b): ', 's');

if strcmpi(choice, 'a')
    %% ========================================
    %% (a) Joint Move - 關節空間軌跡規劃
    %% ========================================
    fprintf('\n執行關節空間軌跡規劃...\n');
    
    % 執行 Joint move
    joint_move_planning(A, B, C, t_AB, t_BC, t_acc, dt);
    
elseif strcmpi(choice, 'b')
    %% ========================================
    %% (b) Cartesian Move - 笛卡爾空間軌跡規劃
    %% ========================================
    fprintf('\n執行笛卡爾空間軌跡規劃...\n');
    
    % 執行 Cartesian move
    cartesian_move_planning(A, B, C, t_AB, t_BC, t_acc, dt);
    
else
    fprintf('無效的選擇。\n');
end

fprintf('\n軌跡規劃完成！\n');