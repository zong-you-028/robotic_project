% inverse_kinematics.m 
% 求解 Stanford Manipulator 的逆向運動學
% ========================================
% 輸入: 4x4 齊次轉換矩陣 T (目標位置與姿態)
% 輸出: Nx6 解矩陣 [th1, th2, d3, th4, th5, th6]
%       - 角度單位：度 (degrees)
%       - d3 單位：英吋 (inches)
% ========================================
% 演算法：代數法 (Algebraic Method)
% - 先解位置 (Position): θ1, θ2, d3
% - 再解姿態 (Orientation): θ4, θ5, θ6 (ZYZ 歐拉角)
% ========================================

function solutions = inverse_kinematics(T)
    
    solutions = []; % 初始化解矩陣
    
    %% --- 常數定義 (來自 DH Table) ---
    d2 = 6.375; % Link 2 的偏移量 (英吋)
    
    %% --- 從目標矩陣 T 提取資訊 ---
    p = T(1:3, 4);      % 位置向量 [Px; Py; Pz]
    R_target = T(1:3, 1:3); % 旋轉矩陣
    
    Pcx = p(1);
    Pcy = p(2);
    Pcz = p(3);

    %% ========================================
    %% 步驟 1: 求解 d3 (位置求解)
    %% ========================================
    % 從 FK 推導: Px^2 + Py^2 + Pz^2 = d2^2 + d3^2
    % 因此: d3^2 = Px^2 + Py^2 + Pz^2 - d2^2
    
    R_squared = Pcx^2 + Pcy^2 + Pcz^2;
    d3_val_squared = R_squared - d2^2;
    
    % 可達性檢查
    if d3_val_squared < -1e-6
        fprintf('錯誤: 目標位置無法到達 (過於接近基座)。\n');
        return;
    elseif abs(d3_val_squared) < 1e-6
        d3_val_squared = 0; % 數值容忍
    end
    
    % d3 有兩個解: ±sqrt(d3_val_squared)
    d3_solutions = [sqrt(d3_val_squared), -sqrt(d3_val_squared)];
    
    %% ========================================
    %% 迴圈 1: 遍歷 d3 的兩個解
    %% ========================================
    for d3 = d3_solutions
        
        %% 步驟 2: 求解 θ2 (肩部關節)
        % 從 FK: Pz = cos(θ2) * d3
        if abs(d3) < 1e-6
            % 奇異點: d3 ≈ 0，Pz 必須也 ≈ 0
            if abs(Pcz) > 1e-6
                continue; % 此 d3 解無效
            else
                continue; % d3=0 不是典型的可達點
            end
        end
        
        cos_theta2 = Pcz / d3;
        
        % 確保 cos 值在 [-1, 1] 範圍內
        if abs(cos_theta2) > 1
            cos_theta2 = sign(cos_theta2);
        end
        
        % θ2 有兩個解 (手臂向上/向下)
        sin_theta2_solutions = [sqrt(1 - cos_theta2^2), -sqrt(1 - cos_theta2^2)];
        
        %% ========================================
        %% 迴圈 2: 遍歷 θ2 的兩個解
        %% ========================================
        for sin_theta2 = sin_theta2_solutions
            theta2_rad = atan2(sin_theta2, cos_theta2);
            
            %% 步驟 3: 求解 θ1 (基座旋轉)
            % 從 FK:
            %   Px = cos(θ1) * sin(θ2) * d3 - sin(θ1) * d2
            %   Py = sin(θ1) * sin(θ2) * d3 + cos(θ1) * d2
            % 使用 atan2 求解
            k1 = d2;
            k2 = sin_theta2 * d3;
            theta1_rad = atan2(Pcy, Pcx) - atan2(k1, k2);
            
            %% 此時已得到完整的位置解: (θ1, θ2, d3)
            
            %% ========================================
            %% 步驟 4: 求解腕部姿態 (θ4, θ5, θ6)
            %% ========================================
            % 計算 R_0^3 (從基座到第3關節的旋轉矩陣)
            c1 = cos(theta1_rad); 
            s1 = sin(theta1_rad);
            c2 = cos(theta2_rad); 
            s2 = sin(theta2_rad);
            
            R_3_0 = [ c1*c2, -s1,  c1*s2;
                      s1*c2,  c1,  s1*s2;
                      -s2,    0,   c2  ];
            
            % 計算腕部旋轉矩陣 R_3^6
            R_6_3 = R_3_0' * R_target;
            
            %% 從 R_3^6 求解 ZYZ 歐拉角 (θ4, θ5, θ6)
            cos_theta5 = R_6_3(3,3);
            
            % 確保 cos 值在 [-1, 1] 範圍內
            if abs(cos_theta5) > 1
                cos_theta5 = sign(cos_theta5);
            end
            
            % θ5 有兩個解 (腕部翻轉)
            sin_theta5_solutions = [sqrt(1 - cos_theta5^2), -sqrt(1 - cos_theta5^2)];
            
            %% ========================================
            %% 迴圈 3: 遍歷 θ5 的兩個解
            %% ========================================
            for sin_theta5 = sin_theta5_solutions
                theta5_rad = atan2(sin_theta5, cos_theta5);
                
                % 檢查萬向鎖 (Gimbal Lock) 情況
                if abs(sin_theta5) < 1e-6
                    % θ5 ≈ 0° 或 180°，θ4 和 θ6 不唯一
                    theta4_rad = 0; % 任意設定 θ4 = 0
                    theta6_rad = atan2(-R_6_3(1,2), R_6_3(1,1));
                else
                    % 標準求解
                    theta4_rad = atan2(R_6_3(2,3), R_6_3(1,3));
                    theta6_rad = atan2(R_6_3(3,2), -R_6_3(3,1));
                end
                
                %% ========================================
                %% 儲存解 (轉換為度並組合)
                %% ========================================
                % 將所有角度從弧度轉換為度
                angles_rad = [theta1_rad, theta2_rad, theta4_rad, theta5_rad, theta6_rad];
                angles_deg = rad2deg(angles_rad);
                
                % 組合最終解 [θ1, θ2, d3, θ4, θ5, θ6]
                solution = [angles_deg(1), angles_deg(2), d3, angles_deg(3), angles_deg(4), angles_deg(5)];
                
                % 加入解矩陣
                solutions(end+1, :) = solution;
                
            end % end θ5 迴圈
        end % end θ2 迴圈
    end % end d3 迴圈
    

end % end function