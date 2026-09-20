function compareNavResults(nav_path_posmind, nav_path_self, save_dir)
% COMPARENAVRESULTS 生成二维轨迹 + 位置/速度/姿态 子图(3x1)

    %% 1. 读取并解析文件
    fprintf('正在读取 POSMind 数据...\n');
    nav_posmind = load(nav_path_posmind);
    fprintf('正在读取 自编程序 数据...\n');
    nav_self = load(nav_path_self);

    % --- 解析 POSMind (10列) ---
    t_pos = nav_posmind(:, 1);
    blh_pos = nav_posmind(:, 2:4);        
    vel_pos_org = nav_posmind(:, 5:7);   % [vE, vN, vU]
    vel_pos = [vel_pos_org(:, 2), vel_pos_org(:, 1), -vel_pos_org(:, 3)]; % 转 [vN, vE, vD]
    att_pos = nav_posmind(:, 8:10);      % [roll, pitch, yaw]

    % --- 解析自编程序 (11列) ---
    t_self = nav_self(:, 2);
    blh_self = nav_self(:, 3:5);         
    vel_self = nav_self(:, 6:8);         
    att_self = nav_self(:, 9:11);        

    %% 2. 坐标转换 (经纬高 -> 局部 NED 米坐标系)
    D2R = pi / 180.0;
    ref_blh = blh_pos(1, :) * D2R;
    ref_lat = ref_blh(1);
    
    a = 6378137.0; f = 1/298.257223563; e2 = 2*f - f^2;
    RM = a * (1 - e2) / (1 - e2 * sin(ref_lat)^2)^(3/2);
    RN = a / sqrt(1 - e2 * sin(ref_lat)^2);
    DR = diag([RM + ref_blh(3), (RN + ref_blh(3))*cos(ref_lat), -1]);

    pos_ned_pos = zeros(length(t_pos), 3);
    blh_pos_rad = blh_pos * D2R;
    for i = 1:length(t_pos)
        pos_ned_pos(i, :) = (DR * (blh_pos_rad(i, :) - ref_blh)')';
    end

    pos_ned_self = zeros(length(t_self), 3);
    blh_self_rad = blh_self * D2R;
    for i = 1:length(t_self)
        pos_ned_self(i, :) = (DR * (blh_self_rad(i, :) - ref_blh)')';
    end

    %% 3. 对齐时间轴与插值
    t_start = max(t_pos(1), t_self(1));
    t_end   = min(t_pos(end), t_self(end));
    idx_pos = t_pos >= t_start & t_pos <= t_end;
    idx_self = t_self >= t_start & t_self <= t_end;
    
    t_ref = t_pos(idx_pos);
    
    pos_ned_self_interp = interp1(t_self(idx_self), pos_ned_self(idx_self, :), t_ref, 'linear');
    vel_self_interp = interp1(t_self(idx_self), vel_self(idx_self, :), t_ref, 'linear');
    att_self_interp = interp1(t_self(idx_self), att_self(idx_self, :), t_ref, 'linear');

    pos_ned_pos_ref = pos_ned_pos(idx_pos, :);
    vel_pos_ref = vel_pos(idx_pos, :);
    att_pos_ref = att_pos(idx_pos, :);

    %% 4. 角度映射修正 
    att_self_corrected = att_self_interp; 
    att_self_corrected(:, 3) = att_self_interp(:, 3) + 90; % Yaw+90

    if ~exist(save_dir, 'dir')
        mkdir(save_dir);
    end

    %% =============== 5. 绘制图表 ===============

    % --- 图1. 二维轨迹对比图 ---
    figure('Name', '二维轨迹对比', 'Color', 'w');
    plot(pos_ned_pos_ref(:, 2), pos_ned_pos_ref(:, 1), 'r-', 'LineWidth', 1.5); hold on;
    plot(pos_ned_self(:, 2), pos_ned_self(:, 1), 'b--', 'LineWidth', 1.5);
    title('二维轨迹对比'); xlabel('East [m]'); ylabel('North [m]');
    legend('POSMind (参考)', '自编程序', 'Location', 'best');
    grid on; axis equal;
    saveas(gcf, fullfile(save_dir, 'trajectory_2d.png'));
    fprintf('已保存: trajectory_2d.png\n');

    % --- 图2. 位置三轴误差 (3个子图) ---
    figure('Name', '位置三轴误差', 'Color', 'w');
    tiledlayout(3, 1);

    nexttile; plot(t_ref, pos_ned_pos_ref(:, 1) - pos_ned_self_interp(:, 1), 'k-', 'LineWidth', 1);
    title('位置误差 (North)'); ylabel('Error [m]'); grid on;

    nexttile; plot(t_ref, pos_ned_pos_ref(:, 2) - pos_ned_self_interp(:, 2), 'k-', 'LineWidth', 1);
    title('位置误差 (East)'); ylabel('Error [m]'); grid on;

    nexttile; plot(t_ref, pos_ned_pos_ref(:, 3) - pos_ned_self_interp(:, 3), 'k-', 'LineWidth', 1);
    title('位置误差 (Down)'); ylabel('Error [m]'); grid on;
    xlabel('Time [s]');
    saveas(gcf, fullfile(save_dir, 'pos_error_3subplots.png'));
    fprintf('已保存: pos_error_3subplots.png\n');

    % --- 图3. 速度三轴误差 (3个子图) ---
    figure('Name', '速度三轴误差', 'Color', 'w');
    tiledlayout(3, 1);

    nexttile; plot(t_ref, vel_pos_ref(:, 1) - vel_self_interp(:, 1), 'b-', 'LineWidth', 1);
    title('速度误差 (North)'); ylabel('Error [m/s]'); grid on;

    nexttile; plot(t_ref, vel_pos_ref(:, 2) - vel_self_interp(:, 2), 'b-', 'LineWidth', 1);
    title('速度误差 (East)'); ylabel('Error [m/s]'); grid on;

    nexttile; plot(t_ref, vel_pos_ref(:, 3) - vel_self_interp(:, 3), 'b-', 'LineWidth', 1);
    title('速度误差 (Down)'); ylabel('Error [m/s]'); grid on;
    xlabel('Time [s]');
    saveas(gcf, fullfile(save_dir, 'vel_error_3subplots.png'));
    fprintf('已保存: vel_error_3subplots.png\n');

    % --- 图4. 姿态三轴误差 (3个子图) ---
    % 1. 计算横滚角误差并修正边界
    roll_diff = att_pos_ref(:, 1) - att_self_corrected(:, 1);
    roll_diff(roll_diff > 90) = roll_diff(roll_diff > 90) - 180;
    roll_diff(roll_diff < -90) = roll_diff(roll_diff < -90) + 180;
    
    % 2. 计算俯仰角误差并修正边界
    pitch_diff = att_pos_ref(:, 2) - att_self_corrected(:, 2);
    idx_p = abs(pitch_diff) > 90;
    pitch_diff(idx_p) = atan2(sin(pitch_diff(idx_p)*pi/180), cos(pitch_diff(idx_p)*pi/180)) * 180/pi;
    
    % 3. 计算航向角误差并修正边界
    yaw_diff = att_pos_ref(:, 3) - att_self_corrected(:, 3);
    idx_y = abs(yaw_diff) > 180;
    yaw_diff(idx_y) = atan2(sin(yaw_diff(idx_y)*pi/180), cos(yaw_diff(idx_y)*pi/180)) * 180/pi;

    figure('Name', '姿态三轴误差', 'Color', 'w');
    tiledlayout(3, 1);

    nexttile; plot(t_ref, roll_diff, 'r-', 'LineWidth', 1);
    title('横滚角误差 (Roll)'); ylabel('Error [deg]'); grid on;

    nexttile; plot(t_ref, pitch_diff, 'r-', 'LineWidth', 1);
    title('俯仰角误差 (Pitch)'); ylabel('Error [deg]'); grid on;

    nexttile; plot(t_ref, yaw_diff, 'r-', 'LineWidth', 1);
    title('航向角误差 (Yaw)'); ylabel('Error [deg]'); grid on;
    xlabel('Time [s]');
    saveas(gcf, fullfile(save_dir, 'att_error_3subplots.png'));
    fprintf('已保存: att_error_3subplots.png\n');

    %% 6. 控制台输出统计
    pos_rms_2d = sqrt(mean((pos_ned_pos_ref(:, 1) - pos_ned_self_interp(:, 1)).^2 + ...
                           (pos_ned_pos_ref(:, 2) - pos_ned_self_interp(:, 2)).^2));
    fprintf('\n============= 对比评估总结 =============\n');
    fprintf('平面位置误差 RMS: %.4f 米\n', pos_rms_2d);
    fprintf('========================================\n');
end