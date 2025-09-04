% solve_question1.m
% 问题一（FY1投放1枚烟幕对M1的干扰）求解脚本
% 输出：数值结果保存在 result_q1.mat 和 result_q1.txt
%       图片保存为 distance_vs_time.png 和 geometry_3d.png
% 说明：脚本包含详细注释，尽量处理中文显示与负号问题

clearvars; close all; clc;

%% ========== 全局显示设置（中文字体与解释器设置） ==========
% 为避免中文显示问题，选择系统中常见的中文字体列表中的一个可用字体
preferredFonts = {'SimHei','Microsoft YaHei','STHeiti','Heiti SC','Arial Unicode MS'};
available = listfonts;
chosenFont = 'Helvetica'; % 备选默认
for i = 1:numel(preferredFonts)
    if any(strcmp(preferredFonts{i}, available))
        chosenFont = preferredFonts{i};
        break;
    end
end

% 关闭默认的 LaTeX 解释器，以避免负号显示问题，并设置默认字体
set(groot, 'defaultTextInterpreter', 'none', ...
           'defaultAxesTickLabelInterpreter', 'none', ...
           'defaultLegendInterpreter', 'none', ...
           'defaultAxesFontName', chosenFont, ...
           'defaultTextFontName', chosenFont);

%% ========== 题目给定常数与初始条件（直接来源于题目说明） ==========
g = 9.8;                   % 重力加速度 (m/s^2)
cloud_radius = 10;         % 烟团有效半径 (m)
cloud_sink_v = 3;          % 烟团下沉速度 (m/s)
cloud_lifetime = 20;      % 烟团在有效半径内可遮蔽时间 (s)

% 无人机 FY1 初始位置
P_FY1 = [17800, 0, 1800];  % (m)

% 无人机受领任务后的投放与飞行参数（题设）
uav_speed = 120;           % UAV 水平速度 (m/s)
t_drop_delay = 1.5;        % 受领任务后投放时延 (s)
fuse_delay = 3.6;          % 引信延时（投放后起爆） (s)

% 导弹 M1 初始位置与速度（朝向假目标原点）
P_m0 = [20000, 0, 2000];   % (m)
missile_speed = 300;       % 导弹速度 (m/s)

% 真目标代表点（由题目说明选择代表点，x=0, y=200, z=5）
P_target = [0, 200, 5];

%% ========== 无人机轨迹、投放点与起爆点计算 ==========
% 飞行方向：朝向假目标（原点）方向的水平单位向量
% 注意：题中 UAV 为“等高度匀速直线飞行”，高度分量不参与水平方向计算
u_horiz = [-P_FY1(1), -P_FY1(2)];            % 指向原点的水平向量
u_horiz = u_horiz / norm(u_horiz);           % 单位向量
v_uav = uav_speed * [u_horiz, 0];            % UAV 三维速度向量（竖直分量为0）

% 投放时刻与投放点
t_drop = t_drop_delay;
P_drop = P_FY1 + v_uav * t_drop;             % 投放点坐标

% 起爆时刻与起爆点（弹体脱离后做抛体运动，初速度等于 UAV 的速度）
t_expl = t_drop + fuse_delay;
% 弹体在引信延时内的位置（忽略空气阻力）
P_c0 = P_drop + v_uav * fuse_delay + [0,0, -0.5 * g * fuse_delay^2]; % 起爆点

% 云团在起爆后中心随时间的运动（下沉）
cloud_t0 = t_expl;                            % 云团形成时刻
cloud_tend = cloud_t0 + cloud_lifetime;      % 云团有效结束时刻

%% ========== 导弹运动参数化 ==========
% 导弹运动方向：指向假目标原点
u_m_dir = -P_m0;                              % 指向原点的向量
u_m_dir = u_m_dir / norm(u_m_dir);            % 单位方向向量
v_missile = missile_speed * u_m_dir;         % 导弹三维速度

% 导弹到达原点的时间（附加信息）
t_missile_arrival = norm(P_m0) / missile_speed;

%% ========== 距离判定函数定义（云团中心到导弹-目标连线的最短距离） ==========
% 我们定义一个函数 F(t) = d(t) - cloud_radius，其中 d(t) 为给定时刻云团中心到
% 导弹到目标连线的最短距离。遮蔽成立当且仅当 F(t) <= 0。

% 内嵌函数：计算点到线段（P_m -> P_target）的最短距离
%  输入：C - 球心 (1x3), Pm - 导弹位置 (1x3), Pt - 目标位置 (1x3)
%  输出：dist - 最短距离 (标量)
point_to_segment_dist = @(C,Pm,Pt) sqrt( max(0, sum((C - (Pm + ...
    max(0, min(1, dot((C - Pm),(Pt - Pm)) / sum((Pt - Pm).^2))) .* (Pt - Pm))).^2)) );

% 为稳定起见，我们将 F(t) 单独实现为一个函数句柄
F = @(t) arrayfun(@(tt) ...
    ( norm( (P_c0 + [0,0,-cloud_sink_v*(tt - cloud_t0)]) - ...
           ( P_m0 + v_missile * tt + ...
             ( clamp_projection(P_c0 + [0,0,-cloud_sink_v*(tt - cloud_t0)] , ...
                                P_m0 + v_missile * tt, P_target) .* (P_target - (P_m0 + v_missile * tt)) ) ) ) - cloud_radius ), ...
    t);

% 辅助函数 clamp_projection 用于计算投影系数 s* 且截断到 [0,1]
% 该函数用作 F 的一部分，放在文件末尾作为本脚本的本地函数（MATLAB R2016b+ 支持）
% 注意：F 使用 arrayfun 包装并在后续使用 fzero/fminbnd 时需要传入标量 t

%% ========== 寻找遮蔽时间区间：粗采样 + 精化求根 ==========
% 搜索区间为云团存在区间与导弹飞行区间的交集
t_search_min = cloud_t0;
t_search_max = min(cloud_tend, t_missile_arrival); % 导弹到达原点后不再考虑
t_search_min, t_search_max; %#ok<ASGLU>

% 粗采样步长（用于识别符号变化区间）
dt_coarse = 1e-3; % 秒，精度与速度折中（0.001 s）
t_grid = t_search_min:dt_coarse:t_search_max;
F_vals = F(t_grid);

% 标记满足遮蔽条件的时间点（粗网格）
mask = (F_vals <= 0);

% 识别连续的遮蔽区间（在粗网格上）
intervals = []; % 每一行表示一个 [t_left, t_right]
in_interval = false;
for k = 1:length(t_grid)
    if mask(k) && ~in_interval
        in_interval = true;
        t_left = t_grid(k);
    elseif ~mask(k) && in_interval
        in_interval = false;
        t_right = t_grid(k-1);
        intervals = [intervals; t_left, t_right];
    end
end
% 若最后仍在区间内，闭合它
if in_interval
    intervals = [intervals; t_left, t_grid(end)];
end

% 若粗网格未发现遮蔽区间，我们还应检测符号变化点以精化根
% 识别 F 值在相邻网格点处是否存在符号变化
sign_changes = [];
for k = 1:length(t_grid)-1
    if (F_vals(k) * F_vals(k+1) < 0)
        sign_changes = [sign_changes; t_grid(k), t_grid(k+1)];
    end
end

% 对每个符号变化区间用 fzero 精化根（双向根）
% 将根两两配对得到极限时间点
refined_roots = [];
if ~isempty(sign_changes)
    for k = 1:size(sign_changes,1)
        a = sign_changes(k,1); b = sign_changes(k,2);
        try
            root = fzero(@(tt) F(tt), [a,b]);
            refined_roots = [refined_roots; root];
        catch
            % 若 fzero 失败，则用中点作为近似
            refined_roots = [refined_roots; 0.5*(a+b)];
        end
    end
    % 对精化根排序并把相邻根配对为区间
    refined_roots = sort(unique(refined_roots));
    if mod(length(refined_roots),2)==0
        intervals = [intervals; reshape(refined_roots,2,[])'];
    end
end

% 若既没有粗网格遮蔽也没有符号变化，但仍可能存在窄小遮蔽（超分辨率），
% 可进一步对最小距离点附近进行局部搜索与二次近似，下面实现最小距离求解并检验
[t_min_approx, d_min_approx] = find_min_distance(F, t_search_min, t_search_max);

% 若 d_min_approx <= 0，则在其附近寻找根（通过二分）
if d_min_approx <= 0
    % 在 t_min_approx 左右扩展小范围找左右边界
    br_range = 0.5; % 最大扩展 0.5 s
    left_bound = max(t_search_min, t_min_approx - br_range);
    right_bound = min(t_search_max, t_min_approx + br_range);
    % 构建精细网格并寻找符号变化
    t_fine = left_bound:1e-4:right_bound;
    F_fine = F(t_fine);
    idx_change = find(F_fine <= 0);
    if ~isempty(idx_change)
        t_left = t_fine(min(idx_change));
        t_right = t_fine(max(idx_change));
        intervals = [intervals; t_left, t_right];
    end
end

% 合并重叠或极近的区间并排序
intervals = merge_intervals(intervals, 1e-6); % 容差 1e-6 s
intervals = sortrows(intervals);

% 计算遮蔽总时长（测度）
total_shadow_time = sum(intervals(:,2) - intervals(:,1));
if isempty(intervals)
    total_shadow_time = 0;
end

%% ========== 输出数值结果并保存文件 ==========
% 显示关键点与结果
fprintf('--- 问题一 关键结果 ---\n');
fprintf('UAV 投放点 P_drop = [%.3f, %.3f, %.3f] m\n', P_drop);
fprintf('烟幕起爆时刻 t_expl = %.3f s\n', t_expl);
fprintf('烟幕起爆点 P_c0 = [%.3f, %.3f, %.3f] m\n', P_c0);
fprintf('导弹到原点时间 t_arrival = %.3f s\n', t_missile_arrival);
fprintf('最小距离发生时刻(近似) t_min = %.6f s, 对应 F(t_min) = %.6e\n', t_min_approx, d_min_approx);
fprintf('遮蔽时间区间（若有，格式为 [t_left, t_right]）:\n');
disp(intervals);
fprintf('遮蔽总时长 = %.6f s\n', total_shadow_time);

% 保存结果到 MAT 文件与文本文件
save('result_q1.mat', 'P_drop', 't_expl', 'P_c0', 't_missile_arrival', ...
    't_min_approx', 'd_min_approx', 'intervals', 'total_shadow_time');

fid = fopen('result_q1.txt','w');
fprintf(fid, 'UAV 投放点 P_drop = [%.6f, %.6f, %.6f] m\n', P_drop);
fprintf(fid, '烟幕起爆时刻 t_expl = %.6f s\n', t_expl);
fprintf(fid, '烟幕起爆点 P_c0 = [%.6f, %.6f, %.6f] m\n', P_c0);
fprintf(fid, '导弹到原点时间 t_arrival = %.6f s\n', t_missile_arrival);
fprintf(fid, '最小距离发生时刻(近似) t_min = %.6f s, 对应 F(t_min) = %.6e\n', t_min_approx, d_min_approx);
fprintf(fid, '遮蔽时间区间（若有，格式为 [t_left, t_right]）:\n');
for k = 1:size(intervals,1)
    fprintf(fid, '[%.6f, %.6f]\n', intervals(k,1), intervals(k,2));
end
fprintf(fid, '遮蔽总时长 = %.6f s\n', total_shadow_time);
fclose(fid);

%% ========== 可视化1：F(t) = d(t) - r 随时间的变化（并标注遮蔽区间） ==========
fig1 = figure('Name','Distance Minus Radius vs Time','NumberTitle','off','Color',[1 1 1]);
set(fig1, 'Position', [100,100,1000,450]);
% 细采样以获得平滑曲线
t_plot = t_search_min:1e-3:t_search_max;
F_plot = F(t_plot);

plot(t_plot, F_plot, 'LineWidth', 1.5, 'Color', [0 0.4470 0.7410]); hold on;
yline(0, '--', 'Color', [0.2 0.2 0.2], 'LineWidth', 1);
xlabel('时间 t (s)', 'FontSize', 12);
ylabel('d(t) - r (m)', 'FontSize', 12);
title('云团中心到导弹-目标连线最短距离减半径随时间的变化', 'FontSize', 14);

% 标出遮蔽区间并填充（半透明）
for k = 1:size(intervals,1)
    x1 = intervals(k,1); x2 = intervals(k,2);
    % 填充矩形
    y_lim = ylim;
    patch([x1 x2 x2 x1], [y_lim(1) y_lim(1) y_lim(2) y_lim(2)], [0.9 0.9 0.2], ...
        'FaceAlpha', 0.2, 'EdgeColor', 'none');
    % 标注区间
    text((x1+x2)/2, y_lim(2)*0.9, sprintf('遮蔽区间 %.3fs-%.3fs', x1, x2), ...
        'HorizontalAlignment', 'center', 'FontSize', 10);
end
grid on;
% 高分辨率保存
print(fig1, 'distance_vs_time.png', '-dpng', '-r300');
drawnow;

%% ========== 可视化2：三维几何（导弹轨迹、云团轨迹、球体在最小距处） ==========
fig2 = figure('Name','3D Geometry','NumberTitle','off','Color',[1 1 1]);
set(fig2, 'Position', [120,120,1000,700]);
ax = axes('Parent', fig2);
hold(ax, 'on'); grid(ax, 'on'); view(3);
xlabel('X (m)', 'FontSize', 12); ylabel('Y (m)', 'FontSize', 12); zlabel('Z (m)', 'FontSize', 12);
title('几何示意：导弹轨迹、无人机投放/起爆点与云团轨迹', 'FontSize', 14);

% 绘制导弹轨迹（从 t=0 到 t_arrival 的一部分，截取到 cloud_tend）
t_traj = 0 : 0.05 : min(t_missile_arrival, cloud_tend+5);
P_m_traj = (P_m0.' + v_missile.' .* t_traj).' ;
plot3(P_m_traj(:,1), P_m_traj(:,2), P_m_traj(:,3), 'r-', 'LineWidth', 1.5);

% 绘制 UAV 飞行路径（从 t=0 到 t_drop）
t_uav_traj = 0 : 0.1 : t_drop;
P_uav_traj = (P_FY1.' + v_uav.' .* t_uav_traj).' ;
plot3(P_uav_traj(:,1), P_uav_traj(:,2), P_uav_traj(:,3), 'b-', 'LineWidth', 1.5);

% 绘制投放点与起爆点
plot3(P_drop(1), P_drop(2), P_drop(3), 'bo', 'MarkerSize',8, 'MarkerFaceColor','b');
text(P_drop(1), P_drop(2), P_drop(3)+30, '投放点', 'FontSize', 11);

plot3(P_c0(1), P_c0(2), P_c0(3), 'mo', 'MarkerSize',8, 'MarkerFaceColor','m');
text(P_c0(1), P_c0(2), P_c0(3)+30, '起爆点', 'FontSize', 11);

% 云团中心轨迹（起爆后）
t_cloud_plot = cloud_t0 : 0.05 : cloud_tend;
C_traj = (P_c0.' + [0;0; -cloud_sink_v] .* (t_cloud_plot - cloud_t0)).';
plot3(C_traj(:,1), C_traj(:,2), C_traj(:,3), '-', 'Color', [0.3010 0.7450 0.9330], 'LineWidth', 1.5);

% 绘制真目标（用小圆柱体表示）
[XC, YC, ZC] = cylinder(7,36); % 半径7，分36面
ZC = ZC * 10; % 高度 10 m
surf(XC + 0, YC + 200, ZC, 'FaceAlpha', 0.3, 'EdgeColor', 'none', 'FaceColor', [0.8 0.8 0.8]);

% 标出云团在最小距离时的球体（若 intervals 非空则用第一个区间中点的时刻）
if ~isempty(intervals)
    t_sample = mean(intervals(1,:));
else
    t_sample = t_min_approx;
end
C_sample = P_c0 + [0,0,-cloud_sink_v*(t_sample - cloud_t0)];
% 绘制球体（云团）表面
[sx, sy, sz] = sphere(48);
surf( C_sample(1) + cloud_radius*sx, C_sample(2) + cloud_radius*sy, C_sample(3) + cloud_radius*sz, ...
    'FaceAlpha', 0.25, 'EdgeColor', 'none', 'FaceColor', [0.9 0.6 0.1]);

% 绘制导弹当前位置（在 t_sample）
Pm_sample = P_m0 + v_missile * t_sample;
plot3(Pm_sample(1), Pm_sample(2), Pm_sample(3), 'k^', 'MarkerSize',8,'MarkerFaceColor','k');
text(Pm_sample(1), Pm_sample(2), Pm_sample(3)+30, sprintf('M1 在 t=%.3fs', t_sample), 'FontSize', 11);

% 设置视图、图例
legend({'导弹轨迹','UAV 轨迹','投放点','起爆点','云团中心轨迹','真目标(圆柱)','云团球面','导弹位置'}, 'Location','northeastoutside');
axis equal;
xlim_auto = xlim; ylim_auto = ylim; zlim_auto = zlim;
% 美化并保存
view([ -30, 15 ]);
camlight headlight; lighting phong;
print(fig2, 'geometry_3d.png', '-dpng', '-r300');
drawnow;

%% ========== 显示保存的图片在 MATLAB 命令窗口（可视化确认） ==========
% 读取刚保存的图片并显示，便于脚本运行后在命令窗口直接查看
img1 = imread('distance_vs_time.png');
figure('Name','Distance vs Time (Saved Image)','NumberTitle','off');
imshow(img1); title('d(t)-r 随时间的变化（已保存）', 'FontSize', 12);

img2 = imread('geometry_3d.png');
figure('Name','3D Geometry (Saved Image)','NumberTitle','off');
imshow(img2); title('几何示意图（已保存）', 'FontSize', 12);

%% ========== 本脚本结束提示 ==========
fprintf('结果与图片已保存到当前目录。\n');
fprintf('MAT 文件: result_q1.mat\n');
fprintf('文本文件: result_q1.txt\n');
fprintf('图片: distance_vs_time.png, geometry_3d.png\n');

%% ========== 本脚本所用的本地函数定义（放在脚本末尾） ==========

function s_clamped = clamp_projection(C, Pm, Pt)
% clamp_projection: 计算点 C 在向量 (Pt-Pm) 上的投影系数 s，并截断到 [0,1]
% 输入：C, Pm, Pt 为 1x3 向量
% 输出：s_clamped 为标量投影系数（已截断）
    v = (Pt - Pm);
    denom = sum(v.^2);
    if denom == 0
        s_clamped = 0;
        return;
    end
    s = dot((C - Pm), v) / denom;
    s_clamped = max(0, min(1, s));
end

function intervals_out = merge_intervals(intervals_in, tol)
% merge_intervals: 合并重叠或接近的区间
% intervals_in: n x 2 数组，每行 [a b], 可能无序或有重叠
% tol: 合并时的时间容差
    if isempty(intervals_in)
        intervals_out = [];
        return;
    end
    % 规范化：确保每行 [a b] 且 a <= b
    A = intervals_in;
    for i = 1:size(A,1)
        if A(i,1) > A(i,2)
            A(i,:) = A(i,[2 1]);
        end
    end
    % 排序
    A = sortrows(A,1);
    merged = A(1,:);
    for i = 2:size(A,1)
        cur = A(i,:);
        last = merged(end,:);
        if cur(1) <= last(2) + tol
            merged(end,2) = max(last(2), cur(2));
        else
            merged = [merged; cur];
        end
    end
    intervals_out = merged;
end

function [t_min, Fmin] = find_min_distance(Fhandle, ta, tb)
% find_min_distance: 在区间 [ta,tb] 上用 fminbnd 找到 F(t) 的最小值点（针对 F(t) = d-r）
% 返回值 t_min 为最小点，Fmin 为对应的函数值
    % 为稳定性，先用粗采样找最小点附近，再局部优化
    t_coarse = linspace(ta, tb, 201);
    Fvals = Fhandle(t_coarse);
    [~, idx] = min(Fvals);
    t0 = t_coarse(idx);
    % 使用 fminbnd 在 [max(ta,t0-1), min(tb,t0+1)] 局部搜索
    left = max(ta, t0 - 2);
    right = min(tb, t0 + 2);
    try
        t_min = fminbnd(@(tt) Fhandle(tt), left, right);
    catch
        t_min = t0;
    end
    Fmin = Fhandle(t_min);
end