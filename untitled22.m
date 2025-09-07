% 清除工作区和图形窗口
clf; clear; clc;

% 设置图形参数
figure('Position', [100, 100, 900, 700]);
hold on;
grid on;
axis equal;

% 定义坐标点
% 假目标原点
origin = [0, 0, 0];

% 真目标下底面圆心
target = [0, 200, 0];

% 导弹位置
M1 = [20000, 0, 2000];
M2 = [19000, 600, 2100];
M3 = [18000, -600, 1900];

% 无人机位置
FY1 = [17800, 0, 1800];
FY2 = [12000, 1400, 1400];
FY3 = [6000, -3000, 700];
FY4 = [11000, 2000, 1800];
FY5 = [13000, -2000, 1300];

% 绘制假目标原点
plot3(origin(1), origin(2), origin(3), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');

% 绘制真目标圆心
plot3(target(1), target(2), target(3), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');

% 绘制导弹位置
plot3(M1(1), M1(2), M1(3), '^', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'Color', 'r');
text(M1(1)+500, M1(2), M1(3), 'M1', 'FontSize', 10, 'Color', 'r');

plot3(M2(1), M2(2), M2(3), '^', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'Color', 'r');
text(M2(1)+500, M2(2), M2(3), 'M2', 'FontSize', 10, 'Color', 'r');

plot3(M3(1), M3(2), M3(3), '^', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'Color', 'r');
text(M3(1)+500, M3(2), M3(3), 'M3', 'FontSize', 10, 'Color', 'r');

% 绘制无人机位置
plot3(FY1(1), FY1(2), FY1(3), 's', 'MarkerSize', 10, 'MarkerFaceColor', 'b', 'Color', 'b');
text(FY1(1)+500, FY1(2), FY1(3), 'FY1', 'FontSize', 10, 'Color', 'b');

plot3(FY2(1), FY2(2), FY2(3), 's', 'MarkerSize', 10, 'MarkerFaceColor', 'b', 'Color', 'b');
text(FY2(1)+500, FY2(2), FY2(3), 'FY2', 'FontSize', 10, 'Color', 'b');

plot3(FY3(1), FY3(2), FY3(3), 's', 'MarkerSize', 10, 'MarkerFaceColor', 'b', 'Color', 'b');
text(FY3(1)+500, FY3(2), FY3(3), 'FY3', 'FontSize', 10, 'Color', 'b');

plot3(FY4(1), FY4(2), FY4(3), 's', 'MarkerSize', 10, 'MarkerFaceColor', 'b', 'Color', 'b');
text(FY4(1)+500, FY4(2), FY4(3), 'FY4', 'FontSize', 10, 'Color', 'b');

plot3(FY5(1), FY5(2), FY5(3), 's', 'MarkerSize', 10, 'MarkerFaceColor', 'b', 'Color', 'b');
text(FY5(1)+500, FY5(2), FY5(3), 'FY5', 'FontSize', 10, 'Color', 'b');

% 设置坐标轴标签和标题
xlabel('X轴 (米)');
ylabel('Y轴 (米)');
zlabel('Z轴 (米)');
title('导弹与无人机三维空间分布');

% 添加图例
legend('假目标原点', '真目标圆心', '导弹', '无人机', 'Location', 'best');

% 设置视角
view(30, 30);

% 添加网格
grid on;

% 调整坐标轴范围以更好地显示所有点
all_points = [origin; target; M1; M2; M3; FY1; FY2; FY3; FY4; FY5];
padding = 1000; % 添加一些边距
xlim([min(all_points(:,1))-padding, max(all_points(:,1))+padding]);
ylim([min(all_points(:,2))-padding, max(all_points(:,2))+padding]);
zlim([min(all_points(:,3))-padding, max(all_points(:,3))+padding]);

hold off;

% 保存图形为PNG文件
saveas(gcf, 'drone_missile_visualization.png');