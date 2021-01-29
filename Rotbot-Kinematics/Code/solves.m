function aws = solves(theta1, theta2, theta3)
    %solve - 运动学正解
    %
    % Syntax: aws = solve(theta1, theta2, theta3)
    % theta1 - 底座关节角
    % theta2 - 大臂关节角
    % theta3 - 小臂关节角
    %
    % 输入为三个关节角
    % 输出为空间坐标
    % 舵机中位0°，左右范围-90°~90°


    % clear;
    % clc;

    % 机械臂固有参数
    % 连杆偏距
    d1 = 0;
    d2 = 0;
    d3 = 0;
    d4 = 0; 
    % 连杆长度
    a0 = 0;
    a1 = 14.8366;
    a2 = 150;
    a3 = 143
    % 连杆扭转角
    alpha0 = 0;
    alpha1 = pi/2;
    alpha2 = 0;
    alpha3 = 0;
    % D-H参数
    % i     alpha(i-1)     a(i-1)   d(i)    theta(i)
    % 1     0              0        0       theta1
    % 2     pi/2           14.8366  0       theta2
    % 3     0              150      0       theta3
    % 4     0              143      0       theta4(= -theta2 - theta3)

    theta4 = -(theta2 + theta3);

    % 计算齐次变换矩阵
    T01 = matrix(alpha0, a0, d1, theta1)
    T12 = matrix(alpha1, a1, d2, theta2)
    T23 = matrix(alpha2, a2, d3, theta3)
    T34 = matrix(alpha3, a3, d4, theta4)
    T03 = T01 * T12 * T23;
    T04 = T01 * T12 * T23 * T34;

    % 静态欧拉角X-Y-Z
    beta = 180 / pi * atan2(-T04(3,1), sqrt(T04(1, 1) * T04(1, 1) + T04(2, 1) * T04(2, 1)));
    alpha = 180 / pi * atan2(T04(2, 1) / cos(beta), T04(1, 1) / cos(beta));
    gama = 180 / pi * atan2(T04(3, 2) / cos(beta), T04(3, 3) / cos(beta));

    if beta == 90
        alpha = 0;
        gama = 180 / pi * atan2(T04(1, 2), T04(2, 2));
    end
    if beta == -90
        alpha = 0;
        gama = -180 / pi * atan2(T04(1, 2), T04(2, 2));
    end

    % 结果输出
    aws = [(T04(1:3, 4))' gama beta alpha] 
    disp("坐标为：")
    loc = T04(1:3, 4)
    disp("欧拉角为：")
    ori = [gama beta alpha]
    disp("该点相对于基座世界坐标系下的齐次变换矩阵(恺恺需)：")
    R = [cos(theta1) sin(theta1) 0;
         -sin(theta1) cos(theta1) 0;
         0 0 1
        ];
    TT = T04;
    % TT(1:3, 1:3) = R;
    TT

    % % test
    % a = [a0 a1 a2 a3];
    % T = {T01, T12, T23, T34};
    % draw(a, T);



    % 画图
    link1 = [a1; 0; 0]; % 杆1在坐标系1下的表达
    link2 = [a2; 0; 0]; % 杆2在坐标系2下的表达
    link3 = [a3; 0; 0]; % 杆3在坐标系3下的表达
    d_1 = T01(1:3, 1:3) * link1; % 杆1在世界坐标系下的表达
    d_2 = d_1 + T01(1:3, 1:3) * T12(1:3, 1:3) * link2; % 杆2在世界坐标系下的表达
    d_3 = d_2 + T01(1:3, 1:3) * T12(1:3, 1:3) * T23(1:3, 1:3) * link3; % 杆3在世界坐标系下的表达

    hold on;
    grid on;
    view(45, 30);
    %axis([-200 200], [-200 200], [-200 200]);
    axis equal;
    set(get(gca, 'XLabel'), 'String', 'x轴');
    set(get(gca, 'YLabel'), 'String', 'y轴');
    set(get(gca, 'ZLabel'), 'String', 'z轴');
    % axis([-100 100], [-100 100], [-200, 200]);
    plot3(0, 0, 0, 'o');
    plot3(d_1(1), d_1(2), d_1(3), 'o');
    plot3(d_2(1), d_2(2), d_2(3), 'o');
    plot3(d_3(1), d_3(2), d_3(3), 'o');
    % 标出该点空间坐标
    cod = ['(' num2str(d_3(1)) ',' num2str(d_3(2)) ',' num2str(d_3(3)) ')'];
    text(d_3(1), d_3(2), d_3(3), cod);
    % 标出该点空间坐标系
    plot3([d_3(1) (d_3(1) + 100 * TT(1, 1))], [d_3(2) (d_3(2) + 100 * TT(2, 1))], [d_3(3) (d_3(3) + 100 * TT(3, 1))], 'LineStyle', '-.', 'LineWidth', 1, 'color', 'r'); % 红色点划线表示x轴方向
    plot3([d_3(1) (d_3(1) + 100 * TT(1, 2))], [d_3(2) (d_3(2) + 100 * TT(2, 2))], [d_3(3) (d_3(3) + 100 * TT(3, 2))], 'LineStyle', '-.', 'LineWidth', 1, 'color', 'g'); % 绿色点划线表示y轴方向
    plot3([d_3(1) (d_3(1) + 100 * TT(1, 3))], [d_3(2) (d_3(2) + 100 * TT(2, 3))], [d_3(3) (d_3(3) + 100 * TT(3, 3))], 'LineStyle', '-.', 'LineWidth', 1, 'color', 'b'); % 蓝色点划线表示z轴方向

    plot3([0 d_1(1)], [0 d_1(2)], [0 d_1(3)], 'LineWidth', 3, 'color', 'r'); % 底座连杆1用红色表示
    plot3([d_1(1) d_2(1)], [d_1(2) d_2(2)], [d_1(3) d_2(3)], 'LineWidth', 3, 'color', 'b'); % （大臂）连杆2用蓝色表示
    plot3([d_2(1) d_3(1)], [d_2(2) d_3(2)], [d_2(3) d_3(3)], 'LineWidth', 3, 'color', 'g'); % （小臂）连杆3用绿色表示

end
