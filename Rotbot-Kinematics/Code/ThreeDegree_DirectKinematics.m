function position = ThreeDegree_DirectKinematics(theta1, theta2, theta3)
    %solve - 运动学正解
    %
    % s输入单位制为角度制
    % Syntax: aws = solve(theta1, theta2, theta3, theta4, theta5) 
    % theta1 - 底座关节角
    % theta2 - 臂1关节角
    % theta3 - 臂2关节角
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
    a3 = 143;
    % 连杆扭转角
    alpha0 = 0;
    alpha1 = 90;
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
    T01 = modify_transfer(alpha0, a0, d1, theta1);
    T12 = modify_transfer(alpha1, a1, d2, theta2);
    T23 = modify_transfer(alpha2, a2, d3, theta3);
    T34 = modify_transfer(alpha3, a3, d4, theta4);
    T04 = T01 * T12 * T23 * T34;

    % 结果输出
    TT = T04

    % 画图
    a = [a0 a1 a2 a3];
    T = {T01, T12, T23, TT};
    draw(a, T);
end