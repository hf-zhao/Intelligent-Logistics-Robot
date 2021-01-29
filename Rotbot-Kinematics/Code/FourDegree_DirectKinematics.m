function position = FourDegree_DirectKinematics(theta1, theta2, theta3, theta4)
    % FourDegree_DirectKinematics - 运动学正解
    %
    % 输入单位制为角度制
    % Syntax: aws = solve(theta1, theta2, theta3, theta4) 
    % theta1 - 底座关节角
    % theta2 - 臂1关节角
    % theta3 - 臂2关节角
    % theta4 - 腕关节角
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
    a1 = 10.5;
    a2 = 104;
    a3 = 88.52;

    % 连杆扭转角
    alpha0 = 0;
    alpha1 = 90;
    alpha2 = 0;
    alpha3 = 0;

    % D-H参数
    % i     alpha(i-1)     a(i-1)   d(i)    theta(i)
    % 1     0              0        0       theta1
    % 2     pi/2           10.5     0       90 - theta2
    % 3     0              104      0       -theta3
    % 4     0              88.52    0       -theta4

    
    % 计算齐次变换矩阵(机械臂姿态)
    T01 = modify_transfer(alpha0, a0, d1, -theta1)
    T12 = modify_transfer(alpha1, a1, d2, 90 - theta2)
    T23 = modify_transfer(alpha2, a2, d3, -theta3)
    T34 = modify_transfer(alpha3, a3, d4, -theta4)
    T04 = T01 * T12 * T23 * T34
    TBW = T04; % 机械臂腕关节相对于基座的姿态矩阵
    % 计算工具相对基座姿态
    TWT = modify_transfer(-90, 138, -35, 0) % 工具坐标系相对腕关节
    TBT = TBW * TWT % 工具坐标系相对基座

    % 结果输出 
    TBW; % 机械臂当前姿态矩阵
    loc = TBT(1:3, 4); % 工具末端当前空间坐标

    % 画图
    a = [a0 a1 a2 a3 138];
    T = {T01, T12, T23, T34, TBT};
    draw(a, T);
end