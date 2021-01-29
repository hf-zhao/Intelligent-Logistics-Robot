function angle = FiveDegree_InverseKinematics(px, py, pz)
%myFun - Description
%
% Syntax: angle = myFun(input)
%
% Long description
    % 定义工具
    syms nx ny nz ox oy oz ax ay az
    % 定义关节角
    syms theta1 theta2 theta3 theta4 theta5
    % 定义关节扭转角变量
    syms alpha0 alpha1 alpha2 alpha3
    % 定义连杆偏距
    syms d1 d2 d3 d4 d5
    % 定义连杆长度
    syms a0 a1 a2 a3 a4
    
    % 机械臂固有参数
    % 连杆偏距
    d1 = 0;
    d2 = 0;
    d3 = 0;
    d4 = 0; 
    % d5 = 65.1509;
    % 连杆长度
    % a0 = 0;
    % a1 = 10;
    % a2 = 104;
    % a3 = 88.52;
    % a4 = 0;
    % 连杆扭转角
    alpha0 = 0;
    alpha1 = 90;
    alpha2 = 0;
    alpha3 = 0;
    alpha4 = 90;
    % D-H参数
    % i     alpha(i-1)     a(i-1)   d(i)    theta(i)
    % 1     0              0        0       theta1
    % 2     -pi/2          10       0       theta2
    % 3     0              104      0       theta3
    % 4     0              88.52    0       theta4
    % 5     pi/2           0        65.1509 theta5

    % 计算齐次变换矩阵
    T01 = modify_transfer(alpha0, a0, d1, theta1)
    T12 = modify_transfer(alpha1, a1, d2, theta2)
    T23 = modify_transfer(alpha2, a2, d3, theta3)
    T34 = modify_transfer(alpha3, a3, d4, theta4)
    T45 = modify_transfer(alpha4, a4, d5, theta5)
    T05 = T01 * T12 * T23 * T34 * T45

    nx = px / sqrt(px^2 + py^2);
    ny = py / sqrt(px^2 + py^2);
    nz = 0;
    ox = -py / sqrt(px^2 + py^2);
    oy = px / sqrt(px^2 + py^2);
    oz = 0;
    ax = 0;
    ay = 0;
    az = 1;
    TT = [nx ox ax px;
          ny oy ay py;
          nz oz az pz;
          0  0  0  1];

    T1 = inv(T01) * TT;
    T2 = T12 * T23 * T34 * T45;
    L = simplify(T1)
    R = simplify(T2)

end