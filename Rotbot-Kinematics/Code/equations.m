% 定义关节转角变量
syms theta1 theta2 theta3
% 定义关节扭转角变量
syms alpha0 alpha1 alpha2 alpha3
% 定义连杆偏距
syms d1 d2 d3 d4
% 定义连杆长度
syms a0 a1 a2 a3
% 定义工具
syms nx ny nz ox oy oz ax ay az px py pz

eqn = [-sin(theta1) * px + cos(theta1) * py ==10];
theta1 = solve(eqn, theta1)