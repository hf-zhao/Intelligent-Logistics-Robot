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

px = 300;
py = 0;
pz = 0;

nx = px / sqrt(px^2 + py^2);
ny = py /sqrt(px^2 + py^2);
nz = 0;
ox = 0;
oy = 0;
oz = 1;
ax = -py /sqrt(px^2 + py^2);
ay = px / sqrt(px^2 + py^2);
az = 0;

theta4 = -(theta2 + theta3);
T01 = matrix(alpha0, a0, d1, theta1);
T12 = matrix(alpha1, a1, d2, theta2);
T23 = matrix(alpha2, a2, d3, theta3);
T34 = matrix(alpha3, a3, d4, theta4);
T04 = T01 * T12 * T23 * T34;
simplify(T04)

TT = [nx ox ax px;
      ny oy ay py;
      nz oz az pz;
      0  0  0  1];

T1 = inv(T01) * TT;
T2 = T12 * T23 * T34;
L = simplify(T1)
R = simplify(T2)

% theta1求解(2,4)
theta1 = atan2d(ny, nx);

% eqn = [L(1, 4) == R(1, 4), L(3,4) == R(3, 4)];
% s = solve(eqn, [theta2, theta3])

% theta2、theta3求解(1,4)(3,4)
t1 = px * cosd(theta1) - a0 * cosd(theta1) + py * sind(theta1) - a1;
t2 = pz;
theta3_1 = acosd((t1^2 + t2^2 - a2^2 - a3^2) / (2 * a2 * a3));
theta3_2 = -theta3_1;

r1_1 = a3 * cosd(theta3_1) + a2;
r2_1 = a3 * sind(theta3_2);
theta2_1 = atan2d(t2, sqrt(r1_1^2 + r2_1^2)) - atan2d(r2_1, r1_1);
r1_2 = a3 * cosd(theta3_2) + a2;
r2_2 = a3 * sind(theta3_2);
theta2_2 = atan2d(t2, sqrt(r1_2^2 + r2_2^2)) - atan2d(r2_2, r1_2);

% K1 = (pz^2 + (px * cosd(theta1) - a0 * cosd(theta1) + py * sind(theta1) - a1)^2 - a3^2 - a2^2) / (2 * a2 * a3);
% % theta3 = acosd(K1)
% theta3_1 = atan2d(sqrt(1- K1^2), K1);
% theta3_2 = atan2d(-sqrt(1 - K1^2), K1);
% theta2_1 = atan2d(pz, sqrt(((a3 * cosd(theta3_1) + a2)^2 + (a3 * sind(theta3_1))^2)^2 - pz^2)) - atan2d(a3 * sind(theta3_1), a2 + a3 * cosd(theta3_1));
% theta2_2 = atan2d(pz, -sqrt(((a3 * cosd(theta3_1) + a2)^2 + (a3 * sind(theta3_1))^2)^2 - pz^2)) - atan2d(a3 * sind(theta3_1), a2 + a3 * cosd(theta3_1));
% theta2_3 = atan2d(pz, sqrt(((a3 * cosd(theta3_2) + a2)^2 + (a3 * sind(theta3_2))^2)^2 - pz^2)) - atan2d(a3 * sind(theta3_2), a2 + a3 * cosd(theta3_2));
% theta2_4 = atan2d(pz, -sqrt(((a3 * cosd(theta3_2) + a2)^2 + (a3 * sind(theta3_2))^2)^2 - pz^2)) - atan2d(a3 * sind(theta3_2), a2 + a3 * cosd(theta3_2));

ans = [theta1 theta2_1 theta3_1;
       theta1 theta2_2 theta3_2]
% ThreeDegree_DirectKinematics(ans(1,1), ans(1,2),ans(1,3));
ThreeDegree_DirectKinematics(ans(2,1), ans(2,2),ans(2,3));
