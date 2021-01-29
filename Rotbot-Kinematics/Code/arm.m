clear;
clc;
%创建连杆
% link(1) = Link(['theta', 0, 'd', 0, 'a', 0, 'alpha',0]);
% link(2) = Link(['theta', 0, 'd', 0, 'a', 14.8366, 'alpha', -pi/2]);
% link(3) = Link(['theta', 0, 'd', 0, 'a', 150, 'alpha',0]);
% three_links = SerialLink([link(1), link(2), link(3)]);
% 
% L1 = Link('d', 0, 'a', 0, 'alpha', -pi/2);%定义连杆
% L2 = Link('d', 1, 'a', 0, 'alpha', pi/2);
% L3 = Link('theta', 0, 'a', 0, 'alpha', 0);
% L4 = Link('d', 0, 'a', 0, 'alpha', -pi/2);
% L5 = Link('d', 0, 'a', 0, 'alpha', pi/2);
% L6 = Link('d', 1, 'a', 0, 'alpha', 0);
% bot = SerialLink([L1 L2 L3 L4 L5 L6]);%连接连杆
% bot.display();%显示D-H参数表
% bot

% L1 = Link([0 0 0 0]);
% L2 = Link([pi/2 0 14.8366 -pi/2]);
% L3 = Link([pi/2 0 150 0]);
% three_links = SerialLink([L1, L2, L3]);

L1 = Link([0 0 0 0]);
L2 = Link([pi/2 0 100 0]);
L3 = Link([pi/2 0 60 0]);
three_links = SerialLink([L1, L2, L3]);
three_links.teach
