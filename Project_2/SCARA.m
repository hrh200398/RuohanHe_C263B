clear; 
close all; 
clc;

%% Parameters
deg = pi/180;

L1 = 0.325;
L2 = 0.225;
d1 = 0.416;
d4 = -0.080;

d3_min = -0.15;
d3_max =  0.00;

q_phys = [0, 90*deg, 0, 0];

scara = buildSCARA_RTB(L1, L2, d1, d4, d3_min, d3_max);

q_rtb = q_phys;
q_rtb(3) = q_phys(3) - d3_min;

q_rtb(3) = min(max(q_rtb(3), scara.links(3).qlim(1)), scara.links(3).qlim(2));

%% Plot model
figure('Name','SCARA RRPR Model');
scara.plot(q_rtb, 'workspace', [-0.8 0.8 -0.8 0.8 0 0.9]);
title('SCARA RRPR model');

%% Function
function scara = buildSCARA_RTB(L1, L2, d1, d4, d3_min, d3_max)

deg = pi/180;

L(1) = Link([0 d1 L1 0 0 0]);     L(1).qlim = [-170 170]*deg;
L(2) = Link([0 0  L2 0 0 0]);     L(2).qlim = [-170 170]*deg;

L(3) = Link([0 0  0  0 1 0]);
L(3).qlim = [0, max(1e-6, d3_max - d3_min)];

L(4) = Link([0 0  0  0 0 0]);     L(4).qlim = [-180 180]*deg;

scara = SerialLink(L, 'name', 'SCARA');

scara.tool = [1 0 0 0;
             0 1 0 0;
             0 0 1 d4;
             0 0 0 1];

scara.links(3).offset = d3_min;
end
