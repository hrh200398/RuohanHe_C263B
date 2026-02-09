clear; 
close all; 
clc;

%% Acceleration limits
deg = pi/180;
a_th  = 50*deg; % rad/s^2
a_lin = 2.0; % m/s^2

%% Robot geometry (SCARA)
L1 = 0.325;  % all in meter
L2 = 0.225;
d1 = 0.416;
d4 = -0.080;

%% Via point heights
zT = 0.300; % travel
zP = 0.280; % pick/place

%% Build all via poses (task space)
Rz = @(psi)[cos(psi) -sin(psi) 0;
            sin(psi)  cos(psi) 0;
            0         0        1];
Tform = @(R,p)[R p(:); 0 0 0 1];

xP = 0.250; % input
yP = 0.225; % input
P  = makeStationPoses(xP, yP, zT, zP, deg);

TFt = P.Ft; TFp = P.Fp;
TC1t = P.C1t; TC1p = P.C1p;
TC2t = P.C2t; TC2p = P.C2p;
TC3t = P.C3t; TC3p = P.C3p;
TC4t = P.C4t; TC4p = P.C4p;

q4_init = 0;
q4_C1 = q4_init + 90*deg;
q4_C2 = q4_C1; 
q4_C3 = q4_init;
q4_C4 = q4_init - 180*deg;

%% Reference pose from reference joint configuration
% ref uses q2=90deg, d3=0
qref = [0, 90*deg, 0, 0];  % [q1 q2 d3 q4]
Tref = scaraFK(qref, L1, L2, d1, d4);

%% Trajectory waypoint sequences (Traj1 to Traj9)
traj = { ...
    {Tref, TFt, TFp}, ...
    {TFp,  TFt, TC1t, TC1p}, ...
    {TC1p, TC1t, TFt,  TFp}, ...
    {TFp,  TFt, TC2t, TC2p}, ...
    {TC2p, TC2t, TFt,  TFp}, ...
    {TFp,  TFt, TC3t, TC3p}, ...
    {TC3p, TC3t, TFt,  TFp}, ...
    {TFp,  TFt, TC4t, TC4p}, ...
    {TC4p, TC4t, TFt,  Tref} ...
};

%% Print for report
T13 = struct();
T13.T_F_pick = TFp;
T13.T_C1 = TC1p;
T13.T_C2 = TC2p;
T13.T_C3 = TC3p;
T13.T_C4 = TC4p;

T14 = struct();
T14.T_ref = Tref;

T14.T_Ft  = TFt;   T14.T_Fp  = TFp;
T14.T_C1t = TC1t;  T14.T_C1p = TC1p;
T14.T_C2t = TC2t;  T14.T_C2p = TC2p;
T14.T_C3t = TC3t;  T14.T_C3p = TC3p;
T14.T_C4t = TC4t;  T14.T_C4p = TC4p;

disp('1.3 (Feeder and 4 chips)');
printT('T_F_pick (Feeder picking pose, w.r.t. {S})', T13.T_F_pick);
printT('T_C1 (PCB target 1 pose, w.r.t. {S})',      T13.T_C1);
printT('T_C2 (PCB target 2 pose, w.r.t. {S})',      T13.T_C2);
printT('T_C3 (PCB target 3 pose, w.r.t. {S})',      T13.T_C3);
printT('T_C4 (PCB target 4 pose, w.r.t. {S})',      T13.T_C4);

disp('1.4 (ALL via points)');
names14 = fieldnames(T14);
for i = 1:numel(names14)
    nm = names14{i};
    printT([nm ' (w.r.t. {S})'], T14.(nm));
end

outFile = 'Report_Transforms_SCARA.txt';
fid = fopen(outFile,'w');

fprintf(fid, "All transforms are homogeneous matrices ^S T (w.r.t. {S}=base0).\n\n");

fprintf(fid, "Requirement 1.3: 5 points");
writeT(fid, "T_F_pick (Feeder picking pose)", T13.T_F_pick);
writeT(fid, "T_C1 (PCB target 1 pose)",       T13.T_C1);
writeT(fid, "T_C2 (PCB target 2 pose)",       T13.T_C2);
writeT(fid, "T_C3 (PCB target 3 pose)",       T13.T_C3);
writeT(fid, "T_C4 (PCB target 4 pose)",       T13.T_C4);

fprintf(fid, "Requirement 1.4: ALL via points");
for i = 1:numel(names14)
    nm = names14{i};
    writeT(fid, nm, T14.(nm));
end

fclose(fid);
fprintf('Saved report matrices to: %s\n', outFile);

%% Functions for printing
function printT(name, T)
    fprintf('%s =\n', name);
    fprintf('[% .6f % .6f % .6f % .6f;\n', T(1,1),T(1,2),T(1,3),T(1,4));
    fprintf(' % .6f % .6f % .6f % .6f;\n', T(2,1),T(2,2),T(2,3),T(2,4));
    fprintf(' % .6f % .6f % .6f % .6f;\n', T(3,1),T(3,2),T(3,3),T(3,4));
    fprintf(' % .6f % .6f % .6f % .6f]\n\n', T(4,1),T(4,2),T(4,3),T(4,4));
end

function writeT(fid, name, T)
    fprintf(fid, "%s =\n", name);
    fprintf(fid, "[% .6f % .6f % .6f % .6f;\n", T(1,1),T(1,2),T(1,3),T(1,4));
    fprintf(fid, " % .6f % .6f % .6f % .6f;\n", T(2,1),T(2,2),T(2,3),T(2,4));
    fprintf(fid, " % .6f % .6f % .6f % .6f;\n", T(3,1),T(3,2),T(3,3),T(3,4));
    fprintf(fid, " % .6f % .6f % .6f % .6f]\n\n", T(4,1),T(4,2),T(4,3),T(4,4));
end

%% Flatten all via poses into ONE waypoint list
Tall = {};
Lall = {};

% start with reference
Tall{1} = traj{1}{1};
Lall{1} = 'REF';

for k = 1:numel(traj)
    W = traj{k};

    for i = 2:numel(W)
        Tall{end+1} = W{i};

        
        if isequal(W{i}, TFt) || isequal(W{i}, TFp)
            Lall{end+1} = 'F';
        elseif isequal(W{i}, TC1t) || isequal(W{i}, TC1p)
            Lall{end+1} = 'C1';
        elseif isequal(W{i}, TC2t) || isequal(W{i}, TC2p)
            Lall{end+1} = 'C2';
        elseif isequal(W{i}, TC3t) || isequal(W{i}, TC3p)
            Lall{end+1} = 'C3';
        elseif isequal(W{i}, TC4t) || isequal(W{i}, TC4p)
            Lall{end+1} = 'C4';
        else
            Lall{end+1} = 'UNK';
        end
    end
end

N = numel(Tall);
fprintf('Total via poses (including reference returns) = %d\n', N);

%% Convert task-space via poses to joint-space waypoints Q (N x 4)

Q = zeros(N,4);

q4_hold = q4_init;   % current tool angle (held during return-to-feeder segments)

for i = 1:N
    qIK = scaraIK(Tall{i}, L1, L2, d1, d4, -1);
    lab = Lall{i};

    if strcmp(lab,'C1')
        q4_hold = q4_C1;
    elseif strcmp(lab,'C2')
        q4_hold = q4_C2;
    elseif strcmp(lab,'C3')
        q4_hold = q4_C3;
    elseif strcmp(lab,'C4')
        q4_hold = q4_C4;
    elseif strcmp(lab,'REF')
        q4_hold = q4_init;
    else
       
    end

    Q(i,:) = [qIK(1) qIK(2) qIK(3) q4_hold];
end

Q(:,1) = unwrap(Q(:,1));
Q(:,2) = unwrap(Q(:,2));
Q(:,4) = unwrap(Q(:,4));

%% Compute near-minimum feasible time stamps T
sf = 1.15;
Td = zeros(N-1,1);

for i = 1:N-1
    dq = abs(Q(i+1,:) - Q(i,:));
    t1 = 2*sqrt(dq(1)/a_th);
    t2 = 2*sqrt(dq(2)/a_th);
    t4 = 2*sqrt(dq(4)/a_th);
    t3 = 2*sqrt(dq(3)/a_lin);
    Td(i) = sf * max([t1 t2 t3 t4]);
end

T = [0; cumsum(Td)];
fprintf('Total execution time (near-min, acc-limited) = %.4f s\n', T(end));

%% CRITICAL FIX: Provide ROW vectors to the given function
Trow = T(:).'; 
P1   = Q(:,1).'; 
P2   = Q(:,2).';
P3   = Q(:,3).';
P4   = Q(:,4).';

Acc_q1 = a_th  * ones(1,N);
Acc_q2 = a_th  * ones(1,N);
Acc_d3 = a_lin * ones(1,N);
Acc_q4 = a_th  * ones(1,N);

%% Generate 4 joint trajectories using the provided function
[f1, ind1] = buildPiecewiseTwoPointLIPB(P1, Trow, Acc_q1);
[f2, ind2] = buildPiecewiseTwoPointLIPB(P2, Trow, Acc_q2);
[f3, ind3] = buildPiecewiseTwoPointLIPB(P3, Trow, Acc_d3);
[f4, ind4] = buildPiecewiseTwoPointLIPB(P4, Trow, Acc_q4);

if ~(ind1 && ind2 && ind3 && ind4)
    warning('Piecewise 2-point LIPB: some segments infeasible under current Td/Acc. Increase sf or enforce Td_min.');
else
    fprintf('All 4 joint trajectories generated successfully (piecewise 2-point LIPB).\n');
end

if ~(ind1 && ind2 && ind3 && ind4)
    warning('Trajectory generation failed (ind=0). Increase sf (e.g., 1.25~1.35) and rerun.');
else
    fprintf('All 4 joint trajectories generated successfully.\n');
end

%% RTB animation
dt_sample = 0.02;
ts = unique([0:dt_sample:Trow(end), Trow]);
ts = ts(:);
q_samp = zeros(numel(ts), 4);

snap = 1e-10;
for k = 1:numel(ts)
    [dmin, j] = min(abs(ts(k) - Trow(:)));
    if dmin < snap
        ts(k) = Trow(j);
    end
end

y1 = f1(ts(1)); y2 = f2(ts(1)); y3 = f3(ts(1)); y4 = f4(ts(1));
q_last = [y1(1) y2(1) y3(1) y4(1)];
if any(~isfinite(q_last))
    q_last = Q(1,:);
end
q_samp(1,:) = q_last;

for k = 2:numel(ts)
    y1 = f1(ts(k)); y2 = f2(ts(k)); y3 = f3(ts(k)); y4 = f4(ts(k));
    q_now = [y1(1) y2(1) y3(1) y4(1)];

    bad = ~isfinite(q_now);
    if any(bad)
        q_now(bad) = q_last(bad); 
    end

    q_samp(k,:) = q_now;
    q_last = q_now;
end

fprintf('Runtime from sampled trajectory = %.4f s\n', ts(end));

%% Prepare animation trajectory
q_anim = real(q_samp);

d3_vals = [[zT zP] - d1 - d4, 0];
d3_min = min(d3_vals);
d3_max = max(d3_vals);

q_anim(:,3) = q_anim(:,3) - d3_min;
q_anim(:,3) = min(max(q_anim(:,3), 0), d3_max - d3_min);

badFrames = any(~isfinite(q_anim), 2);
if any(badFrames)
    fprintf('Removed %d bad frames before animation.\n', nnz(badFrames));
    q_anim(badFrames,:) = [];
end

%% RTB animation
scara = buildSCARA_RTB(L1, L2, d1, d4, zT, zP);

figure;
scara.plot(q_anim, ...
    'workspace', [-1 1 -1 1 0 1], ...
    'delay', dt_sample);

figure; 
subplot(4,1,1); plot(ts, q_samp(:,1)); grid on; ylabel('q1 (rad)');
subplot(4,1,2); plot(ts, q_samp(:,2)); grid on; ylabel('q2 (rad)');
subplot(4,1,3); plot(ts, q_samp(:,3)); grid on; ylabel('d3 (m)');
subplot(4,1,4); plot(ts, q_samp(:,4)); grid on; ylabel('q4 (rad)'); xlabel('t (s)');

%% Figure: Joint velocities vs time
vel_samp = zeros(numel(ts), 4);

for k = 1:numel(ts)
    y1 = f1(ts(k)); y2 = f2(ts(k)); y3 = f3(ts(k)); y4 = f4(ts(k));
    vel_samp(k,1) = y1(2);
    vel_samp(k,2) = y2(2);
    vel_samp(k,3) = y3(2);
    vel_samp(k,4) = y4(2);
end

bad = ~isfinite(vel_samp);
vel_samp(bad) = 0;

figure;
subplot(4,1,1); plot(ts, vel_samp(:,1)); grid on; ylabel('\omega_1 (rad/s)');
subplot(4,1,2); plot(ts, vel_samp(:,2)); grid on; ylabel('\omega_2 (rad/s)');
subplot(4,1,3); plot(ts, vel_samp(:,3)); grid on; ylabel('v_3 (m/s)');
subplot(4,1,4); plot(ts, vel_samp(:,4)); grid on; ylabel('\omega_4 (rad/s)'); xlabel('t (s)');

%% Figure: Joint accelerations vs time
acc_samp = zeros(numel(ts), 4);

for k = 1:numel(ts)
    y1 = f1(ts(k)); y2 = f2(ts(k)); y3 = f3(ts(k)); y4 = f4(ts(k));

    acc_samp(k,1) = y1(3);
    acc_samp(k,2) = y2(3);
    acc_samp(k,3) = y3(3);
    acc_samp(k,4) = y4(3);
end

bad = ~isfinite(acc_samp);
acc_samp(bad) = 0;

figure;
subplot(4,1,1); plot(ts, acc_samp(:,1)); grid on; ylabel('\alpha_1 (rad/s^2)');
subplot(4,1,2); plot(ts, acc_samp(:,2)); grid on; ylabel('\alpha_2 (rad/s^2)');
subplot(4,1,3); plot(ts, acc_samp(:,3)); grid on; ylabel('a_3 (m/s^2)');
subplot(4,1,4); plot(ts, acc_samp(:,4)); grid on; ylabel('\alpha_4 (rad/s^2)'); xlabel('t (s)');

%% End effector position & orientation plotted in 3D

Q_phys = q_anim;
Q_phys(:,3) = q_anim(:,3) + d3_min;

Nn = size(Q_phys,1);

pE = zeros(Nn,3);
xE = zeros(Nn,3); 
yE = zeros(Nn,3);
zE = zeros(Nn,3);

for k = 1:Nn
    T0E = scaraFK(Q_phys(k,:), L1, L2, d1, d4);   % 4x4
    R = T0E(1:3,1:3);
    p = T0E(1:3,4);

    pE(k,:) = p.';
    xE(k,:) = R(:,1).';
    yE(k,:) = R(:,2).';
    zE(k,:) = R(:,3).';
end

figure;
plot3(pE(:,1), pE(:,2), pE(:,3), 'LineWidth', 1.5);
grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('End-effector position & orientation');

hold on;

step = max(1, floor(Nn/30));
idx = 1:step:Nn;

Lax = 0.30;

quiver3(pE(idx,1), pE(idx,2), pE(idx,3), ...
        xE(idx,1), xE(idx,2), xE(idx,3), Lax, ...
        'LineWidth', 1.2, 'MaxHeadSize', 0.8);

quiver3(pE(idx,1), pE(idx,2), pE(idx,3), ...
        yE(idx,1), yE(idx,2), yE(idx,3), Lax, ...
        'LineWidth', 1.2, 'MaxHeadSize', 0.8);

quiver3(pE(idx,1), pE(idx,2), pE(idx,3), ...
        zE(idx,1), zE(idx,2), zE(idx,3), Lax, ...
        'LineWidth', 1.2, 'MaxHeadSize', 0.8);

legend('Position path','x-axis','y-axis','z-axis');

%% IK Functions
function q = scaraIK(Tsd, L1, L2, d1, d4, elbow)

    if nargin < 6, elbow = -1; end
    x = Tsd(1,4); y = Tsd(2,4); z = Tsd(3,4);
    psi = atan2(Tsd(2,1), Tsd(1,1));

    r2 = x^2 + y^2;
    c2 = (r2 - L1^2 - L2^2) / (2*L1*L2);
    c2 = max(min(c2,1),-1);
    s2 = elbow * sqrt(max(0, 1 - c2^2));

    q2 = atan2(s2, c2);
    q1 = atan2(y,x) - atan2(L2*s2, L1 + L2*c2);

    d3 = z - d1 - d4;
    q4 = psi - q1 - q2;

    q = [q1 q2 d3 q4];
end

function T = scaraFK(q, L1, L2, d1, d4)

    q1=q(1); q2=q(2); d3=q(3); q4=q(4);
    x = L1*cos(q1) + L2*cos(q1+q2);
    y = L1*sin(q1) + L2*sin(q1+q2);
    z = d1 + d3 + d4;
    psi = q1 + q2 + q4;

    R = [cos(psi) -sin(psi) 0;
         sin(psi)  cos(psi) 0;
         0         0        1];
    T = [R [x;y;z]; 0 0 0 1];
end

%% Function provided by Guanyu Chen
function [f, ind] = generateMVPLinearParabolicBlendTrajectory(P, T, Acc)
% Author: Guanyu Chen
% Date: 12/31/2025
% Contact: gychen98@ucla.edu
    ind = 1;
    Acc = abs(Acc);
    F = {cell(length(T), 2),cell(length(T), 2),cell(length(T), 2)};
    g = @(t) 0;
    for k = 1:3
        [F{k}{:}] = deal(g);
    end
    T_linear = zeros(1,length(T) - 1);
    T_blend = zeros(1,length(T));
    Td = diff(T);

    if length(P) ~= length(T) || length(Acc) ~= length(T)
        disp("Generation failed! Check input dimensions!")
        ind = 0;
    else
        for i = 1:length(T)
            if i == 1
                acc = Acc(i) .* sign(P(i+1)-P(i));
                T_blend(i) = Td(i) - sqrt(Td(i)^2-2*(P(i+1)-P(i))./acc);
                theta12dot = (P(i+1)-P(i))/(Td(i)-1/2*T_blend(i));
                fl = @(t) P(i) + theta12dot * (t - 1/2 * T_blend(i));
                fb = @(t) P(i) + 1/2 * theta12dot/T_blend(i) * t^2;
                fvl = @(t) theta12dot;
                fvb = @(t) theta12dot/T_blend(i) * t;
                fal = @(t) 0 * theta12dot;
                fab = @(t) theta12dot/T_blend(i);
                F{1}{i,1} = fl;  F{1}{i,2} = fb;
                F{2}{i,1} = fvl; F{2}{i,2} = fvb;
                F{3}{i,1} = fal; F{3}{i,2} = fab;

            elseif i == length(T)
                acc = Acc(:,i) .* sign(P(i)-P(i-1));
                T_blend(i) = Td(i-1) - sqrt(Td(i-1)^2-2*(P(i)-P(i-1))./acc);
                if i == 3
                    theta1dot = (P(i-1)-P(i-2))/(Td(i-2)-1/2*T_blend(i-2));
                else
                    theta1dot = (P(i-1)-P(i-2))/Td(i-2);
                end
                theta2dot = (P(i)-P(i-1))/(Td(i-1)-1/2*T_blend(i));
                acc12 = Acc(:,i) .* sign(theta2dot - theta1dot);
                T_blend(i-1) = (theta2dot - theta1dot)./acc12;

                if i==3
                    T_linear(i-2) = Td(i-2) - 1/2*T_blend(i-1) - T_blend(i-2);
                else
                    T_linear(i-2) = Td(i-2) - 1/2*T_blend(i-1) - 1/2*T_blend(i-2);
                end
                T_linear(i-1) = Td(i-1) - 1/2*T_blend(i-1) - T_blend(i);
                thetanm1ndot = (P(i)-P(i-1))/(Td(i-1)-1/2*T_blend(i));

                fl = @(t) P(i-1) + thetanm1ndot * (1/2*T_blend(i-1)+T_linear(i-1)) + thetanm1ndot * (T_blend(i)) - 1/2 * thetanm1ndot/T_blend(i) * (T_blend(i))^2;
                fb = @(t) P(i-1) + thetanm1ndot * (1/2*T_blend(i-1)+T_linear(i-1)) + thetanm1ndot * (t) - 1/2 * thetanm1ndot/T_blend(i) * (t)^2;
                fvl = @(t) thetanm1ndot * 0;
                fvb = @(t) thetanm1ndot - thetanm1ndot/T_blend(i) * (t);
                fal = @(t) 0 * theta12dot;
                fab = @(t) acc;
                F{1}{i,1} = fl;
                F{2}{i,1} = fvl;
                F{3}{i,1} = fal;
                F{1}{i,2} = fb;
                F{2}{i,2} = fvb;
                F{3}{i,2} = fab;

                fl = @(t) P(i-1) + theta2dot * (t-1/2*T_blend(i-1));
                fb = @(t) P(i-2) + theta1dot * (1/2*T_blend(i-2)+T_linear(i-2)) + theta1dot * (t) + 1/2 * acc12 * (t)^2;
                fvl = @(t) theta2dot;
                fvb = @(t) theta1dot + acc12 * (t);
                fal = @(t) 0 * theta2dot;
                fab = @(t) acc12;
                F{1}{i-1,1} = fl; F{1}{i-1,2} = fb;
                F{2}{i-1,1} = fvl; F{2}{i-1,2} = fvb;
                F{3}{i-1,1} = fal; F{3}{i-1,2} = fab;

            elseif i<length(T)-1 && i > 1
                theta2dot = (P(i+1)-P(i))/Td(i);
                if i ==2
                    theta1dot = (P(i)-P(i-1))/(Td(i-1)-1/2*T_blend(i-1));
                else
                    theta1dot = (P(i)-P(i-1))/Td(i-1);
                end
                acc = Acc(:,i) .* sign(theta2dot - theta1dot);
                T_blend(i) = (theta2dot - theta1dot)./acc;

                if i == 2
                    T_linear(i-1) = Td(i-1) - 1/2*T_blend(i) - T_blend(i-1);
                else
                    T_linear(i-1) = Td(i-1) - 1/2*T_blend(i) - 1/2*T_blend(i-1);
                end

                fl = @(t) P(i) + theta2dot * (t-1/2*T_blend(i));
                fb = @(t) P(i-1) + theta1dot * (1/2*T_blend(i-1)+T_linear(i-1)) + theta1dot * (t) + 1/2 * acc * (t)^2;
                fvl = @(t) theta2dot;
                fvb = @(t) theta1dot + acc * (t);
                fal = @(t) 0 * theta2dot;
                fab = @(t) acc;
                F{1}{i,1} = fl; F{1}{i,2} = fb;
                F{2}{i,1} = fvl; F{2}{i,2} = fvb;
                F{3}{i,1} = fal; F{3}{i,2} = fab;
            end
        end
    end

    T_modified = T;
    T_modified(2:end-1) = T(2:end-1) - 1/2 * T_blend(2:end-1);
    T_modified(end) = T(end) - T_blend(end);
    T_modified = [T_modified, T(end)];

    if ~validateBlendtime(T_blend, Td)
        ind = 0;
    end

    f = @getConfiguration;
    function y = getConfiguration(t)
        if t == T(1)
            idx = 1;
        else
            idx = find(T_modified < t, 1, 'last');
        end
        t = t - T_modified(idx);
        if t <= T_blend(idx)
            f_pos = F{1}{idx,2};
            f_vel = F{2}{idx,2};
            f_acc = F{3}{idx,2};
        else
            f_pos = F{1}{idx,1};
            f_vel = F{2}{idx,1};
            f_acc = F{3}{idx,1};
        end
        y = [f_pos(t);f_vel(t);f_acc(t)];
    end
end

function ind2 = validateBlendtime(T_blend, Td)
    ind2 = 1;
    for i = 1: length(T_blend) - 1
        if 1/2*T_blend(i)+1/2*T_blend(i+1)>Td(i)
            disp("The duration for " + i + " and " + (i+1) + " blend is too long. Try greater acceleration for these blends!")
            ind2 = 0;
        end
    end
end

%% Animation
function scara = buildSCARA_RTB(L1, L2, d1, d4, zT, zP)
deg = pi/180;

% include reference d3=0
d3_vals = [[zT zP] - d1 - d4, 0];   % <-- add 0
d3_min = min(d3_vals);
d3_max = max(d3_vals);

L(1) = Link([0 d1 L1 0 0 0]);      L(1).qlim = [-170 170]*deg;
L(2) = Link([0 0  L2 0 0 0]);      L(2).qlim = [-170 170]*deg;

L(3) = Link([0 0  0  0  1 0]);
L(3).offset = d3_min;
L(3).qlim   = [0, d3_max-d3_min];  % now includes reference height

L(4) = Link([0 d4 0  0  0 0]);     L(4).qlim = [-180 180]*deg;

scara = SerialLink(L, 'name', 'SCARA');
end

%% Feeder and PCB position
function P = makeStationPoses(xP, yP, z_travel, z_place, deg)

    Rz = @(psi)[cos(psi) -sin(psi) 0;
                sin(psi)  cos(psi) 0;
                0         0        1];
    Tform = @(R,p)[R p(:); 0 0 0 1];

    Delta = 0.045;
    feeder_dx = 0.070;

    xF = xP + feeder_dx;
    yF = yP;
    psiF = 0*deg;

    P.Ft = Tform(Rz(psiF), [xF, yF, z_travel]);
    P.Fp = Tform(Rz(psiF), [xF, yF, z_place]);

    Cxy = [ +Delta, -Delta;
            -Delta, -Delta; 
            -Delta, +Delta;
            +Delta, +Delta];

    psi = [-90, 0, 90, 180] * deg;

    P.C1t = Tform(Rz(psi(1)), [xP + Cxy(1,1), yP + Cxy(1,2), z_travel]);
    P.C1p = Tform(Rz(psi(1)), [xP + Cxy(1,1), yP + Cxy(1,2), z_place]);

    P.C2t = Tform(Rz(psi(2)), [xP + Cxy(2,1), yP + Cxy(2,2), z_travel]);
    P.C2p = Tform(Rz(psi(2)), [xP + Cxy(2,1), yP + Cxy(2,2), z_place]);

    P.C3t = Tform(Rz(psi(3)), [xP + Cxy(3,1), yP + Cxy(3,2), z_travel]);
    P.C3p = Tform(Rz(psi(3)), [xP + Cxy(3,1), yP + Cxy(3,2), z_place]);

    P.C4t = Tform(Rz(psi(4)), [xP + Cxy(4,1), yP + Cxy(4,2), z_travel]);
    P.C4p = Tform(Rz(psi(4)), [xP + Cxy(4,1), yP + Cxy(4,2), z_place]);
end

%% 2 points LIPB
function [f, ind] = buildPiecewiseTwoPointLIPB(P, T, Acc)

    ind = 1;
    P   = P(:).';
    T   = T(:).';
    Acc = abs(Acc(:).');

    N = length(T);
    if length(P) ~= N || length(Acc) ~= N || N < 2
        ind = 0;
        f = @(t)[NaN;NaN;NaN];
        return;
    end

    Td = diff(T);
    segA = min(Acc(1:end-1), Acc(2:end));

    seg = struct('p0',{},'p1',{},'T',{},'a',{},'tb',{},'v',{},'s',{},'feas',{});
    for i = 1:N-1
        p0 = P(i); p1 = P(i+1);
        Ti = Td(i);
        a  = segA(i);

        dq = p1 - p0;
        sgn = sign(dq); if sgn==0, sgn = 1; end
        D = abs(dq);

        disc = Ti^2 - 4*D/a;
        if disc < -1e-12
            feas = false;
            tb = 0; v = 0;
            ind = 0;
        else
            feas = true;
            disc = max(disc, 0);
            tb = 0.5*(Ti - sqrt(disc));
            v  = a*tb; 
            tb = max(tb, 0);
            tb = min(tb, Ti/2);
            v  = a*tb;
        end

        seg(i).p0   = p0;
        seg(i).p1   = p1;
        seg(i).T    = Ti;
        seg(i).a    = a;
        seg(i).tb   = tb;
        seg(i).v    = v;
        seg(i).s    = sgn;
        seg(i).feas = feas;
    end

    f = @getY;

    function y = getY(t)
        if t <= T(1)
            y = [P(1); 0; 0];
            return;
        elseif t >= T(end)
            y = [P(end); 0; 0];
            return;
        end

        i = find(T <= t, 1, 'last');
        i = min(max(i,1), N-1);

        tau = t - T(i);
        Si = seg(i);

        if ~Si.feas
            u = tau / Si.T;
            pos = (1-u)*Si.p0 + u*Si.p1;
            vel = (Si.p1 - Si.p0)/Si.T;
            acc = 0;
            y = [pos; vel; acc];
            return;
        end

        a  = Si.a * Si.s;
        tb = Si.tb;
        Ti = Si.T;
        v  = Si.v * Si.s;

        if tau <= tb
            pos = Si.p0 + 0.5*a*tau^2;
            vel = a*tau;
            acc = a;
        elseif tau <= (Ti - tb)
            pos_tb = Si.p0 + 0.5*a*tb^2;
            pos = pos_tb + v*(tau - tb);
            vel = v;
            acc = 0;
        else
            t2 = Ti - tau;
            pos = Si.p1 - 0.5*a*t2^2;
            vel = a*t2;
            acc = -a;
        end

        y = [pos; vel; acc];
    end
end
