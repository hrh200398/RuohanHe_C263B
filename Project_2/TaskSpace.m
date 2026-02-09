clear; 
close all; 
clc;

%% Limits & Robot Geometry
deg = pi/180;
a_th  = 50*deg; % rad/s^2
a_lin = 2.0; % m/s^2

L1 = 0.325; % all in m
L2 = 0.225;
d1 = 0.416;
d4 = -0.080;

zT = 0.300; % travel height
zP = 0.280; % pick/place height (m)

dt = 0.02;  % sampling time (s) ~50 Hz

%% Matrix
Rz = @(psi)[cos(psi) -sin(psi) 0;
            sin(psi)  cos(psi) 0;
            0         0        1];

T  = @(R,p)[R p(:); 0 0 0 1];

wrapToPi_ = @(x) atan2(sin(x), cos(x));

%% Via points 
xP = 0.250;  % input
yP = 0.225; % input
P  = makeStationPoses(xP, yP, zT, zP, deg);

TFt = P.Ft; TFp = P.Fp;
TC1t = P.C1t; TC1p = P.C1p;
TC2t = P.C2t; TC2p = P.C2p;
TC3t = P.C3t; TC3p = P.C3p;
TC4t = P.C4t; TC4p = P.C4p;

%% Reference pose
qref = [0, 90*deg, 0, 0]; 
Tref = scaraFK(qref, L1, L2, d1, d4);

%% Viapoint list
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

%% TASK-SPACE APPROACH

Qdot_task = [];
Qdd_task  = [];

T_total_task = 0;
Q_task = [];
t_task = [];

t_now = 0;
q_prev = [];

maxIter = 30;
tol = 1e-6;
singThresh = 1e-10;

for k = 1:numel(traj)
    W = traj{k};

    for i = 1:numel(W)-1
        T0 = W{i};     T1 = W{i+1};

        p0 = T0(1:3,4);   p1 = T1(1:3,4);

        psi0 = atan2(T0(2,1), T0(1,1));
        psi1 = atan2(T1(2,1), T1(1,1));
        dpsi_wrap = wrapToPi_(psi1 - psi0);
        dpsi = abs(dpsi_wrap);

        L = norm(p1 - p0);

        tf_p   = 2*sqrt(L/a_lin);
        tf_psi = 2*sqrt(dpsi/a_th);
        tf = max(tf_p, tf_psi);
        tb = tf/2;
        q_prev_local_init = q_prev;

        for it = 1:maxIter
            tvec = (0:dt:tf).';
            [s, sdot, sddot] = lspb_scalar_full(tvec, tf, tb);

            dp = (p1 - p0);
            p      = p0.' + s * dp.';
            pdot   = sdot * dp.';
            pddot  = sddot * dp.';

            psi     = psi0 + s * dpsi_wrap;
            psidot  = sdot * dpsi_wrap;
            psiddot = sddot * dpsi_wrap;

            Nn = numel(tvec);
            q_arr    = zeros(Nn,4);
            qdot_arr = zeros(Nn,4);
            qdd_arr  = zeros(Nn,4);

            local_ok = true;
            maxRatio = 0;

            q_prev_local = q_prev_local_init;

            for j = 1:Nn
                Tsd = [Rz(psi(j)) p(j,:).'; 0 0 0 1];

                q = scaraIK(Tsd, L1, L2, d1, d4, -1);
                if ~isempty(q_prev_local)
                    q = unwrapToPrevious(q, q_prev_local);
                end
                q_prev_local = q;
                q_arr(j,:) = q;

                % Jacobian
                J = scaraJacobian(q, L1, L2);
                if rcond(J) < singThresh
                    warning('Near-singular Jacobian at segment %d-%d (sample %d). Increasing tf.', k, i, j);
                    local_ok = false;
                    maxRatio = max(maxRatio, 4); % force increase
                    break;
                end

                xdot  = [pdot(j,:)'; psidot(j)];
                qdot  = J \ xdot;
                qdot_arr(j,:) = qdot.';

                Jdot  = scaraJacobianDot(q, qdot, L1, L2);
                xddot = [pddot(j,:)'; psiddot(j)];
                qddot = J \ (xddot - Jdot*qdot);
                qdd_arr(j,:) = qddot.';

                % accel limits per joint
                ratio1 = abs(qddot(1))/a_th;
                ratio2 = abs(qddot(2))/a_th;
                ratio3 = abs(qddot(3))/a_lin;
                ratio4 = abs(qddot(4))/a_th;
                maxRatio = max([maxRatio ratio1 ratio2 ratio3 ratio4]);
            end

            if local_ok && (maxRatio <= 1 + tol)
                break;
            else
                scale = sqrt(maxRatio) * 1.02;
                tf = tf * max(1.05, scale);
                tb = tf/2;

                if it == maxIter
                    warning('Segment %d-%d reached maxIter; using enlarged tf anyway.', k, i);
                end
            end
        end

        T_total_task = T_total_task + tf;

        tvec = (0:dt:tf).';
        [s, sdot, sddot] = lspb_scalar_full(tvec, tf, tb);

        dp = (p1 - p0);
        Nn = numel(tvec);

        q_arr    = zeros(Nn,4);
        qdot_arr = zeros(Nn,4);
        qdd_arr  = zeros(Nn,4);

        for j = 1:Nn
            p_       = p0 + s(j)*dp;
            psi_     = psi0 + s(j)*dpsi_wrap;

            pdot_    = sdot(j)*dp;
            pddot_   = sddot(j)*dp;

            psidot_  = sdot(j)*dpsi_wrap;
            psiddot_ = sddot(j)*dpsi_wrap;

            Tsd = [Rz(psi_) p_; 0 0 0 1];

            q = scaraIK(Tsd, L1, L2, d1, d4, -1);

            if ~isempty(Q_task)
                q = unwrapToPrevious(q, Q_task(end,:));
            elseif ~isempty(q_prev)
                q = unwrapToPrevious(q, q_prev);
            end
            q_arr(j,:) = q;

            J = scaraJacobian(q, L1, L2);
            xdot  = [pdot_;  psidot_];
            qdot  = J \ xdot;
            qdot_arr(j,:) = qdot.';

            Jdot  = scaraJacobianDot(q, qdot, L1, L2);
            xddot = [pddot_; psiddot_];
            qddot = J \ (xddot - Jdot*qdot);
            qdd_arr(j,:) = qddot.';
        end

        Q_task    = [Q_task;    q_arr];
        Qdot_task = [Qdot_task; qdot_arr];
        Qdd_task  = [Qdd_task;  qdd_arr];
        t_task = [t_task; (t_now + tvec)];
        t_now  = t_now + tf;

        q_prev = Q_task(end,:);
    end
end

fprintf('Total execution time (Task-space, LIPB + Jacobian accel check) = %.3f s\n', T_total_task);

%% Animation and figures

d3_min = min(Q_task(:,3));

Q_anim = Q_task;
Q_anim(:,3) = Q_task(:,3) - d3_min;
Q_anim(:,3) = max(Q_anim(:,3), 0);

scara.links(3).offset = d3_min;

scara = buildSCARA_RTB(L1, L2, d1, d4);
q3_min = min(Q_task(:,3));
Q_anim = Q_task;
Q_anim(:,3) = Q_task(:,3) - q3_min;

scara.links(3).offset = q3_min;

Q_anim(:,3) = max(Q_anim(:,3), 0);

figure;
scara.plot(Q_anim, 'workspace', [-0.6 0.6 -0.6 0.6 0 0.6], ...
           'delay', dt, 'trail', 'k-');

t_task_plot = t_task(:);

figure;
subplot(4,1,1);
plot(t_task_plot, Q_task(:,1)); grid on;
ylabel('q_1 (rad)');

subplot(4,1,2);
plot(t_task_plot, Q_task(:,2)); grid on;
ylabel('q_2 (rad)');

subplot(4,1,3);
plot(t_task_plot, Q_task(:,3)); grid on;
ylabel('d_3 (m)');

subplot(4,1,4);
plot(t_task_plot, Q_task(:,4)); grid on;
ylabel('q_4 (rad)');
xlabel('t (s)');


% v-t plots
figure;
subplot(4,1,1); plot(t_task, Qdot_task(:,1)); grid on; ylabel('\omega_1 (rad/s)');
subplot(4,1,2); plot(t_task, Qdot_task(:,2)); grid on; ylabel('\omega_2 (rad/s)');
subplot(4,1,3); plot(t_task, Qdot_task(:,3)); grid on; ylabel('v_3 (m/s)');
subplot(4,1,4); plot(t_task, Qdot_task(:,4)); grid on; ylabel('\omega_4 (rad/s)'); xlabel('t (s)');

% a-t plots
figure;
subplot(4,1,1); plot(t_task, Qdd_task(:,1)); grid on; ylabel('\alpha_1 (rad/s^2)');
subplot(4,1,2); plot(t_task, Qdd_task(:,2)); grid on; ylabel('\alpha_2 (rad/s^2)');
subplot(4,1,3); plot(t_task, Qdd_task(:,3)); grid on; ylabel('a_3 (m/s^2)');
subplot(4,1,4); plot(t_task, Qdd_task(:,4)); grid on; ylabel('\alpha_4 (rad/s^2)'); xlabel('t (s)');


N = size(Q_task,1);

pE = zeros(N,3);     % end-effector position
xE = zeros(N,3);     % end-effector x-axis direction (unit vector)
yE = zeros(N,3);     % end-effector y-axis direction
zE = zeros(N,3);     % end-effector z-axis direction

for k = 1:N
    T0E = scaraFK(Q_task(k,:), L1, L2, d1, d4);   % 4x4
    R = T0E(1:3,1:3);
    p = T0E(1:3,4);

    pE(k,:) = p.';
    xE(k,:) = R(:,1).';
    yE(k,:) = R(:,2).';
    zE(k,:) = R(:,3).';
end

% 3D position curve
figure;
plot3(pE(:,1), pE(:,2), pE(:,3), 'LineWidth', 1.5);
grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('End-effector position trajectory');

hold on;
step = max(1, floor(N/30));
Lax = 0.3;

idx = 1:step:N;

quiver3(pE(idx,1), pE(idx,2), pE(idx,3), ...
        xE(idx,1), xE(idx,2), xE(idx,3), Lax, 'LineWidth', 1);

quiver3(pE(idx,1), pE(idx,2), pE(idx,3), ...
        yE(idx,1), yE(idx,2), yE(idx,3), Lax, 'LineWidth', 1);

quiver3(pE(idx,1), pE(idx,2), pE(idx,3), ...
        zE(idx,1), zE(idx,2), zE(idx,3), Lax, 'LineWidth', 1);

legend('Position path','x-axis','y-axis','z-axis');

%% Functions

function q = scaraIK(Tsd, L1, L2, d1, d4, elbow)
    if nargin < 6, elbow = -1; end
    x = Tsd(1,4); y = Tsd(2,4); z = Tsd(3,4);
    psi = atan2(Tsd(2,1), Tsd(1,1));

    r2 = x^2 + y^2;
    c2 = (r2 - L1^2 - L2^2)/(2*L1*L2);
    c2 = max(min(c2, 1), -1);
    s2 = elbow*sqrt(max(0, 1 - c2^2));

    q2 = atan2(s2, c2);
    q1 = atan2(y, x) - atan2(L2*s2, L1 + L2*c2);

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


function q_new = unwrapToPrevious(q_new, q_prev)
    wrapToPi_ = @(x) atan2(sin(x), cos(x));
    q_new(1) = q_prev(1) + wrapToPi_(q_new(1) - q_prev(1));
    q_new(2) = q_prev(2) + wrapToPi_(q_new(2) - q_prev(2));
    q_new(4) = q_prev(4) + wrapToPi_(q_new(4) - q_prev(4));
end

function [s, sdot, sddot] = lspb_scalar_full(t, tf, tb)
    t = t(:);
    s = zeros(size(t));
    sdot = zeros(size(t));
    sddot = zeros(size(t));

    if tb <= 0 || tb > tf/2
        error('tb must satisfy 0 < tb <= tf/2');
    end

    a = 1/(tb*(tf - tb));

    for k = 1:numel(t)
        tk = t(k);
        if tk <= tb
            s(k) = 0.5*a*tk^2;
            sdot(k) = a*tk;
            sddot(k) = a;
        elseif tk <= (tf - tb)
            s(k) = a*tb*(tk - tb/2);
            sdot(k) = a*tb;
            sddot(k) = 0;
        else
            dt = tf - tk;
            s(k) = 1 - 0.5*a*dt^2;
            sdot(k) = a*dt;
            sddot(k) = -a;
        end
    end

    s = max(0, min(1, s));
end

function J = scaraJacobian(q, L1, L2)
    % x,y,z,psi w.r.t q1,q2,q3,q4
    q1=q(1); q2=q(2);
    s1 = sin(q1); c1 = cos(q1);
    s12 = sin(q1+q2); c12 = cos(q1+q2);

    dx_dq1 = -L1*s1 - L2*s12;
    dx_dq2 = -L2*s12;
    dy_dq1 =  L1*c1 + L2*c12;
    dy_dq2 =  L2*c12;

    J = [dx_dq1, dx_dq2, 0, 0;
         dy_dq1, dy_dq2, 0, 0;
         0,      0,      1, 0;
         1,      1,      0, 1];
end

function Jd = scaraJacobianDot(q, qdot, L1, L2)
    % time derivative of Jacobian
    q1=q(1); q2=q(2);
    q1d=qdot(1); q2d=qdot(2);
    w12 = q1d + q2d;

    c1 = cos(q1); s1 = sin(q1);
    c12 = cos(q1+q2); s12 = sin(q1+q2);

    d_dt_dx_dq1 = -L1*c1*q1d - L2*c12*w12;
    d_dt_dx_dq2 = -L2*c12*w12;

    d_dt_dy_dq1 = -L1*s1*q1d - L2*s12*w12;
    d_dt_dy_dq2 = -L2*s12*w12;

    Jd = [d_dt_dx_dq1, d_dt_dx_dq2, 0, 0;
          d_dt_dy_dq1, d_dt_dy_dq2, 0, 0;
          0,           0,           0, 0;
          0,           0,           0, 0];
end

function scara = buildSCARA_RTB(L1, L2, d1, d4)
deg = pi/180;

L(1) = Link([0 0 L1 0 0 0]);     L(1).qlim = [-170 170]*deg;
L(2) = Link([0 0 L2 0 0 0]);     L(2).qlim = [-170 170]*deg;
L(3) = Link([0 0 0  0 1 0]);     
L(4) = Link([0 0 0  0 0 0]);     L(4).qlim = [-180 180]*deg;

scara = SerialLink(L, 'name', 'SCARA');


scara.base = [1 0 0 0;
             0 1 0 0;
             0 0 1 d1;
             0 0 0 1];

scara.tool = [1 0 0 0;
             0 1 0 0;
             0 0 1 d4;
             0 0 0 1];
end

%% Feeder & PCB

function P = makeStationPoses(xP, yP, z_travel, z_place, deg)

    Rz = @(psi)[cos(psi) -sin(psi) 0;
                sin(psi)  cos(psi) 0;
                0         0        1];
    Tform = @(R,p)[R p(:); 0 0 0 1];

    Delta = 0.045; % = 0.05 - 0.005
    feeder_dx = 0.070;     % feeder is +x (left) from PCB center

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

