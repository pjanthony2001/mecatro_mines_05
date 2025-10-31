m_w = 0.41 ; %Masse d'une unite (roue + engrenage + rotor)
m_b = 1.8 ; %Masse du chassis du robot

I_x = 9.942 * 1e-3;
I_y = 5.31 * 1e-3;
I_z = 5.953 * 1e-3; %Moment d'inertie du chassis autour de chacun de ses axes, a C

Iw_x = 4.8886 * 1e-4;
Iw_y = 2.5233 * 1e-4;
Iw_z = 2.5307 * 1e-4; %Moment d'inertie de la roue autour de chacun de ses axes, a Cw

l = 0.171 ; %Distance entre les deux roues
d = 0.044 ; %Distance entre A et C
rho = 0.050 ; %position verticale(rayon du roue)
d_i = 0.100 ; %Distance verticale entre l'IMU et l'axe des roues

L = 0 ; %Inductance des moteurs 
R = 2 ; %Resistance des moteurs
k = 0.38 ; %Constante de couple

g = 9.81; %constante de gravitation


M = m_b + 2 * m_w;
I_psi = I_x + I_z + Iw_x + Iw_z + (m_w * l * l) / 2 + (Iw_y * l * l) / (2 * rho * rho);
Itot_y = I_y + 2 * Iw_y;

A_0 = M + (2 * Iw_y) / (rho * rho);
A_1 = m_b * d;
A_2 = -k / rho;
A_3 = m_b * d;
A_4 = Itot_y + m_b * d * d;
A_5 = -m_b * d * g;
A_6 = k;


% A_7 = I_psi;
% A_8 = -(l * k) / (2 * rho);

B_0 = (A_0 * A_6 - A_2 * A_3) / (A_2 * A_5);
B_1 = (A_1 * A_6 - A_2 * A_4) / (A_2 * A_5);

B_3 = - (A_0 * A_5) / (A_0 * A_4 - A_1 * A_3);

B_4 = (A_2 * A_3 - A_0 * A_6) / (A_0 * A_4 - A_1 * A_3);

C_0 = (-2 * k) / (rho * R * B_0);
C_1 = (2 * k / R) * ((B_1 / (rho * B_1)) + 1);
C_2 = 1 / R;


A = [0 1 0 0
     0 0 1 0
     0 0 0 1
     0 (B_4 * C_0) B_3 (B_4 * C_1)]




B = [0 0 
     0 0 
     0 0 
     (B_4 * C_2) (B_4 * C_2)];

% For linear generation of Omega -> alpha
C_3 = 1 / (rho * B_0)
C_4 = -(B_1 / (rho * B_0) + 1)

% For linearisation of y_a
C_5 = -g;
C_6 = 1 / B_0;
C_7 = (d_i - (B_1 / B_0));

% Observer matrix for W_dot = A_W * W + B_W * e_W
A_W = [0 1 0
       0 0 g
       0 0 0];

B_W = [-d_i 0
        0 1
        1 0];

% y_W = C_W * W, y_W = alpha_r * alpha_l

C_W = [2/rho 0 -2];


% Conversion Matrices for X = A_XW * W + B_XW * e_W

A_XW = [B_0 0 B_1
        0 B_0 0
        0 0 1
        0 0 0];


B_XW = [0 0
        (B_1 - (d_i * B_0)) 0
        0 0
        1 0];

[ V , D ] = eig( A )


sys = ss(A,B,[0 0 1 0], 0);
figure(1)

% Simulate the system response to an initial condition
t = 0:0.01:5; % Time vector
u = [zeros(size(t)) 
    zeros(size(t))]; % Input vector (no external input)
[y, t, x] = lsim(sys, u, t, X_0); % Simulate the system
plot(t, y); % Plot the output response

xlabel('Time (s)');
ylabel('Output Response');
title('System Response to Initial Condition');

damping = 0.5;
resonant_freq = sqrt(g / d);
damping_poly = [1 (2 * damping * resonant_freq) (resonant_freq * resonant_freq)];
r_K = roots(damping_poly);
scale = 1;

D(2, 2)

Pol  = pole(sys);
p_K = [-0.1, D(2, 2), scale * r_K(1), scale * r_K(2)];

K = place(A, B, p_K)

% Calculate the closed-loop system response with the state feedback
[y_cl, t_cl, x_cl] = lsim(ss(A - B * K, B, [0 0 1 0], 0), u, t, X_0); % Simulate closed-loop response
figure(2)
plot(t_cl, y_cl); % Plot the closed-loop output response
xlabel('Time (s)');
ylabel('Closed-Loop Output Response');
title('Closed-Loop System Response to Initial Condition');

figure(3)
plot(t_cl, (- K * x_cl')');




bessel_poly = [1 6 15 15];
observer_scale = 100;

p_L = observer_scale * [-0.9412, -0.7456 + 0.7114i, -0.7456 - 0.7114i];
L = place(A_W', C_W', p_L)'




M_0 = [1 0 0 0
       0 A_0 0 A_1
       0 0 1 0
       0 A_3 0 A_4];



D_0 = 2 * A_2 * k / (rho * R);
D_1 = 2 * A_6 * k / (rho * R);
D_3 = A_6 / R;


M_1 = [0 1 0 0
       0 D_0 0 -rho * D_0
       0 0 0 1
       0 D_1 0 -rho * D_1];

inv(M_0) * M_1
[V_, D_] = eig(inv(M_0) * M_1)