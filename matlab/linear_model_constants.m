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
R = 2.181818 ; %Resistance des moteurs
k = 0.3346 ; %Constante de couple

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


B_0  = -2 * k / (rho * R);
B_1 = 2 * k / R;
B_2 = 1 / R;


T = [1 0 0 0 ;
     0 A_0 0 A_1;
     0 0 1 0 ;
     0 A_3 0 A_4];

Q_0 = [0 1 0 0;
      0 -A_2*B_0 0 -A_2*B_1;
      0 0 0 1;
      0 -A_6*B_0 -A_5 -A_6 *B_1];

Q_1 = [0 0;
       -A_2*B_2 -A_2*B_2;
       0 0;
       -A_6*B_2 -A_6*B_2];

A = inv(T) * Q_0;
B = inv(T) * Q_1;

[V, D] = eig(A)

 

% Observer matrix for W_dot = A_W * W + B_W * e_W
A_W = [0 1 0;
       0 0 g;
       0 0 0];

B_W = [-d_i 0;
        0 1;
        1 0];

% y_W = C_W * W, y_W = alpha_r + alpha_l

C_W = [2/rho 0 -2];



damping = 0.7;
resonant_freq = sqrt(g / d);
damping_poly = [1 (2 * damping * resonant_freq) (resonant_freq * resonant_freq)];
r_K = roots(damping_poly);

D(2, 2)

p_K = [-0.1, D(2, 2), r_K(1), r_K(2)];

K = place(A, B, p_K)


bessel_poly = [1 6 15 15];
observer_scale = 500;
%old_one

%p_L = observer_scale * [-0.7456 + 0.7114i,-0.7456 - 0.7114i, -0.9412]

% good one
%p_L = 10*[-35.786800+36.509839i, -35.786800-36.509839i, -61.348800];

ev_bessel = -0.7 + 1i * sqrt(1-0.7^2);
p_L = 50 * [ -1 , ev_bessel , conj(ev_bessel)];

%p_L = 10 * p_K(2:4); % choose corresponding controller poles


L = place(A_W', C_W', p_L)'

[V_L, D_L] = eig(A_W - L * C_W);
[V_K, D_K] = eig(A - B * K);