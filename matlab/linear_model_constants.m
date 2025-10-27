mw = 1 ; %Masse d'une unit´e (roue + engrenage + rotor)
mb = 1 ; %Masse du chassis du robot
Ix = 1 ;Iy = 1 ;Iz = 1 ; %Moment d'inertie du chassis autour de chacun de ses axes, `a C
Iwx = 1 ;Iwy = 1 ;Iwz = 1 ;%Moment d'inertie de la roue autour de chacun de ses axes, `a Cw
Rho = 1 ; %position verticale(rayon du roue)
l = 1 ; %Distance entre les deux roues
d = 1 ; %Distance entre A et C
di = 1 ; %Distance verticale entre l'IMU et l'axe des roues
L = 1 ; %Inductance des moteurs 
R = 1 ; %R´esistance des moteurs
k = 1 ; %Constante de couple
g = 9.81; %constante de gravitation


M = mb+2*mw;
Ipsi = Ix+Iz+Iwx+Iwz+(mw*l*l)/2+(Iwy*l*l)/(2*Rho*Rho);
Itoty = Iy+2*Iwy;


A = M+(2*Iwy)/(Rho*Rho) ; B = mb*d; D = mb*d ; C = -k/Rho ;
E = Itoty+mb*d*d ; F = -mb*d; G = k;

A1 = (F*A)/(E-D*B) ; A2 = (A*G-D*C)/(E-D*B) ; A3 = (-B*F)/(E-D*B);
A4 = (-B*(A*G-D*C)-C*(E-D*B))/(A*(E-D*B)) ; A5 = (k*l)/(2*Rho);
