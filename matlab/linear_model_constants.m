mw = 0 ; %Masse d'une unite (roue + engrenage + rotor)
mb = 0.917 ; %Masse du chassis du robot
Ix = 0 ;Iy = 0 ;Iz = 0 ; %Moment d'inertie du chassis autour de chacun de ses axes, a C
Iwx = 0 ;Iwy = 0.00005 ;Iwz = 0 ;%Moment d'inertie de la roue autour de chacun de ses axes, a Cw
Rho = 0.032 ; %position verticale(rayon du roue)
l = 0.186 ; %Distance entre les deux roues
d = 0.035 ; %Distance entre A et C
di = 0 ; %Distance verticale entre l'IMU et l'axe des roues
L = 0 ; %Inductance des moteurs 
R = 2.2 ; %Resistance des moteurs
k = 0.38 ; %Constante de couple
g = 9.81; %constante de gravitation


M = mb+2*mw;
Ipsi = Ix+Iz+Iwx+Iwz+(mw*l*l)/2+(Iwy*l*l)/(2*Rho*Rho);
Itoty = Iy+2*Iwy;


A = M+(2*Iwy)/(Rho*Rho) ; B = mb*d; D = mb*d ; C = -k/Rho ;
E = Itoty+mb*d*d ; F = -mb*d; G = k;

A1 = (F*A)/(E-D*B) ; A2 = (A*G-D*C)/(E-D*B) ; A3 = (-B*F)/(E-D*B);
A4 = (-B*(A*G-D*C)-C*(E-D*B))/(A*(E-D*B)) ; A5 = (k*l)/(2*Rho);

B1 = A2/(A2*A3-A1*A4) ; B2 = -A4/(A2*A3-A1*A4);

AA = [0 , 1 , 0 , 0;
          0 , 0 , 1 , 0;
          0 , 0 , 0 , 1;
          0 , -(2*A2*k)/(R*Rho*B1) , A1 , A2*(2*k/R+B2/B1)]
    BB = [0 , 0;
          0 , 0;
          0 , 0;
          A2/R , A2/R]
    eig(AA)