%% TPF pendulo invertido - Control por ubicacion arbitraria de polos

clear all
close all

% Parametros del sistema
Kt = 0.17776; % cte de torque del motor
Jm = 2.23*10^-4; % Inercia del motor
bm = 0.0011751; % firccion viscosa del motor
b = 0.0011751; %friccion viscosa del pendulo
Ra = 9.10; % resistencia de armadura del motor
%m = 0.0923; % masa del pendulo
m = 0.1493; % masa del pendulo + pelota de tenis
%m = 0.4923; % masa del pendulo mas la plomada de 400g
M = 0.2275; % masa del carro
g = 9.8; % aceleracion de la gravedad
l = 0.1825; % longitud hasta el centro de masa del pendulo
%l = 0.2425; %longitud del penduo mas la plomada
r = 0.0181; % radio de la polea
I = 0.004212; % inercia del pendulo
n = 1; % relacion de reduccion del motor
La = 6.12*10^-3; % Inductancia de armadura del motor

% matrices que definen el sistema de la planta

%definicion de fer
q = ((M+m)+Jm*((n^2)/(r^2)))*(I+m*l^2)-(m*l)^2; % variable auxiliar

F = [0 1 0 0;
     0 ((-(I+m*l^2)*(b+((n^2)/(r^2))*(((Kt^2)/(Ra))+bm)))/q) ((m^2)*g*(l^2))/q 0;
     0 0 0 1;
     0 (-m*l*(b+((n^2)/(r^2))*(((Kt^2)/(Ra))+bm)))/q (m*g*l*((M+m)+Jm*(n^2)/(r^2)))/q 0];
 
G = [0 ; ((I+m*l^2)*(Kt*n)/(Ra*r))/q ; 0 ; (m*l*(Kt*n)/(Ra*r))/q];

H = [1 0 0 0 ;
    0 0 1 0];
J = [0;0];

% planta_ss = ss(F,G,H,I);
% [num,den] = ss2tf(F,G,H,I);
% planta = tf(num,den);

%Planta definicion de barcelona
%x1 : posicion angular del pendulo respecto a la normal 
%x2 : derivada de x1, velocidad angular del pendulo 
%x3 : posicion del carrito
%x4 : derivada de x3, velocidad del carrito 
% F = [0 1 0 0 ;
%     g*(M+m)/(M*l) 0 0 0;
%     0 0 0 1;
%     -g*m/M 0 0 0];
% G = [0; -1/(M*l); 0; 1/M];
% H = [1 0 0 0; 0 0 1 0];
% J = [0; 0];

%Montamos la planta continua
planta = ss(F,G,H,J);

%Discretizamos la planta
Ts = 0.01;
plantaD = c2d(planta,Ts);
%Discretizamos las matrices de estado --> x(k+1) = Ax(k) + Bu(k)
%                                     --> y(k) = Cx(k) + Du(k)
%Hallamos A,B,C,D
A = plantaD.a;
B = plantaD.b;
C = plantaD.c;
D = plantaD.d;

%Matriz de controlabilidad hallado con formulas --> [A^(n-2)*B A^(n-3)*B...B]
CT = [B A*B];
%Vemos si es controlable
CTR = ctrb(A,B);
rank(CTR);  %El sistema es controlable (FULL RANGE)

%Vemos si es observable
OB = [C;C*A];
OB1 = obsv(A,C);
rank(OB1);   %El sistema es observable (FULL RANGE)

figure
pzmap(plantaD)
title("Planta discreta")

%[P,Z] = pzmap(plantaD);

%Ubicacion del los polos 1 y 2
zita = 0.7; %Factor de amortiguamiento elegido
Ws = 2*pi/Ts;
Wd = Ws/18;
Wn = Wd/(sqrt(1-(zita)^2));
%ubicacion en coordenadas polares
Magnitud_Z = exp(-zita*Wn*Ts);
Fase_Z = Wd*Ts;
%Ubicacion del polo en coordenadas cartesianas
xp1 = Magnitud_Z*cos(Fase_Z);
yp1 = Magnitud_Z*sin(Fase_Z);

%Ubicacion del los polos 3 y 4
zita = 0.7; %Factor de amortiguamiento elegido
Ws = 2*pi/Ts;
Wd = Ws/30;
Wn = Wd/(sqrt(1-(zita)^2));
%ubicacion en coordenadas polares
Magnitud_Z = exp(-zita*Wn*Ts);
Fase_Z = Wd*Ts;
%Ubicacion del polo en coordenadas cartesianas
xp2 = Magnitud_Z*cos(Fase_Z);
yp2 = Magnitud_Z*sin(Fase_Z);

%Vector de polos
%polos = [0.98+0.00025i 0.98-0.00025i   xp1+1i*yp1 xp1-1i*yp1];
polos = [0.965+0.00025i  0.965-0.00025i  0.935+0.00025i  0.935-0.00025i];
%polos = [xp1+1i*yp1 xp1-1i*yp1 xp2+1i*yp2 xp2-1i*yp2  ];
%Hallamos la matriz K de ganancias
K = acker(A,B,polos); %Matriz K de ganancias

%Definimos la matriz A con el lazo cerrado
Ac = A-B*K;
%Definimos la planta en el espacio de estados con LC
planta_SS_CL = ss(Ac,B,C,D,Ts);
figure
step(planta_SS_CL);

N =1000;
t = [0:Ts:(N-1)*Ts];

x = zeros(4,N);
u = zeros(1,N);
%ref = ones(1,N);
ref = 0.1.*ones(1,length(t));%referencia no sobreescribir
%[Nx,Nu,Nbar] = refi(A,B,C,K);

Cx=[1 0 0 0];
Kr=dcgain(ss(A-B*K,B,Cx,0,Ts));
Nbar = 1/Kr

for j=1:N-1
    %se calcula el esfuerzo de control
    u(j) = Nbar*ref(j)-K*x(:,j);
    %se hace evolucionar al sistema
    x(:,j+1) = A*x(:,j) + B*u(j);
end

figure
plot(t,x(1,:))
title('Posicion pendulo');
figure
plot(t,x(3,:))
title('Posicion carrito');
figure
plot(t,u);
title('Esfuerzo de Control');

figure
pzmap(planta_SS_CL)
zgrid
title("Planta Compensada")




