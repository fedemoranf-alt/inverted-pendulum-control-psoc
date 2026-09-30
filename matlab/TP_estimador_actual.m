%% TPF pendulo invertido - Control con estimador actual

clear all
close all

% Parametros del sistema
Kt = 0.17776; % cte de torque del motor
Jm = 2.23*10^-4; % Inercia del motor
bm = 0.0011751; % firccion viscosa del motor
b = 0.029; %friccion viscosa del pendulo
Ra = 9.10; % resistencia de armadura del motor
%m = 0.0923; % masa del pendulo
%m = 0.1493; % masa del pendulo + pelota de tenis
m = 0.4923; % masa del pendulo mas la plomada de 400g
M = 0.2275; % masa del carro
g = 9.8; % aceleracion de la gravedad
%l = 0.1825; % longitud hasta el centro de masa del pendulo
l = 0.2425; %longitud del penduo mas la plomada
r = 0.0181; % radio de la polea
I = 0.004212; % inercia del pendulo
n = 1; % relacion de reduccion del motor
La = 6.12*10^-3; % Inductancia de armadura del motor

% matrices que definen el sistema de la planta

%definicion del sistema
q = ((M+m)+Jm*((n^2)/(r^2)))*(I+m*l^2)-(m*l)^2; % variable auxiliar

F = [0 1 0 0;
     0 ((-(I+m*l^2)*(b+((n^2)/(r^2))*(((Kt^2)/(Ra))+bm)))/q) ((m^2)*g*(l^2))/q 0;
     0 0 0 1;
     0 (-m*l*(b+((n^2)/(r^2))*(((Kt^2)/(Ra))+bm)))/q (m*g*l*((M+m)+Jm*(n^2)/(r^2)))/q 0];
 
G = [0 ; ((I+m*l^2)*(Kt*n)/(Ra*r))/q ; 0 ; (m*l*(Kt*n)/(Ra*r))/q];

H = [1 0 0 0; 
    0 0 1 0];
J = [0;0];

%Montamos la planta continua
planta = ss(F,G,H,J);

%Discretizamos la planta
Ts = 0.005;
plantaD = c2d(planta,Ts);
%Discretizamos las matrices de estado --> x(k+1) = Ax(k) + Bu(k)
%                                     --> y(k) = Cx(k) + Du(k)
%Hallamos A,B,C,D
A = plantaD.a;
B = plantaD.b;
C = plantaD.c;
D = plantaD.d;

%% Ubicamos los polos para el sistema

%polos = [0.7+1i*0.25 0.7-1i*0.25 0.7+1i*0.25 0.7-1i*0.25 ];
polos = [0.95+0.0025i  0.95-0.0025i  0.9+0.0025i  0.9-0.0025i];
K = acker(A,B,polos); %Matriz K de ganancias

%% Ubicar los polos para el estimador

polos_ea = [0.4+1i*0.4 0.4-1i*0.4 0.4+1i*0.4 0.4-1i*0.4 ];
Kea = place(A',(C*A)',polos_ea')';
%%
N =100;
t = [0:Ts:(N-1)*Ts];

x = zeros(4,N);
xea = zeros(4,N);
zea = zeros(4,N);
u = zeros(1,N);
ref = 0.1.*ones(1,length(t));%referencia no sobreescribir

Cx=[1 0 0 0];
Kr=dcgain(ss(A-B*K,B,Cx,0,Ts));
Nbar = 1/Kr;

for j=1:N-1
   
    %estimador de prediccion:
    %se calcula el esfuerzo de control con el estado estimado
    u(j) = Nbar*ref(j)- K*xea(:,j);
    
    %se hace evolucionar la planta
    x(:,j+1) = A*x(:,j) + B*u(j);
    
    %se hace evolucionar el estimador actual:
    zea(:,j+1) = A*xea(:,j) + B*u(j);
    xea(:,j+1) = zea(:,j+1) + Kea*(C*x(:,j+1) - C*zea(:,j+1));
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

%Definimos la matriz A con el lazo cerrado
Ac = A-B*K;
%Definimos la planta en el espacio de estados con LC
planta_SS_CL = ss(Ac,B,C,D,Ts);

figure
pzmap(planta_SS_CL)
zgrid
title("Planta Compensada")
