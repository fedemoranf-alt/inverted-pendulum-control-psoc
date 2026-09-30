%% TPF pendulo invertido - Control Optimo con integrador


clear all
close all

% Parametros del sistema
Kt = 0.17776; % cte de torque del motor
Jm = 2.23*10^-4; % Inercia del motor
bm = 0.0011751; % firccion viscosa del motor
b = 0.029; %friccion viscosa del pendulo
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
Ts = 0.01;
plantaD = c2d(planta,Ts);
%Discretizamos las matrices de estado --> x(k+1) = Ax(k) + Bu(k)
%                                     --> y(k) = Cx(k) + Du(k)
%Hallamos A,B,C,D
A = plantaD.a;
B = plantaD.b;
C = plantaD.c;
D = plantaD.d;


%% SE AGREGA EL INTEGRADOR y se definen los polos del sistema con el control optimo
%Se tiene que definir el nuevo sistema (Ahat, Bhat, Chat, Dhat)
n = max(size(A)); %El orden del sistema
m = 1;            %La cantidad de salidas  que nos interesa
Ahat = [A B;zeros(m,n+m)];
Bhat = [zeros(n,m); eye(m)];
Chat = [C zeros(2,m)];

%Definimos las matrices de pesos para el sistema con INTEGRADOR
Q1 = [1300 0 0 0 0; 
      0 0 0 0 0;
      0 0 30 0 0;
      0 0 0 0 0;
      0 0 0 0 0.001];
Q2 = 1;

%Con integrador
[Khat,S,E] = dlqr(Ahat,Bhat,Q1,Q2);

Aux = [A-eye(size(A)) B; C(1,:)*A C(1,:)*B];
K2K1 = (Khat + [zeros(1,max(size(A))) eye(m)])*inv(Aux);
K2 = K2K1(1,1:n);
K1 = K2K1(1,n+1:n+m); 

%% simulacion del sistema compensado
N =1000;
t = [0:Ts:(N-1)*Ts];

x = zeros(4,N);
%yi = zeros(2,N);
xea = zeros(4,N);
zea = zeros(4,N);
u = zeros(1,N);
ref = 0.1*ones(1,length(t));%referencia no sobreescribir

%para el integrador
V = zeros(m,length(t));
V(m,1) = 0;

Cx=[1 0 0 0];
% Kr=dcgain(ss(A-B*K,B,Cx,0,Ts));
% Nbar = 1/Kr;

for k=2:N-1
    %Se calcula la salida actual
    yi(m,k) = Cx*x(:,k);
    %Ecuacionamiento del integrador 
    V(m,k) = V(m,k-1) + ref(m,k) - yi(m,k);
    
    %se calcula el esfuerzo de control con el estado estimado y el
    %integrador
    u(m,k) = K1*V(m,k)- K2*x(:,k);
    
    %se hace evolucionar la planta
    x(:,k+1) = A*x(:,k) + B*u(k);

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

% %Definimos la matriz A con el lazo cerrado
% Ac = A-B*K;
% %Definimos la planta en el espacio de estados con LC
% planta_SS_CL = ss(Ac,B,C,D,Ts);
% 
% figure
% pzmap(planta_SS_CL)
% zgrid
% title("Planta Compensada")
