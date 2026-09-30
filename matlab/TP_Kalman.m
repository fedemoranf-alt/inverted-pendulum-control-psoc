%% TPF pendulo invertido - Control con KALMAN
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

%% SE AGREGA EL INTEGRADOR y se hallan los polos de la planta con ackerman
%Se tiene que definir el nuevo sistema (Ahat, Bhat, Chat, Dhat)
n = max(size(A)); %El orden del sistema
m = 1;            %La cantidad de salidas  que nos interesa
Ahat = [A B;zeros(m,n+m)];
Bhat = [zeros(n,m); eye(m)];
Chat = [C zeros(2,m)];

%polos_i = [0.95+0.0025i  0.95-0.0025i  0.9+0.0025i  0.9-0.0025i 0.4];
polos_i = [0.968+0.00025i  0.968-0.00025i  0.938+0.00025i  0.938-0.00025i 0.6];
Khat = acker(Ahat,Bhat,polos_i);

Aux = [A-eye(size(A)) B; C(1,:)*A C(1,:)*B];
K2K1 = (Khat + [zeros(1,max(size(A))) eye(m)])*inv(Aux);
K2 = K2K1(1,1:n);
K1 = K2K1(1,n+1:n+m); 

%% SE AGREGA EL INTEGRADOR y se calculan los polos con el control optimo 
% %Se tiene que definir el nuevo sistema (Ahat, Bhat, Chat, Dhat)
% n = max(size(A)); %El orden del sistema
% m = 1;            %La cantidad de salidas  que nos interesa
% Ahat = [A B;zeros(m,n+m)];
% Bhat = [zeros(n,m); eye(m)];
% Chat = [C zeros(2,m)];
% 
% %Definimos las matrices de pesos para el sistema con INTEGRADOR
% Q1 = [1 0 0 0 0; 
%       0 0 0 0 0;
%       0 0 0 0 0;
%       0 0 0 10 0;
%       0 0 0 0 0];
% Q2 = 0.04;
% 
% [Khat,S,E] = dlqr(Ahat,Bhat,Q1,Q2);
% 
% Aux = [A-eye(size(A)) B; C(1,:)*A C(1,:)*B];
% K2K1 = (Khat + [zeros(1,max(size(A))) eye(m)])*inv(Aux);
% K2 = K2K1(1,1:n)
% K1 = K2K1(1,n+1:n+m) 

%% Diseñamos el esquema de kalman con estimador actual
%Rv = (0.01)^2;
%Rw = (1)^2; 
Rv = 50;
Rw = 0.0001;
%con estos valores funciona
%Rvm = [0.002 0 ;0 0.001];         %Varianza del Nivel de ruido de medida
%Rwm = [100000 30 ;80 1000];      %Varianza del Nivel de ruido de proceso
% Kek =
%     0.5955    0.4046
%   -22.4424   22.4504
%    -0.6372    1.6388
%   -66.8425   67.1572

%error en el encoder del pendulo
ep = (360/2400)/2;
em = (360/512)/2;
%matrices de nivel de ruido en el sistema real
Rvm = [ep^2 0 ;0 em^2];         %Varianza del Nivel de ruido de medida
Rwm = [100000 300 ;80 10000];      %Varianza del Nivel de ruido de proceso




Rvi = inv(Rv);

B1 = [1;0;1;0];       %Matriz para el ruido de proceso
sist = ss(A,[B B1],C,[D D],Ts); 

%Matriz Hamiltoniana:
He = [(A'+C'*Rvi*C*inv(A)*B1*Rw*B1') -C'*Rvi*C*inv(A); -inv(A)*B1*Rw*B1' inv(A)];
[KEST,L,P] = kalman(sist,Rwm,Rvm);
poles = eig(He);
polos = [poles(1) poles(6) poles(7) poles(8)];  %Se utilizan los polos estables de la matriz Hamiltoniana

%Kek = place(A',(C*A)',polos)';
Kek = L
% polos_ea = [0.4+1i*0.4 0.4-1i*0.4 0.4+1i*0.4 0.4-1i*0.4 ];
% Kea = place(A',(C*A)',polos_ea')';

%% simulacion del sistema compensado
N =1000;
t = [0:Ts:(N-1)*Ts];

%variables de la planta
x = zeros(4,N);
%yi = zeros(2,N);

% variables del estimador
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

ruido_v=sqrt(Rv)*randn(N);  %medida
ruido_w=sqrt(Rw)*randn(N);  %

for k=2:N-1
    %Se calcula la salida actual
    yi(m,k) = Cx*x(:,k);
    %Ecuacionamiento del integrador 
    V(m,k) = V(m,k-1) + ref(m,k) - yi(m,k);
    
    %se calcula el esfuerzo de control con el estado estimado y el
    %integrador
    u(m,k) = K1*V(m,k)- K2*xea(:,k);
    
    %se hace evolucionar la planta
    %x(:,k+1) = A*x(:,k) + B*u(k);
    x(:,k+1) = A*x(:,k) + B*u(k) + B1*ruido_w(k);
    
    %se hace evolucionar el estimador actual:
    zea(:,k+1) = A*xea(:,k) + B*u(k);
    %xea(:,k+1) = zea(:,k+1) + Kek*(C*x(:,k+1) - C*zea(:,k+1));
    xea(:,k+1) = xea(:,k+1) + L*(C*x(:,k+1)+ ruido_v(k+1) - C*xea(:,k+1)) ;
end

figure
subplot(3,1,1)
plot(t,(180/pi).*x(1,:),'r');
title('Posición del péndulo')
xlabel('Tiempo (seg)','FontSize',8);
ylabel('Grados','FontSize',8)
subplot(3,1,2)
plot(t,x(3,:),'r');
title('Posición del carro')
xlabel('Tiempo (seg)','FontSize',8);
ylabel('Metros','FontSize',8)
subplot(3,1,3)
plot(t,u,'b')
title('Esfuerzo de control')
xlabel('Tiempo (seg)','FontSize',8);
ylabel('Voltaje','FontSize',8)

% %Definimos la matriz A con el lazo cerrado
% Ac = A-B*K;
% %Definimos la planta en el espacio de estados con LC
% planta_SS_CL = ss(Ac,B,C,D,Ts);
% 
% figure
% pzmap(planta_SS_CL)
% zgrid
% title("Planta Compensada")
