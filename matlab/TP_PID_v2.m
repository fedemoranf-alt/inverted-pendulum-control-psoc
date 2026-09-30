%% TPF pendulo invertido - Control PID

clear all
close all

%Parametros del sistema
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


% %Planta definicion de barcelona
numP = [1/(M*l)];
denP = [1 0 -(m+M)*g/(M*l)];
planta = tf(numP,denP);

%discretizacion de la planta
Ts = 0.001;
plantaD = c2d(planta,Ts);

[P,Z] = pzmap(plantaD);

%% Control PID 
gain = 733.32;
numPID = conv([1 -0.9947],[1 -0.9935]);
denPID = conv([1 -1],[1 -0.9404]);
compensadorPID = tf(gain*numPID,denPID,Ts)
%Simmulacion del control digital, con emulacion de la planta
tfinal = 10;

%El tiempo de simulacion
td = [0:Ts:tfinal];

%Se obtienen los polinomios del numerador y denominador de la función de
%transferencia de la planta discretizada 
[numD, denD]= tfdata(plantaD,'v');

a2 = numD(1);   %Coeficiente de z^2 del numerador
a1 = numD(2);   %Coeficiente de z del numerador
a0 = numD(3);   %Coeficiente independiente del numerador
b2 = denD(1);   %Coeficiente de z^2 del denominador
b1 = denD(2);   %Coeficiente de z del denominador
b0 = denD(3);   %Coeficiente independiente del denominador

%Se inicializan los vectores
yd = zeros(size(td));
ed = zeros(size(td));
ud = zeros(size(td));
refd = 0.1.*ones(size(td));



%emulacion del sistema 
for k=4 : max(size(td)) - 1
    
    ed(k) = refd(k) - yd(k);
      
    ud(k) = 26.81*ed(k) + 53.39*ed(k-1) + 26.58*ed(k-2) - 1.978*ud(k-1) - 0.9779*ud(k-2);

    yd(k+1) = - b1*yd(k) - b0*yd(k-1) + a1*ud(k) + a0*ud(k-1);
    
end 

figure
plot(td,yd,'r');
title('Respuesta de la planta discretizada con control PID')
xlabel('Time (ms)','FontSize',8);
ylabel('Amplitude','FontSize',8)

figure
plot(td,ud,'r');
title('Esfuerzo de control')
xlabel('Time (ms)','FontSize',8);
ylabel('Amplitude','FontSize',8)


