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

% % matrices que definen el sistema de la planta
% %definicion con el modelado del motor
% q = ((M+m)+Jm*((n^2)/(r^2)))*(I+m*l^2)-(m*l)^2; % variable auxiliar
% 
% F = [0 1 0 0;
%      0 ((-(I+m*l^2)*(bm+((n^2)/(r^2))*(((Kt^2)/(Ra))+bm)))/q) ((m^2)*g*(l^2))/q 0;
%      0 0 0 1;
%      0 (-m*l*(bm+((n^2)/(r^2))*(((Kt^2)/(Ra))+bm)))/q (m*g*l*((M+m)+Jm*(n^2)/(r^2)))/q 0];
%  
% G = [0 ; ((I+m*l^2)*(Kt*n)/(Ra*r))/q ; 0 ; (m*l*(Kt*n)/(Ra*r))/q];
% 
% H = [1 0 0 0 ;
%     0 0 1 0];
% I = [0;0];
% 
% planta_ss = ss(F,G,H,I);
% [num,den] = ss2tf(F,G,H,I);
% planta = tf(num(1,:),den);

% %Planta definicion de barcelona
numP = [1/(M*l)];
denP = [1 0 -(m+M)*g/(M*l)];
planta = tf(numP,denP);

%discretizacion de la planta
Ts = 0.001;
plantaD = c2d(planta,Ts);

%% Control PID 

%se definen los demas parametros del controlador PID
%modificamos los valores para cumplir los requerimientos deseados
Kp = 4.9; %0.5 // 0.3
Td = 0.04; %0.000735*3.5;
Ti = 3;  %0.00147; 
%se cambiaron los valores para que coincidan con la ecuacion en diferencia
%cuando se le pone un coeficiente  de 0.3 al termino proporcional del pid
numC = Kp*[Td*Ti Ti 1];
denC = [Ti 0];
controlador_PID = tf(numC,denC);
% cloop = feedback(controlador_PID*planta,1);
% 
% figure
% step(cloop)
% title("Planta continua con control PID")

%Simmulacion del control digital, con emulacion de la planta
tfinal = 1;
%t=0:0.001:tfinal;
%El tiempo de simulacion
td = [0:Ts:tfinal];

%Se obtienen los polinomios del numerador y denominador de la función de
%transferencia de la planta discretizada 
[numD, denD]= tfdata(plantaD,'v');
%Se obtienen los coeficientes del numerador y del denominador de la planta
%discretizada
% a4 = numD(1);
% a3 = numD(2);
% a2 = numD(3);   %Coeficiente de z^2 del numerador
% a1 = numD(4);   %Coeficiente de z del numerador
% a0 = numD(5);   %Coeficiente independiente del numerador
% b4 = denD(1);   
% b3 = denD(2);
% b2 = denD(3);   %Coeficiente de z^2 del denominador
% b1 = denD(4);   %Coeficiente de z del denominador
% b0 = denD(5);   %Coeficiente independiente del denominador


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
        
    %ud(k) = ud(k-1)+Kp*((1+(Ts/Ti)+(Td/Ts))*ed(k)-(1+2*(Td/Ts))*ed(k-1)+(Td/Ts)*ed(k-2));
   
    %yd(k+1) = -b3*yd(k) - b2*yd(k-1) - b1*yd(k-2) - b0*yd(k-3) + a3*ud(k) + a2*ud(k-1) + a1*ud(k-2) + a0*ud(k-3);
    
    yd(k+1) = - b1*yd(k-2) - b0*yd(k-3) + a1*ud(k-2) + a0*ud(k-3);
    
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


