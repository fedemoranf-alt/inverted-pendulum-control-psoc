%% TPF pendulo invertido - Control por compensacion con le diseño de lugar de raices

clear all
close all

% Parametros del sistema

%m = 0.0923; % masa del pendulo
m = 0.1493; % masa del pendulo + pelota de tenis
%m = 0.4923; % masa del pendulo mas la plomada de 400g
M = 0.2275; % masa del carro
g = 9.8; % aceleracion de la gravedad
l = 0.1825; % longitud hasta el centro de masa del pendulo
%l = 0.2425; %longitud del penduo mas la plomada


% matrices que definen el sistema de la planta
%
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

%Planta definicion 
numP = [1/(M*l)];
denP = [1 0 -(m+M)*g/(M*l)];
planta = tf(numP,denP);
t = 0:0.1:15;

%discretizacion de la planta
Ts = 0.01;
plantaD = c2d(planta,Ts);
figure
pzmap(plantaD)
title("RL plantaD")

%% Se disenhal el compensador

%controlSystemDesigner(plantaD)
%   3166.7 (z^2 - 0.8328z + 0.2775)
%   -------------------------------
%             (z+0.4167)
 



%% simulacion del control digital del sistema
tfinal = 0.2;
%El tiempo de simulacion
td = [0:Ts:tfinal];


%Se obtienen los polinomios del numerador y denominador de la función de
%transferencia de la planta discretizada 
[numD, denD]= tfdata(plantaD,'v');
%Se obtienen los coeficientes del numerador y del denominador de la planta
%discretizada

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
        
    %ud(k) =  1050*ed(k) - 906.6*ed(k-1) + 0.7432*ud(k-1);
    
    ud(k) =   222.64 *(ed(k) - 1.082*ed(k-1) + 0.4304*ed(k-2)) - (1.0820)*ud(k-1) - (0.1779)*ud(k-2);
    
    yd(k+1) =  - b1*yd(k) - b0*yd(k-1) + a1*ud(k) + a0*ud(k-1);
end

figure 
plot(td,yd,'r');
title('Respuesta al escalón de la planta compensada')
xlabel('Time (ms)','FontSize',8);
ylabel('Amplitude','FontSize',8)

figure;
plot(td,ud,'r');
title('Esfuerzo de control')
xlabel('Time (ms)','FontSize',8);
ylabel('Amplitude','FontSize',8)

%%
cloop = feedback(C3*plantaD,1);
tfinal = 0.2;
%El tiempo de simulacion
td = [0:Ts:tfinal];
U =  0.1.*ones(size(td));
[y T] = lsim(cloop,U);
figure
plot(T,y);
title('Respuesta a un step')
