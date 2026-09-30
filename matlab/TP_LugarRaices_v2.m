%% TPF pendulo invertido - Control por compensacion con le diseño de lugar de raices

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
%definicion de fer
q = ((M+m)+Jm*((n^2)/(r^2)))*(I+m*l^2)-(m*l)^2; % variable auxiliar

F = [0 1 0 0;
     0 ((-(I+m*l^2)*(bm+((n^2)/(r^2))*(((Kt^2)/(Ra))+bm)))/q) ((m^2)*g*(l^2))/q 0;
     0 0 0 1;
     0 (-m*l*(bm+((n^2)/(r^2))*(((Kt^2)/(Ra))+bm)))/q (m*g*l*((M+m)+Jm*(n^2)/(r^2)))/q 0];
 
G = [0 ; ((I+m*l^2)*(Kt*n)/(Ra*r))/q ; 0 ; (m*l*(Kt*n)/(Ra*r))/q];

H = [1 0 0 0 ;
    0 0 1 0];
I = [0;0];

planta_ss = ss(F,G,H,I);
[num,den] = ss2tf(F,G,H,I);
planta = tf(num(1,:),den);

% %Planta definicion de barcelona
% numP = [1/(M*l)];
% denP = [1 0 -(m+M)*g/(M*l)];
% planta = tf(numP,denP);
% t = 0:0.1:15;

%discretizacion de la planta
Ts = 0.01;
plantaD = c2d(planta,Ts);
figure
pzmap(plantaD)
title("RL plantaD")

%% Se agrega el compensador al sistema que ya tiene un integrador 

%para realizar estos calculos primero hallamos los valores de los
%parametros de respuesta en frecuencia
Fs = 1/Ts;
Ws = Fs*(2*pi);
N = 18; % es la cantidad de muestras tomadas en el tiempo de establecimiento
Wd = Ws/N; 
zita = 0.1;
Wn = Wd/sqrt(1 - zita^2); % para sacar el tiempo de establecimiento es tr=1.8/Wn

%Se grafican las trazas en la region de interes, el circulo
zita_c = [0.3:0.2:0.7]; Wn_c = [0:pi/10:pi];
zgrid(zita_c,Wn_c)
hold on;
title(' Grafica de la región aceptable en el plano Z')
%Se calcula el circulo correspondiente a ts = 2 seg y se lo grafica, radio <=exp(-2Ts)
radio = exp(-zita*Wn*Ts);
t = [0:0.01:2*pi];
x = radio*cos(t); y=radio*sin(t);
plot(x,y,'r');
axis([-3 1.2 -2 2])

%Se grafica el punto P
%Se localizan los polos dominantes en lazo cerrado en el plano Z
%Para un lugar geometrico de los factores de amortiguamiento relativo constante
mod_Z = exp(-(2*pi*zita/sqrt(1-zita^2))*Wd/Ws);  
fase_Z = Ts*Wd;
%En coordenadas cartesianas 
xp = mod_Z*cos(fase_Z);
yp = mod_Z*sin(fase_Z);
%Se grafica el punto P
plot(xp,yp,'r*');
hold off;

%Suma de contribuciones angulares en p
%Se obtienen los polos y zeros de la planta
[P,Z] = pzmap(plantaD);
ang_c1 = atan((yp -imag(Z(1,1)))/(xp - real(Z(1,1)))); 
ang_c2 = pi - atan((yp -imag(Z(2,1)))/(-xp + real(Z(2,1))));
ang_c3 = pi - atan((yp -imag(Z(3,1)))/(-xp + real(Z(3,1))));
ang_p_1 = pi - atan((yp -imag(P(1,1)))/(-xp + real(P(1,1))));
ang_p_2 = pi - atan((yp-imag(P(2,1)))/(-xp +real(P(2,1))));
ang_p_3 = pi - atan((yp-imag(P(3,1)))/(-xp + real(P(3,1))));
ang_p_4 = pi - atan((yp-imag(P(4,1)))/(-xp + real(P(4,1))));
angulo = ang_c1 + ang_c2 + ang_c3 - ang_p_1 - ang_p_2 - ang_p_3 - ang_p_4;

%La deficiencia angular es:
delta = -(angulo + pi);
%La función del controlador es proporcionar "delta" grados de adelanto
%Suponiendo un controlador de primer orden GD(Z) = K*(Z+a)/(Z+b)
%y suprimiendo el polo_5 de G(Z). 
a = P(4,1);
%Resolviendo el triangulo formado por el cero - polo del compensador y el punto p
l = sqrt((a - xp)^2 + yp^2);
alfa = asin(yp/l);
beta = pi - (delta + alfa);
Cero_Polo = l*sin(delta)/sin(beta);
b = P(4,1) - Cero_Polo;

%Con lo cual, la funcion de transferencia pulso en lazo abierto es: GD(Z)*G(Z)
%Para el calculo de K se puede utilizar la funcion rlocfind
numC = [1 -a];
denC = [1 -b];
Compensador = tf(numC,denC,Ts);
sistema_comp = Compensador*plantaD;

%ahora hallamos el valor de la gancnai K del sistema en el punto P deseado
figure
rlocus(sistema_comp);
title("Sistema Compensado")
hold on;
axis([-3 1.2 -2 2])
%Se grafican las trazas en la region de interes y el punto P
plot(xp,yp,'r*');
zita_c= [0.3:0.2:0.7]; Wn_c = [0:pi/9:pi];
zgrid(zita_c,Wn_c)
K = rlocfind(sistema_comp);
hold off;
%La funcion de transferencia en lazo cerrado
cloop2 = feedback(K*sistema_comp,1);
figure
step(cloop2);
title("Sistema Compensado con nueva ganancia K")

% simulacion del control digital del sistema
tfinal = 0.2;
%El tiempo de simulacion
td = [0:Ts:tfinal];


%Se obtienen los polinomios del numerador y denominador de la función de
%transferencia de la planta discretizada 
[numD, denD]= tfdata(plantaD,'v');
%Se obtienen los coeficientes del numerador y del denominador de la planta
%discretizada
a4 = numD(1);
a3 = numD(2);
a2 = numD(3);   %Coeficiente de z^2 del numerador
a1 = numD(4);   %Coeficiente de z del numerador
a0 = numD(5);   %Coeficiente independiente del numerador
b4 = denD(1);   
b3 = denD(2);
b2 = denD(3);   %Coeficiente de z^2 del denominador
b1 = denD(4);   %Coeficiente de z del denominador
b0 = denD(5);   %Coeficiente independiente del denominador

%Se inicializan los vectores
yd = zeros(size(td));
ed = zeros(size(td));
ud = zeros(size(td));
refd = 0.1.*ones(size(td));

%emulacion del sistema 
for k=4 : max(size(td)) - 1
    
    ed(k) = refd(k) - yd(k);
        
    ud(k) =  1050*ed(k) - 906.6*ed(k-1) + 0.7432*ud(k-1);
    
    yd(k+1) = -b3*yd(k) - b2*yd(k-1) - b1*yd(k-2) - b0*yd(k-3) + a3*ud(k) + a2*ud(k-1) + a1*ud(k-2) + a0*ud(k-3);
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