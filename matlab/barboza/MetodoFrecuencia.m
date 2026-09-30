%Método Diseño en frecuencia

close all

%m = 0.0923; %Masa del péndulo
m = 0.1493; %Masa pelota de tenis
%m = 0.4923; %Masa plomada
M = 0.2275; %Masa del carro
g = 9.8; %Aceleración de la gravedad
l = 0.1825; %Longitud hasta el centro de masa del péndulo

%Montamos la planta continua
numP = 1/(M*l);
denP = [1 0 -(m+M)*g/(M*l)];
planta = tf(numP,denP);

%Discretizamos la planta
Ts = 1e-3;
plantaZ = c2d(planta,Ts);
[numZ, denZ] = tfdata(plantaZ,'v');

%Se requiere pasar del dominio Z al plano W
plantaW = d2c(plantaZ,'tustin');
[numW, denW] = tfdata(plantaW,'v');

%Se comparan los diagramas de Bode en el plano continuo y en W
w = logspace(-2,3,1000);
[Amp_S, Fase_S, w] = bode(numP,denP,w);
Amp_SdB=20*log10(Amp_S);
[Amp_W, Fase_W, w] = bode(numW,denW,w);
Amp_WdB=20*log10(Amp_W);
%Se grafican las curvas de magnitud
figure(1);
subplot(211);
semilogx(w,Amp_SdB,'r',w,Amp_WdB,'g');
hold on;
grid on;
title('Diagramas de Bode de G(s) y G(W)');
xlabel('Frecuencia (rad/s)');
ylabel('Ganancia dB')
text(300,-60, 'G(W)');
text(300,-90, 'G(S)');
%Se grafican las curvas de fase
subplot(212);
semilogx(w,Fase_S,'r',w,Fase_W - 360,'g');
hold on;
grid on;
xlabel('Frecuencia (rad/s)');
ylabel('Fase grados')
text(10,-170, 'G(S)');
text(10,-245, 'G(W)');

%Para obtener el margen de ganancia y fase pueden utilizarse la función margin
figure(2);
margin(plantaW);

K = 80;

%El angulo de adelanto de fase requerido es de 27 grados. La inclusión de un compensador
%modifica la curva de magnitud y que la frecuencia de cruce se desplaza a la derecha.
%sen Phi = (1-a)/(1+a) --> a = (1 - sen Phi)/(1 + sen Phi);
ang = 40;
a = (1 - sin(ang*pi/180))/(1+ sin(ang*pi/180));

%Se calcula la frecuencia donde la magnitud del sistema sin compensar = -20log(1/sqrt(a))
ganancia = -20*log10(1/sqrt(a));

%Se hace un zoom del bode en la zona de interés (1 < w < 10 rad/seg)
w = logspace(0,10,100);
[Amp_W, Fase_W]= bode(K*numW,denW,w);
Amp_WdB=20*log10(Amp_W);
figure(3);
punto = ganancia.*(ones(size(w)));
subplot(211);
semilogx(w,Amp_WdB);
grid on;
hold on;
semilogx(w,punto,'r');
hold off;
subplot(212);
semilogx(w,Fase_W);
grid on;
title('Diagrama de Bode de la planta G(W)');
disp('Seleccione el punto de frecuencia que corresponde a ganancia')

[frec, valor] = ginput(1);

%Se selecciona esta frecuencia como la nueva frecuencia de cruce Wc
%Se calcula el valor de r
r = 1/(frec*sqrt(a));

%Se determina el compensador
numC = [r^2 r  1];
denC = [(a*r)^2 a*r 1];
compensadorW = tf(numC,denC);
sistema1 = K*compensadorW*plantaW;
figure(4)
margin(sistema1);

%Se transforma el compensador al plano discreto
compensadorZ = c2d(compensadorW,Ts,'tustin');

%Se transforma al plano discreto Z
sistema1Z = c2d(sistema1,Ts,'tustin');

%Se cierra el lazo y se observa el comportamiento cuando se le aplica un
%escalon
sistema_cloopZ = feedback(sistema1Z,1);
figure(5)
step(sistema_cloopZ);

%Se genera la señal de error
t = 0:Ts:10000*Ts;
ref = ones(size(t))';
[y, t] = step(sistema_cloopZ,t);
error = ref - y;
figure(6)
%Se obtiene el esfuerzo de control
u = lsim(sistema_cloopZ,error,t);
title('Esfuerzo de control');
stem(t,u,'r');
stairs(t,u);
% simulacion del control digital del sistema
N = 1000;
%El tiempo de simulacion
td = [0:Ts:(N-1)*Ts];
%Se obtienen los polinomios del numerador y denominador de la función de
%transferencia de la planta discretizada 
[numD, denD]= tfdata(plantaZ,'v');
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
refd = 0.01.*ones(size(td));
%refd = zeros(size(td));
%condicion inicial
%yd(1) = 0.1;

%emulacion del sistema 
for k=3 : max(size(td)) - 1
    
    ed(k) = refd(k) - yd(k);
        
    ud(k) =  1609*ed(k) - 3174*ed(k-1) + 1566*ed(k-2) + 1.869*ud(k-1) - 0.88*ud(k-2);
    
    yd(k+1) = - b1*yd(k) - b0*yd(k-1) + a1*ud(k) + a0*ud(k-1);
end

figure(7)
plot(td,yd,'r');
title('Respuesta de la planta compensada')
xlabel('Time','FontSize',8);
ylabel('Amplitude','FontSize',8)

figure(8)
plot(td,ud,'r');
title('Esfuerzo de control')
xlabel('Time','FontSize',8);
ylabel('Amplitude','FontSize',8)
