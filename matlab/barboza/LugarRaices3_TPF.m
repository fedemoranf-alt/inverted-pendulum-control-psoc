%Método Lugar de Raices

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
plantaD = c2d(planta,Ts);


%Diseñamos el compensador
numC = [1630.6 -2953.01 1345.73];
denC = conv([1 -0.8+0.12i],[1 -0.8-0.12i]);
compensador = tf(numC,denC,Ts);

sistema1 = compensador*plantaD;
sistema1_loop = feedback(sistema1,1);
figure(1)
step(sistema1_loop);
%Se genera la señal de error
t = 0:Ts:10000*Ts;
ref = ones(size(t))';
[y, t] = step(sistema1_loop,t);
error = ref - y;
figure(2)
%Se obtiene el esfuerzo de control
u = lsim(sistema1_loop,error,t);
title('Esfuerzo de control');
stem(t,u,'r');
stairs(t,u);

% simulacion del control digital del sistema
N = 10000;
%El tiempo de simulacion
td = [0:Ts:(N-1)*Ts];
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
refd = 0.01.*ones(size(td));
%refd = zeros(size(td));
%condicion inicial
%yd(50) = 0.1;

%emulacion del sistema
for k=4 : max(size(td)) - 1
    
    ed(k) = refd(k) - yd(k);
        
    ud(k) =  1631*ed(k) - 2953*ed(k-1) + 1346*ed(k-2) + 1.6*ud(k-1) - 0.65*ud(k-2);
    
    yd(k+1) = - b1*yd(k) - b0*yd(k-1) + a1*ud(k) + a0*ud(k-1);
end

figure(3)
plot(td,yd,'r');
title('Respuesta de la planta compensada')
xlabel('Time','FontSize',8);
ylabel('Amplitude','FontSize',8)

figure(4)
plot(td,ud,'r');
title('Esfuerzo de control')
xlabel('Time','FontSize',8);
ylabel('Amplitude','FontSize',8)