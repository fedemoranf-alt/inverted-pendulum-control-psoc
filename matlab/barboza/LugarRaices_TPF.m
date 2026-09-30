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
figure(1)
rlocus(sistema1);

%Se grafican las trazas en la region de interes, el circulo
zita_c = [0.3:0.2:0.7]; Wn_c = [0:pi/10:pi];
zgrid(zita_c,Wn_c)
hold on;
title('Grafica de la región aceptable en el plano Z')

zita = 0.7; %factor de amortiguamiento
Ws = (2*pi)/Ts; %frecuencia de muestreo en radianes
Wd = Ws/33;
%Wn = 260;
Wn = Wd/(sqrt(1 - zita^2)); %frecuencia natural de oscilacion del sistema

%Se calcula el circulo de interes
radio = exp(-zita*Wn*Ts);
t = 0:0.01:2*pi;

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

% %Suma de contribuciones angulares en p
% %Se obtienen los polos y zeros de la planta
% [P,Z] = pzmap(sistema1);
% ang_c1 = atan((yp -imag(Z(1,1)))/(xp - real(Z(1,1))));
% ang_c2 = pi - atan((yp -imag(Z(2,1)))/(-xp + real(Z(2,1))));
% ang_p_1 = pi - atan((yp -imag(P(1,1)))/(-xp + real(P(1,1))));
% ang_p_2 = pi - atan((yp - imag(P(2,1)))/(-xp + real(P(2,1))));
% ang_p_3 = pi - atan((yp - imag(P(3,1)))/(-xp + real(P(3,1))));
% angulo = ang_c1 + ang_c2 - ang_p_1 - ang_p_2 - ang_p_3;
% 
% %La deficiencia angular es:
% delta = -(angulo + pi);
% 
% %La función del controlador es proporcionar "delta" grados de adelanto
% %Suponiendo un controlador de primer orden GD(Z) = K*(Z+a)/(Z+b)
% %y suprimiendo el polo_3 de G(Z). 
% a = P(3,1);
% %Resolviendo el triangulo formado por el cero - polo del compensador y el punto p
% l = sqrt((a - xp)^2 + yp^2);
% alfa = asin(yp/l);
% beta = pi - (delta + alfa);
% Cero_Polo = l*sin(delta)/sin(beta);
% b = P(3,1) - Cero_Polo;
% 
% %Con lo cual, la funcion de transferencia pulso en lazo abierto es: GD(Z)*G(Z)
% %Para el calculo de K se puede utilizar la funcion rlocfind
% numC = [1 -a];
% denC = [1 -b];
% compensador = tf(numC,denC,Ts);
% 
% %Definimos el nuevo sistema
% sistema2 = integrador*compensador*plantaD;
% figure(2)
% rlocus(sistema2);

hold on;
axis([-3 1.2 -2 2])

%Se grafican las trazas en la region de interes y el punto P
plot(xp,yp,'r*');
zita_c= [0.3:0.2:0.7]; Wn_c = [0:pi/10:pi];
zgrid(zita_c,Wn_c)
K = rlocfind(sistema1);
hold off;
%La funcion de transferencia en lazo cerrado
sistema1_loop = feedback(K*sistema1,1);
figure(2)
step(sistema1_loop);

%Se genera la señal de error
t = 0:Ts:10000*Ts;
ref = ones(size(t))';
[y, t] = step(sistema1_loop,t);
error = ref - y;
figure(3)
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
%yd(1) = 0.1;

%emulacion del sistema 
for k=3 : max(size(td)) - 1
    
    ed(k) = refd(k) - yd(k);
        
    ud(k) =  968.7*ed(k) - 1754*ed(k-1) + 799.4*ed(k-2) + 1.6*ud(k-1) - 0.6544*ud(k-2);
    
    yd(k+1) = - b1*yd(k) - b0*yd(k-1) + a1*ud(k) + a0*ud(k-1);
end

figure(4)
plot(td,yd,'r');
title('Respuesta de la planta compensada')
xlabel('Time','FontSize',8);
ylabel('Amplitude','FontSize',8)

figure(5)
plot(td,ud,'r');
title('Esfuerzo de control')
xlabel('Time','FontSize',8);
ylabel('Amplitude','FontSize',8)