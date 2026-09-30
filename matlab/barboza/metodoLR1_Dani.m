close all
opt = stepDataOptions('InputOffset',0,'StepAmplitude',5);

m = 0.0923; %Masa del péndulo
M = 0.2275; %Masa del carro
g = 9.8; %Aceleración de la gravedad
l = 0.1825; %Longitud hasta el centro de masa del péndulo
I = 0.004212; %Inercia del pendulo

%Planta
numP = [1/(M*l)];
denP = [1 0 -(m+M)*g/(M*l)];
planta = tf(numP,denP);
cloop1 = feedback(planta,1); %Cerramos el lazo
%Respuesta al step de la planta en lazo abierto
%figure(1)
%step(planta);
%Respuesta al step de la planta en lazo cerrado
%figure(2)
%step(cloop1);

%Discretizamos la planta
Ts = 10e-3; %Tiempo de muestreo
plantaD = c2d(planta,Ts);

%Se agrega un integrador
[P,Z] = pzmap(plantaD);
numI = [1 -P(1,1)];
denI = [1 -1];
integrador = tf(numI,denI,Ts);
%Definimos el nuevo sistema con integrador
sistema1 = plantaD*integrador;
figure(3)
step(sistema1);
figure(4)
rlocus(sistema1);

%Se grafican las trazas en la region de interes, el circulo
zita_c = [0.3:0.2:0.7]; Wn_c = [0:pi/10:pi];
zgrid(zita_c,Wn_c)
hold on;
title('Grafica de la región aceptable en el plano Z')

zita = 0.7; %factor de amortiguamiento
Ws = (2*pi)/Ts; %frecuencia de muestreo en radianes
Wd = Ws/8;
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

%Suma de contribuciones angulares en p
%Se obtienen los polos y zeros de la planta
[P,Z] = pzmap(sistema1);
%ang_c1 = atan((yp -imag(Z(1,1)))/(xp - real(Z(1,1))));
ang_c2 = atan((yp - imag(Z(2,1)))/(xp - real(Z(2,1))));
%ang_p_1 = pi - atan((yp -imag(P(1,1)))/(-xp + real(P(1,1))));
ang_p_2 = pi - atan((yp - imag(P(2,1)))/(-xp + real(P(2,1))));
ang_p_3 = atan((yp - imag(P(3,1)))/(xp - real(P(3,1))));
angulo = ang_c2 - ang_p_2 - ang_p_3;

%La deficiencia angular es:
delta = -(angulo + pi);

%La función del controlador es proporcionar "delta" grados de adelanto
%Suponiendo un controlador de primer orden GD(Z) = K*(Z+a)/(Z+b)
%y suprimiendo el polo_2 de G(Z). 
a = P(3,1);
%Resolviendo el triangulo formado por el cero - polo del compensador y el punto p
l = sqrt((a - xp)^2 + yp^2);
alfa = asin(yp/l);
beta = pi - (delta + alfa);
Cero_Polo = l*sin(delta)/sin(beta);
b = P(3,1) - Cero_Polo;

%Con lo cual, la funcion de transferencia pulso en lazo abierto es: GD(Z)*G(Z)
%Para el calculo de K se puede utilizar la funcion rlocfind
numC = [1 -a];
denC = [1 -b];
compensador = tf(numC,denC,Ts);

%Definimos el nuevo sistema
sistema2 = compensador*integrador*plantaD;
figure(5)
rlocus(sistema2);
hold on;
axis([-3 1.2 -2 2])

%Se grafican las trazas en la region de interes y el punto P
plot(xp,yp,'r*');
zita_c= [0.3:0.2:0.7]; Wn_c = [0:pi/10:pi];
zgrid(zita_c,Wn_c)
K = rlocfind(sistema2);
hold off;
%La funcion de transferencia en lazo cerrado
sistema2_loop = feedback(K*sistema2,1);
figure(6);
step(sistema2_loop);

%Se genera la senhal de error
t = 0:Ts:5;
ref = 5*ones(size(t))';
[y t] = step(sistema2_loop,t,opt);
error = ref-y;
figure(7)
u = lsim(K*compensador*integrador,error,t,opt);
title('Esfuerzo de control');
stairs(t,u,'r');
pause
hold on
stairs(t,u);