close all;

Kt = 0.17776; %Constante de torque del motor
Jm = 2.23e-4; %Inercia del motor
bm = 0.0011751; %Friciion biscosa del motor
Ra = 9.10; %Resistencia de armadura
m = 0.0923; %Masa del péndulo
M = 0.2275; %Masa del carro
g = 9.8; %Aceleración de la gravedad
l = 0.1825; %Longitud hasta el centro de masa del péndulo
r = 0.0181; % Radio de la polea
I = 0.004212; %Inercia del pendulo
n = 1; %Relacion de reduccion del motor
La = 6.12e-3; %Inductancia de aramadura

numP = Kt;
denP = [La*Jm (Ra*Jm+bm*La) Kt+Ra*bm];
planta = tf(numP,denP);

Ts = 1e-3;
plantaD = c2d(planta,Ts);
cloop = feedback(plantaD,1)

[Pm Zm] = pzmap(plantaD);
