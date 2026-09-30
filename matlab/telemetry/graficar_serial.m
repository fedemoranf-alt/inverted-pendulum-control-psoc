%% Graficas del psoc para controles modernos

%se debe de enviar desde el psoc como printf("%f,%f,%f,\r\n", u, T_rad, X_met);
% copiar y pegar en matlab los datos del putty en un vector con nombre
% datos_serial

esfuerzo = datos(:,1);
pendulo = (180/pi).*datos(:,2);
carro = datos(:,3);

Ts = 0.01;
tfinal = 400;
td = [0:Ts:(tfinal-1)*Ts];

figure
subplot(3,1,1)
plot(td,pendulo)
title('Posición del Péndulo')
ylabel('Grados')
xlabel('Tiempo (seg)')
grid('on')
subplot(3,1,2)
plot(td,carro)
title('Posición del Carro')
ylabel('Metros')
xlabel('Tiempo (seg)')
grid('on')
subplot(3,1,3)
plot(td,esfuerzo)
title('Esfuerzo de Control')
ylabel('Voltaje')
xlabel('Tiempo (seg)')
grid('on')

%% save

save('C:\Users\moran\Desktop\Labs_Auto\TPF_pendulo\Informe\datos_kalman.mat','td','esfuerzo','pendulo','carro')

%% load
% grafico con la envolvente derivada
% aux = load('C:\Users\moran\Desktop\Ts_psoc\matlab\derivada2.mat')
% 
% figure
% plot(aux.tiempo, aux.voltage, aux.tiempo, aux.voltagef)
% title('Eco y Envolvente Filtrada')


%%

% PID		Y
% RL		
% BODE		
% UA POLOS	Y
% EP		
% EA		Y
% CO		
% KALMAN		
% 
% 
% printf("%f,%f,%f\r\n", u, T_rad, X_met);
