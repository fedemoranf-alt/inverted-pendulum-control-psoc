%% Graficas del psoc para controles modernos

%se debe de enviar desde el psoc como printf("%f,%f,%f,\r\n", u, T_rad, X_met);
% copiar y pegar en matlab los datos del putty en un vector con nombre
% datos_serial

esfuerzo = datos(:,1);
pendulo = (180/pi).*datos(:,2);

Ts = 0.1;
tfinal = 2000;
td = [0:Ts:(tfinal-1)*Ts];

figure
subplot(2,1,1)
plot(td,pendulo)
title('Posición del Péndulo')
ylabel('Grados')
xlabel('Tiempo (seg)')
grid('on')
subplot(2,1,2)
plot(td,esfuerzo)
title('Esfuerzo de Control')
ylabel('Voltaje')
xlabel('Tiempo (seg)')
grid('on')

%% save

save('C:\Users\moran\Desktop\Labs_Auto\TPF_pendulo\Informe\datos_bode.mat','td','esfuerzo','pendulo')

%% load
% grafico con la envolvente derivada
% aux = load('C:\Users\moran\Desktop\Ts_psoc\matlab\derivada2.mat')
% 
% figure
% plot(aux.tiempo, aux.voltage, aux.tiempo, aux.voltagef)
% title('Eco y Envolvente Filtrada')




