num = 1; 
den = [1 0 0]; 

sys = tf(num, den);

% Parametri del controllore PD
Kp = 7; % Guadagno proporzionale
Kd = 0.1; % Guadagno derivativo

% Crea la funzione di trasferimento del controllore PD
controller = tf([Kd Kp], [1]);

% Crea la funzione di trasferimento del sistema a ciclo chiuso
sys_cl = feedback(controller * sys, 1);

% Tempo di simulazione
t = 0:0.001:500;

% Calcola la risposta allo scalino del sistema a ciclo chiuso
y = step(sys_cl, t);

% Grafico della risposta allo scalino
plot(t, y);
title('Risposta allo scalino di un doppio integratore con controllore PD');
xlabel('Tempo (s)');
ylabel('Uscita');
grid on;