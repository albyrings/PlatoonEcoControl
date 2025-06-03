% Codice MATLAB per il tuning di un controllore PID per G(s) = 1/s con il criterio di Bode

% Definizione dei parametri di progetto
w_c = 0.1;          % Frequenza di attraversamento desiderata (rad/s)
PM_desired = 60;   % Margine di fase desiderato (gradi)

% 1. Definire il sistema integratore
s = tf('s');
G = 1/s;

% 2. Analisi del sistema in anello aperto non compensato
figure(1);
bode(G);
grid on;
title('Diagramma di Bode del sistema G(s) = 1/s non compensato');

% 3. Progettazione del controllore utilizzando il criterio di Bode
% Per un integratore, la fase è sempre -90°
% Per ottenere un margine di fase di 60°, abbiamo bisogno di una fase totale di -120°
% Quindi il controllore deve contribuire con +30° di fase alla frequenza di attraversamento

% 3.1 Progettazione di un compensatore a ritardo di fase (lead compensator)
alpha = 3;  % Il rapporto tra zero e polo (alpha > 1 per lead compensator)

% Calcolo di T in modo che il massimo contributo di fase avvenga alla frequenza di attraversamento
T = 1/(w_c * sqrt(alpha));

% Calcolo del guadagno K per avere |C(jw_c)G(jw_c)| = 1
mag_lead_at_wc = abs((1 + 1i*w_c*alpha*T)/(1 + 1i*w_c*T));
K = 1/(mag_lead_at_wc * (1/w_c));

% 3.2 Definizione del controllore lead come PID filtrato
C_lead = K * (1 + alpha*T*s)/(1 + T*s);

% 3.3 Conversione al formato PID standard (con filtro sulla parte derivativa)
[num, den] = tfdata(C_lead, 'v');
K_p = num(2)/den(2);
K_d = (num(1)/den(2)) - (K_p*den(1)/den(2));
T_f = den(1)/den(2);  % Costante di tempo del filtro

C_pid = K_p + K_d*s/(1 + T_f*s);

% 4. Analisi del sistema in anello aperto compensato
L = C_pid * G;  % Funzione di trasferimento d'anello

figure(2);
margin(L);
grid on;
title('Diagramma di Bode del sistema compensato L(s) = C(s)G(s)');

% 5. Verifica dei parametri ottenuti
[Gm, Pm, Wcg, Wcp] = margin(L);
fprintf('Parametri del controllore PID:\n');
fprintf('K_p = %.4f\n', K_p);
fprintf('K_i = 0 (sistema già con integratore)\n');
fprintf('K_d = %.4f\n', K_d);
fprintf('T_f = %.4f (costante di tempo del filtro)\n\n', T_f);

fprintf('Performance del sistema in anello chiuso:\n');
fprintf('Frequenza di attraversamento: %.4f rad/s\n', Wcp);
fprintf('Margine di fase: %.2f gradi\n', Pm);

% 6. Risposta al gradino del sistema in anello chiuso
T_cl = feedback(L, 1);  % Sistema in anello chiuso

figure(3);
step(T_cl);
grid on;
title('Risposta al gradino del sistema in anello chiuso');

% 7. Risposta al disturbo
% Simuliamo l'effetto di un disturbo di carico all'uscita
S = 1/(1 + L);  % Funzione di sensitività (risposta ai disturbi)

figure(4);
step(S);
grid on;
title('Risposta del sistema a un disturbo a gradino');

% 8. Conversione alla forma PID standard per l'implementazione
fprintf('\nForma standard del controllore PID per implementazione:\n');
fprintf('C(s) = %.4f * (1 + %.4f*s/(1 + %.4f*s))\n', K_p, K_d/K_p, T_f);
fprintf('oppure\n');
fprintf('C(s) = %.4f + %.4f*s/(1 + %.4f*s)\n', K_p, K_d, T_f);

% 9. Confronto con altri controllori (opzionale)
% PID senza filtro
C_pid_no_filter = K_p + K_d*s;
L_no_filter = C_pid_no_filter * G;

% Solo controllore proporzionale
C_p = K_p;
L_p = C_p * G;

figure(5);
bode(L, 'b', L_no_filter, 'r', L_p, 'g');
grid on;
legend('PID filtrato', 'PID senza filtro', 'Solo P');
title('Confronto tra diverse strategie di controllo');