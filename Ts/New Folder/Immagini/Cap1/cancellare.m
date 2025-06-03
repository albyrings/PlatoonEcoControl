
% Definizione dei parametri di progettazione
w_c = 20;         % Frequenza di attraversamento desiderata (rad/s)
PM_desired = 60;  % Margine di fase desiderato (gradi)

% Definizione del sistema integratore
s = tf('s');
G = 1/s;

% Calcolo dei parametri del controllore lead utilizzando il criterio di Bode
alpha = 3;        % Per ottenere circa 30° di contributo di fase
T = 1/(w_c * sqrt(alpha));  % T = 0.0289 per w_c = 20 rad/s

% Calcolo del guadagno K per avere |C(jw_c)G(jw_c)| = 1
mag_lead_at_wc = abs((1 + 1i*w_c*alpha*T)/(1 + 1i*w_c*T));
K = w_c / mag_lead_at_wc;  % K ≈ 11.54 per w_c = 20 rad/s

% Creazione del controllore lead
C = K * (1 + alpha*T*s)/(1 + T*s);
C_numeric = tf([K*alpha*T K], [T 1]);  % Forma numerica esplicita

% Funzione di trasferimento d'anello L(s) = C(s)G(s)
L = C * G;

% Stampa dei valori dei parametri
fprintf('Parametri del controllore lead:\n');
fprintf('K = %.2f\n', K);
fprintf('alpha = %.2f\n', alpha);
fprintf('T = %.4f\n', T);
fprintf('Controller C(s) = %.2f * (1 + %.4fs)/(1 + %.4fs)\n', K, alpha*T, T);

% Creazione del diagramma di Bode
figure('Position', [100, 100, 800, 600]);

% Diagramma di Bode dell'anello aperto
margin(L);
grid on;

% Aggiunta di annotazioni
title('Bode Diagram of L(s) = C(s)G(s) with \omega_c = 20 rad/s');

   
% Verifica del margine di fase e della frequenza di attraversamento
[Gm, Pm, Wcg, Wcp] = margin(L);
fprintf('Margine di fase: %.2f gradi alla frequenza di %.2f rad/s\n', Pm, Wcp);

% Salvataggio della figura (opzionale)
saveas(gcf, 'bode_high_bandwidth.eps');
print('-dpng', '-r300', 'bode_high_bandwidth.png');