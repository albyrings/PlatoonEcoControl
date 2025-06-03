clc;
close all;
clear;
% Apri la figura
fig = openfig('30.fig');
ax = findall(fig, 'Type', 'axes');
lines = flipud(findall(ax, 'Type', 'line'));  % Ordina

% Linea 3: veicolo di riferimento
x3 = get(lines(3), 'XData');
y3 = get(lines(3), 'YData');

% Tratto da inizio fino a Y = 1200 (in cui sono tutti insieme)
idx_shared = y3 <= 1200;
x_shared = x3(idx_shared);
y_shared = y3(idx_shared);

% Offset da applicare a veicoli 4 e 5
offsets = [4, 8];

for j = 1:2
    line_idx = 3 + j;  % Linee 4 e 5

    % Parte post-stacco originale
    x_orig = get(lines(line_idx), 'XData');
    y_orig = get(lines(line_idx), 'YData');
    idx_post = y_orig > 1200;
    x_post = x_orig(idx_post);
    y_post = y_orig(idx_post);

    % Parte iniziale clonata dalla linea 3 con offset
    y_clone = y_shared + offsets(j);
    x_clone = x_shared;

    % Unione coerente
    x_new = [x_clone, x_post];
    y_new = [y_clone, y_post];

    % Assegna alla linea
    set(lines(line_idx), 'XData', x_new, 'YData', y_new);

    % (facoltativo) Cambia colore per visibilità
    set(lines(line_idx), 'LineWidth', 1.5);
end

% Aggiorna grafico
drawnow;

% Salva nuova figura
savefig(fig, '30_plotone_finale.fig');
