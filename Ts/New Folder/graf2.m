% Adatta veicoli 4 e 5 in 30.fig per farli partire da t=0
clc; clear; close all;

% 1) apro la figura
fig = openfig('30.fig','invisible');
ax  = findall(fig,'Type','axes');
lines = flipud(findall(ax,'Type','line'));  % inverte l’ordine

% 2) prendo linea 3 come riferimento
x3 = get(lines(3),'XData');
y3 = get(lines(3),'YData');

% 3) estraggo il tratto iniziale fino a y=1200
idx_shared = y3 <= 1200;
x_shared   = x3(idx_shared);
y_shared   = y3(idx_shared);

% 4) offset per veicolo 4 e 5
offsets = [ 4,  8];

for j = 1:2
    li = 3 + j;  % linee 4 e 5

    % parte post-1200 della traiettoria originale
    x_o = get(lines(li),'XData');
    y_o = get(lines(li),'YData');
    idx_post = y_o > 1200;
    x_post   = x_o(idx_post);
    y_post   = y_o(idx_post);

    % parte iniziale clonata con offset
    x_new_before = x_shared;
    y_new_before = y_shared + offsets(j);

    % unisco le due parti
    x_new = [x_new_before, x_post];
    y_new = [y_new_before, y_post];

    % aggiorno la linea
    set(lines(li),'XData',x_new,'YData',y_new,'LineWidth',1.5);
end

% 5) ridisegno e salvo
drawnow;
savefig(fig,'30_plotone.fig');