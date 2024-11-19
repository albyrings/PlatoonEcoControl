%% PARAMETRI PROVA

% d_tr(1)=650; %PROVA
% ti_min_re(1)=60; ti_max_re(1)=60; %d_tr(1)/x(2,t);
clear ti_min_re
clear ti_max_re
clear d_tr
clear V__min_re
clear V__max_re
d_tr(1)=x(1,t+1);
ti_min_re(1)=t_tr(1)*T_camp; 
ti_max_re(1)=t_tr(1)*T_camp; % Veri valori dati nel caso di replanning

%% determino in tra quale intersezione mi trovo 

if d_tr(1)>=d(1) && d_tr(1)<d(2)
    num_int=1;
elseif d_tr(1)>=d(2) && d_tr(1)<d(3)
    num_int=2;
elseif d_tr(1)>=d(3) && d_tr(1)<d(4)
    num_int=3;
elseif d_tr(1)>=d(4) && d_tr(1)<d(5)
    num_int=4;
elseif d_tr(1)>=d(5) && d_tr(1)<d(6)
    num_int=5;
elseif d_tr(1)>=d(6) && d_tr(1)<d(7)
    num_int=6;
% elseif d_tr(1)>=d(7)
%     disp('Error');    
end

%% RICREO UN VETTORE DI DISTANZE CONSIDERANDO LA COND INIZIALE CON LUNGHEZZA VARIABILE IN BASE AL VALORE DI DISTANZA OTTENUTO

indice_k=num_int;

for j=2:(n+2)-num_int
    
    d_tr(j)=d_tr(j-1)+(d(indice_k+1)-d_tr(j-1));
    indice_k=indice_k+1;
 
end

%% ricostrusico il rombo con le nuove condizioni iniziali 

sem=num_int;

for v=2:length(d_tr)-1  
    ti_min_re(v)=((d_tr(v)-d_tr(v-1))/(v_max))+ti_min_re(v-1);
    ti_min_re(v) = round(ti_min_re(v)*(1/passo))/(1/passo)
    ti_max_re(v)=((d_tr(v)-d_tr(v-1))/(v_min))+ti_max_re(v-1);
%     %aggiungerei un check sulla velocità minima ma ha senso farlo qui?
%         if v==2 && ((d_tr(v)-d_tr(v-1))/(ti_max_re(v)-ti_max_re(v-1)))<v_min
%             ti_max_re(v)=ti_min_re(v)
%         end
    ti_max_re(v)=min(ti_max_re(v),(tf-((d_d-d_tr(v))/v_max)));
    ti_max_re(v) = round(ti_max_re(v)*(1/passo))/(1/passo)
    %problema sul ti_max in teoria dovrebbe essere k=0 per selezionare la
    %fase giusta 
    prova__0=ti_min_re
    prova__1=ti_max_re
%dati ora ti_min e ti_max vedo se il semaforo dell'i-esima intersezione al
%dato tempo è sullo stato verde e se non lo è ti_min viene messo al primo
%istante della successiva fase verde e invece ti_max viene anticipato
%all'ultimo istante della precedente fase verde.
k_min=0;
k_max=0;
%     if ti_min_re(v)>teta(sem) && ti_min_re(v)<=teta(sem)+T
%         k_min=0;
%     elseif ti_min_re(v)>teta(sem)+T && ti_min_re(v)<=teta(sem)+2*T
%         k_min=1;
%     elseif ti_min_re(v)>teta(sem)+2*T && ti_min_re(v)<=teta(sem)+3*T
%         k_min=2;
%     elseif ti_min_re(v)>teta(sem)+3*T && ti_min_re(v)<=teta(sem)+4*T
%         k_min=3;
%     elseif ti_min_re(v)>teta(sem)+4*T && ti_min_re(v)<=teta(sem)+5*T
%         k_min=4;
%     elseif ti_min_re(v)>teta(sem)+5*T && ti_min_re(v)<=teta(sem)+6*T
%         k_min=5;
%     elseif ti_min_re(v)>teta(sem)+6*T && ti_min_re(v)<=teta(sem)+7*T
%         k_min=6;
%     end
% %stesso discorso per i ti_max_re
%     if ti_max_re(v)>teta(sem) && ti_max_re(v)<=teta(sem)+T
%         k_max=0;
%     elseif ti_max_re(v)>teta(sem)+T && ti_max_re(v)<=teta(sem)+2*T
%         k_max=1;
%     elseif ti_max_re(v)>teta(sem)+2*T && ti_max_re(v)<=teta(sem)+3*T
%         k_max=2;
%     elseif ti_max_re(v)>teta(sem)+3*T && ti_max_re(v)<=teta(sem)+4*T
%         k_max=3;
%     elseif ti_max_re(v)>teta(sem)+4*T && ti_max_re(v)<=teta(sem)+5*T
%         k_max=4;
%     elseif ti_max_re(v)>teta(sem)+5*T && ti_max_re(v)<=teta(sem)+6*T
%         k_max=5;
%     elseif ti_max_re(v)>teta(sem)+6*T && ti_max_re(v)<=teta(sem)+7*T
%         k_max=6;
%     end
k_min=floor((ti_min_re(v)-teta(sem))/T)    
k_max=floor((ti_max_re(v)-teta(sem))/T)

tt=find(abs(time-ti_min_re(v))<passo*0.1);
TT=find(abs(time-ti_max_re(v))<passo*0.1);

if s(sem,tt)==0 && s(sem,TT)==0 && num_int~=1
    if ti_max_re(v)-ti_min_re(v)<(T-T_gr)
%         if teta(sem)> T-T_gr
           ti_min_re(v)=(k_min+1)*T+teta(sem)+passo;
%         else
%            ti_min_re(v)=(k_min+1)*T+teta(sem)+passo;
%         end
           ti_max_re(v)=ti_min_re(v);
    else
%         if teta(sem)> T-T_gr
           ti_min_re(v)=(k_min+1)*T+teta(sem)+passo;
           ti_max_re(v)=k_max*T+teta(sem)+T_gr-passo;
%         else
%            ti_min_re(v)=(k_min+1)*T+teta(sem)+passo;
%            ti_max_re(v)=k_max*T+teta(sem)+T_gr-passo;
%         end
    end
else
        if s(sem,tt)~=1
%             if teta(sem)> T-T_gr
                ti_min_re(v)=(k_min+1)*T+teta(sem)+passo;
%             else
%                 ti_min_re(v)=(k_min+1)*T+teta(sem)+passo;
%             end
        end
        
%  
% TT=find(abs(time-ti_max_re(v))<passo*0.1);

        if s(sem,TT)~=1
%             if teta(sem)> T-T_gr
                ti_max_re(v)=k_max*T+teta(sem)+T_gr-passo;
%             else
                %ho modificato per prova aggiungendo -T come nel caso
                %precedente
%                 ti_max_re(v)=k_max*T+teta(sem)+T_gr-passo;
%             end
        end
                             
end
sem=sem+1; 
end
% if ((d_tr(2)-d_tr(1))/(ti_max_re(2)-ti_max_re(1)))<v_min
%     ti_max_re(2)=ti_min_re(2);
% end

ti_min_re(v+1)=tf;
ti_max_re(v+1)=tf;

ti_min_re
ti_max_re
% 
% 
% 
% 
% % % mostro che i semafori ai nuovi tempi siano nello stato di verde
% 
m_re=length(ti_min_re); %stessa lunghezza sia per ti_min_new che per ti_max_new
% % 
% % for m=1:n-1
% %     Semafori_Post_Primo_Shift_ti_min(m)=s(m,ti_min(m+1));
% %     Semafori_Post_Primo_Shift_ti_max(m)=s(m,ti_max(m+1));
% % end
% 
%
% 
% 
% % a questo punto verifico che i nuovi tempi rispettino il vincolo sulla
% % vel_max e se non dovessero rispettarlo vengono posti al primo istante
% % % ammissibile. [ho un problema sul controllo dello stato del semaforo al nuovo tempo riga 101-104]
% 
% 
% 
% %mostro la matrice degli stati dei semafori
% 
% 
% 
% %mostro i ti_min e ti_max ottenuti dalle formule alle righe 69-71 e i nuovi
% %valori ti_min_new e ti_max_new ottenuti dopo i vari controlli 
% ind_delta_re=1;
% for i=1:5
%     delta_t_re(ind_delta_re)=ti_min_re(i+1)-ti_min_re(i);
%     delta_T_re(ind_delta_re)=ti_max_re(i+1)-ti_max_re(i);
%     ind_delta_re=ind_delta_re+1;
% end
% 
% 
% % 

% 
% 
% 
% if ti_max(n+1)<=ti_max(n)||((d(n+1)-d(n))/(ti_max(n+1)-ti_max(n)))>v_max
%     ti_max(n)=ti_max(n+1)-((d(n+1)-d(n))/v_max);
%     ti_max(n) = round(ti_max(n)*(1/passo))/(1/passo);
%     TT=find(abs(time-ti_max(n))<0.001);    
%         if s(n-1,TT)~=1
%             ti_max(n)=((floor(ti_max(n)/T))*T+(teta(n-1)-1)+T_gr);
%         end
% end
sem_ck=5;
for i=m_re:-1:3
        if ti_max_re(i)<=ti_max_re(i-1)||((d_tr(i)-d_tr(i-1))/(ti_max_re(i)-ti_max_re(i-1)))>(v_max+1)
            ti_max_re(i-1)=ti_max_re(i)-((d_tr(i)-d_tr(i-1))/v_max);
            ti_max_re(i-1) = round(ti_max_re(i-1)*(1/passo))/(1/passo);
            TT=find(abs(time-ti_max_re(i-1))<passo*0.1);
            if s(sem_ck,TT)~=1
%                 if teta(i-1)>(T-T_gr)
                    z_ciclo=floor((ti_max_re(i-1)-teta(sem_ck))/T);
                    ti_max_re(i-1)=z_ciclo*T+(teta(sem_ck))+T_gr-passo;
                    ti_max_re(i-1) = round(ti_max_re(i-1)*(1/passo))/(1/passo);
%                 else
%                     ti_max_re(i-1)=((floor(ti_max_re(i-1)/T))*T+(teta(i-1))+T_gr);
%                     ti_max_re(i-1) = round(ti_max_re(i-1)*(1/passo))/(1/passo);
%                 end
            end
        end
        %prova per risolvere il problema dell'ultimo replanning
        if ((d_tr(i)-d_tr(i-1))/(ti_min_re(i)-ti_min_re(i-1)))>(v_max+1) || ti_min_re(i-1)>ti_max_re(i-1)
            ti_min_re(i-1)=ti_max_re(i-1);
        end
%            ti_min_re(i-1)=ti_min_re(i)-ti_min_re(i-1);
%            ti_min_re(i-1) = round(ti_min_re(i-1)*(1/passo))/(1/passo);
%            tt=find(abs(time-ti_min_re(i-1))<passo*0.1);
%            if s(sem_ck,tt)~=1
%                 if teta(i-1)>(T-T_gr)
%                     ti_min_re(i-1)=(((ti_min_re(i-1))/T)+1)*T+teta(sem_ck)+passo;
%                     ti_min_re(i-1) = round(ti_min_re(i-1)*(1/passo))/(1/passo);
%                 else
%                     ti_min_re(i-1)=((ti_min_re(i-1)/T)+1)*T+(teta(sem_ck))+passo;
%                     ti_min_re(i-1) = round(ti_min_re(i-1)*(1/passo))/(1/passo);
%                 end
%             end
           
%         end
        sem_ck=sem_ck-1;
end

% if ((d_tr(2)-d_tr(1))/(ti_max_re(2)-ti_max_re(1)))>(v_max+1)
%    ti_max_re(2)=ti_min_re(2);
% end

for f=2:m_re
        V__min_re(f-1)=(d_tr(f)-d_tr(f-1))/(ti_max_re(f)-ti_max_re(f-1));
        V__max_re(f-1)=(d_tr(f)-d_tr(f-1))/(ti_min_re(f)-ti_min_re(f-1));
end
% % %     if ti_max(i)<=ti_max(i-1)||((d(i)-d(i-1))/(ti_max(i)-ti_max(i-1)))<v_min
% % %         ti_max(i-1)=ti_max(i)-ceil((d(i)-d(i-1))/v_min);
% % %         if s(i-1,ti_max(i-1))~=1
% % %             ti_max(i-1)=((floor(ti_max(i-1)/T))*T+teta(i-1)+T_gr);
% % %         end
% % %     end
% % % %     if ti_min(i)<=ti_min(i-1)||((d(i)-d(i-1))/(ti_min(i)-ti_min(i-1)))>v_max
% % % %         ti_min(i-1)=ti_min(i)-ceil((d(i)-d(i-1))/v_max);
% % % %         if s(i-1,ti_min(i-1))~=1
% % % %             ti_min(i-1)=((floor(ti_min(i-1)/T)+1))*T+teta(i-1)+1;
% % % %         end
% % % %     end
% % %     if i~=2
% % %         if ti_min(i)<=ti_min(i-1)||((d(i)-d(i-1))/(ti_min(i)-ti_min(i-1)))<v_min
% % %             ti_min(i-1)=ti_min(i)-ceil((d(i)-d(i-1))/v_min);
% % %                 if s(i-1,ti_min(i-1))~=1
% % %                     ti_min(i-1)=((floor(ti_min(i-1)/T))*T+teta(i-1)+T_gr);
% % %                 end
% % %         end
% % %     end
% % %  end
% % %  
% % % % mostro che i semafori ai nuovi tempi siano nello stato di verde
% % 
% % m=length(ti_min); %stessa lunghezza sia per ti_min_new che per ti_max_new
% % % 
% % for m=1:n-1
% %     Semafori_Post_Shift_ti_min(m)=s(m,ti_min(m+1));
% %     Semafori_Post_Shift_ti_max(m)=s(m,ti_max(m+1));
% % end
% % 
% % % % creo i vettori conteneti le velocità tra le rispettive intersezioni 
% for f=1:n
%         V__min(f)=(d(f+1)-d(f))/(ti_max(f+1)-ti_max(f));
%         V__max(f)=(d(f+1)-d(f))/(ti_min(f+1)-ti_min(f));
% end
% % % 
% % 
% % % 
% % % 
V__min_re
V__max_re

for i=1:length(d_tr)
    rombo_distanze(indice_rombo)=d_tr(i);
    rombo_tempi(indice_rombo)=ti_min_re(i);
    indice_rombo=indice_rombo+1;
end
for i=1:length(d_tr)
    rombo_distanze(indice_rombo)=d_tr(i);
    rombo_tempi(indice_rombo)=ti_max_re(i);
    indice_rombo=indice_rombo+1;
end

plot_replanning_ok