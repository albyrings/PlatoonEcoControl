%indico con ti_min il tempo minimo richiesto per passare l'iesima
%intersezione e con ti_max quello massimo. 

for v=2:n   
    ti_min(v)=((d(v)-d(v-1))/(v_max))+ti_min(v-1);
    ti_min(v) = round(ti_min(v)*(1/passo))/(1/passo);
    ti_max(v)=((d(v)-d(v-1))/(v_min))+ti_max(v-1);
    ti_max(v)=min(ti_max(v),(tf-((d(n+1)-d(v))/v_max)));
    ti_max(v) = round(ti_max(v)*(1/passo))/(1/passo);
 
%dati ora ti_min e ti_max vedo se il semaforo dell'i-esima intersezione al
%dato tempo è sullo stato verde e se non lo è ti_min viene messo al primo
%istante della successiva fase verde e invece ti_max viene anticipato
%all'ultimo istante della precedente fase verde.

tt=find(abs(time-ti_min(v))<passo*0.1);
        if s(v-1,tt)~=1
                ti_min(v)=((floor((ti_min(v))/T))+1)*T+teta(v-1)+passo;
        end
% kk=0;   %kk l'ho introdotto per capire in quale ciclo sono 
TT=find(abs(time-ti_max(v))<passo*0.1);
        if s(v-1,TT)~=1
%             while ti_max(v)>kk*T
%                kk=kk+1;
%             end
            if v==2
                ti_max(v)=floor((ti_max(v))/T)*T+teta(v-1)-(T-T_gr)-passo;
            else
                ti_max(v)=floor((ti_max(v))/T)*T+teta(v-1)+T_gr;
            end
        end
               
           
        
end




% % mostro che i semafori ai nuovi tempi siano nello stato di verde

m=length(ti_min); %stessa lunghezza sia per ti_min_new che per ti_max_new
% 
% for m=1:n-1
%     Semafori_Post_Primo_Shift_ti_min(m)=s(m,ti_min(m+1));
%     Semafori_Post_Primo_Shift_ti_max(m)=s(m,ti_max(m+1));
% end

ti_min(n+1)=tf;
ti_max(n+1)=tf;


% a questo punto verifico che i nuovi tempi rispettino il vincolo sulla
% vel_max e se non dovessero rispettarlo vengono posti al primo istante
% % ammissibile. [ho un problema sul controllo dello stato del semaforo al nuovo tempo riga 101-104]



%mostro la matrice degli stati dei semafori



%mostro i ti_min e ti_max ottenuti dalle formule alle righe 69-71 e i nuovi
%valori ti_min_new e ti_max_new ottenuti dopo i vari controlli 

ti_min;
ti_max;
% 
for f=1:n
        V__min(f)=(d(f+1)-d(f))/(ti_max(f+1)-ti_max(f));
        V__max(f)=(d(f+1)-d(f))/(ti_min(f+1)-ti_min(f));
end



if ti_max(n+1)<=ti_max(n)||((d(n+1)-d(n))/(ti_max(n+1)-ti_max(n)))>v_max
    ti_max(n)=ti_max(n+1)-((d(n+1)-d(n))/v_max);
    ti_max(n) = round(ti_max(n)*(1/passo))/(1/passo);
    TT=find(abs(time-ti_max(n))<0.001);    
        if s(n-1,TT)~=1
            ti_max(n)=((floor(ti_max(n)/T))*T+(teta(n-1)-1)+T_gr);
        end
end
% % if ti_min(n+1)<=ti_min(n)||((d(n+1)-d(n))/(ti_min(n+1)-ti_min(n)))<v_min
% %     ti_min(n)=ti_min(n+1)-ceil((d(n+1)-d(n))/v_min);
% %         if s(n-1,ti_min(n))~=1
% %             ti_min(n)=(floor(ti_min(n)/T))*T+teta(n)+T+1;
% %         end
% % end
% %    
for i=n:-1:3
        if ti_max(i)<=ti_max(i-1)||((d(i)-d(i-1))/(ti_max(i)-ti_max(i-1)))>v_max
            ti_max(i-1)=ti_max(i)-((d(i)-d(i-1))/v_max);
            ti_max(i-1) = round(ti_max(i-1)*(1/passo))/(1/passo);
            TT=find(abs(time-ti_max(i-1))<0.001);
            if s(i-2,TT)~=1
                if teta(i-1)>(T-T_gr)
                    ti_max(i-1)=((floor(ti_max(i-1)/T))*T+(teta(i-1))-(T-T_gr));
                    ti_max(i-1) = round(ti_max(i-1)*(1/passo))/(1/passo);
                else
                    ti_max(i-1)=((floor(ti_max(i-1)/T))*T+(teta(i-1))+T_gr);
                    ti_max(i-1) = round(ti_max(i-1)*(1/passo))/(1/passo);
                end
            end
        end
end
% %     if ti_max(i)<=ti_max(i-1)||((d(i)-d(i-1))/(ti_max(i)-ti_max(i-1)))<v_min
% %         ti_max(i-1)=ti_max(i)-ceil((d(i)-d(i-1))/v_min);
% %         if s(i-1,ti_max(i-1))~=1
% %             ti_max(i-1)=((floor(ti_max(i-1)/T))*T+teta(i-1)+T_gr);
% %         end
% %     end
% % %     if ti_min(i)<=ti_min(i-1)||((d(i)-d(i-1))/(ti_min(i)-ti_min(i-1)))>v_max
% % %         ti_min(i-1)=ti_min(i)-ceil((d(i)-d(i-1))/v_max);
% % %         if s(i-1,ti_min(i-1))~=1
% % %             ti_min(i-1)=((floor(ti_min(i-1)/T)+1))*T+teta(i-1)+1;
% % %         end
% % %     end
% %     if i~=2
% %         if ti_min(i)<=ti_min(i-1)||((d(i)-d(i-1))/(ti_min(i)-ti_min(i-1)))<v_min
% %             ti_min(i-1)=ti_min(i)-ceil((d(i)-d(i-1))/v_min);
% %                 if s(i-1,ti_min(i-1))~=1
% %                     ti_min(i-1)=((floor(ti_min(i-1)/T))*T+teta(i-1)+T_gr);
% %                 end
% %         end
% %     end
% %  end
% %  
% % % mostro che i semafori ai nuovi tempi siano nello stato di verde
% 
% m=length(ti_min); %stessa lunghezza sia per ti_min_new che per ti_max_new
% % 
% for m=1:n-1
%     Semafori_Post_Shift_ti_min(m)=s(m,ti_min(m+1));
%     Semafori_Post_Shift_ti_max(m)=s(m,ti_max(m+1));
% end
% 
% % % creo i vettori conteneti le velocità tra le rispettive intersezioni 
for f=1:n
        V__min(f)=(d(f+1)-d(f))/(ti_max(f+1)-ti_max(f));
        V__max(f)=(d(f+1)-d(f))/(ti_min(f+1)-ti_min(f));
end
% % 
% 
% % 
% % 
V__min
V__max
% % 
indice_rombo=1;
for i=1:length(d)
    rombo_distanze(indice_rombo)=d(i);
    rombo_tempi(indice_rombo)=ti_min(i);
    indice_rombo=indice_rombo+1;
end
for i=1:length(d)
    rombo_distanze(indice_rombo)=d(i);
    rombo_tempi(indice_rombo)=ti_max(i);
    indice_rombo=indice_rombo+1;
end

plot_semafori_times
