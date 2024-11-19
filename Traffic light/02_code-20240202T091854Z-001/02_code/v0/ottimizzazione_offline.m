
vincoli=[];
time_zero=0;
for i=2:length(PATH)-2
    nodi_att(i-1)=PATH(i);
end
x_time=sdpvar(5,1,'full'); %varabile di ottimizzazione 'crossing time' dove 5 è dato dal numero di semafori
% vincoli=[vincoli;x(1)==d_zero;x(7)==tf];

delta_time=sdpvar(5,1,'full'); %a questo punto calcolo i delta_t

for i=1:5 %problema, salta la differenza tra il secondo e il primo tempo
    if i==1
        vincoli=[vincoli;delta_time(i)==x_time(i)-time_zero];   
    else
        vincoli=[vincoli;delta_time(i)==x_time(i)-x_time(i-1)];
    end
end



vel=sdpvar(5,1,'full');
for i=1:5
    if i==1
        vincoli=[vincoli;vel(i)==(mat_nodes(nodi_att(i),1)-0)/(delta_time(i))];
        vincoli=[vincoli;vel(i)<=v_max;vel(i)>=v_min];
    else
        vincoli=[vincoli;vel(i)==(mat_nodes(nodi_att(i),1)-mat_nodes(nodi_att(i-1),1))/(delta_time(i))];
%         vincoli=[vel(i)<=v_max;vel(i)>=v_min];
    end
end

% 
%%cost function
%dubbio sulla E_jump perchè fino a n+1? 


% 
% m=sdpvar(7,1,'full');
% mi creo un vettore lungo 5 contenente i nodi attraversati presenti in
% path escludendo nodo iniziale e finale 

for i=1:length(nodi_att)
    k_iesimo=floor(mat_nodes(nodi_att(i),2)/T)
%valori di tempi massimi e minimi di verde
    if teta(i)<T-T_gr
        t_meno=k_iesimo*T+teta(i)+passo
        t_piu=k_iesimo*T+teta(i)+T_gr
    else
        t_meno=k_iesimo*T+teta(i)-T+passo
        t_piu=k_iesimo*T+teta(i)+T_gr-T
    end
    
    %valori dati dal pruning

    t_p_min=ti_min(i+1);
    t_p_max=ti_max(i+1);

    %una volta ottenuti i tempi vado a scegliere il maggiore tra t_p_min e t_meno 
    %e il minore tra t_p_max e t_piu
    
    t_inf=max(t_p_min,t_meno);
    t_sup=min(t_p_max,t_piu);
%     
%     vincoli=[vincoli;x(i)>=m(i);m(i)>=t_p_min(i);m(i)>=t_meno(i);x(i)<=t_sup(i)];
    vincoli=[vincoli;x_time(i)>=t_inf;x_time(i)<=t_sup];
end


fn=(x_time')*(b_1.*(1/h_1.*(h_2.*vel.^2+h_3.*vel+h_0)).*vel+b_2.*(1/h_1.*(h_2.*vel.^2+h_3.*vel+h_0)).^2)+sum(E_jump_new(EDGEPATH));
% 


result=solvesdp(vincoli,fn)

%% Generazione del profilo di velocità di riferimento

crossing_time_st=1;
crossing_time=double(x_time);
crossing_time=round(crossing_time*(1/passo))/(1/passo);
costant_speed=double(vel);

length_crossing_time=length(crossing_time);


for j=1:length_crossing_time+1
    if j==1
            t_ref=find(abs(time-crossing_time(j))<passo*0.1);
        for i=crossing_time_st:t_ref
            vel_star(i)=costant_speed(j);
        end
        t_ref_fin=t_ref;
    elseif j>1 && j<length_crossing_time+1
            t_ref=find(abs(time-crossing_time(j))<passo*0.1);
        for i=t_ref_fin+1:t_ref
            vel_star(i)=costant_speed(j);
        end
        t_ref_fin=t_ref;
    elseif j==length_crossing_time+1
        find_final_t=find(abs(time-tf)<passo*0.1);
        for i=t_ref_fin+1:find_final_t
            vel_star(i)=delta_d(6)/(tf-crossing_time(j-1));
        end
    end
end

%references di velocità dei vari tratti % COMMENTATO
%     figure()
%     plot(time,vel_star)
%     hold on
%     axis([0 210 0 15]);
%     grid on
%     ylabel ('reference speed [m/s]');
%     xlabel ('time [s]');
% 
% for i =1:length(crossing_time)+1
%     if i ==1
%         prova_plot(i)=0;
%     else
%         prova_plot(i)=crossing_time(i-1);
%     end
% end
% prova_plot(7)=200;
% prova_plot=prova_plot*(1/passo);
% figure(3)
% plot(prova_plot,d,'r')