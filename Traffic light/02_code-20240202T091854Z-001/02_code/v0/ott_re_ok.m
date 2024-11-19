
clear x_time_re
clear delta_time_re
clear vel_re
clear vel_star_re
clear t_inf_re
clear t_sup_re

vincoli_re=[];

time_zero_re=t_tr(1)*T_camp; %time in case of replanning
ul=5-num_int+1;

for i=1:length(PATH)-3
    nodi_att_re(i)=PATH(i+1);
end
x_time_re=sdpvar(5-num_int+1,1,'full'); %varabile di ottimizzazione 'crossing time' dove 5 è dato dal numero di semafori
% vincoli=[vincoli;x(1)==d_zero;x(7)==tf];

delta_time_re=sdpvar(5-num_int+1,1,'full'); %a questo punto calcolo i delta_t

for i=1:5-num_int+1 %problema, salta la differenza tra il secondo e il primo tempo
    if i==1
        delta_time_re(i)=x_time_re(i)-time_zero_re;
        %vincoli_re=[vincoli_re;delta_time_re(i)==x_time_re(i)-time_zero_re];   
    else
        delta_time_re(i)=x_time_re(i)-x_time_re(i-1);
        %vincoli_re=[vincoli_re;delta_time_re(i)==x_time_re(i)-x_time_re(i-1)];
    end
end



vel_re=sdpvar(5-num_int+1,1,'full');
for i=1:5-num_int+1
    if i==1
        vel_re(i)=(mat_nodes_re(nodi_att_re(i),1)-d_tr(1))/(delta_time_re(i));
        %vincoli_re=[vincoli_re;vel_re(i)==(mat_nodes_re(nodi_att_re(i),1)-d_tr(1))/(delta_time_re(i))];
        %vincoli_re=[vincoli_re;vel_re(i)<=v_max;vel_re(i)>=v_min];
    else
        vel_re(i)=(mat_nodes_re(nodi_att_re(i),1)-mat_nodes_re(nodi_att_re(i-1),1))/(delta_time_re(i));
        %vincoli_re=[vincoli_re;vel_re(i)==(mat_nodes_re(nodi_att_re(i),1)-mat_nodes_re(nodi_att_re(i-1),1))/(delta_time_re(i))];
        %vincoli=[vel_re(i)<=v_max;vel_re(i)>=v_min];
    end
end

% 
%%cost function
%dubbio sulla E_jump perchè fino a n+1? 


% 
% m=sdpvar(7,1,'full');
% mi creo un vettore lungo 5 contenente i nodi attraversati presenti in
% path escludendo nodo iniziale e finale 
teta_index=num_int;
for i=1:ul
    k_iesimo_re=floor((mat_nodes_re(nodi_att_re(i),2)-teta(teta_index))/T)
%valori di tempi massimi e minimi di verde
%     if teta(teta_index)<T-T_gr
        t_meno_re=k_iesimo_re*T+teta(teta_index)+passo
        t_piu_re=k_iesimo_re*T+teta(teta_index)+T_gr-passo
%     else
%         t_meno_re=k_iesimo_re*T+teta(teta_index)-T+passo
%         t_piu_re=k_iesimo_re*T+teta(teta_index)+T_gr-T-passo
%     end
    
    %valori dati dal pruning dopo replanning

    t_p_min_re=ti_min_re(i+1);
    t_p_max_re=ti_max_re(i+1);

    %una volta ottenuti i tempi vado a scegliere il maggiore tra t_p_min e t_meno 
    %e il minore tra t_p_max e t_piu
    
    t_inf_re(i)=max(t_p_min_re,t_meno_re);
    t_sup_re(i)=min(t_p_max_re,t_piu_re);
%     
%     vincoli=[vincoli;x(i)>=m(i);m(i)>=t_p_min(i);m(i)>=t_meno(i);x(i)<=t_sup(i)];
    vincoli_re=[vincoli_re;x_time_re(i)>=t_inf_re(i);x_time_re(i)<=t_sup_re(i)];

    teta_index=teta_index+1;
end


fn_re=(x_time_re')*(b_1.*(1/h_1.*(h_2.*vel_re.^2+h_3.*vel_re+h_0)).*vel_re+b_2.*(1/h_1.*(h_2.*vel_re.^2+h_3.*vel_re+h_0)).^2)+sum(E_jump_new_re(EDGEPATH));
% 


result_re=solvesdp(vincoli_re,fn_re)

%% Generazione del profilo di velocità di riferimento

crossing_time_st_re=time_zero_re;
crossing_time_re=double(x_time_re);
crossing_time_re=round(crossing_time_re*(1/passo))/(1/passo);
costant_speed_re=double(vel_re);

length_crossing_time_re=length(crossing_time_re);

for j=1:length_crossing_time_re+1
    if j==1
            t_start_ref=find(abs(time-crossing_time_st_re)<passo*0.1);
            t_ref_re=find(abs(time-crossing_time_re(j))<passo*0.1);
        for i=t_start_ref:t_ref_re
            vel_star_re(i)=costant_speed_re(j);
        end
        t_ref_fin_re=t_ref_re;
    elseif j>1 && j<length_crossing_time_re+1
            t_ref_re=find(abs(time-crossing_time_re(j))<passo*0.1);
        for i=t_ref_fin_re+1:t_ref_re
            vel_star_re(i)=costant_speed_re(j);
        end
        t_ref_fin_re=t_ref_re;
    elseif j==length_crossing_time_re+1
        find_final_t_re=find(abs(time-tf)<passo*0.1);
        for i=t_ref_fin_re+1:find_final_t_re+1
            vel_star_re(i)=delta_d_re(end)/(tf-crossing_time_re(j-1));
        end

    end
end
% for i =1:length(crossing_time_re)+1
%     if i ==1
%         prova_plot_re(i)=time_zero_re;
%     else
%         prova_plot_re(i)=crossing_time_re(i-1);
%     end
% end
% prova_plot_re(i+1)=200;
% prova_plot_re=prova_plot_re*(1/passo);
% 
% figure(3)
% plot(prova_plot_re,d_tr,'r')

% references di velocità dei vari tratti
%     figure()
%     plot(Tempo,vel_star_re)
%     hold on
%     axis([0 200 0 15]);
%     grid on
%     ylabel ('reference speed [m/s]');
%     xlabel ('time [s]');
%     
vel_star=vel_star_re; %aggiorno la condizione di velocità di riferimento per le fasi successive.