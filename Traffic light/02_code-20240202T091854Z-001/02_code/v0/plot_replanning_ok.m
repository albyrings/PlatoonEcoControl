%% PLOT STATO SEMAFORI

% figure()
% for ii=1:n-1
%     a=s(ii,:);
%         for jj=1:l
%             a(jj);
%             if a(jj)==1
%                 dist(ii)=d(ii+1)+0*jj;
%                 plot(jj,dist(ii),'g.-')
%                 hold on
%             else
%                 dist(ii)=d(ii+1)+0*jj;
%                 plot(jj,dist(ii),'r.-')
%                 hold on
%             end
%         end
% 
% end
% hold on
% grid on
% xlabel('time [10e-2 s]')
% ylabel('distances [m]')
% % Plot dei limiti massimi del poligono 
figure(1)
ti_min_plot_re=ti_min_re*(1/passo);
ti_max_plot_re=ti_max_re*(1/passo);
plot(ti_min_plot_re,d_tr,'Color',0.5*[1 1 1],'LineStyle','--','Linewidth',0.9)
hold on
plot(ti_max_plot_re,d_tr,'Color',0.5*[1 1 1],'LineStyle','--','Linewidth',0.9)
hold on
% %

% % figure(3)
% 
% % for index=2:length(d_tr)
% %     if ti_min_re(index-1)==0
% %         tt_1=1;
% %     else
% %         tt_1=find(abs(time-ti_min_re(index-1))<passo*0.1);
% %     end
% % 
% %     tt_2=find(abs(time-ti_min_re(index))<passo*0.1);  %trovo l'indice corrispondente al valore ti_min
% % 
% %     if ti_max_re(index-1)==0
% %         TT_1=1;
% %     else
% %         TT_1=find(abs(time-ti_max_re(index-1))<passo*0.1);
% %     end
% % 
% %     TT_2=find(abs(time-ti_max_re(index))<passo*0.1);  %trovo l'indice corrispondente al valote ti_max
% % 
% %     for t=1:l
% %         if t>tt_1 && t<=tt_2
% %                 y=(((d_tr(index)-d_tr(index-1))/((tt_2-tt_1)))).*(t-tt_1)+d_tr(index-1);
% %                 plot(t,y,'.')
% %                 hold on
% %         end
% %         if t>TT_1 && t<=TT_2
% %                 y=(((d_tr(index)-d_tr(index-1))/((TT_2-TT_1)))).*(t-TT_1)+d_tr(index-1);
% %                 plot(t,y,'.')
% %                 hold on
% %         end
% %     end
% % end
% % hold on
% % xlim('auto');
% % ylim([0 2000]);

%% MAP CONSTRUCTION FOR OPTIMAL PATH (trovo i valori intermedi tra ti_min_re e ti_max_re)
%
m_re=length(ti_min_re);
d_zero_re=d_tr(1);
t_zero_re=ti_min_re(1);
node_re{1}=[d_zero,t_zero]; %node in position 1 contains the starting point (0,0)
%node_re{2}=[d(m_re),tf]; %node in position 2 contains the goal point (df,tf)
clear node_re
clear ti_mid_1_re
clear ti_mid_2_re
index=1;
index_max=1;
index_min=1;

% for oo=1:m-1
%     TM=find(abs(time-ti_max(oo))<passo*0.1);
%     tm=find(abs(time-ti_min(oo))<passo*0.1);
%     if oo==1
%         ti_max_N(index_max)=0;
%         index_max=index_max+1;
%         ti_min_N(index_min)=0;
%         index_min=index_min+1;
%     else 
%         if s(oo-1,(TM-1))==0;
%             ti_max_N(index_max)=ti_max(oo);
%             index_max=index_max+1;
%         else
%             if teta(oo-1)<=(T-T_gr)
%                 zz=floor(ti_max(oo)/T)
%                 ti_max_N(index_max)=(ti_max(oo)+((zz*T)+teta(oo-1)))/2;
%                 ti_max_N(index_max) = round(ti_max_N(index_max)*(1/passo))/(1/passo);
%                 index_max=index_max+1;
%             else
%                 zz=floor(ti_max(oo)/T)
%                 ti_max_N(index_max)=(ti_max(oo)+((zz*T)+teta(oo-1)))/2;
%                 ti_max_N(index_max) = round(ti_max_N(index_max)*(1/passo))/(1/passo);
%                 index_max=index_max+1;
%             end
%         end
% 
%         if s(oo-1,(tm+1))==0;
%             ti_min_N(index_min)=ti_min(oo);
%             index_min=index_min+1;
%         else
%             if teta(oo-1)<=(T-T_gr)
%                 zz=floor(ti_min(oo)/T)
%                 ti_min_N(index_min)=(ti_min(oo)+((zz*T)+teta(oo-1)+T_gr))/2;
%                 ti_min_N(index_min) = round(ti_min_N(index_min)*(1/passo))/(1/passo);
%                 index_min=index_min+1;
%             else
%                 zz=floor(ti_min(oo)/T)
%                 ti_min_N(index_min)=(ti_min(oo)+((zz*T)+teta(oo-1)+T_gr-T))/2;
%                 ti_min_N(index_min) = round(ti_min_N(index_min)*(1/passo))/(1/passo);
%                 index_min=index_min+1;
%             end
%         end
%     end
% end
% ti_max_N(index_max)=tf;
% ti_min_N(index_min)=tf;

ti_mid_1_re=zeros(1,m_re);
ti_mid_2_re=zeros(1,m_re);
off_int=num_int;

 for o=1:m_re 
    if ti_max_re(o)-ti_min_re(o)>=T+T_gr && off_int<6 %considero solo fino a off_int uguale a 5
        z=0;
%         if teta(off_int)<=(T-T_gr)
            z=floor((ti_min_re(o)-teta(off_int-1))/T)+1;
            ti_mid_1_re(index)=z*T+teta(off_int-1)+passo;
            ti_mid_1_re(index) = round(ti_mid_1_re(index)*(1/passo))/(1/passo);
            ti_mid_2_re(index)=z*T+teta(off_int-1)+(T_gr)-passo;
            ti_mid_2_re(index) = round(ti_mid_2_re(index)*(1/passo))/(1/passo);        
%         else
%             
%             z=floor(ti_min(o)/T)+1;
%             ti_mid_1_re(index)=z*T+teta(off_int)+passo-T;
%             ti_mid_1_re(index) = round(ti_mid_1_re(index)*(1/passo))/(1/passo);
%             ti_mid_2_re(index)=z*T+teta(off_int)+T_gr-T;
%             ti_mid_2_re(index) = round(ti_mid_2_re(index)*(1/passo))/(1/passo);
%         end
    off_int=off_int+1;
    else 
        off_int=off_int+1;
    end
    
    index=index+1;
    
 end

off_int=num_int; %riporto alla condizione iniziale 
% ck=1
% for i=2:length(ti_min_re)-1
% %     ti_m_re=find(abs(time-ti_min_re(i))<passo*0.1)
% %     check_min_re(i-1)=s(i-1,ti_m_re);
% %     ti_M_re=find(abs(time-ti_max_re(i))<passo*0.1);
% %     check_max_re(i-1)=s(i-1,ti_M_re);
%     if ti_mid_1_re(i)~=0 && ti_mid_2_re(i)~=0
%         ti_mi_1=find(abs(time-ti_mid_1_re(i))<passo*0.1);
%         ti_mi_2=find(abs(time-ti_mid_2_re(i))<passo*0.1);
%         check_mid_1(ck)=s(i-1,ti_mi_1);
%         check_mid_2(ck)=s(i-1,ti_mi_2);
%         ck=ck+1;
%     end
% end
% check_min_re
% check_max_re
% check_mid_1
% check_mid_2

clear node_re

N=1;
for dis=1:m_re
    if dis==1
        node_re{N}=[d_zero_re t_zero_re]
        N=N+1;
    elseif dis~=m_re && dis~=1
        node_re{N}=[d_tr(dis) ti_min_re(dis)]
        N=N+1;
        if ti_mid_1_re(dis)==0 && ti_mid_2_re(dis)==0 && ti_min_re(dis)~=ti_max_re(dis) 
            node_re{N}=[d_tr(dis) ti_max_re(dis)]
            N=N+1;
        elseif ti_mid_1_re(dis)~=0 && ti_mid_2_re(dis)~=0 && ti_min_re(dis)~=ti_max_re(dis) 
            node_re{N}=[d_tr(dis) ti_mid_1_re(dis)]
            N=N+1;
            node_re{N}=[d_tr(dis) ti_mid_2_re(dis)]
            N=N+1;
            node_re{N}=[d_tr(dis) ti_max_re(dis)]
            N=N+1;
        end
    elseif dis==m_re && dis~=1
        node_re{N}=[d_tr(dis) ti_max_re(dis)]
        
    end
end
%% PLOT DEI NUOVI NODI
clear point_dist_re
clear point_time_re
% figure(2)
%     for i=1:N
%         point_dist_re(i)=node_re{i}(1);
%         point_time_re(i)=node_re{i}(2);
%         if i==1
%             pt_re(i)=find(abs(time-point_time_re(i))<passo*0.1); 
%         else
%             pt_re(i)=find(abs(time-point_time_re(i))<passo*0.1);
%         end
%     end
% plot(pt_re,point_dist_re,'o','MarkerSize',5,'MarkerEdgeColor','blu','MarkerFaceColor','cyan')
% hold on
%
clear delta_d_re

z=1;
for i =1: length(d_tr)-1
    delta_d_re(z)=d_tr(i+1)-d_tr(i);
    z=z+1;
end
% 
%% CHECK SUI VARI COLLEGAMENTI
clear st_re
clear dest_re

k=1;
for i=1:N-1
    if node_re{i}(1)==d_tr(1)
    for j=i:N
        if node_re{j}(1)==d_tr(2)
            if node_re{j}(1)-node_re{i}(1)<=d_tr(2)-d_tr(1) && node_re{j}(2)>node_re{i}(2) && node_re{j}(1)~=node_re{i}(1)% && ((node_re{j}(1)-node_re{i}(1))/(node_re{j}(2)-node_re{i}(2)))<=(v_max+1)  && ((node_re{j}(1)-node_re{i}(1))/(node_re{j}(2)-node_re{i}(2)))>=v_min-0.5
                %  nel ciclo if chiedo che la differenza tra le distanze sia inferiore ad
                %  una certa soglia, in questo caso ho considerato le varie delta dist e cho scelto la massima così da non considerare i nodi a più di un semaforo, che
                %  il tempo del nodo sccessivo sia maggiore di quello attuale e che le
                %  velocità rispettino i limiti.
                st_re(k)=i;
                dest_re(k)=j;
                k=k+1;
            end
        end
    end
    else
        for j=i:N
            if node_re{j}(1)-node_re{i}(1)<=max(delta_d) && node_re{j}(2)>node_re{i}(2) && node_re{j}(1)~=node_re{i}(1) && ((node_re{j}(1)-node_re{i}(1))/(node_re{j}(2)-node_re{i}(2)))<=(v_max+1)  && ((node_re{j}(1)-node_re{i}(1))/(node_re{j}(2)-node_re{i}(2)))>=v_min-0.5
                %  nel ciclo if chiedo che la differenza tra le distanze sia inferiore ad
                %  una certa soglia, in questo caso ho considerato le varie delta dist e cho scelto la massima così da non considerare i nodi a più di un semaforo, che
                %  il tempo del nodo sccessivo sia maggiore di quello attuale e che le
                %  velocità rispettino i limiti.
                st_re(k)=i;
                dest_re(k)=j;
                k=k+1;
            end
        end
    end
end
% 
% %ora creo il directed graph da risolvere con djikstra per trovare il
% %percorso ottimo.
% 
% G_re=digraph(st_re,dest_re)
% figure()
% plot(G_re)
% 
% % figure()
% % for p=1:length(st)
% %     d_d=node{dest(p)}(1);
% %     d_st=node{st(p)}(1);
% %     for t=1:l
% %         if st(p)~=1
% %             t_d=find((abs(time-node{dest(p)}(2)))<passo*0.1);
% %             t_st=find((abs(time-node{st(p)}(2)))<passo*0.1);
% %             xx=[d_st d_d];
% %             yy=[t_st t_d];
% %             plot(yy,xx,'g')
% %             hold on
% % 
% % %             if t<=t_st
% % %                 yy=((d_d-d_st)/(t_d-t_st)).*(t-t_st)+d_st;
% % %                 plot(t,yy,'g.')
% % %             end
% % %         y=(((d(index)-d(index-1))/((TT_2-TT_1)))).*(t-TT_1)+d(index-1);
% %         else
% %             t_d=find((abs(time-node{dest(p)}(2)))<passo*0.1);
% %             t_st=find((abs(time-node{st(p)}(2)))<1);
% %             xx=[d_st d_d];
% %             yy=[t_st t_d];
% %             plot(yy,xx,'g')
% %             hold on
% % 
% % %             if t<=t_st
% % %                 yy=((d_d-d_st)/(t_d-t_st)).*(t-t_st)+d_st;
% % %                 plot(t,yy,'g.')
% % %                 hold on
% % %             end
% % 
% %         end
% %     end
% % end
% % 
% %     figure(2)
% %     for i=1:N
% %         point_dist(i)=node_re{i}(1);
% %         point_time(i)=node_re{i}(2);
% %         if i==1
% %             pt_re(i)=find(abs(time-point_time(i))<passo*0.1); %ho messo 1 poichè time parte da 1
% %         else
% %             pt_re(i)=find(abs(time-point_time(i))<passo*0.1);
% %         end
% %     end
% % plot(pt_re,point_dist,'x','MarkerSize',5)
% % hold on
% % 
% % 
% % grid on
% % xlabel('time [10e-1 s]')
% % ylabel('distances [m]')
% 
%
% 
% %%
clear new_node_re
clear new_node_ord_re
clear new_node_ord_fin_re
clear filippo_re
clear d_prec_re
clear elem_re
clear st_new_re
clear dest_new_re
clear mat_nodes_re
clear cost_re

d_d=2000;

new_node_re{1}=node_re{1};
M=2;
for pippo=1:length(dest_re)
    new_node_re{M}=node_re{dest_re(pippo)};
    M=M+1;
end
new_node_re{M}=new_node_re{M-1};

% fino a qui ok

new_node_ord_re{1}=new_node_re{1};
new_node_ord_fin_re{1}=new_node_re{1};
MM=1;
MMM=2;
for ba=2:length(d_tr)
    fi_re=0;
    kappa=1;
    MM=MM+1;
    for em=1:M
        if new_node_re{em}(1)==d_tr(ba); %cerco solo i nodi aventi distanza pari a quella selezionata
        fi_re(kappa)=em; %creo un vettore contenente in numero del nodo avente distanza uguale a quella selezionata
        kappa=kappa+1;
        end
    end
    
    lungh=length(fi_re)
    if lungh<2
%         for ckk=1:lungh
            ckk=1;
            new_node_ord_re{MM}=new_node_re{fi_re(ckk)};
            new_node_ord_fin_re{MMM}=new_node_ord_re{MM};
            MMM=MMM+1;
%         end
    else
        for ckk=lungh:-1:2
            if new_node_re{fi_re(ckk)}(2)<new_node_re{fi_re(ckk-1)}(2)
                new_node_ord_re{MM}=new_node_re{fi_re(ckk-1)};
                MM=MM+1;
                fi_re(ckk-1)=fi_re(ckk);
                new_node_ord_re{MM}=new_node_re{fi_re(ckk)};
            else
                new_node_ord_re{MM}=new_node_re{fi_re(ckk)};
                MM=MM+1;
                new_node_ord_re{MM}=new_node_re{fi_re(ckk-1)};
            end
        end

        d_prec_re=MM-(lungh-1);

            for ord=MM:-1:d_prec_re
                new_node_ord_fin_re{MMM}=new_node_ord_re{ord};
                MMM=MMM+1;
            end
    end
 
end

elem_re=cell2mat(new_node_ord_fin_re);

% creo una matrice 31x2 contenete in prima colonna la distanza del nodo
% "riga" e in seconda colonna il tempo del nodo "riga".
row_re=1;
for col_re=1:2:(length(elem_re))
    mat_nodes_re(row_re,1)=elem_re(col_re);
    mat_nodes_re(row_re,2)=elem_re(col_re+1);
    row_re=row_re+1;
end
    
index_1=0;

for i=1:M
    if new_node_re{i}(1)==d_d
        index_1=index_1+1;
    end
end



kk=1;
for i=1:length(mat_nodes_re)-index_1
    for j=i:length(mat_nodes_re)
        if mat_nodes_re(i,1)==d_tr(1) && mat_nodes_re(j,1)==d_tr(2) && mat_nodes_re(j,1)-mat_nodes_re(i,1)<=d_tr(2)-d_tr(1) && mat_nodes_re(j,2)>mat_nodes_re(i,2) && mat_nodes_re(j,1)~=mat_nodes_re(i,1)% && (mat_nodes_re(j,1)-mat_nodes_re(i,1))/(mat_nodes_re(j,2)-mat_nodes_re(i,2))<=v_max+1 && (mat_nodes_re(j,1)-mat_nodes_re(i,1))/(mat_nodes_re(j,2)-mat_nodes_re(i,2))>=v_min-0.5
            %  nel ciclo if chiedo che la differenza tra le distanze sia inferiore ad
            %  una certa soglia, in questo caso ho considerato le varie delta dist e cho scelto la massima così da non considerare i nodi a più di un semaforo, che
            %  il tempo del nodo sccessivo sia maggiore di quello attuale e che le
            %  velocità rispettino i limiti.
            st_new_re(kk)=i;
            dest_new_re(kk)=j;
            kk=kk+1;
        end
    end
end
kkk=2;
% kk=3 caso simile a quello off
% 
index_ck=1;

for i=1:length(mat_nodes_re)-index_1 %considero solo gli start diversi da (2000,200)   
        for j=i:length(mat_nodes_re)-1
            if mat_nodes_re(j,2)==mat_nodes_re(dest_new_re(kk-1),2) && i==st_new_re(kk-1)
                ctrl_tempi_re=1;
%             elseif mat_nodes_re(j,2)~=mat_nodes_re(dest_new_re(kk-1),2) && i~=st_new_re(kk-1) && mat_nodes_re(i,2)==mat_nodes_re(i-1,2)
%                 ctrl_tempi_re=0;
%             elseif mat_nodes_re(j,2)~=mat_nodes_re(dest_new_re(kk-1),2) && j~=dest_new_re(kk-1)
%                 ctrl_tempi_re=0;
            else
                ctrl_tempi_re=0;
            end
            

        
        if mat_nodes_re(i,1)~=d_tr(1) && mat_nodes_re(j,1)-mat_nodes_re(i,1)<=max(delta_d) && mat_nodes_re(j,2)>mat_nodes_re(i,2) && mat_nodes_re(j,1)~=mat_nodes_re(i,1) && kk~=1 && ctrl_tempi_re==0 && ((mat_nodes_re(j,1)-mat_nodes_re(i,1))/(mat_nodes_re(j,2)-mat_nodes_re(i,2)))<=v_max+1 && ((mat_nodes_re(j,1)-mat_nodes_re(i,1))/(mat_nodes_re(j,2)-mat_nodes_re(i,2)))>=v_min-0.5
            ricerca_re=dest_new_re==j;
            somma_re=sum(ricerca_re);
            if somma_re==0 
                if i==st_new_re(kk-1); %vuol dire che non ho mai incontrato il nodo j-esimo 
                    st_new_re(kk)=i;
                    dest_new_re(kk)=j;
                    kk=kk+1;
                else %riguardare il perchè
                    st_new_re(kk)=i;
                    dest_new_re(kk)=j;
                    kk=kk+1;
                end
            else %in questo caso somma ~= da zero, quindi ho incontrato già il nodo j-esimo
                ck=find(dest_new_re==j);
                %riguardare 
                if j==dest_new_re(kk-1)
                    index_ck=index_ck;
                else
                    index_ck=1;
                end
                
                if mat_nodes_re(i,2)==mat_nodes_re(st_new_re(ck(index_ck)),2)
%                     if j~=dest_new_re(ck(index_ck))
                        st_new_re(kk)=i;
                        dest_new_re(kk)=j;
                        kk=kk+1;
%                     elseif((mat_nodes(j,1)-mat_nodes(i,1))/(mat_nodes(j,2)-mat_nodes(i,2)))==((mat_nodes(dest_new_re(ck(index_ck)),1)-mat_nodes(st_new_re(ck(index_ck)),1))/((mat_nodes(dest_new_re(ck(index_ck)),2)-mat_nodes(st_new_re(ck(index_ck)),2)))) && i~=st_new(ck(index_ck)) && j~=dest_new(ck(index_ck)) 
%                         st_new(kk)=i;
%                         dest_new(kk)=j;
%                         kk=kk+1;
%                     end
                    index_ck=index_ck+1;
                end
            end
            
        end
        
        end
    kkk=kk-1;
    index_ck=1;
end

for i=index_1-1:-1:1
    st_new_re(kk)=M-i;
    dest_new_re(kk)=M;
    kk=kk+1;
end
%%
%%DEF Energie

%Per ogni edge del digraph il peso del collegamento è dato da
%W=E_link+E_jump, la prima legata a termini costanti e la seconda energia
%legata invece al cambiamento di velocità tra i due edges entrante e
%uscente dal nodo.
%E_link=delta_T*(b_1*ui_cost*vi_cost+b_2*(ui_cost)^2
%E_jump=integral(from 0 to t_jump)(b_1*ui_var*vi_var+b_2*(ui_var)^2)dt

%dati presi dal paper di De Nunzio

%PARAMETRI

g=9.81; %m/s^2
Rt=6.066; %transmission ratio
mass=1190; %kg
r=0.2848; %m
a=1.5; %m/s^2
alpha=0; %road slope
a_0=113.5; %N
a_1=0.774; %N/(m/s)
a_2=0.4212; %N/(m/s)^2
b_1=Rt/r;
b_2=0.1515; %ohm
h_0=(a_0/mass)+g*sin(alpha);
h_1=(Rt/(mass*r));
h_2=a_2/mass;
h_3=a_1/mass;
v_0=v_0_re;

clear v_var_re
clear d_v_re_new
clear d_v_re
clear u_re
clear fun_int_re
clear t_jump_re
clear E_jump_re
clear E_jump_new_re
clear E_link_re
clear cost_re
clear delta_T_re
%def di E_link
for i=1:length(st_new_re)
    %         if st_new(i)==1
    %             if ((new_node{dest_new(i)}(1)-new_node{st_new(i)}(1))/(new_node{dest_new(i)}(2)-new_node{st_new(i)}(2)))>v_0
    %                 fun_int(i)= @(TIME) b_1.*((1/h_1).*(a+h_2.*(v_0+a.*TIME).^2)+h_3.*(v_0+a.*TIME)+h_0).*(v_0+a.*TIME)+b_2.*((1/h_1).*(a+h_2.*(v_0+a.*TIME).^2)+h_3.*(v_0+a.*TIME)+h_0).^2;
    %             else
    %                 fun_int(i)= @(TIME) b_1.*((1/h_1).*(-a+h_2.*(v_0-a.*TIME).^2)+h_3.*(v_0-a.*TIME)+h_0).*(v_0-a.*TIME)+b_2.*((1/h_1).*(-a+h_2.*(v_0-a.*TIME).^2)+h_3.*(v_0-a.*TIME)+h_0).^2;
    %             end
    %         else
    %             if ((new_node{dest_new(i)}(1)-new_node{st_new(i)}(1))/(new_node{dest_new(i)}(2)-new_node{st_new(i)}(2)))>((new_node{dest_new(i-1)}(1)-new_node{st_new(i-1)}(1))/(new_node{dest_new(i-1)}(2)-new_node{st_new(i-1)}(2)));
    %                 fun_int(i)= @(TIME) b_1.*((1/h_1).*(a+h_2.*(((new_node{dest_new(i-1)}(1)-new_node{st_new(i-1)}(1))/(new_node{dest_new(i-1)}(2)-new_node{st_new(i-1)}(2)))+a.*TIME).^2)+h_3.*(((new_node{dest_new(i-1)}(1)-new_node{st_new(i-1)}(1))/(new_node{dest_new(i-1)}(2)-new_node{st_new(i-1)}(2)))+a.*TIME)+h_0).*(((new_node{dest_new(i-1)}(1)-new_node{st_new(i-1)}(1))/(new_node{dest_new(i-1)}(2)-new_node{st_new(i-1)}(2)))+a.*TIME)+b_2.*((1/h_1).*(a+h_2.*(((new_node{dest_new(i-1)}(1)-new_node{st_new(i-1)}(1))/(new_node{dest_new(i-1)}(2)-new_node{st_new(i-1)}(2)))+a.*TIME).^2)+h_3.*(((new_node{dest_new(i-1)}(1)-new_node{st_new(i-1)}(1))/(new_node{dest_new(i-1)}(2)-new_node{st_new(i-1)}(2)))+a.*TIME)+h_0).^2;
    %             else
    %                 fun_int(i)= @(TIME) b_1.*((1/h_1).*(-a+h_2.*(((new_node{dest_new(i-1)}(1)-new_node{st_new(i-1)}(1))/(new_node{dest_new(i-1)}(2)-new_node{st_new(i-1)}(2)))-a.*TIME).^2)+h_3.*(((new_node{dest_new(i-1)}(1)-new_node{st_new(i-1)}(1))/(new_node{dest_new(i-1)}(2)-new_node{st_new(i-1)}(2)))-a.*TIME)+h_0).*(((new_node{dest_new(i-1)}(1)-new_node{st_new(i-1)}(1))/(new_node{dest_new(i-1)}(2)-new_node{st_new(i-1)}(2)))-a.*TIME)+b_2.*((1/h_1).*(-a+h_2.*(((new_node{dest_new(i-1)}(1)-new_node{st_new(i-1)}(1))/(new_node{dest_new(i-1)}(2)-new_node{st_new(i-1)}(2)))-a.*TIME).^2)+h_3.*(((new_node{dest_new(i-1)}(1)-new_node{st_new(i-1)}(1))/(new_node{dest_new(i-1)}(2)-new_node{st_new(i-1)}(2)))-a.*TIME)+h_0).^2;
    %             end
    %
    %     if st_new(i)~=1
    %         t_jump(i)=abs((new_node{dest_new(i)}(1)-new_node{st_new(i)}(1))/(new_node{dest_new(i)}(2)-new_node{st_new(i)}(2))-((new_node{dest_new(i-1)}(1)-new_node{st_new(i-1)}(1))/(new_node{dest_new(i-1)}(2)-new_node{st_new(i-1)}(2))))/a;
    %     else
    %         t_jump(i)=abs((new_node{dest_new(i)}(1)-new_node{st_new(i)}(1))/(new_node{dest_new(i)}(2)-new_node{st_new(i)}(2))-v_0)/a;
    %     end
    %
    %     E_jump(i)=integral(fun_int(i),0,t_jump(i));
    %     delta_T=new_node{dest_new(i)}(2)-new_node{st_new(i)}(2);
    %     E_link(i)=delta_T*((b_1*((1/h_1)*(h_2*((new_node{dest_new(i)}(1)-new_node{st_new(i)}(1))/delta_T)^2)+h_3*(((new_node{dest_new(i)}(1)-new_node{st_new(i)}(1))/delta_T)+h_0))*((new_node{dest_new(i)}(1)-new_node{st_new(i)}(1))/delta_T))+b_2*((1/h_1)*(h_2*((new_node{dest_new(i)}(1)-new_node{st_new(i)}(1))/delta_T)^2)+h_3*(((new_node{dest_new(i)}(1)-new_node{st_new(i)}(1))/delta_T)+h_0))^2);
    %     cost(i)=E_jump(i)+E_link(i);
    %         end
    % end
    %


    %
    %
    if st_new_re(i)==1
        if ((new_node_ord_fin_re{dest_new_re(i)}(1)-new_node_ord_fin_re{st_new_re(i)}(1))/(new_node_ord_fin_re{dest_new_re(i)}(2)-new_node_ord_fin_re{st_new_re(i)}(2)))>v_0
            syms TIME
            v_var_re(i)=v_0+a.*TIME;
        else
            syms TIME
            v_var_re(i)=v_0-a.*TIME;
        end
            d_v_re(i)=diff(v_var_re(i));
            d_v_re_new(i)=double(d_v_re(i));
            u_re(i)=(1/h_1).*(d_v_re_new(i)+h_2.*((v_var_re(i)).^2)+h_3.*(v_var_re(i))+h_0);
            fun_int_re(i)=b_1.*(u_re(i)).*(v_var_re(i))+b_2.*(u_re(i)).^2;
            
        
        %         syms TIME
        %         v_var(i)=a.*TIME;
        
        
    elseif st_new_re(i)~=1
        prova_1_re=find(dest_new_re==st_new_re(i));
        if ((new_node_ord_fin_re{dest_new_re(i)}(1)-new_node_ord_fin_re{st_new_re(i)}(1))/(new_node_ord_fin_re{dest_new_re(i)}(2)-new_node_ord_fin_re{st_new_re(i)}(2)))>((new_node_ord_fin_re{dest_new_re(prova_1_re(1))}(1)-new_node_ord_fin_re{st_new_re(prova_1_re(1))}(1))/(new_node_ord_fin_re{dest_new_re(prova_1_re(1))}(2)-new_node_ord_fin_re{st_new_re(prova_1_re(1))}(2)))
            syms TIME
            v_var_re(i)=((new_node_ord_fin_re{dest_new_re(prova_1_re(1))}(1)-new_node_ord_fin_re{st_new_re(prova_1_re(1))}(1))/(new_node_ord_fin_re{dest_new_re(prova_1_re(1))}(2)-new_node_ord_fin_re{st_new_re(prova_1_re(1))}(2)))+a.*TIME;
        else
            syms TIME
            v_var_re(i)=(((new_node_ord_fin_re{dest_new_re(prova_1_re(1))}(1)-new_node_ord_fin_re{st_new_re(prova_1_re(1))}(1))/(new_node_ord_fin_re{dest_new_re(prova_1_re(1))}(2)-new_node_ord_fin_re{st_new_re(prova_1_re(1))}(2)))-a.*TIME);
        end
        d_v_re(i)=diff(v_var_re(i));
        d_v_re_new(i)=double(d_v_re(i));
        u_re(i)=(1/h_1).*(d_v_re_new(i)+h_2.*((v_var_re(i)).^2)+h_3.*(v_var_re(i))+h_0);
        fun_int_re(i)=b_1.*(u_re(i)).*(v_var_re(i))+b_2.*(u_re(i)).^2;
        
        
    end
    % fun_int(i)=b_1.*((1/h_1).*((diff(v_var(i)))+h_2.*((v_var(i)).^2)+h_3.*(v_var(i))+h_0)).*(v_var(i))+b_2.*((1/h_1).*((diff(v_var(i)))+h_2.*((v_var(i)).^2)+h_3.*(v_var(i))+h_0)).^2;
    
    if i<length(st_new_re)-1
        
        if st_new_re(i)~=1
            prova_re=find(dest_new_re==st_new_re(i));
            t_jump_re(i)=abs((new_node_ord_fin_re{dest_new_re(i)}(1)-new_node_ord_fin_re{st_new_re(i)}(1))/(new_node_ord_fin_re{dest_new_re(i)}(2)-new_node_ord_fin_re{st_new_re(i)}(2))-((new_node_ord_fin_re{dest_new_re(prova_re(1))}(1)-new_node_ord_fin_re{st_new_re(prova_re(1))}(1))/(new_node_ord_fin_re{dest_new_re(prova_re(1))}(2)-new_node_ord_fin_re{st_new_re(prova_re(1))}(2))))/a;
            t_jump_re(i)=round(t_jump_re(i)*(1/passo))/(1/passo);
        elseif st_new_re(i)==1
            t_jump_re(i)=abs((new_node_ord_fin_re{dest_new_re(i)}(1)-new_node_ord_fin_re{st_new_re(i)}(1))/(new_node_ord_fin_re{dest_new_re(i)}(2)-new_node_ord_fin_re{st_new_re(i)}(2))-v_0)/a;
        end
        
    else
        prova_re=find(dest_new_re==st_new_re(i));
        t_jump_re(i)=abs(0 -((new_node_ord_fin_re{dest_new_re(prova_re(1))}(1)-new_node_ord_fin_re{st_new_re(prova_re(1))}(1))/(new_node_ord_fin_re{dest_new_re(prova_re(1))}(2)-new_node_ord_fin_re{st_new_re(prova_re(1))}(2))))/a;
        t_jump_re(i)=round(t_jump_re(i)*(1/passo))/(1/passo);
    end
    
    E_jump_re(i)=int(fun_int_re(i),0,t_jump_re(i));
%     u(i)=double(u(i));
%         if u(i)<0
%             test=1;
%         else
%             test=0;
%         end
    delta_T_re=new_node_ord_fin_re{dest_new_re(i)}(2)-new_node_ord_fin_re{st_new_re(i)}(2);
    E_link_re(i)=delta_T_re*((b_1*((1/h_1)*(h_2*((new_node_ord_fin_re{dest_new_re(i)}(1)-new_node_ord_fin_re{st_new_re(i)}(1))/delta_T_re)^2)+h_3*(((new_node_ord_fin_re{dest_new_re(i)}(1)-new_node_ord_fin_re{st_new_re(i)}(1))/delta_T_re)+h_0))*((new_node_ord_fin_re{dest_new_re(i)}(1)-new_node_ord_fin_re{st_new_re(i)}(1))/delta_T_re))+b_2*((1/h_1)*(h_2*((new_node_ord_fin_re{dest_new_re(i)}(1)-new_node_ord_fin_re{st_new_re(i)}(1))/delta_T_re)^2)+h_3*(((new_node_ord_fin_re{dest_new_re(i)}(1)-new_node_ord_fin_re{st_new_re(i)}(1))/delta_T_re)+h_0))^2);
    E_jump_new_re(i)=double(E_jump_re(i));
    if E_jump_new_re(i)<0
        E_jump_new_re(i)=abs(E_jump_new_re(i));
    end
    cost_re(i)=E_jump_new_re(i)+E_link_re(i);
    
    if cost(i)<10^5
        cost(i)=cost(i)*10;
    end
    
end




GG_re=digraph(st_new_re,dest_new_re,cost_re)
[s_1,s_2]=size(GG_re.Edges)

% for i=index_1-2:-1:1
%     GG_re.Edges.Weight(s_1-i) = 0;
% end
GG_re.Edges.Weight(s_1-1) = 0;
GG_re.Edges.Weight(s_1) = 0;

% GG.Edges.Weight(s_1-1) = 0;
% figure()
% plot(GG_re)

[PATH,D,EDGEPATH] = shortestpath(GG_re,1,M,'Method','positive')

ott_re_ok