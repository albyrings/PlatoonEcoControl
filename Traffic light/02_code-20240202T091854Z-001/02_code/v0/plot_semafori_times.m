%
d_v=1;
% 
% %Plot dello stato dei semafori
% figure() %COMMENTO
scrsz = get(0,'Screensize');
fig1 = figure(1);
set(fig1,'Position',[0 0 1.1*scrsz(3)/3 0.9*scrsz(4)/2]);
for ii=1:n-1
    a=s(ii,:);
        for jj=1:l
            a(jj);
            if a(jj)==1
                dist(ii)=d(ii+1)+0*jj;
                plot(jj,dist(ii),'g.-')
                hold on
            else
                dist(ii)=d(ii+1)+0*jj;
                plot(jj,dist(ii),'r.-')
                hold on
            end
        end

end
grid on
box off
xlabel('Time, $t$ [$10^{-2}$ sec]','Interpreter','Latex','Fontsize',14)
ylabel('Distance, $p$ [m]','Interpreter','Latex','Fontsize',14)
set(gca,'TickLabelInterpreter','latex','Fontsize',12);
ylim([0 2000]);
hold on
% Plot dei limiti massimi del poligono
ti_min_plot=ti_min/passo;
ti_max_plot=ti_max/passo;
plot(ti_min_plot,d,'Color',0.5*[1 1 1],'Linewidth',1)
hold on
plot(ti_max_plot,d,'Color',0.5*[1 1 1],'Linewidth',1)
hold on

%%DIMENTICARE
% % for index=2:n+1
% %     if ti_min(index-1)==0
% %         tt_1=1;
% %     else
% %         tt_1=find(abs(time-ti_min(index-1))<passo*0.1);
% %     end
% % 
% %     tt_2=find(abs(time-ti_min(index))<passo*0.1);  %trovo l'indice corrispondente al valote ti_min
% % 
% %     if ti_max(index-1)==0
% %         TT_1=1;
% %     else
% %         TT_1=find(abs(time-ti_max(index-1))<passo*0.1);
% %     end
% % 
% %     TT_2=find(abs(time-ti_max(index))<passo*0.1);  %trovo l'indice corrispondente al valote ti_max
% % 
% %     for t=1:l
% %         if t>tt_1 && t<=tt_2
% %                 y=(((d(index)-d(index-1))/((tt_2-tt_1)))).*(t-tt_1)+d(index-1);
% %                 plot(t,y,'.')
% %                 hold on
% %         end
% %         if t>TT_1 && t<=TT_2
% %                 y=(((d(index)-d(index-1))/((TT_2-TT_1)))).*(t-TT_1)+d(index-1);
% %                 plot(t,y,'.')
% %                 hold on
% %         end
% %     end
% % end
% % hold on
%
% % MAP CONSTRUCTION FOR OPTIMAL PATH
%
m=length(ti_min);
d_zero=0.0000000001;
t_zero=0.0000000001;
node{1}=[d_zero,t_zero]; %node in position 1 contains the starting point (0,0)
node{2}=[d(7),tf]; %node in position 2 contains the goal point (df,tf)
clear node
clear ti_mid
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

% calcolo tempi per nodi interni al poligono 

 for o=1:m
    if ti_max(o)-ti_min(o)>=T+T_gr
        z=0;
%         if teta(o-1)<=(T-T_gr)
            z=floor((ti_min(o)-teta(o-1))/T)+1;
            ti_mid_1(index)=z*T+teta(o-1)+passo;
            ti_mid_1(index) = round(ti_mid_1(index)*(1/passo))/(1/passo);
            ti_mid_2(index)=z*T+teta(o-1)+(T_gr)-passo;
            ti_mid_2(index) = round(ti_mid_2(index)*(1/passo))/(1/passo);        
%         else
%             
%             z=floor(ti_min(o)/T)+1;
%             ti_mid_1(index)=z*T+teta(o-1)-T+passo;
%             ti_mid_1(index) = round(ti_mid_1(index)*(1/passo))/(1/passo);
%             ti_mid_2(index)=z*T+teta(o-1)+T_gr-T;
%             ti_mid_2(index) = round(ti_mid_2(index)*(1/passo))/(1/passo);
%         end
    else
        ti_mid_1(index)=0;
        ti_mid_2(index)=0;
    end
    
    index=index+1;
    
 end
%verifico che i nuovi punti appartengono ad istanti di verde

ck=1;
for i=2:length(ti_min)-1
    ti_m=find(abs(time-ti_min(i))<passo*0.1)
    check_min(i-1)=s(i-1,ti_m);
    ti_M=find(abs(time-ti_max(i))<passo*0.1);
    check_max(i-1)=s(i-1,ti_M);
    if ti_mid_1(i)~=0 && ti_mid_2(i)~=0
        ti_mi_1=find(abs(time-ti_mid_1(i))<passo*0.1);
        ti_mi_2=find(abs(time-ti_mid_2(i))<passo*0.1);
        check_mid_1(ck)=s(i-1,ti_mi_1);
        check_mid_2(ck)=s(i-1,ti_mi_2);
        ck=ck+1;
    end
end
check_min
check_max
check_mid_1
check_mid_2
%creazione dei nodi provando a mettere i punti nuovi "_N"

N=1;
for dis=1:length(d)
    if dis==1
        node{N}=[d_zero t_zero]
        N=N+1;
    elseif dis~=length(d)
        node{N}=[d(dis) ti_min(dis)]
        N=N+1;
        if ti_mid_1(dis)==0 && ti_mid_2(dis)==0 
            node{N}=[d(dis) ti_max(dis)]
            N=N+1;
        else
            node{N}=[d(dis) ti_mid_1(dis)]
            N=N+1;
            node{N}=[d(dis) ti_mid_2(dis)]
            N=N+1;
            node{N}=[d(dis) ti_max(dis)]
            N=N+1;
        end
    else
        node{N}=[d(dis) ti_max(dis)]
        
    end
end
% figure(3)
%     for i=1:18
%         point_dist(i)=node{i}(1);
%         point_time(i)=node{i}(2);
%         if i==1
%             pt(i)=find(abs(time-point_time(i))<passo); %ho messo 1 poichè time parte da 1
%         else
%             pt(i)=find(abs(time-point_time(i))<passo*0.1);
%         end
%     end
% plot(pt,point_dist,'o','MarkerSize',5,'MarkerEdgeColor','k','MarkerFaceColor','cyan')
% hold on

z=1;
for i =1: length(d)-1
    delta_d(z)=d(i+1)-d(i);
    z=z+1;
end

%% CHECK SUI VARI COLLEGAMENTI TRA I NODI PER USARE DIJKSTRA PER PERCORSO OTTIMO
k=1;
for i=1:N-1
    for j=i:N
        if node{j}(1)-node{i}(1)<=max(delta_d) && node{j}(2)>node{i}(2) && ((node{j}(1)-node{i}(1))/(node{j}(2)-node{i}(2)))<=v_max+d_v  && ((node{j}(1)-node{i}(1))/(node{j}(2)-node{i}(2)))>=v_min
            %  nel ciclo if chiedo che la differenza tra le distanze sia inferiore ad
            %  una certa soglia, in questo caso ho considerato le varie delta dist e cho scelto la massima così da non considerare i nodi a più di un semaforo, che
            %  il tempo del nodo sccessivo sia maggiore di quello attuale e che le
            %  velocità rispettino i limiti.
            st(k)=i;
            dest(k)=j;
            k=k+1;
        end
    end
end

%ora creo il directed graph da risolvere con djikstra per trovare il
%percorso ottimo.
% 
% G=digraph(st,dest)
% figure()
% plot(G)

% figure()
% for p=1:length(st)
%     d_d=node{dest(p)}(1);
%     d_st=node{st(p)}(1);
%     for t=1:l
%         if st(p)~=1
%             t_d=find((abs(time-node{dest(p)}(2)))<passo*0.1);
%             t_st=find((abs(time-node{st(p)}(2)))<passo*0.1);
%             xx=[d_st d_d];
%             yy=[t_st t_d];
%             plot(yy,xx,'g')
%             hold on
% 
% %             if t<=t_st
% %                 yy=((d_d-d_st)/(t_d-t_st)).*(t-t_st)+d_st;
% %                 plot(t,yy,'g.')
% %             end
% %         y=(((d(index)-d(index-1))/((TT_2-TT_1)))).*(t-TT_1)+d(index-1);
%         else
%             t_d=find((abs(time-node{dest(p)}(2)))<passo*0.1);
%             t_st=find((abs(time-node{st(p)}(2)))<1);
%             xx=[d_st d_d];
%             yy=[t_st t_d];
%             plot(yy,xx,'g')
%             hold on
% 
% %             if t<=t_st
% %                 yy=((d_d-d_st)/(t_d-t_st)).*(t-t_st)+d_st;
% %                 plot(t,yy,'g.')
% %                 hold on
% %             end
% 
%         end
%     end
% end
% 
%     for i=1:N
%         point_dist(i)=node{i}(1);
%         point_time(i)=node{i}(2);
%         if i==1
%             pt(i)=find(abs(time-point_time(i))<1); %ho messo 1 poichè time parte da 1
%         else
%             pt(i)=find(abs(time-point_time(i))<passo*0.1);
%         end
%     end
% plot(pt,point_dist,'o','MarkerSize',5)
% hold on
% 
% 
% grid on
% xlabel('time [10e-1 s]')
% ylabel('distances [m]')

st
dest


%% RADDOPPIO I NODI PER AVERE SOLO UN LINK IN INGRESSO PER NODO 

clear new_node
d_d=2000;
d_aux=d_d-100;
new_node{1}=node{1};
M=2;
for pippo=1:length(dest)
    new_node{M}=node{dest(pippo)};
    M=M+1;
end
new_node{M}=new_node{M-1};

new_node_ord{1}=new_node{1};
new_node_ord_fin{1}=new_node{1};
MM=1;
MMM=2;
for baghi=2:length(d)
    filippo=0;
    kappa=1;
    MM=MM+1;
    for emma=1:M
        if new_node{emma}(1)==d(baghi);
        filippo(kappa)=emma;
        kappa=kappa+1;
        end
    end
    
    lungh=length(filippo)
    
    for ckk=lungh:-1:2
        if new_node{filippo(ckk)}(2)<new_node{filippo(ckk-1)}(2)
            new_node_ord{MM}=new_node{filippo(ckk-1)};
            MM=MM+1;
            filippo(ckk-1)=filippo(ckk);
            new_node_ord{MM}=new_node{filippo(ckk)};
        else
            new_node_ord{MM}=new_node{filippo(ckk)};
            MM=MM+1;
            new_node_ord{MM}=new_node{filippo(ckk-1)};
        end
    end
    
        d_prec=MM-(lungh-1);
        for ord=MM:-1:d_prec
            new_node_ord_fin{MMM}=new_node_ord{ord};
            MMM=MMM+1;
        end

end

elem=cell2mat(new_node_ord_fin);

% creo una matrice 31x2 contenete in prima colonna la distanza del nodo
% "riga" e in seconda colonna il tempo del nodo "riga".
row=1;
for col=1:2:(length(elem))
    mat_nodes(row,1)=elem(col);
    mat_nodes(row,2)=elem(col+1);
    row=row+1;
end
    
index_1=0;

for i=1:M
    if new_node{i}(1)==d_d
        index_1=index_1+1;
    end
end



kk=1;
for i=1:length(mat_nodes)-2
    for j=i:length(mat_nodes)
        if kk==1 && mat_nodes(j,1)-mat_nodes(i,1)<=350 && mat_nodes(j,2)>mat_nodes(i,2) && (mat_nodes(j,1)-mat_nodes(i,1))/(mat_nodes(j,2)-mat_nodes(i,2))<=v_max && (mat_nodes(j,1)-mat_nodes(i,1))/(mat_nodes(j,2)-mat_nodes(i,2))>=v_min
            %  nel ciclo if chiedo che la differenza tra le distanze sia inferiore ad
            %  una certa soglia, in questo caso ho considerato le varie delta dist e cho scelto la massima così da non considerare i nodi a più di un semaforo, che
            %  il tempo del nodo sccessivo sia maggiore di quello attuale e che le
            %  velocità rispettino i limiti.
            st_new(kk)=i;
            dest_new(kk)=j;
            kk=kk+1;
        end
    end
end
kkk=2;
% 
% 
index_ck=1;

for i=1:length(mat_nodes)-1 %considero solo gli start diversi da (2000,200)   
        for j=i:length(mat_nodes)
            if mat_nodes(j,2)==mat_nodes(dest_new(kk-1),2) && i==st_new(kk-1)
            ctrl_tempi=1;
        elseif mat_nodes(j,2)~=mat_nodes(dest_new(kk-1),2) && i~=st_new(kk-1) && mat_nodes(i,2)==mat_nodes(i-1,2)
            ctrl_tempi=0;
        elseif mat_nodes(j,2)~=mat_nodes(dest_new(kk-1),2) && j~=dest_new(kk-1)
            ctrl_tempi=0;
            end
            

        
        if mat_nodes(j,1)-mat_nodes(i,1)<=max(delta_d) && mat_nodes(j,2)>mat_nodes(i,2) && ((mat_nodes(j,1)-mat_nodes(i,1))/(mat_nodes(j,2)-mat_nodes(i,2)))<=v_max+1 && ((mat_nodes(j,1)-mat_nodes(i,1))/(mat_nodes(j,2)-mat_nodes(i,2)))>=v_min && ctrl_tempi==0 %&& kk~=1
            ricerca=dest_new==j;
            somma=sum(ricerca);
            if somma==0 %vuol dire che non ho main incontrato il nodo j-esimo 
%                 if i==st_new(kk-1); 
%                     st_new(kk)=i;
%                     dest_new(kk)=j;
%                     kk=kk+1;
%                 else
                    st_new(kk)=i;
                    dest_new(kk)=j;
                    kk=kk+1;
%                 end
            else %in questo caso somma ~= da zero, quindi ho incontrato già il nodo j-esimo
                ck=find(dest_new==j);
                
                if j==dest_new(kk-1)
                    index_ck=index_ck;
                else
                    index_ck=1;
                end
                
                if mat_nodes(i,2)==mat_nodes(st_new(ck(index_ck)),2)
%                     if j~=dest_new(ck(index_ck))
                        st_new(kk)=i;
                        dest_new(kk)=j;
                        kk=kk+1;
%                     elseif((mat_nodes(j,1)-mat_nodes(i,1))/(mat_nodes(j,2)-mat_nodes(i,2)))==((mat_nodes(dest_new(ck(index_ck)),1)-mat_nodes(st_new(ck(index_ck)),1))/((mat_nodes(dest_new(ck(index_ck)),2)-mat_nodes(st_new(ck(index_ck)),2)))) && i~=st_new(ck(index_ck)) && j~=dest_new(ck(index_ck)) 
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
    st_new(kk)=M-i;
    dest_new(kk)=M;
    kk=kk+1;
end
%% CALCOLO PESI DEI VARI PERCORSI E UTILIZZO DIJKSTRA PER TROVARE PERCORSO OTTIMO 
% st_new_ck=[1 1 2 2 2 3 3 4 4 4 5 5 5 6 6 7 7 8 9 9 9 10 10 11 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30];
% dest_new_ck=[2 3 4 5 6 7 8 9 10 12 11 13 15 14 16 14 16 17 18 19 20 21 22 21 22 23 23 23 24 24 24 25 26 27 27 28 28 28 29 30 30 30 31 31];
% validation_st=st_new-st_new_ck;
% a_v=sum(validation_st)
% validation_dest=dest_new-dest_new_ck;
% b_v=sum(validation_dest)
% st_new(kk)=M-2;
% dest_new(kk)=M;
% kk=kk+1;
% st_new(kk)=M-1;
% dest_new(kk)=M;

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
v_0=input('please insert a starting speed between 0 and 14: ')
%def di E_link
for i=1:length(st_new)
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
    if st_new(i)==1
        if ((new_node{dest_new(i)}(1)-new_node{st_new(i)}(1))/(new_node{dest_new(i)}(2)-new_node{st_new(i)}(2)))>v_0
            syms TIME
            v_var(i)=v_0+a.*TIME;
        else
            syms TIME
            v_var(i)=v_0-a.*TIME;
        end
            d_v(i)=diff(v_var(i));
            d_v(i)=double(d_v(i));
            u(i)=(1/h_1).*(d_v(i)+h_2.*((v_var(i)).^2)+h_3.*(v_var(i))+h_0);
            
            fun_int(i)=b_1.*(u(i)).*(v_var(i))+b_2.*(u(i)).^2;
            
        
        %         syms TIME
        %         v_var(i)=a.*TIME;
        
        
    elseif st_new(i)~=1
        prova_1=find(dest_new==st_new(i));
        if ((new_node{dest_new(i)}(1)-new_node{st_new(i)}(1))/(new_node{dest_new(i)}(2)-new_node{st_new(i)}(2)))>((new_node{dest_new(prova_1(1))}(1)-new_node{st_new(prova_1(1))}(1))/(new_node{dest_new(prova_1(1))}(2)-new_node{st_new(prova_1(1))}(2)))
            syms TIME
            v_var(i)=((new_node{dest_new(prova_1(1))}(1)-new_node{st_new(prova_1(1))}(1))/(new_node{dest_new(prova_1(1))}(2)-new_node{st_new(prova_1(1))}(2)))+a.*TIME;
        else
            syms TIME
            v_var(i)=(((new_node{dest_new(prova_1(1))}(1)-new_node{st_new(prova_1(1))}(1))/(new_node{dest_new(prova_1(1))}(2)-new_node{st_new(prova_1(1))}(2)))-a.*TIME);
        end
        d_v(i)=diff(v_var(i));
        d_v(i)=double(d_v(i));
        u(i)=(1/h_1).*(d_v(i)+h_2.*((v_var(i)).^2)+h_3.*(v_var(i))+h_0);
        fun_int(i)=b_1.*(u(i)).*(v_var(i))+b_2.*(u(i)).^2;
        
        
    end
    % fun_int(i)=b_1.*((1/h_1).*((diff(v_var(i)))+h_2.*((v_var(i)).^2)+h_3.*(v_var(i))+h_0)).*(v_var(i))+b_2.*((1/h_1).*((diff(v_var(i)))+h_2.*((v_var(i)).^2)+h_3.*(v_var(i))+h_0)).^2;
    
    if i<length(st_new)-1
        
        if st_new(i)~=1
            prova=find(dest_new==st_new(i));
            t_jump(i)=abs((new_node{dest_new(i)}(1)-new_node{st_new(i)}(1))/(new_node{dest_new(i)}(2)-new_node{st_new(i)}(2))-((new_node{dest_new(prova(1))}(1)-new_node{st_new(prova(1))}(1))/(new_node{dest_new(prova(1))}(2)-new_node{st_new(prova(1))}(2))))/a;
            t_jump(i)=round(t_jump(i)*(1/passo))/(1/passo);
        elseif st_new(i)==1
            t_jump(i)=abs((new_node{dest_new(i)}(1)-new_node{st_new(i)}(1))/(new_node{dest_new(i)}(2)-new_node{st_new(i)}(2))-v_0)/a;
        end
        
    else
        prova=find(dest_new==st_new(i));
        t_jump(i)=abs(0 -((new_node{dest_new(prova(1))}(1)-new_node{st_new(prova(1))}(1))/(new_node{dest_new(prova(1))}(2)-new_node{st_new(prova(1))}(2))))/a;
        t_jump(i)=round(t_jump(i)*(1/passo))/(1/passo);
    end
    
    E_jump(i)=int(fun_int(i),0,t_jump(i));
%     u(i)=double(u(i));
%         if u(i)<0
%             test=1;
%         else
%             test=0;
%         end
    delta_T=new_node{dest_new(i)}(2)-new_node{st_new(i)}(2);
    E_link(i)=delta_T*((b_1*((1/h_1)*(h_2*((new_node{dest_new(i)}(1)-new_node{st_new(i)}(1))/delta_T)^2)+h_3*(((new_node{dest_new(i)}(1)-new_node{st_new(i)}(1))/delta_T)+h_0))*((new_node{dest_new(i)}(1)-new_node{st_new(i)}(1))/delta_T))+b_2*((1/h_1)*(h_2*((new_node{dest_new(i)}(1)-new_node{st_new(i)}(1))/delta_T)^2)+h_3*(((new_node{dest_new(i)}(1)-new_node{st_new(i)}(1))/delta_T)+h_0))^2);
    E_jump_new(i)=double(E_jump(i));
    if E_jump_new(i)<0
        E_jump_new(i)=abs(E_jump_new(i));
    end
    cost(i)=E_jump_new(i)+E_link(i);
    if cost(i)<10^5
        cost(i)=cost(i)*10;
    end
end

%% CREO GRAFICO CON NODI SDOPPIATI, ASSEGNO AI VARI COLLEGAMENTI I PESI OTTENUTI E USO DIJKSTRA(comando shortestpath) PER TROVARE PERCORSO OTTIMO


GG=digraph(st_new,dest_new,cost)
[s_1,s_2]=size(GG.Edges)

for i=index_1-2:-1:1
    GG.Edges.Weight(s_1-i) = 0;
end
GG.Edges.Weight(s_1-1) = 0;
GG.Edges.Weight(s_1) = 0;

% fig2 = figure(2);
% set(fig2,'Position',[0 0 1.1*scrsz(3)/3 0.9*scrsz(4)/2]);
% plot(GG)
% set(gca,'TickLabelInterpreter','latex');

[PATH,D,EDGEPATH] = shortestpath(GG,1,M,'Method','positive')

ottimizzazione_offline