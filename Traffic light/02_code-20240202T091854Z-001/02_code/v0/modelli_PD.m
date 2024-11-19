%% esercitazioni modelli 
%% dichiarazione parametri
min_d_re=10;
g=9.81; %m/s^2
Rt=6.066; %transmission ratio
mass=1190; %kg
r=0.2848; %m
a=1.5; %m/s^2
alpha=0; %road slope
a_0=113.5; %N
a_1=0.774; %N/(m/s)
a_2=0.4212; %N/(m/s)^2
a_cost=5;
b_1=Rt/r;
b_2=0.1515; %ohm
h_0=(a_0/mass)+g*sin(alpha);
h_1=(Rt/(mass*r));
h_2=a_2/mass;
h_3=a_1/mass;
% gain_k=10;
K_p=50;
K_d=1;

thr1 = 0.2;
thr2 = 2;

T_camp=passo;
T_sim=200;
Tempo=0:T_camp:T_sim;

flag=zeros(1,length(Tempo)-1);
flag(1)=0;

x_0(1)=0;
x_0(2)=v_0;

y_0=v_0;
u_0=0;
conteggio=0;



%% SIMULATION
x=zeros(2,length(Tempo));
y=zeros(1,length(Tempo));
u=zeros(1,length(Tempo));
dd=zeros(1,length(Tempo));
%sigma=zeros(1,length(Tempo));
err=zeros(1,length(Tempo));
der_v=zeros(1,length(Tempo));
flag=zeros(1,length(Tempo));

%% initial conditions
x(1,1)=x_0(1);
x(2,1)=x_0(2);
y(1)=y_0;
% sigma(1)=x_0(2)-vel_star(1);
err(1)=vel_star(1)-x_0(2);
% err(1)=x_0(2)-vel_star(1);
%u(:,1)=10;
tt_ottimizzazione=0; %tempo iniziale
tt_re_ottimizzazione=0; 
disp('Run simulation')
for t=1:length(Tempo)-1
    %u(t)=(1/h_1)*(-gain_k*sign(sigma(t))); % u sliding mode control u=(ueq-ksign(s))
    
    u(t)=K_p*err(t)+K_d*(der_v(t))+(1/h_1)*(h_2*(x(2,t))^2+h_3*x(2,t)+h_0);
    dd(t)= 40+10*(sin(t/10+0.5)+0.1*rand);
    x(1,t+1)=x(1,t)+T_camp*(x(2,t));
    x(2,t+1)=x(2,t)+T_camp*(h_1*u(t)-h_2*(x(2,t))^2-h_3*x(2,t)-h_0+h_1*dd(t));
    y(t+1)=(x(2,t+1));
    err(t+1)=vel_star(t+1)-y(t+1);
    der_v(t+1)=(err(t+1)-err(t))/T_camp;
    
%     if t==1
%         err_st=0;
%         t_st=0;
%         der_v(t)=v_0;
%     else
%         err_st=err(t-1);
%         t_st=t-1;
%         der_v(t)=vel_star(t)-vel_star(t-1);
%     end
%     u(t)=K_p*err(t)+K_d*((h_1*u(t)-h_2*(x(2,t))^2-h_3*x(2,t)-h_0)-(der_v(t)));
%     x(1,t+1)=x(1,t)+T_camp*(x(2,t));
%     x(2,t+1)=x(2,t)+T_camp*(h_1*u(t)-h_2*(x(2,t))^2-h_3*x(2,t)-h_0);%*+5*h_1*sin(t));
%     y(t+1)=(x(2,t+1));
%     err(t+1)=vel_star(t+1)-y(t+1);
%     sigma(t+1)=x(2,t+1)-vel_star(t+1); %sliding surface
    
% event trigger
    if abs(vel_star(t+1)-y(t+1))>=thr1 && tt_ottimizzazione-tt_re_ottimizzazione>thr2
        d_tr(1)=x(1,t+1);
        if d_tr(1)>d(1) && d_tr(1)<=d(2)-50 % tra 0 e 300
            if d(2)-d_tr(1)>min_d_re
                t_tr(1)=t+1;
                v_0_re=y(t+1); %%velocità per il nuovo calcolo dell'energia durante il replanning 
                flag(t)=1;
        
                re_planner_provak_ok_sem
                tt_re_ottimizzazione=tt_ottimizzazione;
                conteggio=conteggio+1;
            end
        elseif d_tr(1)>d(2) && d_tr(1)<=d(3)-50 %tra 300 e 600
            if d(3)-d_tr(1)>min_d_re
                t_tr(1)=t+1;
                v_0_re=y(t+1); %%velocità per il nuovo calcolo dell'energia durante il replanning 
                flag(t)=1;
        
                re_planner_provak_ok_sem
                tt_re_ottimizzazione=tt_ottimizzazione;
                conteggio=conteggio+1;
            end
        elseif d_tr(1)>d(3) && d_tr(1)<=d(4)-50 %tra 600 e 900
            if d(4)-d_tr(1)>min_d_re
                t_tr(1)=t+1;
                v_0_re=y(t+1); %%velocità per il nuovo calcolo dell'energia durante il replanning 
                flag(t)=1;
        
                re_planner_provak_ok_sem
                tt_re_ottimizzazione=tt_ottimizzazione;
                conteggio=conteggio+1;
            end
        elseif d_tr(1)>d(4) && d_tr(1)<=d(5)-50 %tra 900 e 1200
            if d(5)-d_tr(1)>min_d_re
                t_tr(1)=t+1;
                v_0_re=y(t+1); %%velocità per il nuovo calcolo dell'energia durante il replanning 
                flag(t)=1;
        
                re_planner_provak_ok_sem
                tt_re_ottimizzazione=tt_ottimizzazione;
                conteggio=conteggio+1;
            end
        elseif d_tr(1)>d(5) && d_tr(1)<=d(6)-50 %tra 1200 e 1550
            if d(6)-d_tr(1)>min_d_re
                t_tr(1)=t+1;
                v_0_re=y(t+1); %%velocità per il nuovo calcolo dell'energia durante il replanning 
                flag(t)=1;
        
                re_planner_provak_ok_sem
                tt_re_ottimizzazione=tt_ottimizzazione;
                conteggio=conteggio+1;
            end
        end
        

   
    else
        flag(t)=0;
    end
    tt_ottimizzazione=tt_ottimizzazione+T_camp;
end
% figure(10)
% Tempo_big=Tempo*(1/passo);
for t=1:length(Tempo)
    plot_ref(t)=x(2,t)+err(t);
end
figure(1)
plot(Tempo*(1/passo),x(1,:),'Color',[0, 0.4470, 0.7410],'Linewidth',2);

fig3 = figure(3);
set(fig3,'Position',[0 0 1.1*scrsz(3)/3 0.6*scrsz(4)/2]);
plot(Tempo,plot_ref,'Color',0.5*[1 1 1],'Linewidth',1)
hold on
plot(Tempo,y(1,:),'Color',[0, 0.4470, 0.7410],'Linewidth',1.5);
plot([0 200],[v_max v_max],'k','LineStyle','--','linewidth',0.9);
plot([0 200],[v_min v_min],'k','LineStyle','--','linewidth',0.9);
ylim([4 16])

grid on
box off

xlabel('Time, $t$ [sec]','Interpreter','Latex','Fontsize',14)
ylabel('Speed, $v$ [m s$^{-1}$]','Interpreter','Latex','Fontsize',14)
set(gca,'TickLabelInterpreter','latex','Fontsize',12);

ll = legend('$v$','$v^{\star}$');
set(ll,'Interpreter','Latex','Box','off','Location','best')


fig5 = figure(5);
set(fig5,'Position',[0 0 1.1*scrsz(3)/3 0.6*scrsz(4)/2]);
plot(Tempo,err(1,:),'Linewidth',1.5);
hold on
plot([0 200],[thr1 thr1],'k','LineStyle','--','linewidth',0.9);
plot([0 200],-[thr1 thr1],'k','LineStyle','--','linewidth',0.9);

grid on
box off
ylim([-1.25 1.25])

xlabel('Time, $t$ [sec]','Interpreter','Latex','Fontsize',14)
ylabel('Error, $e$ [m s$^{-1}$]','Interpreter','Latex','Fontsize',14)
set(gca,'TickLabelInterpreter','latex','Fontsize',12);


fig6 = figure(6);
set(fig6,'Position',[0 0 1.1*scrsz(3)/3 0.6*scrsz(4)/2]);
stem(Tempo,flag(:),'k')

grid on
box off
ylim([-0.5 1.5])

xlabel('Time, $t$ [sec]','Interpreter','Latex','Fontsize',14)
ylabel('Flag','Interpreter','Latex','Fontsize',14)
set(gca,'TickLabelInterpreter','latex','Fontsize',12);

%% INDICES
iRMS = rms(u)
RMSe = rms(err)
nflags = sum(flag)

%% SAVE PICTURES
ScriptForFigToPdfFun(fig1,'fig6',0)
ScriptForFigToPdfFun(fig3,'fig7',0)
ScriptForFigToPdfFun(fig5,'fig8',0)
ScriptForFigToPdfFun(fig6,'fig9',0)
