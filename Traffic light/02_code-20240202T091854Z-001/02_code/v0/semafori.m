close all 
clear all
clc

n=6; 
T=30; %[s]
T_gr=10.8;
tf=200; %[s]
teta=[13,3,28,15,5]; %tempo di offset tra i semafori 
passo=input('please insert the step deltaT to be considered: ');


v_min=5; %[m/s] %parametro da modificare in futuro 
v_max=14; %[m/s]

%condizioni iniziali 

d(1)=0; 
t(1)=passo;
ti_min(1)=0;
ti_max(1)=0;


%faccio partire i cicli dal valore i=2 poichè ho imposto l'indice i=1 come condizione inziale.

%costruisco un vettore d contenente le distanze delle intersezioni 
rng('default')

for j=2:n+1
    if j~=n
        d(j)=d(j-1)+300;
    end
    if j==n
        d(j)=d(j-1)+350;
    end
    if j==n+1
        d(j)=d(j-1)+450;
    end
end



cicli=floor(tf/T); %indica il numero di cicli rosso/verde

%in questo secondo passaggio costruisco una matrice in cui le righe
%rappresentano i vari incroci eindico con s lo stato del semaforo 
%s=1 indica che il semaforo è verde e s=0 indica invece che il semaforo è
%rosso.
% s=zeros(n-1,((tf)/(passo)));
time=passo:passo:tf;
nn=size(time);
l=nn(2);
s=zeros(n-1,l);
for i=1:n-1
    if teta(i)<(T-T_gr)  %
       for z=0:cicli
%  z=3;
            for t=passo:passo:tf
                if ((t-(teta(i)))>(z*T) && (t-(teta(i)))<=(z*T+T_gr))
                    tt=find(abs(time-t)<passo*0.1);
                    s(i,tt)=1;
%                 elseif (((t-(teta(i)-1))>(z*T+T_gr)) && (t-(teta(i)-1)<=(z+1)*T))
%                     tt=find(abs(time-t)<0.001);
%                     s(i,tt)=0;
                end
            end
       end
    else
        for z=0:cicli
% z=6;
            for t=passo:passo:tf
                if t<=((teta(i))-(T-T_gr))&& z==0
                    tt=find(abs(time-t)<passo*0.1);
                    s(i,tt)=1;
                    elseif ((t-(teta(i)))>(z*T) && (t-(teta(i)))<=(z*T+T_gr))
                        tt=find(abs(time-t)<passo*0.1);
                        s(i,tt)=1;
%                         elseif (((t-(teta(i)-1))>(z*T+T_gr)) && (t-(teta(i)-1)<=(z+1)*T))
%                             tt=find(abs(time-t)<0.001);
%                             s(i,tt)=0;
                end
            end
        end
    end
end
s;
% 
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
% 

% 
% 
pruning_offline
%map