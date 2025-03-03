function [H]=ARMAX_RT(t,Power_w,Freq_w,Powerss)
%clear all; clear memory; close all;clc
% set(0,'DefaultAxesFontSize',24)
% set(0,'DefaultAxesFontName','Times New Roman')
% set(0,'DefaultAxesLineWidth',2)
% set(0,'defaultLineLineWidth',2)%1.25
%set(gca,'XTickLabel',[])
%%
ng=1;%size(Pgen,1);
 Fs=30;
 Ts=1/Fs;

powerb=Power_w; 
Fgen=double(Freq_w)/60;
for k=1:ng
Pd1=powerb(:,k); 
fd1=Fgen(:,k);
dP=(Pd1-Powerss);df=fd1-1;
dPr(:,k)=dP; dfr(:,k)=df;
end
clear k

kk=1;
Pr1=dPr;
fr1=dfr;
%%  ARMAX Estimation  
ARMAXBegin = tic;
% input = [];  output = []; 

    %Samples taken after the disturbance point
    input   = Pr1;
    output  = fr1;
   
    data  = iddata(output,input,Ts,'InterSample','foh');
    %data1=iddata(dP(pos_tkeo:end),df(pos_tkeo:end),Ts,'InterSample','foh');
    ord = 2;
    na = 2; nb = 2; nc = 2; nk =0; modelOrder = [];
    [set,ny,nu] = size(data);

    modelOrder     =   [na*ones(ny) nb*ones(ny,nu) nc*ones(ny,1) nk*ones(ny,nu)];
%     Opt = armaxOptions('InitialCondition','auto','Focus','prediction','Display','off');
    predictz       =   armax(data,modelOrder);    
    sys_report     =   predictz.Report;
    identModel     =   d2c(predictz,'tustin'); 
%     [A,B,C,D,Km]   =   idssdata(identModel);     % State-space model for the identified model
    sysD=ss(identModel);
    Ac=sysD.A;
    Bc=sysD.B; 
    Cc=sysD.C;
    Dc=sysD.D;
    [sysr,syse,gain] = dcgainmr(identModel,1);
  
%     Beta=sysr.B;

[num2,den2]    =   ss2tf(sysr.A,sysr.B,sysr.C,0);%sysr.D);  % Transfer function from state-space
  G2=tf(num2,den2);
  if size(num2)<2
        H_P (kk)=0;
     else
 % H=Hpst;
    Beta(kk)  =   num2(2);       
    H_P (kk)  =   abs(1/(2*Beta(kk))); 
       end
    
H =H_P';


