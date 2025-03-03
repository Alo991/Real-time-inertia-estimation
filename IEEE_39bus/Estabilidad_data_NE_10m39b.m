% m.file to simulate the 16 machine, 68 bus system
% using the Matlab Power System Toolbox
% % disp('This demo is a simulation of the WSCC 16 machine ')
% % disp('system subject to a load variation node 28 (bus 28), ')
% % disp('cleared in 3 cycles by returning to initial condition ')
% % 
% % disp('The simulation is performed using a')
% % disp('predictor-corrector method with a stepsize of 0.015 s.')
% % disp('Machine transient flux decay represented.')
clear all
clear global
clc

selected_input=1;        %selected the input to be modulated
selected_point=1;

frequency_points_number=5;    %frequency points
f_ini=0.1;               %first frequency point (Hz)
f_end=2;                 %last frequency point  (Hz)

f_step=(f_end-f_ini)/frequency_points_number;
frequency_points=f_ini:f_step:f_end;
frequency_point=frequency_points(selected_point)*(2*pi);

pst_var % set up global variable
global  basmva basrad syn_ref mach_ref sys_freq Mod
global  bus_v bus_ang psi_re psi_im cur_re cur_im bus_int

% synchronous machine variables
global  mac_con mac_pot mac_int busnum
global  mac_ang mac_spd eqprime edprime 
global  curd curq curdg curqg fldcur
global  psidpp psiqpp vex eterm theta ed eq 
global  pmech pelect qelect pij
global  dmac_ang dmac_spd deqprime dedprime mac_em_idx H_eq  exc_con
jay = sqrt(-1);


% %Modelo de las maquinas con la numeraci?n original
% mac_em_idx = [8; 11; 12; 13; 14; 20; 21; 22; 23; 28; 29; 41; 43]; %M?quinas con modelo cl?sico
% Mod=[3 3 3 4 4 3 4 2 4 4 2 2 2 2 3 4 3 3 4 2 2 2 2 4 4 4 4 2 2 4 4 4 4 4 ...
%     4 4 4 4 4 4 2 3 2 3 3 4];
% %Modelo de las maquinas cambiando el nodo 39 por el nodo 1
mac_em_idx = [1 2 3 4 5 6 7 8 9 10]'; %M?quinas con modelo cl?sico
Mod = [4 4 4 4 4 4 4 4 4 4];

% load NE_39n_prony_data
load NE_completo_39n
bus = bus_sol;
%  mac_con(:,17) = 0.01*ones(46,1);
mac_con(:,17) = mac_con(:,16);
mac_con(9,17) = 5*mac_con(9,16);
% exc_con = 0;
basrad = 2*pi*60; % system frequency is 60 Hz
basmva = 100;     % 100 MVA base
syn_ref = 0 ;     % synchronous reference frame

disp(' ')
disp('Performing simulation.')

% simulation
t_switch(1) = 0;     % all time in second+s, start time
t_switch(2) = 0.08;  % time to apply fault
t_switch(3) = 0.13; % time to clear fault, 3 cycles
t_switch(4) = 10;   % end time.5
h =0.0001; %1/(2*1920);%1/240; % integration stepsize
Ts = h;
stepsize = h;
k_switch(1) = round((t_switch(2)-t_switch(1))/stepsize)+1;
k_switch(2) = round((t_switch(3)-t_switch(1))/stepsize)+1;
k_switch(3) = round((t_switch(4)-t_switch(1))/stepsize)+1;

Ng = length(mac_con(:,1));
% step 1: construct reduced Y matrix - all loads are assumed
%         to be of constant impedance load
[Y_red,V_rec] = red_ybus(bus,line);             % pre-fault 
					 % admittance matrix
% create bus matrix with load increased on node 28
bus_f = bus;
%bus_f(19,6) = 5.0; % pulso de carga sistema original
%bus_f(19,7) = 2.0;
bus_f(30,6:7) =  bus(30,6:7)*3; %OJO USE 3 PARA M1 Y 6 PARA M2;
%bus_f(16,9) = -150;
 %bus_f(:,6:7)=bus(:,6:7)*1.15;%Falla vari
%  bus_f(:,6:7)=bus(:,6:7)*1.30;%Falla vari

[Y_red_f,V_rec_f] = red_ybus(bus_f,line);   % fault-on admittance matrix
                ang = bus(:,3)*pi/180;
                  bus_v = bus(:,2).*(cos(ang)+jay*sin(ang));
% step 2: initialization
flag = 0;
  for klm = 1:Ng,
    if Mod(klm) == 4 || Mod(klm) == 3,
      mac_tra(klm,1,bus,flag);    % machine model
    elseif Mod(klm) == 2,
      mac_em(klm,1,bus,flag);    % machine model
    end
  end


%  mac_tra(0,1,bus,flag);    % machine model
  smpexc(0,1,bus,flag,h);    % exciter mode

  %&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
%PARA CALCULAR TODAS LAS POTENCIAS
[n_lin coll]=size(line); 
pijv=zeros(n_lin,1);
for linn=1:n_lin
    nsend_V=line(linn,1);
    nfin_V=line(linn,2);
    [S1v,S2v] = line_pq(bus_v(nsend_V,1),bus_v(nfin_V,1),line(linn,3),line(linn,4),line(linn,5),1.0,0.0);
    pijv(linn,1) = real(S1v);  
end
%&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
  
mach_ref=0;

y0=[mac_ang; mac_spd; edprime; eqprime; Efd];
stepsize=1/240;
t_final=1;
t = [0:stepsize:t_final]'; % time

curterm=zeros(10,1);
a_eterm=zeros(10,1);
 indun=1
 
%  noisesim=Noise.signals(1).values;
%  save datanoise20.mat noisesim Ts
%% Datos PRONY
% F1 = 0.4;    
% Fs = 60;
% H = floor(round(Fs/F1)/2);
% N = 2*H;
% N1 = 2*H + 1;
% tao = 1/Fs;
% K = 2;
% orden = zeros(N-2*K, 2*K);
% orden2 = zeros(N-2*K, 1);
% long = 2*K + 1 : N;
% longs = length(long);
% for m = 1 : longs
%     for k = 1 : 2*K
%         orden(m,k) = 2*K - k + m;
%     end
%     orden2(m,1) = 2*K + m;
% end
% pot = -N/2:N/2;