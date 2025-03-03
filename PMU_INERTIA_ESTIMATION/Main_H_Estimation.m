clear all
clear memory
close all
clc

ip = "10.0.8.221";
puerto = 4712;
idcode = 10;

[tcpobj,CFG2]=Link_PMU(ip,puerto,idcode);

%%

%% Continuously read, maximum of one million readings

phv = [];% stores the phasor values
freq = [];% stores the analog signal value
ndf = 1;
ind=1;
ind2=1;
Wind_size=30;
Ts=1/Wind_size;
t=0:Ts:2;
              
count=1;

while 1
    %tic
    if tcpobj.BytesAvailable>0
        nframes2read = 1;
%         fprintf('Leyendo %i data frames ...\n',nframes2read)
        %dataFrame = readDFnoDecode(tcpobj,CFG2, nframes2read);
        % --------------------------Start
conn=tcpobj;
confFrame2=CFG2;
numFramesALeer=nframes2read;
%%

numFramesLeidos=0;
dFrames= cell([1e6,0]);
while numFramesLeidos <numFramesALeer

    if conn.BytesAvailable > 0
        %fprintf('Numero de bytes pendientes de leer %i\n', conn.BytesAvailable )
        dFrame = read(conn);
       % dFrames{numFramesLeidos+1} = dFrame;      
        numFramesLeidos = numFramesLeidos+1;
    end
    
    %pause(eps);

end
        % -------------------------- End
%%
          [~,phv(ndf,:),Freq(ndf,1),~,~] = ...
           decodeDataFrameV2(dFrame,CFG2);
      
%          PF(ndf,1)= cos((pi/180)*phv(ndf,7) - (pi/180)*phv(ndf,10));
           %Power_M1(ndf,1)= 1.079*3*phv(ndf,1)*phv(ndf,4)*.99;      
           Power(ndf,1)= 1*(phv(ndf,1))*(phv(ndf,4))*1e-4;
%           Freq(ndf,1)=freq(ndf);
            df(ndf,1)=Freq(ndf,1)/60-1;
       
           %% Sliding Window and TKEO

%              Power_w=Power(ind:end);
%              Freq_w=Freq(ind:end);
%              ind=ind+1;      



  if ndf>=3
d=3; 
    %% Teager-Kaiser Operator Energy (Discrete time)
% for w=d:L-d
    x_n= df(1+ndf-3:d+ndf-3);
    psi_d(ndf) = x_n(2)^2-(x_n(1)*x_n(3));
    TKEO_d(ndf)= abs(psi_d(ndf));
         if TKEO_d(ndf)>= 1e-7
            pos_tkeo(ndf)=ndf;   %Position at which the energy change is detected
         end
          
         % INERTIA ESTIMATION
         
         if c1==30 && pos_tkeo(ndf)~=0
             ftk=find(pos_tkeo~=0);
        ind_tk=pos_tkeo(ftk(1))+7;
        Power_w=Power(ind_tk:ind_tk+Wind_size);
        Freq_w =Freq(ind_tk:ind_tk+Wind_size);
       [H]=ARMAX_RT(t,Power_w,Freq_w)
       
             c1=c1+1
        
         end
  end
ndf = ndf+ 1;

    end
end
