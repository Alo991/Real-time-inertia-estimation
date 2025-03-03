function [tcpobj,CFG2]=Link_PMU(ip,puerto,idcode)

global endByte_PHEst endByte_DFREQEst

%% Describe PMU connetion data. 
%  ip = "192.168.1.2";
%  puerto = 4714;
%  idcode = 19;
 %-------------------------------------------------------------------
 %% 
 %% To connect PMU 
field1_1 = 170;
field1_2 = 65; 
%2 FRAMESIZE
field2_1 =0 ; 
field2_2=18;
% 3 field IDCODE

field3_1 = bitshift(idcode,-8);
field3_2 = bitand(idcode,255);
%4 SOC
field4_1 =0; 
field4_2 =0;
field4_3 =0;
field4_4 =0;
%5 FRACSEC
field5_1 =0; 
field5_2 =0;
field5_3 =0;
field5_4 =0;
%6 CMD
field6_1 = 0; 
field6_2= 5; 
%7 EXTFRAME 
%8 CHK

messnoCHK = [field1_1 , field1_2,...
    field2_1,field2_2,...
    field3_1, field3_2 ...
    field4_1, field4_2, field4_3, field4_4...
    field5_1, field5_2, field5_3, field5_4...
    field6_1, field6_2];

[crc1,crc2] = getCRC(messnoCHK);

message = [messnoCHK, crc1, crc2];

tcpobj = tcpclient(ip,puerto, 'ConnectTimeout', 10);
write(tcpobj,uint8(message));

pause(2)% For the PMU to be able to write and return the information.
dFrame = read(tcpobj);
 %-----------------------------end function
 %%
 %% To identify the PMU configuration
% -------------------------------Start function

%dframe: 
dframe=dFrame;
%returns:
%cellPhasors: MATLAB cell object that stores the decoded phasor information
%CFG2: MATLAB struct object in CFG2 format
%1 SYNC 2 bytes
%First byte: AA hex
%Second byte: 21 hex for configuration 1
%31 hex for configuration 2
%Both frames are version 1 (IEEE Std C37.118-2005 [B6])
rdSYNC = dframe(1:2);
%Take bytes 7, 6, 5 which indicate the frame type (starts at 1, not 0)
frameType=  bin2dec([num2str(bitget(rdSYNC(2),7))...
        num2str(bitget(rdSYNC(2),6))...
        num2str(bitget(rdSYNC(2),5))]);

    switch frameType
        case 0
            strFrameType='Data frame';
        case 1
            strFrameType='Configuration frame 1';
        case 3
            strFrameType='Configuration frame 2';
        case 5
            strFrameType='Configuration frame 3';
        case 4
            strFrameType='Command frame';
    end
protocolVersion = bin2dec([num2str(bitget(rdSYNC(2),4))...
        num2str(bitget(rdSYNC(2),3))...
        num2str(bitget(rdSYNC(2),2))...
        num2str(bitget(rdSYNC(2),1))]);
   
   
%% 2 FRAMESIZE 2 bytes
rdFRAMESIZE = dframe(3:4);
%This is the number of bytes received in decimal formats en formato decimal
decFRAMESIZE = typecast([rdFRAMESIZE(2) rdFRAMESIZE(1)],'uint16');

%% 3 IDCODE 2 bytes
rdIDCODE = dframe(5:6);
decIDCODE = typecast([rdIDCODE(2) rdIDCODE(1)],'uint16');

%% 4 SOC 4 bytes
rdSOC = dframe(7:10);
decSOC = typecast([rdSOC(4) rdSOC(3) rdSOC(2) rdSOC(1)],'uint32');

%% 5 FRACSEC 4 bytes
rdFRACSEC = dframe(11:14);
%time quality flags
MSG_TQ = rdFRACSEC(1);
decFRACSEC = typecast( [0 rdFRACSEC(3) rdFRACSEC(2) rdFRACSEC(1)],'uint32');

%% 6 TIME_BASE 4 bytes
rdTIME_BASE = dframe(15:18);
decTIME_BASE = typecast(...
    [ uint8(rdTIME_BASE(2))...
    uint8(rdTIME_BASE(3))...
    uint8(rdTIME_BASE(4))...
    0],...
    'uint32');

%% 7 NUM_PMU 2 bytes 
rdNUM_PMU = dframe(19:20);
decNUM_PMU = typecast( [rdNUM_PMU(2) rdNUM_PMU(1)],'uint16');

%% 8 STN 16 bytes 
rdSTN = dframe(21:36);
strSTN = char(rdSTN);

%% 9 IDCODE 2 bytes 
rdIDCODEsrc = dframe(37:38);
decIDCODEsrc = typecast([rdIDCODEsrc(2) rdIDCODEsrc(1)],'uint16');

%% 10 FORMAT 2 bytes  
% bit 15-4 not used
% bit 3 number type for presenting FREQ and/or DFREQ
% 0 Fixed with 16-bit integer
% 1 32-bit floating point IEEE format
% bit 2 number type for presenting analog values
% 0 16-bit integer (requires conversion factor)
% 1 32-bit floating point
% bit 1 number type for presenting phasors
% 0 16-bit integer
% 1 32-bit floating point
% bit 0 format in which phasors are presented
% 0 Rectangular
% 1 Polar

rdFORMAT = dframe(39:40);
repFREQ = bitget(rdFORMAT(2),4); %Frequency representation
repANALOG = bitget(rdFORMAT(2),3);
repPHASORS = bitget(rdFORMAT(2),2);
repPHASORSpol = bitget(rdFORMAT(2),1); %Polar or rectangular form

if repFREQ == 0 
    strRepFREQ = 'Fijo';
else
    strRepFREQ='Punto flotante';
end

if repANALOG == 0
    strRepANALOG= 'Entero';
else
    strRepANALOG='Punto flotante';
end

if repPHASORS == 0
    strRepPhASORS = 'Entero';
else
    strRepPhASORS = 'Punto flotante';
end

if repPHASORSpol == 0
    strRepPHASORSpol='Rectangular';
else
    strRepPHASORSpol = 'Polar';
end

%% 11 PHNMR 2 bytes number of phasors that will be received in the frame
rdPHNMR = dframe(41:42);
% Two bytes are read, but it is actually a single number. With typecast, the two bytes are combined to create a 16-bit number
decPHNMR = typecast([rdPHNMR(2) rdPHNMR(1)],'uint16');

%% 12 ANNMR 2 bytes Number of analog values that will be received in the dframe
rdANNMR = dframe(43:44);
decANNMR = typecast([rdANNMR(2) rdANNMR(1)],'uint16');

%% 13 DGNMR 2 bytes Number of digital inputs that will be received in the dframe
rdDGNMR = dframe(45:46);
decDGNMR = typecast([rdDGNMR(2) rdDGNMR(1)],'uint16');

%% 14 CHNAM Phasor names and flags (defined in the PMU programming)
% 1 ASCII character per byte
% N = number of phasors + number of analogs + 16 × number of digital inputs
% Converting from 2 bytes to a decimal number
N_CHNAM = 16*(decPHNMR + decANNMR + (16*decDGNMR));
endbyte_CHNAM = 46 + N_CHNAM; 
rdCHNAM = dframe(47:endbyte_CHNAM);
strCHNAM = char(zeros(decPHNMR,16)); % Matrix for 16-byte names
for i = 1:decPHNMR
   inicio = (16*(i-1)+1);
   fin = inicio+15;
   strCHNAM(i,:) = char(rdCHNAM(inicio:fin) );
end
%% 15 PHUNIT 4x PHNMR conversion factor of phasors
% Bits 31-24 (most significant byte):
% (00): Voltage
% (01): Current
% Bits 23-0 (3 least significant bytes):
% Required conversion factor in “FORMAT”
% Unsigned 24-bit integer, multiplied by a factor of 105.
endbyte_rdPHUNIT= endbyte_CHNAM + (4*decPHNMR);
rdPHUNIT = dframe(endbyte_CHNAM+1:endbyte_rdPHUNIT);
%decoding 
arrPHUNIT = zeros(decPHNMR,4); 
for i = 1:decPHNMR
   inicio = (4*(i-1)+1);
   fin = inicio+3;
   arrPHUNIT(i,:) = rdPHUNIT(inicio:fin);
end
facConv = double(typecast(...
        [uint8(arrPHUNIT(i,4))...
        uint8(arrPHUNIT(i,3))...
        uint8(arrPHUNIT(i,2)), ...
        0],...
        'uint32')) / 10^5;

%% 16 ANUNIT 4X ANNMR 
endbyte_ANUNIT = endbyte_rdPHUNIT + (4*decANNMR);
rdANUNIT = dframe(endbyte_rdPHUNIT+1:endbyte_ANUNIT);
%decoding
arrANUNIT = zeros(decANNMR,4); 
for i = 1:decANNMR
   inicio = (4*(i-1)+1);
   fin = inicio+3;
   arrANUNIT(i,:) = rdANUNIT(inicio:fin);
end

%% 17 DIGUNIT 4xDGNMR
%Masks for digital inputs (user-defined)
%Suggested use:
%The 2 most significant bytes can be the default state of the inputs
%The 2 least significant bytes can be the current state of the inputs
endbyte_DIGUNIT = endbyte_ANUNIT + (4*decDGNMR);
rdDIGUNIT = dframe(endbyte_ANUNIT+1:endbyte_DIGUNIT);
%decoding
arrDIGUNIT = zeros(decDGNMR,4); 
for i = 1:decDGNMR
   inicio = (4*(i-1)+1);
   fin = inicio+3;
   arrDIGUNIT(i,:) = rdDIGUNIT(inicio:fin);
end

%18 FNOM 2 bytes
% Nominal Frequency
%Bits 15-1: Reserved (default value 0)
%Bit 0:
%0= FNOM 60 [Hz]
%1= FNOM 50 [Hz]
endbyte_rdFNOM = endbyte_DIGUNIT+2;
rdFNOM = dframe(endbyte_DIGUNIT+1:endbyte_rdFNOM);
if bitget(rdFNOM(1),1) ==0
    valFNOM = 60;
else
    valFNOM =50;
end

%% 19 CFGCNT 2 bytes 
endbyte_CFGCNT = endbyte_rdFNOM+2;
rdCFGCNT = dframe(endbyte_rdFNOM+1:endbyte_CFGCNT);
decCFGCNT= typecast([rdCFGCNT(2) rdCFGCNT(1)], 'uint16');

%% DATA_RATE 2 bytes 
%Rate of phasor data transmissions?2-byte integer word (–32 767 to +32 767)
%If DATA_RATE > 0, rate is number of frames per second.
%If DATA_RATE < 0, dframe(endbyte_CFGNCT+1:end);rate is negative of seconds per frame.
%E.g., DATA_RATE = 15 is 15 frames per second; DATA_RATE = –5 is 1 frame per
%5 s
endbyte_DATA_RATE = endbyte_CFGCNT +2;
rdDATA_RATE = dframe(endbyte_CFGCNT+1:endbyte_DATA_RATE);
decDATA_RATE = typecast([rdDATA_RATE(2) rdDATA_RATE(1)],'int16');
% if decDATA_RATE > 0
%     strDATA_RATE = sprintf('%d frames por segundo\n',decDATA_RATE);
% else
%     strDATA_RATE = sprinft('1 frame por cada %d segundos\n',abs(decDATA_RATE));
% end
%% Ultimo CHK 2 bytes
rdCHK = dframe(endbyte_DATA_RATE+1:end);
decCHK = typecast([rdCHK(2) rdCHK(1)],'uint16');%
strCHK = sprintf('%X',decCHK);
%% Structure
CFG2 = struct(...
    'FRAMETYPE',strFrameType,...
    'VERSION',protocolVersion,...
    'FRAMESIZE',decFRAMESIZE,...
    'IDCODE',decIDCODE,...
    'SOC',decSOC,...
    'MSG_TQ',MSG_TQ,...
    'FRACSEC',decFRACSEC,...
    'TIME_BASE',decTIME_BASE,...
    'NUM_PMU',decNUM_PMU,...
    'STN_NAME',strSTN,...
    'IDCODE_SRC',decIDCODEsrc,...
    'FREQ_REP',repFREQ,...
    'AN_REP',repANALOG,...
    'PH_REP',repPHASORS,...
    'PH_POL_REP',repPHASORSpol,...
    'PH_NUMBER',decPHNMR,...
    'AN_NUMBER',decANNMR,...
    'DG_NUMBER',decDGNMR,...
    'FACCONV', facConv,...
    'FREQ_NOM',valFNOM,...
    'CFGCNT',decCFGCNT,...
    'DATA_RATE',decDATA_RATE,...
    'CHK',strCHK);

%% Create a cell array with phasor data
cellPhasors =cell(decPHNMR,4);
for i =1:decPHNMR
    txtname= ['PH',num2str(i)];
    if arrPHUNIT(i,1) == 0 
        txtype ='Tension';
    else
        txtype='Corriente';
    end
    cellPhasors(i,:)= {txtname,strCHNAM(i,:),txtype,facConv};
end
%---------------------------------End Function
stParam=CFG2;
                if stParam.FREQ_REP == 0
                
                    stParam.FREQ_REP = 'Fijo';
                
                else
                
                    stParam.FREQ_REP = 'Punto flotante';
                
                end
                
                if stParam.AN_REP ==0
                
                    stParam.AN_REP = 'Entero';
               
                else
                
                    stParam.AN_REP = 'Punto flotante';
               
                end
                
                if stParam.PH_REP == 0
                
                    stParam.PH_REP = 'Entero';
               
                else
                
                    stParam.PH_REP = 'Punto flotante';
                
                end
                
                if stParam.PH_POL_REP == 0
                
                    stParam.PH_POL_REP = 'Rectangular';
                
                else
                
                    stParam.PH_POL_REP = 'Polar';
               
                end
                
                if stParam.DATA_RATE > 0
                
                    stParam.DATA_RATE = sprintf('%d frames por segundo', stParam.DATA_RATE);
               
                else
                
                    stParam.DATA_RATE = sprinft('1 frame por cada %d segundos',abs( stParam.DATA_RATE));
                
                end
%%
%%
%fprintf('Enviando cmd 2 (Turn on transmission of data frames)...\n')
cmd=2;
% sendCMD(cmd,idcode,tcpobj);
IDCODE=idcode;
CMD=cmd;
%---------------------------Start Function
%% 1 SYNC 2 bytes
%%Sync byte followed by frame type and version number (AA41 hex).
%Primer byte: AA sync byte
%Seguno byte: 41 0100 0001
%bit7 =reservado(0), bit 6-4:100 command frame, bits 3-0: 0001 version 1
CMD_SYNC_1 = 170; %0xAA
CMD_SYNC_2 =  65; %0x41

%% FRAMESIZE 2 bytes

CMD_FRAMESIZE_1 = 0;
CMD_FRAMESIZE_2 = 18;

%% IDCODE 2 bytes

CMD_IDCODE_1 = bitshift(IDCODE,-8);
CMD_IDCODE_2 =  bitand(IDCODE,255);

%% SOC 4 bytes
CMD_SOC_1 =00;
CMD_SOC_2=00;
CMD_SOC_3=00;
CMD_SOC_4=00;

%% FRACSEC 4 bytes
CMD_FRACSEC_1  = 0; 
CMD_FRACSEC_2  =0; 
CMD_FRACSEC_3  =0;
CMD_FRACSEC_4  =0;

%% EXTFRAME 
%% CMD 2 BYTES 
% Command word bits Definition
% Bits 15–0:
% 0000 0000 0000 0001 Turn off transmission of data frames.
% 0000 0000 0000 0010 Turn on transmission of data frames.
% 0000 0000 0000 0011 Send HDR frame.
% 0000 0000 0000 0100 Send CFG-1 frame.
% 0000 0000 0000 0101 Send CFG-2 frame.
% 0000 0000 0000 0110 Send CFG-3 frame (optional command).
% 0000 0000 0000 1000 Extended frame.
% 0000 0000 xxxx xxxx All undesignated codes reserved.
% 0000 yyyy xxxx xxxx All codes where yyyy ? 0 available for user designation.
% zzzz xxxx xxxx xxxx All codes where zzzz ? 0 reserved.
CMD_CMD_1 = 0;
CMD_CMD_2 = CMD;

%% CHK 

messnoCHK = [CMD_SYNC_1 , CMD_SYNC_2...
    CMD_FRAMESIZE_1, CMD_FRAMESIZE_2...
    CMD_IDCODE_1, CMD_IDCODE_2...
    CMD_SOC_1,CMD_SOC_2,CMD_SOC_3,CMD_SOC_4,...
    CMD_FRACSEC_1, CMD_FRACSEC_2, CMD_FRACSEC_3, CMD_FRACSEC_4, ...
    CMD_CMD_1, CMD_CMD_2];

[crc1,crc2] = getCRC(messnoCHK);
message = [messnoCHK, crc1, crc2];
write(tcpobj,uint8(message))
pause(1)
%dFrame = read(tcpobj);
%------------------------------- End Function
%% Clear buffer
%Checks if there are bytes pending to be read in the socket buffer
%If there are, it reads them
%This is just to clear the buffer and start reading data frames
%from scratch
if tcpobj.BytesAvailable > 0
   read(tcpobj);
end
%%
%Number of frames to read
nframes2read = 1;
% fprintf('Leyendo %i data frames...\n',nframes2read)
% dataFramesDecod = readDFnoDecode(tcpobj,CFG2, nframes2read);
%------------------ Start Function
%conn: objeto tcpclient
conn=tcpobj;
confFrame2=CFG2;
numFramesALeer=nframes2read;
%%

numFramesLeidos=0;
dFrames= cell([1e6,0]);
while numFramesLeidos <numFramesALeer

    if conn.BytesAvailable > 0
               dFrame = read(conn);
%         rdFRAMESIZE = dFrame(3:4);
%         %Este el número de bytes recibidos en formato decimal
%         decFRAMESIZE = typecast([rdFRAMESIZE(2) rdFRAMESIZE(1)],'uint16')
        dFrames{numFramesLeidos+1} = dFrame;      
        numFramesLeidos = numFramesLeidos+1;
    end
    
    pause(eps);

end
%-------------------End Funtion
%%
%% decodificando dataframe
for i=1 : nframes2read
%     dataframeDecod =decodeDataFrameV2(dataFramesDecod{i},CFG2);
%%% ----------------------Start Function

%dFrame: dataframe read from the PMU.
%repPH, repFreq, repAN, etc.They are the data that come in the CGF2 and are necessary to decode
%the dataframe. Maybe the CFG2 structure is used here.

%returns:
%decodedDF: struct object containing the decoded dataframe data
%PHest

%repPH indicates if the variable is 16 bits or 32 bits
%repFREQ integer or floating point
%PHNMR number of phasors
%ANNMR number of analog inputs
%DGNMR number of digital inputs
%facConv conversion factor for the phasors
%repPHpol polar or rectangular representation of the phasors
%repAn integer or floating point values for the analog inputs

repPH = confFrame2.PH_REP;
repFREQ = confFrame2.FREQ_REP;
PHNMR = confFrame2.PH_NUMBER;
ANNMR = confFrame2.AN_NUMBER;
DGNMR = confFrame2.DG_NUMBER;
repPHpol = confFrame2.PH_POL_REP;
facConv = 1;%confFrame2.FACCONV;
repAN = confFrame2.AN_REP;
timeBase = confFrame2.TIME_BASE;



DIGBits=0;

%1 SYNC 2 bytes
%First byte: AA hex
%Second byte: 21 hex for configuration 1
%31 hex for configuration 2
%Both frames are version 1 (IEEE Std C37.118-2005 [B6])
rdSYNC = dFrame(1:2);
frameType=  bin2dec([num2str(bitget(rdSYNC(2),7))...
        num2str(bitget(rdSYNC(2),6))...
        num2str(bitget(rdSYNC(2),5))]);
    
    switch frameType
        case 0
            strFrameType='Data frame';
        case 1
            strFrameType='Configuration frame 1';
        case 3
            strFrameType='Configuration frame 2';
        case 5
            strFrameType='Configuration frame 3';
        case 4
            strFrameType='Command frame';
    end
protocolVersion = bin2dec([num2str(bitget(rdSYNC(2),4))...
        num2str(bitget(rdSYNC(2),3))...
        num2str(bitget(rdSYNC(2),2))...
        num2str(bitget(rdSYNC(2),1))]);
   
%% 2 FRAMESIZE 2 bytes
rdFRAMESIZE = dFrame(3:4);
%This is the number of bytes received in decimal format
decFRAMESIZE = typecast([rdFRAMESIZE(2) rdFRAMESIZE(1)],'uint16');

%% 3 IDCODE 2 bytes
rdIDCODE = dFrame(5:6);
decIDCODE = typecast([rdIDCODE(2) rdIDCODE(1)],'uint16');

%% 4 SOC 4 bytes
rdSOC = dFrame(7:10);
decSOC = typecast([rdSOC(4) rdSOC(3) rdSOC(2) rdSOC(1)],'uint32');

%% 5 FRACSEC 4 bytes
rdFRACSEC = dFrame(11:14);
%time quality flags
MSG_TQ = rdFRACSEC(1); 
decFRACSEC = typecast( [ rdFRACSEC(3) rdFRACSEC(2) rdFRACSEC(1) 0],'uint32');
%% STAT
rdSTAT=typecast(dFrame(15:16),'uint16');
% Bits 03–00: Trigger reason:
% 1111–1000: Available for user definition
% 0111: Digital 0110: Reserved
% 0101: df/dt High 0100: Frequency high or low
% 0011: Phase angle diff 0010: Magnitude high
% 0001: Magnitude low 0000: Manual
rdTriggerRsn = bitand(rdSTAT,15);%To get only bits 03-00 (00001111)

% Bits 05–04: Unlocked time: 00 = sync locked or unlocked < 10 s (best quality)
% \01 = 10 s ? unlocked time < 100 s
% 10 = 100 s < unlock time ? 1000 s
% 11 = unlocked time > 1000 s
rdUnlockedTime = bitshift(bitand(rdSTAT,48),-4);% (0011 0000)

%Bits 08–06: PMU Time Quality. Refer to codes in Table 7.
rdPMUTQ =bitshift(bitand(rdSTAT,448),-6); %0000 0001 1100 0000

% Bit 09: Data modified, 1 if data modified by post processing, 0 otherwise
rdDataMod = bitget(rdSTAT,10);

%Bit 10: Configuration change, set to 1 for 1 min to advise configuration will change, and
%clear to 0 when change effected.
rdConfChng = bitget(rdSTAT,11);

%Bit 11: PMU trigger detected, 0 when no trigger
rdPMUTrigg = bitget(rdSTAT,12);

%Bit 12: Data sorting, 0 by time stamp, 1 by arrival
rdDataSort = bitget(rdSTAT,13);

%Bit 13: PMU sync, 0 when in sync with a UTC traceable time source
rdPMUSync = bitget(rdSTAT,14);

%Bit 15–14: Data error:
%00 = good measurement data, no errors
%01 = PMU error. No information about data
%10 = PMU in test mode (do not use values) or absent data tags have been inserted (do not use values)
%11 = PMU error (do not use values)
rdDataError =  bitshift(bitand(rdSTAT,49152),-14);% 1100 0000 0000 0000
%% 7 PHASORS ESTIMATES 

PHEst = zeros(1,PHNMR); %Number of phasors, magnitude, and angle
if repPH ==0 %16-bit integer
    
    nBytes = 4;%4 bytes 16 bits for phase and 16 bits for angle or real and imaginary
    endByte_PHEst = 17+(nBytes*PHNMR)-1;
    fasores = dFrame (17:endByte_PHEst);
    byte1 = typecast([fasores(2) fasores(1)],'int16');
    byte2  = typecast([fasores(4) fasores(3)],'int16');
    
    if repPHpol==0 %Rectangular representation
        
        for i = 1:PHNMR
            
            PHEst(1,i) = abs(byte1 + byte2) *facConv;%magniutde
            PHEst(1,6+i) = rad2deg(angle(byte1 + byte2)*facConv)/10^4;%Phase
            
        end
      
    else %Polar representation
        
        for i = 1:PHNMR
            
            inicio = nBytes*(i-1)+1;
            fin = inicio+nBytes-1;
            fnbytes = fasores(inicio:fin);
            byte1 = typecast([fnbytes(2) fnbytes(1)],'uint16');
            byte2  = typecast([fnbytes(4) fnbytes(3)],'int16');
            PHEst(1,i) = double(byte1) * facConv; %Magnitude
            PHEst(1,6+i) = rad2deg(double(byte2) /10^4); %angle
        end
        
    end
    
else %32-bit floating point
    for i = 1:PHNMR
    
        nBytes = 8;%8bytes 32bits y 32bits
        endByte_PHEst = 17+(nBytes*PHNMR)-1;
        fasores = dFrame(17:endByte_PHEst);
        inicio = nBytes*(i-1)+1;
        fin = inicio+nBytes-1;
        fnbytes = fasores(inicio:fin);
        bytesMag = uint8([fnbytes(4) fnbytes(3) fnbytes(2) fnbytes(1)]);
        phMagnitude = typecast(bytesMag,'single');
        bytesAng = uint8([fnbytes(8) fnbytes(7) fnbytes(6) fnbytes(5)]);
        phAngle = typecast(bytesAng,'single');
        PHEst(1,i) = phMagnitude * facConv; %Magnitude
        PHEst(1,6+i) = rad2deg(phAngle); %angle
    end
end

%% 8 FREQ Frequency estimation
%% NOTE: Check if it's not f(1) f(2) in the typecast
%The FREQDev is in mHz
if repFREQ ==0 %16-bit integer
    nBytes = 2;
    f = dFrame(endByte_PHEst+1:endByte_PHEst+nBytes);
    FREQEst= double(typecast([ f(2), f(1)],'int16'))/1000; 

else
    nBytes = 4;%32-bit floating point
   f = dFrame(endByte_PHEst+1:endByte_PHEst+nBytes);
    bytesFreq = uint8([f(4) f(3) f(2) f(1)]);
    FREQEst = typecast(bytesFreq,'single');
    
end
endByte_FREQDev = endByte_PHEst + nBytes; 

%% 9 DFREQ 2/4 ROCOF fixed-point or floating-point
% ROCOF, in hertz per second times 100 ( 1000)?
% Range –327.67 to +327.67 Hz per second
if repFREQ ==0 %16-bit integer
    nBytes = 2;
    devf = dFrame(endByte_FREQDev+1:endByte_FREQDev+nBytes);
    DFREQEst= typecast([ devf(2), devf(1)],'int16'); 

else
    nBytes = 4;%32-bit floating point
    devf = dFrame(endByte_FREQDev+1:endByte_FREQDev+nBytes);
    bytesDFREQ = uint8([devf(4) devf(3) devf(2) devf(1)]);
    DFREQEst = typecast(bytesDFREQ,'single');
end
endByte_DFREQEst = endByte_FREQDev + nBytes;

%% 10 ANALOG 
%Analog data, 2 or 4 bytes per value depending on fixed or floating-point
%format used, as indicated by the FORMAT field in configuration 1, 2, and
%3 frames. The number of values is determined by the ANNMR field in
%configuration 1, 2, and 3 frames.
    if repAN ==0 %16-bit integer
        nBytes = 4;
    else
        nBytes = 8;%32-bit floating point
    end
    endByte_ANEst = endByte_DFREQEst+(nBytes*ANNMR)
    ANALOGS = dFrame (endByte_DFREQEst+1:endByte_ANEst);
    
    ANEst = zeros(ANNMR,nBytes); %Number of phasors x number of bits
    for i = 1:ANNMR
        inicio = (nBytes*(i-1)+1);
        fin = inicio+nBytes-1;
        ANEst(i,:) = ANALOGS(inicio:fin); %Phasor estimation
    end

%% 11 Digital 2 × DGNMR
%Digital data, usually representing 16 digital status points (channels). The
%number of values is determined by the DGNMR field in configuration 1,
%2, and 3 frames.

    nBytes = DGNMR*2;

    endByte_DIGITAL = endByte_ANEst+nBytes;
    DIGITAL = dFrame(endByte_ANEst+1:endByte_DIGITAL);
    for i = 1:DGNMR
        inicio = (nBytes*(i-1)+1);
        fin = inicio+nBytes-1;
        DIGBits(i,:) = DIGITAL(inicio:fin);
    end

%% 12   CHK
rdCHK = dFrame(endByte_DIGITAL+1:end);
decCHK = typecast([rdCHK(2) rdCHK(1)],'uint16');%
strCHK = sprintf('%X',decCHK);

%% Calculation of the measurement date
tmPosix = decSOC + (decFRACSEC/timeBase);
measDate = datetime(tmPosix, 'ConvertFrom', 'posixtime');
%% Create a structure with all the read data
decodedDF = struct(...
    'FRAMETYPE',strFrameType,...
    'VERSION',protocolVersion,...
    'FRAMESIZE',decFRAMESIZE,...
    'IDCODE',decIDCODE,...
    'SOC',decSOC,...
    'MSG_TQ',MSG_TQ,...
    'FRACSEC',decFRACSEC,...
    'TRIGRSN',rdTriggerRsn,...
    'UNLCKTIME',rdUnlockedTime,...
    'PMUTQ',rdPMUTQ,...
    'DATAMOD',rdDataMod,...
    'CFGCHNG',rdConfChng,...
    'PMUTRG',rdPMUTrigg,...
    'DATASRT',rdDataSort,...
    'PMUSYNC',rdPMUSync,...
    'DATAERR',rdDataError,...
    'FREQEST',FREQEst,...
    'DFREQEST',DFREQEst,...    
    'MEASDATE', measDate,...
    'CHK',strCHK);
%% 

%%% ----------------------End Function

    %fprintf('Data frame decodificado:\n')
    %disp(dataframeDecod)
    %fprintf('-------------------------\n')
end