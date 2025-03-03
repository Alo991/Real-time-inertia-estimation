% decodeDataFrame 
function [decodedDF, PHEst, FREQEst, ANEst, DIGBits] = decodeDataFrameV2(dFrame, confFrame2)
%dFrame: Dataframe read from the PMU.
%repPH, repFreq, repAN, etc. Are data from CFG2 that are necessary to decode

%returns:
%decodedDF: Struct object containing the decoded dataframe data
%PHest

%repPH indicates whether the variable is 16-bit or 32-bit
%repFREQ integer or floating point
%PHNMR number of phasors
%ANNMR number of analog signals
%DGNMR number of digital signals
%facConv conversion factor for the phasors
%repPHpol representation of phasors: polar or rectangular
%repAn integer or floating point representation of analog values
%
repPH = confFrame2.PH_REP;
repFREQ = confFrame2.FREQ_REP;
PHNMR = confFrame2.PH_NUMBER;
ANNMR = confFrame2.AN_NUMBER;
DGNMR = confFrame2.DG_NUMBER;
repPHpol = confFrame2.PH_POL_REP;
facConv = 1;%confFrame2.FACCONV;
repAN = confFrame2.AN_REP;
timeBase = confFrame2.TIME_BASE;



DIGBits=0;% If there are no digital outputs, set to zero

%1 SYNC 2 bytes
%First byte: AA hex
%Second byte: 21 hex for configuration 1
%31 hex for configuration 2
%Both frames are version 1 (IEEE Std C37.118-2005 [B6])
rdSYNC = dFrame(1:2);
%Extract bits 7,6,5, which indicate the frame type (starting at 1, not zero)
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
MSG_TQ = rdFRACSEC(1); %Tabla 3 del standar
decFRACSEC = typecast( [ rdFRACSEC(3) rdFRACSEC(2) rdFRACSEC(1) 0],'uint32');
%% STAT
rdSTAT=typecast(dFrame(15:16),'uint16');
% Bits 03–00: Trigger reason:
% 1111–1000: Available for user definition
% 0111: Digital 0110: Reserved
% 0101: df/dt High 0100: Frequency high or low
% 0011: Phase angle diff 0010: Magnitude high
% 0001: Magnitude low 0000: Manual
rdTriggerRsn = bitand(rdSTAT,15);%para obtener solo los bits 03-00 (00001111)

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
%% 7 PHASORS ESTIMATES (Conversion factor missing)

PHEst = zeros(1,PHNMR); %Number of phasors, magnitude, and angle
if repPH ==0 %16-bit integer
    
    nBytes = 4;%4 bytes: 16 bits for magnitude and 16 for angle, or real and imaginary
    endByte_PHEst = 17+(nBytes*PHNMR)-1;
    fasores = dFrame (17:endByte_PHEst);
    byte1 = typecast([fasores(2) fasores(1)],'int16');
    byte2  = typecast([fasores(4) fasores(3)],'int16');
    
    if repPHpol==0 %Rectangular representation
        
        for i = 1:PHNMR
            
            PHEst(1,i) = abs(byte1 + byte2) *facConv;%magniutde
            PHEst(1,6+i) = rad2deg(angle(byte1 + byte2)*facConv)/10^4;%phase
            
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
       % The standard handles 32-bit floating point values, which are of type 'single'  
       % in MATLAB 
        bytesMag = uint8([fnbytes(4) fnbytes(3) fnbytes(2) fnbytes(1)]);
        phMagnitude = typecast(bytesMag,'single');
        bytesAng = uint8([fnbytes(8) fnbytes(7) fnbytes(6) fnbytes(5)]);
        phAngle = typecast(bytesAng,'single');
        PHEst(1,i) = phMagnitude * facConv; %Magnitude
        PHEst(1,6+i) = rad2deg(phAngle); %angle
%         fprintf('Fasor %i, factor de conversion %f\n',i,facConv)
%         fprintf('Magnitud: %f (%f)\n',PHEst(1,i),PHEst(1,i)*sqrt(3))
%         fprintf('Ángulo: %f\n',PHEst(1,6+i))
%         fprintf('-------------------------------------\n')
    end
end

%% 8 FREQ Estimated frequency 
%The FREQDev is in mHz  
if repFREQ ==0 %16-bit integer  
    nBytes = 2;
    f = dFrame(endByte_PHEst+1:endByte_PHEst+nBytes);
    FREQEst= double(typecast([ f(2), f(1)],'int16'))/1000; 

else
    nBytes = 4; % 32-bit floating point  
   f = dFrame(endByte_PHEst+1:endByte_PHEst+nBytes);
    bytesFreq = uint8([f(4) f(3) f(2) f(1)]);
    FREQEst = typecast(bytesFreq,'single');
    

end
endByte_FREQDev = endByte_PHEst + nBytes; 

%% 9 DFREQ 2/4 ROCOF punto fijo o flotante
% ROCOF, in hertz per second times 100 ( 1000)?
% Range –327.67 to +327.67 Hz per second
if repFREQ ==0 %16-bit integer 
    nBytes = 2;
    devf = dFrame(endByte_FREQDev+1:endByte_FREQDev+nBytes);
    DFREQEst= typecast([ devf(2), devf(1)],'int16'); 

else
    nBytes = 4;% 32-bit floating point
    devf = dFrame(endByte_FREQDev+1:endByte_FREQDev+nBytes);
    bytesDFREQ = uint8([devf(4) devf(3) devf(2) devf(1)]);
    DFREQEst = typecast(bytesDFREQ,'single');
end
endByte_DFREQEst = endByte_FREQDev + nBytes; %including the first byte

%% 10 ANALOG 
%Analog data, 2 or 4 bytes per value depending on fixed or floating-point
%format used, as indicated by the FORMAT field in configuration 1, 2, and
%3 frames. The number of values is determined by the ANNMR field in
%configuration 1, 2, and 3 frames.
    if repAN ==0 %16-bit integer 
        nBytes = 4;
    else
        nBytes = 8;% 32-bit floating point 
    end
    endByte_ANEst = endByte_DFREQEst+(nBytes*ANNMR); 
    ANALOGS = dFrame (endByte_DFREQEst+1:endByte_ANEst);
    
    ANEst = zeros(ANNMR,nBytes); %Number of Phasors x Number of Bits
    for i = 1:ANNMR
        inicio = (nBytes*(i-1)+1);
        fin = inicio+nBytes-1;
        ANEst(i,:) = ANALOGS(inicio:fin); %Phasor Estimation
    end

%% 11 Digital 2 × DGNMR
%Digital data, usually representing 16 digital status points (channels). The
%number of values is determined by the DGNMR field in configuration 1,
%2, and 3 frames.

    nBytes = DGNMR*2; % Two bytes per digital number

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
    