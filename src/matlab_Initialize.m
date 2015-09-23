function box_out = matlab_Initialize(box_in)
    disp('Matlab initialize function has been called.')
    
    %%%% global variables used in process function
    global time nFFT iiF P PP res ref_average threshold threshold_window OVTK_StimulationId_SegmentStart OVTK_StimulationId_Beep;
    time = 2;
    nFFT = 256;
    sampleFreq = 128;
    fLines = sampleFreq*(1:nFFT/2)/nFFT;
    iiF = find(fLines >=4 & fLines <= 25);
    threshold = 0.3;
    threshold_window = [];
    OVTK_StimulationId_SegmentStart = hex2dec('00008003');
    OVTK_StimulationId_Beep = hex2dec('00008202');
    
    load removedatoms_resparafac_201_vlh_13_lr;
    [n1,n2,n3]=size(res.Xtrain); 

    nEpoch  = n1; 
    nElect  = n2; 
    nFlines = n3; 

    %%%%% atom weights
    atomN=[6];
    A{1}=res1.Xfactors{2}(:,atomN); %%%% electrode_weights
    A{2}=res1.Xfactors{3}(:,atomN); %%%% freq_weights 
    fact=1:length(atomN); %%% tu moze byt viac faktorov, zavisi ako sa daju atomN
    nTS=length(atomN);  %%%  number of time scores/ atoms 

    P  = krb(A{1}(:,fact),A{2}(:,fact)); 
    PP = P'*P;
    
    %%%% (OpenVibe definitions) output header
    nb_channels = 1;
    nb_samples_per_buffer = 1;
    channel_names = cell(1);
    channel_names{1} = sprintf('channel %i',1);
    sampling_rate = 1;
        
    box_in = OV_setStimulationOutputHeader(box_in, 1);
    box_in = OV_setSignalOutputHeader(box_in, 2, nb_channels, nb_samples_per_buffer, channel_names, sampling_rate);
    box_in = OV_setStimulationOutputHeader(box_in, 3);
    
    box_out = box_in;
end