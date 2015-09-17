function box_out = matlab_Initialize(box_in)
    disp('Matlab initialize function has been called.')
    
    %%%% global variables used in process function
    global time nFFT iiF freq_weights electrode_weights ref_average threshold threshold_window OVTK_StimulationId_SegmentStart OVTK_StimulationId_Beep;
    time = 2;
    nFFT = 256;
    sampleFreq = 128;
    fLines = sampleFreq*(1:nFFT/2)/nFFT;
    iiF = find(fLines >=4 & fLines <= 25);
    threshold = 0.3;
    threshold_window = [];
    OVTK_StimulationId_SegmentStart = hex2dec('00008003');
    OVTK_StimulationId_Beep = hex2dec('00008202');
    load freq_weights;
    load electrode_weights;
    
    %%%% set output header
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