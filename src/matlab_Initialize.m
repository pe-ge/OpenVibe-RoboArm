function box_out = matlab_Initialize(box_in)
    disp('Matlab initialize function has been called.')
    
    global nFFT sampleFreq freq_weights electrode_weights;
    nFFT = 256;
    sampleFreq = 128;
    load freq_weights;
    load electrode_weights;
    
    box_out = box_in;
end