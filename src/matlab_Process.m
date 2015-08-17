function box_out = matlab_Process(box_in)

	  global nFFT sampleFreq freq_weights electrode_weights;

    for i = 1: OV_getNbPendingInputChunk(box_in,1)

        [box_in, start_time, end_time, matrix_data] = OV_popInputBuffer(box_in,1);
        
        %%%% iterate over all electrodes
        for i = 1 : size(matrix_data, 1)
            datSeg = matrix_data(i, :);
           
            %%%% subtract mean 
            datSeg = datSeg - mean(datSeg);
           
            %%%% compute spectra for epochs 
            w = window(@hann, length(datSeg));
            datSeg = datSeg .* w';
            yF    = fft(datSeg, nFFT);
            yF    = yF(1:nFFT/2 + 1);
            Pyy   = (abs(yF).^2)./ length(datSeg);

            %%%% to be equal with BCI2000
            spect(1:nFFT/2,1)=2*Pyy(1:nFFT/2);
            fLines = sampleFreq*(1:nFFT/2)/nFFT;

            %%%% filter everything but 4 - 25 Hz
            spect = spect(fLines >= 4);
            fLines = fLines(fLines >= 4);
            spect = spect(fLines <= 25);
            fLines = fLines(fLines <= 25);
            
            %%%% multiply spect with frequency and electrode weights
            result(i) = (freq_weights' * spect) * electrode_weights(i); 
        end
        
        result = sum(result);
        
        %%%% setting output header
        box_in.outputs{1}.header = box_in.inputs{1}.header;
        box_in.outputs{1}.header.nb_channels = 1;
        channel_names = cell(1);
        channel_names{1} = sprintf('channel %i',1);
        box_in.outputs{1}.header.channel_names = channel_names;
        box_in.outputs{1}.header.nb_samples_per_buffer = 1;
        box_in.outputs{1}.header.sampling_rate = 1;
        
        %%%% sending output
        box_in = OV_addOutputBuffer(box_in,1,start_time,end_time,result);
    end

    box_out = box_in;
end