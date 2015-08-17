function box_out = matlab_Process(box_in)

	  global nFFT sampleFreq freq_weights electrode_weights ref_average threshold threshold_window stimulation;

    for i = 1: OV_getNbPendingInputChunk(box_in,1)

        [box_in, start_time, end_time, matrix_data] = OV_popInputBuffer(box_in,1);
        stim_set = [];
        
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
            
            %%%% scale spect by multiplying with frequency weight and electrode weight
            one_dim_signal(i) = (freq_weights' * spect) * electrode_weights(i); 
        end
        
        %%%% final signal is sum of 
        one_dim_signal = sum(one_dim_signal);

        %%%% store processed signal for first 20s in order to compute reference average
        if (end_time < 20)
            ref_average(length(ref_average) + 1) = one_dim_signal;
            
        %%%% in next 10s compute averages for window of overlapping 4 signal values
        elseif (end_time < 30)
            %%%% compute reference average if not computed
            if (length(ref_average) ~= 1)
                ref_average = mean(ref_average);
            end
            
            %%%% fill threshold_window with 4 signal values
            if (length(threshold_window) ~= 4)
                threshold_window(length(threshold_window) + 1) = one_dim_signal;
            %%%% length = 4
            else
                %%%% compute average and compare with reference average
                [ref_average mean(threshold_window)]
                if (ref_average * (1 - threshold) > mean(threshold_window))
                    %%%% if below reference average, send trigger to robotic arm
                    stim_set = [stimulation; box_in.clock; 0];
                    disp('Sending a stimulation...');
                end
                
                %%%% move threshold window 
                threshold_window(1:3) = threshold_window(2:4);
                threshold_window(4) = one_dim_signal;
            end
        end
        
        %%%% sending stimulation and signal
        box_in = OV_addOutputBuffer(box_in,1,start_time,end_time,stim_set);
        box_in = OV_addOutputBuffer(box_in,2,start_time,end_time,one_dim_signal);
        
    end

    box_out = box_in;
end