function box_out = matlab_Process(box_in)

    global time nFFT iiF freq_weights electrode_weights ref_average threshold threshold_window OVTK_StimulationId_SegmentStart OVTK_StimulationId_Beep;
    for i = 1: OV_getNbPendingInputChunk(box_in,1)

        [box_in, start_time, end_time, matrix_data] = OV_popInputBuffer(box_in,1);
        robo_arm_stimulations = [];
        beep_stimulations = [];

        %%%% iterate over all electrodes
        for i = 1 : size(matrix_data, 1)
            datSeg = matrix_data(i, :);
           
            %%%% subtract mean 
            datSeg = datSeg - mean(datSeg);
           
            %%%% compute spectra for epochs 
            w      = window(@hann, length(datSeg));
            datSeg = datSeg .* w';
            yF     = fft(datSeg, nFFT);
            yF     = yF(1:nFFT / 2 + 1);
            Pyy    = (abs(yF).^2) ./ length(datSeg);

            %%%% to be equal with BCI2000
            spect(1:nFFT/2,1) = 2 * Pyy(1:nFFT/2);

            %%%% filter everything but 4 - 25 Hz
            spect = spect(iiF);
            
            %%%% scale spect by multiplying with frequency weight and electrode weight
            one_dim_signal(i) = (freq_weights' * spect) * electrode_weights(i); 
        end
        
        %%%% final signal is sum of component signals
        one_dim_signal = sum(one_dim_signal);

        %%%% store processed signal for first 20s in order to compute reference average
        if (end_time < 20)
            ref_average(end + 1) = one_dim_signal;
            
        %%%% in next 10s compute averages for window of overlapping 4 signal values
        elseif (end_time < 30)
            %%%% compute reference average if not computed
            if (length(ref_average) ~= 1)
                ref_average = mean(ref_average);
                beep_stimulations = [OVTK_StimulationId_Beep; box_in.clock; 0];
                disp('Sending beep...');
            end
            
            %%%% fill threshold_window with 4 signal values
            if (length(threshold_window) ~= 4)
                threshold_window(length(threshold_window) + 1) = one_dim_signal;
            %%%% when length = 4
            else
                %%%% compute average and compare with reference average
                if (ref_average * (1 - threshold) > mean(threshold_window))
                    %%%% if value below reference average, send trigger to robotic arm
                    robo_arm_stimulations = [OVTK_StimulationId_SegmentStart; box_in.clock; 0];
                    disp('Sending a movement trigger...');
                end
                
                %%%% move threshold window
                threshold_window = threshold_window(2:end); %%%% remove first
                threshold_window(end + 1) = one_dim_signal; %%%% append to end
            end
        end
        
        %%%% sending stimulation and signal
        box_in = OV_addOutputBuffer(box_in,1,start_time,end_time,robo_arm_stimulations);
        box_in = OV_addOutputBuffer(box_in,2,start_time,end_time,one_dim_signal);
        box_in = OV_addOutputBuffer(box_in,3,start_time,end_time,beep_stimulations);
        
    end

    box_out = box_in;
end