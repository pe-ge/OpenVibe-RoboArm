function box_out = matlab_Process(box_in)

    global time nFFT iiF P PP res ref_average threshold threshold_window OVTK_StimulationId_SegmentStart OVTK_StimulationId_Beep;
    
    a_relax = 10
    b_pause = 35
    c_robot = 60
    d_pause = 10 

    for i = 1: OV_getNbPendingInputChunk(box_in,1)

        [box_in, start_time, end_time, matrix_data] = OV_popInputBuffer(box_in,1);
        robo_arm_stimulations = [];
        beep_stimulations = [];
        
        time = time + (1 / box_in.clock_frequency); %%%% increment time
        if (time == 2.5) %%%% send initial beep?
          beep_stimulations = [OVTK_StimulationId_Beep; box_in.clock; 0];
          disp('Beep: start of next session...');
        end

        %%%% iterate over all electrodes
        spectAll = [];
        for i = 1 : size(matrix_data, 1)
            datSeg = matrix_data(i, :);
           
            %%%% subtract mean 
            datSeg = datSeg - mean(datSeg);
           
            %%%% compute spectra
            w      = window(@hann, length(datSeg));
            datSeg = datSeg .* w';
            yF     = fft(datSeg, nFFT);
            yF     = yF(1:nFFT / 2 + 1);
            Pyy    = (abs(yF).^2) ./ length(datSeg);

            %%%% to be equal with BCI2000
            spect(1:nFFT/2,1) = 2 * Pyy(1:nFFT/2);
            
            %%%% compute log-power+
            logZeroParam = exp(-15) ; %%%% when computing log this replaces 0 values 
            spect(spect == 0) = logZeroParam; %%%% treat zeros                         
            spect = 10*log10(spect);

            %%%% filter everything but 4 - 25 Hz
            spect = spect(iiF);
            spectAll = [spectAll; spect];
        end
        
        X2 = spectAll - res.meanX{1}';
        XP = P'*X2;
        one_dim_signal = fastnnls(PP,XP);

        %%%% relaxation
        if (time < a_relax)
            ref_average(end + 1) = one_dim_signal;
            
        %%%% first pause    
        elseif (time < a_relax + b_pause)
            if (time == a_relax)
                ref_average = mean(ref_average);
                beep_stimulations = [OVTK_StimulationId_Beep; box_in.clock; 0];
                disp('Beep: first pause...');
            end
            
        %%%% robot movement
        elseif (time < a_relax + b_pause + c_robot)
            if (time == a_relax + b_pause)
                beep_stimulations = [OVTK_StimulationId_Beep; box_in.clock; 0];
                disp('Beep: imagination of movement...');
            end
            
            %%%% fill threshold_window with 4 signal values
            if (length(threshold_window) ~= 4)
                threshold_window(length(threshold_window) + 1) = one_dim_signal;
            %%%% when length = 4
            else
                %%%% compute average and compare with reference average
                if (ref_average * (1 - threshold) > mean(threshold_window))
                    %%%% if value below reference average, send signal to robot
                    robo_arm_stimulations = [OVTK_StimulationId_SegmentStart; box_in.clock; 0];
                    disp('Sending a movement trigger...');
                    time = a_relax + b_pause + c_robot - (1 / box_in.clock_frequency);
                end
                
                %%%% move threshold window
                threshold_window = threshold_window(2:end); %%%% remove first
                threshold_window(end + 1) = one_dim_signal; %%%% append to end
            end
        %%%% 10 seconds pause
        elseif (time < a_relax + b_pause + c_robot + d_pause)
            if (time == a_relax + b_pause + c_robot)
                beep_stimulations = [OVTK_StimulationId_Beep; box_in.clock; 0];
                disp('Beep: end of session...');
            end
        else
            time = 2; %%%% end of one session
        end
        
        %%%% sending stimulation and signal
        box_in = OV_addOutputBuffer(box_in,1,start_time,end_time,robo_arm_stimulations);
        box_in = OV_addOutputBuffer(box_in,2,start_time,end_time,one_dim_signal);
        box_in = OV_addOutputBuffer(box_in,3,start_time,end_time,beep_stimulations);
        
    end

    box_out = box_in;
end