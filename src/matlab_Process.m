function box_out = matlab_Process(box_in)
	clf;
	hold on;

    for i = 1: OV_getNbPendingInputChunk(box_in,1)
        [box_in, start_time, end_time, matrix_data] = OV_popInputBuffer(box_in,1);
		
        robo_arm_stimulations = [];
        beep_stimulations = [];
		stop_stimulation = [];
        
        box_in.time = box_in.time + (1 / box_in.clock_frequency); %%%% increment time
		
        if (box_in.time == box_in.initial_time) %%%% send relax beep?
          beep_stimulations = [box_in.OVTK_StimulationId_Label_00; box_in.clock; 0];
          disp('Beep: relax...');
		  box_in.ref_average = [];
		  box_in.threshold_window = [];
        end
		
		one_dim_signal = process_signal(box_in, matrix_data);

		%%%% ploting one_dim_signal
		box_in.signal_x = move(box_in.signal_x, start_time, 16);
		box_in.signal_y = move(box_in.signal_y, one_dim_signal, 16);
		plot(box_in.signal_x, box_in.signal_y, 'g');

        %%%% relaxation
        if (box_in.time < box_in.initial_time + box_in.a_relax)
            box_in.ref_average(end + 1) = one_dim_signal;
            
        %%%% first pause    
        elseif (box_in.time < box_in.initial_time + box_in.a_relax + box_in.b_pause)
            if (box_in.time == box_in.initial_time + box_in.a_relax)
                box_in.ref_average = mean(box_in.ref_average);
                beep_stimulations = [box_in.OVTK_StimulationId_Label_01; box_in.clock; 0];
                disp('Beep: pause');
            end
            plot(box_in.signal_x, box_in.ref_average * ones(1, length(box_in.signal_x)), 'r');
			plot(box_in.signal_x, box_in.ref_average * (1 - box_in.threshold) * ones(1, length(box_in.signal_x)), '--r');
        %%%% robot movement
        elseif (box_in.time < box_in.initial_time + box_in.a_relax + box_in.b_pause + box_in.c_robot)
            if (box_in.time == box_in.initial_time + box_in.a_relax + box_in.b_pause)
                beep_stimulations = [box_in.OVTK_StimulationId_Label_02; box_in.clock; 0];
                disp('Beep: move');
            end
            
            %%%% fill threshold_window with signal values
            if (length(box_in.threshold_window) ~= box_in.threshold_window_length)
                box_in.threshold_window(end + 1) = one_dim_signal;
            %%%% when length = threshold_window_length
            else
                %%%% compute average and compare with reference average
                if (box_in.ref_average * (1 - box_in.threshold) > mean(box_in.threshold_window))
                    %%%% if value below reference average, send signal to robot
                    robo_arm_stimulations = [box_in.OVTK_StimulationId_SegmentStart; box_in.clock; 0];
                    disp('Sending a movement trigger...');
                    box_in.time = box_in.initial_time + box_in.a_relax + box_in.b_pause + box_in.c_robot - (1 / box_in.clock_frequency);
                end
                
                %%%% move threshold window
				box_in.threshold_window = move(box_in.threshold_window, one_dim_signal, box_in.threshold_window_length);
            end
			plot(box_in.signal_x, box_in.ref_average * ones(1, length(box_in.signal_x)), 'r');
			plot(box_in.signal_x, box_in.ref_average * (1 - box_in.threshold) * ones(1, length(box_in.signal_x)), '--r');
        %%%% end of session pause
        elseif (box_in.time < box_in.initial_time + box_in.a_relax + box_in.b_pause + box_in.c_robot + box_in.d_pause)
            if (box_in.time == box_in.initial_time + box_in.a_relax + box_in.b_pause + box_in.c_robot)
                disp('Beep: pause');
            end
        else
			%%%% end of one session
            box_in.time = box_in.initial_time - (1 / box_in.clock_frequency);
			
			%%%% should stop experiment?
			if (box_in.current_run == box_in.num_runs)
				stop_stimulation = [box_in.OVTK_StimulationId_ExperimentStop; box_in.clock; 0];
			end
			
			box_in.current_run = box_in.current_run + 1;
        end
		
		%%%% set limits on x axis
		xlim([box_in.signal_x(1) box_in.signal_x(1) + 8]);
		%%%% show current run
		title(box_in.current_run);
        
        %%%% send stimulations
        box_in = OV_addOutputBuffer(box_in, 1, start_time, end_time, robo_arm_stimulations);
        box_in = OV_addOutputBuffer(box_in, 2, start_time, end_time, beep_stimulations);
		box_in = OV_addOutputBuffer(box_in, 3, start_time, end_time, stop_stimulation);
    end

    box_out = box_in;
end

function list = move(list, value, len)
	if length(list) >= len
		list = list(2:end); %%%% remove first
	end

	list(end + 1) = value; %%%% append to end
end

function one_dim_signal = process_signal(box_in, matrix_data)
	%%%% iterate over all electrodes
	spectAll = [];
	for t = 1 : size(matrix_data, 1)
		datSeg = matrix_data(t, :);
	   
		%%%% subtract mean 
		datSeg = datSeg - mean(datSeg);
	   
		%%%% compute spectra
		w      = window(@hann, length(datSeg));
		datSeg = datSeg .* w';
		yF     = fft(datSeg, box_in.nFFT);
		yF     = yF(1:box_in.nFFT / 2 + 1);
		Pyy    = (abs(yF).^2) ./ length(datSeg);

		%%%% to be equal with BCI2000
		spect(1:box_in.nFFT/2 + 1,1) = 2 * Pyy(1:box_in.nFFT/2 + 1);
		
		%%%% compute log-power+
		logZeroParam = exp(-15) ; %%%% when computing log this replaces 0 values 
		spect(spect == 0) = logZeroParam; %%%% treat zeros                         
		spect = 10*log10(spect);

		%%%% filter everything but defined range 
		spect = spect(box_in.iiF);
		spectAll = [spectAll; spect];
	end
	
	%%%%%%%%%%%%%%%%%%%% ZMENIT!!!!!!!!!!!! %%%%%%%%%%%%%%
	%X2 = spectAll - box_in.res.meanX{1}';
	X2 = box_in.res.meanX{1}';
	%%%%%%%%%%%%%%%%%%%% ZMENIT!!!!!!!!!!!! %%%%%%%%%%%%%%
	XP = box_in.P' * X2;
	one_dim_signal = fastnnls(box_in.PP,XP);
	one_dim_signal = one_dim_signal(box_in.atomN, 1); 
end