function box_out = matlab_Process(box_in)
    clf;
    hold on;
    
    if box_in.experiment_stopped
        box_out = box_in;
        return
    end

    for i = 1: OV_getNbPendingInputChunk(box_in,1)
        [box_in, start_time, end_time, matrix_data] = OV_popInputBuffer(box_in,1);

        robo_arm_stimulations = [];
        beep_stimulations = [];
        start_stop_experiment = [];
        stimulation_to_save = 'none';

        %%%% relax
        if (box_in.time == box_in.initial_time)
            beep_stimulations = [box_in.OVTK_StimulationId_Label_00; box_in.clock; 0];
            disp('Beep: relax...');
            box_in.ref_average = [];
            box_in.threshold_window = [];
            box_in.a_relax_x = start_time;
            stimulation_to_save = 'relax';

            %%%% start of experiment?
            if (box_in.current_run == 1)
                start_stop_experiment = [box_in.OVTK_StimulationId_ExperimentStart; box_in.clock; 0];
            end
        end

        one_dim_signal = process_signal(box_in, matrix_data);

        %%%% ploting one_dim_signal
        box_in.signal_x = slide_window(box_in.signal_x, start_time, 2 * box_in.x_length);
        box_in.signal_y = slide_window(box_in.signal_y, one_dim_signal, 2 * box_in.x_length);

        %%%% relaxation
        if (box_in.time < box_in.initial_time + box_in.a_relax)
            if box_in.current_run == 1
                box_in.ref_average(end + 1) = one_dim_signal;
            else
                %%%% when new attempt starts, do not take first 4 values for calculating average
                if box_in.ignore_signal_value < box_in.total_ignore_values;
                    box_in.ignore_signal_value = box_in.ignore_signal_value + 1;
                else
                    box_in.ref_average(end + 1) = one_dim_signal;
                end
            end
            plot(box_in.signal_x, box_in.signal_y, 'LineWidth', 2);

        %%%% first pause
        elseif (box_in.time < box_in.initial_time + box_in.a_relax + box_in.b_pause)
            if (box_in.time == box_in.initial_time + box_in.a_relax)
                box_in.ref_average = mean(box_in.ref_average);
                % beep_stimulations = [box_in.OVTK_StimulationId_Label_01; box_in.clock; 0];
                disp('Beep: first pause');

                box_in.b_pause_x = start_time;
                stimulation_to_save = '1st_pause';
            end

            %%%% pause plot
            plot(box_in.signal_x, box_in.signal_y, 'LineWidth', 2);

            plot(box_in.signal_x, box_in.ref_average * ones(1, length(box_in.signal_x)), 'r');
            plot(box_in.signal_x, box_in.ref_average * (1 - box_in.threshold) * ones(1, length(box_in.signal_x)), '--r');
        %%%% robot movement
        elseif (box_in.time < box_in.initial_time + box_in.a_relax + box_in.b_pause + box_in.c_robot)
            if (box_in.time == box_in.initial_time + box_in.a_relax + box_in.b_pause)
                beep_stimulations = [box_in.OVTK_StimulationId_Label_02; box_in.clock; 0];
                disp('Beep: move');

                box_in.c_move_x = start_time;
                stimulation_to_save = 'move';

            %%%% fill threshold_window with signal values
            elseif (length(box_in.threshold_window) ~= box_in.threshold_window_length)
                box_in.threshold_window(end + 1) = one_dim_signal;

            %%%% length equals threshold_window_length
            else
                %%%% compute average and compare with reference average
                if (box_in.ref_average * (1 - box_in.threshold) > mean(box_in.threshold_window))
                    %%%% if value below reference average, send signal to robot
                    robo_arm_stimulations = [box_in.OVTK_StimulationId_SegmentStart; box_in.clock; 0];
                    box_in.robot_moved = true;
                    disp('Sending a movement trigger...');
                    box_in.time = box_in.initial_time + box_in.a_relax + box_in.b_pause + box_in.c_robot - (1 / box_in.clock_frequency);
                    stimulation_to_save = 'robot';
                end

                %%%% move threshold window
                box_in.threshold_window = slide_window(box_in.threshold_window, one_dim_signal, box_in.threshold_window_length);
            end

            %%%% plot signal
            plot(box_in.signal_x, box_in.signal_y, 'LineWidth', 2);

            %%%% plot mean and threshold
            plot(box_in.signal_x, box_in.ref_average * ones(1, length(box_in.signal_x)), 'r');
            plot(box_in.signal_x, box_in.ref_average * (1 - box_in.threshold) * ones(1, length(box_in.signal_x)), '--r');
        %%%% end of session pause
        elseif (box_in.time < box_in.initial_time + box_in.a_relax + box_in.b_pause + box_in.c_robot + box_in.d_pause)
            if (box_in.time == box_in.initial_time + box_in.a_relax + box_in.b_pause + box_in.c_robot)
                %%%% should pause be heard
                if ~box_in.robot_moved
                    beep_stimulations = [box_in.OVTK_StimulationId_Label_01; box_in.clock; 0];
                end
                box_in.robot_moved = false;

                disp('Beep: second pause');
                box_in.d_pause_x = start_time;
                stimulation_to_save = '2nd_pause';
            end

            %%%% plot signal
            plot(box_in.signal_x, box_in.signal_y, 'LineWidth', 2);

            %%%% plot mean and threshold
            plot(box_in.signal_x, box_in.ref_average * ones(1, length(box_in.signal_x)), 'r');
            plot(box_in.signal_x, box_in.ref_average * (1 - box_in.threshold) * ones(1, length(box_in.signal_x)), '--r');
        else
            %%%% end of one session - reset time counter
            box_in.time = box_in.initial_time - (1 / box_in.clock_frequency);

            %%%% should stop experiment?
            if (box_in.current_run == box_in.num_runs)
                start_stop_experiment = [box_in.OVTK_StimulationId_RestStop; box_in.clock; 0];
                box_in.experiment_stopped = true;
            end

            box_in.current_run = box_in.current_run + 1;
            box_in.ignore_signal_value = 0;

            %%%% plot signal
            plot(box_in.signal_x, box_in.signal_y, 'LineWidth', 2);
        end

        %%%% set axis limits
        xlim([box_in.signal_x(1) box_in.signal_x(1) + box_in.x_length]);
        ylim([box_in.y_min box_in.y_max]); % pre vypnutie y-ovych limitov, zakomentovat
        %%%% show current run in title
        title(min(box_in.current_run, box_in.num_runs));

        %%%% vertical lines
        if isfield(box_in, 'a_relax_x')
            plot([box_in.a_relax_x box_in.a_relax_x], [box_in.y_min box_in.y_max], 'Color', box_in.relax_color, 'LineWidth', 2);
        end
        if isfield(box_in, 'b_pause_x')
            plot([box_in.b_pause_x box_in.b_pause_x], [box_in.y_min box_in.y_max], 'Color', box_in.pause_color, 'LineWidth', 2);
        end
        if isfield(box_in, 'c_move_x')
            plot([box_in.c_move_x box_in.c_move_x], [box_in.y_min box_in.y_max], 'Color', box_in.move_color, 'LineWidth', 2);
        end
        if isfield(box_in, 'd_pause_x')
            plot([box_in.d_pause_x box_in.d_pause_x], [box_in.y_min box_in.y_max], 'Color', box_in.pause_color, 'LineWidth', 2);
        end

        %%%% increment time
        box_in.time = box_in.time + (1 / box_in.clock_frequency);

        %%%% send stimulations
        box_in = OV_addOutputBuffer(box_in, 1, start_time, end_time, robo_arm_stimulations);
        box_in = OV_addOutputBuffer(box_in, 2, start_time, end_time, one_dim_signal);
        box_in = OV_addOutputBuffer(box_in, 3, start_time, end_time, beep_stimulations);
        box_in = OV_addOutputBuffer(box_in, 4, start_time, end_time, start_stop_experiment);

        %%%% save
        save_to_file(box_in, end_time, one_dim_signal, stimulation_to_save);
    end

    box_out = box_in;
end

function save_to_file(box_in, time, one_dim_signal, stimulation_to_save)
    time_str = num2str(time, 32);
    signal_str = num2str(one_dim_signal, 32);
    fprintf(box_in.fid, '%s\n', [time_str, ',', signal_str, ',', stimulation_to_save]);
end

function list = slide_window(list, value, len)
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

    X2 = spectAll;
    XP = box_in.P' * X2;
    one_dim_signal = fastnnls(box_in.PP,XP);
end
