function box_out = matlab_Initialize(box_in)
    figure;

    %%%% FFT size
    box_in.nFFT = 256;
    sampleFreq = 128;
    fLines = sampleFreq * (0:box_in.nFFT/2) / box_in.nFFT;
    %%%% OpenVibe ID stimulations
    box_in.OVTK_StimulationId_SegmentStart = hex2dec('00008003');
    box_in.OVTK_StimulationId_Label_00 = hex2dec('00008100');
    box_in.OVTK_StimulationId_Label_01 = hex2dec('00008101');
    box_in.OVTK_StimulationId_Label_02 = hex2dec('00008102');
    box_in.OVTK_StimulationId_ExperimentStart = hex2dec('00008001');
    box_in.OVTK_StimulationId_ExperimentStop = hex2dec('00008002');
    %%%% ploting window
    box_in.signal_x = [];
    box_in.signal_y = [];
    %%%% time counter
    box_in.time = 2;
    box_in.initial_time = 2;
    %%%% number of runs
    box_in.num_runs = box_in.settings(1).value;
    box_in.current_run = 1;
    %%%% when new attempt starts, do not take first 4 values for calculating average
    box_in.ignore_signal_value = 0;
    box_in.total_ignore_values = 4;
    %%%% colors
    box_in.relax_color = [0 0.5 0];
    box_in.pause_color = [1 0.5 0];
    box_in.move_color = [0 0 1];
    %%%% indicates whether movement was triggered (pause should not be heard)
    box_in.robot_moved = false;

    %subName='Jaro';

    box_in.type=box_in.settings(2).value;    % %'Mu' 'SMR1','SMR2','Beta1','Beta2','Alpha'
    box_in.cond=box_in.settings(3).value;  % ,'VLH';  %'VLH'
    box_in.lat =box_in.settings(4).value; %  'L','R','LR';
    box_in.threshold = box_in.settings(5).value;
    box_in.subName = box_in.settings(6).value; 
    box_in.threshold_window_length = box_in.settings(7).value;
    box_in.y_max = box_in.settings(8).value;
    box_in.y_min = box_in.settings(9).value;
    box_in.x_length = box_in.settings(10).value;

    switch box_in.subName
        case  'Tony'
            box_in.iiF = find(fLines >=3.5 & fLines <= 24.5);

            %type='Mu';    % %'Mu' 'SMR1','SMR2','Beta1','Beta2','Alpha'
            %cond='VLH';  % ,'VLH';  %'VLH'
            %lat ='LR'; %  'L','R','LR';
            meanT='Mean';

            load('atoms201')
            eval(['wS=res.',box_in.type,'.',box_in.cond,'.',box_in.lat,'.elecW.',meanT,''';']);
            eval(['wF=res.',box_in.type,'.',box_in.cond,'.',box_in.lat,'.freqW.',meanT,''';']);

            %box_in.threshold = 0.1; % higher threshold the more difficult it will be
            %box_in.threshold_window_length = 4; % 4 = 2 seconds
            box_in.a_relax = 15;
            box_in.b_pause = 6;
            box_in.c_robot = 20;
            box_in.d_pause = 7;

        case  'Jaro'
            box_in.iiF = find(fLines >=4 & fLines <= 25);

            %type='Mu';    % 'SMR1','SMR2','Beta1','Beta2','Alpha'
            %cond='VLH';  % ,'VLH';  %'VLH'
            %lat ='LR'; %  'L','R','LR';
            meanT='Mean';

            load('atoms202')
            eval(['wS=res.',box_in.type,'.',box_in.cond,'.',box_in.lat,'.elecW.',meanT,''';']);
            eval(['wF=res.',box_in.type,'.',box_in.cond,'.',box_in.lat,'.freqW.',meanT,''';']); 

            %box_in.threshold = 0.09; % higher threshold the more difficult it will be
            %box_in.threshold_window_length = 4; % 4 = 2 seconds
            box_in.a_relax = 15;
            box_in.b_pause = 6;
            box_in.c_robot = 20;
            box_in.d_pause = 7;

        % case  'Karina'
            % box_in.iiF = find(fLines >=4 & fLines <= 25);
            % load resPARAFAC-newKarin %karina
            % %load resPARAFAC-301v2 %timotej
            % box_in.atomN=[4]; %karina

            % box_in.threshold = 0.7; % higher threshold the more difficult it will be
            % box_in.threshold_window_length = 5; % 4 = 2 seconds
            % box_in.a_relax = 10;
            % box_in.b_pause = 8;
            % box_in.c_robot = 20;
            % box_in.d_pause = 10;
    end

    box_in.P  = krb(wS,wF);
    box_in.PP = box_in.P'*box_in.P;

    %%%% create output header
    nb_channels = 1;
    nb_samples_per_buffer = 1;
    channel_names = cell(1);
    channel_names{1} = sprintf('channel %i', 1);
    sampling_rate = 1;

    box_in = OV_setStimulationOutputHeader(box_in, 1);
    box_in = OV_setSignalOutputHeader(box_in, 2, nb_channels, nb_samples_per_buffer, channel_names, sampling_rate);
    box_in = OV_setStimulationOutputHeader(box_in, 3);
    box_in = OV_setStimulationOutputHeader(box_in, 4);

    %%%% SAVING
    currentTimeAndDate = now;
    format_time = 'yyyy.mm.dd-HH.MM.SS';
    file_name = ['merania/', box_in.subName, '-', datestr(currentTimeAndDate, format_time)];
    save([file_name, '-params.mat'], 'box_in');

    box_in.fid = fopen([file_name, '.txt'], 'at');

    box_out = box_in;
end
