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
    box_in.OVTK_StimulationId_RestStop = hex2dec('0000800a');
    %%%% ploting window
    box_in.signal_x = [];
    box_in.signal_y = [];
    %%%% time counter
    box_in.time = 2;
    box_in.initial_time = 2;
    %%%% number of runs
    box_in.num_runs = box_in.settings(1).value;
    box_in.current_run = 1;
    box_in.experiment_stopped = false;
    %%%% when new attempt starts, do not take first 4 values for calculating average
    box_in.ignore_signal_value = 0;
    box_in.total_ignore_values = 4;

    %%%% colors
    box_in.relax_color = [0 0.5 0];
    box_in.pause_color = [1 0.5 0];
    box_in.move_color = [0 0 1];
    %%%% indicates whether movement was triggered (pause should not be heard)
    box_in.robot_moved = false;
    
    %%%% wait periods for individual windows
    box_in.a_relax = 15; % relax window at the beginning of whole run
    box_in.b_pause = 6; % followed by pause
    box_in.c_robot = 20; % followed by intention of moving robot
    box_in.c_wait = 2; % but not allowed to move for this amount of seconds
    box_in.d_pause = 7; % followed by final pause

    %%%% box settings from openvibe
    box_in.type=box_in.settings(2).value;    % %'Mu' 'SMR1','SMR2','Beta1','Beta2','Alpha'
    box_in.cond=box_in.settings(3).value;  % ,'VLH';  %'VLH'
    box_in.lat =box_in.settings(4).value; %  'L','R','LR';
    box_in.threshold = box_in.settings(5).value;
    box_in.subName = box_in.settings(6).value; 
    box_in.threshold_window_length = box_in.settings(7).value;
    box_in.y_max = box_in.settings(8).value;
    box_in.y_min = box_in.settings(9).value;
    box_in.x_length = box_in.settings(10).value;
    box_in.filename_prefix = box_in.settings(11).value;

    switch box_in.subName
        case 'Tony'
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
        case 'Filip'
            box_in.iiF = find(fLines >=4 & fLines <= 25);

            %type='Mu';    % 'SMR1','SMR2','Beta1','Beta2','Alpha'
            %cond='VLH';  % ,'VLH';  %'VLH'
            %lat ='LR'; %  'L','R','LR';
            meanT='Mean';

            load('atoms205')
            eval(['wS=res.',box_in.type,'.',box_in.cond,'.',box_in.lat,'.elecW.',meanT,''';']);
            eval(['wF=res.',box_in.type,'.',box_in.cond,'.',box_in.lat,'.freqW.',meanT,''';']); 

            %box_in.threshold = 0.09; % higher threshold the more difficult it will be
            %box_in.threshold_window_length = 4; % 4 = 2 seconds
    end

    box_in.P  = krb(wS,wF);
    box_in.PP = box_in.P'*box_in.P;
    
    %box_in.TMP=[];

    %%%% create output header
    box_in = OV_setStimulationOutputHeader(box_in, 1);
    box_in = OV_setStimulationOutputHeader(box_in, 2);
    box_in = OV_setStimulationOutputHeader(box_in, 3);

    %%%% saving
    currentTimeAndDate = now;
    format_time = 'yyyy.mm.dd-HH.MM.SS';
    %%%% params
    params_path = ['merania/', box_in.filename_prefix, '-params-', box_in.subName, '-', datestr(currentTimeAndDate, format_time), '.mat'];
    save(params_path, 'box_in');
    %%%% 1D signal
    one_dim_path = ['merania/', box_in.filename_prefix, '-one_dim-', box_in.subName, '-', datestr(currentTimeAndDate, format_time), '.txt'];
    box_in.f_one_dim_id = fopen(one_dim_path, 'at');
    %%%% raw signal
    raw_path = ['merania/', box_in.filename_prefix, '-raw_signal-', box_in.subName, '-', datestr(currentTimeAndDate, format_time), '.csv'];
    box_in.f_raw_id = fopen(raw_path, 'at');
    
    box_out = box_in;
end
