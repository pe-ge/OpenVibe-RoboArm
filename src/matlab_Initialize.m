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
	box_in.OVTK_StimulationId_ExperimentStop = hex2dec('00008002');
	%%%% ploting window
	box_in.signal_x = [];
	box_in.signal_y = [];
	%%%% time counter
	box_in.time = 2;
	%%%% start time of signal processing
	box_in.initial_time = 2.5;
	%%%% number of runs
	box_in.num_runs = box_in.settings(1).value;
	box_in.current_run = 1;
	
    Tony=true; 
    
    if Tony
	    box_in.iiF = find(fLines >=4 & fLines <= 25);
        load resPARAFAC.mat
		box_in.res = res;
        box_in.atomN=[4];
		
		box_in.threshold = 0.9; % higher threshold the more difficult it will be
        box_in.threshold_window_length = 4; % 4 = 2 seconds
        box_in.a_relax = 13;
        box_in.b_pause = 8;
        box_in.c_robot = 20;
        box_in.d_pause = 7;
		
		box_in.a_relax = 2;
        box_in.b_pause = 2;
        box_in.c_robot = 2;
        box_in.d_pause = 2;
    else
	    box_in.iiF = find(fLines >=4 & fLines <= 25);
        %load vahyPARAFAC_Igor
        %load weights_igor
        %load resPARAFAC-v1-305;

        load resPARAFAC-newKarin %karina
        %load resPARAFAC-301v2 %timotej

		%load removedAtoms_resPARAFAC-306-atom4
        % load resPARAFAC-v2-305 %lucia
        %load resPARAFAC-v1-303 %petra
        %%%%% atom weights
		box_in.res = res;
        box_in.atomN=[4]; %karina
        %atomN=[7]; %timotej
        

        %atomN=[1]; %lucija 1 / MU, 4  SMR
        %atomN=[2]; %petra  mu/6, smr-2
        %
        %atomN=[3]; % igor mu rhythm
        % atomN=[1]; % igor beta rhythm

		%res=res1; 
        %clear res1; 
		
		% karina treshold 0.9 time 6 3s
        box_in.threshold = 0.7; % higher threshold the more difficult it will be
        box_in.threshold_window_length = 5; % 4 = 2 seconds
        box_in.initial_time = 2.5;
        box_in.a_relax = 10;
        box_in.b_pause = 8;
        box_in.c_robot = 20;
        box_in.d_pause = 10;
    end
    
    %[n1,n2,n3]=size(res.Xtrain); 

    %nEpoch  = n1; 
    %nElect  = n2; 
    %nFlines = n3; 

    %fact=1:length(atomN); %%% tu moze byt viac faktorov, zavisi ako sa daju atomN
    %nTS=length(atomN);  %%%  number of time scores/ atoms 
    
    %P  = krb(A{1}(:,fact),A{2}(:,fact)); 
    box_in.P  = krb(box_in.res.Xfactors{2}, box_in.res.Xfactors{3});
    box_in.PP = box_in.P'*box_in.P;
    
	%%%% create output header
    box_in = OV_setStimulationOutputHeader(box_in, 1);
    box_in = OV_setStimulationOutputHeader(box_in, 2);
	box_in = OV_setStimulationOutputHeader(box_in, 3);
    
    box_out = box_in;
end