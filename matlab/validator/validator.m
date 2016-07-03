clear all;

FILENAME = '2016.06.28-14.14.33';
NUM_CHANNELS = 5;
SAMPLING_RATE = 128; % in Hz
WINDOW_LENGTH_TIME = 2; % in sec
WINDOW_OVERLAP_TIME = 0.5; % in sec
WINDOW_LENGTH = SAMPLING_RATE * WINDOW_LENGTH_TIME;
WINDOW_OVERLAP = SAMPLING_RATE * WINDOW_OVERLAP_TIME;

raw_signal = cell2mat(ReadEDF(['raw-signal-[', FILENAME, '].edf']));
raw_signal = raw_signal(:, 1:NUM_CHANNELS);
params = load(['Tony-', FILENAME, '-params.mat']);
num_samples = size(raw_signal, 1);
num_windows = (num_samples - WINDOW_LENGTH) / WINDOW_OVERLAP + 1;

one_dim_data = [];
for i = 1:num_windows
    % calculate idxs of current window
    window_begin_idx = (i - 1) * WINDOW_OVERLAP + 1;
    window_end_idx = window_begin_idx + WINDOW_LENGTH - 1;
    
    % get signal window
    signal_window = raw_signal(window_begin_idx:window_end_idx, :)';
    
    one_dim = process_signal(params.box_in, signal_window);
    one_dim_data(end + 1) = one_dim;
end