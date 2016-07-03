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