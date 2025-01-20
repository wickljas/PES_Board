%% Parameters
fs = 500;           % Sampling frequency (Hz), must be multiple of 15
f0 = 15;             % Fundamental notch frequency (Hz)
N  = fs/f0;          % Filter length

N = 33;

%% Design the FIR (moving-average) filter
b = (1/N)*ones(1, N);  % FIR coefficients
a = 1;                 % Denominator for an FIR filter

%% Verify DC Gain
dc_gain = sum(b);      % Should be 1
fprintf('DC gain = %f\n', dc_gain);

%% Plot frequency response
figure;
freqz(b, a, 2048, fs);
title('Frequency Response of the Notching Comb FIR Filter');
