%% 1) Get folder where this script is located
scriptDir = fileparts(mfilename('fullpath'));

%% 2) Define Excel file name (your file)
excelFileName = 'data_w_2RC.csv';

%% 3) Build full path to the Excel file
excelFilePath = fullfile(scriptDir, excelFileName);

%% 4) Read the Excel file into a table
T = readtable(excelFilePath);

%% 5) Extract time and ADC data (with conversion)
if iscell(T.pc_timestamp)
    t = datetime(T.pc_timestamp, 'InputFormat','yyyy-MM-dd''T''HH:mm:ss.SSS');
elseif isstring(T.pc_timestamp) || ischar(T.pc_timestamp)
    t = datetime(T.pc_timestamp, 'InputFormat','yyyy-MM-dd''T''HH:mm:ss.SSS');
else
    t = T.pc_timestamp;
end
t_sec = seconds(t - t(1));
x = T.raw_line;
dt = median(diff(t_sec));
Fs = 1 / dt;
fprintf('Estimated Fs = %.2f Hz\n', Fs);

%% 6) FFT of raw signal
x_detrend = x - mean(x);
L  = length(x_detrend);
Y  = fft(x_detrend);
P2 = abs(Y / L);
P1 = P2(1:floor(L/2)+1);
P1(2:end-1) = 2 * P1(2:end-1);
f  = Fs * (0:floor(L/2)) / L;

% Magnitude and dB spectra (raw)
P1_mag = P1;
P1_dB  = 20 * log10(P1 + eps);

%% 7) 2nd-order Butterworth low-pass filter
order = 2;
Fc    = 5;                    % cutoff frequency in Hz (EDIT THIS)
Wn    = Fc / (Fs / 2);        % normalise by Nyquist
[b, a] = butter(order, Wn, 'low');
x_filt = filtfilt(b, a, x);   % zero-phase filtering

%% 8) FFT of filtered signal
x_filt_detrend = x_filt - mean(x_filt);
Y_filt  = fft(x_filt_detrend, L);   % same length as raw for matching f axis
P2_filt = abs(Y_filt / L);
P1_filt = P2_filt(1:floor(L/2)+1);
P1_filt(2:end-1) = 2 * P1_filt(2:end-1);
P1_filt_dB = 20 * log10(P1_filt + eps);

%% 9) Figure 1 — Time domain
figure('Name','Time Domain','NumberTitle','off');

subplot(3,1,1);
plot(t_sec, x, 'Color',[0.2 0.4 0.8]);
xlabel('Time (s)');
ylabel('ADC counts');
title('Raw ADC Signal');
grid on;

subplot(3,1,2);
plot(t_sec, x_filt, 'r');
xlabel('Time (s)');
ylabel('ADC counts');
title(sprintf('Filtered ADC — %d-order Butterworth, Fc = %g Hz', order, Fc));
grid on;

subplot(3,1,3);
plot(t_sec, x,      'Color',[0.5 0.5 0.5]); hold on;
plot(t_sec, x_filt, 'r');
xlabel('Time (s)');
ylabel('ADC counts');
title('Overlay: Raw (gray) vs Filtered (red)');
legend('Raw','Filtered');
grid on;

%% 10) Figure 2 — Frequency domain (raw)
figure('Name','Frequency Domain — Raw','NumberTitle','off');

subplot(2,1,1);
plot(f, P1_mag, 'Color',[0.2 0.4 0.8]);
xlabel('Frequency (Hz)');
ylabel('Magnitude (ADC counts)');
title('Single-Sided Magnitude Spectrum (Raw)');
xlim([0, Fs/2]);
grid on;
xline(Fc, 'r--', sprintf('Fc = %g Hz', Fc), ...
    'LabelVerticalAlignment','bottom', 'LabelHorizontalAlignment','right');

subplot(2,1,2);
plot(f, P1_dB, 'Color',[0.85 0.33 0.10]);
xlabel('Frequency (Hz)');
ylabel('Magnitude (dB)');
title('Single-Sided Spectrum in dB (Raw)');
xlim([0, Fs/2]);
grid on;
xline(Fc, 'r--', sprintf('Fc = %g Hz', Fc), ...
    'LabelVerticalAlignment','bottom', 'LabelHorizontalAlignment','right');
yline(-3, 'k:', '-3 dB', 'LabelHorizontalAlignment','left');

%% 11) Figure 3 — Frequency domain (filtered) + overlay
figure('Name','Frequency Domain — Filtered','NumberTitle','off');

subplot(2,1,1);
plot(f, P1_dB,      'Color',[0.85 0.33 0.10]); hold on;
plot(f, P1_filt_dB, 'Color',[0.13 0.55 0.13], 'LineWidth', 1.2);
xlabel('Frequency (Hz)');
ylabel('Magnitude (dB)');
title(sprintf('Spectrum (dB): Raw vs Filtered — %d-order Butterworth, Fc = %g Hz', order, Fc));
xlim([0, Fs/2]);
grid on;
xline(Fc, 'r--', sprintf('Fc = %g Hz', Fc), ...
    'LabelVerticalAlignment','bottom', 'LabelHorizontalAlignment','right');
yline(-3, 'k:', '-3 dB', 'LabelHorizontalAlignment','left');
legend('Raw','Filtered');

subplot(2,1,2);
plot(f, P1_filt_dB, 'Color',[0.13 0.55 0.13], 'LineWidth', 1.2);
xlabel('Frequency (Hz)');
ylabel('Magnitude (dB)');
title(sprintf('Filtered Signal Spectrum (dB) — Fc = %g Hz', Fc));
xlim([0, Fs/2]);
grid on;
xline(Fc, 'r--', sprintf('Fc = %g Hz', Fc), ...
    'LabelVerticalAlignment','bottom', 'LabelHorizontalAlignment','right');
yline(-3, 'k:', '-3 dB', 'LabelHorizontalAlignment','left');
