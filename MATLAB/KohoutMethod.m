%% Testing (Made by Claude)
%{
fs   = 2;
t    = (0:999)' / fs;
% Add a 0.5m amplitude wave at 12s period on top of the noise
wave_acc = -5 * (2*pi/12)^2 * sin(2*pi/12 * t);
raw_data = wave_acc + 0.01 * randn(1000, 1);
raw_data = raw_data + 9.8065 + 1 * randn(1000, 1);
raw_data([50, 200, 500, 850]) = [100, -85, 150, 90];

[clean_data, spikes, num_spikes] = remove_spikes(raw_data, 6, 5);
mu    = mean(clean_data);
sigma = std(clean_data);

fs = 2;
f1 = 0.02;
f2 = 0.03;
f_data = fiif(clean_data, fs, f1, f2);

N      = numel(clean_data);
f_axis = (0:floor(N/2)) * (fs/N);
psd    = abs(f_data(1:floor(N/2)+1)).^2;

fprintf('Original mean (with spikes): %.4f\n', mean(raw_data));
fprintf('Original std  (with spikes): %.4f\n', std(raw_data));
fprintf('----------------------------------\n');
fprintf('Cleaned mean: %.4f\n', mu);
fprintf('Cleaned std:  %.4f\n', sigma);
fprintf('Total spikes found: %d\n', num_spikes);

figure;
subplot(3,1,1);
hold on;
plot(raw_data, 'b-', 'DisplayName', 'Raw Data');
plot(find(spikes), raw_data(spikes), 'ro', 'MarkerSize', 8, ...
    'LineWidth', 2, 'DisplayName', 'Identified Spikes');
plot(clean_data, 'g-', 'DisplayName', 'Cleaned Data');
yline(mu + 6*sigma, 'k--', 'Upper 6\sigma');
yline(mu - 6*sigma, 'k--', 'Lower 6\sigma');
yline( 0.5*9.8067,  'r--', '0.5g bound');
yline(-0.5*9.8067,  'r--', '-0.5g bound');
legend; title('Spike Removal'); xlabel('Sample'); ylabel('Acceleration');
hold off;

subplot(3,1,2);
plot(f_axis, abs(f_data(1:floor(N/2)+1)));
xlabel('Frequency (Hz)'); ylabel('|FFT|');
title('Frequency domain after integration weights');
xlim([0 1]); grid on;

subplot(3,1,3);
plot(f_axis, psd);
xlabel('Frequency (Hz)'); ylabel('PSD');
title('Power spectral density of displacement');
xlim([0 1]); grid on;
%}

%% Testing (Made by Justin)

datapoints = 1000000;

fs = 640;
t    = (0:datapoints-1)' / fs;
% Add a 0.5m amplitude wave at 12s period on top of the noise
wave_acc = -5 * (2*pi/12)^2 * sin(2*pi/12 * t);
wave_acc = -2 * (2*pi/5)^2 * sin(2*pi/5 * t);
wave_acc = 0;
raw_data = wave_acc + 0.01 * randn(datapoints, 1);
raw_data = raw_data + 9.8065 + 1 * randn(datapoints, 1);
raw_data([50, 200, 500, 850]) = [100, -85, 150, 90];
 
[unspiked, spikes, num_spikes] = remove_spikes(raw_data, 6, 5);
data = filter_and_decimate(unspiked, fs, 8, 1);
fs = 8;
data2 = remove_trend(data);
[clean_data, spikes2, num_spikes2] = remove_spikes(data2, 6, 5);
clean_data = filter_and_decimate(clean_data, fs, 2, 0.5);
fs = 2;

displacement_data = fiifif(clean_data, 2, 0.02, 0.03);

avg_psd = psd_avg(displacement_data, 5, 5);

M   = numel(avg_psd);          % segment length
f_axis = (0:M-1)' * (fs/M);   % frequency axis matching segment length

moments = spectral_moments(avg_psd, f_axis);


figure;
subplot(2,1,1);
plot(f_axis(1:floor(M/2)), avg_psd(1:floor(M/2)));
xlabel('Frequency (Hz)'); ylabel('PSD');
title('Averaged PSD (one-sided)');
xlim([0 fs/2]); grid on;

subplot(2,1,2);
period_axis = 1 ./ f_axis(2:floor(M/2));   % skip DC bin, convert to period
plot(period_axis, avg_psd(2:floor(M/2)));
xlabel('Period (s)'); ylabel('PSD');
title('Averaged PSD vs period');
xlim([4 50]); grid on;                      % wave band of interest
%}

%% Filter Values (made by Claude)
% Stage 1: 400Hz -> 10Hz (or whatever your raw rate is)
% Cutoff at fs_out/4 as discussed
fs1_in  = 416;
fs1_out = 8;
fc1     = 1;   % 1 Hz cutoff
[b1, a1] = butter(2, fc1 / (fs1_in/2), 'low');
fprintf('Stage 1 b: %.10f %.10f %.10f\n', b1(1), b1(2), b1(3));
fprintf('Stage 1 a: %.10f %.10f %.10f\n', a1(1), a1(2), a1(3));

% Stage 2: 8Hz -> 2Hz  (if you want a second stage)
fs2_in  = fs1_out;
fs2_out = 2;
fc2     = fs2_out / 4;   % 0.5 Hz cutoff
[b2, a2] = butter(2, fc2 / (fs2_in/2), 'low');
fprintf('Stage 2 b: %.10f %.10f %.10f\n', b2(1), b2(2), b2(3));
fprintf('Stage 2 a: %.10f %.10f %.10f\n', a2(1), a2(2), a2(3));

%% Create cpp header for pre-write testing:
t = 0:1/416:10/6;
t2 = 1/8:1/8:10/6;
t3 = 1/2:1/2:10/6;
points = length(t);
s1 = 0.3 * sin(2*pi*t*0.9);
s2 = 0.4 * sin(2*pi*t*0.2);
s3 = 0.1 * sin(2*pi*t*2);
s4 = 0.2 * sin(2*pi*t*0.15);
noise = 0.05 * randn(1, points);
offset = -1;
test_data = s1 + s2 + s3 + s4 + noise + offset;
spike_val = 10;
test_data(200) = spike_val;
test_data(123) = -spike_val;
test_data(573) = spike_val;
test_data(305) = -spike_val;
test_data(25:45) = test_data(24);


fid = fopen('../VSCode2/Test/include/test_data.h', 'w');
fprintf(fid, 'const int TEST_DATA_LEN = %d;\n', points);
fprintf(fid, 'float test_input[] = {\n');
fprintf(fid, '%f, ', test_data);
fprintf(fid, '\n};');
fclose(fid);

%% TEST CPP FUNCTIONS ON MC:
% 1. Setup the Serial Connection
% CHANGE 'COM5' to whatever port your ItsyBitsy shows up on (e.g., '/dev/tty.usbmodem...' on Mac)
baud_rate = 115200;
port_name = 'COM5'; 

% Clear any old connections to prevent port lockouts
if exist('mcu', 'var')
    clear mcu;
end

% Open the port
mcu = serialport(port_name, baud_rate);

configureTerminator(mcu, "LF");

% 2. Prepare the data array
% This should match the TEST_DATA_LEN from your C++ code 
cpp_results = zeros(points, 1);
cpp_results2 = zeros(length(t2), 1);
cpp_results3 = zeros(length(t3), 1);

pause(2);
disp('Waiting for data from microcontroller...');
write(mcu, "G", "uint8");
%{
mcu_mu = str2double(readline(mcu));
mcu_sd = str2double(readline(mcu));
mu = mean(test_data);
sd = std(test_data);
%}

% 3. Read the data line by line
for i = 1:points
    % Read one line from the serial port (blocks until data arrives)
    raw_line = readline(mcu); 
    
    % Convert the text back into a number
    cpp_results(i) = str2double(raw_line);
end
numSpikes = str2num(readline(mcu));
numUnresponsive = str2num(readline(mcu));

for i = 1:length(t2)
    % Read one line from the serial port (blocks until data arrives)
    raw_line = readline(mcu); 
    
    % Convert the text back into a number
    cpp_results2(i) = str2double(raw_line);
end
for i = 1:length(t3)
    cpp_results3(i) = str2double(readline(mcu));
end
[unspiked, x, x] = remove_spikes(test_data, 6, 5);

numSpikes2 = str2num(readline(mcu));
numUnresponsive2 = str2num(readline(mcu));
mcu_mu = str2double(readline(mcu));
mcu_sd = str2double(readline(mcu));
mcu_points = str2num(readline(mcu));
disp("Spikes:");
disp(numSpikes);
disp("Unresponsive:");
disp(numUnresponsive);
disp("MCU MU:");
disp(mcu_mu);
disp("MCU SSD:");
disp(mcu_sd);
disp("MCU POINTS:");
disp(mcu_points);
disp("MU:");
disp(mean(unspiked));
disp("SD:");
disp(std(unspiked));
% Close the port so the Arduino IDE can use it again
clear mcu;

% 4. Validate!
% Compare MATLAB's expected output with what the ARM chip calculated
demeaned = unspiked - mean(unspiked);
decimated1 = filter_and_decimate(demeaned, 416, 8, 1);
detrended = remove_trend(decimated1);
detrended = remove_spikes(detrended, 6, 5);
decimated2 = filter_and_decimate(detrended, 8, 2, 0.5);
plot(t', cpp_results);
hold on;
plot(t', test_data);
plot(t', demeaned);
plot(t2', cpp_results2);
plot(t2', decimated1);
plot(t2', detrended);
plot(t3', cpp_results3);
plot(t3', decimated2);
legend("MCU Cleaned", "Actual data", "Matlab Cleaned", "MCU dec1", "Matlab dec1", "Matlab Detrended", "MCU dec2", "Matlab dec2");
title('Output from ItsyBitsy M4 (ARM Cortex-M4)');
%}

%% Create cpp header for post-write testing:
t = 0:1/416:1500;

s1 = 0.3 * sin(2*pi*t*0.9);
s2 = 0.4 * sin(2*pi*t*0.2);
s3 = 0.1 * sin(2*pi*t*2);
s4 = 0.2 * sin(2*pi*t*0.15);
noise = 0.05 * randn(1, length(t));
offset = -1;
test_data = s1 + s2 + s3 + s4 + noise + offset;
spike_val = 10;
test_data(200) = spike_val;
test_data(123) = -spike_val;
test_data(573) = spike_val;
test_data(305) = -spike_val;
test_data(25:45) = test_data(24);

unspiked = remove_spikes(test_data, 6, 5);
demeaned = unspiked - mean(unspiked);
decimated1 = filter_and_decimate(demeaned, 416, 8, 1);
detrended = remove_trend(decimated1);
decimated2 = filter_and_decimate(detrended, 8, 2, 0.5);

points = length(decimated2);
t2 = 1/2:1/2:1500;

fid = fopen('../VSCode2/Test/include/test_data.h', 'w');
fprintf(fid, 'const int TEST_DATA_LEN = %d;\n', points);
fprintf(fid, 'float test_input[] = {\n');
fprintf(fid, '%f, ', decimated2);
fprintf(fid, '\n};');
fprintf(fid, '\nstatic float RAO[] = {\n');
fprintf(fid, '%f, ', ones(points, 1));
fprintf(fid, '\n};');
fclose(fid);

plot(t2',decimated2);
%% TEST CPP FUNCTIONS ON MC:
% 1. Setup the Serial Connection
% CHANGE 'COM5' to whatever port your ItsyBitsy shows up on (e.g., '/dev/tty.usbmodem...' on Mac)
baud_rate = 115200;
port_name = 'COM5'; 

% Clear any old connections to prevent port lockouts
if exist('mcu', 'var')
    clear mcu;
end

% Open the port
mcu = serialport(port_name, baud_rate);

configureTerminator(mcu, "LF");

% 2. Prepare the data array
% This should match the TEST_DATA_LEN from your C++ code 
cpp_results = zeros(points, 1);
cpp_results2 = zeros(points-480, 1);
cpp_results3 = zeros(513, 1);

pause(2);
disp('Waiting for data from microcontroller...');
write(mcu, "G", "uint8");

for i = 1:points
    cpp_results(i) = str2double(readline(mcu));
end
for i = 1:points-440
    cpp_results2(i) = str2double(readline(mcu));
end
for i = 1:513
    cpp_results3(i) = str2double(readline(mcu));
end
mcu_mm2 = str2double(readline(mcu));
mcu_mm1 = str2double(readline(mcu));
mcu_m0 = str2double(readline(mcu));
mcu_m1 = str2double(readline(mcu));
mcu_m2 = str2double(readline(mcu));
mcu_m3 = str2double(readline(mcu));
mcu_swh = str2double(readline(mcu));

mcu_moments = [mcu_mm2, mcu_mm1, mcu_m0, mcu_m1, mcu_m2, mcu_m3];
disp("Completed. Closing SPI...");
clear mcu;

% Validation
displacement = fiifif(decimated2', 2, 0.02, 0.03);
displacement_segs = [displacement(1:512), displacement(513:1024), displacement(1025:1536), displacement(1537:2048), displacement(2049:2560)];
avg_psd = psd_avg(displacement, 4, 5, 2);
f_axis = 0:1/512:1;
moments = spectral_moments(avg_psd, f_axis);

for i = 1:6
    fprintf("MCU moment %d: %d\n", i-3, mcu_moments(i));
    fprintf("Matlab moment %d: %d\n", i-3, moments(i));
end
% Plotting
hold on;
plot(t2(1:2560), displacement);
%plot(t2, cpp_results);
plot(t2(1:2560), cpp_results2);
%plot(f_axis, cpp_results3);
%plot(f_axis, avg_psd);
%legend("MATLAB avgPSD", "MCU avgPSD");


%% Function Definitions
function [total_unresponsive, max_consecutive, frac] = num_cons_unresponsive(data)
    EPSILON = 0.00001;

    total_unresponsive = 0;
    max_consecutive = 0;
    run_length = 0;
    n = numel(data);
    for i=2:n
        if (abs(data(i)-data(i-1)) < EPSILON)
            run_length = run_length+1;
            total_unresponsive = total_unresponsive + 1;
            max_consecutive = max(run_length, max_consecutive);
        else
            run_length = 0;
        end
    end
    frac = total_unresponsive/n;
end

function [unspiked, spikes, num_spikes] = remove_spikes(data, std_devs, max_iter)
    g = 9.8067;
    upper_bound = 1.5 * g;

    is_spike = false(size(data));

    for iter = 1:max_iter
        valid_data = data(~is_spike);
        mu  = mean(valid_data);
        sd  = std(valid_data);

        % Flag statistical spikes AND physically impossible values
        new_is_spike = (abs(data - mu) > std_devs * sd) | (abs(data) > upper_bound);

        if isequal(new_is_spike, is_spike)
            break   % No new spikes remain
        end

        is_spike = new_is_spike;
    end

    num_spikes = sum(is_spike);
    spikes = is_spike;
    unspiked = interpolate_spikes(data, is_spike);
end

function data = interpolate_spikes(data, is_spike)
    n = numel(data);
    spike_idx = find(is_spike);

    for k=1:numel(spike_idx)
        i = spike_idx(k);

        left = i - 1;
        right = i + 1;

        while (left >= 1 && is_spike(left))
            left = left - 1;
        end
        while (right <= n && is_spike(right))
            right = right + 1;
        end

        if (left >= 1 && right <= n)
            data(i) = data(left) + (data(right) - data(left)) * (i - left) / (right - left);
        elseif (left < 1)
            r1 = right;
            r2 = right + 1;

            while (r2 <= n && is_spike(r2))
                right = right + 1;
            end

            if (r2 <= n)
                slope = data(r2) - data(r1);
                data(i) = data(r1) - slope * (r1 - i);
            else 
                data(i) = data(r1);
            end
        else
            l1 = left;
            l2 = left - 1;

            while (l2 >= 1 && is_spike(l2))
                l2 = l2 - 1;
            end

            if (l2 >= 1)
                slope = data(l1 - data(l2));
                data(i) = data(l1) + slope * (i - l1);
            else
                data(i) = data(l1);
            end
        end
    end
end

function data = remove_trend(data)
    k = 0.9995;
    points = numel(data);
    s_prev = 0;
    
    for n = 1:points
        s = data(n) + k * s_prev;    
        data(n) = data(n) - (1 - k) * s;
        s_prev = s;
    end
end

function vert_acc = vertical(x_acc, y_acc, z_acc, roll, pitch)
    vert_acc = -9.8067 * (sin(pitch) * x_acc + sin(roll) * cos(pitch) * y_acc + cos(roll) * cos(pitch) * z_acc);
end

function data = filter_and_decimate(data, fs_raw, fs_out, cut_off)
    ORDER = 2;

    dec_factor = fs_raw / fs_out;

    assert(mod(dec_factor, 1) == 0, 'Decimation factor (fs_raw/fs_out) must be an integer.');

    normalised_cutoff = cut_off / (fs_raw / 2);

    [b, a] = butter(ORDER, normalised_cutoff, 'low');

    filtered = filter(b, a, data);

    data = filtered(dec_factor:dec_factor:end);
end

function f_data = fiif(data, fs, f1, f2)
    N = numel(data);
    
    % Standard FFT frequency array (0 to fs)
    f = (0:N-1) * (fs / N);
    
    % NEW: Create a mirrored array for absolute physical frequencies
    f_phys = f;
    nyquist_idx = f > (fs/2);
    f_phys(nyquist_idx) = fs - f(nyquist_idx); 
    
    fc = fs/2;
    
    % Pass the mirrored frequencies to your math functions
    weights = compute_R(f_phys, f1, f2, fc);
    integral = f_integral(f_phys, N);
    
    int_filt = weights .* integral;
    f_data = fft(data);
    f_data = f_data .* int_filt;
end

function t_disp = fiifif(data, fs, f1, f2)
    f_data = fiif(data, fs, f1, f2);
    t_disp = real(ifft(f_data));
    
    time_to_cut = 1.1 / (f2 - f1);
    samples_to_cut = floor(fs * time_to_cut);
    t_disp = t_disp(samples_to_cut+1:end-samples_to_cut);
end

function R = compute_R(f, f1, f2, fc)
    R = zeros(numel(f), 1);
    for k = 1:numel(f)
        fk = f(k);
        if (fk <= f1)
            R(k) = 0;
        elseif (fk < f2)
            R(k) = 0.5 * (1 - cos(pi * (fk - f1) / (f2 - f1)));
        elseif (fk < fc)
            R(k) = 1;
        else 
            R(k) = 0;
        end
    end
end

function integral = f_integral(f, bins)
    integral = zeros(bins, 1);
    for k = 2:bins
        omega = 2 * pi * f(k);
        integral(k)  = -1 / omega^2;
    end
end

function p = PSD(t_data, taper_coeff, fs)
    N = numel(t_data);
    w = partial_cosine_taper(N, taper_coeff);
    windowed = t_data .* w;
    f_data = fft(windowed, N);
    taper_power = sum(w.^2) / N;
    denom = N * fs * taper_power;
    p = 2 * abs(f_data).^2 / denom;
end

function w = partial_cosine_taper(N, gamma)
    w         = ones(N, 1);
    taper_len = floor(N * gamma / 100);

    for i = 1:taper_len
        w(i)         = 0.5 * (1 - cos(pi * (i-1) / taper_len));
        % Mirror at the other end
        w(N+1-i)     = w(i);
    end
end

function parts = partition(data, segments)
    n = numel(data);
    m = n / (segments + 1);
    assert(mod(m, 1) == 0, 'Division size (numel(data)/(segments+1)) must be an integer.');
    parts = zeros(segments, 2*m);
    for i = 1:segments
        segment = data((i-1)*m+1:(i+1)*m)';
        parts(i,:) = segment;
    end 
end

function avg_psd = psd_avg(data, segments, taper_coeff, fs)
    parts = partition(data, segments);
    psd = zeros(size(parts,2),1);
    for i=1:segments
        part = parts(i,:)';
        current_psd = PSD(part, taper_coeff, fs);
        psd(:) = psd(:) + current_psd;
    end
    avg_psd = psd ./ segments;
    avg_psd = avg_psd(1:length(avg_psd)/2+1); % convert to one-sided
end

function [p_avg, f_axis] = psd_avg2(data_segments, fs, window)
    % Assuming data_segments is a matrix where each column is one segment
    % and 'window' is your taper array.
    
    N = size(data_segments, 1);   % Length of one segment
    N_fft = 2 * N;                % Your zero-padded FFT length
    num_segments = size(data_segments, 2);
    
    % Number of one-sided bins (DC to Nyquist)
    num_bins = floor(N_fft / 2) + 1;
    
    % 1. Create the correct frequency axis
    % Bin indices go from 0 to (num_bins - 1)
    f_axis = (0:(num_bins - 1))' * (fs / N_fft);
    
    % Initialize an array to hold the sum of the PSDs
    p_sum = zeros(num_bins, 1);
    
    % Normalization factor for windowed signal
    norm_factor = fs * sum(window.^2);
    
    for i = 1:num_segments
        % Apply window
        windowed = data_segments(:, i) .* window;
        
        % Compute FFT
        f_data = fft(windowed, N_fft);
        
        % 2. Extract only the one-sided spectrum (positive frequencies)
        f_data_onesided = f_data(1:num_bins);
        
        % 3. Calculate Power Spectral Density for this segment
        p_segment = (1 / norm_factor) * abs(f_data_onesided).^2;
        
        % 4. Conserve power (Multiply by 2, except for DC and Nyquist)
        % We discarded the negative frequencies, so their power must be 
        % added back to the positive frequencies.
        p_segment(2:end-1) = 2 * p_segment(2:end-1);
        
        % Add to our running total
        p_sum = p_sum + p_segment;
    end
    
    % Average the PSDs
    p_avg = p_sum / num_segments;
end

function moments = spectral_moments(psd, f_axis)
    delta_f = f_axis(2)-f_axis(1);
    moments = zeros(6,1);

    valid   = f_axis > 0;
    f_valid = f_axis(valid);
    p_valid = psd(valid);
    for n = -2:3
        tot = sum((f_valid .^ n) .* p_valid');
        moments(n+3) = delta_f * tot;
    end
end
