%% Testing (Made by Claude)
% Generate 1000 points of normal data (mean = 10, std = 2)
raw_data = 9.8065 + 1 * randn(1000, 1);

% Inject some massive spikes (well outside 6 standard deviations)
raw_data([50, 200, 500, 850]) = [100, -85, 150, 90];

% Run the function
[clean_data, spikes, num_spikes] = remove_spikes(raw_data, 6, 5);
mu = mean(clean_data);
sigma = std(clean_data);

% Display results
fprintf('Original mean (with spikes): %.2f\n', mean(raw_data));
fprintf('Original std  (with spikes): %.2f\n', std(raw_data));
fprintf('----------------------------------\n');
fprintf('Cleaned mean: %.2f\n', mu);
fprintf('Cleaned std:  %.2f\n', sigma);
fprintf('Total spikes found: %d\n', sum(spikes));

% Plot the results
figure;
hold on;
plot(raw_data, 'b-', 'DisplayName', 'Raw Data');
plot(find(spikes), raw_data(spikes), 'ro', 'MarkerSize', 8, 'LineWidth', 2, 'DisplayName', 'Identified Spikes');
yline(mu + 6*sigma, 'g--', 'Upper 6\sigma Threshold');
yline(mu - 6*sigma, 'g--', 'Lower 6\sigma Threshold');
yline(9.8065*1.5, 'b--', "Maximum Wave Acc");
yline(9.8065*0.5, 'b--', "Minimum Wave Acc");
legend;
title('Iterative Spike Removal');
hold off;

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
    lower_bound = 0.5 * g;

    is_spike = false(size(data));

    for iter = 1:max_iter
        valid_data = data(~is_spike);
        mu  = mean(valid_data);
        sd  = std(valid_data);

        % Flag statistical spikes AND physically impossible values
        new_is_spike = (abs(data - mu) > std_devs * sd) | (abs(data) > upper_bound) | (abs(data) < lower_bound);

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
    s = zeros(points,1);
    data = data - mean(data);

    s(1) = data(1);
    for n = 2:points
        s(n) = data(n) + k * s(n - 1);    
        data(n) = data(n) - (1 - k) * s(n);
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

    filtered = filtfilt(b, a, data);

    data = filtered(1:dec_factor:end);
end

function 
