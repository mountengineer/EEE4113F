#include <filter.h>

#define SOME_LEN 1000

static const float b1_0 =  0.0000564276f;  // Stage 1
static const float b1_1 =  0.0001128551f;
static const float b1_2 =  0.0000564276f;
static const float a1_1 =  -1.9786407851f;
static const float a1_2 =  0.9788664954f;

static const float b2_0 =  0.0299545822f;  // Stage 2
static const float b2_1 =  0.0599091644f;
static const float b2_2 =  0.0299545822f;
static const float a2_1 =  -1.4542435863f;
static const float a2_2 =  0.5740619151f;

static bool despike(float *data, uint16_t length, bool *is_spike) {
    bool     have_left = false;  
    float    left_val  = 0.0f;  
    uint16_t spike_start = 0;

    for (uint16_t i = 0; i <= length; i++) {
        // Treat one-past-the-end as a virtual non-spike to flush
        // any spike run that reaches the end of the array
        bool current_is_spike = (i < length) && is_spike[i];

        if (!current_is_spike) {
            if (spike_start < i) {
                uint16_t run_start = spike_start;
                uint16_t run_end   = i; // first valid index after spike run

                if (!have_left && run_end >= length) {
                    // Does not make sense
                    //UNCOMMENT Serial.println("This should be impossible. All spikes.");
                    return false;
                } else if (!have_left) {
                    // Spike run at the very start — extrapolate forward
                    // using slope from first two valid points after the run
                    float r1 = data[run_end];
                    // Find second valid point
                    uint16_t next = run_end + 1;
                    while (next < length && is_spike[next]) {
                        next++;
                    }
                    if (next < length) {
                        float slope = (data[next] - r1) / (float)(next - run_end);
                        // Extrapolate backward: points before run_end
                        for (uint16_t j = run_start; j < run_end; j++) {
                            data[j] = r1 - slope * (float)(run_end - j);
                        }
                    } else {
                        // Does not make sense
                        //UNCOMMENT Serial.println("This should be impossible. Only 1 non-spike");
                        return false;
                    }
                } else if (run_end >= length) {
                    // Spike run at the very end — extrapolate forward
                    // Find second-to-last valid point before run
                    int32_t prev2 = (int32_t)run_start - 2;
                    while (prev2 >= 0 && is_spike[prev2]) {
                        prev2--;
                    }
                    if (prev2 >= 0) {
                        float slope = (left_val - data[prev2]) / (float)(run_start - 1 - prev2);
                        for (uint16_t j = run_start; j < length; j++) {
                            data[j] = left_val + slope * (float)(j - run_start + 1);
                        }
                    } else {
                        // Does not make sense
                        //UNCOMMENT Serial.println("This should be impossible. Only 1 non-spike.");
                        return false;
                    }
                } else {
                    // Standard case: spike run with valid data on both sides
                    // Linear interpolation between left_val and data[run_end]
                    float right_val = data[run_end];
                    float span      = (float)(run_end - run_start + 1);
                    // +1 because we interpolate from left_val (at run_start-1)
                    // to right_val (at run_end), so there are span steps total
                    for (uint16_t j = run_start; j < run_end; j++) {
                        float t  = (float)(j - run_start + 1) / span;
                        data[j]  = left_val + t * (right_val - left_val);
                    }
                }
            }

            // Update left boundary tracker
            if (i < length) {
                have_left  = true;
                left_val   = data[i];
                spike_start = i + 1; // next spike run would start here
            }
        }
    }
    return true;
}

void getStats(float *data, float32_t &mean, float32_t &std, uint16_t length) {
    arm_mean_f32(data, length, &mean);
    arm_std_f32(data, length, &std);
}

void detrend(float &data, float &s_prev) {
    float s;
    s = data + DETREND_K * s_prev;
    data = data - (1 - DETREND_K) * s;
    s_prev = s;
}

bool clean(float *data, uint16_t length, uint16_t &numSpikes, uint16_t &numUnresponsive, SensorChannel &sensor, bool first_pass) {
    float mean, sd;
    float prev; 
    bool error = false;

    static bool is_spike[BUFFER_BASE]; // static to avoid stack pressure
    memset(is_spike, 0, length * sizeof(bool));
    
    for (uint8_t iter = 0; iter < DESPIKE_ITER; iter++) {
        bool have_prev = false;
        static float valid[BUFFER_BASE];
        uint16_t valid_count = 0;
        for (uint16_t i = 0; i < length; i++) {
            if (!is_spike[i])  {
                valid[valid_count++] = data[i];
            }
        }

        if (valid_count < 2) {
            error = true;
            break;
        }
        float weighted_sse, combined_sse, weighted_mean;
        getStats(valid, mean, sd, valid_count);
        if (first_pass) {
            float batch_sse = sd * sd * (float)(valid_count - 1);
            float n_prev    = (float)sensor.n;
            float n_batch   = (float)valid_count;
            float delta     = mean - sensor.mean;
            if (sensor.init) {
                // EWA path: sensor has enough history, allow forgetting
                // Blend means and variances directly — no fake sample count scaling
                float sensor_var  = sensor.sse  / (n_prev - 1.0f);
                float batch_var   = batch_sse   / (n_batch - 1.0f);

                weighted_mean = (1.0f - STATS_ALPHA) * sensor.mean + STATS_ALPHA * mean;
                float combined_var = (1.0f - STATS_ALPHA) * sensor_var
                                +          STATS_ALPHA  * batch_var
                                + STATS_ALPHA * (1.0f - STATS_ALPHA) * delta * delta;

                mean         = weighted_mean;
                combined_sse = combined_var * (n_prev - 1.0f);  // re-encode as SSE using stable n
                sd           = sqrtf(combined_var);
            } else {
                // Exact path: accumulate with Chan's parallel merge formula
                float n_new  = n_prev + n_batch;

                mean         = (sensor.mean * n_prev + mean * n_batch) / n_new;
                combined_sse = sensor.sse + batch_sse + delta * delta * (n_prev * n_batch / n_new);
                sd           = sqrtf(combined_sse / (n_new - 1.0f));

                weighted_mean = mean;        // keep in sync so the save block below is uniform
                weighted_sse  = combined_sse;
            }
        }

        float sd_floor  = 0.005f;   // TODO: Ask Robyn if this is valid
        sd = fmaxf(sd, sd_floor);

        // Serial.print("mean: "); Serial.println(mean, 6);
        // Serial.print("std:  "); Serial.println(sd, 6);
        float min_thresh = mean - (sd * STD_DEVS);
        float max_thresh = mean + (sd * STD_DEVS);
        // Serial.print("min: "); Serial.println(min_thresh, 6);
        // Serial.print("max:  "); Serial.println(max_thresh, 6);

        // Also flag physically impossible values (0.5g upper bound)
        // TODO: this

        bool any_new = false;
        for (uint16_t i = 0; i < length; i++) {
            if (have_prev && data[i] == prev) {
                if (iter == 0) {
                    numUnresponsive++;
                }
            } else {
                have_prev = true;
            }
            prev = data[i];
            if (!is_spike[i]) {
                if (data[i] > max_thresh || data[i] < min_thresh ) {//|| (((fabsf(data[i]) > phys_max) || (fabsf(data[i]) < phys_min)) && is_z_acc)) {
                    is_spike[i] = true;
                    any_new     = true;
                    numSpikes++;
                }
            }
        }

        if (!any_new || iter == DESPIKE_ITER - 1)  { // Update stats on final pass
            if (first_pass) {
                if (sensor.init) {
                    sensor.mean = weighted_mean;
                    sensor.sse = weighted_sse;
                    sensor.init = true;
                } else {
                    sensor.mean = mean;
                    sensor.sse = combined_sse;
                    sensor.n += valid_count;
                    if (sensor.n >= STATS_BATCH_THRESH * UNLOADS_PER_DEC * FIFO_THRESHOLD / AXES) {
                        sensor.init = true;
                    }
                }
            }
            break; // converged — no new spikes found this iteration
        }
    }

    // Now interpolate all flagged spikes in one pass
    error |= !despike(data, length, is_spike);
    if (!error && first_pass) {
        for (uint16_t i = 0; i < length; i++) {
            data[i] -= sensor.mean;
        }
    }
    return !error;
}

SensorChannel initSensorChannel() {
    BiquadState state1;
    BiquadState state2;
    state2.first = false;
    DecimationChannel channel1;
    channel1.filter = state1;
    DecimationChannel channel2;
    channel2.filter = state2;
    FiltDecimator dec1;
    dec1.ch1 = channel1;
    dec1.ch2 = channel2;
    SensorChannel sense;
    sense.dec = dec1;
    return sense;
}

float biquad(float x, BiquadState &state) {
    float y;
    if (state.first) {
        y = b1_0 * x + state.w1;
        float new_w1 = b1_1 * x - a1_1 * y + state.w2;
        float new_w2 = b1_2 * x - a1_2 * y;

        state.w1 = new_w1;
        state.w2 = new_w2;
    } else {
        y = b2_0 * x + state.w1;
        float new_w1 = b2_1 * x - a2_1 * y + state.w2;
        float new_w2 = b2_2 * x - a2_2 * y;

        state.w1 = new_w1;
        state.w2 = new_w2;
    }
    return y;
}

uint16_t filtDecimate(float *input, uint16_t input_length, float* output, uint16_t &numSpikes, uint16_t &numUnresponsive, SensorChannel &sensor) {
    float temp;
    uint16_t j = 0;
    uint16_t intermediate_length = (uint16_t)(input_length/DEC1);
    float intermediate[intermediate_length+1];
    for (uint16_t i = 0; i < input_length; i++) {
        if (filtDecimate(input[i], temp, sensor.dec.ch1)) {
            detrend(temp, sensor.s_prev);
            // Serial.println(temp, 6);
            intermediate[j++] = temp;
        }
    }
    
    clean(intermediate, j, numSpikes, numUnresponsive, sensor, false);
    uint16_t count = 0;
    for (uint16_t i = 0; i < j; i++) {
        if (filtDecimate(intermediate[i], temp, sensor.dec.ch2)) {
            // Serial.println(temp, 6);
            output[count++] = temp;
        }
    }
    return count;
}

bool filtDecimate(float x, float &output, DecimationChannel &channel) {
    float y = biquad(x, channel.filter);
    uint16_t counter_lim;
    channel.counter++;
    if (channel.filter.first) {
        counter_lim = DEC1;
    } else {
        counter_lim = DEC2;
    }
    if (channel.counter >= counter_lim) {
        channel.counter = 0;
        output = y;
        return true;
    }
    return false;
}

uint16_t prepForWrite(float *data, float *output, uint16_t &numSpikes, uint16_t &numUnresponsive, uint16_t length, SensorChannel &sensor) {
    if (!clean(data, length, numSpikes, numUnresponsive, sensor, true)) {
        //UNCOMMENT Serial.println("Error in cleaning.");
    }
    return filtDecimate(data, length, output, numSpikes, numUnresponsive, sensor);
}