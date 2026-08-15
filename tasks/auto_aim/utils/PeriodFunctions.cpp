#include "utils/PeriodFunctions.h"


std::vector<double> computeModifiedACF(const std::vector<double>& residual) {
    int n = residual.size();
    if (n == 0) return {};
    
    double residual_mean = 0.0;
    for (double val : residual) {
        residual_mean += val;
    }
    residual_mean /= n;
    
    int max_lag = static_cast<int>(n * 0.8);
    std::vector<double> modified_acf(max_lag + 1, 0.0);
    
    for (int k = 0; k <= max_lag; k++) {
        if (k == 0) {
            double sum = 0.0;
            for (double val : residual) {
                sum += (val - residual_mean) * (val - residual_mean);
            }
            modified_acf[k] = sum / n;
        } else {
            double sum = 0.0;
            for (int i = 0; i < n - k; i++) {
                sum += (residual[i] - residual_mean) * (residual[i + k] - residual_mean);
            }
            modified_acf[k] = sum / (n - k);
        }
    }
    
    return modified_acf;
}

std::vector<double> lagStackWithDecay(const std::vector<double>& signal, int refineMultiple) {
    std::vector<double> refined_signal = linearInterpolation(signal, refineMultiple);
    int result_len = refined_signal.size();
    std::vector<double> result(result_len, 0.0);
    
    for (int lag = 1; lag < result_len; lag++) {
        int lag_n = result_len / lag;
        int lag_left = result_len - lag_n * lag;
        
        std::vector<double> temp(lag, 0.0);
        for (int lag_i = 0; lag_i < lag_n; lag_i++) {
            for (int j = 0; j < lag; j++) {
                temp[j] += refined_signal[lag_i * lag + j];
            }
        }
        
        if (lag_left > 0) {
            for (int j = 0; j < lag_left; j++) {
                temp[j] += refined_signal[result_len - lag_left + j];
                temp[j] /= (lag_n + 1);
            }
            for (int j = lag_left; j < lag; j++) {
                temp[j] /= lag_n;
            }
        } else {
            for (int j = 0; j < lag; j++) {
                temp[j] /= lag_n;
            }
        }
        
        std::vector<double> temp_vec(temp.begin(), temp.end());
        result[lag] = variance(temp_vec) / lag;
    }
    
    return result;
}