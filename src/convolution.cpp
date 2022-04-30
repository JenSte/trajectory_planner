#include "trajectory_planner/convolution.hpp"

#include <fftw3.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace trajectory_planner
{

FFTPlan::FFTPlan(
    fftw_plan plan,
    bool forward,
    size_t time_domain_width,
    size_t time_domain_height,
    size_t frequency_domain_width,
    size_t frequency_domain_height)
    : plan_(plan)
    , forward_(forward)
    , time_domain_width_(time_domain_width)
    , time_domain_height_(time_domain_height)
    , frequency_domain_width_(frequency_domain_width)
    , frequency_domain_height_(frequency_domain_height)
    , plan_valid_(true)
{
}

FFTPlan::~FFTPlan()
{
    reset();
}

FFTPlan FFTPlan::plan_forward(
    Buffer<double>& time_domain_buffer)
{
    size_t time_domain_width = time_domain_buffer.width();
    size_t time_domain_height = time_domain_buffer.height();

    size_t frequency_domain_width = time_domain_width / 2 + 1;
    size_t frequency_domain_height = time_domain_height;

    Buffer<fftw_complex> output(frequency_domain_width, frequency_domain_height);

    fftw_plan plan = fftw_plan_dft_r2c_2d(
        time_domain_height,
        time_domain_width,
        time_domain_buffer.raw(),
        output.raw(),
        FFTW_ESTIMATE);

    if (plan == NULL) {
        throw std::runtime_error("Unable to create FFT plan.");
    }

    return FFTPlan(
        plan,
        true,
        time_domain_width,
        time_domain_height,
        frequency_domain_width,
        frequency_domain_height);
}

FFTPlan FFTPlan::plan_backward(
    Buffer<double>& time_domain_buffer)
{
    size_t time_domain_width = time_domain_buffer.width();
    size_t time_domain_height = time_domain_buffer.height();

    size_t frequency_domain_width = time_domain_width / 2 + 1;
    size_t frequency_domain_height = time_domain_height;

    Buffer<fftw_complex> input(frequency_domain_width, frequency_domain_height);

    fftw_plan plan = fftw_plan_dft_c2r_2d(
        time_domain_height,
        time_domain_width,
        input.raw(),
        time_domain_buffer.raw(),
        FFTW_ESTIMATE);

    if (plan == NULL) {
        throw std::runtime_error("Unable to create FFT plan.");
    }

    return FFTPlan(
        plan,
        false,
        time_domain_width,
        time_domain_height,
        frequency_domain_width,
        frequency_domain_height);
}

void FFTPlan::execute(
    Buffer<double>& input,
    Buffer<fftw_complex>& output) const
{
    if (!forward_) {
        throw std::runtime_error("Executing a backward FFT with a forward plan.");
    }

    if (input.width() != time_domain_width_) {
        throw std::runtime_error("Input buffer has wrong width.");
    }

    if (input.height() != time_domain_height_) {
        throw std::runtime_error("Input buffer has wrong width.");
    }

    if (output.width() != frequency_domain_width_) {
        throw std::runtime_error("Output buffer has wrong width.");
    }

    if (output.height() != frequency_domain_height_) {
        throw std::runtime_error("Output buffer has wrong width.");
    }

    fftw_execute_dft_r2c(plan_, input.raw(), output.raw());
}

void FFTPlan::execute(
    Buffer<fftw_complex>& input,
    Buffer<double>& output) const
{
    if (forward_) {
        throw std::runtime_error("Executing a forward FFT with a backward plan.");
    }

    if (input.width() != frequency_domain_width_) {
        throw std::runtime_error("Input buffer has wrong width.");
    }

    if (input.height() != frequency_domain_height_) {
        throw std::runtime_error("Input buffer has wrong width.");
    }

    if (output.width() != time_domain_width_) {
        throw std::runtime_error("Output buffer has wrong width.");
    }

    if (output.height() != time_domain_height_) {
        throw std::runtime_error("Output buffer has wrong width.");
    }

    fftw_execute_dft_c2r(plan_, input.raw(), output.raw());
}

size_t FFTPlan::frequency_domain_width() const
{
    return frequency_domain_width_;
}

size_t FFTPlan::frequency_domain_height() const
{
    return frequency_domain_height_;
}

void FFTPlan::reset()
{
    if (plan_valid_) {
        fftw_destroy_plan(plan_);
    }
}

size_t fast_fft_size(
    size_t size)
{
    // Pre-computed vector containing numbers that are in the form 2^a+3^b+5^c+7^d,
    // sizes which are best suited for FFTW.
    static std::vector<size_t> sizes;

    if (sizes.empty()) {
        // We precompute values up to this value. Bigger arguments are returned
        // unmodified, but given that this value, with a typical map resolution of
        // 0.02 m/pixel would result in a maximum map size of more than 20 km, it's
        // unlikely that we ever run in this limit.
        size_t maxval = 1024 * 1024;

        size_t factor_2 = 1;

        double stop_2 = log(maxval);
        stop_2 /= log(2.0);
        for (size_t i_2 = 0; i_2 <= floor(stop_2); i_2++) {
            size_t factor_3 = factor_2;

            double stop_3 = log(maxval);
            stop_3 -= i_2 * log(2.0);
            stop_3 /= log(3.0);
            for (size_t i_3 = 0; i_3 <= floor(stop_3); i_3++) {
                size_t factor_5 = factor_3;

                double stop_5 = log(maxval);
                stop_5 -= i_2 * log(2.0);
                stop_5 -= i_3 * log(3.0);
                stop_5 /= log(5.0);
                for (size_t i_5 = 0; i_5 <= floor(stop_5); i_5++) {
                    size_t factor_7 = factor_5;

                    double stop_7 = log(maxval);
                    stop_7 -= i_2 * log(2.0);
                    stop_7 -= i_3 * log(3.0);
                    stop_7 -= i_5 * log(5.0);
                    stop_7 /= log(7.0);
                    for (size_t i_7 = 0; i_7 <= floor(stop_7); i_7++) {
                        sizes.push_back(factor_7);
                        factor_7 *= 7;
                    }
                    factor_5 *= 5;
                }
                factor_3 *= 3;
            }
            factor_2 *= 2;
        }

        // Sort the numbers so we can use a binary search for lookup below.
        std::sort(sizes.begin(), sizes.end());
    }

    const auto it = std::lower_bound(sizes.begin(), sizes.end(), size);
    if (it == sizes.end()) {
        // The argument is bigger than any of our pre-calculated values.
        return size;
    }

    return *it;
}

}
