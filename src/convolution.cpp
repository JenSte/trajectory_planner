#include "trajectory_planner/convolution.hpp"

#include <fftw3.h>

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

    size_t frequency_domain_width = time_domain_width;
    size_t frequency_domain_height = time_domain_height / 2 + 1;

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

    size_t frequency_domain_width = time_domain_width;
    size_t frequency_domain_height = time_domain_height / 2 + 1;

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


}
