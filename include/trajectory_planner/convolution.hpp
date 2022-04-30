#ifndef TRAJECTORY_PLANNER_CONVOLUTION_HPP
#define TRAJECTORY_PLANNER_CONVOLUTION_HPP

#include "trajectory_planner/buffer.hpp"
#include <tuple>

namespace trajectory_planner
{

// A wrapper around a FFTW plan that destroys the plan when the object is freed.
class FFTPlan
{
public:

    // Create a plan for a forward FFT.
    //
    // Argument is non-const because of the FFTW API, and may be destroyed
    // when creating the plan.
    static FFTPlan plan_forward(
        Buffer<double>& time_domain_buffer);

    // Create a plan for a backward FFT.
    //
    // Argument is non-const because of the FFTW API, and may be destroyed
    // when creating the plan.
    static FFTPlan plan_backward(
        Buffer<double>& time_domain_buffer);

    ~FFTPlan();

    // Execute a forward FFT.
    void execute(
        Buffer<double>& input,
        Buffer<fftw_complex>& output) const;

    // Execute a backward FFT.
    //
    // Note that the FFTW function used to implement this is allowed to overwrite
    // the complex input buffer.
    void execute(
        Buffer<fftw_complex>& input,
        Buffer<double>& output) const;

    // Return the width of the spectrum buffer used by this plan.
    size_t frequency_domain_width() const;

    // Return the height of the spectrum buffer used by this plan.
    size_t frequency_domain_height() const;

private:

    // Private constructor, use one of the factory functions
    // to create an instance of this class.
    FFTPlan(
        fftw_plan plan,
        bool forward,
        size_t time_domain_width,
        size_t time_domain_height,
        size_t frequency_domain_width,
        size_t frequency_domain_height);

    // Destroy the FFTW plan, if necessary.
    void reset();

    FFTPlan(const FFTPlan&) = delete;
    FFTPlan& operator=(const FFTPlan&) = delete;

    FFTPlan(FFTPlan&& other) = delete;
    FFTPlan& operator=(FFTPlan&& other) = delete;

    // The FFTW plan used internally.
    fftw_plan plan_;

    // 'true' if this plan is for a forward FFT, 'false' for a backward FFT.
    bool forward_;

    // The size of the buffer in the time domain.
    size_t time_domain_width_;
    size_t time_domain_height_;

    // The size of the buffer in the frequency domain.
    size_t frequency_domain_width_;
    size_t frequency_domain_height_;

    // 'true' if the object contains an initialized 'plan_', 'false' otherwise. Could
    // be used to implement a move constructor (which is currently not needed in the
    // reset of the program).
    bool plan_valid_;
};

// Return a value equal or bigger to 'size' that is well suited for a fast FFT.
//
// Not reentrant, as the function uses an internal cache object.
size_t fast_fft_size(
    size_t size);

}

#endif
