#include "trajectory_planner/buffer.hpp"

namespace trajectory_planner
{

Buffer<double> zero_pad(
    const Buffer<double>& buffer,
    size_t width,
    size_t height)
{
    if (buffer.width() > width) {
        throw std::runtime_error("Unable to pad to a smaller width.");
    }

    if (buffer.height() > height) {
        throw std::runtime_error("Unable to pad to a smaller height.");
    }

    Buffer<double> result(width, height);

    for (size_t y = 0; y < buffer.height(); y++) {
        for (size_t x = 0; x < buffer.width(); x++) {
            result.at(x, y) = buffer.at(x, y);
        }
    }

    return result;
}

void multiply_buffers(
    Buffer<fftw_complex>& a,
    const Buffer<fftw_complex>& b)
{
    if (a.width() != b.width()) {
        throw std::range_error("Buffer widths are not equal.");
    }

    if (a.height() != b.height()) {
        throw std::range_error("Buffer heights are not equal.");
    }

    for (size_t y = 0; y < a.height(); y++) {
        for (size_t x = 0; x < a.width(); x++) {
            // Writing the complex multiplication here "by hand" is about
            // three times faster than using 'std::complex<>'.
            const double ra = a.at(x, y)[0];
            const double ia = a.at(x, y)[1];
            const double rb = b.at(x, y)[0];
            const double ib = b.at(x, y)[1];

            const double r = ra * rb - ia * ib;
            const double i = ra * ib + ia * rb;

            a.at(x, y)[0] = r;
            a.at(x, y)[1] = i;
        }
    }
}

}
