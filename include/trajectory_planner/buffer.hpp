#ifndef TRAJECTORY_PLANNER_BUFFER_HPP
#define TRAJECTORY_PLANNER_BUFFER_HPP

#include <fftw3.h>

#include <cassert>
#include <cstring>
#include <memory>

namespace trajectory_planner
{

// A 2D buffer template for real or complex types. Memory is allocated using the
// FFTW allocation function, so the data can be used when calling FFTW functions.
template <typename T>
class Buffer
{
public:

    // Create a buffer of the requested dimensions. The buffer's memory will be set
    // to all zeros after it is allocated.
    Buffer(size_t width, size_t height)
        : width_(width)
        , height_(height)
        , data_(fftw_malloc(byte_size()), &fftw_free)
    {
        set_zero();
    }

    // Create a new buffer and copy over the data from the given buffer.
    Buffer(const Buffer& buffer)
        : width_(buffer.width())
        , height_(buffer.height())
        , data_(fftw_malloc(byte_size()), &fftw_free)
    {
        memcpy(raw(), buffer.raw(), byte_size());
    }

    // Return the width of the buffer.
    size_t width() const
    {
        return width_;
    }

    // Return the height of the buffer.
    size_t height() const
    {
        return height_;
    }

    // Set the whole buffer memory back to zero.
    void set_zero()
    {
        // In the internal representation of '0.0' all bits are set to zero,
        // so using memset to set all values back to zero like this works.
        memset(data_.get(), 0, byte_size());
    }

    // Returns a 'const' pointer to the internal memory.
    const T* raw() const
    {
        return reinterpret_cast<T*>(data_.get());
    }

    // Return a pointer to the internal memory.
    T* raw()
    {
        return reinterpret_cast<T*>(data_.get());
    }

    // Return a 'const' reference to a single element.
    const T& at(size_t x, size_t y) const
    {
        assert((0 <= x) && (x < width_));
        assert((0 <= y) && (y < height_));

        return raw()[y * width_ + x];
    }

    // Return a reference to a single element.
    T& at(size_t x, size_t y)
    {
        assert((0 <= x) && (x < width_));
        assert((0 <= y) && (y < height_));

        return raw()[y * width_ + x];
    }

private:

    // The width of the buffer pointed to by 'data_'.
    size_t width_;

    // The height of the buffer pointed to by 'data_'.
    size_t height_;

    // The type used to store the raw data pointer, using the deleter function
    // that corresponds to the FFTW allocation function.
    using pointer_type = std::unique_ptr<void, decltype(&fftw_free)>;

    // A pointer to the actual buffer memory.
    pointer_type data_;

    // Return the size of the data pointed to by 'data_'.
    size_t byte_size() const
    {
        return width_ * height_ * sizeof(T);
    }
};

// Create a buffer of the requested size and copy over the content of the
// given buffer. All other elements are left to zero.
Buffer<double> zero_pad(
    const Buffer<double>& buffer,
    size_t width,
    size_t height);

// Multiply each element in 'a' with the corresponding element in 'b'. Both
// buffers need to be of the same size.
void multiply_buffers(
    Buffer<fftw_complex>& a,
    const Buffer<fftw_complex>& b);

}

#endif
