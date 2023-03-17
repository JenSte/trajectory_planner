#include "trajectory_planner/trajectory_planner.hpp"

#include "trajectory_planner/convolution.hpp"

#include <boost/asio/post.hpp>
#include <boost/asio/thread_pool.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <iomanip>
#include <mutex>
#include <thread>
#include <unordered_map>

namespace trajectory_planner
{

Buffer<double> TrajectoryPlanner::create_cost_map(
    const LogCallback& log_callback,
    const bool multi_threaded,
    const double resolution,
    const double inflation_radius,
    const Buffer<double>& occupancy_map)
{
    // The radius of the biggest circle we draw, in pixels.
    const unsigned int pixel_radius =
        std::max(2u, static_cast<unsigned int>(inflation_radius / resolution));

    // The height and width of the canvas we draw the circles on.
    const unsigned int mask_pixel_size = 2 * pixel_radius;

    const size_t padded_width = fast_fft_size(occupancy_map.width() + mask_pixel_size - 1);
    const size_t padded_height = fast_fft_size(occupancy_map.height() + mask_pixel_size - 1);

    Buffer<double> dummy_buffer(padded_width, padded_height);
    FFTPlan plan_forward = FFTPlan::plan_forward(dummy_buffer);
    FFTPlan plan_backward = FFTPlan::plan_backward(dummy_buffer);

    Buffer<double> occupancy_map_padded =
        zero_pad(occupancy_map, padded_width, padded_height);

    Buffer<fftw_complex> occupancy_map_spectrum(
        plan_forward.frequency_domain_width(),
        plan_forward.frequency_domain_height());
    plan_forward.execute(occupancy_map_padded, occupancy_map_spectrum);

    // Stores the cost calculated in the threads.
    std::vector<Buffer<double>> cost_images;
    std::mutex cost_images_mutex;

    // Calculate the combined costs for a given number of radius values. Each thread
    // processes a number of radii (instead of posting a function for each radius
    // to the thread pool), so that the combining of the result of multiple radii can
    // also done in parallel in the threads.
    auto process_work = [&](const std::vector<unsigned int>& work) {
        // The result (the maximum of all layers) for all radii that are passed in
        // is placed here.
        Buffer<double> result(occupancy_map.width(), occupancy_map.height());

        // Buffer to draw a circle on, that is then convoluted with the occupancy
        // map. (Also used for the result of the inverse FFT.)
        Buffer<double> mask_image(padded_width, padded_height);

        // Holds the result of the FFT of the circle.
        Buffer<fftw_complex> mask_image_spectrum(
            plan_forward.frequency_domain_width(),
            plan_forward.frequency_domain_height());

        // For each radius value, a circle is drawn on 'mask_image' and then convoluted
        // with the occupancy map. From the result, each cell is checked if it is
        // within 'radius' distance of something on the occupancy map, and the result
        // is combined in 'result'.
        for (const unsigned int radius: work) {
            mask_image.set_zero();
            draw_circle(mask_image, mask_pixel_size, radius);

            // Convolute the circle with the occupancy map.
            plan_forward.execute(mask_image, mask_image_spectrum);
            multiply_buffers(mask_image_spectrum, occupancy_map_spectrum);
            plan_backward.execute(mask_image_spectrum, mask_image);

            // The cost falls off from 1.0 next to an obstacle to nearly 0.0.
            const double cost = exp((-5.0 * radius) / pixel_radius);

            for (size_t y = 0; y < result.height(); y++) {
                for (size_t x = 0; x < result.width(); x++) {
                    // Get the value that is offset by the center coordinate of the circle.
                    const double pixel_value = mask_image.at(x + pixel_radius, y + pixel_radius);

                    if (pixel_value > 0.5) {
                        result.at(x, y) = std::max(result.at(x, y), cost);
                    }
                }
            }
        }

        std::lock_guard<std::mutex> lock(cost_images_mutex);
        cost_images.emplace_back(std::move(result));
    };

    log_callback("Creating inflation layers...");

    // The number of threads that are used (if threading is requested).
    const unsigned int threads = std::thread::hardware_concurrency();

    std::unique_ptr<boost::asio::thread_pool> pool;
    if (multi_threaded) {
        pool = std::make_unique<boost::asio::thread_pool>(threads);
    }

    // Distribute the work on the threads.
    std::vector<std::vector<unsigned int>> work(threads);
    for (unsigned int r = 0; r < pixel_radius; r++) {
        work.at(r % threads).push_back(r);
    }

    for (const std::vector<unsigned int>& w: work) {
        if (multi_threaded) {
            boost::asio::post(*pool, [&process_work, &w]{ process_work(w); });
        } else {
            process_work(w);
        }
    }

    if (multi_threaded) {
        pool->join();
    }

    log_callback("Combining inflation layers...");

    // Combine the buffers containing the costs.
    Buffer<double> cost_map(occupancy_map.width(), occupancy_map.height());

    // Combine all the resulting cost images into one.
    for (const Buffer<double>& ci: cost_images) {
        for (size_t y = 0; y < cost_map.height(); y ++) {
            for (size_t x = 0; x < cost_map.width(); x ++) {
                cost_map.at(x, y) = std::max(cost_map.at(x, y), ci.at(x, y));
            }
        }
    }

    return cost_map;
}

Costs TrajectoryPlanner::create_costs(
    const LogCallback& log_callback,
    const bool multi_threaded,
    const unsigned int angle_granularity,
    const double resolution,
    const Polygon& footprint,
    const Buffer<double>& occupancy_map,
    const Buffer<double>& cost_map)
{
    // The size of the canvas we'll draw the footprint on.
    const unsigned int footprint_pixel_size =
        calculate_footprint_size(footprint, resolution);

    // Buffers are extended to this size, so that the circular convolution
    // does not distort the result.
    const size_t padded_width = fast_fft_size(occupancy_map.width() + footprint_pixel_size - 1);
    const size_t padded_height = fast_fft_size(occupancy_map.height() + footprint_pixel_size - 1);

    //
    // Process input map.
    //

    // Because the FFT planning function (using FFTW_MEASURE) may destroy
    // the input buffer, we use a temporary buffer for this.
    Buffer<double> dummy_buffer(padded_width, padded_height);
    FFTPlan plan_forward = FFTPlan::plan_forward(dummy_buffer);
    FFTPlan plan_backward = FFTPlan::plan_backward(dummy_buffer);

    Buffer<double> occupancy_map_padded =
        zero_pad(occupancy_map, padded_width, padded_height);
    Buffer<fftw_complex> occupancy_map_spectrum(
        plan_forward.frequency_domain_width(),
        plan_forward.frequency_domain_height());

    Buffer<double> cost_map_padded =
        zero_pad(cost_map, padded_width, padded_height);
    Buffer<fftw_complex> cost_map_spectrum(
        plan_forward.frequency_domain_width(),
        plan_forward.frequency_domain_height());

    plan_forward.execute(occupancy_map_padded, occupancy_map_spectrum);
    plan_forward.execute(cost_map_padded, cost_map_spectrum);

    // The mutex used to protect access to the two data structures below.
    std::mutex costs_mutex;

    // The "calculated" 3D costs are accumulated in this object. The map key is the 3D
    // orientation, the values in the map are (x, y, cell-cost) tuples for this orientation.
    std::unordered_map<unsigned int, std::vector<std::tuple<size_t, size_t, double>>> costs_3d;

    // The occupancy data used by the 5D planner. The map key is the 5D angle index,
    // the values in the map are (x, y) tuples where it is possible to place the robot.
    std::unordered_map<unsigned int, std::vector<std::tuple<size_t, size_t>>> costs_5d;

    // Process the footprint: For a given orientation, the robot's footprint is drawn
    // and then FFT transformed. Then, both spectrums of the maps (occupancy map and
    // cost map) are multiplied with the footprint's spectrum. After transforming the
    // multiplied buffers back, the resulting buffers contain the collisions and the
    // costs for each cell.
    auto process_footprint = [&](bool three_dimensional, double angle, unsigned int index) {
        // Draw the footprint. Because the convolution mirrors one of it's inputs,
        // we draw the robot's footprint rotated by 180 degrees to compensate.
        Buffer<double> footprint_image(padded_width, padded_height);
        unsigned int covered_pixels = draw_footprint(
            footprint_image,
            footprint,
            resolution,
            footprint_pixel_size,
            M_PI + angle);

        // Calculate the spectrum of the footprint.
        Buffer<fftw_complex> footprint_image_spectrum_1(
            plan_forward.frequency_domain_width(),
            plan_forward.frequency_domain_height());

        plan_forward.execute(footprint_image, footprint_image_spectrum_1);

        // In the three dimensional case we need the footprint's spectrum another time,
        // so we make a copy instead of transforming it again.
        Buffer<fftw_complex> footprint_image_spectrum_2 =
            three_dimensional ? footprint_image_spectrum_1.copy() : Buffer<fftw_complex>(1, 1);

        // Multiply the corresponding spectrum buffers.
        multiply_buffers(footprint_image_spectrum_1, occupancy_map_spectrum);
        if (three_dimensional) {
            multiply_buffers(footprint_image_spectrum_2, cost_map_spectrum);
        }

        // Calculate the inverse FFT, thereby completing the convolution.
        Buffer<double> result_buffer_1(padded_width, padded_height);
        plan_backward.execute(footprint_image_spectrum_1, result_buffer_1);

        // Again, this is only needed for the three dimensional case.
        Buffer<double> result_buffer_2 =
            three_dimensional ? Buffer<double>(padded_width, padded_height) : Buffer<double>(1, 1);
        if (three_dimensional) {
            plan_backward.execute(footprint_image_spectrum_2, result_buffer_2);
        }

        // Get the costs for the cells where no collision happens (3D case) or for
        // the cells where the robot can be placed (5D case), and store the costs
        // for this orientation for further processing in the main thread.
        if (three_dimensional) {
            std::vector<std::tuple<size_t, size_t, double>> costs =
                extract_costs_3d(
                    occupancy_map.width(),
                    occupancy_map.height(),
                    footprint_pixel_size / 2,
                    covered_pixels,
                    result_buffer_1,
                    result_buffer_2);

            std::lock_guard<std::mutex> lock(costs_mutex);
            costs_3d.emplace(index, std::move(costs));
        } else {
            std::vector<std::tuple<size_t, size_t>> costs =
                extract_costs_5d(
                    occupancy_map.width(),
                    occupancy_map.height(),
                    footprint_pixel_size / 2,
                    result_buffer_1);

            std::lock_guard<std::mutex> lock(costs_mutex);
            costs_5d.emplace(index, std::move(costs));
        }
    };

    log_callback("Creating cost layers...");

    std::unique_ptr<boost::asio::thread_pool> pool;
    if (multi_threaded) {
        pool = std::make_unique<boost::asio::thread_pool>(
            std::thread::hardware_concurrency());
    }

    // Add work to the thread pool to create the 3D costs.
    for (unsigned int q = 0; q < 4; q++) {
        // The angles for going...
        //  * one field forward,
        //  * two fields forward, one sideward,
        //  * one field forward, one sideward,
        //  * one field forward, two sindeward
        // in the corresponding quadrant.
        const double angle_a = q * M_PI / 2.0;
        const double angle_b = angle_a + atan2(1, 2);
        const double angle_c = angle_a + M_PI / 4.0;
        const double angle_d = angle_a + atan2(2, 1);

        if (multi_threaded) {
            boost::asio::post(*pool, [process_footprint, angle_a, q]{ process_footprint(true, angle_a, q * 4); });
            boost::asio::post(*pool, [process_footprint, angle_b, q]{ process_footprint(true, angle_b, q * 4 + 1); });
            boost::asio::post(*pool, [process_footprint, angle_c, q]{ process_footprint(true, angle_c, q * 4 + 2); });
            boost::asio::post(*pool, [process_footprint, angle_d, q]{ process_footprint(true, angle_d, q * 4 + 3); });
        } else {
            process_footprint(true, angle_a, q * 4);
            process_footprint(true, angle_b, q * 4 + 1);
            process_footprint(true, angle_c, q * 4 + 2);
            process_footprint(true, angle_d, q * 4 + 3);
        }
    }

    // Add work to the thread pool to create the 5D costs.
    for (unsigned int ai = 0; ai < angle_granularity; ai++) {
        // The angle that corresponds to the angle index.
        const double angle = 2 * M_PI * ai / angle_granularity;

        if (multi_threaded) {
            boost::asio::post(
                *pool, [process_footprint, angle, ai]{
                    process_footprint(false, angle, ai);
                });
        } else {
            process_footprint(false, angle, ai);
        }
    }

    if (multi_threaded) {
        pool->join();
    }

    // Use a value that is proportional to the size of the footprint to select
    // poses close to the goal.
    const unsigned int goal_turn_penalty_distance = footprint_pixel_size / 2;
    std::ostringstream ss;
    ss
        << std::setprecision(2) << std::fixed
        << "Goal turn penalty distance: "
        << (goal_turn_penalty_distance * resolution) << " m";
    log_callback(ss.str());

    // Take all the costs and put them in one data structure.
    Costs costs(
        padded_width,
        padded_height,
        angle_granularity,
        goal_turn_penalty_distance);

    log_callback("Combining 3D cost layers...");
    for (const auto& it: costs_3d) {
        for (const auto& v: it.second) {
            costs.set_3d_cost(
                std::get<0>(v),
                std::get<1>(v),
                it.first,
                std::get<2>(v));
        }
    }

    log_callback("Combining 5D cost layers...");
    for (const auto& it: costs_5d) {
        for (const auto& v: it.second) {
            costs.set_5d_cost(
                std::get<0>(v),
                std::get<1>(v),
                it.first);
        }
    }

    return costs;
}

void TrajectoryPlanner::draw_circle(
    Buffer<double>& buffer,
    int canvas_size,
    int radius)
{
    cv::Point center(canvas_size / 2, canvas_size / 2);
    cv::Mat canvas = cv::Mat::zeros(canvas_size, canvas_size, CV_8UC1);
    cv::circle(canvas, center, radius, cv::Scalar(255), cv::FILLED);

    for (int y = 0; y < canvas_size; y ++) {
        for (int x = 0; x < canvas_size; x ++) {
            if (canvas.at<unsigned char>(y, x) != 0) {
                buffer.at(y, x) = 1.0;
            }
        }
    }
}

unsigned int TrajectoryPlanner::draw_footprint(
    Buffer<double>& buffer,
    const Polygon& footprint_polygon,
    double resolution,
    int footprint_size,
    double theta)
{
    // Convert footprint to OpenCV polygon type.
    std::vector<cv::Point> polygon;
    for (const auto& p: footprint_polygon) {
        // Convert meter to pixel coordinates.
        double x = std::get<0>(p) / resolution;
        double y = std::get<1>(p) / resolution;

        // Rotate the point according to the argument.
        double rx = x * cos(theta) - y * sin(theta);
        double ry = x * sin(theta) + y * cos(theta);

        // Shift the points so that the origin of the drawn polygon
        // is in the center of the canvas.
        rx += footprint_size / 2;
        ry += footprint_size / 2;

        // OpenCV treats the coordinates in the opposite order, note that
        // the X and Y arguments are swapped when creating the cv::Point.
        double column = rx;
        double row = ry;
        polygon.push_back(cv::Point(row, column));
    }

    // Draw the filled polygon.
    const cv::Point* ppt[1] = { polygon.data() };
    int npt[] = { static_cast<int>(polygon.size()) };

    cv::Mat canvas = cv::Mat::zeros(footprint_size, footprint_size, CV_8UC1);
    cv::fillPoly(canvas, ppt, npt, 1, cv::Scalar(255));

    unsigned int black = 0;
    for (int y = 0; y < footprint_size; y ++) {
        for (int x = 0; x < footprint_size; x ++) {
            if (canvas.at<unsigned char>(y, x) != 0) {
                buffer.at(y, x) = 1.0;
                black++;
            }
        }
    }

    return std::max(1u, black);
}

std::vector<std::tuple<size_t, size_t, double>> TrajectoryPlanner::extract_costs_3d(
    size_t map_width,
    size_t map_height,
    size_t offset,
    unsigned int footprint_covered_pixels,
    const Buffer<double>& convoluted_occupancy_map,
    const Buffer<double>& convoluted_cost_map)
{
    if (convoluted_occupancy_map.width() < map_width + 2 * offset - 1) {
        throw std::runtime_error("Convoluted occupancy map has wrong width.");
    }

    if (convoluted_occupancy_map.height() < map_height + 2 * offset - 1) {
        throw std::runtime_error("Convoluted occupancy map has wrong height.");
    }

    if (convoluted_occupancy_map.width() != convoluted_cost_map.width()) {
        throw std::runtime_error("Cost and occupancy map do not have the same width.");
    }

    if (convoluted_occupancy_map.height() != convoluted_cost_map.height()) {
        throw std::runtime_error("Cost and occupancy map do not have the same height.");
    }

    // FFTW computes an unnormalized transform, i.e. the values of IFFT(FFT(...)) are
    // multiplied by the size of the input.
    const double fft_factor = (map_width + 2 * offset - 1) * (map_height + 2 * offset - 1);

    std::vector<std::tuple<size_t, size_t, double>> result;

    for (size_t y = 0; y < map_height; y++) {
        for (size_t x = 0; x < map_width; x++) {
            // Where the "functions" of the footprint and the occupancy map overlap,
            // the result of the convolution has a non-zero value, while a value of
            // zero means that the footprint can be placed on this position without
            // colliding with the map.
            //
            // There is no need to remove the FFT bias from the values of the convoluted
            // occupancy map, as we only care about zero/non-zero values here.
            if (convoluted_occupancy_map.at(x + offset, y + offset) < 0.5) {
                // A valid position, store the cost of this position in the result.

                double cost = convoluted_cost_map.at(x + offset, y + offset);

                // Remove the FFT bias.
                cost /= fft_factor;

                // Also normalize for the fact that the footprint image might cover
                // a different number of pixels for different orientations.
                cost /= footprint_covered_pixels;

                result.emplace_back(std::make_tuple(x, y, cost));
            }
        }
    }

    return result;
}

std::vector<std::tuple<size_t, size_t>> TrajectoryPlanner::extract_costs_5d(
    size_t map_width,
    size_t map_height,
    size_t offset,
    const Buffer<double>& convoluted_occupancy_map)
{
    if (convoluted_occupancy_map.width() < map_width + 2 * offset - 1) {
        throw std::runtime_error("Convoluted occupancy map has wrong width.");
    }

    if (convoluted_occupancy_map.height() < map_height + 2 * offset - 1) {
        throw std::runtime_error("Convoluted occupancy map has wrong height.");
    }

    std::vector<std::tuple<size_t, size_t>> result;

    for (size_t y = 0; y < map_height; y++) {
        for (size_t x = 0; x < map_width; x++) {
            if (convoluted_occupancy_map.at(x + offset, y + offset) < 0.5) {
                result.emplace_back(std::make_tuple(x, y));
            }
        }
    }

    return result;
}

}
