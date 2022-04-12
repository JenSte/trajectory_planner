#include "trajectory_planner/trajectory_planner.hpp"

#include "trajectory_planner/planning_3d.hpp"
#include "trajectory_planner/convolution.hpp"

#include <boost/asio/post.hpp>
#include <boost/asio/thread_pool.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <cmath>
#include <iomanip>
#include <thread>
#include <unordered_map>

namespace trajectory_planner
{

TrajectoryPlanner::TrajectoryPlanner(
    LogCallback log_callback,
    std::string hash,
    Buffer<double> occupancy_map,
    Buffer<double> cost_map,
    Costs costs)
    : log_callback_(std::move(log_callback))
    , hash_(std::move(hash))
    , original_occupancy_map_(std::move(occupancy_map))
    , cost_map_(std::move(cost_map))
    , costs_(std::move(costs))
{
}

std::unique_ptr<TrajectoryPlanner> TrajectoryPlanner::create_planner(
    LogCallback log_callback,
    const bool multi_threaded,
    std::string hash,
    unsigned int angle_granularity,
    double resolution,
    double inflation_radius,
    const Polygon& footprint,
    Buffer<double> occupancy_map)
{
    Buffer<double> cost_map = create_cost_map(
        log_callback,
        multi_threaded,
        resolution,
        inflation_radius,
        occupancy_map);
    Costs costs = create_costs(
        log_callback,
        multi_threaded,
        angle_granularity,
        resolution,
        footprint,
        occupancy_map,
        cost_map);

    std::unique_ptr<TrajectoryPlanner> planner;
    planner.reset(new TrajectoryPlanner(
        std::move(log_callback),
        std::move(hash),
        std::move(occupancy_map),
        std::move(cost_map),
        std::move(costs)));
    return planner;
}

std::unique_ptr<TrajectoryPlanner> TrajectoryPlanner::load_planner(
    LogCallback log_callback,
    std::string hash,
    Buffer<double> occupancy_map,
    std::ifstream& ifs)
{
    // Create a dummy cost map when loading the planner from a file,
    // as the cost map is only used for debugging purposes.
    Buffer<double> cost_map(occupancy_map);
    draw_text(cost_map, "Cost map not available for planners loaded from cache.");

    Costs costs(0);
    try {
        // Load the costs object.
        boost::archive::binary_iarchive ia(ifs);
        ia >> costs;
    } catch (const std::exception& e) {
        log_callback(std::string("Error loading planner: ") + e.what());
        return nullptr;
    }

    std::unique_ptr<TrajectoryPlanner> planner;
    planner.reset(new TrajectoryPlanner(
        std::move(log_callback),
        std::move(hash),
        std::move(occupancy_map),
        std::move(cost_map),
        std::move(costs)));
    return planner;
}

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

    const size_t padded_width = occupancy_map.width() + mask_pixel_size - 1;
    const size_t padded_height = occupancy_map.height() + mask_pixel_size - 1;

    Buffer<double> dummy_buffer(padded_width, padded_height);
    FFTPlan plan_forward = FFTPlan::plan_forward(dummy_buffer);
    FFTPlan plan_backward = FFTPlan::plan_backward(dummy_buffer);

    Buffer<double> occupancy_map_padded =
        zero_pad(occupancy_map, padded_width, padded_height);

    Buffer<fftw_complex> occupancy_map_spectrum(
        plan_forward.frequency_domain_width(),
        plan_forward.frequency_domain_height());
    plan_forward.execute(occupancy_map_padded, occupancy_map_spectrum);

    // Stores the cost calculated for a given radius.
    std::unordered_map<unsigned int, Buffer<double>> cost_images;
    std::mutex cost_images_mutex;

    // Create a buffer that contains a non-zero value at positions that are within
    // a range of 'radius' pixels around obstacles in the occupancy map. This is
    // done by doing a convolution between the occupancy map and a circle of the
    // given size. The resulting buffer is stored in 'cost_images'.
    auto calculate_cost = [&](unsigned int radius) {
        Buffer<double> mask_image(padded_width, padded_height);
        draw_circle(mask_image, mask_pixel_size, radius);

        Buffer<fftw_complex> mask_image_spectrum(
            plan_forward.frequency_domain_width(),
            plan_forward.frequency_domain_height());
        plan_forward.execute(mask_image, mask_image_spectrum);

        multiply_buffers(mask_image_spectrum, occupancy_map_spectrum);

        Buffer<double> result(padded_width, padded_height);
        plan_backward.execute(mask_image_spectrum, result);

        std::lock_guard<std::mutex> lock(cost_images_mutex);
        cost_images.emplace(radius, std::move(result));
    };

    log_callback("Creating inflation layers...");

    std::unique_ptr<boost::asio::thread_pool> pool;
    if (multi_threaded) {
        pool = std::make_unique<boost::asio::thread_pool>(
            std::thread::hardware_concurrency());
    }

    for (unsigned int r = 0; r < pixel_radius; r++) {
        if (multi_threaded) {
            boost::asio::post(
                *pool, [calculate_cost, r]{ calculate_cost(r); });
        } else {
            calculate_cost(r);
        }
    }

    if (multi_threaded) {
        pool->join();
    }

    log_callback("Combining inflation layers...");

    // Combine the buffers containing the costs.
    Buffer<double> cost_map(occupancy_map.width(), occupancy_map.height());

    // Combine all the resulting cost images into one.
    for (const auto& it: cost_images) {
        const unsigned int radius = std::get<0>(it);
        const Buffer<double>& buffer = std::get<1>(it);

        // The cost falls off from 1.0 next to an obstalce to nearly 0.0.
        const double cost = exp((-5.0 * radius) / pixel_radius);

        for (size_t y = 0; y < cost_map.height(); y ++) {
            for (size_t x = 0; x < cost_map.width(); x ++) {
                // Get the value that is offset by the center coordinate of the circle.
                const double pixel_value = buffer.at(x + pixel_radius, y + pixel_radius);

                if (pixel_value > 0.5) {
                    cost_map.at(x, y) = std::max(cost_map.at(x, y), cost);
                }
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
    const size_t padded_width = occupancy_map.width() + footprint_pixel_size - 1;
    const size_t padded_height = occupancy_map.height() + footprint_pixel_size - 1;

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

    // The "calculated" costs are accumulated in this object. The map key is the angle
    // index, the values in the map are (x, y, cell-cost) tuples for this orientation.
    std::unordered_map<unsigned int, std::vector<std::tuple<size_t, size_t, double>>> angle_costs;
    std::mutex angle_costs_mutex;

    // Process the footprint: For a given orientation, the robot's footprint is drawn
    // and then FFT transformed. Then, both spectrums of the maps (occupancy map and
    // cost map) are multiplied with the footprint's spectrum. After transforming the
    // multiplied buffers back, the resulting buffers contain the collisions and the
    // costs for each cell.
    auto process_footprint = [&](unsigned int angle_index) {
        // Draw the footprint. Because the convolution mirrors one of it's inputs,
        // we draw the robot's footprint rotated by 180 degrees to compensate.
        Buffer<double> footprint_image(padded_width, padded_height);
        unsigned int covered_pixels = draw_footprint(
            footprint_image,
            footprint,
            resolution,
            footprint_pixel_size,
            M_PI + angle_index * 2 * M_PI / angle_granularity);

        // Calculate the spectrum of the footprint.
        Buffer<fftw_complex> footprint_image_spectrum_1(
            plan_forward.frequency_domain_width(),
            plan_forward.frequency_domain_height());

        plan_forward.execute(footprint_image, footprint_image_spectrum_1);

        // We need the footprint's spectrum two times, so we make a copy
        // instead of transforming it again.
        Buffer<fftw_complex> footprint_image_spectrum_2(footprint_image_spectrum_1);

        // Multiply the corresponding spectrum buffers.
        multiply_buffers(footprint_image_spectrum_1, occupancy_map_spectrum);
        multiply_buffers(footprint_image_spectrum_2, cost_map_spectrum);

        // Calculate the inverse FFT, thereby completing the convolution.
        Buffer<double> result_buffer_1(padded_width, padded_height);
        Buffer<double> result_buffer_2(padded_width, padded_height);
        plan_backward.execute(footprint_image_spectrum_1, result_buffer_1);
        plan_backward.execute(footprint_image_spectrum_2, result_buffer_2);

        // Get the costs for the cells where no collision happens.
        std::vector<std::tuple<size_t, size_t, double>> costs =
            extract_costs(
                occupancy_map.width(),
                occupancy_map.height(),
                footprint_pixel_size / 2,
                covered_pixels,
                result_buffer_1,
                result_buffer_2);

        // Store the costs for this orientation for further processing in the main thread.
        std::lock_guard<std::mutex> lock(angle_costs_mutex);
        angle_costs.emplace(angle_index, std::move(costs));
    };

    log_callback("Creating cost layers...");

    std::unique_ptr<boost::asio::thread_pool> pool;
    if (multi_threaded) {
        pool = std::make_unique<boost::asio::thread_pool>(
            std::thread::hardware_concurrency());
    }

    for (unsigned int ai = 0; ai < angle_granularity; ai++) {
        if (multi_threaded) {
            boost::asio::post(
                *pool, [process_footprint, ai]{ process_footprint(ai); });
        } else {
            process_footprint(ai);
        }
    }

    if (multi_threaded) {
        pool->join();
    }

    log_callback("Combining cost layers...");

    // Take all the costs and put them in one data structure.
    Costs costs(angle_granularity);
    for (const auto& it: angle_costs) {
        for (const auto& v: it.second) {
            costs.set_cost(
                std::get<0>(v),
                std::get<1>(v),
                it.first,
                std::get<2>(v));
        }
    }

    return costs;
}

void TrajectoryPlanner::draw_text(
    Buffer<double>& buffer,
    const std::string& text)
{
    const int face = cv::FONT_HERSHEY_PLAIN;
    const double scale = 1.0;
    const int thickness = 1;

    int baseline = 0;
    cv::Size size = cv::getTextSize(text, face, scale, thickness, &baseline);
    size += cv::Size(0, baseline);
    const cv::Point org(0, size.height + 2 - baseline);

    const cv::Scalar color(255);
    cv::Mat canvas(cv::Mat::zeros(size, CV_8UC1));
    cv::putText(canvas, text, org, face, scale, color, thickness);
    cv::line(canvas, org + cv::Point(0, thickness), org + cv::Point(size.width, thickness), color);

    for (size_t column = 0; column < static_cast<size_t>(canvas.cols); column++) {
        for (size_t row = 0; row < static_cast<size_t>(canvas.rows); row++) {
            const bool pixel = canvas.at<unsigned char>(row, column) != 0;

            {
                // Copy text horizontally.
                const size_t x = size.height + column;
                const size_t y = canvas.rows - 1 - row;

                if ((x < buffer.width()) && (y < buffer.height())) {
                    buffer.at(x, y) = pixel ? 1.0 : 0.0;
                }
            }

            {
                // Copy text vertically.
                const size_t x = row;
                const size_t y = size.height + column;

                if ((x < buffer.width()) && (y < buffer.height())) {
                    buffer.at(x, y) = pixel ? 1.0 : 0.0;
                }
            }
        }
    }
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

const Buffer<double>& TrajectoryPlanner::original_occupancy_map() const
{
    return original_occupancy_map_;
}

const Buffer<double>& TrajectoryPlanner::cost_map() const
{
    return cost_map_;
}

const std::string& TrajectoryPlanner::hash() const
{
    return hash_;
}

unsigned int TrajectoryPlanner::angle_granularity() const
{
    return costs_.angle_granularity();
}

std::vector<std::tuple<size_t, size_t, double>> TrajectoryPlanner::extract_costs(
    size_t map_width,
    size_t map_height,
    size_t offset,
    unsigned int footprint_covered_pixels,
    const Buffer<double>& convoluted_occupancy_map,
    const Buffer<double>& convoluted_cost_map)
{
    if (convoluted_occupancy_map.width() != map_width + 2 * offset - 1) {
        throw std::runtime_error("Convoluted occupancy map has wrong width.");
    }

    if (convoluted_occupancy_map.height() != map_height + 2 * offset - 1) {
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

void TrajectoryPlanner::dump_orientation_maps(
    const std::string& prefix) const
{
    size_t width = original_occupancy_map_.width();
    size_t height = original_occupancy_map_.height();

    auto output_orientation = [&](unsigned int angle_index) {
        cv::Mat canvas = cv::Mat::zeros(height, width, CV_8UC1);

        for (size_t y = 0; y < height; y++) {
            for (size_t x = 0; x < width; x++) {
                size_t row = height - 1 - y;
                size_t column = x;

                double cost = costs_.get_cost(x, y, angle_index);
                if (cost < (0.5 * Costs::invalid_cost)) {
                    // Non-occupied pose.
                    canvas.at<unsigned char>(row, column) = 0;
                } else {
                    // Valid location, create a tone that corresponds to the cost.
                    unsigned int grey = 255 * cost;
                    grey = std::max(0u, std::min(255u, grey));
                    canvas.at<unsigned char>(row, column) = grey;
                }
            }
        }

        // Convert the grayscale values to colors.
        cv::Mat color_image;
        cv::applyColorMap(canvas, color_image, cv::COLORMAP_JET);

        // Do another pass over the colored image and tidy it up a bit.
        for (size_t y = 0; y < height; y++) {
            for (size_t x = 0; x < width; x++) {
                size_t row = height - 1 - y;
                size_t column = x;

                // Color non-occupied cells white.
                double cost = costs_.get_cost(x, y, angle_index);
                if (cost < (0.5 * Costs::invalid_cost)) {
                    color_image.at<cv::Vec3b>(row, column) = cv::Vec3b(255, 255, 255);
                }

                // Draw the obstacles in black.
                if (original_occupancy_map_.at(x, y) > 0.5) {
                    color_image.at<cv::Vec3b>(row, column) = cv::Vec3b(0, 0, 0);
                }
            }
        }

        std::stringstream ss;
        ss << prefix << "map_" << std::setw(3) << std::setfill('0') << angle_index << ".png";
        cv::imwrite(ss.str(), color_image);
    };

    boost::asio::thread_pool pool(std::thread::hardware_concurrency());
    for (unsigned int ai = 0; ai < costs_.angle_granularity(); ai++) {
        boost::asio::post(
            pool,
            [output_orientation, ai]{ output_orientation(ai); }
        );
    }
    pool.join();
}

TrajectoryPlanner::Result TrajectoryPlanner::plan(
    const Pose& start,
    const Pose& goal) const
{
    if (costs_.get_cost(start) < (0.5 * Costs::invalid_cost)) {
        throw std::invalid_argument("Start pose is not valid.");
    }

    if (costs_.get_cost(goal) < (0.5 * Costs::invalid_cost)) {
        throw std::invalid_argument("Goal pose is not valid.");
    }

    three::SearchResult3D search_result_3d = three::plan(costs_, start, goal);

    return Result{search_result_3d.path, search_result_3d};
}

void TrajectoryPlanner::store(
    std::ofstream& ofs) const
{
    boost::archive::binary_oarchive oa(ofs);
    oa << costs_;
}

unsigned int TrajectoryPlanner::calculate_footprint_size(
    const Polygon& footprint_polygon,
    double resolution)
{
    int size = 2;

    for (const auto& p: footprint_polygon) {
        // The distance of the point from the center.
        double dist_meter =
            sqrt(pow(std::get<0>(p), 2.0) + pow(std::get<1>(p), 2.0));

        // Multiply the distance with a value that is a little bit bigger
        // than sqrt(2), so that even if the footprint is rotated by
        // 45 degrees it will still fit.
        dist_meter *= 1.5;

        // The distance in pixels.
        int dist_pixels = dist_meter / resolution;

        size = std::max(size, 2 * dist_pixels);
    }

    return size;
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

}
