#include "imu_core.h"

#include <atomic>
#include <csignal>
#include <iostream>
#include <thread>

namespace
{

using namespace proj3;

std::atomic<bool> g_running{true};

struct Options
{
    std::string i2c_bus = kDefaultI2cBus;
    unsigned int period_ms = 20;
    unsigned int warmup_samples = 100;
    double beta = 0.10;
    std::string output = "/var/www/html/imu-data/imu_quaternion.txt";
};

void signal_handler(int)
{
    g_running = false;
}

Options parse_args(int argc, char ** argv)
{
    Options options;

    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        if ((arg == "--help") || (arg == "-h"))
        {
            std::cout
                << "Usage: ./imu_web_publisher [--i2c-bus /dev/i2c-1] [--period-ms 20]\n"
                << "                          [--warmup-samples 100] [--beta 0.10]\n"
                << "                          [--output /var/www/html/imu-data/imu_quaternion.txt]\n";
            std::exit(0);
        }
        if ((arg == "--i2c-bus") && (i + 1 < argc))
        {
            options.i2c_bus = argv[++i];
            continue;
        }
        if ((arg == "--period-ms") && (i + 1 < argc))
        {
            options.period_ms = static_cast<unsigned int>(std::stoul(argv[++i]));
            continue;
        }
        if ((arg == "--warmup-samples") && (i + 1 < argc))
        {
            options.warmup_samples = static_cast<unsigned int>(std::stoul(argv[++i]));
            continue;
        }
        if ((arg == "--beta") && (i + 1 < argc))
        {
            options.beta = std::stod(argv[++i]);
            continue;
        }
        if ((arg == "--output") && (i + 1 < argc))
        {
            options.output = argv[++i];
            continue;
        }

        throw std::runtime_error("Unknown argument: " + arg);
    }

    if (options.period_ms == 0)
    {
        throw std::runtime_error("--period-ms must be greater than zero");
    }

    return options;
}

} // namespace

int main(int argc, char ** argv)
{
    try
    {
        const Options options = parse_args(argc, argv);

        std::signal(SIGINT, signal_handler);
        std::signal(SIGTERM, signal_handler);

        BerryImuHal imu(options.i2c_bus);
        imu.initialize();

        std::cout << "Task 1 publisher starting\n"
                  << "I2C bus: " << options.i2c_bus << "\n"
                  << "Output file: " << options.output << "\n"
                  << "Loop period: " << options.period_ms << " ms\n"
                  << "Madgwick beta: " << options.beta << "\n"
                  << "Calibrating gyro bias. Keep the IMU still...\n";

        imu.measure_gyro_bias(options.warmup_samples, options.period_ms);

        MadgwickFilter filter(options.beta);
        auto last = std::chrono::steady_clock::now();
        std::uint64_t sequence = 0;

        std::cout << "Publishing live quaternion values. Press Ctrl+C to stop.\n";

        while (g_running)
        {
            const ImuSample sample = imu.read_sample();
            const auto now = std::chrono::steady_clock::now();
            double dt = std::chrono::duration<double>(now - last).count();
            if ((dt <= 0.0) || (dt > 1.0))
            {
                dt = static_cast<double>(options.period_ms) / 1000.0;
            }
            last = now;

            filter.update(dt, sample.gyro_rad_s, sample.accel_g, sample.mag_gauss);
            Quaternion q = filter.quaternion();
            q.normalize();
            ++sequence;

            std::string error;
            if (!write_quaternion_file(options.output, q, sequence, &error))
            {
                std::cerr << "Warning: " << error << '\n';
            }

            std::cout << std::fixed << std::setprecision(6)
                      << "seq=" << sequence
                      << " q=[" << q.w << ", " << q.x << ", " << q.y << ", " << q.z << "]\n";

            std::this_thread::sleep_for(std::chrono::milliseconds(options.period_ms));
        }

        return 0;
    }
    catch (const std::exception & e)
    {
        std::cerr << "Error: " << e.what() << '\n';
        return 1;
    }
}
