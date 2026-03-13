#include "imu_core.h"

#include <arpa/inet.h>
#include <atomic>
#include <cerrno>
#include <condition_variable>
#include <csignal>
#include <iostream>
#include <mutex>
#include <netinet/in.h>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>

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
    int port = 5555;
    std::string web_output = "/var/www/html/imu-data/imu_quaternion.txt";
};

struct SharedPhysicalState
{
    ImuSample sample{};
    std::uint64_t sequence = 0;
    bool ready = false;
    std::mutex mutex;
    std::condition_variable cv;
};

struct SharedQuaternionState
{
    Quaternion quaternion{};
    std::uint64_t sequence = 0;
    bool ready = false;
    std::mutex mutex;
    std::condition_variable cv;
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
                << "Usage: ./imu_multithread_server [--i2c-bus /dev/i2c-1] [--period-ms 20]\n"
                << "                                 [--warmup-samples 100] [--beta 0.10]\n"
                << "                                 [--port 5555]\n"
                << "                                 [--web-output /var/www/html/imu-data/imu_quaternion.txt]\n";
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
        if ((arg == "--port") && (i + 1 < argc))
        {
            options.port = std::stoi(argv[++i]);
            continue;
        }
        if ((arg == "--web-output") && (i + 1 < argc))
        {
            options.web_output = argv[++i];
            continue;
        }

        throw std::runtime_error("Unknown argument: " + arg);
    }

    if (options.period_ms == 0)
    {
        throw std::runtime_error("--period-ms must be greater than zero");
    }
    if ((options.port <= 0) || (options.port > 65535))
    {
        throw std::runtime_error("--port must be between 1 and 65535");
    }

    return options;
}

int create_listen_socket(int port)
{
    const int listen_fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd < 0)
    {
        throw std::runtime_error("socket() failed: " + std::string(std::strerror(errno)));
    }

    int reuse = 1;
    if (setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) != 0)
    {
        ::close(listen_fd);
        throw std::runtime_error("setsockopt(SO_REUSEADDR) failed: " + std::string(std::strerror(errno)));
    }

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_ANY);
    address.sin_port = htons(static_cast<uint16_t>(port));

    if (::bind(listen_fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) != 0)
    {
        ::close(listen_fd);
        throw std::runtime_error("bind() failed: " + std::string(std::strerror(errno)));
    }
    if (::listen(listen_fd, 1) != 0)
    {
        ::close(listen_fd);
        throw std::runtime_error("listen() failed: " + std::string(std::strerror(errno)));
    }
    if (!set_nonblocking(listen_fd))
    {
        ::close(listen_fd);
        throw std::runtime_error("Failed to set listen socket non-blocking");
    }

    return listen_fd;
}

void close_client(int & client_fd)
{
    if (client_fd >= 0)
    {
        ::close(client_fd);
        client_fd = -1;
    }
}

void maybe_accept_client(int listen_fd, int & client_fd)
{
    if (client_fd >= 0)
    {
        return;
    }

    sockaddr_in client_address{};
    socklen_t client_length = sizeof(client_address);
    const int accepted_fd = ::accept(listen_fd, reinterpret_cast<sockaddr *>(&client_address), &client_length);
    if (accepted_fd < 0)
    {
        if ((errno == EAGAIN) || (errno == EWOULDBLOCK))
        {
            return;
        }

        std::cerr << "Warning: accept() failed: " << std::strerror(errno) << '\n';
        return;
    }

    if (!set_nonblocking(accepted_fd))
    {
        std::cerr << "Warning: failed to set client socket non-blocking\n";
    }

    char ip_buffer[INET_ADDRSTRLEN] = {};
    const char * ip_text = inet_ntop(AF_INET, &client_address.sin_addr, ip_buffer, sizeof(ip_buffer));
    std::cout << "Client connected from "
              << (ip_text != nullptr ? ip_text : "unknown")
              << ':'
              << ntohs(client_address.sin_port)
              << '\n';
    client_fd = accepted_fd;
}

bool send_quaternion(int client_fd, const Quaternion & q)
{
    const std::string line = format_quaternion_line(q);
    const ssize_t sent = ::send(client_fd, line.data(), line.size(), MSG_NOSIGNAL | MSG_DONTWAIT);
    return sent == static_cast<ssize_t>(line.size());
}

void acquisition_thread(BerryImuHal & imu, SharedPhysicalState & physical_state, unsigned int period_ms)
{
    while (g_running)
    {
        const ImuSample sample = imu.read_sample();
        {
            std::lock_guard<std::mutex> lock(physical_state.mutex);
            physical_state.sample = sample;
            ++physical_state.sequence;
            physical_state.ready = true;
        }
        physical_state.cv.notify_one();
        std::this_thread::sleep_for(std::chrono::milliseconds(period_ms));
    }

    physical_state.cv.notify_all();
}

void fusion_thread(SharedPhysicalState & physical_state, SharedQuaternionState & quaternion_state, double beta)
{
    MadgwickFilter filter(beta);
    std::uint64_t last_sequence = 0;
    bool first_sample = true;
    auto last_timestamp = std::chrono::steady_clock::now();

    while (g_running)
    {
        ImuSample sample;
        std::uint64_t sample_sequence = 0;
        {
            std::unique_lock<std::mutex> lock(physical_state.mutex);
            physical_state.cv.wait_for(lock, std::chrono::milliseconds(100), [&] {
                return !g_running || (physical_state.ready && physical_state.sequence != last_sequence);
            });

            if (!g_running)
            {
                break;
            }
            if (!physical_state.ready || (physical_state.sequence == last_sequence))
            {
                continue;
            }

            sample = physical_state.sample;
            sample_sequence = physical_state.sequence;
        }

        double dt = 0.05;
        if (first_sample)
        {
            first_sample = false;
        }
        else
        {
            dt = std::chrono::duration<double>(sample.captured_at - last_timestamp).count();
            if ((dt <= 0.0) || (dt > 1.0))
            {
                dt = 0.05;
            }
        }
        last_timestamp = sample.captured_at;
        last_sequence = sample_sequence;

        filter.update(dt, sample.gyro_rad_s, sample.accel_g, sample.mag_gauss);
        Quaternion q = filter.quaternion();
        q.normalize();

        {
            std::lock_guard<std::mutex> lock(quaternion_state.mutex);
            quaternion_state.quaternion = q;
            quaternion_state.sequence = sample_sequence;
            quaternion_state.ready = true;
        }
        quaternion_state.cv.notify_all();
    }

    quaternion_state.cv.notify_all();
}

void socket_thread(SharedQuaternionState & quaternion_state, int port, const std::string & web_output)
{
    const int listen_fd = create_listen_socket(port);
    int client_fd = -1;
    std::uint64_t last_sent_sequence = 0;

    std::cout << "Thread 3 socket server listening on port " << port << '\n';

    while (g_running)
    {
        maybe_accept_client(listen_fd, client_fd);

        Quaternion q;
        std::uint64_t sequence = 0;
        {
            std::unique_lock<std::mutex> lock(quaternion_state.mutex);
            quaternion_state.cv.wait_for(lock, std::chrono::milliseconds(100), [&] {
                return !g_running || (quaternion_state.ready && quaternion_state.sequence != last_sent_sequence);
            });

            if (!g_running)
            {
                break;
            }
            if (!quaternion_state.ready || (quaternion_state.sequence == last_sent_sequence))
            {
                continue;
            }

            q = quaternion_state.quaternion;
            sequence = quaternion_state.sequence;
        }

        std::string error;
        if (!write_quaternion_file(web_output, q, sequence, &error))
        {
            std::cerr << "Warning: " << error << '\n';
        }

        if ((client_fd >= 0) && !send_quaternion(client_fd, q))
        {
            std::cerr << "Client disconnected\n";
            close_client(client_fd);
        }

        last_sent_sequence = sequence;
    }

    close_client(client_fd);
    ::close(listen_fd);
}

} // namespace

int main(int argc, char ** argv)
{
    try
    {
        const Options options = parse_args(argc, argv);

        std::signal(SIGINT, signal_handler);
        std::signal(SIGTERM, signal_handler);
        std::signal(SIGPIPE, SIG_IGN);

        BerryImuHal imu(options.i2c_bus);
        imu.initialize();

        std::cout << "Task 4 threaded IMU socket server\n"
                  << "Thread 1: raw read + physical conversion\n"
                  << "Thread 2: Madgwick fusion + quaternion normalization\n"
                  << "Thread 3: socket transmission + web file publication\n"
                  << "Port: " << options.port << '\n'
                  << "Web mirror: " << options.web_output << '\n'
                  << "Calibrating gyro bias. Keep the IMU still...\n";

        imu.measure_gyro_bias(options.warmup_samples, options.period_ms);

        SharedPhysicalState physical_state;
        SharedQuaternionState quaternion_state;

        std::thread t1(acquisition_thread, std::ref(imu), std::ref(physical_state), options.period_ms);
        std::thread t2(fusion_thread, std::ref(physical_state), std::ref(quaternion_state), options.beta);
        std::thread t3(socket_thread, std::ref(quaternion_state), options.port, std::cref(options.web_output));

        t1.join();
        t2.join();
        t3.join();

        return 0;
    }
    catch (const std::exception & e)
    {
        std::cerr << "Error: " << e.what() << '\n';
        return 1;
    }
}
