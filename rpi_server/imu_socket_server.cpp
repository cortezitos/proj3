#include "imu_core.h"

#include <arpa/inet.h>
#include <atomic>
#include <cerrno>
#include <csignal>
#include <iostream>
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
                << "Usage: ./imu_socket_server [--i2c-bus /dev/i2c-1] [--period-ms 20]\n"
                << "                           [--warmup-samples 100] [--beta 0.10]\n"
                << "                           [--port 5555]\n"
                << "                           [--web-output /var/www/html/imu-data/imu_quaternion.txt]\n";
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

        std::cout << "Task 3 single-thread IMU socket server\n"
                  << "Port: " << options.port << '\n'
                  << "Web mirror: " << options.web_output << '\n'
                  << "Calibrating gyro bias. Keep the IMU still...\n";

        imu.measure_gyro_bias(options.warmup_samples, options.period_ms);

        const int listen_fd = create_listen_socket(options.port);
        int client_fd = -1;

        MadgwickFilter filter(options.beta);
        auto last = std::chrono::steady_clock::now();
        std::uint64_t sequence = 0;

        std::cout << "Listening for Qt client connections...\n";

        while (g_running)
        {
            maybe_accept_client(listen_fd, client_fd);

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
            if (!write_quaternion_file(options.web_output, q, sequence, &error))
            {
                std::cerr << "Warning: " << error << '\n';
            }

            if (client_fd >= 0 && !send_quaternion(client_fd, q))
            {
                std::cerr << "Client disconnected\n";
                close_client(client_fd);
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(options.period_ms));
        }

        close_client(client_fd);
        ::close(listen_fd);
        return 0;
    }
    catch (const std::exception & e)
    {
        std::cerr << "Error: " << e.what() << '\n';
        return 1;
    }
}
