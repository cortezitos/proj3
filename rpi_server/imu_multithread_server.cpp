#include <algorithm>
#include <arpa/inet.h>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <netinet/in.h>
#include <sstream>
#include <string>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <thread>
#include <ctime>
#include <unistd.h>
#include <vector>

#include <linux/i2c-dev.h>

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr const char* kI2cDevice = "/dev/i2c-1";
constexpr uint8_t kAddrLsm6 = 0x6B;
constexpr uint8_t kAddrLis3 = 0x1E;
constexpr uint8_t kRegWhoAmILsm6 = 0x0F;
constexpr uint8_t kRegCtrl1Xl = 0x10;
constexpr uint8_t kRegCtrl2G = 0x11;
constexpr uint8_t kRegOutXLG = 0x22;
constexpr uint8_t kRegOutXLXl = 0x28;
constexpr uint8_t kRegWhoAmILis3 = 0x0F;
constexpr uint8_t kRegCtrlReg1 = 0x20;
constexpr uint8_t kRegCtrlReg2 = 0x21;
constexpr uint8_t kRegCtrlReg3 = 0x22;
constexpr uint8_t kRegOutXL = 0x28;
constexpr double kGyroDpsPerLsb = 0.0175;
constexpr double kAccelGPerLsb = 0.000061;
constexpr double kMagGaussPerLsb = 1.0 / 6842.0;
constexpr int kDefaultPort = 5555;
constexpr const char* kDefaultWebOutput = "/tmp/imu_quaternion.txt";
constexpr auto kThreadWait = std::chrono::milliseconds(100);
constexpr auto kMockSamplePeriod = std::chrono::milliseconds(40);

volatile std::sig_atomic_t g_running = 1;

void onSignal(int) {
    g_running = 0;
}

[[noreturn]] void fatal(const std::string& message) {
    std::cerr << message << "\n";
    std::exit(1);
}

int i2cOpen(const char* path) {
    const int fd = ::open(path, O_RDWR);
    if (fd < 0) {
        fatal(std::string("Failed to open ") + path + ": " + std::strerror(errno));
    }
    return fd;
}

void i2cSetSlave(int fd, uint8_t address) {
    if (ioctl(fd, I2C_SLAVE, address) < 0) {
        std::ostringstream oss;
        oss << "Failed to select I2C slave 0x" << std::hex << static_cast<int>(address)
            << ": " << std::strerror(errno);
        fatal(oss.str());
    }
}

uint8_t read8(int fd, uint8_t address, uint8_t reg) {
    i2cSetSlave(fd, address);
    if (::write(fd, &reg, 1) != 1) {
        fatal(std::string("I2C write(reg) failed: ") + std::strerror(errno));
    }
    uint8_t value = 0;
    if (::read(fd, &value, 1) != 1) {
        fatal(std::string("I2C read failed: ") + std::strerror(errno));
    }
    return value;
}

void write8(int fd, uint8_t address, uint8_t reg, uint8_t value) {
    i2cSetSlave(fd, address);
    const uint8_t payload[2] = {reg, value};
    if (::write(fd, payload, 2) != 2) {
        fatal(std::string("I2C write(reg,val) failed: ") + std::strerror(errno));
    }
}

int16_t read16Le(int fd, uint8_t address, uint8_t regLow) {
    const uint8_t lo = read8(fd, address, regLow);
    const uint8_t hi = read8(fd, address, static_cast<uint8_t>(regLow + 1));
    return static_cast<int16_t>(static_cast<uint16_t>(lo) |
                                (static_cast<uint16_t>(hi) << 8));
}

bool setNonBlocking(int fd) {
    const int flags = fcntl(fd, F_GETFL, 0);
    if (flags < 0) {
        return false;
    }
    return fcntl(fd, F_SETFL, flags | O_NONBLOCK) == 0;
}

struct Quaternion {
    double w{1.0};
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

Quaternion normalizeQuaternion(const Quaternion& q) {
    const double norm = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (norm <= 0.0) {
        return {};
    }
    return {q.w / norm, q.x / norm, q.y / norm, q.z / norm};
}

struct PhysicalImuSample {
    double gx{0.0};
    double gy{0.0};
    double gz{0.0};
    double ax{0.0};
    double ay{0.0};
    double az{1.0};
    double mx{0.2};
    double my{0.0};
    double mz{0.4};
    std::chrono::steady_clock::time_point capturedAt{};
};

struct SharedPhysicalState {
    PhysicalImuSample sample{};
    uint64_t sequence{0};
    bool ready{false};
    std::mutex mutex;
    std::condition_variable cv;
};

struct SharedQuaternionState {
    Quaternion quaternion{};
    uint64_t sequence{0};
    bool ready{false};
    std::mutex mutex;
    std::condition_variable cv;
};

class MadgwickAHRS {
public:
    explicit MadgwickAHRS(double beta = 0.10) : beta_(beta) {}

    void update(
        double gx, double gy, double gz,
        double ax, double ay, double az,
        double mx, double my, double mz,
        double dtSeconds) {
        if ((ax == 0.0) && (ay == 0.0) && (az == 0.0)) {
            return;
        }
        if ((mx == 0.0) && (my == 0.0) && (mz == 0.0)) {
            updateImu(gx, gy, gz, ax, ay, az, dtSeconds);
            return;
        }

        double q1 = q_.w;
        double q2 = q_.x;
        double q3 = q_.y;
        double q4 = q_.z;

        double recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        recipNorm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        const double _2q1mx = 2.0 * q1 * mx;
        const double _2q1my = 2.0 * q1 * my;
        const double _2q1mz = 2.0 * q1 * mz;
        const double _2q2mx = 2.0 * q2 * mx;
        const double _2q1 = 2.0 * q1;
        const double _2q2 = 2.0 * q2;
        const double _2q3 = 2.0 * q3;
        const double _2q4 = 2.0 * q4;
        const double q1q1 = q1 * q1;
        const double q1q2 = q1 * q2;
        const double q1q3 = q1 * q3;
        const double q1q4 = q1 * q4;
        const double q2q2 = q2 * q2;
        const double q2q3 = q2 * q3;
        const double q2q4 = q2 * q4;
        const double q3q3 = q3 * q3;
        const double q3q4 = q3 * q4;
        const double q4q4 = q4 * q4;

        const double hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 +
                          mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 -
                          mx * q3q3 - mx * q4q4;
        const double hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 +
                          _2q2mx * q3 - my * q2q2 + my * q3q3 +
                          _2q3 * mz * q4 - my * q4q4;
        const double _2bx = std::sqrt(hx * hx + hy * hy);
        const double _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 +
                            _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 -
                            mz * q3q3 + mz * q4q4;
        const double _4bx = 2.0 * _2bx;
        const double _4bz = 2.0 * _2bz;

        double s1 = -_2q3 * (2.0 * (q2q4 - q1q3) - ax) +
                    _2q2 * (2.0 * (q1q2 + q3q4) - ay) -
                    _2bz * q3 * (_2bx * (0.5 - q3q3 - q4q4) +
                                 _2bz * (q2q4 - q1q3) - mx) +
                    (-_2bx * q4 + _2bz * q2) *
                        (_2bx * (q2q3 - q1q4) +
                         _2bz * (q1q2 + q3q4) - my) +
                    _2bx * q3 * (_2bx * (q1q3 + q2q4) +
                                 _2bz * (0.5 - q2q2 - q3q3) - mz);

        double s2 = _2q4 * (2.0 * (q2q4 - q1q3) - ax) +
                    _2q1 * (2.0 * (q1q2 + q3q4) - ay) -
                    4.0 * q2 * (1.0 - 2.0 * (q2q2 + q3q3) - az) +
                    _2bz * q4 * (_2bx * (0.5 - q3q3 - q4q4) +
                                 _2bz * (q2q4 - q1q3) - mx) +
                    (_2bx * q3 + _2bz * q1) *
                        (_2bx * (q2q3 - q1q4) +
                         _2bz * (q1q2 + q3q4) - my) +
                    (_2bx * q4 - _4bz * q2) *
                        (_2bx * (q1q3 + q2q4) +
                         _2bz * (0.5 - q2q2 - q3q3) - mz);

        double s3 = -_2q1 * (2.0 * (q2q4 - q1q3) - ax) +
                    _2q4 * (2.0 * (q1q2 + q3q4) - ay) -
                    4.0 * q3 * (1.0 - 2.0 * (q2q2 + q3q3) - az) +
                    (-_4bx * q3 - _2bz * q1) *
                        (_2bx * (0.5 - q3q3 - q4q4) +
                         _2bz * (q2q4 - q1q3) - mx) +
                    (_2bx * q2 + _2bz * q4) *
                        (_2bx * (q2q3 - q1q4) +
                         _2bz * (q1q2 + q3q4) - my) +
                    (_2bx * q1 - _4bz * q3) *
                        (_2bx * (q1q3 + q2q4) +
                         _2bz * (0.5 - q2q2 - q3q3) - mz);

        double s4 = _2q2 * (2.0 * (q2q4 - q1q3) - ax) +
                    _2q3 * (2.0 * (q1q2 + q3q4) - ay) +
                    (-_4bx * q4 + _2bz * q2) *
                        (_2bx * (0.5 - q3q3 - q4q4) +
                         _2bz * (q2q4 - q1q3) - mx) +
                    (-_2bx * q1 + _2bz * q3) *
                        (_2bx * (q2q3 - q1q4) +
                         _2bz * (q1q2 + q3q4) - my) +
                    _2bx * q2 * (_2bx * (q1q3 + q2q4) +
                                 _2bz * (0.5 - q2q2 - q3q3) - mz);

        const double stepNorm = s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4;
        if (stepNorm > 0.0) {
            recipNorm = invSqrt(stepNorm);
            s1 *= recipNorm;
            s2 *= recipNorm;
            s3 *= recipNorm;
            s4 *= recipNorm;
        } else {
            s1 = s2 = s3 = s4 = 0.0;
        }

        const double qDot1 = 0.5 * (-q2 * gx - q3 * gy - q4 * gz) - beta_ * s1;
        const double qDot2 = 0.5 * (q1 * gx + q3 * gz - q4 * gy) - beta_ * s2;
        const double qDot3 = 0.5 * (q1 * gy - q2 * gz + q4 * gx) - beta_ * s3;
        const double qDot4 = 0.5 * (q1 * gz + q2 * gy - q3 * gx) - beta_ * s4;

        q1 += qDot1 * dtSeconds;
        q2 += qDot2 * dtSeconds;
        q3 += qDot3 * dtSeconds;
        q4 += qDot4 * dtSeconds;

        recipNorm = invSqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
        q_ = {q1 * recipNorm, q2 * recipNorm, q3 * recipNorm, q4 * recipNorm};
    }

    Quaternion quaternion() const {
        return q_;
    }

private:
    static double invSqrt(double value) {
        if (value <= 0.0) {
            return 0.0;
        }
        return 1.0 / std::sqrt(value);
    }

    void updateImu(
        double gx, double gy, double gz,
        double ax, double ay, double az,
        double dtSeconds) {
        if ((ax == 0.0) && (ay == 0.0) && (az == 0.0)) {
            return;
        }

        double q1 = q_.w;
        double q2 = q_.x;
        double q3 = q_.y;
        double q4 = q_.z;

        double recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        const double _2q1 = 2.0 * q1;
        const double _2q2 = 2.0 * q2;
        const double _2q3 = 2.0 * q3;
        const double _2q4 = 2.0 * q4;
        const double _4q1 = 4.0 * q1;
        const double _4q2 = 4.0 * q2;
        const double _4q3 = 4.0 * q3;
        const double _8q2 = 8.0 * q2;
        const double _8q3 = 8.0 * q3;
        const double q1q1 = q1 * q1;
        const double q2q2 = q2 * q2;
        const double q3q3 = q3 * q3;
        const double q4q4 = q4 * q4;

        double s1 = _4q1 * q3q3 + _2q3 * ax + _4q1 * q2q2 - _2q2 * ay;
        double s2 = _4q2 * q4q4 - _2q4 * ax + 4.0 * q1q1 * q2 - _2q1 * ay -
                    _4q2 + _8q2 * q2q2 + _8q2 * q3q3 + _4q2 * az;
        double s3 = 4.0 * q1q1 * q3 + _2q1 * ax + _4q3 * q4q4 - _2q4 * ay -
                    _4q3 + _8q3 * q2q2 + _8q3 * q3q3 + _4q3 * az;
        double s4 = 4.0 * q2q2 * q4 - _2q2 * ax + 4.0 * q3q3 * q4 - _2q3 * ay;

        const double stepNorm = s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4;
        if (stepNorm > 0.0) {
            recipNorm = invSqrt(stepNorm);
            s1 *= recipNorm;
            s2 *= recipNorm;
            s3 *= recipNorm;
            s4 *= recipNorm;
        } else {
            s1 = s2 = s3 = s4 = 0.0;
        }

        const double qDot1 = 0.5 * (-q2 * gx - q3 * gy - q4 * gz) - beta_ * s1;
        const double qDot2 = 0.5 * (q1 * gx + q3 * gz - q4 * gy) - beta_ * s2;
        const double qDot3 = 0.5 * (q1 * gy - q2 * gz + q4 * gx) - beta_ * s3;
        const double qDot4 = 0.5 * (q1 * gz + q2 * gy - q3 * gx) - beta_ * s4;

        q1 += qDot1 * dtSeconds;
        q2 += qDot2 * dtSeconds;
        q3 += qDot3 * dtSeconds;
        q4 += qDot4 * dtSeconds;

        recipNorm = invSqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
        q_ = {q1 * recipNorm, q2 * recipNorm, q3 * recipNorm, q4 * recipNorm};
    }

    double beta_;
    Quaternion q_{};
};

class ImuSource {
public:
    explicit ImuSource(bool mockMode) : mockMode_(mockMode) {}

    ~ImuSource() {
        closeDevice();
    }

    void initialize() {
        if (mockMode_) {
            startTime_ = std::chrono::steady_clock::now();
            lastMockSample_ = startTime_;
            std::cout << "Starting in mock IMU mode.\n";
            return;
        }

        fd_ = i2cOpen(kI2cDevice);
        const uint8_t whoLsm6 = read8(fd_, kAddrLsm6, kRegWhoAmILsm6);
        const uint8_t whoLis3 = read8(fd_, kAddrLis3, kRegWhoAmILis3);

        write8(fd_, kAddrLsm6, kRegCtrl1Xl, 0x20);
        write8(fd_, kAddrLsm6, kRegCtrl2G, 0x24);
        write8(fd_, kAddrLis3, kRegCtrlReg1, 0x6C);
        write8(fd_, kAddrLis3, kRegCtrlReg2, 0x00);
        write8(fd_, kAddrLis3, kRegCtrlReg3, 0x00);

        std::cout << "LSM6DS33 WHO_AM_I = 0x" << std::hex << static_cast<int>(whoLsm6) << "\n";
        std::cout << "LIS3MDL  WHO_AM_I = 0x" << std::hex << static_cast<int>(whoLis3) << std::dec << "\n";
    }

    bool readPhysical(PhysicalImuSample& sample) {
        return mockMode_ ? readMock(sample) : readHardware(sample);
    }

private:
    bool readHardware(PhysicalImuSample& sample) {
        const int16_t gxRaw = read16Le(fd_, kAddrLsm6, kRegOutXLG);
        const int16_t gyRaw = read16Le(fd_, kAddrLsm6, static_cast<uint8_t>(kRegOutXLG + 2));
        const int16_t gzRaw = read16Le(fd_, kAddrLsm6, static_cast<uint8_t>(kRegOutXLG + 4));

        const int16_t axRaw = read16Le(fd_, kAddrLsm6, kRegOutXLXl);
        const int16_t ayRaw = read16Le(fd_, kAddrLsm6, static_cast<uint8_t>(kRegOutXLXl + 2));
        const int16_t azRaw = read16Le(fd_, kAddrLsm6, static_cast<uint8_t>(kRegOutXLXl + 4));

        const int16_t mxRaw = read16Le(fd_, kAddrLis3, kRegOutXL);
        const int16_t myRaw = read16Le(fd_, kAddrLis3, static_cast<uint8_t>(kRegOutXL + 2));
        const int16_t mzRaw = read16Le(fd_, kAddrLis3, static_cast<uint8_t>(kRegOutXL + 4));

        sample.gx = static_cast<double>(gxRaw) * kGyroDpsPerLsb * (kPi / 180.0);
        sample.gy = static_cast<double>(gyRaw) * kGyroDpsPerLsb * (kPi / 180.0);
        sample.gz = static_cast<double>(gzRaw) * kGyroDpsPerLsb * (kPi / 180.0);
        sample.ax = static_cast<double>(axRaw) * kAccelGPerLsb;
        sample.ay = static_cast<double>(ayRaw) * kAccelGPerLsb;
        sample.az = static_cast<double>(azRaw) * kAccelGPerLsb;
        sample.mx = static_cast<double>(mxRaw) * kMagGaussPerLsb;
        sample.my = static_cast<double>(myRaw) * kMagGaussPerLsb;
        sample.mz = static_cast<double>(mzRaw) * kMagGaussPerLsb;
        sample.capturedAt = std::chrono::steady_clock::now();
        std::this_thread::sleep_for(kMockSamplePeriod);
        return true;
    }

    bool readMock(PhysicalImuSample& sample) {
        const auto now = std::chrono::steady_clock::now();
        const double t = std::chrono::duration<double>(now - startTime_).count();

        sample.gx = 0.35 * std::cos(1.1 * t);
        sample.gy = 0.28 * std::sin(0.7 * t);
        sample.gz = 0.22 * std::cos(0.5 * t + 0.3);

        sample.ax = 0.25 * std::sin(0.6 * t);
        sample.ay = 0.18 * std::cos(0.8 * t);
        const double azSq = std::max(0.1, 1.0 - sample.ax * sample.ax - sample.ay * sample.ay);
        sample.az = std::sqrt(azSq);

        sample.mx = 0.28 + 0.05 * std::cos(0.4 * t);
        sample.my = 0.03 + 0.07 * std::sin(0.5 * t);
        sample.mz = 0.41 + 0.03 * std::cos(0.3 * t);
        sample.capturedAt = now;

        const auto elapsed = std::chrono::steady_clock::now() - lastMockSample_;
        if (elapsed < kMockSamplePeriod) {
            std::this_thread::sleep_for(kMockSamplePeriod - elapsed);
        }
        lastMockSample_ = std::chrono::steady_clock::now();
        return true;
    }

    void closeDevice() {
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
    }

    bool mockMode_{false};
    int fd_{-1};
    std::chrono::steady_clock::time_point startTime_{};
    std::chrono::steady_clock::time_point lastMockSample_{};
};

struct Options {
    bool mockMode{false};
    int port{kDefaultPort};
    std::string webOutput{kDefaultWebOutput};
};

Options parseOptions(int argc, char** argv) {
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--mock") {
            options.mockMode = true;
        } else if (arg == "--port") {
            if (i + 1 >= argc) {
                fatal("Missing value after --port");
            }
            options.port = std::stoi(argv[++i]);
            if (options.port <= 0 || options.port > 65535) {
                fatal("Port must be between 1 and 65535.");
            }
        } else if (arg == "--web-output") {
            if (i + 1 >= argc) {
                fatal("Missing value after --web-output");
            }
            options.webOutput = argv[++i];
        } else if (arg == "--help") {
            std::cout << "Usage: ./imu_server [--mock] [--port 5555] [--web-output /tmp/imu_quaternion.txt]\n";
            std::exit(0);
        } else {
            fatal("Unknown option: " + arg);
        }
    }
    return options;
}

void writeLatestQuaternionFile(const std::string& path, const Quaternion& q, uint64_t sequence) {
    const std::string tempPath = path + ".tmp";
    std::ofstream out(tempPath, std::ios::trunc);
    if (!out) {
        std::cerr << "Warning: failed to open " << tempPath << " for web output.\n";
        return;
    }

    const auto now = std::chrono::system_clock::now();
    const std::time_t nowTime = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
    localtime_r(&nowTime, &tm);

    out << std::fixed << std::setprecision(6)
        << q.w << ' ' << q.x << ' ' << q.y << ' ' << q.z << "\n"
        << std::put_time(&tm, "%Y-%m-%d %H:%M:%S") << "\n"
        << sequence << "\n";
    out.close();

    if (std::rename(tempPath.c_str(), path.c_str()) != 0) {
        std::cerr << "Warning: failed to publish web output file " << path << ": "
                  << std::strerror(errno) << "\n";
    }
}

std::string formatQuaternionLine(const Quaternion& q) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6)
        << q.w << ' ' << q.x << ' ' << q.y << ' ' << q.z << '\n';
    return oss.str();
}

int createListenSocket(int port) {
    const int listenFd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (listenFd < 0) {
        fatal(std::string("socket() failed: ") + std::strerror(errno));
    }

    int reuse = 1;
    if (setsockopt(listenFd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) != 0) {
        fatal(std::string("setsockopt(SO_REUSEADDR) failed: ") + std::strerror(errno));
    }

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_ANY);
    address.sin_port = htons(static_cast<uint16_t>(port));

    if (::bind(listenFd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0) {
        fatal(std::string("bind() failed: ") + std::strerror(errno));
    }
    if (::listen(listenFd, 4) != 0) {
        fatal(std::string("listen() failed: ") + std::strerror(errno));
    }
    if (!setNonBlocking(listenFd)) {
        fatal(std::string("Failed to set listen socket non-blocking: ") + std::strerror(errno));
    }

    return listenFd;
}

void acceptPendingClients(int listenFd, SharedQuaternionState& shared, std::vector<int>& clients) {
    while (g_running) {
        sockaddr_in clientAddress{};
        socklen_t clientLength = sizeof(clientAddress);
        const int clientFd = ::accept(listenFd, reinterpret_cast<sockaddr*>(&clientAddress), &clientLength);
        if (clientFd < 0) {
            if ((errno == EAGAIN) || (errno == EWOULDBLOCK)) {
                return;
            }
            std::cerr << "accept() failed: " << std::strerror(errno) << "\n";
            return;
        }

        setNonBlocking(clientFd);
        char ipBuffer[INET_ADDRSTRLEN] = {};
        const char* printable = inet_ntop(AF_INET, &clientAddress.sin_addr, ipBuffer, sizeof(ipBuffer));
        std::cout << "Client connected from " << (printable ? printable : "unknown")
                  << ':' << ntohs(clientAddress.sin_port) << "\n";

        {
            std::lock_guard<std::mutex> lock(shared.mutex);
            if (shared.ready) {
                const std::string line = formatQuaternionLine(shared.quaternion);
                const ssize_t sent = ::send(clientFd, line.data(), line.size(), MSG_NOSIGNAL | MSG_DONTWAIT);
                if (sent != static_cast<ssize_t>(line.size())) {
                    ::close(clientFd);
                    continue;
                }
            }
        }

        clients.push_back(clientFd);
    }
}

void closeClients(std::vector<int>& clients) {
    for (const int fd : clients) {
        if (fd >= 0) {
            ::close(fd);
        }
    }
    clients.clear();
}

void samplerLoop(ImuSource& source, SharedPhysicalState& physicalState) {
    while (g_running) {
        PhysicalImuSample sample;
        if (!source.readPhysical(sample)) {
            continue;
        }

        {
            std::lock_guard<std::mutex> lock(physicalState.mutex);
            physicalState.sample = sample;
            ++physicalState.sequence;
            physicalState.ready = true;
        }
        physicalState.cv.notify_one();
    }
    physicalState.cv.notify_all();
}

void fusionLoop(const std::string& webOutput,
                SharedPhysicalState& physicalState,
                SharedQuaternionState& quaternionState) {
    MadgwickAHRS filter(0.10);
    uint64_t processedSequence = 0;
    auto lastSampleTime = std::chrono::steady_clock::now();
    bool firstSample = true;

    while (g_running) {
        PhysicalImuSample sample;
        {
            std::unique_lock<std::mutex> lock(physicalState.mutex);
            physicalState.cv.wait_for(lock, kThreadWait, [&] {
                return !g_running || (physicalState.ready && physicalState.sequence != processedSequence);
            });
            if (!g_running) {
                break;
            }
            if (!physicalState.ready || physicalState.sequence == processedSequence) {
                continue;
            }
            sample = physicalState.sample;
            processedSequence = physicalState.sequence;
        }

        double dt = 0.05;
        if (firstSample) {
            firstSample = false;
        } else {
            dt = std::chrono::duration<double>(sample.capturedAt - lastSampleTime).count();
            if ((dt <= 0.0) || (dt > 1.0)) {
                dt = 0.05;
            }
        }
        lastSampleTime = sample.capturedAt;

        filter.update(sample.gx, sample.gy, sample.gz,
                      sample.ax, sample.ay, sample.az,
                      sample.mx, sample.my, sample.mz,
                      dt);

        const Quaternion q = normalizeQuaternion(filter.quaternion());

        {
            std::lock_guard<std::mutex> lock(quaternionState.mutex);
            quaternionState.quaternion = q;
            ++quaternionState.sequence;
            quaternionState.ready = true;
        }
        quaternionState.cv.notify_all();
        writeLatestQuaternionFile(webOutput, q, processedSequence);
    }
    quaternionState.cv.notify_all();
}

void socketLoop(int port, SharedQuaternionState& quaternionState) {
    const int listenFd = createListenSocket(port);
    std::vector<int> clients;
    uint64_t sentSequence = 0;

    std::cout << "Socket server listening on port " << port << "\n";

    while (g_running) {
        acceptPendingClients(listenFd, quaternionState, clients);

        Quaternion q;
        uint64_t currentSequence = sentSequence;
        {
            std::unique_lock<std::mutex> lock(quaternionState.mutex);
            quaternionState.cv.wait_for(lock, kThreadWait, [&] {
                return !g_running || (quaternionState.ready && quaternionState.sequence != sentSequence);
            });
            if (!g_running) {
                break;
            }
            if (!quaternionState.ready || quaternionState.sequence == sentSequence) {
                continue;
            }
            q = quaternionState.quaternion;
            currentSequence = quaternionState.sequence;
        }

        const std::string line = formatQuaternionLine(q);
        std::vector<int> survivors;
        survivors.reserve(clients.size());
        for (const int clientFd : clients) {
            const ssize_t sent = ::send(clientFd, line.data(), line.size(), MSG_NOSIGNAL | MSG_DONTWAIT);
            if (sent == static_cast<ssize_t>(line.size())) {
                survivors.push_back(clientFd);
                continue;
            }

            if ((sent < 0) && ((errno == EAGAIN) || (errno == EWOULDBLOCK))) {
                survivors.push_back(clientFd);
                continue;
            }

            ::close(clientFd);
        }
        clients.swap(survivors);
        sentSequence = currentSequence;
    }

    closeClients(clients);
    ::close(listenFd);
}

}  // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);
    std::signal(SIGPIPE, SIG_IGN);

    const Options options = parseOptions(argc, argv);

    try {
        ImuSource source(options.mockMode);
        source.initialize();

        SharedPhysicalState physicalState;
        SharedQuaternionState quaternionState;

        std::thread sampler([&] { samplerLoop(source, physicalState); });
        std::thread fusion([&] { fusionLoop(options.webOutput, physicalState, quaternionState); });
        std::thread socketServer([&] { socketLoop(options.port, quaternionState); });

        sampler.join();
        fusion.join();
        socketServer.join();
    } catch (const std::exception& ex) {
        std::cerr << "Fatal exception: " << ex.what() << "\n";
        return 1;
    }

    std::cout << "Server stopped.\n";
    return 0;
}
