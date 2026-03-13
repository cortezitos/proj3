#if __has_include(<GL/freeglut.h>)
#include <GL/freeglut.h>
#else
#include <GL/glut.h>
#endif

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <errno.h>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <sstream>
#include <string>
#include <thread>
#include <unistd.h>
#include <sys/ioctl.h>

#include <linux/i2c-dev.h>

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr uint8_t kLsm6Addr = 0x6B;
constexpr uint8_t kLis3Addr = 0x1E;
constexpr const char * kDefaultI2cBus = "/dev/i2c-1";

std::atomic<bool> g_running{true};

struct Vec3
{
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    Vec3 operator+(const Vec3 & other) const
    {
        return {x + other.x, y + other.y, z + other.z};
    }

    Vec3 operator-(const Vec3 & other) const
    {
        return {x - other.x, y - other.y, z - other.z};
    }

    Vec3 operator*(float scalar) const
    {
        return {x * scalar, y * scalar, z * scalar};
    }

    Vec3 & operator+=(const Vec3 & other)
    {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }
};

struct Quaternion
{
    float w = 1.0f;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    void normalize()
    {
        const float norm = std::sqrt(w * w + x * x + y * y + z * z);
        if (norm <= 0.0f)
        {
            w = 1.0f;
            x = 0.0f;
            y = 0.0f;
            z = 0.0f;
            return;
        }
        w /= norm;
        x /= norm;
        y /= norm;
        z /= norm;
    }
};

struct RawAxes
{
    int16_t x = 0;
    int16_t y = 0;
    int16_t z = 0;
};

struct ImuSample
{
    Vec3 gyro_rad_s;
    Vec3 accel_g;
    Vec3 mag_gauss;
};

struct Options
{
    std::string i2c_bus = kDefaultI2cBus;
    unsigned int period_ms = 20;
    unsigned int warmup_samples = 100;
    float beta = 0.10f;
};

struct SharedState
{
    Quaternion q{};
    std::string status = "Starting IMU...";
    std::mutex mutex;
};

SharedState g_state;

float inv_sqrt(float value)
{
    if (value <= 0.0f)
    {
        return 0.0f;
    }
    return 1.0f / std::sqrt(value);
}

void signal_handler(int)
{
    g_running = false;
}

class I2CBus
{
public:
    explicit I2CBus(const std::string & device)
    {
        fd_ = ::open(device.c_str(), O_RDWR);
        if (fd_ < 0)
        {
            throw std::runtime_error("Failed to open " + device + ": " + std::strerror(errno));
        }
    }

    ~I2CBus()
    {
        if (fd_ >= 0)
        {
            ::close(fd_);
        }
    }

    I2CBus(const I2CBus &) = delete;
    I2CBus & operator=(const I2CBus &) = delete;

    void set_slave(uint8_t addr)
    {
        if (ioctl(fd_, I2C_SLAVE, addr) < 0)
        {
            throw std::runtime_error("Failed to select I2C slave: " + std::string(std::strerror(errno)));
        }
    }

    uint8_t read_u8(uint8_t addr, uint8_t reg)
    {
        set_slave(addr);
        if (::write(fd_, &reg, 1) != 1)
        {
            throw std::runtime_error("I2C register select failed: " + std::string(std::strerror(errno)));
        }
        uint8_t value = 0;
        if (::read(fd_, &value, 1) != 1)
        {
            throw std::runtime_error("I2C byte read failed: " + std::string(std::strerror(errno)));
        }
        return value;
    }

    void write_u8(uint8_t addr, uint8_t reg, uint8_t value)
    {
        set_slave(addr);
        uint8_t buffer[2] = {reg, value};
        if (::write(fd_, buffer, sizeof(buffer)) != static_cast<ssize_t>(sizeof(buffer)))
        {
            throw std::runtime_error("I2C byte write failed: " + std::string(std::strerror(errno)));
        }
    }

    void read_block(uint8_t addr, uint8_t start_reg, uint8_t * buffer, size_t length)
    {
        set_slave(addr);
        if (::write(fd_, &start_reg, 1) != 1)
        {
            throw std::runtime_error("I2C block register select failed: " + std::string(std::strerror(errno)));
        }
        if (::read(fd_, buffer, length) != static_cast<ssize_t>(length))
        {
            throw std::runtime_error("I2C block read failed: " + std::string(std::strerror(errno)));
        }
    }

private:
    int fd_ = -1;
};

class LSM6DS33
{
public:
    explicit LSM6DS33(I2CBus & bus, uint8_t address = kLsm6Addr)
        : bus_(bus), address_(address)
    {
    }

    void verify() const
    {
        const uint8_t who = bus_.read_u8(address_, 0x0F);
        if (who != 0x69)
        {
            throw std::runtime_error("LSM6DS33 WHO_AM_I mismatch");
        }
    }

    void configure() const
    {
        bus_.write_u8(address_, 0x10, 0x8C);
        bus_.write_u8(address_, 0x11, 0x8C);
        bus_.write_u8(address_, 0x12, 0x04);
    }

    RawAxes read_gyro_raw() const
    {
        return read_axes(0x22);
    }

    RawAxes read_accel_raw() const
    {
        return read_axes(0x28);
    }

private:
    RawAxes read_axes(uint8_t first_reg) const
    {
        uint8_t block[6] = {};
        bus_.read_block(address_, first_reg, block, sizeof(block));
        return {
            static_cast<int16_t>(static_cast<uint16_t>(block[0]) | (static_cast<uint16_t>(block[1]) << 8)),
            static_cast<int16_t>(static_cast<uint16_t>(block[2]) | (static_cast<uint16_t>(block[3]) << 8)),
            static_cast<int16_t>(static_cast<uint16_t>(block[4]) | (static_cast<uint16_t>(block[5]) << 8))
        };
    }

    I2CBus & bus_;
    uint8_t address_;
};

class LIS3MDL
{
public:
    explicit LIS3MDL(I2CBus & bus, uint8_t address = kLis3Addr)
        : bus_(bus), address_(address)
    {
    }

    void verify() const
    {
        const uint8_t who = bus_.read_u8(address_, 0x0F);
        if (who != 0x3D)
        {
            throw std::runtime_error("LIS3MDL WHO_AM_I mismatch");
        }
    }

    void configure() const
    {
        bus_.write_u8(address_, 0x20, 0x70);
        bus_.write_u8(address_, 0x21, 0x00);
        bus_.write_u8(address_, 0x22, 0x00);
        bus_.write_u8(address_, 0x23, 0x0C);
    }

    RawAxes read_mag_raw() const
    {
        uint8_t block[6] = {};
        bus_.read_block(address_, static_cast<uint8_t>(0x80 | 0x28), block, sizeof(block));
        return {
            static_cast<int16_t>(static_cast<uint16_t>(block[0]) | (static_cast<uint16_t>(block[1]) << 8)),
            static_cast<int16_t>(static_cast<uint16_t>(block[2]) | (static_cast<uint16_t>(block[3]) << 8)),
            static_cast<int16_t>(static_cast<uint16_t>(block[4]) | (static_cast<uint16_t>(block[5]) << 8))
        };
    }

private:
    I2CBus & bus_;
    uint8_t address_;
};

class BerryImuHal
{
public:
    explicit BerryImuHal(const std::string & i2c_bus)
        : bus_(i2c_bus), lsm6_(bus_), lis3_(bus_)
    {
    }

    void initialize()
    {
        lsm6_.verify();
        lis3_.verify();
        lsm6_.configure();
        lis3_.configure();
    }

    void measure_gyro_bias(unsigned int samples, unsigned int period_ms)
    {
        Vec3 accum{};
        for (unsigned int i = 0; i < samples; ++i)
        {
            accum += raw_gyro_to_rad_s(lsm6_.read_gyro_raw());
            std::this_thread::sleep_for(std::chrono::milliseconds(period_ms));
        }
        const float denom = samples > 0 ? static_cast<float>(samples) : 1.0f;
        gyro_bias_ = accum * (1.0f / denom);
    }

    ImuSample read_sample() const
    {
        ImuSample sample;
        sample.gyro_rad_s = raw_gyro_to_rad_s(lsm6_.read_gyro_raw()) - gyro_bias_;
        sample.accel_g = raw_accel_to_g(lsm6_.read_accel_raw());
        sample.mag_gauss = raw_mag_to_gauss(lis3_.read_mag_raw());
        return sample;
    }

private:
    static Vec3 raw_gyro_to_rad_s(const RawAxes & raw)
    {
        constexpr float kScale = 0.07f * static_cast<float>(kPi) / 180.0f;
        return {raw.x * kScale, raw.y * kScale, raw.z * kScale};
    }

    static Vec3 raw_accel_to_g(const RawAxes & raw)
    {
        constexpr float kScale = 0.000244f;
        return {raw.x * kScale, raw.y * kScale, raw.z * kScale};
    }

    static Vec3 raw_mag_to_gauss(const RawAxes & raw)
    {
        constexpr float kScale = 1.0f / 6842.0f;
        return {raw.x * kScale, raw.y * kScale, raw.z * kScale};
    }

    I2CBus bus_;
    LSM6DS33 lsm6_;
    LIS3MDL lis3_;
    Vec3 gyro_bias_{};
};

class MadgwickFilter
{
public:
    explicit MadgwickFilter(float beta)
        : beta_(beta)
    {
    }

    void reset()
    {
        q_ = {};
    }

    void update(float dt, const Vec3 & gyro, const Vec3 & accel, const Vec3 & mag)
    {
        if (dt <= 0.0f)
        {
            return;
        }

        const float gx = gyro.x;
        const float gy = gyro.y;
        const float gz = gyro.z;
        const float ax = accel.x;
        const float ay = accel.y;
        const float az = accel.z;
        const float mx = mag.x;
        const float my = mag.y;
        const float mz = mag.z;

        if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))
        {
            update_imu(dt, gyro, accel);
            return;
        }

        float q0 = q_.w;
        float q1 = q_.x;
        float q2 = q_.y;
        float q3 = q_.z;

        float q_dot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
        float q_dot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
        float q_dot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
        float q_dot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

        if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
        {
            float norm = inv_sqrt(ax * ax + ay * ay + az * az);
            const float axn = ax * norm;
            const float ayn = ay * norm;
            const float azn = az * norm;

            norm = inv_sqrt(mx * mx + my * my + mz * mz);
            const float mxn = mx * norm;
            const float myn = my * norm;
            const float mzn = mz * norm;

            const float _2q0mx = 2.0f * q0 * mxn;
            const float _2q0my = 2.0f * q0 * myn;
            const float _2q0mz = 2.0f * q0 * mzn;
            const float _2q1mx = 2.0f * q1 * mxn;
            const float _2q0 = 2.0f * q0;
            const float _2q1 = 2.0f * q1;
            const float _2q2 = 2.0f * q2;
            const float _2q3 = 2.0f * q3;
            const float _2q0q2 = 2.0f * q0 * q2;
            const float _2q2q3 = 2.0f * q2 * q3;
            const float q0q0 = q0 * q0;
            const float q0q1 = q0 * q1;
            const float q0q2 = q0 * q2;
            const float q0q3 = q0 * q3;
            const float q1q1 = q1 * q1;
            const float q1q2 = q1 * q2;
            const float q1q3 = q1 * q3;
            const float q2q2 = q2 * q2;
            const float q2q3 = q2 * q3;
            const float q3q3 = q3 * q3;

            const float hx = mxn * q0q0 - _2q0my * q3 + _2q0mz * q2
                + mxn * q1q1 + _2q1 * myn * q2 + _2q1 * mzn * q3
                - mxn * q2q2 - mxn * q3q3;
            const float hy = _2q0mx * q3 + myn * q0q0 - _2q0mz * q1
                + _2q1mx * q2 - myn * q1q1 + myn * q2q2
                + _2q2 * mzn * q3 - myn * q3q3;
            const float _2bx = std::sqrt(hx * hx + hy * hy);
            const float _2bz = -_2q0mx * q2 + _2q0my * q1 + mzn * q0q0
                + _2q1mx * q3 - mzn * q1q1 + _2q2 * myn * q3
                - mzn * q2q2 + mzn * q3q3;
            const float _4bx = 2.0f * _2bx;
            const float _4bz = 2.0f * _2bz;

            float s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - axn)
                + _2q1 * (2.0f * q0q1 + _2q2q3 - ayn)
                - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3)
                + _2bz * (q1q3 - q0q2) - mxn)
                + (-_2bx * q3 + _2bz * q1)
                * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - myn)
                + _2bx * q2 * (_2bx * (q0q2 + q1q3)
                + _2bz * (0.5f - q1q1 - q2q2) - mzn);
            float s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - axn)
                + _2q0 * (2.0f * q0q1 + _2q2q3 - ayn)
                - 4.0f * q1 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - azn)
                + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3)
                + _2bz * (q1q3 - q0q2) - mxn)
                + (_2bx * q2 + _2bz * q0)
                * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - myn)
                + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3)
                + _2bz * (0.5f - q1q1 - q2q2) - mzn);
            float s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - axn)
                + _2q3 * (2.0f * q0q1 + _2q2q3 - ayn)
                - 4.0f * q2 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - azn)
                + (-_4bx * q2 - _2bz * q0)
                * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mxn)
                + (_2bx * q1 + _2bz * q3)
                * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - myn)
                + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3)
                + _2bz * (0.5f - q1q1 - q2q2) - mzn);
            float s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - axn)
                + _2q2 * (2.0f * q0q1 + _2q2q3 - ayn)
                + (-_4bx * q3 + _2bz * q1)
                * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mxn)
                + (-_2bx * q0 + _2bz * q2)
                * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - myn)
                + _2bx * q1 * (_2bx * (q0q2 + q1q3)
                + _2bz * (0.5f - q1q1 - q2q2) - mzn);

            norm = inv_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
            if (norm > 0.0f)
            {
                s0 *= norm;
                s1 *= norm;
                s2 *= norm;
                s3 *= norm;
                q_dot1 -= beta_ * s0;
                q_dot2 -= beta_ * s1;
                q_dot3 -= beta_ * s2;
                q_dot4 -= beta_ * s3;
            }
        }

        q0 += q_dot1 * dt;
        q1 += q_dot2 * dt;
        q2 += q_dot3 * dt;
        q3 += q_dot4 * dt;

        q_ = {q0, q1, q2, q3};
        q_.normalize();
    }

    Quaternion quaternion() const
    {
        return q_;
    }

private:
    void update_imu(float dt, const Vec3 & gyro, const Vec3 & accel)
    {
        const float gx = gyro.x;
        const float gy = gyro.y;
        const float gz = gyro.z;
        const float ax = accel.x;
        const float ay = accel.y;
        const float az = accel.z;

        float q0 = q_.w;
        float q1 = q_.x;
        float q2 = q_.y;
        float q3 = q_.z;

        float q_dot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
        float q_dot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
        float q_dot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
        float q_dot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

        if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
        {
            float norm = inv_sqrt(ax * ax + ay * ay + az * az);
            const float axn = ax * norm;
            const float ayn = ay * norm;
            const float azn = az * norm;

            const float _2q0 = 2.0f * q0;
            const float _2q1 = 2.0f * q1;
            const float _2q2 = 2.0f * q2;
            const float _2q3 = 2.0f * q3;
            const float _4q0 = 4.0f * q0;
            const float _4q1 = 4.0f * q1;
            const float _4q2 = 4.0f * q2;
            const float _8q1 = 8.0f * q1;
            const float _8q2 = 8.0f * q2;
            const float q0q0 = q0 * q0;
            const float q1q1 = q1 * q1;
            const float q2q2 = q2 * q2;
            const float q3q3 = q3 * q3;

            float s0 = _4q0 * q2q2 + _2q2 * axn + _4q0 * q1q1 - _2q1 * ayn;
            float s1 = _4q1 * q3q3 - _2q3 * axn + 4.0f * q0q0 * q1 - _2q0 * ayn
                - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * azn;
            float s2 = 4.0f * q0q0 * q2 + _2q0 * axn + _4q2 * q3q3 - _2q3 * ayn
                - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * azn;
            float s3 = 4.0f * q1q1 * q3 - _2q1 * axn + 4.0f * q2q2 * q3 - _2q2 * ayn;

            norm = inv_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
            if (norm > 0.0f)
            {
                s0 *= norm;
                s1 *= norm;
                s2 *= norm;
                s3 *= norm;
                q_dot1 -= beta_ * s0;
                q_dot2 -= beta_ * s1;
                q_dot3 -= beta_ * s2;
                q_dot4 -= beta_ * s3;
            }
        }

        q0 += q_dot1 * dt;
        q1 += q_dot2 * dt;
        q2 += q_dot3 * dt;
        q3 += q_dot4 * dt;

        q_ = {q0, q1, q2, q3};
        q_.normalize();
    }

    float beta_;
    Quaternion q_{};
};

Options parse_args(int argc, char ** argv)
{
    Options options;
    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        if ((arg == "--help") || (arg == "-h"))
        {
            std::cout
                << "Usage: ./task2 [--i2c-bus /dev/i2c-1] [--period-ms 20] [--warmup-samples 100] [--beta 0.10]\n"
                << "Shows a GLUT cube driven by the live IMU quaternion estimate.\n";
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
            options.beta = std::stof(argv[++i]);
            continue;
        }
        throw std::runtime_error("Unknown argument: " + arg);
    }
    if (options.period_ms == 0)
    {
        throw std::runtime_error("--period-ms must be > 0");
    }
    return options;
}

std::array<float, 16> quaternion_to_matrix(const Quaternion & q_in)
{
    Quaternion q = q_in;
    q.normalize();

    const float w = q.w;
    const float x = q.x;
    const float y = q.y;
    const float z = q.z;

    const float xx = x * x;
    const float yy = y * y;
    const float zz = z * z;
    const float xy = x * y;
    const float xz = x * z;
    const float yz = y * z;
    const float wx = w * x;
    const float wy = w * y;
    const float wz = w * z;

    std::array<float, 16> m{};
    m[0] = 1.0f - 2.0f * (yy + zz);
    m[4] = 2.0f * (xy - wz);
    m[8] = 2.0f * (xz + wy);
    m[12] = 0.0f;

    m[1] = 2.0f * (xy + wz);
    m[5] = 1.0f - 2.0f * (xx + zz);
    m[9] = 2.0f * (yz - wx);
    m[13] = 0.0f;

    m[2] = 2.0f * (xz - wy);
    m[6] = 2.0f * (yz + wx);
    m[10] = 1.0f - 2.0f * (xx + yy);
    m[14] = 0.0f;

    m[3] = 0.0f;
    m[7] = 0.0f;
    m[11] = 0.0f;
    m[15] = 1.0f;
    return m;
}

void draw_axes(float length)
{
    glLineWidth(2.0f);
    glBegin(GL_LINES);
    glColor3f(1.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(length, 0.0f, 0.0f);
    glColor3f(0.0f, 1.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, length, 0.0f);
    glColor3f(0.0f, 0.0f, 1.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, length);
    glEnd();
}

void draw_text(float x, float y, const std::string & text)
{
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    gluOrtho2D(0.0, 1.0, 0.0, 1.0);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    glColor3f(0.92f, 0.92f, 0.92f);
    glRasterPos2f(x, y);
    for (char c : text)
    {
        glutBitmapCharacter(GLUT_BITMAP_8_BY_13, c);
    }

    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
}

std::string format_status(const Quaternion & q)
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3)
        << "q=[" << q.w << ' ' << q.x << ' ' << q.y << ' ' << q.z << ']';
    return oss.str();
}

void imu_thread_main(Options options)
{
    try
    {
        BerryImuHal imu(options.i2c_bus);
        imu.initialize();
        {
            std::lock_guard<std::mutex> lock(g_state.mutex);
            g_state.status = "Calibrating gyro bias. Keep the IMU still...";
        }
        imu.measure_gyro_bias(options.warmup_samples, options.period_ms);

        MadgwickFilter filter(options.beta);
        auto last = std::chrono::steady_clock::now();

        while (g_running)
        {
            const ImuSample sample = imu.read_sample();
            const auto now = std::chrono::steady_clock::now();
            std::chrono::duration<float> dt = now - last;
            last = now;
            filter.update(dt.count(), sample.gyro_rad_s, sample.accel_g, sample.mag_gauss);
            const Quaternion q = filter.quaternion();

            {
                std::lock_guard<std::mutex> lock(g_state.mutex);
                g_state.q = q;
                g_state.status = format_status(q);
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(options.period_ms));
        }
    }
    catch (const std::exception & e)
    {
        std::lock_guard<std::mutex> lock(g_state.mutex);
        g_state.status = std::string("Error: ") + e.what();
        g_running = false;
    }
}

void display()
{
    Quaternion q;
    std::string status;
    {
        std::lock_guard<std::mutex> lock(g_state.mutex);
        q = g_state.q;
        status = g_state.status;
    }

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(0.0, 0.0, 3.2, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);

    draw_axes(1.2f);

    const std::array<float, 16> transform = quaternion_to_matrix(q);
    glPushMatrix();
    glMultMatrixf(transform.data());
    draw_axes(0.9f);
    glColor3f(0.72f, 0.75f, 0.88f);
    glutSolidCube(1.0);
    glColor3f(1.0f, 1.0f, 1.0f);
    glutWireCube(1.01);
    glPopMatrix();

    draw_text(0.03f, 0.95f, "Task 2: IMU Cube Viewer");
    draw_text(0.03f, 0.91f, status);
    draw_text(0.03f, 0.87f, "q or ESC quits");

    glutSwapBuffers();
}

void reshape(int width, int height)
{
    if (height <= 0)
    {
        height = 1;
    }
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0, static_cast<double>(width) / static_cast<double>(height), 0.1, 100.0);
    glMatrixMode(GL_MODELVIEW);
}

void keyboard(unsigned char key, int, int)
{
    if ((key == 27) || (key == 'q'))
    {
        g_running = false;
        std::exit(0);
    }
}

void idle()
{
    glutPostRedisplay();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

void init_gl()
{
    glEnable(GL_DEPTH_TEST);
    glClearColor(0.08f, 0.08f, 0.10f, 1.0f);
}

} // namespace

int main(int argc, char ** argv)
{
    try
    {
        const Options options = parse_args(argc, argv);
        std::signal(SIGINT, signal_handler);
        std::signal(SIGTERM, signal_handler);

        std::thread imu_thread(imu_thread_main, options);

        glutInit(&argc, argv);
        glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
        glutInitWindowSize(1000, 700);
        glutCreateWindow("Task 2 - IMU Quaternion Cube");
        init_gl();
        glutDisplayFunc(display);
        glutReshapeFunc(reshape);
        glutKeyboardFunc(keyboard);
        glutIdleFunc(idle);
        glutMainLoop();

        g_running = false;
        imu_thread.join();
        return 0;
    }
    catch (const std::exception & e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}
