#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <errno.h>
#include <fcntl.h>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unistd.h>
#include <sys/ioctl.h>

#include <linux/i2c-dev.h>

namespace proj3
{

constexpr double kPi = 3.14159265358979323846;
constexpr uint8_t kLsm6Addr = 0x6B;
constexpr uint8_t kLis3Addr = 0x1E;
constexpr const char * kDefaultI2cBus = "/dev/i2c-1";

struct Vec3
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    Vec3 operator+(const Vec3 & other) const
    {
        return {x + other.x, y + other.y, z + other.z};
    }

    Vec3 operator-(const Vec3 & other) const
    {
        return {x - other.x, y - other.y, z - other.z};
    }

    Vec3 operator*(double scalar) const
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
    double w = 1.0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    void normalize()
    {
        const double norm = std::sqrt(w * w + x * x + y * y + z * z);
        if (norm <= 0.0)
        {
            w = 1.0;
            x = 0.0;
            y = 0.0;
            z = 0.0;
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
    std::chrono::steady_clock::time_point captured_at{};
};

inline double inv_sqrt(double value)
{
    if (value <= 0.0)
    {
        return 0.0;
    }

    return 1.0 / std::sqrt(value);
}

inline std::string hex_byte(uint8_t value)
{
    constexpr char digits[] = "0123456789ABCDEF";
    std::string out(2, '0');
    out[0] = digits[(value >> 4) & 0x0F];
    out[1] = digits[value & 0x0F];
    return out;
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
            throw std::runtime_error("Failed to select I2C slave 0x" + hex_byte(addr) + ": " + std::strerror(errno));
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
        constexpr uint8_t kWhoAmIReg = 0x0F;
        constexpr uint8_t kExpected = 0x69;
        const uint8_t who = bus_.read_u8(address_, kWhoAmIReg);
        if (who != kExpected)
        {
            throw std::runtime_error("LSM6DS33 WHO_AM_I mismatch: expected 0x69, got 0x" + hex_byte(who));
        }
    }

    void configure() const
    {
        constexpr uint8_t kCtrl1Xl = 0x10;
        constexpr uint8_t kCtrl2G = 0x11;
        constexpr uint8_t kCtrl3C = 0x12;

        bus_.write_u8(address_, kCtrl1Xl, 0x8C);
        bus_.write_u8(address_, kCtrl2G, 0x8C);
        bus_.write_u8(address_, kCtrl3C, 0x04);
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
        constexpr uint8_t kWhoAmIReg = 0x0F;
        constexpr uint8_t kExpected = 0x3D;
        const uint8_t who = bus_.read_u8(address_, kWhoAmIReg);
        if (who != kExpected)
        {
            throw std::runtime_error("LIS3MDL WHO_AM_I mismatch: expected 0x3D, got 0x" + hex_byte(who));
        }
    }

    void configure() const
    {
        constexpr uint8_t kCtrlReg1 = 0x20;
        constexpr uint8_t kCtrlReg2 = 0x21;
        constexpr uint8_t kCtrlReg3 = 0x22;
        constexpr uint8_t kCtrlReg4 = 0x23;

        bus_.write_u8(address_, kCtrlReg1, 0x70);
        bus_.write_u8(address_, kCtrlReg2, 0x00);
        bus_.write_u8(address_, kCtrlReg3, 0x00);
        bus_.write_u8(address_, kCtrlReg4, 0x0C);
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
    explicit BerryImuHal(const std::string & i2c_bus = kDefaultI2cBus)
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
            const RawAxes raw = lsm6_.read_gyro_raw();
            accum += raw_gyro_to_rad_s(raw);
            std::this_thread::sleep_for(std::chrono::milliseconds(period_ms));
        }

        const double denom = samples > 0 ? static_cast<double>(samples) : 1.0;
        gyro_bias_ = accum * (1.0 / denom);
    }

    ImuSample read_sample() const
    {
        const RawAxes gyro_raw = lsm6_.read_gyro_raw();
        const RawAxes accel_raw = lsm6_.read_accel_raw();
        const RawAxes mag_raw = lis3_.read_mag_raw();

        ImuSample sample;
        sample.gyro_rad_s = raw_gyro_to_rad_s(gyro_raw) - gyro_bias_;
        sample.accel_g = raw_accel_to_g(accel_raw);
        sample.mag_gauss = raw_mag_to_gauss(mag_raw);
        sample.captured_at = std::chrono::steady_clock::now();
        return sample;
    }

private:
    static Vec3 raw_gyro_to_rad_s(const RawAxes & raw)
    {
        constexpr double kScale = 0.07 * (kPi / 180.0);
        return {raw.x * kScale, raw.y * kScale, raw.z * kScale};
    }

    static Vec3 raw_accel_to_g(const RawAxes & raw)
    {
        constexpr double kScale = 0.000244;
        return {raw.x * kScale, raw.y * kScale, raw.z * kScale};
    }

    static Vec3 raw_mag_to_gauss(const RawAxes & raw)
    {
        constexpr double kScale = 1.0 / 6842.0;
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
    explicit MadgwickFilter(double beta)
        : beta_(beta)
    {
    }

    void reset()
    {
        q_ = {};
    }

    void update(double dt, const Vec3 & gyro, const Vec3 & accel, const Vec3 & mag)
    {
        if (dt <= 0.0)
        {
            return;
        }

        const double gx = gyro.x;
        const double gy = gyro.y;
        const double gz = gyro.z;
        const double ax = accel.x;
        const double ay = accel.y;
        const double az = accel.z;
        const double mx = mag.x;
        const double my = mag.y;
        const double mz = mag.z;

        if ((mx == 0.0) && (my == 0.0) && (mz == 0.0))
        {
            update_imu(dt, gyro, accel);
            return;
        }

        double q0 = q_.w;
        double q1 = q_.x;
        double q2 = q_.y;
        double q3 = q_.z;

        double q_dot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz);
        double q_dot2 = 0.5 * ( q0 * gx + q2 * gz - q3 * gy);
        double q_dot3 = 0.5 * ( q0 * gy - q1 * gz + q3 * gx);
        double q_dot4 = 0.5 * ( q0 * gz + q1 * gy - q2 * gx);

        if (!((ax == 0.0) && (ay == 0.0) && (az == 0.0)))
        {
            double norm = inv_sqrt(ax * ax + ay * ay + az * az);
            const double axn = ax * norm;
            const double ayn = ay * norm;
            const double azn = az * norm;

            norm = inv_sqrt(mx * mx + my * my + mz * mz);
            const double mxn = mx * norm;
            const double myn = my * norm;
            const double mzn = mz * norm;

            const double _2q0mx = 2.0 * q0 * mxn;
            const double _2q0my = 2.0 * q0 * myn;
            const double _2q0mz = 2.0 * q0 * mzn;
            const double _2q1mx = 2.0 * q1 * mxn;
            const double _2q0 = 2.0 * q0;
            const double _2q1 = 2.0 * q1;
            const double _2q2 = 2.0 * q2;
            const double _2q3 = 2.0 * q3;
            const double _2q0q2 = 2.0 * q0 * q2;
            const double _2q2q3 = 2.0 * q2 * q3;
            const double q0q0 = q0 * q0;
            const double q0q1 = q0 * q1;
            const double q0q2 = q0 * q2;
            const double q0q3 = q0 * q3;
            const double q1q1 = q1 * q1;
            const double q1q2 = q1 * q2;
            const double q1q3 = q1 * q3;
            const double q2q2 = q2 * q2;
            const double q2q3 = q2 * q3;
            const double q3q3 = q3 * q3;

            const double hx = mxn * q0q0 - _2q0my * q3 + _2q0mz * q2
                + mxn * q1q1 + _2q1 * myn * q2 + _2q1 * mzn * q3
                - mxn * q2q2 - mxn * q3q3;
            const double hy = _2q0mx * q3 + myn * q0q0 - _2q0mz * q1
                + _2q1mx * q2 - myn * q1q1 + myn * q2q2
                + _2q2 * mzn * q3 - myn * q3q3;
            const double _2bx = std::sqrt(hx * hx + hy * hy);
            const double _2bz = -_2q0mx * q2 + _2q0my * q1 + mzn * q0q0
                + _2q1mx * q3 - mzn * q1q1 + _2q2 * myn * q3
                - mzn * q2q2 + mzn * q3q3;
            const double _4bx = 2.0 * _2bx;
            const double _4bz = 2.0 * _2bz;

            double s0 = -_2q2 * (2.0 * q1q3 - _2q0q2 - axn)
                + _2q1 * (2.0 * q0q1 + _2q2q3 - ayn)
                - _2bz * q2 * (_2bx * (0.5 - q2q2 - q3q3)
                + _2bz * (q1q3 - q0q2) - mxn)
                + (-_2bx * q3 + _2bz * q1)
                * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - myn)
                + _2bx * q2 * (_2bx * (q0q2 + q1q3)
                + _2bz * (0.5 - q1q1 - q2q2) - mzn);
            double s1 = _2q3 * (2.0 * q1q3 - _2q0q2 - axn)
                + _2q0 * (2.0 * q0q1 + _2q2q3 - ayn)
                - 4.0 * q1 * (1.0 - 2.0 * q1q1 - 2.0 * q2q2 - azn)
                + _2bz * q3 * (_2bx * (0.5 - q2q2 - q3q3)
                + _2bz * (q1q3 - q0q2) - mxn)
                + (_2bx * q2 + _2bz * q0)
                * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - myn)
                + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3)
                + _2bz * (0.5 - q1q1 - q2q2) - mzn);
            double s2 = -_2q0 * (2.0 * q1q3 - _2q0q2 - axn)
                + _2q3 * (2.0 * q0q1 + _2q2q3 - ayn)
                - 4.0 * q2 * (1.0 - 2.0 * q1q1 - 2.0 * q2q2 - azn)
                + (-_4bx * q2 - _2bz * q0)
                * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mxn)
                + (_2bx * q1 + _2bz * q3)
                * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - myn)
                + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3)
                + _2bz * (0.5 - q1q1 - q2q2) - mzn);
            double s3 = _2q1 * (2.0 * q1q3 - _2q0q2 - axn)
                + _2q2 * (2.0 * q0q1 + _2q2q3 - ayn)
                + (-_4bx * q3 + _2bz * q1)
                * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mxn)
                + (-_2bx * q0 + _2bz * q2)
                * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - myn)
                + _2bx * q1 * (_2bx * (q0q2 + q1q3)
                + _2bz * (0.5 - q1q1 - q2q2) - mzn);

            norm = inv_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
            if (norm > 0.0)
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
    void update_imu(double dt, const Vec3 & gyro, const Vec3 & accel)
    {
        const double gx = gyro.x;
        const double gy = gyro.y;
        const double gz = gyro.z;
        const double ax = accel.x;
        const double ay = accel.y;
        const double az = accel.z;

        double q0 = q_.w;
        double q1 = q_.x;
        double q2 = q_.y;
        double q3 = q_.z;

        double q_dot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz);
        double q_dot2 = 0.5 * ( q0 * gx + q2 * gz - q3 * gy);
        double q_dot3 = 0.5 * ( q0 * gy - q1 * gz + q3 * gx);
        double q_dot4 = 0.5 * ( q0 * gz + q1 * gy - q2 * gx);

        if (!((ax == 0.0) && (ay == 0.0) && (az == 0.0)))
        {
            double norm = inv_sqrt(ax * ax + ay * ay + az * az);
            const double axn = ax * norm;
            const double ayn = ay * norm;
            const double azn = az * norm;

            const double _2q0 = 2.0 * q0;
            const double _2q1 = 2.0 * q1;
            const double _2q2 = 2.0 * q2;
            const double _2q3 = 2.0 * q3;
            const double _4q0 = 4.0 * q0;
            const double _4q1 = 4.0 * q1;
            const double _4q2 = 4.0 * q2;
            const double _8q1 = 8.0 * q1;
            const double _8q2 = 8.0 * q2;
            const double q0q0 = q0 * q0;
            const double q1q1 = q1 * q1;
            const double q2q2 = q2 * q2;
            const double q3q3 = q3 * q3;

            double s0 = _4q0 * q2q2 + _2q2 * axn + _4q0 * q1q1 - _2q1 * ayn;
            double s1 = _4q1 * q3q3 - _2q3 * axn + 4.0 * q0q0 * q1 - _2q0 * ayn
                - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * azn;
            double s2 = 4.0 * q0q0 * q2 + _2q0 * axn + _4q2 * q3q3 - _2q3 * ayn
                - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * azn;
            double s3 = 4.0 * q1q1 * q3 - _2q1 * axn + 4.0 * q2q2 * q3 - _2q2 * ayn;

            norm = inv_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
            if (norm > 0.0)
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

    double beta_;
    Quaternion q_{};
};

inline std::string format_local_timestamp()
{
    const auto now = std::chrono::system_clock::now();
    const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm local_tm{};
    localtime_r(&now_time, &local_tm);

    std::ostringstream out;
    out << std::put_time(&local_tm, "%Y-%m-%d %H:%M:%S");
    return out.str();
}

inline std::string format_quaternion_line(const Quaternion & q)
{
    std::ostringstream out;
    out << std::fixed << std::setprecision(6)
        << q.w << ' ' << q.x << ' ' << q.y << ' ' << q.z << '\n';
    return out.str();
}

inline bool write_quaternion_file(const std::string & path, const Quaternion & q, std::uint64_t sequence, std::string * error = nullptr)
{
    const std::string temp_path = path + ".tmp";
    std::ofstream out(temp_path, std::ios::trunc);
    if (!out)
    {
        if (error != nullptr)
        {
            *error = "Failed to open " + temp_path + ": " + std::strerror(errno);
        }
        return false;
    }

    out << std::fixed << std::setprecision(6)
        << q.w << ' ' << q.x << ' ' << q.y << ' ' << q.z << '\n'
        << format_local_timestamp() << '\n'
        << sequence << '\n';
    out.close();

    if (std::rename(temp_path.c_str(), path.c_str()) != 0)
    {
        if (error != nullptr)
        {
            *error = "Failed to rename " + temp_path + " to " + path + ": " + std::strerror(errno);
        }
        return false;
    }

    return true;
}

inline bool set_nonblocking(int fd)
{
    const int flags = fcntl(fd, F_GETFL, 0);
    if (flags < 0)
    {
        return false;
    }

    return fcntl(fd, F_SETFL, flags | O_NONBLOCK) == 0;
}

} // namespace proj3
