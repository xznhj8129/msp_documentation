#pragma once
#include <cstdint>
#include <cstring>
#include <array>
#include <string>
#include <vector>
#include <variant>
#include <stdexcept>
#include <type_traits>

// Pull official MSP command macros and constants
#include "msp_protocol.h"

namespace msp {

enum class Direction : std::uint8_t { Request = 0, Reply = 1 };

struct BufferReader {
    const std::uint8_t* p;
    const std::uint8_t* end;
    explicit BufferReader(const std::vector<std::uint8_t>& buf) : p(buf.data()), end(buf.data()+buf.size()) {}
    explicit BufferReader(const std::uint8_t* data, std::size_t len) : p(data), end(data+len) {}

    template <typename T>
    T read_le() {
        static_assert(std::is_integral<T>::value, "read_le requires integral type");
        if (end - p < (ptrdiff_t)sizeof(T)) throw std::runtime_error("underflow");
        T v = 0;
        for (std::size_t i=0;i<sizeof(T);++i) v |= (T)p[i] << (8*i);
        p += sizeof(T);
        return v;
    }
    void read_bytes(std::uint8_t* dst, std::size_t n) {
        if (end - p < (ptrdiff_t)n) throw std::runtime_error("underflow");
        std::memcpy(dst, p, n);
        p += n;
    }
    std::string read_string_rest() {
        std::string s;
        s.resize(end - p);
        std::memcpy(s.data(), p, s.size());
        p = end;
        return s;
    }
    std::size_t remaining() const { return (std::size_t)(end - p); }
};

struct BufferWriter {
    std::vector<std::uint8_t> buf;
    template <typename T>
    void write_le(T v) {
        static_assert(std::is_integral<T>::value, "write_le requires integral type");
        for (std::size_t i=0;i<sizeof(T);++i) buf.push_back((std::uint8_t)((v >> (8*i)) & 0xFF));
    }
    void write_bytes(const void* src, std::size_t n) {
        auto p = static_cast<const std::uint8_t*>(src);
        buf.insert(buf.end(), p, p+n);
    }
    void write_string_bytes(const std::string& s) {
        buf.insert(buf.end(), (const std::uint8_t*)s.data(), (const std::uint8_t*)s.data()+s.size());
    }
};


struct MSP_API_VERSION__reply {
    std::uint8_t mspProtocolVersion;
    std::uint8_t apiVersionMajor;
    std::uint8_t apiVersionMinor;

    static std::vector<std::uint8_t> pack(const MSP_API_VERSION__reply& v) {
        BufferWriter w;
        w.write_le(v.mspProtocolVersion);
        w.write_le(v.apiVersionMajor);
        w.write_le(v.apiVersionMinor);
        return std::move(w.buf);
    }

    static MSP_API_VERSION__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_API_VERSION__reply v{};
        v.mspProtocolVersion = r.read_le<std::uint8_t>();
        v.apiVersionMajor = r.read_le<std::uint8_t>();
        v.apiVersionMinor = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP_FC_VARIANT__reply {
    std::array<char,4> fcVariantIdentifier;

    static std::vector<std::uint8_t> pack(const MSP_FC_VARIANT__reply& v) {
        BufferWriter w;
        w.write_bytes(v.fcVariantIdentifier.data(), 4);
        return std::move(w.buf);
    }

    static MSP_FC_VARIANT__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_FC_VARIANT__reply v{};
        r.read_bytes(reinterpret_cast<std::uint8_t*>(v.fcVariantIdentifier.data()), 4);
        return v;
    }
};

struct MSP_FC_VERSION__reply {
    std::uint8_t fcVersionMajor;
    std::uint8_t fcVersionMinor;
    std::uint8_t fcVersionPatch;

    static std::vector<std::uint8_t> pack(const MSP_FC_VERSION__reply& v) {
        BufferWriter w;
        w.write_le(v.fcVersionMajor);
        w.write_le(v.fcVersionMinor);
        w.write_le(v.fcVersionPatch);
        return std::move(w.buf);
    }

    static MSP_FC_VERSION__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_FC_VERSION__reply v{};
        v.fcVersionMajor = r.read_le<std::uint8_t>();
        v.fcVersionMinor = r.read_le<std::uint8_t>();
        v.fcVersionPatch = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP_BOARD_INFO__reply {
    std::array<char,4> boardIdentifier;
    std::uint16_t hardwareRevision;
    std::uint8_t osdSupport;
    std::uint8_t commCapabilities;
    std::uint8_t targetNameLength;
    std::string targetName;

    static std::vector<std::uint8_t> pack(const MSP_BOARD_INFO__reply& v) {
        BufferWriter w;
        w.write_bytes(v.boardIdentifier.data(), 4);
        w.write_le(v.hardwareRevision);
        w.write_le(v.osdSupport);
        w.write_le(v.commCapabilities);
        w.write_le(v.targetNameLength);
        w.write_string_bytes(v.targetName);
        return std::move(w.buf);
    }

    static MSP_BOARD_INFO__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_BOARD_INFO__reply v{};
        r.read_bytes(reinterpret_cast<std::uint8_t*>(v.boardIdentifier.data()), 4);
        v.hardwareRevision = r.read_le<std::uint16_t>();
        v.osdSupport = r.read_le<std::uint8_t>();
        v.commCapabilities = r.read_le<std::uint8_t>();
        v.targetNameLength = r.read_le<std::uint8_t>();
        v.targetName = r.read_string_rest();
        return v;
    }
};

struct MSP_BUILD_INFO__reply {
    std::array<char,11> buildDate;
    std::array<char,8> buildTime;
    std::array<char,7> gitRevision;

    static std::vector<std::uint8_t> pack(const MSP_BUILD_INFO__reply& v) {
        BufferWriter w;
        w.write_bytes(v.buildDate.data(), 11);
        w.write_bytes(v.buildTime.data(), 8);
        w.write_bytes(v.gitRevision.data(), 7);
        return std::move(w.buf);
    }

    static MSP_BUILD_INFO__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_BUILD_INFO__reply v{};
        r.read_bytes(reinterpret_cast<std::uint8_t*>(v.buildDate.data()), 11);
        r.read_bytes(reinterpret_cast<std::uint8_t*>(v.buildTime.data()), 8);
        r.read_bytes(reinterpret_cast<std::uint8_t*>(v.gitRevision.data()), 7);
        return v;
    }
};

struct MSP_INAV_PID__reply {
    std::uint8_t legacyAsyncProcessing;
    std::uint16_t legacyAsyncValue1;
    std::uint16_t legacyAsyncValue2;
    std::uint8_t headingHoldRateLimit;
    std::uint8_t headingHoldLpfFreq;
    std::uint16_t legacyYawJumpLimit;
    std::uint8_t legacyGyroLpf;
    std::uint8_t accLpfHz;
    std::uint8_t reserved1;
    std::uint8_t reserved2;
    std::uint8_t reserved3;
    std::uint8_t reserved4;

    static std::vector<std::uint8_t> pack(const MSP_INAV_PID__reply& v) {
        BufferWriter w;
        w.write_le(v.legacyAsyncProcessing);
        w.write_le(v.legacyAsyncValue1);
        w.write_le(v.legacyAsyncValue2);
        w.write_le(v.headingHoldRateLimit);
        w.write_le(v.headingHoldLpfFreq);
        w.write_le(v.legacyYawJumpLimit);
        w.write_le(v.legacyGyroLpf);
        w.write_le(v.accLpfHz);
        w.write_le(v.reserved1);
        w.write_le(v.reserved2);
        w.write_le(v.reserved3);
        w.write_le(v.reserved4);
        return std::move(w.buf);
    }

    static MSP_INAV_PID__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_INAV_PID__reply v{};
        v.legacyAsyncProcessing = r.read_le<std::uint8_t>();
        v.legacyAsyncValue1 = r.read_le<std::uint16_t>();
        v.legacyAsyncValue2 = r.read_le<std::uint16_t>();
        v.headingHoldRateLimit = r.read_le<std::uint8_t>();
        v.headingHoldLpfFreq = r.read_le<std::uint8_t>();
        v.legacyYawJumpLimit = r.read_le<std::uint16_t>();
        v.legacyGyroLpf = r.read_le<std::uint8_t>();
        v.accLpfHz = r.read_le<std::uint8_t>();
        v.reserved1 = r.read_le<std::uint8_t>();
        v.reserved2 = r.read_le<std::uint8_t>();
        v.reserved3 = r.read_le<std::uint8_t>();
        v.reserved4 = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP_SET_INAV_PID__request {
    std::uint8_t legacyAsyncProcessing;
    std::uint16_t legacyAsyncValue1;
    std::uint16_t legacyAsyncValue2;
    std::uint8_t headingHoldRateLimit;
    std::uint8_t headingHoldLpfFreq;
    std::uint16_t legacyYawJumpLimit;
    std::uint8_t legacyGyroLpf;
    std::uint8_t accLpfHz;
    std::uint8_t reserved1;
    std::uint8_t reserved2;
    std::uint8_t reserved3;
    std::uint8_t reserved4;

    static std::vector<std::uint8_t> pack(const MSP_SET_INAV_PID__request& v) {
        BufferWriter w;
        w.write_le(v.legacyAsyncProcessing);
        w.write_le(v.legacyAsyncValue1);
        w.write_le(v.legacyAsyncValue2);
        w.write_le(v.headingHoldRateLimit);
        w.write_le(v.headingHoldLpfFreq);
        w.write_le(v.legacyYawJumpLimit);
        w.write_le(v.legacyGyroLpf);
        w.write_le(v.accLpfHz);
        w.write_le(v.reserved1);
        w.write_le(v.reserved2);
        w.write_le(v.reserved3);
        w.write_le(v.reserved4);
        return std::move(w.buf);
    }

    static MSP_SET_INAV_PID__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_SET_INAV_PID__request v{};
        v.legacyAsyncProcessing = r.read_le<std::uint8_t>();
        v.legacyAsyncValue1 = r.read_le<std::uint16_t>();
        v.legacyAsyncValue2 = r.read_le<std::uint16_t>();
        v.headingHoldRateLimit = r.read_le<std::uint8_t>();
        v.headingHoldLpfFreq = r.read_le<std::uint8_t>();
        v.legacyYawJumpLimit = r.read_le<std::uint16_t>();
        v.legacyGyroLpf = r.read_le<std::uint8_t>();
        v.accLpfHz = r.read_le<std::uint8_t>();
        v.reserved1 = r.read_le<std::uint8_t>();
        v.reserved2 = r.read_le<std::uint8_t>();
        v.reserved3 = r.read_le<std::uint8_t>();
        v.reserved4 = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP_NAME__reply {
    std::string craftName;

    static std::vector<std::uint8_t> pack(const MSP_NAME__reply& v) {
        BufferWriter w;
        w.write_string_bytes(v.craftName);
        return std::move(w.buf);
    }

    static MSP_NAME__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_NAME__reply v{};
        v.craftName = r.read_string_rest();
        return v;
    }
};

struct MSP_SET_NAME__request {
    std::string craftName;

    static std::vector<std::uint8_t> pack(const MSP_SET_NAME__request& v) {
        BufferWriter w;
        w.write_string_bytes(v.craftName);
        return std::move(w.buf);
    }

    static MSP_SET_NAME__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_SET_NAME__request v{};
        v.craftName = r.read_string_rest();
        return v;
    }
};

struct MSP_NAV_POSHOLD__reply {
    std::uint8_t userControlMode;
    std::uint16_t maxAutoSpeed;
    std::uint16_t maxAutoClimbRate;
    std::uint16_t maxManualSpeed;
    std::uint16_t maxManualClimbRate;
    std::uint8_t mcMaxBankAngle;
    std::uint8_t mcAltHoldThrottleType;
    std::uint16_t mcHoverThrottle;

    static std::vector<std::uint8_t> pack(const MSP_NAV_POSHOLD__reply& v) {
        BufferWriter w;
        w.write_le(v.userControlMode);
        w.write_le(v.maxAutoSpeed);
        w.write_le(v.maxAutoClimbRate);
        w.write_le(v.maxManualSpeed);
        w.write_le(v.maxManualClimbRate);
        w.write_le(v.mcMaxBankAngle);
        w.write_le(v.mcAltHoldThrottleType);
        w.write_le(v.mcHoverThrottle);
        return std::move(w.buf);
    }

    static MSP_NAV_POSHOLD__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_NAV_POSHOLD__reply v{};
        v.userControlMode = r.read_le<std::uint8_t>();
        v.maxAutoSpeed = r.read_le<std::uint16_t>();
        v.maxAutoClimbRate = r.read_le<std::uint16_t>();
        v.maxManualSpeed = r.read_le<std::uint16_t>();
        v.maxManualClimbRate = r.read_le<std::uint16_t>();
        v.mcMaxBankAngle = r.read_le<std::uint8_t>();
        v.mcAltHoldThrottleType = r.read_le<std::uint8_t>();
        v.mcHoverThrottle = r.read_le<std::uint16_t>();
        return v;
    }
};

struct MSP_SET_NAV_POSHOLD__request {
    std::uint8_t userControlMode;
    std::uint16_t maxAutoSpeed;
    std::uint16_t maxAutoClimbRate;
    std::uint16_t maxManualSpeed;
    std::uint16_t maxManualClimbRate;
    std::uint8_t mcMaxBankAngle;
    std::uint8_t mcAltHoldThrottleType;
    std::uint16_t mcHoverThrottle;

    static std::vector<std::uint8_t> pack(const MSP_SET_NAV_POSHOLD__request& v) {
        BufferWriter w;
        w.write_le(v.userControlMode);
        w.write_le(v.maxAutoSpeed);
        w.write_le(v.maxAutoClimbRate);
        w.write_le(v.maxManualSpeed);
        w.write_le(v.maxManualClimbRate);
        w.write_le(v.mcMaxBankAngle);
        w.write_le(v.mcAltHoldThrottleType);
        w.write_le(v.mcHoverThrottle);
        return std::move(w.buf);
    }

    static MSP_SET_NAV_POSHOLD__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_SET_NAV_POSHOLD__request v{};
        v.userControlMode = r.read_le<std::uint8_t>();
        v.maxAutoSpeed = r.read_le<std::uint16_t>();
        v.maxAutoClimbRate = r.read_le<std::uint16_t>();
        v.maxManualSpeed = r.read_le<std::uint16_t>();
        v.maxManualClimbRate = r.read_le<std::uint16_t>();
        v.mcMaxBankAngle = r.read_le<std::uint8_t>();
        v.mcAltHoldThrottleType = r.read_le<std::uint8_t>();
        v.mcHoverThrottle = r.read_le<std::uint16_t>();
        return v;
    }
};

struct MSP_CALIBRATION_DATA__reply {
    std::uint8_t accCalibAxisFlags;
    std::uint16_t accZeroX;
    std::uint16_t accZeroY;
    std::uint16_t accZeroZ;
    std::uint16_t accGainX;
    std::uint16_t accGainY;
    std::uint16_t accGainZ;
    std::uint16_t magZeroX;
    std::uint16_t magZeroY;
    std::uint16_t magZeroZ;
    std::uint16_t opflowScale;
    std::uint16_t magGainX;
    std::uint16_t magGainY;
    std::uint16_t magGainZ;

    static std::vector<std::uint8_t> pack(const MSP_CALIBRATION_DATA__reply& v) {
        BufferWriter w;
        w.write_le(v.accCalibAxisFlags);
        w.write_le(v.accZeroX);
        w.write_le(v.accZeroY);
        w.write_le(v.accZeroZ);
        w.write_le(v.accGainX);
        w.write_le(v.accGainY);
        w.write_le(v.accGainZ);
        w.write_le(v.magZeroX);
        w.write_le(v.magZeroY);
        w.write_le(v.magZeroZ);
        w.write_le(v.opflowScale);
        w.write_le(v.magGainX);
        w.write_le(v.magGainY);
        w.write_le(v.magGainZ);
        return std::move(w.buf);
    }

    static MSP_CALIBRATION_DATA__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_CALIBRATION_DATA__reply v{};
        v.accCalibAxisFlags = r.read_le<std::uint8_t>();
        v.accZeroX = r.read_le<std::uint16_t>();
        v.accZeroY = r.read_le<std::uint16_t>();
        v.accZeroZ = r.read_le<std::uint16_t>();
        v.accGainX = r.read_le<std::uint16_t>();
        v.accGainY = r.read_le<std::uint16_t>();
        v.accGainZ = r.read_le<std::uint16_t>();
        v.magZeroX = r.read_le<std::uint16_t>();
        v.magZeroY = r.read_le<std::uint16_t>();
        v.magZeroZ = r.read_le<std::uint16_t>();
        v.opflowScale = r.read_le<std::uint16_t>();
        v.magGainX = r.read_le<std::uint16_t>();
        v.magGainY = r.read_le<std::uint16_t>();
        v.magGainZ = r.read_le<std::uint16_t>();
        return v;
    }
};

struct MSP_SET_CALIBRATION_DATA__request {
    std::uint16_t accZeroX;
    std::uint16_t accZeroY;
    std::uint16_t accZeroZ;
    std::uint16_t accGainX;
    std::uint16_t accGainY;
    std::uint16_t accGainZ;
    std::uint16_t magZeroX;
    std::uint16_t magZeroY;
    std::uint16_t magZeroZ;
    std::uint16_t opflowScale;
    std::uint16_t magGainX;
    std::uint16_t magGainY;
    std::uint16_t magGainZ;

    static std::vector<std::uint8_t> pack(const MSP_SET_CALIBRATION_DATA__request& v) {
        BufferWriter w;
        w.write_le(v.accZeroX);
        w.write_le(v.accZeroY);
        w.write_le(v.accZeroZ);
        w.write_le(v.accGainX);
        w.write_le(v.accGainY);
        w.write_le(v.accGainZ);
        w.write_le(v.magZeroX);
        w.write_le(v.magZeroY);
        w.write_le(v.magZeroZ);
        w.write_le(v.opflowScale);
        w.write_le(v.magGainX);
        w.write_le(v.magGainY);
        w.write_le(v.magGainZ);
        return std::move(w.buf);
    }

    static MSP_SET_CALIBRATION_DATA__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_SET_CALIBRATION_DATA__request v{};
        v.accZeroX = r.read_le<std::uint16_t>();
        v.accZeroY = r.read_le<std::uint16_t>();
        v.accZeroZ = r.read_le<std::uint16_t>();
        v.accGainX = r.read_le<std::uint16_t>();
        v.accGainY = r.read_le<std::uint16_t>();
        v.accGainZ = r.read_le<std::uint16_t>();
        v.magZeroX = r.read_le<std::uint16_t>();
        v.magZeroY = r.read_le<std::uint16_t>();
        v.magZeroZ = r.read_le<std::uint16_t>();
        v.opflowScale = r.read_le<std::uint16_t>();
        v.magGainX = r.read_le<std::uint16_t>();
        v.magGainY = r.read_le<std::uint16_t>();
        v.magGainZ = r.read_le<std::uint16_t>();
        return v;
    }
};

struct MSP_POSITION_ESTIMATION_CONFIG__reply {
    std::uint16_t weightZBaroP;
    std::uint16_t weightZGPSP;
    std::uint16_t weightZGPSV;
    std::uint16_t weightXYGPSP;
    std::uint16_t weightXYGPSV;
    std::uint8_t minSats;
    std::uint8_t useGPSVelNED;

    static std::vector<std::uint8_t> pack(const MSP_POSITION_ESTIMATION_CONFIG__reply& v) {
        BufferWriter w;
        w.write_le(v.weightZBaroP);
        w.write_le(v.weightZGPSP);
        w.write_le(v.weightZGPSV);
        w.write_le(v.weightXYGPSP);
        w.write_le(v.weightXYGPSV);
        w.write_le(v.minSats);
        w.write_le(v.useGPSVelNED);
        return std::move(w.buf);
    }

    static MSP_POSITION_ESTIMATION_CONFIG__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_POSITION_ESTIMATION_CONFIG__reply v{};
        v.weightZBaroP = r.read_le<std::uint16_t>();
        v.weightZGPSP = r.read_le<std::uint16_t>();
        v.weightZGPSV = r.read_le<std::uint16_t>();
        v.weightXYGPSP = r.read_le<std::uint16_t>();
        v.weightXYGPSV = r.read_le<std::uint16_t>();
        v.minSats = r.read_le<std::uint8_t>();
        v.useGPSVelNED = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP_SET_POSITION_ESTIMATION_CONFIG__request {
    std::uint16_t weightZBaroP;
    std::uint16_t weightZGPSP;
    std::uint16_t weightZGPSV;
    std::uint16_t weightXYGPSP;
    std::uint16_t weightXYGPSV;
    std::uint8_t minSats;
    std::uint8_t useGPSVelNED;

    static std::vector<std::uint8_t> pack(const MSP_SET_POSITION_ESTIMATION_CONFIG__request& v) {
        BufferWriter w;
        w.write_le(v.weightZBaroP);
        w.write_le(v.weightZGPSP);
        w.write_le(v.weightZGPSV);
        w.write_le(v.weightXYGPSP);
        w.write_le(v.weightXYGPSV);
        w.write_le(v.minSats);
        w.write_le(v.useGPSVelNED);
        return std::move(w.buf);
    }

    static MSP_SET_POSITION_ESTIMATION_CONFIG__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_SET_POSITION_ESTIMATION_CONFIG__request v{};
        v.weightZBaroP = r.read_le<std::uint16_t>();
        v.weightZGPSP = r.read_le<std::uint16_t>();
        v.weightZGPSV = r.read_le<std::uint16_t>();
        v.weightXYGPSP = r.read_le<std::uint16_t>();
        v.weightXYGPSV = r.read_le<std::uint16_t>();
        v.minSats = r.read_le<std::uint8_t>();
        v.useGPSVelNED = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP_WP_MISSION_LOAD__request {
    std::uint8_t missionID;

    static std::vector<std::uint8_t> pack(const MSP_WP_MISSION_LOAD__request& v) {
        BufferWriter w;
        w.write_le(v.missionID);
        return std::move(w.buf);
    }

    static MSP_WP_MISSION_LOAD__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_WP_MISSION_LOAD__request v{};
        v.missionID = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP_WP_MISSION_SAVE__request {
    std::uint8_t missionID;

    static std::vector<std::uint8_t> pack(const MSP_WP_MISSION_SAVE__request& v) {
        BufferWriter w;
        w.write_le(v.missionID);
        return std::move(w.buf);
    }

    static MSP_WP_MISSION_SAVE__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_WP_MISSION_SAVE__request v{};
        v.missionID = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP_WP_GETINFO__reply {
    std::uint8_t wpCapabilities;
    std::uint8_t maxWaypoints;
    std::uint8_t missionValid;
    std::uint8_t waypointCount;

    static std::vector<std::uint8_t> pack(const MSP_WP_GETINFO__reply& v) {
        BufferWriter w;
        w.write_le(v.wpCapabilities);
        w.write_le(v.maxWaypoints);
        w.write_le(v.missionValid);
        w.write_le(v.waypointCount);
        return std::move(w.buf);
    }

    static MSP_WP_GETINFO__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_WP_GETINFO__reply v{};
        v.wpCapabilities = r.read_le<std::uint8_t>();
        v.maxWaypoints = r.read_le<std::uint8_t>();
        v.missionValid = r.read_le<std::uint8_t>();
        v.waypointCount = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP_RTH_AND_LAND_CONFIG__reply {
    std::uint16_t minRthDistance;
    std::uint8_t rthClimbFirst;
    std::uint8_t rthClimbIgnoreEmerg;
    std::uint8_t rthTailFirst;
    std::uint8_t rthAllowLanding;
    std::uint8_t rthAltControlMode;
    std::uint16_t rthAbortThreshold;
    std::uint16_t rthAltitude;
    std::uint16_t landMinAltVspd;
    std::uint16_t landMaxAltVspd;
    std::uint16_t landSlowdownMinAlt;
    std::uint16_t landSlowdownMaxAlt;
    std::uint16_t emergDescentRate;

    static std::vector<std::uint8_t> pack(const MSP_RTH_AND_LAND_CONFIG__reply& v) {
        BufferWriter w;
        w.write_le(v.minRthDistance);
        w.write_le(v.rthClimbFirst);
        w.write_le(v.rthClimbIgnoreEmerg);
        w.write_le(v.rthTailFirst);
        w.write_le(v.rthAllowLanding);
        w.write_le(v.rthAltControlMode);
        w.write_le(v.rthAbortThreshold);
        w.write_le(v.rthAltitude);
        w.write_le(v.landMinAltVspd);
        w.write_le(v.landMaxAltVspd);
        w.write_le(v.landSlowdownMinAlt);
        w.write_le(v.landSlowdownMaxAlt);
        w.write_le(v.emergDescentRate);
        return std::move(w.buf);
    }

    static MSP_RTH_AND_LAND_CONFIG__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_RTH_AND_LAND_CONFIG__reply v{};
        v.minRthDistance = r.read_le<std::uint16_t>();
        v.rthClimbFirst = r.read_le<std::uint8_t>();
        v.rthClimbIgnoreEmerg = r.read_le<std::uint8_t>();
        v.rthTailFirst = r.read_le<std::uint8_t>();
        v.rthAllowLanding = r.read_le<std::uint8_t>();
        v.rthAltControlMode = r.read_le<std::uint8_t>();
        v.rthAbortThreshold = r.read_le<std::uint16_t>();
        v.rthAltitude = r.read_le<std::uint16_t>();
        v.landMinAltVspd = r.read_le<std::uint16_t>();
        v.landMaxAltVspd = r.read_le<std::uint16_t>();
        v.landSlowdownMinAlt = r.read_le<std::uint16_t>();
        v.landSlowdownMaxAlt = r.read_le<std::uint16_t>();
        v.emergDescentRate = r.read_le<std::uint16_t>();
        return v;
    }
};

struct MSP_SET_RTH_AND_LAND_CONFIG__request {
    std::uint16_t minRthDistance;
    std::uint8_t rthClimbFirst;
    std::uint8_t rthClimbIgnoreEmerg;
    std::uint8_t rthTailFirst;
    std::uint8_t rthAllowLanding;
    std::uint8_t rthAltControlMode;
    std::uint16_t rthAbortThreshold;
    std::uint16_t rthAltitude;
    std::uint16_t landMinAltVspd;
    std::uint16_t landMaxAltVspd;
    std::uint16_t landSlowdownMinAlt;
    std::uint16_t landSlowdownMaxAlt;
    std::uint16_t emergDescentRate;

    static std::vector<std::uint8_t> pack(const MSP_SET_RTH_AND_LAND_CONFIG__request& v) {
        BufferWriter w;
        w.write_le(v.minRthDistance);
        w.write_le(v.rthClimbFirst);
        w.write_le(v.rthClimbIgnoreEmerg);
        w.write_le(v.rthTailFirst);
        w.write_le(v.rthAllowLanding);
        w.write_le(v.rthAltControlMode);
        w.write_le(v.rthAbortThreshold);
        w.write_le(v.rthAltitude);
        w.write_le(v.landMinAltVspd);
        w.write_le(v.landMaxAltVspd);
        w.write_le(v.landSlowdownMinAlt);
        w.write_le(v.landSlowdownMaxAlt);
        w.write_le(v.emergDescentRate);
        return std::move(w.buf);
    }

    static MSP_SET_RTH_AND_LAND_CONFIG__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_SET_RTH_AND_LAND_CONFIG__request v{};
        v.minRthDistance = r.read_le<std::uint16_t>();
        v.rthClimbFirst = r.read_le<std::uint8_t>();
        v.rthClimbIgnoreEmerg = r.read_le<std::uint8_t>();
        v.rthTailFirst = r.read_le<std::uint8_t>();
        v.rthAllowLanding = r.read_le<std::uint8_t>();
        v.rthAltControlMode = r.read_le<std::uint8_t>();
        v.rthAbortThreshold = r.read_le<std::uint16_t>();
        v.rthAltitude = r.read_le<std::uint16_t>();
        v.landMinAltVspd = r.read_le<std::uint16_t>();
        v.landMaxAltVspd = r.read_le<std::uint16_t>();
        v.landSlowdownMinAlt = r.read_le<std::uint16_t>();
        v.landSlowdownMaxAlt = r.read_le<std::uint16_t>();
        v.emergDescentRate = r.read_le<std::uint16_t>();
        return v;
    }
};

struct MSP_FW_CONFIG__reply {
    std::uint16_t cruiseThrottle;
    std::uint16_t minThrottle;
    std::uint16_t maxThrottle;
    std::uint8_t maxBankAngle;
    std::uint8_t maxClimbAngle;
    std::uint8_t maxDiveAngle;
    std::uint8_t pitchToThrottle;
    std::uint16_t loiterRadius;

    static std::vector<std::uint8_t> pack(const MSP_FW_CONFIG__reply& v) {
        BufferWriter w;
        w.write_le(v.cruiseThrottle);
        w.write_le(v.minThrottle);
        w.write_le(v.maxThrottle);
        w.write_le(v.maxBankAngle);
        w.write_le(v.maxClimbAngle);
        w.write_le(v.maxDiveAngle);
        w.write_le(v.pitchToThrottle);
        w.write_le(v.loiterRadius);
        return std::move(w.buf);
    }

    static MSP_FW_CONFIG__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_FW_CONFIG__reply v{};
        v.cruiseThrottle = r.read_le<std::uint16_t>();
        v.minThrottle = r.read_le<std::uint16_t>();
        v.maxThrottle = r.read_le<std::uint16_t>();
        v.maxBankAngle = r.read_le<std::uint8_t>();
        v.maxClimbAngle = r.read_le<std::uint8_t>();
        v.maxDiveAngle = r.read_le<std::uint8_t>();
        v.pitchToThrottle = r.read_le<std::uint8_t>();
        v.loiterRadius = r.read_le<std::uint16_t>();
        return v;
    }
};

struct MSP_SET_FW_CONFIG__request {
    std::uint16_t cruiseThrottle;
    std::uint16_t minThrottle;
    std::uint16_t maxThrottle;
    std::uint8_t maxBankAngle;
    std::uint8_t maxClimbAngle;
    std::uint8_t maxDiveAngle;
    std::uint8_t pitchToThrottle;
    std::uint16_t loiterRadius;

    static std::vector<std::uint8_t> pack(const MSP_SET_FW_CONFIG__request& v) {
        BufferWriter w;
        w.write_le(v.cruiseThrottle);
        w.write_le(v.minThrottle);
        w.write_le(v.maxThrottle);
        w.write_le(v.maxBankAngle);
        w.write_le(v.maxClimbAngle);
        w.write_le(v.maxDiveAngle);
        w.write_le(v.pitchToThrottle);
        w.write_le(v.loiterRadius);
        return std::move(w.buf);
    }

    static MSP_SET_FW_CONFIG__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_SET_FW_CONFIG__request v{};
        v.cruiseThrottle = r.read_le<std::uint16_t>();
        v.minThrottle = r.read_le<std::uint16_t>();
        v.maxThrottle = r.read_le<std::uint16_t>();
        v.maxBankAngle = r.read_le<std::uint8_t>();
        v.maxClimbAngle = r.read_le<std::uint8_t>();
        v.maxDiveAngle = r.read_le<std::uint8_t>();
        v.pitchToThrottle = r.read_le<std::uint8_t>();
        v.loiterRadius = r.read_le<std::uint16_t>();
        return v;
    }
};

struct MSP_MODE_RANGES__reply {
    std::uint8_t modePermanentId;
    std::uint8_t auxChannelIndex;
    std::uint8_t rangeStartStep;
    std::uint8_t rangeEndStep;

    static std::vector<std::uint8_t> pack(const MSP_MODE_RANGES__reply& v) {
        BufferWriter w;
        w.write_le(v.modePermanentId);
        w.write_le(v.auxChannelIndex);
        w.write_le(v.rangeStartStep);
        w.write_le(v.rangeEndStep);
        return std::move(w.buf);
    }

    static MSP_MODE_RANGES__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_MODE_RANGES__reply v{};
        v.modePermanentId = r.read_le<std::uint8_t>();
        v.auxChannelIndex = r.read_le<std::uint8_t>();
        v.rangeStartStep = r.read_le<std::uint8_t>();
        v.rangeEndStep = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP_SET_MODE_RANGE__request {
    std::uint8_t rangeIndex;
    std::uint8_t modePermanentId;
    std::uint8_t auxChannelIndex;
    std::uint8_t rangeStartStep;
    std::uint8_t rangeEndStep;

    static std::vector<std::uint8_t> pack(const MSP_SET_MODE_RANGE__request& v) {
        BufferWriter w;
        w.write_le(v.rangeIndex);
        w.write_le(v.modePermanentId);
        w.write_le(v.auxChannelIndex);
        w.write_le(v.rangeStartStep);
        w.write_le(v.rangeEndStep);
        return std::move(w.buf);
    }

    static MSP_SET_MODE_RANGE__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_SET_MODE_RANGE__request v{};
        v.rangeIndex = r.read_le<std::uint8_t>();
        v.modePermanentId = r.read_le<std::uint8_t>();
        v.auxChannelIndex = r.read_le<std::uint8_t>();
        v.rangeStartStep = r.read_le<std::uint8_t>();
        v.rangeEndStep = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP_FEATURE__reply {
    std::uint32_t featureMask;

    static std::vector<std::uint8_t> pack(const MSP_FEATURE__reply& v) {
        BufferWriter w;
        w.write_le(v.featureMask);
        return std::move(w.buf);
    }

    static MSP_FEATURE__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_FEATURE__reply v{};
        v.featureMask = r.read_le<std::uint32_t>();
        return v;
    }
};

struct MSP_SET_FEATURE__request {
    std::uint32_t featureMask;

    static std::vector<std::uint8_t> pack(const MSP_SET_FEATURE__request& v) {
        BufferWriter w;
        w.write_le(v.featureMask);
        return std::move(w.buf);
    }

    static MSP_SET_FEATURE__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_SET_FEATURE__request v{};
        v.featureMask = r.read_le<std::uint32_t>();
        return v;
    }
};

struct MSP_BOARD_ALIGNMENT__reply {
    std::uint16_t rollAlign;
    std::uint16_t pitchAlign;
    std::uint16_t yawAlign;

    static std::vector<std::uint8_t> pack(const MSP_BOARD_ALIGNMENT__reply& v) {
        BufferWriter w;
        w.write_le(v.rollAlign);
        w.write_le(v.pitchAlign);
        w.write_le(v.yawAlign);
        return std::move(w.buf);
    }

    static MSP_BOARD_ALIGNMENT__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_BOARD_ALIGNMENT__reply v{};
        v.rollAlign = r.read_le<std::uint16_t>();
        v.pitchAlign = r.read_le<std::uint16_t>();
        v.yawAlign = r.read_le<std::uint16_t>();
        return v;
    }
};

struct MSP_SET_BOARD_ALIGNMENT__request {
    std::uint16_t rollAlign;
    std::uint16_t pitchAlign;
    std::uint16_t yawAlign;

    static std::vector<std::uint8_t> pack(const MSP_SET_BOARD_ALIGNMENT__request& v) {
        BufferWriter w;
        w.write_le(v.rollAlign);
        w.write_le(v.pitchAlign);
        w.write_le(v.yawAlign);
        return std::move(w.buf);
    }

    static MSP_SET_BOARD_ALIGNMENT__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_SET_BOARD_ALIGNMENT__request v{};
        v.rollAlign = r.read_le<std::uint16_t>();
        v.pitchAlign = r.read_le<std::uint16_t>();
        v.yawAlign = r.read_le<std::uint16_t>();
        return v;
    }
};

struct MSP_CURRENT_METER_CONFIG__reply {
    std::uint16_t scale;
    std::uint16_t offset;
    std::uint8_t type;
    std::uint16_t capacity;

    static std::vector<std::uint8_t> pack(const MSP_CURRENT_METER_CONFIG__reply& v) {
        BufferWriter w;
        w.write_le(v.scale);
        w.write_le(v.offset);
        w.write_le(v.type);
        w.write_le(v.capacity);
        return std::move(w.buf);
    }

    static MSP_CURRENT_METER_CONFIG__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_CURRENT_METER_CONFIG__reply v{};
        v.scale = r.read_le<std::uint16_t>();
        v.offset = r.read_le<std::uint16_t>();
        v.type = r.read_le<std::uint8_t>();
        v.capacity = r.read_le<std::uint16_t>();
        return v;
    }
};

struct MSP_SET_CURRENT_METER_CONFIG__request {
    std::uint16_t scale;
    std::uint16_t offset;
    std::uint8_t type;
    std::uint16_t capacity;

    static std::vector<std::uint8_t> pack(const MSP_SET_CURRENT_METER_CONFIG__request& v) {
        BufferWriter w;
        w.write_le(v.scale);
        w.write_le(v.offset);
        w.write_le(v.type);
        w.write_le(v.capacity);
        return std::move(w.buf);
    }

    static MSP_SET_CURRENT_METER_CONFIG__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_SET_CURRENT_METER_CONFIG__request v{};
        v.scale = r.read_le<std::uint16_t>();
        v.offset = r.read_le<std::uint16_t>();
        v.type = r.read_le<std::uint8_t>();
        v.capacity = r.read_le<std::uint16_t>();
        return v;
    }
};

struct MSP_MIXER__reply {
    std::uint8_t mixerMode;

    static std::vector<std::uint8_t> pack(const MSP_MIXER__reply& v) {
        BufferWriter w;
        w.write_le(v.mixerMode);
        return std::move(w.buf);
    }

    static MSP_MIXER__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_MIXER__reply v{};
        v.mixerMode = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP_SET_MIXER__request {
    std::uint8_t mixerMode;

    static std::vector<std::uint8_t> pack(const MSP_SET_MIXER__request& v) {
        BufferWriter w;
        w.write_le(v.mixerMode);
        return std::move(w.buf);
    }

    static MSP_SET_MIXER__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_SET_MIXER__request v{};
        v.mixerMode = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP_RX_CONFIG__reply {
    std::uint8_t serialRxProvider;
    std::uint16_t maxCheck;
    std::uint16_t midRc;
    std::uint16_t minCheck;
    std::uint8_t spektrumSatBind;
    std::uint16_t rxMinUsec;
    std::uint16_t rxMaxUsec;
    std::uint8_t bfCompatRcInterpolation;
    std::uint8_t bfCompatRcInterpolationInt;
    std::uint16_t bfCompatAirModeThreshold;
    std::uint8_t reserved1;
    std::uint32_t reserved2;
    std::uint8_t reserved3;
    std::uint8_t bfCompatFpvCamAngle;
    std::uint8_t receiverType;

    static std::vector<std::uint8_t> pack(const MSP_RX_CONFIG__reply& v) {
        BufferWriter w;
        w.write_le(v.serialRxProvider);
        w.write_le(v.maxCheck);
        w.write_le(v.midRc);
        w.write_le(v.minCheck);
        w.write_le(v.spektrumSatBind);
        w.write_le(v.rxMinUsec);
        w.write_le(v.rxMaxUsec);
        w.write_le(v.bfCompatRcInterpolation);
        w.write_le(v.bfCompatRcInterpolationInt);
        w.write_le(v.bfCompatAirModeThreshold);
        w.write_le(v.reserved1);
        w.write_le(v.reserved2);
        w.write_le(v.reserved3);
        w.write_le(v.bfCompatFpvCamAngle);
        w.write_le(v.receiverType);
        return std::move(w.buf);
    }

    static MSP_RX_CONFIG__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_RX_CONFIG__reply v{};
        v.serialRxProvider = r.read_le<std::uint8_t>();
        v.maxCheck = r.read_le<std::uint16_t>();
        v.midRc = r.read_le<std::uint16_t>();
        v.minCheck = r.read_le<std::uint16_t>();
        v.spektrumSatBind = r.read_le<std::uint8_t>();
        v.rxMinUsec = r.read_le<std::uint16_t>();
        v.rxMaxUsec = r.read_le<std::uint16_t>();
        v.bfCompatRcInterpolation = r.read_le<std::uint8_t>();
        v.bfCompatRcInterpolationInt = r.read_le<std::uint8_t>();
        v.bfCompatAirModeThreshold = r.read_le<std::uint16_t>();
        v.reserved1 = r.read_le<std::uint8_t>();
        v.reserved2 = r.read_le<std::uint32_t>();
        v.reserved3 = r.read_le<std::uint8_t>();
        v.bfCompatFpvCamAngle = r.read_le<std::uint8_t>();
        v.receiverType = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP_SET_RX_CONFIG__request {
    std::uint8_t serialRxProvider;
    std::uint16_t maxCheck;
    std::uint16_t midRc;
    std::uint16_t minCheck;
    std::uint8_t spektrumSatBind;
    std::uint16_t rxMinUsec;
    std::uint16_t rxMaxUsec;
    std::uint8_t bfCompatRcInterpolation;
    std::uint8_t bfCompatRcInterpolationInt;
    std::uint16_t bfCompatAirModeThreshold;
    std::uint8_t reserved1;
    std::uint32_t reserved2;
    std::uint8_t reserved3;
    std::uint8_t bfCompatFpvCamAngle;
    std::uint8_t receiverType;

    static std::vector<std::uint8_t> pack(const MSP_SET_RX_CONFIG__request& v) {
        BufferWriter w;
        w.write_le(v.serialRxProvider);
        w.write_le(v.maxCheck);
        w.write_le(v.midRc);
        w.write_le(v.minCheck);
        w.write_le(v.spektrumSatBind);
        w.write_le(v.rxMinUsec);
        w.write_le(v.rxMaxUsec);
        w.write_le(v.bfCompatRcInterpolation);
        w.write_le(v.bfCompatRcInterpolationInt);
        w.write_le(v.bfCompatAirModeThreshold);
        w.write_le(v.reserved1);
        w.write_le(v.reserved2);
        w.write_le(v.reserved3);
        w.write_le(v.bfCompatFpvCamAngle);
        w.write_le(v.receiverType);
        return std::move(w.buf);
    }

    static MSP_SET_RX_CONFIG__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_SET_RX_CONFIG__request v{};
        v.serialRxProvider = r.read_le<std::uint8_t>();
        v.maxCheck = r.read_le<std::uint16_t>();
        v.midRc = r.read_le<std::uint16_t>();
        v.minCheck = r.read_le<std::uint16_t>();
        v.spektrumSatBind = r.read_le<std::uint8_t>();
        v.rxMinUsec = r.read_le<std::uint16_t>();
        v.rxMaxUsec = r.read_le<std::uint16_t>();
        v.bfCompatRcInterpolation = r.read_le<std::uint8_t>();
        v.bfCompatRcInterpolationInt = r.read_le<std::uint8_t>();
        v.bfCompatAirModeThreshold = r.read_le<std::uint16_t>();
        v.reserved1 = r.read_le<std::uint8_t>();
        v.reserved2 = r.read_le<std::uint32_t>();
        v.reserved3 = r.read_le<std::uint8_t>();
        v.bfCompatFpvCamAngle = r.read_le<std::uint8_t>();
        v.receiverType = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP_LED_COLORS__reply {
    std::uint16_t hue;
    std::uint8_t saturation;
    std::uint8_t value;

    static std::vector<std::uint8_t> pack(const MSP_LED_COLORS__reply& v) {
        BufferWriter w;
        w.write_le(v.hue);
        w.write_le(v.saturation);
        w.write_le(v.value);
        return std::move(w.buf);
    }

    static MSP_LED_COLORS__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_LED_COLORS__reply v{};
        v.hue = r.read_le<std::uint16_t>();
        v.saturation = r.read_le<std::uint8_t>();
        v.value = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP_SET_LED_COLORS__request {
    std::uint16_t hue;
    std::uint8_t saturation;
    std::uint8_t value;

    static std::vector<std::uint8_t> pack(const MSP_SET_LED_COLORS__request& v) {
        BufferWriter w;
        w.write_le(v.hue);
        w.write_le(v.saturation);
        w.write_le(v.value);
        return std::move(w.buf);
    }

    static MSP_SET_LED_COLORS__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_SET_LED_COLORS__request v{};
        v.hue = r.read_le<std::uint16_t>();
        v.saturation = r.read_le<std::uint8_t>();
        v.value = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP_LED_STRIP_CONFIG__reply {
    std::uint32_t legacyLedConfig;

    static std::vector<std::uint8_t> pack(const MSP_LED_STRIP_CONFIG__reply& v) {
        BufferWriter w;
        w.write_le(v.legacyLedConfig);
        return std::move(w.buf);
    }

    static MSP_LED_STRIP_CONFIG__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_LED_STRIP_CONFIG__reply v{};
        v.legacyLedConfig = r.read_le<std::uint32_t>();
        return v;
    }
};

struct MSP_SET_LED_STRIP_CONFIG__request {
    std::uint8_t ledIndex;
    std::uint32_t legacyLedConfig;

    static std::vector<std::uint8_t> pack(const MSP_SET_LED_STRIP_CONFIG__request& v) {
        BufferWriter w;
        w.write_le(v.ledIndex);
        w.write_le(v.legacyLedConfig);
        return std::move(w.buf);
    }

    static MSP_SET_LED_STRIP_CONFIG__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_SET_LED_STRIP_CONFIG__request v{};
        v.ledIndex = r.read_le<std::uint8_t>();
        v.legacyLedConfig = r.read_le<std::uint32_t>();
        return v;
    }
};

struct MSP_RSSI_CONFIG__reply {
    std::uint8_t rssiChannel;

    static std::vector<std::uint8_t> pack(const MSP_RSSI_CONFIG__reply& v) {
        BufferWriter w;
        w.write_le(v.rssiChannel);
        return std::move(w.buf);
    }

    static MSP_RSSI_CONFIG__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_RSSI_CONFIG__reply v{};
        v.rssiChannel = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP_SET_RSSI_CONFIG__request {
    std::uint8_t rssiChannel;

    static std::vector<std::uint8_t> pack(const MSP_SET_RSSI_CONFIG__request& v) {
        BufferWriter w;
        w.write_le(v.rssiChannel);
        return std::move(w.buf);
    }

    static MSP_SET_RSSI_CONFIG__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_SET_RSSI_CONFIG__request v{};
        v.rssiChannel = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP_ADJUSTMENT_RANGES__reply {
    std::uint8_t adjustmentIndex;
    std::uint8_t auxChannelIndex;
    std::uint8_t rangeStartStep;
    std::uint8_t rangeEndStep;
    std::uint8_t adjustmentFunction;
    std::uint8_t auxSwitchChannelIndex;

    static std::vector<std::uint8_t> pack(const MSP_ADJUSTMENT_RANGES__reply& v) {
        BufferWriter w;
        w.write_le(v.adjustmentIndex);
        w.write_le(v.auxChannelIndex);
        w.write_le(v.rangeStartStep);
        w.write_le(v.rangeEndStep);
        w.write_le(v.adjustmentFunction);
        w.write_le(v.auxSwitchChannelIndex);
        return std::move(w.buf);
    }

    static MSP_ADJUSTMENT_RANGES__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_ADJUSTMENT_RANGES__reply v{};
        v.adjustmentIndex = r.read_le<std::uint8_t>();
        v.auxChannelIndex = r.read_le<std::uint8_t>();
        v.rangeStartStep = r.read_le<std::uint8_t>();
        v.rangeEndStep = r.read_le<std::uint8_t>();
        v.adjustmentFunction = r.read_le<std::uint8_t>();
        v.auxSwitchChannelIndex = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP_SET_ADJUSTMENT_RANGE__request {
    std::uint8_t rangeIndex;
    std::uint8_t adjustmentIndex;
    std::uint8_t auxChannelIndex;
    std::uint8_t rangeStartStep;
    std::uint8_t rangeEndStep;
    std::uint8_t adjustmentFunction;
    std::uint8_t auxSwitchChannelIndex;

    static std::vector<std::uint8_t> pack(const MSP_SET_ADJUSTMENT_RANGE__request& v) {
        BufferWriter w;
        w.write_le(v.rangeIndex);
        w.write_le(v.adjustmentIndex);
        w.write_le(v.auxChannelIndex);
        w.write_le(v.rangeStartStep);
        w.write_le(v.rangeEndStep);
        w.write_le(v.adjustmentFunction);
        w.write_le(v.auxSwitchChannelIndex);
        return std::move(w.buf);
    }

    static MSP_SET_ADJUSTMENT_RANGE__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_SET_ADJUSTMENT_RANGE__request v{};
        v.rangeIndex = r.read_le<std::uint8_t>();
        v.adjustmentIndex = r.read_le<std::uint8_t>();
        v.auxChannelIndex = r.read_le<std::uint8_t>();
        v.rangeStartStep = r.read_le<std::uint8_t>();
        v.rangeEndStep = r.read_le<std::uint8_t>();
        v.adjustmentFunction = r.read_le<std::uint8_t>();
        v.auxSwitchChannelIndex = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP_VOLTAGE_METER_CONFIG__reply {
    std::uint8_t vbatScale;
    std::uint8_t vbatMinCell;
    std::uint8_t vbatMaxCell;
    std::uint8_t vbatWarningCell;

    static std::vector<std::uint8_t> pack(const MSP_VOLTAGE_METER_CONFIG__reply& v) {
        BufferWriter w;
        w.write_le(v.vbatScale);
        w.write_le(v.vbatMinCell);
        w.write_le(v.vbatMaxCell);
        w.write_le(v.vbatWarningCell);
        return std::move(w.buf);
    }

    static MSP_VOLTAGE_METER_CONFIG__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_VOLTAGE_METER_CONFIG__reply v{};
        v.vbatScale = r.read_le<std::uint8_t>();
        v.vbatMinCell = r.read_le<std::uint8_t>();
        v.vbatMaxCell = r.read_le<std::uint8_t>();
        v.vbatWarningCell = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP_SET_VOLTAGE_METER_CONFIG__request {
    std::uint8_t vbatScale;
    std::uint8_t vbatMinCell;
    std::uint8_t vbatMaxCell;
    std::uint8_t vbatWarningCell;

    static std::vector<std::uint8_t> pack(const MSP_SET_VOLTAGE_METER_CONFIG__request& v) {
        BufferWriter w;
        w.write_le(v.vbatScale);
        w.write_le(v.vbatMinCell);
        w.write_le(v.vbatMaxCell);
        w.write_le(v.vbatWarningCell);
        return std::move(w.buf);
    }

    static MSP_SET_VOLTAGE_METER_CONFIG__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_SET_VOLTAGE_METER_CONFIG__request v{};
        v.vbatScale = r.read_le<std::uint8_t>();
        v.vbatMinCell = r.read_le<std::uint8_t>();
        v.vbatMaxCell = r.read_le<std::uint8_t>();
        v.vbatWarningCell = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP_SONAR_ALTITUDE__reply {
    std::uint32_t rangefinderAltitude;

    static std::vector<std::uint8_t> pack(const MSP_SONAR_ALTITUDE__reply& v) {
        BufferWriter w;
        w.write_le(v.rangefinderAltitude);
        return std::move(w.buf);
    }

    static MSP_SONAR_ALTITUDE__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_SONAR_ALTITUDE__reply v{};
        v.rangefinderAltitude = r.read_le<std::uint32_t>();
        return v;
    }
};

struct MSP_RX_MAP__reply {
    std::vector<std::uint8_t> rcMap;

    static std::vector<std::uint8_t> pack(const MSP_RX_MAP__reply& v) {
        BufferWriter w;
        for (const auto& e : v.rcMap) w.write_le(e);
        return std::move(w.buf);
    }

    static MSP_RX_MAP__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_RX_MAP__reply v{};
        v.rcMap.clear();
        while (r.remaining() >= sizeof(std::uint8_t)) v.rcMap.push_back(r.read_le<std::uint8_t>());
        return v;
    }
};

struct MSP_SET_RX_MAP__request {
    std::vector<std::uint8_t> rcMap;

    static std::vector<std::uint8_t> pack(const MSP_SET_RX_MAP__request& v) {
        BufferWriter w;
        for (const auto& e : v.rcMap) w.write_le(e);
        return std::move(w.buf);
    }

    static MSP_SET_RX_MAP__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_SET_RX_MAP__request v{};
        v.rcMap.clear();
        while (r.remaining() >= sizeof(std::uint8_t)) v.rcMap.push_back(r.read_le<std::uint8_t>());
        return v;
    }
};

struct MSP_DATAFLASH_SUMMARY__reply {
    std::uint8_t flashReady;
    std::uint32_t sectorCount;
    std::uint32_t totalSize;
    std::uint32_t usedSize;

    static std::vector<std::uint8_t> pack(const MSP_DATAFLASH_SUMMARY__reply& v) {
        BufferWriter w;
        w.write_le(v.flashReady);
        w.write_le(v.sectorCount);
        w.write_le(v.totalSize);
        w.write_le(v.usedSize);
        return std::move(w.buf);
    }

    static MSP_DATAFLASH_SUMMARY__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_DATAFLASH_SUMMARY__reply v{};
        v.flashReady = r.read_le<std::uint8_t>();
        v.sectorCount = r.read_le<std::uint32_t>();
        v.totalSize = r.read_le<std::uint32_t>();
        v.usedSize = r.read_le<std::uint32_t>();
        return v;
    }
};

struct MSP_DATAFLASH_READ__request {
    std::uint32_t address;
    std::uint16_t size;

    static std::vector<std::uint8_t> pack(const MSP_DATAFLASH_READ__request& v) {
        BufferWriter w;
        w.write_le(v.address);
        w.write_le(v.size);
        return std::move(w.buf);
    }

    static MSP_DATAFLASH_READ__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_DATAFLASH_READ__request v{};
        v.address = r.read_le<std::uint32_t>();
        v.size = r.read_le<std::uint16_t>();
        return v;
    }
};

struct MSP_DATAFLASH_READ__reply {
    std::uint32_t address;
    std::vector<uint8_t> data;

    static std::vector<std::uint8_t> pack(const MSP_DATAFLASH_READ__reply& v) {
        BufferWriter w;
        w.write_le(v.address);
        for (const auto& e : v.data) w.write_le(e);
        return std::move(w.buf);
    }

    static MSP_DATAFLASH_READ__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_DATAFLASH_READ__reply v{};
        v.address = r.read_le<std::uint32_t>();
        v.data.clear();
        while (r.remaining() >= sizeof(uint8_t)) v.data.push_back(r.read_le<uint8_t>());
        return v;
    }
};

struct MSP_LOOP_TIME__reply {
    std::uint16_t looptime;

    static std::vector<std::uint8_t> pack(const MSP_LOOP_TIME__reply& v) {
        BufferWriter w;
        w.write_le(v.looptime);
        return std::move(w.buf);
    }

    static MSP_LOOP_TIME__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_LOOP_TIME__reply v{};
        v.looptime = r.read_le<std::uint16_t>();
        return v;
    }
};

struct MSP_SET_LOOP_TIME__request {
    std::uint16_t looptime;

    static std::vector<std::uint8_t> pack(const MSP_SET_LOOP_TIME__request& v) {
        BufferWriter w;
        w.write_le(v.looptime);
        return std::move(w.buf);
    }

    static MSP_SET_LOOP_TIME__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_SET_LOOP_TIME__request v{};
        v.looptime = r.read_le<std::uint16_t>();
        return v;
    }
};

struct MSP_FAILSAFE_CONFIG__reply {
    std::uint8_t failsafeDelay;
    std::uint8_t failsafeOffDelay;
    std::uint16_t failsafeThrottle;
    std::uint8_t legacyKillSwitch;
    std::uint16_t failsafeThrottleLowDelay;
    std::uint8_t failsafeProcedure;
    std::uint8_t failsafeRecoveryDelay;
    std::uint16_t failsafeFWRollAngle;
    std::uint16_t failsafeFWPitchAngle;
    std::uint16_t failsafeFWYawRate;
    std::uint16_t failsafeStickThreshold;
    std::uint16_t failsafeMinDistance;
    std::uint8_t failsafeMinDistanceProc;

    static std::vector<std::uint8_t> pack(const MSP_FAILSAFE_CONFIG__reply& v) {
        BufferWriter w;
        w.write_le(v.failsafeDelay);
        w.write_le(v.failsafeOffDelay);
        w.write_le(v.failsafeThrottle);
        w.write_le(v.legacyKillSwitch);
        w.write_le(v.failsafeThrottleLowDelay);
        w.write_le(v.failsafeProcedure);
        w.write_le(v.failsafeRecoveryDelay);
        w.write_le(v.failsafeFWRollAngle);
        w.write_le(v.failsafeFWPitchAngle);
        w.write_le(v.failsafeFWYawRate);
        w.write_le(v.failsafeStickThreshold);
        w.write_le(v.failsafeMinDistance);
        w.write_le(v.failsafeMinDistanceProc);
        return std::move(w.buf);
    }

    static MSP_FAILSAFE_CONFIG__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_FAILSAFE_CONFIG__reply v{};
        v.failsafeDelay = r.read_le<std::uint8_t>();
        v.failsafeOffDelay = r.read_le<std::uint8_t>();
        v.failsafeThrottle = r.read_le<std::uint16_t>();
        v.legacyKillSwitch = r.read_le<std::uint8_t>();
        v.failsafeThrottleLowDelay = r.read_le<std::uint16_t>();
        v.failsafeProcedure = r.read_le<std::uint8_t>();
        v.failsafeRecoveryDelay = r.read_le<std::uint8_t>();
        v.failsafeFWRollAngle = r.read_le<std::uint16_t>();
        v.failsafeFWPitchAngle = r.read_le<std::uint16_t>();
        v.failsafeFWYawRate = r.read_le<std::uint16_t>();
        v.failsafeStickThreshold = r.read_le<std::uint16_t>();
        v.failsafeMinDistance = r.read_le<std::uint16_t>();
        v.failsafeMinDistanceProc = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP_SET_FAILSAFE_CONFIG__request {
    std::uint8_t failsafeDelay;
    std::uint8_t failsafeOffDelay;
    std::uint16_t failsafeThrottle;
    std::uint8_t legacyKillSwitch;
    std::uint16_t failsafeThrottleLowDelay;
    std::uint8_t failsafeProcedure;
    std::uint8_t failsafeRecoveryDelay;
    std::uint16_t failsafeFWRollAngle;
    std::uint16_t failsafeFWPitchAngle;
    std::uint16_t failsafeFWYawRate;
    std::uint16_t failsafeStickThreshold;
    std::uint16_t failsafeMinDistance;
    std::uint8_t failsafeMinDistanceProc;

    static std::vector<std::uint8_t> pack(const MSP_SET_FAILSAFE_CONFIG__request& v) {
        BufferWriter w;
        w.write_le(v.failsafeDelay);
        w.write_le(v.failsafeOffDelay);
        w.write_le(v.failsafeThrottle);
        w.write_le(v.legacyKillSwitch);
        w.write_le(v.failsafeThrottleLowDelay);
        w.write_le(v.failsafeProcedure);
        w.write_le(v.failsafeRecoveryDelay);
        w.write_le(v.failsafeFWRollAngle);
        w.write_le(v.failsafeFWPitchAngle);
        w.write_le(v.failsafeFWYawRate);
        w.write_le(v.failsafeStickThreshold);
        w.write_le(v.failsafeMinDistance);
        w.write_le(v.failsafeMinDistanceProc);
        return std::move(w.buf);
    }

    static MSP_SET_FAILSAFE_CONFIG__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_SET_FAILSAFE_CONFIG__request v{};
        v.failsafeDelay = r.read_le<std::uint8_t>();
        v.failsafeOffDelay = r.read_le<std::uint8_t>();
        v.failsafeThrottle = r.read_le<std::uint16_t>();
        v.legacyKillSwitch = r.read_le<std::uint8_t>();
        v.failsafeThrottleLowDelay = r.read_le<std::uint16_t>();
        v.failsafeProcedure = r.read_le<std::uint8_t>();
        v.failsafeRecoveryDelay = r.read_le<std::uint8_t>();
        v.failsafeFWRollAngle = r.read_le<std::uint16_t>();
        v.failsafeFWPitchAngle = r.read_le<std::uint16_t>();
        v.failsafeFWYawRate = r.read_le<std::uint16_t>();
        v.failsafeStickThreshold = r.read_le<std::uint16_t>();
        v.failsafeMinDistance = r.read_le<std::uint16_t>();
        v.failsafeMinDistanceProc = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP_SDCARD_SUMMARY__reply {
    std::uint8_t sdCardSupported;
    std::uint8_t sdCardState;
    std::uint8_t fsError;
    std::uint32_t freeSpaceKB;
    std::uint32_t totalSpaceKB;

    static std::vector<std::uint8_t> pack(const MSP_SDCARD_SUMMARY__reply& v) {
        BufferWriter w;
        w.write_le(v.sdCardSupported);
        w.write_le(v.sdCardState);
        w.write_le(v.fsError);
        w.write_le(v.freeSpaceKB);
        w.write_le(v.totalSpaceKB);
        return std::move(w.buf);
    }

    static MSP_SDCARD_SUMMARY__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_SDCARD_SUMMARY__reply v{};
        v.sdCardSupported = r.read_le<std::uint8_t>();
        v.sdCardState = r.read_le<std::uint8_t>();
        v.fsError = r.read_le<std::uint8_t>();
        v.freeSpaceKB = r.read_le<std::uint32_t>();
        v.totalSpaceKB = r.read_le<std::uint32_t>();
        return v;
    }
};

struct MSP_BLACKBOX_CONFIG__reply {
    std::uint8_t blackboxDevice;
    std::uint8_t blackboxRateNum;
    std::uint8_t blackboxRateDenom;
    std::uint8_t blackboxPDenom;

    static std::vector<std::uint8_t> pack(const MSP_BLACKBOX_CONFIG__reply& v) {
        BufferWriter w;
        w.write_le(v.blackboxDevice);
        w.write_le(v.blackboxRateNum);
        w.write_le(v.blackboxRateDenom);
        w.write_le(v.blackboxPDenom);
        return std::move(w.buf);
    }

    static MSP_BLACKBOX_CONFIG__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_BLACKBOX_CONFIG__reply v{};
        v.blackboxDevice = r.read_le<std::uint8_t>();
        v.blackboxRateNum = r.read_le<std::uint8_t>();
        v.blackboxRateDenom = r.read_le<std::uint8_t>();
        v.blackboxPDenom = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP_OSD_CONFIG__reply {
    std::uint8_t osdDriverType;
    std::uint8_t videoSystem;
    std::uint8_t units;
    std::uint8_t rssiAlarm;
    std::uint16_t capAlarm;
    std::uint16_t timerAlarm;
    std::uint16_t altAlarm;
    std::uint16_t distAlarm;
    std::uint16_t negAltAlarm;
    std::vector<std::uint16_t> itemPositions;

    static std::vector<std::uint8_t> pack(const MSP_OSD_CONFIG__reply& v) {
        BufferWriter w;
        w.write_le(v.osdDriverType);
        w.write_le(v.videoSystem);
        w.write_le(v.units);
        w.write_le(v.rssiAlarm);
        w.write_le(v.capAlarm);
        w.write_le(v.timerAlarm);
        w.write_le(v.altAlarm);
        w.write_le(v.distAlarm);
        w.write_le(v.negAltAlarm);
        for (const auto& e : v.itemPositions) w.write_le(e);
        return std::move(w.buf);
    }

    static MSP_OSD_CONFIG__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_OSD_CONFIG__reply v{};
        v.osdDriverType = r.read_le<std::uint8_t>();
        v.videoSystem = r.read_le<std::uint8_t>();
        v.units = r.read_le<std::uint8_t>();
        v.rssiAlarm = r.read_le<std::uint8_t>();
        v.capAlarm = r.read_le<std::uint16_t>();
        v.timerAlarm = r.read_le<std::uint16_t>();
        v.altAlarm = r.read_le<std::uint16_t>();
        v.distAlarm = r.read_le<std::uint16_t>();
        v.negAltAlarm = r.read_le<std::uint16_t>();
        v.itemPositions.clear();
        while (r.remaining() >= sizeof(std::uint16_t)) v.itemPositions.push_back(r.read_le<std::uint16_t>());
        return v;
    }
};

struct MSP_SET_OSD_CONFIG__dataSize____10 {
    std::uint8_t videoSystem;
    std::uint8_t units;
    std::uint8_t rssiAlarm;
    std::uint16_t capAlarm;
    std::uint16_t timerAlarm;
    std::uint16_t altAlarm;
    std::uint16_t distAlarm;
    std::uint16_t negAltAlarm;

    static std::vector<std::uint8_t> pack(const MSP_SET_OSD_CONFIG__dataSize____10& v) {
        BufferWriter w;
        w.write_le(v.videoSystem);
        w.write_le(v.units);
        w.write_le(v.rssiAlarm);
        w.write_le(v.capAlarm);
        w.write_le(v.timerAlarm);
        w.write_le(v.altAlarm);
        w.write_le(v.distAlarm);
        w.write_le(v.negAltAlarm);
        return std::move(w.buf);
    }

    static MSP_SET_OSD_CONFIG__dataSize____10 unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_SET_OSD_CONFIG__dataSize____10 v{};
        v.videoSystem = r.read_le<std::uint8_t>();
        v.units = r.read_le<std::uint8_t>();
        v.rssiAlarm = r.read_le<std::uint8_t>();
        v.capAlarm = r.read_le<std::uint16_t>();
        v.timerAlarm = r.read_le<std::uint16_t>();
        v.altAlarm = r.read_le<std::uint16_t>();
        v.distAlarm = r.read_le<std::uint16_t>();
        v.negAltAlarm = r.read_le<std::uint16_t>();
        return v;
    }
};

struct MSP_SET_OSD_CONFIG__dataSize____3 {
    std::vector<std::uint16_t> itemPositions;

    static std::vector<std::uint8_t> pack(const MSP_SET_OSD_CONFIG__dataSize____3& v) {
        BufferWriter w;
        for (const auto& e : v.itemPositions) w.write_le(e);
        return std::move(w.buf);
    }

    static MSP_SET_OSD_CONFIG__dataSize____3 unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_SET_OSD_CONFIG__dataSize____3 v{};
        v.itemPositions.clear();
        while (r.remaining() >= sizeof(std::uint16_t)) v.itemPositions.push_back(r.read_le<std::uint16_t>());
        return v;
    }
};

using MSP_SET_OSD_CONFIG_variant = std::variant<MSP_SET_OSD_CONFIG__dataSize____10, MSP_SET_OSD_CONFIG__dataSize____3>;
inline MSP_SET_OSD_CONFIG_variant unpack_MSP_SET_OSD_CONFIG(const std::vector<std::uint8_t>& payload) {
    switch (payload.size()) {
    case 3: return MSP_SET_OSD_CONFIG__dataSize____3::unpack(payload);
    default:
        if (payload.size() >= 10) return MSP_SET_OSD_CONFIG__dataSize____10::unpack(payload);
        return MSP_SET_OSD_CONFIG__dataSize____10::unpack(payload);
    }
}

struct MSP_OSD_CHAR_WRITE__dataSize____OSD_CHAR_BYTES___2 {
    std::uint16_t address;
    std::vector<std::uint8_t> charData;

    static std::vector<std::uint8_t> pack(const MSP_OSD_CHAR_WRITE__dataSize____OSD_CHAR_BYTES___2& v) {
        BufferWriter w;
        w.write_le(v.address);
        for (const auto& e : v.charData) w.write_le(e);
        return std::move(w.buf);
    }

    static MSP_OSD_CHAR_WRITE__dataSize____OSD_CHAR_BYTES___2 unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_OSD_CHAR_WRITE__dataSize____OSD_CHAR_BYTES___2 v{};
        v.address = r.read_le<std::uint16_t>();
        v.charData.clear();
        while (r.remaining() >= sizeof(std::uint8_t)) v.charData.push_back(r.read_le<std::uint8_t>());
        return v;
    }
};

struct MSP_OSD_CHAR_WRITE__dataSize____OSD_CHAR_BYTES___1 {
    std::uint8_t address;
    std::vector<std::uint8_t> charData;

    static std::vector<std::uint8_t> pack(const MSP_OSD_CHAR_WRITE__dataSize____OSD_CHAR_BYTES___1& v) {
        BufferWriter w;
        w.write_le(v.address);
        for (const auto& e : v.charData) w.write_le(e);
        return std::move(w.buf);
    }

    static MSP_OSD_CHAR_WRITE__dataSize____OSD_CHAR_BYTES___1 unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_OSD_CHAR_WRITE__dataSize____OSD_CHAR_BYTES___1 v{};
        v.address = r.read_le<std::uint8_t>();
        v.charData.clear();
        while (r.remaining() >= sizeof(std::uint8_t)) v.charData.push_back(r.read_le<std::uint8_t>());
        return v;
    }
};

struct MSP_OSD_CHAR_WRITE__dataSize____OSD_CHAR_VISIBLE_BYTES___2 {
    std::uint16_t address;
    std::vector<std::uint8_t> charData;

    static std::vector<std::uint8_t> pack(const MSP_OSD_CHAR_WRITE__dataSize____OSD_CHAR_VISIBLE_BYTES___2& v) {
        BufferWriter w;
        w.write_le(v.address);
        for (const auto& e : v.charData) w.write_le(e);
        return std::move(w.buf);
    }

    static MSP_OSD_CHAR_WRITE__dataSize____OSD_CHAR_VISIBLE_BYTES___2 unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_OSD_CHAR_WRITE__dataSize____OSD_CHAR_VISIBLE_BYTES___2 v{};
        v.address = r.read_le<std::uint16_t>();
        v.charData.clear();
        while (r.remaining() >= sizeof(std::uint8_t)) v.charData.push_back(r.read_le<std::uint8_t>());
        return v;
    }
};

struct MSP_OSD_CHAR_WRITE__dataSize____OSD_CHAR_VISIBLE_BYTES___1 {
    std::uint8_t address;
    std::vector<std::uint8_t> charData;

    static std::vector<std::uint8_t> pack(const MSP_OSD_CHAR_WRITE__dataSize____OSD_CHAR_VISIBLE_BYTES___1& v) {
        BufferWriter w;
        w.write_le(v.address);
        for (const auto& e : v.charData) w.write_le(e);
        return std::move(w.buf);
    }

    static MSP_OSD_CHAR_WRITE__dataSize____OSD_CHAR_VISIBLE_BYTES___1 unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_OSD_CHAR_WRITE__dataSize____OSD_CHAR_VISIBLE_BYTES___1 v{};
        v.address = r.read_le<std::uint8_t>();
        v.charData.clear();
        while (r.remaining() >= sizeof(std::uint8_t)) v.charData.push_back(r.read_le<std::uint8_t>());
        return v;
    }
};

using MSP_OSD_CHAR_WRITE_variant = std::variant<MSP_OSD_CHAR_WRITE__dataSize____OSD_CHAR_BYTES___2, MSP_OSD_CHAR_WRITE__dataSize____OSD_CHAR_BYTES___1, MSP_OSD_CHAR_WRITE__dataSize____OSD_CHAR_VISIBLE_BYTES___2, MSP_OSD_CHAR_WRITE__dataSize____OSD_CHAR_VISIBLE_BYTES___1>;
inline MSP_OSD_CHAR_WRITE_variant unpack_MSP_OSD_CHAR_WRITE(const std::vector<std::uint8_t>& payload) {
    switch (payload.size()) {
    default:
        if (payload.size() >= 2) return MSP_OSD_CHAR_WRITE__dataSize____OSD_CHAR_BYTES___2::unpack(payload);
        if (payload.size() >= 1) return MSP_OSD_CHAR_WRITE__dataSize____OSD_CHAR_BYTES___1::unpack(payload);
        if (payload.size() >= 2) return MSP_OSD_CHAR_WRITE__dataSize____OSD_CHAR_VISIBLE_BYTES___2::unpack(payload);
        if (payload.size() >= 1) return MSP_OSD_CHAR_WRITE__dataSize____OSD_CHAR_VISIBLE_BYTES___1::unpack(payload);
        return MSP_OSD_CHAR_WRITE__dataSize____OSD_CHAR_BYTES___2::unpack(payload);
    }
}

struct MSP_VTX_CONFIG__reply {
    std::uint8_t vtxDeviceType;
    std::uint8_t band;
    std::uint8_t channel;
    std::uint8_t power;
    std::uint8_t pitMode;
    std::uint8_t vtxReady;
    std::uint8_t lowPowerDisarm;
    std::uint8_t vtxTableAvailable;
    std::uint8_t bandCount;
    std::uint8_t channelCount;
    std::uint8_t powerCount;

    static std::vector<std::uint8_t> pack(const MSP_VTX_CONFIG__reply& v) {
        BufferWriter w;
        w.write_le(v.vtxDeviceType);
        w.write_le(v.band);
        w.write_le(v.channel);
        w.write_le(v.power);
        w.write_le(v.pitMode);
        w.write_le(v.vtxReady);
        w.write_le(v.lowPowerDisarm);
        w.write_le(v.vtxTableAvailable);
        w.write_le(v.bandCount);
        w.write_le(v.channelCount);
        w.write_le(v.powerCount);
        return std::move(w.buf);
    }

    static MSP_VTX_CONFIG__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_VTX_CONFIG__reply v{};
        v.vtxDeviceType = r.read_le<std::uint8_t>();
        v.band = r.read_le<std::uint8_t>();
        v.channel = r.read_le<std::uint8_t>();
        v.power = r.read_le<std::uint8_t>();
        v.pitMode = r.read_le<std::uint8_t>();
        v.vtxReady = r.read_le<std::uint8_t>();
        v.lowPowerDisarm = r.read_le<std::uint8_t>();
        v.vtxTableAvailable = r.read_le<std::uint8_t>();
        v.bandCount = r.read_le<std::uint8_t>();
        v.channelCount = r.read_le<std::uint8_t>();
        v.powerCount = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP_SET_VTX_CONFIG__dataSize____2 {
    std::uint16_t bandChanOrFreq;

    static std::vector<std::uint8_t> pack(const MSP_SET_VTX_CONFIG__dataSize____2& v) {
        BufferWriter w;
        w.write_le(v.bandChanOrFreq);
        return std::move(w.buf);
    }

    static MSP_SET_VTX_CONFIG__dataSize____2 unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_SET_VTX_CONFIG__dataSize____2 v{};
        v.bandChanOrFreq = r.read_le<std::uint16_t>();
        return v;
    }
};

struct MSP_SET_VTX_CONFIG__dataSize____4 {
    std::uint16_t bandChanOrFreq;
    std::uint8_t power;
    std::uint8_t pitMode;

    static std::vector<std::uint8_t> pack(const MSP_SET_VTX_CONFIG__dataSize____4& v) {
        BufferWriter w;
        w.write_le(v.bandChanOrFreq);
        w.write_le(v.power);
        w.write_le(v.pitMode);
        return std::move(w.buf);
    }

    static MSP_SET_VTX_CONFIG__dataSize____4 unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_SET_VTX_CONFIG__dataSize____4 v{};
        v.bandChanOrFreq = r.read_le<std::uint16_t>();
        v.power = r.read_le<std::uint8_t>();
        v.pitMode = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP_SET_VTX_CONFIG__dataSize____5 {
    std::uint16_t bandChanOrFreq;
    std::uint8_t power;
    std::uint8_t pitMode;
    std::uint8_t lowPowerDisarm;

    static std::vector<std::uint8_t> pack(const MSP_SET_VTX_CONFIG__dataSize____5& v) {
        BufferWriter w;
        w.write_le(v.bandChanOrFreq);
        w.write_le(v.power);
        w.write_le(v.pitMode);
        w.write_le(v.lowPowerDisarm);
        return std::move(w.buf);
    }

    static MSP_SET_VTX_CONFIG__dataSize____5 unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_SET_VTX_CONFIG__dataSize____5 v{};
        v.bandChanOrFreq = r.read_le<std::uint16_t>();
        v.power = r.read_le<std::uint8_t>();
        v.pitMode = r.read_le<std::uint8_t>();
        v.lowPowerDisarm = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP_SET_VTX_CONFIG__dataSize____7 {
    std::uint16_t bandChanOrFreq;
    std::uint8_t power;
    std::uint8_t pitMode;
    std::uint8_t lowPowerDisarm;
    std::uint16_t pitModeFreq;

    static std::vector<std::uint8_t> pack(const MSP_SET_VTX_CONFIG__dataSize____7& v) {
        BufferWriter w;
        w.write_le(v.bandChanOrFreq);
        w.write_le(v.power);
        w.write_le(v.pitMode);
        w.write_le(v.lowPowerDisarm);
        w.write_le(v.pitModeFreq);
        return std::move(w.buf);
    }

    static MSP_SET_VTX_CONFIG__dataSize____7 unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_SET_VTX_CONFIG__dataSize____7 v{};
        v.bandChanOrFreq = r.read_le<std::uint16_t>();
        v.power = r.read_le<std::uint8_t>();
        v.pitMode = r.read_le<std::uint8_t>();
        v.lowPowerDisarm = r.read_le<std::uint8_t>();
        v.pitModeFreq = r.read_le<std::uint16_t>();
        return v;
    }
};

struct MSP_SET_VTX_CONFIG__dataSize____9 {
    std::uint16_t bandChanOrFreq;
    std::uint8_t power;
    std::uint8_t pitMode;
    std::uint8_t lowPowerDisarm;
    std::uint16_t pitModeFreq;
    std::uint8_t band;
    std::uint8_t channel;

    static std::vector<std::uint8_t> pack(const MSP_SET_VTX_CONFIG__dataSize____9& v) {
        BufferWriter w;
        w.write_le(v.bandChanOrFreq);
        w.write_le(v.power);
        w.write_le(v.pitMode);
        w.write_le(v.lowPowerDisarm);
        w.write_le(v.pitModeFreq);
        w.write_le(v.band);
        w.write_le(v.channel);
        return std::move(w.buf);
    }

    static MSP_SET_VTX_CONFIG__dataSize____9 unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_SET_VTX_CONFIG__dataSize____9 v{};
        v.bandChanOrFreq = r.read_le<std::uint16_t>();
        v.power = r.read_le<std::uint8_t>();
        v.pitMode = r.read_le<std::uint8_t>();
        v.lowPowerDisarm = r.read_le<std::uint8_t>();
        v.pitModeFreq = r.read_le<std::uint16_t>();
        v.band = r.read_le<std::uint8_t>();
        v.channel = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP_SET_VTX_CONFIG__dataSize____11 {
    std::uint16_t bandChanOrFreq;
    std::uint8_t power;
    std::uint8_t pitMode;
    std::uint8_t lowPowerDisarm;
    std::uint16_t pitModeFreq;
    std::uint8_t band;
    std::uint8_t channel;
    std::uint16_t frequency;

    static std::vector<std::uint8_t> pack(const MSP_SET_VTX_CONFIG__dataSize____11& v) {
        BufferWriter w;
        w.write_le(v.bandChanOrFreq);
        w.write_le(v.power);
        w.write_le(v.pitMode);
        w.write_le(v.lowPowerDisarm);
        w.write_le(v.pitModeFreq);
        w.write_le(v.band);
        w.write_le(v.channel);
        w.write_le(v.frequency);
        return std::move(w.buf);
    }

    static MSP_SET_VTX_CONFIG__dataSize____11 unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_SET_VTX_CONFIG__dataSize____11 v{};
        v.bandChanOrFreq = r.read_le<std::uint16_t>();
        v.power = r.read_le<std::uint8_t>();
        v.pitMode = r.read_le<std::uint8_t>();
        v.lowPowerDisarm = r.read_le<std::uint8_t>();
        v.pitModeFreq = r.read_le<std::uint16_t>();
        v.band = r.read_le<std::uint8_t>();
        v.channel = r.read_le<std::uint8_t>();
        v.frequency = r.read_le<std::uint16_t>();
        return v;
    }
};

struct MSP_SET_VTX_CONFIG__dataSize____14 {
    std::uint16_t bandChanOrFreq;
    std::uint8_t power;
    std::uint8_t pitMode;
    std::uint8_t lowPowerDisarm;
    std::uint16_t pitModeFreq;
    std::uint8_t band;
    std::uint8_t channel;
    std::uint16_t frequency;
    std::uint8_t bandCount;
    std::uint8_t channelCount;
    std::uint8_t powerCount;

    static std::vector<std::uint8_t> pack(const MSP_SET_VTX_CONFIG__dataSize____14& v) {
        BufferWriter w;
        w.write_le(v.bandChanOrFreq);
        w.write_le(v.power);
        w.write_le(v.pitMode);
        w.write_le(v.lowPowerDisarm);
        w.write_le(v.pitModeFreq);
        w.write_le(v.band);
        w.write_le(v.channel);
        w.write_le(v.frequency);
        w.write_le(v.bandCount);
        w.write_le(v.channelCount);
        w.write_le(v.powerCount);
        return std::move(w.buf);
    }

    static MSP_SET_VTX_CONFIG__dataSize____14 unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_SET_VTX_CONFIG__dataSize____14 v{};
        v.bandChanOrFreq = r.read_le<std::uint16_t>();
        v.power = r.read_le<std::uint8_t>();
        v.pitMode = r.read_le<std::uint8_t>();
        v.lowPowerDisarm = r.read_le<std::uint8_t>();
        v.pitModeFreq = r.read_le<std::uint16_t>();
        v.band = r.read_le<std::uint8_t>();
        v.channel = r.read_le<std::uint8_t>();
        v.frequency = r.read_le<std::uint16_t>();
        v.bandCount = r.read_le<std::uint8_t>();
        v.channelCount = r.read_le<std::uint8_t>();
        v.powerCount = r.read_le<std::uint8_t>();
        return v;
    }
};

using MSP_SET_VTX_CONFIG_variant = std::variant<MSP_SET_VTX_CONFIG__dataSize____2, MSP_SET_VTX_CONFIG__dataSize____4, MSP_SET_VTX_CONFIG__dataSize____5, MSP_SET_VTX_CONFIG__dataSize____7, MSP_SET_VTX_CONFIG__dataSize____9, MSP_SET_VTX_CONFIG__dataSize____11, MSP_SET_VTX_CONFIG__dataSize____14>;
inline MSP_SET_VTX_CONFIG_variant unpack_MSP_SET_VTX_CONFIG(const std::vector<std::uint8_t>& payload) {
    switch (payload.size()) {
    case 2: return MSP_SET_VTX_CONFIG__dataSize____2::unpack(payload);
    case 4: return MSP_SET_VTX_CONFIG__dataSize____4::unpack(payload);
    case 5: return MSP_SET_VTX_CONFIG__dataSize____5::unpack(payload);
    case 7: return MSP_SET_VTX_CONFIG__dataSize____7::unpack(payload);
    case 9: return MSP_SET_VTX_CONFIG__dataSize____9::unpack(payload);
    case 11: return MSP_SET_VTX_CONFIG__dataSize____11::unpack(payload);
    case 14: return MSP_SET_VTX_CONFIG__dataSize____14::unpack(payload);
    default:
        return MSP_SET_VTX_CONFIG__dataSize____2::unpack(payload);
    }
}

struct MSP_ADVANCED_CONFIG__reply {
    std::uint8_t gyroSyncDenom;
    std::uint8_t pidProcessDenom;
    std::uint8_t useUnsyncedPwm;
    std::uint8_t motorPwmProtocol;
    std::uint16_t motorPwmRate;
    std::uint16_t servoPwmRate;
    std::uint8_t legacyGyroSync;

    static std::vector<std::uint8_t> pack(const MSP_ADVANCED_CONFIG__reply& v) {
        BufferWriter w;
        w.write_le(v.gyroSyncDenom);
        w.write_le(v.pidProcessDenom);
        w.write_le(v.useUnsyncedPwm);
        w.write_le(v.motorPwmProtocol);
        w.write_le(v.motorPwmRate);
        w.write_le(v.servoPwmRate);
        w.write_le(v.legacyGyroSync);
        return std::move(w.buf);
    }

    static MSP_ADVANCED_CONFIG__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_ADVANCED_CONFIG__reply v{};
        v.gyroSyncDenom = r.read_le<std::uint8_t>();
        v.pidProcessDenom = r.read_le<std::uint8_t>();
        v.useUnsyncedPwm = r.read_le<std::uint8_t>();
        v.motorPwmProtocol = r.read_le<std::uint8_t>();
        v.motorPwmRate = r.read_le<std::uint16_t>();
        v.servoPwmRate = r.read_le<std::uint16_t>();
        v.legacyGyroSync = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP_SET_ADVANCED_CONFIG__request {
    std::uint8_t gyroSyncDenom;
    std::uint8_t pidProcessDenom;
    std::uint8_t useUnsyncedPwm;
    std::uint8_t motorPwmProtocol;
    std::uint16_t motorPwmRate;
    std::uint16_t servoPwmRate;
    std::uint8_t legacyGyroSync;

    static std::vector<std::uint8_t> pack(const MSP_SET_ADVANCED_CONFIG__request& v) {
        BufferWriter w;
        w.write_le(v.gyroSyncDenom);
        w.write_le(v.pidProcessDenom);
        w.write_le(v.useUnsyncedPwm);
        w.write_le(v.motorPwmProtocol);
        w.write_le(v.motorPwmRate);
        w.write_le(v.servoPwmRate);
        w.write_le(v.legacyGyroSync);
        return std::move(w.buf);
    }

    static MSP_SET_ADVANCED_CONFIG__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_SET_ADVANCED_CONFIG__request v{};
        v.gyroSyncDenom = r.read_le<std::uint8_t>();
        v.pidProcessDenom = r.read_le<std::uint8_t>();
        v.useUnsyncedPwm = r.read_le<std::uint8_t>();
        v.motorPwmProtocol = r.read_le<std::uint8_t>();
        v.motorPwmRate = r.read_le<std::uint16_t>();
        v.servoPwmRate = r.read_le<std::uint16_t>();
        v.legacyGyroSync = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP_FILTER_CONFIG__reply {
    std::uint8_t gyroMainLpfHz;
    std::uint16_t dtermLpfHz;
    std::uint16_t yawLpfHz;
    std::uint16_t legacyGyroNotchHz;
    std::uint16_t legacyGyroNotchCutoff;
    std::uint16_t bfCompatDtermNotchHz;
    std::uint16_t bfCompatDtermNotchCutoff;
    std::uint16_t bfCompatGyroNotch2Hz;
    std::uint16_t bfCompatGyroNotch2Cutoff;
    std::uint16_t accNotchHz;
    std::uint16_t accNotchCutoff;
    std::uint16_t legacyGyroStage2LpfHz;

    static std::vector<std::uint8_t> pack(const MSP_FILTER_CONFIG__reply& v) {
        BufferWriter w;
        w.write_le(v.gyroMainLpfHz);
        w.write_le(v.dtermLpfHz);
        w.write_le(v.yawLpfHz);
        w.write_le(v.legacyGyroNotchHz);
        w.write_le(v.legacyGyroNotchCutoff);
        w.write_le(v.bfCompatDtermNotchHz);
        w.write_le(v.bfCompatDtermNotchCutoff);
        w.write_le(v.bfCompatGyroNotch2Hz);
        w.write_le(v.bfCompatGyroNotch2Cutoff);
        w.write_le(v.accNotchHz);
        w.write_le(v.accNotchCutoff);
        w.write_le(v.legacyGyroStage2LpfHz);
        return std::move(w.buf);
    }

    static MSP_FILTER_CONFIG__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_FILTER_CONFIG__reply v{};
        v.gyroMainLpfHz = r.read_le<std::uint8_t>();
        v.dtermLpfHz = r.read_le<std::uint16_t>();
        v.yawLpfHz = r.read_le<std::uint16_t>();
        v.legacyGyroNotchHz = r.read_le<std::uint16_t>();
        v.legacyGyroNotchCutoff = r.read_le<std::uint16_t>();
        v.bfCompatDtermNotchHz = r.read_le<std::uint16_t>();
        v.bfCompatDtermNotchCutoff = r.read_le<std::uint16_t>();
        v.bfCompatGyroNotch2Hz = r.read_le<std::uint16_t>();
        v.bfCompatGyroNotch2Cutoff = r.read_le<std::uint16_t>();
        v.accNotchHz = r.read_le<std::uint16_t>();
        v.accNotchCutoff = r.read_le<std::uint16_t>();
        v.legacyGyroStage2LpfHz = r.read_le<std::uint16_t>();
        return v;
    }
};

struct MSP_SET_FILTER_CONFIG__request {
    std::uint8_t gyroMainLpfHz;
    std::uint16_t dtermLpfHz;
    std::uint16_t yawLpfHz;
    std::uint16_t legacyGyroNotchHz;
    std::uint16_t legacyGyroNotchCutoff;
    std::uint16_t bfCompatDtermNotchHz;
    std::uint16_t bfCompatDtermNotchCutoff;
    std::uint16_t bfCompatGyroNotch2Hz;
    std::uint16_t bfCompatGyroNotch2Cutoff;
    std::uint16_t accNotchHz;
    std::uint16_t accNotchCutoff;
    std::uint16_t legacyGyroStage2LpfHz;

    static std::vector<std::uint8_t> pack(const MSP_SET_FILTER_CONFIG__request& v) {
        BufferWriter w;
        w.write_le(v.gyroMainLpfHz);
        w.write_le(v.dtermLpfHz);
        w.write_le(v.yawLpfHz);
        w.write_le(v.legacyGyroNotchHz);
        w.write_le(v.legacyGyroNotchCutoff);
        w.write_le(v.bfCompatDtermNotchHz);
        w.write_le(v.bfCompatDtermNotchCutoff);
        w.write_le(v.bfCompatGyroNotch2Hz);
        w.write_le(v.bfCompatGyroNotch2Cutoff);
        w.write_le(v.accNotchHz);
        w.write_le(v.accNotchCutoff);
        w.write_le(v.legacyGyroStage2LpfHz);
        return std::move(w.buf);
    }

    static MSP_SET_FILTER_CONFIG__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_SET_FILTER_CONFIG__request v{};
        v.gyroMainLpfHz = r.read_le<std::uint8_t>();
        v.dtermLpfHz = r.read_le<std::uint16_t>();
        v.yawLpfHz = r.read_le<std::uint16_t>();
        v.legacyGyroNotchHz = r.read_le<std::uint16_t>();
        v.legacyGyroNotchCutoff = r.read_le<std::uint16_t>();
        v.bfCompatDtermNotchHz = r.read_le<std::uint16_t>();
        v.bfCompatDtermNotchCutoff = r.read_le<std::uint16_t>();
        v.bfCompatGyroNotch2Hz = r.read_le<std::uint16_t>();
        v.bfCompatGyroNotch2Cutoff = r.read_le<std::uint16_t>();
        v.accNotchHz = r.read_le<std::uint16_t>();
        v.accNotchCutoff = r.read_le<std::uint16_t>();
        v.legacyGyroStage2LpfHz = r.read_le<std::uint16_t>();
        return v;
    }
};

struct MSP_PID_ADVANCED__reply {
    std::uint16_t legacyRollPitchItermIgnore;
    std::uint16_t legacyYawItermIgnore;
    std::uint16_t legacyYawPLimit;
    std::uint8_t bfCompatDeltaMethod;
    std::uint8_t bfCompatVbatPidComp;
    std::uint8_t bfCompatSetpointRelaxRatio;
    std::uint8_t reserved1;
    std::uint16_t legacyPidSumLimit;
    std::uint8_t bfCompatItermThrottleGain;
    std::uint16_t accelLimitRollPitch;
    std::uint16_t accelLimitYaw;

    static std::vector<std::uint8_t> pack(const MSP_PID_ADVANCED__reply& v) {
        BufferWriter w;
        w.write_le(v.legacyRollPitchItermIgnore);
        w.write_le(v.legacyYawItermIgnore);
        w.write_le(v.legacyYawPLimit);
        w.write_le(v.bfCompatDeltaMethod);
        w.write_le(v.bfCompatVbatPidComp);
        w.write_le(v.bfCompatSetpointRelaxRatio);
        w.write_le(v.reserved1);
        w.write_le(v.legacyPidSumLimit);
        w.write_le(v.bfCompatItermThrottleGain);
        w.write_le(v.accelLimitRollPitch);
        w.write_le(v.accelLimitYaw);
        return std::move(w.buf);
    }

    static MSP_PID_ADVANCED__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_PID_ADVANCED__reply v{};
        v.legacyRollPitchItermIgnore = r.read_le<std::uint16_t>();
        v.legacyYawItermIgnore = r.read_le<std::uint16_t>();
        v.legacyYawPLimit = r.read_le<std::uint16_t>();
        v.bfCompatDeltaMethod = r.read_le<std::uint8_t>();
        v.bfCompatVbatPidComp = r.read_le<std::uint8_t>();
        v.bfCompatSetpointRelaxRatio = r.read_le<std::uint8_t>();
        v.reserved1 = r.read_le<std::uint8_t>();
        v.legacyPidSumLimit = r.read_le<std::uint16_t>();
        v.bfCompatItermThrottleGain = r.read_le<std::uint8_t>();
        v.accelLimitRollPitch = r.read_le<std::uint16_t>();
        v.accelLimitYaw = r.read_le<std::uint16_t>();
        return v;
    }
};

struct MSP_SET_PID_ADVANCED__request {
    std::uint16_t legacyRollPitchItermIgnore;
    std::uint16_t legacyYawItermIgnore;
    std::uint16_t legacyYawPLimit;
    std::uint8_t bfCompatDeltaMethod;
    std::uint8_t bfCompatVbatPidComp;
    std::uint8_t bfCompatSetpointRelaxRatio;
    std::uint8_t reserved1;
    std::uint16_t legacyPidSumLimit;
    std::uint8_t bfCompatItermThrottleGain;
    std::uint16_t accelLimitRollPitch;
    std::uint16_t accelLimitYaw;

    static std::vector<std::uint8_t> pack(const MSP_SET_PID_ADVANCED__request& v) {
        BufferWriter w;
        w.write_le(v.legacyRollPitchItermIgnore);
        w.write_le(v.legacyYawItermIgnore);
        w.write_le(v.legacyYawPLimit);
        w.write_le(v.bfCompatDeltaMethod);
        w.write_le(v.bfCompatVbatPidComp);
        w.write_le(v.bfCompatSetpointRelaxRatio);
        w.write_le(v.reserved1);
        w.write_le(v.legacyPidSumLimit);
        w.write_le(v.bfCompatItermThrottleGain);
        w.write_le(v.accelLimitRollPitch);
        w.write_le(v.accelLimitYaw);
        return std::move(w.buf);
    }

    static MSP_SET_PID_ADVANCED__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_SET_PID_ADVANCED__request v{};
        v.legacyRollPitchItermIgnore = r.read_le<std::uint16_t>();
        v.legacyYawItermIgnore = r.read_le<std::uint16_t>();
        v.legacyYawPLimit = r.read_le<std::uint16_t>();
        v.bfCompatDeltaMethod = r.read_le<std::uint8_t>();
        v.bfCompatVbatPidComp = r.read_le<std::uint8_t>();
        v.bfCompatSetpointRelaxRatio = r.read_le<std::uint8_t>();
        v.reserved1 = r.read_le<std::uint8_t>();
        v.legacyPidSumLimit = r.read_le<std::uint16_t>();
        v.bfCompatItermThrottleGain = r.read_le<std::uint8_t>();
        v.accelLimitRollPitch = r.read_le<std::uint16_t>();
        v.accelLimitYaw = r.read_le<std::uint16_t>();
        return v;
    }
};

struct MSP_SENSOR_CONFIG__reply {
    std::uint8_t accHardware;
    std::uint8_t baroHardware;
    std::uint8_t magHardware;
    std::uint8_t pitotHardware;
    std::uint8_t rangefinderHardware;
    std::uint8_t opflowHardware;

    static std::vector<std::uint8_t> pack(const MSP_SENSOR_CONFIG__reply& v) {
        BufferWriter w;
        w.write_le(v.accHardware);
        w.write_le(v.baroHardware);
        w.write_le(v.magHardware);
        w.write_le(v.pitotHardware);
        w.write_le(v.rangefinderHardware);
        w.write_le(v.opflowHardware);
        return std::move(w.buf);
    }

    static MSP_SENSOR_CONFIG__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_SENSOR_CONFIG__reply v{};
        v.accHardware = r.read_le<std::uint8_t>();
        v.baroHardware = r.read_le<std::uint8_t>();
        v.magHardware = r.read_le<std::uint8_t>();
        v.pitotHardware = r.read_le<std::uint8_t>();
        v.rangefinderHardware = r.read_le<std::uint8_t>();
        v.opflowHardware = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP_SET_SENSOR_CONFIG__request {
    std::uint8_t accHardware;
    std::uint8_t baroHardware;
    std::uint8_t magHardware;
    std::uint8_t pitotHardware;
    std::uint8_t rangefinderHardware;
    std::uint8_t opflowHardware;

    static std::vector<std::uint8_t> pack(const MSP_SET_SENSOR_CONFIG__request& v) {
        BufferWriter w;
        w.write_le(v.accHardware);
        w.write_le(v.baroHardware);
        w.write_le(v.magHardware);
        w.write_le(v.pitotHardware);
        w.write_le(v.rangefinderHardware);
        w.write_le(v.opflowHardware);
        return std::move(w.buf);
    }

    static MSP_SET_SENSOR_CONFIG__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_SET_SENSOR_CONFIG__request v{};
        v.accHardware = r.read_le<std::uint8_t>();
        v.baroHardware = r.read_le<std::uint8_t>();
        v.magHardware = r.read_le<std::uint8_t>();
        v.pitotHardware = r.read_le<std::uint8_t>();
        v.rangefinderHardware = r.read_le<std::uint8_t>();
        v.opflowHardware = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP_IDENT__reply {
    std::uint8_t MultiWii_version;
    std::uint8_t Mixer_Mode;
    std::uint8_t MSP_Version;
    std::uint32_t Platform_Capability;

    static std::vector<std::uint8_t> pack(const MSP_IDENT__reply& v) {
        BufferWriter w;
        w.write_le(v.MultiWii_version);
        w.write_le(v.Mixer_Mode);
        w.write_le(v.MSP_Version);
        w.write_le(v.Platform_Capability);
        return std::move(w.buf);
    }

    static MSP_IDENT__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_IDENT__reply v{};
        v.MultiWii_version = r.read_le<std::uint8_t>();
        v.Mixer_Mode = r.read_le<std::uint8_t>();
        v.MSP_Version = r.read_le<std::uint8_t>();
        v.Platform_Capability = r.read_le<std::uint32_t>();
        return v;
    }
};

struct MSP_STATUS__reply {
    std::uint16_t cycleTime;
    std::uint16_t i2cErrors;
    std::uint16_t sensorStatus;
    std::uint32_t activeModesLow;
    std::uint8_t profile;

    static std::vector<std::uint8_t> pack(const MSP_STATUS__reply& v) {
        BufferWriter w;
        w.write_le(v.cycleTime);
        w.write_le(v.i2cErrors);
        w.write_le(v.sensorStatus);
        w.write_le(v.activeModesLow);
        w.write_le(v.profile);
        return std::move(w.buf);
    }

    static MSP_STATUS__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_STATUS__reply v{};
        v.cycleTime = r.read_le<std::uint16_t>();
        v.i2cErrors = r.read_le<std::uint16_t>();
        v.sensorStatus = r.read_le<std::uint16_t>();
        v.activeModesLow = r.read_le<std::uint32_t>();
        v.profile = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP_RAW_IMU__reply {
    std::int16_t accX;
    std::int16_t accY;
    std::int16_t accZ;
    std::int16_t gyroX;
    std::int16_t gyroY;
    std::int16_t gyroZ;
    std::int16_t magX;
    std::int16_t magY;
    std::int16_t magZ;

    static std::vector<std::uint8_t> pack(const MSP_RAW_IMU__reply& v) {
        BufferWriter w;
        w.write_le(v.accX);
        w.write_le(v.accY);
        w.write_le(v.accZ);
        w.write_le(v.gyroX);
        w.write_le(v.gyroY);
        w.write_le(v.gyroZ);
        w.write_le(v.magX);
        w.write_le(v.magY);
        w.write_le(v.magZ);
        return std::move(w.buf);
    }

    static MSP_RAW_IMU__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_RAW_IMU__reply v{};
        v.accX = r.read_le<std::int16_t>();
        v.accY = r.read_le<std::int16_t>();
        v.accZ = r.read_le<std::int16_t>();
        v.gyroX = r.read_le<std::int16_t>();
        v.gyroY = r.read_le<std::int16_t>();
        v.gyroZ = r.read_le<std::int16_t>();
        v.magX = r.read_le<std::int16_t>();
        v.magY = r.read_le<std::int16_t>();
        v.magZ = r.read_le<std::int16_t>();
        return v;
    }
};

struct MSP_SERVO__reply {
    std::vector<std::int16_t> servoOutputs;

    static std::vector<std::uint8_t> pack(const MSP_SERVO__reply& v) {
        BufferWriter w;
        for (const auto& e : v.servoOutputs) w.write_le(e);
        return std::move(w.buf);
    }

    static MSP_SERVO__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_SERVO__reply v{};
        v.servoOutputs.clear();
        while (r.remaining() >= sizeof(std::int16_t)) v.servoOutputs.push_back(r.read_le<std::int16_t>());
        return v;
    }
};

struct MSP_MOTOR__reply {
    std::array<std::uint16_t,8> motorOutputs;

    static std::vector<std::uint8_t> pack(const MSP_MOTOR__reply& v) {
        BufferWriter w;
        for (const auto& e : v.motorOutputs) w.write_le(e);
        return std::move(w.buf);
    }

    static MSP_MOTOR__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_MOTOR__reply v{};
        for (auto& e : v.motorOutputs) e = r.read_le<decltype(e)>();
        return v;
    }
};

struct MSP_RC__reply {
    std::vector<uint16_t> rcChannels;

    static std::vector<std::uint8_t> pack(const MSP_RC__reply& v) {
        BufferWriter w;
        for (const auto& e : v.rcChannels) w.write_le(e);
        return std::move(w.buf);
    }

    static MSP_RC__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_RC__reply v{};
        v.rcChannels.clear();
        while (r.remaining() >= sizeof(uint16_t)) v.rcChannels.push_back(r.read_le<uint16_t>());
        return v;
    }
};

struct MSP_RAW_GPS__reply {
    std::uint8_t fixType;
    std::uint8_t numSat;
    std::int32_t latitude;
    std::int32_t longitude;
    std::int16_t altitude;
    std::uint16_t speed;
    std::uint16_t groundCourse;
    std::uint16_t hdop;

    static std::vector<std::uint8_t> pack(const MSP_RAW_GPS__reply& v) {
        BufferWriter w;
        w.write_le(v.fixType);
        w.write_le(v.numSat);
        w.write_le(v.latitude);
        w.write_le(v.longitude);
        w.write_le(v.altitude);
        w.write_le(v.speed);
        w.write_le(v.groundCourse);
        w.write_le(v.hdop);
        return std::move(w.buf);
    }

    static MSP_RAW_GPS__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_RAW_GPS__reply v{};
        v.fixType = r.read_le<std::uint8_t>();
        v.numSat = r.read_le<std::uint8_t>();
        v.latitude = r.read_le<std::int32_t>();
        v.longitude = r.read_le<std::int32_t>();
        v.altitude = r.read_le<std::int16_t>();
        v.speed = r.read_le<std::uint16_t>();
        v.groundCourse = r.read_le<std::uint16_t>();
        v.hdop = r.read_le<std::uint16_t>();
        return v;
    }
};

struct MSP_COMP_GPS__reply {
    std::uint16_t distanceToHome;
    std::uint16_t directionToHome;
    std::uint8_t gpsHeartbeat;

    static std::vector<std::uint8_t> pack(const MSP_COMP_GPS__reply& v) {
        BufferWriter w;
        w.write_le(v.distanceToHome);
        w.write_le(v.directionToHome);
        w.write_le(v.gpsHeartbeat);
        return std::move(w.buf);
    }

    static MSP_COMP_GPS__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_COMP_GPS__reply v{};
        v.distanceToHome = r.read_le<std::uint16_t>();
        v.directionToHome = r.read_le<std::uint16_t>();
        v.gpsHeartbeat = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP_ATTITUDE__reply {
    std::int16_t roll;
    std::int16_t pitch;
    std::int16_t yaw;

    static std::vector<std::uint8_t> pack(const MSP_ATTITUDE__reply& v) {
        BufferWriter w;
        w.write_le(v.roll);
        w.write_le(v.pitch);
        w.write_le(v.yaw);
        return std::move(w.buf);
    }

    static MSP_ATTITUDE__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_ATTITUDE__reply v{};
        v.roll = r.read_le<std::int16_t>();
        v.pitch = r.read_le<std::int16_t>();
        v.yaw = r.read_le<std::int16_t>();
        return v;
    }
};

struct MSP_ALTITUDE__reply {
    std::int32_t estimatedAltitude;
    std::int16_t variometer;
    std::int32_t baroAltitude;

    static std::vector<std::uint8_t> pack(const MSP_ALTITUDE__reply& v) {
        BufferWriter w;
        w.write_le(v.estimatedAltitude);
        w.write_le(v.variometer);
        w.write_le(v.baroAltitude);
        return std::move(w.buf);
    }

    static MSP_ALTITUDE__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_ALTITUDE__reply v{};
        v.estimatedAltitude = r.read_le<std::int32_t>();
        v.variometer = r.read_le<std::int16_t>();
        v.baroAltitude = r.read_le<std::int32_t>();
        return v;
    }
};

struct MSP_ANALOG__reply {
    std::uint8_t vbat;
    std::uint16_t mAhDrawn;
    std::uint16_t rssi;
    std::int16_t amperage;

    static std::vector<std::uint8_t> pack(const MSP_ANALOG__reply& v) {
        BufferWriter w;
        w.write_le(v.vbat);
        w.write_le(v.mAhDrawn);
        w.write_le(v.rssi);
        w.write_le(v.amperage);
        return std::move(w.buf);
    }

    static MSP_ANALOG__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_ANALOG__reply v{};
        v.vbat = r.read_le<std::uint8_t>();
        v.mAhDrawn = r.read_le<std::uint16_t>();
        v.rssi = r.read_le<std::uint16_t>();
        v.amperage = r.read_le<std::int16_t>();
        return v;
    }
};

struct MSP_RC_TUNING__reply {
    std::uint8_t legacyRcRate;
    std::uint8_t rcExpo;
    std::uint8_t rollRate;
    std::uint8_t pitchRate;
    std::uint8_t yawRate;
    std::uint8_t dynamicThrottlePID;
    std::uint8_t throttleMid;
    std::uint8_t throttleExpo;
    std::uint16_t tpaBreakpoint;
    std::uint8_t rcYawExpo;

    static std::vector<std::uint8_t> pack(const MSP_RC_TUNING__reply& v) {
        BufferWriter w;
        w.write_le(v.legacyRcRate);
        w.write_le(v.rcExpo);
        w.write_le(v.rollRate);
        w.write_le(v.pitchRate);
        w.write_le(v.yawRate);
        w.write_le(v.dynamicThrottlePID);
        w.write_le(v.throttleMid);
        w.write_le(v.throttleExpo);
        w.write_le(v.tpaBreakpoint);
        w.write_le(v.rcYawExpo);
        return std::move(w.buf);
    }

    static MSP_RC_TUNING__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_RC_TUNING__reply v{};
        v.legacyRcRate = r.read_le<std::uint8_t>();
        v.rcExpo = r.read_le<std::uint8_t>();
        v.rollRate = r.read_le<std::uint8_t>();
        v.pitchRate = r.read_le<std::uint8_t>();
        v.yawRate = r.read_le<std::uint8_t>();
        v.dynamicThrottlePID = r.read_le<std::uint8_t>();
        v.throttleMid = r.read_le<std::uint8_t>();
        v.throttleExpo = r.read_le<std::uint8_t>();
        v.tpaBreakpoint = r.read_le<std::uint16_t>();
        v.rcYawExpo = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP_ACTIVEBOXES__reply {
    boxBitmask_t activeModes;

    static std::vector<std::uint8_t> pack(const MSP_ACTIVEBOXES__reply& v) {
        BufferWriter w;
        { auto bytes = boxBitmask_t::pack(v.activeModes); w.write_bytes(bytes.data(), bytes.size()); }
        return std::move(w.buf);
    }

    static MSP_ACTIVEBOXES__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_ACTIVEBOXES__reply v{};
        static constexpr std::size_t __activeModes_size = []{ boxBitmask_t tmp{}; auto b = boxBitmask_t::pack(tmp); return b.size(); }();
        { std::vector<std::uint8_t> chunk(__activeModes_size); r.read_bytes(chunk.data(), __activeModes_size); v.activeModes = boxBitmask_t::unpack(chunk); }
        return v;
    }
};

struct MSP_MISC__reply {
    std::uint16_t midRc;
    std::uint16_t legacyMinThrottle;
    std::uint16_t maxThrottle;
    std::uint16_t minCommand;
    std::uint16_t failsafeThrottle;
    std::uint8_t gpsType;
    std::uint8_t legacyGpsBaud;
    std::uint8_t gpsSbasMode;
    std::uint8_t legacyMwCurrentOut;
    std::uint8_t rssiChannel;
    std::uint8_t reserved1;
    std::uint16_t magDeclination;
    std::uint8_t vbatScale;
    std::uint8_t vbatMinCell;
    std::uint8_t vbatMaxCell;
    std::uint8_t vbatWarningCell;

    static std::vector<std::uint8_t> pack(const MSP_MISC__reply& v) {
        BufferWriter w;
        w.write_le(v.midRc);
        w.write_le(v.legacyMinThrottle);
        w.write_le(v.maxThrottle);
        w.write_le(v.minCommand);
        w.write_le(v.failsafeThrottle);
        w.write_le(v.gpsType);
        w.write_le(v.legacyGpsBaud);
        w.write_le(v.gpsSbasMode);
        w.write_le(v.legacyMwCurrentOut);
        w.write_le(v.rssiChannel);
        w.write_le(v.reserved1);
        w.write_le(v.magDeclination);
        w.write_le(v.vbatScale);
        w.write_le(v.vbatMinCell);
        w.write_le(v.vbatMaxCell);
        w.write_le(v.vbatWarningCell);
        return std::move(w.buf);
    }

    static MSP_MISC__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_MISC__reply v{};
        v.midRc = r.read_le<std::uint16_t>();
        v.legacyMinThrottle = r.read_le<std::uint16_t>();
        v.maxThrottle = r.read_le<std::uint16_t>();
        v.minCommand = r.read_le<std::uint16_t>();
        v.failsafeThrottle = r.read_le<std::uint16_t>();
        v.gpsType = r.read_le<std::uint8_t>();
        v.legacyGpsBaud = r.read_le<std::uint8_t>();
        v.gpsSbasMode = r.read_le<std::uint8_t>();
        v.legacyMwCurrentOut = r.read_le<std::uint8_t>();
        v.rssiChannel = r.read_le<std::uint8_t>();
        v.reserved1 = r.read_le<std::uint8_t>();
        v.magDeclination = r.read_le<std::uint16_t>();
        v.vbatScale = r.read_le<std::uint8_t>();
        v.vbatMinCell = r.read_le<std::uint8_t>();
        v.vbatMaxCell = r.read_le<std::uint8_t>();
        v.vbatWarningCell = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP_BOXNAMES__reply {
    std::string boxNamesString;

    static std::vector<std::uint8_t> pack(const MSP_BOXNAMES__reply& v) {
        BufferWriter w;
        w.write_string_bytes(v.boxNamesString);
        return std::move(w.buf);
    }

    static MSP_BOXNAMES__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_BOXNAMES__reply v{};
        v.boxNamesString = r.read_string_rest();
        return v;
    }
};

struct MSP_PIDNAMES__reply {
    std::string pidNamesString;

    static std::vector<std::uint8_t> pack(const MSP_PIDNAMES__reply& v) {
        BufferWriter w;
        w.write_string_bytes(v.pidNamesString);
        return std::move(w.buf);
    }

    static MSP_PIDNAMES__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_PIDNAMES__reply v{};
        v.pidNamesString = r.read_string_rest();
        return v;
    }
};

struct MSP_WP__request {
    std::uint8_t waypointIndex;

    static std::vector<std::uint8_t> pack(const MSP_WP__request& v) {
        BufferWriter w;
        w.write_le(v.waypointIndex);
        return std::move(w.buf);
    }

    static MSP_WP__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_WP__request v{};
        v.waypointIndex = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP_WP__reply {
    std::uint8_t waypointIndex;
    std::uint8_t action;
    std::int32_t latitude;
    std::int32_t longitude;
    std::int32_t altitude;
    std::uint16_t param1;
    std::uint16_t param2;
    std::uint16_t param3;
    std::uint8_t flag;

    static std::vector<std::uint8_t> pack(const MSP_WP__reply& v) {
        BufferWriter w;
        w.write_le(v.waypointIndex);
        w.write_le(v.action);
        w.write_le(v.latitude);
        w.write_le(v.longitude);
        w.write_le(v.altitude);
        w.write_le(v.param1);
        w.write_le(v.param2);
        w.write_le(v.param3);
        w.write_le(v.flag);
        return std::move(w.buf);
    }

    static MSP_WP__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_WP__reply v{};
        v.waypointIndex = r.read_le<std::uint8_t>();
        v.action = r.read_le<std::uint8_t>();
        v.latitude = r.read_le<std::int32_t>();
        v.longitude = r.read_le<std::int32_t>();
        v.altitude = r.read_le<std::int32_t>();
        v.param1 = r.read_le<std::uint16_t>();
        v.param2 = r.read_le<std::uint16_t>();
        v.param3 = r.read_le<std::uint16_t>();
        v.flag = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP_BOXIDS__reply {
    std::vector<uint8_t> boxIds;

    static std::vector<std::uint8_t> pack(const MSP_BOXIDS__reply& v) {
        BufferWriter w;
        for (const auto& e : v.boxIds) w.write_le(e);
        return std::move(w.buf);
    }

    static MSP_BOXIDS__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_BOXIDS__reply v{};
        v.boxIds.clear();
        while (r.remaining() >= sizeof(uint8_t)) v.boxIds.push_back(r.read_le<uint8_t>());
        return v;
    }
};

struct MSP_SERVO_CONFIGURATIONS__reply {
    std::uint16_t min;
    std::uint16_t max;
    std::uint16_t middle;
    std::uint8_t rate;
    std::uint8_t reserved1;
    std::uint8_t reserved2;
    std::uint8_t legacyForwardChan;
    std::uint32_t legacyReversedSources;

    static std::vector<std::uint8_t> pack(const MSP_SERVO_CONFIGURATIONS__reply& v) {
        BufferWriter w;
        w.write_le(v.min);
        w.write_le(v.max);
        w.write_le(v.middle);
        w.write_le(v.rate);
        w.write_le(v.reserved1);
        w.write_le(v.reserved2);
        w.write_le(v.legacyForwardChan);
        w.write_le(v.legacyReversedSources);
        return std::move(w.buf);
    }

    static MSP_SERVO_CONFIGURATIONS__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_SERVO_CONFIGURATIONS__reply v{};
        v.min = r.read_le<std::uint16_t>();
        v.max = r.read_le<std::uint16_t>();
        v.middle = r.read_le<std::uint16_t>();
        v.rate = r.read_le<std::uint8_t>();
        v.reserved1 = r.read_le<std::uint8_t>();
        v.reserved2 = r.read_le<std::uint8_t>();
        v.legacyForwardChan = r.read_le<std::uint8_t>();
        v.legacyReversedSources = r.read_le<std::uint32_t>();
        return v;
    }
};

struct MSP_NAV_STATUS__reply {
    std::uint8_t navMode;
    std::uint8_t navState;
    std::uint8_t activeWpAction;
    std::uint8_t activeWpNumber;
    std::uint8_t navError;
    std::int16_t targetHeading;

    static std::vector<std::uint8_t> pack(const MSP_NAV_STATUS__reply& v) {
        BufferWriter w;
        w.write_le(v.navMode);
        w.write_le(v.navState);
        w.write_le(v.activeWpAction);
        w.write_le(v.activeWpNumber);
        w.write_le(v.navError);
        w.write_le(v.targetHeading);
        return std::move(w.buf);
    }

    static MSP_NAV_STATUS__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_NAV_STATUS__reply v{};
        v.navMode = r.read_le<std::uint8_t>();
        v.navState = r.read_le<std::uint8_t>();
        v.activeWpAction = r.read_le<std::uint8_t>();
        v.activeWpNumber = r.read_le<std::uint8_t>();
        v.navError = r.read_le<std::uint8_t>();
        v.targetHeading = r.read_le<std::int16_t>();
        return v;
    }
};

struct MSP_3D__reply {
    std::uint16_t deadbandLow;
    std::uint16_t deadbandHigh;
    std::uint16_t neutral;

    static std::vector<std::uint8_t> pack(const MSP_3D__reply& v) {
        BufferWriter w;
        w.write_le(v.deadbandLow);
        w.write_le(v.deadbandHigh);
        w.write_le(v.neutral);
        return std::move(w.buf);
    }

    static MSP_3D__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_3D__reply v{};
        v.deadbandLow = r.read_le<std::uint16_t>();
        v.deadbandHigh = r.read_le<std::uint16_t>();
        v.neutral = r.read_le<std::uint16_t>();
        return v;
    }
};

struct MSP_RC_DEADBAND__reply {
    std::uint8_t deadband;
    std::uint8_t yawDeadband;
    std::uint8_t altHoldDeadband;
    std::uint16_t throttleDeadband;

    static std::vector<std::uint8_t> pack(const MSP_RC_DEADBAND__reply& v) {
        BufferWriter w;
        w.write_le(v.deadband);
        w.write_le(v.yawDeadband);
        w.write_le(v.altHoldDeadband);
        w.write_le(v.throttleDeadband);
        return std::move(w.buf);
    }

    static MSP_RC_DEADBAND__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_RC_DEADBAND__reply v{};
        v.deadband = r.read_le<std::uint8_t>();
        v.yawDeadband = r.read_le<std::uint8_t>();
        v.altHoldDeadband = r.read_le<std::uint8_t>();
        v.throttleDeadband = r.read_le<std::uint16_t>();
        return v;
    }
};

struct MSP_SENSOR_ALIGNMENT__reply {
    std::uint8_t gyroAlign;
    std::uint8_t accAlign;
    std::uint8_t magAlign;
    std::uint8_t opflowAlign;

    static std::vector<std::uint8_t> pack(const MSP_SENSOR_ALIGNMENT__reply& v) {
        BufferWriter w;
        w.write_le(v.gyroAlign);
        w.write_le(v.accAlign);
        w.write_le(v.magAlign);
        w.write_le(v.opflowAlign);
        return std::move(w.buf);
    }

    static MSP_SENSOR_ALIGNMENT__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_SENSOR_ALIGNMENT__reply v{};
        v.gyroAlign = r.read_le<std::uint8_t>();
        v.accAlign = r.read_le<std::uint8_t>();
        v.magAlign = r.read_le<std::uint8_t>();
        v.opflowAlign = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP_LED_STRIP_MODECOLOR__reply {
    std::uint8_t modeIndex;
    std::uint8_t directionOrSpecialIndex;
    std::uint8_t colorIndex;

    static std::vector<std::uint8_t> pack(const MSP_LED_STRIP_MODECOLOR__reply& v) {
        BufferWriter w;
        w.write_le(v.modeIndex);
        w.write_le(v.directionOrSpecialIndex);
        w.write_le(v.colorIndex);
        return std::move(w.buf);
    }

    static MSP_LED_STRIP_MODECOLOR__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_LED_STRIP_MODECOLOR__reply v{};
        v.modeIndex = r.read_le<std::uint8_t>();
        v.directionOrSpecialIndex = r.read_le<std::uint8_t>();
        v.colorIndex = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP_BATTERY_STATE__reply {
    std::uint8_t cellCount;
    std::uint16_t capacity;
    std::uint8_t vbatScaled;
    std::uint16_t mAhDrawn;
    std::int16_t amperage;
    std::uint8_t batteryState;
    std::uint16_t vbatActual;

    static std::vector<std::uint8_t> pack(const MSP_BATTERY_STATE__reply& v) {
        BufferWriter w;
        w.write_le(v.cellCount);
        w.write_le(v.capacity);
        w.write_le(v.vbatScaled);
        w.write_le(v.mAhDrawn);
        w.write_le(v.amperage);
        w.write_le(v.batteryState);
        w.write_le(v.vbatActual);
        return std::move(w.buf);
    }

    static MSP_BATTERY_STATE__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_BATTERY_STATE__reply v{};
        v.cellCount = r.read_le<std::uint8_t>();
        v.capacity = r.read_le<std::uint16_t>();
        v.vbatScaled = r.read_le<std::uint8_t>();
        v.mAhDrawn = r.read_le<std::uint16_t>();
        v.amperage = r.read_le<std::int16_t>();
        v.batteryState = r.read_le<std::uint8_t>();
        v.vbatActual = r.read_le<std::uint16_t>();
        return v;
    }
};

struct MSP_VTXTABLE_POWERLEVEL__request {
    std::uint8_t powerLevelIndex;

    static std::vector<std::uint8_t> pack(const MSP_VTXTABLE_POWERLEVEL__request& v) {
        BufferWriter w;
        w.write_le(v.powerLevelIndex);
        return std::move(w.buf);
    }

    static MSP_VTXTABLE_POWERLEVEL__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_VTXTABLE_POWERLEVEL__request v{};
        v.powerLevelIndex = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP_VTXTABLE_POWERLEVEL__reply {
    std::uint8_t powerLevelIndex;
    std::uint16_t powerValue;
    std::uint8_t labelLength;
    std::string label;

    static std::vector<std::uint8_t> pack(const MSP_VTXTABLE_POWERLEVEL__reply& v) {
        BufferWriter w;
        w.write_le(v.powerLevelIndex);
        w.write_le(v.powerValue);
        w.write_le(v.labelLength);
        w.write_string_bytes(v.label);
        return std::move(w.buf);
    }

    static MSP_VTXTABLE_POWERLEVEL__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_VTXTABLE_POWERLEVEL__reply v{};
        v.powerLevelIndex = r.read_le<std::uint8_t>();
        v.powerValue = r.read_le<std::uint16_t>();
        v.labelLength = r.read_le<std::uint8_t>();
        v.label = r.read_string_rest();
        return v;
    }
};

struct MSP_STATUS_EX__reply {
    std::uint16_t cycleTime;
    std::uint16_t i2cErrors;
    std::uint16_t sensorStatus;
    std::uint32_t activeModesLow;
    std::uint8_t profile;
    std::uint16_t cpuLoad;
    std::uint16_t armingFlags;
    std::uint8_t accCalibAxisFlags;

    static std::vector<std::uint8_t> pack(const MSP_STATUS_EX__reply& v) {
        BufferWriter w;
        w.write_le(v.cycleTime);
        w.write_le(v.i2cErrors);
        w.write_le(v.sensorStatus);
        w.write_le(v.activeModesLow);
        w.write_le(v.profile);
        w.write_le(v.cpuLoad);
        w.write_le(v.armingFlags);
        w.write_le(v.accCalibAxisFlags);
        return std::move(w.buf);
    }

    static MSP_STATUS_EX__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_STATUS_EX__reply v{};
        v.cycleTime = r.read_le<std::uint16_t>();
        v.i2cErrors = r.read_le<std::uint16_t>();
        v.sensorStatus = r.read_le<std::uint16_t>();
        v.activeModesLow = r.read_le<std::uint32_t>();
        v.profile = r.read_le<std::uint8_t>();
        v.cpuLoad = r.read_le<std::uint16_t>();
        v.armingFlags = r.read_le<std::uint16_t>();
        v.accCalibAxisFlags = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP_SENSOR_STATUS__reply {
    std::uint8_t overallHealth;
    std::uint8_t gyroStatus;
    std::uint8_t accStatus;
    std::uint8_t magStatus;
    std::uint8_t baroStatus;
    std::uint8_t gpsStatus;
    std::uint8_t rangefinderStatus;
    std::uint8_t pitotStatus;
    std::uint8_t opflowStatus;

    static std::vector<std::uint8_t> pack(const MSP_SENSOR_STATUS__reply& v) {
        BufferWriter w;
        w.write_le(v.overallHealth);
        w.write_le(v.gyroStatus);
        w.write_le(v.accStatus);
        w.write_le(v.magStatus);
        w.write_le(v.baroStatus);
        w.write_le(v.gpsStatus);
        w.write_le(v.rangefinderStatus);
        w.write_le(v.pitotStatus);
        w.write_le(v.opflowStatus);
        return std::move(w.buf);
    }

    static MSP_SENSOR_STATUS__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_SENSOR_STATUS__reply v{};
        v.overallHealth = r.read_le<std::uint8_t>();
        v.gyroStatus = r.read_le<std::uint8_t>();
        v.accStatus = r.read_le<std::uint8_t>();
        v.magStatus = r.read_le<std::uint8_t>();
        v.baroStatus = r.read_le<std::uint8_t>();
        v.gpsStatus = r.read_le<std::uint8_t>();
        v.rangefinderStatus = r.read_le<std::uint8_t>();
        v.pitotStatus = r.read_le<std::uint8_t>();
        v.opflowStatus = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP_UID__reply {
    std::uint32_t uid0;
    std::uint32_t uid1;
    std::uint32_t uid2;

    static std::vector<std::uint8_t> pack(const MSP_UID__reply& v) {
        BufferWriter w;
        w.write_le(v.uid0);
        w.write_le(v.uid1);
        w.write_le(v.uid2);
        return std::move(w.buf);
    }

    static MSP_UID__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_UID__reply v{};
        v.uid0 = r.read_le<std::uint32_t>();
        v.uid1 = r.read_le<std::uint32_t>();
        v.uid2 = r.read_le<std::uint32_t>();
        return v;
    }
};

struct MSP_GPSSVINFO__reply {
    std::uint8_t protocolVersion;
    std::uint8_t numChannels;
    std::uint8_t hdopHundreds;
    std::uint8_t hdopUnits;

    static std::vector<std::uint8_t> pack(const MSP_GPSSVINFO__reply& v) {
        BufferWriter w;
        w.write_le(v.protocolVersion);
        w.write_le(v.numChannels);
        w.write_le(v.hdopHundreds);
        w.write_le(v.hdopUnits);
        return std::move(w.buf);
    }

    static MSP_GPSSVINFO__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_GPSSVINFO__reply v{};
        v.protocolVersion = r.read_le<std::uint8_t>();
        v.numChannels = r.read_le<std::uint8_t>();
        v.hdopHundreds = r.read_le<std::uint8_t>();
        v.hdopUnits = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP_GPSSTATISTICS__reply {
    std::uint16_t lastMessageDt;
    std::uint32_t errors;
    std::uint32_t timeouts;
    std::uint32_t packetCount;
    std::uint16_t hdop;
    std::uint16_t eph;
    std::uint16_t epv;

    static std::vector<std::uint8_t> pack(const MSP_GPSSTATISTICS__reply& v) {
        BufferWriter w;
        w.write_le(v.lastMessageDt);
        w.write_le(v.errors);
        w.write_le(v.timeouts);
        w.write_le(v.packetCount);
        w.write_le(v.hdop);
        w.write_le(v.eph);
        w.write_le(v.epv);
        return std::move(w.buf);
    }

    static MSP_GPSSTATISTICS__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_GPSSTATISTICS__reply v{};
        v.lastMessageDt = r.read_le<std::uint16_t>();
        v.errors = r.read_le<std::uint32_t>();
        v.timeouts = r.read_le<std::uint32_t>();
        v.packetCount = r.read_le<std::uint32_t>();
        v.hdop = r.read_le<std::uint16_t>();
        v.eph = r.read_le<std::uint16_t>();
        v.epv = r.read_le<std::uint16_t>();
        return v;
    }
};

struct MSP_SET_TX_INFO__request {
    std::uint8_t rssi;

    static std::vector<std::uint8_t> pack(const MSP_SET_TX_INFO__request& v) {
        BufferWriter w;
        w.write_le(v.rssi);
        return std::move(w.buf);
    }

    static MSP_SET_TX_INFO__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_SET_TX_INFO__request v{};
        v.rssi = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP_TX_INFO__reply {
    std::uint8_t rssiSource;
    std::uint8_t rtcDateTimeIsSet;

    static std::vector<std::uint8_t> pack(const MSP_TX_INFO__reply& v) {
        BufferWriter w;
        w.write_le(v.rssiSource);
        w.write_le(v.rtcDateTimeIsSet);
        return std::move(w.buf);
    }

    static MSP_TX_INFO__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_TX_INFO__reply v{};
        v.rssiSource = r.read_le<std::uint8_t>();
        v.rtcDateTimeIsSet = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP_SET_RAW_RC__request {
    std::vector<uint16_t> rcChannels;

    static std::vector<std::uint8_t> pack(const MSP_SET_RAW_RC__request& v) {
        BufferWriter w;
        for (const auto& e : v.rcChannels) w.write_le(e);
        return std::move(w.buf);
    }

    static MSP_SET_RAW_RC__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_SET_RAW_RC__request v{};
        v.rcChannels.clear();
        while (r.remaining() >= sizeof(uint16_t)) v.rcChannels.push_back(r.read_le<uint16_t>());
        return v;
    }
};

struct MSP_SET_RAW_GPS__request {
    std::uint8_t fixType;
    std::uint8_t numSat;
    std::int32_t latitude;
    std::int32_t longitude;
    std::int16_t altitude;
    std::uint16_t speed;
    std::uint16_t groundCourse;

    static std::vector<std::uint8_t> pack(const MSP_SET_RAW_GPS__request& v) {
        BufferWriter w;
        w.write_le(v.fixType);
        w.write_le(v.numSat);
        w.write_le(v.latitude);
        w.write_le(v.longitude);
        w.write_le(v.altitude);
        w.write_le(v.speed);
        w.write_le(v.groundCourse);
        return std::move(w.buf);
    }

    static MSP_SET_RAW_GPS__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_SET_RAW_GPS__request v{};
        v.fixType = r.read_le<std::uint8_t>();
        v.numSat = r.read_le<std::uint8_t>();
        v.latitude = r.read_le<std::int32_t>();
        v.longitude = r.read_le<std::int32_t>();
        v.altitude = r.read_le<std::int16_t>();
        v.speed = r.read_le<std::uint16_t>();
        v.groundCourse = r.read_le<std::uint16_t>();
        return v;
    }
};

struct MSP_SET_RC_TUNING__request {
    std::uint8_t legacyRcRate;
    std::uint8_t rcExpo;
    std::uint8_t rollRate;
    std::uint8_t pitchRate;
    std::uint8_t yawRate;
    std::uint8_t dynamicThrottlePID;
    std::uint8_t throttleMid;
    std::uint8_t throttleExpo;
    std::uint16_t tpaBreakpoint;
    std::uint8_t rcYawExpo;

    static std::vector<std::uint8_t> pack(const MSP_SET_RC_TUNING__request& v) {
        BufferWriter w;
        w.write_le(v.legacyRcRate);
        w.write_le(v.rcExpo);
        w.write_le(v.rollRate);
        w.write_le(v.pitchRate);
        w.write_le(v.yawRate);
        w.write_le(v.dynamicThrottlePID);
        w.write_le(v.throttleMid);
        w.write_le(v.throttleExpo);
        w.write_le(v.tpaBreakpoint);
        w.write_le(v.rcYawExpo);
        return std::move(w.buf);
    }

    static MSP_SET_RC_TUNING__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_SET_RC_TUNING__request v{};
        v.legacyRcRate = r.read_le<std::uint8_t>();
        v.rcExpo = r.read_le<std::uint8_t>();
        v.rollRate = r.read_le<std::uint8_t>();
        v.pitchRate = r.read_le<std::uint8_t>();
        v.yawRate = r.read_le<std::uint8_t>();
        v.dynamicThrottlePID = r.read_le<std::uint8_t>();
        v.throttleMid = r.read_le<std::uint8_t>();
        v.throttleExpo = r.read_le<std::uint8_t>();
        v.tpaBreakpoint = r.read_le<std::uint16_t>();
        v.rcYawExpo = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP_SET_MISC__request {
    std::uint16_t midRc;
    std::uint16_t legacyMinThrottle;
    std::uint16_t legacyMaxThrottle;
    std::uint16_t minCommand;
    std::uint16_t failsafeThrottle;
    std::uint8_t gpsType;
    std::uint8_t legacyGpsBaud;
    std::uint8_t gpsSbasMode;
    std::uint8_t legacyMwCurrentOut;
    std::uint8_t rssiChannel;
    std::uint8_t reserved1;
    std::uint16_t magDeclination;
    std::uint8_t vbatScale;
    std::uint8_t vbatMinCell;
    std::uint8_t vbatMaxCell;
    std::uint8_t vbatWarningCell;

    static std::vector<std::uint8_t> pack(const MSP_SET_MISC__request& v) {
        BufferWriter w;
        w.write_le(v.midRc);
        w.write_le(v.legacyMinThrottle);
        w.write_le(v.legacyMaxThrottle);
        w.write_le(v.minCommand);
        w.write_le(v.failsafeThrottle);
        w.write_le(v.gpsType);
        w.write_le(v.legacyGpsBaud);
        w.write_le(v.gpsSbasMode);
        w.write_le(v.legacyMwCurrentOut);
        w.write_le(v.rssiChannel);
        w.write_le(v.reserved1);
        w.write_le(v.magDeclination);
        w.write_le(v.vbatScale);
        w.write_le(v.vbatMinCell);
        w.write_le(v.vbatMaxCell);
        w.write_le(v.vbatWarningCell);
        return std::move(w.buf);
    }

    static MSP_SET_MISC__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_SET_MISC__request v{};
        v.midRc = r.read_le<std::uint16_t>();
        v.legacyMinThrottle = r.read_le<std::uint16_t>();
        v.legacyMaxThrottle = r.read_le<std::uint16_t>();
        v.minCommand = r.read_le<std::uint16_t>();
        v.failsafeThrottle = r.read_le<std::uint16_t>();
        v.gpsType = r.read_le<std::uint8_t>();
        v.legacyGpsBaud = r.read_le<std::uint8_t>();
        v.gpsSbasMode = r.read_le<std::uint8_t>();
        v.legacyMwCurrentOut = r.read_le<std::uint8_t>();
        v.rssiChannel = r.read_le<std::uint8_t>();
        v.reserved1 = r.read_le<std::uint8_t>();
        v.magDeclination = r.read_le<std::uint16_t>();
        v.vbatScale = r.read_le<std::uint8_t>();
        v.vbatMinCell = r.read_le<std::uint8_t>();
        v.vbatMaxCell = r.read_le<std::uint8_t>();
        v.vbatWarningCell = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP_SET_WP__request {
    std::uint8_t waypointIndex;
    std::uint8_t action;
    std::int32_t latitude;
    std::int32_t longitude;
    std::int32_t altitude;
    std::uint16_t param1;
    std::uint16_t param2;
    std::uint16_t param3;
    std::uint8_t flag;

    static std::vector<std::uint8_t> pack(const MSP_SET_WP__request& v) {
        BufferWriter w;
        w.write_le(v.waypointIndex);
        w.write_le(v.action);
        w.write_le(v.latitude);
        w.write_le(v.longitude);
        w.write_le(v.altitude);
        w.write_le(v.param1);
        w.write_le(v.param2);
        w.write_le(v.param3);
        w.write_le(v.flag);
        return std::move(w.buf);
    }

    static MSP_SET_WP__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_SET_WP__request v{};
        v.waypointIndex = r.read_le<std::uint8_t>();
        v.action = r.read_le<std::uint8_t>();
        v.latitude = r.read_le<std::int32_t>();
        v.longitude = r.read_le<std::int32_t>();
        v.altitude = r.read_le<std::int32_t>();
        v.param1 = r.read_le<std::uint16_t>();
        v.param2 = r.read_le<std::uint16_t>();
        v.param3 = r.read_le<std::uint16_t>();
        v.flag = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP_SELECT_SETTING__request {
    std::uint8_t profileIndex;

    static std::vector<std::uint8_t> pack(const MSP_SELECT_SETTING__request& v) {
        BufferWriter w;
        w.write_le(v.profileIndex);
        return std::move(w.buf);
    }

    static MSP_SELECT_SETTING__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_SELECT_SETTING__request v{};
        v.profileIndex = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP_SET_HEAD__request {
    std::int16_t heading;

    static std::vector<std::uint8_t> pack(const MSP_SET_HEAD__request& v) {
        BufferWriter w;
        w.write_le(v.heading);
        return std::move(w.buf);
    }

    static MSP_SET_HEAD__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_SET_HEAD__request v{};
        v.heading = r.read_le<std::int16_t>();
        return v;
    }
};

struct MSP_SET_SERVO_CONFIGURATION__request {
    std::uint8_t servoIndex;
    std::uint16_t min;
    std::uint16_t max;
    std::uint16_t middle;
    std::uint8_t rate;
    std::uint8_t reserved1;
    std::uint8_t reserved2;
    std::uint8_t legacyForwardChan;
    std::uint32_t legacyReversedSources;

    static std::vector<std::uint8_t> pack(const MSP_SET_SERVO_CONFIGURATION__request& v) {
        BufferWriter w;
        w.write_le(v.servoIndex);
        w.write_le(v.min);
        w.write_le(v.max);
        w.write_le(v.middle);
        w.write_le(v.rate);
        w.write_le(v.reserved1);
        w.write_le(v.reserved2);
        w.write_le(v.legacyForwardChan);
        w.write_le(v.legacyReversedSources);
        return std::move(w.buf);
    }

    static MSP_SET_SERVO_CONFIGURATION__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_SET_SERVO_CONFIGURATION__request v{};
        v.servoIndex = r.read_le<std::uint8_t>();
        v.min = r.read_le<std::uint16_t>();
        v.max = r.read_le<std::uint16_t>();
        v.middle = r.read_le<std::uint16_t>();
        v.rate = r.read_le<std::uint8_t>();
        v.reserved1 = r.read_le<std::uint8_t>();
        v.reserved2 = r.read_le<std::uint8_t>();
        v.legacyForwardChan = r.read_le<std::uint8_t>();
        v.legacyReversedSources = r.read_le<std::uint32_t>();
        return v;
    }
};

struct MSP_SET_MOTOR__request {
    std::array<std::uint16_t,8> motorValues;

    static std::vector<std::uint8_t> pack(const MSP_SET_MOTOR__request& v) {
        BufferWriter w;
        for (const auto& e : v.motorValues) w.write_le(e);
        return std::move(w.buf);
    }

    static MSP_SET_MOTOR__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_SET_MOTOR__request v{};
        for (auto& e : v.motorValues) e = r.read_le<decltype(e)>();
        return v;
    }
};

struct MSP_SET_3D__request {
    std::uint16_t deadbandLow;
    std::uint16_t deadbandHigh;
    std::uint16_t neutral;

    static std::vector<std::uint8_t> pack(const MSP_SET_3D__request& v) {
        BufferWriter w;
        w.write_le(v.deadbandLow);
        w.write_le(v.deadbandHigh);
        w.write_le(v.neutral);
        return std::move(w.buf);
    }

    static MSP_SET_3D__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_SET_3D__request v{};
        v.deadbandLow = r.read_le<std::uint16_t>();
        v.deadbandHigh = r.read_le<std::uint16_t>();
        v.neutral = r.read_le<std::uint16_t>();
        return v;
    }
};

struct MSP_SET_RC_DEADBAND__request {
    std::uint8_t deadband;
    std::uint8_t yawDeadband;
    std::uint8_t altHoldDeadband;
    std::uint16_t throttleDeadband;

    static std::vector<std::uint8_t> pack(const MSP_SET_RC_DEADBAND__request& v) {
        BufferWriter w;
        w.write_le(v.deadband);
        w.write_le(v.yawDeadband);
        w.write_le(v.altHoldDeadband);
        w.write_le(v.throttleDeadband);
        return std::move(w.buf);
    }

    static MSP_SET_RC_DEADBAND__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_SET_RC_DEADBAND__request v{};
        v.deadband = r.read_le<std::uint8_t>();
        v.yawDeadband = r.read_le<std::uint8_t>();
        v.altHoldDeadband = r.read_le<std::uint8_t>();
        v.throttleDeadband = r.read_le<std::uint16_t>();
        return v;
    }
};

struct MSP_SET_SENSOR_ALIGNMENT__request {
    std::uint8_t gyroAlign;
    std::uint8_t accAlign;
    std::uint8_t magAlign;
    std::uint8_t opflowAlign;

    static std::vector<std::uint8_t> pack(const MSP_SET_SENSOR_ALIGNMENT__request& v) {
        BufferWriter w;
        w.write_le(v.gyroAlign);
        w.write_le(v.accAlign);
        w.write_le(v.magAlign);
        w.write_le(v.opflowAlign);
        return std::move(w.buf);
    }

    static MSP_SET_SENSOR_ALIGNMENT__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_SET_SENSOR_ALIGNMENT__request v{};
        v.gyroAlign = r.read_le<std::uint8_t>();
        v.accAlign = r.read_le<std::uint8_t>();
        v.magAlign = r.read_le<std::uint8_t>();
        v.opflowAlign = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP_SET_LED_STRIP_MODECOLOR__request {
    std::uint8_t modeIndex;
    std::uint8_t directionOrSpecialIndex;
    std::uint8_t colorIndex;

    static std::vector<std::uint8_t> pack(const MSP_SET_LED_STRIP_MODECOLOR__request& v) {
        BufferWriter w;
        w.write_le(v.modeIndex);
        w.write_le(v.directionOrSpecialIndex);
        w.write_le(v.colorIndex);
        return std::move(w.buf);
    }

    static MSP_SET_LED_STRIP_MODECOLOR__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_SET_LED_STRIP_MODECOLOR__request v{};
        v.modeIndex = r.read_le<std::uint8_t>();
        v.directionOrSpecialIndex = r.read_le<std::uint8_t>();
        v.colorIndex = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP_SERVO_MIX_RULES__reply {
    std::uint8_t targetChannel;
    std::uint8_t inputSource;
    std::uint16_t rate;
    std::uint8_t speed;
    std::uint8_t reserved1;
    std::uint8_t legacyMax;
    std::uint8_t legacyBox;

    static std::vector<std::uint8_t> pack(const MSP_SERVO_MIX_RULES__reply& v) {
        BufferWriter w;
        w.write_le(v.targetChannel);
        w.write_le(v.inputSource);
        w.write_le(v.rate);
        w.write_le(v.speed);
        w.write_le(v.reserved1);
        w.write_le(v.legacyMax);
        w.write_le(v.legacyBox);
        return std::move(w.buf);
    }

    static MSP_SERVO_MIX_RULES__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_SERVO_MIX_RULES__reply v{};
        v.targetChannel = r.read_le<std::uint8_t>();
        v.inputSource = r.read_le<std::uint8_t>();
        v.rate = r.read_le<std::uint16_t>();
        v.speed = r.read_le<std::uint8_t>();
        v.reserved1 = r.read_le<std::uint8_t>();
        v.legacyMax = r.read_le<std::uint8_t>();
        v.legacyBox = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP_SET_SERVO_MIX_RULE__request {
    std::uint8_t ruleIndex;
    std::uint8_t targetChannel;
    std::uint8_t inputSource;
    std::uint16_t rate;
    std::uint8_t speed;
    std::uint16_t legacyMinMax;
    std::uint8_t legacyBox;

    static std::vector<std::uint8_t> pack(const MSP_SET_SERVO_MIX_RULE__request& v) {
        BufferWriter w;
        w.write_le(v.ruleIndex);
        w.write_le(v.targetChannel);
        w.write_le(v.inputSource);
        w.write_le(v.rate);
        w.write_le(v.speed);
        w.write_le(v.legacyMinMax);
        w.write_le(v.legacyBox);
        return std::move(w.buf);
    }

    static MSP_SET_SERVO_MIX_RULE__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_SET_SERVO_MIX_RULE__request v{};
        v.ruleIndex = r.read_le<std::uint8_t>();
        v.targetChannel = r.read_le<std::uint8_t>();
        v.inputSource = r.read_le<std::uint8_t>();
        v.rate = r.read_le<std::uint16_t>();
        v.speed = r.read_le<std::uint8_t>();
        v.legacyMinMax = r.read_le<std::uint16_t>();
        v.legacyBox = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP_SET_PASSTHROUGH__reply {
    std::uint8_t status;

    static std::vector<std::uint8_t> pack(const MSP_SET_PASSTHROUGH__reply& v) {
        BufferWriter w;
        w.write_le(v.status);
        return std::move(w.buf);
    }

    static MSP_SET_PASSTHROUGH__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_SET_PASSTHROUGH__reply v{};
        v.status = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP_RTC__reply {
    std::int32_t seconds;
    std::uint16_t millis;

    static std::vector<std::uint8_t> pack(const MSP_RTC__reply& v) {
        BufferWriter w;
        w.write_le(v.seconds);
        w.write_le(v.millis);
        return std::move(w.buf);
    }

    static MSP_RTC__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_RTC__reply v{};
        v.seconds = r.read_le<std::int32_t>();
        v.millis = r.read_le<std::uint16_t>();
        return v;
    }
};

struct MSP_SET_RTC__request {
    std::int32_t seconds;
    std::uint16_t millis;

    static std::vector<std::uint8_t> pack(const MSP_SET_RTC__request& v) {
        BufferWriter w;
        w.write_le(v.seconds);
        w.write_le(v.millis);
        return std::move(w.buf);
    }

    static MSP_SET_RTC__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_SET_RTC__request v{};
        v.seconds = r.read_le<std::int32_t>();
        v.millis = r.read_le<std::uint16_t>();
        return v;
    }
};

struct MSP_DEBUGMSG__reply {
    std::string Message_Text;

    static std::vector<std::uint8_t> pack(const MSP_DEBUGMSG__reply& v) {
        BufferWriter w;
        w.write_string_bytes(v.Message_Text);
        return std::move(w.buf);
    }

    static MSP_DEBUGMSG__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_DEBUGMSG__reply v{};
        v.Message_Text = r.read_string_rest();
        return v;
    }
};

struct MSP_DEBUG__reply {
    std::array<std::uint16_t,4> debugValues;

    static std::vector<std::uint8_t> pack(const MSP_DEBUG__reply& v) {
        BufferWriter w;
        for (const auto& e : v.debugValues) w.write_le(e);
        return std::move(w.buf);
    }

    static MSP_DEBUG__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_DEBUG__reply v{};
        for (auto& e : v.debugValues) e = r.read_le<decltype(e)>();
        return v;
    }
};

struct MSP2_COMMON_TZ__reply {
    std::int16_t tzOffsetMinutes;
    std::uint8_t tzAutoDst;

    static std::vector<std::uint8_t> pack(const MSP2_COMMON_TZ__reply& v) {
        BufferWriter w;
        w.write_le(v.tzOffsetMinutes);
        w.write_le(v.tzAutoDst);
        return std::move(w.buf);
    }

    static MSP2_COMMON_TZ__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_COMMON_TZ__reply v{};
        v.tzOffsetMinutes = r.read_le<std::int16_t>();
        v.tzAutoDst = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP2_COMMON_SET_TZ__dataSize____2 {
    std::int16_t tz_offset;

    static std::vector<std::uint8_t> pack(const MSP2_COMMON_SET_TZ__dataSize____2& v) {
        BufferWriter w;
        w.write_le(v.tz_offset);
        return std::move(w.buf);
    }

    static MSP2_COMMON_SET_TZ__dataSize____2 unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_COMMON_SET_TZ__dataSize____2 v{};
        v.tz_offset = r.read_le<std::int16_t>();
        return v;
    }
};

struct MSP2_COMMON_SET_TZ__dataSize____3 {
    std::int16_t tz_offset;
    std::uint8_t tz_automatic_dst;

    static std::vector<std::uint8_t> pack(const MSP2_COMMON_SET_TZ__dataSize____3& v) {
        BufferWriter w;
        w.write_le(v.tz_offset);
        w.write_le(v.tz_automatic_dst);
        return std::move(w.buf);
    }

    static MSP2_COMMON_SET_TZ__dataSize____3 unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_COMMON_SET_TZ__dataSize____3 v{};
        v.tz_offset = r.read_le<std::int16_t>();
        v.tz_automatic_dst = r.read_le<std::uint8_t>();
        return v;
    }
};

using MSP2_COMMON_SET_TZ_variant = std::variant<MSP2_COMMON_SET_TZ__dataSize____2, MSP2_COMMON_SET_TZ__dataSize____3>;
inline MSP2_COMMON_SET_TZ_variant unpack_MSP2_COMMON_SET_TZ(const std::vector<std::uint8_t>& payload) {
    switch (payload.size()) {
    case 2: return MSP2_COMMON_SET_TZ__dataSize____2::unpack(payload);
    case 3: return MSP2_COMMON_SET_TZ__dataSize____3::unpack(payload);
    default:
        return MSP2_COMMON_SET_TZ__dataSize____2::unpack(payload);
    }
}

struct MSP2_COMMON_SETTING__request {
    std::string settingName;

    static std::vector<std::uint8_t> pack(const MSP2_COMMON_SETTING__request& v) {
        BufferWriter w;
        w.write_string_bytes(v.settingName);
        return std::move(w.buf);
    }

    static MSP2_COMMON_SETTING__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_COMMON_SETTING__request v{};
        v.settingName = r.read_string_rest();
        return v;
    }
};

struct MSP2_COMMON_SETTING__reply {
    std::vector<uint8_t> settingValue;

    static std::vector<std::uint8_t> pack(const MSP2_COMMON_SETTING__reply& v) {
        BufferWriter w;
        for (const auto& e : v.settingValue) w.write_le(e);
        return std::move(w.buf);
    }

    static MSP2_COMMON_SETTING__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_COMMON_SETTING__reply v{};
        v.settingValue.clear();
        while (r.remaining() >= sizeof(uint8_t)) v.settingValue.push_back(r.read_le<uint8_t>());
        return v;
    }
};

struct MSP2_COMMON_SET_SETTING__request {
    Varies settingIdentifier;
    std::vector<uint8_t> settingValue;

    static std::vector<std::uint8_t> pack(const MSP2_COMMON_SET_SETTING__request& v) {
        BufferWriter w;
        { auto bytes = Varies::pack(v.settingIdentifier); w.write_bytes(bytes.data(), bytes.size()); }
        for (const auto& e : v.settingValue) w.write_le(e);
        return std::move(w.buf);
    }

    static MSP2_COMMON_SET_SETTING__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_COMMON_SET_SETTING__request v{};
        static constexpr std::size_t __settingIdentifier_size = []{ Varies tmp{}; auto b = Varies::pack(tmp); return b.size(); }();
        { std::vector<std::uint8_t> chunk(__settingIdentifier_size); r.read_bytes(chunk.data(), __settingIdentifier_size); v.settingIdentifier = Varies::unpack(chunk); }
        v.settingValue.clear();
        while (r.remaining() >= sizeof(uint8_t)) v.settingValue.push_back(r.read_le<uint8_t>());
        return v;
    }
};

struct MSP2_COMMON_MOTOR_MIXER__reply {
    std::uint16_t throttleWeight;
    std::uint16_t rollWeight;
    std::uint16_t pitchWeight;
    std::uint16_t yawWeight;
    std::uint16_t throttleWeight;
    std::uint16_t rollWeight;
    std::uint16_t pitchWeight;
    std::uint16_t yawWeight;

    static std::vector<std::uint8_t> pack(const MSP2_COMMON_MOTOR_MIXER__reply& v) {
        BufferWriter w;
        w.write_le(v.throttleWeight);
        w.write_le(v.rollWeight);
        w.write_le(v.pitchWeight);
        w.write_le(v.yawWeight);
        w.write_le(v.throttleWeight);
        w.write_le(v.rollWeight);
        w.write_le(v.pitchWeight);
        w.write_le(v.yawWeight);
        return std::move(w.buf);
    }

    static MSP2_COMMON_MOTOR_MIXER__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_COMMON_MOTOR_MIXER__reply v{};
        v.throttleWeight = r.read_le<std::uint16_t>();
        v.rollWeight = r.read_le<std::uint16_t>();
        v.pitchWeight = r.read_le<std::uint16_t>();
        v.yawWeight = r.read_le<std::uint16_t>();
        v.throttleWeight = r.read_le<std::uint16_t>();
        v.rollWeight = r.read_le<std::uint16_t>();
        v.pitchWeight = r.read_le<std::uint16_t>();
        v.yawWeight = r.read_le<std::uint16_t>();
        return v;
    }
};

struct MSP2_COMMON_SET_MOTOR_MIXER__request {
    std::uint8_t motorIndex;
    std::uint16_t throttleWeight;
    std::uint16_t rollWeight;
    std::uint16_t pitchWeight;
    std::uint16_t yawWeight;

    static std::vector<std::uint8_t> pack(const MSP2_COMMON_SET_MOTOR_MIXER__request& v) {
        BufferWriter w;
        w.write_le(v.motorIndex);
        w.write_le(v.throttleWeight);
        w.write_le(v.rollWeight);
        w.write_le(v.pitchWeight);
        w.write_le(v.yawWeight);
        return std::move(w.buf);
    }

    static MSP2_COMMON_SET_MOTOR_MIXER__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_COMMON_SET_MOTOR_MIXER__request v{};
        v.motorIndex = r.read_le<std::uint8_t>();
        v.throttleWeight = r.read_le<std::uint16_t>();
        v.rollWeight = r.read_le<std::uint16_t>();
        v.pitchWeight = r.read_le<std::uint16_t>();
        v.yawWeight = r.read_le<std::uint16_t>();
        return v;
    }
};

struct MSP2_COMMON_SETTING_INFO__reply {
    std::string settingName;
    std::uint16_t pgn;
    std::uint8_t type;
    std::uint8_t section;
    std::uint8_t mode;
    std::int32_t minValue;
    std::uint32_t maxValue;
    std::uint16_t settingIndex;
    std::uint8_t profileIndex;
    std::uint8_t profileCount;
    std::string lookupNames;
    std::vector<uint8_t> settingValue;

    static std::vector<std::uint8_t> pack(const MSP2_COMMON_SETTING_INFO__reply& v) {
        BufferWriter w;
        w.write_string_bytes(v.settingName);
        w.write_le(v.pgn);
        w.write_le(v.type);
        w.write_le(v.section);
        w.write_le(v.mode);
        w.write_le(v.minValue);
        w.write_le(v.maxValue);
        w.write_le(v.settingIndex);
        w.write_le(v.profileIndex);
        w.write_le(v.profileCount);
        w.write_string_bytes(v.lookupNames);
        for (const auto& e : v.settingValue) w.write_le(e);
        return std::move(w.buf);
    }

    static MSP2_COMMON_SETTING_INFO__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_COMMON_SETTING_INFO__reply v{};
        v.settingName = r.read_string_rest();
        v.pgn = r.read_le<std::uint16_t>();
        v.type = r.read_le<std::uint8_t>();
        v.section = r.read_le<std::uint8_t>();
        v.mode = r.read_le<std::uint8_t>();
        v.minValue = r.read_le<std::int32_t>();
        v.maxValue = r.read_le<std::uint32_t>();
        v.settingIndex = r.read_le<std::uint16_t>();
        v.profileIndex = r.read_le<std::uint8_t>();
        v.profileCount = r.read_le<std::uint8_t>();
        v.lookupNames = r.read_string_rest();
        v.settingValue.clear();
        while (r.remaining() >= sizeof(uint8_t)) v.settingValue.push_back(r.read_le<uint8_t>());
        return v;
    }
};

struct MSP2_COMMON_PG_LIST__request {
    std::uint16_t pgn;

    static std::vector<std::uint8_t> pack(const MSP2_COMMON_PG_LIST__request& v) {
        BufferWriter w;
        w.write_le(v.pgn);
        return std::move(w.buf);
    }

    static MSP2_COMMON_PG_LIST__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_COMMON_PG_LIST__request v{};
        v.pgn = r.read_le<std::uint16_t>();
        return v;
    }
};

struct MSP2_COMMON_PG_LIST__reply {
    std::uint16_t pgn;
    std::uint16_t startIndex;
    std::uint16_t endIndex;

    static std::vector<std::uint8_t> pack(const MSP2_COMMON_PG_LIST__reply& v) {
        BufferWriter w;
        w.write_le(v.pgn);
        w.write_le(v.startIndex);
        w.write_le(v.endIndex);
        return std::move(w.buf);
    }

    static MSP2_COMMON_PG_LIST__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_COMMON_PG_LIST__reply v{};
        v.pgn = r.read_le<std::uint16_t>();
        v.startIndex = r.read_le<std::uint16_t>();
        v.endIndex = r.read_le<std::uint16_t>();
        return v;
    }
};

struct MSP2_COMMON_SERIAL_CONFIG__reply {
    std::uint8_t identifier;
    std::uint32_t functionMask;
    std::uint8_t mspBaudIndex;
    std::uint8_t gpsBaudIndex;
    std::uint8_t telemetryBaudIndex;
    std::uint8_t peripheralBaudIndex;

    static std::vector<std::uint8_t> pack(const MSP2_COMMON_SERIAL_CONFIG__reply& v) {
        BufferWriter w;
        w.write_le(v.identifier);
        w.write_le(v.functionMask);
        w.write_le(v.mspBaudIndex);
        w.write_le(v.gpsBaudIndex);
        w.write_le(v.telemetryBaudIndex);
        w.write_le(v.peripheralBaudIndex);
        return std::move(w.buf);
    }

    static MSP2_COMMON_SERIAL_CONFIG__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_COMMON_SERIAL_CONFIG__reply v{};
        v.identifier = r.read_le<std::uint8_t>();
        v.functionMask = r.read_le<std::uint32_t>();
        v.mspBaudIndex = r.read_le<std::uint8_t>();
        v.gpsBaudIndex = r.read_le<std::uint8_t>();
        v.telemetryBaudIndex = r.read_le<std::uint8_t>();
        v.peripheralBaudIndex = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP2_COMMON_SET_SERIAL_CONFIG__request {
    std::uint8_t identifier;
    std::uint32_t functionMask;
    std::uint8_t mspBaudIndex;
    std::uint8_t gpsBaudIndex;
    std::uint8_t telemetryBaudIndex;
    std::uint8_t peripheralBaudIndex;

    static std::vector<std::uint8_t> pack(const MSP2_COMMON_SET_SERIAL_CONFIG__request& v) {
        BufferWriter w;
        w.write_le(v.identifier);
        w.write_le(v.functionMask);
        w.write_le(v.mspBaudIndex);
        w.write_le(v.gpsBaudIndex);
        w.write_le(v.telemetryBaudIndex);
        w.write_le(v.peripheralBaudIndex);
        return std::move(w.buf);
    }

    static MSP2_COMMON_SET_SERIAL_CONFIG__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_COMMON_SET_SERIAL_CONFIG__request v{};
        v.identifier = r.read_le<std::uint8_t>();
        v.functionMask = r.read_le<std::uint32_t>();
        v.mspBaudIndex = r.read_le<std::uint8_t>();
        v.gpsBaudIndex = r.read_le<std::uint8_t>();
        v.telemetryBaudIndex = r.read_le<std::uint8_t>();
        v.peripheralBaudIndex = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP2_COMMON_SET_RADAR_POS__request {
    std::uint8_t poiIndex;
    std::uint8_t state;
    std::int32_t latitude;
    std::int32_t longitude;
    std::int32_t altitude;
    std::int16_t heading;
    std::uint16_t speed;
    std::uint8_t linkQuality;

    static std::vector<std::uint8_t> pack(const MSP2_COMMON_SET_RADAR_POS__request& v) {
        BufferWriter w;
        w.write_le(v.poiIndex);
        w.write_le(v.state);
        w.write_le(v.latitude);
        w.write_le(v.longitude);
        w.write_le(v.altitude);
        w.write_le(v.heading);
        w.write_le(v.speed);
        w.write_le(v.linkQuality);
        return std::move(w.buf);
    }

    static MSP2_COMMON_SET_RADAR_POS__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_COMMON_SET_RADAR_POS__request v{};
        v.poiIndex = r.read_le<std::uint8_t>();
        v.state = r.read_le<std::uint8_t>();
        v.latitude = r.read_le<std::int32_t>();
        v.longitude = r.read_le<std::int32_t>();
        v.altitude = r.read_le<std::int32_t>();
        v.heading = r.read_le<std::int16_t>();
        v.speed = r.read_le<std::uint16_t>();
        v.linkQuality = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP2_COMMON_SET_MSP_RC_LINK_STATS__request {
    std::uint8_t sublinkID;
    std::uint8_t validLink;
    std::uint8_t rssiPercent;
    std::uint8_t uplinkRSSI_dBm;
    std::uint8_t downlinkLQ;
    std::uint8_t uplinkLQ;
    std::int8_t uplinkSNR;

    static std::vector<std::uint8_t> pack(const MSP2_COMMON_SET_MSP_RC_LINK_STATS__request& v) {
        BufferWriter w;
        w.write_le(v.sublinkID);
        w.write_le(v.validLink);
        w.write_le(v.rssiPercent);
        w.write_le(v.uplinkRSSI_dBm);
        w.write_le(v.downlinkLQ);
        w.write_le(v.uplinkLQ);
        w.write_le(v.uplinkSNR);
        return std::move(w.buf);
    }

    static MSP2_COMMON_SET_MSP_RC_LINK_STATS__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_COMMON_SET_MSP_RC_LINK_STATS__request v{};
        v.sublinkID = r.read_le<std::uint8_t>();
        v.validLink = r.read_le<std::uint8_t>();
        v.rssiPercent = r.read_le<std::uint8_t>();
        v.uplinkRSSI_dBm = r.read_le<std::uint8_t>();
        v.downlinkLQ = r.read_le<std::uint8_t>();
        v.uplinkLQ = r.read_le<std::uint8_t>();
        v.uplinkSNR = r.read_le<std::int8_t>();
        return v;
    }
};

struct MSP2_COMMON_SET_MSP_RC_INFO__request {
    std::uint8_t sublinkID;
    std::uint16_t uplinkTxPower;
    std::uint16_t downlinkTxPower;
    std::array<char,4> band;
    std::array<char,6> mode;

    static std::vector<std::uint8_t> pack(const MSP2_COMMON_SET_MSP_RC_INFO__request& v) {
        BufferWriter w;
        w.write_le(v.sublinkID);
        w.write_le(v.uplinkTxPower);
        w.write_le(v.downlinkTxPower);
        w.write_bytes(v.band.data(), 4);
        w.write_bytes(v.mode.data(), 6);
        return std::move(w.buf);
    }

    static MSP2_COMMON_SET_MSP_RC_INFO__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_COMMON_SET_MSP_RC_INFO__request v{};
        v.sublinkID = r.read_le<std::uint8_t>();
        v.uplinkTxPower = r.read_le<std::uint16_t>();
        v.downlinkTxPower = r.read_le<std::uint16_t>();
        r.read_bytes(reinterpret_cast<std::uint8_t*>(v.band.data()), 4);
        r.read_bytes(reinterpret_cast<std::uint8_t*>(v.mode.data()), 6);
        return v;
    }
};

struct MSP2_SENSOR_RANGEFINDER__request {
    std::uint8_t quality;
    std::int32_t distanceMm;

    static std::vector<std::uint8_t> pack(const MSP2_SENSOR_RANGEFINDER__request& v) {
        BufferWriter w;
        w.write_le(v.quality);
        w.write_le(v.distanceMm);
        return std::move(w.buf);
    }

    static MSP2_SENSOR_RANGEFINDER__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_SENSOR_RANGEFINDER__request v{};
        v.quality = r.read_le<std::uint8_t>();
        v.distanceMm = r.read_le<std::int32_t>();
        return v;
    }
};

struct MSP2_SENSOR_OPTIC_FLOW__request {
    std::uint8_t quality;
    std::int32_t motionX;
    std::int32_t motionY;

    static std::vector<std::uint8_t> pack(const MSP2_SENSOR_OPTIC_FLOW__request& v) {
        BufferWriter w;
        w.write_le(v.quality);
        w.write_le(v.motionX);
        w.write_le(v.motionY);
        return std::move(w.buf);
    }

    static MSP2_SENSOR_OPTIC_FLOW__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_SENSOR_OPTIC_FLOW__request v{};
        v.quality = r.read_le<std::uint8_t>();
        v.motionX = r.read_le<std::int32_t>();
        v.motionY = r.read_le<std::int32_t>();
        return v;
    }
};

struct MSP2_SENSOR_GPS__request {
    std::uint8_t instance;
    std::uint16_t gpsWeek;
    std::uint32_t msTOW;
    std::uint8_t fixType;
    std::uint8_t satellitesInView;
    std::uint16_t hPosAccuracy;
    std::uint16_t vPosAccuracy;
    std::uint16_t hVelAccuracy;
    std::uint16_t hdop;
    std::int32_t longitude;
    std::int32_t latitude;
    std::int32_t mslAltitude;
    std::int32_t nedVelNorth;
    std::int32_t nedVelEast;
    std::int32_t nedVelDown;
    std::uint16_t groundCourse;
    std::uint16_t trueYaw;
    std::uint16_t year;
    std::uint8_t month;
    std::uint8_t day;
    std::uint8_t hour;
    std::uint8_t min;
    std::uint8_t sec;

    static std::vector<std::uint8_t> pack(const MSP2_SENSOR_GPS__request& v) {
        BufferWriter w;
        w.write_le(v.instance);
        w.write_le(v.gpsWeek);
        w.write_le(v.msTOW);
        w.write_le(v.fixType);
        w.write_le(v.satellitesInView);
        w.write_le(v.hPosAccuracy);
        w.write_le(v.vPosAccuracy);
        w.write_le(v.hVelAccuracy);
        w.write_le(v.hdop);
        w.write_le(v.longitude);
        w.write_le(v.latitude);
        w.write_le(v.mslAltitude);
        w.write_le(v.nedVelNorth);
        w.write_le(v.nedVelEast);
        w.write_le(v.nedVelDown);
        w.write_le(v.groundCourse);
        w.write_le(v.trueYaw);
        w.write_le(v.year);
        w.write_le(v.month);
        w.write_le(v.day);
        w.write_le(v.hour);
        w.write_le(v.min);
        w.write_le(v.sec);
        return std::move(w.buf);
    }

    static MSP2_SENSOR_GPS__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_SENSOR_GPS__request v{};
        v.instance = r.read_le<std::uint8_t>();
        v.gpsWeek = r.read_le<std::uint16_t>();
        v.msTOW = r.read_le<std::uint32_t>();
        v.fixType = r.read_le<std::uint8_t>();
        v.satellitesInView = r.read_le<std::uint8_t>();
        v.hPosAccuracy = r.read_le<std::uint16_t>();
        v.vPosAccuracy = r.read_le<std::uint16_t>();
        v.hVelAccuracy = r.read_le<std::uint16_t>();
        v.hdop = r.read_le<std::uint16_t>();
        v.longitude = r.read_le<std::int32_t>();
        v.latitude = r.read_le<std::int32_t>();
        v.mslAltitude = r.read_le<std::int32_t>();
        v.nedVelNorth = r.read_le<std::int32_t>();
        v.nedVelEast = r.read_le<std::int32_t>();
        v.nedVelDown = r.read_le<std::int32_t>();
        v.groundCourse = r.read_le<std::uint16_t>();
        v.trueYaw = r.read_le<std::uint16_t>();
        v.year = r.read_le<std::uint16_t>();
        v.month = r.read_le<std::uint8_t>();
        v.day = r.read_le<std::uint8_t>();
        v.hour = r.read_le<std::uint8_t>();
        v.min = r.read_le<std::uint8_t>();
        v.sec = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP2_SENSOR_COMPASS__request {
    std::uint8_t instance;
    std::uint32_t timeMs;
    std::int16_t magX;
    std::int16_t magY;
    std::int16_t magZ;

    static std::vector<std::uint8_t> pack(const MSP2_SENSOR_COMPASS__request& v) {
        BufferWriter w;
        w.write_le(v.instance);
        w.write_le(v.timeMs);
        w.write_le(v.magX);
        w.write_le(v.magY);
        w.write_le(v.magZ);
        return std::move(w.buf);
    }

    static MSP2_SENSOR_COMPASS__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_SENSOR_COMPASS__request v{};
        v.instance = r.read_le<std::uint8_t>();
        v.timeMs = r.read_le<std::uint32_t>();
        v.magX = r.read_le<std::int16_t>();
        v.magY = r.read_le<std::int16_t>();
        v.magZ = r.read_le<std::int16_t>();
        return v;
    }
};

struct MSP2_SENSOR_BAROMETER__request {
    std::uint8_t instance;
    std::uint32_t timeMs;
    float pressurePa;
    std::int16_t temp;

    static std::vector<std::uint8_t> pack(const MSP2_SENSOR_BAROMETER__request& v) {
        BufferWriter w;
        w.write_le(v.instance);
        w.write_le(v.timeMs);
        { auto bytes = float::pack(v.pressurePa); w.write_bytes(bytes.data(), bytes.size()); }
        w.write_le(v.temp);
        return std::move(w.buf);
    }

    static MSP2_SENSOR_BAROMETER__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_SENSOR_BAROMETER__request v{};
        v.instance = r.read_le<std::uint8_t>();
        v.timeMs = r.read_le<std::uint32_t>();
        static constexpr std::size_t __pressurePa_size = []{ float tmp{}; auto b = float::pack(tmp); return b.size(); }();
        { std::vector<std::uint8_t> chunk(__pressurePa_size); r.read_bytes(chunk.data(), __pressurePa_size); v.pressurePa = float::unpack(chunk); }
        v.temp = r.read_le<std::int16_t>();
        return v;
    }
};

struct MSP2_SENSOR_AIRSPEED__request {
    std::uint8_t instance;
    std::uint32_t timeMs;
    float diffPressurePa;
    std::int16_t temp;

    static std::vector<std::uint8_t> pack(const MSP2_SENSOR_AIRSPEED__request& v) {
        BufferWriter w;
        w.write_le(v.instance);
        w.write_le(v.timeMs);
        { auto bytes = float::pack(v.diffPressurePa); w.write_bytes(bytes.data(), bytes.size()); }
        w.write_le(v.temp);
        return std::move(w.buf);
    }

    static MSP2_SENSOR_AIRSPEED__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_SENSOR_AIRSPEED__request v{};
        v.instance = r.read_le<std::uint8_t>();
        v.timeMs = r.read_le<std::uint32_t>();
        static constexpr std::size_t __diffPressurePa_size = []{ float tmp{}; auto b = float::pack(tmp); return b.size(); }();
        { std::vector<std::uint8_t> chunk(__diffPressurePa_size); r.read_bytes(chunk.data(), __diffPressurePa_size); v.diffPressurePa = float::unpack(chunk); }
        v.temp = r.read_le<std::int16_t>();
        return v;
    }
};

struct MSP2_SENSOR_HEADTRACKER__request {
    Varies ___;

    static std::vector<std::uint8_t> pack(const MSP2_SENSOR_HEADTRACKER__request& v) {
        BufferWriter w;
        { auto bytes = Varies::pack(v.___); w.write_bytes(bytes.data(), bytes.size()); }
        return std::move(w.buf);
    }

    static MSP2_SENSOR_HEADTRACKER__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_SENSOR_HEADTRACKER__request v{};
        static constexpr std::size_t ______size = []{ Varies tmp{}; auto b = Varies::pack(tmp); return b.size(); }();
        { std::vector<std::uint8_t> chunk(______size); r.read_bytes(chunk.data(), ______size); v.___ = Varies::unpack(chunk); }
        return v;
    }
};

struct MSP2_INAV_STATUS__reply {
    std::uint16_t cycleTime;
    std::uint16_t i2cErrors;
    std::uint16_t sensorStatus;
    std::uint16_t cpuLoad;
    std::uint8_t profileAndBattProfile;
    std::uint32_t armingFlags;
    boxBitmask_t activeModes;
    std::uint8_t mixerProfile;

    static std::vector<std::uint8_t> pack(const MSP2_INAV_STATUS__reply& v) {
        BufferWriter w;
        w.write_le(v.cycleTime);
        w.write_le(v.i2cErrors);
        w.write_le(v.sensorStatus);
        w.write_le(v.cpuLoad);
        w.write_le(v.profileAndBattProfile);
        w.write_le(v.armingFlags);
        { auto bytes = boxBitmask_t::pack(v.activeModes); w.write_bytes(bytes.data(), bytes.size()); }
        w.write_le(v.mixerProfile);
        return std::move(w.buf);
    }

    static MSP2_INAV_STATUS__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_INAV_STATUS__reply v{};
        v.cycleTime = r.read_le<std::uint16_t>();
        v.i2cErrors = r.read_le<std::uint16_t>();
        v.sensorStatus = r.read_le<std::uint16_t>();
        v.cpuLoad = r.read_le<std::uint16_t>();
        v.profileAndBattProfile = r.read_le<std::uint8_t>();
        v.armingFlags = r.read_le<std::uint32_t>();
        static constexpr std::size_t __activeModes_size = []{ boxBitmask_t tmp{}; auto b = boxBitmask_t::pack(tmp); return b.size(); }();
        { std::vector<std::uint8_t> chunk(__activeModes_size); r.read_bytes(chunk.data(), __activeModes_size); v.activeModes = boxBitmask_t::unpack(chunk); }
        v.mixerProfile = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP2_INAV_OPTICAL_FLOW__reply {
    std::uint8_t quality;
    std::int16_t flowRateX;
    std::int16_t flowRateY;
    std::int16_t bodyRateX;
    std::int16_t bodyRateY;

    static std::vector<std::uint8_t> pack(const MSP2_INAV_OPTICAL_FLOW__reply& v) {
        BufferWriter w;
        w.write_le(v.quality);
        w.write_le(v.flowRateX);
        w.write_le(v.flowRateY);
        w.write_le(v.bodyRateX);
        w.write_le(v.bodyRateY);
        return std::move(w.buf);
    }

    static MSP2_INAV_OPTICAL_FLOW__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_INAV_OPTICAL_FLOW__reply v{};
        v.quality = r.read_le<std::uint8_t>();
        v.flowRateX = r.read_le<std::int16_t>();
        v.flowRateY = r.read_le<std::int16_t>();
        v.bodyRateX = r.read_le<std::int16_t>();
        v.bodyRateY = r.read_le<std::int16_t>();
        return v;
    }
};

struct MSP2_INAV_ANALOG__reply {
    std::uint8_t batteryFlags;
    std::uint16_t vbat;
    std::uint16_t amperage;
    std::uint32_t powerDraw;
    std::uint32_t mAhDrawn;
    std::uint32_t mWhDrawn;
    std::uint32_t remainingCapacity;
    std::uint8_t percentageRemaining;
    std::uint16_t rssi;

    static std::vector<std::uint8_t> pack(const MSP2_INAV_ANALOG__reply& v) {
        BufferWriter w;
        w.write_le(v.batteryFlags);
        w.write_le(v.vbat);
        w.write_le(v.amperage);
        w.write_le(v.powerDraw);
        w.write_le(v.mAhDrawn);
        w.write_le(v.mWhDrawn);
        w.write_le(v.remainingCapacity);
        w.write_le(v.percentageRemaining);
        w.write_le(v.rssi);
        return std::move(w.buf);
    }

    static MSP2_INAV_ANALOG__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_INAV_ANALOG__reply v{};
        v.batteryFlags = r.read_le<std::uint8_t>();
        v.vbat = r.read_le<std::uint16_t>();
        v.amperage = r.read_le<std::uint16_t>();
        v.powerDraw = r.read_le<std::uint32_t>();
        v.mAhDrawn = r.read_le<std::uint32_t>();
        v.mWhDrawn = r.read_le<std::uint32_t>();
        v.remainingCapacity = r.read_le<std::uint32_t>();
        v.percentageRemaining = r.read_le<std::uint8_t>();
        v.rssi = r.read_le<std::uint16_t>();
        return v;
    }
};

struct MSP2_INAV_MISC__reply {
    std::uint16_t midRc;
    std::uint16_t legacyMinThrottle;
    std::uint16_t maxThrottle;
    std::uint16_t minCommand;
    std::uint16_t failsafeThrottle;
    std::uint8_t gpsType;
    std::uint8_t legacyGpsBaud;
    std::uint8_t gpsSbasMode;
    std::uint8_t rssiChannel;
    std::uint16_t magDeclination;
    std::uint16_t vbatScale;
    std::uint8_t vbatSource;
    std::uint8_t cellCount;
    std::uint16_t vbatCellDetect;
    std::uint16_t vbatMinCell;
    std::uint16_t vbatMaxCell;
    std::uint16_t vbatWarningCell;
    std::uint32_t capacityValue;
    std::uint32_t capacityWarning;
    std::uint32_t capacityCritical;
    std::uint8_t capacityUnit;

    static std::vector<std::uint8_t> pack(const MSP2_INAV_MISC__reply& v) {
        BufferWriter w;
        w.write_le(v.midRc);
        w.write_le(v.legacyMinThrottle);
        w.write_le(v.maxThrottle);
        w.write_le(v.minCommand);
        w.write_le(v.failsafeThrottle);
        w.write_le(v.gpsType);
        w.write_le(v.legacyGpsBaud);
        w.write_le(v.gpsSbasMode);
        w.write_le(v.rssiChannel);
        w.write_le(v.magDeclination);
        w.write_le(v.vbatScale);
        w.write_le(v.vbatSource);
        w.write_le(v.cellCount);
        w.write_le(v.vbatCellDetect);
        w.write_le(v.vbatMinCell);
        w.write_le(v.vbatMaxCell);
        w.write_le(v.vbatWarningCell);
        w.write_le(v.capacityValue);
        w.write_le(v.capacityWarning);
        w.write_le(v.capacityCritical);
        w.write_le(v.capacityUnit);
        return std::move(w.buf);
    }

    static MSP2_INAV_MISC__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_INAV_MISC__reply v{};
        v.midRc = r.read_le<std::uint16_t>();
        v.legacyMinThrottle = r.read_le<std::uint16_t>();
        v.maxThrottle = r.read_le<std::uint16_t>();
        v.minCommand = r.read_le<std::uint16_t>();
        v.failsafeThrottle = r.read_le<std::uint16_t>();
        v.gpsType = r.read_le<std::uint8_t>();
        v.legacyGpsBaud = r.read_le<std::uint8_t>();
        v.gpsSbasMode = r.read_le<std::uint8_t>();
        v.rssiChannel = r.read_le<std::uint8_t>();
        v.magDeclination = r.read_le<std::uint16_t>();
        v.vbatScale = r.read_le<std::uint16_t>();
        v.vbatSource = r.read_le<std::uint8_t>();
        v.cellCount = r.read_le<std::uint8_t>();
        v.vbatCellDetect = r.read_le<std::uint16_t>();
        v.vbatMinCell = r.read_le<std::uint16_t>();
        v.vbatMaxCell = r.read_le<std::uint16_t>();
        v.vbatWarningCell = r.read_le<std::uint16_t>();
        v.capacityValue = r.read_le<std::uint32_t>();
        v.capacityWarning = r.read_le<std::uint32_t>();
        v.capacityCritical = r.read_le<std::uint32_t>();
        v.capacityUnit = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP2_INAV_SET_MISC__request {
    std::uint16_t midRc;
    std::uint16_t legacyMinThrottle;
    std::uint16_t legacyMaxThrottle;
    std::uint16_t minCommand;
    std::uint16_t failsafeThrottle;
    std::uint8_t gpsType;
    std::uint8_t legacyGpsBaud;
    std::uint8_t gpsSbasMode;
    std::uint8_t rssiChannel;
    std::uint16_t magDeclination;
    std::uint16_t vbatScale;
    std::uint8_t vbatSource;
    std::uint8_t cellCount;
    std::uint16_t vbatCellDetect;
    std::uint16_t vbatMinCell;
    std::uint16_t vbatMaxCell;
    std::uint16_t vbatWarningCell;
    std::uint32_t capacityValue;
    std::uint32_t capacityWarning;
    std::uint32_t capacityCritical;
    std::uint8_t capacityUnit;

    static std::vector<std::uint8_t> pack(const MSP2_INAV_SET_MISC__request& v) {
        BufferWriter w;
        w.write_le(v.midRc);
        w.write_le(v.legacyMinThrottle);
        w.write_le(v.legacyMaxThrottle);
        w.write_le(v.minCommand);
        w.write_le(v.failsafeThrottle);
        w.write_le(v.gpsType);
        w.write_le(v.legacyGpsBaud);
        w.write_le(v.gpsSbasMode);
        w.write_le(v.rssiChannel);
        w.write_le(v.magDeclination);
        w.write_le(v.vbatScale);
        w.write_le(v.vbatSource);
        w.write_le(v.cellCount);
        w.write_le(v.vbatCellDetect);
        w.write_le(v.vbatMinCell);
        w.write_le(v.vbatMaxCell);
        w.write_le(v.vbatWarningCell);
        w.write_le(v.capacityValue);
        w.write_le(v.capacityWarning);
        w.write_le(v.capacityCritical);
        w.write_le(v.capacityUnit);
        return std::move(w.buf);
    }

    static MSP2_INAV_SET_MISC__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_INAV_SET_MISC__request v{};
        v.midRc = r.read_le<std::uint16_t>();
        v.legacyMinThrottle = r.read_le<std::uint16_t>();
        v.legacyMaxThrottle = r.read_le<std::uint16_t>();
        v.minCommand = r.read_le<std::uint16_t>();
        v.failsafeThrottle = r.read_le<std::uint16_t>();
        v.gpsType = r.read_le<std::uint8_t>();
        v.legacyGpsBaud = r.read_le<std::uint8_t>();
        v.gpsSbasMode = r.read_le<std::uint8_t>();
        v.rssiChannel = r.read_le<std::uint8_t>();
        v.magDeclination = r.read_le<std::uint16_t>();
        v.vbatScale = r.read_le<std::uint16_t>();
        v.vbatSource = r.read_le<std::uint8_t>();
        v.cellCount = r.read_le<std::uint8_t>();
        v.vbatCellDetect = r.read_le<std::uint16_t>();
        v.vbatMinCell = r.read_le<std::uint16_t>();
        v.vbatMaxCell = r.read_le<std::uint16_t>();
        v.vbatWarningCell = r.read_le<std::uint16_t>();
        v.capacityValue = r.read_le<std::uint32_t>();
        v.capacityWarning = r.read_le<std::uint32_t>();
        v.capacityCritical = r.read_le<std::uint32_t>();
        v.capacityUnit = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP2_INAV_BATTERY_CONFIG__reply {
    std::uint16_t vbatScale;
    std::uint8_t vbatSource;
    std::uint8_t cellCount;
    std::uint16_t vbatCellDetect;
    std::uint16_t vbatMinCell;
    std::uint16_t vbatMaxCell;
    std::uint16_t vbatWarningCell;
    std::uint16_t currentOffset;
    std::uint16_t currentScale;
    std::uint32_t capacityValue;
    std::uint32_t capacityWarning;
    std::uint32_t capacityCritical;
    std::uint8_t capacityUnit;

    static std::vector<std::uint8_t> pack(const MSP2_INAV_BATTERY_CONFIG__reply& v) {
        BufferWriter w;
        w.write_le(v.vbatScale);
        w.write_le(v.vbatSource);
        w.write_le(v.cellCount);
        w.write_le(v.vbatCellDetect);
        w.write_le(v.vbatMinCell);
        w.write_le(v.vbatMaxCell);
        w.write_le(v.vbatWarningCell);
        w.write_le(v.currentOffset);
        w.write_le(v.currentScale);
        w.write_le(v.capacityValue);
        w.write_le(v.capacityWarning);
        w.write_le(v.capacityCritical);
        w.write_le(v.capacityUnit);
        return std::move(w.buf);
    }

    static MSP2_INAV_BATTERY_CONFIG__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_INAV_BATTERY_CONFIG__reply v{};
        v.vbatScale = r.read_le<std::uint16_t>();
        v.vbatSource = r.read_le<std::uint8_t>();
        v.cellCount = r.read_le<std::uint8_t>();
        v.vbatCellDetect = r.read_le<std::uint16_t>();
        v.vbatMinCell = r.read_le<std::uint16_t>();
        v.vbatMaxCell = r.read_le<std::uint16_t>();
        v.vbatWarningCell = r.read_le<std::uint16_t>();
        v.currentOffset = r.read_le<std::uint16_t>();
        v.currentScale = r.read_le<std::uint16_t>();
        v.capacityValue = r.read_le<std::uint32_t>();
        v.capacityWarning = r.read_le<std::uint32_t>();
        v.capacityCritical = r.read_le<std::uint32_t>();
        v.capacityUnit = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP2_INAV_SET_BATTERY_CONFIG__request {
    std::uint16_t vbatScale;
    std::uint8_t vbatSource;
    std::uint8_t cellCount;
    std::uint16_t vbatCellDetect;
    std::uint16_t vbatMinCell;
    std::uint16_t vbatMaxCell;
    std::uint16_t vbatWarningCell;
    std::uint16_t currentOffset;
    std::uint16_t currentScale;
    std::uint32_t capacityValue;
    std::uint32_t capacityWarning;
    std::uint32_t capacityCritical;
    std::uint8_t capacityUnit;

    static std::vector<std::uint8_t> pack(const MSP2_INAV_SET_BATTERY_CONFIG__request& v) {
        BufferWriter w;
        w.write_le(v.vbatScale);
        w.write_le(v.vbatSource);
        w.write_le(v.cellCount);
        w.write_le(v.vbatCellDetect);
        w.write_le(v.vbatMinCell);
        w.write_le(v.vbatMaxCell);
        w.write_le(v.vbatWarningCell);
        w.write_le(v.currentOffset);
        w.write_le(v.currentScale);
        w.write_le(v.capacityValue);
        w.write_le(v.capacityWarning);
        w.write_le(v.capacityCritical);
        w.write_le(v.capacityUnit);
        return std::move(w.buf);
    }

    static MSP2_INAV_SET_BATTERY_CONFIG__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_INAV_SET_BATTERY_CONFIG__request v{};
        v.vbatScale = r.read_le<std::uint16_t>();
        v.vbatSource = r.read_le<std::uint8_t>();
        v.cellCount = r.read_le<std::uint8_t>();
        v.vbatCellDetect = r.read_le<std::uint16_t>();
        v.vbatMinCell = r.read_le<std::uint16_t>();
        v.vbatMaxCell = r.read_le<std::uint16_t>();
        v.vbatWarningCell = r.read_le<std::uint16_t>();
        v.currentOffset = r.read_le<std::uint16_t>();
        v.currentScale = r.read_le<std::uint16_t>();
        v.capacityValue = r.read_le<std::uint32_t>();
        v.capacityWarning = r.read_le<std::uint32_t>();
        v.capacityCritical = r.read_le<std::uint32_t>();
        v.capacityUnit = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP2_INAV_RATE_PROFILE__reply {
    std::uint8_t throttleMid;
    std::uint8_t throttleExpo;
    std::uint8_t dynamicThrottlePID;
    std::uint16_t tpaBreakpoint;
    std::uint8_t stabRcExpo;
    std::uint8_t stabRcYawExpo;
    std::uint8_t stabRollRate;
    std::uint8_t stabPitchRate;
    std::uint8_t stabYawRate;
    std::uint8_t manualRcExpo;
    std::uint8_t manualRcYawExpo;
    std::uint8_t manualRollRate;
    std::uint8_t manualPitchRate;
    std::uint8_t manualYawRate;

    static std::vector<std::uint8_t> pack(const MSP2_INAV_RATE_PROFILE__reply& v) {
        BufferWriter w;
        w.write_le(v.throttleMid);
        w.write_le(v.throttleExpo);
        w.write_le(v.dynamicThrottlePID);
        w.write_le(v.tpaBreakpoint);
        w.write_le(v.stabRcExpo);
        w.write_le(v.stabRcYawExpo);
        w.write_le(v.stabRollRate);
        w.write_le(v.stabPitchRate);
        w.write_le(v.stabYawRate);
        w.write_le(v.manualRcExpo);
        w.write_le(v.manualRcYawExpo);
        w.write_le(v.manualRollRate);
        w.write_le(v.manualPitchRate);
        w.write_le(v.manualYawRate);
        return std::move(w.buf);
    }

    static MSP2_INAV_RATE_PROFILE__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_INAV_RATE_PROFILE__reply v{};
        v.throttleMid = r.read_le<std::uint8_t>();
        v.throttleExpo = r.read_le<std::uint8_t>();
        v.dynamicThrottlePID = r.read_le<std::uint8_t>();
        v.tpaBreakpoint = r.read_le<std::uint16_t>();
        v.stabRcExpo = r.read_le<std::uint8_t>();
        v.stabRcYawExpo = r.read_le<std::uint8_t>();
        v.stabRollRate = r.read_le<std::uint8_t>();
        v.stabPitchRate = r.read_le<std::uint8_t>();
        v.stabYawRate = r.read_le<std::uint8_t>();
        v.manualRcExpo = r.read_le<std::uint8_t>();
        v.manualRcYawExpo = r.read_le<std::uint8_t>();
        v.manualRollRate = r.read_le<std::uint8_t>();
        v.manualPitchRate = r.read_le<std::uint8_t>();
        v.manualYawRate = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP2_INAV_SET_RATE_PROFILE__request {
    std::uint8_t throttleMid;
    std::uint8_t throttleExpo;
    std::uint8_t dynamicThrottlePID;
    std::uint16_t tpaBreakpoint;
    std::uint8_t stabRcExpo;
    std::uint8_t stabRcYawExpo;
    std::uint8_t stabRollRate;
    std::uint8_t stabPitchRate;
    std::uint8_t stabYawRate;
    std::uint8_t manualRcExpo;
    std::uint8_t manualRcYawExpo;
    std::uint8_t manualRollRate;
    std::uint8_t manualPitchRate;
    std::uint8_t manualYawRate;

    static std::vector<std::uint8_t> pack(const MSP2_INAV_SET_RATE_PROFILE__request& v) {
        BufferWriter w;
        w.write_le(v.throttleMid);
        w.write_le(v.throttleExpo);
        w.write_le(v.dynamicThrottlePID);
        w.write_le(v.tpaBreakpoint);
        w.write_le(v.stabRcExpo);
        w.write_le(v.stabRcYawExpo);
        w.write_le(v.stabRollRate);
        w.write_le(v.stabPitchRate);
        w.write_le(v.stabYawRate);
        w.write_le(v.manualRcExpo);
        w.write_le(v.manualRcYawExpo);
        w.write_le(v.manualRollRate);
        w.write_le(v.manualPitchRate);
        w.write_le(v.manualYawRate);
        return std::move(w.buf);
    }

    static MSP2_INAV_SET_RATE_PROFILE__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_INAV_SET_RATE_PROFILE__request v{};
        v.throttleMid = r.read_le<std::uint8_t>();
        v.throttleExpo = r.read_le<std::uint8_t>();
        v.dynamicThrottlePID = r.read_le<std::uint8_t>();
        v.tpaBreakpoint = r.read_le<std::uint16_t>();
        v.stabRcExpo = r.read_le<std::uint8_t>();
        v.stabRcYawExpo = r.read_le<std::uint8_t>();
        v.stabRollRate = r.read_le<std::uint8_t>();
        v.stabPitchRate = r.read_le<std::uint8_t>();
        v.stabYawRate = r.read_le<std::uint8_t>();
        v.manualRcExpo = r.read_le<std::uint8_t>();
        v.manualRcYawExpo = r.read_le<std::uint8_t>();
        v.manualRollRate = r.read_le<std::uint8_t>();
        v.manualPitchRate = r.read_le<std::uint8_t>();
        v.manualYawRate = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP2_INAV_AIR_SPEED__reply {
    std::uint32_t airspeed;

    static std::vector<std::uint8_t> pack(const MSP2_INAV_AIR_SPEED__reply& v) {
        BufferWriter w;
        w.write_le(v.airspeed);
        return std::move(w.buf);
    }

    static MSP2_INAV_AIR_SPEED__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_INAV_AIR_SPEED__reply v{};
        v.airspeed = r.read_le<std::uint32_t>();
        return v;
    }
};

struct MSP2_INAV_OUTPUT_MAPPING__reply {
    std::uint8_t usageFlags;

    static std::vector<std::uint8_t> pack(const MSP2_INAV_OUTPUT_MAPPING__reply& v) {
        BufferWriter w;
        w.write_le(v.usageFlags);
        return std::move(w.buf);
    }

    static MSP2_INAV_OUTPUT_MAPPING__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_INAV_OUTPUT_MAPPING__reply v{};
        v.usageFlags = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP2_INAV_MC_BRAKING__reply {
    std::uint16_t brakingSpeedThreshold;
    std::uint16_t brakingDisengageSpeed;
    std::uint16_t brakingTimeout;
    std::uint8_t brakingBoostFactor;
    std::uint16_t brakingBoostTimeout;
    std::uint16_t brakingBoostSpeedThreshold;
    std::uint16_t brakingBoostDisengageSpeed;
    std::uint8_t brakingBankAngle;

    static std::vector<std::uint8_t> pack(const MSP2_INAV_MC_BRAKING__reply& v) {
        BufferWriter w;
        w.write_le(v.brakingSpeedThreshold);
        w.write_le(v.brakingDisengageSpeed);
        w.write_le(v.brakingTimeout);
        w.write_le(v.brakingBoostFactor);
        w.write_le(v.brakingBoostTimeout);
        w.write_le(v.brakingBoostSpeedThreshold);
        w.write_le(v.brakingBoostDisengageSpeed);
        w.write_le(v.brakingBankAngle);
        return std::move(w.buf);
    }

    static MSP2_INAV_MC_BRAKING__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_INAV_MC_BRAKING__reply v{};
        v.brakingSpeedThreshold = r.read_le<std::uint16_t>();
        v.brakingDisengageSpeed = r.read_le<std::uint16_t>();
        v.brakingTimeout = r.read_le<std::uint16_t>();
        v.brakingBoostFactor = r.read_le<std::uint8_t>();
        v.brakingBoostTimeout = r.read_le<std::uint16_t>();
        v.brakingBoostSpeedThreshold = r.read_le<std::uint16_t>();
        v.brakingBoostDisengageSpeed = r.read_le<std::uint16_t>();
        v.brakingBankAngle = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP2_INAV_SET_MC_BRAKING__request {
    std::uint16_t brakingSpeedThreshold;
    std::uint16_t brakingDisengageSpeed;
    std::uint16_t brakingTimeout;
    std::uint8_t brakingBoostFactor;
    std::uint16_t brakingBoostTimeout;
    std::uint16_t brakingBoostSpeedThreshold;
    std::uint16_t brakingBoostDisengageSpeed;
    std::uint8_t brakingBankAngle;

    static std::vector<std::uint8_t> pack(const MSP2_INAV_SET_MC_BRAKING__request& v) {
        BufferWriter w;
        w.write_le(v.brakingSpeedThreshold);
        w.write_le(v.brakingDisengageSpeed);
        w.write_le(v.brakingTimeout);
        w.write_le(v.brakingBoostFactor);
        w.write_le(v.brakingBoostTimeout);
        w.write_le(v.brakingBoostSpeedThreshold);
        w.write_le(v.brakingBoostDisengageSpeed);
        w.write_le(v.brakingBankAngle);
        return std::move(w.buf);
    }

    static MSP2_INAV_SET_MC_BRAKING__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_INAV_SET_MC_BRAKING__request v{};
        v.brakingSpeedThreshold = r.read_le<std::uint16_t>();
        v.brakingDisengageSpeed = r.read_le<std::uint16_t>();
        v.brakingTimeout = r.read_le<std::uint16_t>();
        v.brakingBoostFactor = r.read_le<std::uint8_t>();
        v.brakingBoostTimeout = r.read_le<std::uint16_t>();
        v.brakingBoostSpeedThreshold = r.read_le<std::uint16_t>();
        v.brakingBoostDisengageSpeed = r.read_le<std::uint16_t>();
        v.brakingBankAngle = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP2_INAV_OUTPUT_MAPPING_EXT__reply {
    std::uint8_t timerId;
    std::uint8_t usageFlags;

    static std::vector<std::uint8_t> pack(const MSP2_INAV_OUTPUT_MAPPING_EXT__reply& v) {
        BufferWriter w;
        w.write_le(v.timerId);
        w.write_le(v.usageFlags);
        return std::move(w.buf);
    }

    static MSP2_INAV_OUTPUT_MAPPING_EXT__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_INAV_OUTPUT_MAPPING_EXT__reply v{};
        v.timerId = r.read_le<std::uint8_t>();
        v.usageFlags = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP2_INAV_TIMER_OUTPUT_MODE__dataSize____0 {

    static std::vector<std::uint8_t> pack(const MSP2_INAV_TIMER_OUTPUT_MODE__dataSize____0& v) {
        BufferWriter w;
        return std::move(w.buf);
    }

    static MSP2_INAV_TIMER_OUTPUT_MODE__dataSize____0 unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_INAV_TIMER_OUTPUT_MODE__dataSize____0 v{};
        return v;
    }
};

struct MSP2_INAV_TIMER_OUTPUT_MODE__dataSize____1 {
    std::uint8_t timerIndex;

    static std::vector<std::uint8_t> pack(const MSP2_INAV_TIMER_OUTPUT_MODE__dataSize____1& v) {
        BufferWriter w;
        w.write_le(v.timerIndex);
        return std::move(w.buf);
    }

    static MSP2_INAV_TIMER_OUTPUT_MODE__dataSize____1 unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_INAV_TIMER_OUTPUT_MODE__dataSize____1 v{};
        v.timerIndex = r.read_le<std::uint8_t>();
        return v;
    }
};

using MSP2_INAV_TIMER_OUTPUT_MODE_variant = std::variant<MSP2_INAV_TIMER_OUTPUT_MODE__dataSize____0, MSP2_INAV_TIMER_OUTPUT_MODE__dataSize____1>;
inline MSP2_INAV_TIMER_OUTPUT_MODE_variant unpack_MSP2_INAV_TIMER_OUTPUT_MODE(const std::vector<std::uint8_t>& payload) {
    switch (payload.size()) {
    case 0: return MSP2_INAV_TIMER_OUTPUT_MODE__dataSize____0::unpack(payload);
    case 1: return MSP2_INAV_TIMER_OUTPUT_MODE__dataSize____1::unpack(payload);
    default:
        return MSP2_INAV_TIMER_OUTPUT_MODE__dataSize____0::unpack(payload);
    }
}

struct MSP2_INAV_SET_TIMER_OUTPUT_MODE__request {
    std::uint8_t timerIndex;
    std::uint8_t outputMode;

    static std::vector<std::uint8_t> pack(const MSP2_INAV_SET_TIMER_OUTPUT_MODE__request& v) {
        BufferWriter w;
        w.write_le(v.timerIndex);
        w.write_le(v.outputMode);
        return std::move(w.buf);
    }

    static MSP2_INAV_SET_TIMER_OUTPUT_MODE__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_INAV_SET_TIMER_OUTPUT_MODE__request v{};
        v.timerIndex = r.read_le<std::uint8_t>();
        v.outputMode = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP2_INAV_MIXER__reply {
    std::uint8_t motorDirectionInverted;
    std::uint8_t reserved1;
    std::uint8_t motorStopOnLow;
    std::uint8_t platformType;
    std::uint8_t hasFlaps;
    std::uint16_t appliedMixerPreset;
    std::uint8_t maxMotors;
    std::uint8_t maxServos;

    static std::vector<std::uint8_t> pack(const MSP2_INAV_MIXER__reply& v) {
        BufferWriter w;
        w.write_le(v.motorDirectionInverted);
        w.write_le(v.reserved1);
        w.write_le(v.motorStopOnLow);
        w.write_le(v.platformType);
        w.write_le(v.hasFlaps);
        w.write_le(v.appliedMixerPreset);
        w.write_le(v.maxMotors);
        w.write_le(v.maxServos);
        return std::move(w.buf);
    }

    static MSP2_INAV_MIXER__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_INAV_MIXER__reply v{};
        v.motorDirectionInverted = r.read_le<std::uint8_t>();
        v.reserved1 = r.read_le<std::uint8_t>();
        v.motorStopOnLow = r.read_le<std::uint8_t>();
        v.platformType = r.read_le<std::uint8_t>();
        v.hasFlaps = r.read_le<std::uint8_t>();
        v.appliedMixerPreset = r.read_le<std::uint16_t>();
        v.maxMotors = r.read_le<std::uint8_t>();
        v.maxServos = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP2_INAV_SET_MIXER__request {
    std::uint8_t motorDirectionInverted;
    std::uint8_t reserved1;
    std::uint8_t motorStopOnLow;
    std::uint8_t platformType;
    std::uint8_t hasFlaps;
    std::uint16_t appliedMixerPreset;
    std::uint8_t maxMotors;
    std::uint8_t maxServos;

    static std::vector<std::uint8_t> pack(const MSP2_INAV_SET_MIXER__request& v) {
        BufferWriter w;
        w.write_le(v.motorDirectionInverted);
        w.write_le(v.reserved1);
        w.write_le(v.motorStopOnLow);
        w.write_le(v.platformType);
        w.write_le(v.hasFlaps);
        w.write_le(v.appliedMixerPreset);
        w.write_le(v.maxMotors);
        w.write_le(v.maxServos);
        return std::move(w.buf);
    }

    static MSP2_INAV_SET_MIXER__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_INAV_SET_MIXER__request v{};
        v.motorDirectionInverted = r.read_le<std::uint8_t>();
        v.reserved1 = r.read_le<std::uint8_t>();
        v.motorStopOnLow = r.read_le<std::uint8_t>();
        v.platformType = r.read_le<std::uint8_t>();
        v.hasFlaps = r.read_le<std::uint8_t>();
        v.appliedMixerPreset = r.read_le<std::uint16_t>();
        v.maxMotors = r.read_le<std::uint8_t>();
        v.maxServos = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP2_INAV_OSD_LAYOUTS__reply {
    std::uint8_t layoutCount;
    std::uint8_t itemCount;

    static std::vector<std::uint8_t> pack(const MSP2_INAV_OSD_LAYOUTS__reply& v) {
        BufferWriter w;
        w.write_le(v.layoutCount);
        w.write_le(v.itemCount);
        return std::move(w.buf);
    }

    static MSP2_INAV_OSD_LAYOUTS__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_INAV_OSD_LAYOUTS__reply v{};
        v.layoutCount = r.read_le<std::uint8_t>();
        v.itemCount = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP2_INAV_OSD_SET_LAYOUT_ITEM__request {
    std::uint8_t layoutIndex;
    std::uint8_t itemIndex;
    std::uint16_t itemPosition;

    static std::vector<std::uint8_t> pack(const MSP2_INAV_OSD_SET_LAYOUT_ITEM__request& v) {
        BufferWriter w;
        w.write_le(v.layoutIndex);
        w.write_le(v.itemIndex);
        w.write_le(v.itemPosition);
        return std::move(w.buf);
    }

    static MSP2_INAV_OSD_SET_LAYOUT_ITEM__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_INAV_OSD_SET_LAYOUT_ITEM__request v{};
        v.layoutIndex = r.read_le<std::uint8_t>();
        v.itemIndex = r.read_le<std::uint8_t>();
        v.itemPosition = r.read_le<std::uint16_t>();
        return v;
    }
};

struct MSP2_INAV_OSD_ALARMS__reply {
    std::uint8_t rssiAlarm;
    std::uint16_t timerAlarm;
    std::uint16_t altAlarm;
    std::uint16_t distAlarm;
    std::uint16_t negAltAlarm;
    std::uint16_t gForceAlarm;
    std::int16_t gForceAxisMinAlarm;
    std::int16_t gForceAxisMaxAlarm;
    std::uint8_t currentAlarm;
    std::uint16_t imuTempMinAlarm;
    std::uint16_t imuTempMaxAlarm;
    std::uint16_t baroTempMinAlarm;
    std::uint16_t baroTempMaxAlarm;
    std::uint16_t adsbWarnDistance;
    std::uint16_t adsbAlertDistance;

    static std::vector<std::uint8_t> pack(const MSP2_INAV_OSD_ALARMS__reply& v) {
        BufferWriter w;
        w.write_le(v.rssiAlarm);
        w.write_le(v.timerAlarm);
        w.write_le(v.altAlarm);
        w.write_le(v.distAlarm);
        w.write_le(v.negAltAlarm);
        w.write_le(v.gForceAlarm);
        w.write_le(v.gForceAxisMinAlarm);
        w.write_le(v.gForceAxisMaxAlarm);
        w.write_le(v.currentAlarm);
        w.write_le(v.imuTempMinAlarm);
        w.write_le(v.imuTempMaxAlarm);
        w.write_le(v.baroTempMinAlarm);
        w.write_le(v.baroTempMaxAlarm);
        w.write_le(v.adsbWarnDistance);
        w.write_le(v.adsbAlertDistance);
        return std::move(w.buf);
    }

    static MSP2_INAV_OSD_ALARMS__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_INAV_OSD_ALARMS__reply v{};
        v.rssiAlarm = r.read_le<std::uint8_t>();
        v.timerAlarm = r.read_le<std::uint16_t>();
        v.altAlarm = r.read_le<std::uint16_t>();
        v.distAlarm = r.read_le<std::uint16_t>();
        v.negAltAlarm = r.read_le<std::uint16_t>();
        v.gForceAlarm = r.read_le<std::uint16_t>();
        v.gForceAxisMinAlarm = r.read_le<std::int16_t>();
        v.gForceAxisMaxAlarm = r.read_le<std::int16_t>();
        v.currentAlarm = r.read_le<std::uint8_t>();
        v.imuTempMinAlarm = r.read_le<std::uint16_t>();
        v.imuTempMaxAlarm = r.read_le<std::uint16_t>();
        v.baroTempMinAlarm = r.read_le<std::uint16_t>();
        v.baroTempMaxAlarm = r.read_le<std::uint16_t>();
        v.adsbWarnDistance = r.read_le<std::uint16_t>();
        v.adsbAlertDistance = r.read_le<std::uint16_t>();
        return v;
    }
};

struct MSP2_INAV_OSD_SET_ALARMS__request {
    std::uint8_t rssiAlarm;
    std::uint16_t timerAlarm;
    std::uint16_t altAlarm;
    std::uint16_t distAlarm;
    std::uint16_t negAltAlarm;
    std::uint16_t gForceAlarm;
    std::int16_t gForceAxisMinAlarm;
    std::int16_t gForceAxisMaxAlarm;
    std::uint8_t currentAlarm;
    std::uint16_t imuTempMinAlarm;
    std::uint16_t imuTempMaxAlarm;
    std::uint16_t baroTempMinAlarm;
    std::uint16_t baroTempMaxAlarm;

    static std::vector<std::uint8_t> pack(const MSP2_INAV_OSD_SET_ALARMS__request& v) {
        BufferWriter w;
        w.write_le(v.rssiAlarm);
        w.write_le(v.timerAlarm);
        w.write_le(v.altAlarm);
        w.write_le(v.distAlarm);
        w.write_le(v.negAltAlarm);
        w.write_le(v.gForceAlarm);
        w.write_le(v.gForceAxisMinAlarm);
        w.write_le(v.gForceAxisMaxAlarm);
        w.write_le(v.currentAlarm);
        w.write_le(v.imuTempMinAlarm);
        w.write_le(v.imuTempMaxAlarm);
        w.write_le(v.baroTempMinAlarm);
        w.write_le(v.baroTempMaxAlarm);
        return std::move(w.buf);
    }

    static MSP2_INAV_OSD_SET_ALARMS__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_INAV_OSD_SET_ALARMS__request v{};
        v.rssiAlarm = r.read_le<std::uint8_t>();
        v.timerAlarm = r.read_le<std::uint16_t>();
        v.altAlarm = r.read_le<std::uint16_t>();
        v.distAlarm = r.read_le<std::uint16_t>();
        v.negAltAlarm = r.read_le<std::uint16_t>();
        v.gForceAlarm = r.read_le<std::uint16_t>();
        v.gForceAxisMinAlarm = r.read_le<std::int16_t>();
        v.gForceAxisMaxAlarm = r.read_le<std::int16_t>();
        v.currentAlarm = r.read_le<std::uint8_t>();
        v.imuTempMinAlarm = r.read_le<std::uint16_t>();
        v.imuTempMaxAlarm = r.read_le<std::uint16_t>();
        v.baroTempMinAlarm = r.read_le<std::uint16_t>();
        v.baroTempMaxAlarm = r.read_le<std::uint16_t>();
        return v;
    }
};

struct MSP2_INAV_OSD_PREFERENCES__reply {
    std::uint8_t videoSystem;
    std::uint8_t mainVoltageDecimals;
    std::uint8_t ahiReverseRoll;
    std::uint8_t crosshairsStyle;
    std::uint8_t leftSidebarScroll;
    std::uint8_t rightSidebarScroll;
    std::uint8_t sidebarScrollArrows;
    std::uint8_t units;
    std::uint8_t statsEnergyUnit;

    static std::vector<std::uint8_t> pack(const MSP2_INAV_OSD_PREFERENCES__reply& v) {
        BufferWriter w;
        w.write_le(v.videoSystem);
        w.write_le(v.mainVoltageDecimals);
        w.write_le(v.ahiReverseRoll);
        w.write_le(v.crosshairsStyle);
        w.write_le(v.leftSidebarScroll);
        w.write_le(v.rightSidebarScroll);
        w.write_le(v.sidebarScrollArrows);
        w.write_le(v.units);
        w.write_le(v.statsEnergyUnit);
        return std::move(w.buf);
    }

    static MSP2_INAV_OSD_PREFERENCES__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_INAV_OSD_PREFERENCES__reply v{};
        v.videoSystem = r.read_le<std::uint8_t>();
        v.mainVoltageDecimals = r.read_le<std::uint8_t>();
        v.ahiReverseRoll = r.read_le<std::uint8_t>();
        v.crosshairsStyle = r.read_le<std::uint8_t>();
        v.leftSidebarScroll = r.read_le<std::uint8_t>();
        v.rightSidebarScroll = r.read_le<std::uint8_t>();
        v.sidebarScrollArrows = r.read_le<std::uint8_t>();
        v.units = r.read_le<std::uint8_t>();
        v.statsEnergyUnit = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP2_INAV_OSD_SET_PREFERENCES__request {
    std::uint8_t videoSystem;
    std::uint8_t mainVoltageDecimals;
    std::uint8_t ahiReverseRoll;
    std::uint8_t crosshairsStyle;
    std::uint8_t leftSidebarScroll;
    std::uint8_t rightSidebarScroll;
    std::uint8_t sidebarScrollArrows;
    std::uint8_t units;
    std::uint8_t statsEnergyUnit;

    static std::vector<std::uint8_t> pack(const MSP2_INAV_OSD_SET_PREFERENCES__request& v) {
        BufferWriter w;
        w.write_le(v.videoSystem);
        w.write_le(v.mainVoltageDecimals);
        w.write_le(v.ahiReverseRoll);
        w.write_le(v.crosshairsStyle);
        w.write_le(v.leftSidebarScroll);
        w.write_le(v.rightSidebarScroll);
        w.write_le(v.sidebarScrollArrows);
        w.write_le(v.units);
        w.write_le(v.statsEnergyUnit);
        return std::move(w.buf);
    }

    static MSP2_INAV_OSD_SET_PREFERENCES__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_INAV_OSD_SET_PREFERENCES__request v{};
        v.videoSystem = r.read_le<std::uint8_t>();
        v.mainVoltageDecimals = r.read_le<std::uint8_t>();
        v.ahiReverseRoll = r.read_le<std::uint8_t>();
        v.crosshairsStyle = r.read_le<std::uint8_t>();
        v.leftSidebarScroll = r.read_le<std::uint8_t>();
        v.rightSidebarScroll = r.read_le<std::uint8_t>();
        v.sidebarScrollArrows = r.read_le<std::uint8_t>();
        v.units = r.read_le<std::uint8_t>();
        v.statsEnergyUnit = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP2_INAV_SELECT_BATTERY_PROFILE__request {
    std::uint8_t batteryProfileIndex;

    static std::vector<std::uint8_t> pack(const MSP2_INAV_SELECT_BATTERY_PROFILE__request& v) {
        BufferWriter w;
        w.write_le(v.batteryProfileIndex);
        return std::move(w.buf);
    }

    static MSP2_INAV_SELECT_BATTERY_PROFILE__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_INAV_SELECT_BATTERY_PROFILE__request v{};
        v.batteryProfileIndex = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP2_INAV_DEBUG__reply {
    std::vector<std::uint32_t> debugValues;

    static std::vector<std::uint8_t> pack(const MSP2_INAV_DEBUG__reply& v) {
        BufferWriter w;
        for (const auto& e : v.debugValues) w.write_le(e);
        return std::move(w.buf);
    }

    static MSP2_INAV_DEBUG__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_INAV_DEBUG__reply v{};
        v.debugValues.clear();
        while (r.remaining() >= sizeof(std::uint32_t)) v.debugValues.push_back(r.read_le<std::uint32_t>());
        return v;
    }
};

struct MSP2_BLACKBOX_CONFIG__reply {
    std::uint8_t blackboxSupported;
    std::uint8_t blackboxDevice;
    std::uint16_t blackboxRateNum;
    std::uint16_t blackboxRateDenom;
    std::uint32_t blackboxIncludeFlags;

    static std::vector<std::uint8_t> pack(const MSP2_BLACKBOX_CONFIG__reply& v) {
        BufferWriter w;
        w.write_le(v.blackboxSupported);
        w.write_le(v.blackboxDevice);
        w.write_le(v.blackboxRateNum);
        w.write_le(v.blackboxRateDenom);
        w.write_le(v.blackboxIncludeFlags);
        return std::move(w.buf);
    }

    static MSP2_BLACKBOX_CONFIG__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_BLACKBOX_CONFIG__reply v{};
        v.blackboxSupported = r.read_le<std::uint8_t>();
        v.blackboxDevice = r.read_le<std::uint8_t>();
        v.blackboxRateNum = r.read_le<std::uint16_t>();
        v.blackboxRateDenom = r.read_le<std::uint16_t>();
        v.blackboxIncludeFlags = r.read_le<std::uint32_t>();
        return v;
    }
};

struct MSP2_SET_BLACKBOX_CONFIG__request {
    std::uint8_t blackboxDevice;
    std::uint16_t blackboxRateNum;
    std::uint16_t blackboxRateDenom;
    std::uint32_t blackboxIncludeFlags;

    static std::vector<std::uint8_t> pack(const MSP2_SET_BLACKBOX_CONFIG__request& v) {
        BufferWriter w;
        w.write_le(v.blackboxDevice);
        w.write_le(v.blackboxRateNum);
        w.write_le(v.blackboxRateDenom);
        w.write_le(v.blackboxIncludeFlags);
        return std::move(w.buf);
    }

    static MSP2_SET_BLACKBOX_CONFIG__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_SET_BLACKBOX_CONFIG__request v{};
        v.blackboxDevice = r.read_le<std::uint8_t>();
        v.blackboxRateNum = r.read_le<std::uint16_t>();
        v.blackboxRateDenom = r.read_le<std::uint16_t>();
        v.blackboxIncludeFlags = r.read_le<std::uint32_t>();
        return v;
    }
};

struct MSP2_INAV_TEMP_SENSOR_CONFIG__reply {
    std::uint8_t type;
    uint64_t address;
    std::uint16_t alarmMin;
    std::uint16_t alarmMax;
    std::uint8_t osdSymbol;
    std::array<char,TEMPERATURE_LABEL_LEN> label;

    static std::vector<std::uint8_t> pack(const MSP2_INAV_TEMP_SENSOR_CONFIG__reply& v) {
        BufferWriter w;
        w.write_le(v.type);
        { auto bytes = uint64_t::pack(v.address); w.write_bytes(bytes.data(), bytes.size()); }
        w.write_le(v.alarmMin);
        w.write_le(v.alarmMax);
        w.write_le(v.osdSymbol);
        w.write_bytes(v.label.data(), sizeof(v.label));
        return std::move(w.buf);
    }

    static MSP2_INAV_TEMP_SENSOR_CONFIG__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_INAV_TEMP_SENSOR_CONFIG__reply v{};
        v.type = r.read_le<std::uint8_t>();
        static constexpr std::size_t __address_size = []{ uint64_t tmp{}; auto b = uint64_t::pack(tmp); return b.size(); }();
        { std::vector<std::uint8_t> chunk(__address_size); r.read_bytes(chunk.data(), __address_size); v.address = uint64_t::unpack(chunk); }
        v.alarmMin = r.read_le<std::uint16_t>();
        v.alarmMax = r.read_le<std::uint16_t>();
        v.osdSymbol = r.read_le<std::uint8_t>();
        r.read_bytes(reinterpret_cast<std::uint8_t*>(v.label.data()), sizeof(v.label));
        return v;
    }
};

struct MSP2_INAV_SET_TEMP_SENSOR_CONFIG__request {
    std::uint8_t type;
    uint64_t address;
    std::uint16_t alarmMin;
    std::uint16_t alarmMax;
    std::uint8_t osdSymbol;
    std::array<char,TEMPERATURE_LABEL_LEN> label;

    static std::vector<std::uint8_t> pack(const MSP2_INAV_SET_TEMP_SENSOR_CONFIG__request& v) {
        BufferWriter w;
        w.write_le(v.type);
        { auto bytes = uint64_t::pack(v.address); w.write_bytes(bytes.data(), bytes.size()); }
        w.write_le(v.alarmMin);
        w.write_le(v.alarmMax);
        w.write_le(v.osdSymbol);
        w.write_bytes(v.label.data(), sizeof(v.label));
        return std::move(w.buf);
    }

    static MSP2_INAV_SET_TEMP_SENSOR_CONFIG__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_INAV_SET_TEMP_SENSOR_CONFIG__request v{};
        v.type = r.read_le<std::uint8_t>();
        static constexpr std::size_t __address_size = []{ uint64_t tmp{}; auto b = uint64_t::pack(tmp); return b.size(); }();
        { std::vector<std::uint8_t> chunk(__address_size); r.read_bytes(chunk.data(), __address_size); v.address = uint64_t::unpack(chunk); }
        v.alarmMin = r.read_le<std::uint16_t>();
        v.alarmMax = r.read_le<std::uint16_t>();
        v.osdSymbol = r.read_le<std::uint8_t>();
        r.read_bytes(reinterpret_cast<std::uint8_t*>(v.label.data()), sizeof(v.label));
        return v;
    }
};

struct MSP2_INAV_TEMPERATURES__reply {
    std::int16_t temperature;

    static std::vector<std::uint8_t> pack(const MSP2_INAV_TEMPERATURES__reply& v) {
        BufferWriter w;
        w.write_le(v.temperature);
        return std::move(w.buf);
    }

    static MSP2_INAV_TEMPERATURES__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_INAV_TEMPERATURES__reply v{};
        v.temperature = r.read_le<std::int16_t>();
        return v;
    }
};

struct MSP_SIMULATOR__request {
    std::uint8_t simulatorVersion;
    std::uint8_t simulatorFlags_t;
    std::uint8_t gpsFixType;
    std::uint8_t gpsNumSat;
    std::int32_t gpsLat;
    std::int32_t gpsLon;
    std::int32_t gpsAlt;
    std::uint16_t gpsSpeed;
    std::uint16_t gpsCourse;
    std::int16_t gpsVelN;
    std::int16_t gpsVelE;
    std::int16_t gpsVelD;
    std::int16_t imuRoll;
    std::int16_t imuPitch;
    std::int16_t imuYaw;
    std::int16_t accX;
    std::int16_t accY;
    std::int16_t accZ;
    std::int16_t gyroX;
    std::int16_t gyroY;
    std::int16_t gyroZ;
    std::uint32_t baroPressure;
    std::int16_t magX;
    std::int16_t magY;
    std::int16_t magZ;
    std::uint8_t vbat;
    std::uint16_t airspeed;
    std::uint8_t extFlags;

    static std::vector<std::uint8_t> pack(const MSP_SIMULATOR__request& v) {
        BufferWriter w;
        w.write_le(v.simulatorVersion);
        w.write_le(v.simulatorFlags_t);
        w.write_le(v.gpsFixType);
        w.write_le(v.gpsNumSat);
        w.write_le(v.gpsLat);
        w.write_le(v.gpsLon);
        w.write_le(v.gpsAlt);
        w.write_le(v.gpsSpeed);
        w.write_le(v.gpsCourse);
        w.write_le(v.gpsVelN);
        w.write_le(v.gpsVelE);
        w.write_le(v.gpsVelD);
        w.write_le(v.imuRoll);
        w.write_le(v.imuPitch);
        w.write_le(v.imuYaw);
        w.write_le(v.accX);
        w.write_le(v.accY);
        w.write_le(v.accZ);
        w.write_le(v.gyroX);
        w.write_le(v.gyroY);
        w.write_le(v.gyroZ);
        w.write_le(v.baroPressure);
        w.write_le(v.magX);
        w.write_le(v.magY);
        w.write_le(v.magZ);
        w.write_le(v.vbat);
        w.write_le(v.airspeed);
        w.write_le(v.extFlags);
        return std::move(w.buf);
    }

    static MSP_SIMULATOR__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_SIMULATOR__request v{};
        v.simulatorVersion = r.read_le<std::uint8_t>();
        v.simulatorFlags_t = r.read_le<std::uint8_t>();
        v.gpsFixType = r.read_le<std::uint8_t>();
        v.gpsNumSat = r.read_le<std::uint8_t>();
        v.gpsLat = r.read_le<std::int32_t>();
        v.gpsLon = r.read_le<std::int32_t>();
        v.gpsAlt = r.read_le<std::int32_t>();
        v.gpsSpeed = r.read_le<std::uint16_t>();
        v.gpsCourse = r.read_le<std::uint16_t>();
        v.gpsVelN = r.read_le<std::int16_t>();
        v.gpsVelE = r.read_le<std::int16_t>();
        v.gpsVelD = r.read_le<std::int16_t>();
        v.imuRoll = r.read_le<std::int16_t>();
        v.imuPitch = r.read_le<std::int16_t>();
        v.imuYaw = r.read_le<std::int16_t>();
        v.accX = r.read_le<std::int16_t>();
        v.accY = r.read_le<std::int16_t>();
        v.accZ = r.read_le<std::int16_t>();
        v.gyroX = r.read_le<std::int16_t>();
        v.gyroY = r.read_le<std::int16_t>();
        v.gyroZ = r.read_le<std::int16_t>();
        v.baroPressure = r.read_le<std::uint32_t>();
        v.magX = r.read_le<std::int16_t>();
        v.magY = r.read_le<std::int16_t>();
        v.magZ = r.read_le<std::int16_t>();
        v.vbat = r.read_le<std::uint8_t>();
        v.airspeed = r.read_le<std::uint16_t>();
        v.extFlags = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP_SIMULATOR__reply {
    std::uint16_t stabilizedRoll;
    std::uint16_t stabilizedPitch;
    std::uint16_t stabilizedYaw;
    std::uint16_t stabilizedThrottle;
    std::uint8_t debugFlags;
    std::uint32_t debugValue;
    std::int16_t attitudeRoll;
    std::int16_t attitudePitch;
    std::int16_t attitudeYaw;
    std::uint8_t osdHeader;
    std::uint8_t osdRows;
    std::uint8_t osdCols;
    std::uint8_t osdStartY;
    std::uint8_t osdStartX;
    std::vector<uint8_t> osdRleData;

    static std::vector<std::uint8_t> pack(const MSP_SIMULATOR__reply& v) {
        BufferWriter w;
        w.write_le(v.stabilizedRoll);
        w.write_le(v.stabilizedPitch);
        w.write_le(v.stabilizedYaw);
        w.write_le(v.stabilizedThrottle);
        w.write_le(v.debugFlags);
        w.write_le(v.debugValue);
        w.write_le(v.attitudeRoll);
        w.write_le(v.attitudePitch);
        w.write_le(v.attitudeYaw);
        w.write_le(v.osdHeader);
        w.write_le(v.osdRows);
        w.write_le(v.osdCols);
        w.write_le(v.osdStartY);
        w.write_le(v.osdStartX);
        for (const auto& e : v.osdRleData) w.write_le(e);
        return std::move(w.buf);
    }

    static MSP_SIMULATOR__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP_SIMULATOR__reply v{};
        v.stabilizedRoll = r.read_le<std::uint16_t>();
        v.stabilizedPitch = r.read_le<std::uint16_t>();
        v.stabilizedYaw = r.read_le<std::uint16_t>();
        v.stabilizedThrottle = r.read_le<std::uint16_t>();
        v.debugFlags = r.read_le<std::uint8_t>();
        v.debugValue = r.read_le<std::uint32_t>();
        v.attitudeRoll = r.read_le<std::int16_t>();
        v.attitudePitch = r.read_le<std::int16_t>();
        v.attitudeYaw = r.read_le<std::int16_t>();
        v.osdHeader = r.read_le<std::uint8_t>();
        v.osdRows = r.read_le<std::uint8_t>();
        v.osdCols = r.read_le<std::uint8_t>();
        v.osdStartY = r.read_le<std::uint8_t>();
        v.osdStartX = r.read_le<std::uint8_t>();
        v.osdRleData.clear();
        while (r.remaining() >= sizeof(uint8_t)) v.osdRleData.push_back(r.read_le<uint8_t>());
        return v;
    }
};

struct MSP2_INAV_SERVO_MIXER__reply {
    std::uint8_t targetChannel;
    std::uint8_t inputSource;
    std::uint16_t rate;
    std::uint8_t speed;
    std::uint8_t conditionId;
    std::uint8_t targetChannel;
    std::uint8_t inputSource;
    std::uint16_t rate;
    std::uint8_t speed;
    std::uint8_t conditionId;

    static std::vector<std::uint8_t> pack(const MSP2_INAV_SERVO_MIXER__reply& v) {
        BufferWriter w;
        w.write_le(v.targetChannel);
        w.write_le(v.inputSource);
        w.write_le(v.rate);
        w.write_le(v.speed);
        w.write_le(v.conditionId);
        w.write_le(v.targetChannel);
        w.write_le(v.inputSource);
        w.write_le(v.rate);
        w.write_le(v.speed);
        w.write_le(v.conditionId);
        return std::move(w.buf);
    }

    static MSP2_INAV_SERVO_MIXER__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_INAV_SERVO_MIXER__reply v{};
        v.targetChannel = r.read_le<std::uint8_t>();
        v.inputSource = r.read_le<std::uint8_t>();
        v.rate = r.read_le<std::uint16_t>();
        v.speed = r.read_le<std::uint8_t>();
        v.conditionId = r.read_le<std::uint8_t>();
        v.targetChannel = r.read_le<std::uint8_t>();
        v.inputSource = r.read_le<std::uint8_t>();
        v.rate = r.read_le<std::uint16_t>();
        v.speed = r.read_le<std::uint8_t>();
        v.conditionId = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP2_INAV_SET_SERVO_MIXER__request {
    std::uint8_t ruleIndex;
    std::uint8_t targetChannel;
    std::uint8_t inputSource;
    std::uint16_t rate;
    std::uint8_t speed;
    std::uint8_t conditionId;

    static std::vector<std::uint8_t> pack(const MSP2_INAV_SET_SERVO_MIXER__request& v) {
        BufferWriter w;
        w.write_le(v.ruleIndex);
        w.write_le(v.targetChannel);
        w.write_le(v.inputSource);
        w.write_le(v.rate);
        w.write_le(v.speed);
        w.write_le(v.conditionId);
        return std::move(w.buf);
    }

    static MSP2_INAV_SET_SERVO_MIXER__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_INAV_SET_SERVO_MIXER__request v{};
        v.ruleIndex = r.read_le<std::uint8_t>();
        v.targetChannel = r.read_le<std::uint8_t>();
        v.inputSource = r.read_le<std::uint8_t>();
        v.rate = r.read_le<std::uint16_t>();
        v.speed = r.read_le<std::uint8_t>();
        v.conditionId = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP2_INAV_LOGIC_CONDITIONS__reply {
    std::uint8_t enabled;
    std::uint8_t activatorId;
    std::uint8_t operation;
    std::uint8_t operandAType;
    std::uint32_t operandAValue;
    std::uint8_t operandBType;
    std::uint32_t operandBValue;
    std::uint8_t flags;

    static std::vector<std::uint8_t> pack(const MSP2_INAV_LOGIC_CONDITIONS__reply& v) {
        BufferWriter w;
        w.write_le(v.enabled);
        w.write_le(v.activatorId);
        w.write_le(v.operation);
        w.write_le(v.operandAType);
        w.write_le(v.operandAValue);
        w.write_le(v.operandBType);
        w.write_le(v.operandBValue);
        w.write_le(v.flags);
        return std::move(w.buf);
    }

    static MSP2_INAV_LOGIC_CONDITIONS__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_INAV_LOGIC_CONDITIONS__reply v{};
        v.enabled = r.read_le<std::uint8_t>();
        v.activatorId = r.read_le<std::uint8_t>();
        v.operation = r.read_le<std::uint8_t>();
        v.operandAType = r.read_le<std::uint8_t>();
        v.operandAValue = r.read_le<std::uint32_t>();
        v.operandBType = r.read_le<std::uint8_t>();
        v.operandBValue = r.read_le<std::uint32_t>();
        v.flags = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP2_INAV_SET_LOGIC_CONDITIONS__request {
    std::uint8_t conditionIndex;
    std::uint8_t enabled;
    std::uint8_t activatorId;
    std::uint8_t operation;
    std::uint8_t operandAType;
    std::uint32_t operandAValue;
    std::uint8_t operandBType;
    std::uint32_t operandBValue;
    std::uint8_t flags;

    static std::vector<std::uint8_t> pack(const MSP2_INAV_SET_LOGIC_CONDITIONS__request& v) {
        BufferWriter w;
        w.write_le(v.conditionIndex);
        w.write_le(v.enabled);
        w.write_le(v.activatorId);
        w.write_le(v.operation);
        w.write_le(v.operandAType);
        w.write_le(v.operandAValue);
        w.write_le(v.operandBType);
        w.write_le(v.operandBValue);
        w.write_le(v.flags);
        return std::move(w.buf);
    }

    static MSP2_INAV_SET_LOGIC_CONDITIONS__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_INAV_SET_LOGIC_CONDITIONS__request v{};
        v.conditionIndex = r.read_le<std::uint8_t>();
        v.enabled = r.read_le<std::uint8_t>();
        v.activatorId = r.read_le<std::uint8_t>();
        v.operation = r.read_le<std::uint8_t>();
        v.operandAType = r.read_le<std::uint8_t>();
        v.operandAValue = r.read_le<std::uint32_t>();
        v.operandBType = r.read_le<std::uint8_t>();
        v.operandBValue = r.read_le<std::uint32_t>();
        v.flags = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP2_INAV_LOGIC_CONDITIONS_STATUS__reply {
    std::vector<std::uint32_t> conditionValues;

    static std::vector<std::uint8_t> pack(const MSP2_INAV_LOGIC_CONDITIONS_STATUS__reply& v) {
        BufferWriter w;
        for (const auto& e : v.conditionValues) w.write_le(e);
        return std::move(w.buf);
    }

    static MSP2_INAV_LOGIC_CONDITIONS_STATUS__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_INAV_LOGIC_CONDITIONS_STATUS__reply v{};
        v.conditionValues.clear();
        while (r.remaining() >= sizeof(std::uint32_t)) v.conditionValues.push_back(r.read_le<std::uint32_t>());
        return v;
    }
};

struct MSP2_INAV_GVAR_STATUS__reply {
    std::vector<std::uint32_t> gvarValues;

    static std::vector<std::uint8_t> pack(const MSP2_INAV_GVAR_STATUS__reply& v) {
        BufferWriter w;
        for (const auto& e : v.gvarValues) w.write_le(e);
        return std::move(w.buf);
    }

    static MSP2_INAV_GVAR_STATUS__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_INAV_GVAR_STATUS__reply v{};
        v.gvarValues.clear();
        while (r.remaining() >= sizeof(std::uint32_t)) v.gvarValues.push_back(r.read_le<std::uint32_t>());
        return v;
    }
};

struct MSP2_INAV_PROGRAMMING_PID__reply {
    std::uint8_t enabled;
    std::uint8_t setpointType;
    std::uint32_t setpointValue;
    std::uint8_t measurementType;
    std::uint32_t measurementValue;
    std::uint16_t gainP;
    std::uint16_t gainI;
    std::uint16_t gainD;
    std::uint16_t gainFF;

    static std::vector<std::uint8_t> pack(const MSP2_INAV_PROGRAMMING_PID__reply& v) {
        BufferWriter w;
        w.write_le(v.enabled);
        w.write_le(v.setpointType);
        w.write_le(v.setpointValue);
        w.write_le(v.measurementType);
        w.write_le(v.measurementValue);
        w.write_le(v.gainP);
        w.write_le(v.gainI);
        w.write_le(v.gainD);
        w.write_le(v.gainFF);
        return std::move(w.buf);
    }

    static MSP2_INAV_PROGRAMMING_PID__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_INAV_PROGRAMMING_PID__reply v{};
        v.enabled = r.read_le<std::uint8_t>();
        v.setpointType = r.read_le<std::uint8_t>();
        v.setpointValue = r.read_le<std::uint32_t>();
        v.measurementType = r.read_le<std::uint8_t>();
        v.measurementValue = r.read_le<std::uint32_t>();
        v.gainP = r.read_le<std::uint16_t>();
        v.gainI = r.read_le<std::uint16_t>();
        v.gainD = r.read_le<std::uint16_t>();
        v.gainFF = r.read_le<std::uint16_t>();
        return v;
    }
};

struct MSP2_INAV_SET_PROGRAMMING_PID__request {
    std::uint8_t pidIndex;
    std::uint8_t enabled;
    std::uint8_t setpointType;
    std::uint32_t setpointValue;
    std::uint8_t measurementType;
    std::uint32_t measurementValue;
    std::uint16_t gainP;
    std::uint16_t gainI;
    std::uint16_t gainD;
    std::uint16_t gainFF;

    static std::vector<std::uint8_t> pack(const MSP2_INAV_SET_PROGRAMMING_PID__request& v) {
        BufferWriter w;
        w.write_le(v.pidIndex);
        w.write_le(v.enabled);
        w.write_le(v.setpointType);
        w.write_le(v.setpointValue);
        w.write_le(v.measurementType);
        w.write_le(v.measurementValue);
        w.write_le(v.gainP);
        w.write_le(v.gainI);
        w.write_le(v.gainD);
        w.write_le(v.gainFF);
        return std::move(w.buf);
    }

    static MSP2_INAV_SET_PROGRAMMING_PID__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_INAV_SET_PROGRAMMING_PID__request v{};
        v.pidIndex = r.read_le<std::uint8_t>();
        v.enabled = r.read_le<std::uint8_t>();
        v.setpointType = r.read_le<std::uint8_t>();
        v.setpointValue = r.read_le<std::uint32_t>();
        v.measurementType = r.read_le<std::uint8_t>();
        v.measurementValue = r.read_le<std::uint32_t>();
        v.gainP = r.read_le<std::uint16_t>();
        v.gainI = r.read_le<std::uint16_t>();
        v.gainD = r.read_le<std::uint16_t>();
        v.gainFF = r.read_le<std::uint16_t>();
        return v;
    }
};

struct MSP2_INAV_PROGRAMMING_PID_STATUS__reply {
    std::vector<std::uint32_t> pidOutputs;

    static std::vector<std::uint8_t> pack(const MSP2_INAV_PROGRAMMING_PID_STATUS__reply& v) {
        BufferWriter w;
        for (const auto& e : v.pidOutputs) w.write_le(e);
        return std::move(w.buf);
    }

    static MSP2_INAV_PROGRAMMING_PID_STATUS__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_INAV_PROGRAMMING_PID_STATUS__reply v{};
        v.pidOutputs.clear();
        while (r.remaining() >= sizeof(std::uint32_t)) v.pidOutputs.push_back(r.read_le<std::uint32_t>());
        return v;
    }
};

struct MSP2_PID__reply {
    std::uint8_t P;
    std::uint8_t I;
    std::uint8_t D;
    std::uint8_t FF;

    static std::vector<std::uint8_t> pack(const MSP2_PID__reply& v) {
        BufferWriter w;
        w.write_le(v.P);
        w.write_le(v.I);
        w.write_le(v.D);
        w.write_le(v.FF);
        return std::move(w.buf);
    }

    static MSP2_PID__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_PID__reply v{};
        v.P = r.read_le<std::uint8_t>();
        v.I = r.read_le<std::uint8_t>();
        v.D = r.read_le<std::uint8_t>();
        v.FF = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP2_SET_PID__request {
    std::uint8_t P;
    std::uint8_t I;
    std::uint8_t D;
    std::uint8_t FF;

    static std::vector<std::uint8_t> pack(const MSP2_SET_PID__request& v) {
        BufferWriter w;
        w.write_le(v.P);
        w.write_le(v.I);
        w.write_le(v.D);
        w.write_le(v.FF);
        return std::move(w.buf);
    }

    static MSP2_SET_PID__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_SET_PID__request v{};
        v.P = r.read_le<std::uint8_t>();
        v.I = r.read_le<std::uint8_t>();
        v.D = r.read_le<std::uint8_t>();
        v.FF = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP2_INAV_FWUPDT_PREPARE__request {
    std::uint32_t firmwareSize;

    static std::vector<std::uint8_t> pack(const MSP2_INAV_FWUPDT_PREPARE__request& v) {
        BufferWriter w;
        w.write_le(v.firmwareSize);
        return std::move(w.buf);
    }

    static MSP2_INAV_FWUPDT_PREPARE__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_INAV_FWUPDT_PREPARE__request v{};
        v.firmwareSize = r.read_le<std::uint32_t>();
        return v;
    }
};

struct MSP2_INAV_FWUPDT_STORE__request {
    std::vector<uint8_t> firmwareChunk;

    static std::vector<std::uint8_t> pack(const MSP2_INAV_FWUPDT_STORE__request& v) {
        BufferWriter w;
        for (const auto& e : v.firmwareChunk) w.write_le(e);
        return std::move(w.buf);
    }

    static MSP2_INAV_FWUPDT_STORE__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_INAV_FWUPDT_STORE__request v{};
        v.firmwareChunk.clear();
        while (r.remaining() >= sizeof(uint8_t)) v.firmwareChunk.push_back(r.read_le<uint8_t>());
        return v;
    }
};

struct MSP2_INAV_FWUPDT_EXEC__request {
    std::uint8_t updateType;

    static std::vector<std::uint8_t> pack(const MSP2_INAV_FWUPDT_EXEC__request& v) {
        BufferWriter w;
        w.write_le(v.updateType);
        return std::move(w.buf);
    }

    static MSP2_INAV_FWUPDT_EXEC__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_INAV_FWUPDT_EXEC__request v{};
        v.updateType = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP2_INAV_SAFEHOME__request {
    std::uint8_t safehomeIndex;

    static std::vector<std::uint8_t> pack(const MSP2_INAV_SAFEHOME__request& v) {
        BufferWriter w;
        w.write_le(v.safehomeIndex);
        return std::move(w.buf);
    }

    static MSP2_INAV_SAFEHOME__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_INAV_SAFEHOME__request v{};
        v.safehomeIndex = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP2_INAV_SAFEHOME__reply {
    std::uint8_t safehomeIndex;
    std::uint8_t enabled;
    std::int32_t latitude;
    std::int32_t longitude;

    static std::vector<std::uint8_t> pack(const MSP2_INAV_SAFEHOME__reply& v) {
        BufferWriter w;
        w.write_le(v.safehomeIndex);
        w.write_le(v.enabled);
        w.write_le(v.latitude);
        w.write_le(v.longitude);
        return std::move(w.buf);
    }

    static MSP2_INAV_SAFEHOME__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_INAV_SAFEHOME__reply v{};
        v.safehomeIndex = r.read_le<std::uint8_t>();
        v.enabled = r.read_le<std::uint8_t>();
        v.latitude = r.read_le<std::int32_t>();
        v.longitude = r.read_le<std::int32_t>();
        return v;
    }
};

struct MSP2_INAV_SET_SAFEHOME__request {
    std::uint8_t safehomeIndex;
    std::uint8_t enabled;
    std::int32_t latitude;
    std::int32_t longitude;

    static std::vector<std::uint8_t> pack(const MSP2_INAV_SET_SAFEHOME__request& v) {
        BufferWriter w;
        w.write_le(v.safehomeIndex);
        w.write_le(v.enabled);
        w.write_le(v.latitude);
        w.write_le(v.longitude);
        return std::move(w.buf);
    }

    static MSP2_INAV_SET_SAFEHOME__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_INAV_SET_SAFEHOME__request v{};
        v.safehomeIndex = r.read_le<std::uint8_t>();
        v.enabled = r.read_le<std::uint8_t>();
        v.latitude = r.read_le<std::int32_t>();
        v.longitude = r.read_le<std::int32_t>();
        return v;
    }
};

struct MSP2_INAV_MISC2__reply {
    std::uint32_t uptimeSeconds;
    std::uint32_t flightTimeSeconds;
    std::uint8_t throttlePercent;
    std::uint8_t autoThrottleFlag;

    static std::vector<std::uint8_t> pack(const MSP2_INAV_MISC2__reply& v) {
        BufferWriter w;
        w.write_le(v.uptimeSeconds);
        w.write_le(v.flightTimeSeconds);
        w.write_le(v.throttlePercent);
        w.write_le(v.autoThrottleFlag);
        return std::move(w.buf);
    }

    static MSP2_INAV_MISC2__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_INAV_MISC2__reply v{};
        v.uptimeSeconds = r.read_le<std::uint32_t>();
        v.flightTimeSeconds = r.read_le<std::uint32_t>();
        v.throttlePercent = r.read_le<std::uint8_t>();
        v.autoThrottleFlag = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP2_INAV_LOGIC_CONDITIONS_SINGLE__request {
    std::uint8_t conditionIndex;

    static std::vector<std::uint8_t> pack(const MSP2_INAV_LOGIC_CONDITIONS_SINGLE__request& v) {
        BufferWriter w;
        w.write_le(v.conditionIndex);
        return std::move(w.buf);
    }

    static MSP2_INAV_LOGIC_CONDITIONS_SINGLE__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_INAV_LOGIC_CONDITIONS_SINGLE__request v{};
        v.conditionIndex = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP2_INAV_LOGIC_CONDITIONS_SINGLE__reply {
    std::uint8_t enabled;
    std::uint8_t activatorId;
    std::uint8_t operation;
    std::uint8_t operandAType;
    std::uint32_t operandAValue;
    std::uint8_t operandBType;
    std::uint32_t operandBValue;
    std::uint8_t flags;

    static std::vector<std::uint8_t> pack(const MSP2_INAV_LOGIC_CONDITIONS_SINGLE__reply& v) {
        BufferWriter w;
        w.write_le(v.enabled);
        w.write_le(v.activatorId);
        w.write_le(v.operation);
        w.write_le(v.operandAType);
        w.write_le(v.operandAValue);
        w.write_le(v.operandBType);
        w.write_le(v.operandBValue);
        w.write_le(v.flags);
        return std::move(w.buf);
    }

    static MSP2_INAV_LOGIC_CONDITIONS_SINGLE__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_INAV_LOGIC_CONDITIONS_SINGLE__reply v{};
        v.enabled = r.read_le<std::uint8_t>();
        v.activatorId = r.read_le<std::uint8_t>();
        v.operation = r.read_le<std::uint8_t>();
        v.operandAType = r.read_le<std::uint8_t>();
        v.operandAValue = r.read_le<std::uint32_t>();
        v.operandBType = r.read_le<std::uint8_t>();
        v.operandBValue = r.read_le<std::uint32_t>();
        v.flags = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP2_INAV_ESC_RPM__reply {
    std::uint32_t escRpm;

    static std::vector<std::uint8_t> pack(const MSP2_INAV_ESC_RPM__reply& v) {
        BufferWriter w;
        w.write_le(v.escRpm);
        return std::move(w.buf);
    }

    static MSP2_INAV_ESC_RPM__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_INAV_ESC_RPM__reply v{};
        v.escRpm = r.read_le<std::uint32_t>();
        return v;
    }
};

struct MSP2_INAV_ESC_TELEM__reply {
    std::uint8_t motorCount;
    std::vector<escSensorData_t> escData;

    static std::vector<std::uint8_t> pack(const MSP2_INAV_ESC_TELEM__reply& v) {
        BufferWriter w;
        w.write_le(v.motorCount);
        for (const auto& e : v.escData) {
            auto bytes = escSensorData_t::pack(e);
            w.write_bytes(bytes.data(), bytes.size());
        }
        return std::move(w.buf);
    }

    static MSP2_INAV_ESC_TELEM__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_INAV_ESC_TELEM__reply v{};
        v.motorCount = r.read_le<std::uint8_t>();
        v.escData.clear();
        static constexpr std::size_t __escData_elem_size = []{ escSensorData_t tmp{}; auto b = escSensorData_t::pack(tmp); return b.size(); }();
        while (r.remaining() >= __escData_elem_size) {
            std::vector<std::uint8_t> chunk(__escData_elem_size);
            r.read_bytes(chunk.data(), __escData_elem_size);
            v.escData.push_back(escSensorData_t::unpack(chunk));
        }
        return v;
    }
};

struct MSP2_INAV_LED_STRIP_CONFIG_EX__reply {
    std::uint16_t ledConfig;

    static std::vector<std::uint8_t> pack(const MSP2_INAV_LED_STRIP_CONFIG_EX__reply& v) {
        BufferWriter w;
        w.write_le(v.ledConfig);
        return std::move(w.buf);
    }

    static MSP2_INAV_LED_STRIP_CONFIG_EX__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_INAV_LED_STRIP_CONFIG_EX__reply v{};
        v.ledConfig = r.read_le<std::uint16_t>();
        return v;
    }
};

struct MSP2_INAV_SET_LED_STRIP_CONFIG_EX__request {
    std::uint8_t ledIndex;
    std::uint16_t ledConfig;

    static std::vector<std::uint8_t> pack(const MSP2_INAV_SET_LED_STRIP_CONFIG_EX__request& v) {
        BufferWriter w;
        w.write_le(v.ledIndex);
        w.write_le(v.ledConfig);
        return std::move(w.buf);
    }

    static MSP2_INAV_SET_LED_STRIP_CONFIG_EX__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_INAV_SET_LED_STRIP_CONFIG_EX__request v{};
        v.ledIndex = r.read_le<std::uint8_t>();
        v.ledConfig = r.read_le<std::uint16_t>();
        return v;
    }
};

struct MSP2_INAV_FW_APPROACH__request {
    std::uint8_t approachIndex;

    static std::vector<std::uint8_t> pack(const MSP2_INAV_FW_APPROACH__request& v) {
        BufferWriter w;
        w.write_le(v.approachIndex);
        return std::move(w.buf);
    }

    static MSP2_INAV_FW_APPROACH__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_INAV_FW_APPROACH__request v{};
        v.approachIndex = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP2_INAV_FW_APPROACH__reply {
    std::uint8_t approachIndex;
    std::uint32_t approachAlt;
    std::uint32_t landAlt;
    std::uint8_t approachDirection;
    std::int16_t landHeading1;
    std::int16_t landHeading2;
    std::uint8_t isSeaLevelRef;

    static std::vector<std::uint8_t> pack(const MSP2_INAV_FW_APPROACH__reply& v) {
        BufferWriter w;
        w.write_le(v.approachIndex);
        w.write_le(v.approachAlt);
        w.write_le(v.landAlt);
        w.write_le(v.approachDirection);
        w.write_le(v.landHeading1);
        w.write_le(v.landHeading2);
        w.write_le(v.isSeaLevelRef);
        return std::move(w.buf);
    }

    static MSP2_INAV_FW_APPROACH__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_INAV_FW_APPROACH__reply v{};
        v.approachIndex = r.read_le<std::uint8_t>();
        v.approachAlt = r.read_le<std::uint32_t>();
        v.landAlt = r.read_le<std::uint32_t>();
        v.approachDirection = r.read_le<std::uint8_t>();
        v.landHeading1 = r.read_le<std::int16_t>();
        v.landHeading2 = r.read_le<std::int16_t>();
        v.isSeaLevelRef = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP2_INAV_SET_FW_APPROACH__request {
    std::uint8_t approachIndex;
    std::uint32_t approachAlt;
    std::uint32_t landAlt;
    std::uint8_t approachDirection;
    std::int16_t landHeading1;
    std::int16_t landHeading2;
    std::uint8_t isSeaLevelRef;

    static std::vector<std::uint8_t> pack(const MSP2_INAV_SET_FW_APPROACH__request& v) {
        BufferWriter w;
        w.write_le(v.approachIndex);
        w.write_le(v.approachAlt);
        w.write_le(v.landAlt);
        w.write_le(v.approachDirection);
        w.write_le(v.landHeading1);
        w.write_le(v.landHeading2);
        w.write_le(v.isSeaLevelRef);
        return std::move(w.buf);
    }

    static MSP2_INAV_SET_FW_APPROACH__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_INAV_SET_FW_APPROACH__request v{};
        v.approachIndex = r.read_le<std::uint8_t>();
        v.approachAlt = r.read_le<std::uint32_t>();
        v.landAlt = r.read_le<std::uint32_t>();
        v.approachDirection = r.read_le<std::uint8_t>();
        v.landHeading1 = r.read_le<std::int16_t>();
        v.landHeading2 = r.read_le<std::int16_t>();
        v.isSeaLevelRef = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP2_INAV_GPS_UBLOX_COMMAND__request {
    std::vector<uint8_t> ubxCommand;

    static std::vector<std::uint8_t> pack(const MSP2_INAV_GPS_UBLOX_COMMAND__request& v) {
        BufferWriter w;
        for (const auto& e : v.ubxCommand) w.write_le(e);
        return std::move(w.buf);
    }

    static MSP2_INAV_GPS_UBLOX_COMMAND__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_INAV_GPS_UBLOX_COMMAND__request v{};
        v.ubxCommand.clear();
        while (r.remaining() >= sizeof(uint8_t)) v.ubxCommand.push_back(r.read_le<uint8_t>());
        return v;
    }
};

struct MSP2_INAV_RATE_DYNAMICS__reply {
    std::uint8_t sensitivityCenter;
    std::uint8_t sensitivityEnd;
    std::uint8_t correctionCenter;
    std::uint8_t correctionEnd;
    std::uint8_t weightCenter;
    std::uint8_t weightEnd;

    static std::vector<std::uint8_t> pack(const MSP2_INAV_RATE_DYNAMICS__reply& v) {
        BufferWriter w;
        w.write_le(v.sensitivityCenter);
        w.write_le(v.sensitivityEnd);
        w.write_le(v.correctionCenter);
        w.write_le(v.correctionEnd);
        w.write_le(v.weightCenter);
        w.write_le(v.weightEnd);
        return std::move(w.buf);
    }

    static MSP2_INAV_RATE_DYNAMICS__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_INAV_RATE_DYNAMICS__reply v{};
        v.sensitivityCenter = r.read_le<std::uint8_t>();
        v.sensitivityEnd = r.read_le<std::uint8_t>();
        v.correctionCenter = r.read_le<std::uint8_t>();
        v.correctionEnd = r.read_le<std::uint8_t>();
        v.weightCenter = r.read_le<std::uint8_t>();
        v.weightEnd = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP2_INAV_SET_RATE_DYNAMICS__request {
    std::uint8_t sensitivityCenter;
    std::uint8_t sensitivityEnd;
    std::uint8_t correctionCenter;
    std::uint8_t correctionEnd;
    std::uint8_t weightCenter;
    std::uint8_t weightEnd;

    static std::vector<std::uint8_t> pack(const MSP2_INAV_SET_RATE_DYNAMICS__request& v) {
        BufferWriter w;
        w.write_le(v.sensitivityCenter);
        w.write_le(v.sensitivityEnd);
        w.write_le(v.correctionCenter);
        w.write_le(v.correctionEnd);
        w.write_le(v.weightCenter);
        w.write_le(v.weightEnd);
        return std::move(w.buf);
    }

    static MSP2_INAV_SET_RATE_DYNAMICS__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_INAV_SET_RATE_DYNAMICS__request v{};
        v.sensitivityCenter = r.read_le<std::uint8_t>();
        v.sensitivityEnd = r.read_le<std::uint8_t>();
        v.correctionCenter = r.read_le<std::uint8_t>();
        v.correctionEnd = r.read_le<std::uint8_t>();
        v.weightCenter = r.read_le<std::uint8_t>();
        v.weightEnd = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP2_INAV_EZ_TUNE__reply {
    std::uint8_t enabled;
    std::uint16_t filterHz;
    std::uint8_t axisRatio;
    std::uint8_t response;
    std::uint8_t damping;
    std::uint8_t stability;
    std::uint8_t aggressiveness;
    std::uint8_t rate;
    std::uint8_t expo;
    std::uint8_t snappiness;

    static std::vector<std::uint8_t> pack(const MSP2_INAV_EZ_TUNE__reply& v) {
        BufferWriter w;
        w.write_le(v.enabled);
        w.write_le(v.filterHz);
        w.write_le(v.axisRatio);
        w.write_le(v.response);
        w.write_le(v.damping);
        w.write_le(v.stability);
        w.write_le(v.aggressiveness);
        w.write_le(v.rate);
        w.write_le(v.expo);
        w.write_le(v.snappiness);
        return std::move(w.buf);
    }

    static MSP2_INAV_EZ_TUNE__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_INAV_EZ_TUNE__reply v{};
        v.enabled = r.read_le<std::uint8_t>();
        v.filterHz = r.read_le<std::uint16_t>();
        v.axisRatio = r.read_le<std::uint8_t>();
        v.response = r.read_le<std::uint8_t>();
        v.damping = r.read_le<std::uint8_t>();
        v.stability = r.read_le<std::uint8_t>();
        v.aggressiveness = r.read_le<std::uint8_t>();
        v.rate = r.read_le<std::uint8_t>();
        v.expo = r.read_le<std::uint8_t>();
        v.snappiness = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP2_INAV_EZ_TUNE_SET__request {
    std::uint8_t enabled;
    std::uint16_t filterHz;
    std::uint8_t axisRatio;
    std::uint8_t response;
    std::uint8_t damping;
    std::uint8_t stability;
    std::uint8_t aggressiveness;
    std::uint8_t rate;
    std::uint8_t expo;
    std::uint8_t snappiness;

    static std::vector<std::uint8_t> pack(const MSP2_INAV_EZ_TUNE_SET__request& v) {
        BufferWriter w;
        w.write_le(v.enabled);
        w.write_le(v.filterHz);
        w.write_le(v.axisRatio);
        w.write_le(v.response);
        w.write_le(v.damping);
        w.write_le(v.stability);
        w.write_le(v.aggressiveness);
        w.write_le(v.rate);
        w.write_le(v.expo);
        w.write_le(v.snappiness);
        return std::move(w.buf);
    }

    static MSP2_INAV_EZ_TUNE_SET__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_INAV_EZ_TUNE_SET__request v{};
        v.enabled = r.read_le<std::uint8_t>();
        v.filterHz = r.read_le<std::uint16_t>();
        v.axisRatio = r.read_le<std::uint8_t>();
        v.response = r.read_le<std::uint8_t>();
        v.damping = r.read_le<std::uint8_t>();
        v.stability = r.read_le<std::uint8_t>();
        v.aggressiveness = r.read_le<std::uint8_t>();
        v.rate = r.read_le<std::uint8_t>();
        v.expo = r.read_le<std::uint8_t>();
        v.snappiness = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP2_INAV_SELECT_MIXER_PROFILE__request {
    std::uint8_t mixerProfileIndex;

    static std::vector<std::uint8_t> pack(const MSP2_INAV_SELECT_MIXER_PROFILE__request& v) {
        BufferWriter w;
        w.write_le(v.mixerProfileIndex);
        return std::move(w.buf);
    }

    static MSP2_INAV_SELECT_MIXER_PROFILE__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_INAV_SELECT_MIXER_PROFILE__request v{};
        v.mixerProfileIndex = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP2_ADSB_VEHICLE_LIST__reply {
    std::uint8_t maxVehicles;
    std::uint8_t callsignLength;
    std::uint32_t totalVehicleMsgs;
    std::uint32_t totalHeartbeatMsgs;
    std::vector<adsbVehicle_t> adsbVehicle;

    static std::vector<std::uint8_t> pack(const MSP2_ADSB_VEHICLE_LIST__reply& v) {
        BufferWriter w;
        w.write_le(v.maxVehicles);
        w.write_le(v.callsignLength);
        w.write_le(v.totalVehicleMsgs);
        w.write_le(v.totalHeartbeatMsgs);
        for (const auto& e : v.adsbVehicle) {
            auto bytes = adsbVehicle_t::pack(e);
            w.write_bytes(bytes.data(), bytes.size());
        }
        return std::move(w.buf);
    }

    static MSP2_ADSB_VEHICLE_LIST__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_ADSB_VEHICLE_LIST__reply v{};
        v.maxVehicles = r.read_le<std::uint8_t>();
        v.callsignLength = r.read_le<std::uint8_t>();
        v.totalVehicleMsgs = r.read_le<std::uint32_t>();
        v.totalHeartbeatMsgs = r.read_le<std::uint32_t>();
        v.adsbVehicle.clear();
        static constexpr std::size_t __adsbVehicle_elem_size = []{ adsbVehicle_t tmp{}; auto b = adsbVehicle_t::pack(tmp); return b.size(); }();
        while (r.remaining() >= __adsbVehicle_elem_size) {
            std::vector<std::uint8_t> chunk(__adsbVehicle_elem_size);
            r.read_bytes(chunk.data(), __adsbVehicle_elem_size);
            v.adsbVehicle.push_back(adsbVehicle_t::unpack(chunk));
        }
        return v;
    }
};

struct MSP2_INAV_CUSTOM_OSD_ELEMENTS__reply {
    std::uint8_t maxElements;
    std::uint8_t maxTextLength;
    std::uint8_t maxParts;

    static std::vector<std::uint8_t> pack(const MSP2_INAV_CUSTOM_OSD_ELEMENTS__reply& v) {
        BufferWriter w;
        w.write_le(v.maxElements);
        w.write_le(v.maxTextLength);
        w.write_le(v.maxParts);
        return std::move(w.buf);
    }

    static MSP2_INAV_CUSTOM_OSD_ELEMENTS__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_INAV_CUSTOM_OSD_ELEMENTS__reply v{};
        v.maxElements = r.read_le<std::uint8_t>();
        v.maxTextLength = r.read_le<std::uint8_t>();
        v.maxParts = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP2_INAV_CUSTOM_OSD_ELEMENT__request {
    std::uint8_t elementIndex;

    static std::vector<std::uint8_t> pack(const MSP2_INAV_CUSTOM_OSD_ELEMENT__request& v) {
        BufferWriter w;
        w.write_le(v.elementIndex);
        return std::move(w.buf);
    }

    static MSP2_INAV_CUSTOM_OSD_ELEMENT__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_INAV_CUSTOM_OSD_ELEMENT__request v{};
        v.elementIndex = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP2_INAV_CUSTOM_OSD_ELEMENT__reply__CUSTOM_ELEMENTS_PARTS {
    std::uint8_t partType;
    std::uint16_t partValue;

    static std::vector<std::uint8_t> pack(const MSP2_INAV_CUSTOM_OSD_ELEMENT__reply__CUSTOM_ELEMENTS_PARTS& v) {
        BufferWriter w;
        w.write_le(v.partType);
        w.write_le(v.partValue);
        return std::move(w.buf);
    }

    static MSP2_INAV_CUSTOM_OSD_ELEMENT__reply__CUSTOM_ELEMENTS_PARTS unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_INAV_CUSTOM_OSD_ELEMENT__reply__CUSTOM_ELEMENTS_PARTS v{};
        v.partType = r.read_le<std::uint8_t>();
        v.partValue = r.read_le<std::uint16_t>();
        return v;
    }
};

struct MSP2_INAV_CUSTOM_OSD_ELEMENT__reply {
    std::vector<MSP2_INAV_CUSTOM_OSD_ELEMENT__reply__CUSTOM_ELEMENTS_PARTS> CUSTOM_ELEMENTS_PARTS;
    std::uint8_t visibilityType;
    std::uint16_t visibilityValue;
    std::string elementText;

    static std::vector<std::uint8_t> pack(const MSP2_INAV_CUSTOM_OSD_ELEMENT__reply& v) {
        BufferWriter w;
        for (const auto& e : v.CUSTOM_ELEMENTS_PARTS) {
            auto bytes = MSP2_INAV_CUSTOM_OSD_ELEMENT__reply__CUSTOM_ELEMENTS_PARTS::pack(e);
            w.write_bytes(bytes.data(), bytes.size());
        }
        w.write_le(v.visibilityType);
        w.write_le(v.visibilityValue);
        w.write_string_bytes(v.elementText);
        return std::move(w.buf);
    }

    static MSP2_INAV_CUSTOM_OSD_ELEMENT__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_INAV_CUSTOM_OSD_ELEMENT__reply v{};
        v.CUSTOM_ELEMENTS_PARTS.clear();
        static constexpr std::size_t __CUSTOM_ELEMENTS_PARTS_elem_size = []{ MSP2_INAV_CUSTOM_OSD_ELEMENT__reply__CUSTOM_ELEMENTS_PARTS tmp{}; auto b = MSP2_INAV_CUSTOM_OSD_ELEMENT__reply__CUSTOM_ELEMENTS_PARTS::pack(tmp); return b.size(); }();
        while (r.remaining() >= __CUSTOM_ELEMENTS_PARTS_elem_size) {
            std::vector<std::uint8_t> chunk(__CUSTOM_ELEMENTS_PARTS_elem_size);
            r.read_bytes(chunk.data(), __CUSTOM_ELEMENTS_PARTS_elem_size);
            v.CUSTOM_ELEMENTS_PARTS.push_back(MSP2_INAV_CUSTOM_OSD_ELEMENT__reply__CUSTOM_ELEMENTS_PARTS::unpack(chunk));
        }
        v.visibilityType = r.read_le<std::uint8_t>();
        v.visibilityValue = r.read_le<std::uint16_t>();
        v.elementText = r.read_string_rest();
        return v;
    }
};

struct MSP2_INAV_SET_CUSTOM_OSD_ELEMENTS__request__CUSTOM_ELEMENTS_PARTS {
    std::uint8_t partType;
    std::uint16_t partValue;

    static std::vector<std::uint8_t> pack(const MSP2_INAV_SET_CUSTOM_OSD_ELEMENTS__request__CUSTOM_ELEMENTS_PARTS& v) {
        BufferWriter w;
        w.write_le(v.partType);
        w.write_le(v.partValue);
        return std::move(w.buf);
    }

    static MSP2_INAV_SET_CUSTOM_OSD_ELEMENTS__request__CUSTOM_ELEMENTS_PARTS unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_INAV_SET_CUSTOM_OSD_ELEMENTS__request__CUSTOM_ELEMENTS_PARTS v{};
        v.partType = r.read_le<std::uint8_t>();
        v.partValue = r.read_le<std::uint16_t>();
        return v;
    }
};

struct MSP2_INAV_SET_CUSTOM_OSD_ELEMENTS__request {
    std::uint8_t elementIndex;
    std::vector<MSP2_INAV_SET_CUSTOM_OSD_ELEMENTS__request__CUSTOM_ELEMENTS_PARTS> CUSTOM_ELEMENTS_PARTS;
    std::uint8_t visibilityType;
    std::uint16_t visibilityValue;
    std::string elementText;

    static std::vector<std::uint8_t> pack(const MSP2_INAV_SET_CUSTOM_OSD_ELEMENTS__request& v) {
        BufferWriter w;
        w.write_le(v.elementIndex);
        for (const auto& e : v.CUSTOM_ELEMENTS_PARTS) {
            auto bytes = MSP2_INAV_SET_CUSTOM_OSD_ELEMENTS__request__CUSTOM_ELEMENTS_PARTS::pack(e);
            w.write_bytes(bytes.data(), bytes.size());
        }
        w.write_le(v.visibilityType);
        w.write_le(v.visibilityValue);
        w.write_string_bytes(v.elementText);
        return std::move(w.buf);
    }

    static MSP2_INAV_SET_CUSTOM_OSD_ELEMENTS__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_INAV_SET_CUSTOM_OSD_ELEMENTS__request v{};
        v.elementIndex = r.read_le<std::uint8_t>();
        v.CUSTOM_ELEMENTS_PARTS.clear();
        static constexpr std::size_t __CUSTOM_ELEMENTS_PARTS_elem_size = []{ MSP2_INAV_SET_CUSTOM_OSD_ELEMENTS__request__CUSTOM_ELEMENTS_PARTS tmp{}; auto b = MSP2_INAV_SET_CUSTOM_OSD_ELEMENTS__request__CUSTOM_ELEMENTS_PARTS::pack(tmp); return b.size(); }();
        while (r.remaining() >= __CUSTOM_ELEMENTS_PARTS_elem_size) {
            std::vector<std::uint8_t> chunk(__CUSTOM_ELEMENTS_PARTS_elem_size);
            r.read_bytes(chunk.data(), __CUSTOM_ELEMENTS_PARTS_elem_size);
            v.CUSTOM_ELEMENTS_PARTS.push_back(MSP2_INAV_SET_CUSTOM_OSD_ELEMENTS__request__CUSTOM_ELEMENTS_PARTS::unpack(chunk));
        }
        v.visibilityType = r.read_le<std::uint8_t>();
        v.visibilityValue = r.read_le<std::uint16_t>();
        v.elementText = r.read_string_rest();
        return v;
    }
};

struct MSP2_INAV_OUTPUT_MAPPING_EXT2__reply {
    std::uint8_t timerId;
    std::uint32_t usageFlags;
    std::uint8_t pinLabel;

    static std::vector<std::uint8_t> pack(const MSP2_INAV_OUTPUT_MAPPING_EXT2__reply& v) {
        BufferWriter w;
        w.write_le(v.timerId);
        w.write_le(v.usageFlags);
        w.write_le(v.pinLabel);
        return std::move(w.buf);
    }

    static MSP2_INAV_OUTPUT_MAPPING_EXT2__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_INAV_OUTPUT_MAPPING_EXT2__reply v{};
        v.timerId = r.read_le<std::uint8_t>();
        v.usageFlags = r.read_le<std::uint32_t>();
        v.pinLabel = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP2_INAV_SERVO_CONFIG__reply {
    std::uint16_t min;
    std::uint16_t max;
    std::uint16_t middle;
    std::uint8_t rate;

    static std::vector<std::uint8_t> pack(const MSP2_INAV_SERVO_CONFIG__reply& v) {
        BufferWriter w;
        w.write_le(v.min);
        w.write_le(v.max);
        w.write_le(v.middle);
        w.write_le(v.rate);
        return std::move(w.buf);
    }

    static MSP2_INAV_SERVO_CONFIG__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_INAV_SERVO_CONFIG__reply v{};
        v.min = r.read_le<std::uint16_t>();
        v.max = r.read_le<std::uint16_t>();
        v.middle = r.read_le<std::uint16_t>();
        v.rate = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP2_INAV_SET_SERVO_CONFIG__request {
    std::uint8_t servoIndex;
    std::uint16_t min;
    std::uint16_t max;
    std::uint16_t middle;
    std::uint8_t rate;

    static std::vector<std::uint8_t> pack(const MSP2_INAV_SET_SERVO_CONFIG__request& v) {
        BufferWriter w;
        w.write_le(v.servoIndex);
        w.write_le(v.min);
        w.write_le(v.max);
        w.write_le(v.middle);
        w.write_le(v.rate);
        return std::move(w.buf);
    }

    static MSP2_INAV_SET_SERVO_CONFIG__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_INAV_SET_SERVO_CONFIG__request v{};
        v.servoIndex = r.read_le<std::uint8_t>();
        v.min = r.read_le<std::uint16_t>();
        v.max = r.read_le<std::uint16_t>();
        v.middle = r.read_le<std::uint16_t>();
        v.rate = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP2_INAV_GEOZONE__request {
    std::uint8_t geozoneIndex;

    static std::vector<std::uint8_t> pack(const MSP2_INAV_GEOZONE__request& v) {
        BufferWriter w;
        w.write_le(v.geozoneIndex);
        return std::move(w.buf);
    }

    static MSP2_INAV_GEOZONE__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_INAV_GEOZONE__request v{};
        v.geozoneIndex = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP2_INAV_GEOZONE__reply {
    std::uint8_t geozoneIndex;
    std::uint8_t type;
    std::uint8_t shape;
    std::uint32_t minAltitude;
    std::uint32_t maxAltitude;
    std::uint8_t isSeaLevelRef;
    std::uint8_t fenceAction;
    std::uint8_t vertexCount;

    static std::vector<std::uint8_t> pack(const MSP2_INAV_GEOZONE__reply& v) {
        BufferWriter w;
        w.write_le(v.geozoneIndex);
        w.write_le(v.type);
        w.write_le(v.shape);
        w.write_le(v.minAltitude);
        w.write_le(v.maxAltitude);
        w.write_le(v.isSeaLevelRef);
        w.write_le(v.fenceAction);
        w.write_le(v.vertexCount);
        return std::move(w.buf);
    }

    static MSP2_INAV_GEOZONE__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_INAV_GEOZONE__reply v{};
        v.geozoneIndex = r.read_le<std::uint8_t>();
        v.type = r.read_le<std::uint8_t>();
        v.shape = r.read_le<std::uint8_t>();
        v.minAltitude = r.read_le<std::uint32_t>();
        v.maxAltitude = r.read_le<std::uint32_t>();
        v.isSeaLevelRef = r.read_le<std::uint8_t>();
        v.fenceAction = r.read_le<std::uint8_t>();
        v.vertexCount = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP2_INAV_SET_GEOZONE__request {
    std::uint8_t geozoneIndex;
    std::uint8_t type;
    std::uint8_t shape;
    std::uint32_t minAltitude;
    std::uint32_t maxAltitude;
    std::uint8_t isSeaLevelRef;
    std::uint8_t fenceAction;
    std::uint8_t vertexCount;

    static std::vector<std::uint8_t> pack(const MSP2_INAV_SET_GEOZONE__request& v) {
        BufferWriter w;
        w.write_le(v.geozoneIndex);
        w.write_le(v.type);
        w.write_le(v.shape);
        w.write_le(v.minAltitude);
        w.write_le(v.maxAltitude);
        w.write_le(v.isSeaLevelRef);
        w.write_le(v.fenceAction);
        w.write_le(v.vertexCount);
        return std::move(w.buf);
    }

    static MSP2_INAV_SET_GEOZONE__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_INAV_SET_GEOZONE__request v{};
        v.geozoneIndex = r.read_le<std::uint8_t>();
        v.type = r.read_le<std::uint8_t>();
        v.shape = r.read_le<std::uint8_t>();
        v.minAltitude = r.read_le<std::uint32_t>();
        v.maxAltitude = r.read_le<std::uint32_t>();
        v.isSeaLevelRef = r.read_le<std::uint8_t>();
        v.fenceAction = r.read_le<std::uint8_t>();
        v.vertexCount = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP2_INAV_GEOZONE_VERTEX__request {
    std::uint8_t geozoneIndex;
    std::uint8_t vertexId;

    static std::vector<std::uint8_t> pack(const MSP2_INAV_GEOZONE_VERTEX__request& v) {
        BufferWriter w;
        w.write_le(v.geozoneIndex);
        w.write_le(v.vertexId);
        return std::move(w.buf);
    }

    static MSP2_INAV_GEOZONE_VERTEX__request unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_INAV_GEOZONE_VERTEX__request v{};
        v.geozoneIndex = r.read_le<std::uint8_t>();
        v.vertexId = r.read_le<std::uint8_t>();
        return v;
    }
};

struct MSP2_INAV_GEOZONE_VERTEX__reply {
    std::uint8_t geozoneIndex;
    std::uint8_t vertexId;
    std::int32_t latitude;
    std::int32_t longitude;
    std::uint32_t radius;

    static std::vector<std::uint8_t> pack(const MSP2_INAV_GEOZONE_VERTEX__reply& v) {
        BufferWriter w;
        w.write_le(v.geozoneIndex);
        w.write_le(v.vertexId);
        w.write_le(v.latitude);
        w.write_le(v.longitude);
        w.write_le(v.radius);
        return std::move(w.buf);
    }

    static MSP2_INAV_GEOZONE_VERTEX__reply unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_INAV_GEOZONE_VERTEX__reply v{};
        v.geozoneIndex = r.read_le<std::uint8_t>();
        v.vertexId = r.read_le<std::uint8_t>();
        v.latitude = r.read_le<std::int32_t>();
        v.longitude = r.read_le<std::int32_t>();
        v.radius = r.read_le<std::uint32_t>();
        return v;
    }
};

struct MSP2_INAV_SET_GEOZONE_VERTEX__polygon {
    std::uint8_t geozoneIndex;
    std::uint8_t vertexId;
    std::int32_t latitude;
    std::int32_t longitude;

    static std::vector<std::uint8_t> pack(const MSP2_INAV_SET_GEOZONE_VERTEX__polygon& v) {
        BufferWriter w;
        w.write_le(v.geozoneIndex);
        w.write_le(v.vertexId);
        w.write_le(v.latitude);
        w.write_le(v.longitude);
        return std::move(w.buf);
    }

    static MSP2_INAV_SET_GEOZONE_VERTEX__polygon unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_INAV_SET_GEOZONE_VERTEX__polygon v{};
        v.geozoneIndex = r.read_le<std::uint8_t>();
        v.vertexId = r.read_le<std::uint8_t>();
        v.latitude = r.read_le<std::int32_t>();
        v.longitude = r.read_le<std::int32_t>();
        return v;
    }
};

struct MSP2_INAV_SET_GEOZONE_VERTEX__circle {
    std::uint8_t geozoneIndex;
    std::uint8_t vertexId;
    std::int32_t latitude;
    std::int32_t longitude;
    std::uint32_t radius;

    static std::vector<std::uint8_t> pack(const MSP2_INAV_SET_GEOZONE_VERTEX__circle& v) {
        BufferWriter w;
        w.write_le(v.geozoneIndex);
        w.write_le(v.vertexId);
        w.write_le(v.latitude);
        w.write_le(v.longitude);
        w.write_le(v.radius);
        return std::move(w.buf);
    }

    static MSP2_INAV_SET_GEOZONE_VERTEX__circle unpack(const std::vector<std::uint8_t>& payload) {
        BufferReader r(payload);
        MSP2_INAV_SET_GEOZONE_VERTEX__circle v{};
        v.geozoneIndex = r.read_le<std::uint8_t>();
        v.vertexId = r.read_le<std::uint8_t>();
        v.latitude = r.read_le<std::int32_t>();
        v.longitude = r.read_le<std::int32_t>();
        v.radius = r.read_le<std::uint32_t>();
        return v;
    }
};

using MSP2_INAV_SET_GEOZONE_VERTEX_variant = std::variant<MSP2_INAV_SET_GEOZONE_VERTEX__polygon, MSP2_INAV_SET_GEOZONE_VERTEX__circle>;
inline MSP2_INAV_SET_GEOZONE_VERTEX_variant unpack_MSP2_INAV_SET_GEOZONE_VERTEX(const std::vector<std::uint8_t>& payload) {
    switch (payload.size()) {
    default:
        return MSP2_INAV_SET_GEOZONE_VERTEX__polygon::unpack(payload);
    }
}


} // namespace msp
