/*
 * SPDX-FileCopyrightText: The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_TAG "MainlineSensorsSsc"

#include "SscSensor.h"

#include <android-base/logging.h>
#include <android-base/stringprintf.h>
#include <libsensors_common/SensorEvents.h>
#include <libsensors_common/SensorTypes.h>
#include <libsensors_common/Settings.h>

#include <libssc-sensor-magnetometer.h>
#include <libssc.h>

#include <algorithm>
#include <cerrno>

namespace aidl::android::hardware::sensors::mainline {

namespace {

constexpr int64_t kDefaultPeriodNs = 100LL * 1000 * 1000;
constexpr int32_t kOneSecondUs = 1000 * 1000;
// The DSP streams at its fixed rate; do not advertise faster than this.
constexpr int32_t kMinDelayFloorUs = 5000;
constexpr float kProximityNearCm = 0.0f;
constexpr float kProximityFarCm = 5.0f;

struct KindInfo {
    SscSensorKind kind;
    const char* name;
    SensorType android_type;
};

const KindInfo kKinds[] = {
        {SscSensorKind::kAccelerometer, "accel", SensorType::ACCELEROMETER},
        {SscSensorKind::kGyroscope, "gyro", SensorType::GYROSCOPE},
        {SscSensorKind::kMagnetometer, "magn", SensorType::MAGNETIC_FIELD},
        {SscSensorKind::kLight, "light", SensorType::LIGHT},
        {SscSensorKind::kProximity, "proximity", SensorType::PROXIMITY},
        // libssc reduces the DSP rotation vector to an azimuth in degrees;
        // ORIENTATION (azimuth, pitch, roll) is the closest Android type.
        {SscSensorKind::kCompass, "compass", SensorType::ORIENTATION},
};

const KindInfo& GetKindInfo(SscSensorKind kind) {
    for (const auto& info : kKinds) {
        if (info.kind == kind) {
            return info;
        }
    }
    return kKinds[0];
}

std::string ReadStringProperty(GObject* object, const char* name) {
    gchar* value = nullptr;
    g_object_get(object, name, &value, nullptr);
    std::string result = value != nullptr ? value : "";
    g_free(value);
    return result;
}

float ReadFloatProperty(GObject* object, const char* name) {
    gfloat value = 0.0f;
    g_object_get(object, name, &value, nullptr);
    return value;
}

std::string ErrorMessage(GError* error) {
    return error != nullptr && error->message != nullptr ? error->message : "unknown error";
}

}  // namespace

const char* SscSensorKindName(SscSensorKind kind) {
    return GetKindInfo(kind).name;
}

SscSensor::SscSensor(SscSensorKind kind, int32_t handle, SSCSensor* sensor)
    : kind_(kind), sensor_(sensor), period_ns_(kDefaultPeriodNs) {
    info_.sensorHandle = handle;
    info_.type = GetKindInfo(kind).android_type;
    config_key_ = std::string("ssc.") + GetKindInfo(kind).name;
    DeriveSensorInfo();
}

SscSensor::~SscSensor() {
    Disconnect();
    if (sensor_ != nullptr) {
        g_object_unref(sensor_);
    }
}

std::unique_ptr<SscSensor> SscSensor::Create(SscSensorKind kind, int32_t handle) {
    GError* error = nullptr;
    SSCSensor* sensor = nullptr;
    switch (kind) {
        case SscSensorKind::kAccelerometer:
            sensor = SSC_SENSOR(ssc_sensor_accelerometer_new_sync(nullptr, &error));
            break;
        case SscSensorKind::kGyroscope:
            sensor = SSC_SENSOR(ssc_sensor_gyroscope_new_sync(nullptr, &error));
            break;
        case SscSensorKind::kMagnetometer:
            sensor = SSC_SENSOR(ssc_sensor_magnetometer_new_sync(nullptr, &error));
            break;
        case SscSensorKind::kLight:
            sensor = SSC_SENSOR(ssc_sensor_light_new_sync(nullptr, &error));
            break;
        case SscSensorKind::kProximity:
            sensor = SSC_SENSOR(ssc_sensor_proximity_new_sync(nullptr, &error));
            break;
        case SscSensorKind::kCompass:
            sensor = SSC_SENSOR(ssc_sensor_compass_new_sync(nullptr, &error));
            break;
    }
    if (sensor == nullptr) {
        LOG(INFO) << "SSC " << SscSensorKindName(kind) << " not available: " << ErrorMessage(error);
        g_clear_error(&error);
        return nullptr;
    }
    return std::unique_ptr<SscSensor>(new SscSensor(kind, handle, sensor));
}

void SscSensor::DeriveSensorInfo() {
    ApplySensorTypeDefaults(&info_);
    info_.version = 1;

    GObject* object = G_OBJECT(sensor_);
    const std::string name = ReadStringProperty(object, SSC_SENSOR_NAME);
    const std::string vendor = ReadStringProperty(object, SSC_SENSOR_VENDOR);
    const float sample_rate = ReadFloatProperty(object, SSC_SENSOR_SAMPLE_RATE);

    auto traits = GetSensorTypeTraits(info_.type);
    const std::string label = traits ? std::string(traits->label) : toString(info_.type);
    info_.name = name.empty() ? "SSC " + label : name + " " + label;
    info_.vendor = vendor.empty() ? "Qualcomm" : vendor;

    if (GetReportingMode(info_.flags) == ReportingMode::kContinuous) {
        // The DSP streams at one fixed rate; slower rates are obtained by
        // decimation in this backend.
        if (sample_rate > 0.0f) {
            info_.minDelayUs =
                    std::max(static_cast<int32_t>(1.0e6f / sample_rate), kMinDelayFloorUs);
        }
        info_.maxDelayUs = std::max(kOneSecondUs, info_.minDelayUs);
    }
    if (kind_ == SscSensorKind::kProximity) {
        info_.maxRange = kProximityFarCm;
        info_.resolution = kProximityFarCm;
    }
    if (kind_ == SscSensorKind::kCompass) {
        info_.maxRange = 360.0f;
        info_.resolution = 1.0f;
    }

    // Optional configuration overrides.
    Settings& settings = Settings::Get();
    auto name_override = settings.GetString(config_key_ + ".name");
    if (name_override.has_value() && !name_override->empty()) info_.name = *name_override;
    auto vendor_override = settings.GetString(config_key_ + ".vendor");
    if (vendor_override.has_value() && !vendor_override->empty()) {
        info_.vendor = *vendor_override;
    }
    auto power = settings.GetDouble(config_key_ + ".power");
    if (power.has_value() && *power >= 0.0) info_.power = static_cast<float>(*power);
    auto max_range = settings.GetDouble(config_key_ + ".max_range");
    if (max_range.has_value() && *max_range > 0.0) info_.maxRange = static_cast<float>(*max_range);
    auto resolution = settings.GetDouble(config_key_ + ".resolution");
    if (resolution.has_value() && *resolution > 0.0) {
        info_.resolution = static_cast<float>(*resolution);
    }
    auto min_delay = settings.GetInt(config_key_ + ".min_delay_us");
    if (min_delay.has_value()) info_.minDelayUs = static_cast<int32_t>(*min_delay);
    auto max_delay = settings.GetInt(config_key_ + ".max_delay_us");
    if (max_delay.has_value()) info_.maxDelayUs = static_cast<int32_t>(*max_delay);
    auto wake_up = settings.GetBool(config_key_ + ".wake_up");
    if (wake_up.has_value()) {
        if (*wake_up) {
            info_.flags |= SensorInfo::SENSOR_FLAG_BITS_WAKE_UP;
        } else {
            info_.flags &= ~SensorInfo::SENSOR_FLAG_BITS_WAKE_UP;
        }
    }
    // libssc already applies the mount matrix reported by the DSP; this one is
    // applied on top for boards where that matrix is wrong.
    auto matrix_text = settings.GetString(config_key_ + ".mount_matrix");
    if (matrix_text.has_value()) {
        auto matrix = MountMatrix::Parse(*matrix_text);
        if (matrix.has_value()) {
            extra_mount_matrix_ = *matrix;
            LOG(INFO) << info_.name << ": extra mount matrix [" << matrix->ToString() << "]";
        } else {
            LOG(WARNING) << info_.name << ": invalid " << config_key_ << ".mount_matrix '"
                         << *matrix_text << "'";
        }
    }

    LOG(INFO) << "SSC " << SscSensorKindName(kind_) << ": name='" << name << "' vendor='" << vendor
              << "' sample_rate=" << sample_rate << " Hz";
}

void SscSensor::SetPeriodNs(int64_t period_ns) {
    period_ns_.store(period_ns > 0 ? period_ns : kDefaultPeriodNs);
}

void SscSensor::Connect() {
    if (signal_id_ != 0) {
        return;
    }
    switch (kind_) {
        case SscSensorKind::kAccelerometer:
        case SscSensorKind::kGyroscope:
        case SscSensorKind::kMagnetometer:
            signal_id_ = g_signal_connect(sensor_, "measurement", G_CALLBACK(OnVec3), this);
            break;
        case SscSensorKind::kLight:
        case SscSensorKind::kCompass:
            signal_id_ = g_signal_connect(sensor_, "measurement", G_CALLBACK(OnScalar), this);
            break;
        case SscSensorKind::kProximity:
            signal_id_ = g_signal_connect(sensor_, "measurement", G_CALLBACK(OnNear), this);
            break;
    }
}

void SscSensor::Disconnect() {
    if (signal_id_ != 0 && sensor_ != nullptr) {
        g_signal_handler_disconnect(sensor_, signal_id_);
    }
    signal_id_ = 0;
}

int32_t SscSensor::Activate(bool enabled) {
    if (active_.load() == enabled) {
        return 0;
    }
    GError* error = nullptr;
    gboolean ok = FALSE;
    if (enabled) {
        last_emit_ns_ = 0;
        last_event_.reset();
        // Connect before opening so that the first report is not missed.
        Connect();
        switch (kind_) {
            case SscSensorKind::kAccelerometer:
                ok = ssc_sensor_accelerometer_open_sync(SSC_SENSOR_ACCELEROMETER(sensor_), nullptr,
                                                        &error);
                break;
            case SscSensorKind::kGyroscope:
                ok = ssc_sensor_gyroscope_open_sync(SSC_SENSOR_GYROSCOPE(sensor_), nullptr, &error);
                break;
            case SscSensorKind::kMagnetometer:
                ok = ssc_sensor_magnetometer_open_sync(SSC_SENSOR_MAGNETOMETER(sensor_), nullptr,
                                                       &error);
                break;
            case SscSensorKind::kLight:
                ok = ssc_sensor_light_open_sync(SSC_SENSOR_LIGHT(sensor_), nullptr, &error);
                break;
            case SscSensorKind::kProximity:
                ok = ssc_sensor_proximity_open_sync(SSC_SENSOR_PROXIMITY(sensor_), nullptr, &error);
                break;
            case SscSensorKind::kCompass:
                ok = ssc_sensor_compass_open_sync(SSC_SENSOR_COMPASS(sensor_), nullptr, &error);
                break;
        }
        if (!ok) {
            LOG(ERROR) << "Failed to open SSC " << SscSensorKindName(kind_) << ": "
                       << ErrorMessage(error);
            g_clear_error(&error);
            Disconnect();
            return -EIO;
        }
        active_.store(true);
        LOG(INFO) << "SSC " << SscSensorKindName(kind_) << " opened";
        return 0;
    }

    active_.store(false);
    Disconnect();
    switch (kind_) {
        case SscSensorKind::kAccelerometer:
            ok = ssc_sensor_accelerometer_close_sync(SSC_SENSOR_ACCELEROMETER(sensor_), nullptr,
                                                     &error);
            break;
        case SscSensorKind::kGyroscope:
            ok = ssc_sensor_gyroscope_close_sync(SSC_SENSOR_GYROSCOPE(sensor_), nullptr, &error);
            break;
        case SscSensorKind::kMagnetometer:
            ok = ssc_sensor_magnetometer_close_sync(SSC_SENSOR_MAGNETOMETER(sensor_), nullptr,
                                                    &error);
            break;
        case SscSensorKind::kLight:
            ok = ssc_sensor_light_close_sync(SSC_SENSOR_LIGHT(sensor_), nullptr, &error);
            break;
        case SscSensorKind::kProximity:
            ok = ssc_sensor_proximity_close_sync(SSC_SENSOR_PROXIMITY(sensor_), nullptr, &error);
            break;
        case SscSensorKind::kCompass:
            ok = ssc_sensor_compass_close_sync(SSC_SENSOR_COMPASS(sensor_), nullptr, &error);
            break;
    }
    if (!ok) {
        LOG(WARNING) << "Failed to close SSC " << SscSensorKindName(kind_) << ": "
                     << ErrorMessage(error);
        g_clear_error(&error);
    }
    LOG(INFO) << "SSC " << SscSensorKindName(kind_) << " closed";
    return 0;
}

void SscSensor::Emit(Event event) {
    if (!active_.load() || !callback_) {
        return;
    }
    if (GetReportingMode(info_.flags) == ReportingMode::kContinuous) {
        const int64_t period = period_ns_.load();
        if (last_emit_ns_ != 0 && event.timestamp - last_emit_ns_ < period - period / 10) {
            return;
        }
    } else if (last_event_.has_value() && HaveSamePayload(*last_event_, event)) {
        return;
    }
    last_emit_ns_ = event.timestamp;
    last_event_ = event;
    LOG(VERBOSE) << "SSC event: " << EventToString(event);
    callback_(event, IsWakeUpSensor(info_.flags));
}

void SscSensor::OnVec3(SSCSensor* /* sensor */, gfloat x, gfloat y, gfloat z, gpointer user_data) {
    auto* self = static_cast<SscSensor*>(user_data);
    float fx = x;
    float fy = y;
    float fz = z;
    self->extra_mount_matrix_.Apply(&fx, &fy, &fz);
    self->Emit(MakeVec3Event(self->GetHandle(), self->info_.type, GetBootTimeNs(), fx, fy, fz));
}

void SscSensor::OnScalar(SSCSensor* /* sensor */, gfloat value, gpointer user_data) {
    auto* self = static_cast<SscSensor*>(user_data);
    if (self->kind_ == SscSensorKind::kCompass) {
        // ORIENTATION: azimuth, pitch, roll. Only the azimuth is known.
        self->Emit(MakeVec3Event(self->GetHandle(), self->info_.type, GetBootTimeNs(), value, 0.0f,
                                 0.0f));
        return;
    }
    self->Emit(MakeScalarEvent(self->GetHandle(), self->info_.type, GetBootTimeNs(), value));
}

void SscSensor::OnNear(SSCSensor* /* sensor */, gboolean near, gpointer user_data) {
    auto* self = static_cast<SscSensor*>(user_data);
    self->Emit(MakeScalarEvent(self->GetHandle(), self->info_.type, GetBootTimeNs(),
                               near ? kProximityNearCm : kProximityFarCm));
}

std::string SscSensor::Describe() const {
    return ::android::base::StringPrintf("%s [ssc %s]", SensorInfoToString(info_).c_str(),
                                         SscSensorKindName(kind_));
}

}  // namespace aidl::android::hardware::sensors::mainline
