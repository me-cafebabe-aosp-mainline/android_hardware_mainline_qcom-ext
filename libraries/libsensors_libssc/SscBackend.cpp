/*
 * SPDX-FileCopyrightText: The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_TAG "MainlineSensorsSscBackend"

#include "SscBackend.h"

#include <android-base/logging.h>

#include <libssc.h>

#include <gio/gio.h>

#include <fcntl.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <future>

namespace aidl::android::hardware::sensors::mainline {

static constexpr int64_t kNanosecondsPerSecond = 1000LL * 1000 * 1000;
static constexpr int32_t kDefaultMaxDelayUs = 10 * 1000 * 1000;

extern "C" __attribute__((visibility("default"))) ISensorBackend* CreateSensorBackend() {
    return new SscBackend();
}

SscBackend::SscBackend() = default;

SscBackend::~SscBackend() {
    Deinitialize();
}

std::string SscBackend::GetName() const {
    return "SSC";
}

void SscBackend::PostEvents(const std::vector<Event>& events, bool wakeup) {
    if (post_events_callback_) {
        post_events_callback_(events, wakeup);
    }
}

static int64_t GetBootTimestampNs() {
    struct timespec ts;
    clock_gettime(CLOCK_BOOTTIME, &ts);
    return ts.tv_sec * kNanosecondsPerSecond + ts.tv_nsec;
}

void SscBackend::AccelMeasurementCb(SSCSensorAccelerometer* /* sensor */,
                                     gfloat x, gfloat y, gfloat z,
                                     gpointer user_data) {
    auto* entry = static_cast<SscSensorEntry*>(user_data);
    if (!entry->enabled.load()) {
        return;
    }

    Event event;
    event.sensorHandle = entry->handle;
    event.sensorType = entry->android_type;
    event.timestamp = GetBootTimestampNs();

    EventPayload::Vec3 vec3 = {
            .x = x,
            .y = y,
            .z = z,
            .status = SensorStatus::ACCURACY_HIGH,
    };
    event.payload.set<EventPayload::Tag::vec3>(vec3);

    entry->backend->PostEvents({event}, false);
}

void SscBackend::GyroMeasurementCb(SSCSensorGyroscope* /* sensor */,
                                    gfloat x, gfloat y, gfloat z,
                                    gpointer user_data) {
    auto* entry = static_cast<SscSensorEntry*>(user_data);
    if (!entry->enabled.load()) {
        return;
    }

    Event event;
    event.sensorHandle = entry->handle;
    event.sensorType = entry->android_type;
    event.timestamp = GetBootTimestampNs();

    EventPayload::Vec3 vec3 = {
            .x = x,
            .y = y,
            .z = z,
            .status = SensorStatus::ACCURACY_HIGH,
    };
    event.payload.set<EventPayload::Tag::vec3>(vec3);

    entry->backend->PostEvents({event}, false);
}

#ifdef ENABLE_MAGNETOMETER
void SscBackend::MagnMeasurementCb(SSCSensorMagnetometer* /* sensor */,
                                    gfloat x, gfloat y, gfloat z,
                                    gpointer user_data) {
    auto* entry = static_cast<SscSensorEntry*>(user_data);
    if (!entry->enabled.load()) {
        return;
    }

    Event event;
    event.sensorHandle = entry->handle;
    event.sensorType = entry->android_type;
    event.timestamp = GetBootTimestampNs();

    EventPayload::Vec3 vec3 = {
            .x = x,
            .y = y,
            .z = z,
            .status = SensorStatus::ACCURACY_HIGH,
    };
    event.payload.set<EventPayload::Tag::vec3>(vec3);

    entry->backend->PostEvents({event}, false);
}
#endif

void SscBackend::LightMeasurementCb(SSCSensorLight* /* sensor */,
                                     gfloat intensity,
                                     gpointer user_data) {
    auto* entry = static_cast<SscSensorEntry*>(user_data);
    if (!entry->enabled.load()) {
        return;
    }

    Event event;
    event.sensorHandle = entry->handle;
    event.sensorType = entry->android_type;
    event.timestamp = GetBootTimestampNs();
    event.payload.set<EventPayload::Tag::scalar>(intensity);

    entry->backend->PostEvents({event}, false);
}

void SscBackend::ProxMeasurementCb(SSCSensorProximity* /* sensor */,
                                     gboolean near,
                                     gpointer user_data) {
    auto* entry = static_cast<SscSensorEntry*>(user_data);
    if (!entry->enabled.load()) {
        return;
    }

    Event event;
    event.sensorHandle = entry->handle;
    event.sensorType = entry->android_type;
    event.timestamp = GetBootTimestampNs();
    event.payload.set<EventPayload::Tag::scalar>(near ? 0.0f : 1.0f);

    bool wakeup = (entry->sensor_info.flags &
                   static_cast<int32_t>(SensorInfo::SENSOR_FLAG_BITS_WAKE_UP)) != 0;
    entry->backend->PostEvents({event}, wakeup);
}

void SscBackend::CompassMeasurementCb(SSCSensorCompass* /* sensor */,
                                       gfloat azimuth,
                                       gpointer user_data) {
    auto* entry = static_cast<SscSensorEntry*>(user_data);
    if (!entry->enabled.load()) {
        return;
    }

    Event event;
    event.sensorHandle = entry->handle;
    event.sensorType = entry->android_type;
    event.timestamp = GetBootTimestampNs();

    EventPayload::Vec3 vec3 = {
            .x = azimuth,
            .y = 0.0f,
            .z = 0.0f,
            .status = SensorStatus::ACCURACY_HIGH,
    };
    event.payload.set<EventPayload::Tag::vec3>(vec3);

    entry->backend->PostEvents({event}, false);
}

static gboolean WakeupPipeCb(GIOChannel* /* source */, GIOCondition /* condition */,
                              gpointer data) {
    auto* backend = static_cast<SscBackend*>(data);
    backend->DrainWakeupPipe();
    backend->DispatchCommands();
    return TRUE;
}

void SscBackend::SetupWakeupSource() {
    if (pipe2(wakeup_pipe_fd_, O_CLOEXEC | O_NONBLOCK) != 0) {
        LOG(WARNING) << "Failed to create wakeup pipe: " << strerror(errno);
        return;
    }

    GIOChannel* channel = g_io_channel_unix_new(wakeup_pipe_fd_[0]);
    wakeup_source_id_ = g_io_add_watch(channel, static_cast<GIOCondition>(G_IO_IN | G_IO_HUP),
                                        WakeupPipeCb, this);
    g_io_channel_unref(channel);
}

void SscBackend::TeardownWakeupSource() {
    if (wakeup_source_id_ != 0) {
        g_source_remove(wakeup_source_id_);
        wakeup_source_id_ = 0;
    }
    if (wakeup_pipe_fd_[0] >= 0) {
        close(wakeup_pipe_fd_[0]);
        wakeup_pipe_fd_[0] = -1;
    }
    if (wakeup_pipe_fd_[1] >= 0) {
        close(wakeup_pipe_fd_[1]);
        wakeup_pipe_fd_[1] = -1;
    }
}

void SscBackend::DrainWakeupPipe() {
    char buf[64];
    while (read(wakeup_pipe_fd_[0], buf, sizeof(buf)) > 0) {
    }
}

void SscBackend::WakeupWorker() {
    if (wakeup_pipe_fd_[1] >= 0) {
        char byte = 1;
        ssize_t ret = write(wakeup_pipe_fd_[1], &byte, 1);
        if (ret < 0) {
            LOG(WARNING) << "Failed to write to wakeup pipe: " << strerror(errno);
        }
    }
}

void SscBackend::DispatchCommands() {
    std::queue<Command> local_queue;
    {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        std::swap(local_queue, commands_);
    }

    while (!local_queue.empty()) {
        Command cmd = std::move(local_queue.front());
        local_queue.pop();

        if (cmd.type == CommandType::kShutdown) {
            stop_worker_.store(true);
            cmd.result_promise.set_value(0);
            break;
        }

        int32_t result = -EINVAL;
        auto it = sensors_.find(cmd.handle);
        if (it != sensors_.end()) {
            bool enable = (cmd.type == CommandType::kOpen);
            result = ActivateSensor(it->second.get(), enable);
        }
        cmd.result_promise.set_value(result);
    }
}

void SscBackend::WorkerLoop() {
    LOG(INFO) << "SSC worker thread started";

    while (!stop_worker_.load()) {
        g_main_context_iteration(g_main_context_default(), TRUE);
    }

    LOG(INFO) << "SSC worker thread stopped";
}

static std::string ReadGObjectStringProp(GObject* object, const gchar* prop_name) {
    gchar* value = nullptr;
    g_object_get(object, prop_name, &value, nullptr);
    if (value == nullptr) {
        return "";
    }
    std::string result(value);
    g_free(value);
    return result;
}

static gfloat ReadGObjectFloatProp(GObject* object, const gchar* prop_name,
                                    gfloat default_value) {
    gfloat value = default_value;
    g_object_get(object, prop_name, &value, nullptr);
    return value;
}

static int32_t ComputeMinDelayUs(gfloat sample_rate) {
    if (sample_rate > 0.0f) {
        return static_cast<int32_t>(1000000.0f / sample_rate);
    }
    return 0;
}

static float GetDefaultMaxRange(SscSensorKind kind) {
    switch (kind) {
        case SscSensorKind::kAccelerometer:
            return 78.4f;
        case SscSensorKind::kGyroscope:
            return 34.9f;
#ifdef ENABLE_MAGNETOMETER
        case SscSensorKind::kMagnetometer:
            return 4912.0f;
#endif
        case SscSensorKind::kLight:
            return 65535.0f;
        case SscSensorKind::kProximity:
            return 1.0f;
        case SscSensorKind::kCompass:
            return 360.0f;
    }
    return 100.0f;
}

static float GetDefaultResolution(SscSensorKind kind) {
    switch (kind) {
        case SscSensorKind::kAccelerometer:
            return 0.001f;
        case SscSensorKind::kGyroscope:
            return 0.001f;
#ifdef ENABLE_MAGNETOMETER
        case SscSensorKind::kMagnetometer:
            return 0.1f;
#endif
        case SscSensorKind::kLight:
            return 1.0f;
        case SscSensorKind::kProximity:
            return 1.0f;
        case SscSensorKind::kCompass:
            return 1.0f;
    }
    return 1.0f;
}

bool SscBackend::TryCreateAccelerometer() {
    GError* error = nullptr;
    SSCSensorAccelerometer* sensor = ssc_sensor_accelerometer_new_sync(nullptr, &error);
    if (sensor == nullptr) {
        if (error != nullptr) {
            LOG(INFO) << "SSC accelerometer not available: " << error->message;
            g_error_free(error);
        }
        return false;
    }

    auto entry = std::make_unique<SscSensorEntry>();
    entry->handle = next_handle_++;
    entry->kind = SscSensorKind::kAccelerometer;
    entry->android_type = SensorType::ACCELEROMETER;
    entry->ssc_sensor = SSC_SENSOR(sensor);
    entry->backend = this;

    std::string name = ReadGObjectStringProp(G_OBJECT(sensor), SSC_SENSOR_NAME);
    std::string vendor = ReadGObjectStringProp(G_OBJECT(sensor), SSC_SENSOR_VENDOR);
    gfloat sample_rate = ReadGObjectFloatProp(G_OBJECT(sensor), SSC_SENSOR_SAMPLE_RATE, 100.0f);

    entry->sensor_info.sensorHandle = entry->handle;
    entry->sensor_info.name = name.empty() ? "SSC Accelerometer" : name;
    entry->sensor_info.vendor = vendor.empty() ? "Qualcomm" : vendor;
    entry->sensor_info.version = 1;
    entry->sensor_info.type = SensorType::ACCELEROMETER;
    entry->sensor_info.typeAsString = "";
    entry->sensor_info.maxRange = GetDefaultMaxRange(entry->kind);
    entry->sensor_info.resolution = GetDefaultResolution(entry->kind);
    entry->sensor_info.power = 0.13f;
    entry->sensor_info.minDelayUs = ComputeMinDelayUs(sample_rate);
    entry->sensor_info.maxDelayUs = kDefaultMaxDelayUs;
    entry->sensor_info.fifoReservedEventCount = 0;
    entry->sensor_info.fifoMaxEventCount = 0;
    entry->sensor_info.requiredPermission = "";
    entry->sensor_info.flags =
            static_cast<int32_t>(SensorInfo::SENSOR_FLAG_BITS_DATA_INJECTION);

    sensors_[entry->handle] = std::move(entry);
    LOG(INFO) << "SSC accelerometer discovered: name='" << name << "' vendor='" << vendor
              << "' sample_rate=" << sample_rate << " Hz";
    return true;
}

bool SscBackend::TryCreateGyroscope() {
    GError* error = nullptr;
    SSCSensorGyroscope* sensor = ssc_sensor_gyroscope_new_sync(nullptr, &error);
    if (sensor == nullptr) {
        if (error != nullptr) {
            LOG(INFO) << "SSC gyroscope not available: " << error->message;
            g_error_free(error);
        }
        return false;
    }

    auto entry = std::make_unique<SscSensorEntry>();
    entry->handle = next_handle_++;
    entry->kind = SscSensorKind::kGyroscope;
    entry->android_type = SensorType::GYROSCOPE;
    entry->ssc_sensor = SSC_SENSOR(sensor);
    entry->backend = this;

    std::string name = ReadGObjectStringProp(G_OBJECT(sensor), SSC_SENSOR_NAME);
    std::string vendor = ReadGObjectStringProp(G_OBJECT(sensor), SSC_SENSOR_VENDOR);
    gfloat sample_rate = ReadGObjectFloatProp(G_OBJECT(sensor), SSC_SENSOR_SAMPLE_RATE, 100.0f);

    entry->sensor_info.sensorHandle = entry->handle;
    entry->sensor_info.name = name.empty() ? "SSC Gyroscope" : name;
    entry->sensor_info.vendor = vendor.empty() ? "Qualcomm" : vendor;
    entry->sensor_info.version = 1;
    entry->sensor_info.type = SensorType::GYROSCOPE;
    entry->sensor_info.typeAsString = "";
    entry->sensor_info.maxRange = GetDefaultMaxRange(entry->kind);
    entry->sensor_info.resolution = GetDefaultResolution(entry->kind);
    entry->sensor_info.power = 0.13f;
    entry->sensor_info.minDelayUs = ComputeMinDelayUs(sample_rate);
    entry->sensor_info.maxDelayUs = kDefaultMaxDelayUs;
    entry->sensor_info.fifoReservedEventCount = 0;
    entry->sensor_info.fifoMaxEventCount = 0;
    entry->sensor_info.requiredPermission = "";
    entry->sensor_info.flags =
            static_cast<int32_t>(SensorInfo::SENSOR_FLAG_BITS_DATA_INJECTION);

    sensors_[entry->handle] = std::move(entry);
    LOG(INFO) << "SSC gyroscope discovered: name='" << name << "' vendor='" << vendor
              << "' sample_rate=" << sample_rate << " Hz";
    return true;
}

bool SscBackend::TryCreateMagnetometer() {
#ifdef ENABLE_MAGNETOMETER
    GError* error = nullptr;
    SSCSensorMagnetometer* sensor = ssc_sensor_magnetometer_new_sync(nullptr, &error);
    if (sensor == nullptr) {
        if (error != nullptr) {
            LOG(INFO) << "SSC magnetometer not available: " << error->message;
            g_error_free(error);
        }
        return false;
    }

    auto entry = std::make_unique<SscSensorEntry>();
    entry->handle = next_handle_++;
    entry->kind = SscSensorKind::kMagnetometer;
    entry->android_type = SensorType::MAGNETIC_FIELD;
    entry->ssc_sensor = SSC_SENSOR(sensor);
    entry->backend = this;

    std::string name = ReadGObjectStringProp(G_OBJECT(sensor), SSC_SENSOR_NAME);
    std::string vendor = ReadGObjectStringProp(G_OBJECT(sensor), SSC_SENSOR_VENDOR);
    gfloat sample_rate = ReadGObjectFloatProp(G_OBJECT(sensor), SSC_SENSOR_SAMPLE_RATE, 25.0f);

    entry->sensor_info.sensorHandle = entry->handle;
    entry->sensor_info.name = name.empty() ? "SSC Magnetometer" : name;
    entry->sensor_info.vendor = vendor.empty() ? "Qualcomm" : vendor;
    entry->sensor_info.version = 1;
    entry->sensor_info.type = SensorType::MAGNETIC_FIELD;
    entry->sensor_info.typeAsString = "";
    entry->sensor_info.maxRange = GetDefaultMaxRange(entry->kind);
    entry->sensor_info.resolution = GetDefaultResolution(entry->kind);
    entry->sensor_info.power = 0.13f;
    entry->sensor_info.minDelayUs = ComputeMinDelayUs(sample_rate);
    entry->sensor_info.maxDelayUs = kDefaultMaxDelayUs;
    entry->sensor_info.fifoReservedEventCount = 0;
    entry->sensor_info.fifoMaxEventCount = 0;
    entry->sensor_info.requiredPermission = "";
    entry->sensor_info.flags =
            static_cast<int32_t>(SensorInfo::SENSOR_FLAG_BITS_DATA_INJECTION);

    sensors_[entry->handle] = std::move(entry);
    LOG(INFO) << "SSC magnetometer discovered: name='" << name << "' vendor='" << vendor
              << "' sample_rate=" << sample_rate << " Hz";
    return true;
#else
    LOG(INFO) << "SSC magnetometer support is disabled at compile time.";
    return false;
#endif
}

bool SscBackend::TryCreateLight() {
    GError* error = nullptr;
    SSCSensorLight* sensor = ssc_sensor_light_new_sync(nullptr, &error);
    if (sensor == nullptr) {
        if (error != nullptr) {
            LOG(INFO) << "SSC light sensor not available: " << error->message;
            g_error_free(error);
        }
        return false;
    }

    auto entry = std::make_unique<SscSensorEntry>();
    entry->handle = next_handle_++;
    entry->kind = SscSensorKind::kLight;
    entry->android_type = SensorType::LIGHT;
    entry->ssc_sensor = SSC_SENSOR(sensor);
    entry->backend = this;

    std::string name = ReadGObjectStringProp(G_OBJECT(sensor), SSC_SENSOR_NAME);
    std::string vendor = ReadGObjectStringProp(G_OBJECT(sensor), SSC_SENSOR_VENDOR);

    entry->sensor_info.sensorHandle = entry->handle;
    entry->sensor_info.name = name.empty() ? "SSC Light Sensor" : name;
    entry->sensor_info.vendor = vendor.empty() ? "Qualcomm" : vendor;
    entry->sensor_info.version = 1;
    entry->sensor_info.type = SensorType::LIGHT;
    entry->sensor_info.typeAsString = "";
    entry->sensor_info.maxRange = GetDefaultMaxRange(entry->kind);
    entry->sensor_info.resolution = GetDefaultResolution(entry->kind);
    entry->sensor_info.power = 0.13f;
    entry->sensor_info.minDelayUs = 0;
    entry->sensor_info.maxDelayUs = 0;
    entry->sensor_info.fifoReservedEventCount = 0;
    entry->sensor_info.fifoMaxEventCount = 0;
    entry->sensor_info.requiredPermission = "";
    entry->sensor_info.flags =
            static_cast<int32_t>(SensorInfo::SENSOR_FLAG_BITS_ON_CHANGE_MODE);

    sensors_[entry->handle] = std::move(entry);
    LOG(INFO) << "SSC light sensor discovered: name='" << name << "' vendor='" << vendor << "'";
    return true;
}

bool SscBackend::TryCreateProximity() {
    GError* error = nullptr;
    SSCSensorProximity* sensor = ssc_sensor_proximity_new_sync(nullptr, &error);
    if (sensor == nullptr) {
        if (error != nullptr) {
            LOG(INFO) << "SSC proximity sensor not available: " << error->message;
            g_error_free(error);
        }
        return false;
    }

    auto entry = std::make_unique<SscSensorEntry>();
    entry->handle = next_handle_++;
    entry->kind = SscSensorKind::kProximity;
    entry->android_type = SensorType::PROXIMITY;
    entry->ssc_sensor = SSC_SENSOR(sensor);
    entry->backend = this;

    std::string name = ReadGObjectStringProp(G_OBJECT(sensor), SSC_SENSOR_NAME);
    std::string vendor = ReadGObjectStringProp(G_OBJECT(sensor), SSC_SENSOR_VENDOR);

    entry->sensor_info.sensorHandle = entry->handle;
    entry->sensor_info.name = name.empty() ? "SSC Proximity Sensor" : name;
    entry->sensor_info.vendor = vendor.empty() ? "Qualcomm" : vendor;
    entry->sensor_info.version = 1;
    entry->sensor_info.type = SensorType::PROXIMITY;
    entry->sensor_info.typeAsString = "";
    entry->sensor_info.maxRange = GetDefaultMaxRange(entry->kind);
    entry->sensor_info.resolution = GetDefaultResolution(entry->kind);
    entry->sensor_info.power = 0.13f;
    entry->sensor_info.minDelayUs = 0;
    entry->sensor_info.maxDelayUs = 0;
    entry->sensor_info.fifoReservedEventCount = 0;
    entry->sensor_info.fifoMaxEventCount = 0;
    entry->sensor_info.requiredPermission = "";
    entry->sensor_info.flags =
            static_cast<int32_t>(SensorInfo::SENSOR_FLAG_BITS_ON_CHANGE_MODE |
                                 SensorInfo::SENSOR_FLAG_BITS_WAKE_UP);

    sensors_[entry->handle] = std::move(entry);
    LOG(INFO) << "SSC proximity sensor discovered: name='" << name << "' vendor='" << vendor
              << "'";
    return true;
}

bool SscBackend::TryCreateCompass() {
    GError* error = nullptr;
    SSCSensorCompass* sensor = ssc_sensor_compass_new_sync(nullptr, &error);
    if (sensor == nullptr) {
        if (error != nullptr) {
            LOG(INFO) << "SSC compass not available: " << error->message;
            g_error_free(error);
        }
        return false;
    }

    auto entry = std::make_unique<SscSensorEntry>();
    entry->handle = next_handle_++;
    entry->kind = SscSensorKind::kCompass;
    entry->android_type = SensorType::ORIENTATION;
    entry->ssc_sensor = SSC_SENSOR(sensor);
    entry->backend = this;

    std::string name = ReadGObjectStringProp(G_OBJECT(sensor), SSC_SENSOR_NAME);
    std::string vendor = ReadGObjectStringProp(G_OBJECT(sensor), SSC_SENSOR_VENDOR);
    gfloat sample_rate = ReadGObjectFloatProp(G_OBJECT(sensor), SSC_SENSOR_SAMPLE_RATE, 10.0f);

    entry->sensor_info.sensorHandle = entry->handle;
    entry->sensor_info.name = name.empty() ? "SSC Compass" : name;
    entry->sensor_info.vendor = vendor.empty() ? "Qualcomm" : vendor;
    entry->sensor_info.version = 1;
    entry->sensor_info.type = SensorType::ORIENTATION;
    entry->sensor_info.typeAsString = "";
    entry->sensor_info.maxRange = GetDefaultMaxRange(entry->kind);
    entry->sensor_info.resolution = GetDefaultResolution(entry->kind);
    entry->sensor_info.power = 0.13f;
    entry->sensor_info.minDelayUs = ComputeMinDelayUs(sample_rate);
    entry->sensor_info.maxDelayUs = kDefaultMaxDelayUs;
    entry->sensor_info.fifoReservedEventCount = 0;
    entry->sensor_info.fifoMaxEventCount = 0;
    entry->sensor_info.requiredPermission = "";
    entry->sensor_info.flags =
            static_cast<int32_t>(SensorInfo::SENSOR_FLAG_BITS_DATA_INJECTION);

    sensors_[entry->handle] = std::move(entry);
    LOG(INFO) << "SSC compass discovered: name='" << name << "' vendor='" << vendor
              << "' sample_rate=" << sample_rate << " Hz";
    return true;
}

void SscBackend::DiscoverSensors() {
    TryCreateAccelerometer();
    TryCreateGyroscope();
    TryCreateMagnetometer();
    TryCreateLight();
    TryCreateProximity();
    TryCreateCompass();
}

int32_t SscBackend::Initialize(const PostEventsCallback& callback) {
    std::lock_guard<std::mutex> lock(mutex_);
    post_events_callback_ = callback;

    SetupWakeupSource();

    DiscoverSensors();

    if (sensors_.empty()) {
        LOG(INFO) << "SSC backend initialized with 0 sensors";
        TeardownWakeupSource();
        return 0;
    }

    stop_worker_.store(false);
    worker_thread_ = std::thread(&SscBackend::WorkerLoop, this);

    LOG(INFO) << "SSC backend initialized with " << sensors_.size() << " sensor(s)";
    return 0;
}

int32_t SscBackend::ActivateSensor(SscSensorEntry* entry, bool enabled) {
    if (entry->enabled.load() == enabled) {
        return 0;
    }

    GError* error = nullptr;

    if (enabled) {
        switch (entry->kind) {
            case SscSensorKind::kAccelerometer:
                entry->measurement_id = g_signal_connect(
                        entry->ssc_sensor, "measurement",
                        G_CALLBACK(AccelMeasurementCb), entry);
                if (!ssc_sensor_accelerometer_open_sync(
                            SSC_SENSOR_ACCELEROMETER(entry->ssc_sensor), nullptr, &error)) {
                    LOG(WARNING) << "Failed to open SSC accelerometer: "
                                 << (error ? error->message : "unknown");
                    g_signal_handler_disconnect(entry->ssc_sensor, entry->measurement_id);
                    entry->measurement_id = 0;
                    if (error) g_error_free(error);
                    return -EIO;
                }
                break;

            case SscSensorKind::kGyroscope:
                entry->measurement_id = g_signal_connect(
                        entry->ssc_sensor, "measurement",
                        G_CALLBACK(GyroMeasurementCb), entry);
                if (!ssc_sensor_gyroscope_open_sync(
                            SSC_SENSOR_GYROSCOPE(entry->ssc_sensor), nullptr, &error)) {
                    LOG(WARNING) << "Failed to open SSC gyroscope: "
                                 << (error ? error->message : "unknown");
                    g_signal_handler_disconnect(entry->ssc_sensor, entry->measurement_id);
                    entry->measurement_id = 0;
                    if (error) g_error_free(error);
                    return -EIO;
                }
                break;

#ifdef ENABLE_MAGNETOMETER
            case SscSensorKind::kMagnetometer:
                entry->measurement_id = g_signal_connect(
                        entry->ssc_sensor, "measurement",
                        G_CALLBACK(MagnMeasurementCb), entry);
                if (!ssc_sensor_magnetometer_open_sync(
                            SSC_SENSOR_MAGNETOMETER(entry->ssc_sensor), nullptr, &error)) {
                    LOG(WARNING) << "Failed to open SSC magnetometer: "
                                 << (error ? error->message : "unknown");
                    g_signal_handler_disconnect(entry->ssc_sensor, entry->measurement_id);
                    entry->measurement_id = 0;
                    if (error) g_error_free(error);
                    return -EIO;
                }
                break;
#endif

            case SscSensorKind::kLight:
                entry->measurement_id = g_signal_connect(
                        entry->ssc_sensor, "measurement",
                        G_CALLBACK(LightMeasurementCb), entry);
                if (!ssc_sensor_light_open_sync(
                            SSC_SENSOR_LIGHT(entry->ssc_sensor), nullptr, &error)) {
                    LOG(WARNING) << "Failed to open SSC light sensor: "
                                 << (error ? error->message : "unknown");
                    g_signal_handler_disconnect(entry->ssc_sensor, entry->measurement_id);
                    entry->measurement_id = 0;
                    if (error) g_error_free(error);
                    return -EIO;
                }
                break;

            case SscSensorKind::kProximity:
                entry->measurement_id = g_signal_connect(
                        entry->ssc_sensor, "measurement",
                        G_CALLBACK(ProxMeasurementCb), entry);
                if (!ssc_sensor_proximity_open_sync(
                            SSC_SENSOR_PROXIMITY(entry->ssc_sensor), nullptr, &error)) {
                    LOG(WARNING) << "Failed to open SSC proximity sensor: "
                                 << (error ? error->message : "unknown");
                    g_signal_handler_disconnect(entry->ssc_sensor, entry->measurement_id);
                    entry->measurement_id = 0;
                    if (error) g_error_free(error);
                    return -EIO;
                }
                break;

            case SscSensorKind::kCompass:
                entry->measurement_id = g_signal_connect(
                        entry->ssc_sensor, "measurement",
                        G_CALLBACK(CompassMeasurementCb), entry);
                if (!ssc_sensor_compass_open_sync(
                            SSC_SENSOR_COMPASS(entry->ssc_sensor), nullptr, &error)) {
                    LOG(WARNING) << "Failed to open SSC compass: "
                                 << (error ? error->message : "unknown");
                    g_signal_handler_disconnect(entry->ssc_sensor, entry->measurement_id);
                    entry->measurement_id = 0;
                    if (error) g_error_free(error);
                    return -EIO;
                }
                break;
        }

        entry->enabled.store(true);
        LOG(INFO) << "SSC sensor " << entry->handle << " activated";
    } else {
        entry->enabled.store(false);

        if (entry->measurement_id != 0) {
            g_signal_handler_disconnect(entry->ssc_sensor, entry->measurement_id);
            entry->measurement_id = 0;
        }

        switch (entry->kind) {
            case SscSensorKind::kAccelerometer:
                if (!ssc_sensor_accelerometer_close_sync(
                            SSC_SENSOR_ACCELEROMETER(entry->ssc_sensor), nullptr, &error)) {
                    LOG(WARNING) << "Failed to close SSC accelerometer: "
                                 << (error ? error->message : "unknown");
                    if (error) g_error_free(error);
                }
                break;

            case SscSensorKind::kGyroscope:
                if (!ssc_sensor_gyroscope_close_sync(
                            SSC_SENSOR_GYROSCOPE(entry->ssc_sensor), nullptr, &error)) {
                    LOG(WARNING) << "Failed to close SSC gyroscope: "
                                 << (error ? error->message : "unknown");
                    if (error) g_error_free(error);
                }
                break;

#ifdef ENABLE_MAGNETOMETER
            case SscSensorKind::kMagnetometer:
                if (!ssc_sensor_magnetometer_close_sync(
                            SSC_SENSOR_MAGNETOMETER(entry->ssc_sensor), nullptr, &error)) {
                    LOG(WARNING) << "Failed to close SSC magnetometer: "
                                 << (error ? error->message : "unknown");
                    if (error) g_error_free(error);
                }
                break;
#endif

            case SscSensorKind::kLight:
                if (!ssc_sensor_light_close_sync(
                            SSC_SENSOR_LIGHT(entry->ssc_sensor), nullptr, &error)) {
                    LOG(WARNING) << "Failed to close SSC light sensor: "
                                 << (error ? error->message : "unknown");
                    if (error) g_error_free(error);
                }
                break;

            case SscSensorKind::kProximity:
                if (!ssc_sensor_proximity_close_sync(
                            SSC_SENSOR_PROXIMITY(entry->ssc_sensor), nullptr, &error)) {
                    LOG(WARNING) << "Failed to close SSC proximity sensor: "
                                 << (error ? error->message : "unknown");
                    if (error) g_error_free(error);
                }
                break;

            case SscSensorKind::kCompass:
                if (!ssc_sensor_compass_close_sync(
                            SSC_SENSOR_COMPASS(entry->ssc_sensor), nullptr, &error)) {
                    LOG(WARNING) << "Failed to close SSC compass: "
                                 << (error ? error->message : "unknown");
                    if (error) g_error_free(error);
                }
                break;
        }

        LOG(INFO) << "SSC sensor " << entry->handle << " deactivated";
    }

    return 0;
}

void SscBackend::Deinitialize() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!worker_thread_.joinable() && sensors_.empty()) {
            post_events_callback_ = nullptr;
            return;
        }
    }

    if (worker_thread_.joinable()) {
        for (auto& [handle, entry] : sensors_) {
            if (entry->enabled.load()) {
                Command cmd;
                cmd.type = CommandType::kClose;
                cmd.handle = handle;
                auto future = cmd.result_promise.get_future();
                {
                    std::lock_guard<std::mutex> lock(cmd_mutex_);
                    commands_.push(std::move(cmd));
                }
                WakeupWorker();
                future.get();
            }
        }

        {
            Command cmd;
            cmd.type = CommandType::kShutdown;
            cmd.handle = 0;
            auto future = cmd.result_promise.get_future();
            {
                std::lock_guard<std::mutex> lock(cmd_mutex_);
                commands_.push(std::move(cmd));
            }
            WakeupWorker();
            future.get();
        }

        worker_thread_.join();
    }

    for (auto& [handle, entry] : sensors_) {
        if (entry->ssc_sensor != nullptr) {
            g_object_unref(entry->ssc_sensor);
            entry->ssc_sensor = nullptr;
        }
    }
    sensors_.clear();

    TeardownWakeupSource();

    std::lock_guard<std::mutex> lock(mutex_);
    post_events_callback_ = nullptr;
    LOG(INFO) << "SSC backend deinitialized";
}

std::vector<SensorInfo> SscBackend::GetSensorsList() {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<SensorInfo> result;
    for (const auto& [handle, entry] : sensors_) {
        result.push_back(entry->sensor_info);
    }
    return result;
}

int32_t SscBackend::Activate(int32_t sensor_handle, bool enabled) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (sensors_.find(sensor_handle) == sensors_.end()) {
            return -EINVAL;
        }
    }

    if (!worker_thread_.joinable()) {
        return -EIO;
    }

    Command cmd;
    cmd.type = enabled ? CommandType::kOpen : CommandType::kClose;
    cmd.handle = sensor_handle;
    auto future = cmd.result_promise.get_future();
    {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        commands_.push(std::move(cmd));
    }
    WakeupWorker();
    return future.get();
}

int32_t SscBackend::Batch(int32_t sensor_handle, int64_t /* sampling_period_ns */,
                           int64_t /* max_report_latency_ns */) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (sensors_.find(sensor_handle) == sensors_.end()) {
        return -EINVAL;
    }
    return 0;
}

int32_t SscBackend::Flush(int32_t sensor_handle) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = sensors_.find(sensor_handle);
    if (it == sensors_.end()) {
        return -EINVAL;
    }

    auto& entry = it->second;
    if (!entry->enabled.load()) {
        return -EINVAL;
    }

    Event ev;
    ev.sensorHandle = sensor_handle;
    ev.sensorType = SensorType::META_DATA;
    EventPayload::MetaData meta = {
            .what = EventPayload::MetaData::MetaDataEventType::META_DATA_FLUSH_COMPLETE,
    };
    ev.payload.set<EventPayload::Tag::meta>(meta);

    if (post_events_callback_) {
        post_events_callback_({ev}, false);
    }

    return 0;
}

int32_t SscBackend::SetOperationMode(OperationMode mode) {
    std::lock_guard<std::mutex> lock(mutex_);
    operation_mode_ = mode;
    return 0;
}

}  // namespace aidl::android::hardware::sensors::mainline
