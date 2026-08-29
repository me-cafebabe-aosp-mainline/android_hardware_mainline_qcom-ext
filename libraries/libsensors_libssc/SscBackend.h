/*
 * SPDX-FileCopyrightText: The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <libsensors_mainline/SensorBackend.h>

#include <glib.h>

#include <atomic>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>

typedef struct _SSCSensor SSCSensor;
typedef struct _SSCSensorAccelerometer SSCSensorAccelerometer;
typedef struct _SSCSensorGyroscope SSCSensorGyroscope;
#ifdef ENABLE_MAGNETOMETER
typedef struct _SSCSensorMagnetometer SSCSensorMagnetometer;
#endif
typedef struct _SSCSensorLight SSCSensorLight;
typedef struct _SSCSensorProximity SSCSensorProximity;
typedef struct _SSCSensorCompass SSCSensorCompass;

namespace aidl::android::hardware::sensors::mainline {

enum class SscSensorKind {
    kAccelerometer,
    kGyroscope,
#ifdef ENABLE_MAGNETOMETER
    kMagnetometer,
#endif
    kLight,
    kProximity,
    kCompass,
};

struct SscSensorEntry {
    int32_t handle;
    SscSensorKind kind;
    SensorType android_type;
    SensorInfo sensor_info;
    SSCSensor* ssc_sensor = nullptr;
    gulong measurement_id = 0;
    std::atomic_bool enabled{false};
    class SscBackend* backend = nullptr;
};

class SscBackend : public ISensorBackend {
  public:
    SscBackend();
    ~SscBackend() override;

    std::string GetName() const override;

    int32_t Initialize(const PostEventsCallback& callback) override;

    void Deinitialize() override;

    std::vector<SensorInfo> GetSensorsList() override;

    int32_t Activate(int32_t sensor_handle, bool enabled) override;

    int32_t Batch(int32_t sensor_handle, int64_t sampling_period_ns,
                  int64_t max_report_latency_ns) override;

    int32_t Flush(int32_t sensor_handle) override;

    int32_t SetOperationMode(OperationMode mode) override;

    void PostEvents(const std::vector<Event>& events, bool wakeup);

    void DrainWakeupPipe();
    void DispatchCommands();

  private:
    enum class CommandType {
        kOpen,
        kClose,
        kShutdown,
    };

    struct Command {
        CommandType type;
        int32_t handle;
        std::promise<int32_t> result_promise;
    };

    void WorkerLoop();
    void WakeupWorker();

    void DiscoverSensors();
    bool TryCreateAccelerometer();
    bool TryCreateGyroscope();
    bool TryCreateMagnetometer();
    bool TryCreateLight();
    bool TryCreateProximity();
    bool TryCreateCompass();

    int32_t ActivateSensor(SscSensorEntry* entry, bool enabled);

    void SetupWakeupSource();
    void TeardownWakeupSource();

    static void AccelMeasurementCb(SSCSensorAccelerometer* sensor, gfloat x, gfloat y,
                                   gfloat z, gpointer user_data);
    static void GyroMeasurementCb(SSCSensorGyroscope* sensor, gfloat x, gfloat y,
                                  gfloat z, gpointer user_data);
#ifdef ENABLE_MAGNETOMETER
    static void MagnMeasurementCb(SSCSensorMagnetometer* sensor, gfloat x, gfloat y,
                                  gfloat z, gpointer user_data);
#endif
    static void LightMeasurementCb(SSCSensorLight* sensor, gfloat intensity,
                                   gpointer user_data);
    static void ProxMeasurementCb(SSCSensorProximity* sensor, gboolean near,
                                  gpointer user_data);
    static void CompassMeasurementCb(SSCSensorCompass* sensor, gfloat azimuth,
                                     gpointer user_data);

    std::map<int32_t, std::unique_ptr<SscSensorEntry>> sensors_;
    int32_t next_handle_ = 1;
    PostEventsCallback post_events_callback_;
    OperationMode operation_mode_ = OperationMode::NORMAL;
    std::mutex mutex_;

    std::thread worker_thread_;
    std::atomic_bool stop_worker_{false};
    std::mutex cmd_mutex_;
    std::queue<Command> commands_;
    int wakeup_pipe_fd_[2] = {-1, -1};
    guint wakeup_source_id_ = 0;
};

extern "C" ISensorBackend* CreateSensorBackend();

}  // namespace aidl::android::hardware::sensors::mainline
