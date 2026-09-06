/*
 * SPDX-FileCopyrightText: The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <libsensors_common/MountMatrix.h>
#include <libsensors_mainline/SensorBackend.h>

#include <glib-object.h>

#include <atomic>
#include <functional>
#include <memory>
#include <optional>
#include <string>

typedef struct _SSCSensor SSCSensor;

namespace aidl::android::hardware::sensors::mainline {

// libssc sensor flavours.
enum class SscSensorKind {
    kAccelerometer,
    kGyroscope,
    kMagnetometer,
    kLight,
    kProximity,
    kCompass,
};

const char* SscSensorKindName(SscSensorKind kind);

/*
 * One sensor exposed by libssc.
 *
 * Owns the SSCSensor GObject, translates its "measurement" signal into Android
 * events and derives the SensorInfo from the attributes reported by the DSP
 * (name, vendor, sample rate) plus the shared per-type defaults and the
 * "ssc.<kind>.*" settings.
 *
 * Every method except the getters must be called on the GLib worker thread.
 */
class SscSensor {
  public:
    using EventCallback = std::function<void(const Event& event, bool wakeup)>;

    // Creates the libssc object synchronously; returns nullptr when the DSP does
    // not provide this sensor kind.
    static std::unique_ptr<SscSensor> Create(SscSensorKind kind, int32_t handle);

    ~SscSensor();

    SscSensor(const SscSensor&) = delete;
    SscSensor& operator=(const SscSensor&) = delete;

    SscSensorKind GetKind() const { return kind_; }
    const SensorInfo& GetInfo() const { return info_; }
    int32_t GetHandle() const { return info_.sensorHandle; }
    bool IsActive() const { return active_.load(); }

    void SetCallback(EventCallback callback) { callback_ = std::move(callback); }
    void SetPeriodNs(int64_t period_ns);

    // Opens/closes the libssc stream. Returns 0 or a negative errno.
    int32_t Activate(bool enabled);

    std::string Describe() const;

  private:
    SscSensor(SscSensorKind kind, int32_t handle, SSCSensor* sensor);

    void DeriveSensorInfo();
    void Connect();
    void Disconnect();
    void Emit(Event event);

    static void OnVec3(SSCSensor* sensor, gfloat x, gfloat y, gfloat z, gpointer user_data);
    static void OnScalar(SSCSensor* sensor, gfloat value, gpointer user_data);
    static void OnNear(SSCSensor* sensor, gboolean near, gpointer user_data);

    const SscSensorKind kind_;
    SSCSensor* const sensor_;
    SensorInfo info_;
    std::string config_key_;
    MountMatrix extra_mount_matrix_;
    EventCallback callback_;

    std::atomic<bool> active_{false};
    std::atomic<int64_t> period_ns_;
    gulong signal_id_ = 0;

    // Filtering state (worker thread only).
    int64_t last_emit_ns_ = 0;
    std::optional<Event> last_event_;
};

}  // namespace aidl::android::hardware::sensors::mainline
