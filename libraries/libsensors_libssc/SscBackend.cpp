/*
 * SPDX-FileCopyrightText: The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_TAG "MainlineSensorsSsc"

#include "SscBackend.h"

#include <android-base/logging.h>
#include <android-base/strings.h>
#include <libsensors_common/SensorEvents.h>
#include <libsensors_common/Settings.h>

#include <cerrno>
#include <chrono>
#include <thread>

namespace aidl::android::hardware::sensors::mainline {

namespace {

constexpr int kDiscoveryRetryMs = 1000;

const SscSensorKind kAllKinds[] = {
        SscSensorKind::kAccelerometer, SscSensorKind::kGyroscope, SscSensorKind::kMagnetometer,
        SscSensorKind::kLight,         SscSensorKind::kProximity, SscSensorKind::kCompass,
};

std::vector<SscSensorKind> ConfiguredKinds() {
    std::vector<SscSensorKind> kinds;
    std::string configured = Settings::Get().GetString("ssc.sensors", "");
    if (configured.empty()) {
        kinds.assign(std::begin(kAllKinds), std::end(kAllKinds));
        return kinds;
    }
    for (const auto& token : ::android::base::Split(configured, ",")) {
        std::string name = ::android::base::Trim(token);
        bool found = false;
        for (SscSensorKind kind : kAllKinds) {
            if (::android::base::EqualsIgnoreCase(name, SscSensorKindName(kind))) {
                kinds.push_back(kind);
                found = true;
                break;
            }
        }
        if (!found && !name.empty()) {
            LOG(WARNING) << "ssc.sensors: unknown sensor kind '" << name << "'";
        }
    }
    return kinds;
}

}  // namespace

DEFINE_SENSOR_BACKEND(SscBackend, 0)

SscBackend::SscBackend() = default;

SscBackend::~SscBackend() {
    Deinitialize();
}

std::string SscBackend::GetName() const {
    return "ssc";
}

void SscBackend::OnEvent(const Event& event, bool wakeup) {
    // Runs on the GLib worker thread. post_events_ is only written while the
    // worker is stopped, so no lock is needed here; taking mutex_ would
    // deadlock with Activate(), which holds it while waiting for the worker.
    if (paused_.load() || !post_events_) {
        return;
    }
    post_events_({event}, wakeup);
}

void SscBackend::DiscoverSensors() {
    // Runs on the GLib worker thread. The sensor DSP may still be booting
    // when the HAL starts; optionally keep trying for a while.
    const int64_t wait_ms = Settings::Get().GetInt("ssc.discovery_wait_ms", 0);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(wait_ms);
    std::vector<SscSensorKind> kinds = ConfiguredKinds();

    while (true) {
        for (SscSensorKind kind : kinds) {
            bool already = false;
            for (const auto& [handle, sensor] : sensors_) {
                if (sensor->GetKind() == kind) already = true;
            }
            if (already) {
                continue;
            }
            std::unique_ptr<SscSensor> sensor = SscSensor::Create(kind, next_handle_);
            if (!sensor) {
                continue;
            }
            next_handle_++;
            sensor->SetCallback(
                    [this](const Event& event, bool wakeup) { OnEvent(event, wakeup); });
            LOG(INFO) << "SSC sensor discovered: " << sensor->Describe();
            sensors_[sensor->GetHandle()] = std::move(sensor);
        }
        if (sensors_.size() == kinds.size() || std::chrono::steady_clock::now() >= deadline) {
            break;
        }
        LOG(INFO) << "Waiting for the sensor DSP (" << sensors_.size() << "/" << kinds.size()
                  << " sensors found)";
        std::this_thread::sleep_for(std::chrono::milliseconds(kDiscoveryRetryMs));
    }
}

int32_t SscBackend::Initialize(const PostEventsCallback& callback) {
    post_events_ = callback;
    worker_.Start();
    worker_.Invoke([this]() {
        DiscoverSensors();
        return 0;
    });
    std::lock_guard<std::mutex> lock(mutex_);
    LOG(INFO) << "SSC backend initialized with " << sensors_.size() << " sensor(s)";
    if (sensors_.empty()) {
        worker_.Stop();
    }
    return 0;
}

void SscBackend::Deinitialize() {
    if (worker_.IsRunning()) {
        worker_.Invoke([this]() {
            for (auto& [handle, sensor] : sensors_) {
                if (sensor->IsActive()) {
                    sensor->Activate(false);
                }
            }
            // GObjects must be released on the thread owning the main context.
            sensors_.clear();
            return 0;
        });
        worker_.Stop();
    }
    // Leave a clean slate: Initialize() may be called again to retry
    // discovery while the frontend waits for late sensors.
    std::lock_guard<std::mutex> lock(mutex_);
    sensors_.clear();
    next_handle_ = 1;
    post_events_ = nullptr;
    LOG(INFO) << "SSC backend deinitialized";
}

std::vector<SensorInfo> SscBackend::GetSensorsList() {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<SensorInfo> list;
    for (const auto& [handle, sensor] : sensors_) {
        list.push_back(sensor->GetInfo());
    }
    return list;
}

SscSensor* SscBackend::FindSensor(int32_t handle) {
    auto it = sensors_.find(handle);
    return it == sensors_.end() ? nullptr : it->second.get();
}

int32_t SscBackend::Activate(int32_t sensor_handle, bool enabled) {
    std::lock_guard<std::mutex> lock(mutex_);
    SscSensor* sensor = FindSensor(sensor_handle);
    if (sensor == nullptr) {
        return -EINVAL;
    }
    int32_t ret = worker_.Invoke([sensor, enabled]() { return sensor->Activate(enabled); });
    LOG(INFO) << "SSC sensor " << sensor_handle << (enabled ? " activated" : " deactivated")
              << " -> " << ret;
    return ret;
}

int32_t SscBackend::Batch(int32_t sensor_handle, int64_t sampling_period_ns,
                          int64_t /* max_report_latency_ns */) {
    std::lock_guard<std::mutex> lock(mutex_);
    SscSensor* sensor = FindSensor(sensor_handle);
    if (sensor == nullptr) {
        return -EINVAL;
    }
    // The DSP rate is fixed; the requested period is honoured by decimation.
    sensor->SetPeriodNs(sampling_period_ns);
    LOG(DEBUG) << "SSC sensor " << sensor_handle << " period " << sampling_period_ns / 1000
               << " us";
    return 0;
}

int32_t SscBackend::Flush(int32_t sensor_handle) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (FindSensor(sensor_handle) == nullptr) {
        return -EINVAL;
    }
    return kFlushHandledByFrontend;
}

int32_t SscBackend::SetOperationMode(OperationMode mode) {
    paused_.store(mode != OperationMode::NORMAL);
    LOG(INFO) << "SSC backend operation mode " << toString(mode);
    return 0;
}

}  // namespace aidl::android::hardware::sensors::mainline
