/*
 * SPDX-FileCopyrightText: The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <libsensors_mainline/SensorBackend.h>

#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <vector>

#include "GlibWorker.h"
#include "SscSensor.h"

namespace aidl::android::hardware::sensors::mainline {

/*
 * Backend for sensors managed by the Qualcomm Sensor Core (SLPI/ADSP) through
 * libssc (https://gitlab.com/dylanvanassche/libssc).
 *
 * All libssc calls run on a GLib worker thread (see GlibWorker). Requests from
 * the frontend are forwarded to that thread synchronously; measurements are
 * delivered from that thread through the frontend callback.
 */
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

  private:
    void DiscoverSensors();
    void OnEvent(const Event& event, bool wakeup);
    SscSensor* FindSensor(int32_t handle);

    // Serialises the ISensorBackend entry points. Never taken on the worker
    // thread.
    std::mutex mutex_;
    GlibWorker worker_;
    std::map<int32_t, std::unique_ptr<SscSensor>> sensors_;
    int32_t next_handle_ = 1;
    // Written only while the worker is stopped.
    PostEventsCallback post_events_;
    std::atomic<bool> paused_{false};
};

}  // namespace aidl::android::hardware::sensors::mainline
