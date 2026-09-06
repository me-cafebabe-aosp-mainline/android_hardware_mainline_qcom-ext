/*
 * SPDX-FileCopyrightText: The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <thread>

namespace aidl::android::hardware::sensors::mainline {

/*
 * Runs the GLib default main context on a dedicated thread.
 *
 * libssc is a GLib/GIO library: its asynchronous QMI operations and its
 * "measurement" signals are dispatched from the default GMainContext, and its
 * *_sync() helpers iterate that context while waiting. Everything touching
 * libssc therefore has to happen on the thread owning the default context,
 * which is what Invoke() provides.
 */
class GlibWorker {
  public:
    using Task = std::function<int32_t()>;

    GlibWorker() = default;
    ~GlibWorker();

    GlibWorker(const GlibWorker&) = delete;
    GlibWorker& operator=(const GlibWorker&) = delete;

    void Start();
    void Stop();
    bool IsRunning() const { return running_.load(); }

    // Runs `task` on the worker thread and waits for its result. When called
    // from the worker thread itself the task runs inline.
    int32_t Invoke(const Task& task);

  private:
    void Run();

    std::thread thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> stop_requested_{false};
};

}  // namespace aidl::android::hardware::sensors::mainline
