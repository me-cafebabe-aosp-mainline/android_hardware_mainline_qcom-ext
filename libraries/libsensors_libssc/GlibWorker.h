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
 * libssc therefore has to happen on the thread running the default context,
 * which is what Invoke() provides.
 *
 * Note that neither g_main_context_acquire() nor g_main_context_invoke() can
 * be used to implement this: the former gives up when another thread happens
 * to hold the context, and the latter runs the callback on the calling thread
 * whenever that thread can acquire the context.
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
    // from a callback dispatched by the worker itself, the task runs inline.
    // Must not be called from the worker thread outside of a dispatch.
    int32_t Invoke(const Task& task);

  private:
    void Run();

    std::thread thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> stop_requested_{false};
};

}  // namespace aidl::android::hardware::sensors::mainline
