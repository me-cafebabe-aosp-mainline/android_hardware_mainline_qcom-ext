/*
 * SPDX-FileCopyrightText: The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_TAG "MainlineSensorsSsc"

#include "GlibWorker.h"

#include <android-base/logging.h>
#include <glib.h>

#include <pthread.h>

#include <cerrno>
#include <future>

namespace aidl::android::hardware::sensors::mainline {

namespace {

struct InvokeContext {
    const GlibWorker::Task* task = nullptr;
    std::promise<int32_t> result;
    // Only touched on the thread dispatching the source.
    bool completed = false;
};

gboolean InvokeTrampoline(gpointer data) {
    auto* context = static_cast<InvokeContext*>(data);
    const int32_t result = (*context->task)();
    context->completed = true;
    context->result.set_value(result);
    return G_SOURCE_REMOVE;
}

// Runs when the source is destroyed, after the callback or instead of it.
void InvokeDestroyed(gpointer data) {
    auto* context = static_cast<InvokeContext*>(data);
    if (!context->completed) {
        LOG(ERROR) << "GLib task discarded before it could run";
        context->completed = true;
        context->result.set_value(-EIO);
    }
}

}  // namespace

GlibWorker::~GlibWorker() {
    Stop();
}

void GlibWorker::Start() {
    if (running_.load()) {
        return;
    }
    // A previous run may have finished without being joined.
    if (thread_.joinable()) {
        thread_.join();
    }
    stop_requested_.store(false);
    running_.store(true);
    thread_ = std::thread(&GlibWorker::Run, this);
}

void GlibWorker::Stop() {
    if (!thread_.joinable()) {
        running_.store(false);
        return;
    }
    stop_requested_.store(true);
    // Interrupt the blocking iteration. g_main_context_invoke() must not be
    // used here: it can run its callback on the calling thread instead of
    // waking the worker, which would make the join below block forever.
    g_main_context_wakeup(g_main_context_default());
    thread_.join();
    running_.store(false);
}

int32_t GlibWorker::Invoke(const Task& task) {
    if (!running_.load()) {
        LOG(ERROR) << "GLib worker not running";
        return -EIO;
    }
    // Re-entrant call from a callback dispatched by the worker itself: the
    // context is already owned by this thread, run the task directly.
    if (g_main_context_is_owner(g_main_context_default())) {
        return task();
    }

    InvokeContext context;
    context.task = &task;
    std::future<int32_t> future = context.result.get_future();

    /*
     * Attach the task as an idle source instead of using
     * g_main_context_invoke(): the latter runs the callback on the calling
     * thread whenever that thread can acquire the context, which defeats the
     * whole purpose of this class. libssc is not thread safe and its
     * *_sync() helpers iterate the default main context, so its calls must
     * happen on the thread running that context.
     *
     * g_source_attach() only queues the source and wakes the context, so the
     * task is always dispatched by the worker thread.
     */
    GSource* source = g_idle_source_new();
    g_source_set_priority(source, G_PRIORITY_DEFAULT);
    g_source_set_callback(source, InvokeTrampoline, &context, InvokeDestroyed);
    g_source_attach(source, g_main_context_default());
    g_source_unref(source);

    return future.get();
}

void GlibWorker::Run() {
    pthread_setname_np(pthread_self(), "ssc-glib");
    GMainContext* context = g_main_context_default();

    /*
     * Do not acquire the context explicitly: ownership may briefly be held by
     * another thread (g_main_context_invoke() and the libssc *_sync() helpers
     * take it), and g_main_context_acquire() gives up immediately in that
     * case. A blocking iteration waits for ownership instead, so the loop
     * below owns the context whenever it dispatches.
     */
    LOG(INFO) << "GLib worker thread started";
    while (!stop_requested_.load()) {
        g_main_context_iteration(context, TRUE);
    }
    LOG(INFO) << "GLib worker thread stopped";
}

}  // namespace aidl::android::hardware::sensors::mainline
