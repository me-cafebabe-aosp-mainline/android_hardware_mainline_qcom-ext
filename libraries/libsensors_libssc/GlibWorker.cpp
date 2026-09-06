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
    const GlibWorker::Task* task;
    std::promise<int32_t> result;
};

gboolean InvokeTrampoline(gpointer data) {
    auto* context = static_cast<InvokeContext*>(data);
    context->result.set_value((*context->task)());
    return G_SOURCE_REMOVE;
}

gboolean WakeupTrampoline(gpointer /* data */) {
    return G_SOURCE_REMOVE;
}

}  // namespace

GlibWorker::~GlibWorker() {
    Stop();
}

void GlibWorker::Start() {
    if (running_.load()) {
        return;
    }
    stop_requested_.store(false);
    running_.store(true);
    thread_ = std::thread(&GlibWorker::Run, this);
}

void GlibWorker::Stop() {
    if (!thread_.joinable()) {
        return;
    }
    stop_requested_.store(true);
    // Wake the loop so it notices the stop request.
    g_main_context_invoke(nullptr, WakeupTrampoline, nullptr);
    thread_.join();
    running_.store(false);
}

int32_t GlibWorker::Invoke(const Task& task) {
    if (!running_.load()) {
        LOG(ERROR) << "GLib worker not running";
        return -EIO;
    }
    if (g_main_context_is_owner(g_main_context_default())) {
        return task();
    }
    InvokeContext context;
    context.task = &task;
    std::future<int32_t> future = context.result.get_future();
    g_main_context_invoke(nullptr, InvokeTrampoline, &context);
    return future.get();
}

void GlibWorker::Run() {
    pthread_setname_np(pthread_self(), "ssc-glib");
    GMainContext* context = g_main_context_default();
    if (!g_main_context_acquire(context)) {
        LOG(ERROR) << "Cannot acquire the default GLib main context";
        running_.store(false);
        return;
    }
    LOG(INFO) << "GLib worker thread started";
    while (!stop_requested_.load()) {
        g_main_context_iteration(context, TRUE);
    }
    g_main_context_release(context);
    LOG(INFO) << "GLib worker thread stopped";
}

}  // namespace aidl::android::hardware::sensors::mainline
