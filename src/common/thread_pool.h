// Inspired by cartographer/common/thread_pool.h
// Simplified header-only implementation using C++ standard library (no absl).
//
// A fixed-size thread pool that executes Task objects in dependency order.
//
// Design (mirrors Cartographer ThreadPool):
//   - tasks_not_ready_: map Task* → shared_ptr, holds tasks awaiting deps
//   - task_queue_:       deque of shared_ptr, holds tasks ready to run
//   - Schedule() inserts into tasks_not_ready_; if task is immediately ready
//     it is moved directly into task_queue_
//   - NotifyDependenciesCompleted() moves task from not_ready → queue
//   - Workers run at nice +10 (Linux) to avoid blocking foreground threads
//   - WaitUntilIdle() blocks until queue empty and no active workers
//
// Usage:
//   lightning::common::ThreadPool pool(4);
//
//   auto t1 = std::make_shared<Task>();
//   t1->SetWorkItem([]{ heavy_a(); });
//   pool.Schedule(t1);
//
//   auto t2 = std::make_shared<Task>();
//   t2->SetWorkItem([]{ heavy_b(); });
//   t2->AddDependency(t1);
//   pool.Schedule(t2);
//
//   pool.WaitUntilIdle();

#pragma once

#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>

#include <glog/logging.h>

#include "common/task.h"

namespace lightning {
namespace common {

class ThreadPool {
   public:
    explicit ThreadPool(int num_threads) {
        CHECK_GT(num_threads, 0);
        for (int i = 0; i < num_threads; ++i) {
            pool_.emplace_back([this] { DoWork(); });
        }
    }

    ~ThreadPool() {
        {
            std::lock_guard<std::mutex> lk(mutex_);
            running_ = false;
        }
        cv_.notify_all();
        for (auto& t : pool_) {
            if (t.joinable()) t.join();
        }
    }

    ThreadPool(const ThreadPool&) = delete;
    ThreadPool& operator=(const ThreadPool&) = delete;

    // Schedule a task. WorkItem must already be set.
    // Returns weak_ptr; expiry means task has completed.
    std::weak_ptr<Task> Schedule(std::shared_ptr<Task> task) {
        std::weak_ptr<Task> weak = task;
        bool ready = task->SetThreadPool(this);
        {
            std::lock_guard<std::mutex> lk(mutex_);
            if (ready) {
                task_queue_.push_back(std::move(task));
            } else {
                tasks_not_ready_.emplace(task.get(), std::move(task));
            }
        }
        if (ready) cv_.notify_one();
        return weak;
    }

    // Block until all queued tasks have finished.
    void WaitUntilIdle() {
        std::unique_lock<std::mutex> lk(mutex_);
        cv_idle_.wait(lk, [this] {
            return task_queue_.empty() && active_count_ == 0;
        });
    }

   private:
    friend class Task;

    // Called by Task::Execute → dependent Task::OnDependencyCompleted
    void NotifyDependenciesCompleted(Task* task_raw) {
        {
            std::lock_guard<std::mutex> lk(mutex_);
            auto it = tasks_not_ready_.find(task_raw);
            CHECK(it != tasks_not_ready_.end());
            task_queue_.push_back(std::move(it->second));
            tasks_not_ready_.erase(it);
        }
        cv_.notify_one();
    }

    void DoWork() {
#ifdef __linux__
        nice(10);
#endif
        for (;;) {
            std::shared_ptr<Task> task;
            {
                std::unique_lock<std::mutex> lk(mutex_);
                cv_.wait(lk, [this] { return !task_queue_.empty() || !running_; });
                if (task_queue_.empty()) return;
                task = std::move(task_queue_.front());
                task_queue_.pop_front();
                ++active_count_;
            }
            CHECK_EQ(task->GetState(), Task::DEPENDENCIES_COMPLETED);
            task->Execute();
            {
                std::lock_guard<std::mutex> lk(mutex_);
                --active_count_;
                if (task_queue_.empty() && active_count_ == 0) {
                    cv_idle_.notify_all();
                }
            }
        }
    }

    std::mutex mutex_;
    std::condition_variable cv_;
    std::condition_variable cv_idle_;
    bool running_ = true;
    int active_count_ = 0;
    std::vector<std::thread> pool_;
    std::deque<std::shared_ptr<Task>> task_queue_;
    std::unordered_map<Task*, std::shared_ptr<Task>> tasks_not_ready_;
};

// ── Inline definition of Task::OnDependencyCompleted ────────────────────────
// Must be defined AFTER ThreadPool (circular header dependency).
// task.h declares it; thread_pool.h provides the body.
inline bool Task::OnDependencyCompleted() {
    bool became_ready = false;
    {
        std::lock_guard<std::mutex> lk(mutex_);
        CHECK(state_ == NEW || state_ == DISPATCHED);
        --uncompleted_deps_;
        if (uncompleted_deps_ == 0 && state_ == DISPATCHED) {
            state_ = DEPENDENCIES_COMPLETED;
            became_ready = true;
        }
    }
    if (became_ready && pool_ != nullptr) {
        pool_->NotifyDependenciesCompleted(this);
    }
    return became_ready;
}

}  // namespace common
}  // namespace lightning
