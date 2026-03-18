// Inspired by cartographer/common/task.h
// Simplified header-only implementation using C++ standard library (no absl).
//
// A Task wraps a std::function<void()> work item and tracks dependencies.
// When all dependencies have completed, the owning ThreadPool queues it
// for execution by a worker thread.
//
// Lifecycle: NEW → DISPATCHED → DEPENDENCIES_COMPLETED → RUNNING → COMPLETED
//
// Thread safety: all public methods are thread-safe.

#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <set>

#include <glog/logging.h>

namespace lightning {
namespace common {

class ThreadPool;  // forward

class Task {
   public:
    friend class ThreadPool;

    using WorkItem = std::function<void()>;
    enum State { NEW, DISPATCHED, DEPENDENCIES_COMPLETED, RUNNING, COMPLETED };

    Task() = default;
    ~Task() {
        std::lock_guard<std::mutex> lk(mutex_);
        if (state_ != NEW && state_ != COMPLETED) {
            LOG(WARNING) << "Task destroyed between dispatch and completion";
        }
    }

    State GetState() {
        std::lock_guard<std::mutex> lk(mutex_);
        return state_;
    }

    // Must be called before Schedule(). State must be NEW.
    void SetWorkItem(const WorkItem& work_item) {
        std::lock_guard<std::mutex> lk(mutex_);
        CHECK_EQ(state_, NEW);
        work_item_ = work_item;
    }

    // Add a dependency that must complete before this task runs.
    // If dependency's weak_ptr is already expired it is counted as done.
    // State must be NEW.
    void AddDependency(std::weak_ptr<Task> dependency) {
        std::shared_ptr<Task> shared_dep;
        {
            std::lock_guard<std::mutex> lk(mutex_);
            CHECK_EQ(state_, NEW);
            shared_dep = dependency.lock();
            if (shared_dep) {
                ++uncompleted_deps_;
            }
        }
        if (shared_dep) {
            shared_dep->AddDependentTask(this);
        }
    }

   private:
    // Called by ThreadPool::Schedule. Returns true if immediately ready.
    bool SetThreadPool(ThreadPool* pool) {
        std::lock_guard<std::mutex> lk(mutex_);
        CHECK_EQ(state_, NEW);
        state_ = DISPATCHED;
        pool_ = pool;
        if (uncompleted_deps_ == 0) {
            state_ = DEPENDENCIES_COMPLETED;
            return true;  // ready immediately → caller enqueues
        }
        return false;
    }

    // Called by a completed dependency.
    // Returns true if this task just became ready (and notifies ThreadPool).
    // DEFINITION is in thread_pool.h (after ThreadPool is fully declared,
    // to break the circular header dependency).
    bool OnDependencyCompleted();

    // Register a dependent task.
    void AddDependentTask(Task* dependent) {
        bool already_done = false;
        {
            std::lock_guard<std::mutex> lk(mutex_);
            if (state_ == COMPLETED) {
                already_done = true;
            } else {
                dependents_.insert(dependent);
            }
        }
        if (already_done) {
            dependent->OnDependencyCompleted();
        }
    }

    // Execute the work item. Called by worker thread.
    // Returns the set of dependents to notify (to avoid calling under lock).
    void Execute() {
        {
            std::lock_guard<std::mutex> lk(mutex_);
            CHECK_EQ(state_, DEPENDENCIES_COMPLETED);
            state_ = RUNNING;
        }
        if (work_item_) work_item_();
        std::set<Task*> to_notify;
        {
            std::lock_guard<std::mutex> lk(mutex_);
            state_ = COMPLETED;
            to_notify = std::move(dependents_);
        }
        for (Task* dep : to_notify) {
            dep->OnDependencyCompleted();
        }
    }

    std::mutex mutex_;
    WorkItem work_item_;
    ThreadPool* pool_ = nullptr;
    State state_ = NEW;
    unsigned int uncompleted_deps_ = 0;
    std::set<Task*> dependents_;
};

}  // namespace common
}  // namespace lightning
