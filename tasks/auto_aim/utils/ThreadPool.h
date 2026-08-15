#pragma once

#include <condition_variable>
#include <cstddef>
#include <functional>
#include <future>
#include <iterator>
#include <memory>
#include <mutex>
#include <queue>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

namespace utils {

// 通用固定大小线程池。
// 线程常驻复用，避免频繁创建/销毁 std::thread 的开销；
// 池内线程在无任务时阻塞等待（不空转），用事件通知唤醒。
class ThreadPool {
public:
    explicit ThreadPool(size_t n_threads = std::thread::hardware_concurrency())
        : workers_(n_threads == 0 ? 1 : n_threads) {
        for (size_t i = 0; i < workers_.size(); ++i) {
            workers_[i] = std::thread(&ThreadPool::workerLoop, this);
        }
    }

    ~ThreadPool() { shutdown(); }

    ThreadPool(const ThreadPool&) = delete;
    ThreadPool& operator=(const ThreadPool&) = delete;

    // 提交一个任务，返回 std::future 用于等待结果/异常。
    template <typename F, typename... Args>
    auto submit(F&& f, Args&&... args)
        -> std::future<std::invoke_result_t<F, Args...>> {
        using ReturnType = std::invoke_result_t<F, Args...>;
        auto task = std::make_shared<std::packaged_task<ReturnType()>>(
            std::bind(std::forward<F>(f), std::forward<Args>(args)...));
        std::future<ReturnType> future = task->get_future();
        {
            std::lock_guard<std::mutex> lock(mtx_);
            if (stop_) {
                throw std::runtime_error(
                    "ThreadPool::submit called on stopped pool");
            }
            tasks_.emplace([task]() { (*task)(); });
        }
        cv_.notify_one();
        return future;
    }

    // 并行遍历 [begin, end)：将区间分成若干块提交到池中，全部完成后返回。
    // f 会被按值拷贝进每个任务（拷贝的是 lambda 本身，引用捕获无额外开销）。
    template <typename Iter, typename F>
    void parallel_for_each(Iter begin, Iter end, F&& f) {
        const size_t total = static_cast<size_t>(std::distance(begin, end));
        if (total == 0) {
            return;
        }
        const size_t n_workers = workers_.size();
        const size_t chunk_size = (total + n_workers - 1) / n_workers;
        std::vector<std::future<void>> futures;
        futures.reserve((total + chunk_size - 1) / chunk_size);

        Iter it = begin;
        while (it != end) {
            Iter chunk_begin = it;
            Iter chunk_end = it;
            size_t step = 0;
            while (step < chunk_size && chunk_end != end) {
                ++chunk_end;
                ++step;
            }
            it = chunk_end;
            F f_copy = f;  // 每个任务持有一份 functor 拷贝
            futures.emplace_back(submit([chunk_begin, chunk_end, f_copy]() mutable {
                for (Iter p = chunk_begin; p != chunk_end; ++p) {
                    f_copy(*p);
                }
            }));
        }
        for (auto& future : futures) {
            future.get();  // 等待全部完成，若任务抛出异常则在此传播
        }
    }

    size_t threadCount() const { return workers_.size(); }

    // 停止接收新任务，等待已提交任务完成后 join 所有线程。幂等。
    void shutdown() {
        {
            std::lock_guard<std::mutex> lock(mtx_);
            if (stop_) {
                return;
            }
            stop_ = true;
        }
        cv_.notify_all();
        for (auto& worker : workers_) {
            if (worker.joinable()) {
                worker.join();
            }
        }
    }

private:
    void workerLoop() {
        while (true) {
            std::function<void()> task;
            {
                std::unique_lock<std::mutex> lock(mtx_);
                cv_.wait(lock, [this]() { return stop_ || !tasks_.empty(); });
                if (stop_ && tasks_.empty()) {
                    return;
                }
                task = std::move(tasks_.front());
                tasks_.pop();
            }
            task();
        }
    }

    std::vector<std::thread> workers_;
    std::queue<std::function<void()>> tasks_;
    std::mutex mtx_;
    std::condition_variable cv_;
    bool stop_ = false;
};

// 进程级单例：所有组件共享同一个池，统一线程预算，避免各自建池导致线程爆炸。
// 首次调用时决定线程数；可在启动早期（创建池前）用
//   utils::threadPool(8);
// 显式指定线程数，之后再调用 utils::threadPool() 均返回同一个池。
inline ThreadPool& threadPool(size_t n_threads = 0) {
    static size_t thread_count =
        n_threads > 0 ? n_threads : std::thread::hardware_concurrency();
    static ThreadPool pool(thread_count);
    return pool;
}

}  // namespace utils
