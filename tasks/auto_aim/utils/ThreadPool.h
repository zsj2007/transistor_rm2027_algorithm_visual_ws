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

    size_t threadCount() const { return workers_.size(); }//返回 workers_（vector 里存的线程对象）有多少个

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
    }//每个 worker 线程一出生就泡在这个循环里：睡觉 → 被叫醒 → 取任务 → 执行 → 再睡觉，直到打烊才下班。

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
//线程池的"总机"——整个程序共享一个全局线程池的入口。它用了一个经典技巧：函数内的 static 局部变量实现单例
//threadPool() = "懒加载 + 线程安全 + 自动销毁"的全局单例线程池：第一次调用按需创建（线程数可指定或自动用核数），之后所有人拿到的都是同一个池，程序结束时自动收摊。

//一个补充：static 局部变量这套"只初始化一次、线程安全、自动析构"的保证，其实正是你前面学的局部 static 的通用威力——这里只是用它来装线程池而已。整个线程池项目到此就完整闭环了
