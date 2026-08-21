#include "shm/VisualizerShm.h"

#include <cerrno>
#include <cstdio>
#include <cstring>

#include <sys/stat.h>
#include <time.h>

namespace {

// 把 BGR8 Mat 拷入共享内存中的固定缓冲；不满足条件（空图/非 BGR8/超上限）则清空该图像
template <typename ImageT>
void copyImage(const cv::Mat& src, ImageT& dst)
{
    dst.cols = 0;
    dst.rows = 0;
    dst.type = 0;
    dst.bytes = 0;
    if (src.empty()) {
        return;
    }
    if (src.depth() != CV_8U || src.channels() != 3) {
        std::fprintf(stderr, "[VisualizerShm] skip image: only CV_8UC3 supported\n");
        return;
    }
    const size_t bytes = src.total() * src.elemSize();
    if (bytes > sizeof(dst.data)) {
        std::fprintf(stderr,
            "[VisualizerShm] skip image: %zux%zu exceeds buffer limit %zu\n",
            static_cast<size_t>(src.cols), static_cast<size_t>(src.rows), sizeof(dst.data));
        return;
    }
    dst.cols = src.cols;
    dst.rows = src.rows;
    dst.type = src.type();
    dst.bytes = static_cast<uint32_t>(bytes);
    if (src.isContinuous()) {
        std::memcpy(dst.data, src.data, bytes);
    } else {
        cv::Mat continuous;
        src.copyTo(continuous);
        std::memcpy(dst.data, continuous.data, bytes);
    }
}

// 打开（必要时创建）共享内存段；段大小/魔数不匹配时重建
int createSharedSegment(key_t key, size_t required_size, uint32_t magic)
{
    int shm_id = shmget(key, required_size, IPC_CREAT | 0666);
    if (shm_id == -1) {
        std::perror("[VisualizerShm] shmget failed");
        return -1;
    }

    struct shmid_ds info;
    if (shmctl(shm_id, IPC_STAT, &info) == -1) {
        std::perror("[VisualizerShm] shmctl(IPC_STAT) failed");
        return -1;
    }

    // 旧段尺寸不匹配：删掉重建（本段仅用于可视化，可安全回收）
    if (info.shm_segsz != required_size) {
        if (shmctl(shm_id, IPC_RMID, nullptr) == -1) {
            std::perror("[VisualizerShm] shmctl(IPC_RMID) failed");
            return -1;
        }
        shm_id = shmget(key, required_size, IPC_CREAT | 0666);
        if (shm_id == -1) {
            std::perror("[VisualizerShm] shmget (recreate) failed");
            return -1;
        }
    }

    void* addr = shmat(shm_id, nullptr, 0);
    if (addr == reinterpret_cast<void*>(-1)) {
        std::perror("[VisualizerShm] shmat failed");
        return -1;
    }

    auto* layout = static_cast<visualizer_shm::SharedLayout*>(addr);
    if (layout->magic != magic || layout->layout_size != required_size) {
        std::memset(addr, 0, required_size);
        layout->magic = magic;
        layout->layout_size = static_cast<uint32_t>(required_size);
    }
    shmdt(addr);
    return shm_id;
}

// 打开（必要时创建）命名信号量，并清掉残留计数
sem_t* openSemaphore(const char* name, bool reset)
{
    sem_t* sem = sem_open(name, O_CREAT | O_EXCL, 0666, 0);
    if (sem == SEM_FAILED) {
        sem = sem_open(name, 0);
    }
    if (sem != SEM_FAILED && reset) {
        visualizer_shm::resetSemaphore(sem);
    }
    return sem;
}

}  // namespace

// ==================== Writer ====================

VisualizerShmWriter::VisualizerShmWriter(const std::shared_ptr<YAML::Node>& config_file_ptr)
{
    const key_t key = visualizer_shm::shmKeyFromConfig(config_file_ptr);
    const size_t shm_size = sizeof(visualizer_shm::SharedLayout);

    shm_id_ = createSharedSegment(key, shm_size, visualizer_shm::kMagic);
    if (shm_id_ == -1) {
        return;
    }

    sem_ = openSemaphore(visualizer_shm::kSemaphoreName, /*reset=*/true);
    if (sem_ == SEM_FAILED) {
        std::fprintf(stderr, "[VisualizerShm] sem_open failed\n");
        return;
    }

    shared_ = static_cast<visualizer_shm::SharedLayout*>(shmat(shm_id_, nullptr, 0));
    if (shared_ == reinterpret_cast<void*>(-1)) {
        shared_ = nullptr;
        std::perror("[VisualizerShm] shmat failed");
        return;
    }
    valid_ = true;
}

VisualizerShmWriter::~VisualizerShmWriter()
{
    if (shared_ != nullptr) {
        shmdt(shared_);
        shared_ = nullptr;
    }
    if (sem_ != SEM_FAILED) {
        sem_close(sem_);
        sem_ = SEM_FAILED;
    }
}

bool VisualizerShmWriter::publish(const visualizer_shm::DebugData& debug,
                                  const cv::Mat& raw_frame,
                                  const cv::Mat& rmm_frame,
                                  const cv::Mat& cdo_frame)
{
    if (!valid_ || shared_ == nullptr) {
        return false;
    }
    // 没有可视化进程挂接时跳过整帧发布，避免每帧 10+MB 的无效拷贝
    if (!shared_->reader_attached) {
        return true;
    }

    visualizer_shm::Snapshot& snap = shared_->snapshot;
    const uint64_t frame_id = next_frame_id_++;

    snap.debug = debug;
    copyImage(raw_frame, snap.raw_frame);
    copyImage(rmm_frame, snap.rmm_frame);
    copyImage(cdo_frame, snap.cdo_frame);
    const int64_t now_ms = static_cast<int64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
    snap.writer_timestamp_ms = now_ms;

    // 发布帧率：writer 自身滚动 1s 窗口实测（等价于流水线吞吐，无接收侧采样抖动）
    publish_times_ms_.push_back(now_ms);
    while (!publish_times_ms_.empty() && now_ms - publish_times_ms_.front() > 1000) {
        publish_times_ms_.pop_front();
    }
    if (publish_times_ms_.size() >= 2) {
        const double span_s =
            static_cast<double>(now_ms - publish_times_ms_.front()) / 1000.0;
        snap.writer_fps = static_cast<float>(
            static_cast<double>(publish_times_ms_.size() - 1) / span_s);
    } else {
        snap.writer_fps = 0.0f;
    }

    // 帧号最后写：保证 reader 读到的 frame_id 之前的数据是完整的一帧
    __sync_synchronize();
    snap.frame_id = frame_id;
    sem_post(sem_);
    return true;
}

// ==================== Reader ====================

VisualizerShmReader::VisualizerShmReader(const std::shared_ptr<YAML::Node>& config_file_ptr)
{
    const key_t key = visualizer_shm::shmKeyFromConfig(config_file_ptr);
    const size_t shm_size = sizeof(visualizer_shm::SharedLayout);

    shm_id_ = createSharedSegment(key, shm_size, visualizer_shm::kMagic);
    if (shm_id_ == -1) {
        return;
    }

    sem_ = openSemaphore(visualizer_shm::kSemaphoreName, /*reset=*/false);
    if (sem_ == SEM_FAILED) {
        std::fprintf(stderr, "[VisualizerShm] sem_open failed\n");
        return;
    }

    shared_ = static_cast<visualizer_shm::SharedLayout*>(shmat(shm_id_, nullptr, 0));
    if (shared_ == reinterpret_cast<void*>(-1)) {
        shared_ = nullptr;
        std::perror("[VisualizerShm] shmat failed");
        return;
    }

    if (shared_->magic != visualizer_shm::kMagic ||
        shared_->layout_size != sizeof(visualizer_shm::SharedLayout)) {
        std::fprintf(stderr, "[VisualizerShm] shared memory not initialized by writer yet\n");
        shmdt(shared_);
        shared_ = nullptr;
        return;
    }

    shared_->reader_attached = true;
    valid_ = true;
}

VisualizerShmReader::~VisualizerShmReader()
{
    if (shared_ != nullptr) {
        shared_->reader_attached = false;
        shmdt(shared_);
        shared_ = nullptr;
    }
    if (sem_ != SEM_FAILED) {
        sem_close(sem_);
        sem_ = SEM_FAILED;
    }
}

bool VisualizerShmReader::waitForSnapshot(visualizer_shm::Snapshot& out, int timeout_ms)
{
    if (!valid_ || shared_ == nullptr || sem_ == SEM_FAILED) {
        return false;
    }

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (true) {
        const auto remaining = deadline - std::chrono::steady_clock::now();
        if (remaining <= std::chrono::milliseconds(0)) {
            return false;
        }

        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        const int64_t remain_ms = std::chrono::duration_cast<std::chrono::milliseconds>(remaining).count();
        ts.tv_sec += remain_ms / 1000;
        ts.tv_nsec += (remain_ms % 1000) * 1000000L;
        if (ts.tv_nsec >= 1000000000L) {
            ts.tv_sec += 1;
            ts.tv_nsec -= 1000000000L;
        }

        if (sem_timedwait(sem_, &ts) != 0) {
            return false;  // 超时
        }

        // 读到新帧号后整体拷贝，再校验帧号没变；变了说明拷贝期间被覆盖，重试
        for (int attempt = 0; attempt < 4; ++attempt) {
            const uint64_t frame_id = shared_->snapshot.frame_id;
            if (frame_id == 0 || frame_id == last_frame_id_) {
                break;  // 残留通知，回到等待
            }
            out = shared_->snapshot;
            __sync_synchronize();
            if (shared_->snapshot.frame_id == frame_id) {
                last_frame_id_ = frame_id;
                return true;
            }
        }
        // 4 次校验都失败（极端竞争），回到等待下一条通知
    }
}
