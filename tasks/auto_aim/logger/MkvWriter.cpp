#include "logger/MkvWriter.h"

MkvAllIntraWriter::MkvAllIntraWriter(size_t maxQueueSize)
    : fmtCtx_(nullptr), codecCtx_(nullptr), swsCtx_(nullptr),
      videoStream_(nullptr), frame_(nullptr),
      maxQueueSize_(maxQueueSize), stop_(false), error_(false), frameCount_(0) {}

MkvAllIntraWriter::~MkvAllIntraWriter() {
    close();
}

bool MkvAllIntraWriter::open(const std::string& filename, int width, int height, double fps, int64_t bitrate) {
    int ret;

    // 如果已经打开，先关闭
    close();

    // 1. 分配输出上下文
    ret = avformat_alloc_output_context2(&fmtCtx_, nullptr, nullptr, filename.c_str());
    if (ret < 0 || !fmtCtx_) {
        char errbuf[AV_ERROR_MAX_STRING_SIZE];
        av_make_error_string(errbuf, AV_ERROR_MAX_STRING_SIZE, ret);
        std::cerr << "Could not create output context, error: " << errbuf << std::endl;
        return false;
    }

    // 2. 查找 H.264 编码器
    AVCodec* codec = avcodec_find_encoder(AV_CODEC_ID_H264);
    if (!codec) {
        std::cerr << "H.264 encoder not found" << std::endl;
        goto cleanup;
    }

    // 3. 添加视频流
    videoStream_ = avformat_new_stream(fmtCtx_, codec);
    if (!videoStream_) {
        std::cerr << "Could not create video stream" << std::endl;
        goto cleanup;
    }
    videoStream_->id = fmtCtx_->nb_streams - 1;

    // 4. 分配编码器上下文
    codecCtx_ = avcodec_alloc_context3(codec);
    if (!codecCtx_) {
        std::cerr << "Could not allocate codec context" << std::endl;
        goto cleanup;
    }

    // 基本参数
    codecCtx_->width = width;
    codecCtx_->height = height;
    codecCtx_->time_base = {1, static_cast<int>(fps)};
    videoStream_->time_base = codecCtx_->time_base;
    codecCtx_->framerate = {static_cast<int>(fps), 1};
    codecCtx_->pix_fmt = AV_PIX_FMT_YUV420P;
    codecCtx_->bit_rate = bitrate;
    codecCtx_->gop_size = 1;
    codecCtx_->keyint_min = 1;
    codecCtx_->max_b_frames = 0;

    // x264 特定参数：全关键帧
    av_opt_set(codecCtx_->priv_data, "x264-params", "keyint=1:min-keyint=1", 0);
    av_opt_set(codecCtx_->priv_data, "preset", "ultrafast", 0);
    codecCtx_->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;

    // 打开编码器
    ret = avcodec_open2(codecCtx_, codec, nullptr);
    if (ret < 0) {
        char errbuf[AV_ERROR_MAX_STRING_SIZE];
        av_make_error_string(errbuf, AV_ERROR_MAX_STRING_SIZE, ret);
        std::cerr << "Could not open codec, error: " << errbuf << std::endl;
        goto cleanup;
    }

    // 复制参数到流
    ret = avcodec_parameters_from_context(videoStream_->codecpar, codecCtx_);
    if (ret < 0) {
        char errbuf[AV_ERROR_MAX_STRING_SIZE];
        av_make_error_string(errbuf, AV_ERROR_MAX_STRING_SIZE, ret);
        std::cerr << "Failed to copy codec parameters to stream, error: " << errbuf << std::endl;
        goto cleanup;
    }

    // 打开输出文件
    if (!(fmtCtx_->oformat->flags & AVFMT_NOFILE)) {
        ret = avio_open(&fmtCtx_->pb, filename.c_str(), AVIO_FLAG_WRITE);
        if (ret < 0) {
            char errbuf[AV_ERROR_MAX_STRING_SIZE];
            av_make_error_string(errbuf, AV_ERROR_MAX_STRING_SIZE, ret);
            std::cerr << "Could not open output file, error: " << errbuf << std::endl;
            goto cleanup;
        }
    }

    // 写入文件头
    ret = avformat_write_header(fmtCtx_, nullptr);
    if (ret < 0) {
        char errbuf[AV_ERROR_MAX_STRING_SIZE];
        av_make_error_string(errbuf, AV_ERROR_MAX_STRING_SIZE, ret);
        std::cerr << "Error writing header, error: " << errbuf << std::endl;
        goto cleanup;
    }

    // 初始化像素转换器（BGR -> YUV420P）
    swsCtx_ = sws_getContext(width, height, AV_PIX_FMT_BGR24,
                             width, height, AV_PIX_FMT_YUV420P,
                             SWS_BILINEAR, nullptr, nullptr, nullptr);
    if (!swsCtx_) {
        std::cerr << "Could not initialize sws context" << std::endl;
        goto cleanup;
    }

    // 分配 AVFrame 用于编码
    frame_ = av_frame_alloc();
    if (!frame_) {
        std::cerr << "Could not allocate frame" << std::endl;
        goto cleanup;
    }
    frame_->format = codecCtx_->pix_fmt;
    frame_->width  = codecCtx_->width;
    frame_->height = codecCtx_->height;
    ret = av_frame_get_buffer(frame_, 0);
    if (ret < 0) {
        char errbuf[AV_ERROR_MAX_STRING_SIZE];
        av_make_error_string(errbuf, AV_ERROR_MAX_STRING_SIZE, ret);
        std::cerr << "Could not allocate frame buffer, error: " << errbuf << std::endl;
        goto cleanup;
    }

    // 启动后台线程
    stop_ = false;
    error_ = false;
    frameCount_ = 0;
    worker_ = std::thread(&MkvAllIntraWriter::encodingLoop, this);

    return true;

cleanup:
    if (swsCtx_) sws_freeContext(swsCtx_), swsCtx_ = nullptr;
    if (frame_) av_frame_free(&frame_);
    if (codecCtx_) avcodec_free_context(&codecCtx_);
    if (fmtCtx_ && !(fmtCtx_->oformat->flags & AVFMT_NOFILE) && fmtCtx_->pb)
        avio_closep(&fmtCtx_->pb);
    if (fmtCtx_) avformat_free_context(fmtCtx_), fmtCtx_ = nullptr;
    videoStream_ = nullptr;
    return false;
}

bool MkvAllIntraWriter::writeFrame(const cv::Mat& bgrMat, bool dropWhenFull) {
    if (error_ || stop_) {
        std::cerr << "Writer is in error or stopped state" << std::endl;
        return false;
    }

    std::unique_lock<std::mutex> lock(mtx_);
    // 队列满处理
    if (frameQueue_.size() >= maxQueueSize_) {
        if (dropWhenFull) {
            // 丢弃该帧
            return true;
        } else {
            // 等待队列有空间
            cvNotFull_.wait(lock, [this] { return frameQueue_.size() < maxQueueSize_ || stop_ || error_; });
            if (stop_ || error_) return false;
        }
    }

    // 深拷贝图像数据，避免原 Mat 被销毁
    frameQueue_.push(bgrMat.clone());
    cvNotEmpty_.notify_one();
    return true;
}

void MkvAllIntraWriter::encodingLoop() {
    // 临时 AVFrame 用于转换（避免重复分配）
    AVFrame* yuvFrame = av_frame_alloc();
    if (!yuvFrame) {
        std::cerr << "Failed to allocate YUV frame in encoding thread" << std::endl;
        error_ = true;
        return;
    }
    yuvFrame->format = codecCtx_->pix_fmt;
    yuvFrame->width  = codecCtx_->width;
    yuvFrame->height = codecCtx_->height;
    if (av_frame_get_buffer(yuvFrame, 0) < 0) {
        std::cerr << "Failed to allocate YUV frame buffer" << std::endl;
        av_frame_free(&yuvFrame);
        error_ = true;
        return;
    }

    while (true) {
        cv::Mat bgrMat;
        {
            std::unique_lock<std::mutex> lock(mtx_);
            cvNotEmpty_.wait(lock, [this] { return !frameQueue_.empty() || stop_; });
            if (stop_ && frameQueue_.empty()) break;
            if (frameQueue_.empty()) continue;
            bgrMat = std::move(frameQueue_.front());
            frameQueue_.pop();
            cvNotFull_.notify_one();
        }

        // 转换 BGR -> YUV
        const int in_linesize[1] = { static_cast<int>(bgrMat.step) };
        sws_scale(swsCtx_, &bgrMat.data, in_linesize, 0, codecCtx_->height,
                  yuvFrame->data, yuvFrame->linesize);

        // 设置 PTS
        int64_t pts = frameCount_++;
        yuvFrame->pts = pts;

        // 编码并写入
        if (!encodeAndWriteFrame(yuvFrame)) {
            error_ = true;
            break;
        }
    }

    // 刷新编码器
    encodeAndWriteFrame(nullptr);

    av_frame_free(&yuvFrame);
}

bool MkvAllIntraWriter::encodeAndWriteFrame(AVFrame* yuvFrame) {
    int ret;

    // 发送帧（可能为 nullptr 表示刷新）
    ret = avcodec_send_frame(codecCtx_, yuvFrame);
    if (ret < 0 && ret != AVERROR_EOF) {
        char errbuf[AV_ERROR_MAX_STRING_SIZE];
        av_make_error_string(errbuf, AV_ERROR_MAX_STRING_SIZE, ret);
        std::cerr << "Error sending frame to encoder: " << errbuf << std::endl;
        return false;
    }

    // 接收所有可用的编码包
    AVPacket* pkt = av_packet_alloc();
    while (true) {
        ret = avcodec_receive_packet(codecCtx_, pkt);
        if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
            break;
        } else if (ret < 0) {
            char errbuf[AV_ERROR_MAX_STRING_SIZE];
            av_make_error_string(errbuf, AV_ERROR_MAX_STRING_SIZE, ret);
            std::cerr << "Error receiving packet: " << errbuf << std::endl;
            av_packet_free(&pkt);
            return false;
        }

        pkt->stream_index = videoStream_->index;
        av_packet_rescale_ts(pkt, codecCtx_->time_base, videoStream_->time_base);

        ret = av_interleaved_write_frame(fmtCtx_, pkt);
        if (ret < 0) {
            char errbuf[AV_ERROR_MAX_STRING_SIZE];
            av_make_error_string(errbuf, AV_ERROR_MAX_STRING_SIZE, ret);
            std::cerr << "Error writing frame: " << errbuf << std::endl;
            av_packet_free(&pkt);
            return false;
        }
        av_packet_unref(pkt);
    }
    av_packet_free(&pkt);
    return true;
}

void MkvAllIntraWriter::close() {
    if (worker_.joinable()) {
        {
            std::lock_guard<std::mutex> lock(mtx_);
            stop_ = true;
        }
        cvNotEmpty_.notify_all();
        worker_.join();
    }

    // 写入文件尾
    if (fmtCtx_ && codecCtx_ && videoStream_) {
        av_write_trailer(fmtCtx_);
    }

    // 释放 FFmpeg 资源
    if (swsCtx_) {
        sws_freeContext(swsCtx_);
        swsCtx_ = nullptr;
    }
    if (frame_) {
        av_frame_free(&frame_);
    }
    if (codecCtx_) {
        avcodec_free_context(&codecCtx_);
    }
    if (fmtCtx_ && !(fmtCtx_->oformat->flags & AVFMT_NOFILE) && fmtCtx_->pb) {
        avio_closep(&fmtCtx_->pb);
    }
    if (fmtCtx_) {
        avformat_free_context(fmtCtx_);
        fmtCtx_ = nullptr;
    }
    videoStream_ = nullptr;
    frameCount_ = 0;
    error_ = false;
}
