#include "ffmpeg_video_reader.h"

FFmpegVideoReader::FFmpegVideoReader()
    : fmt_ctx_(nullptr), codec_ctx_(nullptr), frame_(nullptr), rgb_frame_(nullptr),
      sws_ctx_(nullptr), video_stream_index_(-1), fps_(0), total_frames_(0),
      width_(0), height_(0) {
    av_register_all();
}

FFmpegVideoReader::~FFmpegVideoReader() {
    release();
}

bool FFmpegVideoReader::open(const std::string& filename) {
    if (avformat_open_input(&fmt_ctx_, filename.c_str(), nullptr, nullptr) != 0) {
        return false;
    }

    if (avformat_find_stream_info(fmt_ctx_, nullptr) < 0) {
        avformat_close_input(&fmt_ctx_);
        fmt_ctx_ = nullptr;
        return false;
    }

    for (unsigned i = 0; i < fmt_ctx_->nb_streams; i++) {
        if (fmt_ctx_->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
            video_stream_index_ = i;
            break;
        }
    }

    if (video_stream_index_ == -1) {
        avformat_close_input(&fmt_ctx_);
        fmt_ctx_ = nullptr;
        return false;
    }

    AVCodecParameters* codecpar = fmt_ctx_->streams[video_stream_index_]->codecpar;
    AVCodec* codec = avcodec_find_decoder(codecpar->codec_id);
    if (!codec) {
        avformat_close_input(&fmt_ctx_);
        fmt_ctx_ = nullptr;
        return false;
    }

    codec_ctx_ = avcodec_alloc_context3(codec);
    if (!codec_ctx_) {
        avformat_close_input(&fmt_ctx_);
        fmt_ctx_ = nullptr;
        return false;
    }

    if (avcodec_parameters_to_context(codec_ctx_, codecpar) < 0) {
        avcodec_free_context(&codec_ctx_);
        avformat_close_input(&fmt_ctx_);
        fmt_ctx_ = nullptr;
        return false;
    }

    if (avcodec_open2(codec_ctx_, codec, nullptr) < 0) {
        avcodec_free_context(&codec_ctx_);
        avformat_close_input(&fmt_ctx_);
        fmt_ctx_ = nullptr;
        return false;
    }

    width_ = codec_ctx_->width;
    height_ = codec_ctx_->height;

    AVStream* stream = fmt_ctx_->streams[video_stream_index_];
    fps_ = av_q2d(stream->avg_frame_rate);
    if (fps_ <= 0) {
        fps_ = av_q2d(stream->r_frame_rate);
    }
    if (fps_ <= 0) {
        fps_ = 30.0;
    }

    total_frames_ = stream->nb_frames;
    if (total_frames_ <= 0) {
        total_frames_ = (int)(stream->duration * fps_ / AV_TIME_BASE);
    }

    frame_ = av_frame_alloc();
    rgb_frame_ = av_frame_alloc();

    int num_bytes = av_image_get_buffer_size(AV_PIX_FMT_BGR24, width_, height_, 1);
    uint8_t* buffer = (uint8_t*)av_malloc(num_bytes);
    av_image_fill_arrays(rgb_frame_->data, rgb_frame_->linesize, buffer, AV_PIX_FMT_BGR24, width_, height_, 1);

    sws_ctx_ = sws_getContext(width_, height_, codec_ctx_->pix_fmt,
                              width_, height_, AV_PIX_FMT_BGR24,
                              SWS_BILINEAR, nullptr, nullptr, nullptr);

    return true;
}

bool FFmpegVideoReader::read(cv::Mat& frame) {
    AVPacket packet;
    while (av_read_frame(fmt_ctx_, &packet) >= 0) {
        if (packet.stream_index == video_stream_index_) {
            int ret = avcodec_send_packet(codec_ctx_, &packet);
            if (ret < 0) {
                av_packet_unref(&packet);
                continue;
            }

            ret = avcodec_receive_frame(codec_ctx_, frame_);
            if (ret == 0) {
                sws_scale(sws_ctx_, frame_->data, frame_->linesize,
                          0, height_, rgb_frame_->data, rgb_frame_->linesize);

                cv::Mat(height_, width_, CV_8UC3, rgb_frame_->data[0], rgb_frame_->linesize[0]).copyTo(frame);
                av_packet_unref(&packet);
                return true;
            }
        }
        av_packet_unref(&packet);
    }
    return false;
}

void FFmpegVideoReader::release() {
    if (sws_ctx_) {
        sws_freeContext(sws_ctx_);
        sws_ctx_ = nullptr;
    }
    if (rgb_frame_) {
        if (rgb_frame_->data[0]) av_free(rgb_frame_->data[0]);
        av_frame_free(&rgb_frame_);
    }
    if (frame_) {
        av_frame_free(&frame_);
    }
    if (codec_ctx_) {
        avcodec_free_context(&codec_ctx_);
    }
    if (fmt_ctx_) {
        avformat_close_input(&fmt_ctx_);
    }
}
