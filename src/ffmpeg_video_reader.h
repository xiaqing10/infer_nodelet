#ifndef FFMPEG_VIDEO_READER_H
#define FFMPEG_VIDEO_READER_H

#include <string>
#include <opencv2/core/core.hpp>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
#include <libavutil/imgutils.h>
}

class FFmpegVideoReader {
public:
    FFmpegVideoReader();
    ~FFmpegVideoReader();

    bool open(const std::string& filename);
    bool read(cv::Mat& frame);
    void release();

    double getFPS() const { return fps_; }
    int getTotalFrames() const { return total_frames_; }
    int getWidth() const { return width_; }
    int getHeight() const { return height_; }
    bool isOpened() const { return fmt_ctx_ != nullptr; }

private:
    AVFormatContext* fmt_ctx_;
    AVCodecContext* codec_ctx_;
    AVFrame* frame_;
    AVFrame* rgb_frame_;
    SwsContext* sws_ctx_;
    int video_stream_index_;
    double fps_;
    int total_frames_;
    int width_;
    int height_;
};

#endif
