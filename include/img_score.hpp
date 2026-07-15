#include <opencv2/opencv.hpp>
#include <vector>

//  遮挡检测
double blockDetect(const cv::Mat& srcImg) {

    cv::Mat grayImg, edges;
    cv::cvtColor(srcImg, grayImg, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(grayImg, grayImg, cv::Size(3, 3), 0);
    cv::Canny(grayImg, edges, 0, 0);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;

    // 初始轮廓检测
    cv::findContours(edges, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);
    int initialContourCount = static_cast<int>(hierarchy.size());

    // 细化后的轮廓检测
    cv::Canny(grayImg, edges, 0, 15);
    cv::findContours(edges, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);
    int refinedContourCount = static_cast<int>(hierarchy.size());

    // 防止除以零的情况
    if (initialContourCount == 0) {
      initialContourCount = 1;
    }

    double blockEffect = 1.0 - static_cast<double>(refinedContourCount) / static_cast<double>(initialContourCount);
    return blockEffect;

}

// 黑屏检测
double signalDetect(const cv::Mat& srcImg) {
  int32_t width = srcImg.cols;
  int32_t height = srcImg.rows;

  // 缩小图像尺寸，直到宽度和高度不大于352和288
  while (width > 352 && height > 288) {
    width >>= 1;
    height >>= 1;
  }

  cv::Mat resizedImg, grayImg;
  cv::resize(srcImg, resizedImg, cv::Size(width, height));
  cv::cvtColor(resizedImg, grayImg, cv::COLOR_BGR2GRAY);

  // 计算高斯模糊尺寸
  int32_t blurWidth = static_cast<int32_t>(0.2 * grayImg.cols);
  if (blurWidth % 2 == 0) {
    blurWidth += 1;
  }

  int32_t blurHeight = static_cast<int32_t>(0.3 * grayImg.rows);
  if (blurHeight % 2 == 0) {
    blurHeight += 1;
  }

  // 应用高斯模糊和Canny边缘检测
  cv::GaussianBlur(grayImg, grayImg, cv::Size(21, 21), 0);
  cv::Canny(grayImg, grayImg, 0, 0);

  // 查找轮廓
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(grayImg, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

  double sum = static_cast<double>(contours.size());
  double imgArea = static_cast<double>(grayImg.cols * grayImg.rows);
  double res = (sum * sum) / imgArea;
  res *= res;

  double result = 1.0 - (res / 100.0 > 1.0 ? 1.0 : res / 100.0);
  return result;
}

// 图像亮度检测
double brightnessDetect(const cv::Mat& srcImg) {

  cv::Mat labImg;
  cv::cvtColor(srcImg, labImg, cv::COLOR_BGR2Lab);

  double brightness = 0.0;
  const int32_t width = srcImg.cols;
  const int32_t height = srcImg.rows;

  for (int32_t r = 0; r < height; ++r) {
    for (int32_t c = 0; c < width; ++c) {
      uint8_t luminance = labImg.at<cv::Vec3b>(r, c)[0];
      brightness += static_cast<double>(luminance) / 2.55 / static_cast<double>(width * height);
    }
  }

  // std::cout << "brightness: " << brightness << std::endl;

  return brightness /100.0;
}

// 雪花噪点
double snowNoiseDetect(const cv::Mat& srcImg) {

  cv::Mat grayImg;
  if (srcImg.channels() == 1) {
    grayImg = srcImg;
  } else {
    cv::cvtColor(srcImg, grayImg, cv::COLOR_BGR2GRAY);
  }

  cv::Mat filteredImg;
  cv::Mat kernel = (cv::Mat_<double>(3, 3) << 1, 2, 1, 2, 4, 2, 1, 2, 1);
  kernel /= 16.0;
  cv::filter2D(grayImg, filteredImg, grayImg.depth(), kernel, cv::Point(-1, -1));

  double signalPower = 0.0;
  double noisePower = 0.0;

  for (int r = 0; r < grayImg.rows; ++r) {
    for (int c = 0; c < grayImg.cols; ++c) {
      int filteredPixel = filteredImg.at<uchar>(r, c);
      int originalPixel = grayImg.at<uchar>(r, c);
      signalPower += static_cast<double>(originalPixel) * static_cast<double>(originalPixel) / 255.0 / 255.0;
      noisePower += static_cast<double>(originalPixel - filteredPixel) *
                    static_cast<double>(originalPixel - filteredPixel) / 255.0 / 255.0;
    }
  }

  if (signalPower < 1.0) {
    signalPower = 1.0;
  }
  if (noisePower < 1.0) {
    noisePower = 1.0;
  }

  double snr = 20.0 * std::log10(signalPower / noisePower);

  return snr / 100.0;
}

// 图像模糊检测
double sharpnessDetect(const cv::Mat& srcImg) {
  const int kGaussianSize = 3;  // 高斯模糊的核大小

  cv::Mat grayImg;
  if (srcImg.channels() != 1) {
    cv::cvtColor(srcImg, grayImg, cv::COLOR_BGR2GRAY);
  } else {
    grayImg = srcImg;
  }

  cv::Mat blurredImg;
  cv::GaussianBlur(grayImg, blurredImg, cv::Size(kGaussianSize, kGaussianSize), 0);

  uint64_t sumFver = 0;
  uint64_t sumFhor = 0;
  uint64_t sumVver = 0;
  uint64_t sumVhor = 0;
  double blurFactor = 0.0;

  for (int r = 0; r < grayImg.rows; ++r) {
    for (int c = 0; c < grayImg.cols; ++c) {
      uint64_t diffFver = 0;
      uint64_t diffFhor = 0;
      uint64_t diffBver = 0;
      uint64_t diffBhor = 0;
      if (r != 0) {
        diffFver = static_cast<uint64_t>(std::abs(grayImg.at<uchar>(r, c) - grayImg.at<uchar>(r - 1, c)));
      }
      if (c != 0) {
        diffFhor = static_cast<uint64_t>(std::abs(grayImg.at<uchar>(r, c) - grayImg.at<uchar>(r, c - 1)));
      }
      if (r != 0) {
        diffBver = static_cast<uint64_t>(std::abs(blurredImg.at<uchar>(r, c) - blurredImg.at<uchar>(r - 1, c)));
      }
      if (c != 0) {
        diffBhor = static_cast<uint64_t>(std::abs(blurredImg.at<uchar>(r, c) - blurredImg.at<uchar>(r, c - 1)));
      }

      uint64_t verDiff = (diffFver > diffBver) ? (diffFver - diffBver) : 0;
      uint64_t horDiff = (diffFhor > diffBhor) ? (diffFhor - diffBhor) : 0;

      sumFver += diffFver;
      sumFhor += diffFhor;
      sumVver += verDiff;
      sumVhor += horDiff;
    }
  }

  double bFver = (static_cast<double>(sumFver - sumVver)) / (static_cast<double>(sumFver) + 1.0);
  double bFhor = (static_cast<double>(sumFhor - sumVhor)) / (static_cast<double>(sumFhor) + 1.0);
  blurFactor = (bFver > bFhor) ? bFver : bFhor;

  return 1.0 - blurFactor;
}


// 白天和晚上的判断

bool DayOrNight(Mat mat)
{
    //转换颜色空间为HSV
    cv::Mat hsvImage;
    cv::cvtColor(mat, hsvImage, cv::COLOR_BGR2HSV);
    //提取亮度通道
    std::vector<cv::Mat> channels;
    cv::split(hsvImage, channels);
    cv::Mat brightness = channels[2];
    //计算亮度阈值
    cv::Mat thresholdImage;
    cv::threshold(brightness, thresholdImage, 60, 255, cv::THRESH_BINARY);
    //统计像素数量
    int whitePixels = cv::countNonZero(thresholdImage);
    int blackPixels = thresholdImage.total() - whitePixels;
    return (whitePixels > blackPixels);
}
