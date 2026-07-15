#ifndef RVF_SYSTEM_CODE_HPP_
#define RVF_SYSTEM_CODE_HPP_
#include <string>
#include <memory>
#include <map>

namespace rvf
{
    namespace system
    {
        enum class ModuleCode : int32_t
        {
            kUnknown = -1,
            kRadarBridge = 200,
            kRtsp = 200,
            kInfer = 300,
            kProject = 400,
            kCameraEvent = 500,
            kFusion = 600,
            kTrafficEvent = 700,
            kKafkaBridge = 800,
            kZmqProtoBridge = 900,
        }; // enum class ModuleCode
    
        enum class KeyCode : int32_t
        {
            kOK = 0, // 特别指令，结果正确 / 调用正常
            kFailed = 1, // 无特别定义的失败
            kUninitialized = 2, // 未初始化
            kParametersInvalid = 3, // 参数不合法
            kSelfCheckFailed = 4, // 自检失败
            kTimeout = 5, // 超时
            kInputException = 6, // 输入异常
            kDeviceError = 7, // 硬件，外设、传感器类错误
            kUnSupport = 8, // 指令不支持

            kModelLoadFailed = 21, // 模型加载失败 
            kNoCameraData = 22, // 摄像头无数据 
            kNoRadarData = 23, // 雷达无数据 
            kCameraRadarOversize = 24 , // 相机雷达时间戳差异过大
            kNoInferData = 25, // 推理无输出 
            kBlockScreen = 26, // 遮挡
            kBlackScreen = 27, // 黑屏 
            kSnowScreen = 28, // 雪花 
            kBrightnessScreen = 29, // 过亮 
            kBlurScreen = 30, // 模糊
            kImgCodeError = 31 // 图像无法解码
        }; // enum class KeyCode

        template<typename Code_T>
        class RVFCode final
        {
        public:
            explicit RVFCode(ModuleCode module_code)
            {
                ModuleImpl_ = module_code;
            }
            ~RVFCode() {}

            int32_t GetKeyCode(KeyCode code)
            {
                return code == KeyCode::kOK ? static_cast<int32_t>(KeyCode::kOK)
                        : static_cast<int32_t>(ModuleImpl_) + static_cast<int32_t>(code);
            }
            int32_t GetCode(Code_T code)
            {
                return static_cast<int32_t>(ModuleImpl_) + static_cast<int32_t>(code);
            }

        private:
            ModuleCode ModuleImpl_{ModuleCode::kUnknown};
        };
    }
}
#endif