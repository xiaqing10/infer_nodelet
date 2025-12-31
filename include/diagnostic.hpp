
#include <string>
#include <memory>
#include <map>
#include <diagnostic_updater/diagnostic_updater.h>
#include "rvf_system_code.hpp"

enum ERROR_CODE_LEVEL{
    OK = 0,
    WARN = 1,
    ERROR = 2,
    STALE = 3 
};

class Diagnostic
{
public:
    Diagnostic();
    ~Diagnostic();    

    void init(std::string diagnosticName,std::string hardwareName,rvf::system::ModuleCode moduleCode); // 诊断的模块错误码编号
    // errorCode    : 错误编码，详情见KeyCode
    // errorInfo    : 错误信息，详细错误描述字段
    int32_t PubDiagnosticData(ERROR_CODE_LEVEL errorCodeLvl,
                              rvf::system::KeyCode errorCode,
                              std::string errorInfo);

private:
    void diagnosticCallback(diagnostic_updater::DiagnosticStatusWrapper &stat);

private:
    ERROR_CODE_LEVEL errorCodeLvl_;
    rvf::system::KeyCode  errorCode_;
    std::string errorInfo_;
    diagnostic_updater::Updater updater_;
};


Diagnostic::Diagnostic() {
}

Diagnostic::~Diagnostic() {
    // updater_.shutdown();
}   
                    
void Diagnostic::init(std::string diagnosticName,
                        std::string hardwareName,
               rvf::system::ModuleCode moduleCode)
    {
        errorCodeLvl_ = ERROR_CODE_LEVEL::OK;
        errorCode_ = rvf::system::KeyCode::kOK;
        errorInfo_ = "No problem";

        updater_.setHardwareID(hardwareName);
        updater_.add(diagnosticName, this, &Diagnostic::diagnosticCallback);
    }

void Diagnostic::diagnosticCallback(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
    rvf::system::RVFCode<rvf::system::KeyCode> code(rvf::system::ModuleCode::kInfer);

    // key value
    stat.add("Error Codes", code.GetKeyCode(errorCode_));

    // level & message
    switch(errorCodeLvl_)
    {
        case ERROR_CODE_LEVEL::OK: { stat.summary(diagnostic_msgs::DiagnosticStatus::OK, errorInfo_); break; }
        case ERROR_CODE_LEVEL::WARN: { stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, errorInfo_); break; }
        case ERROR_CODE_LEVEL::ERROR: { stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, errorInfo_); break; }
        case ERROR_CODE_LEVEL::STALE: { stat.summary(diagnostic_msgs::DiagnosticStatus::STALE, errorInfo_); break; }
    }
}

int32_t Diagnostic::PubDiagnosticData(ERROR_CODE_LEVEL errorCodeLvl,
                                      rvf::system::KeyCode errorCode,
                                      std::string errorInfo)
{
    errorCodeLvl_ = errorCodeLvl;
    errorCode_ = errorCode;
    errorInfo_ = errorInfo;

    updater_.force_update(); 

    return 0;
}
