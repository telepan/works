#ifndef LOG_UTILS_H
#define LOG_UTILS_H

#include <QtGlobal>

enum class ELogLevel : quint8
{
    Debug = 0,
    Info,
    Warning,
    Error,
};

struct LogManagerParam
{
    ELogLevel level{ELogLevel::Warning}; // 日志级别
    bool logContext{false};              // 登记日志发生的线程、文件名、行号等信息
    bool logToStdout{false};             // 写入标准输出
    bool rfu{false};                     // 保留字段，暂时未使用
};
Q_STATIC_ASSERT_X(sizeof(LogManagerParam) == 4, "LogManagerParam size must be 4 bytes");

extern bool checkLogLevel(ELogLevel level);

#define LOG_ALL qDebug()
#define LOG_DEBUG qDebug()

#define LOG_INFO                         \
    if (!checkLogLevel(ELogLevel::Info)) \
        (void)0;                         \
    else                                 \
        qInfo()

#define LOG_WARN                            \
    if (!checkLogLevel(ELogLevel::Warning)) \
        (void)0;                            \
    else                                    \
        qWarning()

#define LOG_ERROR                         \
    if (!checkLogLevel(ELogLevel::Error)) \
        (void)0;                          \
    else                                  \
        qCritical()

#define LOG_DEBUG_FMT(...) LOG_ALL << QString::asprintf(__VA_ARGS__);
#define LOG_INFO_FMT(...) LOG_INFO << QString::asprintf(__VA_ARGS__);
#define LOG_WARN_FMT(...) LOG_WARN << QString::asprintf(__VA_ARGS__);
#define LOG_ERROR_FMT(...) LOG_ERROR << QString::asprintf(__VA_ARGS__);

#endif // LOG_UTILS_H