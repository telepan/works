#ifndef LOG_UTILS_H
#define LOG_UTILS_H

enum class ELogLevel
{
    None = -1,
    Debug = 0,
    Info,
    Warning,
    Error,
};

extern bool checkLogLevel(ELogLevel level);

#define LOG_DEBUG_FMT(...)                                                                    \
    if (checkLogLevel(ELogLevel::Debug))                                                      \
    {                                                                                         \
        LogManager::instance().writeLog(QString("DEBUG: ") + QString::asprintf(__VA_ARGS__)); \
    }

#define LOG_INFO_FMT(...)                                                                    \
    if (checkLogLevel(ELogLevel::Info))                                                      \
    {                                                                                        \
        LogManager::instance().writeLog(QString("INFO: ") + QString::asprintf(__VA_ARGS__)); \
    }

#define LOG_WARN_FMT(...)                                                                       \
    if (checkLogLevel(ELogLevel::Warning))                                                      \
    {                                                                                           \
        LogManager::instance().writeLog(QString("WARNING: ") + QString::asprintf(__VA_ARGS__)); \
    }

#define LOG_ERROR_FMT(...)                                                                    \
    if (checkLogLevel(ELogLevel::Error))                                                      \
    {                                                                                         \
        LogManager::instance().writeLog(QString("ERROR: ") + QString::asprintf(__VA_ARGS__)); \
    }

// New, efficient, and type-safe stream-based logging macros.

// Usage: LOG_DEBUG << "Message" << 123;

#define LOG_DEBUG                         \
    if (!checkLogLevel(ELogLevel::Debug)) \
        (void)0;                          \
    else                                  \
        qDebug()

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

#endif // LOG_UTILS_H