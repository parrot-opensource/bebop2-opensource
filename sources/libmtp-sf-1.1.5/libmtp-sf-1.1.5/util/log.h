/*
 * file: log.h
 */

#ifndef _LOG_H_
#define _LOG_H_

#ifdef ANDROID
#include <cutils/logd.h>
#define LOG_INFO ANDROID_LOG_INFO
#define logger(_lvl, _fmt, args...) __android_log_print(_lvl, "mtpprobe", _fmt , ##args)
#else
#include <syslog.h>
#define logger(_lvl, _fmt, args...)  syslog(_lvl, _fmt, ##args)
#endif

#endif /* _LOG_H_ */
