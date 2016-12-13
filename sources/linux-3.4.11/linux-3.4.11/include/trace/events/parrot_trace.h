#undef TRACE_SYSTEM
#define TRACE_SYSTEM parrot_trace

#if !defined(_PARROT_TRACE_H) || defined(TRACE_HEADER_MULTI_READ)
#define _PARROT_TRACE_H
#include <linux/tracepoint.h>

TRACE_EVENT(user_kevent_start,
	
        TP_PROTO(int evt),

	TP_ARGS(evt),

	TP_STRUCT__entry(
		__field(	int,	event_start)
	),

	TP_fast_assign(
		__entry->event_start = evt;
	),

	TP_printk("start = %d", __entry->event_start)
)

TRACE_EVENT(user_kevent_stop,
	
        TP_PROTO(int evt),

	TP_ARGS(evt),

	TP_STRUCT__entry(
		__field(	int,	event_stop)
	),

	TP_fast_assign(
		__entry->event_stop = evt;
	),

	TP_printk("stop = %d", __entry->event_stop)
)

TRACE_EVENT(user_kmessage,

	TP_PROTO(char *str),

	TP_ARGS(str),

	TP_STRUCT__entry(
		__string(	message,	str)
	),

	TP_fast_assign(
		__assign_str(message, str);
	),

	TP_printk("message = %s", __get_str(message))
)

#ifdef CREATE_TRACE_POINTS
EXPORT_TRACEPOINT_SYMBOL(user_kevent_start);
EXPORT_TRACEPOINT_SYMBOL(user_kevent_stop);
EXPORT_TRACEPOINT_SYMBOL(user_kmessage);
#endif

#endif

#include <trace/define_trace.h>
