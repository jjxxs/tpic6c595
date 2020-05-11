#undef TRACE_SYSTEM
#define TRACE_SYSTEM tpic6c595

#if !defined(_TPIC6C595_TRACE_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TPIC6C595_TRACE_H

#include <linux/tracepoint.h>

TRACE_EVENT(me_silly,

            TP_PROTO(unsigned long time, unsigned long count),

            TP_ARGS(time, count),

            TP_STRUCT__entry(
                    __field(	unsigned long,	time	)
                    __field(	unsigned long,	count	)
            ),

            TP_fast_assign(
                    __entry->time = jiffies;
                    __entry->count = count;
            ),

            TP_printk("time=%lu count=%lu", __entry->time, __entry->count)
);

#endif /* _TPIC6C595_TRACE_H */

/* This part must be outside protection */
#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .
#define TRACE_INCLUDE_FILE tpic6c595-trace
#include <trace/define_trace.h>