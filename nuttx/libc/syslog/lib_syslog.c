/****************************************************************************
 * libc/syslog/lib_syslog.c
 *
 *   Copyright (C) 2007-2009, 2011-2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <syslog.h>

#include <nuttx/clock.h>
#include <nuttx/streams.h>

#include "syslog/syslog.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Global Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Global Constant Data
 ****************************************************************************/

/****************************************************************************
 * Global Variables
 ****************************************************************************/

/****************************************************************************
 * Private Constant Data
 ****************************************************************************/
#if ( defined(CONFIG_SYSLOG_PRIORITY) || defined(CONFIG_SYSLOG_COLOR) )
#   define PRIORITY_NBR 8
#endif

#if ( defined(CONFIG_SYSLOG_PRIORITY) || defined(CONFIG_SYSLOG_COLOR) )
static const char const * priority_str_tb[PRIORITY_NBR] =
  {
    "EME",
    "ALE",
    "CRI",
    "ERR",
    "WAR",
    "NOT",
    "INF",
    "DEB"
  };
#endif

#if defined(CONFIG_SYSLOG_COLOR)

/* ANSI string code for text color */ 

#define TEXT_BLACK      "\033[30m"
#define TEXT_RED        "\033[31m"
#define TEXT_GREEN      "\033[32m"
#define TEXT_YELLOW     "\033[33m"
#define TEXT_BLUE       "\033[34m"
#define TEXT_MAGENTA    "\033[35m"
#define TEXT_CYAN       "\033[36m"
#define TEXT_WHITE      "\033[37m"
#define TEXT_DEFAULT    "\033[39m"


/* ANSI string code for background color */

#define BACK_BLACK      "\033[40m"
#define BACK_RED        "\033[41m"
#define BACK_GREEN      "\033[42m"
#define BACK_YELLOW     "\033[43m"
#define BACK_BLUE       "\033[44m"
#define BACK_MAGENTA    "\033[45m"
#define BACK_CYAN       "\033[46m"
#define BACK_WHITE      "\033[47m"
#define BACK_DEFAULT    "\033[49m"

static const char const * priority_color_tb[PRIORITY_NBR] =
  {

    /* EMERG Text Color */

#   if   defined(CONFIG_SYSLOG_COLOR_EMERG_TEXT_BLACK)
    TEXT_BLACK
#   elif defined(CONFIG_SYSLOG_COLOR_EMERG_TEXT_RED    )
    TEXT_RED
#   elif defined(CONFIG_SYSLOG_COLOR_EMERG_TEXT_GREEN  )
    TEXT_GREEN
#   elif defined(CONFIG_SYSLOG_COLOR_EMERG_TEXT_YELLOW )
    TEXT_YELLOW
#   elif defined(CONFIG_SYSLOG_COLOR_EMERG_TEXT_BLUE   )
    TEXT_BLUE
#   elif defined(CONFIG_SYSLOG_COLOR_EMERG_TEXT_MAGENTA)
    TEXT_MAGENTA
#   elif defined(CONFIG_SYSLOG_COLOR_EMERG_TEXT_CYAN   )
    TEXT_CYAN
#   elif defined(CONFIG_SYSLOG_COLOR_EMERG_TEXT_WHITE  )
    TEXT_WHITE
#   elif defined(CONFIG_SYSLOG_COLOR_EMERG_TEXT_DEFAULT)
    TEXT_DEFAULT
#   else
    ""
#   endif

    /* EMERG Background Color */

#   if   defined(CONFIG_SYSLOG_COLOR_EMERG_BACK_BLACK)
    BACK_BLACK
#   elif defined(CONFIG_SYSLOG_COLOR_EMERG_BACK_RED    )
    BACK_RED
#   elif defined(CONFIG_SYSLOG_COLOR_EMERG_BACK_GREEN  )
    BACK_GREEN
#   elif defined(CONFIG_SYSLOG_COLOR_EMERG_BACK_YELLOW )
    BACK_YELLOW
#   elif defined(CONFIG_SYSLOG_COLOR_EMERG_BACK_BLUE   )
    BACK_BLUE
#   elif defined(CONFIG_SYSLOG_COLOR_EMERG_BACK_MAGENTA)
    BACK_MAGENTA
#   elif defined(CONFIG_SYSLOG_COLOR_EMERG_BACK_CYAN   )
    BACK_CYAN
#   elif defined(CONFIG_SYSLOG_COLOR_EMERG_BACK_WHITE  )
    BACK_WHITE
#   elif defined(CONFIG_SYSLOG_COLOR_EMERG_BACK_DEFAULT)
    BACK_DEFAULT
#   else
    ""
#   endif
    ,

    /* ALERT Text Color */

#   if   defined(CONFIG_SYSLOG_COLOR_ALERT_TEXT_BLACK)
    TEXT_BLACK
#   elif defined(CONFIG_SYSLOG_COLOR_ALERT_TEXT_RED    )
    TEXT_RED
#   elif defined(CONFIG_SYSLOG_COLOR_ALERT_TEXT_GREEN  )
    TEXT_GREEN
#   elif defined(CONFIG_SYSLOG_COLOR_ALERT_TEXT_YELLOW )
    TEXT_YELLOW
#   elif defined(CONFIG_SYSLOG_COLOR_ALERT_TEXT_BLUE   )
    TEXT_BLUE
#   elif defined(CONFIG_SYSLOG_COLOR_ALERT_TEXT_MAGENTA)
    TEXT_MAGENTA
#   elif defined(CONFIG_SYSLOG_COLOR_ALERT_TEXT_CYAN   )
    TEXT_CYAN
#   elif defined(CONFIG_SYSLOG_COLOR_ALERT_TEXT_WHITE  )
    TEXT_WHITE
#   elif defined(CONFIG_SYSLOG_COLOR_ALERT_TEXT_DEFAULT)
    TEXT_DEFAULT
#   else
    ""
#   endif

    /* ALERT Background Color */

#   if   defined(CONFIG_SYSLOG_COLOR_ALERT_BACK_BLACK)
    BACK_BLACK
#   elif defined(CONFIG_SYSLOG_COLOR_ALERT_BACK_RED    )
    BACK_RED
#   elif defined(CONFIG_SYSLOG_COLOR_ALERT_BACK_GREEN  )
    BACK_GREEN
#   elif defined(CONFIG_SYSLOG_COLOR_ALERT_BACK_YELLOW )
    BACK_YELLOW
#   elif defined(CONFIG_SYSLOG_COLOR_ALERT_BACK_BLUE   )
    BACK_BLUE
#   elif defined(CONFIG_SYSLOG_COLOR_ALERT_BACK_MAGENTA)
    BACK_MAGENTA
#   elif defined(CONFIG_SYSLOG_COLOR_ALERT_BACK_CYAN   )
    BACK_CYAN
#   elif defined(CONFIG_SYSLOG_COLOR_ALERT_BACK_WHITE  )
    BACK_WHITE
#   elif defined(CONFIG_SYSLOG_COLOR_ALERT_BACK_DEFAULT)
    BACK_DEFAULT
#   else
    ""
#   endif
    ,

    /* CRITICAL Text Color */

#   if   defined(CONFIG_SYSLOG_COLOR_CRITICAL_TEXT_BLACK)
    TEXT_BLACK
#   elif defined(CONFIG_SYSLOG_COLOR_CRITICAL_TEXT_RED    )
    TEXT_RED
#   elif defined(CONFIG_SYSLOG_COLOR_CRITICAL_TEXT_GREEN  )
    TEXT_GREEN
#   elif defined(CONFIG_SYSLOG_COLOR_CRITICAL_TEXT_YELLOW )
    TEXT_YELLOW
#   elif defined(CONFIG_SYSLOG_COLOR_CRITICAL_TEXT_BLUE   )
    TEXT_BLUE
#   elif defined(CONFIG_SYSLOG_COLOR_CRITICAL_TEXT_MAGENTA)
    TEXT_MAGENTA
#   elif defined(CONFIG_SYSLOG_COLOR_CRITICAL_TEXT_CYAN   )
    TEXT_CYAN
#   elif defined(CONFIG_SYSLOG_COLOR_CRITICAL_TEXT_WHITE  )
    TEXT_WHITE
#   elif defined(CONFIG_SYSLOG_COLOR_CRITICAL_TEXT_DEFAULT)
    TEXT_DEFAULT
#   else
    ""
#   endif

    /* CRITICAL Background Color */

#   if   defined(CONFIG_SYSLOG_COLOR_CRITICAL_BACK_BLACK)
    BACK_BLACK
#   elif defined(CONFIG_SYSLOG_COLOR_CRITICAL_BACK_RED    )
    BACK_RED
#   elif defined(CONFIG_SYSLOG_COLOR_CRITICAL_BACK_GREEN  )
    BACK_GREEN
#   elif defined(CONFIG_SYSLOG_COLOR_CRITICAL_BACK_YELLOW )
    BACK_YELLOW
#   elif defined(CONFIG_SYSLOG_COLOR_CRITICAL_BACK_BLUE   )
    BACK_BLUE
#   elif defined(CONFIG_SYSLOG_COLOR_CRITICAL_BACK_MAGENTA)
    BACK_MAGENTA
#   elif defined(CONFIG_SYSLOG_COLOR_CRITICAL_BACK_CYAN   )
    BACK_CYAN
#   elif defined(CONFIG_SYSLOG_COLOR_CRITICAL_BACK_WHITE  )
    BACK_WHITE
#   elif defined(CONFIG_SYSLOG_COLOR_CRITICAL_BACK_DEFAULT)
    BACK_DEFAULT
#   else
    ""
#   endif
    ,

    /* ERROR Text Color */

#   if   defined(CONFIG_SYSLOG_COLOR_ERROR_TEXT_BLACK)
    TEXT_BLACK
#   elif defined(CONFIG_SYSLOG_COLOR_ERROR_TEXT_RED    )
    TEXT_RED
#   elif defined(CONFIG_SYSLOG_COLOR_ERROR_TEXT_GREEN  )
    TEXT_GREEN
#   elif defined(CONFIG_SYSLOG_COLOR_ERROR_TEXT_YELLOW )
    TEXT_YELLOW
#   elif defined(CONFIG_SYSLOG_COLOR_ERROR_TEXT_BLUE   )
    TEXT_BLUE
#   elif defined(CONFIG_SYSLOG_COLOR_ERROR_TEXT_MAGENTA)
    TEXT_MAGENTA
#   elif defined(CONFIG_SYSLOG_COLOR_ERROR_TEXT_CYAN   )
    TEXT_CYAN
#   elif defined(CONFIG_SYSLOG_COLOR_ERROR_TEXT_WHITE  )
    TEXT_WHITE
#   elif defined(CONFIG_SYSLOG_COLOR_ERROR_TEXT_DEFAULT)
    TEXT_DEFAULT
#   else
    ""
#   endif

    /* ERROR Background Color */

#   if   defined(CONFIG_SYSLOG_COLOR_ERROR_BACK_BLACK)
    BACK_BLACK
#   elif defined(CONFIG_SYSLOG_COLOR_ERROR_BACK_RED    )
    BACK_RED
#   elif defined(CONFIG_SYSLOG_COLOR_ERROR_BACK_GREEN  )
    BACK_GREEN
#   elif defined(CONFIG_SYSLOG_COLOR_ERROR_BACK_YELLOW )
    BACK_YELLOW
#   elif defined(CONFIG_SYSLOG_COLOR_ERROR_BACK_BLUE   )
    BACK_BLUE
#   elif defined(CONFIG_SYSLOG_COLOR_ERROR_BACK_MAGENTA)
    BACK_MAGENTA
#   elif defined(CONFIG_SYSLOG_COLOR_ERROR_BACK_CYAN   )
    BACK_CYAN
#   elif defined(CONFIG_SYSLOG_COLOR_ERROR_BACK_WHITE  )
    BACK_WHITE
#   elif defined(CONFIG_SYSLOG_COLOR_ERROR_BACK_DEFAULT)
    BACK_DEFAULT
#   else
    ""
#   endif
    ,

    /* WARNING Text Color */

#   if   defined(CONFIG_SYSLOG_COLOR_WARNING_TEXT_BLACK)
    TEXT_BLACK
#   elif defined(CONFIG_SYSLOG_COLOR_WARNING_TEXT_RED    )
    TEXT_RED
#   elif defined(CONFIG_SYSLOG_COLOR_WARNING_TEXT_GREEN  )
    TEXT_GREEN
#   elif defined(CONFIG_SYSLOG_COLOR_WARNING_TEXT_YELLOW )
    TEXT_YELLOW
#   elif defined(CONFIG_SYSLOG_COLOR_WARNING_TEXT_BLUE   )
    TEXT_BLUE
#   elif defined(CONFIG_SYSLOG_COLOR_WARNING_TEXT_MAGENTA)
    TEXT_MAGENTA
#   elif defined(CONFIG_SYSLOG_COLOR_WARNING_TEXT_CYAN   )
    TEXT_CYAN
#   elif defined(CONFIG_SYSLOG_COLOR_WARNING_TEXT_WHITE  )
    TEXT_WHITE
#   elif defined(CONFIG_SYSLOG_COLOR_WARNING_TEXT_DEFAULT)
    TEXT_DEFAULT
#   else
    ""
#   endif

    /* WARNING Background Color */

#   if   defined(CONFIG_SYSLOG_COLOR_WARNING_BACK_BLACK)
    BACK_BLACK
#   elif defined(CONFIG_SYSLOG_COLOR_WARNING_BACK_RED    )
    BACK_RED
#   elif defined(CONFIG_SYSLOG_COLOR_WARNING_BACK_GREEN  )
    BACK_GREEN
#   elif defined(CONFIG_SYSLOG_COLOR_WARNING_BACK_YELLOW )
    BACK_YELLOW
#   elif defined(CONFIG_SYSLOG_COLOR_WARNING_BACK_BLUE   )
    BACK_BLUE
#   elif defined(CONFIG_SYSLOG_COLOR_WARNING_BACK_MAGENTA)
    BACK_MAGENTA
#   elif defined(CONFIG_SYSLOG_COLOR_WARNING_BACK_CYAN   )
    BACK_CYAN
#   elif defined(CONFIG_SYSLOG_COLOR_WARNING_BACK_WHITE  )
    BACK_WHITE
#   elif defined(CONFIG_SYSLOG_COLOR_WARNING_BACK_DEFAULT)
    BACK_DEFAULT
#   else
    ""
#   endif
    ,

    /* NOTICE Text Color */

#   if   defined(CONFIG_SYSLOG_COLOR_NOTICE_TEXT_BLACK)
    TEXT_BLACK
#   elif defined(CONFIG_SYSLOG_COLOR_NOTICE_TEXT_RED    )
    TEXT_RED
#   elif defined(CONFIG_SYSLOG_COLOR_NOTICE_TEXT_GREEN  )
    TEXT_GREEN
#   elif defined(CONFIG_SYSLOG_COLOR_NOTICE_TEXT_YELLOW )
    TEXT_YELLOW
#   elif defined(CONFIG_SYSLOG_COLOR_NOTICE_TEXT_BLUE   )
    TEXT_BLUE
#   elif defined(CONFIG_SYSLOG_COLOR_NOTICE_TEXT_MAGENTA)
    TEXT_MAGENTA
#   elif defined(CONFIG_SYSLOG_COLOR_NOTICE_TEXT_CYAN   )
    TEXT_CYAN
#   elif defined(CONFIG_SYSLOG_COLOR_NOTICE_TEXT_WHITE  )
    TEXT_WHITE
#   elif defined(CONFIG_SYSLOG_COLOR_NOTICE_TEXT_DEFAULT)
    TEXT_DEFAULT
#   else
    ""
#   endif

    /* NOTICE Background Color */

#   if   defined(CONFIG_SYSLOG_COLOR_NOTICE_BACK_BLACK)
    BACK_BLACK
#   elif defined(CONFIG_SYSLOG_COLOR_NOTICE_BACK_RED    )
    BACK_RED
#   elif defined(CONFIG_SYSLOG_COLOR_NOTICE_BACK_GREEN  )
    BACK_GREEN
#   elif defined(CONFIG_SYSLOG_COLOR_NOTICE_BACK_YELLOW )
    BACK_YELLOW
#   elif defined(CONFIG_SYSLOG_COLOR_NOTICE_BACK_BLUE   )
    BACK_BLUE
#   elif defined(CONFIG_SYSLOG_COLOR_NOTICE_BACK_MAGENTA)
    BACK_MAGENTA
#   elif defined(CONFIG_SYSLOG_COLOR_NOTICE_BACK_CYAN   )
    BACK_CYAN
#   elif defined(CONFIG_SYSLOG_COLOR_NOTICE_BACK_WHITE  )
    BACK_WHITE
#   elif defined(CONFIG_SYSLOG_COLOR_NOTICE_BACK_DEFAULT)
    BACK_DEFAULT
#   else
    ""
#   endif
    ,

    /* INFO Text Color */

#   if   defined(CONFIG_SYSLOG_COLOR_INFO_TEXT_BLACK)
    TEXT_BLACK
#   elif defined(CONFIG_SYSLOG_COLOR_INFO_TEXT_RED    )
    TEXT_RED
#   elif defined(CONFIG_SYSLOG_COLOR_INFO_TEXT_GREEN  )
    TEXT_GREEN
#   elif defined(CONFIG_SYSLOG_COLOR_INFO_TEXT_YELLOW )
    TEXT_YELLOW
#   elif defined(CONFIG_SYSLOG_COLOR_INFO_TEXT_BLUE   )
    TEXT_BLUE
#   elif defined(CONFIG_SYSLOG_COLOR_INFO_TEXT_MAGENTA)
    TEXT_MAGENTA
#   elif defined(CONFIG_SYSLOG_COLOR_INFO_TEXT_CYAN   )
    TEXT_CYAN
#   elif defined(CONFIG_SYSLOG_COLOR_INFO_TEXT_WHITE  )
    TEXT_WHITE
#   elif defined(CONFIG_SYSLOG_COLOR_INFO_TEXT_DEFAULT)
    TEXT_DEFAULT
#   else
    ""
#   endif

    /* INFO Background Color */

#   if   defined(CONFIG_SYSLOG_COLOR_INFO_BACK_BLACK)
    BACK_BLACK
#   elif defined(CONFIG_SYSLOG_COLOR_INFO_BACK_RED    )
    BACK_RED
#   elif defined(CONFIG_SYSLOG_COLOR_INFO_BACK_GREEN  )
    BACK_GREEN
#   elif defined(CONFIG_SYSLOG_COLOR_INFO_BACK_YELLOW )
    BACK_YELLOW
#   elif defined(CONFIG_SYSLOG_COLOR_INFO_BACK_BLUE   )
    BACK_BLUE
#   elif defined(CONFIG_SYSLOG_COLOR_INFO_BACK_MAGENTA)
    BACK_MAGENTA
#   elif defined(CONFIG_SYSLOG_COLOR_INFO_BACK_CYAN   )
    BACK_CYAN
#   elif defined(CONFIG_SYSLOG_COLOR_INFO_BACK_WHITE  )
    BACK_WHITE
#   elif defined(CONFIG_SYSLOG_COLOR_INFO_BACK_DEFAULT)
    BACK_DEFAULT
#   else
    ""
#   endif
    ,

    /* DEBUG Text Color */

#   if   defined(CONFIG_SYSLOG_COLOR_DEBUG_TEXT_BLACK)
    TEXT_BLACK
#   elif defined(CONFIG_SYSLOG_COLOR_DEBUG_TEXT_RED    )
    TEXT_RED
#   elif defined(CONFIG_SYSLOG_COLOR_DEBUG_TEXT_GREEN  )
    TEXT_GREEN
#   elif defined(CONFIG_SYSLOG_COLOR_DEBUG_TEXT_YELLOW )
    TEXT_YELLOW
#   elif defined(CONFIG_SYSLOG_COLOR_DEBUG_TEXT_BLUE   )
    TEXT_BLUE
#   elif defined(CONFIG_SYSLOG_COLOR_DEBUG_TEXT_MAGENTA)
    TEXT_MAGENTA
#   elif defined(CONFIG_SYSLOG_COLOR_DEBUG_TEXT_CYAN   )
    TEXT_CYAN
#   elif defined(CONFIG_SYSLOG_COLOR_DEBUG_TEXT_WHITE  )
    TEXT_WHITE
#   elif defined(CONFIG_SYSLOG_COLOR_DEBUG_TEXT_DEFAULT)
    TEXT_DEFAULT
#   else
    ""
#   endif

    /* DEBUG Background Color */

#   if   defined(CONFIG_SYSLOG_COLOR_DEBUG_BACK_BLACK)
    BACK_BLACK
#   elif defined(CONFIG_SYSLOG_COLOR_DEBUG_BACK_RED    )
    BACK_RED
#   elif defined(CONFIG_SYSLOG_COLOR_DEBUG_BACK_GREEN  )
    BACK_GREEN
#   elif defined(CONFIG_SYSLOG_COLOR_DEBUG_BACK_YELLOW )
    BACK_YELLOW
#   elif defined(CONFIG_SYSLOG_COLOR_DEBUG_BACK_BLUE   )
    BACK_BLUE
#   elif defined(CONFIG_SYSLOG_COLOR_DEBUG_BACK_MAGENTA)
    BACK_MAGENTA
#   elif defined(CONFIG_SYSLOG_COLOR_DEBUG_BACK_CYAN   )
    BACK_CYAN
#   elif defined(CONFIG_SYSLOG_COLOR_DEBUG_BACK_WHITE  )
    BACK_WHITE
#   elif defined(CONFIG_SYSLOG_COLOR_DEBUG_BACK_DEFAULT)
    BACK_DEFAULT
#   else
    ""
#   endif
  };
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vsyslog_internal
 *
 * Description:
 *   This is the internal implementation of vsyslog (see the description of
 *   syslog and vsyslog below)
 *
 ****************************************************************************/

static inline int vsyslog_internal(int priority, FAR const char *fmt, va_list ap)
{
#if defined(CONFIG_SYSLOG)
  struct lib_outstream_s stream;
#elif CONFIG_NFILE_DESCRIPTORS > 0
  struct lib_rawoutstream_s stream;
#elif defined(CONFIG_ARCH_LOWPUTC)
  struct lib_outstream_s stream;
#endif
#if defined(CONFIG_SYSLOG_PRIORITY)
  const char const *priority_str = "UNK";
#endif
#if defined(CONFIG_SYSLOG_COLOR)
  const char const *priority_color = "";
#endif

#if defined(CONFIG_SYSLOG_TIMESTAMP)
  struct timespec ts;
  int ret;

  /* Get the current time */

  ret = clock_systimespec(&ts);
#endif

#if ( defined(CONFIG_SYSLOG_PRIORITY) || defined(CONFIG_SYSLOG_PRIORITY) )
  if ( ( priority >= 0 ) && ( priority < PRIORITY_NBR ) )
    {
#   if defined(CONFIG_SYSLOG_PRIORITY) 
      priority_str = priority_str_tb[priority];
#   endif
#   if defined(CONFIG_SYSLOG_COLOR)
      priority_color = priority_color_tb[priority];
#   endif
    }
#endif

#if defined(CONFIG_SYSLOG)
  /* Wrap the low-level output in a stream object and let lib_vsprintf
   * do the work.
   */

  lib_syslogstream((FAR struct lib_outstream_s *)&stream);

#if defined(CONFIG_SYSLOG_COLOR)
  /* Pre-pend the message with the respective ANSI color */

  (void)lib_sprintf((FAR struct lib_outstream_s *)&stream,priority_color);
#endif

#if defined(CONFIG_SYSLOG_PRIORITY)
  /* Pre-pend the message with the current time */

  (void)lib_sprintf((FAR struct lib_outstream_s *)&stream,priority_str);
#endif

#if defined(CONFIG_SYSLOG_TIMESTAMP)
  /* Pre-pend the message with the current time */

  if (ret == OK)
    {
      (void)lib_sprintf((FAR struct lib_outstream_s *)&stream,
                        "[%6d.%06d]",
                         ts.tv_sec, ts.tv_nsec/1000);
    }
#endif

  return lib_vsprintf((FAR struct lib_outstream_s *)&stream, fmt, ap);

#elif CONFIG_NFILE_DESCRIPTORS > 0
  /* Wrap the stdout in a stream object and let lib_vsprintf
   * do the work.
   */

  lib_rawoutstream(&stream, 1);

#if defined(CONFIG_SYSLOG_COLOR)
  /* Pre-pend the message with the respective ANSI color */

  (void)lib_sprintf((FAR struct lib_outstream_s *)&stream,priority_color);
#endif

#if defined(CONFIG_SYSLOG_PRIORITY)
  /* Pre-pend the message with the current time */

  (void)lib_sprintf((FAR struct lib_outstream_s *)&stream,priority_str);
#endif
  
#if defined(CONFIG_SYSLOG_TIMESTAMP)
  /* Pre-pend the message with the current time */

  if (ret == OK)
    {
      (void)lib_sprintf((FAR struct lib_outstream_s *)&stream,
                        "[%6d.%06d]",
                         ts.tv_sec, ts.tv_nsec/1000);
    }
#endif

  return lib_vsprintf(&stream.public, fmt, ap);

#elif defined(CONFIG_ARCH_LOWPUTC)
  /* Wrap the low-level output in a stream object and let lib_vsprintf
   * do the work.
   */

  lib_lowoutstream((FAR struct lib_outstream_s *)&stream);

#if defined(CONFIG_SYSLOG_COLOR)
  /* Pre-pend the message with the respective ANSI color */

  (void)lib_sprintf((FAR struct lib_outstream_s *)&stream,priority_color);
#endif

#if defined(CONFIG_SYSLOG_PRIORITY)
  /* Pre-pend the message with the current time */

  (void)lib_sprintf((FAR struct lib_outstream_s *)&stream,priority_str);
#endif

#if defined(CONFIG_SYSLOG_TIMESTAMP)
  /* Pre-pend the message with the current time */

  if (ret == OK)
    {
      (void)lib_sprintf((FAR struct lib_outstream_s *)&stream,
                        "[%6d.%06d]",
                         ts.tv_sec, ts.tv_nsec/1000);
    }
#endif

  return lib_vsprintf((FAR struct lib_outstream_s *)&stream, fmt, ap);

#else /* CONFIG_SYSLOG */
  return 0;
#endif /* CONFIG_SYSLOG */
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vsyslog
 *
 * Description:
 *   The function vsyslog() performs the same task as syslog() with the
 *   difference that it takes a set of arguments which have been obtained
 *   using the stdarg variable argument list macros.
 *
 ****************************************************************************/

int vsyslog(int priority, FAR const char *fmt, va_list ap)
{
  int ret = 0;

  /* Check if this priority is enabled */

  if ((g_syslog_mask & LOG_MASK(priority)) != 0)
    {
      /* Yes.. let vsylog_internal do the deed */

      ret = vsyslog_internal(priority, fmt, ap);
    }

  return ret;
}

/****************************************************************************
 * Name: syslog
 *
 * Description:
 *   syslog() generates a log message. The priority argument is formed by
 *   ORing the facility and the level values (see include/syslog.h). The
 *   remaining arguments are a format, as in printf and any arguments to the
 *   format.
 *
 *   The NuttX implementation does not support any special formatting
 *   characters beyond those supported by printf.
 *
 ****************************************************************************/

int syslog(int priority, FAR const char *fmt, ...)
{
  va_list ap;
  int ret;

  /* Let vsyslog do the work */

  va_start(ap, fmt);
  ret = vsyslog(priority, fmt, ap);
  va_end(ap);

  return ret;
}
