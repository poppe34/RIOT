/*
 * Copyright (C) 2008, 2009, 2010  Kaspar Schleiser <kaspar@schleiser.de>
 * Copyright (C) 2013 INRIA
 * Copyright (C) 2013 Ludwig Ortmann <ludwig.ortmann@fu-berlin.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     examples
 * @{
 *
 * @file
 * @brief       Default application that shows a lot of functionality of RIOT
 *
 * @author      Kaspar Schleiser <kaspar@schleiser.de>
 * @author      Oliver Hahm <oliver.hahm@inria.fr>
 * @author      Ludwig Ortmann <ludwig.ortmann@fu-berlin.de>
 *
 * @}
 */

#include <stdio.h>
#include <string.h>

#include "thread.h"
#include "shell.h"
#include "shell_commands.h"

#if FEATURE_PERIPH_RTC
#include "periph/rtc.h"
#endif

#ifdef MODULE_LTC4150
#include "ltc4150.h"
#endif


#include "net/gnrc/pktdump.h"
#include "net/gnrc.h"
#include "ksz8851snl.h"

#define MSG_QUEUE_SIZE 1
msg_t msgq[MSG_QUEUE_SIZE];


int main(void)
{
#ifdef MODULE_LTC4150
    ltc4150_start();
#endif

#ifdef FEATURE_PERIPH_RTC
    rtc_init();
#endif


//    gnrc_netreg_entry_t dump;

//    dump.pid = gnrc_pktdump_getpid();
//    dump.demux_ctx = GNRC_NETREG_DEMUX_CTX_ALL;
//    gnrc_netreg_register(GNRC_NETTYPE_UNDEF, &dump);

    msg_init_queue(msgq, MSG_QUEUE_SIZE);

    (void) puts("Welcome to RIOT!");

    temp_set_mac();
    char line_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(NULL, line_buf, SHELL_DEFAULT_BUFSIZE);

    return 0;
}