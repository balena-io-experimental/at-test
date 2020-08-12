#include <stdio.h>  
#include <ctype.h>
#include <stdlib.h> 
#ifndef __QUECTEL_QLOG_H
#define __QUECTEL_QLOG_H
#include <string.h>   
#include <unistd.h>   
#include <fcntl.h>   
#include <errno.h>   
#include <termios.h> 
#include <time.h>  
#include <signal.h>
#include <sys/time.h> 
#include <sys/stat.h>
#include <sys/socket.h>
#include <poll.h>
#include <netinet/in.h>
#include <pthread.h>
#include <dirent.h>
#include <sys/ioctl.h>
#include <linux/version.h>
#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 20) 
#include <linux/usb/ch9.h>
#else
#include <linux/usb_ch9.h>
#endif
#include <linux/usbdevice_fs.h>

typedef unsigned int uint32_t;
#define TFTP_F "tftp:"

typedef struct {
    int (*init_filter)(int fd, const char *conf);
    int (*logfile_create)(const char *logfile_dir, const char *logfile_suffix, unsigned logfile_seq);
    int (*logfile_init)(int logfd, unsigned logfile_seq);
    size_t (*logfile_save)(int logfd, const void *buf, size_t size);
    int (*logfile_close)(int logfd);
} qlog_ops_t;

extern qlog_ops_t mdm_qlog_ops;
extern qlog_ops_t asr_qlog_ops;
extern qlog_ops_t tty2tcp_qlog_ops;
extern int g_is_asr_chip;
extern int g_is_usb_disconnect;
extern int g_mdm_use_cfg2;
extern int tty2tcp_sockfd;
extern unsigned g_rx_log_count;
extern unsigned qlog_exit_requested;
extern ssize_t asr_send_cmd(int fd, const unsigned char *buf, size_t size);
extern ssize_t mdm_send_cmd(int fd, const unsigned char *buf, size_t size);

extern uint32_t qlog_le32 (uint32_t v32);
extern uint64_t qlog_le64 (uint64_t v64);
extern ssize_t qlog_poll_write(int fd, const void *buf, size_t size, unsigned timeout_mesc);
extern ssize_t qlog_poll_read(int fd,  void *pbuf, size_t size, unsigned timeout_msec);

extern unsigned qlog_msecs(void);
#define qlog_raw_log(fmt, arg... ) do { unsigned msec = qlog_msecs(); printf("\r[%03d.%03d] " fmt,  msec/1000, msec%1000, ## arg); fflush(stdout);} while (0)
#define qlog_dbg(fmt, arg... ) do { unsigned msec = qlog_msecs(); printf("[%03d.%03d] " fmt,  msec/1000, msec%1000, ## arg); fflush(stdout);} while (0)
 int sahara_catch_dump(int port_fd, const char *path_to_save_files, int do_reset);
#endif
