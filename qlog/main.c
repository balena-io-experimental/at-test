/******************************************************************************
  @file    ql-tty2tcp.c
  @brief   enter point.

  DESCRIPTION
  QLog Tool for USB and PCIE of Quectel wireless cellular modules.

  INITIALIZATION AND SEQUENCING REQUIREMENTS
  None.

  ---------------------------------------------------------------------------
  Copyright (c) 2016 - 2020 Quectel Wireless Solution, Co., Ltd.  All Rights Reserved.
  Quectel Wireless Solution Proprietary and Confidential.
  ---------------------------------------------------------------------------
******************************************************************************/
#include "qlog.h"
#include "getopt.h"
#include <sys/statfs.h>

#define QLOG_VERSION "1.4.4"  //when release, rename to V1.X

#define LOGFILE_SIZE_MIN (2*1024*1024)
#define LOGFILE_SIZE_MAX (512*1024*1024)
#define LOGFILE_SIZE_DEFAULT (128*1024*1024)
#define LOGFILE_NUM 512
static char s_logfile_List[LOGFILE_NUM][32];
static unsigned s_logfile_num = 0;
static unsigned s_logfile_seq;
static unsigned s_logfile_idx;
unsigned qlog_exit_requested = 0;
static unsigned exit_after_usb_disconnet = 0;
int g_mdm_use_cfg2 = 0;
unsigned g_rx_log_count = 0;
int g_is_asr_chip = 0;
int g_is_usb_disconnect = 0;
static int qlog_init_filter_finished = 0;

#define safe_close_fd(_fd) do { if (_fd != -1) { int tmpfd = _fd; _fd = -1; close(tmpfd); }} while(0)

struct ql_usb_device_info {
    int idVendor;
    int idProduct;
    int bNumInterfaces;
    int busnum;
    int devnum;
    char usbdevice_pah[255+32];
    int dm_intf;
    char ttyDM[255+32];
    int dm_ep_in; //mdm~ 0x81, asr-0x84
    int dm_ep_out; //mdm~0x01; asr-0x03
};
static struct ql_usb_device_info s_usb_device_info[8];

struct arguments
{
    // arguments
    char ttyDM[256];
    char logdir[256];

    // configurations
    int logfile_num;
    int logfile_sz;
    const char *filter_cfg;

    // profiles
    int usbfd;
    int usb_sockets[2];
    int ttyfd;
    const  struct ql_usb_device_info *ql_dev;
};

static struct arguments *qlog_args;

extern int asr_catch_dump(int ttyfd, const char *logfile_dir);

uint32_t qlog_le32 (uint32_t v32) {
    uint32_t tmp = v32;
    const int is_bigendian = 1;

    if ( (*(char*)&is_bigendian) == 0 ) {
        unsigned char *s = (unsigned char *)(&v32);
        unsigned char *d = (unsigned char *)(&tmp);
        d[0] = s[3];
        d[1] = s[2];
        d[2] = s[1];
        d[3] = s[0];
    }
    return tmp;
}

uint64_t qlog_le64(uint64_t v64) {
    const uint64_t is_bigendian = 1;
    uint64_t tmp = v64;
	
    if ((*(char*)&is_bigendian) == 0) {
        unsigned char *s = (unsigned char *)(&v64);
        unsigned char *d = (unsigned char *)(&tmp);
        d[0] = s[7];
        d[1] = s[6];
        d[2] = s[5];
        d[3] = s[4];
        d[4] = s[3];
        d[5] = s[2];
        d[6] = s[1];
        d[7] = s[0];
    }
    return tmp;
}

unsigned qlog_msecs(void) {
    static unsigned start = 0;
    unsigned now;
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    now = (unsigned)ts.tv_sec*1000 + (unsigned)(ts.tv_nsec / 1000000);
    if (start == 0)
        start = now;
    return now - start;
}

static int get_value_from_file(const char *fname, int base)
{
    char buff[64] = {'\0'};

    int fd = open(fname, O_RDONLY);
    if (fd <= 0)
    {
        if (errno != ENOENT)
            qlog_dbg("Fail to open %s,  errno: %d (%s)\n", fname, errno, strerror(errno));
        return -1;
    }
    read(fd, buff, sizeof(buff));
    close(fd);
    return strtoul(buff, NULL, base);
}

#if 0
void qlog_get_vidpid_by_ttyport(const char *ttyport, int *idVendor, int *idProduct, int *bNumInterfaces) {
    char syspath[255];
    char sysport[64];
    int count;
    char *pchar = NULL;
    int vid, pid, ifnum;

    memset(idVendor, 0x00, 5);
    memset(idProduct, 0x00, 5);
    memset(bNumInterfaces, 0x00, 5);
    
    snprintf(sysport, sizeof(sysport), "/sys/class/tty/%s", &ttyport[strlen("/dev/")]);
    count = readlink(sysport, syspath, sizeof(syspath) - 1);
    if (count < strlen(":1.0/tty"))
        return;

    // ttyUSB0 -> ../../devices/soc0/soc/2100000.aips-bus/2184200.usb/ci_hdrc.1/usb1/1-1/1-1:1.0/ttyUSB0/tty/ttyUSB0
    pchar = strstr(syspath, ":1.0/tty"); //MDM
    if (pchar == NULL)
        pchar = strstr(syspath, ":1.2/tty"); //ASR

    if (pchar == NULL) {
        qlog_dbg("%s is not a usb-to-serial device?\n", ttyport);
        return;
    }

    *pchar = '\0';
    while (*pchar != '/')
        pchar--;

    strcpy(sysport, pchar + 1);
    
    snprintf(syspath, sizeof(syspath), "/sys/bus/usb/devices/%s/idVendor", sysport);
    vid = get_value_from_file(syspath, 16);
    if (idVendor)
        *idVendor = vid;

    snprintf(syspath, sizeof(syspath), "/sys/bus/usb/devices/%s/idProduct", sysport);
    pid = get_value_from_file(syspath, 16);
    if (idProduct)
        *idProduct = pid;

    snprintf(syspath, sizeof(syspath), "/sys/bus/usb/devices/%s/bNumInterfaces", sysport);
    ifnum = get_value_from_file(syspath, 10);
    if (bNumInterfaces)
        *bNumInterfaces = ifnum;

    qlog_dbg("%s idVendor=%04x, idProduct=%04x, bNumInterfaces=%d\n", __func__, vid, pid, ifnum);
}

static int get_vidpid_by_usbfs(char *usbfs_path, int *idVendor, int *idProduct, int *bNumInterfaces)
{
    int usbfsfd = -1;
    ssize_t desclength;
    unsigned char devdesc[1024] = {0};
    int vid = 0;
    int pid = 0;
    int ifnum = 0;

    if (!usbfs_path || usbfs_path[0] == '\0')
    {
        qlog_dbg("invalid usbfs device (NULL or empty)\n");
        return -1;
    }

    usbfsfd = open(usbfs_path, O_RDWR | O_NDELAY);
    if (usbfsfd < 0)
    {
        qlog_dbg("open %s failed, error=%d(%s)\n", usbfs_path, errno, strerror(errno));
        return -1;
    }

    desclength = read(usbfsfd, devdesc, sizeof(devdesc));
    if (desclength < sizeof(struct usb_device_descriptor))
    {
        qlog_dbg("error read, descriptor length(%zd) should be sizeof(usb_device_descriptor)=%zd\n",
                 desclength, sizeof(struct usb_device_descriptor));
        return -1;
    }

    struct usb_device_descriptor *device = (struct usb_device_descriptor *)devdesc;
    if (device->bLength == sizeof(struct usb_device_descriptor) && device->bDescriptorType == USB_DT_DEVICE)
    {
        if (device->idVendor == 0x2c7c)
        {
            vid = device->idVendor;
            pid = device->idProduct;
            if (idVendor)
                *idVendor = device->idVendor;
            if (idProduct)
                *idProduct = device->idProduct;

            struct usb_config_descriptor *config = (struct usb_config_descriptor *)(devdesc + device->bLength);
            if (config->bLength == sizeof(struct usb_config_descriptor) && config->bDescriptorType == USB_DT_CONFIG)
            {
                ifnum = config->bNumInterfaces;
                if (bNumInterfaces)
                    *bNumInterfaces = config->bNumInterfaces;
            }

            qlog_dbg("usbfs node = %s, usbfsfd = %d\n", usbfs_path, usbfsfd);
            qlog_dbg("%s: idVendor=%x, idProduct=%x, bNumInterfaces=%d\n", __func__, vid, pid, ifnum);
        }
        else
        {
            qlog_dbg("usbfs node = %s, usbfsfd = %d\n", usbfs_path, usbfsfd);
            qlog_dbg("%s: idVendor=%x, idProduct=%x, bNumInterfaces=%d\n", __func__, vid, pid, ifnum);
            qlog_dbg("is that right???\n");
        }
    }
    else
    {
        qlog_dbg ("device->bLength == sizeof(struct usb_device_descriptor) && device->bDescriptorType == USB_DT_DEVICE\n");
        return -1;
    }

    return usbfsfd;
}

int qlog_get_vidpid_by_usbfs(char *usbfs_path, int *idVendor, int *idProduct, int *bNumInterfaces)
{
    if (usbfs_path && usbfs_path[0] != '\0')
    { // user provide usbfs path
        return get_vidpid_by_usbfs(usbfs_path, idVendor, idProduct, bNumInterfaces);
    }
    else
    { // auto find usbfs path
        const char *usbfs_rootdir = "/dev/bus/usb";
        DIR *rootdir = NULL;
        DIR *subdir = NULL;
        char busdir[269];
        struct dirent *entptr = NULL;

        rootdir = opendir(usbfs_rootdir);
        if (rootdir == NULL) {
            qlog_dbg("opendir %s failed, errno = %d(%s)\n", usbfs_rootdir, errno, strerror(errno));
            return -1;
        }

        while ((entptr = readdir(rootdir)) != NULL) {
            if (!strcmp(entptr->d_name, ".") || !strcmp(entptr->d_name, ".."))
                continue;

            sprintf(busdir, "%s/%s", usbfs_rootdir, entptr->d_name);
            subdir = opendir(busdir);

            while ((entptr = readdir(subdir)) != NULL)  {
                if (!strcmp(entptr->d_name, ".") || !strcmp(entptr->d_name, ".."))
                    continue;

                sprintf(usbfs_path, "%s/%s", busdir, entptr->d_name);
                int usbfsfd = get_vidpid_by_usbfs(usbfs_path, idVendor, idProduct, bNumInterfaces);
                if (usbfsfd > 0)
                    return usbfsfd;
                close(usbfsfd);
            }
            closedir(subdir);
        }
        closedir(rootdir);
        return -1;
    }
}
#endif

#define RX_URB_SIZE (16*1024) //16KB for catch mdm dump
static void* qlog_usbfs_read(void *arg)
{
    struct usbdevfs_bulktransfer bulk;
    void *pbuf;
    struct arguments *args = (struct arguments *)arg;
    int n = 0;

    pbuf = malloc(RX_URB_SIZE);
    if (pbuf == NULL) {
        qlog_dbg("%s malloc %d fail\n", __func__, RX_URB_SIZE);
        return NULL;
    }

    bulk.ep = args->ql_dev->dm_ep_in;
    bulk.len = RX_URB_SIZE;
    bulk.data = (void *)pbuf;
    bulk.timeout = 0; // keep waiting

    while (qlog_exit_requested == 0)
    {
        int nwrites = 0;
        int count = 0;

        n = ioctl(args->usbfd, USBDEVFS_BULK, &bulk);
        if (n < 0) {
            qlog_dbg("%s n = %d, errno: %d (%s)\n", __func__, n, errno, strerror(errno));
            g_is_usb_disconnect = 1;
            break;
        }
        else if (n == 0) {
            //zero length packet
        }

        if (n > 0) {
            // printf("urb nreads = %d\n", n);
            while (count < n) {
                do {
                    nwrites = write(args->usb_sockets[1], pbuf, n);
                } while (nwrites == -1 && errno == EAGAIN);

                count += nwrites;
            }
        }
    }

    free(pbuf);
    return NULL;
}

static size_t qlog_usbfs_write(struct arguments *args, const void *pbuf, size_t size)
{
    struct usbdevfs_urb bulk;
    struct usbdevfs_urb *urb = &bulk;
    int n = 0;

    memset(urb, 0, sizeof(struct usbdevfs_urb));
    urb->type = USBDEVFS_URB_TYPE_BULK;
    urb->endpoint = args->ql_dev->dm_ep_out;
    urb->status = -1;
    urb->buffer = (void *)pbuf;
    urb->buffer_length = size;
    urb->usercontext = urb;
    urb->flags = 0;

    n = ioctl(args->usbfd, USBDEVFS_SUBMITURB, urb);
    if (n < 0) {
        qlog_dbg("%s submit n = %d, errno: %d (%s)\n", __func__, n, errno, strerror(errno));
        return 0;
    }
        
    urb = NULL;
    n = ioctl(args->usbfd, USBDEVFS_REAPURB, &urb);
    if (n < 0) {
        qlog_dbg("%s reap n = %d, errno: %d (%s)\n", __func__, n, errno, strerror(errno));
        return 0;
    }

    if (urb && urb->status == 0 && urb->actual_length) {
        // qlog_dbg("urb->actual_length = %u\n", urb->actual_length);
        return urb->actual_length;
    }

    return 0;
}

ssize_t qlog_poll_read(int fd,  void *pbuf, size_t size, unsigned timeout_msec) {
    unsigned t = 500;
    size_t rc = 0;

    if (timeout_msec < 500)
        timeout_msec = 500;
    else if (timeout_msec > 20000)
        timeout_msec = 20000;

    timeout_msec = (timeout_msec+(t-1))/t*t;
    
    while(qlog_exit_requested == 0 && timeout_msec > 0)
    {
        struct pollfd pollfds[1];
        int ret = -1;
        
        pollfds[0].events = POLLIN;
        pollfds[0].fd = fd;

        ret = poll(pollfds, 1, t);

        if (g_is_usb_disconnect)
            break;

        if (ret == 0) {
            timeout_msec -= t;
            if (timeout_msec == 0) {
                rc = -1;
                errno = ETIMEDOUT;
                break;
            }
            continue;
        }        
        else if (ret < 0) {
            qlog_dbg("poll(handlefd) =%d, errno: %d (%s)\n", ret, errno, strerror(errno));
            break;
        }

        if (pollfds[0].revents & (POLLERR | POLLHUP | POLLNVAL)) {
            qlog_dbg("poll fd=%d, revents = %04x\n", pollfds[0].fd, pollfds[0].revents);
            break;
        }

        if (pollfds[0].revents & (POLLIN)) {
            rc = read(fd, pbuf, size);
            break;
        }
    }   

    return rc;
}

ssize_t qlog_poll_write(int fd, const void *buf, size_t size, unsigned timeout_msec) {
    ssize_t wc = 0;
    ssize_t nbytes;

    if (fd == qlog_args->ttyfd && fd == qlog_args->usb_sockets[0]) {
        return qlog_usbfs_write(qlog_args, buf, size);
    }

    nbytes = write(fd, buf+wc, size-wc);

    if (nbytes <= 0) {
        if (errno != EAGAIN) {
            qlog_dbg("Fail to write fd = %d, errno : %d (%s)\n", fd, errno, strerror(errno));
            goto out;
        }
        else {
            nbytes = 0;
        }
    }

    wc += nbytes;

    while (wc < size) {
        int ret;
        struct pollfd pollfds[] = {{fd, POLLOUT, 0}};

        ret = poll(pollfds, 1, timeout_msec);

        if (ret <= 0) {
            qlog_dbg("Fail to poll fd = %d, errno : %d (%s)\n", fd, errno, strerror(errno));
            break;
        }

        if (pollfds[0].revents & (POLLERR | POLLHUP | POLLNVAL)) {
            qlog_dbg("Fail to poll fd = %d, revents = %04x\n", fd, pollfds[0].revents);
            break;
        }

        if (pollfds[0].revents & (POLLOUT)) {
            nbytes = write(fd, buf+wc, size-wc);
            
            if (nbytes <= 0) {
                qlog_dbg("Fail to write fd = %d, errno : %d (%s)\n", fd, errno, strerror(errno));
                break;
            }
            wc += nbytes;
        }
    }

out:
    if (wc != size) {
        qlog_dbg("%s fd=%d, size=%zd, timeout=%d, wc=%zd\n", __func__, fd, size, timeout_msec, wc);
    }
    
    return (wc);
}

static int qlog_is_not_dir(const char *logdir) {
    return (!strncmp(logdir, TFTP_F, strlen(TFTP_F)) || !strncmp(logdir, "/dev/null", strlen("/dev/null")));
}

static const char * qlog_time_name(void) {
    static char time_name[32];
    time_t ltime;
    struct tm *currtime;

    time(&ltime);
    currtime = localtime(&ltime);

    snprintf(time_name, sizeof(time_name), "%04d%02d%02d_%02d%02d%02d",
    	(currtime->tm_year+1900), (currtime->tm_mon+1), currtime->tm_mday,
    	currtime->tm_hour, currtime->tm_min, currtime->tm_sec);

    return time_name;
}

static int qlog_logfile_create(const char *logfile_dir, const char *logfile_suffix, unsigned logfile_seq) {
    int logfd;
    char shortname[32];
    char filename[255+1];

    //delete old logfile
    if (s_logfile_num && s_logfile_List[logfile_seq%s_logfile_num][0]) {
        sprintf(filename, "%s/%s.%s", logfile_dir, s_logfile_List[logfile_seq%s_logfile_num], logfile_suffix);
        if (access(filename, R_OK) == 0) {
            remove(filename);
        }
    }

    snprintf(shortname, sizeof(shortname), "%s_%04d", qlog_time_name(), logfile_seq);
    sprintf(filename, "%s/%s.%s", logfile_dir, shortname, logfile_suffix);

    if (!strncmp(logfile_dir, "/dev/null", strlen("/dev/null")))
        logfd = open("/dev/null", O_WRONLY, 0444);
    else
        logfd = open(filename, O_CREAT | O_WRONLY | O_TRUNC, 0444);
    if (logfd <= 0) {
        qlog_dbg("Fail to create new logfile! errno : %d (%s)\n", errno, strerror(errno));
    }

    qlog_dbg("%s %s logfd=%d\n", __func__, filename, logfd);

    if (s_logfile_num) {
        s_logfile_idx = (logfile_seq%s_logfile_num);
        strcpy(s_logfile_List[s_logfile_idx], shortname);
    }

    return logfd;
}

static size_t qlog_logfile_save(int logfd, const void *buf, size_t size) {
    return qlog_poll_write(logfd, buf, size, 1000);
}

static int qlog_logfile_close(int logfd) {
    return close(logfd);
}

static void* qlog_logfile_init_filter_thread(void* arg) {
    void **thread_args = (void **)arg;
    qlog_ops_t *qlog_ops = (qlog_ops_t *)thread_args[0];
    int *ttyfd = (int *)thread_args[1];
    const char *filter_cfg =  ( const char *)thread_args[2];

    if (qlog_ops->init_filter)
        qlog_ops->init_filter(*ttyfd, filter_cfg);

    qlog_dbg("qlog_init_filter_finished\n");
    qlog_init_filter_finished = 1;
    return NULL;
}

static int qlog_handle(int handlefd, const char *logfile_dir, size_t logfile_size, unsigned logfile_num, const char *filter_cfg) {
    ssize_t savelog_size = 0;
    void *rbuf;
    const size_t rbuf_size = RX_URB_SIZE;
    static int logfd = -1;
    const char *logfile_suffix = g_is_asr_chip ? "sdl" : "qmdl";
    static qlog_ops_t qlog_ops;
    pthread_t thread_id1;
    pthread_attr_t thread_attr;
    const void *thread_args[3];
    size_t total_read = 0;
    unsigned long now_msec = 0;
            
    if (!g_is_asr_chip) {
        if (filter_cfg && strstr(filter_cfg, ".cfg2")) {
            g_mdm_use_cfg2 = 1;
            logfile_suffix = "qmdl2";
        }
    }

    if (logfile_dir[0] == '9' && atoi(logfile_dir) >= 9000) {
        filter_cfg = logfile_dir;
        qlog_ops = tty2tcp_qlog_ops;
        exit_after_usb_disconnet = 1; // do not continue when tty2tcp mode
    }
    else {
        qlog_ops = g_is_asr_chip ? asr_qlog_ops : mdm_qlog_ops;
    }

    if (!qlog_ops.logfile_create)
        qlog_ops.logfile_create = qlog_logfile_create;
    if (!qlog_ops.logfile_save)
        qlog_ops.logfile_save = qlog_logfile_save;
    if (!qlog_ops.logfile_close)
        qlog_ops.logfile_close = qlog_logfile_close;

    rbuf = malloc(rbuf_size);
    if (rbuf == NULL) {
          qlog_dbg("Fail to malloc rbuf_size=%zd, errno: %d (%s)\n", rbuf_size, errno, strerror(errno));
          return -1;
    }

    thread_args[0] = &qlog_ops;
    thread_args[1] = &handlefd;
    thread_args[2] = filter_cfg;
    pthread_attr_init(&thread_attr);
    pthread_attr_setdetachstate(&thread_attr, PTHREAD_CREATE_DETACHED);
    pthread_create(&thread_id1, &thread_attr, qlog_logfile_init_filter_thread, (void*)thread_args);

    now_msec = qlog_msecs();
    while(qlog_exit_requested == 0) {
        ssize_t rc, wc;
        unsigned long n;
        
        rc = qlog_poll_read(handlefd, rbuf, rbuf_size, 15000);
        if (rc <= 0)
            break;

        n = qlog_msecs();
        total_read += rc;

    	if ((total_read >= (16*1024*1024)) || (n >= (now_msec + 5000))) {
    		qlog_dbg("recv: %zdM %zdK %zdB  in %ld msec\n", total_read/(1024*1024),
    			total_read/1024%1024,total_read%1024, n-now_msec);
    		now_msec = n;
    		total_read = 0;
        }
            
        g_rx_log_count++;

        if (logfd == -1) {
            logfd = qlog_ops.logfile_create(logfile_dir, logfile_suffix, s_logfile_seq);
            if (logfd <= 0) {
                break;
            }
            if (qlog_ops.logfile_init)
                qlog_ops.logfile_init(logfd, s_logfile_seq);
                s_logfile_seq++;
        }
                        
        wc = qlog_ops.logfile_save(logfd, rbuf, rc);
 
        if (wc != rc) {
            qlog_dbg("savelog fail %zd/%zd, break\n", wc, rc);
            exit_after_usb_disconnet = 1; // do not continue when not usb disconnect
            break;
        }

        savelog_size += wc;

        if (savelog_size >= logfile_size) {
            savelog_size = 0;
            qlog_ops.logfile_close(logfd);
            logfd = -1;
        }
    }   

    if (logfd > 0)
        qlog_ops.logfile_close(logfd);
    logfd = -1;
    free(rbuf);

    return 0;
}

static void ql_sigaction(int signal_num) {
    qlog_dbg("recv signal %d\n", signal_num);
    qlog_exit_requested = 1;
}

static void qlog_usage(const char *self, const char *dev) {
    qlog_dbg("Usage: %s -p <log port> -s <log save dir> -f filter_cfg -n <log file max num> -b <log file size MBytes>\n", self);
    qlog_dbg("Default: %s -p %s -s %s -n %d -b %d to save log to local disk\n",
        self, dev, ".", LOGFILE_NUM, LOGFILE_SIZE_DEFAULT/1024/1024);
    qlog_dbg("    -p    The port to catch log (default '/dev/ttyUSB0')\n");
    qlog_dbg("    -s    Dir to save log, default is '.' \n");
    qlog_dbg("          if set as '9000', QLog will run in TCP Server Mode, and can be connected with 'QPST/QWinLog/CATStudio'!\n");
    qlog_dbg("    -f    filter cfg for catch log, can be found in directory 'conf'. if not set this arg, will use default filter conf\n");
    qlog_dbg("          and UC200T&EC200T do not need filter cfg.\n");
    qlog_dbg("    -n    max num of log file to save, range is '0~512'. default is 0. 0 means no limit.\n");
    qlog_dbg("          or QLog will auto delete oldtest log file if exceed max num\n");
    qlog_dbg("    -m    max size of single log file, unit is MBytes, range is '2~512', default is 128\n");
    qlog_dbg("    -q    Exit after usb disconnet\n");
	
    qlog_dbg("\nFor example: %s -s .\n", self);
}

static struct arguments *parser_args(int argc, char **argv)
{
    int opt;
    static struct arguments args = {
        .ttyDM = "",
        .logdir = "qlog_files",
        .logfile_num = LOGFILE_NUM,
        .logfile_sz = LOGFILE_SIZE_DEFAULT,
        .filter_cfg = NULL,
    };

    optind = 1; //call by popen(), optind mayby is not 1
    while (-1 != (opt = getopt(argc, argv, "p:s:n:m:f:qh")))
    {
        switch (opt)
        {
        case 'p':
            if (optarg[0] == 't') //ttyUSB0
                snprintf(args.ttyDM, sizeof(args.ttyDM), "/dev/%s", optarg);
            else if (optarg[0] == 'U') //USB0
                snprintf(args.ttyDM, sizeof(args.ttyDM), "/dev/tty%s", optarg);
            else if (optarg[0] == '/')
                snprintf(args.ttyDM, sizeof(args.ttyDM), "%s", optarg);
            else
            {
                qlog_dbg("unknow dev %s\n", optarg);
                goto error;
            }
            qlog_dbg("will use device: %s\n", args.ttyDM);
            break;
        case 's':
            snprintf(args.logdir, sizeof(args.logdir), "%s", optarg);
            qlog_dbg("will save log into dir: %s\n", args.logdir);
            break;
        case 'n':
            args.logfile_num = atoi(optarg);
            if (args.logfile_num < 0)
                args.logfile_num = 0;
            else if (args.logfile_num > LOGFILE_NUM)
                args.logfile_num = LOGFILE_NUM;
            s_logfile_num = args.logfile_num;
            break;
        case 'm':
            args.logfile_sz = atoi(optarg) * 1024 * 1024;
            if (args.logfile_sz < LOGFILE_SIZE_MIN)
                args.logfile_sz = LOGFILE_SIZE_MIN;
            else if (args.logfile_sz > LOGFILE_SIZE_MAX)
                args.logfile_sz = LOGFILE_SIZE_MAX;
            break;
        case 'f':
            args.filter_cfg = optarg;
            break;
        case 'q':
            exit_after_usb_disconnet = 1;
            break;
        case 'h':
        default:
            qlog_usage(argv[0], "/dev/ttyUSB0");
            goto error;
        }
    }

    qlog_dbg("will use filter file: %s\n", args.filter_cfg ? args.filter_cfg : "default filter");

    return &args;
error:
    return NULL;
}

static int serial_open(const char *device)
{
    int ttyfd = open(device, O_RDWR | O_NDELAY | O_NOCTTY);
    if (ttyfd < 0)
    {
        qlog_dbg("Fail to open %s, errno : %d (%s)\n", device, errno, strerror(errno));
    }
    else
    {
        qlog_dbg("open %s ttyfd = %d\n", device, ttyfd);
        struct termios ios;
        memset(&ios, 0, sizeof(ios));
        tcgetattr(ttyfd, &ios);
        cfmakeraw(&ios);
        cfsetispeed(&ios, B115200);
        cfsetospeed(&ios, B115200);
        tcsetattr(ttyfd, TCSANOW, &ios);
    }
    return ttyfd;
}

static int qlog_avail_space_for_dump(const char *dir, long need_MB) {
    long free_space = 0;
    struct statfs stat;

    if (!statfs(dir, &stat)) {
        free_space = stat.f_bavail*(stat.f_bsize/512)/2; //KBytes
    }
    else {
        qlog_dbg("statfs %s, errno : %d (%s)\n", dir, errno, strerror(errno));        
    }

    free_space = (free_space/1024);
    if (free_space < need_MB) {
        qlog_dbg("free space is %ldMBytes, need %ldMB\n", free_space, need_MB);
        return 0;
    }

    return 1;
}

static inline int drv_is_asr(int idProduct)
{
    int is_asr = 0;
    if ((idProduct & 0xF000) == 0x6000) // ASR
        is_asr = 1;
    return is_asr;
}

struct usbfs_getdriver
{
    unsigned int interface;
    char driver[255 + 1];
};

struct usbfs_ioctl
{
    int ifno;       /* interface 0..N ; negative numbers reserved */
    int ioctl_code; /* MUST encode size + direction of data so the
			 * macros in <asm/ioctl.h> give correct values */
    void *data;     /* param buffer (in, or out) */
};

#define IOCTL_USBFS_DISCONNECT _IO('U', 22)
#define IOCTL_USBFS_CONNECT _IO('U', 23)

static int usbfs_is_kernel_driver_alive(int fd, int ifnum)
{
    struct usbfs_getdriver getdrv;
    getdrv.interface = ifnum;
    if (ioctl(fd, USBDEVFS_GETDRIVER, &getdrv) < 0)
    {
        if (errno != ENODATA)
            qlog_dbg("%s ioctl USBDEVFS_GETDRIVER on interface %d failed, kernel driver may be inactive\n", __func__, ifnum);
        return 0;
    }
    qlog_dbg("%s find interface %d has match the driver %s\n", __func__, ifnum, getdrv.driver);
    return 1;
}

static void usbfs_detach_kernel_driver(int fd, int ifnum)
{
    struct usbfs_ioctl operate;
    operate.data = NULL;
    operate.ifno = ifnum;
    operate.ioctl_code = IOCTL_USBFS_DISCONNECT;
    if (ioctl(fd, USBDEVFS_IOCTL, &operate) < 0)
        qlog_dbg("%s detach kernel driver failed\n", __func__);
    else
        qlog_dbg("%s detach kernel driver success\n", __func__);
}

static int prepare(struct arguments *args)
{
    int ret = -1;
    const  struct ql_usb_device_info *usb_dev = args->ql_dev;

    if (usb_dev->ttyDM[0] == '\0')
    {
        char dev_bus_usb_dev[64];
        pthread_t thread_id;
        pthread_attr_t usb_thread_attr;
    
        snprintf(dev_bus_usb_dev, sizeof(dev_bus_usb_dev), "/dev/bus/usb/%03d/%03d", usb_dev->busnum, usb_dev->devnum);

        args->usbfd = open(dev_bus_usb_dev, O_RDWR | O_NDELAY);
        qlog_dbg("open %s usbfd = %d\n", dev_bus_usb_dev, args->usbfd);
        if (args->usbfd < 0)
        {
            qlog_dbg("usbfs open %s failed, errno: %d (%s)\n", dev_bus_usb_dev, errno, strerror(errno));
            goto error;
        }

        if (usbfs_is_kernel_driver_alive(args->usbfd, usb_dev->dm_intf))
            usbfs_detach_kernel_driver(args->usbfd, usb_dev->dm_intf);

        ret = ioctl(args->usbfd, USBDEVFS_CLAIMINTERFACE, &usb_dev->dm_intf); // attach usbfs driver
        if (ret != 0)
        {
            safe_close_fd(args->usbfd);
            qlog_dbg("ioctl USBDEVFS_CLAIMINTERFACE failed, errno = %d(%s)\n", errno, strerror(errno));
            goto error;
        }

        socketpair(AF_LOCAL, SOCK_STREAM, 0, args->usb_sockets);
        args->ttyfd = args->usb_sockets[0];

        pthread_attr_init(&usb_thread_attr);
        pthread_attr_setdetachstate(&usb_thread_attr, PTHREAD_CREATE_DETACHED);
        pthread_create(&thread_id, &usb_thread_attr, qlog_usbfs_read, (void*)args);
    }
    else
    {
        args->ttyfd = serial_open(usb_dev->ttyDM);
        if (args->ttyfd < 0)
        {
            qlog_dbg("tty open %s failed, errno: %d (%s)\n", usb_dev->ttyDM, errno, strerror(errno));
            goto error;
        }
    }

    return 0;
error:
    return ret;
}

static int ql_find_quectel_modules(void)
{
    DIR *usb_dir = NULL, *dev_dir = NULL;
    struct dirent *usb = NULL, *dev = NULL;
    const char *usbpath = "/sys/bus/usb/devices";
    struct ql_usb_device_info ql_dev;
    int modules_num = 0;

    usb_dir = opendir(usbpath);
    if (NULL == usb_dir)
        return modules_num;

    while (NULL != (usb = readdir(usb_dir)))
    {
        char devpath[256*3] = {'\0'};
        if (usb->d_name[0] == '.' || usb->d_name[0] == 'u')
            continue;

        memset(&ql_dev, 0, sizeof(struct ql_usb_device_info));

        snprintf(devpath, sizeof(devpath), "%s/%s/idVendor", usbpath, usb->d_name);
        ql_dev.idVendor = get_value_from_file(devpath, 16);
        if (ql_dev.idVendor != 0x2c7c)
            continue;

        snprintf(devpath, sizeof(devpath), "%s/%s/idProduct", usbpath, usb->d_name);
        ql_dev.idProduct = get_value_from_file(devpath, 16);

        snprintf(devpath, sizeof(devpath), "%s/%s/bNumInterfaces", usbpath, usb->d_name);
        ql_dev.bNumInterfaces = get_value_from_file(devpath, 10);

        snprintf(devpath, sizeof(devpath), "%s/%s/busnum", usbpath, usb->d_name);
        ql_dev.busnum = get_value_from_file(devpath, 10);

        snprintf(devpath, sizeof(devpath), "%s/%s/devnum", usbpath, usb->d_name);
        ql_dev.devnum = get_value_from_file(devpath, 10);

        ql_dev.dm_intf = 0;
        if (ql_dev.bNumInterfaces > 1) {
            if (drv_is_asr(ql_dev.idProduct)) { //ASR
                ql_dev.dm_intf = 2;
                ql_dev.dm_ep_in = 0x84;
                ql_dev.dm_ep_out = 0x03;
            }
            else {
                ql_dev.dm_ep_in = 0x81;
                ql_dev.dm_ep_out = 0x01;
            }
        }
        else {
            if (drv_is_asr(ql_dev.idProduct)) { //ASR
                ql_dev.dm_ep_in = 0x82;
                ql_dev.dm_ep_out = 0x01;
            }
            else {
                ql_dev.dm_ep_in = 0x81;
                ql_dev.dm_ep_out = 0x01;
            }
        }

        snprintf(devpath, sizeof(devpath), "%s/%s/%s:1.%d", usbpath, usb->d_name, usb->d_name, ql_dev.dm_intf);
        if (access(devpath, F_OK))
            continue;

        // find tty device
        dev_dir = opendir(devpath);
        if (dev_dir)
        {
            while (NULL != (dev = readdir(dev_dir)))
            {
                if (!strncasecmp(dev->d_name, "tty", 3))
                {
                    snprintf(ql_dev.ttyDM, sizeof(ql_dev.ttyDM), "/dev/%s", dev->d_name);
                    break;
                }
            }    
            closedir(dev_dir);

            if (!strcmp(ql_dev.ttyDM, "/dev/tty")) { //find tty not ttyUSBx or ttyACMx
                snprintf(devpath, sizeof(devpath), "%s/%s/%s:1.%d/tty", usbpath, usb->d_name, usb->d_name, ql_dev.dm_intf);

                dev_dir = opendir(devpath);
                if (dev_dir)
                {
                    while (NULL != (dev = readdir(dev_dir)))
                    {
                        if (!strncasecmp(dev->d_name, "tty", 3))
                        {
                                snprintf(ql_dev.ttyDM, sizeof(ql_dev.ttyDM), "/dev/%s", dev->d_name);
                                break;
                        }
                    }
                    closedir(dev_dir);
                }
            }
        }

        snprintf(ql_dev.usbdevice_pah, sizeof(ql_dev.usbdevice_pah), "%s/%s", usbpath, usb->d_name);
        qlog_dbg("Find [%d] idVendor=%04x, idProduct=%04x, bNumInterfaces=%d, ttyDM=%s, busnum=%03d, dev=%03d, usbdevice_pah=%s\n",
            modules_num,
            ql_dev.idVendor, ql_dev.idProduct, ql_dev.bNumInterfaces, ql_dev.ttyDM,
            ql_dev.busnum, ql_dev.devnum, ql_dev.usbdevice_pah);
 
        if (modules_num < 8)
            s_usb_device_info[modules_num++] = ql_dev;
    }
 
    closedir(usb_dir);
    return modules_num;
}

int main(int argc, char **argv)
{
    int ret = -1;
    struct arguments *args;
    int modules_num = 0;
    int cur_module = 0;
    int loop_times = 0;

    qlog_dbg("QLog Version: Quectel_QLog_Linux&Android_V%s\n", QLOG_VERSION);

    args = parser_args(argc, argv);
    if (!args)
    {
        return 0;
    }

    signal(SIGTERM, ql_sigaction);
    signal(SIGHUP, ql_sigaction);
    signal(SIGINT, ql_sigaction);

__restart:
    if (qlog_exit_requested)
        return 0;

    args->ttyfd = args->usbfd = args->usb_sockets[0] = args->usb_sockets[1] = -1;
    args->ql_dev = NULL;
    memset(s_usb_device_info, 0, sizeof(s_usb_device_info));
    cur_module = modules_num = 0;

    if (!strncmp(args->ttyDM, "/dev/mhi", strlen("/dev/mhi"))) {
        struct ql_usb_device_info *mhi_dev = &s_usb_device_info[0];

        memset(mhi_dev, 0, sizeof(struct ql_usb_device_info));
        mhi_dev->idVendor = 0x2C7C;
        mhi_dev->idProduct = 0x0800;
        if (!strncmp(args->ttyDM, "/dev/mhi_DIAG", strlen("/dev/mhi_DIAG")))
            mhi_dev->bNumInterfaces = 5;
        else if (!strncmp(args->ttyDM, "/dev/mhi_SAHARA", strlen("/dev/mhi_SAHARA")))
            mhi_dev->bNumInterfaces = 1;
        strncpy(mhi_dev->ttyDM, args->ttyDM, sizeof(mhi_dev->ttyDM));
        modules_num = 1;
    }

    if (modules_num == 0) {
        modules_num = ql_find_quectel_modules();
        if (modules_num == 0) {
            qlog_dbg("No Quectel Modules found, Wait for connect or Press CTRL+C to quit!\n");
            sleep(2);
            goto __restart;
        }
    }

    if (!strncmp(args->ttyDM, "/dev/ttyUSB", strlen("/dev/ttyUSB"))) {
        for (cur_module = 0; cur_module < modules_num; cur_module++) {
            if (!strcmp(args->ttyDM, s_usb_device_info[cur_module].ttyDM)) {
                break;
            }
        }
        if (cur_module == modules_num) {
            qlog_dbg("No %s find, wait for connect!", args->ttyDM);
            goto __restart;
        }
    }

    args->ql_dev = &s_usb_device_info[cur_module];
    g_is_asr_chip = drv_is_asr(args->ql_dev->idProduct);
    g_is_usb_disconnect = 0;

    qlog_args = args;
    ret = prepare(args);
    if (ret < 0)
    {
        qlog_dbg("arg do prepare failed\n");
        return ret;
    }

    loop_times++;
    if (access(args->logdir, F_OK) && errno == ENOENT)
        mkdir(args->logdir, 0755);

    qlog_dbg("Press CTRL+C to stop catch log.\n");
    if (args->ql_dev->bNumInterfaces == 1) {
        s_logfile_List[s_logfile_idx][0] = '\0'; //to prevent this log delete, log before dump
        char dump_dir[255];

        if (qlog_is_not_dir(args->logdir))
            snprintf(dump_dir, sizeof(dump_dir), "%s", args->logdir);
        else {
            snprintf(dump_dir, sizeof(dump_dir), "%s/dump_%s", args->logdir, qlog_time_name());
            mkdir(dump_dir, 0755);
            if (!qlog_avail_space_for_dump(dump_dir, g_is_asr_chip ? 128 : 256)) {
                 qlog_dbg("no enouth disk to save dump\n");
                 qlog_exit_requested = 1;
                 goto error;
           }
        }
        
        if (g_is_asr_chip)
        {
            qlog_dbg("catch dump for asr chipset\n");
            ret = asr_catch_dump(args->ttyfd, dump_dir);
        }
        else
        {
            qlog_dbg("catch dump for mdm chipset\n");
            ret = sahara_catch_dump(args->ttyfd, dump_dir, 1);
        }
    }
    else if (args->ql_dev->bNumInterfaces > 1) {
        if (args->usbfd == -1)
            qlog_dbg("catch log via tty port\n");
        else {
            qlog_dbg("catch log via usbfs\n");
        }
        ret = qlog_handle(args->ttyfd, args->logdir, args->logfile_sz, args->logfile_num, args->filter_cfg);
    } 
    else {
        qlog_dbg("unknow state! quit!\n");
        goto error;
    }

error:    
    if (args->usbfd != -1)
    {
        ioctl(args->usbfd, USBDEVFS_RELEASEINTERFACE, &args->ql_dev->dm_intf);
        safe_close_fd(args->usbfd);
        safe_close_fd(args->usb_sockets[0]);
        safe_close_fd(args->usb_sockets[1]);
    }
    else {
        safe_close_fd(args->ttyfd);
    }

    if (qlog_exit_requested == 0 && exit_after_usb_disconnet == 0) {
        sleep(1);
        goto __restart;
    }

    return ret;
}
