// SPDX-License-Identifier: (GPL-2.0)
/*
 * Miyoo Flip Hacky hax hax hax Analogue uInput Helper
 *
 * Copyright (c) 2024 Gamma <thegammasqueeze@gmail.com>
 */

/*******************************************************************************
 * File: miyoostick.c
 *
 * Example userland joystick application:
 *  - Sets up a virtual gamepad via /dev/uinput
 *  - Opens /dev/ttyS1, reads stick frames, calibrates them
 *  - Publishes the results as analog axes (two sticks)
 *  - Provides a few example buttons
 *
 * Build (example):
 *    gcc -o miyoostick miyoostick.c -lpthread
 *
 * Run:
 *    ./miyoostick
 *
 * Check via:
 *    getevent -p  (Android) or evtest (Linux desktop)
 *    dumpsys input (Android)
 *
 ******************************************************************************/

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include <termios.h>
#include <linux/uinput.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/time.h>
#include <sys/types.h>

/* -------------------------------------------------------------------------
   Defines / constants
   ------------------------------------------------------------------------- */
#define UINPUT_DEVICE    "/dev/uinput"
#define SERIAL_DEVICE    "/dev/ttyS1"

// Adjust these to taste
#define JOYSTICK_NAME    "Miyoo Virtual Stick"
#define VENDOR_ID        0x045e     // e.g. Microsoft
#define PRODUCT_ID       0x028e     // e.g. Xbox 360 pad
#define VERSION_ID       0x0001

// The custom protocol: 6 bytes
//   Byte0 = 0xFF (magic start)
//   Byte1 = axisYL
//   Byte2 = axisXL
//   Byte3 = axisYR
//   Byte4 = axisXR
//   Byte5 = 0xFE (magic end)
#define FRAME_SIZE       6
#define MAGIC_START      0xFF
#define MAGIC_END        0xFE

// For the calibration example, we want these final ranges
#define AXIS_MIN         (-32760)
#define AXIS_MAX         ( 32760)

// We will gather 50 frames for auto-cal
#define CALIBRATION_FRAMES 50

// We'll do an adjustable radial deadzone ratio (10% = 0.10f)
static float g_deadzone_ratio = 0.10f;

/* -------------------------------------------------------------------------
   Structs / global
   ------------------------------------------------------------------------- */

// Simple per-stick calibration
typedef struct {
    int x_min;
    int x_max;
    int x_zero;
    int y_min;
    int y_max;
    int y_zero;
} stick_cal_t;

// We keep default calibration, but we will override x_zero,y_zero with an average
static stick_cal_t g_left_cal = {
    .x_min  = 85,
    .x_max  = 200,
    .x_zero = 130,
    .y_min  = 85,
    .y_max  = 200,
    .y_zero = 130
};

static stick_cal_t g_right_cal = {
    .x_min  = 85,
    .x_max  = 200,
    .x_zero = 130,
    .y_min  = 85,
    .y_max  = 200,
    .y_zero = 130
};

// The uinput file descriptor
static int g_uinput_fd = -1;

// The serial read thread
static pthread_t g_serial_thread;

/* -------------------------------------------------------------------------
   Utility: set up /dev/uinput for a 2-stick gamepad
   ------------------------------------------------------------------------- */
static int setup_uinput_device(void)
{
    int fd;
    struct uinput_setup usetup;

    fd = open(UINPUT_DEVICE, O_WRONLY | O_NONBLOCK);
    if(fd < 0) {
        fprintf(stderr, "ERROR: Cannot open %s: %s\n", UINPUT_DEVICE, strerror(errno));
        return -1;
    }

    // Enable key events
    ioctl(fd, UI_SET_EVBIT, EV_KEY);
    // Enable abs events
    ioctl(fd, UI_SET_EVBIT, EV_ABS);

    // -- Buttons
    ioctl(fd, UI_SET_KEYBIT, BTN_A);
    ioctl(fd, UI_SET_KEYBIT, BTN_B);
    ioctl(fd, UI_SET_KEYBIT, BTN_X);
    ioctl(fd, UI_SET_KEYBIT, BTN_Y);
    ioctl(fd, UI_SET_KEYBIT, BTN_TL);
    ioctl(fd, UI_SET_KEYBIT, BTN_TR);
    ioctl(fd, UI_SET_KEYBIT, BTN_SELECT);
    ioctl(fd, UI_SET_KEYBIT, BTN_START);

    // -- 4 axes: Left stick => ABS_X, ABS_Y
    //             Right stick => ABS_RX, ABS_RY
    ioctl(fd, UI_SET_ABSBIT, ABS_X);
    ioctl(fd, UI_SET_ABSBIT, ABS_Y);
    ioctl(fd, UI_SET_ABSBIT, ABS_RX);
    ioctl(fd, UI_SET_ABSBIT, ABS_RY);

    // Setup axis ranges
    struct uinput_abs_setup abs_setup;
    memset(&abs_setup, 0, sizeof(abs_setup));

    // ABS_X
    abs_setup.code = ABS_X;
    abs_setup.absinfo.value = 0;
    abs_setup.absinfo.minimum = AXIS_MIN;
    abs_setup.absinfo.maximum = AXIS_MAX;
    abs_setup.absinfo.fuzz    = 16;
    abs_setup.absinfo.flat    = 16;
    if(ioctl(fd, UI_ABS_SETUP, &abs_setup) < 0) {
        fprintf(stderr, "ERROR: UI_ABS_SETUP ABS_X failed\n");
    }

    // ABS_Y
    abs_setup.code = ABS_Y;
    abs_setup.absinfo.value   = 0;
    abs_setup.absinfo.minimum = AXIS_MIN;
    abs_setup.absinfo.maximum = AXIS_MAX;
    abs_setup.absinfo.fuzz    = 16;
    abs_setup.absinfo.flat    = 16;
    if(ioctl(fd, UI_ABS_SETUP, &abs_setup) < 0) {
        fprintf(stderr, "ERROR: UI_ABS_SETUP ABS_Y failed\n");
    }

    // ABS_RX
    abs_setup.code = ABS_RX;
    abs_setup.absinfo.value   = 0;
    abs_setup.absinfo.minimum = AXIS_MIN;
    abs_setup.absinfo.maximum = AXIS_MAX;
    abs_setup.absinfo.fuzz    = 16;
    abs_setup.absinfo.flat    = 16;
    if(ioctl(fd, UI_ABS_SETUP, &abs_setup) < 0) {
        fprintf(stderr, "ERROR: UI_ABS_SETUP ABS_RX failed\n");
    }

    // ABS_RY
    abs_setup.code = ABS_RY;
    abs_setup.absinfo.value   = 0;
    abs_setup.absinfo.minimum = AXIS_MIN;
    abs_setup.absinfo.maximum = AXIS_MAX;
    abs_setup.absinfo.fuzz    = 16;
    abs_setup.absinfo.flat    = 16;
    if(ioctl(fd, UI_ABS_SETUP, &abs_setup) < 0) {
        fprintf(stderr, "ERROR: UI_ABS_SETUP ABS_RY failed\n");
    }

    // Now create device
    memset(&usetup, 0, sizeof(usetup));
    usetup.id.bustype = BUS_USB;
    usetup.id.vendor  = VENDOR_ID;
    usetup.id.product = PRODUCT_ID;
    usetup.id.version = VERSION_ID;
    strncpy(usetup.name, JOYSTICK_NAME, UINPUT_MAX_NAME_SIZE);

    if(ioctl(fd, UI_DEV_SETUP, &usetup) < 0) {
        fprintf(stderr, "ERROR: UI_DEV_SETUP failed\n");
        close(fd);
        return -1;
    }

    if(ioctl(fd, UI_DEV_CREATE) < 0) {
        fprintf(stderr, "ERROR: UI_DEV_CREATE failed\n");
        close(fd);
        return -1;
    }

    printf("Created uinput device %s successfully.\n", JOYSTICK_NAME);
    return fd;
}

/* -------------------------------------------------------------------------
   Emit an ABS event with a final SYN
   ------------------------------------------------------------------------- */
static void emit_abs_event(int fd, unsigned int code, int value)
{
    struct input_event ie;

    // ABS event
    memset(&ie, 0, sizeof(ie));
    ie.type = EV_ABS;
    ie.code = code;
    ie.value = value;
    if(write(fd, &ie, sizeof(ie)) < 0) {
        fprintf(stderr, "ERROR: write abs_event failed: %s\n", strerror(errno));
    }

    // SYN event
    memset(&ie, 0, sizeof(ie));
    ie.type  = EV_SYN;
    ie.code  = SYN_REPORT;
    ie.value = 0;
    if(write(fd, &ie, sizeof(ie)) < 0) {
        fprintf(stderr, "ERROR: write syn_event failed: %s\n", strerror(errno));
    }
}

/* -------------------------------------------------------------------------
   Example calibration logic: Map [x_min..x_max] -> [AXIS_MIN..AXIS_MAX]
   If raw < x_zero => negative side. If raw > x_zero => positive side.
   ------------------------------------------------------------------------- */
static int calibrate_axis_x(const stick_cal_t* cal, int raw)
{
    if(!cal) return 0;

    int rangePos = (cal->x_max - cal->x_zero);  // positive side
    int rangeNeg = (cal->x_zero - cal->x_min);  // negative side
    int result   = 0;

    if(raw > cal->x_zero) {
        int diff = raw - cal->x_zero;
        if(rangePos != 0) {
            result = (diff * AXIS_MAX) / rangePos;
            if(result > AXIS_MAX) result = AXIS_MAX;
        }
    } else if(raw < cal->x_zero) {
        int diff = cal->x_zero - raw;
        if(rangeNeg != 0) {
            result = -(diff * (-AXIS_MIN)) / rangeNeg; // -(-32760) = 32760
            if(result < AXIS_MIN) result = AXIS_MIN;
        }
    }
    return result;
}

static int calibrate_axis_y(const stick_cal_t* cal, int raw)
{
    if(!cal) return 0;

    int rangePos = (cal->y_max - cal->y_zero);
    int rangeNeg = (cal->y_zero - cal->y_min);
    int result   = 0;

    if(raw > cal->y_zero) {
        int diff = raw - cal->y_zero;
        if(rangePos != 0) {
            result = (diff * AXIS_MAX) / rangePos;
            if(result > AXIS_MAX) result = AXIS_MAX;
        }
    } else if(raw < cal->y_zero) {
        int diff = cal->y_zero - raw;
        if(rangeNeg != 0) {
            result = -(diff * (-AXIS_MIN)) / rangeNeg;
            if(result < AXIS_MIN) result = AXIS_MIN;
        }
    }
    return result;
}

/* -------------------------------------------------------------------------
   We apply a radial deadzone after the calibration. For a 10% ratio:
   if (lx^2 + ly^2 < (0.1 * AXIS_MAX)^2 ) => clamp to 0,0
   Similarly for Rx,Ry.
   ------------------------------------------------------------------------- */
static void apply_deadzone(int *px, int *py)
{
    int x = *px;
    int y = *py;

    long long magSq = (long long)x * (long long)x + (long long)y * (long long)y;
    long long dead  = (long long)(g_deadzone_ratio * (float)AXIS_MAX);
    long long deadSq= dead * dead;

    if(magSq <= deadSq) {
        // inside deadzone
        *px = 0;
        *py = 0;
    }
}

/* -------------------------------------------------------------------------
   Handle a 6-byte frame: [FF, YL, XL, YR, XR, FE]
   Convert to two pairs:
     leftX = calibrate_axis_x(...)
     leftY = calibrate_axis_y(...)
     rightX ...
     rightY ...
   Then apply radial deadzone, then emit to uinput
   ------------------------------------------------------------------------- */
static void handle_frame(uint8_t *frame)
{
    // frame[0] = 0xFF, frame[1] = axisYL, frame[2] = axisXL,
    // frame[3] = axisYR, frame[4] = axisXR, frame[5] = 0xFE
    if(frame[0] != MAGIC_START || frame[5] != MAGIC_END) {
        // Not a valid frame, ignore
        return;
    }

    int rawYL = frame[1];
    int rawXL = frame[2];
    int rawYR = frame[3];
    int rawXR = frame[4];

    // Calibrate
    int leftX  = calibrate_axis_x(&g_left_cal,  rawXL);
    int leftY  = calibrate_axis_y(&g_left_cal,  rawYL);
    int rightX = calibrate_axis_x(&g_right_cal, rawXR);
    int rightY = calibrate_axis_y(&g_right_cal, rawYR);

    // Apply radial deadzone to each stick
    apply_deadzone(&leftX, &leftY);
    apply_deadzone(&rightX, &rightY);

    // Now emit
    emit_abs_event(g_uinput_fd, ABS_X, leftX);
    emit_abs_event(g_uinput_fd, ABS_Y, leftY);
    emit_abs_event(g_uinput_fd, ABS_RX, rightX);
    emit_abs_event(g_uinput_fd, ABS_RY, rightY);
}

/* -------------------------------------------------------------------------
   Setup the serial /dev/ttyS1 for 9600 8N1
   ------------------------------------------------------------------------- */
static int setup_serial_port(const char *port)
{
    int fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
    if(fd < 0) {
        fprintf(stderr, "ERROR: open %s failed: %s\n", port, strerror(errno));
        return -1;
    }
    // Make blocking
    fcntl(fd, F_SETFL, 0);

    // Optional: isatty test
    struct termios options;
    memset(&options, 0, sizeof(options));
    if(tcgetattr(fd, &options) < 0) {
        fprintf(stderr, "ERROR: tcgetattr() %s\n", strerror(errno));
        close(fd);
        return -1;
    }

    // 9600, 8N1, no flow ctrl
    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);
    options.c_cflag |= (CLOCAL | CREAD); // local line, enable receiver
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8; // 8 data bits
    options.c_cflag &= ~PARENB; // no parity
    options.c_cflag &= ~CSTOPB; // 1 stop bit
    options.c_cflag &= ~CRTSCTS; // no flow ctrl

    // Make raw
    options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP
                         | INLCR | IGNCR | ICRNL | IXON);
    options.c_oflag &= ~OPOST;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    // VMIN=1, VTIME=0 => read returns as soon as 1 byte is available
    options.c_cc[VTIME] = 0;
    options.c_cc[VMIN]  = 1;

    tcflush(fd, TCIFLUSH);
    if(tcsetattr(fd, TCSANOW, &options) < 0) {
        fprintf(stderr, "ERROR: tcsetattr() failed: %s\n", strerror(errno));
        close(fd);
        return -1;
    }

    printf("Opened serial port %s successfully.\n", port);
    return fd;
}

/* -------------------------------------------------------------------------
   We'll do a simple auto-cal: read 50 frames from the serial, sum them up,
   then use the average as each stick's .x_zero / .y_zero.

   This means you do NOT move the sticks (or keep them near center) during this
   50-frame read if you want a good center. If you want a more robust approach
   (like letting the user move the stick fully), you'd do something else.

   We'll do a blocking read approach here (like handle_frame does) for simplicity.
   We read 50 times, store sums of YL,XL,YR,XR. Then set the zeros.
   The user can still adjust x_min / x_max in the g_left_cal, etc., as needed.
   ------------------------------------------------------------------------- */
static void auto_calibrate_serial(int fd)
{
    printf("Auto-calibration: reading %d frames to find average center...\n", CALIBRATION_FRAMES);

    long sumYL = 0, sumXL = 0, sumYR = 0, sumXR = 0;
    uint8_t buf[FRAME_SIZE];
    int framesRead = 0;
    while(framesRead < CALIBRATION_FRAMES) {
        ssize_t n = read(fd, buf, FRAME_SIZE);
        if(n < 0) {
            fprintf(stderr, "Auto-cal read error: %s\n", strerror(errno));
            usleep(10000);
            continue;
        } else if(n < FRAME_SIZE) {
            // partial read, ignore
            continue;
        }

        if(buf[0] == MAGIC_START && buf[5] == MAGIC_END) {
            sumYL += buf[1];
            sumXL += buf[2];
            sumYR += buf[3];
            sumXR += buf[4];
            framesRead++;
        }
    }

    // compute average
    int avgYL = (int)(sumYL / CALIBRATION_FRAMES);
    int avgXL = (int)(sumXL / CALIBRATION_FRAMES);
    int avgYR = (int)(sumYR / CALIBRATION_FRAMES);
    int avgXR = (int)(sumXR / CALIBRATION_FRAMES);

    printf("  Average YL=%d, XL=%d, YR=%d, XR=%d\n", avgYL, avgXL, avgYR, avgXR);

    // We store these as x_zero,y_zero in our calibrations
    g_left_cal.x_zero  = avgXL;
    g_left_cal.y_zero  = avgYL;
    g_right_cal.x_zero = avgXR;
    g_right_cal.y_zero = avgYR;

    printf("Auto-calibration complete.\n");
}

/* -------------------------------------------------------------------------
   Serial reading thread
   ------------------------------------------------------------------------- */
static void *serial_thread_func(void *arg)
{
    int fd = *((int*)arg);
    uint8_t buffer[FRAME_SIZE];
    ssize_t n;

    while(1) {
        // Read exactly 6 bytes.
        n = read(fd, buffer, FRAME_SIZE);
        if(n < 0) {
            fprintf(stderr, "Serial read error: %s\n", strerror(errno));
            usleep(10000);
            continue;
        } else if(n == 0) {
            // no data
            usleep(10000);
            continue;
        } else if(n < FRAME_SIZE) {
            // partial read
            continue;
        }

        // parse
        handle_frame(buffer);
    }
    return NULL;
}

/* -------------------------------------------------------------------------
   main()
   ------------------------------------------------------------------------- */
int main(int argc, char *argv[])
{
    int serial_fd;
    printf("Starting miyoostick...\n");

    // 1) Setup the uinput device
    g_uinput_fd = setup_uinput_device();
    if(g_uinput_fd < 0) {
        fprintf(stderr, "ERROR: Could not setup uinput device.\n");
        return 1;
    }

    // 2) Setup the serial port
    serial_fd = setup_serial_port(SERIAL_DEVICE);
    if(serial_fd < 0) {
        close(g_uinput_fd);
        return 1;
    }

    // 3) Auto-calibrate by reading 50 frames (stick presumably near center)
    auto_calibrate_serial(serial_fd);

    // 4) Create background thread for reading frames
    pthread_create(&g_serial_thread, NULL, serial_thread_func, &serial_fd);

    // 5) Just sleep forever
    while(1) {
        sleep(1);
    }

    // On exit, clean up
    close(serial_fd);

    // Destroy the uinput device
    ioctl(g_uinput_fd, UI_DEV_DESTROY);
    close(g_uinput_fd);

    return 0;
}
