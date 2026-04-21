/*
 * Algorithm to process Wiimote IR tracking data into a usable pointer position
 * by tracking the sensor bar.
 *
 * Copyright (c) 2008-2011 Hector Martin "marcan" <hector@marcansoft.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef WIIPOINTER_H
#define WIIPOINTER_H

// Information on one IR dot from the Wiimote
typedef struct ir_dot_t {
    int visible;    // Does the dot exist (is it visible)?
    int rx;         // X coordinate (0-1023)
    int ry;         // Y coordinate (0-768)
} ir_dot_t;

typedef struct fdot_t {
    float x,y;
} fdot_t;

// Holds state information on the sensor bar
typedef struct sb_t {
    fdot_t dots[2];
    fdot_t acc_dots[2];
    fdot_t rot_dots[2];
    float angle;
    float off_angle;
    float score;
} sb_t;

typedef struct ir_t {
    // Input data from wiimote: fill this in yourself
    struct ir_dot_t dot[4]; // IR dots from camera sensor
    float roll;             // Roll from accelerometer (rotation) in radians.
                            // You can calculate this as atan2(x, z). If roll
                            // data is unreliable (wiimote is significantly
                            // accelerating) then you should supply the last
                            // known good value.

    // Internal state
    int num_dots;
    int state;
    sb_t sensorbar;

    // Output data
    int raw_valid;          // Is the raw position valid?
    float ax;               // Raw X coordinate (-512..512, 0 is center)
    float ay;               // Raw Y coordinate (-512..512, 0 is center)
    float distance;         // Pixel width of the sensor bar
    float z;                // Wiimote to sensor bar distance in meters
    float angle;            // Angle of the wiimote to the sensor bar (radians)

    int smooth_valid;       // Is the smoothed position valid?
    float sx;               // Smoothed X coordinate
    float sy;               // Smoothed Y coordinate
    float error_cnt;        // Error count from smoothing algorithm
    float glitch_cnt;       // Glitch count from smoothing algorithm
} ir_t;

void initialize_ir(struct ir_t* ir);
void process_ir_data(struct ir_t* ir);

#endif
