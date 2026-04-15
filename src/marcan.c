/*
 * Algorithm to process Wiimote IR tracking data into a usable pointer position
 * by tracking the sensor bar.
 *
 * Copyright (c) 2008-2011 Hector Martin "marcan" <marcan@marcan.st>
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

/* USAGE:
 *
 * First:
 * - Allocate an ir_t and call initialize_ir() on it.
 *
 * Every time you get data from the Wiimote:
 * - Fill in ir->dot[] and ir->roll with data from the wiimote (IR dots and
 *   accelerometer roll)
 * - Call process_ir_data(ir);
 * - If ir->smooth_valid, the position in ir->sx and ir->sy is valid.
 * - Scale and translate the position for your screen and sensor bar placement
 *   (above or below screen). This is up to you.
 *
 * The returned position is roughly in the range (-512..512) for both X and Y,
 * where 0 is center and 512 is about the maximum offset. The actual returned
 * values will not necessarily cover that range (e.g. don't expect more than
 * -384..384 for Y if the wiimote is level). This range represents a square
 * screen. The values might exceed -512 or 512 under some circumstances.
 *
 * Keep in mind that you want to map the screen to a subset of this space, both
 * because presumably your screen doesn't have a 1:1 aspect ratio, and because
 * the Wiimote won't be able to cover the entire space. The worst case scenario
 * is when the Wiimote is sideways, where the X coordinate might only cover
 * about -384..384, and the corresponding Y coordinate range would only be
 * -216..216 for a 16:9 screen. There is a tradeoff here: using a larger range
 * means not being able to reach the edges of the screen with the wiimote turned
 * sideways, while using a smaller range means the cursor moves faster and is
 * harder to control.
 *
 * For example, a conservative mapping for a 16:9 screen might be:
 *
 * X left = -340, X right = 340
 * If sensor bar is below screen:
 *  Y top = -290
 *  Y bottom = 92
 * If sensor bar is above screen:
 *  Y top = -92
 *  Y bottom = 290
 *
 * While a wider mapping might be:
 * X left = -430, X right = 430
 * If sensor bar is below screen:
 *  Y top = -290
 *  Y bottom = 194
 * If sensor bar is above screen:
 *  Y top = -194
 *  Y bottom = 290
 *
 * Wider than the above starts having trouble at the edges.
 *
 * Notes on signs and ranges:
 * Raw Wiimote IR data maps 0,0 to the bottom left corner of the sensor's field
 * of view (this corresponds to pointing the wiimote up and to the right). This
 * is the format expected in ir->dot. ir->roll should be 0 when the wiimote is
 * level and should increase as it is rotated clockwise, covering a -pi to pi
 * range. Output data has a positive X when pointing to the right of the sensor
 * bar, and a positive Y when pointing under the sensor bar, with 0,0
 *  corresponding to pointing directly at the sensor bar.
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include "wiipointer.h"

#define DEBUG

#ifdef DEBUG
#define DEBUG_PRINT(...) fprintf(stderr, __VA_ARGS__)
#else
#define DEBUG_PRINT(...)
#endif

enum {
    IR_STATE_DEAD = 0,
    IR_STATE_GOOD,
    IR_STATE_SINGLE,
    IR_STATE_LOST,
};

// half-height of the IR sensor if half-width is 1
#define HEIGHT (384.0f / 512.0f)
// maximum sensor bar slope (tan(35 degrees))
#define MAX_SB_SLOPE 0.7f
// minimum sensor bar width in view, relative to half of the IR sensor area
#define MIN_SB_WIDTH 0.1f

// physical dimensions
// cm center to center of emitters
#define SB_WIDTH    19.5f
// half-width in cm of emitters
#define SB_DOT_WIDTH 2.25f
// half-height in cm of emitters (with some tolerance)
#define SB_DOT_HEIGHT 1.0f

#define SB_DOT_WIDTH_RATIO (SB_DOT_WIDTH / SB_WIDTH)
#define SB_DOT_HEIGHT_RATIO (SB_DOT_HEIGHT / SB_WIDTH)

// dots further out than these coords are allowed to not be picked up
// otherwise assume something's wrong
//#define SB_OFF_SCREEN_X 0.8f
//#define SB_OFF_SCREEN_Y (0.8f * HEIGHT)

// disable, may be doing more harm than good due to sensor pickup glitches
#define SB_OFF_SCREEN_X 0.0f
#define SB_OFF_SCREEN_Y 0.0f

// if a point is closer than this to one of the previous SB points
// when it reappears, consider it the same instead of trying to guess
// which one of the two it is
#define SB_SINGLE_NOGUESS_DISTANCE (100.0 * 100.0)

// width of the sensor bar in pixels at one meter from the Wiimote
#define SB_Z_COEFFICIENT 256.0f

// distance in meters from the center of the FOV to the left or right edge,
// when the wiimote is at one meter
#define WIIMOTE_FOV_COEFFICIENT 0.39f

#define SQUARED(x) ((x) * (x))
#define WMAX(x,y) ((x > y)?(x):(y))
#define WMIN(x,y) ((x < y)?(x):(y))

static void rotate_dots(struct fdot_t* in, struct fdot_t *out, int count,
                        float theta)
{
    float s, c;
    int i;

    if (theta == 0) {
        for (i = 0; i < count; ++i) {
            out[i].x = in[i].x;
            out[i].y = in[i].y;
        }
        return;
    } 

    s = sin(theta);
    c = cos(theta);

    for (i = 0; i < count; ++i) {
        out[i].x = (c * in[i].x) + (-s * in[i].y);
        out[i].y = (s * in[i].x) + (c * in[i].y);
    }
}

static void find_sensorbar(struct ir_t* ir)
{
    struct fdot_t dots[4];
    struct fdot_t acc_dots[4];
    struct sb_t cand;
    struct sb_t candidates[6];
    struct sb_t sb;

    fdot_t difference;

    int num_candidates = 0;

    int i;
    int j;
    int first, second;

    DEBUG_PRINT("IR: angle: %.05f\n",ir->roll);

    /* count visible dots and populate dots structure */
    /* dots[] is in -1..1 units for width */
    ir->num_dots = 0;
    for (i = 0; i < 4; i++) {
        if (ir->dot[i].visible) {
            dots[ir->num_dots].x = -(ir->dot[i].rx - 512.0f) / 512.0f;
            dots[ir->num_dots].y = (ir->dot[i].ry - 384.0f) / 512.0f;
            DEBUG_PRINT("IR: dot %d at (%d,%d) (%.03f,%.03f)\n",
                        ir->num_dots, ir->dot[i].rx,ir->dot[i].ry,
                        dots[ir->num_dots].x, dots[ir->num_dots].y);
            ir->num_dots++;
        }
    }

    DEBUG_PRINT("IR: found %d dots\n",ir->num_dots);

    // nothing to track
    if(ir->num_dots == 0) {
        if(ir->state != IR_STATE_DEAD)
            ir->state = IR_STATE_LOST;
        ir->ax = 0;
        ir->ay = 0;
        ir->distance = 0.0f;
        ir->raw_valid = 0;
        return;
    }

    /* ==== Find the Sensor Bar ==== */

    // first rotate according to accelerometer orientation
    rotate_dots(dots, acc_dots, ir->num_dots, ir->roll);
    if (ir->num_dots > 1) {
        DEBUG_PRINT("IR: locating sensor bar candidates\n");

        // iterate through all dot pairs
        for (first = 0; first < (ir->num_dots-1); first++) {
            for (second = (first+1); second < ir->num_dots; second++) {
                DEBUG_PRINT("IR: trying dots %d and %d\n", first, second);
                // order the dots leftmost first into cand
                // storing both the raw dots and the accel-rotated dots
                if (acc_dots[first].x > acc_dots[second].x) {
                    cand.dots[0] = dots[second];
                    cand.dots[1] = dots[first];
                    cand.acc_dots[0] = acc_dots[second];
                    cand.acc_dots[1] = acc_dots[first];
                } else {
                    cand.dots[0] = dots[first];
                    cand.dots[1] = dots[second];
                    cand.acc_dots[0] = acc_dots[first];
                    cand.acc_dots[1] = acc_dots[second];
                }
                difference.x = cand.acc_dots[1].x - cand.acc_dots[0].x;
                difference.y = cand.acc_dots[1].y - cand.acc_dots[0].y;

                // check angle
                if (fabsf(difference.y / difference.x) > MAX_SB_SLOPE)
                    continue;
                DEBUG_PRINT("IR: passed angle check\n");
                // rotate to the true sensor bar angle
                cand.off_angle = -atan2(difference.y, difference.x);
                cand.angle = cand.off_angle + ir->roll;
                rotate_dots(cand.dots, cand.rot_dots, 2, cand.angle);
                DEBUG_PRINT("IR: off_angle: %.02f, angle: %.02f\n",
                            cand.off_angle, cand.angle);
                // recalculate x distance - y should be zero now, so ignore it
                difference.x = cand.rot_dots[1].x - cand.rot_dots[0].x;

                // check distance
                if (difference.x < MIN_SB_WIDTH)
                    continue;

                // middle dot check. If there's another source somewhere in the
                // middle of this candidate, then this can't be a sensor bar
                for (i = 0; i < ir->num_dots; i++) {
                    float wadj, hadj;
                    struct fdot_t tdot;
                    if (i == first || i == second)
                        continue;
                    hadj = SB_DOT_HEIGHT_RATIO * difference.x;
                    wadj = SB_DOT_WIDTH_RATIO * difference.x;
                    rotate_dots(&dots[i], &tdot, 1, cand.angle);
                    if (((cand.rot_dots[0].x + wadj) < tdot.x) &&
                        ((cand.rot_dots[1].x - wadj) > tdot.x) &&
                        ((cand.rot_dots[0].y + hadj) > tdot.y) &&
                        ((cand.rot_dots[0].y - hadj) < tdot.y))
                        break;
                }
                // failed middle dot check
                if (i < ir->num_dots)
                    continue;
                DEBUG_PRINT("IR: passed middle dot check\n");

                cand.score = 1 / (cand.rot_dots[1].x - cand.rot_dots[0].x);

                // we have a candidate, store it
                DEBUG_PRINT("IR: new candidate %d\n", num_candidates);
                candidates[num_candidates++] = cand;
            }
        }
    }

    if (num_candidates == 0) {
        int closest = -1;
        int closest_to = 0;
        float best = 999.0f;
        float d;
        float dx[2];
        struct sb_t sbx[2];
        // no sensor bar candidates, try to work with a lone dot
        DEBUG_PRINT("IR: no candidates\n");
        switch (ir->state) {
            case IR_STATE_DEAD:
                DEBUG_PRINT("IR: we're dead\n");
                // we've never seen a sensor bar before, so we're screwed
                ir->ax = 0.0f;
                ir->ay = 0.0f;
                ir->distance = 0.0f;
                ir->raw_valid = 0;
                return;
            case IR_STATE_GOOD:
            case IR_STATE_SINGLE:
            case IR_STATE_LOST:
                DEBUG_PRINT("IR: trying to keep track of single dot\n");
                // try to find the dot closest to the previous sensor bar
                // position
                for (i = 0; i < ir->num_dots; i++) {
                    DEBUG_PRINT("IR: checking dot %d (%.02f, %.02f)\n",
                                i, acc_dots[i].x,acc_dots[i].y);
                    for (j = 0; j < 2; j++) {
                        DEBUG_PRINT("      to dot %d (%.02f, %.02f)\n",
                                    j, ir->sensorbar.acc_dots[j].x,
                                    ir->sensorbar.acc_dots[j].y);
                        d = SQUARED(acc_dots[i].x - ir->sensorbar.acc_dots[j].x);
                        d += SQUARED(acc_dots[i].y - ir->sensorbar.acc_dots[j].y);
                        if (d < best) {
                            best = d;
                            closest_to = j;
                            closest = i;
                        }
                    }
                }
                DEBUG_PRINT("IR: closest dot is %d to %d\n",
                            closest, closest_to);
                if (ir->state != IR_STATE_LOST ||
                    best < SB_SINGLE_NOGUESS_DISTANCE) {
                    // now work out where the other dot would be, in the acc
                    // frame
                    sb.acc_dots[closest_to] = acc_dots[closest];
                    sb.acc_dots[closest_to^1].x = (ir->sensorbar.acc_dots[closest_to^1].x
                                                   - ir->sensorbar.acc_dots[closest_to].x
                                                   + acc_dots[closest].x);
                    sb.acc_dots[closest_to^1].y = (ir->sensorbar.acc_dots[closest_to^1].y
                                                   - ir->sensorbar.acc_dots[closest_to].y
                                                   + acc_dots[closest].y);
                    // get the raw frame
                    rotate_dots(sb.acc_dots, sb.dots, 2, -ir->roll);
                    if ((fabsf(sb.dots[closest_to^1].x) < SB_OFF_SCREEN_X) &&
                        (fabsf(sb.dots[closest_to^1].y) < SB_OFF_SCREEN_Y)) {
                        // this dot should be visible but isn't, since the
                        // candidate section failed. fall through and try to
                        // pick out the sensor bar without previous information
                        DEBUG_PRINT("IR: dot falls on screen, falling through\n");
                    } else {
                        // calculate the rotated dots frame
                        // angle tends to drift, so recalculate
                        sb.off_angle = -atan2(sb.acc_dots[1].y - sb.acc_dots[0].y,
                                              sb.acc_dots[1].x - sb.acc_dots[0].x);
                        sb.angle = ir->sensorbar.off_angle + ir->roll;
                        rotate_dots(sb.acc_dots, sb.rot_dots, 2,
                                    ir->sensorbar.off_angle);
                        DEBUG_PRINT("IR: kept track of single dot\n");
                        break;
                    }
                } else {
                    DEBUG_PRINT("IR: lost the dot and new one is too far away\n");
                }
                // try to find the dot closest to the sensor edge
                DEBUG_PRINT("IR: trying to find best dot\n");
                for (i = 0; i < ir->num_dots; i++) {
                    d = WMIN(1.0f - fabsf(dots[i].x), HEIGHT - fabsf(dots[i].y));
                    if (d < best) {
                        best = d;
                        closest = i;
                    }
                }
                DEBUG_PRINT("IR: best dot: %d\n",closest);
                // now try it as both places in the sensor bar
                // and pick the one that places the other dot furthest off-screen
                for (i = 0; i < 2; i++) {
                    sbx[i].acc_dots[i] = acc_dots[closest];
                    sbx[i].acc_dots[i^1].x = (ir->sensorbar.acc_dots[i^1].x
                                              - ir->sensorbar.acc_dots[i].x
                                              + acc_dots[closest].x;
                    sbx[i].acc_dots[i^1].y = (ir->sensorbar.acc_dots[i^1].y
                                              - ir->sensorbar.acc_dots[i].y
                                              + acc_dots[closest].y;
                    rotate_dots(sbx[i].acc_dots, sbx[i].dots, 2, -ir->roll);
                    dx[i] = WMAX(fabsf(sbx[i].dots[i^1].x),
                                 fabsf(sbx[i].dots[i^1].y / HEIGHT));
                }
                if (dx[0] > dx[1]) {
                    DEBUG_PRINT("IR: dot is LEFT: %.02f > %.02f\n",
                                dx[0], dx[1]);
                    sb = sbx[0];
                } else {
                    DEBUG_PRINT("IR: dot is RIGHT: %.02f < %.02f\n",
                                dx[0], dx[1]);
                    sb = sbx[1];
                }
                // angle tends to drift, so recalculate
                sb.off_angle = -(atan2(sb.acc_dots[1].y - sb.acc_dots[0].y,
                                       sb.acc_dots[1].x - sb.acc_dots[0].x));
                sb.angle = ir->sensorbar.off_angle + ir->roll;
                rotate_dots(sb.acc_dots, sb.rot_dots, 2,
                            ir->sensorbar.off_angle);
                DEBUG_PRINT("IR: found new dot to track\n");
                break;
        }
        sb.score = 0;
        ir->state = IR_STATE_SINGLE;
    } else {
        int bestidx = 0;
        float best = 0.0f;
        DEBUG_PRINT("IR: finding best candidate\n");
        // look for the best candidate
        // for now, the formula is simple: pick the one with the smallest distance
        for (i = 0; i < num_candidates; i++) {
            if (candidates[i].score > best) {
                bestidx = i;
                best = candidates[i].score;
            }
        }
        DEBUG_PRINT("IR: best candidate: %d\n",bestidx);
        sb = candidates[bestidx];
        ir->state = IR_STATE_GOOD;
    }

    ir->raw_valid = 1;
    ir->ax = ((sb.rot_dots[0].x + sb.rot_dots[1].x) / 2) * 512.0;
    ir->ay = ((sb.rot_dots[0].y + sb.rot_dots[1].y) / 2) * 512.0;
    ir->sensorbar = sb;
    ir->distance = (sb.rot_dots[1].x - sb.rot_dots[0].x) * 512.0;
}

#define SMOOTH_IR_RADIUS 8.0f
#define SMOOTH_IR_SPEED 0.25f
#define SMOOTH_IR_DEADZONE 2.5f

void apply_ir_smoothing(struct ir_t *ir)
{
    float dx, dy, d, theta;

    DEBUG_PRINT("Smooth: OK (%.02f, %.02f) LAST (%.02f, %.02f) ",
                ir->ax, ir->ay, ir->sx, ir->sy);
    dx = ir->ax - ir->sx;
    dy = ir->ay - ir->sy;
    d = sqrtf(dx * dx + dy * dy);
    if (d > SMOOTH_IR_DEADZONE) {
        if (d < SMOOTH_IR_RADIUS) {
            DEBUG_PRINT("INSIDE\n");
            ir->sx += dx * SMOOTH_IR_SPEED;
            ir->sy += dy * SMOOTH_IR_SPEED;
        } else {
            DEBUG_PRINT("OUTSIDE\n");
            theta = atan2f(dy, dx);
            ir->sx = ir->ax - cosf(theta) * SMOOTH_IR_RADIUS;
            ir->sy = ir->ay - sinf(theta) * SMOOTH_IR_RADIUS;
        }
    } else {
        DEBUG_PRINT("DEADZONE\n");
    }
}

// max number of errors before cooked data drops out
#define ERROR_MAX_COUNT 8
// max number of glitches before cooked data updates
#define GLITCH_MAX_COUNT 5
// squared delta over which we consider something a glitch
#define GLITCH_DIST (150.0f * 150.0f)

void process_ir_data(struct ir_t* ir)
{
    float d;

    find_sensorbar(ir);

    if (ir->raw_valid) {
        ir->angle = ir->sensorbar.angle;
        ir->z = SB_Z_COEFFICIENT / ir->distance;
        if (ir->error_cnt >= ERROR_MAX_COUNT) {
            ir->sx = ir->ax;
            ir->sy = ir->ay;
            ir->glitch_cnt = 0;
        } else {
            d = SQUARED(ir->ax - ir->sx) + SQUARED(ir->ay - ir->sy);
            if (d > GLITCH_DIST) {
                if (ir->glitch_cnt > GLITCH_MAX_COUNT) {
                    apply_ir_smoothing(ir);
                    ir->glitch_cnt = 0;
                } else {
                    ir->glitch_cnt++;
                }
            } else {
                ir->glitch_cnt = 0;
                apply_ir_smoothing(ir);
            }
        }
        ir->smooth_valid = 1;
        ir->error_cnt = 0;
    } else {
        if (ir->error_cnt >= ERROR_MAX_COUNT) {
            ir->smooth_valid = 0;
        } else {
            ir->smooth_valid = 1;
            ir->error_cnt++;
        }
    }
}

void initialize_ir(struct ir_t* ir)
{
    memset(ir, 0, sizeof(*ir));
    ir->error_cnt = ERROR_MAX_COUNT;
}
