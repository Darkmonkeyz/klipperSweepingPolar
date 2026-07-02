// Polar kinematics stepper pulse time generation
//
// Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <math.h> // sqrt
#include <stddef.h>
#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // __visible
#include "itersolve.h" // struct stepper_kinematics
#include "trapq.h" // move_get_coord

struct sweeping_polar_stepper {
    struct stepper_kinematics sk;
    double distfrombed;
};

static double
sweeping_polar_stepper_sweeparm_calc_position(struct stepper_kinematics *sk, struct move *m
                                   , double move_time)
{
    struct coord c = move_get_coord(m, move_time);
    //

    struct sweeping_polar_stepper *sps = container_of(sk, struct sweeping_polar_stepper, sk);
    double distfrombed = sps->distfrombed;
    double sweeparmangle = acos((c.x*c.x + c.y*c.y - 2*distfrombed*distfrombed)/(2*distfrombed*distfrombed));
    //
    return sweeparmangle;
}

static double
sweeping_polar_stepper_angle_calc_position(struct stepper_kinematics *sk, struct move *m
                                  , double move_time)
{
    struct coord c = move_get_coord(m, move_time);
    //
    struct sweeping_polar_stepper *sps = container_of(sk, struct sweeping_polar_stepper, sk);
    double distfrombed = sps->distfrombed;
    double sweeparmangle = acos((c.x*c.x + c.y*c.y - 2*distfrombed*distfrombed)/(2*distfrombed*distfrombed));
    double angle = atan2(c.y,c.x) - atan2((distfrombed*sin(sweeparmangle)), (distfrombed + distfrombed*cos(sweeparmangle)));
    //
    if (angle - sk->commanded_pos > M_PI)
        angle -= 2. * M_PI;
    else if (angle - sk->commanded_pos < -M_PI)
        angle += 2. * M_PI;
    return angle;
}

static void
sweeping_polar_stepper_angle_post_fixup(struct stepper_kinematics *sk)
{
    // Normalize the stepper_bed angle
    if (sk->commanded_pos < -M_PI)
        sk->commanded_pos += 2 * M_PI;
    else if (sk->commanded_pos > M_PI)
        sk->commanded_pos -= 2 * M_PI;
}

// struct stepper_kinematics * __visible
// polar_stepper_alloc(char type, double distfrombed)//added distfrombed
// {
//     struct stepper_kinematics *sk = malloc(sizeof(*sk));
//     memset(sk, 0, sizeof(*sk));
//     if (type == 'r') {
//         sk->calc_position_cb = sweeping_polar_stepper_sweeparm_calc_position;
//     } else if (type == 'a') {
//         sk->calc_position_cb = sweeping_polar_stepper_angle_calc_position;
//         sk->post_cb = sweeping_polar_stepper_angle_post_fixup;
//     }
//     sk->active_flags = AF_X | AF_Y;
//     //
//     sk->distfrombed = distfrombed;
//     //
//     return sk;
// }



//This is the structure for setting up each stepper in the sweeping polar stepper kinematics type. you can call it with type r to assign the stepper as a sweeping arm or you can assign it type 'a' to assign it the bed. This then allows the step solvers to pull from the logic above. 
struct stepper_kinematics * __visible
sweeping_polar_stepper_alloc(char type, double distfrombed)
{
    struct sweeping_polar_stepper *sps = malloc(sizeof(*sps));
    memset(sps, 0, sizeof(*sps));
    sps->distfrombed = distfrombed;
    if (type == 'r') {
        sps->sk.calc_position_cb = sweeping_polar_stepper_sweeparm_calc_position;
    } else if (type == 'a') {
        sps->sk.calc_position_cb = sweeping_polar_stepper_angle_calc_position;
        sps->sk.post_cb = sweeping_polar_stepper_angle_post_fixup;
    }
    sps->sk.active_flags = AF_X | AF_Y;
    return &sps->sk;
}
