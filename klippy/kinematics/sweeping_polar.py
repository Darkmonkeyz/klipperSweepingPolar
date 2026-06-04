# Code for handling the kinematics of polar robots
#
# Copyright (C) 2018-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, math
import stepper


def distance_to_center(p1, p2):
    ab_x = p2[0]-p1[0]
    ab_y = p2[1]-p1[1]
    ap_x = -p1[0]
    ap_y = -p1[1]

    ab_ap_dot_product = ab_x * ap_x + ab_y * ap_y
    ab_length = math.sqrt(ab_x ** 2 + ab_y ** 2)

    # Check if the projected point lies on the bounded line segment
    if ab_ap_dot_product <= 0:
        dist = math.sqrt(ap_x ** 2 + ap_y ** 2)
    elif ab_ap_dot_product >= ab_length ** 2:
        dist = math.sqrt(p2[0] ** 2 + p2[1] ** 2)
    else:
        dist = abs(ab_x * ap_y - ab_y * ap_x) / ab_length
    return dist


class SweepingPolarKinematics:
    def __init__(self, toolhead, config):
        # Setup axis steppers
        stepper_bed = stepper.PrinterStepper(config.getsection('stepper_bed'),
                                             units_in_radians=True)
        #
        stepper_arm = stepper.PrinterStepper(config.getsection('stepper_arm'), units_in_radians=True)

        self.distfrombed = distfrombed = config.getfloat('distfrombed', above=0.)
        #
        rail_z = stepper.LookupMultiRail(config.getsection('stepper_z'))
        stepper_bed.setup_itersolve('sweeping_polar_stepper_alloc', b'a', distfrombed)
        stepper_arm.setup_itersolve('sweeping_polar_stepper_alloc', b'r', distfrombed)
        rail_z.setup_itersolve('cartesian_stepper_alloc', b'z')
        self.rails = [rail_z]
        self.steppers = [stepper_bed, stepper_arm] + [ s for r in self.rails
                                          for s in r.get_steppers() ]
        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())
        # Setup boundary checks
        self.max_velocity, self.max_accel = toolhead.get_max_velocity()
        self.max_z_velocity = config.getfloat(
            'max_z_velocity', self.max_velocity, above=0.,
            maxval=self.max_velocity)
        self.max_z_accel = config.getfloat(
            'max_z_accel', self.max_accel, above=0., maxval=self.max_accel)
        self.v_rad_max = config.getfloat(
            'max_angular_velocity', above=0., default=0)
        self.limit_z = (1.0, -1.0)
        self.limit_xy2 = -1.
        #max_xy = self.rails[0].get_range()[1]
        min_z, max_z = self.rails[0].get_range()
        self.axes_min = toolhead.Coord((-distfrombed, -distfrombed, min_z))
        self.axes_max = toolhead.Coord((distfrombed, distfrombed, max_z))

    def get_steppers(self):
        return list(self.steppers)
    

    def calc_position(self, stepper_positions):
        bed_angle = stepper_positions[self.steppers[0].get_name()]
        arm_angle = stepper_positions[self.steppers[1].get_name()]
        z_pos = stepper_positions[self.rails[0].get_name()]
        #
        #
        #
        return [self.distfrombed*math.cos(bed_angle) + self.distfrombed*math.cos(bed_angle+arm_angle), self.distfrombed*math.sin(bed_angle) + self.distfrombed*math.sin(bed_angle+arm_angle),
                z_pos]
    
    def set_position(self, newpos, homing_axes):
        for s in self.steppers:
            s.set_position(newpos)

    def clear_homing_state(self, clear_axes):
        # XXX - homing not implemented
        pass


    def home(self, homing_state):
        # XXX - homing not implemented
        homing_state.set_axes([0, 1, 2])
        homing_state.set_homed_position([0., 0., 0.])

    def check_move(self, move):
        end_pos = move.end_pos
        xy2 = end_pos[0]**2 + end_pos[1]**2
        if xy2 > self.limit_xy2:
            if self.limit_xy2 < 0.:
                raise move.move_error("Must home axis first")
            raise move.move_error()
        if move.axes_d[2]:
            if end_pos[2] < self.limit_z[0] or end_pos[2] > self.limit_z[1]:
                if self.limit_z[0] > self.limit_z[1]:
                    raise move.move_error("Must home axis first")
                raise move.move_error()
            # Move with Z - update velocity and accel for slower Z axis
            z_ratio = move.move_d / abs(move.axes_d[2])
            move.limit_speed(self.max_z_velocity * z_ratio,
                             self.max_z_accel * z_ratio)
        # Slow down near center
        if move.axes_d[0] or move.axes_d[1]:
            if self.v_rad_max == 0:
                return
            min_dist = distance_to_center(move.start_pos[0:2],
                                              move.end_pos[0:2])
            if min_dist == 0:
                return
            v_angular = math.sqrt(move.max_cruise_v2) / min_dist
            if self.v_rad_max < v_angular:
                scale_radius = self.v_rad_max/v_angular
                move.limit_speed(self.max_velocity * scale_radius,
                                 self.max_accel * scale_radius)

    def get_status(self, eventtime):
        xy_home = "xy" if self.limit_xy2 >= 0. else ""
        z_home = "z" if self.limit_z[0] <= self.limit_z[1] else ""
        return {
            'homed_axes': xy_home + z_home,
            'axis_minimum': self.axes_min,
            'axis_maximum': self.axes_max,
        }

def load_kinematics(toolhead, config):
    return SweepingPolarKinematics(toolhead, config)
