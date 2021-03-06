#!/usr/bin/env python

from fysom import Fysom
import random
import servo_if
import subprocess
import time


# CONSTANTS

NEAR_BALL_AREA = 4000
NEAR_FACE_AREA = 15000

def play_sound(filename):
    pipe = subprocess.Popen(['mpg123', filename])

class FSMClient(object):
    def run(self):
        self.fsm.do_action()
        
    def state(self):
        return self.fsm.current


class FidoBrain(FSMClient):
    def __init__(self, inputs, outputs):
        self.inputs = inputs
        self.outputs = outputs
        self.fsm = Fysom({'initial': 'SeekingBall',
                          'events': [
                              {'name': 'found_ball', 'src': 'SeekingBall', 'dst': 'TrackingBall'},
                              {'name': 'lost_ball', 'src': ['TrackingBall', 'ApproachingBall'], 'dst': 'SeekingBall'},
                              {'name': 'ball_on_ground', 'src': 'TrackingBall', 'dst': 'ApproachingBall'},
                              {'name': 'ball_above_ground', 'src': 'ApproachingBall', 'dst': 'TrackingBall'},
                              {'name': 'near_ball', 'src': 'ApproachingBall', 'dst': 'FinalApproach'},
                              {'name': 'at_ball', 'src': 'FinalApproach', 'dst': 'PickingUpBall'},
                              {'name': 'have_ball', 'src': 'PickingUpBall', 'dst': 'TurningAround'},
                              {'name': 'turned_around', 'src': 'TurningAround', 'dst': 'SeekingFace'},
                              {'name': 'check_for_face', 'src': 'SeekingFace', 'dst': 'CheckingForFace'},
                              {'name': 'no_face', 'src': 'CheckingForFace', 'dst': 'SeekingFace'},
                              {'name': 'found_face', 'src': 'CheckingForFace', 'dst': 'ApproachingFace'},
                              {'name': 'lost_face', 'src': 'ApproachingFace', 'dst': 'SeekingFace'},
                              {'name': 'at_face', 'src': ['SeekingFace', 'ApproachingFace'], 'dst': 'DroppingBall'},
                              {'name': 'dropped_ball', 'src': 'DroppingBall', 'dst': 'SeekingBall'},
                              {'name': 'posed_for_play', 'src': 'PosingForPlay', 'dst': 'SeekingBall'}],
                          'callbacks': {
                              'onenterSeekingBall': self._enterSeekingBall,
                              'doSeekingBall': self._doSeekingBall,
                              'onenterTrackingBall': self._enterTrackingBall,
                              'doTrackingBall': self._doTrackingBall,
                              'onenterApproachingBall': self._enterApproachingBall,
                              'doApproachingBall': self._doApproachingBall,
                              'onenterFinalApproach': self._enterFinalApproach,
                              'doFinalApproach': self._doFinalApproach,
                              'onenterPickingUpBall': self._enterPickingUpBall,
                              'doPickingUpBall': self._doPickingUpBall,
                              'onenterTurningAround': self._enterTurningAround,
                              'doTurningAround': self._doTurningAround,
                              'onenterSeekingFace': self._enterSeekingFace,
                              'doSeekingFace': self._doSeekingFace,
                              'onenterCheckingForFace': self._enterCheckingForFace,
                              'doCheckingForFace': self._doCheckingForFace,
                              'onenterApproachingFace': self._enterApproachingFace,
                              'doApproachingFace': self._doApproachingFace,
                              'onenterDroppingBall': self._enterDroppingBall,
                              'doDroppingBall': self._doDroppingBall,
                              'onenterPosingForPlay': self._enterPosingForPlay,
                              'doPosingForPlay': self._doPosingForPlay}})

    def should_find_ball(self):
        return self.fsm.current in ['SeekingBall', 'TrackingBall', 'ApproachingBall', 'FinalApproach']

    def should_find_face(self):
        return self.fsm.current in ['SeekingFace', 'CheckingForFace', 'ApproachingFace']

    #
    # fsm action methods (private)
    #
    def _enterSeekingBall(self, event):
        pass

    def _doSeekingBall(self):
        self.outputs['left_wheel'] = -100
        self.outputs['right_wheel'] = 100
        self.outputs['legs'] = servo_if.LEGS_UP
        self.outputs['neck'] = servo_if.NECK_START
        self.outputs['head'] = servo_if.HEAD_CENTER
        if self.inputs['ball_area'] > 50:
            self.fsm.found_ball()

    def _enterTrackingBall(self, event):
        pass

    def _doTrackingBall(self):
        #if we lose the ball, go back to seeking; if it's on the ground, approach it.
        if self.inputs['ball_area'] <= 40:
            self.fsm.lost_ball()
        elif self.inputs['ball_area'] >= 50 and self.inputs['ball_y'] >= 120 and self.outputs['neck'] < servo_if.NECK_CENTER:
            self.fsm.ball_on_ground()
        elif random.random() > 0.95:
            self.outputs['jaw'] = (servo_if.JAW_OPEN + servo_if.JAW_CLOSED_EMPTY) / 2
            play_sound("bark03.mp3")

    def _enterApproachingBall(self, event):
        self.outputs['legs'] = servo_if.LEGS_UP

    def _doApproachingBall(self):
        if self.inputs['ball_area'] <= 40:
            self.fsm.lost_ball()
        elif self.inputs['ball_y'] < 240 and self.outputs['neck'] > servo_if.NECK_CENTER:
            self.fsm.ball_above_ground()
        elif self.inputs['avg_ball_area'] > NEAR_BALL_AREA:
            #self.outputs['left_wheel'] = 0
            #self.outputs['right_wheel'] = 0
            self.fsm.near_ball()

    def _enterFinalApproach(self, event):
        self.last_ir_l = self.inputs['ir_l']
        self.last_ir_m = self.inputs['ir_m']
        self.last_ir_r = self.inputs['ir_r']
        
    def _doFinalApproach(self):
        ir_l = self.inputs['ir_l']
        ir_m = self.inputs['ir_m']
        ir_r = self.inputs['ir_r']
        fast_approach_speed = 140
        slow_approach_speed = 80
        rotate_speed = 70
        ir_detect_dist = 60
        if (ir_l < 60 and ir_m < 60) or (ir_m < 60 and ir_r < 60):
            # at least one IR sensor sees the ball; center it using the 3 IR sensors
            self.last_ir_l = ir_l
            self.last_ir_m = ir_m
            self.last_ir_r = ir_r
            if ir_m < ir_l and ir_m < ir_r:
                # centered; continue forward until it's at the right distance
                if ir_m > 20:
                    # approach ball fast
                    self.outputs['left_wheel'] = fast_approach_speed
                    self.outputs['right_wheel'] = fast_approach_speed
                elif ir_m > 11:
                    # approach ball
                    self.outputs['left_wheel'] = slow_approach_speed
                    self.outputs['right_wheel'] = slow_approach_speed
                elif ir_m < 10:
                    # too close, back up!
                    self.outputs['left_wheel'] = -slow_approach_speed
                    self.outputs['right_wheel'] = -slow_approach_speed
                else:
                    # close enough!
                    self.outputs['left_wheel'] = 0
                    self.outputs['right_wheel'] = 0
                    self.fsm.at_ball()
            elif ir_l < 11 or ir_r < 11:
                # too close to obstacle, back up!
                self.outputs['left_wheel'] = -fast_approach_speed
                self.outputs['right_wheel'] = -fast_approach_speed
            elif ir_l < ir_r:
                # rotate left
                self.outputs['left_wheel'] = -rotate_speed
                self.outputs['right_wheel'] = rotate_speed
            else:
                # rotate right
                self.outputs['left_wheel'] = rotate_speed
                self.outputs['right_wheel'] = -rotate_speed
        else:
            # no ir sensor sees ball; rotate in direction of IR that last saw it
            if self.last_ir_l < self.last_ir_r:
                # rotate left
                self.outputs['left_wheel'] = -rotate_speed
                self.outputs['right_wheel'] = rotate_speed
            else:
                # rotate right
                self.outputs['left_wheel'] = rotate_speed
                self.outputs['right_wheel'] = -rotate_speed


    def _enterPickingUpBall(self, event):
        self.outputs['left_wheel'] = 0
        self.outputs['right_wheel'] = 0
        self.pickup_timer = time.time() + 0.5

    def _doPickingUpBall(self):
        if time.time() >= self.pickup_timer:
            servo_if.grab_ball()
            self.fsm.have_ball()

    def _enterTurningAround(self, event):
        self.turn_timer = time.time() + 1.0

    def _doTurningAround(self):
        turn_speed = 250
        self.outputs['left_wheel'] = -turn_speed
        self.outputs['right_wheel'] = turn_speed
        if time.time() >= self.turn_timer:
            self.fsm.turned_around()
            
    def _enterSeekingFace(self, event):
        self.outputs['head'] = servo_if.HEAD_CENTER
        self.outputs['neck'] = servo_if.NECK_CENTER + 10
        servo_if.ramp_servo(servo_if.NECK, servo_if.NECK_CENTER + 10, 3)
        self.outputs['left_wheel'] = -50
        self.outputs['right_wheel'] = 50
        self.seekingFace_timer = time.time() + 0.2

    def _doSeekingFace(self):
        #self.outputs['head'] = servo_if.HEAD_CENTER
        #self.outputs['neck'] = servo_if.NECK_UP + 1 if self.outputs['neck'] < servo_if.NECK_UP else servo_if.NECK_UP - 1
        #self.outputs['left_wheel'] = -50
        #self.outputs['right_wheel'] = 50
        if time.time() >= self.seekingFace_timer:
            self.fsm.check_for_face()

    def _enterCheckingForFace(self, event):
        self.outputs['left_wheel'] = 0
        self.outputs['right_wheel'] = 0
        self.checkingFace_timer = time.time() + 1.0

    def _doCheckingForFace(self):
        if self.inputs['face_area'] > 0:
            self.fsm.found_face()
        elif time.time() >= self.checkingFace_timer:
            self.fsm.no_face()        

    def _enterApproachingFace(self, event):
        self.lost_face_timer = 0
        pass

    def _doApproachingFace(self):
        if self.inputs['face_area'] <= 0:
            self.lost_face_timer = self.lost_face_timer + 1
            if self.lost_face_timer > 2:
                self.fsm.lost_face()
                self.outputs['left_wheel'] = 0
                self.outputs['right_wheel'] = 0
        else:
            self.lost_face_timer = 0
            if self.inputs['face_area'] > NEAR_FACE_AREA:
                self.fsm.at_face()

    def _enterDroppingBall(self, event):
        self.outputs['left_wheel'] = 0
        self.outputs['right_wheel'] = 0

    def _doDroppingBall(self):
        servo_if.throw_ball()
        self.fsm.dropped_ball()

    def _enterPosingForPlay(self, event):
        servo_if.pose_for_play()
        self.outputs['legs'] = servo_if.LEGS_DOWN
        self.pose_timer = time.time() + 2.0

    def _doPosingForPlay(self):
        if time.time() >= self.pose_timer:
            self.outputs['legs'] = servo_if.LEGS_UP
            self.fsm.posed_for_play()


class FidoTail(FSMClient):
    def __init__(self, inputs, outputs):
        self.inputs = inputs
        self.outputs = outputs
        self.fsm = Fysom({'initial': 'Idle',
                          'events': [
                              {'name': 'start_wagging', 'src': ['Idle', 'Stopping'], 'dst': 'StartingWagging'},
                              {'name': 'started', 'src': 'StartingWagging', 'dst': 'WaggingRight'},
                              {'name': 'timeout', 'src': 'WaggingLeft', 'dst': 'WaggingRight'},
                              {'name': 'timeout', 'src': 'WaggingRight', 'dst': 'WaggingLeft'},
                              {'name': 'stop_wagging', 'src': ['WaggingLeft', 'WaggingRight'], 'dst': 'Stopping'},
                              {'name': 'stop_wagging', 'src': 'Idle', 'dst': 'Idle'},
                              {'name': 'stopped', 'src': 'Stopping', 'dst': 'Idle'}],
                          'callbacks': {
                              'onenterStartingWagging': self._enterStartingWagging,
                              'onenterWaggingRight': self._enterWaggingRight,
                              'doWaggingRight': self._doWaggingRight,
                              'onenterWaggingLeft': self._enterWaggingLeft,
                              'doWaggingLeft': self._doWaggingLeft,
                              'onenterStopping': self._enterStopping}})

    #
    # fsm action methods (private)
    #
    def _enterStartingWagging(self, event):
        self.delay = event.delay
        self.count = event.count
        self.fsm.started()
        
    def _enterWaggingRight(self, event):
        #servo_if.set_servo_reliably(servo_if.TAIL, servo_if.TAIL_RIGHT)
        self.outputs['tail'] = servo_if.TAIL_RIGHT
        self.timeout_time = time.time() + self.delay

    def _doWaggingRight(self):
        if time.time() >= self.timeout_time:
            self.fsm.timeout()

    def _enterWaggingLeft(self, event):
        #servo_if.set_servo_reliably(servo_if.TAIL, servo_if.TAIL_LEFT)
        self.outputs['tail'] = servo_if.TAIL_LEFT
        self.timeout_time = time.time() + self.delay

    def _doWaggingLeft(self):
        if time.time() >= self.timeout_time:
            self.count -= 1
            if self.count > 0:
                self.fsm.timeout()
            else:
                self.fsm.stop_wagging()
            
    def _enterStopping(self, event):
        #servo_if.set_servo(servo_if.TAIL, servo_if.TAIL_CENTER)
        self.outputs['tail'] = servo_if.TAIL_CENTER
        self.fsm.stopped()

    #
    # public interface methods
    #
    def start_wagging(self, delay=0.25, count=5):
        self.fsm.start_wagging(delay=delay, count=count)

    def stop_wagging(self):
        self.fsm.stop_wagging()

    def is_wagging(self):
        return self.fsm.current != 'Idle'
        
