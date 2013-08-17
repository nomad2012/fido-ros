#!/usr/bin/env python

from fysom import Fysom
import servo_if
import time

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
                              {'name': 'found_face', 'src': 'SeekingFace', 'dst': 'ApproachingFace'},
                              {'name': 'lost_face', 'src': 'ApproachingFace', 'dst': 'SeekingFace'},
                              {'name': 'at_face', 'src': ['SeekingFace', 'ApproachingFace'], 'dst': 'DroppingBall'},
                              {'name': 'dropped_ball', 'src': 'DroppingBall', 'dst': 'SeekingBall'}],
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
                              'onenterApproachingFace': self._enterApproachingFace,
                              'doApproachingFace': self._doApproachingFace,
                              'onenterDroppingBall': self._enterDroppingBall,
                              'doDroppingBall': self._doDroppingBall}})

    #
    # fsm action methods (private)
    #
    def _enterSeekingBall(self, event):
        pass

    def _doSeekingBall(self):
        self.outputs['left_wheel'] = 0
        self.outputs['right_wheel'] = 0
        self.outputs['neck'] = servo_if.NECK_START
        if self.inputs['ball_area'] > 50:
            self.fsm.found_ball()

    def _enterTrackingBall(self, event):
        pass

    def _doTrackingBall(self):
        #if we lose the ball, go back to seeking; if it's on the ground, approach it.
        if self.inputs['ball_area'] <= 0:
            self.fsm.lost_ball()
        elif self.inputs['ball_area'] >= 50 and self.outputs['neck'] < servo_if.NECK_CENTER:
            self.fsm.ball_on_ground()

    def _enterApproachingBall(self, event):
        pass

    def _doApproachingBall(self):
        if self.inputs['ball_area'] <= 0:
            self.fsm.lost_ball()
        elif self.inputs['ball_y'] < 240 and self.outputs['neck'] > servo_if.NECK_CENTER:
            self.fsm.ball_above_ground()
        elif self.inputs['ball_area'] > 7000:
            self.fsm.near_ball()

    def _enterFinalApproach(self, event):
        pass
        
    def _doFinalApproach(self):
        ir_l = self.inputs['ir_l']
        ir_m = self.inputs['ir_m']
        ir_r = self.inputs['ir_r']
        if (ir_l < 50 and ir_m < 50) or (ir_m < 50 and ir_r < 50):
            # at least one IR sensor sees the ball; center it using the 3 IR sensors
            if ir_m < ir_l and ir_m < ir_r:
                # centered; continue forward until it's at the right distance
                if ir_m > 20:
                    # approach ball fast
                    self.outputs['left_wheel'] = 60
                    self.outputs['right_wheel'] = 60
                elif ir_m > 11:
                    # approach ball
                    self.outputs['left_wheel'] = 40
                    self.outputs['right_wheel'] = 40
                else:
                    # close enough!
                    self.outputs['left_wheel'] = 0
                    self.outputs['right_wheel'] = 0
                    self.fsm.at_ball()
            elif ir_l < ir_r:
                # rotate left
                self.outputs['left_wheel'] = -80
                self.outputs['right_wheel'] = 80
            else:
                # rotate right
                self.outputs['left_wheel'] = 80
                self.outputs['right_wheel'] = -80
        else:
            # no ir sensor sees ball; rotate in direction head is pointing
            if self.outputs['head'] < servo_if.HEAD_CENTER:
                # rotate left
                self.outputs['left_wheel'] = -80
                self.outputs['right_wheel'] = 80
            else:
                # rotate right
                self.outputs['left_wheel'] = 80
                self.outputs['right_wheel'] = -80


    def _enterPickingUpBall(self, event):
        self.outputs['left_wheel'] = 0
        self.outputs['right_wheel'] = 0
        self.pickup_timer = time.time() + 2.0

    def _doPickingUpBall(self):
        if time.time() >= self.pickup_timer:
            servo_if.grab_ball()
            self.fsm.have_ball()

    def _enterTurningAround(self, event):
        self.turn_timer = time.time() + 1.0

    def _doTurningAround(self):
        self.outputs['left_wheel'] = -120
        self.outputs['right_wheel'] = 120
        if time.time() >= self.turn_timer:
            self.fsm.turned_around()
            
    def _enterSeekingFace(self, event):
        self.outputs['left_wheel'] = 0
        self.outputs['right_wheel'] = 0

    def _doSeekingFace(self):
        self.fsm.found_face()

    def _enterApproachingFace(self, event):
        pass

    def _doApproachingFace(self):
        self.fsm.at_face()

    def _enterDroppingBall(self, event):
        pass

    def _doDroppingBall(self):
        servo_if.throw_ball()
        self.fsm.dropped_ball()


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
        
