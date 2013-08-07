#!/usr/bin/env python

from fysom import Fysom
import servo_if
import time

class FSMClient(object):
    def run(self):
        self.fsm.do_action()
        

class FidoBrain(FSMClient):
    def __init__(self):
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
                              {'name': 'at_face', 'src': 'SeekingFace', 'dst': 'DroppingBall'},
                              {'name': 'dropped_ball', 'src': 'DroppingBall', 'dst': 'SeekingBall'}],
                          'callbacks': {
                              'onenterSeekingBall', self.enterSeekingBall,
                              'doSeekingBall', self.doSeekingBall,
                              'onenterTrackingBall', self.enterTrackingBall,
                              'doTrackingBall', self.doTrackingBall,
                              'onenterApproachingBall', self.enterApproachingBall,
                              'doApproachingBall', self.doApproachingball,
                              'onenterFinalApproach', self.enterFinalApproach,
                              'onenterPickingUpBall', self.enterPickingUpBall,
                              'onenterTurningAround', self.enterTurningAround,
                              'onenterSeekingFace', self.enterSeekingFace,
                              'doSeekingFace', self.doSeekingFace,
                              'onenterApproachingFace', self.enterApproachingFace,
                              'doApproachingFace', self.doApproachingFace,
                              'onenterDroppingBall', self.enterDroppingBall}})

    def enterSeekingBall(self, event):
        pass

    def doSeekingBall(self):
        pass

    def enterTrackingBall(self, event):
        pass

    def doTrackingBall(self):
        pass

    def enterApproachingBall(self, event):
        pass

    def doApproachingBall(self):
        pass

    def enterFinalApproach(self, event):
        pass

    def enterPickingUpBall(self, event):
        servo_if.grab_ball()
        self.fsm.have_ball()

    def enterTurningAround(self, event):
        pass

    def enterSeekingFace(self, event):
        pass

    def doSeekingFace(self):
        pass

    def enterApproachingFace(self, event):
        pass

    def doApproachingFace(self):
        pass

    def enterDroppingBall(self, event):
        servo_if.drop_ball()
        self.fsm.dropped_ball()


class FidoTail(FSMClient):
    def __init__(self):
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
                              'onenterStartingWagging': self.enterStartingWagging,
                              'onenterWaggingRight': self.enterWaggingRight,
                              'doWaggingRight': self.doWaggingRight,
                              'onenterWaggingLeft': self.enterWaggingLeft,
                              'doWaggingLeft': self.doWaggingLeft,
                              'onenterStopping': self.enterStopping}})

    def enterStartingWagging(self, event):
        self.delay = event.delay
        self.count = event.count
        self.fsm.started()
        
    def enterWaggingRight(self, event):
        servo_if.set_servo_reliably(servo_if.TAIL, servo_if.TAIL_RIGHT)
        self.timeout_time = time.time() + self.delay

    def doWaggingRight(self):
        if time.time() >= self.timeout_time:
            self.fsm.timeout()

    def enterWaggingLeft(self, event):
        servo_if.set_servo_reliably(servo_if.TAIL, servo_if.TAIL_LEFT)
        self.timeout_time = time.time() + self.delay

    def doWaggingLeft(self):
        if time.time() >= self.timeout_time:
            self.count -= 1
            if self.count > 0:
                self.fsm.timeout()
            else:
                self.fsm.stop_wagging()
            
    def enterStopping(self, event):
        servo_if.set_servo(servo_if.TAIL, servo_if.TAIL_CENTER)
        self.fsm.stopped()
