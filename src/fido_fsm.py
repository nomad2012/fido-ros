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
    def __init__(self, outputs):
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
                              {'name': 'at_face', 'src': 'SeekingFace', 'dst': 'DroppingBall'},
                              {'name': 'dropped_ball', 'src': 'DroppingBall', 'dst': 'SeekingBall'}],
                          'callbacks': {
                              'onenterSeekingBall': self._enterSeekingBall,
                              'doSeekingBall': self._doSeekingBall,
                              'onenterTrackingBall': self._enterTrackingBall,
                              'doTrackingBall': self._doTrackingBall,
                              'onenterApproachingBall': self._enterApproachingBall,
                              'doApproachingBall': self._doApproachingBall,
                              'onenterFinalApproach': self._enterFinalApproach,
                              'onenterPickingUpBall': self._enterPickingUpBall,
                              'onenterTurningAround': self._enterTurningAround,
                              'onenterSeekingFace': self._enterSeekingFace,
                              'doSeekingFace': self._doSeekingFace,
                              'onenterApproachingFace': self._enterApproachingFace,
                              'doApproachingFace': self._doApproachingFace,
                              'onenterDroppingBall': self._enterDroppingBall}})

    #
    # fsm action methods (private)
    #
    def _enterSeekingBall(self, event):
        pass

    def _doSeekingBall(self):
        pass

    def _enterTrackingBall(self, event):
        pass

    def _doTrackingBall(self):
        pass

    def _enterApproachingBall(self, event):
        pass

    def _doApproachingBall(self):
        pass

    def _enterFinalApproach(self, event):
        pass

    def _enterPickingUpBall(self, event):
        servo_if.grab_ball()
        self.fsm.have_ball()

    def _enterTurningAround(self, event):
        pass

    def _enterSeekingFace(self, event):
        pass

    def _doSeekingFace(self):
        pass

    def _enterApproachingFace(self, event):
        pass

    def _doApproachingFace(self):
        pass

    def _enterDroppingBall(self, event):
        servo_if.drop_ball()
        self.fsm.dropped_ball()


class FidoTail(FSMClient):
    def __init__(self, outputs):
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
