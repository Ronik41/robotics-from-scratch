"""Deterministic firmware checks: explicit virtual clocks and quantized plant."""
import math
from pathlib import Path
import sys
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/rover_control'))
from core import Controller


class FirmwareTests(unittest.TestCase):
    def setUp(self):
        self.c = Controller()
        self.c.encoder(1., 1, [0,0], 2048, .02, 1.01, 1.)
        self.c.encoder(1.02, 2, [0,0], 2048, .02, 1.03, 1.02)

    def command(self, v=.2, w=0., t=1.04):
        return self.c.command(t, [v,0.,0.,0.,0.,w], 'base_link', t, t)

    def test_quantized_closed_loop_forward_reverse_turn(self):
        for v, w in [(.2,0.),(-.2,0.),(0.,.5)]:
            self.setUp()
            speed, position = [0.,0.], [0.,0.]
            trace = []
            for k in range(1,301):
                t = 1.02+k*.02
                self.command(v,w,t)
                self.c.encoder(t, k+2, [round(p*2048/(2*math.pi)) for p in position], 2048,.02,t+.01,t)
                for j in range(2):
                    u = self.c.effort[j]
                    applied = math.copysign(max(0.,abs(u)-.04),u)
                    speed[j] += (applied-.04*speed[j])/.12*.02
                    position[j] += speed[j]*.02
                trace.append(list(speed))
            for actual, target in zip(speed,self.c.target):
                self.assertAlmostEqual(actual,target,delta=.08)
            self.assertTrue(all(abs(x) <= 2 for x in self.c.effort))

    def test_saturation_acceleration_and_antiwindup(self):
        self.command(100.,1.)
        self.assertTrue(self.c.saturated)
        self.assertEqual(max(abs(x) for x in self.c.target),4.)
        previous = [0.,0.]
        for k in range(1,201):
            t=1.04+k*.02
            self.command(100.,1.,t)
            # Locked wheels: persistent large error must not wind up forever.
            self.c.encoder(t,k+2,[0,0],2048,.02 if k>1 else .04,t+.01,t)
            for a,b in zip(previous,self.c.ramped):
                self.assertLessEqual(abs(b-a), .160001 if k==1 else .080001)
            previous = list(self.c.ramped)
        self.assertTrue(all(abs(x)<=2 for x in self.c.effort))
        self.assertTrue(all(abs(x)<=1.2 for x in self.c.integral))
        self.assertTrue(any(abs(x)==2 for x in self.c.effort))

    def test_zero_command_brakes_and_clears_integral(self):
        self.command()
        self.c.integral=[.6,.6]
        self.command(0.,t=1.05)
        self.assertEqual(self.c.tick(1.05,1.05),'IDLE')
        self.assertEqual(self.c.integral,[0.,0.])
        self.assertEqual(self.c.effort,[0.,0.])

    def test_feedback_changes_effort_without_changing_target(self):
        self.command()
        self.c.encoder(1.04,3,[0,0],2048,.02,1.05,1.05)
        effort=list(self.c.effort); target=list(self.c.target)
        # Counts say wheels are already faster than the target. PID must oppose
        # the overspeed, despite receiving exactly the same desired velocity.
        self.c.encoder(1.06,4,[15,15],2048,.02,1.07,1.07)
        self.assertEqual(self.c.target,target)
        self.assertTrue(all(a<b for a,b in zip(self.c.effort,effort)))
        self.assertTrue(all(x<0 for x in self.c.effort))

    def test_timeout_stops_without_new_feedback_and_wall_clock(self):
        self.command()
        self.c.tick(1.55,1.55)
        self.assertEqual(self.c.state,'COMMAND_TIMEOUT')
        self.assertEqual(self.c.effort,[0.,0.])
        self.setUp(); self.command()
        self.assertEqual(self.c.tick(1.05,2.05),'COMMAND_TIMEOUT')

    def test_estop_latch_release_reset_no_replay(self):
        self.command()
        self.c.set_estop(True)
        self.command(t=1.05)
        self.assertEqual(self.c.tick(1.05,1.05),'ESTOP')
        self.assertFalse(self.c.reset(1.05,1.05))
        self.c.set_estop(False)
        self.assertFalse(self.c.reset(1.05,1.05))
        self.command(0.,t=1.06)
        self.assertTrue(self.c.reset(1.06,1.06))
        self.assertEqual(self.c.tick(1.06,1.06),'WAIT_COMMAND')
        self.assertEqual(self.c.effort,[0.,0.])
        self.command(t=1.07)
        self.assertEqual(self.c.tick(1.07,1.07),'ACTIVE')

    def test_reset_refused_while_moving_or_feedback_stale(self):
        self.c.set_estop(True); self.c.set_estop(False); self.command(0.)
        self.c.measured=[.2,0.]
        self.assertFalse(self.c.reset(1.04,1.04))
        self.c.measured=[0.,0.]
        self.assertFalse(self.c.reset(1.4,1.4))

    def test_stale_duplicate_future_nonfinite_wrong_frame_axes(self):
        bad = [(.1,[.2,0,0,0,0,0],'base_link'),
               (1.04,[.2,0,0,0,0,0],'base_link'),
               (2.,[.2,0,0,0,0,0],'base_link'),
               (1.05,[math.nan,0,0,0,0,0],'base_link'),
               (1.05,[math.inf,0,0,0,0,0],'base_link'),
               (1.05,[.2,0,0,0,0,0],'map'),
               (1.05,[.2,1,0,0,0,0],'base_link'),
               (1.05,[1e308,0,0,0,0,1e308],'base_link')]
        for stamp, values, frame in bad:
            self.setUp(); self.command()
            self.assertFalse(self.c.command(stamp,values,frame,1.06,1.06))
            self.assertEqual(self.c.command_stamp,1.04)
            self.assertEqual(self.c.tick(1.06,1.06),'INVALID_COMMAND')
            self.assertEqual(self.c.effort,[0.,0.])

    def test_missing_bad_and_reordered_encoder(self):
        self.command()
        self.assertEqual(self.c.tick(1.3,1.3),'ENCODER_TIMEOUT')
        for stamp,index,counts,cpr,period,metadata in [
            (1.02,2,[0,0],2048,.02,True),
            (1.06,3,[0,0],100,.04,True),
            (1.06,3,[99999,0],2048,.04,True),
            (1.06,3,[0,0],2048,.04,False)]:
            self.setUp(); self.command()
            self.assertFalse(self.c.encoder(stamp,index,counts,cpr,period,1.07,1.07,metadata))
            self.assertEqual(self.c.tick(1.07,1.07),'INVALID_ENCODER')
            self.assertEqual(self.c.effort,[0.,0.])


if __name__ == '__main__':
    unittest.main()
