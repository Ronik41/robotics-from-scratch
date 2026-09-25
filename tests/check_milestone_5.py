"""Live torque-control safety acceptance; Gazebo pose is a test-only oracle."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import statistics
import subprocess
import time
import xml.etree.ElementTree as ET
from gz.msgs10.double_v_pb2 import Double_V
import rclpy
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from check_milestone_3 import Probe as BaseProbe, require, ROOT


class Probe(BaseProbe):
    def __init__(self):
        super().__init__()
        self.destroy_publisher(self.pub)
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 1)
        self.last_command_stamp = -1
        self.state = self.driver = None
        self.trace = []
        self.create_subscription(String, '/firmware/state', self.on_state, 100)
        self.create_subscription(String, '/motor/driver_state', lambda m: setattr(self,'driver',json.loads(m.data)),100)
        self.estop = self.create_client(SetBool, '/safety/estop')
        self.reset = self.create_client(Trigger, '/safety/reset')

    def on_state(self,m):
        self.state=json.loads(m.data)
        self.trace.append(self.state)

    def command(self,v=0.,w=0.,age=0.,frame='base_link',axis=0.):
        m=TwistStamped()
        ns=round((self.clock-age)*1e9)
        if age == 0 and math.isfinite(v) and frame == 'base_link' and axis == 0:
            if ns <= self.last_command_stamp:
                return
            self.last_command_stamp = ns
        m.header.stamp.sec,m.header.stamp.nanosec=divmod(ns,10**9)
        m.header.frame_id=frame
        m.twist.linear.x=float(v); m.twist.linear.y=axis; m.twist.angular.z=float(w)
        self.pub.publish(m)

    def coast(self,seconds):
        start=self.clock
        self.wait(lambda:self.clock-start>=seconds,120)

    def call(self,client,req):
        require(client.wait_for_service(timeout_sec=10),'Safety service missing')
        future=client.call_async(req)
        self.wait(future.done,15)
        reply=future.result()
        return {'success':reply.success,'message':reply.message}

    def stopped(self,name):
        self.coast(.8)
        self.wait(lambda: self.truth is not None and abs(self.clock-self.truth[3])<.1)
        before=list(self.truth)
        self.coast(.5)
        after=list(self.truth)
        distance=math.dist(before[:2],after[:2])
        speed=max(abs(x) for x in self.driver['wheel_rad_s'])
        require(distance<.005,f'{name}: rover still moves: {distance} m')
        require(speed<.1,f'{name}: wheels still move: {speed} rad/s')
        return {'distance_over_half_second_m':distance,'wheel_speed_rad_s':speed,
                'firmware':dict(self.state),'driver':dict(self.driver)}


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--evidence',type=Path,required=True)
    parser.add_argument('--existing',action='store_true')
    args=parser.parse_args(); args.evidence.mkdir(parents=True,exist_ok=True)
    result={'status':'FAIL','checks':{}}
    process=probe=log=None
    rclpy.init()
    try:
        if not args.existing:
            log=(args.evidence/'launch.log').open('w')
            process=subprocess.Popen(['python3',str(ROOT/'scripts/rover_sim.py'),'--drive','--sensors','--firmware',
                                      '--evidence',str(args.evidence/'runtime')],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        probe=Probe()
        probe.wait(lambda:probe.state is not None and probe.driver is not None and probe.truth is not None and probe.clock>0,120)
        probe.drive(.3)
        require(probe.pub.get_subscription_count()==1,'cmd_vel must have exactly one firmware consumer')
        for name,v,w in [('forward',.2,0.),('reverse',-.2,0.),('turn',0.,.5)]:
            before=list(probe.truth); begin=len(probe.trace)
            probe.drive(3.5 if w else 2.5,v,w)
            after=list(probe.truth)
            samples=[s for s in probe.trace[begin:] if s['sim_time']>probe.clock-.6]
            require(len(samples)>3,'Insufficient control observations')
            target=[(v-w*.24)/.14,(v+w*.24)/.14]
            measured=[statistics.mean(s['measured_rad_s'][i] for s in samples) for i in range(2)]
            require(all(abs(a-b)<.15 for a,b in zip(measured,target)),f'{name}: tracking {measured} vs {target}')
            require(all(s['state']=='ACTIVE' for s in samples),f'{name}: control unexpectedly inhibited')
            if v:
                forward=(after[0]-before[0])*math.cos(before[2])+(after[1]-before[1])*math.sin(before[2])
                require(forward*v>0 and abs(forward)>.25,f'{name}: physical motion {forward}')
            else:
                require(after[2]-before[2]>.5,'Physical turn absent')
            result['checks'][name]={'measured_rad_s':measured,'target_rad_s':target,'truth_before':before,'truth_after':after}
            probe.drive(.6)

        begin=len(probe.trace)
        probe.drive(1.5,100.)
        samples=probe.trace[begin:]
        require(any(s['speed_saturated'] for s in samples),'Speed saturation missing')
        require(all(max(abs(x) for x in s['target_rad_s'])<=4.00001 for s in samples),'Wheel setpoint exceeds limit')
        require(all(max(abs(x) for x in s['effort_nm'])<=2 for s in samples),'Effort exceeds limit')
        # State samples can skip control updates, so compare against elapsed sim time.
        active=[s for s in samples if s['state']=='ACTIVE']
        for a,b in zip(active,active[1:]):
            require(max(abs(y-x) for x,y in zip(a['ramped_rad_s'],b['ramped_rad_s']))
                    <=4*(b['sim_time']-a['sim_time']+.04)+1e-6,'Acceleration ramp exceeds bound')
        result['checks']['saturation']={'max_target_rad_s':max(max(abs(x) for x in s['target_rad_s']) for s in samples),
                                        'max_effort_nm':max(max(abs(x) for x in s['effort_nm']) for s in samples)}
        start=probe.clock; before=list(probe.truth)
        probe.wait(lambda:probe.state['state']=='COMMAND_TIMEOUT',30)
        latency=probe.clock-start
        require(latency<.65,f'Command timeout late: {latency}')
        result['checks']['timeout']=probe.stopped('timeout')
        result['checks']['timeout']['observed_timeout_s']=latency
        result['checks']['timeout']['travel_to_stop_m']=math.dist(before[:2],probe.truth[:2])
        require(result['checks']['timeout']['travel_to_stop_m']<.5,'Timeout stopping distance exceeded nominal budget')

        probe.drive(1.,.2)
        estop_before=list(probe.truth)
        require(probe.call(probe.estop,SetBool.Request(data=True))['success'],'E-stop service failed')
        probe.drive(.5,.2) # command stream cannot override the latch
        require(probe.state['state']=='ESTOP' and probe.driver['estop_latched'],'E-stop not latched on both boundaries')
        result['checks']['estop']=probe.stopped('estop')
        result['checks']['estop']['travel_to_stop_m']=math.dist(estop_before[:2],probe.truth[:2])
        require(result['checks']['estop']['travel_to_stop_m']<.12,'E-stop stopping distance exceeded nominal budget')
        require(not probe.call(probe.reset,Trigger.Request())['success'],'Reset accepted while asserted')
        probe.call(probe.estop,SetBool.Request(data=False))
        require(not probe.call(probe.reset,Trigger.Request())['success'],'Reset accepted without fresh zero')
        probe.drive(.25)
        reset=probe.call(probe.reset,Trigger.Request())
        require(reset['success'],f'Reset refused: {reset}')
        probe.coast(.15)
        require(probe.state['state']=='WAIT_COMMAND','Reset replayed old command')
        result['checks']['reset']=reset
        probe.drive(1.,-.2)
        require(probe.state['state']=='ACTIVE' and not probe.driver['estop_latched'],'Post-reset motion refused')
        require(min(abs(x) for x in probe.state['measured_rad_s'])>.8,'Post-reset wheels did not move')

        for name,kwargs in [('stale',{'age':2.}),('future',{'age':-2.}),('nan',{'v':math.nan}),
                            ('wrong_frame',{'frame':'map'}),('unsupported_axis',{'axis':1.})]:
            probe.drive(.7,.2)
            count=probe.state['rejected_commands']
            probe.command(**kwargs)
            probe.wait(lambda:probe.state['rejected_commands']>count)
            require(probe.state['state']=='INVALID_COMMAND',f'{name}: not inhibited')
            result['checks'][name]=probe.stopped(name)

        # Loss of the actual sensor process must stop even with healthy commands.
        if not args.existing:
            sensor_pids=subprocess.check_output(['pgrep','-f','^python3 /workspace/src/rover_sensors/node.py'],text=True).split()
            require(len(sensor_pids)==1,'Cannot identify sensor process')
            sensor_pid=int(sensor_pids[0]); probe.drive(1.,.2)
            os.kill(sensor_pid,signal.SIGSTOP)
            try:
                probe.drive(.4,.2)
                require(probe.state['state']=='ENCODER_TIMEOUT','Missing encoder did not inhibit motion')
                result['checks']['encoder_suspended']=probe.stopped('encoder suspended')
            finally:
                os.kill(sensor_pid,signal.SIGCONT)
            probe.drive(.8,.2)
            require(probe.state['state']=='ACTIVE','Encoder stream did not recover with new commands')
            probe.drive(.3)

        # Verify independent driver watchdog while ROS firmware is suspended.
        # This is a fresh-container-only fault; desktop stays available for inspection.
        if not args.existing:
            pids=subprocess.check_output(['pgrep','-f','^python3 /workspace/src/rover_control/node.py'],text=True).split()
            require(len(pids)==1,'Cannot uniquely identify firmware process')
            pid=int(pids[0]); probe.drive(1.,.2)
            os.kill(pid,signal.SIGSTOP)
            try:
                start=probe.clock
                probe.wait(lambda:probe.driver['state']=='WATCHDOG',30)
                latency=probe.clock-start
                require(latency<.3,'Independent motor watchdog late')
                result['checks']['firmware_suspended']=probe.stopped('firmware suspended')
                result['checks']['firmware_suspended']['driver_watchdog_observed_s']=latency
                motor=probe.gz.advertise('/motor/effort',Double_V)
                # Driver applies a small-effort deadband independently of PID.
                start=probe.clock; last=-1.
                def low_effort():
                    nonlocal last
                    if probe.clock-last >= .03:
                        motor.publish(Double_V(data=[probe.clock,.02,.02,1.]))
                        last=probe.clock
                    return probe.clock-start>.4
                probe.wait(low_effort)
                require(probe.driver['state']=='DRIVE' and probe.driver['applied_nm']==[0.,0.],
                        'Motor deadband did not remove small effort')
                result['checks']['deadband']=dict(probe.driver)
                for name,data in [('driver_stale',[probe.clock-2.,1.,1.,1.]),
                                  ('driver_malformed',[probe.clock]),
                                  ('driver_overlimit',[probe.clock,3.,3.,1.])]:
                    rejected=probe.driver['rejected']
                    motor.publish(Double_V(data=data))
                    probe.wait(lambda:probe.driver['rejected']>rejected)
                    require(probe.driver['state']!='DRIVE',name+': motor accepted invalid frame')
                    result['checks'][name]=dict(probe.driver)
            finally:
                os.kill(pid,signal.SIGCONT)
            probe.coast(.3)

        topics=dict(probe.get_topic_names_and_types())
        require(not any('pose' in n or 'native_odom' in n or n=='/map' for n in topics),'Privileged pose entered ROS')
        subscriptions=probe.get_subscriber_names_and_types_by_node('sim_firmware','/')
        require({n for n,_ in subscriptions}=={'/clock','/cmd_vel','/wheel/encoders'},f'Unexpected firmware input: {subscriptions}')
        if not args.existing:
            model=ET.parse(args.evidence/'runtime/rover.sdf')
            plugins=[p.get('name') for p in model.findall('.//plugin')]
            require(plugins.count('rover::MotorDriver')==1 and not any('DiffDrive' in p for p in plugins),
                    'Model retains ideal drive bypass')
            result['model_plugins']=plugins
        result['subscriptions']=subscriptions
        result['checks']['final']=probe.stopped('final')
        result['status']='PASS'
    except Exception as exc:
        result['error']=str(exc)
        raise
    finally:
        if probe:
            probe.command()
            (args.evidence/'control-trace.json').write_text(json.dumps(probe.trace,indent=2)+'\n')
            probe.destroy_node()
        (args.evidence/'result.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result),flush=True)
        if process:
            os.killpg(process.pid,signal.SIGTERM)
            process.wait(timeout=20)
        if log: log.close()
        rclpy.shutdown()


if __name__=='__main__':
    main()
