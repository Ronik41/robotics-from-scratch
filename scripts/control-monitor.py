"""Read-only desktop view of both safety boundaries, with stale-state warnings."""
import json
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Monitor(Node):
    def __init__(self):
        super().__init__('control_monitor')
        self.samples = {}
        for topic in ['/firmware/state','/motor/driver_state']:
            self.create_subscription(String,topic,lambda m,t=topic:self.receive(t,m),10)
        self.create_timer(.2,self.render)

    def receive(self,topic,message):
        self.samples[topic]=(json.loads(message.data),time.monotonic())

    def render(self):
        lines=['MILESTONE 5 | ENCODER PID + MOTOR SAFETY', '='*64]
        for topic in ['/firmware/state','/motor/driver_state']:
            lines.append(topic)
            if topic not in self.samples:
                lines.append('  WAITING FOR TELEMETRY'); continue
            state, received=self.samples[topic]
            age=time.monotonic()-received
            lines.append(f"  {state['state']:18}  e-stop latch: {state['estop_latched']}  age: {age:.1f}s"+
                         ('  STALE!' if age>1 else ''))
            for key in ['target_rad_s','measured_rad_s','effort_nm','applied_nm',
                        'rejected_commands','rejected_encoders','rejected']:
                if key in state:
                    value=state[key]
                    if isinstance(value,list): value=', '.join(f'{v: .3f}' for v in value)
                    lines.append(f'  {key:22} {value}')
            lines.append('')
        lines += ['k in teleop = brake; a movement key expires after 0.5 sim s.',
                  'E-stop: /safety/estop (SetBool true)',
                  'Reset: release input, fresh zero, stationary, /safety/reset',
                  'Simulation model only; physical safety needs hardware circuits.']
        print('\033[2J\033[H'+'\n'.join(lines),end='',flush=True)


rclpy.init()
node=Monitor()
try:
    rclpy.spin(node)
finally:
    node.destroy_node()
    rclpy.shutdown()
