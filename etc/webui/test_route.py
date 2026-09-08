"""Drive RobotLink's route state machine with the ROS layer stubbed out."""
import sys, types, math

def mod(name, **attrs):
    m = types.ModuleType(name)
    for k, v in attrs.items(): setattr(m, k, v)
    sys.modules[name] = m
    return m

class Any:
    def __init__(self, *a, **k): pass
    def __getattr__(self, n): return Any()
    def __call__(self, *a, **k): return Any()

class Msg:
    def __init__(self, **k):
        self.header = Any(); self.pose = Any(); self.poses = k.get('poses', [])
        for kk, vv in k.items(): setattr(self, kk, vv)

mod('rclpy', init=Any(), shutdown=Any(), spin=Any(), create_node=Any())
mod('rclpy.node', Node=object)
mod('rclpy.qos', QoSProfile=Any, DurabilityPolicy=Any(), ReliabilityPolicy=Any())
mod('rclpy.time', Time=Any)
mod('rclpy.action', ActionClient=Any)
mod('flask', Flask=Any, Response=Any, jsonify=Any(), request=Any(), send_from_directory=Any())
mod('waitress', serve=Any())
mod('tf2_ros', Buffer=Any, TransformListener=Any)
mod('action_msgs.msg', GoalStatus=types.SimpleNamespace(
    STATUS_SUCCEEDED=4, STATUS_CANCELED=5, STATUS_ABORTED=6))
mod('action_msgs')
class PoseStamped:
    def __init__(self): self.header = Any(); self.pose = Any()
mod('geometry_msgs.msg', PoseStamped=PoseStamped, PoseWithCovarianceStamped=Msg, Twist=Msg)
mod('geometry_msgs')
mod('explore_lite_msgs.msg', ExploreStatus=types.SimpleNamespace(EXPLORATION_COMPLETE='c'))
mod('explore_lite_msgs')
mod('nav2_msgs.action', NavigateToPose=Any, NavigateThroughPoses=types.SimpleNamespace(Goal=Msg))
mod('nav2_msgs.srv', SetInitialPose=Any)
mod('nav2_msgs')
mod('nav_msgs.msg', OccupancyGrid=Any); mod('nav_msgs')
mod('std_msgs.msg', Bool=Any); mod('std_msgs')

src = open('etc/webui/server.py').read()
ns = {'__name__': 'srv'}
exec(compile(src, 'server.py', 'exec'), ns)
RobotLink = ns['RobotLink']

class R(RobotLink):
    """Bypass the ROS constructor; keep the route logic. send_goal is
    stubbed so a test can decide how each waypoint's goal ends."""
    def __init__(self, wps):
        self.waypoints = [{'x': x, 'y': y, 'yaw': 0.0} for x, y in wps]
        self.route_active = False; self.route_loop = False
        self.route_idx = 0; self.route_passed = 0; self.route_error = None
        self._route_poses = []; self.sent = []; self._pose = None
        self.nav_state = 'idle'
    def pose(self): return self._pose
    def cancel_goal(self): self.nav_state = 'canceled'
    def send_goal(self, x, y, yaw):
        self.sent.append(self._route_poses.index(
            {'x': x, 'y': y, 'yaw': yaw}))
        return True, ""

fails = []
def check(label, got, want):
    ok = got == want
    print(f"    {'PASS' if ok else 'FAIL'}  {label}: {got}" + ("" if ok else f"  want {want}"))
    if not ok: fails.append(label)

def arrive(r):   # the current waypoint's goal succeeded
    r._route_on_goal_done('succeeded')

print("  === one-shot route visits every waypoint, in order ===")
r = R([(0,0),(1,0),(2,0)])
r._route_poses = list(r.waypoints); r.route_loop = False
r.route_active = True; r.route_idx = 0; r._route_send_current()
check("first goal", r.sent, [0])
arrive(r); check("then wp2", r.sent, [0,1])
arrive(r); check("then wp3", r.sent, [0,1,2])
arrive(r)
check("finishes after the last", r.route_active, False)
check("says so", r.route_error, "route complete")
check("counted them all", r.route_passed, 3)

print("  === looping route wraps and keeps going ===")
r2 = R([(0,0),(1,0),(2,0)])
r2._route_poses = list(r2.waypoints); r2.route_loop = True
r2.route_active = True; r2.route_idx = 0; r2._route_send_current()
for _ in range(5): arrive(r2)
check("wrapped twice", r2.sent, [0,1,2,0,1,2])
check("still running", r2.route_active, True)
check("passed keeps climbing", r2.route_passed, 5)

print("  === a waypoint it cannot reach stops the route and says which ===")
r3 = R([(0,0),(1,0),(2,0)])
r3._route_poses = list(r3.waypoints); r3.route_active = True
r3.route_idx = 0; r3._route_send_current()
arrive(r3)
r3._route_on_goal_done('aborted')
check("stopped", r3.route_active, False)
check("names the waypoint", r3.route_error, "could not reach waypoint 2")

print("  === a near miss still counts as reached ===")
rn = R([(0,0),(1,0),(2,0)])
rn._route_poses = list(rn.waypoints); rn.route_active = True
rn.route_idx = 0; rn._route_send_current()
rn._pose = {'x': 0.33, 'y': 0.0, 'yaw': 0.0}      # 0.33 m short of wp1
rn._route_on_goal_done('aborted')
check("carried on", rn.route_active, True)
check("counted it", rn.route_passed, 1)
check("moved to wp2", rn.sent, [0, 1])

print("  === but a real miss still stops the route ===")
rf = R([(0,0),(1,0),(2,0)])
rf._route_poses = list(rf.waypoints); rf.route_active = True
rf.route_idx = 0; rf._route_send_current()
rf._pose = {'x': -3.0, 'y': 0.0, 'yaw': 0.0}      # nowhere near
rf._route_on_goal_done('aborted')
check("stopped", rf.route_active, False)
check("named it", rf.route_error, "could not reach waypoint 1")

print("  === cancelling ends it quietly, not as a failure ===")
r4 = R([(0,0),(1,0)])
r4._route_poses = list(r4.waypoints); r4.route_active = True
r4._route_send_current(); r4._route_on_goal_done('canceled')
check("stopped", r4.route_active, False)
check("no error text", r4.route_error, None)

print("  === stop is idempotent and sends nothing further ===")
r5 = R([(0,0),(1,0),(2,0)])
r5._route_poses = list(r5.waypoints); r5.route_active = True
r5._route_send_current(); n = len(r5.sent)
r5.route_stop(); r5.route_stop()
r5._route_on_goal_done('succeeded')
check("inactive", r5.route_active, False)
check("no send after stop", len(r5.sent), n)

print("  === where a route begins ===")
r6 = R([(0,0),(1,0),(2,0)]); r6._route_poses = list(r6.waypoints)
r6.route_loop = True
r6._pose = {'x': 1.02, 'y': 0.0, 'yaw': 0.0}
check("loop, on wp1 -> next", r6._route_first_index(), 2)
r6._pose = {'x': 5.0, 'y': 5.0, 'yaw': 0.0}
check("loop, far -> nearest", r6._route_first_index(), 2)
r6.route_loop = False
check("one-shot, far -> 0", r6._route_first_index(), 0)
r6._pose = {'x': 0.05, 'y': 0.0, 'yaw': 0.0}
check("one-shot, on wp0 -> 1", r6._route_first_index(), 1)

print()
print("  RESULT:", "ALL PASS" if not fails else f"{len(fails)} FAILED: {fails}")
sys.exit(1 if fails else 0)
