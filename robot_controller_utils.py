import hebi
from math import pi
from time import sleep, time
import numpy as np
import threading
import sys
import trace

joint_state = {"base": 0, "shoulder": 0, "elbow": 0}

class KThread(threading.Thread):
  """A subclass of threading.Thread, with a kill() method."""
  def __init__(self, *args, **keywords):
    threading.Thread.__init__(self, *args, **keywords)
    self.killed = False

  def start(self):
    """Start the thread."""
    self.__run_backup = self.run
    self.run = self.__run      # Force the Thread to install our trace.
    threading.Thread.start(self)

  def __run(self):
    """Hacked run function, which installs the trace."""
    sys.settrace(self.globaltrace)
    self.__run_backup()
    self.run = self.__run_backup

  def globaltrace(self, frame, why, arg):
    if why == 'call':
      return self.localtrace
    else:
      return None

  def localtrace(self, frame, why, arg):
    if self.killed:
      if why == 'line':
        raise SystemExit()
    return self.localtrace

  def kill(self):
    self.killed = True


class Hebi3dofArm(object):
  def __init__(self):
    self.family_name = "Test Family"
    self.module_name = ["base", "shoulder", "elbow"]

  def initialize_arm(self):
    lookup = hebi.Lookup()
    # Wait 2 seconds for the module list to populate
    sleep(2.0)
    self.group = lookup.get_group_from_names([self.family_name], self.module_name)
    self.base = lookup.get_group_from_names([self.family_name], [self.module_name[0]])
    self.shoulder = lookup.get_group_from_names([self.family_name], [self.module_name[1]])
    self.elbow = lookup.get_group_from_names([self.family_name], [self.module_name[2]])

    if self.group is None:
      print('Group not found: Did you forget to set the module family and name above?')
      exit(1)

    self.group_feedback = hebi.GroupFeedback(self.group.size)
    self.base_feedback = hebi.GroupFeedback(self.base.size)
    self.shoulder_feedback = hebi.GroupFeedback(self.shoulder.size)
    self.elbow_feedback = hebi.GroupFeedback(self.elbow.size)
    if all(joint is None for joint in [self.group.get_next_feedback(reuse_fbk=self.group_feedback),
                                    self.base.get_next_feedback(reuse_fbk=self.base_feedback),
                                    self.shoulder.get_next_feedback(reuse_fbk=self.shoulder_feedback), 
                                    self.elbow.get_next_feedback(reuse_fbk=self.elbow_feedback)]):
      print('Error getting feedback.')
      exit(1)

    # Start logging in the background
    self.group.start_log('logs', mkdirs=True)

    self.base_command = hebi.GroupCommand(self.base.size)
    self.shoulder_command = hebi.GroupCommand(self.shoulder.size)
    self.elbow_command = hebi.GroupCommand(self.elbow.size)

    self.joints = {"base": ["base", self.base, self.base_feedback, self.base_command], "shoulder": ["shoulder", self.shoulder, self.shoulder_feedback, self.shoulder_command], "elbow": ["elbow", self.elbow, self.elbow_feedback, self.elbow_command]}      

  def rotate_joints(self, motor, angle):
    current_joint_name, joint, joint_feedback, joint_command = motor
    other_joints = [self.joints[joint] for joint in self.module_name if joint != current_joint_name]
    positions = np.zeros((joint.size, 2), dtype=np.float64)
    offset = [pi] * joint.size
    current_pos = joint_feedback.position
    positions[:, 0] = current_pos
    positions[:, 1] = current_pos + float(angle)*pi/180
    time_vector = [0, 3]
    trajectory = hebi.trajectory.create_trajectory(time_vector, positions)
    duration = trajectory.duration   
    start = time()
    t = time() - start
    joint_state[current_joint_name] = 1

    while t < duration:
        joint.get_next_feedback(reuse_fbk=joint_feedback)
        t = time() - start
        pos, vel, acc = trajectory.get_state(t)
        joint_command.position = pos
        joint_command.velocity = vel
        joint.send_command(joint_command)
    joint_state[current_joint_name] = 0

  def handle_joints(self):
    while True:
      for joint, state in joint_state.items():
        if not state:
          current_joint_name, joint, joint_feedback, joint_command = self.joints[joint]
          joint_command.position = joint_feedback.position
          joint.send_command(joint_command)

class StateMachine(object):
    def __init__(self):
        self.arm = Hebi3dofArm()
        self.states = {"HOME":[{'base': -30}, {'shoulder': -55}, {'elbow': 90}],
                "C1":[{'base': 45}, {'shoulder': 30}, {'elbow': 365}],
                "C2":[{'base': 70}, {'shoulder': 30}, {'elbow': 365}],
                "C3":[{'base': 95}, {'shoulder': 30}, {'elbow': 365}],
                "C4":[{'base': 120}, {'shoulder': 30}, {'elbow': 365}],
                "C5":[{'base': 145}, {'shoulder': 30}, {'elbow': 365}],
                "C6":[{'base': 170}, {'shoulder': 30}, {'elbow': 365}],
                "C7":[{'base': 195}, {'shoulder': 30}, {'elbow': 365}]}
    def go_home(self):
        print("Robot Started: State transition to HOME")
        base, shoulder, elbow = self.states["HOME"]
        self.arm.rotate_joints(self.arm.joints["shoulder"], shoulder["shoulder"])
        self.arm.rotate_joints(self.arm.joints["base"], base["base"])
        self.arm.rotate_joints(self.arm.joints["elbow"], elbow["elbow"])
    
    def state_transition(self, state):
        print("State transition from HOME to {}".format(state))
        base, shoulder, elbow = self.states[state]
        self.arm.rotate_joints(self.arm.joints["base"], base["base"])
        self.arm.rotate_joints(self.arm.joints["shoulder"], shoulder["shoulder"])
        self.arm.rotate_joints(self.arm.joints["elbow"], elbow["elbow"])        
        print("State transition from {} to HOME".format(state))
        self.arm.rotate_joints(self.arm.joints["shoulder"], -shoulder["shoulder"] - 10)
        self.arm.rotate_joints(self.arm.joints["base"], -base["base"] - 1.5)
        # self.arm.rotate_joints(self.arm.joints["elbow"], -elbow["elbow"])        

    def main(self):
        try:
            self.arm.initialize_arm()
            thread = KThread(target=self.arm.handle_joints,)
            thread.start()
            self.go_home()
            status = 1
            while status:
                print("Please enter which state to transition")
                s = input()
                if s == 'c':
                    status = 0
                else:
                    self.state_transition(s)
            thread.kill()
            self.arm.group.stop_log()
        except KeyboardInterrupt:
            thread.kill()
            sys.exit(0)