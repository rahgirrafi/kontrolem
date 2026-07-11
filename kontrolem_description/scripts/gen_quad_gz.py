#!/usr/bin/env python3
# Generate the Gazebo quadruped URDF for M6.2: same body as
# floating_quadruped.ros2_control.urdf, but with visual/collision/friction so
# Gazebo's contact solver has real feet + ground, IgnitionSystem for the 12
# joints, GzBaseStateSystem for base+contact, and the <gazebo> plugin that boots
# the controller_manager in-gz. Startup hold = the world's DetachableJoint weld
# (see quadruped.world.sdf); joint damping MUST stay near zero — DART swallows
# commanded joint forces entirely under high SDF damping (M6.2 finding 2).
#
# Usage: python3 gen_quad_gz.py ../urdf/floating_quadruped_gz.urdf
import sys

# leg -> (x, y) hip origin on the base
LEGS = {"FL": (0.2, 0.15), "FR": (0.2, -0.15), "RL": (-0.2, 0.15), "RR": (-0.2, -0.15)}
DAMP = 0.01      # CHAMP-parity: near-zero (implicit damping in DART blocks applied joint forces at high values; and the WBC model has none)
FRICTION = 0.2   # CHAMP-parity dry friction
POSTURE = {"hipx": 0.0, "hipy": 0.7, "knee": -1.4}

def leg_links_joints(leg, x, y):
    s = []
    s.append(f'''  <joint name="hipx_{leg}" type="revolute">
    <parent link="base_link"/><child link="hipx_{leg}_link"/>
    <origin xyz="{x} {y} 0" rpy="0 0 0"/><axis xyz="1 0 0"/>
    <limit effort="40" velocity="20" lower="-0.8" upper="0.8"/>
    <dynamics damping="{DAMP}" friction="{FRICTION}"/>
  </joint>
  <link name="hipx_{leg}_link"><inertial><origin xyz="0 0 0" rpy="0 0 0"/><mass value="0.3"/>
    <inertia ixx="3e-4" ixy="0" ixz="0" iyy="3e-4" iyz="0" izz="3e-4"/></inertial>
    <visual><geometry><sphere radius="0.03"/></geometry></visual></link>''')
    s.append(f'''  <joint name="hipy_{leg}" type="revolute">
    <parent link="hipx_{leg}_link"/><child link="thigh_{leg}"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 1 0"/>
    <limit effort="40" velocity="20" lower="-2.0" upper="2.0"/>
    <dynamics damping="{DAMP}" friction="{FRICTION}"/>
  </joint>
  <link name="thigh_{leg}"><inertial><origin xyz="0 0 -0.09" rpy="0 0 0"/><mass value="0.5"/>
    <inertia ixx="0.0015" ixy="0" ixz="0" iyy="0.0015" iyz="0" izz="0.0002"/></inertial>
    <visual><origin xyz="0 0 -0.09" rpy="0 0 0"/><geometry><cylinder radius="0.02" length="0.18"/></geometry></visual></link>''')
    s.append(f'''  <joint name="knee_{leg}" type="revolute">
    <parent link="thigh_{leg}"/><child link="calf_{leg}"/>
    <origin xyz="0 0 -0.18" rpy="0 0 0"/><axis xyz="0 1 0"/>
    <limit effort="40" velocity="20" lower="-2.6" upper="0.0"/>
    <dynamics damping="{DAMP}" friction="{FRICTION}"/>
  </joint>
  <link name="calf_{leg}"><inertial><origin xyz="0 0 -0.09" rpy="0 0 0"/><mass value="0.25"/>
    <inertia ixx="0.0008" ixy="0" ixz="0" iyy="0.0008" iyz="0" izz="0.0001"/></inertial>
    <visual><origin xyz="0 0 -0.09" rpy="0 0 0"/><geometry><cylinder radius="0.015" length="0.18"/></geometry></visual></link>''')
    s.append(f'''  <joint name="foot_{leg}_fixed" type="fixed">
    <parent link="calf_{leg}"/><child link="foot_{leg}"/><origin xyz="0 0 -0.18" rpy="0 0 0"/>
  </joint>
  <link name="foot_{leg}"><inertial><origin xyz="0 0 0" rpy="0 0 0"/><mass value="0.04"/>
    <inertia ixx="1e-5" ixy="0" ixz="0" iyy="1e-5" iyz="0" izz="1e-5"/></inertial>
    <visual><geometry><sphere radius="0.02"/></geometry></visual>
    <collision><geometry><sphere radius="0.02"/></geometry></collision></link>''')
    return "\n".join(s)

def ign_joint(name):
    p = POSTURE["hipx"] if name.startswith("hipx") else POSTURE["hipy"] if name.startswith("hipy") else POSTURE["knee"]
    return f'''    <joint name="{name}">
      <command_interface name="effort"/>
      <state_interface name="position"><param name="initial_value">{p}</param></state_interface>
      <state_interface name="velocity"><param name="initial_value">0.0</param></state_interface>
    </joint>'''

JOINTS = [f"{k}_{leg}" for leg in LEGS for k in ("hipx", "hipy", "knee")]
FEET = [f"foot_{leg}" for leg in LEGS]

parts = []
parts.append('<?xml version="1.0"?>')
parts.append('''<!-- GENERATED (gen_quad_gz.py) — floating quadruped for the M6.2 WBC-in-Gazebo
     validation. Same body as floating_quadruped.ros2_control.urdf, but with
     visual/collision + foot friction so Gazebo's contact/friction solver stands
     it (not pinned feet), IgnitionSystem for the 12 joints, and GzBaseStateSystem
     (kontrolem_gz) exporting the base pose/twist + contact <gpio> read straight
     from the ECM. A <gazebo> tag boots the controller_manager in-gz. The WBC
     (KontrolemController) + quad_stand_controllers.yaml are UNCHANGED. Startup
     hold = the world anchor's DetachableJoint weld (D11 analogue); joint damping
     stays near zero (high SDF damping makes DART swallow commanded forces).
     __CTRL_YAML__/__DESC_YAML__ are substituted by the launch.
     Regenerate with scripts/gen_quad_gz.py. -->''')
parts.append('<robot name="floating_quadruped">')
parts.append('''  <link name="base_link">
    <inertial><origin xyz="0 0 0" rpy="0 0 0"/><mass value="5.0"/>
      <inertia ixx="0.08" ixy="0" ixz="0" iyy="0.15" iyz="0" izz="0.18"/></inertial>
    <visual><geometry><box size="0.4 0.3 0.1"/></geometry></visual>
    <collision><geometry><box size="0.4 0.3 0.1"/></geometry></collision>
  </link>''')
for leg, (x, y) in LEGS.items():
    parts.append(leg_links_joints(leg, x, y))

# Foot friction (gz reads these <gazebo reference> surface params on URDF->SDF).
for foot in FEET:
    parts.append(f'''  <gazebo reference="{foot}">
    <mu1>1.0</mu1><mu2>1.0</mu2>
    <kp>1e6</kp><kd>100.0</kd>
    <maxContacts>1</maxContacts>
  </gazebo>''')

# ros2_control block 1: joints via IgnitionSystem.
parts.append('  <ros2_control name="floating_quadruped_gz" type="system">')
parts.append('    <hardware><plugin>ign_ros2_control/IgnitionSystem</plugin></hardware>')
parts.extend(ign_joint(j) for j in JOINTS)
parts.append('  </ros2_control>')

# ros2_control block 2: base + contact via GzBaseStateSystem (reads the ECM).
parts.append('''  <ros2_control name="base_state" type="system">
    <hardware>
      <plugin>kontrolem_gz/GzBaseStateSystem</plugin>
      <param name="model_name">floating_quadruped</param>
      <param name="base_link">base_link</param>
    </hardware>
    <gpio name="floating_base">
      <state_interface name="pose.position.x"/>
      <state_interface name="pose.position.y"/>
      <state_interface name="pose.position.z"/>
      <state_interface name="pose.orientation.x"/>
      <state_interface name="pose.orientation.y"/>
      <state_interface name="pose.orientation.z"/>
      <state_interface name="pose.orientation.w"/>
      <state_interface name="twist.linear.x"/>
      <state_interface name="twist.linear.y"/>
      <state_interface name="twist.linear.z"/>
      <state_interface name="twist.angular.x"/>
      <state_interface name="twist.angular.y"/>
      <state_interface name="twist.angular.z"/>
    </gpio>
    <gpio name="contact">
      <state_interface name="contact.foot_FL"/>
      <state_interface name="contact.foot_FR"/>
      <state_interface name="contact.foot_RL"/>
      <state_interface name="contact.foot_RR"/>
    </gpio>
  </ros2_control>''')

# Boot the controller_manager inside gz-sim. (The base is held during the
# spawn->activation gap by a DetachableJoint declared on the world's static anchor
# model, with THIS model's base_link as the child — see quadruped.world.sdf.)
parts.append('''  <gazebo>
    <plugin filename="ign_ros2_control-system"
            name="ign_ros2_control::IgnitionROS2ControlPlugin">
      <parameters>__CTRL_YAML__</parameters>
      <parameters>__DESC_YAML__</parameters>
      <controller_manager_name>controller_manager</controller_manager_name>
    </plugin>
  </gazebo>''')
parts.append('</robot>')

out = "\n".join(parts) + "\n"
with open(sys.argv[1], "w") as f:
    f.write(out)
print("wrote", sys.argv[1], "(%d joints, %d feet)" % (len(JOINTS), len(FEET)))
