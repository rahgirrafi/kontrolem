#!/usr/bin/env python3
# M7 — generate the Gazebo URDF for the Unitree Go2 WBC-in-Gazebo validation.
# Unlike gen_quad_gz.py (which builds the toy quadruped's body from scratch), this
# is an ADAPTER: it reads the vendored clean Go2 URDF (geometry+inertials only) and
# INJECTS, before </robot>, the tags Gazebo + gz_ros2_control need:
#   1. per-foot <gazebo reference> contact friction (Go2/CHAMP-proven mu/kp/kd);
#   2. <ros2_control name="go2_gz"> IgnitionSystem for the 12 joints (effort cmd,
#      position/velocity state, initial_value = the offline-proven nominal posture);
#   3. <ros2_control name="base_state"> kontrolem_gz/GzBaseStateSystem exporting the
#      base pose/twist (13) + contact (4) <gpio> read from the ECM;
#   4. the <gazebo> tag that boots the controller_manager in-gz.
# Startup hold = the world anchor's DetachableJoint weld (see go2.world.sdf); the Go2
# URDF carries no joint damping, which is what we want (high SDF damping makes DART
# swallow commanded joint forces — M6.2 finding). q_nom/base_height come from the
# offline proof (scratchpad go2_offline.py: base_height 0.2868, thigh 0.9, calf -1.8).
#
# It also strips the SolidWorks-export white material overrides so the GUI renders the
# meshes' own colors (see strip_white_materials).
#
# Usage: python3 gen_go2_gz.py ../urdf/go2.ros2_control.urdf ../urdf/go2_gz.urdf
import re
import sys

LEGS = ["FL", "FR", "RL", "RR"]
JOINTS = [f"{l}_{j}_joint" for l in LEGS for j in ("hip", "thigh", "calf")]
FEET = [f"{l}_foot" for l in LEGS]
# Nominal standing posture (offline-proven; feet grounded at base_height 0.2868).
POSTURE = {"hip": 0.0, "thigh": 0.9, "calf": -1.8}


def ign_joint(name):
    kind = name.split("_")[1]          # hip | thigh | calf
    p = POSTURE[kind]
    return f'''    <joint name="{name}">
      <command_interface name="effort"/>
      <state_interface name="position"><param name="initial_value">{p}</param></state_interface>
      <state_interface name="velocity"><param name="initial_value">0.0</param></state_interface>
    </joint>'''


def build_injection():
    parts = []
    # 1. Foot friction (Go2/CHAMP-proven soft contact in Fortress).
    for foot in FEET:
        parts.append(f'''  <gazebo reference="{foot}">
    <mu1>0.6</mu1><mu2>0.6</mu2>
    <kp>1e4</kp><kd>1.0</kd>
    <maxContacts>1</maxContacts>
  </gazebo>''')

    # 2. ros2_control block 1: the 12 joints via IgnitionSystem.
    parts.append('  <ros2_control name="go2_gz" type="system">')
    parts.append('    <hardware><plugin>ign_ros2_control/IgnitionSystem</plugin></hardware>')
    parts.extend(ign_joint(j) for j in JOINTS)
    parts.append('  </ros2_control>')

    # 3. ros2_control block 2: base + contact via GzBaseStateSystem (reads the ECM).
    contact_ifaces = "\n".join(
        f'      <state_interface name="contact.{f}"/>' for f in FEET)
    parts.append(f'''  <ros2_control name="base_state" type="system">
    <hardware>
      <plugin>kontrolem_gz/GzBaseStateSystem</plugin>
      <param name="model_name">go2</param>
      <param name="base_link">base</param>
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
{contact_ifaces}
    </gpio>
  </ros2_control>''')

    # 4. Boot the controller_manager inside gz-sim (base held by the world anchor's
    #    DetachableJoint until the test publishes on /go2/detach — see go2.world.sdf).
    parts.append('''  <gazebo>
    <plugin filename="ign_ros2_control-system"
            name="ign_ros2_control::IgnitionROS2ControlPlugin">
      <parameters>__CTRL_YAML__</parameters>
      <parameters>__DESC_YAML__</parameters>
      <controller_manager_name>controller_manager</controller_manager_name>
    </plugin>
  </gazebo>''')
    return "\n".join(parts)


def strip_white_materials(urdf):
    """Remove the SolidWorks-export material overrides <material name=""><color
    rgba="1 1 1 1"/></material>. They paint every visual white (overriding the .dae
    meshes' real colors -> colorless robot) and, all sharing an empty name, fight the
    meshes' embedded materials in Ogre (-> flicker in the GUI). CHAMP renders these
    same meshes with NO urdf material, letting the .dae colors show — do the same.
    Pinocchio ignores materials, so only the render (gz) URDF needs this."""
    pattern = r'\s*<material\s+name="">\s*<color\s+rgba="1 1 1 1"\s*/>\s*</material>'
    return re.sub(pattern, '', urdf)


def main(src, dst):
    with open(src) as f:
        urdf = f.read()
    if "</robot>" not in urdf:
        raise SystemExit("no </robot> in " + src)
    urdf = strip_white_materials(urdf)
    header = ('<!-- GENERATED (gen_go2_gz.py) from go2.ros2_control.urdf — adds\n'
              '     gz_ros2_control (IgnitionSystem joints), kontrolem_gz base/contact,\n'
              '     foot friction, and the in-gz controller_manager. Regenerate via\n'
              '     scripts/gen_go2_gz.py. __CTRL_YAML__/__DESC_YAML__ substituted by launch. -->\n')
    injected = build_injection() + "\n</robot>"
    out = urdf.replace("</robot>", injected)
    # drop the SolidWorks-only comment? keep; just prepend our marker after xml decl.
    out = out.replace('<?xml version="1.0" encoding="utf-8"?>\n',
                      '<?xml version="1.0" encoding="utf-8"?>\n' + header, 1)
    with open(dst, "w") as f:
        f.write(out)
    print(f"wrote {dst} ({len(JOINTS)} joints, {len(FEET)} feet)")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
