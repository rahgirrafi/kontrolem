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


def imu_sensor():
    # M8: a real IMU on the Go2's imu link (base-fixed). Feeds /imu/data ->
    # BaseEstimatorController. Small Gaussian noise (CHAMP-style) so the estimator is
    # exercised against realistic measurements, not a perfect oracle. 500 Hz = a fresh
    # sample every control tick (a real Go2 IMU runs several hundred Hz to 1 kHz), which
    # keeps the gyro-only attitude tight through a fast push transient.
    def axis(stddev):
        return (f'<noise type="gaussian"><mean>0.0</mean>'
                f'<stddev>{stddev}</stddev></noise>')
    return f'''  <gazebo reference="imu">
    <sensor name="imu_sensor" type="imu">
      <always_on>true</always_on>
      <update_rate>500</update_rate>
      <topic>imu/data</topic>
      <imu>
        <angular_velocity>
          <x>{axis(0.0003)}</x><y>{axis(0.0003)}</y><z>{axis(0.0003)}</z>
        </angular_velocity>
        <linear_acceleration>
          <x>{axis(0.017)}</x><y>{axis(0.017)}</y><z>{axis(0.017)}</z>
        </linear_acceleration>
      </imu>
    </sensor>
  </gazebo>'''


def preserve_foot_joint(foot):
    # Keep each foot as its OWN link in the SDF. By default sdformat's URDF->SDF lumps a
    # fixed-joint child into its parent (foot -> calf), renaming the foot collision to
    # "<calf>_fixed_joint_lump__<foot>_collision_N" and parenting the contact sensor to
    # the calf — fragile to reference. disableFixedJointLumping keeps FL_foot a real
    # link (collision "FL_foot_collision", sensor parented to FL_foot) so it matches
    # Pinocchio's foot frame. (The URDF's dont_collapse attribute alone is ignored by
    # this sdformat; the <gazebo> joint extension is the reliable control.)
    joint = f"{foot}_joint"
    return f'''  <gazebo reference="{joint}">
    <preserveFixedJoint>true</preserveFixedJoint>
    <disableFixedJointLumping>true</disableFixedJointLumping>
  </gazebo>'''


def contact_sensor(foot):
    # M8: a real per-foot contact sensor. With the foot preserved as its own link
    # (preserve_foot_joint), its single collision is named "<foot>_collision". The
    # ignition-gazebo-contact-system populates a ContactSensorData component on the
    # sensor entity; GzBaseStateSystem reads it from the ECM (real stance, replacing
    # M7's constant all-stance) and exports the same contact.<foot> gpio.
    return f'''  <gazebo reference="{foot}">
    <sensor name="{foot}_contact" type="contact">
      <always_on>true</always_on>
      <update_rate>500</update_rate>
      <contact>
        <collision>{foot}_collision</collision>
      </contact>
    </sensor>
  </gazebo>'''


def build_injection():
    parts = []
    # 0. Sensors (walking-ready: real IMU + real per-foot contact — the sensors a
    #    physical Go2 actually has, reused by every future estimator/gait). Feet are
    #    kept as their own links first, so the contact sensors/collisions name cleanly.
    parts.extend(preserve_foot_joint(f) for f in FEET)
    parts.append(imu_sensor())
    parts.extend(contact_sensor(f) for f in FEET)

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
      <!-- base_source ecm|estimate (M8): ecm = sim ground truth; estimate subscribes to
           the BaseEstimatorController's /base_odom (closed-loop sim-to-real). The launch
           substitutes __BASE_SOURCE__ from the base_source arg. Contact stays real. -->
      <param name="base_source">__BASE_SOURCE__</param>
      <param name="odom_topic">/base_odom</param>
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

    # 5. Ground-truth base odometry (M8 VALIDATION ONLY — not in the control loop).
    #    Full 3D world pose + body-frame twist on /base_truth_odom (same convention
    #    OdometryBaseBridge expects, proven in M6.3). The e2e compares the estimator's
    #    /base_odom against this to gate open-loop accuracy and log estimate-vs-truth.
    parts.append('''  <gazebo>
    <plugin filename="ignition-gazebo-odometry-publisher-system"
            name="ignition::gazebo::systems::OdometryPublisher">
      <dimensions>3</dimensions>
      <odom_frame>world</odom_frame>
      <robot_base_frame>base</robot_base_frame>
      <odom_topic>/base_truth_odom</odom_topic>
      <odom_publish_frequency>200</odom_publish_frequency>
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
