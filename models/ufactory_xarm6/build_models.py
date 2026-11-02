"""Generate the xArm6 MJCF models from one description of the arm.

    python models/ufactory_xarm6/build_models.py

Writes:
  models/ufactory_xarm6/xarm6.xml            the arm with the xArm gripper (fixed base)
  models/ufactory_xarm6/xarm6_nohand.xml     the arm alone, for the IK solver
  models/ufactory_xarm6/scene.xml            the arm, a floor and a cube
  models/stanford_tidybot/tidybot_xarm6.xml  the TidyBot base with the arm
  models/stanford_tidybot/scene_xarm6.xml    the base with the arm, a floor and a cube

Sources:
  arm kinematics (joint origins), joint limits and link inertials: UFACTORY's
  xarm_ros2 xarm_description (config/kinematics/default/xarm6_default_kinematics.yaml,
  urdf/xarm6/xarm6_robot_macro.xacro, config/link_inertial/xarm6_type6_HT_BR2.yaml);
  joint ranges as on the controller (J1/J4/J6 +-360 deg, J3 -225..11 deg);
  arm meshes: xarm_description/meshes/xarm6/visual (BSD-3-Clause, UFACTORY);
  the xArm gripper (bodies, linkage constraints, actuator) and its meshes:
  MuJoCo Menagerie's ufactory_xarm7 (BSD-3-Clause, UFACTORY), unchanged except
  for class names.
"""

from pathlib import Path

HERE = Path(__file__).resolve().parent
MODELS = HERE.parent

# where the arm's base sits on the TidyBot (the Kinova's mount: the arm plate's top)
MOUNT_POS = "0 0 0.335"
TCP_Z = 0.172                    # the gripper's TCP (between the fingertips) above its base
STANDALONE_POS = "0 0 0.12"      # the arm with its gripper on its own: on a 12 cm riser, its zero pose clear of the floor

ARM_DEFAULTS = """
    <default class="xarm6">
      <geom type="mesh" contype="0" conaffinity="0" group="2" material="xarm6/white"/>
      <joint axis="0 0 1" armature="0.1" frictionloss="1"/>
      <default class="xarm6/collision">
        <geom type="mesh" contype="1" conaffinity="1" group="3"/>
      </default>
      <default class="xarm6/large">
        <joint damping="10"/>
        <position kp="2000" kv="100"/>
      </default>
      <default class="xarm6/medium">
        <joint damping="5"/>
        <position kp="1000" kv="50"/>
      </default>
      <default class="xarm6/small">
        <joint damping="2"/>
        <position kp="500" kv="30"/>
      </default>
    </default>"""

GRIPPER_DEFAULTS = """
    <default class="xarm_gripper">
      <geom type="mesh" material="xarm6/black"/>
      <default class="xarm_gripper/spring_link">
        <joint range="0 0.85" stiffness="0.05" springref="2.62" damping="0.00125"/>
      </default>
      <default class="xarm_gripper/driver">
        <joint range="0 0.85" armature="0.005" damping="0.1" solreflimit="0.005 1"/>
      </default>
      <default class="xarm_gripper/follower">
        <joint range="0 0.85" solreflimit="0.005 1"/>
      </default>
      <default class="xarm_gripper/visual">
        <geom type="mesh" contype="0" conaffinity="0" group="2"/>
      </default>
      <default class="xarm_gripper/pad_box1">
        <geom type="box" friction="0.7" solimp="0.95 0.99 0.001" solref="0.004 1" mass="0" priority="1"
          size="0.015 0.002 0.0095" rgba="0.0 0.1 0.7 1" group="3"/>
      </default>
      <default class="xarm_gripper/pad_box2">
        <geom type="box" friction="0.6" solimp="0.95 0.99 0.001" solref="0.004 1" mass="0" priority="1"
          size="0.015 0.002 0.0095" rgba="0.0 0.5 0.5 1" group="3"/>
      </default>
    </default>"""

ARM_ASSETS = """
    <material name="xarm6/white" rgba="1 1 1 1"/>
    <material name="xarm6/gray" rgba="0.753 0.753 0.753 1"/>
    <material name="xarm6/black" rgba="0.149 0.149 0.149 1"/>
    <mesh name="xarm6/link_base" file="xarm6/link_base.stl"/>
    <mesh name="xarm6/link1" file="xarm6/link1.stl"/>
    <mesh name="xarm6/link2" file="xarm6/link2.stl"/>
    <mesh name="xarm6/link3" file="xarm6/link3.stl"/>
    <mesh name="xarm6/link4" file="xarm6/link4.stl"/>
    <mesh name="xarm6/link5" file="xarm6/link5.stl"/>
    <mesh name="xarm6/link6" file="xarm6/link6.stl"/>"""

GRIPPER_ASSETS = """
    <mesh name="xarm_gripper/base_link" file="xarm_gripper/base_link.stl"/>
    <mesh name="xarm_gripper/left_outer_knuckle" file="xarm_gripper/left_outer_knuckle.stl"/>
    <mesh name="xarm_gripper/left_finger" file="xarm_gripper/left_finger.stl"/>
    <mesh name="xarm_gripper/left_inner_knuckle" file="xarm_gripper/left_inner_knuckle.stl"/>
    <mesh name="xarm_gripper/right_outer_knuckle" file="xarm_gripper/right_outer_knuckle.stl"/>
    <mesh name="xarm_gripper/right_finger" file="xarm_gripper/right_finger.stl"/>
    <mesh name="xarm_gripper/right_inner_knuckle" file="xarm_gripper/right_inner_knuckle.stl"/>"""


def link(mesh):
    return (f'<geom mesh="xarm6/{mesh}"/>'
            f'<geom class="xarm6/collision" mesh="xarm6/{mesh}"/>')


def gripper(wrist_camera):
    # beside the gripper (off the jaws' axis), aimed at the TCP 23 deg off the tool axis
    cam = ('<camera name="wrist" pos="0.065 0 0.02" xyaxes="0 1 0 0.9194 0 0.3932" fovy="60" resolution="640 360"/>'
           if wrist_camera else "")
    return f"""
              <body name="xarm_gripper/base_link" quat="0 0 0 1" childclass="xarm_gripper">
                <inertial pos="-0.00065489 -0.0018497 0.048028" quat="0.997403 -0.0717512 -0.0061836 0.000477479"
                  mass="0.54156" diaginertia="0.000471093 0.000332307 0.000254799"/>
                <geom mesh="xarm_gripper/base_link" material="xarm6/white"/>
                {cam}
                <site name="pinch_site" pos="0 0 {TCP_Z}"/>
                <body name="left_outer_knuckle" pos="0 0.035 0.059098">
                  <inertial pos="0 0.021559 0.015181" quat="0.47789 0.87842 0 0" mass="0.033618"
                    diaginertia="1.9111e-05 1.79089e-05 1.90167e-06"/>
                  <joint name="left_driver_joint" axis="1 0 0" class="xarm_gripper/driver"/>
                  <geom mesh="xarm_gripper/left_outer_knuckle"/>
                  <body name="left_finger" pos="0 0.035465 0.042039">
                    <inertial pos="0 -0.016413 0.029258" quat="0.697634 0.115353 -0.115353 0.697634"
                      mass="0.048304" diaginertia="1.88037e-05 1.7493e-05 3.56792e-06"/>
                    <joint name="left_finger_joint" axis="-1 0 0" class="xarm_gripper/follower"/>
                    <geom class="xarm_gripper/visual" mesh="xarm_gripper/left_finger"/>
                    <geom class="xarm_gripper/pad_box1" name="left_finger_pad_1" pos="0 -0.024003 0.032"/>
                    <geom class="xarm_gripper/pad_box2" name="left_finger_pad_2" pos="0 -0.024003 0.050"/>
                  </body>
                </body>
                <body name="left_inner_knuckle" pos="0 0.02 0.074098">
                  <inertial pos="1.86601e-06 0.0220468 0.0261335" quat="0.664139 -0.242732 0.242713 0.664146"
                    mass="0.0230126" diaginertia="8.34216e-06 6.0949e-06 2.75601e-06"/>
                  <joint name="left_inner_knuckle_joint" axis="1 0 0" class="xarm_gripper/spring_link"/>
                  <geom mesh="xarm_gripper/left_inner_knuckle"/>
                </body>
                <body name="right_outer_knuckle" pos="0 -0.035 0.059098">
                  <inertial pos="0 -0.021559 0.015181" quat="0.87842 0.47789 0 0" mass="0.033618"
                    diaginertia="1.9111e-05 1.79089e-05 1.90167e-06"/>
                  <joint name="right_driver_joint" axis="-1 0 0" class="xarm_gripper/driver"/>
                  <geom mesh="xarm_gripper/right_outer_knuckle"/>
                  <body name="right_finger" pos="0 -0.035465 0.042039">
                    <inertial pos="0 0.016413 0.029258" quat="0.697634 -0.115356 0.115356 0.697634"
                      mass="0.048304" diaginertia="1.88038e-05 1.7493e-05 3.56779e-06"/>
                    <joint name="right_finger_joint" axis="1 0 0" class="xarm_gripper/follower"/>
                    <geom class="xarm_gripper/visual" mesh="xarm_gripper/right_finger"/>
                    <geom class="xarm_gripper/pad_box1" name="right_finger_pad_1" pos="0 0.024003 0.032"/>
                    <geom class="xarm_gripper/pad_box2" name="right_finger_pad_2" pos="0 0.024003 0.050"/>
                  </body>
                </body>
                <body name="right_inner_knuckle" pos="0 -0.02 0.074098">
                  <inertial pos="1.866e-06 -0.022047 0.026133" quat="0.66415 0.242702 -0.242721 0.664144"
                    mass="0.023013" diaginertia="8.34209e-06 6.0949e-06 2.75601e-06"/>
                  <joint name="right_inner_knuckle_joint" axis="-1 0 0" class="xarm_gripper/spring_link"/>
                  <geom mesh="xarm_gripper/right_inner_knuckle"/>
                </body>
              </body>"""


def arm(pos, hand, wrist_camera=False):
    """The arm's bodies from its base (at pos) to the flange; with hand, the gripper on it."""
    # without the hand: the site on the flange, turned as the gripper is (so its
    # orientation is the same frame the full model's pinch site reports)
    tool = gripper(wrist_camera) if hand else '<site name="pinch_site" quat="0 0 0 1"/>'
    return f"""
    <body name="xarm6/link_base" pos="{pos}" childclass="xarm6">
      <inertial pos="-0.021131 -0.0016302 0.056488" quat="0.696843 0.20176 0.10388 0.680376" mass="0.88556"
        diaginertia="0.00382023 0.00335282 0.00167725"/>
      {link("link_base")}
      <body name="xarm6/link1" pos="0 0 0.267">
        <inertial pos="0.00022 0.02951 -0.0124" mass="2.3814"
          fullinertia="0.0058562 0.0050316 0.003536652 -1.79e-05 3.55e-06 -0.000888336"/>
        <joint name="joint1" class="xarm6/large" range="-6.28319 6.28319"/>
        {link("link1")}
        <body name="xarm6/link2" quat="1 -1 0 0">
          <inertial pos="0.03881 -0.22783 0.03496" mass="2.2675"
            fullinertia="0.028315776 0.0058 0.0273447 0.005 0.00066546 -0.0045741"/>
          <joint name="joint2" class="xarm6/large" range="-2.059 2.0944"/>
          {link("link2")}
          <body name="xarm6/link3" pos="0.0535 -0.2845 0">
            <inertial pos="0.07041 0.11631 0.0107" mass="1.875"
              fullinertia="0.0063483 0.00379758 0.00595768 -0.0015397 0.00096858 0.00186567"/>
            <joint name="joint3" class="xarm6/medium" range="-3.927 0.19198"/>
            {link("link3")}
            <body name="xarm6/link4" pos="0.0775 0.3425 0" quat="1 -1 0 0">
              <inertial pos="-0.00018 0.01798 -0.02291" mass="1.3192"
                fullinertia="0.004896 0.00445694 0.00134332 -6.925e-06 -1.418e-05 -0.00023186"/>
              <joint name="joint4" class="xarm6/medium" range="-6.28319 6.28319"/>
              {link("link4")}
              <body name="xarm6/link5" quat="1 1 0 0">
                <inertial pos="0.0651 0.03096 0.00315" mass="1.33854"
                  fullinertia="0.00146378 0.00184192 0.002333524 -0.000450624 0.000284306 0.000130866"/>
                <joint name="joint5" class="xarm6/small" range="-1.69297 3.14159"/>
                {link("link5")}
                <body name="xarm6/link6" pos="0.076 0.097 0" quat="1 -1 0 0">
                  <inertial pos="0 -0.00677 -0.01098" mass="0.17"
                    fullinertia="9.3e-05 5.87e-05 0.000132 0 0 -3.6e-06"/>
                  <joint name="joint6" class="xarm6/small" range="-6.28319 6.28319"/>
                  <geom mesh="xarm6/link6" material="xarm6/gray"/>
                  <geom class="xarm6/collision" mesh="xarm6/link6"/>
                  {tool}
                </body>
              </body>
            </body>
          </body>
        </body>
      </body>
    </body>"""


ARM_ACTUATORS = """
    <position class="xarm6/large" name="joint1" joint="joint1" ctrlrange="-6.28319 6.28319" forcerange="-163.2 163.2"/>
    <position class="xarm6/large" name="joint2" joint="joint2" ctrlrange="-2.059 2.0944" forcerange="-163.2 163.2"/>
    <position class="xarm6/medium" name="joint3" joint="joint3" ctrlrange="-3.927 0.19198" forcerange="-81.6 81.6"/>
    <position class="xarm6/medium" name="joint4" joint="joint4" ctrlrange="-6.28319 6.28319" forcerange="-81.6 81.6"/>
    <position class="xarm6/small" name="joint5" joint="joint5" ctrlrange="-1.69297 3.14159" forcerange="-30.6 30.6"/>
    <position class="xarm6/small" name="joint6" joint="joint6" ctrlrange="-6.28319 6.28319" forcerange="-30.6 30.6"/>"""

GRIPPER_RULES = """
  <contact>
    <exclude body1="right_inner_knuckle" body2="right_outer_knuckle"/>
    <exclude body1="right_inner_knuckle" body2="right_finger"/>
    <exclude body1="left_inner_knuckle" body2="left_outer_knuckle"/>
    <exclude body1="left_inner_knuckle" body2="left_finger"/>
  </contact>

  <tendon>
    <fixed name="split">
      <joint joint="right_driver_joint" coef="0.5"/>
      <joint joint="left_driver_joint" coef="0.5"/>
    </fixed>
  </tendon>

  <equality>
    <connect anchor="0 0.015 0.015" body1="right_finger" body2="right_inner_knuckle" solref="0.005 1"/>
    <connect anchor="0 -0.015 0.015" body1="left_finger" body2="left_inner_knuckle" solref="0.005 1"/>
    <joint joint1="left_driver_joint" joint2="right_driver_joint" polycoef="0 1 0 0 0" solref="0.005 1"/>
  </equality>"""

# 0 open .. 255 closed: tendon target 0.333 * ctrl / 100, the drivers' 0.85 at 255
GRIPPER_ACTUATOR = """
    <general name="fingers_actuator" tendon="split" forcerange="-50 50" ctrlrange="0 255" biastype="affine"
      gainprm="0.333" biasprm="0 -100 -10"/>"""

# (radians) retract: the gripper pointing down, its TCP 0.25 m ahead of and above the
# arm's base; home: pointing forward, the TCP 0.45 m ahead, 0.40 m up (IK, no contacts
# with the TidyBot base)
RETRACT = "0 -0.84125 -0.57072 0 1.41197 0"
HOME = "0 -0.5236 -0.16232 0 -0.88663 0"

SCENE_EXTRAS = """
  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.1 0.1 0.1" specular="0 0 0"/>
    <rgba haze="0.15 0.25 0.35 1"/>
    <global azimuth="120" elevation="-20"/>
  </visual>

  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="3072"/>
    <texture type="2d" name="groundplane" builtin="checker" mark="edge" rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3"
      markrgb="0.8 0.8 0.8" width="300" height="300"/>
    <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="5 5"/>
  </asset>"""


def scene(model_name, include, statistic, cube_pos):
    return f"""<mujoco model="{model_name}">
  <include file="{include}"/>

  <statistic center="{statistic}" extent="1.0" meansize="0.05"/>
{SCENE_EXTRAS}

  <worldbody>
    <light pos="0 0 1.5" directional="true"/>
    <geom name="floor" size="0 0 0.05" type="plane" material="groundplane"/>
    <body name="cube" pos="{cube_pos}">
      <freejoint/>
      <geom type="box" size="0.02 0.02 0.02" rgba=".5 .7 .5 1" mass="0.1"/>
    </body>
  </worldbody>
</mujoco>
"""


HEADER = "<!-- Generated by models/ufactory_xarm6/build_models.py: edit that, then run it. -->\n"


def standalone(hand):
    name = "xarm6" if hand else "xarm6_nohand"
    n_hand_qpos = 6 if hand else 0
    keys = "".join(f'\n    <key name="{k}" qpos="{q}{" 0" * n_hand_qpos}" ctrl="{q}{" 0" if hand else ""}"/>'
                   for k, q in (("home", HOME), ("retract", RETRACT)))
    return HEADER + f"""<mujoco model="{name}">
  <compiler angle="radian" autolimits="true" meshdir="../assets"/>

  <option integrator="implicitfast"/>

  <default>{ARM_DEFAULTS}{GRIPPER_DEFAULTS if hand else ""}
    <site size="0.001" rgba="1 0 0 1" group="4"/>
  </default>

  <asset>{ARM_ASSETS}{GRIPPER_ASSETS if hand else ""}
  </asset>

  <worldbody>{arm(STANDALONE_POS if hand else "0 0 0", hand, wrist_camera=hand)}
  </worldbody>
{GRIPPER_RULES if hand else ""}
  <actuator>{ARM_ACTUATORS}{GRIPPER_ACTUATOR if hand else ""}
  </actuator>

  <keyframe>{keys}
  </keyframe>
</mujoco>
"""


def tidybot():
    """The TidyBot base (as in tidybot.xml) with the xArm6 and its gripper on the arm plate."""
    return HEADER + f"""<mujoco model="tidybot_xarm6">
  <compiler angle="radian" autolimits="true" meshdir="../assets"/>

  <option integrator="implicitfast"/>
  <option cone="elliptic" impratio="10"/>

  <default>
    <default class="base">
      <default class="base/visual">
        <geom type="mesh" contype="0" conaffinity="0" group="2"/>
      </default>
      <default class="base/collision">
        <geom type="mesh" group="3"/>
      </default>
    </default>{ARM_DEFAULTS}{GRIPPER_DEFAULTS}
    <site size="0.001" rgba="0.5 0.5 0.5 0.3" group="4"/>
  </default>

  <asset>
    <material name="black" rgba="0.11372549 0.11372549 0.10196078 1"/>
    <material name="gray" rgba="0.62745098 0.65098039 0.64705882 1"/>
    <material name="dark_gray" rgba="0.35294118 0.37647059 0.37254902 1"/>
    <mesh file="base/bumper.stl"/>
    <mesh file="base/bottom_plate.stl"/>
    <mesh file="base/body.stl"/>
    <mesh file="base/top_plate.stl"/>
    <mesh file="base/arm_plate.stl"/>{ARM_ASSETS}{GRIPPER_ASSETS}
  </asset>

  <worldbody>
    <body name="base_link">
      <inertial pos="0 0 0.035" mass="60" diaginertia="0.001 0.001 0.001"/>
      <joint name="joint_x" type="slide" axis="1 0 0"/>
      <joint name="joint_y" type="slide" axis="0 1 0"/>
      <joint name="joint_th"/>
      <geom class="base/visual" mesh="bumper" material="black"/>
      <geom class="base/visual" mesh="bottom_plate" material="dark_gray"/>
      <geom class="base/visual" mesh="body" material="gray"/>
      <geom class="base/visual" mesh="top_plate" material="dark_gray"/>
      <geom class="base/visual" mesh="arm_plate" material="gray"/>
      <geom class="base/collision" mesh="bumper"/>
      <geom class="base/collision" mesh="bottom_plate"/>
      <geom class="base/collision" mesh="body"/>
      <geom class="base/collision" mesh="top_plate"/>
      <geom class="base/collision" mesh="arm_plate"/>
      <camera name="base" pos="0.2525 0 0.335" euler="0 -0.7853981634 -1.5707963268" fovy="52.23384539951277"
        resolution="640 360"/>{arm(MOUNT_POS, hand=True, wrist_camera=True)}
    </body>
  </worldbody>

  <contact>
    <exclude body1="base_link" body2="xarm6/link_base"/>
  </contact>
{GRIPPER_RULES}

  <actuator>
    <position name="joint_x" joint="joint_x" kp="1000000" kv="50000"/>
    <position name="joint_y" joint="joint_y" kp="1000000" kv="50000"/>
    <position name="joint_th" joint="joint_th" kp="50000" kv="1000"/>{ARM_ACTUATORS}{GRIPPER_ACTUATOR}
  </actuator>
</mujoco>
"""


def main():
    out = {
        HERE / "xarm6.xml": standalone(hand=True),
        HERE / "xarm6_nohand.xml": standalone(hand=False),
        HERE / "scene.xml": HEADER + scene("xarm6 scene", "xarm6.xml", "0.2 0 0.3", "0.35 0 0.02"),
        MODELS / "stanford_tidybot" / "tidybot_xarm6.xml": tidybot(),
        MODELS / "stanford_tidybot" / "scene_xarm6.xml": HEADER + scene("tidybot_xarm6 scene", "tidybot_xarm6.xml",
                                                                         "0.25 0 0.6", "0.6 0 0.02"),
    }
    for path, text in out.items():
        path.write_text(text)
        print("wrote", path.relative_to(MODELS.parent))


if __name__ == "__main__":
    main()
