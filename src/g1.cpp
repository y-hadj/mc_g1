#include "g1.h"
#include "config.h"

#include <mc_rbdyn/RobotLoader.h>
#include <mc_rtc/constants.h>
#include <mc_rtc/logging.h>
#include <RBDyn/parsers/urdf.h>

namespace mc_robots
{

inline static std::string g1Variant(const std::string & variant)
{
  std::string fullName = "g1_" + variant;
  mc_rtc::log::info("G1RobotModule uses the G1 variant: '{}'", fullName);
  return fullName;
}

inline static bool has29DofUpperBody(const std::string & variant)
{
  return variant == "29dof" || variant == "29dof_no_hands";
}

G1RobotModule::G1RobotModule(const std::string & variant)
: RobotModule(mc_rtc::G1_DESCRIPTION_PATH, 
              g1Variant(variant),
              std::string(mc_rtc::G1_DESCRIPTION_PATH) + "/urdf/" + g1Variant(variant) + ".urdf")
{
  mc_rtc::log::success("G1RobotModule loaded with name: {}", name);
  rsdf_dir = std::string(mc_rtc::G1_DESCRIPTION_PATH) + "/rsdf";

  mc_rtc::log::success("G1RobotModule using URDF \"{}\"", urdf_path);
  mc_rtc::log::success("G1RobotModule using path \"{}\" for rsdf", rsdf_dir);

  // True if the robot has a fixed base, false otherwise
  bool fixed = false;
  // Makes all the basic initialization that can be done from an URDF file
  init(rbd::parsers::from_urdf_file(urdf_path, fixed));

  _ref_joint_order = {
      "left_hip_pitch_joint",      "left_hip_roll_joint",       "left_hip_yaw_joint",
      "left_knee_joint",           "left_ankle_pitch_joint",    "left_ankle_roll_joint",
      "right_hip_pitch_joint",     "right_hip_roll_joint",      "right_hip_yaw_joint",
      "right_knee_joint",          "right_ankle_pitch_joint",   "right_ankle_roll_joint",
      "waist_yaw_joint"};

  if(has29DofUpperBody(variant))
  {
    _ref_joint_order.push_back("waist_roll_joint");
    _ref_joint_order.push_back("waist_pitch_joint");
  }

  _ref_joint_order.insert(_ref_joint_order.end(), {
      "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
      "left_shoulder_yaw_joint",   "left_elbow_joint",          "left_wrist_roll_joint"});

  if(has29DofUpperBody(variant))
  {
    _ref_joint_order.push_back("left_wrist_pitch_joint");
    _ref_joint_order.push_back("left_wrist_yaw_joint");
  }

  _ref_joint_order.insert(_ref_joint_order.end(), {
      "right_shoulder_pitch_joint","right_shoulder_roll_joint", "right_shoulder_yaw_joint",
      "right_elbow_joint",         "right_wrist_roll_joint"});

  if(has29DofUpperBody(variant))
  {
    _ref_joint_order.push_back("right_wrist_pitch_joint");
    _ref_joint_order.push_back("right_wrist_yaw_joint");
  }

  using namespace mc_rtc::constants;
  _stance["left_hip_yaw_joint"] = {0.0};
  _stance["left_hip_roll_joint"] = {0.0};
  _stance["left_hip_pitch_joint"] = {-0.312};
  _stance["left_knee_joint"] = {0.669};
  _stance["left_ankle_pitch_joint"] = {-0.363};
  _stance["left_ankle_roll_joint"] = {0.0};
  _stance["right_hip_yaw_joint"] = {0.0};
  _stance["right_hip_roll_joint"] = {0.0};
  _stance["right_hip_pitch_joint"] = {-0.312};
  _stance["right_knee_joint"] = {0.669};
  _stance["right_ankle_pitch_joint"] = {-0.363};
  _stance["right_ankle_roll_joint"] = {0.0};
  _stance["waist_yaw_joint"] = {0.0};
  _stance["left_shoulder_pitch_joint"] = {0.2};
  _stance["left_shoulder_roll_joint"] = {0.2};
  _stance["left_shoulder_yaw_joint"] = {0.0};
  _stance["left_elbow_joint"] = {0.6};
  _stance["left_wrist_roll_joint"] = {0.0};
  _stance["right_shoulder_pitch_joint"] = {0.2};
  _stance["right_shoulder_roll_joint"] = {-0.2};
  _stance["right_shoulder_yaw_joint"] = {0.0};
  _stance["right_elbow_joint"] = {0.6};
  _stance["right_wrist_roll_joint"] = {0.0};

  if(has29DofUpperBody(variant))
  {
    _stance["waist_roll_joint"] = {0.0};
    _stance["waist_pitch_joint"] = {0.0};
    _stance["left_wrist_pitch_joint"] = {0.0};
    _stance["left_wrist_yaw_joint"] = {0.0};
    _stance["right_wrist_pitch_joint"] = {0.0};
    _stance["right_wrist_yaw_joint"] = {0.0};
  }

  _default_attitude = {{1., 0., 0., 0., 0., 0., 0.76}};

  // Sensors
  _bodySensors.emplace_back("Accelerometer", "torso_link",sva::PTransformd(Eigen::Vector3d(-0.03959, -0.00224, 0.13792)));
  _bodySensors.emplace_back("FloatingBase", "pelvis", sva::PTransformd::Identity());

  // Foot wrench sensors to bind mc_rtc force tasks
  _forceSensors.emplace_back("LeftFootForceSensor", "left_ankle_roll_link", sva::PTransformd::Identity());  // bc F/T sensors would normally sit in ankle
  _forceSensors.emplace_back("RightFootForceSensor", "right_ankle_roll_link", sva::PTransformd::Identity());


  _minimalSelfCollisions = {mc_rbdyn::Collision("torso_link", "left_shoulder_yaw_link", 0.02, 0.001, 0.),
                            mc_rbdyn::Collision("torso_link", "right_shoulder_yaw_link", 0.02, 0.001, 0.),
                            mc_rbdyn::Collision("torso_link", "left_elbow_link", 0.05, 0.03, 0.),
                            mc_rbdyn::Collision("torso_link", "right_elbow_link", 0.05, 0.03, 0.),
                            mc_rbdyn::Collision("pelvis_contour_link", "left_shoulder_yaw_link", 0.05, 0.03, 0.),
                            mc_rbdyn::Collision("pelvis_contour_link", "right_shoulder_yaw_link", 0.05, 0.03, 0.),
                            mc_rbdyn::Collision("pelvis_contour_link", "left_elbow_link", 0.05, 0.03, 0.),
                            mc_rbdyn::Collision("pelvis_contour_link", "right_elbow_link", 0.05, 0.03, 0.),
                            mc_rbdyn::Collision("left_hip_pitch_link", "right_hip_pitch_link", 0.02, 0.01, 0.),
                            mc_rbdyn::Collision("left_knee_link", "right_knee_link", 0.02, 0.01, 0.),
                            mc_rbdyn::Collision("left_ankle_pitch_link", "right_ankle_pitch_link", 0.02, 0.01, 0.),
                            mc_rbdyn::Collision("left_ankle_roll_link_0", "right_ankle_roll_link_0", 0.02, 0.01, 0.),
                            mc_rbdyn::Collision("left_ankle_pitch_link", "right_knee_link", 0.02, 0.01, 0.),
                            mc_rbdyn::Collision("right_ankle_pitch_link", "left_knee_link", 0.02, 0.01, 0.)};
  _commonSelfCollisions = _minimalSelfCollisions;

  // Default LIPM stabilizer configuration (for ismpc_walking)
  _lipmStabilizerConfig.leftFootSurface = "LeftFootCenter";
  _lipmStabilizerConfig.rightFootSurface = "RightFootCenter";
  _lipmStabilizerConfig.torsoBodyName = "torso_link";
  // CoM at the half-sitting stance (hardcoded for G1-Revo2)
  _lipmStabilizerConfig.comHeight = 0.69;
  _lipmStabilizerConfig.torsoPitch = 0;
  _lipmStabilizerConfig.comActiveJoints = {"Root",
                                           "left_hip_pitch_joint",
                                           "left_hip_roll_joint",
                                           "left_hip_yaw_joint",
                                           "left_knee_joint",
                                           "left_ankle_pitch_joint",
                                           "left_ankle_roll_joint",
                                           "right_hip_pitch_joint",
                                           "right_hip_roll_joint",
                                           "right_hip_yaw_joint",
                                           "right_knee_joint",
                                           "right_ankle_pitch_joint",
                                           "right_ankle_roll_joint"};
}

static mc_rbdyn::RobotModule * makeG1WithRevo2(const std::string & module_name)
{
  auto g1NoHands = std::make_shared<mc_robots::G1RobotModule>("29dof_no_hands");

  auto leftRevo2 = mc_rbdyn::RobotLoader::get_robot_module("Revo2_LeftHand");
  auto rightRevo2 = mc_rbdyn::RobotLoader::get_robot_module("Revo2_RightHand");

  if(!leftRevo2 || !rightRevo2)
  {
    mc_rtc::log::error("Failed to load Revo2 modules while creating {}", module_name);
    return nullptr;
  }

  auto g1Left = g1NoHands->connect(
      *leftRevo2,
      "left_tool_attach",
      "left_base_link",
      "",
      mc_rbdyn::RobotModule::ConnectionParameters{}
        .name(module_name)
        .bodySensorMapping({{"FloatingBase", "FloatingBase_LeftHand"}}));

  auto g1Both = g1Left.connect(
      *rightRevo2,
      "right_tool_attach",
      "right_base_link",
      "",
      mc_rbdyn::RobotModule::ConnectionParameters{}
        .name(module_name)
        .bodySensorMapping({{"FloatingBase", "FloatingBase_RightHand"}}));

  auto * module = new mc_rbdyn::RobotModule(g1Both);

  // Hand wrench sensors, added after the G1-Revo2 merge 
  // (bc the palm surfaces on Revo2 base links, which only exist after merge)
  module->_forceSensors.emplace_back("LeftHandForceSensor", "left_base_link", sva::PTransformd::Identity());
  module->_forceSensors.emplace_back("RightHandForceSensor", "right_base_link", sva::PTransformd::Identity());

  return module;
}

} // namespace mc_robots

extern "C"
{
  ROBOT_MODULE_API void MC_RTC_ROBOT_MODULE(std::vector<std::string> & names)
  {
    names = {"G1", "G1_23dof", "G1_29dof", "G1_no_hands", "G1_Revo2"};
  }
  ROBOT_MODULE_API void destroy(mc_rbdyn::RobotModule * ptr)
  {
    delete ptr;
  }
  ROBOT_MODULE_API mc_rbdyn::RobotModule * create(const std::string & n)
  {
    ROBOT_MODULE_CHECK_VERSION("G1")
    if(n == "G1" || n == "G1_23dof")
    {
      return new mc_robots::G1RobotModule("23dof");
    }
    else if(n == "G1_29dof")
    {
      return new mc_robots::G1RobotModule("29dof");
    }
    else if(n == "G1_no_hands")
    {
      return new mc_robots::G1RobotModule("29dof_no_hands");
    }
    else if(n == "G1_Revo2")
    {
      return mc_robots::makeG1WithRevo2("g1_29dof_revo2");
    }
    else
    {
      mc_rtc::log::error("G1 module cannot create an object of type {}", n);
      return nullptr;
    }
  }
}
