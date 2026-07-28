#include "drok_arm_kinematics/forward_kinematics.hpp"
#include "drok_arm_kinematics/robot_model.hpp"
#include "drok_arm_kinematics/robot_model_loader.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

double parseDouble(
  const char * text,
  const std::string & name)
{
  try {
    std::size_t parsed_length = 0;

    const std::string value_string(text);

    const double value =
      std::stod(
      value_string,
      &parsed_length);

    if (
      parsed_length !=
      value_string.size() ||
      !std::isfinite(value))
    {
      throw std::runtime_error(
              "Invalid numeric value.");
    }

    return value;

  } catch (const std::exception &) {
    throw std::runtime_error(
            "Invalid numerical value for " +
            name + ": " + text);
  }
}

double readEnvironmentDouble(
  const char * name,
  const double default_value)
{
  const char * text =
    std::getenv(name);

  if (text == nullptr) {
    return default_value;
  }

  const double value =
    parseDouble(text, name);

  if (value < 0.0) {
    throw std::runtime_error(
            std::string(name) +
            " must not be negative.");
  }

  return value;
}

std::vector<const drok_arm_kinematics::JointModel *>
movableJoints(
  const drok_arm_kinematics::RobotModel & model)
{
  std::vector<
    const drok_arm_kinematics::JointModel *> joints;

  joints.reserve(
    static_cast<std::size_t>(
      model.movable_joint_count));

  for (const auto & joint : model.chain) {
    if (
      joint.type == "revolute" ||
      joint.type == "continuous" ||
      joint.type == "prismatic")
    {
      joints.push_back(&joint);
    }
  }

  return joints;
}

void applyJointLimits(
  std::vector<double> & q,
  const std::vector<
    const drok_arm_kinematics::JointModel *> & joints)
{
  if (q.size() != joints.size()) {
    throw std::runtime_error(
            "Joint vector size mismatch.");
  }

  for (std::size_t index = 0;
    index < q.size();
    ++index)
  {
    const auto & joint =
      *joints[index];

    if (
      joint.type == "continuous" ||
      !joint.has_limit)
    {
      continue;
    }

    q[index] =
      std::clamp(
      q[index],
      joint.lower,
      joint.upper);
  }
}

double minimumJointLimitMargin(
  const std::vector<double> & q,
  const std::vector<
    const drok_arm_kinematics::JointModel *> & joints)
{
  double minimum_margin =
    std::numeric_limits<double>::infinity();

  for (std::size_t index = 0;
    index < q.size();
    ++index)
  {
    const auto & joint =
      *joints[index];

    if (
      joint.type == "continuous" ||
      !joint.has_limit)
    {
      continue;
    }

    minimum_margin =
      std::min(
      minimum_margin,
      std::min(
        q[index] - joint.lower,
        joint.upper - q[index]));
  }

  return minimum_margin;
}

double uprightAngle(
  const Eigen::Vector3d & current_up,
  const Eigen::Vector3d & target_up)
{
  const double cosine =
    std::clamp(
    current_up.dot(target_up),
    -1.0,
    1.0);

  return std::acos(cosine);
}

void printUsage()
{
  std::cerr
    << "Usage:\n"
    << "  solve_ik_upright "
    << "<robot_geometry.yaml> "
    << "<x> <y> <z> "
    << "<q1> <q2> <q3> <q4> <q5> <q6>\n\n"
    << "Constraint:\n"
    << "  TCP position = target XYZ\n"
    << "  TCP local +Z = world +Z\n"
    << "  TCP yaw is free\n";
}

}  // namespace

int main(
  int argc,
  char ** argv)
{
  if (argc != 11) {
    printUsage();
    return 1;
  }

  try {
    const std::filesystem::path geometry_path =
      argv[1];

    const Eigen::Vector3d target_position(
      parseDouble(argv[2], "x"),
      parseDouble(argv[3], "y"),
      parseDouble(argv[4], "z"));

    std::vector<double> initial_q(6);

    for (std::size_t index = 0;
      index < initial_q.size();
      ++index)
    {
      initial_q[index] =
        parseDouble(
        argv[index + 5],
        "q" + std::to_string(index + 1));
    }

    const auto model =
      drok_arm_kinematics::RobotModelLoader::
      loadFromYaml(geometry_path);

    if (model.movable_joint_count != 6) {
      throw std::runtime_error(
              "solve_ik_upright requires "
              "exactly six movable joints.");
    }

    const auto joints =
      movableJoints(model);

    if (joints.size() != 6) {
      throw std::runtime_error(
              "Could not extract six movable joints.");
    }

    const drok_arm_kinematics::ForwardKinematics
      forward_kinematics(model);

    /*
     * Task:
     *
     * y(q) =
     * [
     *   TCP position,
     *   TCP local-Z direction in world
     * ]
     *
     * yd =
     * [
     *   target position,
     *   world +Z
     * ]
     *
     * The upright direction has only two independent
     * degrees of constraint, so this is a rank-5 task.
     */

    const Eigen::Vector3d target_up =
      Eigen::Vector3d::UnitZ();

    /*
     * Upright orientation has only two independent
     * constraints.
     *
     * Construct an orthonormal basis B whose columns
     * span the plane perpendicular to target_up.
     *
     * Upright error:
     *
     *   e_u = B^T (u_d - u(q))
     *
     * Therefore the task dimension is exactly:
     *
     *   3 position + 2 upright = 5
     */
    const Eigen::Vector3d basis_reference =
      (
      std::abs(target_up.z()) < 0.9 ?
      Eigen::Vector3d::UnitZ() :
      Eigen::Vector3d::UnitX());

    const Eigen::Vector3d tangent_1 =
      (
      basis_reference -
      target_up *
      target_up.dot(basis_reference)
      ).normalized();

    const Eigen::Vector3d tangent_2 =
      target_up.cross(tangent_1).normalized();

    Eigen::Matrix<double, 3, 2> upright_basis;

    upright_basis.col(0) = tangent_1;
    upright_basis.col(1) = tangent_2;

    const std::size_t maximum_iterations = 2000;

    const double position_tolerance =
      readEnvironmentDouble(
      "DROK_UPRIGHT_POSITION_TOLERANCE",
      1.0e-5);

    const double upright_tolerance =
      readEnvironmentDouble(
      "DROK_UPRIGHT_ANGLE_TOLERANCE",
      1.0e-5);

    const double damping =
      readEnvironmentDouble(
      "DROK_UPRIGHT_DAMPING",
      1.0e-2);

    const double numerical_delta =
      readEnvironmentDouble(
      "DROK_UPRIGHT_NUMERICAL_DELTA",
      1.0e-6);

    const double maximum_joint_step =
      readEnvironmentDouble(
      "DROK_UPRIGHT_MAXIMUM_JOINT_STEP",
      0.05);

    const double position_weight =
      readEnvironmentDouble(
      "DROK_UPRIGHT_POSITION_WEIGHT",
      1.0);

    const double upright_weight =
      readEnvironmentDouble(
      "DROK_UPRIGHT_AXIS_WEIGHT",
      0.5);

    /*
     * Secondary null-space terms.
     *
     * The first term keeps the result near the waypoint seed.
     * The second term gently pushes limited joints toward
     * the center of their ranges.
     */
    const double seed_gain =
      readEnvironmentDouble(
      "DROK_UPRIGHT_SEED_GAIN",
      0.10);

    /*
     * The limit barrier is inactive in the normal region.
     * It activates only when a limited joint enters the
     * configured soft-margin zone.
     */
    const double joint_limit_soft_margin =
      readEnvironmentDouble(
      "DROK_UPRIGHT_LIMIT_SOFT_MARGIN",
      0.261799388);

    const double joint_limit_barrier_gain =
      readEnvironmentDouble(
      "DROK_UPRIGHT_LIMIT_BARRIER_GAIN",
      0.02);

    /*
     * Limit the total null-space motion applied during
     * one numerical iteration. This prevents yaw freedom
     * from creating a large posture jump.
     */
    const double maximum_secondary_step =
      readEnvironmentDouble(
      "DROK_UPRIGHT_MAX_SECONDARY_STEP",
      0.002);

    if (
      damping <= 0.0 ||
      numerical_delta <= 0.0 ||
      maximum_joint_step <= 0.0 ||
      position_weight <= 0.0 ||
      upright_weight <= 0.0 ||
      joint_limit_soft_margin <= 0.0 ||
      maximum_secondary_step <= 0.0)
    {
      throw std::runtime_error(
              "Upright IK numerical parameters "
              "must be positive.");
    }

    std::vector<double> q =
      initial_q;

    applyJointLimits(
      q,
      joints);

    bool success = false;
    std::size_t iterations =
      maximum_iterations;

    double final_position_error =
      std::numeric_limits<double>::infinity();

    double final_upright_error =
      std::numeric_limits<double>::infinity();

    for (std::size_t iteration = 0;
      iteration < maximum_iterations;
      ++iteration)
    {
      const Eigen::Matrix4d current_transform =
        forward_kinematics.compute(q);

      const Eigen::Vector3d current_position =
        current_transform.block<3, 1>(0, 3);

      const Eigen::Matrix3d current_rotation =
        current_transform.block<3, 3>(0, 0);

      /*
       * gripper_tcp local +Z expressed in world.
       */
      const Eigen::Vector3d current_up =
        current_rotation.col(2);

      final_position_error =
        (target_position - current_position).norm();

      final_upright_error =
        uprightAngle(
        current_up,
        target_up);

      if (
        final_position_error <=
        position_tolerance &&
        final_upright_error <=
        upright_tolerance)
      {
        success = true;
        iterations = iteration;
        break;
      }

      Eigen::Matrix<double, 5, 1> error;

      error.head<3>() =
        position_weight *
        (target_position - current_position);

      error.tail<2>() =
        upright_weight *
        upright_basis.transpose() *
        (target_up - current_up);

      Eigen::MatrixXd jacobian(
        5,
        static_cast<Eigen::Index>(q.size()));

      for (std::size_t index = 0;
        index < q.size();
        ++index)
      {
        std::vector<double> perturbed_q =
          q;

        double perturbation =
          numerical_delta;

        const auto & joint =
          *joints[index];

        if (
          joint.has_limit &&
          joint.type != "continuous" &&
          q[index] + perturbation >
          joint.upper)
        {
          perturbation =
            -numerical_delta;
        }

        perturbed_q[index] +=
          perturbation;

        applyJointLimits(
          perturbed_q,
          joints);

        const Eigen::Matrix4d perturbed_transform =
          forward_kinematics.compute(
          perturbed_q);

        const Eigen::Vector3d perturbed_position =
          perturbed_transform.block<3, 1>(
          0, 3);

        const Eigen::Vector3d perturbed_up =
          perturbed_transform.block<3, 3>(
          0, 0).col(2);

        jacobian.block<3, 1>(
          0,
          static_cast<Eigen::Index>(index)) =
          position_weight *
          (perturbed_position - current_position) /
          perturbation;

        jacobian.block<2, 1>(
          3,
          static_cast<Eigen::Index>(index)) =
          upright_weight *
          upright_basis.transpose() *
          (perturbed_up - current_up) /
          perturbation;
      }

      const Eigen::Matrix<double, 5, 5>
        regularized_matrix =
        jacobian * jacobian.transpose() +
        damping * damping *
        Eigen::Matrix<double, 5, 5>::Identity();

      const Eigen::MatrixXd pseudo_inverse =
        jacobian.transpose() *
        regularized_matrix.ldlt().solve(
        Eigen::Matrix<double, 5, 5>::Identity());

      Eigen::VectorXd delta_q =
        pseudo_inverse * error;

      /*
       * One degree of null space remains because yaw
       * around world Z is unconstrained.
       */
      const Eigen::MatrixXd null_space =
        Eigen::MatrixXd::Identity(
        static_cast<Eigen::Index>(q.size()),
        static_cast<Eigen::Index>(q.size())) -
        pseudo_inverse * jacobian;

      Eigen::VectorXd secondary(
        static_cast<Eigen::Index>(q.size()));

      for (std::size_t index = 0;
        index < q.size();
        ++index)
      {
        /*
         * Continuity objective:
         *
         *   q -> previous waypoint seed
         *
         * Since initial_q is the immediately previous
         * Cartesian sample, this preserves the IK branch.
         */
        double value =
          seed_gain *
          (initial_q[index] - q[index]);

        const auto & joint =
          *joints[index];

        if (
          joint.has_limit &&
          joint.type != "continuous")
        {
          const double lower_distance =
            std::max(
            q[index] - joint.lower,
            0.0);

          const double upper_distance =
            std::max(
            joint.upper - q[index],
            0.0);

          /*
           * Bounded soft barrier:
           *
           * lower side:
           *   +k ((m-d_lower)/m)^2
           *
           * upper side:
           *   -k ((m-d_upper)/m)^2
           *
           * It is exactly zero outside the soft-margin
           * region and increases smoothly near a limit.
           */
          if (
            lower_distance <
            joint_limit_soft_margin)
          {
            const double ratio =
              (
              joint_limit_soft_margin -
              lower_distance
              ) /
              joint_limit_soft_margin;

            value +=
              joint_limit_barrier_gain *
              ratio * ratio;
          }

          if (
            upper_distance <
            joint_limit_soft_margin)
          {
            const double ratio =
              (
              joint_limit_soft_margin -
              upper_distance
              ) /
              joint_limit_soft_margin;

            value -=
              joint_limit_barrier_gain *
              ratio * ratio;
          }
        }

        secondary[
          static_cast<Eigen::Index>(index)] =
          value;
      }

      Eigen::VectorXd secondary_step =
        null_space * secondary;

      const double largest_secondary_step =
        secondary_step.cwiseAbs().maxCoeff();

      if (
        largest_secondary_step >
        maximum_secondary_step)
      {
        secondary_step *=
          maximum_secondary_step /
          largest_secondary_step;
      }

      delta_q += secondary_step;

      if (!delta_q.allFinite()) {
        throw std::runtime_error(
                "Upright IK produced "
                "a non-finite joint step.");
      }

      const double largest_step =
        delta_q.cwiseAbs().maxCoeff();

      if (
        largest_step >
        maximum_joint_step)
      {
        delta_q *=
          maximum_joint_step /
          largest_step;
      }

      for (std::size_t index = 0;
        index < q.size();
        ++index)
      {
        q[index] +=
          delta_q[
          static_cast<Eigen::Index>(index)];
      }

      applyJointLimits(
        q,
        joints);
    }

    const Eigen::Matrix4d solved_transform =
      forward_kinematics.compute(q);

    const Eigen::Vector3d solved_position =
      solved_transform.block<3, 1>(0, 3);

    const Eigen::Matrix3d solved_rotation =
      solved_transform.block<3, 3>(0, 0);

    const Eigen::Vector3d solved_up =
      solved_rotation.col(2);

    const double solved_yaw =
      std::atan2(
      solved_rotation(1, 0),
      solved_rotation(0, 0));

    final_position_error =
      (target_position - solved_position).norm();

    final_upright_error =
      uprightAngle(
      solved_up,
      target_up);

    const double minimum_margin =
      minimumJointLimitMargin(
      q,
      joints);

    std::cout
      << std::fixed
      << std::setprecision(9);

    std::cout
      << "========================================\n"
      << " DROK Upright 5-DoF IK Solver\n"
      << "========================================\n"
      << "IK mode: upright\n"
      << "----------------------------------------\n"
      << "Target position [m]\n"
      << "x = " << target_position.x() << '\n'
      << "y = " << target_position.y() << '\n'
      << "z = " << target_position.z() << '\n'
      << "----------------------------------------\n"
      << "Constraint\n"
      << "TCP local +Z = world +Z\n"
      << "TCP yaw      = free\n"
      << "----------------------------------------\n"
      << "Initial joint positions [rad]\n";

    for (std::size_t index = 0;
      index < initial_q.size();
      ++index)
    {
      std::cout
        << "q" << index + 1
        << " = "
        << initial_q[index]
        << '\n';
    }

    std::cout
      << "----------------------------------------\n"
      << "IK result joint positions [rad]\n";

    for (std::size_t index = 0;
      index < q.size();
      ++index)
    {
      std::cout
        << "q" << index + 1
        << " = "
        << q[index]
        << '\n';
    }

    std::cout
      << "----------------------------------------\n"
      << "Solved TCP local +Z in world\n"
      << solved_up.transpose() << '\n'
      << "Solved TCP yaw    : "
      << solved_yaw << " rad\n"
      << "----------------------------------------\n"
      << "Success           : "
      << std::boolalpha
      << success << '\n'
      << "Iterations        : "
      << iterations << '\n'
      << "Position error    : "
      << final_position_error
      << " m\n"
      << "Upright error     : "
      << final_upright_error
      << " rad\n"
      << "Orientation error : "
      << final_upright_error
      << " rad\n"
      << "Minimum margin    : "
      << minimum_margin
      << " rad\n"
      << "Message           : "
      << (
        success ?
        "Upright IK converged." :
        "Upright IK reached the maximum iteration count.")
      << '\n'
      << "----------------------------------------\n"
      << "Machine-readable joint result\n"
      << "JOINT_RESULT=";

    for (std::size_t index = 0;
      index < q.size();
      ++index)
    {
      if (index > 0) {
        std::cout << ',';
      }

      std::cout << q[index];
    }

    std::cout
      << '\n'
      << "========================================\n";

    return success ? 0 : 2;

  } catch (const std::exception & exception) {
    std::cerr
      << "[ERROR] "
      << exception.what()
      << '\n';

    return 1;
  }
}
