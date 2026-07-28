#include "drok_arm_kinematics/forward_kinematics.hpp"
#include "drok_arm_kinematics/robot_model_loader.hpp"
#include "drok_arm_kinematics/transform.hpp"

#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

Eigen::Vector3d readVector3(
  const YAML::Node & node,
  const std::string & label)
{
  if (
    !node ||
    !node.IsSequence() ||
    node.size() != 3)
  {
    throw std::runtime_error(
            label + " must contain three values.");
  }

  Eigen::Vector3d result;

  for (std::size_t index = 0;
    index < 3;
    ++index)
  {
    result[
      static_cast<Eigen::Index>(index)] =
      node[index].as<double>();
  }

  if (!result.allFinite()) {
    throw std::runtime_error(
            label + " contains a non-finite value.");
  }

  return result;
}

Eigen::Matrix3d readMatrix3(
  const YAML::Node & node,
  const std::string & label)
{
  if (
    !node ||
    !node.IsSequence() ||
    node.size() != 3)
  {
    throw std::runtime_error(
            label + " must contain three rows.");
  }

  Eigen::Matrix3d result;

  for (std::size_t row = 0;
    row < 3;
    ++row)
  {
    const auto row_node =
      node[row];

    if (
      !row_node.IsSequence() ||
      row_node.size() != 3)
    {
      throw std::runtime_error(
              label + " must be a 3x3 matrix.");
    }

    for (std::size_t column = 0;
      column < 3;
      ++column)
    {
      result(
        static_cast<Eigen::Index>(row),
        static_cast<Eigen::Index>(column)) =
        row_node[column].as<double>();
    }
  }

  if (!result.allFinite()) {
    throw std::runtime_error(
            label + " contains a non-finite value.");
  }

  const double orthogonality_error =
    (
    result.transpose() * result -
    Eigen::Matrix3d::Identity()
    ).norm();

  const double determinant =
    result.determinant();

  if (
    orthogonality_error > 1.0e-7 ||
    std::abs(determinant - 1.0) > 1.0e-7)
  {
    throw std::runtime_error(
            label + " is not a valid rotation matrix.");
  }

  return result;
}

std::vector<double> readJointVector(
  const YAML::Node & node,
  const std::string & label)
{
  if (
    !node ||
    !node.IsSequence() ||
    node.size() != 6)
  {
    throw std::runtime_error(
            label + " must contain six values.");
  }

  std::vector<double> result(6);

  for (std::size_t index = 0;
    index < result.size();
    ++index)
  {
    result[index] =
      node[index].as<double>();

    if (!std::isfinite(result[index])) {
      throw std::runtime_error(
              label + " contains a non-finite value.");
    }
  }

  return result;
}

double rotationAngle(
  const Eigen::Matrix3d & rotation)
{
  const double cosine =
    std::clamp(
    0.5 * (rotation.trace() - 1.0),
    -1.0,
    1.0);

  return std::acos(cosine);
}

struct ValidationLimits
{
  double maximum_cross_track_error{0.0};
  double maximum_endpoint_error{0.0};
  double maximum_longitudinal_overshoot{0.0};
  double maximum_longitudinal_backtrack{0.0};

  double maximum_full_orientation_error{0.0};
  double maximum_upright_tilt_error{0.0};
};

ValidationLimits loadValidationLimits(
  const YAML::Node & document)
{
  const auto position =
    document["position"];

  const auto orientation =
    document["orientation"];

  if (!position || !orientation) {
    throw std::runtime_error(
            "Validation configuration is incomplete.");
  }

  ValidationLimits limits;

  limits.maximum_cross_track_error =
    position[
    "maximum_cross_track_error_m"].as<double>();

  limits.maximum_endpoint_error =
    position[
    "maximum_endpoint_error_m"].as<double>();

  limits.maximum_longitudinal_overshoot =
    position[
    "maximum_longitudinal_overshoot_m"].as<double>();

  limits.maximum_longitudinal_backtrack =
    position[
    "maximum_longitudinal_backtrack_m"].as<double>();

  limits.maximum_full_orientation_error =
    orientation[
    "maximum_full_orientation_error_rad"].as<double>();

  limits.maximum_upright_tilt_error =
    orientation[
    "maximum_upright_tilt_error_rad"].as<double>();

  const std::vector<double> values = {
    limits.maximum_cross_track_error,
    limits.maximum_endpoint_error,
    limits.maximum_longitudinal_overshoot,
    limits.maximum_longitudinal_backtrack,
    limits.maximum_full_orientation_error,
    limits.maximum_upright_tilt_error,
  };

  for (const double value : values) {
    if (
      !std::isfinite(value) ||
      value <= 0.0)
    {
      throw std::runtime_error(
              "All validation limits must be positive.");
    }
  }

  return limits;
}

bool isFullPoseSegment(
  const std::string & segment)
{
  return
    segment == "PICK_APPROACH" ||
    segment == "PICK_LIFT" ||
    segment == "PLACE_RETREAT";
}

bool isUprightSegment(
  const std::string & segment)
{
  return
    segment == "TRANSFER" ||
    segment == "PLACE_DESCEND";
}

struct BlockMetrics
{
  double maximum_cross_track_error{0.0};
  double start_endpoint_error{0.0};
  double goal_endpoint_error{0.0};
  double maximum_longitudinal_overshoot{0.0};
  double maximum_longitudinal_backtrack{0.0};
  double maximum_orientation_error{0.0};

  std::size_t cross_track_sample{0};
  std::size_t orientation_sample{0};

  std::size_t sample_count{0};
  bool pass{false};
};

void printUsage()
{
  std::cerr
    << "Usage:\n"
    << "  validate_timed_cylinder_fk "
    << "<robot_geometry.yaml> "
    << "<timed_path.yaml> "
    << "<cartesian_path.yaml> "
    << "<grasp_candidates.yaml> "
    << "<place_retreat_target.yaml> "
    << "<validation_config.yaml> "
    << "<report.yaml>\n";
}

}  // namespace

int main(
  int argc,
  char ** argv)
{
  if (argc != 8) {
    printUsage();
    return 1;
  }

  try {
    const std::filesystem::path geometry_path =
      argv[1];

    const std::filesystem::path timed_path =
      argv[2];

    const std::filesystem::path cartesian_path =
      argv[3];

    const std::filesystem::path candidate_path =
      argv[4];

    const std::filesystem::path retreat_path =
      argv[5];

    const std::filesystem::path validation_path =
      argv[6];

    const std::filesystem::path report_path =
      argv[7];

    const YAML::Node timed_document =
      YAML::LoadFile(
      timed_path.string());

    const YAML::Node cartesian_document =
      YAML::LoadFile(
      cartesian_path.string());

    const YAML::Node candidate_document =
      YAML::LoadFile(
      candidate_path.string());

    const YAML::Node retreat_document =
      YAML::LoadFile(
      retreat_path.string());

    const YAML::Node validation_document =
      YAML::LoadFile(
      validation_path.string());

    const ValidationLimits limits =
      loadValidationLimits(
      validation_document);

    const auto model =
      drok_arm_kinematics::RobotModelLoader::
      loadFromYaml(
      geometry_path);

    if (model.movable_joint_count != 6) {
      throw std::runtime_error(
              "Exactly six movable joints are required.");
    }

    const drok_arm_kinematics::ForwardKinematics
      forward_kinematics(model);

    std::map<int, Eigen::Vector3d>
      cartesian_positions;

    const auto cartesian_records =
      cartesian_document["path"];

    if (
      !cartesian_records ||
      !cartesian_records.IsSequence())
    {
      throw std::runtime_error(
              "Cartesian path records are missing.");
    }

    for (const auto & record : cartesian_records) {
      const int global_index =
        record["global_index"].as<int>();

      cartesian_positions[global_index] =
        readVector3(
        record["position"],
        "Cartesian position");
    }

    const int candidate_index =
      cartesian_document[
      "candidate_index"].as<int>();

    Eigen::Matrix3d pickup_rotation =
      Eigen::Matrix3d::Identity();

    bool candidate_found = false;

    const auto candidates =
      candidate_document["candidates"];

    for (const auto & candidate : candidates) {
      if (
        candidate["index"].as<int>() ==
        candidate_index)
      {
        pickup_rotation =
          readMatrix3(
          candidate["rotation_world_tcp"],
          "Pickup rotation");

        candidate_found = true;
        break;
      }
    }

    if (!candidate_found) {
      throw std::runtime_error(
              "Selected grasp candidate was not found.");
    }

    const Eigen::Vector3d retreat_rpy =
      readVector3(
      retreat_document[
        "retreat_target_rpy_rad"],
      "Retreat target RPY");

    const Eigen::Matrix3d retreat_rotation =
      drok_arm_kinematics::rotationFromRpy(
      retreat_rpy);

    const auto blocks =
      timed_document["blocks"];

    if (
      !blocks ||
      !blocks.IsSequence() ||
      blocks.size() == 0)
    {
      throw std::runtime_error(
              "Timed motion blocks are missing.");
    }

    YAML::Node report;
    report["version"] = 1;
    report["source_files"]["timed_path"] =
      timed_path.string();
    report["source_files"]["cartesian_path"] =
      cartesian_path.string();
    report["source_files"]["geometry"] =
      geometry_path.string();

    YAML::Node block_reports;

    bool overall_pass = true;

    double global_cross_track = 0.0;
    double global_endpoint_error = 0.0;
    double global_overshoot = 0.0;
    double global_backtrack = 0.0;
    double global_orientation_error = 0.0;

    std::size_t total_samples = 0;

    std::cout
      << std::fixed
      << std::setprecision(9);

    std::cout
      << "============================================================\n"
      << " TIMED CYLINDER PATH FK VALIDATION\n"
      << "============================================================\n";

    for (const auto & block : blocks) {
      const std::string block_name =
        block["name"].as<std::string>();

      const std::string segment =
        block["source_segment"].as<std::string>();

      const int start_index =
        block["start_global_index"].as<int>();

      const int end_index =
        block["end_global_index"].as<int>();

      if (
        cartesian_positions.count(start_index) == 0 ||
        cartesian_positions.count(end_index) == 0)
      {
        throw std::runtime_error(
                "A block endpoint is missing from "
                "the Cartesian path.");
      }

      const Eigen::Vector3d start_position =
        cartesian_positions.at(start_index);

      const Eigen::Vector3d goal_position =
        cartesian_positions.at(end_index);

      const Eigen::Vector3d line_vector =
        goal_position - start_position;

      const double line_length =
        line_vector.norm();

      if (line_length <= 1.0e-12) {
        throw std::runtime_error(
                block_name +
                " has zero Cartesian length.");
      }

      const double line_length_squared =
        line_vector.squaredNorm();

      const auto points =
        block["points"];

      if (
        !points ||
        !points.IsSequence() ||
        points.size() < 2)
      {
        throw std::runtime_error(
                block_name +
                " does not contain enough timed points.");
      }

      if (
        !isFullPoseSegment(segment) &&
        !isUprightSegment(segment))
      {
        throw std::runtime_error(
                "Unknown segment mode: " + segment);
      }

      const Eigen::Matrix3d target_rotation =
        (
        segment == "PLACE_RETREAT" ?
        retreat_rotation :
        pickup_rotation);

      BlockMetrics metrics;
      metrics.sample_count =
        points.size();

      Eigen::Vector3d first_position =
        Eigen::Vector3d::Zero();

      Eigen::Vector3d last_position =
        Eigen::Vector3d::Zero();

      double previous_alpha = 0.0;
      bool have_previous_alpha = false;

      for (std::size_t sample_index = 0;
        sample_index < points.size();
        ++sample_index)
      {
        const auto point =
          points[sample_index];

        const auto q =
          readJointVector(
          point["positions"],
          block_name + " joint positions");

        const Eigen::Matrix4d transform =
          forward_kinematics.compute(q);

        const Eigen::Vector3d position =
          transform.block<3, 1>(0, 3);

        const Eigen::Matrix3d rotation =
          transform.block<3, 3>(0, 0);

        if (sample_index == 0) {
          first_position = position;
        }

        if (sample_index + 1 == points.size()) {
          last_position = position;
        }

        const double alpha =
          (
          position - start_position
          ).dot(line_vector) /
          line_length_squared;

        const Eigen::Vector3d
          infinite_line_projection =
          start_position +
          alpha * line_vector;

        const double cross_track_error =
          (
          position -
          infinite_line_projection
          ).norm();

        if (
          cross_track_error >
          metrics.maximum_cross_track_error)
        {
          metrics.maximum_cross_track_error =
            cross_track_error;

          metrics.cross_track_sample =
            sample_index;
        }

        const double overshoot =
          std::max(
          {
            0.0,
            -alpha * line_length,
            (alpha - 1.0) * line_length,
          });

        metrics.maximum_longitudinal_overshoot =
          std::max(
          metrics.maximum_longitudinal_overshoot,
          overshoot);

        if (have_previous_alpha) {
          const double backtrack =
            std::max(
            0.0,
            (previous_alpha - alpha)
            * line_length);

          metrics.maximum_longitudinal_backtrack =
            std::max(
            metrics.maximum_longitudinal_backtrack,
            backtrack);
        }

        previous_alpha = alpha;
        have_previous_alpha = true;

        double orientation_error = 0.0;

        if (isFullPoseSegment(segment)) {
          orientation_error =
            rotationAngle(
            target_rotation.transpose() *
            rotation);
        } else {
          const double upright_cosine =
            std::clamp(
            rotation.col(2).dot(
              Eigen::Vector3d::UnitZ()),
            -1.0,
            1.0);

          orientation_error =
            std::acos(
            upright_cosine);
        }

        if (
          orientation_error >
          metrics.maximum_orientation_error)
        {
          metrics.maximum_orientation_error =
            orientation_error;

          metrics.orientation_sample =
            sample_index;
        }
      }

      metrics.start_endpoint_error =
        (
        first_position -
        start_position
        ).norm();

      metrics.goal_endpoint_error =
        (
        last_position -
        goal_position
        ).norm();

      const double maximum_endpoint_error =
        std::max(
        metrics.start_endpoint_error,
        metrics.goal_endpoint_error);

      const double orientation_limit =
        (
        isFullPoseSegment(segment) ?
        limits.maximum_full_orientation_error :
        limits.maximum_upright_tilt_error);

      metrics.pass =
        metrics.maximum_cross_track_error <=
        limits.maximum_cross_track_error &&
        maximum_endpoint_error <=
        limits.maximum_endpoint_error &&
        metrics.maximum_longitudinal_overshoot <=
        limits.maximum_longitudinal_overshoot &&
        metrics.maximum_longitudinal_backtrack <=
        limits.maximum_longitudinal_backtrack &&
        metrics.maximum_orientation_error <=
        orientation_limit;

      overall_pass =
        overall_pass && metrics.pass;

      total_samples +=
        metrics.sample_count;

      global_cross_track =
        std::max(
        global_cross_track,
        metrics.maximum_cross_track_error);

      global_endpoint_error =
        std::max(
        global_endpoint_error,
        maximum_endpoint_error);

      global_overshoot =
        std::max(
        global_overshoot,
        metrics.maximum_longitudinal_overshoot);

      global_backtrack =
        std::max(
        global_backtrack,
        metrics.maximum_longitudinal_backtrack);

      global_orientation_error =
        std::max(
        global_orientation_error,
        metrics.maximum_orientation_error);

      YAML::Node block_report;

      block_report["name"] =
        block_name;

      block_report["source_segment"] =
        segment;

      block_report["sample_count"] =
        metrics.sample_count;

      block_report[
        "maximum_cross_track_error_m"] =
        metrics.maximum_cross_track_error;

      block_report[
        "maximum_cross_track_sample"] =
        metrics.cross_track_sample;

      block_report[
        "start_endpoint_error_m"] =
        metrics.start_endpoint_error;

      block_report[
        "goal_endpoint_error_m"] =
        metrics.goal_endpoint_error;

      block_report[
        "maximum_longitudinal_overshoot_m"] =
        metrics.maximum_longitudinal_overshoot;

      block_report[
        "maximum_longitudinal_backtrack_m"] =
        metrics.maximum_longitudinal_backtrack;

      block_report[
        "maximum_orientation_error_rad"] =
        metrics.maximum_orientation_error;

      block_report[
        "maximum_orientation_sample"] =
        metrics.orientation_sample;

      block_report["pass"] =
        metrics.pass;

      block_reports.push_back(
        block_report);

      std::cout
        << "\n[" << block_name << "]\n"
        << "  segment            : "
        << segment << '\n'
        << "  samples            : "
        << metrics.sample_count << '\n'
        << "  cross-track error  : "
        << metrics.maximum_cross_track_error
        << " m\n"
        << "  start endpoint     : "
        << metrics.start_endpoint_error
        << " m\n"
        << "  goal endpoint      : "
        << metrics.goal_endpoint_error
        << " m\n"
        << "  overshoot          : "
        << metrics.maximum_longitudinal_overshoot
        << " m\n"
        << "  backtrack          : "
        << metrics.maximum_longitudinal_backtrack
        << " m\n"
        << "  orientation/tilt   : "
        << metrics.maximum_orientation_error
        << " rad\n"
        << "  result             : "
        << (
          metrics.pass ?
          "PASS" :
          "FAIL")
        << '\n';
    }

    report["blocks"] =
      block_reports;

    report["summary"]["sample_count"] =
      total_samples;

    report["summary"][
      "maximum_cross_track_error_m"] =
      global_cross_track;

    report["summary"][
      "maximum_endpoint_error_m"] =
      global_endpoint_error;

    report["summary"][
      "maximum_longitudinal_overshoot_m"] =
      global_overshoot;

    report["summary"][
      "maximum_longitudinal_backtrack_m"] =
      global_backtrack;

    report["summary"][
      "maximum_orientation_error_rad"] =
      global_orientation_error;

    report["summary"]["pass"] =
      overall_pass;

    YAML::Emitter emitter;
    emitter << report;

    std::ofstream output_stream(
      report_path);

    if (!output_stream) {
      throw std::runtime_error(
              "Could not open the report output file.");
    }

    output_stream
      << emitter.c_str()
      << '\n';

    std::cout
      << "\n============================================================\n"
      << "OVERALL RESULT: "
      << (
        overall_pass ?
        "PASS" :
        "FAIL")
      << '\n'
      << "============================================================\n"
      << "Samples                    : "
      << total_samples << '\n'
      << "Maximum cross-track error  : "
      << global_cross_track << " m\n"
      << "Maximum endpoint error     : "
      << global_endpoint_error << " m\n"
      << "Maximum overshoot          : "
      << global_overshoot << " m\n"
      << "Maximum backtrack          : "
      << global_backtrack << " m\n"
      << "Maximum orientation error  : "
      << global_orientation_error << " rad\n"
      << "Saved:\n"
      << report_path << '\n';

    return overall_pass ? 0 : 2;

  } catch (const std::exception & exception) {
    std::cerr
      << "[ERROR] "
      << exception.what()
      << '\n';

    return 1;
  }
}
