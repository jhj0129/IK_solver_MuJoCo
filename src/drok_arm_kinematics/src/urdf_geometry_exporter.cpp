#include <tinyxml2.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace
{

struct JointInfo
{
  std::string name;
  std::string type;
  std::string parent;
  std::string child;

  std::array<double, 3> origin_xyz{0.0, 0.0, 0.0};
  std::array<double, 3> origin_rpy{0.0, 0.0, 0.0};
  std::array<double, 3> axis{0.0, 0.0, 0.0};

  bool has_limit{false};
  double lower{0.0};
  double upper{0.0};
  double effort{0.0};
  double velocity{0.0};
};

std::array<double, 3> parseVector3(
  const char * text,
  const std::array<double, 3> & default_value)
{
  if (text == nullptr) {
    return default_value;
  }

  std::array<double, 3> result = default_value;

  std::stringstream stream(text);

  if (!(stream >> result[0] >> result[1] >> result[2])) {
    throw std::runtime_error(
            std::string("Vector3 parsing failed: ") + text);
  }

  return result;
}

double parseRequiredDouble(
  const tinyxml2::XMLElement * element,
  const char * attribute_name,
  const std::string & context)
{
  if (element == nullptr) {
    throw std::runtime_error(
            "Missing XML element while parsing " + context);
  }

  double value = 0.0;

  if (element->QueryDoubleAttribute(attribute_name, &value) !=
    tinyxml2::XML_SUCCESS)
  {
    throw std::runtime_error(
            "Missing or invalid attribute '" +
            std::string(attribute_name) +
            "' in " + context);
  }

  return value;
}

JointInfo parseJoint(const tinyxml2::XMLElement * joint_element)
{
  JointInfo joint;

  const char * name = joint_element->Attribute("name");
  const char * type = joint_element->Attribute("type");

  if (name == nullptr || type == nullptr) {
    throw std::runtime_error(
            "A joint is missing its name or type attribute.");
  }

  joint.name = name;
  joint.type = type;

  const auto * parent_element =
    joint_element->FirstChildElement("parent");

  const auto * child_element =
    joint_element->FirstChildElement("child");

  if (parent_element == nullptr || child_element == nullptr) {
    throw std::runtime_error(
            "Joint " + joint.name +
            " is missing parent or child.");
  }

  const char * parent_link =
    parent_element->Attribute("link");

  const char * child_link =
    child_element->Attribute("link");

  if (parent_link == nullptr || child_link == nullptr) {
    throw std::runtime_error(
            "Joint " + joint.name +
            " has an invalid parent or child definition.");
  }

  joint.parent = parent_link;
  joint.child = child_link;

  const auto * origin_element =
    joint_element->FirstChildElement("origin");

  if (origin_element != nullptr) {
    joint.origin_xyz = parseVector3(
      origin_element->Attribute("xyz"),
      {0.0, 0.0, 0.0});

    joint.origin_rpy = parseVector3(
      origin_element->Attribute("rpy"),
      {0.0, 0.0, 0.0});
  }

  const auto * axis_element =
    joint_element->FirstChildElement("axis");

  if (joint.type == "fixed") {
    joint.axis = {0.0, 0.0, 0.0};
  } else if (axis_element != nullptr) {
    joint.axis = parseVector3(
      axis_element->Attribute("xyz"),
      {1.0, 0.0, 0.0});
  } else {
    // URDF default axis for revolute/prismatic joints.
    joint.axis = {1.0, 0.0, 0.0};
  }

  const auto * limit_element =
    joint_element->FirstChildElement("limit");

  if (limit_element != nullptr) {
    joint.has_limit = true;

    joint.lower = parseRequiredDouble(
      limit_element, "lower",
      joint.name + " limit");

    joint.upper = parseRequiredDouble(
      limit_element, "upper",
      joint.name + " limit");

    joint.effort = parseRequiredDouble(
      limit_element, "effort",
      joint.name + " limit");

    joint.velocity = parseRequiredDouble(
      limit_element, "velocity",
      joint.name + " limit");
  }

  return joint;
}

YAML::Node vectorToYaml(const std::array<double, 3> & vector)
{
  YAML::Node node(YAML::NodeType::Sequence);

  node.push_back(vector[0]);
  node.push_back(vector[1]);
  node.push_back(vector[2]);

  node.SetStyle(YAML::EmitterStyle::Flow);

  return node;
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 5) {
    std::cerr
      << "Usage:\n"
      << "  urdf_geometry_exporter "
      << "<urdf_path> <base_frame> <tool_frame> <output_yaml>\n";

    return 1;
  }

  const std::filesystem::path urdf_path = argv[1];
  const std::string base_frame = argv[2];
  const std::string tool_frame = argv[3];
  const std::filesystem::path output_path = argv[4];

  try {
    tinyxml2::XMLDocument document;

    const auto result =
      document.LoadFile(urdf_path.string().c_str());

    if (result != tinyxml2::XML_SUCCESS) {
      throw std::runtime_error(
              "Failed to read URDF: " +
              urdf_path.string());
    }

    const auto * robot_element =
      document.FirstChildElement("robot");

    if (robot_element == nullptr) {
      throw std::runtime_error(
              "The URDF does not contain a <robot> element.");
    }

    const char * robot_name_attribute =
      robot_element->Attribute("name");

    const std::string robot_name =
      robot_name_attribute != nullptr ?
      robot_name_attribute : "unknown_robot";

    std::unordered_map<std::string, JointInfo>
    joint_by_child;

    for (
      const auto * joint_element =
        robot_element->FirstChildElement("joint");
      joint_element != nullptr;
      joint_element =
        joint_element->NextSiblingElement("joint"))
    {
      JointInfo joint = parseJoint(joint_element);

      if (joint_by_child.count(joint.child) != 0U) {
        throw std::runtime_error(
                "Multiple joints have the same child link: " +
                joint.child);
      }

      joint_by_child.emplace(joint.child, joint);
    }

    std::vector<JointInfo> reversed_chain;
    std::string current_link = tool_frame;

    while (current_link != base_frame) {
      const auto iterator =
        joint_by_child.find(current_link);

      if (iterator == joint_by_child.end()) {
        throw std::runtime_error(
                "Could not trace a joint from link '" +
                current_link +
                "' toward base frame '" +
                base_frame + "'.");
      }

      reversed_chain.push_back(iterator->second);
      current_link = iterator->second.parent;

      if (reversed_chain.size() >
        joint_by_child.size())
      {
        throw std::runtime_error(
                "A cycle was detected in the URDF joint tree.");
      }
    }

    std::reverse(
      reversed_chain.begin(),
      reversed_chain.end());

    YAML::Node root;

    root["robot"]["name"] = robot_name;
    root["robot"]["base_frame"] = base_frame;
    root["robot"]["tool_frame"] = tool_frame;
    root["robot"]["chain_joint_count"] =
      static_cast<int>(reversed_chain.size());

    int movable_joint_count = 0;

    for (const auto & joint : reversed_chain) {
      if (joint.type != "fixed") {
        ++movable_joint_count;
      }
    }

    root["robot"]["movable_joint_count"] =
      movable_joint_count;

    YAML::Node chain(YAML::NodeType::Sequence);

    for (std::size_t index = 0;
      index < reversed_chain.size();
      ++index)
    {
      const auto & joint = reversed_chain[index];

      YAML::Node node;

      node["index"] = static_cast<int>(index);
      node["name"] = joint.name;
      node["type"] = joint.type;
      node["parent"] = joint.parent;
      node["child"] = joint.child;

      node["origin"]["xyz"] =
        vectorToYaml(joint.origin_xyz);

      node["origin"]["rpy"] =
        vectorToYaml(joint.origin_rpy);

      node["axis"] =
        vectorToYaml(joint.axis);

      if (joint.has_limit) {
        node["limit"]["lower"] = joint.lower;
        node["limit"]["upper"] = joint.upper;
        node["limit"]["effort"] = joint.effort;
        node["limit"]["velocity"] = joint.velocity;
      }

      chain.push_back(node);
    }

    root["kinematic_chain"] = chain;

    std::filesystem::create_directories(
      output_path.parent_path());

    std::ofstream output_file(output_path);

    if (!output_file.is_open()) {
      throw std::runtime_error(
              "Failed to open output file: " +
              output_path.string());
    }

    output_file << root;
    output_file << '\n';
    output_file.close();

    std::cout
      << "========================================\n"
      << " URDF geometry export completed\n"
      << "========================================\n"
      << "Robot             : " << robot_name << '\n'
      << "Base frame        : " << base_frame << '\n'
      << "Tool frame        : " << tool_frame << '\n'
      << "Chain joints      : "
      << reversed_chain.size() << '\n'
      << "Movable joints    : "
      << movable_joint_count << '\n'
      << "Output            : "
      << output_path << '\n'
      << "----------------------------------------\n";

    for (std::size_t index = 0;
      index < reversed_chain.size();
      ++index)
    {
      const auto & joint = reversed_chain[index];

      std::cout
        << index << ": "
        << joint.name
        << " [" << joint.type << "] "
        << joint.parent
        << " -> "
        << joint.child
        << '\n';
    }

    std::cout
      << "========================================\n";

  } catch (const std::exception & exception) {
    std::cerr
      << "[ERROR] "
      << exception.what()
      << '\n';

    return 1;
  }

  return 0;
}
