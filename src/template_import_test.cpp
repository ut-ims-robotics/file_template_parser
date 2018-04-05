/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
*
* This node is used for testing and developing of the template parser
*
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "ros/ros.h"
#include "ros/package.h"
#include "file_template_parser/file_template_parser.h"
#include <stdexcept>

// main
int main(int argc, char **argv)
{
  ros::init(argc, argv, "template_import_test");
  ros::NodeHandle n;

  try
  {
    std::string base_path = "/home/robert/catkin_ws/src/file_template_parser/test/";

    tp::TemplateContainer f_template = tp::importFileTemplate(base_path + "template.xml");

    // Print out the arguments
    f_template.printArguments();

    // Print out the body
    std::cout << f_template.processTemplate() << std::endl;

    // Set the arguments
    f_template.setArgument("noun", "dog");
    f_template.setArgument("adjective", "blue");
    f_template.setArgument("ai_name", "start");

    std::cout << f_template.processTemplate() << std::endl;

    // Save the processed template
    tp::processAndSaveTemplate(f_template, base_path, "template_output");
  }
  catch(std::runtime_error e)
  {
    ROS_ERROR_STREAM(e.what());
  }

  catch(std::exception e)
  {
    ROS_ERROR_STREAM(e.what());
  }

  return 0;
}
