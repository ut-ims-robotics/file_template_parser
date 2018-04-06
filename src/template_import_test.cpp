/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
*
* This node is used for testing and developing of the template parser
*
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "ros/ros.h"
#include "ros/package.h"
#include "file_template_parser/file_template_parser.h"
#include <stdexcept>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "template_import_test");
  ros::NodeHandle n;

  try
  {
    /*
     * Get the test folder path
     */
    std::string base_path = ros::package::getPath(ROS_PACKAGE_NAME) + "/test/";

    /*
     * Import the templates
     */
    tp::TemplateContainer t_random(base_path + "templates/template.xml");
    tp::TemplateContainer t_cmakelists(base_path + "templates/temoto_ai_cmakelists.xml");
    tp::TemplateContainer t_packagexml(base_path + "templates/temoto_ai_packagexml.xml");

    /*
     * Print out the arguments
     */
    t_random.printArguments();

    /*
     * Print out the body
     */
    // std::cout << t_random.processTemplate() << std::endl;

    /*
     * Set the arguments
     */
    t_random.setArgument("noun", "dog");
    t_random.setArgument("adjective", "blue");
    t_random.setArgument("ai_name", "start");
    t_cmakelists.setArgument("ai_name", "ai_name_test");
    t_packagexml.setArgument("ai_name", "ai_name_test");

    /*
     * Save the processed template
     */
    t_random.processAndSaveTemplate(base_path, "template_output");
    t_cmakelists.processAndSaveTemplate(base_path, "CmakeLists");
    t_packagexml.processAndSaveTemplate(base_path, "package");
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
