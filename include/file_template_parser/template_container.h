#ifndef TEMPLATE_CONTAINER_H
#define TEMPLATE_CONTAINER_H

#include <string>
#include <map>

typedef std::map<std::string, std::string> Argmap;

namespace tp
{

class TemplateContainer
{
public:

  /**
   * @brief TemplateContainer
   * @param arguments
   * @param body
   */
  TemplateContainer(Argmap arguments, std::string body);

  /**
   * @brief setArgument
   * @param argument
   * @param value
   */
  void setArgument(std::string argument, std::string value);

  /**
   * @brief getArgument
   * @param argument
   * @return
   */
  std::string getArgument(std::string argument) const;

  /**
   * @brief processTemplate
   * @return
   */
  std::string processTemplate() const;

private:
  Argmap arguments_;
  std::string body_;
  std::map<std::string, TemplateContainer> sub_containers_;
};

} // tp namespace
#endif
