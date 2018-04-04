#include "file_template_parser/template_container.h"
#include <stdexcept>

namespace tp
{

// TemplateContainer constructor
TemplateContainer::TemplateContainer(Argmap arguments, std::string body)
: arguments_(arguments),
  body_(body)
{}

// Set template argument
void TemplateContainer::setArgument(std::string argument, std::string value)
{
  if (arguments_.find(argument) != arguments_.end())
  {
    arguments_[argument] = value;
  }
  else
  {
    throw std::runtime_error("Cannot recognize the argument '" + argument + "'");
  }
}

// Get template argument
std::string TemplateContainer::getArgument(std::string argument) const
{
  return arguments_.find(argument)->second;
}

// Process the template
std::string TemplateContainer::processTemplate() const
{
  return body_;
}

} // tp namespace
