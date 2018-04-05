#include "file_template_parser/template_container.h"
#include <stdexcept>
#include <iostream>
#include <boost/algorithm/string/replace.hpp>

namespace tp
{

// TemplateContainer constructor
TemplateContainer::TemplateContainer(Argmap arguments, std::string body, std::string extension)
: arguments_(arguments),
  body_(body),
  extension_(extension)
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

// Get the argument map
Argmap TemplateContainer::getArgmap() const
{
  return arguments_;
}

// Get the extension
std::string TemplateContainer::getExtension() const
{
  return extension_;
}

// Print the arguments
void TemplateContainer::printArguments() const
{
  for (auto arg : arguments_)
  {
    std::cout << " * name: " << arg.first << "; value: " << arg.second << std::endl;
  }
}

// Process the template
std::string TemplateContainer::processTemplate() const
{
  std::string processed_body = body_;

  for (auto arg : arguments_)
  {
    boost::replace_all(processed_body, "$(arg " + arg.first + ")", arg.second);
  }

  return processed_body;
}

} // tp namespace
