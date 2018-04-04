#include "file_template_parser/template_import.h"
#include <boost/filesystem/operations.hpp>
#include <stdexcept>

namespace tp
{

/**
 * @brief getAttribute
 * @param attribute_name
 * @param xml_element
 * @return
 */
std::string getAttribute(std::string attribute_name, TiXmlElement* xml_element)
{
  const char* attribute = xml_element->Attribute(attribute_name.c_str());
  if (attribute == NULL)
  {
    throw std::runtime_error("Missing attribute: '" + attribute_name + "'");
  }

  return std::string(attribute);
}

/**
 * @brief getArguments
 * @param root_element
 * @return
 */
Argmap getArguments(TiXmlElement* root_element)
{
  Argmap arguments;

  // Iterate through all arguments
  for (TiXmlElement* arg_element = root_element;
                     arg_element != NULL;
                     arg_element = arg_element->NextSiblingElement("arg"))
  {
    std::string name          = getAttribute("name", arg_element);
    std::string default_value = getAttribute("default", arg_element);
    arguments[name] = default_value;
  }

  // Check if there are any arguments
  if (arguments.empty())
  {
    throw std::runtime_error("The file template does not contain arguments");
  }

  return arguments;
}

/**
 * @brief getBody
 * @param root_element
 * @return
 */
std::string getBody(TiXmlElement* root_element)
{
  TiXmlNode* body_node = root_element->NextSibling("body");

  // Check the body element
  if(body_node == NULL)
  {
    throw std::runtime_error("The file template does not contain a body element");
  }

  std::string body = body_node->ToText()->Value();

  if (body.empty())
  {
    throw std::runtime_error("The body element does not contain body text");
  }

  return body;
}

// Import file template
TemplateContainer importFileTemplate(std::string file_path)
{

  TiXmlDocument template_xml;

  try
  {
    // Open the template file
    if(template_xml.LoadFile(file_path))
    {
      throw std::runtime_error("Cannot open the file template");
    }

    // Get the root element
    TiXmlElement* root_element = template_xml.FirstChildElement();

    // Check if any element was received
    if( root_element == NULL )
    {
      throw std::runtime_error("No xml elements");
    }

    // Get template arguments
    Argmap arguments = getArguments(root_element);

    // Get template body
    std::string body = getBody(root_element);

    return TemplateContainer(arguments, body);
  }
  catch(std::exception e)
  {
    throw std::runtime_error(std::string(e.what()) + ": in '" + file_path + "'");
  }

  template_xml.Clear();
}

} // tp namespace
