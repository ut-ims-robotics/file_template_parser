#include "file_template_parser/template_import.h"
#include <boost/filesystem/operations.hpp>
#include <stdexcept>
#include <fstream>

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
  for (TiXmlElement* arg_element = root_element->FirstChildElement("arg");
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
  TiXmlElement* body_element = root_element->FirstChildElement("body");

  // Check the body element
  if(body_element == NULL)
  {
    throw std::runtime_error("The file template does not contain a body element");
  }

  const char* body = body_element->GetText();

  // Check the text
  if (body == NULL)
  {
    throw std::runtime_error("The text in the 'body' is compromised");
  }

  return std::string(body);
}

// Import file template
TemplateContainer importFileTemplate(std::string file_path)
{

  TiXmlDocument template_xml;

  try
  {
    // Open the template file
    if(!template_xml.LoadFile(file_path))
    {
      throw std::runtime_error("Cannot open the file template");
    }

    // Get the root element
    TiXmlElement* root_element = template_xml.FirstChildElement("f_template");

    // Check if any element was received
    if( root_element == NULL )
    {
      throw std::runtime_error("Missing 'f_template' element");
    }

    // Get the output file extension hint
    std::string extension = getAttribute("extension", root_element);

    // Get template arguments
    Argmap arguments = getArguments(root_element);

    // Get template body
    std::string body = getBody(root_element);

    return TemplateContainer(arguments, body, extension);
  }
  catch(std::runtime_error e)
  {
    throw std::runtime_error(std::string(e.what()) + ": in '" + file_path + "'");
  }

  catch(std::exception e)
  {
    throw std::runtime_error(std::string(e.what()) + ": in '" + file_path + "'");
  }

  template_xml.Clear();
}

// Process and save the template
void processAndSaveTemplate(const TemplateContainer& f_template,
                            std::string base_path,
                            std::string name)
{
  std::ofstream file_out(base_path + "/" + name + f_template.getExtension());
  file_out << f_template.processTemplate();
  file_out.close();
}

} // tp namespace
