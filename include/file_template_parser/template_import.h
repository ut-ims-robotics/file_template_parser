#ifndef TEMPLATE_IMPORT_H
#define TEMPLATE_IMPORT_H

#include "file_template_parser/template_container.h"
#include <tinyxml.h>

namespace tp
{

/**
 * @brief importFileTemplate
 * @param file_path
 * @return
 */
TemplateContainer importFileTemplate(std::string file_path);

/**
 * @brief processAndSaveTemplate
 * @param f_template
 * @param base_path
 * @param name
 */
void processAndSaveTemplate(const TemplateContainer& f_template,
                            std::string base_path,
                            std::string name);

} // tp namespace

#endif
