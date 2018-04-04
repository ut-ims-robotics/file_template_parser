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

} // tp namespace

#endif
