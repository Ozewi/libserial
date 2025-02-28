/**
 * libserial
 * Manejo de comunicaciones por puerto serie
 *
 * @file      version.cpp
 * @brief     Información de versión de la biblioteca
 * @author    José Luis Sánchez Arroyo
 * @date      2025.02.25
 * @version   1.8
 *
 * Copyright (c) 2005-2025 José Luis Sánchez Arroyo
 * This software is distributed under the terms of the LGPL version 2 and comes WITHOUT ANY WARRANTY.
 * Please read the file COPYING.LIB for further details.
 */

namespace libserial {

constexpr char LIBRARY_VERSION[] = "1.8";

/**
 * @brief   Identificador de la versión de la biblioteca
 */
const char* version()
{
  return LIBRARY_VERSION;
}

} // namespace
