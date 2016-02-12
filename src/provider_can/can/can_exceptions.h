/**
 * \file	can_exceptions.h
 * \author	David Mercier <david.mercier@gmail.com>
 * \date	29/04/2008
 *
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

// TODO: Move to lib_atlas

#ifndef PROVIDER_CAN_EXCEPTION_H
#define PROVIDER_CAN_EXCEPTION_H

#include <exception>

namespace provider_can {

class ExceptionCanDeviceNotFound : public std::exception {
  virtual const char *what() const throw() { return "CAN device not found"; }
};

}

#endif  // EXCEPTION_H
