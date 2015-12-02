/*******************************************************************************
    copyright            : (C) 2008 S.O.N.I.A. Project
    author               : David Mercier <david.mercier@gmail.com>
    begin                : April 29th, 2008
*******************************************************************************/

#ifndef PROVIDER_CAN_EXCEPTION_H
#define PROVIDER_CAN_EXCEPTION_H

#include <exception>

namespace provider_can {

class ExceptionCanDeviceNotFound : public std::exception {
  virtual const char *what() const throw() { return "CAN device not found"; }
};

class ExceptionInvalidServerSocket : public std::exception {
  virtual const char *what() const throw() {
    return "Unable to instantiate the ServerSocket";
  }
};

class ExceptionConfigFileNotFound : public std::exception {
  virtual const char *what() const throw() { return "Config file not found"; }
};

class ExceptionConfigKeyNotFound : public std::exception {
  virtual const char *what() const throw() { return "Config key not found"; }
};

class ExceptionPropertyNotFound : public std::exception {
  virtual const char *what() const throw() { return "Property not found"; }
};
}

#endif  // EXCEPTION_H
