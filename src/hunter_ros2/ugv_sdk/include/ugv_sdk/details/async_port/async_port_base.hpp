/*
 * async_port_base.hpp
 *
 * Created on: Jun 22, 2021 16:19
 * Description:
 *
 * Copyright (c) 2021 Weston Robot Pte. Ltd.
 */

#ifndef ASYNC_PORT_BASE_HPP
#define ASYNC_PORT_BASE_HPP

#include <thread>
#include <mutex>
#include <functional>
#include <iostream>

#include <boost/asio.hpp>

namespace westonrobot {
class AsyncPortBase {
 public:
  AsyncPortBase(std::string port) : port_(port) {}
  virtual ~AsyncPortBase(){};

  // do not allow copy
  AsyncPortBase() = delete;
  AsyncPortBase(const AsyncPortBase& other) = delete;

  virtual bool IsOpened() const { return port_opened_; }

  bool StartListening() {
    if (SetupPort()) {
      io_thread_ = std::thread([this]() { io_context_.run(); });
      return true;
    }
    std::cerr
        << "[ERROR] Failed to setup port, please check if specified port exits "
           "or if you have proper permissions to access it"
        << std::endl;
    return false;
  };
  virtual void StopService() {}

 protected:
  std::string port_;
  bool port_opened_ = false;

  boost::asio::io_service io_context_;

  std::thread io_thread_;

  virtual bool SetupPort() = 0;
};
}  // namespace westonrobot

#endif /* ASYNC_PORT_BASE_HPP */
