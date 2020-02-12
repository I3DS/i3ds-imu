///////////////////////////////////////////////////////////////////////////\file
///
///   Copyright 2018 SINTEF AS
///
///   This Source Code Form is subject to the terms of the Mozilla
///   Public License, v. 2.0. If a copy of the MPL was not distributed
///   with this file, You can obtain one at https://mozilla.org/MPL/2.0/
///
////////////////////////////////////////////////////////////////////////////////

#include <csignal>
#include <iostream>
#include <unistd.h>
#include <string>
#include <vector>
#include <memory>

#include <i3ds/configurator.hpp>
#include <boost/program_options.hpp>

#include "i3ds/communication.hpp"
#include "imu_dmu30.hpp"

#define BOOST_LOG_DYN_LINK

#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>

#include <cstddef>
#include <memory>
#include <type_traits>
#include <utility>

namespace po = boost::program_options;
namespace logging = boost::log;

std::atomic<bool> running(false);

i3ds::ImuDmu30::Ptr imu;

void signal_handler(int)
{
  BOOST_LOG_TRIVIAL(info) << "do_deactivate()";
  imu->stop();
  running = false;
}

#define DEFAULT_DEVICE "/dev/ttyUSB0"

int main(int argc, char** argv)
{
  std::string device;
  std::string debug_file;
  i3ds::SensorConfigurator configurator;
  po::options_description desc("Allowed camera control options");
  configurator.add_common_options(desc);
  desc.add_options()
      ("device,d", po::value(&device)->default_value(DEFAULT_DEVICE), "Path to COM-device")
      ("debug_file,D", po::value(&debug_file), "Debug file to use instead of COM-device")
      ;
  po::variables_map vm = configurator.parse_common_options(desc, argc, argv);

  BOOST_LOG_TRIVIAL(info) << "Node ID:     " << configurator.node_id;
  BOOST_LOG_TRIVIAL(info) << "Device: " << device;
  BOOST_LOG_TRIVIAL(info) << "Debug file: " << debug_file;


  i3ds::Context::Ptr context = i3ds::Context::Create();

  i3ds::Server server(context);

  imu = i3ds::ImuDmu30::Create(context, configurator.node_id, device);

  imu->Attach(server);

  running = true;
  signal(SIGINT, signal_handler);

  server.Start();

  while (running)
  {
    sleep(1);
  }

  server.Stop();

  return 0;
}
