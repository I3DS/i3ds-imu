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

volatile bool running;

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
  unsigned int node_id;
  std::string device;

  po::options_description desc("Allowed camera control options");

  desc.add_options()
  ("help,h", "Produce this message")
  ("node,n", po::value<unsigned int>(&node_id)->default_value(10), "Node ID of IMU")
  ("device,d", po::value(&device)->default_value(DEFAULT_DEVICE), "Path to COM-device")
  ("verbose,v", "Print verbose output")
  ("quite,q", "Quiet ouput")
  ("print", "Print the camera configuration")
  ;

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);

  if (vm.count("help"))
    {
      std::cout << desc << std::endl;
      return -1;
    }

  if (vm.count("quiet"))
    {
      logging::core::get()->set_filter(logging::trivial::severity >= logging::trivial::warning);
    }
  else if (!vm.count("verbose"))
    {
      logging::core::get()->set_filter(logging::trivial::severity >= logging::trivial::info);
    }

  po::notify(vm);

  BOOST_LOG_TRIVIAL(info) << "Node ID:     " << node_id;
  BOOST_LOG_TRIVIAL(info) << "Device: " << device;

  i3ds::Context::Ptr context = i3ds::Context::Create();

  i3ds::Server server(context);

  imu = i3ds::ImuDmu30::Create(context, node_id, device);
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
