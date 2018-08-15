///////////////////////////////////////////////////////////////////////////\file
///
///   Copyright 2018 SINTEF AS
///
///   This Source Code Form is subject to the terms of the Mozilla
///   Public License, v. 2.0. If a copy of the MPL was not distributed
///   with this file, You can obtain one at https://mozilla.org/MPL/2.0/
///
////////////////////////////////////////////////////////////////////////////////

#include "imu_dmu30.hpp"
#include <cstdint>
#include <cstddef>
#include <byteswap.h>

#include <fcntl.h>      // File control definitions
#include <termios.h>

#define BOOST_LOG_DYN_LINK

#include <iostream>
#include <boost/format.hpp>
#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>

// 0x55AA is reversed due to endianness
#define SYNC_BYTE 0xAA55

struct dmu30_frame {
  uint16_t sync_bytes;
  uint16_t message_count;
  float axis_x_rate;
  float axis_x_acceleration;
  float axis_y_rate;
  float axis_y_acceleration;
  float axis_z_rate;
  float axis_z_acceleration;
  float aux_input_voltage;
  float average_temperature;
  float axis_x_delta_theta;
  float axis_x_vel;
  float axis_y_delta_theta;
  float axis_y_vel;
  float axis_z_delta_theta;
  float axis_z_vel;
  uint16_t startup_flags;
  uint16_t operation_flags;
  uint16_t error_flags;
  uint16_t checksum;
};


i3ds::ImuDmu30 *latest_imu = nullptr;

extern "C" {
  extern void i3ds_handle_imu_message(Message_Type data);
  void i3ds_send_imu_message(IMUMeasurement20 message) {
    BOOST_LOG_TRIVIAL(trace) << "Got message from ADA!";
    if (latest_imu == nullptr){
      BOOST_LOG_TRIVIAL(error) << "Publisher is null. Not transmitting";
      return;
    }
    latest_imu->publish_message(message);
  }
}

i3ds::ImuDmu30::ImuDmu30(Context::Ptr context, NodeID id, std::string device)
  : IMU(id),
    publisher_(context, id),
    device_(device)
{
  latest_imu = this;
}

void
i3ds::ImuDmu30::run()
{
  auto data = std::make_shared<Message_Type>();
  while(running_) {
    if (!read_data(data)) {
      BOOST_LOG_TRIVIAL(warning) << "Could not read frame.";
    }
    BOOST_LOG_TRIVIAL(trace) << "Got message!"; 
  }
}

bool ensure_read(int device, void *buf, size_t n_bytes) {
  size_t ix=0;
  do {
    int n = read(device, (char *)buf + ix, n_bytes - ix);

    if (n < 0) {
      BOOST_LOG_TRIVIAL(error) << "Could not read from device.";
      return false;
    }

    if (n == 0) {
      BOOST_LOG_TRIVIAL(warning) << "Read zero bytes from device";
      continue;
    }

    ix += n;
    
  } while (ix < n_bytes);

  return true;
}

#define swap_bytes_16(X) __bswap_16(X)
#define swap_bytes_32(X) __bswap_32(X)

bool i3ds::ImuDmu30::read_data(const std::shared_ptr<Message_Type> data) {
  dmu30_frame frame;
  uint8_t *buf = (uint8_t *)&frame;
  unsigned int skipped = 0;

  if (!ensure_read(com_, buf, 2)) {
    return false;
  }

  while (frame.sync_bytes != SYNC_BYTE) {
    buf[0] = buf[1];
    if (!ensure_read(com_, buf+1, 1)) {
      return false;
    }
    skipped++;
  }

  if (skipped > 0) {
    BOOST_LOG_TRIVIAL(warning) << "Skipped " << skipped << " bytes to sync.";
  }

  if (!ensure_read(com_, buf + 2, sizeof(dmu30_frame)-2)) {
    return false;
  }
  
  uint16_t *u16buf = (uint16_t *)&frame;
  size_t n_shorts = sizeof(dmu30_frame)/2;
  // Checksum is OK if the sum of all 16-bit values is 0
  uint16_t checksum = 0;
  for (size_t i=0; i<n_shorts; i++) {
    checksum += swap_bytes_16(u16buf[i]);
  }

  if (checksum != 0) {
    BOOST_LOG_TRIVIAL(warning) << "Checksum error!";
    return false;
  }

  // Swap due to endianness.
  uint32_t *ubuf32 = (uint32_t *)&frame;
  size_t n_ints = sizeof(dmu30_frame)/4;
  for(size_t i=1; i<n_ints - 2; i++) {
    ubuf32[i] = swap_bytes_32(ubuf32[i]);
  }
  
  for(size_t i=0; i<2; i++) {
    u16buf[i] = swap_bytes_16(u16buf[i]);
  }
  for(size_t i=n_shorts - 4; i<n_shorts; i++) {
    u16buf[i] = swap_bytes_16(u16buf[i]);
  }
  // Swapping done
  
  data->message_count = frame.message_count;
  data->axis_x_rate = frame.axis_x_rate;
  data->axis_x_acceleration = frame.axis_x_acceleration;
  data->axis_y_rate = frame.axis_y_rate;
  data->axis_y_acceleration = frame.axis_y_acceleration;
  data->axis_z_rate = frame.axis_z_rate;
  data->axis_z_acceleration = frame.axis_z_acceleration;
  data->aux_input_voltage = frame.aux_input_voltage;
  data->average_temperature = frame.average_temperature;
  data->axis_x_delta_theta = frame.axis_x_delta_theta;
  data->axis_x_vel = frame.axis_x_vel;
  data->axis_y_delta_theta = frame.axis_y_delta_theta;
  data->axis_y_vel = frame.axis_y_vel;
  data->axis_z_delta_theta = frame.axis_z_delta_theta;
  data->axis_z_vel = frame.axis_z_vel;
  memcpy(&data->startup_flags, &frame.startup_flags, sizeof(frame.startup_flags));
  memcpy(&data->operation_flags, &frame.operation_flags, sizeof(frame.operation_flags));
  memcpy(&data->error_flags, &frame.error_flags, sizeof(frame.error_flags));

  i3ds_handle_imu_message(*data);

  return true;
}


bool i3ds::ImuDmu30::is_sampling_supported(SampleCommand sample) {
  // TODO(sigurdal) implement?
  return true;
}

void i3ds::ImuDmu30::open_device() {
  com_ = open(device_.c_str(), O_RDWR | O_NOCTTY);
  if ( com_ < 0 )
  {
    BOOST_LOG_TRIVIAL(error) << "Error " << errno << " opening " << "/dev/ttyUSB0" << ": " << strerror (errno);
    throw i3ds::CommandError(error_value, "Could not open device.");
  }

  struct termios tty;
  memset (&tty, 0, sizeof tty);
  if ( tcgetattr ( com_, &tty ) != 0 )
  {
    BOOST_LOG_TRIVIAL(error) <<  "Error " << errno << " from tcgetattr: " << strerror(errno);
    throw i3ds::CommandError(error_value, "Could not open tty.");
  }
  
  cfsetospeed(&tty, B460800);
  cfsetispeed(&tty, B460800);

  //Clear size.
  tty.c_cflag &= ~CSIZE;

  // Disable flow control, parity, odd/even parity, hang-up-on-close
  tty.c_cflag &= ~(CRTSCTS | PARENB | PARODD | HUPCL);
  
  // Enable two stop bits, 8-bit mode, ignore modem-status-lines
  tty.c_cflag |= CSTOPB | CS8 | CLOCAL;

  cfmakeraw(&tty);

  tcflush(com_, TCIFLUSH);
  if ( tcsetattr ( com_, TCSANOW, &tty ) != 0) {
    BOOST_LOG_TRIVIAL(error) << "Error " << errno << " from tcsetattr";
    throw i3ds::CommandError(error_value, "Could not set tty parameters.");
  }
}


void i3ds::ImuDmu30::do_activate() {
  open_device();
}

void i3ds::ImuDmu30::do_start() {
  running_ = true;
  worker_ = std::thread(&i3ds::ImuDmu30::run, this);  
}

void i3ds::ImuDmu30::do_stop() {
  stop();
}

void i3ds::ImuDmu30::do_deactivate() {
  close(com_);
}
