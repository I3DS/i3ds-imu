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

i3ds::ImuDmu30 *latest_imu = nullptr;

extern "C" {
    void i3ds_handle_imu_message(std::shared_ptr<Message_Type> data, i3ds_asn1::Timepoint timestamp_us )
    {
        i3ds_asn1::IMUMeasurement20 msg;                                \
        msg.samples.nCount = 1;                                     \
        msg.samples.arr[0].axis_x_rate = data->axis_x_rate;         \
        msg.samples.arr[0].axis_x_acceleration = data->axis_x_acceleration; \
        msg.samples.arr[0].axis_y_rate = data->axis_y_rate;         \
        msg.samples.arr[0].axis_y_acceleration = data->axis_y_acceleration; \
        msg.samples.arr[0].axis_z_rate = data->axis_z_rate;         \
        msg.samples.arr[0].axis_z_acceleration = data->axis_z_acceleration; \
        msg.attributes.timestamp = timestamp_us;                    \
        msg.attributes.validity = i3ds_asn1::sample_valid;          \
        msg.batch_size = 1;
        latest_imu->publish_message(msg);
    }

    void i3ds_send_imu_message(i3ds_asn1::IMUMeasurement20 message) {
        BOOST_LOG_TRIVIAL(trace) << "Got message from ADA!";
        if (latest_imu == nullptr){
            BOOST_LOG_TRIVIAL(error) << "Publisher is null. Not transmitting";
            return;
        }
        latest_imu->publish_message(message);
    }
}

i3ds::ImuDmu30::ImuDmu30(Context::Ptr context, i3ds_asn1::NodeID id, std::string device)
  : IMU(id),
    device_(device),
    publisher_(context, id),
    batches_(20),
    msg_idx_(0)
{
  latest_imu = this;
}

void i3ds::ImuDmu30::send(std::shared_ptr<Message_Type> data)
{
    if (msg_idx_ == 0) {
        msg_.batch_size = 0;
        msg_.samples.nCount = 0;
        msg_.attributes.validity = i3ds_asn1::sample_invalid;
        msg_.attributes.timestamp = get_timestamp();
    }
    // store samples until batches_ or 20
    // samples.arr[20]
    if (msg_idx_ < batches_) {
        msg_.samples.arr[msg_idx_].axis_x_rate = data->axis_x_rate;
        msg_.samples.arr[msg_idx_].axis_x_acceleration = data->axis_x_acceleration;
        msg_.samples.arr[msg_idx_].axis_y_rate = data->axis_y_rate;
        msg_.samples.arr[msg_idx_].axis_y_acceleration = data->axis_y_acceleration;
        msg_.samples.arr[msg_idx_].axis_z_rate = data->axis_z_rate;
        msg_.samples.arr[msg_idx_].axis_z_acceleration = data->axis_z_acceleration;
        msg_idx_++;
        msg_.samples.nCount = msg_idx_;
    }
    if (msg_idx_ == batches_) {
        msg_.attributes.validity = i3ds_asn1::sample_valid;
        // we don't care about msg_.attributes.arr
        msg_.batch_size = batches_;
        publish_message(msg_);
        msg_idx_ = 0;
    }
}

void
i3ds::ImuDmu30::run()
{
    // FIXME: this should probably be make_unique
    auto data = std::make_shared<Message_Type>();
    while(running_) {
        if (!read(data)) {
            BOOST_LOG_TRIVIAL(warning) << "Could not read frame.";
        } else {
            send(data);
            //i3ds_handle_imu_message(data, get_timestamp());
        }
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

void i3ds::ImuDmu30::debug()
{
    BOOST_LOG_TRIVIAL(info) << "i3ds::ImuDmu30::" << __func__ << "()";
}

bool i3ds::ImuDmu30::read_single_frame(struct dmu30_frame * frame)
{
    // FIXME: this blocks on incoming data.  Either blocks on device, or
    // blocks on semaphore/conditional (debug-file) to get new data

  uint8_t *buf = (uint8_t *)frame;
  unsigned int skipped = 0;

  if (!ensure_read(com_, buf, 2)) {
    return false;
  }

  while (frame->sync_bytes != SYNC_BYTE) {
    buf[0] = buf[1];
    if (!ensure_read(com_, buf+1, 1)) {
      return false;
    }
    skipped++;
  }

  if (skipped > 0) {
    BOOST_LOG_TRIVIAL(warning) << "Skipped " << skipped << " bytes to sync.";
  }

  if (!ensure_read(com_, buf + 2, sizeof(*frame)-2)) {
    return false;
  }

  return true;
}

bool i3ds::ImuDmu30::read(const std::shared_ptr<Message_Type> data)
{
    struct dmu30_frame frame;

    if (!read_single_frame(&frame))
        return false;

    if (!verify_checksum(&frame)) {
        BOOST_LOG_TRIVIAL(warning) << "Checksum error!";
        return false;
    }

    uint16_t *u16buf = (uint16_t *)&frame;
    size_t n_shorts = sizeof(struct dmu30_frame)/2;
    uint32_t *ubuf32 = (uint32_t *)&frame;
    size_t n_ints = sizeof(struct dmu30_frame)/4;

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

    return true;
}

bool i3ds::ImuDmu30::is_sampling_supported(i3ds_asn1::SampleCommand) {
  // TODO(sigurdal) implement?
  return true;
}

int i3ds::ImuDmu30::open_device()
{
    int com;
    BOOST_LOG_TRIVIAL(info) << "i3ds::ImuDmu30::" << __func__ << "()";

  com = open(device_.c_str(), O_RDWR | O_NOCTTY);
  if ( com < 0 )
  {
    BOOST_LOG_TRIVIAL(error) << "Error " << errno << " opening " << "/dev/ttyUSB0" << ": " << strerror (errno);
    throw i3ds::CommandError(i3ds_asn1::error_value, "Could not open device.");
  }

  struct termios tty;
  memset (&tty, 0, sizeof tty);
  if ( tcgetattr ( com, &tty ) != 0 )
  {
    BOOST_LOG_TRIVIAL(error) <<  "Error " << errno << " from tcgetattr: " << strerror(errno);
    throw i3ds::CommandError(i3ds_asn1::error_value, "Could not open tty.");
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

  tcflush(com, TCIFLUSH);
  if ( tcsetattr ( com, TCSANOW, &tty ) != 0) {
    BOOST_LOG_TRIVIAL(error) << "Error " << errno << " from tcsetattr";
    throw i3ds::CommandError(i3ds_asn1::error_value, "Could not set tty parameters.");
  }

  return com;
}

void i3ds::ImuDmu30::close_device(int device)
{
    BOOST_LOG_TRIVIAL(debug) << "i3ds::ImuDmu30::" << __func__ << "()";
    if (device >= 0)
        close(device);
}

void i3ds::ImuDmu30::do_activate() {
    BOOST_LOG_TRIVIAL(debug) << "i3ds::ImuDmu30::" << __func__ << "()";
    com_ = open_device();
}

void i3ds::ImuDmu30::do_start() {
    running_ = true;
    BOOST_LOG_TRIVIAL(debug) << "i3ds::ImuDmu30::" << __func__ << "()";
    worker_ = std::thread(&i3ds::ImuDmu30::run, this);
}

void i3ds::ImuDmu30::do_stop() {
    BOOST_LOG_TRIVIAL(debug) << "i3ds::ImuDmu30::" << __func__ << "()";
    stop();
}

void i3ds::ImuDmu30::do_deactivate() {
    BOOST_LOG_TRIVIAL(debug) << "i3ds::ImuDmu30::" << __func__ << "()";
    close_device(com_);
    com_ = -1;
}
