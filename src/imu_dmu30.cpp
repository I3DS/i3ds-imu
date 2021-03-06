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
#include <math.h>

#include <fcntl.h>      // File control definitions
#include <termios.h>

#ifndef BOOST_LOG_DYN_LINK
#define BOOST_LOG_DYN_LINK
#endif

#include <iostream>
#include <boost/format.hpp>
#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>

#include <sys/stat.h>
bool i3ds::ImuDmu30::valid_device()
{
    struct stat s;
    if (stat(device_.c_str(), &s) == -1) {
        BOOST_LOG_TRIVIAL(error) << "i3ds::ImuDmu30::" << __func__ << "() Provided device (" << device_ << ") not found";
        return false;
    }

    // Should have a valid device-ID, and size should be 0 (character device)
    if (s.st_rdev == 0 || s.st_size != 0) {
        BOOST_LOG_TRIVIAL(error) << "i3ds::ImuDmu30::" << __func__ << "() Invalid device (" << device_ << ")";
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

    frame.sync_bytes = be16toh(frame.sync_bytes);
    frame.message_count = be16toh(frame.message_count);
    uint32_t *ubuf32 = (uint32_t *)&frame;
    size_t n_ints = sizeof(struct dmu30_frame)/4;
    for(size_t i = 1; i < (n_ints-2); i++)
        ubuf32[i] = swap_bytes_32(ubuf32[i]);
    frame.startup_flags = be16toh(frame.startup_flags);
    frame.operation_flags = be16toh(frame.operation_flags);
    frame.error_flags = be16toh(frame.error_flags);
    frame.checksum = be16toh(frame.checksum);

    // Swapping done
    data->message_count = frame.message_count;
    data->axis_x_rate = frame.axis_x_rate;
    data->axis_x_acceleration = frame.axis_x_acceleration;
    data->axis_y_rate = frame.axis_y_rate;
    data->axis_y_acceleration = frame.axis_y_acceleration;
    data->axis_z_rate = frame.axis_z_rate;
    data->axis_z_acceleration = frame.axis_z_acceleration;
    data->aux_input_voltage = frame.aux_input_voltage;
    data->average_temperature = frame.average_temperature + 273.15;
    data->axis_x_delta_theta = frame.axis_x_delta_theta;
    data->axis_x_vel = frame.axis_x_vel;
    data->axis_y_delta_theta = frame.axis_y_delta_theta;
    data->axis_y_vel = frame.axis_y_vel;
    data->axis_z_delta_theta = frame.axis_z_delta_theta;
    data->axis_z_vel = frame.axis_z_vel;
    memcpy(&data->startup_flags, &frame.startup_flags, sizeof(frame.startup_flags));
    memcpy(&data->operation_flags, &frame.operation_flags, sizeof(frame.operation_flags));
    memcpy(&data->error_flags, &frame.error_flags, sizeof(frame.error_flags));

    latest_temp_ = data->average_temperature;
    return true;
}

void i3ds::ImuDmu30::run()
{
    // FIXME: this should probably be make_unique
    auto data = std::make_shared<Message_Type>();
    while(running_) {
        if (!read(data)) {
            BOOST_LOG_TRIVIAL(warning) << "Could not read frame.";
        } else {
            send(data);
        }
    }
}

void i3ds::ImuDmu30::set_batch_size(int batch_size)
{
    if (batch_size < 1 || batch_size > IMU_DMU30_MAX_BATCH)
        throw std::invalid_argument("batch_size out of range ([1..20])");
    batches_ = batch_size;
}

void i3ds::ImuDmu30::debug()
{
    BOOST_LOG_TRIVIAL(info) << "i3ds::ImuDmu30::" << __func__ << "()";
}

bool i3ds::ImuDmu30::is_sampling_supported(i3ds_asn1::SampleCommand sample)
{
    // period for IMU is 200Hz, i.e. 5000us, we batch together samples
    // in increments of 5ms periods
    if (sample.batch_size > 1) {
        BOOST_LOG_TRIVIAL(debug) << "i3ds::ImuDmu30::" << __func__ << "() attempting to set batch-size directly";
        if (sample.batch_size > IMU_DMU30_MAX_BATCH) {
            BOOST_LOG_TRIVIAL(error) << "i3ds::ImuD mu30::" << __func__ << "() invalid batch size ([1..20])";
            return false;
        }
        batches_ = sample.batch_size;
        period_ = IMU_DMU30_BASE_PERIOD_US * batches_;
    } else if (sample.period % IMU_DMU30_BASE_PERIOD_US != 0 ||
               sample.period > IMU_DMU30_BASE_PERIOD_US * IMU_DMU30_MAX_BATCH) {
        BOOST_LOG_TRIVIAL(error)  << "i3ds::ImuDmu30::" << __func__ << "() Invalid period (must be multiple of " << IMU_DMU30_BASE_PERIOD_US << ")";
        return false;
    } else {
        period_ = sample.period;
        batches_ = period_ / IMU_DMU30_BASE_PERIOD_US;
    }

    BOOST_LOG_TRIVIAL(info) << "i3ds::ImuDmu30::" << __func__ << "() period: " << period_;
    BOOST_LOG_TRIVIAL(info) << "i3ds::ImuDmu30::" << __func__ << "() batch_size: " << batches_;

    return true;
}

// --------------------------------------------------
// protected region

static bool do_read(int device, void *buf, size_t n_bytes) {
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
bool i3ds::ImuDmu30::ensure_read(void *buf, size_t n_bytes)
{
    return do_read(com_, buf, n_bytes);
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

int i3ds::ImuDmu30::open_device()
{
    int com;
    BOOST_LOG_TRIVIAL(info) << "i3ds::ImuDmu30::" << __func__ << "()";

  com = open(device_.c_str(), O_RDWR | O_NOCTTY);
  if ( com < 0 )
  {
    BOOST_LOG_TRIVIAL(error) << "Error " << errno << " opening " << "/dev/ttyUSB0" << ": " << strerror (errno);
    throw i3ds::CommandError(i3ds_asn1::ResultCode_error_value, "Could not open device.");
  }

  struct termios tty;
  memset (&tty, 0, sizeof tty);
  if ( tcgetattr ( com, &tty ) != 0 )
  {
    BOOST_LOG_TRIVIAL(error) <<  "Error " << errno << " from tcgetattr: " << strerror(errno);
    throw i3ds::CommandError(i3ds_asn1::ResultCode_error_value, "Could not open tty.");
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
    throw i3ds::CommandError(i3ds_asn1::ResultCode_error_value, "Could not set tty parameters.");
  }

  return com;
}

void i3ds::ImuDmu30::close_device(int device)
{
    BOOST_LOG_TRIVIAL(debug) << "i3ds::ImuDmu30::" << __func__ << "()";
    if (device >= 0)
        close(device);
}

bool i3ds::ImuDmu30::read_single_frame(struct dmu30_frame * frame)
{
    // FIXME: this blocks on incoming data.  Either blocks on device, or
    // blocks on semaphore/conditional (debug-file) to get new data

    if (!frame)
        return false;

  uint8_t *buf = (uint8_t *)frame;
  unsigned int skipped = 0;

  if (!ensure_read(buf, 2)) {
    return false;
  }

  while (frame->sync_bytes != SYNC_BYTE) {
    buf[0] = buf[1];
    if (!ensure_read(buf+1, 1)) {
      return false;
    }
    skipped++;
  }

  if (skipped > 0) {
      BOOST_LOG_TRIVIAL(warning) << "Skipped " << skipped << " bytes to sync.";
  }

  if (!ensure_read(buf + 2, sizeof(*frame)-2)) {
    return false;
  }

  return true;
}

void i3ds::ImuDmu30::send(std::shared_ptr<Message_Type> data)
{
    const double g2ms2 = 9.81;
    const double d2rad = M_PI / 180.0;

    if (msg_idx_ == 0) {
        msg_.batch_size = 0;
        msg_.samples.nCount = 0;
        msg_.attributes.validity = i3ds_asn1::SampleValidity_sample_invalid;
        msg_.attributes.timestamp = get_timestamp();
    }
    // store samples until batches_ or 20
    // samples.arr[20]
    if (msg_idx_ < batches_) {
        msg_.samples.arr[msg_idx_].axis_x_rate         = d2rad * data->axis_x_rate;
        msg_.samples.arr[msg_idx_].axis_x_acceleration = g2ms2 * data->axis_x_acceleration;
        msg_.samples.arr[msg_idx_].axis_y_rate         = d2rad * data->axis_y_rate;
        msg_.samples.arr[msg_idx_].axis_y_acceleration = g2ms2 * data->axis_y_acceleration;
        msg_.samples.arr[msg_idx_].axis_z_rate         = d2rad * data->axis_z_rate;
        msg_.samples.arr[msg_idx_].axis_z_acceleration = g2ms2 * data->axis_z_acceleration;
        msg_idx_++;
        msg_.samples.nCount = msg_idx_;
    }
    if (msg_idx_ == batches_) {
        msg_.attributes.validity = i3ds_asn1::SampleValidity_sample_valid;
        // we don't care about msg_.attributes.arr
        msg_.batch_size = batches_;
        publish_message(msg_);
        msg_idx_ = 0;
    }
}
