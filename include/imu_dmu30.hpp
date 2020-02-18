///////////////////////////////////////////////////////////////////////////\file
///
///   Copyright 2018 SINTEF AS
///
///   This Source Code Form is subject to the terms of the Mozilla
///   Public License, v. 2.0. If a copy of the MPL was not distributed
///   with this file, You can obtain one at https://mozilla.org/MPL/2.0/
///
////////////////////////////////////////////////////////////////////////////////

#ifndef __I3DS_IMU_DMU30_HPP
#define __I3DS_IMU_DMU30_HPP

#include <i3ds/publisher.hpp>
#include <i3ds/periodic.hpp>

#include <i3ds/imu_sensor.hpp>
#include "i3ds/DMU30.h"

#include "imu_helper.hpp"
namespace i3ds
{

class ImuDmu30 : public IMU
{

  public:
    typedef std::shared_ptr<ImuDmu30> Ptr;

    static Ptr Create(i3ds::Context::Ptr context, i3ds_asn1::NodeID id, std::string device)
    {
      return std::make_shared<ImuDmu30>(context, id, device);
    }

    ImuDmu30(i3ds::Context::Ptr context, i3ds_asn1::NodeID id, std::string device);
    ~ImuDmu30(){
      stop();
    }

    // Supported period.
    virtual bool is_sampling_supported(i3ds_asn1::SampleCommand sample);

    bool read(const std::shared_ptr<Message_Type> data);
    void run();
    void stop() {
      running_ = false;
      if (worker_.joinable()) {
	worker_.join();
      }
    }

    void publish_message(i3ds_asn1::IMUMeasurement20 &message) {
      publisher_.Send<MeasurementTopic>(message);
    }

    void set_name(std::string name) {
        set_device_name(name);
    }

    virtual void debug();

protected:

    // Actions.
    virtual void do_activate();
    virtual void do_start();
    virtual void do_stop();
    virtual void do_deactivate();

    // IMU specific operations
    virtual int open_device();
    void close_device(int device);
    virtual bool read_single_frame(struct dmu30_frame *);
    void send(std::shared_ptr<Message_Type> data);

    // Called periodically to accumulated and send samples.
    bool send_sample(unsigned long timestamp_us);

    // Generat a single ADC reading with specified resolution.
    std::vector<float> read_imu();


    std::string device_;
    int com_;

private:
    Publisher publisher_;

    // MeasurementTopic::Data frame_;

      std::atomic<bool> running_;

    // Number of batches inserted.
    i3ds_asn1::BatchCount batches_;
    i3ds_asn1::IMUMeasurement20 msg_;
    i3ds_asn1::BatchCount msg_idx_;

    std::thread worker_;
};

class ImuDmu30_Debug : public ImuDmu30
{
    public:
        static Ptr Create(i3ds::Context::Ptr context, i3ds_asn1::NodeID id, std::string debug_file)
        {
            return std::make_shared<ImuDmu30_Debug>(context, id, debug_file);
        }

        ImuDmu30_Debug(i3ds::Context::Ptr context,
                       i3ds_asn1::NodeID id,
                       std::string device) :
            i3ds::ImuDmu30(context, id, device)
    {};

    virtual int open_device();
    virtual bool read_single_frame(struct dmu30_frame *);
    virtual void debug();
};

} // namespace i3ds

#endif // __I3DS_IMU_DMU30_HPP
