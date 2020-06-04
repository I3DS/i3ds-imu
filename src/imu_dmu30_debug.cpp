#include "imu_dmu30.hpp"
#include <fcntl.h>      // File control definitions

#define BOOST_LOG_DYN_LINK
#include <boost/format.hpp>
#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>

#include <i3ds/DMU30.h>

int i3ds::ImuDmu30_Debug::open_device()
{
    int fd = open(device_.c_str(), O_RDONLY);
    if (fd <= 0) {
        BOOST_LOG_TRIVIAL(error) << "i3ds::ImuDmu30_Debug::" << __func__ << "() opening " << device_ << " FAILED";
        return -1;
    }
    BOOST_LOG_TRIVIAL(debug) << "i3ds::ImuDmu30_Debug::" << __func__ << "() -> " << device_ << " com_ = " << fd;
    return fd;
}

static bool do_read(int fd, void *buf, size_t n_bytes)
{
    ssize_t rb = 0;
    int retries = 3;

again:
    rb = read(fd, buf, n_bytes);
    if (rb == 0) {
        BOOST_LOG_TRIVIAL(warning) << "i3ds::ImuDmu30_Debug::" << __func__ << "() EOF reached, resetting head";
        lseek(fd, SEEK_SET, 0);
        if (--retries < 0)
            return false;
        goto again;
    }

    if (rb < 0)
        return false;

    return true;
}

// Do same as in imu_dmu30.cpp, i3ds::ImuDmu::read() crashes with
// os read, so drop out of namespace to avoid name collision
bool i3ds::ImuDmu30_Debug::ensure_read(void *buf, size_t n_bytes)
{
    return do_read(com_, buf, n_bytes);
}

bool i3ds::ImuDmu30_Debug::read_single_frame(struct dmu30_frame * frame)
{
    if (com_ < 0)
        return false;

    // Fake slow read, DM30 outputs 200msg/sec
    usleep(5000);

    return i3ds::ImuDmu30::read_single_frame(frame);
}

void i3ds::ImuDmu30_Debug::debug()
{
    BOOST_LOG_TRIVIAL(debug) << "i3ds::ImuDmu30::" << __func__ << "()";
    do_activate();
    do_start();
}
