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

// Do same as in imu_dmu30.cpp, i3ds::ImuDmu::read() crashes with
// os read, so drop out of namespace to avoid name collision
static ssize_t ensure_read(int fd, void *buf, size_t n_bytes) {
    uint16_t *hdr = (uint16_t *)buf;
    uint8_t *ptr = (uint8_t *)buf;
    ssize_t rb = 0;

    do {
        rb = read(fd, ptr, 2);
        if (rb == 0)        // EOF
            return 0;

        if (rb < 0)
            return -1;

        if (rb != 2) {
            BOOST_LOG_TRIVIAL(error) << "i3ds::ImuDmu30_Debug::" << __func__ \
                                     << "() FAILED reading 2 bytes from file! " << rb;

            return -1;
        }
    } while (*hdr != SYNC_BYTE);
    rb += read(fd, ptr+2, n_bytes-2);
    return rb;
}

bool i3ds::ImuDmu30_Debug::read_single_frame(struct dmu30_frame * frame)
{
    // Fake slow read, DM30 outputs 200msg/sec
    usleep(5000);

    if (!frame || com_ < 0)
        return false;

    int retries = 3;
again:
    ssize_t br = ensure_read(com_, frame, sizeof(*frame));

    if (br < 0) {
        BOOST_LOG_TRIVIAL(info) << "i3ds::ImuDmu30_Debug::" << __func__ << "() read FAILED!";
        return false;
    }

    if (br == 0) {
        BOOST_LOG_TRIVIAL(warning) << "i3ds::ImuDmu30_Debug::" << __func__ << "() EOF reached, resetting head";
        lseek(com_, SEEK_SET, 0);
        retries--;
        if (retries > 0)
            goto again;
        return false;
    }

    if (br != sizeof(*frame)) {
        BOOST_LOG_TRIVIAL(error) << "i3ds::ImuDmu30_Debug::" << __func__ << "() read invalid size, we don't know where we are in the file now";
        return false;
    }
    // BOOST_LOG_TRIVIAL(error) << "i3ds::ImuDmu30_Debug::" << __func__ << "() read " << br << " bytes from " << device_;
    return true;
}

void i3ds::ImuDmu30_Debug::debug()
{
    BOOST_LOG_TRIVIAL(debug) << "i3ds::ImuDmu30::" << __func__ << "()";
    do_activate();
    do_start();
}
