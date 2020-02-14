#include <stdio.h>
#include <endian.h>
#include <string.h>
#include <byteswap.h>

#include "imu_helper.hpp"
#define SAMPLE_FILE "test_static.dat"
int main(int argc, char *argv[])
{
	FILE *fp = fopen(SAMPLE_FILE, "wb");
	if (!fp) {
		fprintf(stderr, "Failed top open file");
		return 1;
	}

	struct dmu30_frame fr;
	size_t ctr = 0;

	for (size_t i = 0; i < 2048; i++) {
		memset(&fr, 0, sizeof(struct dmu30_frame));

		fr.sync_bytes = SYNC_BYTE;
		fr.message_count = htobe16(i);

		fr.axis_x_rate         = float_be( 1.0  );
		fr.axis_x_acceleration = float_be( 0.1  );
		fr.axis_y_rate         = float_be( 2.0  );
		fr.axis_y_acceleration = float_be( 0.2  );
		fr.axis_z_rate         = float_be( 3.0  );
		fr.axis_z_acceleration = float_be( 0.3  );
		fr.aux_input_voltage   = float_be( 2.54 );
		fr.average_temperature = float_be(19.47 );
		fr.axis_x_delta_theta  = float_be( 0.01 );
		fr.axis_x_vel          = float_be( 1.0  );
		fr.axis_y_delta_theta  = float_be( 0.01 );
		fr.axis_y_vel          = float_be( 2.0  );
		fr.axis_z_delta_theta  = float_be( 0.005);
		fr.axis_z_vel          = float_be( 3.0  );

		fr.startup_flags = htobe16(0xdead);
		fr.operation_flags = htobe16(0xbeef);
		fr.error_flags = htobe16(0x1337);
		/* Calculate checksum */
		fr.checksum = gen_checksum(&fr);

		/* validate */
		if (!verify_checksum(&fr))
			continue;
		size_t written = fwrite(&fr, 1, sizeof(struct dmu30_frame), fp);
		if (written != sizeof(struct dmu30_frame)) {
			fprintf(stderr, "Failed writing struct to file, only wrote %zu bytes\n", written);
			break;
		}
		ctr++;
	}
	fflush(fp);
	fclose(fp);
	printf("Wrote %zd samples to %s\n", ctr, SAMPLE_FILE);
	return 0;
}
