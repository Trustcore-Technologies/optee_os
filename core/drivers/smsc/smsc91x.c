/*
 * smsc91x.c
 *
 *  Created on: Nov 10, 2022
 *      Author: rami
 */


#include <assert.h>
#include <drivers/smsc/smsc91x.h>
#include <drivers/rtc.h>
#include <io.h>
#include <kernel/dt.h>


static int
smsc91x_probe(const void *fdt __unused, struct device *dev, const void *data) {
	EMSG("SMSC bitches!!!!!");
	return 0;
}


static const struct dt_device_match smsc91x_match_table[] = {
	{ .compatible = "smsc" },
	{ }
};

DEFINE_DT_DRIVER(smsc91x_dt_driver) = {
	.name = "smc",
	.type = DT_DRIVER_NOTYPE,
	.match_table = smsc91x_match_table,
	.probe = smsc91x_probe,
};
