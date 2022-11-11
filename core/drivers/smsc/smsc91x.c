/*
 * smsc91x.c
 *
 *  Created on: Nov 10, 2022
 *      Author: rami
 */


#include <assert.h>
#include <drivers/smsc/smsc91x.h>
#include <config.h>
#include <initcall.h>
#include <kernel/dt.h>
#include <kernel/dt_driver.h>
#include <libfdt.h>
#include <malloc.h>
#include <sys/queue.h>
#include <tee_api_defines_extensions.h>
#include <tee_api_types.h>


static TEE_Result
smsc91x_probe(const void *fdt __unused, int dev, const void *data) {
	EMSG("SMSC bitches!!!!!");
	return 0;
}


static const struct dt_device_match smsc91x_match_table[] = {
	{ .compatible = "smc91x", },
	{ }
};

DEFINE_DT_DRIVER(smc_dt_driver) = {
	.name = "smc91x",
	.type = DT_DRIVER_NOTYPE,
	.match_table = smsc91x_match_table,
	.probe = smsc91x_probe,
};

