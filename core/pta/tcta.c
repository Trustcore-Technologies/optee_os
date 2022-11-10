
#include <kernel/pseudo_ta.h>

#define TA_NAME		"tcta.ta"

#define TA_TRUSTCORE_UUID \
        { 0x8aaaf200, 0x2450, 0x11e4, \
                { 0xab, 0xe2, 0x00, 0x02, 0xa5, 0xd5, 0xc5, 0x1b} }

#define TA_UUID         TA_TRUSTCORE_UUID

uint32_t trustcore_smc_handler(uint64_t arg1, uint64_t arg2, uint64_t arg3)
{
    EMSG("Trustcore args in pta");
    EMSG("arg1 0x%"PRIx64, arg1);
    EMSG("arg2 0x%"PRIx64, arg2);
    EMSG("arg3 0x%"PRIx64, arg3);

    return 0;
}

TEE_Result tc_entry_point(void)
{
    EMSG("Trustcore pta");
}

static TEE_Result invoke_command(void *psess __unused,
				 uint32_t cmd, uint32_t ptypes,
				 TEE_Param params[TEE_NUM_PARAMS])
{
    EMSG("Trustcore pta invoke command");
    return TEE_ERROR_BAD_PARAMETERS;
}

pseudo_ta_register(.uuid = TA_UUID, .name = TA_NAME,
		   .flags = PTA_DEFAULT_FLAGS,
                   .create_entry_point = tc_entry_point,
                   .invoke_command_entry_point = invoke_command);
