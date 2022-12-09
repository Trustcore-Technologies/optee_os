
#include <kernel/pseudo_ta.h>
#include <mm/core_mmu.h>
#include <sm/optee_smc.h>

#include "tc_shared/tc_common.h"
#include "tc_shared/tc_ipc.h"

#define TA_NAME		"tcta.ta"

#define TA_TRUSTCORE_UUID                                       \
    { 0x8aaaf200, 0x2450, 0x11e4,                               \
      { 0xab, 0xe2, 0x00, 0x02, 0xa5, 0xd5, 0xc5, 0x1b} }

#define TA_UUID         TA_TRUSTCORE_UUID

static tc_shmem_t *shm = NULL;

void tc_handle_data_from_ree(tc_shmem_t *);

uint32_t trustcore_smc_handler(uint8_t sub_id, uint64_t arg1,
			       uint64_t arg2, uint64_t arg3)
{
    DBG("Trustcore args in pta");
    DBG("subid %d", sub_id);
    DBG("arg1 0x%" PRIx64, arg1);
    DBG("arg2 0x%" PRIx64, arg2);
    DBG("arg3 0x%" PRIx64, arg3);

    /* Virtual address of the shared memory */
    vaddr_t *va;

    switch (sub_id) {
    case TC_SHMEM_TEE_SUB_ID:
	va = (vaddr_t *) core_mmu_add_mapping(MEM_AREA_RAM_NSEC,
					      arg1, sizeof(tc_shmem_t));
	if (!va) {
	    ERR("Bad arg address 0x%" PRIxPA, arg1);
	    return OPTEE_SMC_RETURN_EBADADDR;
	}
	shm = (tc_shmem_t *) va;
	INFO("TCT Shared memory at 0x%" PRIxVA, shm);
	break;
    case TC_NOTIFY_DATA_TEE_SUB_ID:
        if(!shm) {
	    ERR("Shared memory not yet initialized");
	    return OPTEE_SMC_RETURN_ENOTAVAIL;
        }
	tc_handle_data_from_ree(shm);
        break;
    default:
	return OPTEE_SMC_RETURN_EBADCMD;
    }
    return OPTEE_SMC_RETURN_OK;
}

static TEE_Result invoke_command(void *psess __unused,
				 uint32_t cmd, uint32_t ptypes,
				 TEE_Param params[TEE_NUM_PARAMS])
{
    ERR("Trustcore pta invoke command");
    return TEE_ERROR_BAD_PARAMETERS;
}

pseudo_ta_register(.uuid = TA_UUID, .name = TA_NAME,
                   .flags = PTA_DEFAULT_FLAGS,
                   .invoke_command_entry_point = invoke_command);
