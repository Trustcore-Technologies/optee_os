
#include <kernel/pseudo_ta.h>
#include <mm/core_memprot.h>
#include <sm/optee_smc.h>
#include <optee_msg.h>


#define TA_NAME		"tcta.ta"

#define TA_TRUSTCORE_UUID \
        { 0x8aaaf200, 0x2450, 0x11e4, \
                { 0xab, 0xe2, 0x00, 0x02, 0xa5, 0xd5, 0xc5, 0x1b} }

#define TA_UUID         TA_TRUSTCORE_UUID

#define TC_SHMEM_TEE_SUB_ID                       0
#define TC_NOTIFY_DATA_TEE_SUB_ID                 1
#define TC_POLL_DATA_TEE_SUB_ID                   2


typedef struct _tc_mempool_buf tc_mempool_buf_t;

struct _tc_mempool_buf {
    uint8_t data[1600];
    tc_mempool_buf_t *next;
};

typedef struct {
    tc_mempool_buf_t *head;
    tc_mempool_buf_t *tail;
} tc_mempool_t;

typedef struct {
    tc_mempool_buf_t *front;
    tc_mempool_buf_t *back;
} tc_queue_t;

#define MAX_SHMEM_POOL_SIZE 20

typedef struct {
    uint8_t tee_data_ready;
    tc_queue_t ree_tee_q;
    tc_queue_t tee_ree_q;
    tc_mempool_t ree_tee_mempool;
    tc_mempool_buf_t ree_tee_mempool_buf[MAX_SHMEM_POOL_SIZE];
}tc_shmem_t;

uint32_t trustcore_smc_handler(uint8_t sub_id, uint64_t arg1,
                               uint64_t arg2, uint64_t arg3)
{
    EMSG("Trustcore args in pta");
    EMSG("subid %d", sub_id);
    EMSG("arg1 0x%"PRIx64, arg1);
    EMSG("arg2 0x%"PRIx64, arg2);
    EMSG("arg3 0x%"PRIx64, arg3);
    
    /* Virtual address of the shared memory */
    vaddr_t *va;
    tc_shmem_t *tct_shm;

    switch(sub_id)
    {
	    case TC_SHMEM_TEE_SUB_ID:
		    va = (vaddr_t)core_mmu_add_mapping(MEM_AREA_RAM_NSEC,arg1,sizeof(tc_shmem_t));
		    EMSG("VA 0x%" PRIx64 " ", va);
		    if (!va)
                        goto bad_addr;
                    tct_shm = (tc_shmem_t *)va;
		    EMSG("TCT Shared memory at 0x%" PRIx64 " ", tct_shm);
		    EMSG("tee_data_ready arrived as:%d", tct_shm->tee_data_ready);
		    break;
	    case TC_NOTIFY_DATA_TEE_SUB_ID:
		    break;
	    case TC_POLL_DATA_TEE_SUB_ID:
		    break;
    }
    return 0;
    bad_addr:
        EMSG("Bad arg address 0x%"PRIxPA, arg1);
        return OPTEE_SMC_RETURN_EBADADDR;

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
                   .invoke_command_entry_point = invoke_command);
