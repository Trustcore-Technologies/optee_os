
#include "tc_shared/tc_common.h"
#include "tc_shared/tc_ipc.h"

TC_ATOMIC_SET(u8);

#if 0
/*
This is just a placeholder to illustrate the queue and mempool interaction
*/
void tc_send_data_to_ree(tc_shmem_t * shm)
{
    tc_mempool_buf_t *tcbuf = NULL;
    u16 pkt_len = 0;

    DBG("inside %s", __func__);

    /* Move received packet into a new pbuf */
    pkt_len = TC_NET_MTU;	//min(TC_NET_MTU, mb->m_pktlen);

    DBG("Received pkt of len %d to send out to REE", pkt_len);

    tcbuf = tc_mempool_get_buffer(&shm->ree_tee_mempool);
    if (!tcbuf) {
	return;
    }
	
    //Copy driver's data into tcbuf
    
    tc_queue_push(&shm->tee_ree_q, tcbuf);

    //Mark to inform REE that TEE has placed packets in queue
    tc_atomic_set_u8(&shm->tee_data_ready, 1);
}
#endif

void tc_handle_data_from_ree(tc_shmem_t * shm)
{
    tc_mempool_buf_t *tcbuf = NULL;
    u16 num_pkts, pkt_len = 0;

    DBG("inside %s", __func__);

    num_pkts = 0;
    do {
	tcbuf = tc_queue_pop(&shm->ree_tee_q);
	if (!tcbuf) {
	    break;
	}
	pkt_len = tcbuf->pkt_len;

	//Interface with driver

	num_pkts++;
	tc_mempool_free_buffer(&shm->ree_tee_mempool, tcbuf);
    } while (1);
    DBG("End of received packet processing, processed %d packets", num_pkts);
}
