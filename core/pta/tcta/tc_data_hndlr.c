
#include "tc_shared/tc_common.h"
#include "tc_shared/tc_ipc.h"

void tc_send_data_to_ree(tc_shmem_t * shm)
{
    tc_mempool_buf_t *tcbuf = NULL;
    u16 pkt_len = 0;

    DBG("inside %s", __func__);

    /* Move received packet into a new pbuf */
    pkt_len = TC_NET_MTU;	//min(TC_NET_MTU, mb->m_pktlen);

    DBG("Received pkt len %d", pkt_len);

    tcbuf = tc_mempool_get_buffer(&shm->ree_tee_mempool);
    if (!tcbuf) {
	return;
    }
    //Interface with driver

    tc_queue_push(&shm->tee_ree_q, tcbuf);

}

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
