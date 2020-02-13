/* $Id$ */
/*
 * Copyright (C) 2008-2011 Teluu Inc. (http://www.teluu.com)
 * Copyright (C) 2003-2008 Benny Prijono <benny@prijono.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/*
 * Transport Adapter :
 *    - add PTT state to sip
 *    - PTT state is transmitted on RTP packet as extended headers.
 *    extended header format :
 *        PTT_id 	uint32_t
 *        PTT_state uint32_t
 *
 */

#include "transport_adapter.h"
#include <pj/assert.h>
#include <pj/log.h>
#include <pj/pool.h>
#include <pjmedia/endpoint.h>
#include <pjsua.h>

/* Transport functions prototypes */
static pj_status_t transport_get_info(pjmedia_transport *tp,
                                      pjmedia_transport_info *info);
static pj_status_t
transport_attach(pjmedia_transport *tp, void *user_data,
                 const pj_sockaddr_t *rem_addr, const pj_sockaddr_t *rem_rtcp,
                 unsigned addr_len, void (*rtp_cb)(void *, void *, pj_ssize_t),
                 void (*rtcp_cb)(void *, void *, pj_ssize_t));
static void transport_detach(pjmedia_transport *tp, void *strm);
static pj_status_t transport_send_rtp(pjmedia_transport *tp, const void *pkt,
                                      pj_size_t size);
static pj_status_t transport_send_rtcp(pjmedia_transport *tp, const void *pkt,
                                       pj_size_t size);
static pj_status_t transport_send_rtcp2(pjmedia_transport *tp,
                                        const pj_sockaddr_t *addr,
                                        unsigned addr_len, const void *pkt,
                                        pj_size_t size);
static pj_status_t transport_media_create(pjmedia_transport *tp,
                                          pj_pool_t *sdp_pool, unsigned options,
                                          const pjmedia_sdp_session *rem_sdp,
                                          unsigned media_index);
static pj_status_t transport_encode_sdp(pjmedia_transport *tp,
                                        pj_pool_t *sdp_pool,
                                        pjmedia_sdp_session *local_sdp,
                                        const pjmedia_sdp_session *rem_sdp,
                                        unsigned media_index);
static pj_status_t transport_media_start(pjmedia_transport *tp, pj_pool_t *pool,
                                         const pjmedia_sdp_session *local_sdp,
                                         const pjmedia_sdp_session *rem_sdp,
                                         unsigned media_index);
static pj_status_t transport_media_stop(pjmedia_transport *tp);
static pj_status_t transport_simulate_lost(pjmedia_transport *tp,
                                           pjmedia_dir dir, unsigned pct_lost);
static pj_status_t transport_destroy(pjmedia_transport *tp);

/* The transport operations */
static struct pjmedia_transport_op tp_adapter_op = {
    &transport_get_info,     &transport_attach,        &transport_detach,
    &transport_send_rtp,     &transport_send_rtcp,     &transport_send_rtcp2,
    &transport_media_create, &transport_encode_sdp,    &transport_media_start,
    &transport_media_stop,   &transport_simulate_lost, &transport_destroy};

tp_adapter_t *ptt_adapter;

pj_caching_pool adapter_cache;
pj_pool_t *adapter_pool;
pjmedia_rtp_ext_hdr exthdr_init = {
    (pj_uint16_t)0x0010,     (pj_uint16_t)0x0400,     (pj_uint32_t)0x00000000,
    (pj_uint32_t)0x00000000};

void pjmedia_ptt_setmedia(int id_media, int id_sipmedia, int vox_idle) {
  if (id_media > 0)
    ptt_adapter->id_media = id_media;
  if (id_sipmedia > 0)
    ptt_adapter->id_sipmedia = id_sipmedia;
  if (vox_idle >= 0)
    ptt_adapter->vox_idle = vox_idle;
  PJ_LOG(3, ("TRANS>", "MEDIA (%d:%d:%d) LOC: %d, REM: %d, VOX: %d", id_media,
             id_sipmedia, vox_idle, ptt_adapter->id_media,
             ptt_adapter->id_sipmedia, ptt_adapter->vox_idle));
}

/*
 * Create the adapter.
 */
PJ_DEF(pj_status_t)
pjmedia_adapter_create(const char *name, pjmedia_transport *transport,
                           pj_bool_t del_base, pjmedia_transport **p_tp) {
  struct tp_adapter *adapter;

  if (name == NULL)
    name = "tpPTT%p";

  /* Create the pool and initialize the adapter structure */
  PJ_LOG(1, ("TRANS>", "%s, Creating Adapter ...", transport->name));
  pj_caching_pool_init(&adapter_cache, NULL, 0);
  adapter_pool = pj_pool_create(&adapter_cache.factory, name, 512, 512, NULL);
  adapter = PJ_POOL_ZALLOC_T(adapter_pool, struct tp_adapter);
  adapter->pool = adapter_pool;
  pj_ansi_strncpy(adapter->base.name, adapter_pool->obj_name,
                  sizeof(adapter->base.name));
  adapter->base.type =
      (pjmedia_transport_type)(PJMEDIA_TRANSPORT_TYPE_USER + 1);
  adapter->base.op = &tp_adapter_op;

  /* Save the transport as the slave transport */
  adapter->slave_tp = transport;
  adapter->del_base = del_base;
  adapter->ptt_State = PTT_IDLE | PTT_TX_LOC_IDLE;
  adapter->ptt_data.ptt_id = 3000;
  adapter->init_state = PJ_TRUE;

  /* Done */
  *p_tp = &adapter->base;
  ptt_adapter = adapter;
  PJ_LOG(1, ("TRANS>", " ADPATER %s Created ...", adapter->base.name));
  return PJ_SUCCESS;
}

pj_uint16_t pjmedia_ptt_get_state() {
  return ((pj_uint16_t)ptt_adapter->ptt_State);
}

void pjmedia_ptt_stopTx() {
  PJ_LOG(3, ("TRANS>", "SIP_AUDIO STOPPED..."));
  if (ptt_adapter->ptt_State & (PTT_TX_LOC_GRANTED))
    ptt_adapter->ptt_State &= ~PTT_TX_LOC_GRANTED;
    ptt_adapter->ptt_State |= PTT_TX_LOC_END;
    ptt_adapter->end_cnt = 1;
    ptt_adapter->vox_idle = 0;
    return;
    // pjsua_conf_disconnect(ptt_adapter->id_media, ptt_adapter->id_sipmedia);
    //}
  }
}

void pjmedia_ptt_outCall() {
  if (ptt_adapter->init_state) {
    PJ_LOG(3, ("TRANS>>", "AISGW Outgoing Call started ..."));
    ptt_adapter->init_state = PJ_FALSE;
    ptt_adapter->ptt_State |= PTT_IN_CALL;
    ptt_adapter->out_call = PJ_TRUE;
  }
}

/*
void pjmedia_ptt_idle() {
    PJ_LOG(3,("TRANS>","AIS TX LOC IDLE"));
    ptt_adapter->ptt_State ~= PTT_TX_LOC_IDLE;
    ptt_adapter->ptt_State |= PTT_TX_LOC_IDLE;
}
*/

/*
 * get_info() is called to get the transport addresses to be put
 * in SDP c= line and a=rtcp line.
 */
static pj_status_t transport_get_info(pjmedia_transport *tp,
                                      pjmedia_transport_info *info) {
  struct tp_adapter *adapter = (struct tp_adapter *)tp;

  /* Since we don't have our own connection here, we just pass
   * this function to the slave transport.
   */

  return pjmedia_transport_get_info(adapter->slave_tp, info);
}

static void transport_update_ptt_rem_state(tp_adapter_t *adapter,
                                           pj_uint16_t seq,
                                           pjmedia_rtp_ext_hdr *rtpexthdr) {
  adapter->ptt_data.to_id = rtpexthdr->ptt_id_to;
  adapter->ptt_data.from_id = rtpexthdr->ptt_id_from;
  // adapter->tx_sql = rtpexthdr->ptt_tx_sql;
  // adapter->tx_data = rtpexthdr->ptt_idx;
  PJ_LOG(3, ("TRANS>", "REM RTP-EXTHDR: %04X, PTT_STATE: %04X",
             pj_ntohl(rtpexthdr->ptt_tx_sql), adapter->ptt_State));
  if (adapter->init_state) {
    PJ_LOG(3, ("TRANS>", " AISGW Incomming Call"));
    if (adapter->ptt_State & (PTT_IDLE | PTT_TX_LOC_IDLE)) {
      if (pj_ntohl(rtpexthdr->ptt_tx_sql) & (PTT_REQ_MASK | PTT_REM_TX_MASK)) {
        PJ_LOG(3, ("TRANS>", " PTT_REM TX REQ ..."));
      }
      adapter->ptt_State = (PTT_IN_CALL | PTT_TX_REM_REQ | PTT_TX_LOC_IDLE);
      adapter->init_state = PJ_FALSE;
      return;
    }
  }

  if (adapter->ptt_State & PTT_IN_CALL) {
    if (adapter->ptt_State & (PTT_TX_LOC_REQ | PTT_TX_LOC_WAIT)) {
      if (pj_ntohl(rtpexthdr->ptt_tx_sql) &
          (PTT_TX_REQ_GRANTED + adapter->ptt_cnt)) {
        if ((pj_ntohl(rtpexthdr->ptt_tx_sql) & PTT_REQ_MASK) == 0) {
          PJ_LOG(3, ("TRANS>", " PTT_LOC TX Granted ..."));
          adapter->ptt_State &= ~(PTT_TX_LOC_REQ | PTT_TX_LOC_WAIT);
          adapter->ptt_State |= PTT_TX_LOC_GRANTED;
          pjsua_conf_connect(adapter->id_media, adapter->id_sipmedia);
          return;
        }
      }
      return;
    }
  }
  /*
          if (adapter->ptt_State & (PTT_IN_CALL|PTT_TX_LOC_IDLE)) {
                  if (rtpexthdr->ptt_tx_sql & (PTT_TX_REQN)) {
                          adapter->ptt_State &= ~PTT_TX_LOC_IDLE;
                          return;
                  }
                  return;
          }

          if (rtpexthdr->ptt_tx_sql & (PTT_REQ_MASK | PTT_REM_TX_MASK)) {
                  PJ_LOG(3,("TRANS>"," PTT_REM TX REQ ..."));
                  adapter->ptt_State |= PTT_TX_REM_REQ;
                  return;
          }
  */
  //	if ((pj_ntohl(rtpexthdr->ptt_tx_sql) & PTT_REQ_MASK) == 0) {
  if (pj_ntohl(rtpexthdr->ptt_tx_sql) & (PTT_REM_REQN_MASK)) {
    PJ_LOG(3, ("TRANS>", " PTT_REM TX GRANTED ..."));
    adapter->ptt_State &= ~PTT_TX_REM_REQ;
    adapter->ptt_State |= PTT_TX_REM_GRANTED;
    adapter->vox_idle = 25;
    pjsua_conf_connect(adapter->id_sipmedia, adapter->id_media);
    // pjsua_conf_disconnect(adapter->id_media, adapter->id_sipmedia);
    return;
  }
  //		return;
  //	}

  if (pj_ntohl(rtpexthdr->ptt_tx_sql) & (PTT_REM_TX_MASK)) {
    if (pj_ntohl(rtpexthdr->ptt_tx_sql) & (PTT_LOC_TX_MASK)) {
      if (adapter->ptt_State & PTT_TX_REQ_END) {
        adapter->end_cnt++;
        if (adapter->end_cnt >= 3) {
          PJ_LOG(3, ("TRANS>", " PTT_REM TX END ..."));
          adapter->ptt_State &= ~PTT_TX_REQ_END;
          adapter->ptt_State = (PTT_IN_CALL | PTT_TX_LOC_IDLE);
          pjsua_conf_disconnect(adapter->id_sipmedia, adapter->id_media);
          return;
        }
        PJ_LOG(3, ("TRANS>", " PTT_REM TX END REQ %d...", adapter->end_cnt));
        // pjsua_conf_connect(adapter->id_media, adapter->id_sipmedia);
        return;
      }

      adapter->ptt_State &= ~(PTT_TX_REM_REQ | PTT_TX_REM_GRANTED);
      adapter->ptt_State |= PTT_TX_REQ_END;
      adapter->end_cnt = 1;
      PJ_LOG(3, ("TRANS>", " PTT_REM TX END REQ %d...", adapter->end_cnt));
      return;
    }
  }
  /*
    if (pj_ntohl(rtpexthdr->ptt_tx_sql) & (PTT_REQ_MASK | PTT_LOC_TX_MASK)) {
      if (adapter->ptt_State & PTT_TX_REQ_END) {
        PJ_LOG(3, ("TRANS>", " PTT_REM TX END ..."));
        adapter->ptt_State &= ~PTT_TX_REQ_END;
        adapter->ptt_State = (PTT_IN_CALL | PTT_TX_LOC_IDLE);
        pjsua_conf_disconnect(adapter->id_sipmedia, adapter->id_media);
        // pjsua_conf_connect(adapter->id_media, adapter->id_sipmedia);
        return;
      }
    }
    */
}

static pj_status_t transport_update_ptt_loc_state(tp_adapter_t *adapter,
                                                  pj_uint16_t seq,
                                                  pjmedia_rtp_ext_hdr *exthdr) {
  //*exthdr = (pjmedia_rtp_ext_hdr) ptt_init;
  //	pj_memcpy((pj_uint8_t *)exthdr, (pj_uint8_t *)&ptt_init,
  // sizeof(pjmedia_rtp_ext_hdr));
  if (adapter->ptt_State &
      (PTT_TX_REM_GRANTED | PTT_TX_REM_REQ | PTT_TX_REQ_END)) {
    return PJ_EBUSY;
  }

  if (((adapter->ptt_State & PTT_IN_CALL) == 0) ||
      (adapter->ptt_State & PTT_TX_LOC_WAIT) || (adapter->vox_idle > 21))
    return PJ_EBUSY;

  *exthdr = ptt_init;

  adapter->ptt_data.to_id = 901;
  exthdr->ptt_id_to = pj_htonl(901);
  adapter->ptt_data.from_id = adapter->ptt_data.ptt_id;
  exthdr->ptt_id_from = pj_htonl(adapter->ptt_data.ptt_id);

  // adapter->tx_sql = exthdr.ptt_tx_sql;
  // adater->tx_data = exthdr.ptt_idx;
  PJ_LOG(5, ("TRANS>", " LOC PTT_STATE: %04X", adapter->ptt_State));

  if (adapter->ptt_State & PTT_TX_LOC_IDLE) {
    PJ_LOG(3, ("TRANS>", " PTT_LOC TX REQ ..."));
    exthdr->ptt_tx_sql = pj_htonl(PTT_TX_REQ + adapter->ptt_cnt);
    exthdr->ptt_idx = 0x0;
    adapter->ptt_State &= ~PTT_TX_LOC_IDLE;
    adapter->ptt_State |= PTT_TX_LOC_WAIT;
    return PJ_SUCCESS;
  }
  /*
    if (adapter->ptt_State & PTT_TX_LOC_REQ) {
      PJ_LOG(3, ("TRANS>", " PTT_LOC TX REQ NEXT ..."));
      exthdr->ptt_tx_sql = pj_htonl(PTT_TX_REQ);
      exthdr->ptt_idx = pj_htonl(PTT_TX_DATA);
      adapter->ptt_State &= ~PTT_TX_LOC_REQ;
      adapter->ptt_State |= PTT_TX_LOC_REQN;
      return PJ_SUCCESS;
    }

    if ((adapter->ptt_State & PTT_TX_LOC_REQN)) {
      PJ_LOG(3, ("TRANS>", " PTT_LOC TX Waiting for granted ..."));

      // if (adapter->ptt_State & PTT_TX_LOC_REQN) {
      exthdr->ptt_tx_sql = pj_htonl(PTT_TX_REQN);
      exthdr->ptt_idx = pj_htonl(PTT_TX_DATAN);
      adapter->ptt_State &= ~PTT_TX_LOC_REQN;
      adapter->ptt_State |= PTT_TX_LOC_WAIT;
      //}
      return PJ_SUCCESS;
    }
  */
  if ((adapter->ptt_State & PTT_TX_LOC_WAIT)) {
    PJ_LOG(3, ("TRANS>", " PTT_LOC TX Waiting for granted ..."));

    // if (adapter->ptt_State & PTT_TX_LOC_REQN) {
    // exthdr->ptt_tx_sql = pj_htonl(PTT_TX_REQN);
    // exthdr->ptt_idx = pj_htonl(PTT_TX_DATAN);
    // adapter->ptt_State &= ~PTT_TX_LOC_REQN;
    // adapter->ptt_State |= PTT_TX_LOC_WAIT;
    //}
    return PJ_SUCCESS;
  }

  if (adapter->ptt_State & (PTT_TX_LOC_GRANTED)) {
    if (adapter->in_call == PJ_FALSE)
      PJ_LOG(3, ("TRANS>", " PTT_LOC TX Granted ..."));
    adapter->in_call = PJ_TRUE;
    exthdr->ptt_tx_sql = pj_htonl(PTT_TX_CMD + adapter->ptt_cnt);
    exthdr->ptt_idx = 0x0;
    // adapter->ptt_cnt +=2;
    //	pjsua_conf_connect(adapter->id_media, adapter->id_sipmedia);
    // adapter->ptt_State |= PTT_TX_LOC_;
    return PJ_SUCCESS;
  }

  if ((adapter->ptt_State & PTT_TX_LOC_END)) {
    PJ_LOG(3, ("TRANS>", " PTT_LOC TX REQ END %d ...", adapter->end_cnt));
    if (adapter->end_cnt == 1)
      adapter->ptt_cnt += 2;
    exthdr->ptt_tx_sql = pj_htonl(PTT_TX_CMD_END + adapter->ptt_cnt);
    exthdr->ptt_idx = 0x0;
    if (adapter->end_cnt >= 3) {
      adapter->ptt_State &= ~PTT_TX_LOC_END;
      adapter->ptt_State |= PTT_TX_LOC_REQ_END;
    }
    adapter->end_cnt += 1;

    // adapter->end_cnt = 1;
    return PJ_SUCCESS;
  }

  if (adapter->ptt_State & PTT_TX_LOC_REQ_END) {
    if (adapter->end_cnt < 3) {
      adapter->end_cnt += 1;
      PJ_LOG(3, ("TRANS>", " PTT_LOC TX REQ END %d ...", adapter->end_cnt));

      exthdr->ptt_tx_sql = pj_htonl(PTT_TX_CMD_END + adapter->ptt_cnt);
      exthdr->ptt_idx = 0x0;
      // adapter->ptt_State &= ~PTT_TX_LOC_END;
      adapter->ptt_State |= PTT_TX_LOC_REQ_END;
      // adapter->end_cnt += 1;
      return PJ_SUCCESS;
    }
    PJ_LOG(3, ("TRANS>", " PTT_LOC TX END ..."));
    exthdr->ptt_tx_sql = pj_htonl(PTT_TX_END);
    exthdr->ptt_idx = pj_htonl(PTT_TX_DATA);
    // adapter->ptt_cnt+=2;
    adapter->ptt_State &= ~PTT_TX_LOC_REQ_END;
    adapter->ptt_State |= (PTT_IN_CALL | PTT_TX_LOC_IDLE);
    // adapter->in_call = PJ_FALSE;
    adapter->vox_idle = 25;
    pjsua_conf_disconnect(ptt_adapter->id_media, ptt_adapter->id_sipmedia);
    return PJ_SUCCESS;
  }
}

/* This is our RTP callback, that is called by the slave transport when it
 * receives RTP packet.
 */
static void transport_rtp_cb(void *user_data, void *pkt, pj_ssize_t size) {
  struct tp_adapter *adapter = (struct tp_adapter *)user_data;
  pj_uint16_t *hdr = (pj_uint16_t *)(pkt);

  pjmedia_rtp_hdr *rtphdr = (pjmedia_rtp_hdr *)pkt;

  PJ_LOG(3, ("TRANS>", "RTP CB, Hdr: %04X, Seq: %d, Hdr.X: %X, HDR.PT: %d",
             *hdr, pj_ntohs(rtphdr->seq), rtphdr->x, rtphdr->pt));
  pj_assert(adapter->stream_rtp_cb != NULL);

  // pjmedia_rtp_hdr *rtphdr;
  pjmedia_rtp_ext_hdr *rtpexthdr;
  pj_uint8_t *payload;
  pj_uint16_t offset;

  if (rtphdr->x) {
    rtpexthdr = (pjmedia_rtp_ext_hdr *)(pkt + sizeof(pjmedia_rtp_hdr));
    transport_update_ptt_rem_state(adapter, rtphdr->seq, rtpexthdr);
    offset = (sizeof(pjmedia_rtp_hdr) + sizeof(pjmedia_rtp_ext_hdr));
  } else
    offset = (sizeof(pjmedia_rtp_hdr));

  payload = ((pj_uint8_t *)pkt) + offset;
  /* Call stream's callback */
  adapter->stream_rtp_cb(adapter->stream_user_data, pkt, size);
}

/* This is our RTCP callback, that is called by the slave transport when it
 * receives RTCP packet.
 */
static void transport_rtcp_cb(void *user_data, void *pkt, pj_ssize_t size) {
  struct tp_adapter *adapter = (struct tp_adapter *)user_data;

  pj_assert(adapter->stream_rtcp_cb != NULL);

  /* Call stream's callback */
  adapter->stream_rtcp_cb(adapter->stream_user_data, pkt, size);
}

/*
 * attach() is called by stream to register callbacks that we should
 * call on receipt of RTP and RTCP packets.
 */
static pj_status_t
transport_attach(pjmedia_transport *tp, void *user_data,
                 const pj_sockaddr_t *rem_addr, const pj_sockaddr_t *rem_rtcp,
                 unsigned addr_len, void (*rtp_cb)(void *, void *, pj_ssize_t),
                 void (*rtcp_cb)(void *, void *, pj_ssize_t)) {
  struct tp_adapter *adapter = (struct tp_adapter *)tp;
  pj_status_t status;

  /* In this example, we will save the stream information and callbacks
   * to our structure, and we will register different RTP/RTCP callbacks
   * instead.
   */
  PJ_LOG(3, ("TRANS>", "%s, Adapter attaching, PTT_STATE %04X", tp->name,
             adapter->ptt_State));
  pj_assert(adapter->stream_user_data == NULL);
  adapter->stream_user_data = user_data;
  adapter->stream_rtp_cb = rtp_cb;
  adapter->stream_rtcp_cb = rtcp_cb;
  adapter->ptt_State = PTT_IDLE | PTT_TX_LOC_IDLE;

  status =
      pjmedia_transport_attach(adapter->slave_tp, adapter, rem_addr, rem_rtcp,
                               addr_len, &transport_rtp_cb, &transport_rtcp_cb);
  if (status != PJ_SUCCESS) {
    PJ_LOG(3,
           ("TRANS>", "Adapter attachment is not Success, Status %X", status));
    adapter->stream_user_data = NULL;
    adapter->stream_rtp_cb = NULL;
    adapter->stream_rtcp_cb = NULL;
    return status;
  }
  PJ_LOG(3, ("TRANS>", "%s, Adapter attached, PTT_STATE %04X",
             adapter->slave_tp->name, adapter->ptt_State));

  return PJ_SUCCESS;
}

/*
 * detach() is called when the media is terminated, and the stream is
 * to be disconnected from us.
 */
static void transport_detach(pjmedia_transport *tp, void *strm) {
  struct tp_adapter *adapter = (struct tp_adapter *)tp;

  PJ_UNUSED_ARG(strm);
  PJ_LOG(3, ("TRANS>", "Adapter detached .."));

  if (adapter->stream_user_data != NULL) {
    pjmedia_transport_detach(adapter->slave_tp, adapter);
    adapter->stream_user_data = NULL;
    adapter->stream_rtp_cb = NULL;
    adapter->stream_rtcp_cb = NULL;
  }
}

/*
 * send_rtp() is called to send RTP packet. The "pkt" and "size" argument
 * contain both the RTP header and the payload.
 */
static pj_status_t transport_send_rtp(pjmedia_transport *tp, const void *pkt,
                                      pj_size_t size) {
  struct tp_adapter *adapter = (struct tp_adapter *)tp;
  pjmedia_rtp_hdr *rtphdr;
  pjmedia_rtp_ext_hdr rtpexthdr;
  pj_uint8_t payload[size + sizeof(pjmedia_rtp_ext_hdr) + 32];
  pj_uint16_t offset;
  pj_uint16_t *hdr = (pj_uint16_t *)pkt;
  rtphdr = (pjmedia_rtp_hdr *)pkt;
  pj_size_t sent_size;
  
  PJ_LOG(3, ("TRANS>",
             "%d | Send RTP Check, Size: %d, Seq: %d, Hdr: %04X,  Hdr.X: %X, "
             "Hdr.pt: %d",
             adapter->vox_idle, size, pj_ntohs(rtphdr->seq), *hdr, rtphdr->x,
             rtphdr->pt));
  
  pj_memcpy(payload, (pj_uint8_t *)pkt + sizeof(pjmedia_rtp_hdr),
            (size - sizeof(pjmedia_rtp_hdr)));
 
  if (adapter->ptt_State & (PTT_TX_LOC_GRANTED | PTT_TX_LOC_END)) {
    /*pj_memcpy((pj_uint8_t *)pkt + sizeof(pjmedia_rtp_hdr) +
                  sizeof(pjmedia_rtp_ext_hdr),
              payload, (size - sizeof(pjmedia_rtp_hdr)));*/
    sent_size = size + sizeof(pjmedia_rtp_ext_hdr);
  } else
    sent_size = sizeof(pjmedia_rtp_hdr) + sizeof(pjmedia_rtp_ext_hdr);

  if (transport_update_ptt_loc_state(adapter, rtphdr->seq, &rtpexthdr) !=
      PJ_SUCCESS)
    return PJ_SUCCESS;

  rtphdr->x = 1;
  rtphdr->m = 0;

  pj_memcpy((pj_uint8_t *)pkt + sizeof(pjmedia_rtp_hdr), &rtpexthdr,
            sizeof(pjmedia_rtp_ext_hdr));

  if (adapter->ptt_State & (PTT_TX_LOC_GRANTED | PTT_TX_LOC_END)) {
    pj_memcpy((pj_uint8_t *)pkt + sizeof(pjmedia_rtp_hdr) +
                  sizeof(pjmedia_rtp_ext_hdr),
              payload, (size - sizeof(pjmedia_rtp_hdr)));
  }
  
  /* You may do some processing to the RTP packet here if you want. */
  PJ_LOG(3, ("TRANS>",
             " %d | Send RTP OK, Size: %d, Seq: %d, Hdr: %04X,  Hdr.X: %X, "
             "extHdr: %04X",
             adapter->vox_idle, sent_size, pj_ntohs(rtphdr->seq), *hdr,
             rtphdr->x, rtphdr->pt, pj_ntohl(rtpexthdr.ptt_tx_sql)));

  /* Send the packet using the slave transport */
  return pjmedia_transport_send_rtp(adapter->slave_tp, pkt, sent_size);
}

/*
 * send_rtcp() is called to send RTCP packet. The "pkt" and "size" argument
 * contain the RTCP packet.
 */
static pj_status_t transport_send_rtcp(pjmedia_transport *tp, const void *pkt,
                                       pj_size_t size) {
  struct tp_adapter *adapter = (struct tp_adapter *)tp;

  /* You may do some processing to the RTCP packet here if you want. */

  /* Send the packet using the slave transport */
  return pjmedia_transport_send_rtcp(adapter->slave_tp, pkt, size);
}

/*
 * This is another variant of send_rtcp(), with the alternate destination
 * address in the argument.
 */
static pj_status_t transport_send_rtcp2(pjmedia_transport *tp,
                                        const pj_sockaddr_t *addr,
                                        unsigned addr_len, const void *pkt,
                                        pj_size_t size) {
  struct tp_adapter *adapter = (struct tp_adapter *)tp;
  return pjmedia_transport_send_rtcp2(adapter->slave_tp, addr, addr_len, pkt,
                                      size);
}

/*
 * The media_create() is called when the transport is about to be used for
 * a new call.
 */
static pj_status_t transport_media_create(pjmedia_transport *tp,
                                          pj_pool_t *sdp_pool, unsigned options,
                                          const pjmedia_sdp_session *rem_sdp,
                                          unsigned media_index) {
  struct tp_adapter *adapter = (struct tp_adapter *)tp;
  adapter->ptt_State = PTT_IDLE | PTT_TX_LOC_IDLE;
  adapter->in_call = PJ_FALSE;
  adapter->out_call = PJ_FALSE;
  adapter->vox_idle = 0;
  adapter->ptt_cnt = 0;
  adapter->end_cnt = 0;
  /* if "rem_sdp" is not NULL, it means we are UAS. You may do some
   * inspections on the incoming SDP to verify that the SDP is acceptable
   * for us. If the SDP is not acceptable, we can reject the SDP by
   * returning non-PJ_SUCCESS.
   */
  if (rem_sdp) {
    /* Do your stuff.. */
  }

  /* Once we're done with our initialization, pass the call to the
   * slave transports to let it do it's own initialization too.
   */
  return pjmedia_transport_media_create(adapter->slave_tp, sdp_pool, options,
                                        rem_sdp, media_index);
}

/*
 * The encode_sdp() is called when we're about to send SDP to remote party,
 * either as SDP offer or as SDP answer.
 */
static pj_status_t transport_encode_sdp(pjmedia_transport *tp,
                                        pj_pool_t *sdp_pool,
                                        pjmedia_sdp_session *local_sdp,
                                        const pjmedia_sdp_session *rem_sdp,
                                        unsigned media_index) {
  struct tp_adapter *adapter = (struct tp_adapter *)tp;

  /* If "rem_sdp" is not NULL, it means we're encoding SDP answer. You may
   * do some more checking on the SDP's once again to make sure that
   * everything is okay before we send SDP.
   */
  if (rem_sdp) {
    /* Do checking stuffs here.. */
  }

  /* You may do anything to the local_sdp, e.g. adding new attributes, or
   * even modifying the SDP if you want.
   */
  if (1) {
    /* Say we add a proprietary attribute here.. */
    /*
    pjmedia_sdp_attr *my_attr;

    my_attr = PJ_POOL_ALLOC_T(sdp_pool, pjmedia_sdp_attr);
    pj_strdup2(sdp_pool, &my_attr->name, "X-adapter");
    pj_strdup2(sdp_pool, &my_attr->value, "some value");

    pjmedia_sdp_attr_add(&local_sdp->media[media_index]->attr_count,
                         local_sdp->media[media_index]->attr,
                         my_attr);
*/
  }

  /* And then pass the call to slave transport to let it encode its
   * information in the SDP. You may choose to call encode_sdp() to slave
   * first before adding your custom attributes if you want.
   */
  return pjmedia_transport_encode_sdp(adapter->slave_tp, sdp_pool, local_sdp,
                                      rem_sdp, media_index);
}

/*
 * The media_start() is called once both local and remote SDP have been
 * negotiated successfully, and the media is ready to start. Here we can start
 * committing our processing.
 */
static pj_status_t transport_media_start(pjmedia_transport *tp, pj_pool_t *pool,
                                         const pjmedia_sdp_session *local_sdp,
                                         const pjmedia_sdp_session *rem_sdp,
                                         unsigned media_index) {
  struct tp_adapter *adapter = (struct tp_adapter *)tp;

  /* Do something.. */
  PJ_LOG(3, ("TRANS>", " Adapter media start .."));

  // adapter->ptt_State = PTT_IDLE | PTT_TX_LOC_IDLE;

  /* And pass the call to the slave transport */
  return pjmedia_transport_media_start(adapter->slave_tp, pool, local_sdp,
                                       rem_sdp, media_index);
}

/*
 * The media_stop() is called when media has been stopped.
 */
static pj_status_t transport_media_stop(pjmedia_transport *tp) {
  struct tp_adapter *adapter = (struct tp_adapter *)tp;
  PJ_LOG(3, ("TRANS>", " Adapter media stopped .."));

  /* Do something.. */
 /* And pass the call to the slave transport */
  return pjmedia_transport_media_stop(adapter->slave_tp);
}

/*
 * simulate_lost() is called to simulate packet lost
 */
static pj_status_t transport_simulate_lost(pjmedia_transport *tp,
                                           pjmedia_dir dir, unsigned pct_lost) {
  struct tp_adapter *adapter = (struct tp_adapter *)tp;
  return pjmedia_transport_simulate_lost(adapter->slave_tp, dir, pct_lost);
}

/*
 * destroy() is called when the transport is no longer needed.
 */
static pj_status_t transport_destroy(pjmedia_transport *tp) {
  struct tp_adapter *adapter = (struct tp_adapter *)tp;

  /* Close the slave transport */
  PJ_LOG(3, ("TRANS>", " Adapter destroyed .."));

  if (adapter->del_base) {
    pjmedia_transport_close(adapter->slave_tp);
  }

  /* Self destruct.. */
  pj_pool_release(adapter->pool);
  pj_caching_pool_destroy(&adapter_cache);

  return PJ_SUCCESS;
}
