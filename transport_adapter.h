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
#ifndef __PJMEDIA_TRANSPORT_ADAPTER_PTT_H__
#define __PJMEDIA_TRANSPORT_ADAPTER_PTT_H__

/**
 * @file transport_adapter_sample.h
 * @brief Sample Media Transport Adapter
 */

#include <pjmedia/rtp.h>
#include <pjmedia/transport.h>

/**
 * @defgroup PJMEDIA_TRANSPORT_ADAPTER_SAMPLE Sample Transport Adapter
 * @ingroup PJMEDIA_TRANSPORT
 * @brief Example on how to create transport adapter.
 * @{
 *
 * This describes a sample implementation of transport adapter, similar to
 * the way the SRTP transport adapter works.
 */

PJ_BEGIN_DECL

typedef enum {
  PTT_IDLE = 0x0100,
  PTT_IN_CALL = 0x0400,
  PTT_TX_REM_REQ = 0x0001,
  PTT_TX_REM_REQN = 0x0002,
  PTT_TX_REM_GRANTED = 0x0004,
  PTT_TX_REM_END = 0x0008,
  PTT_TX_LOC_REQ = 0x0010,
  PTT_TX_LOC_REQN = 0x0020,
  PTT_TX_LOC_GRANTED = 0x0040,
  PTT_TX_LOC_END = 0x0080,
  PTT_TX_LOC_IDLE = 0x0200,
  PTT_TX_REQ_END = 0x0800,
  PTT_TX_LOC_WAIT = 0x1000,
  PTT_TX_LOC_REQ_END = 0x2000
} ptt_state_t;

struct ptt_data_s {
  pj_uint32_t ptt_id;
  pj_uint32_t from_id;
  pj_uint32_t to_id;
  pj_uint32_t tx_sql;
  pj_uint32_t tx_data;
};

/* The transport adapter instance */
struct tp_adapter {
  pjmedia_transport base;
  pj_bool_t del_base;
  pj_bool_t attached;
  pj_uint16_t ptt_State;
  struct ptt_data_s ptt_data;
  void *user_data;
  pj_pool_t *pool;
  pj_bool_t init_state;
  pj_bool_t in_call;
  pj_bool_t out_call;

  /* Stream information. */
  void *stream_user_data;
  void (*stream_rtp_cb)(void *user_data, void *pkt, pj_ssize_t);
  void (*stream_rtcp_cb)(void *user_data, void *pkt, pj_ssize_t);
  int id_media;
  int id_sipmedia;
  int vox_idle;
  int ptt_cnt;
  int end_cnt;

  /* Add your own member here.. */
  pjmedia_transport *slave_tp;
};

typedef struct tp_adapter tp_adapter_t;
typedef struct ptt_data_s ptt_data_t;

#define PTT_REM_TX_MASK 0x00020000
#define PTT_REM_REQN_MASK 0x00040000
#define PTT_LOC_TX_MASK 0x00010000
#define PTT_REQ_MASK 0x00080000
#define PTT_MSG 0x13000000
#define PTT_DATA 0x05000000

/* TX REQ RTP seq=1,2 wait for TX_GRANTED */
#define PTT_TX_REQ 0x13000010
#define PTT_TX_DATA 0x00000000

/* TX REQ RTP seq=3 if no TX_GRANTED */
//#define PTT_TX_REQN 0x13030012
//#define PTT_TX_DATAN 0x00000000
/* TX CMD with payloads seq=4,... after TX GRANTED */
#define PTT_TX_CMD 0x13020010
#define PTT_TX_CMD_END 0x13030010

#define PTT_TX_END 0x13090000

/* TX_GRANTED */
#define PTT_TX_REQ_GRANTED 0x13010010
/**
 * Create the transport adapter, specifying the underlying transport to be
 * used to send and receive RTP/RTCP packets.
 *
 * @param endpt		The media endpoint.
 * @param name		Optional name to identify this media transport
 *			for logging purposes.
 * @param base_tp	The base/underlying media transport to send and
 * 			receive RTP/RTCP packets.
 * @param del_base	Specify whether the base transport should also be
 * 			destroyed when destroy() is called upon us.
 * @param p_tp		Pointer to receive the media transport instance.
 *
 * @return		PJ_SUCCESS on success, or the appropriate error code.
 */
PJ_DECL(pj_status_t)
pjmedia_ptt_adapter_create(const char *name, pjmedia_transport *base_tp,
                           pj_bool_t del_base, pjmedia_transport **p_tp);

PJ_DECL(pj_uint16_t) pjmedia_ptt_get_state();
PJ_DECL(void) pjmedia_ptt_setmedia(int id_media, int id_sipmedia, int vox_idle);
PJ_DECL(void) pjmedia_ptt_stopTx();
PJ_DECL(void) pjmedia_ptt_outCall();
PJ_END_DECL

/**
 * @}
 */

#endif /* __PJMEDIA_TRANSPORT_ADAPTER_PTT_H__ */
