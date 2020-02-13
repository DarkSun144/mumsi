#include "PjsuaCommunicator.hpp"
#include "transport_adapter.h"
#include <pjlib.h>
#include <pjsua-lib/pjsua.h>

#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>

using namespace std;

namespace sip {
using namespace log4cpp;

class _LogWriter : public pj::LogWriter {
public:
  _LogWriter(Category &logger) : logger(logger) {}

  virtual void write(const pj::LogEntry &entry) override {

    auto message = entry.msg.substr(0, entry.msg.size() - 1); // remove newline

    logger << prioritiesMap.at(entry.level) << message;
  }

private:
  log4cpp::Category &logger;

  std::map<int, Priority::Value> prioritiesMap = {
      {1, Priority::ERROR}, {2, Priority::WARN},  {3, Priority::NOTICE},
      {4, Priority::INFO},  {5, Priority::DEBUG}, {6, Priority::DEBUG}};
};

class _MumlibAudioMedia : public pj::AudioMedia {
public:
  _MumlibAudioMedia(sip::PjsuaCommunicator &comm, int frameTimeLength)
      : communicator(comm) {
    createMediaPort(frameTimeLength);
    registerMediaPort(&mediaPort);
  }

  ~_MumlibAudioMedia() { unregisterMediaPort(); }

private:
  pjmedia_port mediaPort;
  sip::PjsuaCommunicator &communicator;

  static pj_status_t callback_getFrame(pjmedia_port *port,
                                       pjmedia_frame *frame) {
    auto *communicator =
        static_cast<sip::PjsuaCommunicator *>(port->port_data.pdata);
    return communicator->mediaPortGetFrame(port, frame);
  }

  static pj_status_t callback_putFrame(pjmedia_port *port,
                                       pjmedia_frame *frame) {
    auto *communicator =
        static_cast<sip::PjsuaCommunicator *>(port->port_data.pdata);
    return communicator->mediaPortPutFrame(port, frame);
  }

  void createMediaPort(int frameTimeLength) {
    auto name = pj_str((char *)"MumsiMediaPort");

    if (frameTimeLength != 10 and frameTimeLength != 20 and
        frameTimeLength != 40 and frameTimeLength != 60) {
      throw sip::Exception((boost::format("valid frame time length value: %d. "
                                          "valid values are: 10, 20, 40, 60") %
                            frameTimeLength)
                               .str());
    }

    pj_status_t status = pjmedia_port_info_init(
        &(mediaPort.info), &name, PJMEDIA_SIG_CLASS_PORT_AUD('s', 'i'),
        SAMPLING_RATE, 1, 16, SAMPLING_RATE * frameTimeLength / 1000);

    if (status != PJ_SUCCESS) {
      throw sip::Exception("error while calling pjmedia_port_info_init()",
                           status);
    }

    mediaPort.port_data.pdata = &communicator;

    mediaPort.get_frame = &callback_getFrame;
    mediaPort.put_frame = &callback_putFrame;
  }
};

class _Call : public pj::Call {
public:
  _Call(sip::PjsuaCommunicator &comm, pj::Account &acc,
        int call_id = PJSUA_INVALID_ID)
      : pj::Call(acc, call_id), communicator(comm), account(acc) {}

  virtual void onCallState(pj::OnCallStateParam &prm) override;

  virtual void onCallMediaState(pj::OnCallMediaStateParam &prm) override;

  virtual void onDtmfDigit(pj::OnDtmfDigitParam &prm) override;

  virtual void
  onCreateMediaTransport(pj::OnCreateMediaTransportParam &prm) override;

  virtual void onCallSdpCreated(pj::OnCallSdpCreatedParam &prm) override;

private:
  sip::PjsuaCommunicator &communicator;
  pj::Account &account;
};

class _Account : public pj::Account {
public:
  _Account(sip::PjsuaCommunicator &comm) : communicator(comm) {}

  virtual void onRegState(pj::OnRegStateParam &prm) override;

  virtual void onIncomingCall(pj::OnIncomingCallParam &prm) override;
  std::string virt_id;

  void makeCall(string dstUri, pj::CallOpParam &prm);
  void hangup(pj::CallOpParam &prm);

  _Call *myCall;

private:
  sip::PjsuaCommunicator &communicator;

  bool available = true;

  friend class _Call;
};

void _Call::onCallState(pj::OnCallStateParam &prm) {
  auto ci = getInfo();

  communicator.logger.info("CALL> Call %d state=%s.", ci.id,
                           ci.stateText.c_str());

  string address = ci.remoteUri;

  boost::replace_all(address, "<", "");
  boost::replace_all(address, ">", "");

  if (ci.state == PJSIP_INV_STATE_CONFIRMED) {
    string msgText;
    if (communicator.out_call <= 0) {
      communicator.inc_call = true;
      msgText = "CALL> Incoming call from " + address + ".";
    } else if (communicator.out_call >= 1) {
      auto &acc = dynamic_cast<_Account &>(account);
      communicator.out_call = 2;
      communicator.inc_call = false;
      communicator.in_call = true;
      communicator.mumble_idle = 0;
      communicator.call_idle = 0;
      acc.available = false;

      msgText = "CALL> Outgoing call to " + address + ".";
      pjmedia_ptt_outCall();
    }

    communicator.mumble_idle = 0;
    communicator.vox_idle = 0;
    communicator.logger.notice(msgText);
    communicator.onStateChange(msgText);
    auto *aud_med = static_cast<pj::AudioMedia *>(getMedia(0));
    if (aud_med)
      pjmedia_ptt_setmedia(communicator.media->getPortId(),
                           aud_med->getPortId(), communicator.vox_idle);
  } else if (ci.state == PJSIP_INV_STATE_DISCONNECTED) {
    auto &acc = dynamic_cast<_Account &>(account);

    if (not acc.available) {
      auto msgText = "CALL> Call from " + address + " finished.";
      communicator.mixer->clear();

      communicator.logger.notice(msgText);
      communicator.onStateChange(msgText);

      acc.available = true;
      communicator.call_disconnected = true;
      communicator.out_call = 0;
      communicator.in_call = false;
      communicator.inc_call = false;
      communicator.hangup_state = false;
      communicator.mumble_idle = 0;
      communicator.call_idle = 0;
    }

    delete this;
  } else if (ci.state == PJSIP_INV_STATE_CALLING) {
    auto &acc = dynamic_cast<_Account &>(account);
    communicator.out_call = 2;
    communicator.in_call = true;
    communicator.inc_call = false;
    acc.available = false;
  } else if (ci.state == PJSIP_INV_STATE_CONNECTING) {
    auto *aud_med = static_cast<pj::AudioMedia *>(getMedia(0));
    communicator.in_call = true;
   communicator.vox_idle = 0;
    communicator.hangup_state = false;
   if (aud_med)
      pjmedia_ptt_setmedia(communicator.media->getPortId(),
                           aud_med->getPortId(), communicator.vox_idle);
  }
} // namespace sip

void _Call::onCallMediaState(pj::OnCallMediaStateParam &prm) {
  auto ci = getInfo();
  pj_uint16_t myptt_State = pjmedia_ptt_get_state();

  if (ci.media.size() != 1) {
    throw sip::Exception("CALL> ci.media.size is not 1");
  }

  if (ci.media[0].status == PJSUA_CALL_MEDIA_ACTIVE) {
    auto *aud_med = static_cast<pj::AudioMedia *>(getMedia(0));
    communicator.logger.info("PTT STATE: %04X", myptt_State);

  } else if (ci.media[0].status == PJSUA_CALL_MEDIA_NONE) {
    communicator.logger.debug("CALL> no MEDIA detected");
  }
}

void _Call::onDtmfDigit(pj::OnDtmfDigitParam &prm) {
  communicator.logger.notice("DTMF digit '%s' (call %d).", prm.digit.c_str(),
                             getId());
}

void _Call::onCreateMediaTransport(pj::OnCreateMediaTransportParam &prm) {
  auto ci = getInfo();

  auto *aud_med = static_cast<pj::AudioMedia *>(getMedia(0));
  pjmedia_transport *remTp = (pjmedia_transport *)prm.mediaTp;
  communicator.logger.info("CALL> %s, Creating Transport Adapter for CallId %d",
                           (pjmedia_transport *)remTp->name, ci.id);
  pjmedia_transport *adapter;
  pj_status_t status;

  status = pjmedia_ptt_adapter_create(NULL, (pjmedia_transport *)prm.mediaTp,
                                      prm.flags, &adapter);
  if (status != PJ_SUCCESS) {
    communicator.logger.info("CALL> Error Creating Transport Adapter %d",
                             status);
  } else {
    prm.mediaTp = adapter;
    communicator.logger.info(
        "CALL> Transport Adapter %s is created for CallId: %d, ", adapter->name,
        ci.id);
  }
}

void _Call::onCallSdpCreated(pj::OnCallSdpCreatedParam &prm) {

  string ptt_group_sdpattr = "a=floor\r\n";
  prm.sdp.wholeSdp += ptt_group_sdpattr;
}

void _Account::onRegState(pj::OnRegStateParam &prm) {
  pj::AccountInfo ai = getInfo();
  communicator.logger << log4cpp::Priority::INFO
                      << (ai.regIsActive ? "Register:" : "Unregister:")
                      << " code=" << prm.code;
}

void _Account::onIncomingCall(pj::OnIncomingCallParam &iprm) {
  auto *call = new _Call(communicator, *this, iprm.callId);

  string uri = call->getInfo().remoteUri;

  communicator.logger.info("ACC> Incoming call from %s.", uri.c_str());

  pj::CallOpParam param;

  if (communicator.uriValidator.validateUri(uri)) {
    if (available) {
      param.statusCode = PJSIP_SC_OK;
      available = false;
    } else {
      param.statusCode = PJSIP_SC_BUSY_EVERYWHERE;
    }

    call->answer(param);
  } else {
    communicator.logger.warn("ACC> Refusing call from %s.", uri.c_str());
    param.statusCode = PJSIP_SC_SERVICE_UNAVAILABLE;
    call->hangup(param);
  }
}

void _Account::makeCall(string dstUri, pj::CallOpParam &prm) {
  myCall = new _Call(communicator, *this, -1);
  myCall->makeCall(dstUri, prm);
}

void _Account::hangup(pj::CallOpParam &prm) {
  // auto *call = new _Call(communicator, *this, -1);
  myCall->hangup(prm);
}
} // namespace sip

sip::PjsuaCommunicator::PjsuaCommunicator(
    IncomingConnectionValidator &validator, int frameTimeLength)
    : logger(log4cpp::Category::getInstance("SipCommunicator")),
      pjsuaLogger(log4cpp::Category::getInstance("Pjsua")),
      uriValidator(validator) {
  pj_status_t status;

  logWriter.reset(new sip::_LogWriter(pjsuaLogger));

  endpoint.libCreate();

  pj::EpConfig endpointConfig;
  endpointConfig.uaConfig.userAgent = "Mumble-SIP gateway";
  endpointConfig.uaConfig.maxCalls = 1;

  endpointConfig.logConfig.writer = logWriter.get();
  endpointConfig.logConfig.level = 5;

  endpointConfig.medConfig.noVad = true;

  endpoint.libInit(endpointConfig);

  pj_caching_pool_init(&cachingPool, &pj_pool_factory_default_policy, 0);

  vox_pool = pj_pool_create(&cachingPool.factory, "voxpool", 255, 255, NULL);
  status - pjmedia_silence_det_create(vox_pool, SAMPLING_RATE,
                                      (SAMPLING_RATE * frameTimeLength / 1000),
                                      &vox_detect);

  if (status == PJ_SUCCESS) {
    pjmedia_silence_det_set_name(vox_detect, "VOX-MumB");
    pjmedia_silence_det_set_params(vox_detect, 800, -1, -1);
    pjmedia_silence_det_set_adaptive(vox_detect, 50);
    logger.info("Voice Silence detector is created..");
  }

  mixer.reset(new mixer::AudioFramesMixer(cachingPool.factory));

  media.reset(new _MumlibAudioMedia(*this, frameTimeLength));

  logger.info("Created Pjsua communicator with frame length %d ms.",
              frameTimeLength);
}

void sip::PjsuaCommunicator::connect(std::string host, std::string iduser,
                                     std::string vuser, std::string user,
                                     std::string password, unsigned int port) {
  pj::TransportConfig transportConfig;

  transportConfig.port = port;

  endpoint.transportCreate(PJSIP_TRANSPORT_UDP,
                           transportConfig); // todo try catch

  endpoint.libStart();

  pj_status_t status = pjsua_set_null_snd_dev();
  if (status != PJ_SUCCESS) {
    throw sip::Exception("error in pjsua_set_null_std_dev()", status);
  }

  myAccount[0].reset(new _Account(*this));
  myAccount[1].reset(new _Account(*this));
  myAccount[2].reset(new _Account(*this));
  registerAccount(host, iduser, vuser, user, password);
}

sip::PjsuaCommunicator::~PjsuaCommunicator() { endpoint.libDestroy(); }
/*
   Processing Incomming audio from mumble and push to buffers
*/
void sip::PjsuaCommunicator::sendPcmSamples(int sessionId, int sequenceNumber,
                                            int16_t *samples,
                                            unsigned int length) {
  pj_uint16_t myptt_State = 0x0;
  if (in_call) {
    myptt_State = pjmedia_ptt_get_state();
    if ((myptt_State & PTT_TX_LOC_IDLE) && (vox_idle >= 25)) {
      vox_idle = 0;
      pjmedia_ptt_setmedia(-1, -1, vox_idle);
    }
  }
  pj_status_t status;
  pj_int32_t vox_level;

  vox_state =
      pjmedia_silence_det_detect(vox_detect, samples, length, &vox_level);
  logger.debug(
      "sendPcmSamples> Call State: %d | VOX: %d Audio level %d | ais:.%04X",
      out_call, vox_state, vox_level, myptt_State);

  if (vox_state) {
    logger.debug("sendPcmSamples> Silence %d|%d detected ...%04X", call_idle,
                 mumble_idle, myptt_State);

    if ((myptt_State & PTT_TX_LOC_GRANTED) && ((mumble_idle % 20) == 0) &&
        (mumble_idle > 0)) {
      if (in_call) {
        pjmedia_ptt_stopTx();
      }
    }
    if ((out_call) && ((call_idle % 400) == 0) && (call_idle > 0) &&
        (!hangup_state)) {
      pj::CallOpParam prm(true);
      myAccount[0]->hangup(prm);
      hangup_state = true;
    }
  } else {
    mumble_idle = 0;
    call_idle = 0;
    if (in_call)
      pjmedia_ptt_setmedia(-1, -1, mumble_idle);

    if (in_call || (out_call >= 2)) {
      mixer->addFrameToBuffer(sessionId, sequenceNumber, samples, length);
    } else if ((out_call <= 0)) {

      pj::AccountInfo srcAccInfo = myAccount[0]->getInfo();
      pj::AccountInfo dstAccInfo = myAccount[1]->getInfo();
      pjsuaLogger.debug("Start Invite from %s to %s", srcAccInfo.uri.c_str(),
                        dstAccInfo.uri.c_str());
      pj::CallOpParam prm(true);
      pj::SipHeader sh;
      sh.hName = "PTT-Reach";
      sh.hValue = "group";
      prm.txOption.headers.push_back(sh);
      prm.txOption.targetUri = dstAccInfo.uri;
      pjsuaLogger.debug("txOption.headers size: %d",
                        prm.txOption.headers.size());
      
	  myAccount[0]->makeCall(dstAccInfo.uri, prm);
      in_call = true;
      out_call = 1;
      mixer->addFrameToBuffer(sessionId, sequenceNumber, samples, length);
    }
  }
}

pj_status_t sip::PjsuaCommunicator::mediaPortGetFrame(pjmedia_port *port,
                                                      pjmedia_frame *frame) {
  frame->type = PJMEDIA_FRAME_TYPE_AUDIO;
  pj_int16_t *samples = static_cast<pj_int16_t *>(frame->buf);
  pj_size_t count = frame->size / 2 / PJMEDIA_PIA_CCNT(&(port->info));

  int old_vox = vox_idle;
  pj_int16_t dins = 0x0;

  const int readSamples = mixer->getMixedSamples(samples, count);
  pj_uint16_t myptt_State = 0x0;
  if (in_call)
    myptt_State = pjmedia_ptt_get_state();
 mumble_idle++;
  call_idle++;
  if ((readSamples < count)) {
    if (in_call && (myptt_State & PTT_TX_LOC_GRANTED) && (mumble_idle > 0) &&
        (mumble_idle % 20 == 0)) {
      pjmedia_ptt_stopTx();
      dins = 0xd5;
    }

    if (myptt_State & (PTT_TX_LOC_END | PTT_TX_LOC_GRANTED))
      dins = 0xd5;

    pjsuaLogger.debug("MEDIA %s> VOX: %d, PTT: %04X, Requested %d samples, "
                      "available %d, filling remaining with %d.",
                      pj_strbuf(&port->info.name), mumble_idle, myptt_State,
                      count, readSamples, dins);
    for (int i = readSamples; i < count; ++i) {
      samples[i] = dins;
    }
  }
 
  if ((out_call) && ((call_idle % 400) == 0) && (!hangup_state) &&
      (call_idle > 0)) {
    pjsuaLogger.debug("MEIDA %s> Vox State: %d|%d Call Idle reach threashold",
                      pj_strbuf(&port->info.name), call_idle, mumble_idle);
    pj::CallOpParam prm(true);
    myAccount[0]->hangup(prm);
    hangup_state = true;
  }
   if (call_disconnected)
    call_disconnected = false;

  return PJ_SUCCESS;
}

pj_status_t sip::PjsuaCommunicator::mediaPortPutFrame(pjmedia_port *port,
                                                      pjmedia_frame *frame) {
  pj_int16_t *samples = static_cast<pj_int16_t *>(frame->buf);
  pj_size_t count = frame->size / 2 / PJMEDIA_PIA_CCNT(&port->info);
  frame->type = PJMEDIA_FRAME_TYPE_AUDIO;
  pj_uint16_t myptt_State = 0x0;
  if (in_call)
    myptt_State = pjmedia_ptt_get_state();

  if (count > 0) {
    pjsuaLogger.debug("MEDIA %s> PTT: %04X, Calling "
                      "onIncomingPcmSamples with %d samples.",
                      pj_strbuf(&port->info.name), myptt_State, count);
    onIncomingPcmSamples(samples, count);
  }

  return PJ_SUCCESS;
}

void sip::PjsuaCommunicator::registerAccount(string host, string iduser,
                                             string vuser, string user,
                                             string password) {
  string uri = "sip:" + user + "@" + host;
  string vuri = "sip:" + vuser + "@" + host;
  string iduri = "sip:" + iduser + "@" + host;
  bool accDefault = PJ_FALSE;
  int header_size;
  int n;
  pj::AccountConfig idaccountConfig, vaccountConfig, accountConfig;

  pj::AuthCredInfo cred("digest", "*", user, 0, password);
#if 1
  idaccountConfig.idUri = iduri;
  idaccountConfig.regfromUri = iduri;
  idaccountConfig.regConfig.registrarUri = "sip:" + host;

  idaccountConfig.sipConfig.authCreds.push_back(cred);
  logger.info("Registering account for URI: %s.", iduri.c_str());
  myAccount[2]->create(idaccountConfig, PJ_TRUE);
#endif
  vaccountConfig.idUri = vuri;
  vaccountConfig.regfromUri = vuri;
  vaccountConfig.regConfig.registrarUri = "sip:" + host;

  vaccountConfig.sipConfig.authCreds.push_back(cred);
  logger.info("Registering account for URI: %s.", vuri.c_str());
  myAccount[0]->create(vaccountConfig, PJ_TRUE);

  accountConfig.idUri = uri;
  accountConfig.regfromUri = vuri;
  accountConfig.regConfig.registrarUri = "sip:" + host;
  accountConfig.sipConfig.authCreds.push_back(cred);
  pj::SipHeader aisgw_header;
  aisgw_header.hName = "PTT-Reach";
  aisgw_header.hValue = "group";
  accountConfig.regConfig.headers.push_back(aisgw_header);

  logger.info("Registering account for URI: %s.", uri.c_str());
  myAccount[1]->create(accountConfig, PJ_FALSE);
}
