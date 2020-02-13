#pragma once

#include "AudioFramesMixer.hpp"
#include "IncomingConnectionValidator.hpp"

#include <pjmedia.h>
#include <pjsua-lib/pjsua.h>

#include <pjsua2.hpp>

#undef isblank

#include <boost/noncopyable.hpp>
#include <log4cpp/Category.hh>

#include <bits/unique_ptr.h>
#include <climits>
#include <stdexcept>
#include <string>

struct ais_rtp_ext_header8 {
  uint32_t id_to;
  uint32_t id_from;
  uint32_t dummy23[2];
  uint32_t fix3840;
  uint32_t dummy5;
  uint32_t data6;
  uint32_t data7;
};

namespace sip {

constexpr int DEFAULT_PORT = 5060;
constexpr int SAMPLING_RATE = 48000;

class Exception : public std::runtime_error {
public:
  Exception(const char *title) : std::runtime_error(title) { mesg += title; }

  Exception(std::string title) : std::runtime_error(title) { mesg += title; }

  Exception(const char *title, pj_status_t status) : std::runtime_error(title) {
    char errorMsgBuffer[500];
    pj_strerror(status, errorMsgBuffer, sizeof(errorMsgBuffer));

    mesg += title;
    mesg += ": ";
    mesg += errorMsgBuffer;
  }

  virtual const char *what() const throw() override { return mesg.c_str(); }

private:
  std::string mesg;
};

class _LogWriter;

class _Account;

class _Call;

class _MumlibAudioMedia;

class PjsuaCommunicator : boost::noncopyable {
public:
  PjsuaCommunicator(IncomingConnectionValidator &validator,
                    int frameTimeLength);

  void connect(std::string host, std::string iduser, std::string vuser,
               std::string user, std::string password,
               unsigned int port = DEFAULT_PORT);

  virtual ~PjsuaCommunicator();

  void sendPcmSamples(int sessionId, int sequenceNumber, int16_t *samples,
                      unsigned int length);

  std::function<void(int16_t *, int)> onIncomingPcmSamples;

  std::function<void(std::string)> onStateChange;

  pj_status_t mediaPortGetFrame(pjmedia_port *port, pjmedia_frame *frame);

  pj_status_t mediaPortPutFrame(pjmedia_port *port, pjmedia_frame *frame);
  bool in_call = false;
  bool inc_call = false;
  int out_call = 0;
  bool vox_state = false;
  bool call_disconnected = false;
  pj_pool_t *vox_pool;
  pjmedia_silence_det *vox_detect;
  _Call *outCall;
  int mumble_idle;
  int call_idle;
  int vox_idle;
  bool hangup_state = false;

private:
  log4cpp::Category &logger;
  log4cpp::Category &pjsuaLogger;

  std::unique_ptr<mixer::AudioFramesMixer> mixer;

  std::unique_ptr<_LogWriter> logWriter;
  std::unique_ptr<_Account> myAccount[3];
  // std::unique_ptr<_Account> account;
  // std::unique_ptr < _Call > myCall;

  std::unique_ptr<_MumlibAudioMedia> media;

  pj_caching_pool cachingPool;

  pj::Endpoint endpoint;

  IncomingConnectionValidator &uriValidator;

  void registerAccount(std::string host, std::string iduser, std::string vuser,
                       std::string user, std::string password);

  friend class _Call;

  friend class _Account;

  friend class _MumlibAudioMedia;
};

} // namespace sip
