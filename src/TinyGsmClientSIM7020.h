/**
 * @file       TinyGsmClientSIM7020.h
 * @author     Bostjan Sorgo
 * @license    LGPL-3.0
 * @copyright  Copyright (c) 2016 Bostjan Sorgo
 * @date       Nov 2016
 * 29.11.2019 Battery support
 */

#ifndef SRC_TINYGSMCLIENTSIM7020_H_
#define SRC_TINYGSMCLIENTSIM7020_H_
// #pragma message("TinyGSM:  TinyGsmClientSIM7020")

// #define TINY_GSM_DEBUG Serial
// #define TINY_GSM_USE_HEX

#define TINY_GSM_MUX_COUNT 5

#include "TinyGsmModem.tpp"
#include "TinyGsmGPRS.tpp"
#include "TinyGsmTCP.tpp"
#include "TinyGsmTime.tpp"
#include "TinyGsmBattery.tpp"

#define GSM_NL "\r\n"
static const char GSM_OK[] TINY_GSM_PROGMEM        = "OK" GSM_NL;
static const char GSM_ERROR[] TINY_GSM_PROGMEM     = "ERROR" GSM_NL;
static const char GSM_CME_ERROR[] TINY_GSM_PROGMEM = GSM_NL "+CME ERROR:";

enum RegStatus {
  REG_NO_RESULT    = -1,
  REG_UNREGISTERED = 0,
  REG_SEARCHING    = 2,
  REG_DENIED       = 3,
  REG_OK_HOME      = 1,
  REG_OK_ROAMING   = 5,
  REG_UNKNOWN      = 4,
};

class TinyGsmSim7020 : public TinyGsmModem<TinyGsmSim7020>,
                       public TinyGsmGPRS<TinyGsmSim7020>,
                       public TinyGsmTCP<TinyGsmSim7020, READ_AND_CHECK_SIZE,
                                         TINY_GSM_MUX_COUNT>,
                       public TinyGsmTime<TinyGsmSim7020>,
                       public TinyGsmBattery<TinyGsmSim7020> {
  friend class TinyGsmModem<TinyGsmSim7020>;
  friend class TinyGsmGPRS<TinyGsmSim7020>;
  friend class TinyGsmTCP<TinyGsmSim7020, READ_AND_CHECK_SIZE,
                          TINY_GSM_MUX_COUNT>;
  friend class TinyGsmTime<TinyGsmSim7020>;
  friend class TinyGsmBattery<TinyGsmSim7020>;

  /*
   * Inner Client
   */
 public:
  class GsmClientSim7020 : public GsmClient {
    friend class TinyGsmSim7020;

   public:
    GsmClientSim7020() {}

    explicit GsmClientSim7020(TinyGsmSim7020& modem, uint8_t mux = 1) {
      init(&modem, mux);
    }

    bool init(TinyGsmSim7020* modem, uint8_t mux = 1) {
      this->at       = modem;
      this->mux      = mux;
      sock_available = 0;
      prev_check     = 0;
      sock_connected = false;
      got_data       = false;

      at->sockets[mux] = this;

      return true;
    }

   public:
    virtual int connect(const char* host, uint16_t port, int timeout_s) {
      stop();
      TINY_GSM_YIELD();
      rx.clear();
      sock_connected = at->modemConnect(host, port, mux, false, timeout_s);
      return sock_connected;
    }
    TINY_GSM_CLIENT_CONNECT_OVERRIDES

    virtual void stop(uint32_t maxWaitMs) {
      dumpModemBuffer(maxWaitMs);
      at->sendAT(GF("+CIPCLOSE="), mux, GF(",1"));  // Quick close
      sock_connected = false;
      at->waitResponse();
    }
    void stop() override {
      stop(15000L);
    }

    /*
     * Extended API
     */

    String remoteIP() TINY_GSM_ATTR_NOT_IMPLEMENTED;
  };

  /*
   * Inner Secure Client
   */
  // Not yet implemented

  /*
   * Constructor
   */
 public:
  explicit TinyGsmSim7020(Stream& stream) : stream(stream) {
    memset(sockets, 0, sizeof(sockets));
  }

  /*
   * Basic functions
   */
 protected:
  bool initImpl(const char* pin = NULL) {
    DBG(GF("### TinyGSM Version:"), TINYGSM_VERSION);

    if (!testAT()) { return false; }

    sendAT(GF("E0"));  // Echo Off
    if (waitResponse() != 1) { return false; }

#ifdef TINY_GSM_DEBUG
    sendAT(GF("+CMEE=2"));  // turn on verbose error codes
#else
    sendAT(GF("+CMEE=0"));  // turn off error codes
#endif
    waitResponse();

    DBG(GF("### Modem:"), getModemName());

    // Enable Local Time Stamp for getting network time
    sendAT(GF("+CLTS=1"));
    if (waitResponse(10000L) != 1) { return false; }

    int ret = getSimStatus();
    // if the sim isn't ready and a pin has been provided, try to unlock the sim
    if (ret != SIM_READY && pin != NULL && strlen(pin) > 0) {
      simUnlock(pin);
      return (getSimStatus() == SIM_READY);
    } else {
      // if the sim is ready, or it's locked but no pin has been provided,
      // return true
      return (ret == SIM_READY || ret == SIM_LOCKED);
    }
  }

  String getModemNameImpl() {
    String name = "SIMCom SIM7020";

    sendAT(GF("+GMM"));
    String res2;
    if (waitResponse(1000L, res2) != 1) { return name; }
    res2.replace(GSM_NL "OK" GSM_NL, "");
    res2.replace("_", " ");
    res2.trim();

    name = res2;
    DBG("### Modem:", name);
    return name;
  }

  bool factoryDefaultImpl() {
    sendAT(GF("&FZE0&W"));  // Factory + Reset + Echo Off + Write
    waitResponse();
    sendAT(GF("+IPR=0"));  // Auto-baud
    waitResponse();
    sendAT(GF("+IFC=0,0"));  // No Flow Control
    waitResponse();
    sendAT(GF("+ICF=3,3"));  // 8 data 0 parity 1 stop
    waitResponse();
    sendAT(GF("+CSCLK=0"));  // Disable Slow Clock
    waitResponse();
    sendAT(GF("&W"));  // Write configuration
    return waitResponse() == 1;
  }

  bool thisHasSSL() {
    return false;
  }

  bool thisHasWifi() {
    return false;
  }

  bool thisHasGPRS() {
    return true;
  }

  /*
   * Power functions
   */
 protected:
  bool restartImpl() {
    if (!testAT()) { return false; }
    sendAT(GF("Z"));
    if (waitResponse(10000) != 1) { return false; }
    sendAT(GF("+CPSMS=0"));
    if (waitResponse() != 1) { return false; }
    return init();
  }

  bool powerOffImpl() {
    sendAT(GF("+CPOWD=1"));
    return waitResponse(10000L, GF("NORMAL POWER DOWN")) == 1;
  }

  // During sleep, the SIM7020 module has its serial communication disabled. In
  // order to reestablish communication pull the DRT-pin of the SIM7020 module
  // LOW for at least 50ms. Then use this function to disable sleep mode. The
  // DTR-pin can then be released again.
  bool sleepEnableImpl(bool enable = true) {
    sendAT(GF("+CSCLK="), enable);
    return waitResponse() == 1;
  }

  /*
   * Generic network functions
   */
 public:
  RegStatus getRegistrationStatus() {
    return (RegStatus)getRegistrationStatusXREG("CGREG");
  }

 protected:
  bool isNetworkConnectedImpl() {
    if (!testAT()) { return false; }
    RegStatus s = getRegistrationStatus();
    return (s == REG_OK_HOME || s == REG_OK_ROAMING);
  }

  String getLocalIPImpl() {
    sendAT(GF("+IPADDR"));  // Inquire Socket PDP address
    // sendAT(GF("+CGPADDR=1"));  // Show PDP address
    String res;
    if (waitResponse(10000L, res) != 1) {
      return "";
    }
    res.replace(GSM_NL "OK" GSM_NL, "");
    res.replace(GSM_NL, "");
    res.trim();
    return res;
  }

  /*
   * GPRS functions
   */
 protected:
  bool gprsConnectImpl(const char* apn, const char* user = NULL,
                       const char* pwd = NULL) {
    if (apn && strlen(apn) > 0) {
      // Turn off radio while updating APN
      sendAT(GF("+CFUN=0"));
      if (waitResponse(15000L) != 1) { return false; }
      // Set default PSD connection setting
      if (user && strlen(user) > 0) {
        sendAT(GF("*MCGDEFCONT=\"IP\",\""), apn, GF("\",\""), user, GF("\",\""),
               pwd, GF("\""));
      } else {
        sendAT(GF("*MCGDEFCONT=\"IP\",\""), apn, '"');
      }
      if (waitResponse(1000L) != 1) { return false; }
      // Turn the radio back on
      sendAT(GF("+CFUN=1"));
      if (waitResponse(10000L) != 1) { return false; }
    }

    // check signal quality
    // TODO(bsorgo):  why is this necessary?
    sendAT(GF("+CSQ"));
    if (waitResponse() != 1) { return false; }

    // Attached PS domain and get IP address automatically
    // TODO(bsorgo):  wait for "+CGCONTRDP" response and read ip?
    sendAT(GF("+CGCONTRDP"));
    if (waitResponse() != 1) { return false; }

    // Set to multi-IP
    sendAT(GF("+CIPMUX=1"));
    if (waitResponse() != 1) { return false; }
    delay(1000);

    // Set to get data manually
    sendAT(GF("+CIPRXGET=1"));
    if (waitResponse() != 1) { return false; }

    // Attach to GPRS
    sendAT(GF("+CGATT=1"));
    if (waitResponse(60000L) != 1) { return false; }

    // Put in "quick send" mode (thus no extra "Send OK")
    sendAT(GF("+CIPQSEND=1"));
    if (waitResponse() != 1) { return false; }

    // Start task
    sendAT(GF("+CSTT"));
    if (waitResponse(60000L) != 1) { return false; }

    // Bring Up Wireless Connection
    sendAT(GF("+CIICR"));
    if (waitResponse(60000L) != 1) { return false; }

    // Get Local IP Address, only assigned after connection
    sendAT(GF("+CIFSR;E0"));
    if (waitResponse(10000L) != 1) { return false; }

    // Configure Domain Name Server (DNS)
    sendAT(GF("+CDNSCFG=\"8.8.8.8\",\"8.8.4.4\""));
    if (waitResponse() != 1) { return false; }

    return true;
  }

  bool gprsDisconnectImpl() {
    // Shut the TCP/IP connection
    // CIPSHUT will close *all* open connections
    sendAT(GF("+CIPSHUT"));
    if (waitResponse(60000L) != 1) { return false; }

    sendAT(GF("+CGACT=0,1"));  // Deactivate cid
    if (waitResponse(60000L) != 1) { return false; }

    sendAT(GF("+CGCONTRDP"));  // Release IP
    if (waitResponse(60000L) != 1) { return false; }
    return true;
  }

  bool isGprsConnectedImpl() {
    sendAT(GF("AT+CGCONTRDP"));
    if (waitResponse(1000L) != 1) { return false; }
    return true;
  }

  /*
   * SIM card functions
   */
 protected:
  // Able to follow all SIM card functions as inherited from the template

  /*
   * Phone Call functions
   */
 protected:
  // TODO(bsorgo): verify that these are not available
  bool callAnswerImpl() TINY_GSM_ATTR_NOT_AVAILABLE;
  bool callNumberImpl(const String& number) TINY_GSM_ATTR_NOT_AVAILABLE;
  bool callHangupImpl() TINY_GSM_ATTR_NOT_AVAILABLE;
  bool dtmfSendImpl(char cmd,
                    int  duration_ms = 100) TINY_GSM_ATTR_NOT_AVAILABLE;

  /*
   * Messaging functions
   */
 protected:
  // TODO(bsorgo): verify that these are not available
  String sendUSSDImpl(const String& code) TINY_GSM_ATTR_NOT_AVAILABLE;
  bool   sendSMSImpl(const String& number,
                     const String& text) TINY_GSM_ATTR_NOT_AVAILABLE;
  bool   sendSMS_UTF16Impl(const String& number, const void* text,
                           size_t len) TINY_GSM_ATTR_NOT_AVAILABLE;

  /*
   * Location functions
   */
 protected:
  // TODO(bsorgo): verify that this is not available
  String getGsmLocationImpl() TINY_GSM_ATTR_NOT_AVAILABLE;

  /*
   * Time functions
   */
 protected:
  // Can follow the standard CCLK function in the template

  /*
   * Battery & temperature functions
   */
 protected:
  // Use: float vBatt = modem.getBattVoltage() / 1000.0;
  uint16_t getBattVoltageImpl() {
    sendAT(GF("+CBC"));
    if (waitResponse(GF(GSM_NL "+CBC:")) != 1) { return 0; }
    streamSkipUntil(',');  // Skip battery charge level
    // return voltage in mV
    uint16_t res = stream.readStringUntil('\n').toInt();
    // Wait for final OK
    waitResponse();
    return res;
  }

  int8_t getBattPercentImpl() {
    sendAT(GF("+CBC"));
    if (waitResponse(GF(GSM_NL "+CBC:")) != 1) { return false; }
    // Read battery charge level
    int res = stream.readStringUntil(',').toInt();
    // Wait for final OK
    waitResponse();
    return res;
  }

  uint8_t getBattChargeStateImpl() TINY_GSM_ATTR_NOT_AVAILABLE;

  bool getBattStatsImpl(uint8_t& chargeState, int8_t& percent,
                        uint16_t& milliVolts) {
    sendAT(GF("+CBC?"));
    if (waitResponse(GF(GSM_NL "+CBC:")) != 1) { return false; }
    percent    = stream.readStringUntil(',').toInt();
    milliVolts = stream.readStringUntil('\n').toInt();
    // Wait for final OK
    waitResponse();
    return true;
  }

  /*
   * Client related functions
   */
 protected:
  bool modemConnect(const char* host, uint16_t port, uint8_t mux,
                    bool ssl = false, int timeout_s = 75) {
    if (ssl) { DBG("SSL not yet supported on this module!"); }
    int      rsp;
    uint32_t timeout_ms = ((uint32_t)timeout_s) * 1000;

    sendAT(GF("+CIPSTART="), mux, ',', GF("\"TCP"), GF("\",\""), host,
           GF("\","), port);
    rsp = waitResponse(
        timeout_ms, GF("CONNECT OK" GSM_NL), GF("CONNECT FAIL" GSM_NL),
        GF("ALREADY CONNECT" GSM_NL), GF("ERROR" GSM_NL),
        GF("CLOSE OK" GSM_NL));  // Happens when HTTPS handshake fails
    return (1 == rsp);
  }

  int16_t modemSend(const void* buff, size_t len, uint8_t mux) {
    sendAT(GF("+CIPSEND="), mux, ',', (uint16_t)len);
    if (waitResponse(GF(">")) != 1) { return 0; }
    stream.write(reinterpret_cast<const uint8_t*>(buff), len);
    stream.flush();
    if (waitResponse(GF(GSM_NL "DATA ACCEPT:")) != 1) { return 0; }
    streamSkipUntil(',');  // Skip mux
    return stream.readStringUntil('\n').toInt();
  }

  size_t modemRead(size_t size, uint8_t mux) {
#ifdef TINY_GSM_USE_HEX
    sendAT(GF("+CIPRXGET=3,"), mux, ',', (uint16_t)size);
    if (waitResponse(GF("+CIPRXGET:")) != 1) { return 0; }
#else
    sendAT(GF("+CIPRXGET=2,"), mux, ',', (uint16_t)size);
    if (waitResponse(GF("+CIPRXGET:")) != 1) { return 0; }
#endif
    streamSkipUntil(',');  // Skip Rx mode 2/normal or 3/HEX
    streamSkipUntil(',');  // Skip mux
    int len_requested = stream.readStringUntil(',').toInt();
    //  ^^ Requested number of data bytes (1-1460 bytes)to be read
    int len_confirmed = stream.readStringUntil('\n').toInt();
    // ^^ Confirmed number of data bytes to be read, which may be less than
    // requested. 0 indicates that no data can be read. This is actually be the
    // number of bytes that will be remaining after the read
    for (int i = 0; i < len_requested; i++) {
      uint32_t startMillis = millis();
#ifdef TINY_GSM_USE_HEX
      while (stream.available() < 2 &&
             (millis() - startMillis < sockets[mux]->_timeout)) {
        TINY_GSM_YIELD();
      }
      char buf[4] = {
          0,
      };
      buf[0] = stream.read();
      buf[1] = stream.read();
      char c = strtol(buf, NULL, 16);
#else
      while (!stream.available() &&
             (millis() - startMillis < sockets[mux]->_timeout)) {
        TINY_GSM_YIELD();
      }
      char c = stream.read();
#endif
      sockets[mux]->rx.put(c);
    }
    DBG("### READ:", len_requested, "from", mux);
    // sockets[mux]->sock_available = modemGetAvailable(mux);
    sockets[mux]->sock_available = len_confirmed;
    waitResponse();
    return len_requested;
  }

  size_t modemGetAvailable(uint8_t mux) {
    sendAT(GF("+CIPRXGET=4,"), mux);
    size_t result = 0;
    if (waitResponse(GF("+CIPRXGET:")) == 1) {
      streamSkipUntil(',');  // Skip mode 4
      streamSkipUntil(',');  // Skip mux
      result = stream.readStringUntil('\n').toInt();
      waitResponse();
    }
    DBG("### Available:", result, "on", mux);
    if (!result) { sockets[mux]->sock_connected = modemGetConnected(mux); }
    return result;
  }

  bool modemGetConnected(uint8_t mux) {
    sendAT(GF("+CIPSTATUS="), mux);
    waitResponse(GF("+CIPSTATUS"));
    int res = waitResponse(GF(",\"CONNECTED\""), GF(",\"CLOSED\""),
                           GF(",\"CLOSING\""), GF(",\"REMOTE CLOSING\""),
                           GF(",\"INITIAL\""));
    waitResponse();
    return 1 == res;
  }

  /*
   * Utilities
   */
 public:
  // TODO(vshymanskyy): Optimize this!
  uint8_t waitResponse(uint32_t timeout_ms, String& data,
                       GsmConstStr r1 = GFP(GSM_OK),
                       GsmConstStr r2 = GFP(GSM_ERROR),
                       GsmConstStr r3 = GFP(GSM_CME_ERROR),
                       GsmConstStr r4 = NULL, GsmConstStr r5 = NULL) {
    /*String r1s(r1); r1s.trim();
    String r2s(r2); r2s.trim();
    String r3s(r3); r3s.trim();
    String r4s(r4); r4s.trim();
    String r5s(r5); r5s.trim();
    DBG("### ..:", r1s, ",", r2s, ",", r3s, ",", r4s, ",", r5s);*/
    data.reserve(64);
    uint8_t  index       = 0;
    uint32_t startMillis = millis();
    do {
      TINY_GSM_YIELD();
      while (stream.available() > 0) {
        TINY_GSM_YIELD();
        int a = stream.read();
        if (a <= 0) continue;  // Skip 0x00 bytes, just in case
        data += static_cast<char>(a);
        if (r1 && data.endsWith(r1)) {
          index = 1;
          goto finish;
        } else if (r2 && data.endsWith(r2)) {
          index = 2;
          goto finish;
        } else if (r3 && data.endsWith(r3)) {
          if (r3 == GFP(GSM_CME_ERROR)) {
            streamSkipUntil('\n');  // Read out the error
          }
          index = 3;
          goto finish;
        } else if (r4 && data.endsWith(r4)) {
          index = 4;
          goto finish;
        } else if (r5 && data.endsWith(r5)) {
          index = 5;
          goto finish;
        } else if (data.endsWith(GF(GSM_NL "+CIPRXGET:"))) {
          String mode = stream.readStringUntil(',');
          if (mode.toInt() == 1) {
            int mux = stream.readStringUntil('\n').toInt();
            if (mux >= 0 && mux < TINY_GSM_MUX_COUNT && sockets[mux]) {
              sockets[mux]->got_data = true;
            }
            data = "";
            DBG("### Got Data:", mux);
          } else {
            data += mode;
          }
        } else if (data.endsWith(GF(GSM_NL "+RECEIVE:"))) {
          int mux = stream.readStringUntil(',').toInt();
          int len = stream.readStringUntil('\n').toInt();
          if (mux >= 0 && mux < TINY_GSM_MUX_COUNT && sockets[mux]) {
            sockets[mux]->got_data       = true;
            sockets[mux]->sock_available = len;
          }
          data = "";
          DBG("### Got Data:", len, "on", mux);
        } else if (data.endsWith(GF("CLOSED" GSM_NL))) {
          int nl   = data.lastIndexOf(GSM_NL, data.length() - 8);
          int coma = data.indexOf(',', nl + 2);
          int mux  = data.substring(nl + 2, coma).toInt();
          if (mux >= 0 && mux < TINY_GSM_MUX_COUNT && sockets[mux]) {
            sockets[mux]->sock_connected = false;
          }
          data = "";
          DBG("### Closed: ", mux);
        }
      }
    } while (millis() - startMillis < timeout_ms);
  finish:
    if (!index) {
      data.trim();
      if (data.length()) { DBG("### Unhandled:", data); }
      data = "";
    }
    // data.replace(GSM_NL, "/");
    // DBG('<', index, '>', data);
    return index;
  }

  uint8_t waitResponse(uint32_t timeout_ms, GsmConstStr r1 = GFP(GSM_OK),
                       GsmConstStr r2 = GFP(GSM_ERROR),
                       GsmConstStr r3 = GFP(GSM_CME_ERROR),
                       GsmConstStr r4 = NULL, GsmConstStr r5 = NULL) {
    String data;
    return waitResponse(timeout_ms, data, r1, r2, r3, r4, r5);
  }

  uint8_t waitResponse(GsmConstStr r1 = GFP(GSM_OK),
                       GsmConstStr r2 = GFP(GSM_ERROR),
                       GsmConstStr r3 = GFP(GSM_CME_ERROR),
                       GsmConstStr r4 = NULL, GsmConstStr r5 = NULL) {
    return waitResponse(1000, r1, r2, r3, r4, r5);
  }

 protected:
  Stream&           stream;
  GsmClientSim7020* sockets[TINY_GSM_MUX_COUNT];
  const char*       gsmNL = GSM_NL;
};

#endif  // SRC_TINYGSMCLIENTSIM7020_H_
