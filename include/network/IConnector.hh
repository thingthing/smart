#ifndef     I_CONNECTOR_H_
# define    I_CONNECTOR_H_

#include <string>

namespace   Network
{
  class       IConnector
  {
  public:
    IConnector() :
    _rxBuffer(MAX_RX_BUFFER_SIZE),
    _txBuffer(MAX_TX_BUFFER_SIZE)
    {};
    virtual ~IConnector() {};

    virtual bool connectTo(const std::string &ip, unsigned short port) = 0;
    virtual void disconnect() = 0;
    virtual bool isConnected() const = 0;
    virtual int getSocket() const = 0;
    virtual CircularBuffer &getReadBuffer() {return this->_rxBuffer;};
    virtual CircularBuffer &getWriteBuffer() {return this->_txBuffer;};

    CircularBuffer                                  _rxBuffer;
    CircularBuffer                                  _txBuffer;


    static const unsigned int   MAX_RX_BUFFER_SIZE = (64 * 1024);
    static const unsigned int   MAX_TX_BUFFER_SIZE = (64 * 1024);
  };

}
#endif
