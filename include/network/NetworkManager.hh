#ifndef		NETWORKMANAGER_HH_
# define	NETWORKMANAGER_HH_

#include <poll.h>
#include "AProtocol.h"
#include "AThread.h"
#include "NonCopyable.h"
#include "CircularBuffer.h"
#include "event/Dispatcher.h"
#include "IConnector.hh"

namespace   Network
{

class		NetworkManager : public AThread, public Utils::Dispatcher
{
public:
    NetworkManager(IConnector *connector);
    virtual ~NetworkManager();

    bool    send(const std::string &data);
    bool    connectTo(const std::string &ip, unsigned short port);
    void    disconnect();

    IConnector *getConnector() const;

protected:
    NON_COPYABLE(NetworkManager)

    virtual void    run(); // From AThread

    CircularBuffer              _rxBuffer;
    CircularBuffer              _txBuffer;
    IConnector                  *_connector;
    pollfd                      _fdset;
    int                         _byteRead;
    int                         _byteWritten;

    static const unsigned int   MAX_RX_BUFFER_SIZE = (64*1024);
    static const unsigned int   MAX_TX_BUFFER_SIZE = (64*1024);
};

}
#endif
