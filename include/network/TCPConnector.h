#ifndef     NETWORK_MANAGER_H_
# define    NETWORK_MANAGER_H_

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/sendfile.h>
#include <arpa/inet.h>
#include <poll.h>

#include <string>
#include <vector>

#include "AProtocol.h"
#include "CircularBuffer.h"
#include "ANetworkAdapter.h"

namespace   Network
{
class       TCPConnector : public ANetworkAdapter
{
public:
    TCPConnector();
    virtual ~TCPConnector();

    bool            connectTo(const std::string &ip, unsigned short port);
    virtual bool    send(const std::string &data);

protected:
    virtual void    run();
    bool            initSockAddr(const std::string &ip, unsigned short port);

    struct sockaddr_in          _sa;
    int                         _socket;
    CircularBuffer              _rxBuffer;
    CircularBuffer              _txBuffer;
    pollfd                      _fdset;
    int                         _byteRead;
    int                         _byteWritten;

    static const unsigned int   MAX_RX_BUFFER_SIZE = (64*1024);
    static const unsigned int   MAX_TX_BUFFER_SIZE = (64*1024);
};
}
#endif
