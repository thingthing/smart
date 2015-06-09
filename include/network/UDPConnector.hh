#ifndef     UDP_CONNECTOR_H_
# define    UDP_CONNECTOR_H_

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/sendfile.h>
#include <arpa/inet.h>

#include <string>
#include <vector>

#include "AProtocol.h"
#include "IConnector.hh"

namespace   Network
{
class       UDPConnector : public IConnector
{
public:
    UDPConnector();
    virtual ~UDPConnector();

    virtual bool    connectTo(const std::string &ip, unsigned short port);
    virtual void    disconnect();
    virtual bool    isConnected() const;
    virtual int     getSocket() const;


protected:
    bool    initSockAddr(const std::string &ip, unsigned short port);

    struct sockaddr_in          _sa;
    int                         _socket;

};
}
#endif