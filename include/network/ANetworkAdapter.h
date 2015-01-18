#ifndef		INETWORKADAPTER_H_
# define	INETWORKADAPTER_H_

#include "AProtocol.h"

// TODO : make this inherit from thread instead.

namespace   Network
{

class		ANetworkAdapter
{
public:
    virtual ~ANetworkAdapter(){}

    void            setProtocol(AProtocol &protocol) {_protocol = &protocol;}
    virtual bool    send(const Packet &packet) = 0;
    virtual void    start() = 0;
    virtual void    stop() = 0;

protected:
    AProtocol       *_protocol;
};

}
#endif
