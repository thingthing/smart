#ifndef		INETWORKADAPTER_H_
# define	INETWORKADAPTER_H_

#include "AProtocol.h"
#include "AThread.h"

// TODO : make this inherit from thread instead.

namespace   Network
{

class		ANetworkAdapter : public AThread
{
public:
    virtual ~ANetworkAdapter(){}

    void            setProtocol(AProtocol &protocol) {_protocol = &protocol;}
    virtual bool    send(const std::string &data) = 0;

protected:
    virtual void    run() = 0; // From AThread

    AProtocol       *_protocol;
};

}
#endif
