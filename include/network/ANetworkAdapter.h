#ifndef		INETWORKADAPTER_H_
# define	INETWORKADAPTER_H_

#include "AProtocol.h"
#include "AThread.h"
#include "utils/NonCopyable.h"
#include "utils/event/Dispatcher.h"

namespace   Network
{

class		ANetworkAdapter : public AThread, public Utils::Dispatcher
{
public:
    ANetworkAdapter(){}
    virtual ~ANetworkAdapter(){}

    virtual bool    send(const std::string &data) = 0;

protected:
    NON_COPYABLE(ANetworkAdapter)

    virtual void    run() = 0; // From AThread
};

}
#endif
