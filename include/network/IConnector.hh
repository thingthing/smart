#ifndef     I_CONNECTOR_H_
# define    I_CONNECTOR_H_

#include <string>

namespace   Network
{
class       IConnector
{
public:
    virtual ~IConnector() {};

    virtual bool connectTo(const std::string &ip, unsigned short port) = 0;
    virtual void disconnect() = 0;
    virtual bool isConnected() const = 0;
    virtual int getSocket() const = 0;
};

}
#endif
