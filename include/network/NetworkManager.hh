/** @defgroup Network Files network related
 *
 * @brief Files related to network
 */

/**
 * @class NetworkManager
 *
 * @ingroup Network
 *
 * @brief Manager of networks connectors
 *
 * @author Nicolas
 *
 * @version 1.0
 *
 * @date 09/05/2015
 *
 */
#ifndef     NETWORKMANAGER_HH_
# define    NETWORKMANAGER_HH_

#include <poll.h>
#include <string>
#include "AProtocol.h"
#include "AThread.h"
#include "NonCopyable.h"
#include "CircularBuffer.h"
#include "event/Dispatcher.h"
#include "IConnector.hh"

namespace   Network
{

    class       NetworkManager : public AThread, public Utils::Dispatcher
    {
    public:
        NetworkManager();
        virtual ~NetworkManager();

        bool    send(Network::APacketBase const &packet, const std::string &connector_id);
        bool    send(const std::string &chunk, const std::string &connector_id);
        bool    connectTo(const std::string &ip, unsigned short port);
        bool    connectTo(const std::string &ip, unsigned short port, const std::string &connector_id);
        void    disconnect();
        void    disconnect(const std::string &connector_id);

        IConnector      *getConnector(std::string const & connector_id) const;
        void            setConnector(std::string const &key, IConnector *connector);


    protected:
        NON_COPYABLE(NetworkManager)

        virtual void    run(); // From AThread

        static const unsigned int   MAX_CONNEXION = 10;

        pollfd                                  _fdset[MAX_CONNEXION];
        int                                     _byteRead;
        int                                     _byteWritten;
        std::map<std::string, IConnector *>     _connectors;
        std::map<std::string, pollfd &>         _fdsetList;
        ComPacket                               _packet;
};

}
#endif
