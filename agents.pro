TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
    src/core/main.cpp \
    src/core/Core.cpp \
    src/network/CircularBuffer.cpp \
    src/protocol/AgentProtocol.cpp \
    src/network/TCPConnector.cpp \
    src/network/Packet.cpp \
    src/slam/CovarianceMatrice.cpp \
    src/slam/DataAssociation.cpp \
    src/slam/KalmanGainMatrice.cpp \
    src/slam/Landmarks.cpp \
    src/slam/Slam.cpp \
    src/slam/SystemStateMatrice.cpp \
    src/utils/AThread.cpp \
    src/utils/event/Dispatcher.cpp \
    src/protocol/AProtocol.cpp

HEADERS += \
    include/network/CircularBuffer.h \
    include/protocol/AgentProtocol.h \
    include/protocol/AProtocol.h \
    include/network/TCPSocket.h \
    include/network/Socket.h \
    include/network/TCPConnector.h \
    include/network/ANetworkAdapter.h \
    include/network/Packet.h \
    include/core/defines.h \
    include/slam/Agent.hh \
    include/slam/CovarianceMatrice.hh \
    include/slam/DataAssociation.hh \
    include/slam/KalmanGainMatrice.hh \
    include/slam/Landmarks.hh \
    include/slam/Slam.hh \
    include/slam/SystemStateMatrice.hh \
    include/core/Agent.hh \
    include/core/Core.h \
    include/utils/AThread.h \
    include/utils/NonCopyable.h \
    include/utils/event/Dispatcher.h

INCLUDEPATH += include \
              include/core \
              include/network \
              include/protocol  \
              include/slam    \
              include/utils    \
              /usr/include/jsoncpp

OTHER_FILES += \
    Makefile \
    CMakeLists.txt \
    src/CMakeLists.txt
