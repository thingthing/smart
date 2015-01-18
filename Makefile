NAME    = agent

SRC	=						\
        src/core/main.cpp					\
        src/network/TCPConnector.cpp			\
	src/protocol/AgentProtocol.cpp			\
        src/network/CircularBuffer.cpp                  \
        src/network/Packet.cpp                          \
        src/core/Core.cpp

OBJ	= $(SRC:.cpp=.o)

RM	= rm -rf

INCLUDES    +=  -I.				\
                -Iinclude                       \
                -Iinclude/core                  \
                -Iinclude/protocol              \
                -Iinclude/network

DEFINES     =

COMMONFLAGS = -Wall -Wextra -ggdb3 -pthread -std=c++11 $(INCLUDES)
#COMMONFLAGS = -Wall -Wextra -O3 -pthread -std=c++11 $(INCLUDES)
#COMMONFLAGS = -Wall -Wextra -pthread -std=c++11 $(INCLUDES)

CXXFLAGS = $(DEFINES) $(COMMONFLAGS)

LDFLAGS =

$(NAME): $(OBJ)
	$(CXX) $^ $(CXXFLAGS) $(LDFLAGS) -o $@

all: $(NAME)

%.o: %.cpp
	$(CXX) -c $< $(CXXFLAGS) -o $@

clean:
	@$(RM) $(OBJ)

fclean: clean
	@$(RM) $(NAME)

re: fclean all

.PHONY: all clean fclean re
