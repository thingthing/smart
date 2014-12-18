NAME	=	test
CXX	=	g++
RM	=	rm -f
INCLUDE	=	./igloo-igloo.1.1.1/ ../Include/
CXXFLAGS	= -W -Wall $(foreach dir, $(INCLUDE), -I$(dir))

SRCS	=	main.cpp			\
		../src/Landmarks.cpp		\
		../src/DataAssociation.cpp	\
		CustomConstraint.cpp		\
		test_landmark_extraction.cpp	\
		test_data_association.cpp

OBJS	=	$(SRCS:.cpp=.o)

all	: CXXFLAGS += -D UNITTEST
all	: $(NAME)

$(NAME)	: $(OBJS)
	$(CXX) $(OBJS) -o $(NAME)

clean	:
	$(RM) $(OBJS)

fclean	: clean
	$(RM) $(NAME)

re	: fclean all
