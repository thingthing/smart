NAME	=	test
CXX	=	g++
RM	=	rm -f
INCLUDE	=	
CXXFLAGS	= -I./igloo-igloo.1.1.1/ -I../Include/

SRCS	=	main.cpp	\
		test_landmark_extraction.cpp

OBJS	=	$(SRCS:.cpp=.o)

all	: $(NAME)

$(NAME)	: $(OBJS)
	$(CXX) $(OBJS) -o $(NAME)

clean	:
	$(RM) $(OBJS)

fclean	: clean
	$(RM) $(NAME)

re	: fclean all