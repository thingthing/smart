NAME	=	smartapi
CXX	=	g++
RM	=	rm -f
INCLUDE	=	./Include/
CXXFLAGS	= -W -Wall $(foreach dir, $(INCLUDE), -I$(dir))

SRCS	=	./src/main.cpp			\
		./src/Landmarks.cpp		\
		./src/DataAssociation.cpp	\
		./src/SystemStateMatrice.cpp

OBJS	=	$(SRCS:.cpp=.o)

all	: $(NAME)

unit	:
	make re -C Test/

unitfclean 	:
		make fclean -C Test/

$(NAME)	: $(OBJS)
	$(CXX) $(OBJS) -o $(NAME)

clean	:
	$(RM) $(OBJS)

fclean	: clean
	$(RM) $(NAME)

re	: fclean all
