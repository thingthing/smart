#ifndef		NON_COPYABLE_H_
# define	NON_COPYABLE_H_

# define	NON_COPYABLE(classname)		\
    protected: \
  classname	&operator=(const classname &) = delete;	\
  classname(const classname &) = delete;

#endif
