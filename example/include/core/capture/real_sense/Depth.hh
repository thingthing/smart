#ifndef			_DEPTH_HH_
#define			_DEPTH_HH_

class Depth
{
public:
	Depth(const uint16_t *depth, size_t height, size_t width)
		: _depth(depth), _height(height), _width(width) {}
	~Depth() {}
	
private:
	Depth();

public:
	const uint16_t	*_depth;
	size_t			_height;
	size_t			_width;
};

#endif			/*! _DEPTH_HH_ */