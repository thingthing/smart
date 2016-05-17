#ifndef		_IMAGE_HH_
#define		_IMAGE_HH_

class Image
{
public:
	Image(const uint8_t* rgb_image, size_t height, size_t width)
		: _rgb(rgb_image), _height(height), _width(width) {}
	~Image() {}
	
private:
	Image();

public:
	const uint8_t	*_rgb;
	size_t			_height;
	size_t			_width;		
};

#endif		/*! _IMAGE_HH_ */