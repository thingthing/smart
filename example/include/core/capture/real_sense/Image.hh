#ifndef		_IMAGE_HH_
#define		_IMAGE_HH_

class Image
{
public:
	Image(const uint8_t* rgb_image, size_t height, size_t width,
		double fx, double fy, double cx, double cy, const float *disto)
		: _rgb(rgb_image), _height(height), _width(width),
		_fx(fx), _fy(fy), _cx(cx), _cy(cy), _disto(disto) {}
	~Image() {}
	
private:
	Image();

public:
	const uint8_t	*_rgb;
	size_t			_height;
	size_t			_width;
	//focal
	double			_fx;
	double			_fy;
	//Center point
	double			_cx;
	double			_cy;
	//Distortion matrice
	const float	*_disto;
};

#endif		/*! _IMAGE_HH_ */