#ifndef			_DEPTH_HH_
#define			_DEPTH_HH_

class Depth
{
public:
	Depth(const uint16_t *depth, size_t height, size_t width,
		double fx, double fy, double cx, double cy, const float *disto)
		: _depth(depth), _height(height), _width(width),
		_fx(fx), _fy(fy), _cx(cx), _cy(cy), _disto(disto) {}
	~Depth() {}
	
private:
	Depth();

public:
	const uint16_t	*_depth;
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

#endif			/*! _DEPTH_HH_ */