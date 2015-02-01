#ifndef		_MOVEMENT_H_
# define	_MOVEMENT_H_

class Movement
{
public:

	class pos3d
	{
	public:
		pos3d(void) {}
		pos3d(float x, float y, float z) : _x(x), _y(y), _z(z) {}
		~pos3d() {}

		updatePos(float x, float y, float z) {}
		addToPos(float x, float y, float z) {}
	private:
		float	_x;
		float	_y;
		float	_z;
	};

	Movement();
	~Movement();

	void	updateStackMovement(pos3d);
	void	updateStackMovement(float x, float y, float z);

	// A remplir une fois que l'on a les robots..
	void	goUp();
	void	goDown();
	void	goForward();
	void	goBack();
	void	goLeft();
	void	goRight();

private:
	pos3d	_movementStack;
};

#endif	/* !_MOVEMENT_H_ */
