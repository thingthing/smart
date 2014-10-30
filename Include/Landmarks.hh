#ifndef		_LANDMARKS_HH_
# define	_LANDMARKS_HH_

#include <vector>
#include <map>

#define	CONVERSION 		(Math.PI / 180.0) // Convert to radians
#define MAXLANDMARKS	3000	// Max number of landmarks
#define	MAXERROR		0.5		// If a landmarks is within this distance of another landmarks, its the same landmarks
#define	MINOBSERVATIONS	15		// Number of times a landmark must be observed to be recongnized as a landmark
#define	LIFE			40		// Use to reset life counter (counter use to determine whether to discard a landmark or not)
#define	MAX_RANGE		1.0
#define	MAXTRIALS		1000	// RANSAC: max times to run algorithm
#define MAXSAMPLE		10 		// RANSAC: randomly select x points
#define MINLINEPOINTS	30 		// RANSAC: if less than x points left, don't bother trying to find a consensus (stop algorithm)
#define RANSAC_TOLERANCE 0.05	// RANSAC: if point is within x distance of line, its part of the line
#define RANSAC_CONSENSUS 30 	// RANSAC: at leat x votes required to determine if its a line
#define	DEGRESSPERSCAN	0.5

class Landmarks
{
public:
	class  Landmark
	{
	public:
		 Landmark();
		~ Landmark();

	public:
		double	pos[2];	// landmarks (x, y) position relative to map
		int		id;		// lanndmarks unique ID
		int		life; 	// a life counter to determine whether to discard a landmarl
		int		totalTimeObserved;	// the number of times we have seen the landmark
		double 	range;	// last observed range to landmark
		double	bearing;	// last observed bearing to landmark

		// RANSAC : Store equation of a line to be reused
		double	a;
		double	b;
		double 	rangeError;	// distance from robot position to the wall we are using as a landmark (to calculate error)
		double	bearingError; // bearing from robot position to the wall we are using as a landmark (to calculate error)
	};


public:
	Landmarks();
	~Landmarks();

	Landmarks(double degreePerScan);

	int						getSLamId(int id);
	int						addSlamId(int landmarkId, int slamId);
	int						removeBadLandmarks(double laserdata[], double robotPosition[]); // Possibly change array to vector?
	std::vector<Landmark *>	updateAndAddLineLandmarks(std::vector<Landmark *> extractedLandmarks);
	std::vector<Landmark *>	updateAndAddLandmarkUsingEKFResults(bool matched[], int id[], double ranges[], double bearings[], double robotPosition[]);
	int						updateLineLandmark(Landmark *lm);
	std::vector<Landmark *>	extractLineLandmarks(double laserdata[], double robotPosition[]);
	int						alignLandmarkData(std::vector<Landmark *> extractedLandmarks, bool &matched[], int &id[], double &ranges[], double &bearings[], std::map<double, double> &lmrks, std::map<double, double> exlmrks);
	int						addToDB(Landmark *lm);
	
	int						getDBSize() const;
	std::vector<Landmark *> getLandmarkDB() const;

private:
	Landmark 				*updateLandmark(bool matched, int id, double distance, double readingNo, double robotPosition[]);
	Landmark 				*udpdateLandmark(Landmark *lm);

	void					leastSquaresLineEstimate(double laserdata[], double robotPosition[], int selectPoints[], int arraySize, double &a, double &b);
	double					distanceToLine(double x, double y, double a, double b);
	std::vector<Landmark *> extractSpikeLandmarks(double laserdata[], double robotPosition[]);
	Landmark 				*getLandmark(double range, int readingNo, double robotPosition[]);
	Landmark 				*getLineLandmark(double a, double b, double robotPosition[]);
	Landmark 				*getLine(double a, double b);
	Landmark 				*getOrigin();
	void					getClosestAssociation(Landmark *lm, int &id, int &totalTimeObserved);
	int 					getAssociation(Landmark *lm);
	std::vector<Landmark *> removeDouble(std::vector<Landmark *> extractedLandmarks);

	double 					distance(double x1, double y1, double x2, double y2) const;
	double 					distance(const Landmark *&lm1, const Landmark *&lm2) const;


private:
	std::vector<Landmark *>	landmarkDB;
	int						DBSize;
	std::map<int, int>		IDtoID;
	int						EKFLandmarks;
};

#endif