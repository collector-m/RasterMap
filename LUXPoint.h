#ifndef LUXPOINT_H_INCLUDED
#define LUXPOINT_H_INCLUDED

#include "Point.h"

#define LUX_MAX_POINTS 11360
#define LUX_MAX_OBJECTS 256
#define LUX_MAX_CONTOURPOINTS 256

typedef struct Header {
	float m_startAngle;
	float m_endAngle;
	unsigned int m_scannerStatus;
	unsigned int m_flags;
	unsigned int m_scanNumber;
	unsigned char m_deviceID;
	unsigned char m_scannerType;
	unsigned int m_uPoints;
} LUX_header;

typedef struct RawPoint {
	float m_x;
	float m_y;
	float m_z;
	float m_dist;
	float m_hAngle;
	float m_vAngle;
	float m_echoWidth;
	unsigned short m_flags;
	unsigned char m_deviceID;
	unsigned char m_channel;
	unsigned char m_subchannel;
	unsigned char m_millisec;
	unsigned short m_microsec;
	unsigned short m_segmentId;
} LUX_RawPoint;

typedef struct RawPoints{
	float m_startAngle;
	float m_endAngle;
	unsigned int m_scannerStatus;
	unsigned int m_flags;
	unsigned int m_scanNumber;
	unsigned char m_deviceID;
	unsigned char m_scannerType;
	unsigned int m_uPoints;
	LUX_RawPoint m_pts[LUX_MAX_POINTS];
} LUX_RawPoints;

typedef struct RawObject{
	enum ObjectClassification
	{
		Unclassified = 0,	///< = 0: No classification determined (yet)
		UnknownSmall = 1,	///< = 1: Relatively small structure like pole, tree, single bush, etc. which does not fit to other classes.
		UnknownBig = 2,		///< = 2: Bigger structure which does not fit other classes.
		Pedestrian = 3,		///< = 3: Pedestrian, usually determined by moving behaviour.
		Bike = 4,			///< = 4: bike, motor bike
		Car = 5,			///< = 5: Passenger car.
		Truck = 6,			///< = 6: Big vehicle, bus, truck.
		Structure_Pylon = 7,			///< = 7: Only enhanced, special classification (staf).
		Structure_Beacon = 8,			///< = 8: Only enhanced, special classification (staf).
		Structure_GuardRail = 9,		///< = 9: Only enhanced, special classification (staf).
		Structure_ConcreteBarrier = 10, ///< = 10: Only enhanced, special classification (staf).
		NumClasses,			///< Number of currently defined classes except Unknown.
		Unknown = 15		///< = 15: Compatibility to earlier software which does not determine between UnknownBig or -Small or Unclassified.
	};
	struct LUXPoint2D{
		float m_x;
		float m_y;
	};

	unsigned short	m_objectId;
	unsigned short  m_flags; ///< reserved
	// Data from the tracking/classification:
	unsigned int 	m_objectAge;            ///< number of scans in which this object has been tracked, or instead time?
	unsigned short	m_hiddenStatusAge;      ///< Counts how long the object has not been observed but only predicted.

	ObjectClassification m_classification; ///< The object class that is most likely for this object.
	unsigned int  m_classificationAge;           ///< Counts how long the object has been classified in the current classification.
	float   m_classificationQuality;       ///< The quality of the current classification.

	LUXPoint2D m_centerPoint;      ///< Center point of object rectangle, given in Vehicle coordinate system.
	LUXPoint2D m_centerPointSigma;
	float   m_courseAngle;      ///< named by ISO 8855; also called Orientation or Heading [rad]
	float   m_courseAngleSigma; // in [rad]
	LUXPoint2D m_relativeVelocity; ///< Velocity of this object [meter/seconds], relative to the vehicle coordinate system.
	LUXPoint2D m_relativeVelocitySigma;
	LUXPoint2D m_absoluteVelocity; ///< Velocity of this object [meter/seconds] as absolute velocity; the orientation is relative to the vehicle coordinate system.
	LUXPoint2D m_absoluteVelocitySigma;

	LUXPoint2D m_objectBox;         ///< The object's length and width as a rectangle, relative to the object's coordinate system.
	LUXPoint2D m_objectBoxSigma;
	LUXPoint2D m_boundingBoxCenter; ///< Center of the bounding box.
	LUXPoint2D m_boundingBox;       ///< A rectangle in parallel to the vehicle coordinate system (a paraxial rectangle) that contains (bounds) all of this object's points

	// These components are also proposed
	LUXPoint2D m_closestPoint; ///< The point of this object that is closest to the origin of the vehicle coordinate system.

	// This can also be calculated
	LUXPoint2D m_contourPoints[LUX_MAX_CONTOURPOINTS]; ///< A poly-line that describes the outline of the current object measurement.

	unsigned long long m_vehicleWLANid;    ///< An identifier to be used by WLAN fusion algorithms.
	float m_objectHeight;      ///< The height of this object in [m] (most probably received through WLAN data).
	float m_objectHeightSigma; ///< The standard deviation of the height of this object in [m] (most probably received through WLAN data).

	float m_objectMass; ///< The mass of this object in [kilogram] (as received e.g. through WLAN data)

	/** True, if this object is valid. */
	bool m_isValid;
} LUX_RawObject;

typedef struct RawObjects
{
	unsigned int m_uObjects;
	LUX_RawObject m_objs[LUX_MAX_OBJECTS];
} LUX_RawObjects;

const float filter_right = 1.8;
const float filter_left = -1.8;
const float filter_front = 80.0;
const float filter_back = 0.0;

class LUXPoint: public Point
{
public:
//    LUXPoint(int channel = 0, double dist = 0, double x = 0, double y = 0, double z = 0, double hAngle = 0, double vAngle = 0);

    LUXPoint(int channel = 0, double dist = 0, double x = 0, double y = 0);
    //unnecessary functions
    //LUXPoint(const LUXPoint& lux_pt);
    //LUXPoint& operator=(const LUXPoint& lux_pt);
    //~LUXPoint();

    //void setChannel(int channel);
    //void setZ(double z);
    //void setAngle(double hAngle, double vAngle);
    //void setHorizonalAngle(double hAngle);
    //void setVerticalAngle(double vAngle);

    //int getChannel() const;
    //double getZ() const;
    //double getHorizonalAngle() const;
    //double getVerticalAngle() const;

    int channel;
	bool IsInFilterBox();
	bool IsInFilterBox() const;
    //double z;
    //double hAngle, vAngle;  //horizon angle, vertical angle
private:

};



#endif // LUXPOINT_H_INCLUDED
