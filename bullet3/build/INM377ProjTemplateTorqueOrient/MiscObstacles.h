#ifndef MISC_OBSTACLES_H
#define MISC_OBSTACLES_H

#include "btBulletDynamicsCommon.h"
#include "Obstacle.h"

// Confinement within a sphere of a given radius centred on the origin
class BowlObstacle : public Obstacle {
	const btScalar radius;

public:
    BowlObstacle(): radius(0.0){}
	BowlObstacle(btScalar r) : radius(r) {}

	virtual bool missed(const btVector3 &pos, btVector3 &target) const;

	virtual bool inPath(const btVector3 &pos, const btVector3 &vel,
		btScalar boundingRadius,
		btScalar &distance, btVector3 &avoidPoint) const;
};

// Solid sphere
class SphereObstacle : public Obstacle {
	const btVector3 centre;
	const btScalar radius;

public:
    SphereObstacle(): centre(btVector3(0,0,0)), radius(0.0){}
	SphereObstacle(const btVector3 &c, btScalar r) :
		centre(c), radius(r) {}

	virtual bool missed(const btVector3 &pos, btVector3 &target) const;

	virtual bool inPath(const btVector3 &pos, const btVector3 &vel,
		btScalar boundingRadius,
		btScalar &distance, btVector3 &avoidPoint) const;
};

// Vertical column of infinite height
class ColumnObstacle : public Obstacle {
	const btVector3 centre;
	const btScalar radius;

public:
    ColumnObstacle(): centre(btVector3(0,0,0)), radius(0.0){}
	ColumnObstacle(const btVector3 &c, btScalar r) :
		centre(c), radius(r) {}

	virtual bool missed(const btVector3 &pos, btVector3 &target) const;

	virtual bool inPath(const btVector3 &pos, const btVector3 &vel,
		btScalar boundingRadius,
		btScalar &distance, btVector3 &avoidPoint) const;
};

// Confinement on the origin side of an infinite plane
class PlaneObstacle : public Obstacle {
	// The plane passes through this point, perpendicular to the vector
	const btVector3 offset;

public:
    PlaneObstacle(): offset(btVector3(0,0,0)){}
	PlaneObstacle(const btVector3 &o) : offset(o) {}

	virtual bool missed(const btVector3 &pos, btVector3 &target) const;

	virtual bool inPath(const btVector3 &pos, const btVector3 &vel,
		btScalar boundingRadius,
		btScalar &distance, btVector3 &avoidPoint) const;
};

#endif
