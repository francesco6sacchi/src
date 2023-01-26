#pragma once

#ifndef __H_INCLUDE_GEOMETRY__
#define __H_INCLUDE_GEOMETRY__

#include <list>
#include <vector>

#include "general.h"

// Forward declarations
class Triangle;

// Support methods
std::vector<Triangle> ParseSTLfile(string path);
double RegisterPoints(std::vector<Eigen::Vector3d>* pSet1, std::vector<Eigen::Vector3d>* pSet2, Eigen::Matrix4d* pResult);

class Segment {
public:
// constructors
	Segment() {};
    Segment(Eigen::Vector3d p1, Eigen::Vector3d p2): mPoint1(p1), mPoint2(p2) {};
// methods
    double DistanceFrom(Segment s, Eigen::Vector3d* pVec);
    double DistanceFrom(Segment s) { return this->DistanceFrom(s, NULL); };
    double DistanceFrom(Eigen::Vector3d q, Eigen::Vector3d* pVec);
    double DistanceFrom(Eigen::Vector3d q) { return this->DistanceFrom(q, NULL); };
    bool IsInCapsule(double radius, Eigen::Vector3d p) { return this->DistanceFrom(p) <= radius; };
// getter
    Eigen::Vector3d* GetPoint1() { return &(this->mPoint1); };
    Eigen::Vector3d* GetPoint2() { return &(this->mPoint2); };
private:
// data
    Eigen::Vector3d mPoint1;
    Eigen::Vector3d mPoint2;
};

class Triangle {
public:
// constructors
	Triangle() {};
    Triangle(Eigen::Vector3d p1, Eigen::Vector3d p2, Eigen::Vector3d p3): mPoint1(p1), mPoint2(p2), mPoint3(p3) {
		this->mNormal = ((p2 - p1).cross(p3 - p1)).normalized();
	};
    Triangle(Eigen::Vector3d p1, Eigen::Vector3d p2, Eigen::Vector3d p3, Eigen::Vector3d n): mPoint1(p1), mPoint2(p2), mPoint3(p3), mNormal(n) { };
// methods
    double DistanceFrom(Segment s, Eigen::Vector3d* pVec);
    double DistanceFrom(Segment s) { return this->DistanceFrom(s, NULL); };
    double CheckRay(Eigen::Vector3d& p, Eigen::Vector3d &q);
    bool CheckInside(Eigen::Vector3d p);
    Eigen::Vector3d Centroid() { return .3333333333 * (this->mPoint1 + this->mPoint2 + this->mPoint3); };
// getters
    Eigen::Vector3d* GetPoint1() { return &(this->mPoint1); };
    Eigen::Vector3d* GetPoint2() { return &(this->mPoint2); };
    Eigen::Vector3d* GetPoint3() { return &(this->mPoint3); };
	Eigen::Vector3d* GetNormal() { return &(this->mNormal); };
// setters
    void Shift(Eigen::Vector3d& s) { this->mPoint1 += s; this->mPoint2 += s; this->mPoint3 += s; };
private:
// data
    Eigen::Vector3d mPoint1;
    Eigen::Vector3d mPoint2;
    Eigen::Vector3d mPoint3;
	Eigen::Vector3d mNormal;
};

class Plane {
public:
// constructors
    Plane(Eigen::Vector3d p1, Eigen::Vector3d p2, Eigen::Vector3d p3);
// methods
    double DistanceFrom(Eigen::Vector3d q, Eigen::Vector3d* pVec);
    double DistanceFrom(Eigen::Vector3d q) { return this->DistanceFrom(q, NULL); };
    double DistanceFrom(Segment s, Eigen::Vector3d* pVec);
    double DistanceFrom(Segment s) { return this->DistanceFrom(s, NULL); };
    Eigen::Vector3d NormalTo(Eigen::Vector3d p);
    Eigen::Vector3d Normal() { return this->mCoeffs.block<3,1>(0,0); };
// getters
    double Offset() { return this->mCoeffs(4); };
	Eigen::Vector4d* GetCoeffs() { return &(this->mCoeffs); };
private:
// data
	Eigen::Vector4d mCoeffs;   // nx, ny, nz, and d
    Eigen::Vector3d mPoint1;
    Eigen::Vector3d mPoint2;
    Eigen::Vector3d mPoint3;
};

class PointCloud {
public:
// constructors
	PointCloud() { this->mPoints = std::list<Eigen::Vector3d>(); };
	PointCloud(std::list<Eigen::Vector3d> points) { this->mPoints = points; };
	PointCloud(std::list<Eigen::Vector3d>* pPoints) { this->mPoints = *pPoints; };
// methods
    int Size() { return this->mPoints.size(); };
	Eigen::Vector3d* Append(Eigen::Vector3d p);
	void CleanUp() { this->mPoints.clear(); };
    void SweepLin(Eigen::Vector3d dir, double tinf, double tsup);
    void SweepRot(Eigen::Matrix4d T, unsigned int a, bool flipAxis, double tinf, double tsup);
    void ComputeConvexHull(std::list<Triangle>& chull);
// getters
	std::list<Eigen::Vector3d>* GetPoints() { return &(this->mPoints); };
protected:
// data
    std::list<Eigen::Vector3d> mPoints;
};

#endif
