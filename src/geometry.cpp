#include "../include/geometry.h"
#include "../include/QuickHull.hpp"

#include <numeric>      // std::accumulate
#include <assert.h>
#include <float.h>
#include <vector>
#include <Eigen/SVD>
#include <Eigen/Dense>

#define EPS_GEOMETRY_SWEEP		1e-2

double Segment::DistanceFrom(Segment s, Eigen::Vector3d* pVec) {

    Eigen::Vector3d d1 = this->mPoint2 - this->mPoint1, d2 = *s.GetPoint2() - *s.GetPoint1(), r = this->mPoint1 - *s.GetPoint1(), c1, c2;
    double a = d1.dot(d1); // Squared length of segment S1, always nonnegative
	double b = d1.dot(d2), c = d1.dot(r);
    double e = d2.dot(d2); // Squared length of segment S2, always nonnegative
	double f = d2.dot(r), t, v, temp;
    double denom = a*e-b*b; // Always nonnegative
    
    // If segments not parallel, compute closest point on L1 to L2 and
    // clamp to segment S1. Else pick arbitrary s (here 0)
    if (denom != .0) {
        temp = (b*f - c*e) / denom;
        if(temp < .0) v = 0;
        else if(temp > 1.0) v = 1.0;
        else v = temp;
    } else v = .0;
    
    // Compute point on L2 closest to S1(s) using
    // t = Dot((P1 + D1*s) - P2,D2) / Dot(D2,D2) = (b*s + f) / e
    t = (b*v + f) / e;
    
    // If t in [0,1] done. Else clamp t, recompute s for the new value
    // of t using s = Dot((P2 + D2*t) - P1,D1) / Dot(D1,D1)= (t*b - c) / a
    // and clamp s to [0, 1]
    if (t < .0) {
        t = .0;
        temp = -c / a;
        if(temp < .0) v = .0;
        else if(temp > 1.0) v = 1.0;
        else v = temp;
    } else if (t > 1.0) {
        t = 1.0;
        temp = (b - c) / a;
        if(temp < .0) v = .0;
        else if(temp > 1.0) v = 1.0;
        else v = temp;
    }
    
    // returning distance
    c1 = this->mPoint1 + d1 * v;
    c2 = *s.GetPoint1() + d2 * t;
    if(pVec != NULL) *pVec = c2;
    return (c1 - c2).norm();
    
}

double Segment::DistanceFrom(Eigen::Vector3d q, Eigen::Vector3d* pVec) {
    
    Eigen::Vector3d p = this->mPoint1, dir = this->mPoint2 - this->mPoint1;
    double t = dir.dot(q - p) / dir.dot(dir);
    Eigen::Vector3d tempVec = q - (p + t*dir);
    double temp = tempVec.norm();
    if (t < 0) {
        if(pVec != NULL) *pVec = q - this->mPoint1;
        return (q - this->mPoint1).norm();
    } else if (t > 1) {
        if(pVec != NULL) *pVec = q - this->mPoint2;
        return (q - this->mPoint2).norm();
    } else {
        if(pVec != NULL) *pVec = tempVec;
        return temp;
    }
    
}

double Triangle::DistanceFrom(Segment s, Eigen::Vector3d* pVec) {
    
    // the minimum distance occurs either
    //   (a) between an endpoint of the segment and the interior of the triangle or
    //   (b) between the segment and an edge of the triangle
    Eigen::Vector3d vA, vB, v12, v13, v23, best_v;
    double dA, dB, d12, d13, d23, best_d = DBL_MAX;
    
    // testing segment points against triangle (plane)
    Plane p = Plane(*this->GetPoint1(), *this->GetPoint2(), *this->GetPoint3());
    dA = p.DistanceFrom(*s.GetPoint1(), &vA);
    dB = p.DistanceFrom(*s.GetPoint2(), &vB);
    if(this->CheckInside(vA)) {
        if(this->CheckInside(vB)) {
            if(dA < dB) {
                best_v = vA;
                best_d = dA;
            } else {
                best_v = vB;
                best_d = dB;
            }
        }
    } else {
        if(this->CheckInside(vB)) {
            best_v = vB;
            best_d = dB;
        }
    }
    
    // testing segment against edges
    d12 = Segment(*this->GetPoint1(), *this->GetPoint2()).DistanceFrom(s, &v12);
    if(d12 < best_d) {
        best_d = d12;
        best_v = v12;
    }
    d13 = Segment(*this->GetPoint1(), *this->GetPoint3()).DistanceFrom(s, &v13);
    if(d13 < best_d) {
        best_d = d13;
        best_v = v13;
    }
    d23 = Segment(*this->GetPoint2(), *this->GetPoint3()).DistanceFrom(s, &v23);
    if(d23 < best_d) {
        best_d = d23;
        best_v = v23;
    }
    
    // returning
    if(pVec != NULL) *pVec = best_v;
    return best_d;
    
}

double Triangle::CheckRay(Eigen::Vector3d& p, Eigen::Vector3d &q) {
    
    // ray is p + t * (q − p), t >= 0
    Eigen::Vector3d ab = this->mPoint2 - this->mPoint1, ac = this->mPoint3 - this->mPoint1, qp = p - q, ap, e, n = ab.cross(ac);
    
    // Compute denominator d. If d <= 0, segment is parallel to or points
    // away from triangle, so exit early
    double d = qp.dot(n);
    if (d <= .0) return -1.0;
    
    // Compute intersection t value of pq with plane of triangle.
    // A ray intersects iff 0 <= t. Delay dividing by d until intersection has been found to pierce triangle
    ap = p - this->mPoint1;
    double t = ap.dot(n);
    if (t < .0) return -1.0;
    
    // Compute barycentric coordinate components and test if within bounds
    e = qp.cross(ap);
    double v = ac.dot(e);
    if (v < .0 || v > d) return -1.0;
    double w = -(ab.dot(e));
    if (w < .0 || v + w > d) return -1.0;
    
    // Segment/ray intersects triangle. Perform delayed division
    return t / d;
    
}

bool Triangle::CheckInside(Eigen::Vector3d p) {

    // Translate point and triangle so that point lies at origin
    Eigen::Vector3d a = this->mPoint1 - p, b = this->mPoint2 - p, c = this->mPoint3 - p;
    
    // Compute normal vectors for triangles pab and pbc
    Eigen::Vector3d u = b.cross(c), v = c.cross(a);
    
    // Make sure they are both pointing in the same direction
    if (u.dot(v) < .0) return false;
    
    // Compute normal vector for triangle pca
    Eigen::Vector3d w = a.cross(b);
    
    // Make sure it points in the same direction as the first two
    if (u.dot(w) < .0) return false;
    
    // Otherwise P must be in (or on) the triangle
    else return true;

}

Plane::Plane(Eigen::Vector3d p1, Eigen::Vector3d p2, Eigen::Vector3d p3) {

	Eigen::Matrix<double, 3, 4> CollMat;
    this->mPoint1 = p1;
    this->mPoint2 = p2;
    this->mPoint3 = p3;
    
    // check for collinearity of points
	CollMat <<	1.0, p1.transpose(), 1.0, p2.transpose(), 1.0, p3.transpose();
	Eigen::FullPivLU<Eigen::MatrixXd> CollMatLU(CollMat);
    if (CollMatLU.rank() < 3) throw COLLINEAR_POINTS_EXCEPTION;
	Eigen::Vector3d PlaneNormal = (p2 - p1).cross(p3 - p1);
    PlaneNormal.normalize();
    
    // storing parameters
	double ConstantTerm = - PlaneNormal.dot(p1);
	this->mCoeffs << PlaneNormal, ConstantTerm;

}

double Plane::DistanceFrom(Eigen::Vector3d q, Eigen::Vector3d* pVec) {
    
    Eigen::Vector3d PlaneNormal = this->Normal();
    if(PlaneNormal.dot(q - this->mPoint1) < .0) PlaneNormal = -PlaneNormal;
    double d = fabs(PlaneNormal.dot(q - this->mPoint1));
    if(pVec != NULL) *pVec = q - PlaneNormal*d;
    return d;
    
}

double Plane::DistanceFrom(Segment s, Eigen::Vector3d* pVec) {
    
    Eigen::Vector3d s1 = *(s.GetPoint1());
    Eigen::Vector3d s2 = *(s.GetPoint2());
    double d1 = this->DistanceFrom(s1), d2 = this->DistanceFrom(s2), d;
    Eigen::Vector3d PlaneNormal = this->Normal();
    if (PlaneNormal.dot(s1 - this->mPoint1)*PlaneNormal.dot(s2 - this->mPoint1) < 0) {
        if(pVec != NULL) *pVec = Eigen::Vector3d(0,0,0);
        return .0;
    } else {
        if(d1 < d2) {
            if(pVec != NULL) *pVec = s1 - PlaneNormal*d1;
            return d1;
        } else {
            if(pVec != NULL) *pVec = s2 - PlaneNormal*d2;
            return d2;
        }
    }
    
}

Eigen::Vector3d Plane::NormalTo(Eigen::Vector3d p) {
    
    if(this->DistanceFrom(p) == .0) throw COLLINEAR_POINTS_EXCEPTION;
    Eigen::Vector3d v1 = this->mPoint2 - this->mPoint1;
    Eigen::Vector3d v2 = this->mPoint3 - this->mPoint1;
    Eigen::Vector3d n = (v1.cross(v2)).normalized();
    if(((p-this->mPoint1).dot(n)) < .0) return -n;
    else return n;
    
}

Eigen::Vector3d* PointCloud::Append(Eigen::Vector3d p) {
    
    this->mPoints.push_back(p);
    return &(this->mPoints.back()); // return pointer to last element
    
}

void PointCloud::SweepRot(Eigen::Matrix4d T, unsigned int a, bool flipAxis, double tinf, double tsup) {
    
    if (this->mPoints.size() == 0) throw EMPTY_POINT_CLOUD_EXCEPTION;
    if(tinf > tsup || a > 2) throw NON_NEGATIVE_VALUE_EXCEPTION;
    
    double delta;
    Eigen::Vector3d p1, p2, temp, pm;
    PointCloud npc = PointCloud();
    Eigen::Matrix3d Rot  = T.block<3,3>(0,0), RotT = Rot.transpose();;
    Eigen::Vector3d Tran = T.block<3,1>(0,3), Axis;
    if(flipAxis) Axis = -RotT*T.block<3,1>(0, a);
    else Axis = RotT*T.block<3,1>(0, a);
    Eigen::Matrix3d Rinf; Rinf = Eigen::AngleAxisd(tinf, Axis);
    Eigen::Matrix3d Rsup; Rsup = Eigen::AngleAxisd(tsup, Axis);
    
    // Computing new points
    std::list<Eigen::Vector3d>::iterator it;
    for (it = this->mPoints.begin(); it != this->mPoints.end(); ++it) {
        p1 = Rinf*(RotT*(*it)-RotT*Tran);
        p2 = Rsup*(RotT*(*it)-RotT*Tran);
        pm = .5*(p1+p2);
        delta = (pm-p1).norm()*tan(.5*(tsup-tinf));
        temp = Axis.cross(p1-p2);
        if(temp.norm()*delta > EPS_GEOMETRY_SWEEP) npc.Append(Rot*(pm + delta*(temp.normalized())) + Tran);
        npc.Append(Rot*p1 + Tran);
        npc.Append(Rot*p2 + Tran);
    }
    
    // Appending new points to original point cloud
    this->mPoints.clear();
    for (it = npc.mPoints.begin(); it != npc.mPoints.end(); ++it) this->Append(*it);

}

// additional functions
std::vector<Triangle> ParseSTLfile(std::string path) {

    std::ifstream STLfile(path.c_str(), std::ios::in | std::ios::binary);
    if (!STLfile) throw FILE_HANDLING_EXCEPTION;

    std::vector<Triangle> output;
    Eigen::Vector3d v1, v2, v3, n;
    float* f_ptr;
    char header_info[80] = "", n_triangles[4], dummy[2], f_buf[sizeof(float)];
    STLfile.read(header_info, 80);
    STLfile.read(n_triangles, 4);
    unsigned int* r = (unsigned int*)n_triangles;
    unsigned int num_triangles = *r, i, j;
    for (i = 0; i < num_triangles; i++) {
        
        // normal
        for (j = 0; j < 3; j++) {
            STLfile.read(f_buf, 4);
            f_ptr = (float*) f_buf;
            n(j) = (double)(*f_ptr);
        }
        
        // Dividing by 10^6 because triangles dimensions are in m^-6 CREDO
        // 1st vertex
        for (j = 0; j < 3; j++) {
            STLfile.read(f_buf, 4);
            f_ptr = (float*) f_buf;
            v1(j) = (double)(*f_ptr)/1000000;
        }
        
        // 2nd vertex
        for (j = 0; j < 3; j++) {
            STLfile.read(f_buf, 4);
            f_ptr = (float*) f_buf;
            v2(j) = (double)(*f_ptr) / 1000000;
        }
        
        // 3rd vertex
        for (j = 0; j < 3; j++) {
            STLfile.read(f_buf, 4);
            f_ptr = (float*) f_buf;
            v3(j) = (double)(*f_ptr) / 1000000;
        }
        
        // adding triangle
        output.push_back(Triangle(v1, v2, v3, n));
        
        // read dummy
        STLfile.read(dummy, 2);
        
    }
    
    return output;

}

double RegisterPoints(std::vector<Eigen::Vector3d>* pSet1, std::vector<Eigen::Vector3d>* pSet2, Eigen::Matrix4d* pResult) {
    
    double max_error, error;
    *pResult = Eigen::Matrix4d::Identity();
    if(pSet1->size() == 0 || pSet2->size() == 0) throw EMPTY_POINT_CLOUD_EXCEPTION;
    else if(pSet1->size() != pSet2->size()) throw NON_MACHING_DIMENSIONS_EXECPTION;
    
    // centroid of the first set (pi') and second set (pi)
    unsigned int i;
    Eigen::Vector3d p_prime = Eigen::Vector3d(.0,.0,.0), p = p_prime, Tran;
    for(i = 0; i < pSet1->size(); i++) {
        p += (*pSet2)[i];
        p_prime += (*pSet1)[i];
    }
    p /= double(pSet2->size());
    p_prime /= double(pSet1->size());
    
    // registration based on Arun et al. T-PAMI 1987
    // two point sets pi (model) and pi' (scene) are such that pi' = R*pi + T
    Eigen::Matrix3d Rot, H = Eigen::Matrix3d::Zero(), V, Utran;
    for(i = 0; i < pSet1->size(); i++) H += ((*pSet2)[i] - p) * ((*pSet1)[i] - p_prime).transpose();
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullV | Eigen::ComputeFullU);
    svd.computeU(); Utran = svd.matrixU().transpose();
    svd.computeV(); V = svd.matrixV();
    Rot = V * Utran;
    if(Rot.determinant() < .0) {
        // it may happen that SVD returns a reflection (see Arun et al. Section IV point (2))
        V.block<3,1>(0,2) = -V.block<3,1>(0,2);
        Rot = V * Utran;
    }
    Tran = p_prime - Rot * p;
    
    // assigning output
    pResult->block<3,3>(0,0) = Rot;
    pResult->block<3,1>(0,3) = Tran;
    
    // computing and returning maximum registration error
    max_error = .0;
    for(i = 0; i < pSet1->size(); i++) {
        error = ((*pSet1)[i] - (Rot * (*pSet2)[i] + Tran)).norm();
        if(error > max_error) max_error = error;
    }
    return max_error;
    
}
