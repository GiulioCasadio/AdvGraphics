#pragma once

#include <math.h>
#include "vector3.h"
#include "point3.h"
#include "versor3.h"
#include "euler.h"
#include "quaternion.h"
#include "matrix3.h"

/* AxisAngle class */
/* this class is a candidate to store a rotation! */
/* as such, it implements all expected methods    */

//class Matrix3;
class Quaternion;
class Euler;

class AxisAngle{
public:

    /* fields */
    // TODO A-Field: DONE which fields to store? (also add a constuctor taking these fields).
    Scalar x, y, z, angle;

    AxisAngle(Scalar a, Scalar b, Scalar c, Scalar ang): x(a), y(b), z(c), angle(ang) {}
    AxisAngle(const AxisAngle& a){
        AxisAngle(a.x,a.y,a.z,a.angle);
    }

    // TODO A-Ide DONE: this constructor construct the identity rotation
    AxisAngle(){
        Versor3 v = Versor3::forward();
        x = v.x;
        y = v.y;
        z = v.z;
        angle = 0.0;
    }

    // TODO A-FromPoint DONE
    // returns a AxisAngle encoding a point
    //AxisAngle(const Point3& p) {
    //    // TODO
    //    Versor3 v = normalize(p.asVector());
    //    x = v.x;
    //    y = v.y;
    //    z = v.z;
    //    angle = 0.0;
    //}

    //Vector3 apply( Vector3  v) const {
    //    // TODO A-App: how to apply a rotation of this type?
    //    return Vector3();
    //}

    // Rotations can be applied to versors or vectors just as well
    /*Versor3 apply( Versor3 dir ) const {
        return apply( dir.asVector() ).asVersor();
    }

    Point3 apply( Point3 p ) const {
        return apply( p.asVector() ).asPoint();
    }*/

    // syntactic sugar: "R( p )" as a synomim of "R.apply( p )"
    /*Versor3 operator() (Versor3 p) { return apply(p); }
    Point3  operator() (Point3  p) { return apply(p); }
    Vector3 operator() (Vector3 p) { return apply(p); }*/

    //Versor3 axisX() const;  // TODO A-Ax a
    //Versor3 axisY() const;  // TODO A-Ax b
    //Versor3 axisZ() const;  // TODO A-Ax c

    // conjugate
    AxisAngle operator * (AxisAngle r) const {
        return AxisAngle();
    }

    AxisAngle inverse() const{
        // TODO A-Inv a DONE
        return AxisAngle(-x, -y, -z, angle);
    }

    void invert(){
        // TODO A-Inv b DONE
        x *= -1;
        y *= -1;
        z *= -1;
    }

    // returns a rotation to look toward target, if you are in eye, and the up-vector is up
    //static AxisAngle lookAt( Point3 eye, Point3 target, Versor3 up = Versor3::up() ){
    //    // TODO A-LookAt
    //    return AxisAngle();
    //}

    //// returns a rotation
    //static AxisAngle toFrom( Versor3 to, Versor3 from ){
    //    // TODO A-ToFrom
    //    return AxisAngle();
    //}

    /*static AxisAngle toFrom( Vector3 to, Vector3 from ){
        return toFrom( normalize(to) , normalize(from) );
    }*/

    // conversions to this representation
    static AxisAngle from(Matrix3 mat) {
        Scalar x, y, z, angle;
        x = (mat.m[2][1] - mat.m[1][2]) / sqrt(pow((mat.m[2][1] - mat.m[1][2]), 2) + pow((mat.m[0][2] - mat.m[2][0]), 2) + pow((mat.m[1][0] - mat.m[0][1]), 2));
        y = (mat.m[0][2] - mat.m[2][0]) / sqrt(pow((mat.m[2][1] - mat.m[1][2]), 2) + pow((mat.m[0][2] - mat.m[2][0]), 2) + pow((mat.m[1][0] - mat.m[0][1]), 2));
        z = (mat.m[1][0] - mat.m[0][1]) / sqrt(pow((mat.m[2][1] - mat.m[1][2]), 2) + pow((mat.m[0][2] - mat.m[2][0]), 2) + pow((mat.m[1][0] - mat.m[0][1]), 2));
        angle = acos((mat.m[0][0] + mat.m[1][1] + mat.m[2][2] - 1) / 2);

        return AxisAngle(x,y,z,angle);
    }// TODO M2A DONE
    static AxisAngle from(Euler e) {
        return from(Matrix3::from(e));
    }// TODO E2A DONE
    static AxisAngle from(Quaternion q) {
        return AxisAngle(q.xi / sqrt(1 - pow(q.d,2)), q.yi / sqrt(1 - pow(q.d, 2)), q.zi / sqrt(1 - pow(q.d, 2)),2*acos(q.d));
    }// TODO Q2A DONE

    // does this AxisAngle encode a poont?
    //bool isPoint() const{
    //    // TODO A-isP
    //    return false;
    //}

    void printf() const {
        std::printf("x: %f, y: %f, z: %f, angle: %f \n", x, y, z, angle);
    } // TODO Print DONE
};


// interpolation or roations
//inline AxisAngle lerp( const AxisAngle& a,const AxisAngle& b, Scalar t){
//    // TODO A-Lerp: how to interpolate AxisAngles
//    // hints: shortest path! Also, consdider them are 4D unit vectors.
//    return AxisAngle();
//}





