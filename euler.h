#pragma once

#include <math.h>
#include "versor3.h"
#include "point3.h"
#include "vector3.h"
#include "matrix3.h"
#include <stdio.h>

/* Euler class */
/* this class is a candidate to store a rotation! */
/* as such, it implements all the expected methods    */

class Quaternion;
class AxisAngle;
class Matrix3;


class Euler{
public:

    /* fields */
    // TODO E-Fields DONE: which fields to store? (also add a constuctor taking these fields).
    Scalar x,y,z; // in ordine rollio, beccheggio e bardata

    // TODO E-Ide DONE: this constructor construct the identity rotation
    Euler() : x(0.0), y(0.0), z(0.0) {}

    Euler(Scalar a, Scalar b, Scalar c) : x(a), y(b), z(c) {}

    // TODO E-Constr DONE
    // row major order!
    Euler(Scalar m00, Scalar m01, Scalar m02,
        Scalar m10, Scalar m11, Scalar m12,
        Scalar m20, Scalar m21, Scalar m22)
    {
        Scalar ms[3][3] = { {m00,m01,m02},{m10,m11,m12},{m20,m21,m22} };
        Matrix3 mat = Matrix3(ms);

        Euler e = from(mat);
        x = e.x;
        y = e.y;
        z = e.z;
    }

    Vector3 apply( Vector3  v) const {
        // TODO E-App: how to apply a rotation of this type?
        return Vector3();
    }

    // Rotations can be applied to versors or vectors just as well
    Versor3 apply( Versor3 dir ) const {
        return apply( dir.asVector() ).asVersor();
    }

    Point3 apply( Point3 p ) const {
        return apply( p.asVector() ).asPoint();
    }

    // syntactic sugar: "R( p )" as a synomim of "R.apply( p )"
    Versor3 operator() (Versor3 p) { return apply(p); }
    Point3  operator() (Point3  p) { return apply(p); }
    Vector3 operator() (Vector3 p) { return apply(p); }

    //Versor3 axisX() const;  // TODO E-Ax a
    //Versor3 axisY() const;  // TODO E-Ax b
    //Versor3 axisZ() const;  // TODO E-Ax c

    // conjugate
    Euler operator * (Euler b) const {
        return Euler();
    }

    ////Euler inverse() const{
    ////    // TODO E-Inv a
    ////    return Euler();
    ////}

    ////void invert() const{
    ////    // TODO E-Inv b
    ////}

    ////// specific methods for Eulers...
    ////Euler transposed() const{
    ////    // TODO E-Transp a
    ////    return Euler();
    ////}

    ////void transpose(){
    ////    // TODO E-Transp b
    ////}

    ////// returns a rotation to look toward target, if you are in eye, and the up-vector is up
    ////static Euler lookAt( Point3 eye, Point3 target, Versor3 up = Versor3::up() ){
    ////    // TODO E-LookAt
    ////    return Euler();
    ////}

    // returns a rotation
    //static Euler toFrom( Versor3 to, Versor3 from ){
    //    // TODO E-ToFrom
    //    return Euler();
    //}

    //static Euler toFrom( Vector3 to, Vector3 from ){
    //    return toFrom( normalize(to) , normalize(from) );
    //}

    // conversions to this representation
    static Euler from(Quaternion q) {
        Scalar pole = q.xi * q.yi + q.zi * q.d;
        Scalar x, y, z = 0;

        y = asin(2 * q.xi * q.yi + 2 * q.zi * q.d);

        if (pole == 0.5) {
            x = 2 * atan2(q.xi, q.d);
            return Euler(x, y, z);
        }
        else if (pole == -0.5) {
            x = -2 * atan2(q.xi, q.d);
            return Euler(x, y, z);
        }
        x = atan2(2 * q.yi * q.d - 2 * q.xi * q.zi, 1 - 2 * pow(q.yi, 2) - 2 * pow(q.zi, 2));
        z = atan2(2 * q.xi * q.d - 2 * q.yi * q.zi, 1 - 2 * pow(q.xi, 2) - 2 * pow(q.zi, 2));
        return Euler(x, y, z);
    }// TODO Q2E DONE
    static Euler from(Matrix3 m) {
        return from(Quaternion::from(m));
    }// TODO M2E DONE
    static Euler from(AxisAngle a) {
        return from(Quaternion::from(a));
    }// TODO A2E DONE

    // does this Euler encode a rotation?
    //bool isRot() const{
    //    // TODO E-isR
    //    return false;
    //}

    //// return a rotation matrix around an axis
    //static Euler rotationX( Scalar angleDeg );   // TODO E-Rx
    //static Euler rotationY( Scalar angleDeg );   // TODO E-Rx
    //static Euler rotationZ( Scalar angleDeg );   // TODO E-Rx

    void printf() const {
    } // TODO Print
};


// interpolation or roations
inline Euler directLerp( const Euler& a,const Euler& b, Scalar t){
    // TODO E-directLerp: how to interpolate Eulers
    return Euler();
}

inline Euler lerp( const Euler& a,const Euler& b, Scalar t){
    // TODO E-smartLerp: how to interpolate Eulers
    return Euler();
}


