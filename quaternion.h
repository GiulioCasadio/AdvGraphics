#pragma once

#include <math.h>
#include "vector3.h"
#include "point3.h"
#include "versor3.h"
#include "euler.h"
#include "matrix3.h"
#include "axis_angle.h"

/* Quaternion class */
/* this class is a candidate to store a rotation! */
/* as such, it implements all expected methods    */

class Matrix3;
class AxisAngle;
class Euler;

class Quaternion{
public:

    /* fields */
    // TODO Q-Fields: DONE which fields to store? (also add a constuctor taking these fields).

    Scalar xi, yi, zi, d;

    Quaternion(Scalar a, Scalar b, Scalar c, Scalar d) :xi(a), yi(b), zi(c), d(d) {
        // TODO Q-Constr DONE
    }

    // TODO Q-Ide: DONE this constructor construct the identity rotation
    // aka no rotation
    Quaternion():Quaternion(0,0,0,1){}

    // TODO Q-FromPoint DONE
    // returns a quaternion encoding a point
    Quaternion( const Point3& p):xi(p.x), yi(p.y), zi(p.z), d(0) {
        // TODO
    }

    Vector3 apply( Vector3  v) const {
        // TODO Q-App: how to apply a rotation of this type?
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

    Versor3 axisX() const {
        return inverse().apply(Versor3::right());
    }// TODO Q-Ax a DONE
    Versor3 axisY() const {
        return inverse().apply(Versor3::up());
    }  // TODO Q-Ax b DONE
    Versor3 axisZ() const {
        return inverse().apply(Versor3::forward());
    }  // TODO Q-Ax c DONE

    // conjugate
    Quaternion operator * (Quaternion r) const {
        Scalar xres = (d * r.xi) + (xi * r.d) + (yi * r.zi) - (zi * r.yi);
        Scalar yres = (d * r.yi) - (xi * r.zi) + (yi * r.d) + (zi * r.xi);
        Scalar zres = (d * r.zi) + (xi * r.yi) - (yi * r.xi) + (zi * r.d);
        Scalar dres = (d * r.d) - (xi * r.xi) - (yi * r.yi) - (zi * r.zi);

        return Quaternion(xres, yres, zres, dres);
    }

    Quaternion operator /= (Scalar s) const {
        return Quaternion(xi / s, yi / s, zi / s, d / s);
    }

    Quaternion inverse() const{
        // TODO Q-Inv a DONE
        Quaternion inv = conjugated();
        Scalar norm = abs(sqrt(pow(xi, 2)+ pow(yi, 2)+ pow(zi, 2)+ pow(d, 2)));
        inv /= pow(norm,2);
        return inv;
    }

    void invert() const{
        Quaternion inv = conjugated();
        Scalar norm = abs(sqrt(pow(xi, 2) + pow(yi, 2) + pow(zi, 2) + pow(d, 2)));
        const_cast <Quaternion*> (this)->xi /= pow(norm, 2);
        const_cast <Quaternion*> (this)->yi /= pow(norm, 2);
        const_cast <Quaternion*> (this)->zi /= pow(norm, 2);
        const_cast <Quaternion*> (this)->d /= pow(norm, 2);

        // TODO Q-Inv b DONE
    }

    // specific methods for quaternions...
    Quaternion conjugated() const{
        // TODO Q-Conj a DONE
        return Quaternion(-xi, -yi, -zi, d);
    }

    void conjugate(){
        xi *= -1;
        yi *= -1;
        zi *= -1;

        // TODO Q-Conj b DONE
    }

    // returns a rotation to look toward target, if you are in eye, and the up-vector is up
    static Quaternion lookAt( Point3 eye, Point3 target, Versor3 up = Versor3::up() ){
        // TODO Q-LookAt
        return Quaternion();
    }

    // returns a rotation
    //static Quaternion toFrom( Versor3 to, Versor3 from ){
    //    // TODO Q-ToFrom
    //    //return Quaternion::from(AxisAngle::toFrom(to, from));
    //}

    /*static Quaternion toFrom( Vector3 to, Vector3 from ){
        return toFrom( normalize(to) , normalize(from) );
    }*/

    // conversions to this representation
    static Quaternion from(const  Matrix3& m) {
        Scalar a, b, c, d;

        d = sqrt(1 + m.m[0][0] + m.m[1][1] + m.m[2][2]) / 2;
        a = (m.m[2][1] - m.m[1][2]) / (4 * d);
        b = (m.m[0][2] - m.m[2][0]) / (4 * d);
        c = (m.m[1][0] - m.m[0][1]) / (4 * d);

        return Quaternion(a, b, c, d);
    }// TODO M2Q DONE
    static Quaternion from(const  Euler& e) {
        return Quaternion(from(AxisAngle::from(e)));
    }// TODO E2Q DONE
    
    static Quaternion from(const AxisAngle& e) {
        return Quaternion(e.x * sin(e.angle/2), e.y * sin(e.angle / 2), e.z * sin(e.angle / 2), cos(e.angle / 2));
    }// TODO A2Q DONE

    // does this quaternion encode a rotation?
    //bool isRot() const{
    //    // TODO Q-isR
    //    return false;
    //}

    //// does this quaternion encode a point?
    //bool isPoint() const{
    //    // TODO Q-isP
    //    return false;
    //}

    void printf() const {
        std::printf("x: %f, y: %f, z: %f, d: %f \n", xi, yi, zi, d);
    } // TODO Print DONE
};


// interpolation or roations
//inline Quaternion lerp( const Quaternion& a,const Quaternion& b, Scalar t){
//    // TODO Q-Lerp: how to interpolate quaternions
//    // hints: shortest path! Also, consdider them are 4D unit vectors.
//    return Quaternion();
//}



