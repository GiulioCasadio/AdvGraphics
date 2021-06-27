#pragma once

#include <math.h>
#include "vector3.h"
#include "point3.h"
#include "versor3.h"
#include "quaternion.h"
#include "euler.h"
#include "axis_angle.h"
#include <iostream>

/* Matrix3 class */
/* this class is a candidate to store a rotation! */
/* as such, it implements all the expected methods    */

class Quaternion;
class AxisAngle;
class Euler;

class Matrix3 {
public:

    /* fields */
    // TODO M-Fields:  DONE which fields to store? (also add a constuctor taking these fields).
    Scalar m[3][3];

    Matrix3(Scalar mat[3][3]) {
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                m[i][j] = mat[i][j];
    }

    // TODO M-Ide:  DONE this constructor construct the identity rotation
    Matrix3() {
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++) {
                if (i == j)
                    m[i][j] = 1.0;
                else
                    m[i][j] = 0.0;
            }
    }

    // constructor that takes as input the coefficient (RAW-MAJOR order!)
    Matrix3(Scalar m00, Scalar m01, Scalar m02,
        Scalar m10, Scalar m11, Scalar m12,
        Scalar m20, Scalar m21, Scalar m22) {
        // TODO M-Constr DONE
        m[0][0] = m00; m[0][1] = m01; m[0][2] = m02;
        m[1][0] = m10; m[1][1] = m11; m[1][2] = m12;
        m[2][0] = m20; m[2][1] = m21; m[2][2] = m22;
    }

    Vector3 apply(Vector3  v) const {
        // TODO M-App: DONE how to apply a rotation of this type?
        Vector3 r;
        r.x = m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z;
        r.y = m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z;
        r.z = m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z;
        return r;
    }

    // Rotations can be applied to versors or vectors just as well
    Versor3 apply(Versor3 dir) const {
        return apply(dir.asVector()).asVersor();
    }

    Point3 apply(Point3 p) const {
        return apply(p.asVector()).asPoint();
    }

    // syntactic sugar: "R( p )" as a synomim of "R.apply( p )"
    Versor3 operator() (Versor3 p) { return apply(p); }
    Point3  operator() (Point3  p) { return apply(p); }
    Vector3 operator() (Vector3 p) { return apply(p); }

    Versor3 axisX() const {
        return Vector3(m[0][0], m[1][0], m[2][0]).asVersor(); //devo normalizzare per rendere in modulo 1?
    }// TODO M-Ax a DONE
    Versor3 axisY() const {
        return Vector3(m[0][1], m[1][1], m[2][1]).asVersor();
    }// TODO M-Ax b DONE
    Versor3 axisZ() const {
        return Vector3(m[0][2], m[1][2], m[2][2]).asVersor();
    }  // TODO M-Ax c DONE

    // combine two rotations (r goes first!)
    Matrix3 operator * (Matrix3 r) const {
        Matrix3 res;
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++) {
                res.m[i][j] = m[i][0] * r.m[0][j] + m[i][1] * r.m[1][i] + m[i][2] * r.m[2][i];
            }

        return res;
    }

    bool operator == (Matrix3 aux) const {
        for (int i = 0; i < 3; i++)for(int i=0;i<3;i++)
            for (int j = 0; j < 3; j++) {
                if (aux.m[i][j] != m[i][j])
                    return false;
            }
        return true;
    }

    Matrix3 inverse() const {
        // TODO M-Inv a DONE

        // Calcolo determinante
        Scalar det = 0;
        Matrix3 inv;
        for (int i = 0; i < 3; i++) {
            det += m[0][i] * m[1][(i + 1) % 3] * m[2][(i + 2) % 3];
        }
        det -= m[0][2] * m[1][1] * m[2][0];
        det -= m[0][1] * m[1][0] * m[2][2];
        det -= m[0][0] * m[1][2] * m[2][1];

        // Posso invertire solo se la matrice non e' singolare. 
        if (det != 0) {
            // creo matrice dei cofattori, faccio la trasposta della matrice e divido per il reciproco del determinante di m
            inv.m[0][0] = (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * (-det);
            inv.m[0][1] = -(m[0][1] * m[2][2] - m[0][2] * m[2][1]) * (-det);
            inv.m[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * (-det);

            inv.m[1][0] = -(m[1][0] * m[2][2] - m[1][2] * m[2][0]) * (-det);
            inv.m[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * (-det);
            inv.m[1][2] = -(m[0][0] * m[1][2] - m[0][2] * m[1][0]) * (-det);

            inv.m[2][0] = (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * (-det);
            inv.m[2][1] = -(m[0][0] * m[2][1] - m[0][1] * m[2][0]) * (-det);
            inv.m[2][2] = (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * (-det);
            return inv;
        }
        else // Qualora lo fosse allora stampo un errore e restituisco una matrice vuota.
            std::printf("Errore, matrice non invertibile");

        return Matrix3();
    }

    void invert() const {
        // TODO M-Inv b DONE

        Scalar inv[3][3];
        // Calcolo determinante
        Scalar det = 0;
        for (int i = 0; i < 3; i++) {
            det += m[0][i] * m[1][(i + 1) % 3] * m[2][(i + 2) % 3];
        }
        det -= m[0][2] * m[1][1] * m[2][0];
        det -= m[0][1] * m[1][0] * m[2][2];
        det -= m[0][0] * m[1][2] * m[2][1];

        if (det != 0) {
            // creo matrice dei cofattori, faccio la trasposta della matrice e divido per il reciproco del determinante di m
            inv[0][0] = (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * (-det);
            inv[0][1] = -(m[0][1] * m[2][2] - m[0][2] * m[2][1]) * (-det);
            inv[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * (-det);

            inv[1][0] = -(m[1][0] * m[2][2] - m[1][2] * m[2][0]) * (-det);
            inv[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * (-det);
            inv[1][2] = -(m[0][0] * m[1][2] - m[0][2] * m[1][0]) * (-det);

            inv[2][0] = (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * (-det);
            inv[2][1] = -(m[0][0] * m[2][1] - m[0][1] * m[2][0]) * (-det);
            inv[2][2] = (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * (-det);

            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    const_cast <Matrix3*> (this)->m[i][j] = inv[i][j];
                }
            }
        }
        else
            std::printf("Errore, matrice non invertibile");
        // Posso invertire solo se la matrice non e' singolare. 
        // Qualora lo fosse allora stampo un errore e non modifico la matrice.

    }

    // returns a rotation to look toward target, if you are in eye, and the up-vector is up
    static Matrix3 lookAt(Point3 eye, Point3 target, Versor3 up = Versor3::up()) {
        // TODO M-LookAt
        return Matrix3();
    }

    // returns a rotation
    static Matrix3 toFrom(Versor3 to, Versor3 from) {
        // TODO M-ToFrom
        return Matrix3();
    }

    static Matrix3 toFrom(Vector3 to, Vector3 from) {
        return toFrom(normalize(to), normalize(from));
    }

    // conversions to this representation
    static Matrix3 from(Quaternion q) {
        Scalar matrice[3][3];

        matrice[0][0] = 1 - 2 * (pow(q.yi,2) + pow(q.zi, 2));
        matrice[0][1] = 2 * q.xi * q.yi - 2 * q.zi * q.d;
        matrice[0][2] = 2 * q.xi * q.zi + 2 * q.yi * q.d;

        matrice[1][0] = 2 * q.xi * q.yi + 2 * q.zi * q.d;
        matrice[1][1] = 1 - 2 * pow(q.xi,2) - 2 * pow(q.zi,2);
        matrice[1][2] = 2 * q.yi * q.zi - 2 * q.xi * q.d;

        matrice[2][0] = 2 * q.xi * q.zi - 2 * q.yi * q.d;
        matrice[2][1] = 2 * q.yi * q.zi + 2 * q.xi * q.d;
        matrice[2][2] = 1 - 2 * pow(q.xi,2) - 2 * pow(q.yi,2);

        return Matrix3(matrice);
    }// TODO Q2M DONE
    static Matrix3 from(Euler e) {
        return from(Quaternion::from(e));
    }// TODO E2M DONE
    static Matrix3 from(AxisAngle e){
        return from(Quaternion::from(e));
    } // TODO A2M DONE
     // does this Matrix3 encode a rotation?
    bool isRot() const{ // TODO M-isR DONE
        // Calcolo determinante
        Scalar det = 0;
        for (int i = 0; i < 3; i++) {
            det += m[0][i] * m[1][(i + 1) % 3] * m[2][(i + 2) % 3];
        }
        det -= m[0][2] * m[1][1] * m[2][0];
        det -= m[0][1] * m[1][0] * m[2][2];
        det -= m[0][0] * m[1][2] * m[2][1];

        // Controllo se é ortogonale
        Matrix3 trasp = Matrix3(m[0][0], m[1][0], m[2][0], 
                                m[0][1], m[1][1], m[2][1],
                                m[0][2], m[1][2], m[2][2]);

        Matrix3 aux = Matrix3(const_cast<Matrix3*> (this)->m);

        Matrix3 orto = trasp * aux;

        if (det == 1 && orto==Matrix3()) 
            return true;

        return false;
    }

    /// return a rotation matrix around an axis
    static Matrix3 rotationX(Scalar angleDeg) {
        Matrix3 res = Matrix3(); // identita'

        res.m[1][1] = cos(angleDeg);
        res.m[1][2] = -sin(angleDeg);
        res.m[2][1] = sin(angleDeg);
        res.m[2][2] = cos(angleDeg);

        return res;
    }// TODO M-Rx DONE
    static Matrix3 rotationY(Scalar angleDeg) {
        Matrix3 res = Matrix3(); // identita'

        res.m[0][0] = cos(angleDeg);
        res.m[0][2] = sin(angleDeg);
        res.m[2][0] = -sin(angleDeg);
        res.m[2][2] = cos(angleDeg);

        return res;
    }// TODO M-Ry DONE
    static Matrix3 rotationZ( Scalar angleDeg ) {
        Matrix3 res = Matrix3(); // identita'

        res.m[0][0] = cos(angleDeg);
        res.m[0][1] = -sin(angleDeg);
        res.m[1][0] = sin(angleDeg);
        res.m[1][1] = cos(angleDeg);

        return res;
    }   // TODO M-Rz DONE

    void printf() const {
        std::printf("/ %f, %f, %f \\\n", m[0][0], m[0][1], m[0][2]);
        std::printf("| %f, %f, %f |\n", m[1][0], m[1][1], m[1][2]);
        std::printf("\\ %f, %f, %f /\n", m[2][0], m[2][1], m[2][2]);
    } // TODO Print DONE
};


// interpolation of roations
inline Matrix3 directLerp( const Matrix3& a,const Matrix3& b, Scalar t){
    // TODO M-directLerp: how to interpolate Matrix3s
    return Matrix3();
}

inline Matrix3 lerp( const Matrix3& a,const Matrix3& b, Scalar t){
    // TODO M-smartLerp: how to interpolate Matrix3s
    return Matrix3();
}



