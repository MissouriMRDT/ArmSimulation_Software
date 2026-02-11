#include "RoveMatrix.h"

TransfMatrix Identity(float scale) {
    return {
        scale, 0.0f, 0.0f, 0.0f,
        0.0f, scale, 0.0f, 0.0f,
        0.0f, 0.0f, scale, 0.0f,
        0.0f, 0.0f, 0.0f, scale
    };
}

TransfMatrix Rotation(float x, float y, float z) 
{
    TransfMatrix result = Identity();

    float cosz = cosf(-z);
    float sinz = sinf(-z);
    float cosy = cosf(-y);
    float siny = sinf(-y);
    float cosx = cosf(-x);
    float sinx = sinf(-x);

    result.m00 = cosz*cosy;
    result.m10 = (cosz*siny*sinx) - (sinz*cosx);
    result.m20 = (cosz*siny*cosx) + (sinz*sinx);
    result.m01 = sinz*cosy;
    result.m11 = (sinz*siny*sinx) + (cosz*cosx);
    result.m21 = (sinz*siny*cosx) - (cosz*sinx);
    result.m02 = -siny;
    result.m12 = cosy*sinx;
    result.m22 = cosy*cosx;

    return result;
}

TransfMatrix Translation(float x, float y, float z) 
{
    TransfMatrix result = Identity();
    result.m03 = x;
    result.m13 = y;
    result.m23 = z;
    return result;
}

TransfMatrix operator * (const TransfMatrix& left, const TransfMatrix& right) 
{
    TransfMatrix result = { 0 };

    // well, if it ain't broke don't fix it
    result.m00 = left.m00*right.m00 + left.m01*right.m10 + left.m02*right.m20 + left.m03*right.m30;
    result.m01 = left.m00*right.m01 + left.m01*right.m11 + left.m02*right.m21 + left.m03*right.m31;
    result.m02 = left.m00*right.m02 + left.m01*right.m12 + left.m02*right.m22 + left.m03*right.m32;
    result.m03 = left.m00*right.m03 + left.m01*right.m13 + left.m02*right.m23 + left.m03*right.m33;
    result.m10 = left.m10*right.m00 + left.m11*right.m10 + left.m12*right.m20 + left.m13*right.m30;
    result.m11 = left.m10*right.m01 + left.m11*right.m11 + left.m12*right.m21 + left.m13*right.m31;
    result.m12 = left.m10*right.m02 + left.m11*right.m12 + left.m12*right.m22 + left.m13*right.m32;
    result.m13 = left.m10*right.m03 + left.m11*right.m13 + left.m12*right.m23 + left.m13*right.m33;
    result.m20 = left.m20*right.m00 + left.m21*right.m10 + left.m22*right.m20 + left.m23*right.m30;
    result.m21 = left.m20*right.m01 + left.m21*right.m11 + left.m22*right.m21 + left.m23*right.m31;
    result.m22 = left.m20*right.m02 + left.m21*right.m12 + left.m22*right.m22 + left.m23*right.m32;
    result.m23 = left.m20*right.m03 + left.m21*right.m13 + left.m22*right.m23 + left.m23*right.m33;
    result.m30 = left.m30*right.m00 + left.m31*right.m10 + left.m32*right.m20 + left.m33*right.m30;
    result.m31 = left.m30*right.m01 + left.m31*right.m11 + left.m32*right.m21 + left.m33*right.m31;
    result.m32 = left.m30*right.m02 + left.m31*right.m12 + left.m32*right.m22 + left.m33*right.m32;
    result.m33 = left.m30*right.m03 + left.m31*right.m13 + left.m32*right.m23 + left.m33*right.m33;

    return result;
}

Vector operator * (TransfMatrix mat, Vector v)
{
    Vector result = { 0, 0, 0 };

    float x = v.x;
    float y = v.y;
    float z = v.z;

    result.x = mat.m00*x + mat.m01*y + mat.m02*z + mat.m03;
    result.y = mat.m10*x + mat.m11*y + mat.m12*z + mat.m13;
    result.z = mat.m20*x + mat.m21*y + mat.m22*z + mat.m23;

    return result;
}

void operator *= (Vector &v, float n)
{
    v.x *= n;
    v.y *= n;
    v.z *= n;
}
