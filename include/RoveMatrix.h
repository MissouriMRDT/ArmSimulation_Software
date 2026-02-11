#ifndef ROVE_MATRIX_H
#define ROVE_MATRIX_H

#include <math.h>

// Represents a column vector of [x, y, z, 1]
struct Vector {
    float x;
    float y;
    float z;
};

// Represents a 4x4 transformation matrix
struct TransfMatrix {
    union {
        struct {
            float m00, m01, m02, m03;  // Matrix first row (4 components)
            float m10, m11, m12, m13;  // Matrix second row (4 components)
            float m20, m21, m22, m23; // Matrix third row (4 components)
            float m30, m31, m32, m33; // Matrix fourth row (4 components)
        };
        float values[4][4];
    };
};

TransfMatrix Identity(float scale = 1.0f);
TransfMatrix Rotation(float x, float y, float z);
TransfMatrix Translation(float x, float y, float z);

TransfMatrix operator * (const TransfMatrix& left, const TransfMatrix& right);
Vector operator * (TransfMatrix mat, Vector v);
void operator *= (Vector &v, float n);

#endif /*ROVE_MATRIX_H*/
