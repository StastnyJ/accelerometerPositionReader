#define DEFAULT_CALM_VECTOR \
    { .size = 1, .alpha = 0., .beta = -PI / 2, .gamma = -PI / 2 }
#define NULL_POLAR_VECTOR \
    { .size = 0, .alpha = 0., .beta = 0, .gamma = 0 }
#define NULL_VECTOR3D \
    { .x = 0, .y = 0, .z = 0 }

struct PolarVector {
    float size, alpha, beta, gamma;
};

struct Vector3D {
    float x, y, z;
};

struct PolarVector toPolar(struct Vector3D vector);
struct PolarVector getAngleDiff(struct PolarVector a, struct PolarVector b);
struct PolarVector rotateBy(struct PolarVector v, struct PolarVector a);

struct Vector3D toCartesian(struct PolarVector v);
float absoluteValue(struct Vector3D v);
