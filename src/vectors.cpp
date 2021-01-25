#include "vectors.h"

#include <math.h>

#define PI M_PI

float absoluteValue(float a) { return a < 0 ? -1 * a : a; }

float myAtan(float a, float b) {
    if (b == 0) {
        if (a == 0)
            return 0;
        else
            return a / absoluteValue(a) * PI / 2;
    }
    int resSign = 1;
    int modifyRes = 0;
    if (a < 0) {
        resSign = -1;
        a *= -1;
    }
    if (b < 0) {
        modifyRes = 1;
        b *= -1;
    }
    return resSign * (modifyRes ? PI - atan(a / b) : atan(a / b));
}

struct PolarVector toPolar(struct Vector3D vector) {
    struct PolarVector res;
    res.size =
        sqrt(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z);
    res.alpha = myAtan(vector.y, vector.x);
    res.beta = myAtan(vector.z, vector.x);
    res.gamma = myAtan(vector.z, vector.y);
    return res;
}

struct PolarVector getAngleDiff(struct PolarVector a, struct PolarVector b) {
    struct PolarVector res;
    res.size = 0;
    res.alpha = a.alpha - b.alpha;
    res.beta = a.beta - b.beta;
    res.gamma = a.gamma - b.gamma;
    return res;
}

struct PolarVector rotateBy(struct PolarVector v, struct PolarVector a) {
    struct PolarVector res;
    res.size = v.size;
    res.alpha = v.alpha - a.alpha;
    res.beta = v.beta - a.beta;
    res.gamma = v.gamma - a.gamma;
    return res;
}

struct Vector3D toCartesian(struct PolarVector v) {
    struct Vector3D res;

    float commonDivisor =
        sqrt(1 - cos(v.beta) * cos(v.beta) * cos(v.gamma) * cos(v.gamma));

    if (commonDivisor == 0) commonDivisor = 0.00001f;

    res.x = v.size * absoluteValue(cos(v.beta)) *
            sqrt(1 - cos(v.gamma) * cos(v.gamma)) / commonDivisor;

    res.y = v.size * absoluteValue(cos(v.gamma)) *
            sqrt(1 - cos(v.beta) * cos(v.beta)) / commonDivisor;

    res.z = v.size * absoluteValue(sin(v.beta)) *
            sqrt(1 - sin(v.alpha) * sin(v.alpha)) /
            sqrt(1 - sin(v.alpha) * sin(v.alpha) * sin(v.beta) * sin(v.beta));

    if (absoluteValue(v.beta) > PI / 2) res.x *= -1;
    if (absoluteValue(v.gamma) > PI / 2) res.y *= -1;
    if (v.beta < 0) res.z *= -1;

    return res;
}