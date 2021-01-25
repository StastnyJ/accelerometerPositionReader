#include <math.h>

#include "../src/vectors.h"
#include "stdio.h"

#define PI M_PI

void printPolarVector(struct PolarVector v) {
    printf("[%f | %f π, %f π, %f π]\n", v.size, v.alpha / PI, v.beta / PI,
           v.gamma / PI);
}

void printVector3D(struct Vector3D v) {
    printf("[%f, %f, %f]\n", v.x, v.y, v.z);
}

int main() {
    printPolarVector(toPolar({.x = 1, .y = 1, .z = 1}));  // π/4, π/4, π/4
    printPolarVector(toPolar({.x = 0, .y = 0, .z = 1}));  // 0, π/2, π/2
    printPolarVector(toPolar({.x = 1, .y = -1, .z = -1}));  // -π/4, -π/4, -3π/4
    printPolarVector(toPolar({.x = 1, .y = 1, .z = -1}));  // π/4, -π/4, -π/4
    printf("\n");
    printPolarVector(toPolar({.x = 0, .y = 1, .z = -1}));
    printPolarVector(toPolar({.x = 0, .y = 0, .z = -1}));
    printPolarVector(toPolar({.x = 1, .y = 0, .z = -1}));
    printPolarVector(toPolar({.x = 0.75f, .y = 0.52f, .z = -0.39f}));
    printf("----------------------------\n");
    printVector3D(toCartesian(toPolar({.x = 1, .y = 1, .z = 1})));
    printVector3D(toCartesian(toPolar({.x = 1, .y = 1, .z = -1})));
    printVector3D(toCartesian(toPolar({.x = 1, .y = -1, .z = 1})));
    printVector3D(toCartesian(toPolar({.x = -1, .y = -1, .z = 1})));
    printf("\n");
    printVector3D(toCartesian(toPolar({.x = 1, .y = -1, .z = -1})));
    printVector3D(toCartesian(toPolar({.x = 0, .y = 1, .z = -1})));
    printVector3D(toCartesian(toPolar({.x = 0, .y = 0, .z = -1})));
    printVector3D(toCartesian(toPolar({.x = 1, .y = 0, .z = -1})));
    printf("\n");
    printVector3D(toCartesian(toPolar({.x = 0.42f, .y = 0.43f, .z = -0.12f})));
}
