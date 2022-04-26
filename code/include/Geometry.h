#pragma once
#pragma once

#include <glm/glm.hpp>
#include "ParticleSystem.h"

class Plane
{
public:
    Plane(glm::vec3 planePoint, glm::vec3 planeNormal);
    ~Plane();

    glm::vec3 planePoint;
    glm::vec3 planeNormal;
    float planeD;

    bool CheckBottomColision(glm::vec3 particlePos);
    bool CheckTopColision(glm::vec3 particlePos);
    bool CheckLeftColision(glm::vec3 particlePos);
    bool CheckRightColision(glm::vec3 particlePos);
    bool CheckFrontColision(glm::vec3 particlePos);
    bool CheckBackColision(glm::vec3 particlePos);

    glm::vec3* CalculateParticleMirror(glm::vec3 currentPos, glm::vec3 currentVel);
  
private:
    Plane* plane;
};

class CustomSphere {

public:
    CustomSphere(float radiusS, glm::vec3 centerS);
    ~CustomSphere();

    bool CheckCollisionSphere(glm::vec3 particlePos);
    glm::vec3* CalculateParticleMirror(glm::vec3 previousPos, glm::vec3 currentPos, glm::vec3 currentVel);
    glm::vec3 CalculatePointOfCollision(glm::vec3 particlePos, glm::vec3 currentParticlePos);

    glm::vec3 sphereCenter;
    float sphereRadius;

    void SphereMovement(bool enable);
    int sphereMovement;

    enum MOVEMENT {
        LEFT,
        RIGHT,
        STOP
    };

private:
    CustomSphere* sphereC;
};

class CustomCapsule
{
public:
    CustomCapsule(glm::vec3 posA, glm::vec3 posB, float radius);
    ~CustomCapsule();

    glm::vec3 CalculateVectorAB();
    bool CheckCollisionCapsule(glm::vec3 particlePos);
    void CalculateCollision(glm::vec3 previousPos, glm::vec3 currentPos);
    glm::vec3* CalculateParticleMirror(glm::vec3 previousPos, glm::vec3 currentPos, glm::vec3 currentVel);

private:
    CustomCapsule* capsule;

    glm::vec3 positionA;
    glm::vec3 positionB;
    float capsuleRadius;
};