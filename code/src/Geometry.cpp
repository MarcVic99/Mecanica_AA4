#include "Geometry.h"

glm::vec3 CalculateVectorBetweenTwoPoints(glm::vec3 firstP, glm::vec3 secondP) {
    return firstP - secondP;
}

glm::vec3 CalculatePerpendicularVector(glm::vec3 vector, float Px, float Py)
{
    //vector · newVector = 0
     //vx * nx + vy * ny + vz * nz = 0
    glm::vec3 newVector;
    float Pz;

    Pz = ((vector.x * Px) + (vector.y * Py)) / vector.z;

    return newVector = glm::vec3(Px, Py, Pz);
}

float CalculatePlaneD(glm::vec3 normalVector, glm::vec3 planePoint)
{
    //Components of plane's normal vector
    float A, B, C, D;

    //Components of plane's point 
    float x, y, z;

    A = normalVector.x;
    B = normalVector.y;
    C = normalVector.z;

    x = planePoint.x;
    y = planePoint.y;
    z = planePoint.z;

    D = -(A * x) - (B * y) - (C * z);

    return D;
}

// === PLANE ===
Plane::Plane(glm::vec3 point, glm::vec3 normal)
{
    planePoint = point;
    planeNormal = normal;
}

Plane::~Plane()
{

}

bool Plane::CheckBottomColision(glm::vec3 particlePos)
{
    if (particlePos.y <= planePoint.y)
    {
        return true;
    }
    return false;
}

bool Plane::CheckTopColision(glm::vec3 particlePos) 
{
    if (particlePos.y >= planePoint.y)
    {
        return true;
    }
    return false;
}

bool Plane::CheckLeftColision(glm::vec3 particlePos)
{
    if (particlePos.x <= planePoint.x)
    {
        return true;
    }
    return false;
}

bool Plane::CheckRightColision(glm::vec3 particlePos)
{
    if (particlePos.x >= planePoint.x)
    {
        return true;
    }
    return false;
}
bool Plane::CheckFrontColision(glm::vec3 particlePos)
{
    if (particlePos.z >= planePoint.z)
    {
        return true;
    }
    return false;
}

bool Plane::CheckBackColision(glm::vec3 particlePos)
{
    if (particlePos.z <= planePoint.z)
    {
        return true;
    }
    return false;
}

glm::vec3* Plane::CalculateParticleMirror(glm::vec3 currentPos, glm::vec3 currentVel)
{
    float planeD = CalculatePlaneD(glm::normalize(planeNormal), planePoint);

    glm::vec3 mirrorRes[2];

    // == Mirror ==
    //p = p' -2(n * p' + d) * n
    mirrorRes[0] = currentPos - 2.f * (planeNormal * currentPos + planeD) * planeNormal;
    //v = v' -2(n * v') * n
    mirrorRes[1] = currentVel - 2.f * (planeNormal * currentVel) * planeNormal;

    return mirrorRes;
}


// === SPHERE ===
CustomSphere::CustomSphere(float radiusS, glm::vec3 centerS) 
{
    sphereRadius = radiusS;
    sphereCenter = centerS;
    sphereMovement = STOP;
}

CustomSphere::~CustomSphere() 
{

}

bool CustomSphere::CheckCollisionSphere(glm::vec3 particlePos) 
{
    glm::vec3 vecRes = CalculateVectorBetweenTwoPoints(sphereCenter, particlePos);
    float magnitudeVector = glm::length(vecRes);

    return magnitudeVector <= sphereRadius;
}

glm::vec3* CustomSphere::CalculateParticleMirror(glm::vec3 previousPos, glm::vec3 currentPos, glm::vec3 currentVel)
{
    glm::vec3 mirrorRes[2];

    glm::vec3 pointOfCollision = CalculatePointOfCollision(previousPos, currentPos);
    glm::vec3 normalVectorPlane = glm::normalize(CalculateVectorBetweenTwoPoints(pointOfCollision, sphereCenter));
 
    float planeD = CalculatePlaneD(normalVectorPlane, pointOfCollision);

    //Apply form: P = P' - 2(n * P' + D) * n
    mirrorRes[0] = currentPos - (2.f * (glm::dot(normalVectorPlane, currentPos) + planeD) * normalVectorPlane);
    
    // Apply form: V = V' - 2(n * V') * n
    mirrorRes[1] = currentVel - (2.f * (glm::dot(normalVectorPlane, currentVel)) * normalVectorPlane);

    return mirrorRes;
}

//Aixo es crida a CalculateParticleMirror()
glm::vec3 CustomSphere::CalculatePointOfCollision(glm::vec3 previousParticlePos, glm::vec3 currentParticlePos) 
{
    float λRect[2];

    float a;
    float b;
    float c;
    glm::vec3 pointOfCollision;

    glm::vec3 vecRes = CalculateVectorBetweenTwoPoints(currentParticlePos, previousParticlePos);

    //Ax + By + Cz + D = 0     
    //Pc = P + λV <--------------- Hem de trobar la lambda (ens donarà 2 res)
    // (x - Cx)^2 + (y - Cy)^2 + (z - Cz)^2 = r^2

    c = (pow(previousParticlePos.x - sphereCenter.x, 2) + 
        pow(previousParticlePos.y - sphereCenter.y, 2) + 
        pow(previousParticlePos.z - sphereCenter.z, 2)) - pow(sphereRadius, 2);

    b = (2 * glm::dot(previousParticlePos, vecRes)) + (-2 * glm::dot(sphereCenter, vecRes));
    /*b = (2 * previousParticlePos.x * vecRes.x) + (-2 * sphereCenter.x * vecRes.x) +
        (2 * previousParticlePos.y * vecRes.y) + (-2 * sphereCenter.y * vecRes.y) +
        (2 * previousParticlePos.z * vecRes.z) + (-2 * sphereCenter.z * vecRes.z);*/

    a = pow(vecRes.x, 2) +
        pow(vecRes.y, 2) +
        pow(vecRes.z, 2);

    λRect[0] = ( (-b) + sqrt(pow(b, 2.f) - (4.f * a * c))) / (2.f * a);
    λRect[1] = ( (-b) - sqrt(pow(b, 2.f) - (4.f * a * c))) / (2.f * a);


    if (λRect[0] >= λRect[1])
    {
       pointOfCollision = previousParticlePos + (λRect[1] * vecRes);
    }
    else
    {
        pointOfCollision = previousParticlePos + (λRect[0] * vecRes);
    }

    return pointOfCollision;
}

void CustomSphere::SphereMovement(bool enable)
{
    if (enable) 
    {
        switch (sphereMovement) 
        {
        case LEFT:
            sphereCenter.x -= 0.1f;
            break;

        case RIGHT:
            sphereCenter.x += 0.1f;
            break;
        }
    }
}


// === CAPSULE ===
CustomCapsule::CustomCapsule(glm::vec3 posA, glm::vec3 posB, float radius)
{
    positionA = posA;
    positionB = posB;
    capsuleRadius = radius;
}

CustomCapsule::~CustomCapsule()
{

}

glm::vec3 CustomCapsule::CalculateVectorAB()
{
    return positionB - positionA;
}

bool CustomCapsule::CheckCollisionCapsule(glm::vec3 particlePos)
{
    return glm::distance(CalculateVectorAB(), particlePos) <= capsuleRadius;
}

void CustomCapsule::CalculateCollision(glm::vec3 previousPos, glm::vec3 currentPos)
{
    glm::vec3 vecAB = CalculateVectorAB();
    glm::vec3 kPreviousPos;
    glm::vec3 kCurrentPos;
    float lambdaPrevPos;
    float lambdaCurrPos;

    // lambda = -PA / AB
    //lambdaPrevPos = -(positionA - previousPos) / vecAB;
    //lambdaCurrPos = -(positionA - currentPos) / vecAB;
    //glm::clamp(lambdaPrevPos, positionA, positionB);

    // K = A + lambda * AB
    //kPreviousPos = positionA + lambdaPrevPos * vecAB;
    //kCurrentPos = positionA + lambdaCurrPos * vecAB;

}

glm::vec3* CustomCapsule::CalculateParticleMirror(glm::vec3 previousPos, glm::vec3 currentPos, glm::vec3 currentVel)
{
    glm::vec3 mirrorRes[2];

    // TODO

    return mirrorRes;
}