#pragma once
#include <glm\glm.hpp>
#include "MeshTest.h"

class VerlettIntegrator
{
public:
	VerlettIntegrator();
	~VerlettIntegrator();

	void StepVerlett(MeshTest* mesh, float dt);

	glm::vec3 GetAcceleration(glm::vec3 forceAcum, float mass);
	glm::vec3 GetNewPosition(glm::vec3 currPos, glm::vec3 lastPos, glm::vec3 accel, float dt);
	glm::vec3 GetNewVelocity(glm::vec3 currPos, glm::vec3 lastPos, float dt);

private:
	VerlettIntegrator* verlettIntegrator;
};

