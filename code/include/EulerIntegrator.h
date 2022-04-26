#pragma once
#include <glm\glm.hpp>
#include "ParticleSystem.h"

class EulerIntegrator
{
public:
	EulerIntegrator();
	~EulerIntegrator();

	static void Step(ParticleSystem* ps, float dt);

private:
	EulerIntegrator* eulerIntegrator;
};

