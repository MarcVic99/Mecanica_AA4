#include "EulerIntegrator.h"

EulerIntegrator::EulerIntegrator()
{
}

EulerIntegrator::~EulerIntegrator()
{
}

void EulerIntegrator::Step(ParticleSystem* ps, float dt)
{
	glm::vec3 previousPosition;
	glm::vec3 currentPosition;
	glm::vec3 previousVelocity;
	glm::vec3 currentVelocity;
	glm::vec3 acceleration = { 0.0f, -9.81f, 0.0f };

	// Logic to make particles fall down
	for (int i = 0; i < ps->GetNumberOfParticles(); i++)
	{
		// Update previous position & velocity
		previousPosition = ps->GetCurrentParticlePosition(i);
		previousVelocity = ps->GetCurrentParticleVelocity(i);

		// Update position
		currentPosition[0] = previousPosition[0] + dt * previousVelocity[0];
		currentPosition[1] = previousPosition[1] + dt * previousVelocity[1];
		currentPosition[2] = previousPosition[2] + dt * previousVelocity[2];

		// Update velocity
		currentVelocity[0] = previousVelocity[0] + dt * acceleration[0];
		currentVelocity[1] = previousVelocity[1] + dt * acceleration[1];
		currentVelocity[2] = previousVelocity[2] + dt * acceleration[2];

		// Update the paricle's position
		ps->SetParticlePosition(i, currentPosition);
		ps->SetParticleVelocity(i, currentVelocity);
	}
}
