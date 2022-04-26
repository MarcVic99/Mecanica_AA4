#include <glm\glm.hpp>
#include "TeleportingParticles.h";
#include "EulerIntegrator.h";

// Auxiliar methods
#pragma region auxiliar
glm::vec3 GetParticleInitialPosition(int id, int numParticles)
{
	float margin = 0.1f;
	float available_length = 2 * (5.f - margin);
	float offset = available_length / (numParticles - 1);

	float x, y, z;
	x = z = -5.f + margin + id * offset;
	y = 9.9f;

	return glm::vec3(x, y, z);
}

bool CheckHasTravessedFloor(glm::vec3 particle)
{
	// This should be done with:
	// (n * p + d) * (n * p' + d) * n
	if (particle[1] <= 0.0f)
	{
		return true;
	}

	return false;
}

#pragma endregion

#pragma region class
TeleportingParticles::TeleportingParticles()
{
	numParticles = 40;
	particles = new ParticleSystem(numParticles);

	for (int i = 0; i < numParticles; i++)
	{
		particles->SetParticlePosition(i, GetParticleInitialPosition(i, numParticles));
	}

	// Enable the rendering of particles in the framework 
	extern bool renderParticles; renderParticles = true;
}

TeleportingParticles::~TeleportingParticles()
{
	delete particles;
}

void TeleportingParticles::Update(float dt)
{
	EulerIntegrator eulerInt;

	eulerInt.Step(particles, dt);

	// Check if a particle travessed the floor plane. Restart its position if it had
	for (int i = 0; i < numParticles; i++)
	{
		if (CheckHasTravessedFloor(particles->GetCurrentParticlePosition(i)))
		{
			particles->SetParticlePosition(i, GetParticleInitialPosition(i, numParticles));
		}
	}
}

void TeleportingParticles::RenderUpdate()
{
	particles->Render();
}

void TeleportingParticles::RenderGui() {};

#pragma endregion