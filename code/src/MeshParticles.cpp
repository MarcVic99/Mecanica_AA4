#include "MeshParticles.h"
#include <random>
#include <algorithm>


/////////Forward declarations
namespace LilSpheres
{
	extern const int maxParticles;
	extern int firstParticleIdx;
	extern int particleCount;
	extern void updateParticles(int startIdx, int count, float* array_data);
}

glm::vec3 GetParticleInitialPositionAA3(int id, int numParticles)
{
	float margin = 0.1f;
	float available_length = 2 * (5.f - margin);
	float offset = available_length / (numParticles - 1);

	float x, y, z;
	x = z = -5.f + margin + id * offset;
	y = 9.9f;

	return glm::vec3(x, y, z);
}

MeshParticles::MeshParticles(int numParticles)
{
	currentPositions = new glm::vec3[numParticles];
	currentVelocities = new glm::vec3[numParticles];

	previousPositions = new glm::vec3[numParticles];
	previousVelocities = new glm::vec3[numParticles];

	startingPositions = new glm::vec3[numParticles];
	startingVelocities = new glm::vec3[numParticles];

	forceAcumulator = new glm::vec3[numParticles];

	isStatic = new bool[numParticles];

	numMeshParticles = numParticles;

	for (int i = 0; i < numParticles; i++)
	{
		if (i == 0 || i == 13)
		{
			isStatic[i] = true;
		}
		else {
			isStatic[i] = false;
		}
		/*currentPositions[i] = glm::vec3(0.f, 0.f, 0.f);
		currentVelocities[i] = glm::vec3(0.f, 0.f, 0.f);

		previousPositions[i] = glm::vec3(0.f, 0.f, 0.f);
		previousVelocities[i] = glm::vec3(0.f, 0.f, 0.f);
		*/
		startingPositions[i] = currentPositions[i];
		startingVelocities[i] = currentVelocities[i];

		forceAcumulator[i] = glm::vec3(0.f, 0.f, 0.f);
	}
};

MeshParticles::~MeshParticles()
{
	printf("Destruct the particle system\n");
	delete currentPositions;
	delete currentVelocities;
	delete previousPositions;
	delete previousVelocities;
	delete startingPositions;
	delete startingVelocities;

};

int MeshParticles::GetNumberOfParticles()
{
	return numMeshParticles;
};

void MeshParticles::Render()
{
	LilSpheres::firstParticleIdx = 0;
	LilSpheres::particleCount = GetNumberOfParticles();

	LilSpheres::updateParticles(0, GetNumberOfParticles(), &(currentPositions[0].x));
}

void MeshParticles::SetParticlePosition(int particleId, glm::vec3 position)
{
	SetPreviousParticlePosition(particleId);
	currentPositions[particleId] = position;
}

glm::vec3 MeshParticles::GetCurrentParticlePosition(int particleId)
{
	return currentPositions[particleId];
}

void MeshParticles::SetParticleVelocity(int particleId, glm::vec3 velocity)
{
	SetPreviousParticleVelocity(particleId);
	currentVelocities[particleId] = velocity;
}

glm::vec3 MeshParticles::GetCurrentParticleVelocity(int particleId)
{
	return currentVelocities[particleId];
}

void MeshParticles::SetPreviousParticlePosition(int particleId)
{
	previousPositions[particleId] = GetCurrentParticlePosition(particleId);
}

glm::vec3 MeshParticles::GetPreviousParticlePosition(int particleId)
{
	return previousPositions[particleId];
}

void MeshParticles::SetPreviousParticleVelocity(int particleId)
{
	previousVelocities[particleId] = GetCurrentParticleVelocity(particleId);
}

glm::vec3 MeshParticles::GetPreviousParticleVelocity(int particleId)
{
	return previousVelocities[particleId];
}

void MeshParticles::SetMirrorParticlePosition(int particleId, glm::vec3 position)
{
	currentPositions[particleId] = position;
}

void MeshParticles::SetMirrorParticleVelocity(int particleId, glm::vec3 velocity)
{
	currentVelocities[particleId] = velocity / glm::vec3 (2,2,2);
}

void MeshParticles::ResetParticleForce(int particleId)
{
	forceAcumulator[particleId] = glm::vec3(0.f, -9.8f, 0.f);
}