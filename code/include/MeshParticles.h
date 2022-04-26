#pragma once
#include <glm\glm.hpp>
#include "Geometry.h"

class MeshParticles
{
public:
	MeshParticles(int numParticles);
	~MeshParticles();

	int GetNumberOfParticles();

	void Render();
	void SetParticlePosition(int particleId, glm::vec3 position);
	glm::vec3 GetCurrentParticlePosition(int particleId);
	void SetParticleVelocity(int particleId, glm::vec3 velocity);
	glm::vec3 GetCurrentParticleVelocity(int particleId);

	void SetPreviousParticlePosition(int particleId);
	glm::vec3 GetPreviousParticlePosition(int particleId);
	void SetPreviousParticleVelocity(int particleId);
	glm::vec3 GetPreviousParticleVelocity(int particleId);

	void SetMirrorParticlePosition(int particleId, glm::vec3 position);
	void SetMirrorParticleVelocity(int particleId, glm::vec3 velocity);
	void ResetParticleForce(int particleId);

	int numMeshParticles;

	glm::vec3* forceAcumulator;

	glm::vec3* currentPositions;
	glm::vec3* currentVelocities;

	glm::vec3* previousPositions;
	glm::vec3* previousVelocities;

	glm::vec3* startingPositions;
	glm::vec3* startingVelocities;

	bool* isStatic;
	const float particleMass = 1.f;

};

