#include "ParticleSystem.h"
#include <random>
#include <algorithm>

glm::vec3 CalculateVectorBetweenTwoPointsP(glm::vec3 firstP, glm::vec3 secondP) {
	return firstP - secondP;
}

glm::vec3 CalculatePerpendicularVectorP(glm::vec3 vector, float Px, float Py)
{
	//vector · newVector = 0
	 //vx * nx + vy * ny + vz * nz = 0
	glm::vec3 newVector;
	float Pz;

	Pz = ((vector.x * Px) + (vector.y * Py)) / vector.z;

	return newVector = glm::vec3(Px, Py, Pz);
}
/////////Forward declarations
namespace LilSpheres
{
	extern const int maxParticles;
	extern int firstParticleIdx;
	extern int particleCount;
	extern void updateParticles(int startIdx, int count, float* array_data);
}

glm::vec3 GetParticleInitialPositionAA2(int id, int numParticles)
{
	float margin = 0.1f;
	float available_length = 2 * (5.f - margin);
	float offset = available_length / (numParticles - 1);

	float x, y, z;
	x = z = -5.f + margin + id * offset;
	y = 9.9f;

	return glm::vec3(x, y, z);
}

ParticleSystem::ParticleSystem(int numParticles) : maxParticles(numParticles)
{
	particleMode = CASCADE;

	currentNumParticles = maxParticles;
	currentPositions = new glm::vec3[maxParticles];
	currentVelocities = new glm::vec3[maxParticles];

	previousPositions = new glm::vec3[maxParticles];
	previousVelocities = new glm::vec3[maxParticles];

	startingPositions = new glm::vec3[maxParticles];
	startingVelocities = new glm::vec3[maxParticles];

	currentLifespan = new float[maxParticles];
	maxParticleLifetime = 80.f;

	cascadeStartingPoint = glm::vec3(-4.f, 4.f, -4.f);
	cascadeEndingPoint = glm::vec3(-4.f, 4.f, 2.f);

	cascadeRotationAngle = 45.f;
	cascadeStartingVelocity = glm::vec3 (0.f, 5.f, 5.f);

	fountainPosition = glm::vec3(0.f, 4.f, 0.f);
	delayTime = new float [maxParticles];
	delayRange = 30;
	fountainRange = 300;

	for (int i = 0; i < maxParticles; i++)
	{
		currentPositions[i] = glm::vec3(0.f, 0.f, 0.f);
		currentVelocities[i] = glm::vec3(0.f, 0.f, 0.f);

		previousPositions[i] = glm::vec3(0.f, 0.f, 0.f);
		previousVelocities[i] = glm::vec3(0.f, 0.f, 0.f);

		startingPositions[i] = currentPositions[i];
		startingVelocities[i] = currentVelocities[i];

		currentLifespan[i] = 0.f;
		delayTime[i] = rand() % delayRange;
	}
};

ParticleSystem::~ParticleSystem()
{
	printf("Destruct the particle system\n");
	delete currentPositions;
	delete currentVelocities;
	delete previousPositions;
	delete previousVelocities;
	delete startingPositions;
	delete startingVelocities;
	delete currentLifespan;
	delete delayTime;
};

int ParticleSystem::GetNumberOfParticles()
{
	return currentNumParticles;
};

void ParticleSystem::Render()
{
	LilSpheres::firstParticleIdx = 0;
	LilSpheres::particleCount = GetNumberOfParticles();

	LilSpheres::updateParticles(0, GetNumberOfParticles(), &(currentPositions[0].x));
}

void ParticleSystem::SetParticlePosition(int particleId, glm::vec3 position)
{
	SetPreviousParticlePosition(particleId);
	currentPositions[particleId] = position;
}

glm::vec3 ParticleSystem::GetCurrentParticlePosition(int particleId)
{
	return currentPositions[particleId];
}

void ParticleSystem::SetParticleVelocity(int particleId, glm::vec3 velocity)
{
	SetPreviousParticleVelocity(particleId);
	currentVelocities[particleId] = velocity;
}

glm::vec3 ParticleSystem::GetCurrentParticleVelocity(int particleId)
{
	return currentVelocities[particleId];
}

void ParticleSystem::SetPreviousParticlePosition(int particleId)
{
	previousPositions[particleId] = GetCurrentParticlePosition(particleId);
}

glm::vec3 ParticleSystem::GetPreviousParticlePosition(int particleId)
{
	return previousPositions[particleId];
}

void ParticleSystem::SetPreviousParticleVelocity(int particleId)
{
	previousVelocities[particleId] = GetCurrentParticleVelocity(particleId);
}

glm::vec3 ParticleSystem::GetPreviousParticleVelocity(int particleId)
{
	return previousVelocities[particleId];
}

void ParticleSystem::SetMirrorParticlePosition(int particleId, glm::vec3 position)
{
	currentPositions[particleId] = position;
}

void ParticleSystem::SetMirrorParticleVelocity(int particleId, glm::vec3 velocity)
{
	currentVelocities[particleId] = velocity;
}

float ParticleSystem::GetCurrentLifespan(int particleId)
{
	return currentLifespan[particleId];
}

bool ParticleSystem::CheckParticleLifespan(int particleId)
{
	return currentLifespan[particleId] >= maxParticleLifetime;
}

void ParticleSystem::IncrementCurrentLifespan(int particleId)
{
	++currentLifespan[particleId];
}

void ParticleSystem::DecrementDelayTime(int particleId)
{
	--delayTime[particleId];
}

bool ParticleSystem::CheckParticleDelay(int particleId)
{
	return delayTime[particleId] <= 0;
}

void ParticleSystem::SetMaxLifetime(int newVal)
{
	maxParticleLifetime = newVal;
}

void ParticleSystem::SetCascadePoints(glm::vec3 startingP, glm::vec3 endingP, float cascadeAngle)
{
	cascadeStartingPoint = startingP;
	cascadeEndingPoint = endingP;
	cascadeRotationAngle = cascadeAngle;
}

void ParticleSystem::ResetParticle(int particleId)
{
	SetParticlePosition(particleId, GetParticleInitialPositionAA2(particleId, currentNumParticles));
	SetParticleVelocity(particleId, startingVelocities[particleId]);
	currentLifespan[particleId] = 0.f;
}

void ParticleSystem::ResetParticleCascade(int particleId)
{
	CascadeMode(particleId);
	currentLifespan[particleId] = 0.f;
	delayTime[particleId] = rand() % delayRange;
}

void ParticleSystem::ResetParticleFountain(int particleId)
{
	FountainMode(particleId);
	currentLifespan[particleId] = 0.f;
	delayTime[particleId] = rand() % delayRange;
}

void ParticleSystem::SetNumParticles(int newVal)
{
	currentNumParticles = newVal;
}

void ParticleSystem::CascadeMode(int particleId)
{
	glm::vec3 ABVector = CalculateVectorBetweenTwoPointsP(cascadeEndingPoint, cascadeStartingPoint);

	int randomPositionPerCent = rand() % 100;

	currentPositions[particleId].x = ((ABVector.x * randomPositionPerCent) / 100) + cascadeStartingPoint.x;
	currentPositions[particleId].y = ((ABVector.y * randomPositionPerCent) / 100) + cascadeStartingPoint.y;
	currentPositions[particleId].z = ((ABVector.z * randomPositionPerCent) / 100) + cascadeStartingPoint.z;

	glm::vec3 vectorU = glm::normalize(CalculatePerpendicularVectorP(ABVector, currentPositions[particleId].x, currentPositions[particleId].y));
	glm::vec3 vectorPerpendicularToUAndAB = glm::normalize(glm::cross(ABVector, vectorU));

	currentVelocities[particleId].x = glm::cos(glm::radians(cascadeRotationAngle)) * glm::length(vectorU) * 4;
		currentVelocities[particleId].y = glm::sin(glm::radians(cascadeRotationAngle)) * glm::length(vectorPerpendicularToUAndAB) * 4;
		currentVelocities[particleId].z = 0;

	SetStartingValues();
}

void ParticleSystem::SetStartingValues()
{
	startingPositions = currentPositions;
	startingVelocities = currentVelocities;
}

void ParticleSystem::FountainMode(int particleId) 
{
	currentPositions[particleId].x = fountainPosition.x; 
	currentPositions[particleId].y = fountainPosition.y; 
	currentPositions[particleId].z = fountainPosition.z; 

	int randomRangeX = rand() % 100 + (-50);
	int randomRangeZ = rand() % 100 + (-50);

	glm::vec3 fountainPointToGo = glm::vec3(randomRangeX, fountainPosition.y + 1, randomRangeZ);

	glm::vec3 velocityVector = glm::normalize(CalculateVectorBetweenTwoPointsP(fountainPointToGo, fountainPosition));


	currentVelocities[particleId].x = velocityVector.x * 2;
	currentVelocities[particleId].y = velocityVector.y * fountainRange;
	currentVelocities[particleId].z = velocityVector.z * 2;

	SetStartingValues();
}

void ParticleSystem::SetFountainValues(glm::vec3 fountainPos, int dispersion)
{
	fountainPosition = fountainPos;
	fountainRange = dispersion;
}
