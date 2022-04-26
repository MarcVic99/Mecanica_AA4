#pragma once
#include <glm/glm.hpp>
#include "Simulator.h"
#include "Spring.h"
#include "MeshParticles.h"
#include <vector>

extern bool renderCloth;

namespace ClothMesh
{
	extern void updateClothMesh(float* aray_data);
	extern const int numCols;
	extern const int numRows;
}

class MeshTest
{
public:
	MeshTest();
	~MeshTest();

	int GetIndex(int col, int row);

	void RenderUpdateMesh();

	void Update();

	void SetInitialMeshPosition();
	void SetInitialMeshParticlePositionAndVelocity();

	int GetNumMeshParticles();
	int GetNumCols();
	int GetNumRows();

	void ApplySpringForces();

	void SetInitialPositionSprings();

	void SetAcceleration(int index, glm::vec3 springForce);

	float GetParticleDistance(int firstParticleIndx, int secondParticleIndx);
	glm::vec3 GetParticlePosition(int particleId);

	glm::vec3* positions;
	glm::vec3* forceAcumulator;

	MeshParticles* meshParticles;
	
	std::vector<Spring> structuralSpringsHorizontal;
	std::vector<Spring> structuralSpringsVertical;
	
	std::vector<Spring> bendingSpringsHorizontal;
	std::vector<Spring> bendingSpringsVertical;

	std::vector<Spring> shearSpringsRight; // ( \ )
	std::vector<Spring> shearSpringsLeft; // ( / )
};

