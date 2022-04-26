#include "MeshTest.h"
#include "Spring.h"

MeshTest::MeshTest()
{
	renderCloth = true;
	
	positions = new glm::vec3[ClothMesh::numCols * ClothMesh::numRows];
	SetInitialMeshPosition();

	meshParticles = new MeshParticles(ClothMesh::numCols * ClothMesh::numRows);
	SetInitialMeshParticlePositionAndVelocity();

	structuralSpringsHorizontal = std::vector<Spring>((ClothMesh::numCols - 1) * ClothMesh::numRows);
	structuralSpringsVertical = std::vector<Spring>(ClothMesh::numCols * (ClothMesh::numRows - 1));

	bendingSpringsHorizontal = std::vector<Spring>((ClothMesh::numCols - 2) * ClothMesh::numRows);
	bendingSpringsVertical = std::vector<Spring>(ClothMesh::numCols * (ClothMesh::numRows - 2));

	shearSpringsRight = std::vector<Spring>((ClothMesh::numCols - 1) * (ClothMesh::numRows - 1));
	shearSpringsLeft = std::vector<Spring>((ClothMesh::numCols - 1) * (ClothMesh::numRows - 1));

	SetInitialPositionSprings();
}

MeshTest::~MeshTest()
{
	delete meshParticles;
	renderCloth = false;
}


int MeshTest::GetIndex(int col, int row)
{
	return col + (row * ClothMesh::numCols);
}

void MeshTest::Update()
{
	for (float row = 0; row < ClothMesh::numRows; row++)
	{
		for (float col = 0; col < ClothMesh::numCols; col++)
		{
			int indx = GetIndex(col, row);
			positions[indx] = glm::vec3(meshParticles->currentPositions[indx].x, meshParticles->currentPositions[indx].y, meshParticles->currentPositions[indx].z);
		}
	}
	ApplySpringForces();
}

void MeshTest::RenderUpdateMesh()
{
	meshParticles->Render();
	ClothMesh::updateClothMesh(&(positions[0].x));
}

void MeshTest::SetInitialMeshPosition()
{
	for (float row = 0; row < ClothMesh::numRows; row++)
	{
		for (float col = 0; col < ClothMesh::numCols; col++)
		{
			int indx = GetIndex(col, row);
			positions[indx] = glm::vec3((row - 10), 10, (col - 5));
			positions[indx] = positions[indx] / glm::vec3(3, 1, 3);
		}
	}
}

void MeshTest::SetInitialMeshParticlePositionAndVelocity()
{
	for (int i = 0; i < meshParticles->numMeshParticles; i++) 
	{
		meshParticles->currentPositions[i] = positions[i];
		meshParticles->currentVelocities[i] = glm::vec3(0,0,0);
		meshParticles->previousPositions[i] = positions[i];
		meshParticles->startingPositions[i] = positions[i];
	}
}

int MeshTest::GetNumMeshParticles()
{
	return meshParticles->numMeshParticles;
}

int MeshTest::GetNumCols()
{
	return ClothMesh::numCols;
}

int MeshTest::GetNumRows()
{
	return ClothMesh::numRows;
}

void MeshTest::SetInitialPositionSprings()
{	
	int index = 0;
	for (int row = 0; row < ClothMesh::numRows; row++) 
	{
		for (int col = 0; col < ClothMesh::numCols - 1; col++) 
		{
			// Structural Horizontal
		
			structuralSpringsHorizontal[index].firstParticleIndex = GetIndex(col, row);
			structuralSpringsHorizontal[index].secondParticleIndex = GetIndex(col + 1, row);

			structuralSpringsHorizontal[index].equilibriumDistance = GetParticleDistance(GetIndex(col, row), GetIndex(col + 1, row));

			structuralSpringsHorizontal[index].constantK = 500.f;
			structuralSpringsHorizontal[index].damping = 20.f;
			
			index++;
		}
	}	
	
	index = 0;
	for (int row = 0; row < ClothMesh::numRows - 1; row++) 
	{
		for (int col = 0; col < ClothMesh::numCols; col++) 
		{
			// Structural Vertical
			structuralSpringsVertical[index].firstParticleIndex = GetIndex(col, row);
			structuralSpringsVertical[index].secondParticleIndex = GetIndex(col, row + 1);

			structuralSpringsVertical[index].equilibriumDistance = GetParticleDistance(GetIndex(col, row), GetIndex(col, row + 1));
			structuralSpringsVertical[index].constantK = 500.f;
			structuralSpringsVertical[index].damping = 20.f;

			index++;
		}
	}	
	
	index = 0;
	for (int row = 0; row < ClothMesh::numRows; row++)
	{
		for (int col = 0; col < ClothMesh::numCols - 2; col++)
		{
			// Bending Horizontal
			bendingSpringsHorizontal[index].firstParticleIndex = GetIndex(col, row);
			bendingSpringsHorizontal[index].secondParticleIndex = GetIndex(col + 2, row);

			bendingSpringsHorizontal[index].equilibriumDistance = GetParticleDistance(GetIndex(col, row), GetIndex(col + 2, row));
			bendingSpringsHorizontal[index].constantK = 1000.f;
			bendingSpringsHorizontal[index].damping = 40.f;

			index++;
		}
	}

	index = 0;
	for (int row = 0; row < ClothMesh::numRows - 2; row++)
	{
		for (int col = 0; col < ClothMesh::numCols; col++)
		{
			// Bending Vertical
			bendingSpringsVertical[index].firstParticleIndex = GetIndex(col, row);
			bendingSpringsVertical[index].secondParticleIndex = GetIndex(col, row + 2);

			bendingSpringsVertical[index].equilibriumDistance = GetParticleDistance(GetIndex(col, row), GetIndex(col, row + 2));
			bendingSpringsVertical[index].constantK = 1000.f;
			bendingSpringsVertical[index].damping = 40.f;

			index++;
		}
	}

	index = 0;
	for (int row = 0; row < ClothMesh::numRows - 1; row++)
	{
		for (int col = 0; col < ClothMesh::numCols - 1; col++)
		{
			// Shear Right
			shearSpringsRight[index].firstParticleIndex = GetIndex(col, row);
			shearSpringsRight[index].secondParticleIndex = GetIndex(col + 1, row + 1);

			shearSpringsRight[index].equilibriumDistance = GetParticleDistance(GetIndex(col, row), GetIndex(col + 1, row + 1));
			shearSpringsRight[index].constantK = 700.f;
			shearSpringsRight[index].damping = 25.f;

			index++;
		}
	}

	index = 0;
	for (int row = 0; row < ClothMesh::numRows - 1; row++)
	{
		for (int col = 1; col < ClothMesh::numCols; col++)
		{
			// Shear Left
			shearSpringsLeft[index].firstParticleIndex = GetIndex(col, row);
			shearSpringsLeft[index].secondParticleIndex = GetIndex(col - 1, row + 1);

			shearSpringsLeft[index].equilibriumDistance = GetParticleDistance(GetIndex(col, row), GetIndex(col - 1, row + 1));
			shearSpringsLeft[index].constantK = 700.f;
			shearSpringsLeft[index].damping = 25.f;

			index++;
		}
	}
}
void MeshTest::ApplySpringForces()
{
	glm::vec3 springForce = glm::vec3(0.0f, 0.0f, 0.0f);

	// == STRUCTURAL ==
	for (int i = 0; i < structuralSpringsHorizontal.size(); i++)
	{
		springForce = structuralSpringsHorizontal[i].GetStrenghtBetweenTwoPositions(meshParticles->currentPositions[structuralSpringsHorizontal[i].firstParticleIndex],
			meshParticles->currentPositions[structuralSpringsHorizontal[i].secondParticleIndex],
			meshParticles->currentVelocities[structuralSpringsHorizontal[i].firstParticleIndex],
			meshParticles->currentVelocities[structuralSpringsHorizontal[i].secondParticleIndex]);

		SetAcceleration(structuralSpringsHorizontal[i].firstParticleIndex, springForce);
		SetAcceleration(structuralSpringsHorizontal[i].secondParticleIndex, -springForce);
	}

	for (int i = 0; i < structuralSpringsVertical.size(); i++)
	{
		springForce = structuralSpringsVertical[i].GetStrenghtBetweenTwoPositions(meshParticles->currentPositions[structuralSpringsVertical[i].firstParticleIndex],
			meshParticles->currentPositions[structuralSpringsVertical[i].secondParticleIndex],
			meshParticles->currentVelocities[structuralSpringsVertical[i].firstParticleIndex],
			meshParticles->currentVelocities[structuralSpringsVertical[i].secondParticleIndex]);

		SetAcceleration(structuralSpringsVertical[i].firstParticleIndex, springForce);
		SetAcceleration(structuralSpringsVertical[i].secondParticleIndex, -springForce);
	}

	// == BENDING ==
	for (int i = 0; i < bendingSpringsHorizontal.size(); i++)
	{
		springForce = bendingSpringsHorizontal[i].GetStrenghtBetweenTwoPositions(meshParticles->currentPositions[bendingSpringsHorizontal[i].firstParticleIndex],
			meshParticles->currentPositions[bendingSpringsHorizontal[i].secondParticleIndex],
			meshParticles->currentVelocities[bendingSpringsHorizontal[i].firstParticleIndex],
			meshParticles->currentVelocities[bendingSpringsHorizontal[i].secondParticleIndex]);

		SetAcceleration(bendingSpringsHorizontal[i].firstParticleIndex, springForce);
		SetAcceleration(bendingSpringsHorizontal[i].secondParticleIndex, -springForce);
	}

	for (int i = 0; i < bendingSpringsVertical.size(); i++)
	{
		springForce = bendingSpringsVertical[i].GetStrenghtBetweenTwoPositions(meshParticles->currentPositions[bendingSpringsVertical[i].firstParticleIndex],
			meshParticles->currentPositions[bendingSpringsVertical[i].secondParticleIndex],
			meshParticles->currentVelocities[bendingSpringsVertical[i].firstParticleIndex],
			meshParticles->currentVelocities[bendingSpringsVertical[i].secondParticleIndex]);

		SetAcceleration(bendingSpringsVertical[i].firstParticleIndex, springForce);
		SetAcceleration(bendingSpringsVertical[i].secondParticleIndex, -springForce);
	}

	// == SHEAR ==
	for (int i = 0; i < shearSpringsRight.size(); i++)
	{
		springForce = shearSpringsRight[i].GetStrenghtBetweenTwoPositions(meshParticles->currentPositions[shearSpringsRight[i].firstParticleIndex],
			meshParticles->currentPositions[shearSpringsRight[i].secondParticleIndex],
			meshParticles->currentVelocities[shearSpringsRight[i].firstParticleIndex],
			meshParticles->currentVelocities[shearSpringsRight[i].secondParticleIndex]);

		SetAcceleration(shearSpringsRight[i].firstParticleIndex, springForce);
		SetAcceleration(shearSpringsRight[i].secondParticleIndex, -springForce);
	}

	for (int i = 0; i < shearSpringsLeft.size(); i++)
	{
		springForce = shearSpringsLeft[i].GetStrenghtBetweenTwoPositions(meshParticles->currentPositions[shearSpringsLeft[i].firstParticleIndex],
			meshParticles->currentPositions[shearSpringsLeft[i].secondParticleIndex],
			meshParticles->currentVelocities[shearSpringsLeft[i].firstParticleIndex],
			meshParticles->currentVelocities[shearSpringsLeft[i].secondParticleIndex]);

		SetAcceleration(shearSpringsLeft[i].firstParticleIndex, springForce);
		SetAcceleration(shearSpringsLeft[i].secondParticleIndex, -springForce);
	}
}


void MeshTest::SetAcceleration(int index, glm::vec3 springForce)
{
	meshParticles->forceAcumulator[index] += springForce / meshParticles->particleMass;
}

float MeshTest::GetParticleDistance(int firstParticleIndx, int secondParticleIndx)
{
	return glm::distance(meshParticles->startingPositions[firstParticleIndx], meshParticles->startingPositions[secondParticleIndx]);
}

glm::vec3 MeshTest::GetParticlePosition(int particleId)
{
	return meshParticles->currentPositions[particleId];
}