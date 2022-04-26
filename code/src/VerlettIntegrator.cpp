#include "VerlettIntegrator.h"

VerlettIntegrator::VerlettIntegrator()
{
}

VerlettIntegrator::~VerlettIntegrator()
{
}


void VerlettIntegrator::StepVerlett(MeshTest* mesh, float dt)
{
	int currParticle = 0;

	//Spring spring; // Test spring value (MUST BE DELETED & CHANGED FOR ARRAY)
	
	glm::vec3 previousPosition;
	glm::vec3 currentPosition;
	glm::vec3 nextPosition;

	glm::vec3 currentVelocity;

	for (int i = 0; i < mesh->GetNumRows(); i++)
	{
		for (int j = 0; j < mesh->GetNumCols(); j++)
		{
			previousPosition = mesh->meshParticles->GetPreviousParticlePosition(currParticle);
			currentPosition = mesh->meshParticles->GetCurrentParticlePosition(currParticle);
			
			if (!mesh->meshParticles->isStatic[currParticle])
			{
				//Xt+1 = Xt + (Xt - Xt-1) + f/m * dt^2
				nextPosition = currentPosition + (currentPosition - previousPosition) +
					mesh->meshParticles->forceAcumulator[currParticle] * (dt * dt);

				// Set values
				mesh->meshParticles->SetParticlePosition(currParticle, nextPosition);

				// Reset values
				mesh->meshParticles->ResetParticleForce(currParticle);
			}
			currParticle++;
		}
	}
	currParticle = 0;
}

glm::vec3 VerlettIntegrator::GetNewPosition(glm::vec3 currPos, glm::vec3 lastPos, glm::vec3 accel, float dt)
{
	return currPos + (currPos - lastPos) + accel * (dt * dt);
}

glm::vec3 VerlettIntegrator::GetNewVelocity(glm::vec3 newPos, glm::vec3 currPos, float dt)
{
	return (newPos - currPos) / dt;
}