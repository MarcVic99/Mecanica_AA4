#pragma once
#pragma once

#include "Simulator.h";
#include "ParticleSystem.h"

class TeleportingParticles : public Simulator 
{
public:
	TeleportingParticles();
	~TeleportingParticles();

	void Update(float dt);
	void RenderUpdate();
	void RenderGui();

private:
	ParticleSystem* particles;
	int numParticles;
};