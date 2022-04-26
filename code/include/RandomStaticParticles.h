#pragma once

#include "Simulator.h";
#include "ParticleSystem.h"

class RandomStaticParticles : public Simulator {
public:
	RandomStaticParticles();
	~RandomStaticParticles();

	void Update(float dt);
	void RenderUpdate();
	void RenderGui();
private:
	float minX, minY, minZ;
	float maxX, maxY, maxZ;
	ParticleSystem* particles;
};