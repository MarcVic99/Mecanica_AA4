#include <imgui\imgui.h>
#include <imgui\imgui_impl_sdl_gl3.h>
#include <glm\glm.hpp>
#include <glm\gtc\matrix_transform.hpp>
#include <cstdio>
#include "RandomStaticParticles.h"
#include "ParticleSystem.h"


RandomStaticParticles::RandomStaticParticles() 
{
	minX = minZ = -5.f;
	maxX = maxZ = 5.f;
	minY = 0.f;
	maxY = 10.f;

	int maxParticles = 100;
	particles = new ParticleSystem(maxParticles);

	// Enable the rendering of particles in the framework 
	extern bool renderParticles; renderParticles = true;
}

void RandomStaticParticles::RenderGui() 
{
	// Create 6 sliders to select the maximum and minimum positions to spawn particles
	ImGui::SliderFloat("Min x", &minX, -5.f, 0.f);
	ImGui::SliderFloat("Max x", &maxX, 0.f, 5.f);

	ImGui::SliderFloat("Min y", &minY, 0.f, 5.f);
	ImGui::SliderFloat("Max y", &maxY, 5.f, 10.f);

	ImGui::SliderFloat("Min z", &minZ, -5.f, 0.f);
	ImGui::SliderFloat("Max z", &maxZ, 0.f, 5.f);
}

void RandomStaticParticles::Update(float dt) 
{
	for (int i = 0; i < particles->GetNumberOfParticles(); i++) 
	{
		float x = minX + (float)rand() / (RAND_MAX / (maxX - minX));
		float y = minY + (float)rand() / (RAND_MAX / (maxY - minY));
		float z = minZ + (float)rand() / (RAND_MAX / (maxZ - minZ));

		printf("Creating particle with position %.2f, %.2f, %.2f\n", x, y, z);
		particles->SetParticlePosition(i, glm::vec3(x, y, z));
	}
}

void RandomStaticParticles::RenderUpdate() 
{
	particles->Render();
}

RandomStaticParticles::~RandomStaticParticles()
{
	printf("Destruct the random static particles\n");
	delete particles;
}

