#pragma once
#pragma once

#include "Simulator.h"
#include <glm/glm.hpp>
#include <imgui/imgui.h>

namespace Sphere {
	extern void updateSphere(glm::vec3 pos, float radius);
}
namespace Capsule {
	extern void updateCapsule(glm::vec3 posA, glm::vec3 posB, float radius);
}
namespace LilSpheres {
	extern const int maxParticles;
	extern int firstParticleIdx, particleCount;
	extern void updateParticles(int startIdx, int count, float* array_data);
}

extern bool renderSphere, renderCapsule, renderParticles;


class Gui : public Simulator {
private:
	glm::vec3 sphereCenter;
	glm::vec3 capsuleA;
	bool guiCapsuleChanged;

	enum Movement {
		LEFT,
		STOP,
		RIGHT,
	};

	int sphereMovement;  // Int perque, tot i ser Movement, ho farem servir a la GUI

public:
	Gui() {
		sphereCenter = glm::vec3(0.f, 1.f, 0.f);
		capsuleA = glm::vec3(0.f, 1.f, 0.f);
		sphereMovement = Movement::STOP;

		for (int i = 0; i < LilSpheres::maxParticles; i++) {
			float x = -5.f + (float)rand() / (RAND_MAX / 10.f);
			float y = 0.f + (float)rand() / (RAND_MAX / 10.f);
			float z = -5.f + (float)rand() / (RAND_MAX / 10.f);

			glm::vec3 pos = glm::vec3(x, y, z);
			LilSpheres::updateParticles(i, 1, &pos.x); //POC EFICIENT; PERO DEMOSTRA COM ACTUALITZAR NOMES CERTES PARTICULES
		}
	}

	void Update(float dt) {
		if (renderSphere) {
			switch (sphereMovement) {
			case Movement::LEFT:
				sphereCenter.x -= 0.3f;
				break;
			case Movement::RIGHT:
				sphereCenter.x += 0.3f;
				break;
			}
		}
	};

	void RenderUpdate() {
		Sphere::updateSphere(sphereCenter, 1.f);


		if (guiCapsuleChanged) Capsule::updateCapsule(capsuleA, glm::vec3(1.f, 1.f, 2.f), 1.f);
	}

	void RenderGui() {
		guiCapsuleChanged = false;

		ImGui::Checkbox("Show particles", &renderParticles);
		ImGui::Checkbox("Show sphere", &renderSphere);
		guiCapsuleChanged |= ImGui::Checkbox("Show capsule", &renderCapsule);

		if (renderParticles) {
			ImGui::SliderInt(
				"First particle", //label
				&LilSpheres::firstParticleIdx, // where the value exists
				0, // min
				LilSpheres::maxParticles // max
			);
			ImGui::DragInt(
				"Number of particles", //label
				&LilSpheres::particleCount, // where the value exists
				1.f, // drag speed
				0, // min
				LilSpheres::maxParticles // max
			);
		}

		if (renderSphere) {
			ImGui::InputFloat3(
				"Sphere center",
				&sphereCenter.x // pointer to an array of 3 floats
			);

			ImGui::RadioButton("Move left", &sphereMovement, Movement::LEFT);
			ImGui::RadioButton("STOP", &sphereMovement, Movement::STOP);
			ImGui::RadioButton("Move right", &sphereMovement, Movement::RIGHT);

			if (ImGui::Button("Reset sphere")) {
				sphereCenter = glm::vec3(0.f, 1.f, 0.f);
			}
		}

		if (renderCapsule) {
			guiCapsuleChanged |= ImGui::SliderFloat3(
				"Capsule A",
				&capsuleA.x,
				-5.f,
				10.f
			);
		}
	}

	~Gui() {
		renderParticles = false;
		renderSphere = false;
		renderCapsule = false;
	}
};

