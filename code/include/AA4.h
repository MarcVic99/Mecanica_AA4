#pragma once
#include <Simulator.h>
#include <glm/glm.hpp>
#include <vector>

namespace AA4 {

	struct RbState {
		// TODO
		glm::vec3 centerOfMass;
		glm::vec3 linearMomentum;
		glm::mat3 angularMomentum;
	};


	class RigidBody {
	public:
		RigidBody(float mass) : rbMass(mass) {};
		RbState GetState() const;
		void SetState(RbState state);
		glm::vec3 GetLinearVelocity() const;
		glm::mat3 GetInverseInertiaTensor() const;
		glm::vec3 GetAngularVelocity() const;
		glm::mat3 GetRotationMatrix() const;
		float GetMassInverse() const;

		virtual void Render() const = 0;
	protected:
		glm::mat4 GetTransformMatrix() const;
		float rbMass;
		float rbAngle;
	private:
		RbState state;
		// TODO: Add other attributes
	};

	class RigidCube : public RigidBody {
	public:
		RigidCube(float mass) : RigidBody(mass) {};
		void Render() const;
	};

	class RigidWall : public RigidBody {
	public:
		void Render() const;
	};

	class RigidSphere : public RigidBody {
	public:
		void Render() const;
	};

	RbState SemiImplicitEuler(const RigidBody* rb, float dt);

	class AA4Simulator : public Simulator {
	public:
		AA4Simulator();
		void Update(float dt);
		void RenderUpdate();
		void RenderGui();

		~AA4Simulator();
	private:
		RigidBody* simulatedObject;
		std::vector<RigidBody*> obstacles;  // Populate with walls
	};
}
