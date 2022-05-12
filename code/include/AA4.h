#pragma once
#include <Simulator.h>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <vector>

namespace AA4 
{

	struct RbState 
	{
		glm::vec3 centerOfMass;

		glm::vec3 P; // Moment linear
		glm::vec3 L; // Moment Angu-L-ar

		glm::quat rotQuat;
	};


	class RigidBody 
	{
	public:
		RigidBody() : RigidBody(1.f, glm::mat3(1.f), glm::vec3(0.f), glm::vec3(0.f), glm::vec3(0.f), glm::quat()) {};
		RigidBody(float mass, glm::mat3 Ibody, glm::vec3 CoM, glm::vec3 v, glm::vec3 w, glm::quat quatRotation)
		{
			this->rbMass = mass;
			this->InverseIbody = glm::inverse(Ibody);

			state.centerOfMass = CoM;
			state.P = mass * v;
			state.L = glm::inverse(GetInverseInertiaTensor()) * w;

			// normalized quat
			state.rotQuat = quatRotation;
		};

		RbState GetState() const;
		void SetState(RbState state);
		glm::vec3 GetLinearVelocity() const;
		glm::mat3 GetInverseInertiaTensor() const;
		glm::vec3 GetAngularVelocity() const;
		glm::mat3 GetRotationMatrix() const;
		float GetMass() const;
		float GetMassInverse() const;
		glm::quat GetQuat(float angle, glm::vec3 axis) const;

		glm::vec4 GetFace(int id) const;
		glm::vec3 GetVertex(int id) const;

		bool DetectCollision(std::vector<RigidBody*> planeRb, RigidBody* cubeRb) const;

		virtual void Render() const = 0;

	protected:
		glm::mat4 GetTransformMatrix() const;

		float rbMass;
		float rbAngle;
		glm::mat3 InverseIbody;

		std::vector<glm::vec4> faces;
		std::vector<glm::vec3> vertex;

	private:
		RbState state;
		// TODO: Add other attributes
	};

	class RigidCube : public RigidBody 
	{
	public:
		RigidCube(float mass, glm::vec3 CoM, glm::vec3 v, glm::vec3 w, glm::quat qRotation) : 
			RigidBody(mass, ComputeIbody(mass, 1.f, 1.f, 1.f), CoM, v, w, qRotation) {};
		
		void Render() const;

	private:
		static glm::mat3 ComputeIbody(float mass, float depth, float width, float height);
	};

	class RigidWall : public RigidBody 
	{
	public:
		RigidWall(glm::vec3 planePoint, glm::vec3 planeN);
		void Render() const;
		float CalculatePlaneD(glm::vec3 normalVector, glm::vec3 planePoint);

		glm::vec3 GetPlaneCoM() { return planeCenterOfMass; };
		glm::vec3 GetPlaneNormal() { return planeNormal; };
		float GetPlaneD() { return planeD; };

	private:
		glm::vec3 planeCenterOfMass;
		glm::vec3 planeNormal;
		float planeD;
	};

	class RigidSphere : public RigidBody 
	{
	public:
		void Render() const;
	};

	RbState SemiImplicitEuler(std::vector<RigidBody*> rbWall, RigidBody* rbCube, float dt);

	class AA4Simulator : public Simulator 
	{
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
