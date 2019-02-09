#pragma once
#include <chrono>
#include <cstdlib>
#include "robot.h"
#include <stdexcept>
#include <vector>

#define G 9.81
#define PI 3.14159265

class PhysicsEngine {
public:
	PhysicsEngine(Robot* robot, int sampleTime, double dampingRatio);
	void main(std::vector<double> targetPositions);
	Robot* getRobot() const;

private:
	Robot* robot;
	double dampingRatio; //
	double sampleTime; //ms
	double timeSinceLastUpdate;
	std::chrono::time_point<std::chrono::system_clock> lastUpdate = std::chrono::system_clock::now();

private:
	std::vector<double> calcVoltages(std::vector<double> targetPositions);
	std::vector<double> calcTorques(std::vector<double> voltages);
	std::vector<double> calcAcceleration(std::vector<double> torques);
	std::vector<double> calcInertia();
	std::vector<double> calcLoad();
	void updateAngularVelocities(std::vector<double> acceleration);
	void updateAngularPositions();
};

PhysicsEngine::PhysicsEngine(Robot* robot, int sampleTime, double dampingRatio)
	: robot(robot) {
	if (dampingRatio <= 0) {
		std::cout << "Damping ratio must be positive";
		std::cin.get();
		exit(2);
	}

	if (sampleTime <= 0) {
		std::cout << "Sample time must be positive";
		std::cin.get();
		exit(2);
	}

	this->sampleTime = sampleTime;
	this->dampingRatio = dampingRatio;
	std::cout << std::endl << "PHYSICS ENGINE CREATED" << std::endl;
}

void PhysicsEngine::main(std::vector<double> targetPositions) {

	if (targetPositions.size() != robot->getDOF()) {
		std::cout << "Size of targetPositions vector must be equal to the number of DOF of the robot";
		std::cin.get();
		exit(2);
	}

	std::chrono::time_point<std::chrono::system_clock> currentTime = std::chrono::system_clock::now();
	timeSinceLastUpdate = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - lastUpdate).count();
	if (timeSinceLastUpdate < sampleTime) {
		return;
	}
	std::vector<double> voltages = calcVoltages(targetPositions);
	std::vector<double> torques = calcTorques(voltages);
	std::vector<double> accelerationsPerUnitTime = calcAcceleration(torques);
	updateAngularVelocities(accelerationsPerUnitTime);
	updateAngularPositions();

	lastUpdate = std::chrono::system_clock::now();
	
	std::cout << "Motor Positions --  1st axis: " << 
		robot->getMotors()[0]->getAngularPosition() << " 2nd axis: " <<
		robot->getMotors()[1]->getAngularPosition() << " 3rd axis: " <<
		robot->getMotors()[2]->getAngularPosition() << " 4th axis: " <<
		robot->getMotors()[3]->getAngularPosition() << "\r";
}

std::vector<double> PhysicsEngine::calcVoltages(std::vector<double> targetPositions) {
	std::vector<double> voltages;

	for (int i = 0; i < robot->getArms().size(); i++) {
		Arm* arm = robot->getArms()[i];
		Motor* motor = robot->getMotors()[i];

		double error = targetPositions[i] - motor->getAngularPosition();

		motor->getController()->addError(error);

		PIDController* controller = motor->getController();

		double voltage = error * controller->getP() +
						controller->getErrorSum() * controller->getI() +
						(controller->getPrevError() - error) * controller->getD();
		if (voltage > motor->getMaxVoltage())
			voltage = motor->getMaxVoltage();

		voltages.push_back(voltage);

		motor->getController()->setPrevError(error);
	}
	return voltages;
}

std::vector<double> PhysicsEngine::calcTorques(std::vector<double> voltages) {
	std::vector<double> torques;

	for (int i = 0; i < robot->getArms().size(); i++) {
		Motor* motor = robot->getMotors()[i];
		double torque = motor->calcTorque(voltages[i]);
		torques.push_back(torque);
	}
	return torques;
}

std::vector<double> PhysicsEngine::calcAcceleration(std::vector<double> torques) {
	std::vector<double> inertias = calcInertia();
	std::vector<double> loads = calcLoad();
	std::vector<double> accelerations;

	for (int i = 0; i < robot->getArms().size(); i++) {
		Motor* motor = robot->getMotors()[i];
		double acceleration = (torques[i] - loads[i] - dampingRatio * motor->getAngularVelocity()) / inertias[i];
		accelerations.push_back(acceleration / 1000); // rad/s^2
	}
	return accelerations;
}

std::vector<double> PhysicsEngine::calcInertia() {
	std::vector<double> inertias;

	for (int i = 0; i < robot->getArms().size(); i++) {
		Arm* arm = robot->getArms()[i];
		double inertia = 0;

		if (arm->getRotationAxis() == 'Z') {
			inertia += arm->getMass() * pow(arm->getRadius(), 2) / 2;

			for (int j = i + 1; j < robot->getArms().size(); j++) {
				Arm* nextArm = robot->getArms()[j];
				inertia += arm->getMass() * pow(arm->getLength(), 2) * 
							pow(cos(nextArm->getAngularPosition('Y') * PI / 180), 2) / 12;
				
				double distance = sqrt(pow(arm->getCOM().x - nextArm->getCOM().x, 2) +
									pow(arm->getCOM().y - nextArm->getCOM().y, 2) +
									pow(arm->getCOM().z - nextArm->getCOM().z, 2));
				double parallelAxis = nextArm->getMass() * pow(distance, 2);
				inertia += parallelAxis;
			}
		}
		else if (arm->getRotationAxis() == 'Y') {
			inertia += arm->getMass() * pow(arm->getLength(), 2) / 3;

			for (int j = i + 1; j < robot->getArms().size(); j++) {
				Arm* nextArm = robot->getArms()[j];

				inertia += arm->getMass() * pow(arm->getLength(), 2) / 12;

				double distance = sqrt(pow(arm->getOrigin()->x - nextArm->getCOM().x, 2) +
									pow(arm->getOrigin()->y - nextArm->getCOM().y, 2) +
									pow(arm->getOrigin()->z - nextArm->getCOM().z, 2));
				double parallelAxis = nextArm->getMass() * pow(distance, 2);
				inertia += parallelAxis;
			}
		}
		inertias.push_back(inertia);
	}
	return inertias;
}

std::vector<double> PhysicsEngine::calcLoad() {
	std::vector<double> loads(robot->getArms().size(), 0);

	for (int i = 0; i < robot->getArms().size(); i++) {
		Arm* arm = robot->getArms()[i];
		double load = 0;

		if (arm->getRotationAxis() == 'Y') {
			for (int j = i; j < robot->getArms().size(); j++) {
				double distance = sqrt(pow(arm->getOrigin()->x - robot->getArms()[j]->getCOM().x, 2) +
										pow(arm->getOrigin()->z - robot->getArms()[j]->getCOM().z, 2));
				load += distance * robot->getArms()[j]->getMass() * G;
			}
		}
		loads[i] = load;
	}
	return loads;
}

void PhysicsEngine::updateAngularVelocities(std::vector<double> acceleration) {
	for (int i = 0; i < robot->getArms().size(); i++) {
		Motor* motor = robot->getMotors()[i];
		motor->increaseAngularVelocity(acceleration[i]);
	}
}

void PhysicsEngine::updateAngularPositions() {
	for (int i = 0; i < robot->getArms().size(); i++) {
		Arm* arm = robot->getArms()[i];
		Motor* motor = robot->getMotors()[i];
		double positionUpdateAmount = motor->getAngularVelocity() / PI * 180 * (this->timeSinceLastUpdate / 1000); //degrees
		motor->increaseAngularPosition(positionUpdateAmount);
	}
}

Robot* PhysicsEngine::getRobot() const {
	return robot;
}
