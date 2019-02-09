#pragma once
#include "Arm.h"
#include <cstdlib>
#include <iostream>
#include "Motor.h"
#include <stdexcept>
#include <vector>

class Robot {
public:
	Robot(unsigned DOF);
	Arm* addArm(double originZ, double originY, double originX, int len, double mass, int radius, char rotAxis);
	Arm* addArm(int len, double mass, int radius, char rotAxis);
	void addMotor(Arm* arm, double maxVolts, double maxTorque, double p, double i, double d);
	std::vector<Arm*> getArms() const;
	std::vector<Motor*> getMotors() const;
	unsigned getDOF() const;

private:
	unsigned DOF;
	std::vector<Arm*> arms;
	std::vector<Motor*> motors;
};

Robot::Robot(unsigned DOF) : DOF(DOF) {}

Arm* Robot::addArm(double originZ, double originY, double originX, int len, double mass, int radius, char rotAxis) {
	if (arms.size() >= DOF) {
		std::cout << "Too many arms";
		std::cin.get();
		exit(2);
	}

	Point* origin = new Point(originX, originY, originZ);
	Arm* newArm = new Arm(origin, len, mass, radius, rotAxis, arms.size());
	arms.push_back(newArm);
	return newArm;
}

Arm* Robot::addArm(int len, double mass, int radius, char rotAxis) {
	if (arms.size() == 0) {
		std::cout << "No initial arm";
		std::cin.get();
		exit(2);
	}

	if (arms.size() >= DOF) {
		std::cout << "Too many arms";
		std::cin.get();
		exit(2);
	}

	Arm* preceedingArm = arms.back();
	Point* tipOfPreceedingArm = preceedingArm->getTip();

	Arm* newArm = new Arm(tipOfPreceedingArm, len, mass, radius, rotAxis, arms.size());
	arms.push_back(newArm);
	return newArm;
}

void Robot::addMotor(Arm* arm, double maxVolts, double maxTorque, double p = 1, double i = 0, double d = 0) {
	Motor* motor = new Motor(arm, maxVolts, maxTorque, p, i, d);
	motors.push_back(motor);
}

std::vector<Arm*> Robot::getArms() const {
	return arms;
}

std::vector<Motor*> Robot::getMotors() const {
	return motors;
}

unsigned Robot::getDOF() const {
	return DOF;
}
