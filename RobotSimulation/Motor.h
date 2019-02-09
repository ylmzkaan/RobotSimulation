#pragma once
#include "Arm.h"
#include <cstdlib>
#include <iostream>
#include "PIDController.h"
#include <stdexcept>

class Motor {
private:
	double torqueConstant = 0;
	double maxVolts = 0; //V
	double maxTorque = 0; //Nm
	PIDController* controller;
	Arm* _arm;
	double angularPosition; //degrees
	double angularVelocity; //degrees

public:
	Motor(Arm* arm, double maxVolts, double maxTorque, double p, double i, double d);
	void setAngularPosition(double value);
	void increaseAngularPosition(double value);
	double getAngularPosition();
	void increaseAngularVelocity(double value);
	double getAngularVelocity();
	double calcTorque(double volts) const;
	PIDController* getController() const;
	double getMaxVoltage() const;
};

Motor::Motor(Arm* arm, double maxVolts, double maxTorque, double p, double i, double d)
	: _arm(arm) {
	
	if (maxTorque <= 0) {
		std::cout << "maxTorque must be positive";
		std::cin.get();
		exit(2);
	}

	if (maxVolts <= 0) {
		std::cout << "maxVolts must be positive";
		std::cin.get();
		exit(2);
	}
	
	this->maxVolts = maxVolts;
	this->maxTorque = maxTorque;
	controller = new PIDController(p, i, d);
	torqueConstant = maxTorque / maxVolts;

	angularPosition = 0.0; //degrees
	angularVelocity = 0.0; //degrees

	char rotAxis = _arm->getRotationAxis();
	_arm->setAngularPosition(angularPosition, rotAxis);

	std::cout << "New motor created" << std::endl;
	std::cout << "\tTorque constant: " << torqueConstant << std::endl;
}

void Motor::setAngularPosition(double value) {
	angularPosition = std::fmod(value, 360);
	char rotAxis = _arm->getRotationAxis();
	_arm->setAngularPosition(value, rotAxis);
}

void Motor::increaseAngularPosition(double value) {
	angularPosition = std::fmod(angularPosition + value, 360);	
	char rotAxis = _arm->getRotationAxis();
	_arm->setAngularPosition(angularPosition, rotAxis);
}

double Motor::getAngularPosition() {
	return angularPosition;
}

void Motor::increaseAngularVelocity(double value) {
	angularVelocity += value;
}

double Motor::getAngularVelocity() {
	return angularVelocity;
}

double Motor::calcTorque(double volts) const {
	if (volts > maxVolts)
		volts = maxVolts;
	return volts * torqueConstant * 1000; //Nmm
}

PIDController* Motor::getController() const {
	return controller;
}

double Motor::getMaxVoltage() const {
	return maxVolts;
}