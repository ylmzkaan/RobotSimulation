#pragma once
#include <iostream>
#include <vector>

class PIDController {
private:
	double P;
	double I;
	double D;
	double prevError = 0;
	double errorSum = 0;
	double errorAccumulationLimit;
	std::vector<double> errors;

public:
	PIDController(double p, double i, double d, int errorAccumulationLimit);
	void setP(double value);
	void setI(double value);
	void setD(double value);
	void addError(double value);
	void setPrevError(double value);
	double getP() const;
	double getI() const;
	double getD() const;
	double getPrevError() const;
	double getErrorSum() const;
};

PIDController::PIDController(double p, double i, double d, int errorAccumulationLimit=10) {
	P = p;
	I = i;
	D = d;
	std::cout << "New PID controller created" << std::endl;
	std::cout << "\tP: " << P << " I: " << I << " D: " << D << std::endl;
}

void PIDController::setP(double value) {
	P = value;
}

void PIDController::setI(double value) {
	I = value;
}

void PIDController::setD(double value) {
	D = value;
}

void PIDController::addError(double value) {
	errors.push_back(value);
	errorSum += value;
	if (errors.size() > errorAccumulationLimit) {
		errorSum -= *(errors.begin());
		errors.erase(errors.begin());
	}
}
void PIDController::setPrevError(double value) {
	prevError = value;
}

double PIDController::getP() const {
	return P;
}

double PIDController::getI() const {
	return I;
}

double PIDController::getD() const {
	return D;
}

double PIDController::getPrevError() const {
	return prevError;
}

double PIDController::getErrorSum() const {
	return errorSum;
}