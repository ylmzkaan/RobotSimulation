#pragma once
#include <cstdlib>
#include <iostream>
#include <math.h>
#include <stdexcept>

#define PI 3.14159265

struct Point {
	double x = 0;
	double y = 0;
	double z = 0;
	Point() : x(0), y(0), z(0) {}
	Point(double x, double y, double z) : x(x), y(y), z(z) {}
};

class Arm {
public:
	Arm(Point* origin, int len, double mass, int radius, char rotAxis, int id);
	Point* getOrigin() const;
	Point* getTip() const;
	Point getCOM() const;
	char getRotationAxis() const;
	int getLength() const;
	double getMass() const;
	int getRadius() const;
	int getId() const;
	void updateTip(double angularPosition, char axis);
	void updateCOM();
	double getAngularPosition(char axis);
	void setAngularPosition(double value, char axis);
	void increaseAngularPosition(double value, char axis);

private:
	int armId;
	Point* origin; //mm
	Point* tip; //mm
	Point COM; //mm
	char rotationAxis;
	int length; //mm
	double mass; //kg
	int radius; //mm
	double angularPositionY;
	double angularPositionZ;
};

Arm::Arm(Point* origin, int len, double _mass, int _radius, char rotAxis, int id)
	: origin(origin), armId(id) {

	if (rotAxis != 'Y' && rotAxis != 'Z') {
		std::cout << "Arm objects with rotations axes other than Y and Z haven't implemented yet!";
		std::cin.get();
		exit(2);
	}

	if (_mass <= 0.0) {
		std::cout << "Mass of an arm must be positive";
		std::cin.get();
		exit(2);
	}
	
	if (len <= 0) {
		std::cout << "Length of an arm must be positive";
		std::cin.get();
		exit(2);
	}

	if (_radius <= 0) {
		std::cout << "Radius of an arm must be positive";
		std::cin.get();
		exit(2);
	}

	length = len;
	radius = _radius;
	mass = _mass;
	rotationAxis = rotAxis;
	tip = new Point();
	updateTip(0.0, rotAxis);

	std::cout << "New arm created. Id: " << armId << std::endl;
	std::cout << "\tOrigin X: " << origin->x << " Y: " << origin->y << " Z: " << origin->z << std::endl;
	std::cout << "\tTip X: " << tip->x << " Y: " << tip->y << " Z: " << tip->z << std::endl;
	std::cout << "\tRotation axis: " << rotationAxis << std::endl;
}

void Arm::updateTip(double angularPosition, char axis) {
	if (axis != 'Y' && axis != 'Z') {
		std::cout << "Arms can't rotate in axes other than Y and Z";
		std::cin.get();
		exit(2);
	}

	if (axis == 'Y') {
		tip->x = origin->x + length * cos(angularPosition * PI / 180);
		tip->z = origin->z + length * sin(angularPosition * PI / 180);
	}
	else if (axis == 'Z') {
		tip->x = origin->x + length * cos(angularPosition * PI / 180);
		tip->y = origin->y + length * sin(angularPosition * PI / 180);
	}
	updateCOM();
}

void Arm::updateCOM() {
	COM.x = (tip->x - origin->x) / 2 + origin->x;
	COM.y = (tip->y - origin->y) / 2 + origin->y;
	COM.z = (tip->z - origin->z) / 2 + origin->z;
}

Point* Arm::getOrigin() const {
	return origin;
}

Point* Arm::getTip() const {
	return tip;
}

Point Arm::getCOM() const {
	return COM;
}

char Arm::getRotationAxis() const {
	return rotationAxis;
}

int Arm::getLength() const {
	return length;
}

double Arm::getMass() const {
	return mass;
}

int Arm::getRadius() const {
	return radius;
}

int Arm::getId() const {
	return armId;
}

double Arm::getAngularPosition(char axis) {
	if (axis != 'Y' && axis != 'Z') {
		std::cout << "Arms can only have a non-trivial angular position in axes Y and Z";
		std::cin.get();
		exit(2);
	}

	if (axis == 'Y')
		return angularPositionY;
	else if (axis == 'Z')
		return angularPositionZ;
}

void Arm::setAngularPosition(double value, char axis) {
	if (axis != 'Y' && axis != 'Z') {
		std::cout << "Arms can't rotate in axes other than Y and Z";
		std::cin.get();
		exit(2);
	}

	value = std::fmod(value, 360);
	if (axis == 'Y')
		angularPositionY = value;
	else if (axis == 'Z')
		angularPositionZ = value;
	updateTip(value, axis);
}

void Arm::increaseAngularPosition(double value, char axis) {
	if (axis != 'Y' && axis != 'Z') {
		std::cout << "Arms can't rotate in axes other than Y and Z";
		std::cin.get();
		exit(2);
	}

	if (axis == 'Y') {
		angularPositionY = std::fmod(angularPositionY + value, 360);
		updateTip(angularPositionY, axis);
	} 
	else if (axis == 'Z') {
		angularPositionZ = std::fmod(angularPositionY + value, 360);
		updateTip(angularPositionZ, axis);
	}
}