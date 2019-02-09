#include "robot.h"
#include "PhysicsEngine.h"
#include <iostream>
#include <vector>
#include <GLFW/glfw3.h>

Robot* initializeRobot();
void display(GLFWwindow* window, PhysicsEngine PE);

Robot* robot = initializeRobot();
std::vector<double> targetPositions = { 90, 90, 0, 0 };
PhysicsEngine PE = PhysicsEngine(robot, 10, 0.5);

int main() {

	/* Initialize the library */
	if (!glfwInit())
		return -1;

	GLFWwindow* window = glfwCreateWindow(1024, 768, "Simulation", NULL, NULL);

	if (!window) {
		glfwTerminate();
		return -1;
	}
	/* Make the window's context current */
	glfwMakeContextCurrent(window);

	/* Loop until the user closes the window */
	display(window, PE);

	glfwTerminate();
	std::cin.get();
	return 0;
}

Robot* initializeRobot() {
	Robot* robot = new Robot(4);
	Arm* newArm;

	newArm = robot->addArm(0.0, 0.0, 0.0, 50, 0.2, 30, 'Z');
	robot->addMotor(newArm, 12, 1.16, 5, 0, 5);
	robot->getArms()[0]->setAngularPosition(90, 'Y');

	newArm = robot->addArm(200, 0.6, 30, 'Y');
	robot->addMotor(newArm, 12, 6.33, 20, 0, 0);

	newArm = robot->addArm(200, 0.6, 30, 'Y');
	robot->addMotor(newArm, 12, 4.22, 15);

	newArm = robot->addArm(100, 0.3, 30, 'Y');
	robot->addMotor(newArm, 12, 1.16, 15);

	return robot;
}

void display(GLFWwindow* window, PhysicsEngine PE) {
	while (!glfwWindowShouldClose(window)) {
		PE.main(targetPositions);

		/* Render here */
		glClear(GL_COLOR_BUFFER_BIT);
		
		glBegin(GL_LINE_STRIP);
		glVertex2f(0.00,0.00);
		glVertex2f(10.00,20.00);

		glVertex2f(50.00,100.00);
		glVertex2f(80.00, 300.00);
		glEnd();
		/*
		for (int i = 0; i < PE.getRobot()->getArms().size(); i++) {
			Arm* arm = PE.getRobot()->getArms()[i];
			glBegin(GL_LINES);
				glVertex3f(arm->getOrigin()->x, arm->getOrigin()->y, arm->getOrigin()->z);
				glVertex3f(arm->getTip()->x, arm->getTip()->y, arm->getTip()->z);
			glEnd();
		}*/
		
		/* Swap front and back buffers */
		glfwSwapBuffers(window);
		/* Poll for and process events */
		glfwPollEvents();
	}
}
