/***********************************************************************/
/**                                                                    */
/** test.cpp                                                           */
/**                                                                    */
/** Copyright (c) 2016, Service Robotics Lab.                          */ 
/**                     http://robotics.upo.es                         */
/**                                                                    */
/** All rights reserved.                                               */
/**                                                                    */
/** Authors:                                                           */
/** Ignacio Perez-Hurtado (maintainer)                                 */
/** Jesus Capitan                                                      */
/** Fernando Caballero                                                 */
/** Luis Merino                                                        */
/**                                                                    */
/** Version 1.0:                                                       */
/** Author:                                                            */
/** Noé Pérez-Higueras                                                 */
/**                                                                    */   
/** This software may be modified and distributed under the terms      */
/** of the BSD license. See the LICENSE file for details.              */
/**                                                                    */
/** http://www.opensource.org/licenses/BSD-3-Clause                    */
/**                                                                    */
/***********************************************************************/

#include <iostream>
#include <lightsfm/sfm.hpp>
#include <lightsfm/hsfm.hpp>

int main()
{
	hsfm::HSFM;
	std::cout<<"TEST"<<std::endl;
	
	// Initial params
	utils::Vector2d initialPosition1(0,0);
	utils::Vector2d initialPosition2(2,2);
	utils::Vector2d initialPosition3(4,4);
	utils::Angle initialYaw(0);
	double initialLinVel = 0;
	double initialAngVel = 0;
	double dt = 0.05;

	// Test agent
	hsfm::Agent agent1(initialPosition1, initialYaw, initialLinVel, initialAngVel);
	hsfm::Agent agent2(initialPosition2, initialYaw, initialLinVel, initialAngVel);
	hsfm::Agent agent3(initialPosition3, initialYaw, initialLinVel, initialAngVel);

	// Set goals
	hsfm::Goal goal1;
	goal1.center.set(3,3);
	goal1.radius = 0.3;
	hsfm::Goal goal2;
	goal1.center.set(5,5);
	goal1.radius = 0.3;
	std::list<hsfm::Goal> goals = {goal1, goal2};
	agent1.goals = goals;
	agent2.goals = goals;
	agent3.goals = goals;

	std::vector<hsfm::Agent> other_agents = {agent2, agent3};
	//TBD: Test Map
	for (unsigned i = 0; i < 100; i++) {
		hsfm::HSFM.computeForces(agent1, other_agents);
		hsfm::HSFM.updatePosition(agent1, dt);

		std::cout<<"Iteration: "<<i<<std::endl;
		std::cout<<"Time: "<<(i+1)*dt<<std::endl;
		std::cout<<"Agent1 Position: "<<agent1.position<<std::endl;
		std::cout<<"Agent1 Yaw: "<<agent1.yaw<<std::endl;
		std::cout<<"Agent1 Global Force: "<<agent1.forces.globalForce<<std::endl;
		std::cout<<"Agent1 Torque Force: "<<agent1.forces.torqueForce<<std::endl;
		std::cout<<"Agent1 Desired Force: "<<agent1.forces.desiredForce<<std::endl;
		std::cout<<"Agent1 Obstacle Force: "<<agent1.forces.obstacleForce<<std::endl;
		std::cout<<"Agent1 Social Force: "<<agent1.forces.socialForce<<std::endl;
		std::cout<<"Agent1 Group Force: "<<agent1.forces.groupForce<<std::endl;
		std::cout<<"Agent1 Velocity: "<<agent1.velocity<<std::endl;
		std::cout<<"Agent1 Linear Velocity: "<<agent1.linearVelocity<<std::endl;
		std::cout<<"Agent1 Angular Velocity: "<<agent1.angularVelocity<<std::endl;
		std::cout<<std::endl;
	}

	std::cout<<"END TEST"<<std::endl;

	return 0;
}
