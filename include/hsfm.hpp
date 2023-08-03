/***********************************************************************/
/**                                                                    */
/** sfm.hpp                                                            */
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
/** This software may be modified and distributed under the terms      */
/** of the BSD license. See the LICENSE file for details.              */
/**                                                                    */
/** http://www.opensource.org/licenses/BSD-3-Clause                    */
/**                                                                    */
/***********************************************************************/

#ifndef _HSFM_HPP_
#define _HSFM_HPP_

#include "map.hpp"
#include "vector2d.hpp"
#include <cmath>
#include <unordered_map>
#include <vector>

namespace hsfm {
struct Forces {
  utils::Vector2d desiredForce; // f_i^o
  utils::Vector2d obstacleForce; // f_i^w
  utils::Vector2d socialForce; // f_i^p
  utils::Vector2d interactionForce; // f_i^e = f_i^p + f_i^w
  utils::Vector2d groupForce; // h(p_i, r_i, d) = [h(p_i, r_i^f, d^f),h(p_i, r_i^o, d^o)]
  utils::Vector2d globalForce; // u_i = [u_i^f, u_i^o]
  double torqueForce; // u_i^theta
};

struct Parameters {
  Parameters()
      : relaxationTime(0.5), kOrthogonal(1), kDamping(500), kLambda(0.3), alpha(3),
        groupDistanceForward(2), groupDistanceOrthogonal(1), k1g(200), k2g(200),
        Ai(2000), Aw(2000), Bi(0.08), Bw(0.08), k1(120000), k2(240000) {}

  double relaxationTime;
  double kOrthogonal;
  double kDamping;
  double kLambda;
  double alpha;
  double groupDistanceForward;
  double groupDistanceOrthogonal;
  double k1g;
  double k2g;
  double Ai;
  double Aw;
  double Bi;
  double Bw;
  double k1;
  double k2;
};

struct Goal {
  utils::Vector2d center;
  double radius;
};

struct Agent {
  Agent()
      : desiredVelocity(0.6), radius(0.35), cyclicGoals(false),
        teleoperated(false), antimove(false), linearVelocity(0),
        angularVelocity(0), groupId(-1), mass(75) {}

  Agent(double linearVelocity, double angularVelocity)
      : desiredVelocity(0.6), radius(0.35), cyclicGoals(false),
        teleoperated(true), antimove(false), linearVelocity(linearVelocity),
        angularVelocity(angularVelocity), groupId(-1), mass(75) {}

  Agent(const utils::Vector2d &position, const utils::Angle &yaw,
        double linearVelocity, double angularVelocity)
      : position(position), yaw(yaw), desiredVelocity(0.6), radius(0.35),
        cyclicGoals(false), teleoperated(true), antimove(false),
        linearVelocity(linearVelocity), angularVelocity(angularVelocity),
        groupId(-1), mass(75) {}

  void move(double dt); // only if teleoperated

  utils::Vector2d position;
  utils::Vector2d velocity;
  utils::Vector2d initPosition;
  utils::Angle initYaw;
  utils::Angle yaw;
  utils::Vector2d movement;
  double desiredVelocity;
  double radius;
  std::list<Goal> goals;
  bool cyclicGoals;
  bool teleoperated;
  bool antimove;
  double linearVelocity;
  double angularVelocity;
  int groupId;
  double mass;

  double inertia = 0.5 * mass * radius * radius;

  double kTheta;
  double kOmega;

  int id;

  Forces forces;
  Parameters params;
  std::vector<utils::Vector2d> obstacles1;
  std::vector<utils::Vector2d> obstacles2;
};

struct Group {
  utils::Vector2d center;
  std::vector<unsigned> agents;
};

class SocialForceModel {
public:
  SocialForceModel(SocialForceModel const &) = delete;
  void operator=(SocialForceModel const &) = delete;
  ~SocialForceModel() {}

  static SocialForceModel &getInstance() {
    static SocialForceModel singleton;
    return singleton;
  }

#define HSFM SocialForceModel::getInstance()

  std::vector<Agent> &computeForces(std::vector<Agent> &agents, utils::Map *map = NULL) const;
  void computeForces(Agent &me, std::vector<Agent> &agents, utils::Map *map = NULL);
  std::vector<Agent> &updatePosition(std::vector<Agent> &agents,
                                     double dt) const;
  void updatePosition(Agent &me, double dt) const;

private:
#define PW(x) ((x) * (x))
  SocialForceModel() {}
  utils::Vector2d computeDesiredForce(Agent &agent) const;
  void computeObstacleForce(Agent &agent, utils::Map *map) const;
  void computeSocialForce(unsigned index, std::vector<Agent> &agents) const;
  void computeSocialForce(Agent &agent, std::vector<Agent> &agents) const;
  void computeGroupForce(unsigned index,
                         const utils::Vector2d &desiredDirection,
                         std::vector<Agent> &agents,
                         const std::unordered_map<int, Group> &groups) const;
  void computeGroupForce(Agent &me, const utils::Vector2d &desiredDirection,
                         std::vector<Agent> &agents, Group &group) const;
};

inline utils::Vector2d
SocialForceModel::computeDesiredForce(Agent &agent) const {
  // Method to compute the desired force that attracts the agent towards the goal
  utils::Vector2d desiredDirection;
  if (!agent.goals.empty() && (agent.goals.front().center - agent.position).norm() > agent.goals.front().radius) {
    utils::Vector2d diff = agent.goals.front().center - agent.position;
    desiredDirection = diff.normalized();
    agent.forces.desiredForce = agent.mass * (agent.desiredVelocity * desiredDirection - agent.velocity) / agent.params.relaxationTime;
    agent.antimove = false;
    // Debug prints
    // std::cout<<"Desired direction: "<<desiredDirection<<std::endl;
    // std::cout<<"Agent velocity: "<<agent.velocity<<std::endl;
    // std::cout<<std::endl;
  } else {
    agent.forces.desiredForce = -agent.velocity / agent.params.relaxationTime;
    agent.antimove = true;
  }
  return desiredDirection;
}

inline void SocialForceModel::computeObstacleForce(Agent &agent, utils::Map *map) const {
  // Method to compute the repulsive force coming from obstacles
  if (agent.obstacles1.size() > 0 || agent.obstacles2.size() > 0) {
    // Obstacles already available
    agent.forces.obstacleForce.set(0, 0);
    for (unsigned i = 0; i < agent.obstacles1.size(); i++) {
      utils::Vector2d minDiff = agent.position - agent.obstacles1[i];
      double distance = minDiff.norm();
      utils::Vector2d t_iw = minDiff.normalized().tangent(); //Tangent
      utils::Vector2d delta_v_iw = -agent.velocity * t_iw;
      agent.forces.obstacleForce += (agent.params.Aw * std::exp((agent.radius - distance)/ agent.params.Bw) + agent.params.k1 * std::max(double(0), agent.radius - distance)) * minDiff.normalized() - agent.params.k2 * std::max(double(0), agent.radius - distance) * delta_v_iw * t_iw;
    }
    for (unsigned i = 0; i < agent.obstacles2.size(); i++) {
      utils::Vector2d minDiff = agent.position - agent.obstacles2[i];
      double distance = minDiff.norm();
      utils::Vector2d t_iw = minDiff.normalized().tangent(); //Tangent
      utils::Vector2d delta_v_iw = -agent.velocity * t_iw;
      agent.forces.obstacleForce += (agent.params.Aw * std::exp((agent.radius - distance)/ agent.params.Bw) + agent.params.k1 * std::max(double(0), agent.radius - distance)) * minDiff.normalized() - agent.params.k2 * std::max(double(0), agent.radius - distance) * delta_v_iw * t_iw;
    }
    agent.forces.obstacleForce /= (double)(agent.obstacles1.size() + agent.obstacles2.size());
  } else if (map != NULL) {
    // Obstacles not available yet
    agent.forces.obstacleForce.set(0, 0);
    const utils::Map::Obstacle &obs = map->getNearestObstacle(agent.position);
    utils::Vector2d minDiff = agent.position - obs.position;
    double distance = minDiff.norm();
    utils::Vector2d t_iw = minDiff.normalized().tangent(); //Tangent
      utils::Vector2d delta_v_iw = -agent.velocity * t_iw;
      agent.forces.obstacleForce += (agent.params.Aw * std::exp((agent.radius - distance)/ agent.params.Bw) + agent.params.k1 * std::max(double(0), agent.radius - distance)) * minDiff.normalized() - agent.params.k2 * std::max(double(0), agent.radius - distance) * delta_v_iw * t_iw;
  } else {
    // No obstacles
    agent.forces.obstacleForce.set(0, 0);
  }
}

inline void SocialForceModel::computeSocialForce(unsigned index, std::vector<Agent> &agents) const {
  // Method to compute the repulsion force generated from pedestrians interactions
  Agent &agent = agents[index];
  agent.forces.socialForce.set(0, 0);
  for (unsigned i = 0; i < agents.size(); i++) {
    if (i == index) {
      continue;
    }
    double r_ij = agent.radius + agents[i].radius;
    utils::Vector2d diff = agent.position - agents[i].position;
    double d_ij = diff.norm();
    utils::Vector2d n_ij = diff.normalized();
    utils::Vector2d t_ij = n_ij.tangent();
    utils::Vector2d delta_v_ij = (agents[i].velocity - agent.velocity) * t_ij;
    agent.forces.socialForce += (agent.params.Ai * std::exp((r_ij - d_ij)/agent.params.Bi) + agent.params.k1 * std::max(double(0), r_ij - d_ij)) * n_ij + agent.params.k2 * std::max(double(0), r_ij - d_ij) * delta_v_ij * t_ij;
  }
}

inline void SocialForceModel::computeSocialForce(Agent &me, std::vector<Agent> &agents) const {
  // Method to compute the repulsion force generated from pedestrians interactions
  me.forces.socialForce.set(0, 0);
  for (unsigned i = 0; i < agents.size(); i++) {
    if (agents[i].id == me.id) {
      continue;
    }
    double r_ij = me.radius + agents[i].radius;
    utils::Vector2d diff = me.position - agents[i].position;
    double d_ij = diff.norm();
    utils::Vector2d n_ij = diff.normalized();
    utils::Vector2d t_ij = n_ij.tangent();
    utils::Vector2d delta_v_ij = (agents[i].velocity - me.velocity) * t_ij;
    me.forces.socialForce += (me.params.Ai * std::exp((r_ij - d_ij)/me.params.Bi) + me.params.k1 * std::max(double(0), r_ij - d_ij)) * n_ij + me.params.k2 * std::max(double(0), r_ij - d_ij) * delta_v_ij * t_ij;
  }
}

inline void SocialForceModel::computeGroupForce(unsigned index, const utils::Vector2d &desiredDirection, std::vector<Agent> &agents, const std::unordered_map<int, Group> &groups) const {
  Agent &agent = agents[index];
  agent.forces.groupForce.set(0, 0);
  if (groups.count(agent.groupId) == 0 || groups.at(agent.groupId).agents.size() < 2) {
    return;
  }
  const Group &group = groups.at(agent.groupId);
  utils::Vector2d com = group.center;
  std::vector<double> rotationVecForward {agent.yaw.cos(), agent.yaw.sin()}; // r_i^f
  std::vector<double> rotationVecOrthogonal {-agent.yaw.sin(), agent.yaw.cos()}; // r_i^o
  utils::Vector2d p_i = com - agent.position;

  // Forward force
  double forwardDist = p_i * rotationVecForward;
  if (std::abs(forwardDist) > agent.params.groupDistanceForward) {
    agent.forces.groupForce.setX(agent.params.k1g);
  } else {
    agent.forces.groupForce.setX(double(0));
  }

  // Orthogonal force
  double orthogonalDist = p_i * rotationVecOrthogonal;
  if (std::abs(orthogonalDist) > agent.params.groupDistanceOrthogonal) {
    agent.forces.groupForce.setY(agent.params.k2g);
  } else {
    agent.forces.groupForce.setY(double(0));
  }
}

inline void SocialForceModel::computeGroupForce(Agent &me, const utils::Vector2d &desiredDirection, std::vector<Agent> &agents, Group &group) const {
  me.forces.groupForce.set(0, 0);
  if (group.agents.size() < 2) {
    return;
  }
  utils::Vector2d com = group.center;
  std::vector<double> rotationVecForward {me.yaw.cos(), me.yaw.sin()}; // r_i^f
  std::vector<double> rotationVecOrthogonal {-me.yaw.sin(), me.yaw.cos()}; // r_i^o
  utils::Vector2d p_i = com - me.position;

  // Forward force
  double forwardDist = p_i * rotationVecForward;
  if (std::abs(forwardDist) > me.params.groupDistanceForward) {
    me.forces.groupForce.setX(me.params.k1g);
  } else {
    me.forces.groupForce.setX(double(0));
  }

  // Orthogonal force
  double orthogonalDist = p_i * rotationVecOrthogonal;
  if (std::abs(orthogonalDist) > me.params.groupDistanceOrthogonal) {
    me.forces.groupForce.setY(me.params.k2g);
  } else {
    me.forces.groupForce.setY(double(0));
  }
}

inline std::vector<Agent> &SocialForceModel::computeForces(std::vector<Agent> &agents, utils::Map *map) const {
  std::unordered_map<int, Group> groups;
  for (unsigned i = 0; i < agents.size(); i++) {
    if (agents[i].groupId < 0) {
      continue;
    }
    groups[agents[i].groupId].agents.push_back(i);
    groups[agents[i].groupId].center += agents[i].position;
  }
  for (auto it = groups.begin(); it != groups.end(); ++it) {
    it->second.center /= (double)(it->second.agents.size());
  }

  for (unsigned i = 0; i < agents.size(); i++) {
    utils::Vector2d desiredDirection = computeDesiredForce(agents[i]);
    computeObstacleForce(agents[i], map);
    computeSocialForce(i, agents);
    computeGroupForce(i, desiredDirection, agents, groups);
    agents[i].forces.interactionForce = agents[i].forces.obstacleForce + agents[i].forces.socialForce;

    // Rotational matrix
    std::vector<double> rotationalMatrixForward {agents[i].yaw.cos(), agents[i].yaw.sin()}; // r_i^f
    std::vector<double> rotationalMatrixOrthogonal {-agents[i].yaw.sin(), agents[i].yaw.cos()}; // r_i^o

    // Forward global force
    agents[i].forces.globalForce.setX((agents[i].forces.desiredForce + agents[i].forces.interactionForce) * rotationalMatrixForward + agents[i].forces.groupForce.getX()); // u_i^f
    // Compute orthogonal velocity of the agent
    double orthogonalVelocity = agents[i].velocity * rotationalMatrixOrthogonal;
    agents[i].forces.globalForce.setY(agents[i].params.kOrthogonal * (agents[i].forces.interactionForce) * rotationalMatrixOrthogonal - agents[i].params.kDamping * orthogonalVelocity + agents[i].forces.groupForce.getY()); //u_i^o
    // Torque parameters
    agents[i].kTheta = agents[i].inertia * agents[i].params.kLambda * agents[i].forces.desiredForce.norm();
    agents[i].kOmega = agents[i].inertia * (1 + agents[i].params.alpha) * sqrt((agents[i].params.kLambda * agents[i].forces.desiredForce.norm()) / agents[i].params.alpha);
    // TBD: Compute angular velocity of the agent
    // Torque force
    agents[i].forces.torqueForce = -agents[i].kTheta * (agents[i].yaw - agents[i].forces.desiredForce.angle()).toRadian() - (agents[i].kOmega * agents[i].angularVelocity);
  }
  return agents;
}

inline void SocialForceModel::computeForces(Agent &me, std::vector<Agent> &agents, utils::Map *map) {
  // form the group
  Group mygroup;
  if (me.groupId != -1) {
    mygroup.agents.push_back(me.id);
    mygroup.center = me.position;
    for (unsigned i = 0; i < agents.size(); i++) {
      if (agents[i].id == me.id) {
        continue;
      }
      if (agents[i].groupId == me.groupId) {
        mygroup.agents.push_back(i);
        mygroup.center += agents[i].position;
      }
    }
    mygroup.center /= (double)mygroup.agents.size();
  }

  // Compute agent's forces
  utils::Vector2d desiredDirection = computeDesiredForce(me);
  computeObstacleForce(me, map);
  computeSocialForce(me, agents);
  computeGroupForce(me, desiredDirection, agents, mygroup);
  me.forces.interactionForce = me.forces.obstacleForce + me.forces.socialForce;

  // Rotational matrix
  std::vector<double> rotationalMatrixForward {me.yaw.cos(), me.yaw.sin()}; // r_i^f
  std::vector<double> rotationalMatrixOrthogonal {-me.yaw.sin(), me.yaw.cos()}; // r_i^o
  // std::vector<double> rotationalMatrixForward {std::cos(me.yaw.negative()), std::sin(me.yaw.negative())}; // r_i^f
  // std::vector<double> rotationalMatrixOrthogonal {-std::sin(me.yaw.negative()), std::cos(me.yaw.negative())}; // r_i^o

  // Forward global force
  me.forces.globalForce.setX((me.forces.desiredForce + me.forces.interactionForce) * rotationalMatrixForward + me.forces.groupForce.getX()); // u_i^f
  // Compute orthogonal velocity of the agent
  double orthogonalVelocity = me.velocity * rotationalMatrixOrthogonal;
  me.forces.globalForce.setY(me.params.kOrthogonal * (me.forces.interactionForce) * rotationalMatrixOrthogonal - me.params.kDamping * orthogonalVelocity + me.forces.groupForce.getY()); //u_i^o
  // Torque parameters
  me.kTheta = me.inertia * me.params.kLambda * me.forces.desiredForce.norm();
  me.kOmega = me.inertia * (1 + me.params.alpha) * sqrt((me.params.kLambda * me.forces.desiredForce.norm()) / me.params.alpha);
  // TBD: Compute angular velocity of the agent
  // Torque force
  me.forces.torqueForce = -me.kTheta * (me.yaw - me.forces.desiredForce.angle()).toRadian() - (me.kOmega * me.angularVelocity);
}

inline void Agent::move(double dt) {
  double imd = linearVelocity * dt;
  utils::Vector2d inc(
      imd * std::cos(yaw.toRadian() + angularVelocity * dt * 0.5),
      imd * std::sin(yaw.toRadian() + angularVelocity * dt * 0.5));
  yaw += utils::Angle::fromRadian(angularVelocity * dt);
  position += inc;
  velocity.set(linearVelocity * yaw.cos(), linearVelocity * yaw.sin());
}

inline std::vector<Agent> &SocialForceModel::updatePosition(std::vector<Agent> &agents, double dt) const {
  for (unsigned i = 0; i < agents.size(); i++) {
    agents[i].initPosition = agents[i].position;
    agents[i].initYaw = agents[i].yaw;
    if (agents[i].teleoperated) {
      double imd = agents[i].linearVelocity * dt;
      utils::Vector2d inc(imd * std::cos(agents[i].yaw.toRadian() +
                                         agents[i].angularVelocity * dt * 0.5),
                          imd * std::sin(agents[i].yaw.toRadian() +
                                         agents[i].angularVelocity * dt * 0.5));
      agents[i].yaw += utils::Angle::fromRadian(agents[i].angularVelocity * dt);
      agents[i].position += inc;
      agents[i].velocity.set(agents[i].linearVelocity * agents[i].yaw.cos(),
                             agents[i].linearVelocity * agents[i].yaw.sin());
    } else {
      // Rotation matrix
      double rotationMatrix[2][2] = {{agents[i].yaw.cos(), -agents[i].yaw.sin()},{agents[i].yaw.sin(), agents[i].yaw.cos()}};

      // Bodyframe velocity
      utils::Vector2d vB_i = (agents[i].forces.globalForce / agents[i].mass) * dt;

      agents[i].velocity += vB_i * rotationMatrix;
      if (agents[i].velocity.norm() > agents[i].desiredVelocity) {
        agents[i].velocity.normalize();
        agents[i].velocity *= agents[i].desiredVelocity;
      }
      agents[i].linearVelocity = vB_i.norm();
      agents[i].angularVelocity += (agents[i].forces.torqueForce / agents[i].inertia) * dt;

      agents[i].yaw += utils::Angle(agents[i].angularVelocity * dt);
      agents[i].position += agents[i].velocity * dt;
    }
    agents[i].movement = agents[i].position - agents[i].initPosition;
    if (!agents[i].goals.empty() &&
        (agents[i].goals.front().center - agents[i].position).norm() <=
            agents[i].goals.front().radius) {
      Goal g = agents[i].goals.front();
      agents[i].goals.pop_front();
      if (agents[i].cyclicGoals) {
        agents[i].goals.push_back(g);
      }
    }
  }
  return agents;
}

inline void SocialForceModel::updatePosition(Agent &agent, double dt) const {

  agent.initPosition = agent.position;
  agent.initYaw = agent.yaw;

  // Rotation matrix
  double rotationMatrix[2][2] = {{agent.initYaw.cos(), -agent.initYaw.sin()},{agent.initYaw.sin(), agent.initYaw.cos()}};
  // double rotationMatrix[2][2] = {{std::cos(agent.initYaw.negative()), -std::sin(agent.initYaw.negative())},{std::sin(agent.initYaw.negative()), std::cos(agent.initYaw.negative())}};

  // Bodyframe velocity
  utils::Vector2d vB_i = (agent.forces.globalForce / agent.mass) * dt;

  agent.velocity += vB_i * rotationMatrix;

  if (agent.velocity.norm() > agent.desiredVelocity) {
    agent.velocity.normalize();
    agent.velocity *= agent.desiredVelocity;
  }

  agent.linearVelocity = agent.velocity.norm();
  agent.angularVelocity += (agent.forces.torqueForce / agent.inertia) * dt;

  agent.position += agent.velocity * dt;
  agent.yaw += utils::Angle(agent.angularVelocity * dt).toRadian();

  // Debug prints
  // std::cout<<"Desired force: "<<agent.forces.desiredForce<<std::endl;
  // std::cout<<"Social force: "<<agent.forces.socialForce<<std::endl;
  // std::cout<<"Obstacle force: "<<agent.forces.obstacleForce<<std::endl;
  // std::cout<<"Interaction force (obstacle + social): "<<agent.forces.interactionForce<<std::endl;
  // std::cout<<"Torque force: "<<agent.forces.torqueForce<<std::endl;
  // std::cout<<"Group force: "<<agent.forces.groupForce<<std::endl;
  // std::cout<<"Global force: "<<agent.forces.globalForce<<std::endl;
  // std::cout<<"Agent translational velocity: "<<agent.velocity<<std::endl;
  // std::cout<<"Agent angular velocity: "<<agent.angularVelocity<<std::endl;
  // std::cout<<"Agent position: "<<agent.position<<std::endl;
  // std::cout<<"Agent yaw: "<<agent.yaw<<std::endl;
  // std::cout<<std::endl;

  agent.movement = agent.position - agent.initPosition;
  if (!agent.goals.empty() &&
      (agent.goals.front().center - agent.position).norm() <=
          agent.goals.front().radius) {
    Goal g = agent.goals.front();
    agent.goals.pop_front();
    if (agent.cyclicGoals) {
      agent.goals.push_back(g);
    }
  }
}

} // namespace hsfm
#endif
