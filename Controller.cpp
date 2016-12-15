/*
 * Copyright (c) 2011-2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Karen Liu <karenliu@cc.gatech.edu>,
 *            Jeongseok Lee <jslee02@gmail.com>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include "Controller.h"

Controller::Controller(dart::dynamics::SkeletonPtr _skel, dart::constraint::ConstraintSolver* _constrSolver, double _t) {
  mSkel = _skel;
  mConstraintSolver = _constrSolver;
  mTimestep = _t;
  mVision = NULL;

  ////////////////////////////
  // my variables
  mApplyTorque = false;
  mVerifyLandingX = true;
  mManualRelease = false;

  mY = 0;
  mRows = 0; mColumns = 0;
  mPrev = -1; mCurr = -1;
  mV = 0; mMin = -1; mMax = -1;
  mSpan = 2.0;
  /////////////////////////////


  int nDof = mSkel->getNumDofs();
  mKp = Eigen::MatrixXd::Identity(nDof, nDof);
  mKd = Eigen::MatrixXd::Identity(nDof, nDof);
  mTorques.resize(nDof);
  mDefaultPose.resize(nDof);
  mDesiredDofs.resize(nDof);
  
  // Set default pose as the initial pose when the controller is instantiated
  mDefaultPose = mSkel->getPositions();
  mDesiredDofs = mDefaultPose;
  
  mTorques.setZero();

  // Using SPD results in simple spring coefficients
  for (int i = 0; i < nDof; i++)
    mKp(i, i) = 400.0;
  for (int i = 0; i < nDof; i++)
    mKd(i, i) = 40.0;

  // Global dofs don't have PD control
  for (int i = 0; i < 6; i++) {
    mKp(i, i) = 0.0;
    mKd(i, i) = 0.0;
  }

  // Make shoulders and elbows loose
  std::vector<int> dofIndex;
  dofIndex.push_back((mSkel->getDof("j_bicep_left_z")->getIndexInSkeleton()));
  dofIndex.push_back((mSkel->getDof("j_bicep_left_y")->getIndexInSkeleton()));
  dofIndex.push_back((mSkel->getDof("j_bicep_left_x")->getIndexInSkeleton()));
  dofIndex.push_back((mSkel->getDof("j_forearm_left")->getIndexInSkeleton()));
  dofIndex.push_back((mSkel->getDof("j_bicep_right_z")->getIndexInSkeleton()));
  dofIndex.push_back((mSkel->getDof("j_bicep_right_y")->getIndexInSkeleton()));
  dofIndex.push_back((mSkel->getDof("j_bicep_right_x")->getIndexInSkeleton()));
  dofIndex.push_back((mSkel->getDof("j_forearm_right")->getIndexInSkeleton()));
  for (int i = 0; i < dofIndex.size(); i++) {
    int index = dofIndex[i];
    mKp(index, index) = 20.0;
    mKd(index, index) = 2.0;
  }

  // Make wrists even looser
  dofIndex.clear();
  dofIndex.push_back((mSkel->getDof("j_hand_left_1")->getIndexInSkeleton()));
  dofIndex.push_back((mSkel->getDof("j_hand_left_2")->getIndexInSkeleton()));
  dofIndex.push_back((mSkel->getDof("j_hand_right_1")->getIndexInSkeleton()));
  dofIndex.push_back((mSkel->getDof("j_hand_right_2")->getIndexInSkeleton()));
  for (int i = 0; i < dofIndex.size(); i++) {
    int index = dofIndex[i];
    mKp(index, index) = 1.0;
    mKd(index, index) = 0.1;
  }

  for (int i = 0; i < nDof; i++)
    mSkel->getDof(i)->setDampingCoefficient(0.01);
  mPreOffset = 0.0;
  mLeftHandHold = NULL;
  mRightHandHold = NULL;
  mFootContact = NULL;
  mLeftHandContact = NULL;
  mRightHandContact = NULL;
  mTimer = 300;
  mState = "STAND";
}

Controller::~Controller() {
}

Eigen::VectorXd Controller::getTorques() {
  return mTorques;
}

double Controller::getTorque(int _index) {
  return mTorques[_index];
}

void Controller::setDesiredDof(int _index, double _val) {
  mDesiredDofs[_index] = _val;
}

void Controller::computeTorques(int _currentFrame) {
  mCurrentFrame = _currentFrame;
  mTorques.setZero();
  if (mState == "STAND") {
    stand();
  } else if (mState == "CROUCH") {
    crouch();
  } else if (mState == "JUMP") {
    jump();
  } else if (mState == "REACH") {
    reach();
  } else if (mState == "GRAB") {
    grab();
  } else if (mState == "RELEASE") {
    release();
  } else if (mState == "SWING") {
    swing();
  } else {
    std::cout << "Illegal state: " << mState << std::endl;
  }

  // Just to make sure no illegal torque is used. Do not remove this.
  for (int i = 0; i < 6; i++) {
    mTorques[i] = 0.0;
  }
}

void Controller::checkContactState() {
  mFootContact = NULL;
  mLeftHandContact = NULL;
  mRightHandContact = NULL;
  dart::collision::CollisionDetector* cd = mConstraintSolver->getCollisionDetector();
  int nContacts = cd->getNumContacts();
  for (int i = 0; i < nContacts; i++) {
    dart::dynamics::BodyNodePtr body1 = cd->getContact(i).bodyNode1.lock().get();
    dart::dynamics::BodyNodePtr body2 = cd->getContact(i).bodyNode2.lock().get();
  
    if (body1 == mSkel->getBodyNode("h_heel_left") || body1 == mSkel->getBodyNode("h_heel_left")
        || body1 == mSkel->getBodyNode("h_heel_right") || body1 == mSkel->getBodyNode("h_heel_right"))
      mFootContact = body2;
    if (body2 == mSkel->getBodyNode("h_heel_left") || body2 == mSkel->getBodyNode("h_heel_left")
        || body2 == mSkel->getBodyNode("h_heel_right") || body2 == mSkel->getBodyNode("h_heel_right"))
      mFootContact = body1;
    if (body1->isCollidable() && body1 == mSkel->getBodyNode("h_hand_left"))
      mLeftHandContact = body2;
    if (body2->isCollidable() && body2 == mSkel->getBodyNode("h_hand_left"))
      mLeftHandContact = body1;
    if (body1->isCollidable() && body1 == mSkel->getBodyNode("h_hand_right"))
      mRightHandContact = body2;
    if (body2->isCollidable() && body2 == mSkel->getBodyNode("h_hand_right"))
      mRightHandContact = body1;
  }
}

void Controller::stand() {
  // Change to default standing pose
  mDesiredDofs = mDefaultPose;
  stablePD();
  ankleStrategy();
  mTimer--;
  mY = mSkel->getCOM().y();

  // Switch to crouch if time is up
  if (mTimer == 0) {
    mState = "CROUCH";
    mTimer = 500;
    std::cout << mCurrentFrame << ": " << "STAND -> CROUCH" << std::endl;
  }
}

void Controller::crouch() {

  // Change to crouching pose
  mDesiredDofs = mDefaultPose;
  mDesiredDofs[mSkel->getDof("j_thigh_left_z")->getIndexInSkeleton()] = 0.7;
  mDesiredDofs[mSkel->getDof("j_thigh_right_z")->getIndexInSkeleton()] = 0.7;
  mDesiredDofs[mSkel->getDof("j_shin_left")->getIndexInSkeleton()] = -1.1;
  mDesiredDofs[mSkel->getDof("j_shin_right")->getIndexInSkeleton()] = -1.1;
  mDesiredDofs[mSkel->getDof("j_heel_left_1")->getIndexInSkeleton()] = 0.6;
  mDesiredDofs[mSkel->getDof("j_heel_right_1")->getIndexInSkeleton()] = 0.6;
  mDesiredDofs[mSkel->getDof("j_abdomen_2")->getIndexInSkeleton()] = -0.2;

  // After a while, lean forward
  if (mTimer < 200) {
    mDesiredDofs[mSkel->getDof("j_heel_left_1")->getIndexInSkeleton()] = 1.0;
    mDesiredDofs[mSkel->getDof("j_heel_right_1")->getIndexInSkeleton()] = 1.0;
  }

  stablePD();
  ankleStrategy();
  mTimer--;

  if (mTimer == 0) {
    mState = "JUMP";
    std::cout << mCurrentFrame << ": " << "CROUCH -> JUMP" << std::endl;

  }
}

void Controller::jump() {
  // Change to leaping pose
  mDesiredDofs = mDefaultPose;
  mDesiredDofs[mSkel->getDof("j_thigh_left_z")->getIndexInSkeleton()] = 0.2;
  mDesiredDofs[mSkel->getDof("j_thigh_right_z")->getIndexInSkeleton()] = 0.2;
  mDesiredDofs[mSkel->getDof("j_shin_left")->getIndexInSkeleton()] = -0.2;
  mDesiredDofs[mSkel->getDof("j_shin_right")->getIndexInSkeleton()] = -0.2;
  mDesiredDofs[mSkel->getDof("j_heel_left_1")->getIndexInSkeleton()] = -0.2;
  mDesiredDofs[mSkel->getDof("j_heel_right_1")->getIndexInSkeleton()] = -0.2;
  mDesiredDofs[mSkel->getDof("j_abdomen_2")->getIndexInSkeleton()] = -0.2;
  mDesiredDofs[mSkel->getDof("j_bicep_left_z")->getIndexInSkeleton()] = 0.3;
  mDesiredDofs[mSkel->getDof("j_bicep_left_y")->getIndexInSkeleton()] = -1.0;
  mDesiredDofs[mSkel->getDof("j_bicep_right_z")->getIndexInSkeleton()] = 0.3;
  mDesiredDofs[mSkel->getDof("j_bicep_right_y")->getIndexInSkeleton()] = 1.0;
  mDesiredDofs[mSkel->getDof("j_forearm_left")->getIndexInSkeleton()] = 0.5;
  mDesiredDofs[mSkel->getDof("j_forearm_right")->getIndexInSkeleton()] = 0.5;
  
  stablePD();

  // Use Jacobian transpose to compute pushing torques
  Eigen::Vector3d vf(-1100.0, -2600, 0.0);
  Eigen::Vector3d offset(0.05, -0.02, 0.0);
  virtualForce(vf, mSkel->getBodyNode("h_heel_left"), offset);
  virtualForce(vf, mSkel->getBodyNode("h_heel_right"), offset);

  checkContactState();
  if (mFootContact == NULL) {
    mState = "REACH";
    std::cout << mCurrentFrame << ": " << "JUMP -> REACH" << std::endl;
  }
}

void Controller::reach() {
  // Change to reaching pose
  mDesiredDofs = mDefaultPose;
  mDesiredDofs[mSkel->getDof("j_thigh_left_z")->getIndexInSkeleton()] = 0.2;
  mDesiredDofs[mSkel->getDof("j_thigh_right_z")->getIndexInSkeleton()] = 0.2;
  mDesiredDofs[mSkel->getDof("j_shin_left")->getIndexInSkeleton()] = -0.2;
  mDesiredDofs[mSkel->getDof("j_shin_right")->getIndexInSkeleton()] = -0.2;
  mDesiredDofs[mSkel->getDof("j_heel_left_1")->getIndexInSkeleton()] = -0.2;
  mDesiredDofs[mSkel->getDof("j_heel_right_1")->getIndexInSkeleton()] = -0.2;
  mDesiredDofs[mSkel->getDof("j_abdomen_2")->getIndexInSkeleton()] = -0.2;
  mDesiredDofs[mSkel->getDof("j_bicep_left_z")->getIndexInSkeleton()] = 0.7;
  mDesiredDofs[mSkel->getDof("j_bicep_left_y")->getIndexInSkeleton()] = -2.3;
  mDesiredDofs[mSkel->getDof("j_bicep_right_z")->getIndexInSkeleton()] = 0.7;
  mDesiredDofs[mSkel->getDof("j_bicep_right_y")->getIndexInSkeleton()] = 2.3;
  mDesiredDofs[mSkel->getDof("j_forearm_left")->getIndexInSkeleton()] = 0.4;
  mDesiredDofs[mSkel->getDof("j_forearm_right")->getIndexInSkeleton()] = 0.4;
  stablePD();

  checkContactState();
  if (mFootContact) { // If feet are in contact again, go back to JUMP and continue to push
    mState = "JUMP";
    std::cout << mCurrentFrame << ": " << "REACH -> JUMP" << std::endl;
  } else if (mLeftHandContact || mRightHandContact) {
    mState = "GRAB";
    mTimer = 500;
    std::cout << mCurrentFrame << ": " << "REACH -> GRAB" << std::endl;
  } else {
    mState = "REACH";
  }
}

void Controller::grab() {
  leftHandGrab();
  rightHandGrab();

  mDesiredDofs = mDefaultPose;
  mDesiredDofs[mSkel->getDof("j_thigh_left_z")->getIndexInSkeleton()] = 0.2;
  mDesiredDofs[mSkel->getDof("j_thigh_right_z")->getIndexInSkeleton()] = 0.2;
  mDesiredDofs[mSkel->getDof("j_shin_left")->getIndexInSkeleton()] = -0.2;
  mDesiredDofs[mSkel->getDof("j_shin_right")->getIndexInSkeleton()] = -0.2;
  mDesiredDofs[mSkel->getDof("j_heel_left_1")->getIndexInSkeleton()] = -0.2;
  mDesiredDofs[mSkel->getDof("j_heel_right_1")->getIndexInSkeleton()] = -0.2;
  mDesiredDofs[mSkel->getDof("j_abdomen_2")->getIndexInSkeleton()] = -0.2;
  mDesiredDofs[mSkel->getDof("j_bicep_left_z")->getIndexInSkeleton()] = 0.7;
  mDesiredDofs[mSkel->getDof("j_bicep_left_y")->getIndexInSkeleton()] = -2.3;
  mDesiredDofs[mSkel->getDof("j_bicep_right_z")->getIndexInSkeleton()] = 0.7;
  mDesiredDofs[mSkel->getDof("j_bicep_right_y")->getIndexInSkeleton()] = 2.3;
  mDesiredDofs[mSkel->getDof("j_forearm_left")->getIndexInSkeleton()] = 0.4;
  mDesiredDofs[mSkel->getDof("j_forearm_right")->getIndexInSkeleton()] = 0.4;
  stablePD();
  mTimer--;

  if (mTimer == 0) {
    mState = "SWING";
    std::cout << mCurrentFrame << ": " << "GRAB -> SWING" << std::endl;
  }
}  

// Travis' variables
bool dump_swing = false;
int state = 1;
int last_state = 1;
bool first_pass = true;
double x_mag = 0;
double angle_thigh_left_z = 0;
double angle_thigh_right_z = 0;
double angle_knee_left = 0;
double angle_knee_right = 0;

void Controller::swing() {
  // TODO: Need a better controller to increase the speed
  // and land at the right moment
  
  //////////////////////////////////////////////////////////////
  // Change dof desired positions to gain momentum using SPD
  gainMomentum();
  stablePD();

  // TODO: Figure out the condition to release the bar

  //////////////////////////////////////////////////////////////
  // Compute release condition
  computePlatformVelocity();  // compute platform speed
  computePlatformLimits(); // compute platform limits
  adjustPlatformVelocitySign();
  
  double h, xL;
  computeTimeOfFlightAndLandingX(h, xL); // time of flight and range for projectle motion if relese NOW

  double xP=-1000000;
  boolean release = false;
  boolean hasPlatformPosition = computePlatformPosition(h, xP);
  if(hasPlatformPosition)
  {
	  if (abs(xP - xL) < 0.25*mSpan) release = true;
  }

  // Display values on console
  if (mApplyTorque || release || mManualRelease)
  {
	  //mTorques[mSkel->getDof("j_hand_left_1")->getIndexInSkeleton()] += 2000;
	  //mTorques[mSkel->getDof("j_hand_right_1")->getIndexInSkeleton()] += 2000;
	  
	  std::cout << "\n == == == == == == == == == == == == == == == == == == =\n";
	  //std::cout << "prev: " << mPrev << ",  curr: " << mCurr << "\n";
	  if(mV!=0) std::cout << "Platform Speed: " << mV << "\n";
	  else std::cout << "platform speed not computed yet." << "\n";

	  if(mMin!=-1 && mMax!=-1) std::cout << "min: " << mMin << ",  max: " << mMax << "\n";
	  else std::cout<<"limits not computed yet." <<"\n";

	  std::cout <<"time of flight: " << h <<" and  landing X: " << xL << "\n";

	  if(hasPlatformPosition) std::cout << "platform position X: " << xP << "\n";
	  else std::cout << "platform position not computable yet." << "\n";

	  if (release) std::cout << "relese - YES" << "\n";
	  else if (mManualRelease) std::cout << "relese - MANUAL" << "\n";
	  else std::cout << "release - NO" << "\n";

	  std::cout<< "\n == == == == == == == == == == == == == == == == == == =\n";
	  mApplyTorque = false; 
  }
  
  if (release || mManualRelease) {
    mState = "RELEASE";
    std::cout << mCurrentFrame << ": " << "SWING -> RELEASE" << std::endl;
  }
}

void Controller::release() {
  leftHandRelease();
  rightHandRelease();

  mDesiredDofs = mDefaultPose;
  mDesiredDofs[mSkel->getDof("j_abdomen_2")->getIndexInSkeleton()] = -1.5;
  mDesiredDofs[mSkel->getDof("j_shin_left")->getIndexInSkeleton()] = -1.5;
  mDesiredDofs[mSkel->getDof("j_shin_right")->getIndexInSkeleton()] = -1.5;
  stablePD();

  if (mVerifyLandingX && mSkel->getCOM().y() <= mY)
  { 
	  std::cout << "<actual landing X: " << mSkel->getCOM().x() <<"\n";
	  mVerifyLandingX = false;
  }
}
  
void Controller::stablePD() {
  Eigen::VectorXd q = mSkel->getPositions();
  Eigen::VectorXd dq = mSkel->getVelocities();
  Eigen::VectorXd constrForces = mSkel->getConstraintForces();
  Eigen::MatrixXd invM = (mSkel->getMassMatrix() + mKd * mTimestep).inverse();
  Eigen::VectorXd p = -mKp * (q + dq * mTimestep - mDesiredDofs);
  Eigen::VectorXd d = -mKd * dq;
  Eigen::VectorXd qddot =
      invM * (-mSkel->getCoriolisAndGravityForces() + p + d + constrForces);

  mTorques += p + d - mKd * qddot * mTimestep;
}

void Controller::ankleStrategy() {
  Eigen::Vector3d com = mSkel->getCOM();
  Eigen::Vector3d cop = mSkel->getBodyNode("h_heel_left")->getTransform()
                        * Eigen::Vector3d(0.05, 0, 0);
  double offset = com[0] - cop[0];
   if (offset < 0.1 && offset > 0.0) {
    double k1 = 200.0;
    double k2 = 100.0;
    double kd = 10.0;
    mTorques[mSkel->getDof("j_heel_left_1")->getIndexInSkeleton()] += -k1 * offset + kd * (mPreOffset - offset);
    mTorques[mSkel->getDof("j_toe_left")->getIndexInSkeleton()] += -k2 * offset + kd * (mPreOffset - offset);
    mTorques[mSkel->getDof("j_heel_right_1")->getIndexInSkeleton()] += -k1 * offset + kd * (mPreOffset - offset);
    mTorques[mSkel->getDof("j_toe_right")->getIndexInSkeleton()] += -k2 * offset + kd * (mPreOffset - offset);
    mPreOffset = offset;
  } else if (offset > -0.2 && offset < -0.05) {
    double k1 = 2000.0;
    double k2 = 100.0;
    double kd = 100.0;
    mTorques[mSkel->getDof("j_heel_left_1")->getIndexInSkeleton()] += -k1 * offset + kd * (mPreOffset - offset);
    mTorques[mSkel->getDof("j_toe_left")->getIndexInSkeleton()] += -k2 * offset + kd * (mPreOffset - offset);
    mTorques[mSkel->getDof("j_heel_right_1")->getIndexInSkeleton()] += -k1 * offset + kd * (mPreOffset - offset);
    mTorques[mSkel->getDof("j_toe_right")->getIndexInSkeleton()] += -k2 * offset + kd * (mPreOffset - offset);
    mPreOffset = offset;
  }  
}

void Controller::virtualForce(Eigen::Vector3d _force, dart::dynamics::BodyNode* _bodyNode, Eigen::Vector3d _offset) {
  Eigen::MatrixXd jacobian = mSkel->getLinearJacobian(_bodyNode, _offset);
  mTorques += jacobian.transpose() * _force;
}

void Controller::leftHandGrab() {  
  if (mLeftHandHold != NULL)
    return;
  checkContactState();
  if (mLeftHandContact == NULL)
    return;
  dart::dynamics::BodyNode* bd = mSkel->getBodyNode("h_hand_left");
  dart::constraint::WeldJointConstraint *hold = new dart::constraint::WeldJointConstraint(bd, mLeftHandContact);
  mConstraintSolver->addConstraint(hold);
  bd->setCollidable(false);
  mLeftHandHold = hold;
}

void Controller::leftHandRelease() {
  if (mLeftHandHold == NULL)
    return;
  mConstraintSolver->removeConstraint(mLeftHandHold);
  mSkel->getBodyNode("h_hand_left")->setCollidable(true);
  mLeftHandHold = NULL;
}

void Controller::rightHandGrab() {  
  if (mRightHandHold != NULL)
    return;

  checkContactState();
  if (mRightHandContact == NULL)
    return;
  dart::dynamics::BodyNode* bd = mSkel->getBodyNode("h_hand_right");
  dart::constraint::WeldJointConstraint *hold = new dart::constraint::WeldJointConstraint(bd, mRightHandContact);
  mConstraintSolver->addConstraint(hold);
  bd->setCollidable(false);
  mRightHandHold = hold;
}

void Controller::rightHandRelease() {
  if (mRightHandHold == NULL)
    return;
  mConstraintSolver->removeConstraint(mRightHandHold);
  mSkel->getBodyNode("h_hand_right")->setCollidable(true);
  mRightHandHold = NULL;
}

void Controller::setState(std::string _state) {
  mState = _state;
}

dart::dynamics::SkeletonPtr Controller::getSkel() {
  return mSkel;
}

Eigen::VectorXd Controller::getDesiredDofs() {
  return mDesiredDofs;
}

Eigen::MatrixXd Controller::getKp() {
  return mKp;
}

Eigen::MatrixXd Controller::getKd() {
  return mKd;
}


void Controller::computeTimeOfFlightAndLandingX(double &h, double &xL) {
	Eigen::Vector3d C = mSkel->getCOM();
	Eigen::Vector3d V = mSkel->getCOMLinearVelocity();

	double u = V.dot(Eigen::Vector3d(1, 0, 0));
	double v = V.dot(Eigen::Vector3d(0, 1, 0));

	double y = C.y() - mY;

	h = (v + sqrt(v*v + 2 * 9.8*y)) / 9.8;

	double R = u*h;

	xL = C.x() + R; // xlanding=xCOM+Range

	//std::cout << "standing CG: " << mY << "  swing CG: " << C.x() << " , " << C.y() << " , " << C.z() << "\n======================\n";
}

int Controller::computePlatformTopEdgeInImage()
{
	int topEdgeRow = mRows;

	for (int r = mRows - 1; r >= 0; r--)
	{
		int flipCount = 0;
		int row = std::max(r - 1, 0);
		unsigned char rp = (*mVision)[row*mColumns * 4];
		unsigned char gp = (*mVision)[row*mColumns * 4 + 1];
		unsigned char bp = (*mVision)[row*mColumns * 4 + 2];
		//unsigned char ap = (*mVision)[row*mColumns * 4];

		for (int col = 1; col < mColumns; col++)
		{
			unsigned char r = (*mVision)[row*mColumns * 4 + col * 4];
			unsigned char g = (*mVision)[row*mColumns * 4 + col * 4 + 1];
			unsigned char b = (*mVision)[row*mColumns * 4 + col * 4 + 2];
			//unsigned char a = (*mVision)[row*mColumns * 4 + col * 4+3];

			if (abs(r - rp)>0 || abs(g - gp)>0 || abs(b - bp)>0) flipCount++;
			rp = r; gp = g; bp = b; //ap=a;
		}

		if (flipCount == 2)
		{
			topEdgeRow = r;  break;
		}
	}
	
	return topEdgeRow;
}

void Controller::computePlatformVelocity()
{
	if (mV == 0)
	{
		if (mPrev == -1)
		{
			mPrev = computePlatformTopEdgeInImage();
		}
		else if (mCurr == -1)
		{
			int newPosition = computePlatformTopEdgeInImage();
			if (newPosition != mPrev) mCurr = newPosition;
		}
		else if (mCurr != -1 && mPrev != -1)
		{
			mV = double((mCurr - mPrev) / 17.0);
		}
	}
}

void Controller::adjustPlatformVelocitySign()
{
	if (mMin!=-1 && mMax!=-1 && mV != 0)
	{		
		if (mPrev < mCurr)
		{
			if (mV < 0) mV = -1.0*mV;
		}
		else if (mCurr < mPrev)
		{
			if (mV > 0) mV = -1.0*mV;
		}

		int next = computePlatformTopEdgeInImage();
		if (next != mCurr)
		{
			mPrev = mCurr;
			mCurr = next;
		}
	}
}

void Controller::computePlatformLimits()
{
	if (mMin == -1 && mMax == -1 && mV != 0)
	{
		int next = computePlatformTopEdgeInImage();
		if (mPrev<mCurr && next<mCurr)
		{
			mMax = mCurr;
			mMin = mCurr - 140; // hardcoded for now
		}
		else if (next != mCurr)
		{
			mPrev = mCurr;
			mCurr = next;
		}
	}
}

boolean Controller::computePlatformPosition(double h, double &xP)
{
	if (mMin != -1 && mMax != -1 && mV != 0)
	{
		int current = computePlatformTopEdgeInImage(); // current position of top edge in image

		int frameRate = 60; // 60Hz
		int f = ceil(h*frameRate); // duration in terms of number of frames

		// v is number of rows moved per frame
		int future = current + ceil(mV*f); //future position of top edge in image

		if (future > mMax) { future = mMax - (future - mMax); }
		else if (future < mMin) { future = mMin + (mMin - future); }

		double platformSize = mSpan;
		double xLow = 1.4;
		double xHigh = 4.6;
		double fraction = double(current - mMin) / double(mMax - mMin);
		double xCurrent = xLow + fraction*(xHigh - xLow);

		xP = xCurrent - platformSize*0.5; // x coordinate of the center of the plaform

		//std::cout << "current: " <<current << ", mMin:" << mMin << ", mMax: " << mMax << ", xHigh:" << xHigh << ", xLow:" << xLow << "\n";
		//std::cout << "fraction: " << fraction <<"\n";
		//std::cout << "xCurrent: " << xCurrent << ", f:" << f << ", future: " << future << ", xP:" << xP << "\n";
		return true;
	}
	else return false;
}


void Controller::gainMomentum()
{
	double PI_4 = 3.14 / 4.0;
	double PI_2 = 3.14 / 2.0;

	double forward_thigh_z = PI_4;
	double backward_thigh_z = -PI_4;
	double forward_knee = 0;
	double backward_knee = -PI_2;

	double x_bar = 0.85; // this is in .skel file but it is private
	double scale_thigh = 0; // testing for now
	double scale_knee = 0;

	Eigen::Vector3d comV = mSkel->getCOMLinearVelocity();
	Eigen::Vector3d comP = mSkel->getCOM();
	int index_thigh_left_z = mSkel->getDof("j_thigh_left_z")->getIndexInSkeleton();
	int index_thigh_right_z = mSkel->getDof("j_thigh_right_z")->getIndexInSkeleton();
	int index_knee_left = mSkel->getDof("j_shin_left")->getIndexInSkeleton();
	int index_knee_right = mSkel->getDof("j_shin_right")->getIndexInSkeleton();
	//double angle_thigh_left_z = mSkel->getPositions()(index_thigh_left_z);
	//double angle_thigh_right_z = mSkel->getPositions()(index_thigh_right_z);
	//double angle_knee_left = mSkel->getPositions()(index_knee_left);
	//double angle_knee_right = mSkel->getPositions()(index_knee_right);

	if (comV.x() > 0 && comV.y() > 0) {
		if (last_state == 1 || last_state == 0) {
			state = 0;
			if (state != last_state) {
				last_state = state;
				std::cout << comV.norm() << "\n";
				//std::cout << "idle forward\n";
			}
			mDesiredDofs = mDefaultPose;
			//scale_knee = ((comP.x() - x_bar) / (2 * x_mag)) + 0.5;
			if (!first_pass) {
				mDesiredDofs[index_thigh_left_z] = forward_thigh_z;
				mDesiredDofs[index_thigh_right_z] = forward_thigh_z;
				//mDesiredDofs[index_knee_left] = angle_knee_left + (forward_knee - angle_knee_left)*scale_knee;
				//mDesiredDofs[index_knee_right] = angle_knee_right + (forward_knee - angle_knee_right)*scale_knee;
				mDesiredDofs[index_knee_left] = forward_knee;
				mDesiredDofs[index_knee_right] = forward_knee;
			}
		}
	}
	else if (comV.x() > 0 && comV.y() < 0) {
		if (last_state == 2 || last_state == 1) {
			state = 1;
			if (state != last_state) {
				last_state = state;
				x_mag = abs(comP.x() - x_bar);
				angle_thigh_left_z = mSkel->getPositions()(index_thigh_left_z);
				angle_thigh_right_z = mSkel->getPositions()(index_thigh_right_z);
				angle_knee_left = mSkel->getPositions()(index_knee_left);
				angle_knee_right = mSkel->getPositions()(index_knee_right);
				//std::cout << "forward\n";
			}
			mDesiredDofs = mDefaultPose;
			scale_thigh = 1.0 - (abs(comP.x() - x_bar) / x_mag);
			//scale_knee = ((comP.x() - x_bar)/(2*x_mag)) + 0.5;
			scale_knee = scale_thigh;
			if (!first_pass) {
				mDesiredDofs[index_thigh_left_z] = angle_thigh_left_z + (forward_thigh_z - angle_thigh_left_z)*scale_thigh;
				mDesiredDofs[index_thigh_right_z] = angle_thigh_right_z + (forward_thigh_z - angle_thigh_right_z)*scale_thigh;
				mDesiredDofs[index_knee_left] = angle_knee_left + (forward_knee - angle_knee_left)*scale_knee;
				mDesiredDofs[index_knee_right] = angle_knee_right + (forward_knee - angle_knee_right)*scale_knee;
			}
		}
	}
	else if (comV.x() < 0 && comV.y() > 0) {
		if (last_state == 3 || last_state == 2) {
			state = 2;
			if (state != last_state) {
				last_state = state;
				std::cout << comV.norm() << "\n";
				//std::cout << "idle backward\n";
			}
			mDesiredDofs = mDefaultPose;
			//scale_knee = ((comP.x() - x_bar) / (-2.0 * x_mag)) + 0.5;
			mDesiredDofs[index_thigh_left_z] = backward_thigh_z;
			mDesiredDofs[index_thigh_right_z] = backward_thigh_z;
			//mDesiredDofs[index_knee_left] = angle_knee_left + (backward_knee - angle_knee_left)*scale_knee;
			//mDesiredDofs[index_knee_right] = angle_knee_right + (backward_knee - angle_knee_right)*scale_knee;
			mDesiredDofs[index_knee_left] = backward_knee;
			mDesiredDofs[index_knee_right] = backward_knee;
			if (first_pass) first_pass = false;
		}
	}
	else if (comV.x() < 0 && comV.y() < 0) {
		if (last_state == 0 || last_state == 3) {
			state = 3;
			if (state != last_state) {
				last_state = state;
				x_mag = abs(comP.x() - x_bar);
				angle_thigh_left_z = mSkel->getPositions()(index_thigh_left_z);
				angle_thigh_right_z = mSkel->getPositions()(index_thigh_right_z);
				angle_knee_left = mSkel->getPositions()(index_knee_left);
				angle_knee_right = mSkel->getPositions()(index_knee_right);
				//std::cout << "backward\n";
			}
			mDesiredDofs = mDefaultPose;
			scale_thigh = 1.0 - (abs(comP.x() - x_bar) / x_mag);
			//scale_knee = ((comP.x() - x_bar) / (-2.0 * x_mag)) + 0.5;
			scale_knee = scale_thigh;
			mDesiredDofs[index_thigh_left_z] = angle_thigh_left_z + (backward_thigh_z - angle_thigh_left_z)*scale_thigh;
			mDesiredDofs[index_thigh_right_z] = angle_thigh_right_z + (backward_thigh_z - angle_thigh_right_z)*scale_thigh;
			mDesiredDofs[index_knee_left] = angle_knee_left + (backward_knee - angle_knee_left)*scale_knee;
			mDesiredDofs[index_knee_right] = angle_knee_right + (backward_knee - angle_knee_right)*scale_knee;
		}
	}
	else {
		mDesiredDofs = mDefaultPose;
		//std::cout << "\n\n" << "fringe case" << "\n\n";
	}
	mDesiredDofs[mSkel->getDof("j_bicep_left_z")->getIndexInSkeleton()] = 1;
	mDesiredDofs[mSkel->getDof("j_bicep_left_y")->getIndexInSkeleton()] = -2.6;
	mDesiredDofs[mSkel->getDof("j_bicep_right_z")->getIndexInSkeleton()] = 1;
	mDesiredDofs[mSkel->getDof("j_bicep_right_y")->getIndexInSkeleton()] = 2.6;
	mDesiredDofs[mSkel->getDof("j_forearm_left")->getIndexInSkeleton()] = 0.4;
	mDesiredDofs[mSkel->getDof("j_forearm_right")->getIndexInSkeleton()] = 0.4;

	//mDesiredDofs[mSkel->getDof("j_shin_left")->getIndexInSkeleton()] = 0;
	//mDesiredDofs[mSkel->getDof("j_shin_right")->getIndexInSkeleton()] = 0;
}