/*
 * VAgent.cpp
 *
 *  Created on: May 31, 2015
 *      Author: Sina M. Baharlou (Sina.Baharlou@gmail.com)
 */

#include "VAgent.hpp"



VAgent::VAgent(cv::Mat& stateMap,cv::Mat& policyMap):
stateMap(stateMap),
policyMap(policyMap),
agentPos(Vec2i(DEFAULT_POS)),
lastAction(RIGHT),
correctFactor(DEFAULT_CF),
agentColour(Vec3b(DEFAULT_AGENT_COLOR)),
active(true)
{

	mapSize=cv::Size(stateMap.cols,stateMap.rows);

	// -- Create Visit Map
	visitMap=cv::Mat(stateMap.rows,stateMap.cols,CV_8UC1);
	visitMap=(cv::Scalar)0;


	// -- Initializing distribution variables --

	rndAction=std::uniform_int_distribution<int>(FIRST_ACTION,LAST_ACTION);
	//rndState=std::uniform_int_distribution<int>(0,MAX(mapSize.width,mapSize.height));
	rndReal=std::uniform_real_distribution<double>(0,1);


	// -- Init random engine --

	 struct timespec ts;
	 unsigned theTick = 0U;
	 clock_gettime( CLOCK_REALTIME, &ts );
	 theTick  = ts.tv_nsec / 1000000;
	 theTick += ts.tv_sec * 1000;
	 rndEngine.seed(theTick);






}



Action 	VAgent::getRandomAction()	// -- Get Random action
{

	return (Action)rndAction(rndEngine);
}


bool 	VAgent::takeCorrectAction()		// -- If agent should take a correct action
{

	double random=(double)rndReal(rndEngine);	// -- Get a double random number between 0 and 1

	// -- Check if agent should take a greedy action
	if (random<=correctFactor)
		return true;
	return false;
}


float 	VAgent::takeAction(Action action)	// -- Take the specified action --
{
	bool obstacle=false;
	float reward=0;

	agentPos=transFunction(action,obstacle); 		// -- Update the position

	// -- Get the reward --
	if (obstacle)
	{
		reward=DEFAULT_OBSTACLE_REWARD;
	}
	else
	{
		reward=0;;
	}
	return reward;
}

Vec2i 	VAgent::transFunction(Action action,bool& obstacle) // -- Transition function
{

	obstacle=false;
	int agentX=agentPos[0];
	int agentY=agentPos[1];

	// -- Apply the action --
	switch (action)
	{

	case UP: 	agentY--;	break;
	case DOWN:	agentY++;	break;
	case LEFT:	agentX--;	break;
	case RIGHT:	agentX++;	break;
	case STAY:				break;
	}

	// -- Check boundaries and obstacles --
	if (agentX<0 || agentY<0 ||
			agentX>mapSize.width ||
			agentY>mapSize.height ||
			stateMap.at<uchar>(agentY,agentX)!=FREE_CELL)
	{
		obstacle=true;

		return agentPos;
	}

	return Vec2i(agentX,agentY);

}

Action 	VAgent::selectAction()	// -- Select action --
{

	// -- Take random action --

	if (!takeCorrectAction())
		return getRandomAction();

	// -- Take policy action --

	return getPolicyAction();

}

Action 	VAgent::getPolicyAction()		// -- Search for the best action from the learned Q-Values
{
	return (Action)policyMap.at<uchar>(agentPos[1],agentPos[0]);

}

void 	VAgent::stepUpdate()	// -- Step forward and update Q-Map
{

	Vec2i oldPos=agentPos;

	// -- Select an action--
	Action action=selectAction();

	// -- Take the action and get the reward --
	takeAction(action);

	// -- Update visit map
	visitMap.at<unsigned char>(oldPos(1),oldPos(0))++;	// -- Mark as visited

}

void 	VAgent::reset(Vec2i position)	// -- Reset position - total reward and visit map
{
	agentPos=position;
	visitMap=(Scalar)0.0;
}



