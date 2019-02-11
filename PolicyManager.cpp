/*
 * PolicyManager.cpp
 *
 *  Created on: May 31, 2015
 *      Author: Sina M. Baharlou (Sina.Baharlou@gmail.com)
 */


#include "PolicyManager.hpp"

PolicyManager::PolicyManager(cv::Mat& stateMap):stateMap(stateMap)
{
	// -- Get state map dimension --
	mapW=stateMap.cols;
	mapH=stateMap.rows;

	// -- Create Value-Map --
	vMap=cv::Mat(mapH,mapW, CV_32FC1);
	vMap=0.0;

	// -- Create Policy-Map --

	pMap=cv::Mat(mapH,mapW, CV_8UC1);
	pMap=(Scalar)0;
}

void PolicyManager::addGoals(Goal* goal)
{
	goals.push_back(goal);
}

void PolicyManager::updateStateMap(cv::Mat& stateMap)
{
	this->stateMap=stateMap;
}

void PolicyManager::updatePolicy()		// -- Create best policy map
{

	for (int i=0;i<mapW;i++)
		for (int j=0;j<mapH;j++)
		{

			Action bestAction=RIGHT;

			float vMax=-std::numeric_limits<float>::min();

			float vTemp;

			bool obstacle=false;

			// -- Search --

			for (int a=FIRST_ACTION;a<=LAST_ACTION;a++)
			{
				Vec2i newPos=transFunction(Vec2i(i,j),(Action)a,obstacle);

				vTemp=vMap.at<float>(newPos[1],newPos[0]);

				if ( vTemp>vMax)
				{
					vMax=vTemp;
					bestAction=(Action)a;
				}
			}

			pMap.at<uchar>(j,i)=bestAction;

		}

}


cv::Mat& PolicyManager::getPolicy()
{
	return pMap;
}

void PolicyManager::valueIterate()
{
	vMap=0.0;

	for (int vIndex=0;vIndex<VALUE_ITERATION_COUNT;vIndex++ )
	{

		// -- Create temporary vMap
		cv::Mat vMapTemp;
		vMap.copyTo(vMapTemp);
		// --

		for (int i=0;i<mapW;i++)
			for (int j=0;j<mapH;j++)
			{
				// -- Check whether the cell is a state or obstacle
				if (stateMap.at<uchar>(j,i)==OBSTACLE_CELL)continue;

				// -- Find Max V value
				float vMax=-std::numeric_limits<float>::min();
				float vTemp=0;
				for (int action=FIRST_ACTION;action<LAST_ACTION;action++)
				{
					float reward=0;
					bool obstacle=false;
					Vec2i newState=transFunction(Vec2i(i,j),(Action)action,obstacle);

					// -- Get goal reward --
					for (int g=0;g<goals.size();g++)
						if (goals[g]->goalPos[0]==i && goals[g]->goalPos[1]==j)
							reward+=goals[g]->goalValue;

					// --

					vTemp=reward+DISCOUNT_FACTOR *vMapTemp.at<float>(newState[1],newState[0]);
					if (vTemp>vMax)vMax=vTemp;
				}
				// --

				vMap.at<float>(j,i)=vMax;
			}

	}
}


Vec2i PolicyManager::transFunction(Vec2i agentPos,Action action,bool& obstacle) // -- Transition function
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
			agentX>mapW ||
			agentY>mapH ||
			stateMap.at<uchar>(agentY,agentX)!=FREE_CELL)
	{
		obstacle=true;
		return agentPos;
	}

	return Vec2i(agentX,agentY);

}





