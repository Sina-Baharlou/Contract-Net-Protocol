//============================================================================
// Name        : QLearning.cpp
// Author      : Sina M.Baharlou
// Version     :
// Copyright   :
// Description :
//============================================================================

#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <random>

#include <limits>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "Actions.hpp"
#include "Goal.hpp"
#include "VAgent.hpp"
#include "DAgent.hpp"
#include "ObstacleManager.hpp"
#include "PolicyManager.hpp"



using namespace std;
using namespace cv;

#define AGENT_COUNT 5
#define MAP_FILE "map.png"
#define GOAL_POSS 17,30

VAgent *agents;
DAgent** dAgent;
std::vector<DAgent*> dAgents;
std::vector<VAgent*> vAgents;


cv::Mat enlargeMat(cv::Mat input,float scaleFactor,int interpolation=CV_INTER_NN)
{
	cv::Mat output;
	cv::Size size(input.rows*scaleFactor,input.cols*scaleFactor);

	cv::resize(input,output,size,0,0,interpolation);
	return output;
}


int main()
{

	// -- Load default map file --

	cv::Mat stateMap=imread(MAP_FILE,1);
	cv::cvtColor( stateMap, stateMap, CV_BGR2GRAY );

	// -- Define a Goal

	Goal myGoal(Vec2i(GOAL_POSS),2.1);

	// -- Create Obstacle Manager --
	ObstacleManager obsManager(stateMap);

	// -- Create Policy Manager --
	PolicyManager pManager(obsManager.finalMap);
	pManager.addGoals(&myGoal);



	// -- Create Agents --

	for (int i=0;i<AGENT_COUNT;i++)
	{
		// -- Value-iteration agents--
		VAgent* vAgent=new VAgent(obsManager.finalMap,pManager.pMap);
		vAgent->agentPos=Vec2i(4+rand()%26,4+rand()%10);
		obsManager.addObstacle(&vAgent->agentPos);

		vAgents.push_back(vAgent);


		// -- Defender Agents --
		DAgent* dAgent=new  DAgent(obsManager.finalMap,vAgent,&myGoal);
		dAgent->agentPos=Vec2i(4+rand()%26,15+rand()%10);
		dAgent->agentColour=Vec3b(0,0,200);
		obsManager.addObstacle(&dAgent->agentPos);

		dAgents.push_back(dAgent);
	}


	// -- Start CNP --

	DAgent::startCNP();


	// -- Update Map --

	obsManager.updateMap();
	pManager.valueIterate();
	pManager.updatePolicy();
	int nFinish=0;
	int elapsedTime=0;



	while(true)
	{
		elapsedTime++;
		for (int i=0;i<AGENT_COUNT;i++)
		{

			VAgent* vAgent=vAgents[i];
			DAgent* dAgent=dAgents[i];

			// -- Update VAgent --
			if (vAgent->active)
			{
				vAgent->stepUpdate();
				obsManager.updateMap();
			}

			// -- Update DAgent --
			if (dAgent->active)
			{
				dAgent->stepUpdate();
				obsManager.updateMap();
			}

			// -- Check if agent reaches the goal --
			if (vAgent->agentPos==myGoal.goalPos)
			{

				vAgents[i]->active=false;
				vAgents[i]->agentPos=Vec2i(3,i+2);
				nFinish++;
				for (int l=0;l<AGENT_COUNT;l++)
					if (dAgents[l]->opponent==vAgents[i])
					{
						//dAgents[l]->opponent=NULL;
						dAgents[l]->active=false;
						dAgents[l]->agentPos=Vec2i(28,l+2);

						
					}

				if (nFinish==AGENT_COUNT)
				{

					for (int i=0;i<AGENT_COUNT;i++)
						{

						VAgent* v=vAgents[i];
						DAgent* d=dAgents[i];

						v->active=true;
						d->active=true;
						v->reset(Vec2i(4+rand()%25,4+rand()%10));
						d->reset(Vec2i(4+rand()%25,15+rand()%10));

						}
					//cout<<"Round ended..."<<endl;
					nFinish=0;
					DAgent::startCNP();
					cv::waitKey(100);
				}

			}


			pManager.valueIterate();
			pManager.updatePolicy();
		}


		// -- Render agents and goal

		cv::Mat finalRender;
		stateMap.copyTo(finalRender);
		cv::cvtColor( finalRender, finalRender, CV_GRAY2BGR );

		for (int i=0;i<AGENT_COUNT;i++)
		{
			VAgent* vAgent=vAgents[i];
			DAgent* dAgent=dAgents[i];

			finalRender.at<cv::Vec3b>(vAgent->agentPos(1),vAgent->agentPos(0))=vAgent->agentColour;
			finalRender.at<cv::Vec3b>(dAgent ->agentPos(1),dAgent->agentPos(0))=dAgent->agentColour;
		}

		// -- render goal --
		finalRender.at<cv::Vec3b>(myGoal.goalPos[1],myGoal.goalPos[0])=Vec3b(0,255,0);

		cv::imshow("State Map",enlargeMat(finalRender,10,CV_INTER_CUBIC ));


		int key=cv::waitKey(1);



	}
	return 0;
}

