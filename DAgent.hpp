/*
 * DAgent.hpp
 *
 *  Created on: May 26, 2015
 *      Author: Sina M. Baharlou (Sina.Baharlou@gmail.com)
 */

#ifndef DAGENT_HPP_
#define DAGENT_HPP_


// -- Includes  --

#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <random>
#include <limits>

// -- OpenCV --

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "Actions.hpp"
#include "VAgent.hpp"
#include "Goal.hpp"


using namespace std;
using namespace cv;

// -- Definitions --


#define DEFAULT_POS 2,2
#define DEFAULT_GF	0.8
#define DEFAULT_CF	0.95
#define DEFAULT_AGENT_COLOR 255,0,0
#define DEFAULT_OBSTACLE_REWARD 0
#define CELL_AGENT 22  
#define FREE_CELL 255

#define DISTANCE_TO_GOAL 2
#define TOTAL_DISTANCE_TO_GOAL 15
#define MAX_GREEDY_DISTANCE 1
#define PRINT



class DAgent
{

public:

	cv::Mat stateMap;						// -- States
	cv::Mat visitMap;						// -- Visit map

	Vec2i agentPos;							// -- Agent Position (State)
	cv::Size mapSize;						// -- Domain size



	// -- CNP Parameters
	VAgent *opponent;		// -- current task
	VAgent *cnpOffer;		// -- awarded task
	Goal* goal;				// -- current goal
	static std::vector<DAgent*> coWorkers;	// -- CO-workers

	// -- Distributions --
	typedef std::default_random_engine RndEngine;
	std::uniform_int_distribution<int> rndAction;	// -- sampling from integer uniform  distribution for setting random states
	std::uniform_int_distribution<int> rndState;	// -- sampling from integer uniform  distribution for taking random action
	std::uniform_real_distribution<double> rndReal; // -- sampling from float uniform  distribution for other probability task
	RndEngine rndEngine;							// -- random variable generator engine


	// --  Rewards and last action --

	Action lastAction;
	int distToGoal;

	// -- Rates --

	float greedyFactor;
	float correctFactor;

	// -- Agent characteristics

	Vec3b agentColour;
	bool active;

	DAgent(){}
	DAgent(cv::Mat& stateMap,VAgent*opponent,Goal* goal);


	Action getRandomAction();	// -- Get Random action

	bool takeGreedyAction();		// -- If agent should take a Greedy action ( Get closer to the opponent )

	bool takeCorrectAction();		// -- If agent should take a Correct action

	float takeAction(Action action);	// -- Take the specified action --

	Vec2i transFunction(Action action,bool& obstacle); // -- Transition function

	Action selectAction();	// -- Select action --

	float getDist(Vec2i src,Vec2i dst);	// -- Get manhattan distance

	Action getBestAction();		// -- Search for the best action ( Reach the nearest axis of manhattan distance )

	void stepUpdate();	// -- Step forward and update Q-Map

	void reset(Vec2i position);	// -- Reset position - total reward and visit map

	bool evalGoal(VAgent* opp,float& distance); // -- evaluate goal

	static int getBids(VAgent* opponent,bool onlyfree=false);

	static void startCNP();

};



#endif /* DAGENT_HPP_ */
