/*
 * VAgent.hpp
 *
 *  Created on: May 26, 2015
 *      Author: Sina M. Baharlou (Sina.Baharlou@gmail.com)
 */

#ifndef VAGENT_HPP_
#define VAGENT_HPP_


// -- Include --

#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <random>
#include <limits>

// -- Opencv --

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "Actions.hpp"
#include "Goal.hpp"

using namespace std;
using namespace cv;



// -- Definitions --

#define DEFAULT_POS 2,2
#define DEFAULT_CF	0.99
#define DEFAULT_AGENT_COLOR 255,0,0
#define DEFAULT_OBSTACLE_REWARD 0
#define CELL_AGENT 22  
#define FREE_CELL 255


// -- Prototypes --

class VAgent
{

public:

	cv::Mat stateMap;						// -- States
	cv::Mat visitMap;						// -- Visit map
	cv::Mat policyMap;						// -- Policy map
	Vec2i agentPos;							// -- Agent Position (State)
	cv::Size mapSize;						// -- Domain size

	// -- Distributions --
	typedef std::default_random_engine RndEngine;
	std::uniform_int_distribution<int> rndAction;	// -- sampling from integer uniform  distribution for setting random states
	//std::uniform_int_distribution<int> rndState;	// -- sampling from integer uniform  distribution for taking random action
	std::uniform_real_distribution<double> rndReal; // -- sampling from float uniform  distribution for other probability task
	RndEngine rndEngine;							// -- random variable generator engine


	// --  Rewards and last action --

	Action lastAction;

	// -- Rates --

	float correctFactor;

	// -- Agent characteristics

	Vec3b agentColour;
	
    bool active;

	VAgent(){}
	
	VAgent(cv::Mat& stateMap,cv::Mat& policyMap);

	Action getRandomAction();	// -- Get Random action

	bool takeCorrectAction();		// -- If agent should take a correct action

	float takeAction(Action action);	// -- Take the specified action --

	Vec2i transFunction(Action action,bool& obstacle); // -- Transition function

	Action selectAction();	// -- Select action --

	Action getPolicyAction();		// -- Search for the best action from the learned Q-Values

	void stepUpdate();	// -- Step forward and update Q-Map
	
	void reset(Vec2i position);	// -- Reset position - total reward and visit map
	
	
};



#endif /* VAGENT_HPP_ */
