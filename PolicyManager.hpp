/*
 * PolicyManager.hpp
 *
 *  Created on: May 31, 2015
 *      Author: Sina M. Baharlou (Sina.Baharlou@gmail.com)
 */

#ifndef POLICYMANAGER_HPP_
#define POLICYMANAGER_HPP_


// -- Includes --

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

#define FREE_CELL 255
#define OBSTACLE_CELL 0
#define VALUE_ITERATION_COUNT 45
#define DISCOUNT_FACTOR 0.90

// -- Prototypes --


class PolicyManager
{

public :

	cv::Mat stateMap;
	cv::Mat vMap;		// -- Value Iteration map
	cv::Mat pMap;		// -- Best policy map
	int mapW;
	int mapH;


	std::vector<Vec2i*> obstacles;
	std::vector<Goal*> goals;

	PolicyManager(cv::Mat& stateMap);

	void addGoals(Goal* goal);

	void updateStateMap(cv::Mat& stateMap);

	void updatePolicy();	// -- Create best policy map

	cv::Mat& getPolicy();

	void valueIterate();


	Vec2i transFunction(Vec2i agentPos,Action action,bool& obstacle); // -- Transition function

};

#endif /* POLICYMANAGER_HPP_ */
