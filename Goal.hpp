/*
 * Goal.hpp
 *
 *  Created on: May 31, 2015
 *      Author: Sina M. Baharlou (Sina.Baharlou@gmail.com)
 */

#ifndef GOAL_HPP_
#define GOAL_HPP_

// -- Includes  --

#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <random>
#include <limits>

// -- OpenCV --

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace cv;
using namespace std;

// -- Prototypes --
class Goal
{
public:
	Vec2i goalPos;
	float goalValue;
	Goal(Vec2i goalPos,float goalValue):goalPos(goalPos),goalValue(goalValue){}
};



#endif /* GOAL_HPP_ */
