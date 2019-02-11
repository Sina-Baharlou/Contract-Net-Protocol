/*
 * ObstacleManager.hpp
 *
 *  Created on: May 31, 2015
 *      Author: Sina M. Baharlou (Sina.Baharlou@gmail.com)
 */

#ifndef OBS_HPP_
#define OBS_HPP_

// -- Includes --

#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <random>
#include <limits>

// -- OpenCV --

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace std;
using namespace cv;

// -- Definitions --

#define FREE_CELL 255
#define OBSTACLE_CELL 0

// -- Prototypes --


class ObstacleManager
{


public:
	cv::Mat stateMap;
	cv::Mat finalMap;
	int mapW;
	int mapH;

	std::vector<Vec2i*> obstacles;

	ObstacleManager(cv::Mat& stateMap);

	void addObstacle(Vec2i* pos);

	void clearObstacles();

	cv::Mat& updateMap();

};

#endif /* OBS_HPP_ */
