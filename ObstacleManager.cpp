/*
 * ObstacleManager.cpp
 *
 *  Created on: May 31, 2015
 *      Author: Sina M. Baharlou (Sina.Baharlou@gmail.com)
 */




#include "ObstacleManager.hpp"


ObstacleManager::ObstacleManager(cv::Mat& stateMap):stateMap(stateMap)
{
	// -- Get state map dimension --
	mapW=stateMap.cols;
	mapH=stateMap.rows;
	// --
	stateMap.copyTo(finalMap);
}
void ObstacleManager::addObstacle(Vec2i* pos)
{
	obstacles.push_back(pos);
}

void ObstacleManager::clearObstacles()
{
	obstacles.clear();
}
cv::Mat& ObstacleManager::updateMap()
{
	stateMap.copyTo(finalMap);

	for (int i=0;i<obstacles.size();i++)
	{
		int x=obstacles[i]->val[0];
		int y=obstacles[i]->val[1];

		if (0<=x && x<mapW && 0<=y && y<mapH)
			finalMap.at<uchar>(y,x)=OBSTACLE_CELL;

	}

	return finalMap;
}


