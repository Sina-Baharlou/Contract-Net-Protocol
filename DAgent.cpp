/*
 * DAgent.cpp
 *
 *  Created on: May 31, 2015
 *      Author: Sina M. Baharlou (Sina.Baharlou@gmail.com)
 */



#include "DAgent.hpp"

std::vector<DAgent*> DAgent::coWorkers;	// -- CO-workers

DAgent::DAgent(cv::Mat& stateMap,VAgent*opponent,Goal* goal):
stateMap(stateMap),
opponent(opponent),
goal(goal),
cnpOffer(NULL),
distToGoal(0),
agentPos(Vec2i(DEFAULT_POS)),
lastAction(RIGHT),
active(true),
greedyFactor(DEFAULT_GF),
correctFactor(DEFAULT_CF),
agentColour(Vec3b(DEFAULT_AGENT_COLOR))
{

	mapSize=cv::Size(stateMap.cols,stateMap.rows);

	// -- Create Visit Map
    
	visitMap=cv::Mat(stateMap.rows,stateMap.cols,CV_8UC1);
	visitMap=(cv::Scalar)0;


	// -- Initializing distribution variables --

	rndAction=std::uniform_int_distribution<int>(FIRST_ACTION,LAST_ACTION);
	rndReal=std::uniform_real_distribution<double>(0,1);



	// -- Init random engine --

	 struct timespec ts;
	 unsigned theTick = 0U;
	 clock_gettime( CLOCK_REALTIME, &ts );
	 theTick  = ts.tv_nsec / 1000000;
	 theTick += ts.tv_sec * 1000;
	 rndEngine.seed(theTick);


	// -- Add agent to coWorkers list --

	DAgent::coWorkers.push_back(this);

}



Action DAgent::getRandomAction()	// -- Get Random action
{
	return (Action)rndAction(rndEngine);
}


bool DAgent::takeGreedyAction()		// -- If agent should take a Greedy action ( Get closer to the opponent )
{

	double random=(double)rndReal(rndEngine);	// -- Get a double random number between 0 and 1

	// -- Check if the random number is below or equal the range
	if (random<=greedyFactor)
		return true;
	return false;
}

bool DAgent::takeCorrectAction()		// -- If agent should take a Correct action
{

	double random=(double)rndReal(rndEngine);	// -- Get a double random number between 0 and 1

	// -- Check if the random number is below or equal the range
	if (random<=correctFactor)
		return true;
	return false;
}

float DAgent::takeAction(Action action)	// -- Take the specified action --
{
	bool obstacle=false;
	float reward=0;

	agentPos=transFunction(action,obstacle); 		// -- Update the position

	distToGoal=getDist(agentPos,goal->goalPos);		// -- update distance to goal

	// -- Get the reward --
	if (obstacle)
	{
		reward=DEFAULT_OBSTACLE_REWARD;
	}
	else
	{
		reward=0;
	}
	return reward;
}

Vec2i DAgent::transFunction(Action action,bool& obstacle) // -- Transition function
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
			stateMap.at<uchar>(agentY,agentX)!=255)
	{
		obstacle=true;
		return agentPos;
	}

	if (getDist(Vec2i(agentX,agentY),goal->goalPos) <DISTANCE_TO_GOAL)
		return agentPos;

	// -- Check if the total sum of
	int totalDistance=getDist(Vec2i(agentX,agentY),goal->goalPos);

	for (int i=0;i<coWorkers.size();i++)
		if (coWorkers[i]!=this)
			totalDistance+=coWorkers[i]->distToGoal;

	if (totalDistance<TOTAL_DISTANCE_TO_GOAL)
		return agentPos;
	// --

	return Vec2i(agentX,agentY);

}

Action DAgent::selectAction()	// -- Select action --
{
	// -- Take random action --

	if (!takeCorrectAction())
		return getRandomAction();

	// -- Take policy action --

	return getBestAction();

}

float DAgent::getDist(Vec2i src,Vec2i dst)	// -- Get manhattan distance
{
	return abs(src[0]-dst[0]) +abs(src[1]-dst[1]);
}

Action DAgent::getBestAction()		// -- Search for the best action ( Reach the nearest axis of manhattan distance )
{

	// -------

	int& gX=goal->goalPos[0];
	int& gY=goal->goalPos[1];

	int& oX=opponent->agentPos[0];
	int& oY=opponent->agentPos[1];

	int& mX= agentPos[0];
	int& mY= agentPos[1];

	double dX=abs(gX-oX);
	double dY=abs(gY-oY);


	// ------

	if (dX>=dY)
	{
		if (mY==oY)		// -- It has already reached the specified position ( Stay or take a greedy action )
		{

			if (takeGreedyAction())	 // -- Take a greedy action ( go closer )
			{
				int dist=mX-oX;

				if (abs(dist)>MAX_GREEDY_DISTANCE && dist<0)
					return RIGHT;
				if (abs(dist)>MAX_GREEDY_DISTANCE && dist>0)
					return LEFT;

			}

			// -- STAY --
			return STAY;

		}

		if (mY-oY>0)
			return UP;
		return DOWN;
	}
	else
	{
		if (mX==oX)	// -- It has already reached the specified position ( Stay or take a greedy action )
		{

			if (takeGreedyAction())// -- Take a greedy action ( go closer )
			{
				int dist=mY-oY;

				if (abs(dist)>MAX_GREEDY_DISTANCE && dist<0)
					return DOWN;
				if (abs(dist)>MAX_GREEDY_DISTANCE && dist>0)
					return UP;

			}
			// -- STAY --
			return STAY;
		}

		if (mX-oX>0)
			return LEFT;
		return RIGHT;
	}

	return STAY;
}



void DAgent::stepUpdate()	// -- Step forward and update Q-Map
{
	if (opponent==NULL)
		return;

	Vec2i oldPos=agentPos;

	// -- Select an action--
	Action action=selectAction();
	// -- Take the action and get the reward --
	takeAction(action);



	// -- Update visit map
	visitMap.at<unsigned char>(oldPos(1),oldPos(0))++;	// -- Mark as visited

}

void DAgent::reset(Vec2i position)	// -- Reset position - total reward and visit map
{
	agentPos=position;
	visitMap=(Scalar)0.0;
}


// -- ***  CNP Methods *** --


bool DAgent::evalGoal(VAgent* opp,float& distance)
{

	float myDist=getDist(agentPos,goal->goalPos);
	float oppDist=getDist(opp->agentPos,goal->goalPos);
	distance=getDist(agentPos,opp->agentPos);

	if (myDist<oppDist-5)
		return true;

	return false;

}

 int DAgent::getBids(VAgent* opponent,bool onlyfree)
{

	int bestSupplier=-1;
	float minDist=std::numeric_limits<float>::max();

	// -- Find the best bid --
	for (int i=0;i<coWorkers.size();i++)
	{
		DAgent* supplier=coWorkers[i];

		if (onlyfree && supplier->opponent!=NULL)
			continue;
		// -- Skip if supplier has already an offer
		if (supplier->cnpOffer!=NULL)
		{

			continue;

		}
		// -- Check if the supplier is elibigle
		float suppDist=0;
		bool eligible=supplier->evalGoal(opponent,suppDist);
		cout<<" *** Bid #"<<i<<" is eligible : "<<eligible<<", Distance :"<<suppDist<<endl;
		// -- Set if it's the best
		if (eligible && suppDist<minDist)
		{
			minDist=suppDist;
			bestSupplier=i;
		}



	}
	return bestSupplier;

}


 void DAgent::startCNP()
{
	bool cnpSent=true;

	int index=0;
	int bestFinder=0;
	int indexCnp=0;
	while(cnpSent)
	{
#ifdef PRINT
		cout<<"-- -- -- Starting CNP :"<<index++<<" -- -- "<<endl;
#endif
		// -- Realize agent CNP Situation  --
		// -- 1- Have no tasks
		// -- 2- Have only one task
		// -- 	  2-1 Not eligible for current task
		// --	  2-2 Eligible for current task
		// --	  2-3 Eligible for Awarded Task
		// -- 3 - Have two tasks
		// -- 	  3-1 Eligible for both ( Start cnp for the first one )
		// -- 	  4-1 Not eligible for current task but eligible for awarded task
		indexCnp++;
		if (indexCnp>10)
			break;
		cnpSent=false;
		for (int i=0;i<coWorkers.size();i++)
		{
			DAgent* manager=coWorkers[i];

			VAgent* currentTask=manager->opponent;
			VAgent* cnpTask=manager->cnpOffer;

			// -- 1- Have no tasks
			if (currentTask==NULL && cnpTask==NULL)
			{
#ifdef PRINT
					cout<<"Agent :"<<i<<" has no tasks.. so skip it."<<endl;
#endif
				continue;
			}
			// -- 2- Have only one task
			if ( (currentTask!=NULL && cnpTask==NULL) || (currentTask==NULL && cnpTask!=NULL))
			{

#ifdef PRINT
				cout<<"Agent :"<<i<<" has one tasks.."<<endl;
#endif
				if (currentTask!=NULL) // -- Has current task
				{
					float distance=0;
					bool eligible=manager->evalGoal(currentTask,distance);

					if ( bestFinder <coWorkers.size() || !eligible) // -- 2-1 Not eligible for current task
					{
						bestFinder++;
#ifdef PRINT
						cout<<"Agent :"<<i<<" is not eligible for the task, listening for the bids..."<<endl;
#endif
						int bestSupplier=getBids(currentTask);

						if (bestSupplier==-1)
						{
#ifdef PRINT
							cout<<"No bids!"<<endl;
#endif
							continue;	  // -- No reply
						}
						// -- Replied
						coWorkers[bestSupplier]->cnpOffer=currentTask;
						manager->opponent=NULL;
						cnpSent=true;
#ifdef PRINT
						cout<<"Agent :"<<i<<" awarded his action to agent :"<<bestSupplier<<" CNP SENT"<<endl;
#endif
					}
					else
#ifdef PRINT
						cout<<"Agent :"<<i<<" is eligible for the task."<<endl;

#endif
					;
					//else		   // -- 2-2 Eligible for current task

				}
				else		// -- 2-3 Eligible for Awarded Task
				{
#ifdef PRINT
					cout<<"Agent :"<<i<<" take awarded action as his main action."<<endl;
#endif
					// -- Take it as current task
					manager->opponent=cnpTask;
					manager->cnpOffer=NULL;
				}

				continue;
			}


			// -- 3 - Have two tasks
			if (currentTask!=NULL && cnpTask!=NULL)
			{
#ifdef PRINT
				cout<<"Agent :"<<i<<" has two action. listetning for the bids for the first action..."<<endl;
#endif
				int bestSupplier=getBids(currentTask);

				if (bestSupplier==-1)
				{
#ifdef PRINT
					cout<<"No bids.. so take his own original action."<<endl;
#endif
					continue;	  // -- No reply Take cnp action ( or take the best )
				}


				// -- Replied
				coWorkers[bestSupplier]->cnpOffer=currentTask;
				manager->opponent=cnpTask;
				manager->cnpOffer=NULL;
				cnpSent=true;
#ifdef PRINT
				cout<<"Agent :"<<i<<" awarded his action to agent :"<<bestSupplier<<" CNP SENT"<<endl;
#endif

			}

		}

	}


}






