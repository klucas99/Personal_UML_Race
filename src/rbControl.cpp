/**
rbControl.cpp is intended to communicate with uml_race's racetrack.launch.
Currently, it publishes a Twist, which allows for movement, and subscribes to a LaserScan.
Both publication and subscription work.

Issues to be fixed: 
* How can the information given by 'ranges' be used? I want the robot to head towards
the highest values in the 'ranges'; how does one map that? (rviz? can rviz be used in a 2D simulation?)

* How can I used controlCallback to modify my Twist object? Currently creating Twist (& its Vector3  objects) inside of main's while-loop, which is outside of the scope of controlCallback. Globals? Would moving these files outside of while-loop affect publication? What if I moved them outside of main entirely?

Addtional notes:
- 'ranges' member is a vector of float, can be used like vector of floats. Research floats, vectors in C++, ROS. Specifically: std::vector<float>. Do I need to convert this to work with it?
-http://stackoverflow.com/questions/4289612/getting-array-from-stdvector
- Possible process for controlCallback:
	-Access vector of floats
	-Convert to array? <- possibly not needed
	-Iterate through array/vector, store greatest value
	-Use stored value to change angular


**/

/**
Notes as of 23 June 2016: at a point where, if left running all day(s), would probably finish eventually. Navigates straight line v. successfully; difficulty with the first sharp turn. May need to allow robot to get closer to wall to take turns properly?

minEl at start of uml_race is 1.94
*/

/**
Notes as of June 24: Robot is v. close to being able to complete. Major problem is @ start; 
variable turning speeds lead to a slow left turn followed by a sharp right, running rb into wall. back-tracking software eventually gets it out of these jams, but pref. to avoid. adjust turn so start ~1.74 ish? more different turn speeds?

while not as smooth as past travel patterns, current is fastest & gets out of (most) jams effectively (eventually). 
*/

/*
Notes as of June 27: Sent self version that runs maze. Currently, trying to get rbt to run maze in under 70s (big step down from 4min). To accomplish, have sped up general speeds. However, speeding the robot up means less time to detect walls. Robot is mostly fine, but tends to bounce & get turned around at 90 degree turns. Slowing down robot around curves could help with this, but would affect traveling speed: "snaking" motion depends on quick turns.

As the robot has gone faster, its curves have tightened. Ideally, "snaking" would involve a wide, fast turn, and going around corners would be tight but slow. However, currently seems to be reversed: "snaking" is fast & tight, rounding corners is too slow (leads to occasional crashing).

Need to loosen up curves.

Could try working off central angle for speed/turning?

Big problem w. crashing is not crashing itself, it's that crashing tends to send the robot off in the wrong direction. Try to recalculate direction robot should move after crash.

Ex:

start controlCallback{

chgDr //some bool variable to determine how to calc. dir

if (min..) //greatest
{go straight}

else if (min..) //ranges
{
	if(chgDr true)
	{current calc}
	else(false)
	{diff. calc (base off cen. ang?)}
}

else if (min..) //min. val of min
{ change value of chgDr }

}//end cCall

end Ex

What this would have to do is to prevent backtracking. Figure out how to do this math.

For "snaking": see previously working version of code. Why did that one run "looser"? (Did slower speeds mean that we approached the walls more slowly, allowing for more time to move away from the walls? Could we replicate this at faster speeds? Would starting the robot at an angle (starting turns at 1.94) mean we could try shallower angles?)

Look up how we could trigonometrically determine where we've already been?

Some difficulty reading/interpreting curr. output. think of a way to more clearly restructure it.

All in all, v successful day.
*/

/*
Notes June 28:

Had a very sucessful run, up until last 90 degree turn. Consistently turns left at this turn when should be turning right; turns straight into wall. Check sideways for wall? 

There is a problem checking for walls: left element, lsMsg.ranges[0] seems to be relatively accurate, but difficulty obtaining accurate right element. Adding it to for-loop to see if that obtains it better?

Right and left were reversed. Correcting: right = lsMsg.ranges[0], left for-loop


Curr ver: ran mze in 3 min. However, there are pauses where robot freezes & no output given. Figure out when these are occuring & why. Usually when approaching a wall w. relatively similar rightEl, leftEl. What would solve this?

Other issue is rb occationally gets stuck bouncing. (Turn during backing away?)
*/

/*
Notes June 30:
Well, abandoning past versions entirely. doubtful if they could be moved under 70s. New, follow-wall algorithm
to be implemented.

Implemented a following-wall algorithm. Following subsections:
-if minEl is too small, robot should back up
-if near ideal position (1.94 units from right), drive straight
-if leftEl or rightEl are too great - robot is leaving walls; should turn towards walls
-otherwise, turn based off of right wall

Has most difficulty w. sharp turns. left/rightEl too great code helps w. this; not perfect solution. Gets too close to wall; follows it straight into itself. cenEl is prob. best solution for this; should not affect regular travel, when cenEl should be approx. 5-3 units. Set up display for cenEl, determine a possible value for statement.
*/


#include "ros/ros.h"
//allows for primary code from ros project
#include "std_msgs/String.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
//standard String data

#include <sstream>
//standard c++ include; String concatination, etc
//String functionality

//#include <algorithm>
//#include <vector>
//#include <array>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */

	geometry_msgs::Twist msg;

	geometry_msgs::Vector3 ang;
	geometry_msgs::Vector3 lin;
        
	//bool angDr = true; //positive/true/left, negative/false/right
	//bool chngAg = false; //you only want to change the angle' direction as you switch distances

void controlCallback(const sensor_msgs::LaserScan lsMsg)
{
  //ROS_INFO("I heard that!");
	
	// minimum element in ranges
	double minEl = lsMsg.ranges[0];
	//index of minimum element
	int minIndex = 0;
	//element in center of ranges; directly in front of rbt.
	double cenEl = lsMsg.ranges[(lsMsg.ranges.size()/2)];
	//rightmost element in ranges; element directly to right of rbt.
	double rightEl = lsMsg.ranges[0];
	//leftmost element in ranges; element directly to left of rbt.
	double leftEl = 0.0;
	//theta of minimum angle
	double minAg = 0.0;
	//ideal distance from wall
	double idealDis = 1.94;
	
	//find index of minimum element, minEl
	for (int i = 0; i < lsMsg.ranges.size(); i++)
	{
		if(lsMsg.ranges[i] < minEl)
		{
			//obtain index of min, min
			minIndex = i;
			minEl = lsMsg.ranges[i];			
		}
		//iterate through to find leftmost element
		leftEl = lsMsg.ranges[i];
	}
	
	//calculate minAg: minIndex is number of elements to go through to get to
	//the min, angle_increment represents how much theta increases per element
	//shouldn't this be: minAg = (minIndex + 1) * lsMsg.angle_increment; instead?

	minAg = minIndex * lsMsg.angle_increment;

	//follow-wall formula
	//base off right wall

	//If you are already too close to the wall, start backing up
	if (minEl <= 0.70)
	{	
		ROS_INFO("Min:%f,AngularSpeed:%f, Cen:%f, Left:%f, Right:%f",minEl,ang.z,cenEl,leftEl,rightEl);	
		lin.x = -3;
		//lin.x = -5;
		ang.z = 0;
		//ang.z = (idealDis - rightEl) * 1.5;
	}
	//if equals, straight
	//else if ( rightEl >= (idealDis - 0.05) && rightEl <= (idealDis + 0.10))
	//if you fall in a short range around the ideal distance, go straight quickly
	else if ( rightEl >= (idealDis - 0.10) && rightEl <= (idealDis + 0.10) && cenEl >= 3.0)
	//else if ( rightEl >= (idealDis - 0.15) && rightEl <= (idealDis + 0.15) && cenEl >= 3.0)
	{ ang.z = 0; lin.x = 4;
	  ROS_INFO("Straight. Cen: %f, Left: %f, Right: %f", cenEl, leftEl,rightEl);	
	}// end else if ( rightEl >= (idealDis - 0.10) && rightEl <= (idealDis + 0.10) && cenEl >= 3.0)
		//if rightEl = range_max, rotate robot to right
		//if(rightEl == lsMsg.range_max){ ang.z = 1; lin.x = 0.5;}
	//else if (rightEl == lsMsg.range_max && leftEl == lsMsg.range_max)
	//else if (rightEl>=3.6&&leftEl>=3.6||rightEl >= 4.8&&cenEl<=3.6||leftEl >= 4.8&&cenEl<=3.6)
/*
	else if (rightEl>=3.6&&leftEl>=3.6)
	//else if (rightEl >= 3.0 && leftEl >= 4.8 || leftEl >= 3.0 && rightEl >= 4.8 ) 
	//else if (rightEl >= 2.0 && leftEl >= 4.8 || leftEl >= 2.0 && rightEl >= 4.8 )
	{ lin.x = minEl - 0.4;
	  ang.z = -(idealDis - rightEl) * 1.5;
	  //ang.z = -(idealDis - rightEl) * 1.25;
	  ROS_INFO("Slow. Cen: %f, Left: %f, Right: %f", cenEl, leftEl,rightEl);
	}	
*/
	//else, change by (idealDis - rightEl)
	//If you are not moving straight or backing up, turn
	else
	{ //ang.z = (idealDis - rightEl) * 1.5; lin.x = 1;
	
		//If robot is much too far from ideal distance
		if (minEl <= idealDis - 0.25) //if (minEl <=1.50)
		{ 
		  //Base speed off of cenEl; lots of space, go fast, little space slow
		  //If you are very close to running straight into the wall, slow movement, fast turn
		  //if (cenEl < 2.5 && minEl > 1.20){lin.x = minEl; ang.z = (idealDis - rightEl) * 2.0;}
		  //else if (cenEl < 2.5){lin.x = minEl- 0.3; ang.z = (idealDis - rightEl) * 2.0;}
		//if (cenEl < 2.5){lin.x = minEl- 0.3; ang.z = (idealDis - rightEl) * 1.75;} //lin.x = minEl- 0.3
		  //else if(cenEl < 2.5){lin.x = minEl - 0.5;}
		  //If you are close to running straight into the wall
		//else if(cenEl < 3.5){lin.x = minEl;ang.z = (idealDis - rightEl) * 1.25;}
		//else if(cenEl < 3.5){lin.x = minEl;ang.z = (idealDis - rightEl) * 1.25;}// -1;}//lin.x=minEl- 0.4;}
		  //if(cenEl < 3){lin.x = minEl - 1;}


		  if (cenEl < 3){lin.x = minEl; ang.z = (idealDis - rightEl) * 1.75;} //lin.x = minEl- 0.3
		  //If you are close to running straight into the wall
		  else if(cenEl < 3.5 && minEl > 2.0){lin.x = cenEl;ang.z = (idealDis - rightEl) * 1.5;}
		  else if(cenEl < 3.5){lin.x = minEl;ang.z = (idealDis - rightEl) * 1.25;}
		  //Otherwise:
		  else { lin.x = 4; ang.z = (idealDis - rightEl) * 1.25;} //lin.x = 2.5;}
		  //else { lin.x = 3; ang.z = (idealDis - rightEl) * 1.25;} //lin.x = 2.5;}
		  //lin.x = minEl - 0.4;
		  //ang.z = (idealDis - rightEl) * 1.5;	
		  //ang.z = (idealDis - rightEl) * 1.25;

		  //Calculate angle w. a special case: if opposite side of (idealDis - rightEl) is long,
		  //reverse direction: -(idealDis - rightEl)
		  //If (idealDis - rightEl) is negative, pre-flip right turn; postive left
		  if( ang.z > 0 && rightEl >= 4.8 || ang.z < 0 && leftEl >= 4.8)
		  //if we are turning left & there is space on right or vice versa
		  { ang.z = -ang.z; } //flip direction
//part of if (minEl <= 1.50)
ROS_INFO("Tight turning. Speed:%f, Angular Speed:%f, Cen:%f, Left:%f, Right:%f",lin.x,ang.z,cenEl,leftEl,rightEl);
		}//end if (minEl <= 1.50)
		//Otherwise, if closer to wall, turn more gently
		else
		{ //lin.x = 2;
		  //lin.x = 3;
		  //Base speed off of cenEl; lots of space, go fast, little space slow
		  //if(cenEl < 2.0){lin.x = minEl;}
		  //If you are headed straight for the wall (is it even possible to get into this?)
		  if(cenEl < 1.2){lin.x = minEl;}
		  //Otherwise, if you are farther from the wall, go faster
		  else { lin.x = 4; }//3; }
		  ang.z = (idealDis - rightEl) * 0.75;
		  //ang.z = (idealDis - rightEl) * 0.5;
		  //ang.z = (idealDis - rightEl) * 0.25;
		  //ang.z = (idealDis - rightEl) * 3;
		
		  //Calculate angle w. a special case: if opposite side of (idealDis - rightEl) is long,
		  //reverse direction: -(idealDis - rightEl)
		  //If (idealDis - rightEl) is negative, right turn; postive left
		  if( ang.z > 0 && rightEl >= 4.8 || ang.z < 0 && leftEl >= 4.8)
		  //if we are turning left & there is space on right or vice versa
		  { ang.z = -ang.z; } //flip direction

		ROS_INFO("Wide turning. Angular Speed:%f, Cen:%f, Left:%f, Right:%f",ang.z,cenEl,leftEl,rightEl);
		}//end else to (minEl <= idealDis - 0.25)
	}//end else


//This is original, "snaking"/turning away from wall code. We're going to try for a complete redo here.	
/*
	//A series of if statements determining the speed & angle at which the robot travels.
	//These are based off of the minimum element in ranges. Min el is greatest crash risk; ergo,
	//makes sense to base speed off it.

	//if(cenEl >= 3.00 && minEl >= 1.94)

	//Gretest category: travelling straight forward.
	//@ start race, minEl is at 1.94, so straight travel.
	if(minEl >= 1.90)
	{//ROS_INFO("%f >= 1.74, will be turning %s", minEl, angDr?"left":"right");
	// ROS_INFO("%f >= 3.50, will be turning %s", cenEl, angDr?"left":"right");	
		ROS_INFO("%f >= 1.90,  Left: %f, Right: %f", minEl, leftEl,rightEl);	
		lin.x = 3; //ang.x = 1;
		ang.z = 0; //moving straight	
		//if (leftEl > rightEl) { ang.x = 0.1;}
		//else if (leftEl < rightEl) { ang.x = -0.1;}	
	}
	//else if (3.00 > cenEl && minEl > 0.55 || minEl < 1.94)
	
	//else if from June 27

	//Crash at approx. 60; 70 gives some more buffer.
	//This section does include various angular speeds.
	else if (1.90 > minEl && minEl > 0.70)
	{	
		//This calculation deterimines if the object the robot is detecting @ minEl
		//is to the left of the rbt or to the right. It should return a positive value if
		//the wall is to the right, or a negative value if the wall is to the left.
		//The robot should move in the opposite direction of the wall: if the wall is to
		//the left, the robot should turn right, and vice versa.
		if( minEl*cos(minAg) >= 0 ) // positive //on right, so turn left
		{
		    //Occationally, there are issues where, when the rbt's beam is split, where the above calculation
		    //does not determine the best move pattern for the rbt. Here, we say that the robot, having 
		    //detected a wall on the left, should only move to the right if there is move room on the rbt's right
		    // than its left.
		    //Shouldn't this be: if( rightEl >= leftEl)
		    //if( leftEl >= rightEl)// "not declared in this scope?" ORIGINAL
		    if( rightEl >= leftEl)
		    {
			//A series of if statements determining the robot's speed. Linear should increase,	
			//angular should decrease.	
			if(1.90 > minEl && minEl > 1.84)
			{
				ang.z = 0.5;
				lin.x = 3;
			ROS_INFO("1.90 > %f > 0.70, turning left widest. Left: %f, Right: %f", minEl,leftEl,rightEl);	
			}
			else if(1.84 >= minEl && minEl > 1.64)
			{
				ang.z = 1.5;
				lin.x = 2.5;
			ROS_INFO("1.90 > %f > 0.70, turning left tight. Left: %f, Right: %f", minEl,leftEl,rightEl);	
			}  
			else if(1.64 >= minEl && minEl > 1.20)
			{
				ang.z = 3;
				lin.x = 2;
			ROS_INFO("1.90>%f>0.70, turning left sharp fast. Left: %f, Right: %f", minEl,leftEl,rightEl);	
			}
			else
			{
				ang.z = 2.5;
				lin.x = 2;
			ROS_INFO("1.90>%f>0.70, turning left sharpest. Left: %f, Right: %f", minEl,leftEl,rightEl);
			} 
		     }//end if( lE > rE)
		//if there is less space on the side where the robot is supposed to turn compared to the side
		//where it is supposed to turn, checking needs to occur.
		//Take several points from each side to figure out where there is more space?
		//--risks backtracking.
		     else
		     {//ang.z = -0.1; lin.x = 0.1;
		      //ang.z = 0.3; 
		      //ROS_INFO("Wall on Right, Left: %f, Right: %f",leftEl,rightEl);

			//determine linear speed based off of minEl
			if ( minEl < 1.20) 
	                { lin.x = 0.5;}
	        	else
		        { lin.x = 2; }

			//determine angle.
			//Currently, checks according to distance. If half of the the left side is greater than the
			//entire right side, then the robot should still turn to the left; else, it should turn to the right.
			//But shouldn't this be reversed?

			//if ( (leftEl/2) >= rightEl) 
		        //{
			//	ang.z = 1.5;
			//	ROS_INFO("Wall on right, turning left, Left: %f, Right: %f",leftEl,rightEl);
		        //}
		        //else 
		        //{
			//	ang.z = -1.5;
			//	ROS_INFO(" Wall on right, turning right, Left: %f, Right: %f",leftEl,rightEl);
		     	//}
			//REVERSED
			if ( (rightEl/2) >= leftEl) 
		        {
				ang.z = 1.5;
				ROS_INFO("Wall on right, turning left, Left: %f, Right: %f",leftEl,rightEl);
		        }
		        else 
		        {
				ang.z = -1.5;
				ROS_INFO(" Wall on right, turning right, Left: %f, Right: %f",leftEl,rightEl);
		     	}
		     }					
		}//turn left
		else if( minEl*cos(minAg) < 0 ) // negative //on left, so turn right
		{
		    if( rightEl >= leftEl)
		    {
			//ang.z = 0.5;
			if(1.90 > minEl && minEl > 1.84)
			{
				ang.z = -0.5;
				lin.x = 3;
			ROS_INFO("1.90 > %f > 0.70, turning right widest. Left: %f, Right: %f", minEl,leftEl,rightEl);	
			}
			else if(1.84 >= minEl && minEl > 1.64)
			{
				ang.z = -1.5;
				lin.x = 2.5;
			ROS_INFO("1.90 > %f > 0.70, turning right tight. Left: %f, Right: %f", minEl,leftEl,rightEl);	
			}  
			else if(1.64 >= minEl && minEl > 1.20)
			{
				ang.z = -3;
				lin.x = 2;
			ROS_INFO("1.90>%f>0.70, turning right sharp fast. Left: %f, Right: %f", minEl,leftEl,rightEl);	
			}
			else
			{
				ang.z = -2.5;
				lin.x = 2;
			ROS_INFO("1.90>%f>0.70, turning right sharpest. Left: %f, Right: %f", minEl,leftEl,rightEl);
			} 
		    }//end if (rE > lE)	
		    else
		    {//ang.z = -0.1; lin.x = -0.1;
		     //determine linear speed
		     if ( minEl < 1.20) 
		     { lin.x = 0.5;}
		     else //right
		     { lin.x = 2; }
			//determine angle
		     if ( (rightEl/2) >=  leftEl) 
		     {
			ang.z = -1.5;
			ROS_INFO("Wall on left, turning right, Left: %f, Right: %f",leftEl,rightEl);
		     }
		     else //right
		     {
			ang.z = 1.5;
			ROS_INFO(" Wall on left, turning left, Left: %f, Right: %f",leftEl,rightEl);
		     }
		    }	
		}//turn right code
	}//end else-if (1.90 > minEl && minEl > 0.70)
*/

/*

this else-if was played w. in regards to using leftEl, rightEl. Broken, but some concepts may be salvagable?
	else if (1.90 > minEl && minEl > 0.70)
	{	
		//ROS_INFO("1.74 > %f > 0.55, turning %s", minEl, angDr?"left":"right" );
		//ROS_INFO("1.94 > %f > 0.55, turning %s", cenEl, angDr?"left":"right" );

		//lin.x = 1;
		
		if( minEl*cos(minAg) > 0) // positive
		{
			//ang.z = 0.5;
			if(1.90 > minEl && minEl > 1.84)
			{
				ang.z = 0.5;
				lin.x = 3;
				ROS_INFO("1.90 > %f > 1.84, turning left widest. Left: %f, Right: %f", minEl,leftEl,rightEl);
			}
			else if(1.84 > minEl && minEl > 1.74)
			{
				ang.z = 1.5;
				lin.x = 2.5;
				ROS_INFO("1.84 > %f > 1.74, turning left tight. Left: %f, Right: %f", minEl, leftEl, rightEl);
			}  
			else
			{
				ang.z = 3;
				lin.x = 1.5;
				ROS_INFO("1.74 > %f > 0.70, turning left sharp. Left: %f, Right: %f", minEl, leftEl, rightEl);
			} 
					
		}//turn left
		else if ( minEl*cos(minAg) < 0) // negative
		//else if ( rightEl > leftEl || minEl*cos(minAg) <= 0 && !(rightEl < leftEl) )
		{
			//ang.z = -0.5;
			//ROS_INFO("1.74 > %f > 0.70, turning right", minEl);
			if(1.90 > minEl && minEl > 1.84)
			{
				ang.z = -0.5;
				lin.x = 3;
			ROS_INFO("1.90 > %f > 1.84, turning right widest. Left: %f, Right: %f", minEl, leftEl, rightEl);
			}
			else if(1.84 > minEl && minEl > 1.74)
			{
				ang.z = -1.5;
				lin.x = 2.5;
				ROS_INFO("1.84 > %f > 1.74, turning right tight. Left: %f, Right: %f", minEl,leftEl, rightEl);
			}  
			else
			{
				ang.z = -3;
				lin.x = 1.5;
				ROS_INFO("1.74 > %f > 0.70, turning right sharp. Left: %f, Right: %f", minEl, leftEl,rightEl);
			} 	
		}//turn right
		
	}// match else if (1.94 > minEl > 0.80)


	else if (minEl <= 0.70)
	{	
	//ROS_INFO("min %f<=0.55,will be turning %s",minEl,angDr?"left":"right");
		ROS_INFO("min %f<=0.70, Left: %f, Right: %f", minEl, leftEl,rightEl);	
		lin.x = -3;
		ang.z = 0;
		if (leftEl > rightEl) { ang.x = -1;}
		else if (leftEl < rightEl) { ang.x = 1;}
	}
*/
	

	//ROS_INFO("%f", minEl);	


	/** begin June 24 minEl else version
	if(minEl >= 1.74)
	{ROS_INFO("%f >= 1.74, will be turning %s", minEl, angDr?"left":"right");		
		lin.x = 0.5; //ang.x = 1;
		ang.z = 0; //moving straight
		if (chngAg) //if true
		{
			angDr = !angDr;
		}
		
		chngAg = false;
	}
	else if (1.74 > minEl && minEl > 0.55)
	{	
		ROS_INFO("1.74 > %f > 0.55, turning %s", minEl, angDr?"left":"right" );
		chngAg = true; //if we enter next section, change  rates
		angDr = angDr; //angDr should continue left/right during this time = stay 						itself		
		//lin.x = 0.5; //slow down
		lin.x = 1;
		//ang.z = 2;
		if (angDr) //if true/left
		{
			//ang.z = 1;
			
			if (1.94 > minEl && minEl > 1.60)
			{ang.z = 0.5;
   			 ROS_INFO("1.94>%f>1.60, turning slowest %s",minEl,angDr?"left":"right");
			}
			else if (1.60 >= minEl)
			{ang.z = 1;
   			 ROS_INFO("1.60 > %f > 1.34, turning slow %s",minEl,angDr?"left":"right");
			}

			
			//if (1.94 > minEl && minEl > 1.74)
			//{ang.z = 0.5;
   			// ROS_INFO("1.94>%f>1.74, turning slowest %s",minEl,angDr?"left":"right");
			//}
			//else if (1.74 >= minEl && minEl > 1.34)
			//{ang.z = 1;
   			// ROS_INFO("1.74 > %f > 1.34, turning slow %s",minEl,angDr?"left":"right");
			//}
			//else if ( minEl <= 1.34)
			//{ang.z = 0.5;
   			// ROS_INFO("1.34>=%f>0.55,turning fast %s",minEl, angDr?"left":"right" );
			//}
			
		}
		else if (!angDr) //if false/right
		{
			//ang.z = -1;
			
			if (1.94 > minEl && minEl > 1.60)
			{ang.z = -0.5;
   			 ROS_INFO("1.94>%f>1.60, turning slowest %s",minEl,angDr?"left":"right");
			}
			else if (1.60 >= minEl)
			{ang.z = -1;
   			 ROS_INFO("1.60 > %f > 1.34, turning slow %s",minEl,angDr?"left":"right");
			}

		
			//if (1.94 > minEl && minEl > 1.74)
			//{ang.z = -0.5;
   			// ROS_INFO("1.94>%f>1.74, turning slowest %s",minEl,angDr?"left":"right");
			//}
			//else if (1.74 >= minEl && minEl > 1.34)
			//{ang.z = -1;
   			// ROS_INFO("1.74 > %f > 1.34, turning slow %s",minEl,angDr?"left":"right");
			//}
			//else if ( minEl <= 1.34)
			//{ang.z = -0.5;
   			// ROS_INFO("1.34>=%f>0.55,turning fast %s",minEl, angDr?"left":"right" );
			//}
			
			
		}
		
	}// match else if (1.94 > minEl > 0.80)
	//else if (minEl <= 0.80) //was not entering this section, even when appropriate?
	else if (minEl <= 0.55)
	{	
		ROS_INFO("%f<= 0.55, will be turning %s", minEl, angDr?"left":"right");		
		lin.x = -2;
		ang.z = 0;
		//angDr = !angDr;
		if (chngAg) //if true
		{angDr = !angDr;}

		//if (angDr) //if true/left
		//{ang.z = 1;}
		//else if (!angDr) //if false/right
		//{ang.z = -1;}
		chngAg = false; //don't need to change it again

	}  */ //end June 24 minEl else version

/**	else if (0.70 < minEl < 1.50)
	{	lin.x = 1;
		if (chngAg) //if true
		{
			ang.z = -ang.z; //should only occur when first enter this state

			if (angSp) //if true/left
			{angSp = false;} //set to right
			else if (!angSp)//if false/left
			{angSp = true;} //set to left
		}
		chngAg = false; //don't need to change it again
	} */

	/**else if (minEl < 1.00)
	{	
		ROS_INFO("< 1.00 %f turning %s", minEl, angDr?"left":"right");		
		lin.x = -3;
		ang.z = 0;
		ROS_INFO("%f", minEl);
		//angDr = !angDr;
		if (chngAg) //if true
		{
			angDr = !angDr;			
			//if (angDr) //if true/left
			//{angDr = false;} //set to right
			//else if (!angDr)//if false/left
			//{angDr = true;} //set to left
		}
		chngAg = false; //don't need to change it again
	} */
/**
	else if (1.94 > minEl > 1.50)
	{
		lin.x = 1; ang.x = 1;
		/ROS_INFO("%f", minEl);
	}
	else if (minEl <= 1.50)
	{
		lin.x = 1; ang.x = 1;
		ROS_INFO("%f", minEl);
	}
*/
	
	//if(lin.x == 0)
	//{ang.x = 1; lin.x = ;}
}//end funtion


int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "rbControl");
	//initializes code as ROS node: can be publisher, subscriber, multiple of both
	//"rbControl" is the name of the rosnode

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;
//allows us to create pub, sub; handles network handling from ros

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  //ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  //ros::Publisher chatter_pub = n.advertise<geometry_msgs::Vector3>("chatter", 1000);

  ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("/robot/cmd_vel", 1000);

  ros::Subscriber sub = n.subscribe("/robot/base_scan", 1000, controlCallback);


//creates a publisher. an object that handles the publishing of data to ros. 
// typed: needs to know hwhat kind of data is takes
//here, String data from std_msgs
//publishes on topic chatter


  ros::Rate loop_rate(10);
//most ros programs consists of continuing loops: this runs 10 time per second
  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
//while ros is still running

	//geometry_msgs::Twist msg; RESTORE

	//geometry_msgs::Vector3 ang;
	//geometry_msgs::Vector3 lin;

  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
/**
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
**/

//	geometry_msgs::Twist msg;

//	geometry_msgs::Vector3 ang;
//	geometry_msgs::Vector3 lin;
	ang.x = 0;
	ang.y = 0;
	//ang.z = 0;

	//lin.x = 1;
	lin.y = 0;
	lin.z = 0;

	msg.angular = ang;
	msg.linear  = lin;


   
//Sting appending
    //msg.data = ss.str();

    //ROS_INFO("%s", msg.data.c_str());
	//ROS_INFO("%f %f &f", msg.x, msg.y, msg.z);
//similar to printf; formatted print
//everything in printed by ROS_INFO is published, so other machines can access

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);
//use publish method to 

    ros::spinOnce();
//for internal callbacks; internal workings
//if subscriber, here's where it happens

    loop_rate.sleep();
//avoid bandwidth issues?
    ++count;
  }


  return 0;
}
