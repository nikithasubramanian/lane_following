#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/UInt8.h>
#include <image_transport/image_transport.h>
// Includes for dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <grid_follow/GridFollowConfig.h>

#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/image_encodings.h>


/***********Find Obstacle**************/
#include <sensor_msgs/LaserScan.h>

// Includes for dynamic reconfigure
#include <sense_obstacles/FindObstacleConfig.h>

#include <vector>
#include "std_msgs/Bool.h"

#include <algorithm> // std::min_element
#include <iterator>  // std::begin, std::end

#include <unistd.h>


class GridFollow
{
public:
    GridFollow();
    void configCallback(grid_follow::GridFollowConfig &config, uint32_t level);
    void gridCallback(const nav_msgs::OccupancyGrid& grid);
    //Sign detection subscriber function
    void stop_detect_Callback(const std_msgs::UInt8 msg);
    
    //Obstacle detection subscriber function
    void obsatcle_detect_Callback(const std_msgs::UInt8 msg);
    
    
    /****Find_Obstacle*******/
    void scanCb(const sensor_msgs::LaserScan& msg);

    

private:

    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber grid_sub_;
    ros::Subscriber stop_sub_;
    ros::Subscriber obs_sub_;
    
    

    dynamic_reconfigure::Server<grid_follow::GridFollowConfig> server_;
    grid_follow::GridFollowConfig config_;

    int dist_;
    
    //initiation of variables
    float speed = 0.5f;
    float turn = 0;
    int min=0,i=0,timer=10;
    int obs=0,states=1;
    
    /**********find_obstacle***************/
    bool flag;
    
    //Assuming there is no sign detected	
    bool sign;
    
   
    
};

GridFollow::GridFollow() : nh_{"~"}
{
    grid_sub_  = nh_.subscribe("/birds_eye/grid", 10, &GridFollow::gridCallback,this);
    
    //Sign detection subscriber
    stop_sub_  = nh_.subscribe("/stop_sign_detection/stop_sign", 10, &GridFollow::stop_detect_Callback,this);
    
    //Sign detection subscriber
    obs_sub_  = nh_.subscribe("/avoid_obstacle/obstacle_detect", 10, &GridFollow::obsatcle_detect_Callback,this);
    

    // Publish on the twist command topic
    pub_ = nh_.advertise<geometry_msgs::Twist>("/prizm/twist_controller/twist_cmd", 10);
    
    
    // Dynamic Reconfigure
    server_.setCallback(boost::bind(&GridFollow::configCallback, this, _1, _2));

    // Load defaults
    server_.getConfigDefault(config_);

    dist_ = 1023;
    
    //Assuming that initially there is no obstacle 
    flag = false;
    
}

void GridFollow::configCallback(grid_follow::GridFollowConfig &config, uint32_t level)
{
    config_ = config;
}

//Function to get the published value from the sign detection program
void GridFollow::stop_detect_Callback(const std_msgs::UInt8 msg)
{
	
	//Checks if the value is greater than 0 i.e There is a stop sign infront of the robot
	//if so sets the sign value to true
	if(msg.data > 0){
	    sign = true;
	    
	}
	//if there is no sign detected, sets the sign to false
	if(msg.data == 0){
	    sign = false;
	    	
	}
	
}
//Function to get the published value from the avoid obstacle program
void GridFollow::obsatcle_detect_Callback(const std_msgs::UInt8 msg)
{
        //Checks there is an obstacle infront of the robot whose angle
        //will be inbetween 95 to 270, if yes, sets the flag value to true
	if(msg.data > 95 && msg.data < 270)
	{
		flag = true;
		obs = msg.data;
	}
	//else sets it to false
	else
	{
		flag = false;
	}
		
	//Displays the boolean variable value
    	ROS_INFO(flag ? "True" : "False");
}


void GridFollow::gridCallback(const nav_msgs::OccupancyGrid& grid)
{
    int width = grid.info.width;
    int height = grid.info.height;

    int origin_x = grid.info.origin.position.x;
    int origin_y = grid.info.origin.position.y;
    

    /*
       0,0    x
         +-------+
       y |       |
         |       |
         +-------+

    */

    /***** YOUR CODE HERE *****/
    geometry_msgs::Twist twist;
    
    //Setting the linear and angular value to speed and turn correspondingly
    twist.linear.x = speed;
    twist.angular.z = turn;
    
    //If the sign value is true i.e there is a dead end sign
   //make the robot stop 
    if(sign == true )
    {
    	//Checks if its the first stop sign detected
    	//if so it spins 180 degree and continues the run
    	if(states == 1)
    	{
    	        //sleep value
    		ros::Rate rate(0.183);
    		
    		//Setting the twist values and publishing them
    		twist.linear.x = 0.0f;
    		twist.angular.z = -0.3;
    		
    		ROS_INFO("\nStop sign 1     %d",states);
    		pub_.publish(twist);
    		
    		rate.sleep();
    		
    		//stopping the robot    		
    		twist.linear.x = 0.0f;
    		twist.angular.z = 0.0;
    		
    		//incrementing states value
        	states=2;
        }
        //if second stop sign is detected the robot stops
        else if(states == 2)
        {
        	//Setting the twist values and publishing them
        	twist.linear.x = 0.0f;
    		twist.angular.z = 0.0;
    		
    		ROS_INFO("\nStop sign 2     %d",states);
    		pub_.publish(twist);
        }
    }
    
    //if flag value is true,i.e three is an obstacle infront of the robot
    else if(flag == true)
    {
        //if there is an obstacle on left, turn right
    	if(obs > 185 )
    	{
    		turn = -0.15; ROS_INFO("inside obs1");
    	}
    	//if there is an obstacle on right, turn left
    	if(obs < 175)
    	{
    		turn = 0.15;ROS_INFO("inside obs2");
    	}

    	speed = 0.4f;
    	
    	//Publishing twist values
    	pub_.publish(twist);
    }
    
    else
    {
	    //initialising the variables
	    int min = 1000,min_index=0;
	    int count = 0,black=0,green=0;
	    int colour[4],black_1[4],green_1[4],x_0=0,x_1=0,colour_0=0;
	    int ht_0 = 0,ht_01=height/4,ht_0_1;
	    speed = 0.6f;
	    
	    //If the sign value is false i.e there is no dead end sign
	    //the robot should move
	    
		//division from column 0 to 1/2frame - 50
	    	colour_0 = 0;ht_0 = 0; ht_01 = height/2 - 50;
	    	// loop that runs along the rows
		for (int x = 0; x < width; x++)
		{
		    // loop that runs along the column
		    for (int y = ht_0; y < ht_01; y++)
		    {
			// Convert x,y to array index
			// grid.data is 1D so we use the following to convert a 2D index to 1D
			int index = x + y * width;

			// Non-zero = occupied
			if (grid.data[index] != 0)
			{
			    colour_0++;
			}
			//black count
			if (grid.data[index] == 100)
			{
			    black++;
			}
			//green count
			if (grid.data[index] == 127)
			{
			    green++;
			}
		
		    }
		}
		//Copying the counter value to corresponding 1D array
		colour[x_0]=colour_0;
		black_1[x_0]=black; green_1[x_0]=green;
		
		//Incrementing the array index
		x_0++;  
	
		//division from column 1/2frame - 50 to 1/2 frame
		colour_0 = 0;black=0;green=0;ht_0 = height/2 - 50; ht_01 = height/2; 
	    	// loop that runs along the rows
		for (int x = 0; x < width; x++)
		{
		    // loop that runs along the column
		    for (int y = ht_0; y < ht_01; y++)
		    {
			// Convert x,y to array index
			// grid.data is 1D so we use the following to convert a 2D index to 1D
			int index = x + y * width;

			// Non-zero = occupied
			if (grid.data[index] != 0)
			{
			    colour_0++;
			}
		        //black values
			if (grid.data[index] == 100)
			{
			    black++;
			}
			//green values
			if (grid.data[index] == 127)
			{
			    green++;
			}
		
		    }
		}
		//Copying the counter value to correspoding 1D array
		colour[x_0]=colour_0;
		black_1[x_0]=black; green_1[x_0]=green;
		
		//Incrementing the array index
		x_0++;  
	
		//division from column 1/2frame to 1/2 frame + 50
		colour_0 = 0;black=0;green=0;ht_0 = height/2 ; ht_01 = height/2 + 50;
	    	// loop that runs along the rows
		for (int x = 0; x < width; x++)
		{
		    // loop that runs along the column
		    for (int y = ht_0; y < ht_01; y++)
		    {
			// Convert x,y to array index
			// grid.data is 1D so we use the following to convert a 2D index to 1D
			int index = x + y * width;

			// Non-zero = occupied
			if (grid.data[index] != 0)
			{
			    colour_0++;
			}
			//black counter
			if (grid.data[index] == 100)
			{
			    black++;
			}
			//green counter
			if (grid.data[index] == 127)
			{
			    green++;
			}
		
		    }
		}
		//Copying the counter value to corresponding 1D array
		colour[x_0]=colour_0;
		black_1[x_0]=black; green_1[x_0]=green;
		//Incrementing the array index
		x_0++;
	
		//division from column 1/2frame + 50 to 1 frame 
		colour_0 = 0;black=0;green=0;ht_0 = height/2 + 50 ; ht_01 = height;
	    	// loop that runs along the rows
		for (int x = 0; x < width; x++)
		{
		    // loop that runs along the column
		    for (int y = ht_0; y < ht_01; y++)
		    {
			// Convert x,y to array index
			// grid.data is 1D so we use the following to convert a 2D index to 1D
			int index = x + y * width;

			// Non-zero = occupied
			if (grid.data[index] != 0)
			{
			    colour_0++;
			}
		        //black counter
			if (grid.data[index] == 100)
			{
			    black++;
			}
			//green counter
			if (grid.data[index] == 127)
			{
			    green++;
			}
		
		    }
		}
		//Copying the counter value to corresponding 1D array
		colour[x_0]=colour_0;
		black_1[x_0]=black; green_1[x_0]=green;
		//Incrementing the array index
		x_0++; 
	
	
		//Displaying the array values
		for(int i=0;i< 4;i++)
		 ROS_INFO("colour pixels %d",colour[i]); 
	
		ROS_INFO("\n");
		//Displaying the array values
		for(int i=0;i< 4;i++)
		 ROS_INFO("green pixels%d",green_1[i]); 
		 
		ROS_INFO("\n");
	
	        //Checks if the green line is on left or right of robot
	        //if green line is on right
		if(green_1[0] + green_1[1] < green_1[2] + green_1[3])
		{
			
			ROS_INFO(" green on right");
			
			//Corner condition
			//Turns right until robot detects the green line
			if((abs((green_1[0] + green_1[1]) - (green_1[2] + green_1[3])) < 10) && green_1[2] < 270 && green_1[3] < 140)
			{
				/*if(((green_1[0] + green_1[1]) - (green_1[2] + green_1[3])) > 0)
					turn = 0.2;
				else if (((green_1[0] + green_1[1]) - (green_1[2] + green_1[3])) < 0)
					turn = -0.2;*/
				turn = -0.2;
				ROS_INFO(" turn to see green on right");	
			
				
			}
			//if robot should be moving along staright path
			//it checks for column values of colour pixels and turn accordingly
			else if(colour[1] < colour[2])
			{	
				//turn left if column 1 is less than column2 
				turn = 0.2;
				ROS_INFO("column 1 %d",colour[1]);
			}
			else if(colour[1] > colour[2])
			{
				//turn right if column 1 is greater than column2
				turn = -0.2;
				ROS_INFO("column 2 %d",colour[2]);
			}
	
		}
		//if green line is on left
		if(green_1[0] + green_1[1] > green_1[2] + green_1[3])
		{
			
			ROS_INFO(" green on left");
		 
		        //Corner condition
			//Turns left until robot detects the green line
			if((abs((green_1[0] + green_1[1]) - (green_1[2] + green_1[3])) < 10) && green_1[0] < 270 && green_1[1] < 140)
			{
				/*if(((green_1[0] + green_1[1]) - (green_1[2] + green_1[3])) > 0)
					turn = 0.2;
				else if (((green_1[0] + green_1[1]) - (green_1[2] + green_1[3])) < 0)
					turn = -0.2;
				*/
				turn = 0.2;
				ROS_INFO(" turn to see green on left");	
			}
			//if robot should be moving along staright path
			//it checks for column values of colour pixels and turn accordingly
			else if(colour[1] < colour[2])
			{	
				//turn left if column 1 is less than column2 
				turn = 0.2;
				ROS_INFO("column 1 %d",colour[1]);
			}
			else if(colour[1] > colour[2])
			{
				//turn right if column 1 is greater than column2
				turn = -0.2;
				ROS_INFO("column 2 %d",colour[2]);
			}
	
		}
	     //Publishing twist values
	     pub_.publish(twist);	    
    }
    	    
    /**************************/
}





int main(int argc, char** argv)
{
    ros::init(argc, argv, "grid_follow");

    GridFollow sd{};

    ROS_INFO_STREAM("grid_follow running!");
    ros::spin();
    return 0;
}
