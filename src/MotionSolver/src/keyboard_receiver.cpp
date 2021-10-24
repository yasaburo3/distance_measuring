#include <iostream>
#include "ros/ros.h"
#include "std_msgs/Int64.h"

#include <termio.h>
#include <stdio.h>

using namespace std;

int scanKeyboard()
{
    int input;
    struct termios new_settings;
    struct termios stored_settings;
    tcgetattr(0,&stored_settings);
    new_settings = stored_settings;
    new_settings.c_lflag &= (~ICANON);
    new_settings.c_cc[VTIME] = 0;
    tcgetattr(0,&stored_settings);
    new_settings.c_cc[VMIN] = 1;
    tcsetattr(0,TCSANOW,&new_settings);
      
    input = getchar();
      
    tcsetattr(0,TCSANOW,&stored_settings);
    return input;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "keyboard_receiver");
    ros::NodeHandle nh;
    ros::Publisher msg_pub = nh.advertise<std_msgs::Int64>("/key_pressed", 3);

    std_msgs::Int64 key_msg;
    cout << "Press E to start measurement." << endl;
    cout << "Press Q to quit." << endl;

    int input;
    while(ros::ok()){
        input = scanKeyboard();
        if(input == 113 || input == 81)     
            break;
        
        key_msg.data = input;
        msg_pub.publish(key_msg);
        cout << "\ninput received: " << char(input) << endl;
        ros::spinOnce();

    }
    return 0;
}

