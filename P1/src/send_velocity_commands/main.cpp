#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

class RobotDriver{
    private:
    //! The node handle we'll be using
    ros::NodeHandle nh_;
    //! We will be publishing to the "/base_controller/command" topic to issue commands
    ros::Publisher cmd_vel_pub_;

    public:
    //! ROS node initialization
    RobotDriver(ros::NodeHandle &nh){
        nh_ = nh;
        //set up the publisher for the cmd_vel topic
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    }

    //! Loop forever while sending drive commands based on keyboard input
    bool driveKeyboard(){
        std::cout << "Type a command and then press enter.  "
        "Use 'w,a,s,d' to move the robot"
        "'.' to exit.\n";

        //we will be sending commands of type "twist"
        geometry_msgs::Twist base_cmd;

        char cmd[50];
        while(nh_.ok()){

            std::cin.getline(cmd, 50);

            base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;   
            if(cmd[0]=='w'){
                //move forward
                base_cmd.linear.x = 0.25;
            }else if(cmd[0]=='s'){
                //move backwards
                base_cmd.linear.x = -0.25;
            }else if(cmd[0]=='a'){
                //turn left (yaw) in the same place
                base_cmd.angular.z = 0.5;
                base_cmd.linear.x = 0.0;
            }else if(cmd[0]=='d'){
                //turn right (yaw) in the same place
                base_cmd.angular.z = -0.5;
                base_cmd.linear.x = 0.0;
            }else if(cmd[0]=='.'){
                //quit
                break;
            }else{
                std::cout << "unknown command:" << cmd << "\n";
                continue;
            }
            

            //publish the assembled command
            cmd_vel_pub_.publish(base_cmd);
        }
        return true;
    }

};

int main(int argc, char** argv)
{
    //init the ROS node
    ros::init(argc, argv, "robot_driver");
    ros::NodeHandle nh;

    RobotDriver driver(nh);
    driver.driveKeyboard();
}

// CON ESTO SE PUEDE MOVER SIN EL \n
/*#include <termios.h>
int main() {
    struct termios oldt, newt;
    char ch;
    ros::init(argc, argv, "robot_driver");
    ros::NodeHandle nh;

    RobotDriver driver(nh);
    // Get the current terminal attributes
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;

    // Set the terminal to raw mode
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    std::cout << "Use 'w' for forward, 's' for backward. Press 'q' to quit." << std::endl;

    while (true) {
        // Read a single character
        if (read(STDIN_FILENO, &ch, 1) != 1) {
            // Handle error or continue
            continue;
        }

        // Process the input
        switch (ch) {
            case 'w':
                std::cout << "Move forward" << std::endl;
                // Add your logic for moving forward
                break;
            case 's':
                std::cout << "Move backward" << std::endl;
                // Add your logic for moving backward
                break;
            case 'q':
                std::cout << "Quitting" << std::endl;
                // Add your logic for quitting
                break;
            default:
                // Handle other keys as needed
                break;
        }

        // Break the loop on 'q'
        if (ch == 'q') {
            break;
        }
    }

    // Restore the terminal attributes
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

    return 0;
}*/