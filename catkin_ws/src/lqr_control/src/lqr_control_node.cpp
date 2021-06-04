#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <eigen3/Eigen/Dense>
#include <tf/tf.h>

Eigen::VectorXf states(5);
Eigen::VectorXf states_rot(3);
Eigen::VectorXf U(2);
Eigen::VectorXf U_rot(2);
Eigen::MatrixXf K(2,5);
Eigen::MatrixXf K_rot(2,3);

class controller {

    private:

    ros::Publisher left_motor_pub;
    ros::Publisher right_motor_pub;
    ros::Subscriber odometry_subscriber;
    ros::Subscriber imu_subscriber;
    ros::Subscriber reference_position_topic;
    float current_time;
    float dt;
    float error;
    float ref;
    float integral_error = 0.0;
    float current_posx;
    float current_velx;
    float current_pitch_rate; 
    tf::Quaternion my_quaternion;
    double roll, pitch, yaw;
    float Yaw;
    float current_yaw_rate;
    float Pitch;
    float rot_integral_error = 0.0;
    float rot_error;
    std_msgs::Float64 left_torque;
    std_msgs::Float64 right_torque;



    public:

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    controller(ros::NodeHandle *nh) {

        left_motor_pub = nh->advertise<std_msgs::Float64>("/myrobot/left_wheel_controller/command", 10);    
        right_motor_pub = nh->advertise<std_msgs::Float64>("/myrobot/right_wheel_controller/command", 10); 
        odometry_subscriber = nh->subscribe("/ground_truth/state", 5, &controller::callback_odom, this);
        imu_subscriber = nh->subscribe("/imu", 5, &controller::callback_imu, this);
        reference_position_topic = nh->subscribe("/reference_position", 5, &controller::callback_reference_position, this);
        
        /*K << -7.0711, -23.4108, -35.2185, -126.3742, -34.6422,
             -7.0711, -23.4108, -35.2185, -126.3742, -34.6422;  // SLOW CONTROLLER GAINS*/

        K << -22.3607, -52.2212, -49.7984, -155.4824, -42.8428,
             -22.3607, -52.2212, -49.7984, -155.4824, -42.8428; // FAST CONTROLLER GAINS.

            
        
        /*K_rot << -0.7071, -1.4328, -1.0981, // The rows of the gain matrix are.. 
                 0.7071, 1.4328, 1.0981;    //..switched from what was obtained from MATLAB based on how the..
                                            //..wheels are oriented in the urdf.*/

        
        K_rot << -707.1068, -122.0235, -10.4933,
                  707.1068, -122.0235, -10.4933;  // These values correspond to a very fast yaw controller. 
                                                  // THESE MAKE THE SYSTEM UNSTABLE.
        current_posx = 0;
        current_velx = 0;
        current_pitch_rate = 0; 
        current_yaw_rate = 0.0;
        dt = 0;
        error = 0.0;
        pitch = 0.0;
        Pitch = 0.0;
        rot_error = 0.0;
        yaw = 0.0;
        left_torque.data = 0;
        right_torque.data = 0;
        current_time = ros::Time::now().toSec();
        ROS_INFO("current_time while cunstructing object = %f \n", current_time);
        ref = 0;
        rot_integral_error = 0.0;
        rot_error = 0.0;
        states << integral_error, current_posx, current_velx, Pitch, current_pitch_rate;
        states_rot << rot_integral_error, yaw, current_yaw_rate;
        ROS_INFO("These are the rotational states upon initiazation %f %f %f \n", states_rot(0), states_rot(1),
        states_rot(2) );
    }

    
    void callback_reference_position(const std_msgs::Float64::ConstPtr& msg){
        ref = msg->data;
    }


    void callback_odom(const nav_msgs::Odometry::ConstPtr& msg) {

        current_posx = msg->pose.pose.position.x;
        ROS_INFO("This is the current position = %f \n", current_posx);
        current_velx = msg->twist.twist.linear.x;
        tf::Quaternion my_quaternion(msg->pose.pose.orientation.x,
                        msg->pose.pose.orientation.y,
                        msg->pose.pose.orientation.z,
                        msg->pose.pose.orientation.w);
        tf::Matrix3x3 m(my_quaternion);
        m.getRPY(roll, pitch, yaw);
        ROS_INFO("This is the yaw from odom = %f \n", yaw);
        //ROS_INFO("%f", pitch);

    }

    void callback_imu(const sensor_msgs::Imu::ConstPtr& msg) {
        
        /*ROS_INFO("Value of same current_time from imu callback = %f \n", current_time);
        ROS_INFO("Time fetched right now from imu callback = %f \n", ros::Time::now().toSec());  */
                            
        dt = ros::Time::now().toSec() - current_time;
        if(dt>1){
            dt = 0.000000;
        }
        ROS_INFO("dt = %f \n", dt);
        error = current_posx - ref;
        rot_error = yaw;// - 0.5;
        ROS_INFO("rotational error = %f \n", rot_error);
        rot_integral_error = rot_integral_error + rot_error*dt;  
        integral_error = integral_error + error*dt;
        //double_integral_error = double_integral_error + integral_error*dt;
        current_pitch_rate = msg->angular_velocity.y;
        current_yaw_rate = msg->angular_velocity.z;
        ROS_INFO("current yaw rate from IMU  = %f \n", current_yaw_rate);
        Pitch = pitch;
        Yaw = yaw;
       /* ROS_INFO("double_integral_error = %f \n", double_integral_error);
        ROS_INFO("integral_error = %f \n", integral_error);
        ROS_INFO("current_posx = %f \n", current_posx);
        ROS_INFO("current_velx = %f \n", current_velx);
        ROS_INFO("Pitch = %f \n", Pitch);
        ROS_INFO("current_pitch_rate = %f \n", current_pitch_rate); */
        states << integral_error, current_posx, current_velx, Pitch, current_pitch_rate;
        states_rot << rot_integral_error, Yaw, current_yaw_rate;
        ROS_INFO("These are the yaw states. %f %f %f \n", states_rot(0), states_rot(1), states_rot(2));
        //states(1,0) = 200.0;
        //ROS_INFO("These are the states. %f %f %f %f %f %f \n", states(0), states(1), states(2), states(3), states(4), states(5));
         
        //states << current_posx, current_velx, Pitch, current_pitch_rate; 
        U << -K*states;
        ROS_INFO("These are the torque values %f %f \n", U(0), U(1));
        U_rot << -K_rot*states_rot;
        ROS_INFO("These are the torque values for rotational part %f %f \n", U_rot(0), U_rot(1));
        left_torque.data = U_rot(0) + U(0);
        right_torque.data = U_rot(1) + U(1);
        left_motor_pub.publish(left_torque);
        right_motor_pub.publish(right_torque);


        //current_posx = msg->pose.pose.position.x;
        //current_velx = msg->twist.twist.linear.x;
        current_time = ros::Time::now().toSec();
        
        ROS_INFO("Time fetched right now at the end of imu callback = %f \n", ros::Time::now().toSec());
    }

};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "controller");
    ros::NodeHandle nh;
    controller nc = controller(&nh);
    ros::Rate loop_rate(400);
    ros::spin();
    loop_rate.sleep();
}