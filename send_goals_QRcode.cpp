#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <cstring>
#include <qrencode.h>

using namespace std;

// Action specification for move_base
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class QRCodeReader {
private:
    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_pub_;
    MoveBaseClient ac_;

public:
    QRCodeReader() : ac_("move_base", true) {
        // 로봇 제어 퍼블리셔 설정
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

        // Wait for the action server to come up
        while(!ac_.waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the move_base action server to come up");
        }
    }

    void readQRCodeAndMoveRobot() {
        // QR 코드에 담을 데이터 입력
        const char *data = "goal.target_pose.pose.position.x = 0.752;, goal.target_pose.pose.position.y = 0.859;, goal.target_pose.pose.orientation.w = 1.0;";

        // QR 코드 생성
        QRcode *qr = QRcode_encodeString(data, 0, QR_ECLEVEL_L, QR_MODE_8, 1);

        // 데이터 추출 및 로봇 제어
        processQRCode(qr);

        // QR 코드 메모리 해제
        QRcode_free(qr);
    }

    void processQRCode(QRcode *qr) {
        // 데이터 추출
        string qr_data;
        for (int i = 0; i < qr->length; ++i) {
            qr_data += qr->data[i];
        }

        // 데이터 분석하여 로봇 제어
        controlRobot(qr_data);
    }

    void controlRobot(const string& qr_data) {
        // 데이터 분석
        double x, y, w;
        if (sscanf(qr_data.c_str(), "goal.target_pose.pose.position.x = %lf;, goal.target_pose.pose.position.y = %lf;, goal.target_pose.pose.orientation.w = %lf;", &x, &y, &w) == 3) {
            // 로봇을 제어하기 위한 코드
            moveRobotToGoal(x, y, w);
        } else {
            ROS_ERROR("Failed to parse QR code data");
        }
    }

    void moveRobotToGoal(double x, double y, double w) {
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = x;
        goal.target_pose.pose.position.y = y;
        goal.target_pose.pose.orientation.w = w;

        ROS_INFO("Sending goal");
        ac_.sendGoal(goal);

        ac_.waitForResult();

        if(ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("The robot has arrived at the goal location");
        else
            ROS_INFO("The robot failed to reach the goal location for some reason");
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "qr_code_reader_cpp");
    QRCodeReader qr_code_reader;
    qr_code_reader.readQRCodeAndMoveRobot();
    return 0;
}
