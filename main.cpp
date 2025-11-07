#include "robot_move_pattern.h"
#include <cmath>
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>

const static std::string ROBOT_IP = "192.168.1.2";

int main(int argc, char* argv[])
{
    ur_rtde::RTDEReceiveInterface ur_receive(ROBOT_IP);
    // ur_rtde::RTDEControlInterface ur_control(ROBOT_IP);
    std::vector<double> current_tool_pose = ur_receive.getActualTCPPose();
    // 沿着Z方向移动10cm
    // std::vector<double> target_tool_pose = moveLInToolCoor(current_tool_pose, { 1, 1, 1 }, -0.1);

    for (const auto& i : current_tool_pose) {
        std::cout << i << " ";
    }

    std::cout << "\n";

    // ur_control.moveL(target_tool_pose, 0.1, 0.1);
    std::vector<double> target_tool_pose = roateInBaseCoor(current_tool_pose, { 0, 0, 1 }, M_PI_2);
    for (const auto& i : target_tool_pose) {
        std::cout << i << " ";
    }
    std::cout << "\n";

    // ur_control.moveJ_IK(target_tool_pose, 0.1, 0.1);

    return 0;
}
