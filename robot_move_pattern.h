#ifndef ROBOR_MOVE_PATTERN_H
#define ROBOR_MOVE_PATTERN_H
#include <vector>

/*
 * 将工具坐标系的点转化到世界坐标系 R * P_t = P_b，R由rx ry rz确定
 */

/******************************************************************************
 * Function:         moveLInToolCoor
 * Description:      将机器人工具末端（TCP）沿着direction方向移动d长度
 * Where:
 *                   const std::vector<double>& current_tool_pose - 当前工具的位姿 {x y z rx ry rz} 其中(rx ry rz)为旋转向量 使用弧度（rad）
 *                   const std::vector<double>& direction - 在工具坐标系下的方向
 *                   const double d - 移动距离，可以为负 单位为m
 * Return:           std::vector<double> - 返回移动后的目标位姿;
 * Error:
 *****************************************************************************/
std::vector<double> moveLInToolCoor(const std::vector<double>& current_tool_pose, const std::vector<double>& direction, const double d);

/******************************************************************************
 * Function:         roateInToolCoor
 * Description:      使机器人的工具末端（TCP）位置不便，绕着direction方向旋转theta角度。
 * Where:
 *                   const std::vector<double>& current_tool_pose - 当前工具的位姿 {x y z rx ry rz} 其中(rx ry rz)为旋转向量 使用弧度（rad）
 *                   const std::vector<double>& direction - 旋转轴在工具坐标系下的方向
 *                   const double theta - 旋转的角度，方向由右手准则确定，单位为弧度（rad）
 * Return:           std::vector<double> - 返回移动后的目标位姿 ;
 * Error:
 *****************************************************************************/
std::vector<double> roateInToolCoor(const std::vector<double>& current_tool_pose, const std::vector<double>& direction, const double theta);

/******************************************************************************
 * Function:         roateInToolCoor
 * Description:      使机器人的工具末端（TCP）位置不便，绕着direction方向旋转theta角度。
 * Where:
 *                   const std::vector<double>& current_tool_pose - 当前工具的位姿 {x y z rx ry rz} 其中(rx ry rz)为旋转向量 使用弧度（rad）
 *                   const std::vector<double>& direction - 旋转轴在基坐标系下的方向
 *                   const double theta - 旋转的角度，方向由右手准则确定，单位为弧度（rad）
 * Return:           std::vector<double> - 返回移动后的目标位姿 ;
 * Error:
 *****************************************************************************/
std::vector<double> roateInBaseCoor(const std::vector<double>& current_tool_pose, const std::vector<double>& direction, const double theta);

#endif /* ifndef ROBOR_MOVE_PATTERN_H */
