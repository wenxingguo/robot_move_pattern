#include "../robot_move_pattern.h"
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

PYBIND11_MODULE(robot_move_pattern, rmp)
{
    rmp.doc() = "To make the robot move according to a specific motion pattern";
    rmp.def("moveLInToolCoor", &moveLInToolCoor);
    rmp.def("roateInToolCoor", &roateInToolCoor);
    rmp.def("roateInBaseCoor", &roateInBaseCoor);
}
