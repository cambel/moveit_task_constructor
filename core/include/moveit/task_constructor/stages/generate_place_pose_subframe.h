#pragma once

#include <moveit/task_constructor/stages/generate_place_pose.h>

namespace moveit {
namespace core {
MOVEIT_CLASS_FORWARD(JointModelGroup)
}
}

namespace moveit {
namespace task_constructor {
namespace stages {

class GeneratePlacePoseSubframe : public GeneratePlacePose
{
    public:
    GeneratePlacePoseSubframe(const std::string& name = "generate place pose");

    void compute() override;

    protected:
    void onNewSolution(const SolutionBase& s) override;

};

}
}
}