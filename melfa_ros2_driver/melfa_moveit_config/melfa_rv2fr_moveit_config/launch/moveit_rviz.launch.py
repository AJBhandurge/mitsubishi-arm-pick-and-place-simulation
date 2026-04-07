from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch


def generate_launch_description():
    moveit_config = (
    MoveItConfigsBuilder("rv2fr", package_name="melfa_rv2fr_moveit_config")
    .planning_pipelines(
        pipelines=[
            "ompl",
            "pilz_industrial_motion_planner"
        ],
        default_planning_pipeline="ompl"
    )
    .to_moveit_configs()
)

    return generate_moveit_rviz_launch(moveit_config)
