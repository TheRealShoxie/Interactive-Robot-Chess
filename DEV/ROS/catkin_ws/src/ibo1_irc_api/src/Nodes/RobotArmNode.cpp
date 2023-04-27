moveit::planning_interface::MoveGroupInterface* move_group_arm_ptr;
// Setting up the movegroup for arm and gripper
    moveit::planning_interface::MoveGroupInterface move_group_arm("arm");
    

    move_group_arm_ptr = &move_group_arm;