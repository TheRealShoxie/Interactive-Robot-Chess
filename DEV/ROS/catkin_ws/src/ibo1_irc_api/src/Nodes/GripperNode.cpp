moveit::planning_interface::MoveGroupInterface* move_group_gripper_ptr;

// Look into MoveGroupInterface Options

moveit::planning_interface::MoveGroupInterface move_group_gripper("gripper");
move_group_gripper_ptr = &move_group_gripper;


    // Gripper test
    for(auto &test : move_group_gripper_ptr->getJoints()){
        cout << test << endl;
    }

    string joint = "finger_joint";

    move_group_gripper_ptr->setJointValueTarget(joint, 0);
    move_group_gripper_ptr->setMaxVelocityScalingFactor(0.02);
    move_group_gripper_ptr->setMaxAccelerationScalingFactor(0.02);
    move_group_gripper_ptr->move();

    //min = 0; max = 0.7