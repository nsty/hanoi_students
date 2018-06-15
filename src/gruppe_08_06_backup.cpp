void moveSlice(int from, int to, bool approvalRequired) {
    assert(from >= 0 && from <= 2);
    assert(to >= 0 && to <= 2);

    // Hier Code einfügen

    //oberhalb From Tower
    geometry_msgs::PoseStamped tower_pose_from_oben = tower_poses_[from];
    tower_pose_from_oben.pose.position.z += 0.15;
    planAndMove(tower_pose_from_oben, approvalRequired);

    //unten from Tower
    geometry_msgs::PoseStamped tower_pose_from_unten = tower_poses_[from];
    tower_pose_from_unten.pose.position.z += slice_height_*(tower_nSlices_[from]-1);
    planAndMove(tower_pose_from_unten, approvalRequired);

    //nehmen und greifen
    tower_nSlices_[from] -= 1;
    gripperClose();

    //fahre wieder nach oben
    planAndMove(tower_pose_from_oben, approvalRequired);

    //fahre zu to oberhalb
    geometry_msgs::PoseStamped tower_pose_to_oben = tower_poses_[to];
    tower_pose_to_oben.pose.position.z += 0.15;
    planAndMove(tower_pose_to_oben, approvalRequired);

    //fahre zu to stab
    geometry_msgs::PoseStamped tower_pose_to_stab = tower_poses_[to];
    tower_pose_to_stab.pose.position.z += 0.08;
    planAndMove(tower_pose_to_stab, approvalRequired);

    //loslassen
    gripperOpen();
    tower_nSlices_[to] += 1;

    //wieder hoch
    planAndMove(tower_pose_to_oben, approvalRequired);


  }

  void moveSlice(int from, int to, bool approvalRequired) {
    assert(from >= 0 && from <= 2);
    assert(to >= 0 && to <= 2);

    // Hier Code einfügen

    //oberhalb From Tower
    geometry_msgs::PoseStamped tower_pose_from_oben = tower_poses_[from];
    tower_pose_from_oben.pose.position.z += 0.15;
    //planAndMove(tower_pose_from_oben, approvalRequired);

    //unten from Tower
    geometry_msgs::PoseStamped tower_pose_from_unten = tower_poses_[from];
    tower_pose_from_unten.pose.position.z += slice_height_*(tower_nSlices_[from]-1);
    //planAndMove(tower_pose_from_unten, approvalRequired);


    std::vector<geometry_msgs::Pose> vorGreifen;
    vorGreifen.push_back(tower_pose_from_oben.pose);
    vorGreifen.push_back(tower_pose_from_unten.pose);
    moveAlongCartesianPathInWorldCoords(vorGreifen, 0.01, 0.0, true, approvalRequired);
    vorGreifen.clear();

    //nehmen und greifen
    tower_nSlices_[from] -= 1;
    gripperClose();

    //fahre wieder nach oben
    //planAndMove(tower_pose_from_oben, approvalRequired);

    //fahre zu to oberhalb
    geometry_msgs::PoseStamped tower_pose_to_oben = tower_poses_[to];
    tower_pose_to_oben.pose.position.z += 0.15;
    //planAndMove(tower_pose_to_oben, approvalRequired);

    //fahre zu to stab
    geometry_msgs::PoseStamped tower_pose_to_stab = tower_poses_[to];
    tower_pose_to_stab.pose.position.z += 0.08;
    //planAndMove(tower_pose_to_stab, approvalRequired);

    vorGreifen.push_back(tower_pose_from_oben.pose);
    vorGreifen.push_back(tower_pose_to_oben.pose);
    vorGreifen.push_back(tower_pose_to_stab.pose);
    moveAlongCartesianPathInWorldCoords(vorGreifen, 0.01, 0.0, true, approvalRequired);
    vorGreifen.clear();

    //loslassen
    gripperOpen();
    tower_nSlices_[to] += 1;

    //wieder hoch
    planAndMove(tower_pose_to_oben, approvalRequired);


  }

  void moveSlice(int from, int to, bool approvalRequired) {
    assert(from >= 0 && from <= 2);
    assert(to >= 0 && to <= 2);

    // Hier Code einfügen

    //oberhalb From Tower
    geometry_msgs::PoseStamped tower_pose_from_oben = tower_poses_[from];
    tower_pose_from_oben.pose.position.z += 0.15;
    planAndMove(tower_pose_from_oben, approvalRequired);

    //unten from Tower
    geometry_msgs::PoseStamped tower_pose_from_unten = tower_poses_[from];
    tower_pose_from_unten.pose.position.z += slice_height_*(tower_nSlices_[from]-1);
    publishPoseGoalLinear(tower_pose_from_unten);

    //nehmen und greifen
    tower_nSlices_[from] -= 1;
    gripperClose();

    //fahre wieder nach oben
    publishPoseGoalLinear(tower_pose_from_oben);

    //fahre zu to oberhalb
    geometry_msgs::PoseStamped tower_pose_to_oben = tower_poses_[to];
    tower_pose_to_oben.pose.position.z += 0.15;
    planAndMove(tower_pose_to_oben, approvalRequired);

    //fahre zu to stab
    geometry_msgs::PoseStamped tower_pose_to_stab = tower_poses_[to];
    tower_pose_to_stab.pose.position.z += 0.08;
    publishPoseGoalLinear(tower_pose_to_stab);

    //loslassen
    gripperOpen();
    tower_nSlices_[to] += 1;

    //wieder hoch
    publishPoseGoalLinear(tower_pose_to_oben);


  }