find_package(rostest REQUIRED)

add_rostest_gtest(franka_hw_mjsim_test
    launch/franka_hw_mjsim.test
    main.cpp
    jacobian_tests.cpp
    fk_tests.cpp
    gravity_tests.cpp
)

add_dependencies(franka_hw_mjsim_test
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(franka_hw_mjsim_test
    ${catkin_LIBRARIES}
    franka_hw_sim_mujoco
)

target_include_directories(franka_hw_mjsim_test PUBLIC
    ${catkin_INCLUDE_DIRS}
)

add_rostest_gtest(franka_hw_sim_mujoco_test
    launch/franka_hw_sim_mujoco.test
    franka_hw_sim_mujoco_test.cpp
)

add_dependencies(franka_hw_sim_mujoco_test
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(franka_hw_sim_mujoco_test
    ${catkin_LIBRARIES}
    franka_hw_sim_mujoco
)

target_include_directories(franka_hw_sim_mujoco_test PUBLIC
    ${catkin_INCLUDE_DIRS}
)

add_rostest_gtest(franka_gripper_sim_mujoco_test launch/franka_gripper_sim_mujoco.test
  franka_gripper_sim_mujoco_test.cpp
  gripper_sim_mujoco_test_setup.cpp
  gripper_sim_mujoco_test_setup.h
)

add_dependencies(franka_gripper_sim_mujoco_test
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(franka_gripper_sim_mujoco_test
  ${CATKIN_LIBRARIES}
  franka_gripper_sim_mujoco
)

target_include_directories(franka_gripper_sim_mujoco_test PUBLIC
  ${CATKIN_INCLUDE_DIRS}
)

add_rostest_gtest(franka_gripper_sim_mujoco_test_with_object
  launch/franka_gripper_sim_mujoco_with_object.test
  franka_gripper_sim_mujoco_test_with_object.cpp
  gripper_sim_mujoco_test_setup.cpp
  gripper_sim_mujoco_test_setup.h
)

add_dependencies(franka_gripper_sim_mujoco_test_with_object
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(franka_gripper_sim_mujoco_test_with_object
  ${CATKIN_LIBRARIES}
  franka_gripper_sim_mujoco
)

target_include_directories(franka_gripper_sim_mujoco_test_with_object PUBLIC
  ${CATKIN_INCLUDE_DIRS}
)

catkin_add_gtest(franka_hw_sim_mujoco_controller_verifier_test
  main.cpp
  controller_verifier_test.cpp
)

add_dependencies(franka_hw_sim_mujoco_controller_verifier_test
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(franka_hw_sim_mujoco_controller_verifier_test
  ${CATKIN_LIBRARIES}
  franka_hw_sim_mujoco
)

target_include_directories(franka_hw_sim_mujoco_controller_verifier_test PUBLIC
  ${CATKIN_INCLUDE_DIRS}
)
