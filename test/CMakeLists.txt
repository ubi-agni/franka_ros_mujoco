find_package(rostest REQUIRED)

add_rostest_gtest(franka_hw_mujoco_test
    launch/franka_hw_mujoco.test
    main.cpp
    jacobian_tests.cpp
    fk_tests.cpp
    gravity_tests.cpp
)

add_dependencies(franka_hw_mujoco_test
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(franka_hw_mujoco_test
    ${catkin_LIBRARIES}
    franka_hw_mujoco
)

target_include_directories(franka_hw_mujoco_test PUBLIC
    ${catkin_INCLUDE_DIRS}
)
