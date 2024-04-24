cmake_minimum_required(VERSION 3.8)
project(ltl_automaton_msg_srv)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LTLPlan.msg"
  "msg/LTLState.msg"
  "msg/LTLStateArray.msg"
  "msg/LTLStateRuns.msg"
  "msg/TransitionSystemState.msg"
  "msg/TransitionSystemStateStamped.msg"
  "msg/SynchroConfirm.msg"
  "msg/SynchroReply.msg"
  "msg/SynchroRequest.msg"
  "srv/ClosestState.srv"
  "srv/FinishCollab.srv"
  "srv/TaskPlanning.srv"
  "srv/TrapCheck.srv"
  "srv/DynamicParameters.srv"
  DEPENDENCIES std_msgs
)

ament_package()
