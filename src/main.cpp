#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <sensor_msgs/msg/joint_state.h>
#include <rosidl_runtime_c/string_functions.h>

#include <robot_messages/msg/wheels_command.h>

#include "Wheels.h"

const unsigned int health_timer_timeout = 1000; // ms
const unsigned int wheels_timer_timeout = 100;

rcl_publisher_t health_publisher;
rcl_publisher_t wheels_publisher;
rcl_subscription_t wheels_subscriber;
std_msgs__msg__Int32 health_msg;
robot_messages__msg__WheelsCommand wheels_speed_msg;
sensor_msgs__msg__JointState wheels_state_msg;
void init_wheel_joint_message()
{
  // Initialize the structure
  sensor_msgs__msg__JointState__init(&wheels_state_msg);

  // 2. Allocate memory for Names (4 strings)
  rosidl_runtime_c__String__Sequence__init(&wheels_state_msg.name, 4);
  rosidl_runtime_c__String__assign(&wheels_state_msg.name.data[0], "lf_joint");
  rosidl_runtime_c__String__assign(&wheels_state_msg.name.data[1], "rf_joint");
  rosidl_runtime_c__String__assign(&wheels_state_msg.name.data[2], "rb_joint");
  rosidl_runtime_c__String__assign(&wheels_state_msg.name.data[3], "lb_joint");

  // 3. Allocate memory for Velocity (4 floats)
  wheels_state_msg.velocity.data = (double *)malloc(4 * sizeof(double));
  wheels_state_msg.velocity.size = 4;
  wheels_state_msg.velocity.capacity = 4;

  // 4. Allocate memory for Position (4 floats)
  wheels_state_msg.position.data = (double *)malloc(4 * sizeof(double));
  wheels_state_msg.position.size = 4;
  wheels_state_msg.position.capacity = 4;

  // Optional: effort is usually empty, but good to set size to 0
  wheels_state_msg.effort.size = 0;

  rosidl_runtime_c__String__assign(&wheels_state_msg.header.frame_id, "base_link");
}

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t health_timer;
rcl_timer_t wheels_timer;

Wheels *wheels = NULL;

#define RCCHECK(fn)              \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
      error_loop();              \
    }                            \
  }
#define RCSOFTCHECK(fn)                \
  {                                    \
    rcl_ret_t temp_rc = fn;            \
    if ((temp_rc != RCL_RET_OK))       \
    {                                  \
      log_e("A rclc error occured !"); \
    }                                  \
  }

// Error handle loop
void error_loop()
{
  while (true)
  {
    log_e("A blocking rclc error occured !");
    delay(1000);
  }
}

void health_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {
    log_d("Publishing health (%d)", health_msg.data);
    RCSOFTCHECK(rcl_publish(&health_publisher, &health_msg, NULL));
    health_msg.data++;
  }
}

void wheels_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {
    wheelsState current_state = wheels->get_current_state();

    wheels_state_msg.velocity.data[0] = current_state.w1;
    wheels_state_msg.velocity.data[1] = current_state.w2;
    wheels_state_msg.velocity.data[2] = current_state.w3;
    wheels_state_msg.velocity.data[3] = current_state.w4;

    wheels_state_msg.position.data[0] = current_state.p1;
    wheels_state_msg.position.data[1] = current_state.p2;
    wheels_state_msg.position.data[2] = current_state.p3;
    wheels_state_msg.position.data[3] = current_state.p4;

    RCSOFTCHECK(rcl_publish(&wheels_publisher, &wheels_state_msg, NULL));
  }
}

void wheels_command_callback(const void *msgin)
{
  const robot_messages__msg__WheelsCommand *wheels_speed_msg = (const robot_messages__msg__WheelsCommand *)msgin;
  if (wheels_speed_msg != NULL)
  {
    log_v("Received wheel command: %f, %f, %f, %f", wheels_speed_msg->lf_joint, wheels_speed_msg->rf_joint, wheels_speed_msg->rb_joint, wheels_speed_msg->lb_joint);
    wheels->set_speed(wheels_speed_msg->lf_joint, wheels_speed_msg->rf_joint, wheels_speed_msg->rb_joint, wheels_speed_msg->lb_joint);
  }
}

void setup()
{
  log_i("Initializing");
  // Init hardware
  wheels = new Wheels();

  // Configure serial transport
  Serial.begin(115200);
  set_microros_serial_transports(Serial);

  while (rmw_uros_ping_agent(1000, 1) != RMW_RET_OK)
  {
    static uint i = 0;
    log_w("Waiting for agent, %d failed pings", ++i);
  }
  log_i("Agent found, starting up");

  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
      &health_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "micro_ros_platformio_time_elapsed"));
  RCCHECK(rclc_publisher_init_default(
      &wheels_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
      "/micro_controller/joint_states"));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
      &wheels_subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(robot_messages, msg, WheelsCommand),
      "/wheel_commands"));

  // create timer,
  RCCHECK(rclc_timer_init_default2(
      &health_timer,
      &support,
      RCL_MS_TO_NS(health_timer_timeout),
      health_timer_callback, true));
  RCCHECK(rclc_timer_init_default2(
      &wheels_timer,
      &support,
      RCL_MS_TO_NS(wheels_timer_timeout),
      wheels_timer_callback, true));

  // init msgs
  health_msg.data = 0;
  init_wheel_joint_message();

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &health_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &wheels_timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &wheels_subscriber, &wheels_speed_msg, &wheels_command_callback, ON_NEW_DATA));

  // Energize hardware
  wheels->enable_motors();
}

void loop()
{
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
