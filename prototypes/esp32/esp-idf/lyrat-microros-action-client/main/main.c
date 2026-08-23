#define RCUTILS_LOG_MIN_SEVERITY RCUTILS_LOG_MIN_SEVERITY_DEBUG

#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "esp_log.h"
#include <uros_network_interfaces.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <my_interfaces/action/gripper_control.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}


static const char *TAG = "lyrat-microros-action-client";

my_interfaces__action__GripperControl_SendGoal_Request gripper_goal_request;

bool goals_completed = false;

rclc_action_client_t action_client;

void goal_request_callback(rclc_action_goal_handle_t * goal_handle, bool accepted, void * context)
{
  (void) context;

  my_interfaces__action__GripperControl_SendGoal_Request * request =
    (my_interfaces__action__GripperControl_SendGoal_Request *) goal_handle->ros_goal_request;
  printf(
    "Goal request (command: %s): %s\n",
    request->goal.command.data,
    accepted ? "Accepted" : "Rejected"
  );
}

void feedback_callback(rclc_action_goal_handle_t * goal_handle, void * ros_feedback, void * context)
{
  (void) context;

  my_interfaces__action__GripperControl_SendGoal_Request * request =
    (my_interfaces__action__GripperControl_SendGoal_Request *) goal_handle->ros_goal_request;

  printf(
    "Goal Feedback (command: %s) [",
    request->goal.command.data
  );
}

void result_request_callback(
  rclc_action_goal_handle_t * goal_handle, void * ros_result_response,
  void * context)
{
  (void) context;

  my_interfaces__action__GripperControl_SendGoal_Request * request =
    (my_interfaces__action__GripperControl_SendGoal_Request *) goal_handle->ros_goal_request;

  printf(
    "Goal Result (command: %s) [",
    request->goal.command.data
  );

  my_interfaces__action__GripperControl_GetResult_Response * result =
    (my_interfaces__action__GripperControl_GetResult_Response *) ros_result_response;

  if (result->status == GOAL_STATE_SUCCEEDED) {
    printf("%s ", result->result.message.data);
  } else if (result->status == GOAL_STATE_CANCELED) {
    printf("CANCELED ");
  } else {
    printf("ABORTED ");
  }

  printf("\b]\n");
}

void cancel_request_callback(
  rclc_action_goal_handle_t * goal_handle, bool cancelled,
  void * context)
{
  (void) context;

  my_interfaces__action__GripperControl_SendGoal_Request * request =
    (my_interfaces__action__GripperControl_SendGoal_Request *) goal_handle->ros_goal_request;

  printf(
    "Goal cancel request (order: %s): %s\n",
    request->goal.command.data,
    cancelled ? "Accepted" : "Rejected");
}


void app_main(void)
{

  #if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif

  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_t support;

  // Create init_options.
	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	RCCHECK(rcl_init_options_init(&init_options, allocator));

  rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
  ESP_LOGI(TAG, "micro_ros_task set udp options");

	// Static Agent IP and port can be used instead of autodisvery.
	RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));

  ESP_LOGI(TAG, "micro_ros_task support init");

  // Create init_options
	RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
  ESP_LOGI(TAG, "micro_ros_task support init done");

  // Create node
  rcl_node_t node = rcl_get_zero_initialized_node();
	RCCHECK(rclc_node_init_default(&node, "demo_action_client_node", "", &support));
  ESP_LOGI(TAG, "micro_ros_task node init done");

  // Create action client
  RCCHECK(rclc_action_client_init_default(
    &action_client,
    &node,
    ROSIDL_GET_ACTION_TYPE_SUPPORT(my_interfaces, GripperControl),
    "move_to_position"
  ));
  ESP_LOGI(TAG, "micro_ros_task clienr init done");


  // Create executor
  rclc_executor_t executor;
  // Note:
  // If you need more than the default number of publisher/subscribers, etc., you
  // need to configure the micro-ROS middleware also!
  // See documentation in the executor.h at the function rclc_executor_init()
  // for more details.
  rclc_executor_init(&executor, &support.context, 6, &allocator);

  my_interfaces__action__GripperControl_FeedbackMessage ros_feedback;
  my_interfaces__action__GripperControl_GetResult_Response ros_result_response;

  gripper_goal_request.goal.command.data = "open";

  rclc_executor_add_action_client(
    &executor,
    &action_client,
    10,
    &ros_result_response,
    &ros_feedback,
    goal_request_callback,
    feedback_callback,
    result_request_callback,
    cancel_request_callback,
    (void *) &action_client
  );

  rclc_sleep_ms(1000);

  if (RCL_RET_OK !=
    rclc_action_send_goal_request(&action_client, &gripper_goal_request, NULL))
  {
    printf("Error sending initial goal\n");
    goals_completed = true;
  }

  while (!goals_completed) {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    rclc_sleep_ms(100);
  }

  // clean up
  rclc_executor_fini(&executor);
  (void)!rcl_node_fini(&node);

}
