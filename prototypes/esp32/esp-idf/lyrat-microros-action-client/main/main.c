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

#include <example_interfaces/action/fibonacci.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}


#define MAX_FIBONACCI_ORDER 500
#define GOALS_NUMBER 5

static const char *TAG = "lyrat-microros-action-client";


example_interfaces__action__Fibonacci_SendGoal_Request ros_goal_request[GOALS_NUMBER];
uint32_t goals_value[GOALS_NUMBER] = {1, 15, 20, 55, 201};
bool goals_completed = false;

rclc_action_client_t action_client;

void goal_request_callback(rclc_action_goal_handle_t * goal_handle, bool accepted, void * context)
{
  (void) context;

  example_interfaces__action__Fibonacci_SendGoal_Request * request =
    (example_interfaces__action__Fibonacci_SendGoal_Request *) goal_handle->ros_goal_request;
  printf(
    "Goal request (order: %ld): %s\n",
    request->goal.order,
    accepted ? "Accepted" : "Rejected"
  );
}

void feedback_callback(rclc_action_goal_handle_t * goal_handle, void * ros_feedback, void * context)
{
  (void) context;

  example_interfaces__action__Fibonacci_SendGoal_Request * request =
    (example_interfaces__action__Fibonacci_SendGoal_Request *) goal_handle->ros_goal_request;

  printf(
    "Goal Feedback (order: %ld) [",
    request->goal.order
  );

  example_interfaces__action__Fibonacci_FeedbackMessage * feedback =
    (example_interfaces__action__Fibonacci_FeedbackMessage *) ros_feedback;

  for (size_t i = 0; i < feedback->feedback.sequence.size; i++) {
    printf("%ld ", feedback->feedback.sequence.data[i]);
  }
  printf("\b]\n");

  if (request->goal.order == 20 && feedback->feedback.sequence.size == 10) {
    rclc_action_send_cancel_request(goal_handle);
  }
}

void result_request_callback(
  rclc_action_goal_handle_t * goal_handle, void * ros_result_response,
  void * context)
{
  (void) context;

  static size_t goal_count = 1;

  example_interfaces__action__Fibonacci_SendGoal_Request * request =
    (example_interfaces__action__Fibonacci_SendGoal_Request *) goal_handle->ros_goal_request;

  printf(
    "Goal Result (order: %ld) [",
    request->goal.order
  );

  example_interfaces__action__Fibonacci_GetResult_Response * result =
    (example_interfaces__action__Fibonacci_GetResult_Response *) ros_result_response;

  if (result->status == GOAL_STATE_SUCCEEDED) {
    for (size_t i = 0; i < result->result.sequence.size; i++) {
      printf("%ld ", result->result.sequence.data[i]);
    }
  } else if (result->status == GOAL_STATE_CANCELED) {
    printf("CANCELED ");
  } else {
    printf("ABORTED ");
  }

  printf("\b]\n");

  // Request next action
  if (goal_count < GOALS_NUMBER) {
    if (RCL_RET_OK !=
      rclc_action_send_goal_request(&action_client, &ros_goal_request[goal_count], NULL))
    {
      printf("Error sending request nº %d\n", goal_count);
    } else {
      goal_count++;

      if (goal_count == GOALS_NUMBER) {
        goals_completed = true;
      }
    }
  }
}

void cancel_request_callback(
  rclc_action_goal_handle_t * goal_handle, bool cancelled,
  void * context)
{
  (void) context;

  example_interfaces__action__Fibonacci_SendGoal_Request * request =
    (example_interfaces__action__Fibonacci_SendGoal_Request *) goal_handle->ros_goal_request;

  printf(
    "Goal cancel request (order: %ld): %s\n",
    request->goal.order,
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
    ROSIDL_GET_ACTION_TYPE_SUPPORT(example_interfaces, Fibonacci),
    "fibonacci"
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

  example_interfaces__action__Fibonacci_FeedbackMessage ros_feedback;
  example_interfaces__action__Fibonacci_GetResult_Response ros_result_response;

  ros_feedback.feedback.sequence.capacity = MAX_FIBONACCI_ORDER;
  ros_feedback.feedback.sequence.size = 0;
  ros_feedback.feedback.sequence.data = (int32_t *) malloc(
    ros_feedback.feedback.sequence.capacity * sizeof(int32_t));

  ros_result_response.result.sequence.capacity = MAX_FIBONACCI_ORDER;
  ros_result_response.result.sequence.size = 0;
  ros_result_response.result.sequence.data = (int32_t *) malloc(
    ros_result_response.result.sequence.capacity * sizeof(int32_t));

  for (size_t i = 0; i < 5; i++) {
    ros_goal_request[i].goal.order = goals_value[i];
  }

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
    rclc_action_send_goal_request(&action_client, &ros_goal_request[0], NULL))
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
