/* i2c - Simple example

   Simple I2C example that shows how to initialize I2C
   as well as reading and writing from and to registers for a sensor connected over I2C.

   The sensor used in this example is a MPU9250 inertial measurement unit.

   For other examples please check:
   https://github.com/espressif/esp-idf/tree/master/examples

   See README.md file to get detailed usage of this example.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "esp_log.h"
#include "neotrellis.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

rcl_publisher_t publisher;
rcl_subscription_t subscriber;
std_msgs__msg__Int32 send_msg;
std_msgs__msg__Int32 recv_msg;


static const char *TAG = "neotrellis-app";

static bool led_on[3] = {false, false, false};

void toggleLed(uint8_t led)
{
    struct rgb_color lc;

    lc.red = 0x00;
    lc.green = 0x00;
    lc.blue = 0x00;
    if (led_on[led])
    {
        if (led == 0)
            lc.red = 0xFF;
        if (led == 1)
            lc.green = 0xFF;
        if (led == 2)
            lc.blue = 0xFF;
        led_on[led] = false;
    }
    else
    {
        if (led == 0)
            lc.red = 0x50;
        if (led == 1)
            lc.green = 0x50;
        if (led == 2)
            lc.blue = 0x50;

        led_on[led] = true;
    }
    neotrellis_set_led(led, &lc);
}

void button0Func()
{
    ESP_LOGI(TAG, "Button0");
    toggleLed(0);
}

void button1Func()
{
    ESP_LOGI(TAG, "Button1");
    toggleLed(1);
}

void button2Func()
{
    ESP_LOGI(TAG, "Button2");
    toggleLed(2);
}

static void led_task(void *pArgs)
{
    struct rgb_color lc;
    lc.red = 0x00;
    lc.green = 0x00;
    lc.blue = 0x00;

    toggleLed(0);
    toggleLed(1);
    toggleLed(2);

    while (1)
    {
        for (int led = 4; led < neotrellis_get_number_of_leds(); led++)
        {
            if (neotrellis_is_button_pressed(0))
                lc.red = 0x20;
            if (neotrellis_is_button_pressed(1))
                lc.green = 0x20;
            if (neotrellis_is_button_pressed(2))
                lc.blue = 0x20;
            neotrellis_set_led(led, &lc);
            vTaskDelay(100 / portTICK_PERIOD_MS);

            lc.red = 0x00;
            lc.green = 0x00;
            lc.blue = 0x00;
            neotrellis_set_led(led, &lc);
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
    }
    vTaskDelete(NULL);
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	(void) last_call_time;
	if (timer != NULL) {
		RCSOFTCHECK(rcl_publish(&publisher, &send_msg, NULL));
		printf("Sent: %d\n",  (int)  send_msg.data);
		send_msg.data++;
	}
}

void subscription_callback(const void * msgin)
{
	const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
	printf("Received: %d\n",  (int)  msg->data);
}

void micro_ros_task(void * arg)
{

    ESP_LOGI(TAG, "micro_ros_task start");

	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// Create init_options.
	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	RCCHECK(rcl_init_options_init(&init_options, allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
	rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    ESP_LOGI(TAG, "micro_ros_task set udp options");

	// Static Agent IP and port can be used instead of autodisvery.
	RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
	//RCCHECK(rmw_uros_discover_agent(rmw_options));
#endif
    ESP_LOGI(TAG, "micro_ros_task support init");

// Setup support structure.
	RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
    ESP_LOGI(TAG, "micro_ros_task support init done");

	// Create node.
	rcl_node_t node = rcl_get_zero_initialized_node();
	RCCHECK(rclc_node_init_default(&node, "int32_publisher_subscriber_rclc", "", &support));

	// Create publisher.
	RCCHECK(rclc_publisher_init_default(
		&publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"int32_publisher"));

	// Create subscriber.
	RCCHECK(rclc_subscription_init_default(
		&subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"int32_subscriber"));

	// Create timer.
	rcl_timer_t timer = rcl_get_zero_initialized_timer();
	const unsigned int timer_timeout = 1000;
	RCCHECK(rclc_timer_init_default2(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		timer_callback,
		true));

	// Create executor.
	rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
	unsigned int rcl_wait_timeout = 1000;   // in ms
	RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout)));

	// Add timer and subscriber to executor.
	RCCHECK(rclc_executor_add_timer(&executor, &timer));
	RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &recv_msg, &subscription_callback, ON_NEW_DATA));

	// Spin forever.
	send_msg.data = 0;
	while(1){
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        vTaskDelay(1000 / portTICK_PERIOD_MS);
	}

	// Free resources.
	RCCHECK(rcl_subscription_fini(&subscriber, &node));
	RCCHECK(rcl_publisher_fini(&publisher, &node));
	RCCHECK(rcl_node_fini(&node));

  	vTaskDelete(NULL);
}

void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_ERROR); // set all components to ERROR level
    esp_log_level_set(TAG, ESP_LOG_INFO);  // set all components to ERROR level
    ESP_LOGI(TAG, "Starting");

    neotrellis_init();

    neotrellis_set_button_callback(0, button0Func);
    neotrellis_set_button_callback(1, button1Func);
    neotrellis_set_button_callback(2, button2Func);

    xTaskCreate(led_task, "led_task", 2048, NULL, 5, NULL);

#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif

    //pin micro-ros task in APP_CPU to make PRO_CPU to deal with wifi:
    xTaskCreate(micro_ros_task,
            "uros_task",
            16000,
            NULL,
            5,
            NULL);
    while (1)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}