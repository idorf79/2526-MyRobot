// Example usage of led_matrix_gfx for a 29x5 NeoPixel matrix on GPIO 42.
// Drop this in as your project's main/main.c (and add
// led_matrix_gfx as a component, see README.md).

#include "led_matrix_gfx.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif


#define MATRIX_WIDTH  29
#define MATRIX_HEIGHT 5
#define MATRIX_GPIO   42

static const char *EX_TAG = "wiring_test";

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

rcl_publisher_t publisher;
rcl_subscription_t subscriber;
std_msgs__msg__Int32 send_msg;
std_msgs__msg__Int32 recv_msg;

int command = 0;

bool blink_left = false;
bool blink_right = false;

// Cycles through all 8 (layout x first_line_reversed) combinations,
// drawing "HI" on each and printing the combo to the log. Watch the
// panel and match it to the log line where "HI" reads correctly and
// left-to-right (not mirrored). Use that combo in your real config.
static void wiring_self_test(led_matrix_t *unused)
{
    (void)unused;
    struct { lm_layout_t layout; const char *name; } layouts[] = {
        { LM_LAYOUT_ROWS,               "LM_LAYOUT_ROWS" },
        { LM_LAYOUT_ROWS_SERPENTINE,    "LM_LAYOUT_ROWS_SERPENTINE" },
        { LM_LAYOUT_COLUMNS,            "LM_LAYOUT_COLUMNS" },
        { LM_LAYOUT_COLUMNS_SERPENTINE, "LM_LAYOUT_COLUMNS_SERPENTINE" },
    };

    for (int li = 0; li < 4; li++) {
        for (int rev = 0; rev < 2; rev++) {
            led_matrix_config_t config = {
                .gpio_num = MATRIX_GPIO,
                .width = MATRIX_WIDTH,
                .height = MATRIX_HEIGHT,
                .layout = layouts[li].layout,
                .first_line_reversed = (bool)rev,
                .brightness = 40,
            };
            led_matrix_t *m = led_matrix_init(&config);
            if (!m) continue;

            ESP_LOGI(EX_TAG, "layout=%s first_line_reversed=%d",
                     layouts[li].name, rev);

            lm_clear(m);
            lm_draw_text(m, 0, 0, "HI", LM_WHITE);
            lm_show(m);
            vTaskDelay(pdMS_TO_TICKS(2500));

            led_matrix_deinit(m);
        }
    }
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
    command = (int) msg->data;
}

void micro_ros_task(void * arg)
{

    ESP_LOGI(EX_TAG, "micro_ros_task start");

	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// Create init_options.
	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	RCCHECK(rcl_init_options_init(&init_options, allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
	rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    ESP_LOGI(EX_TAG, "micro_ros_task set udp options");

	// Static Agent IP and port can be used instead of autodisvery.
	RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
	//RCCHECK(rmw_uros_discover_agent(rmw_options));
#endif
    ESP_LOGI(EX_TAG, "micro_ros_task support init");

// Setup support structure.
	RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
    ESP_LOGI(EX_TAG, "micro_ros_task support init done");

	// Create node.
	rcl_node_t node = rcl_get_zero_initialized_node();
	RCCHECK(rclc_node_init_default(&node, "ttiny29x5_publisher_subscriber_rclc", "", &support));
    ESP_LOGI(EX_TAG, "micro_ros_task node init done");

	// Create publisher.
	RCCHECK(rclc_publisher_init_default(
		&publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"ttiny29x5_publisher"));

	// Create subscriber.
	RCCHECK(rclc_subscription_init_default(
		&subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"ttiny29x5_subscriber"));

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
    led_matrix_config_t config = {
        .gpio_num = MATRIX_GPIO,
        .width = MATRIX_WIDTH,
        .height = MATRIX_HEIGHT,
        // If text still looks wrong, try the other 3 combinations of
        // layout / first_line_reversed -- see wiring_self_test() below
        // and README.md "Wiring / layout" section.
        .layout = LM_LAYOUT_COLUMNS_SERPENTINE,
        .first_line_reversed = false,
        .brightness = 60, // keep this low to start with -- 145 LEDs at full
                           // white brightness draws a *lot* of current
    };

    led_matrix_t *matrix = led_matrix_init(&config);
    if (!matrix) {
        return; // check the "led_matrix_gfx" log tag for the reason
    }

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

    // Uncomment this to walk through all 8 layout combinations one at a
    // time and find the one that matches your physical wiring, instead of
    // guessing. See its definition below.
    // wiring_self_test(matrix);

    const int blink_time = 500;

    while (true){

        switch (command)
        {
        case 0:
            lm_clear(matrix);
            lm_show(matrix);
            vTaskDelay(pdMS_TO_TICKS(100));        
            break;
        case 1:
            // Blink left
            lm_clear(matrix);
            lm_show(matrix);
            vTaskDelay(pdMS_TO_TICKS(blink_time));
            lm_fill_rect(matrix, 0, 0, MATRIX_WIDTH / 2, MATRIX_HEIGHT, LM_DARK_ORANGE);
            lm_show(matrix);
            vTaskDelay(pdMS_TO_TICKS(blink_time));
            break;
        case 2:
            // Blink Right
            lm_clear(matrix);
            lm_show(matrix);
            vTaskDelay(pdMS_TO_TICKS(blink_time));
            lm_fill_rect(matrix, MATRIX_WIDTH / 2, 0, MATRIX_WIDTH, MATRIX_HEIGHT, LM_DARK_ORANGE);
            lm_show(matrix);
            vTaskDelay(pdMS_TO_TICKS(blink_time));
            break;
        case 3:
            // --- 1. static shapes ---------------------------------------------
            lm_clear(matrix);
            lm_fill_rect(matrix, 0, 0, MATRIX_WIDTH, MATRIX_HEIGHT, LM_WHITE);
            lm_draw_rect(matrix, 0, 0, MATRIX_WIDTH, MATRIX_HEIGHT, LM_RED);
            lm_draw_line(matrix, 0, 0, MATRIX_WIDTH - 1, MATRIX_HEIGHT - 1, LM_GREEN);
            lm_fill_circle(matrix, 14, 2, 2, LM_BLUE);
            lm_show(matrix);
            vTaskDelay(pdMS_TO_TICKS(3000));

            // --- 2. static text --------------------------------------------
            lm_clear(matrix);
            lm_draw_text(matrix, 0, 0, "HI", LM_DARK_ORANGE);
            lm_show(matrix);
            vTaskDelay(pdMS_TO_TICKS(3000));

            // --- 3. scrolling text -------------------------------------------
            char *message = "HELLO FROM ESP32S3 - PURE ESP-IDF NEOPIXEL MATRIX";
            int text_w = lm_text_width(message);

            for (int offset = MATRIX_WIDTH; offset > -text_w; offset--) {
                lm_clear(matrix);
                lm_draw_text(matrix, offset, 0, message, LM_WHITE);
                lm_show(matrix);
                vTaskDelay(pdMS_TO_TICKS(60)); // scroll speed
            }

            // --- 4. scrolling text -------------------------------------------
            const char *message01 = "abcdefghijklmnopqrstuvwxyz";
            text_w = lm_text_width(message01);

            for (int offset = MATRIX_WIDTH; offset > -text_w; offset--) {
                lm_clear(matrix);
                lm_draw_text(matrix, offset, 0, message01, LM_YELLOW);
                lm_show(matrix);
                vTaskDelay(pdMS_TO_TICKS(60)); // scroll speed
            }    
            
            // --- 5. scrolling text -------------------------------------------
            const char *message02 = "ABCDEFGHIJKLMNOPQRSTUVWXYZ";
            text_w = lm_text_width(message02);

            for (int offset = MATRIX_WIDTH; offset > -text_w; offset--) {
                lm_clear(matrix);
                lm_draw_text(matrix, offset, 0, message02, LM_WHITE);
                lm_show(matrix);
                vTaskDelay(pdMS_TO_TICKS(60)); // scroll speed
            }

            // --- 5. scrolling text -------------------------------------------
            const char *message03 = "0123456789";
            text_w = lm_text_width(message03);

            for (int offset = MATRIX_WIDTH; offset > -text_w; offset--) {
                lm_clear(matrix);
                lm_draw_text(matrix, offset, 0, message03, LM_ORANGE);
                lm_show(matrix);
                vTaskDelay(pdMS_TO_TICKS(60)); // scroll speed
            }

            // Audi
            // Blink left
            for (int blinkCounter = 0; blinkCounter <= 5; blinkCounter++){
                lm_clear(matrix);
                lm_show(matrix);
                vTaskDelay(pdMS_TO_TICKS(blink_time));
                for (int width = 0; width <= (MATRIX_WIDTH / 2); width++)
                {
                    lm_fill_rect(matrix, (MATRIX_WIDTH / 2) - width, 0, width , MATRIX_HEIGHT, LM_DARK_ORANGE);
                    lm_show(matrix);
                    vTaskDelay(pdMS_TO_TICKS(blink_time / (MATRIX_WIDTH / 2)));
                }
            }
            // Blink Right
            for (int blinkCounter = 0; blinkCounter <= 5; blinkCounter++){
                lm_clear(matrix);
                lm_show(matrix);
                vTaskDelay(pdMS_TO_TICKS(blink_time));
                for (int width = 0; width <= (MATRIX_WIDTH / 2); width++)
                {
                    lm_fill_rect(matrix, (MATRIX_WIDTH / 2), 0, width, MATRIX_HEIGHT, LM_DARK_ORANGE);
                    lm_show(matrix);
                    vTaskDelay(pdMS_TO_TICKS(blink_time / (MATRIX_WIDTH / 2)));
                }
            }

            // Normal Blink
            // Blink left
            for (int blinkCounter = 0; blinkCounter <= 5; blinkCounter++){
                lm_clear(matrix);
                lm_show(matrix);
                vTaskDelay(pdMS_TO_TICKS(blink_time));
                lm_fill_rect(matrix, 0, 0, MATRIX_WIDTH / 2, MATRIX_HEIGHT, LM_DARK_ORANGE);
                lm_show(matrix);
                vTaskDelay(pdMS_TO_TICKS(blink_time));
            }

            // Blink Right
            for (int blinkCounter = 0; blinkCounter <= 5; blinkCounter++){
                lm_clear(matrix);
                lm_show(matrix);
                vTaskDelay(pdMS_TO_TICKS(blink_time));
                lm_fill_rect(matrix, MATRIX_WIDTH / 2, 0, MATRIX_WIDTH, MATRIX_HEIGHT, LM_DARK_ORANGE);
                lm_show(matrix);
                vTaskDelay(pdMS_TO_TICKS(blink_time));
            }
            lm_clear(matrix);

            // --- static text --------------------------------------------
            lm_draw_text(matrix, (MATRIX_WIDTH / 2), 0, "3", LM_RED);
            lm_show(matrix);
            vTaskDelay(pdMS_TO_TICKS(1000));

            // --- static text --------------------------------------------
            lm_draw_text(matrix, (MATRIX_WIDTH / 4), 0, "2", LM_BLUE);
            lm_show(matrix);
            vTaskDelay(pdMS_TO_TICKS(1000));

            // --- static text --------------------------------------------
            lm_draw_text(matrix, (MATRIX_WIDTH / 4) * 3, 0, "1", LM_GREEN);
            lm_show(matrix);
            vTaskDelay(pdMS_TO_TICKS(1000));

            // --- dynamic text --------------------------------------------
            for (int counter = 1; counter <= 10; counter ++){
                char text[3];
                itoa(counter, text, 10);
                lm_clear(matrix);
                lm_draw_text(matrix, (MATRIX_WIDTH / 4) * 2, 0, text, LM_MAGENTA);
                lm_show(matrix);
                vTaskDelay(pdMS_TO_TICKS(100));
            }

            vTaskDelay(pdMS_TO_TICKS(2500));

            lm_clear(matrix);
            lm_show(matrix);
        /* code */
            break;
        default:
            vTaskDelay(pdMS_TO_TICKS(100));

            break;
        }
    
    }
}
