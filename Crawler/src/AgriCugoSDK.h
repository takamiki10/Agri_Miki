#ifndef CUGOCMDMODE_H
    #define CUGOCMDMODE_H

    #include <Ticker.h>              // Ticker library for timers
    #include <esp_task_wdt.h>        // ESP32 Watchdog Timer (WDT) API
    //------------------- Constants -------------------

    // CuGo specifications (V4 parameters)
    #define CUGO_WHEEL_RADIUS_L        0.03858
    #define CUGO_WHEEL_RADIUS_R        0.03858
    #define CUGO_TREAD                 0.380
    #define CUGO_ENCODER_RESOLUTION    600.0
    #define CUGO_MAX_MOTOR_RPM         130.0f    // Max motor speed
    #define CUGO_NORMAL_MOTOR_RPM      90.0f     // Normal motor speed

    // PID position control gain adjustment
    #define CUGO_L_COUNT_KP            50.0f
    #define CUGO_L_COUNT_KI            0.5f 
    #define CUGO_L_COUNT_KD            10.0f
    #define CUGO_R_COUNT_KP            50.0f
    #define CUGO_R_COUNT_KI            0.5f 
    #define CUGO_R_COUNT_KD            10.0f
    #define CUGO_L_MAX_COUNT_I         120.0f
    #define CUGO_R_MAX_COUNT_I         120.0f

    // Operation modes
    #define CUGO_RC_MODE               0
    #define CUGO_CMD_MODE              1

    // Odometry definitions
    #define CUGO_ODO_X                 0
    #define CUGO_ODO_Y                 1
    #define CUGO_ODO_THETA             2
    #define CUGO_ODO_DEGREE            3

    // Motor definitions
    #define CUGO_MOTOR_LEFT            0
    #define CUGO_MOTOR_RIGHT           1

    // LD2-related constants
    #define CUGO_LD2_COUNT_MAX         65536 
    #define NVIC_SYSRESETREQ           2

    //------------------- Global Variables -------------------

    // Operation and run mode variables
    extern int cugo_old_runmode;
    extern int cugo_runmode;

    // Count-related variables
    extern long int cugo_count_prev_L;
    extern long int cugo_count_prev_R;
    extern long int cugo_target_count_L;
    extern long int cugo_target_count_R;
    extern long int cugo_start_count_L;
    extern long int cugo_start_count_R;
    extern long int cugo_current_count_L;
    extern long int cugo_current_count_R;
    extern volatile long cugo_current_encoder_R;
    extern volatile long cugo_current_encoder_L;
    extern long int cugo_prev_encoder_L;
    extern long int cugo_prev_encoder_R;

    // Odometry variables
    extern unsigned long long int cugo_calc_odometer_time;
    extern float cugo_odometer_theta;
    extern float cugo_odometer_x;
    extern float cugo_odometer_y;
    extern float cugo_odometer_degree;
    extern long int cugo_odometer_count_theta;
    extern bool cugo_direction_L; 
    extern bool cugo_direction_R;

    // LD2-related variables
    extern volatile long cugo_ld2_id;
    extern volatile long cugo_ld2_feedback_hz;
    extern volatile long cugo_ld2_feedback_dutation;
    extern bool cugo_switching_reset;

    //------------------- Function Declarations -------------------

    // Initialization
    void cugo_init();
    void IRAM_ATTR cugo_timer_handler0(); // Updated to take no arguments
    void cugo_init_display();
    void cugo_reset_pid_gain();
    void cugo_check_mode_change();
    void cugo_reset();

    // Motor Control
    void cugo_rpm_direct_instructions(float left, float right);
    void cugo_stop();

    // Movement (Forward/Backward, Rotation)
    void cugo_move_forward(float target_distance);
    void cugo_move_forward(float target_distance, float target_rpm); 
    void cugo_move_forward_raw(float target_distance, float target_rpm);
    void cugo_turn_clockwise(float target_degree);
    void cugo_turn_clockwise(float target_degree, float target_rpm); 
    void cugo_turn_clockwise_raw(float target_degree, float target_rpm);
    void cugo_turn_counterclockwise(float target_degree);
    void cugo_turn_counterclockwise(float target_degree, float target_rpm); 
    void cugo_turn_counterclockwise_raw(float target_degree, float target_rpm);  

    // Polar Coordinate Movement
    void cugo_curve_theta_raw(float target_radius, float target_degree, float target_rpm);
    void cugo_curve_distance_raw(float target_radius, float target_distance, float target_rpm);

    // Wait Functions
    void cugo_wait(unsigned long long int wait_ms);
    void cugo_long_wait(unsigned long long int wait_seconds);

    // Odometry-related Functions
    float cugo_check_odometer(int check_number); 
    void cugo_start_odometer();
    void cugo_calc_odometer();
    void cugo_reset_odometer();

    // Utility Functions
    void cugo_calc_necessary_rotate(float degree); 
    void cugo_calc_necessary_count(float distance); 
    bool cugo_check_count_achievement(int motor_num_);
    void cugo_move_pid(float target_rpm, bool use_pid); 

    // Test Functions
    void cugo_test(int test_number);

    // LD2-related Functions
    // Utility Functions
    void ld2_float_to_frame(float data, long int start, unsigned char* index);
    void ld2_frame_to_float(unsigned char* index, long int start, float* data);
    void ld2_frame_to_short(unsigned char* index, long int start, short* data);

    // Communication Functions
    void ld2_write_cmd(unsigned char cmd[10]);
    void ld2_get_cmd();

    // Configuration Functions
    void ld2_set_feedback(unsigned char freq_index, unsigned char kindof_data);
    void ld2_set_control_mode(unsigned char mode);
    void ld2_set_encoder(unsigned char frame[12]);
    void ld2_encoder_reset();

    //------------------- Button-related Functions (if needed, uncomment) -------------------
    // #define CUGO_CMD_BUTTON_PIN 5
    // extern bool cugo_button_check;
    // extern volatile unsigned long long cugo_button_start_time;
    // extern int cugo_button_count;
    // void cugo_button_interrupt();
    // bool cugo_check_button();
    // int cugo_check_button_times();
    // void cugo_reset_button_times();
    // long int cugo_button_press_time();

#endif
