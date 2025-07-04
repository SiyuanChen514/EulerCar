#include "servo_response/servo_util.hpp"
#include <thread>
#include <chrono>


void set_period(int id, int period){
    int period_fd;
    switch (id) {
        case 1:
            period_fd = open(PWM1_PERIOD_PATH, O_WRONLY);
            break;
        case 15:
            period_fd = open(PWM15_PERIOD_PATH, O_WRONLY);
            break;
        default:
            period_fd = open(PWM1_PERIOD_PATH, O_WRONLY);
    }
    
    
    if (period_fd == -1) {
        perror("Error opening period");
        exit(EXIT_FAILURE);
    }

    dprintf(period_fd, "%d", period);

    close(period_fd);
}


void set_duty_cycle(int id, int dc){
    int duty_cycle_fd;
    switch (id) {
        case 1:
            duty_cycle_fd = open(PWM1_DUTY_CYCLE_PATH, O_WRONLY);
            break;
        case 15:
            duty_cycle_fd = open(PWM15_DUTY_CYCLE_PATH, O_WRONLY);
            break;
        default:
            duty_cycle_fd = open(PWM1_DUTY_CYCLE_PATH, O_WRONLY);
    }
    
    if (duty_cycle_fd == -1) {
        perror("Error opening duty cycle");
        exit(EXIT_FAILURE);
    }

    dprintf(duty_cycle_fd, "%d", dc);

    close(duty_cycle_fd);
}



void disable_pwm(int id){
    int enable_fd;
    switch (id) {
        case 1:
            enable_fd = open(PWM1_ENABLE_PATH, O_WRONLY);
            break;
        case 15:
            enable_fd = open(PWM15_ENABLE_PATH, O_WRONLY);
            break;
        default:
            enable_fd = open(PWM1_ENABLE_PATH, O_WRONLY);
    }

    if (enable_fd == -1) {
        perror("Error opening enable");
        exit(EXIT_FAILURE);
    }

    if (write(enable_fd, "0", 2) == -1) {
        perror("Error disabling PWM");
        close(enable_fd);
        exit(EXIT_FAILURE);
    }

    close(enable_fd);
}

void run_servo(int id, int dc){
    int export_fd = open(PWM_EXPORT_PATH, O_WRONLY);
    if (export_fd == -1) {
        perror("Error opening export");
        exit(EXIT_FAILURE);
    }
    dprintf (export_fd, "%d", id);
    close(export_fd);

    set_period(id, 20000000);
    set_duty_cycle(id, dc);
    
    int enable_fd;
    switch (id) {
        case 1:
            enable_fd = open(PWM1_ENABLE_PATH, O_WRONLY);
            break;
        case 15:
            enable_fd = open(PWM15_ENABLE_PATH, O_WRONLY);
            break;
        default:
            enable_fd = open(PWM1_ENABLE_PATH, O_WRONLY);
    }

    if (enable_fd == -1) {
        perror("Error opening enable");
        exit(EXIT_FAILURE);
    }

    if (write(enable_fd, "1", 2) == -1) {
        perror("Error enabling PWM");
        close(enable_fd);
        exit(EXIT_FAILURE);
    }
    close(enable_fd);
}

void close_servo(int id){
    int unexport_fd = open(PWM_UNEXPORT_PATH, O_WRONLY);
    if (unexport_fd == -1) {
        perror("Error opening unexport");
        exit(EXIT_FAILURE);
    }
    disable_pwm(id);
    dprintf (unexport_fd, "%d", id);
    close(unexport_fd);
}

// void down(){
//     run_servo(VERTICAL_SERVO_ID, CLOCKWISE);
//     std::this_thread::sleep_for(std::chrono::milliseconds(DELAY_TIME_MS));
//     close_servo(VERTICAL_SERVO_ID);
// }

// void up(){
//     run_servo(VERTICAL_SERVO_ID, COUNTERCLOCKWISE);
//     std::this_thread::sleep_for(std::chrono::milliseconds(DELAY_TIME_MS));
//     close_servo(VERTICAL_SERVO_ID);
// }

void open(uint16_t time){
    run_servo(HORIZONTAL_SERVO_ID, 500000);
    std::this_thread::sleep_for(std::chrono::milliseconds(time));
    close_servo(HORIZONTAL_SERVO_ID);
}

void close(uint16_t time){
    run_servo(HORIZONTAL_SERVO_ID, 2500000);
    std::this_thread::sleep_for(std::chrono::milliseconds(time));
    close_servo(HORIZONTAL_SERVO_ID);
}