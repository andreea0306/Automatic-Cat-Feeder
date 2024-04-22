#include <iostream>
#include <chrono>
#include <thread>
#include <functional>
#include <cstdio>
#include <wiringPi.h>
#include <lgpio.h>

int const piPin = 1;

class Timer {
public:
    // constructor
    Timer(std::chrono::milliseconds interval, std::function<void()> callback)
        : interval(interval), callback(callback), running(false) {}

    void start() {
        running = true;
        thread = std::thread([this]() {
            while (running) {
                // wait for the interval time to be spent and after that call ISR function
		std::this_thread::sleep_for(interval);
                callback();
            }
            });
    }

    void stop() {
        running = false;
        if (thread.joinable()) {
            thread.join();
        }
    }

protected:
    std::chrono::milliseconds interval; // interval of timer 
    std::function<void()> callback; // ISR function
    std::thread thread; 
    bool running; // status of timer
};
/*
void moveServo() {

    std::cout<<"[DEBUG]: first move"<<std::endl;
    pwmWrite(piPin, 550);
    delay(500);
    std::cout<<"[DEBUG]: second move back"<<std::endl;
    pwmWrite(piPin, 350);
    delay(500);

}

void timerCallback() {
    std::cout << "[DEBUG]: Timer ISR called!"<<std::endl;
    moveServo();
}
*/
