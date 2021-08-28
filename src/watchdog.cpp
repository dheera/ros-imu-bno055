#include "watchdog/watchdog.h"

namespace watchdog {

Watchdog::Watchdog() {
  callback = [] {
      std::cerr << "watchdog triggered\n";
      kill(getpid(), SIGTERM);
  };
  isRunning = false;
}

Watchdog::Watchdog(std::function<void()> _callback) {
  callback = _callback;
  isRunning = false;
}

Watchdog::~Watchdog() {
  stop();
}

void Watchdog::start(unsigned int _interval) {
    std::unique_lock<std::mutex> lock(mutex);
    if(isRunning) return;

    lastRefreshTime = std::chrono::steady_clock::now();
    interval = _interval;
    isRunning = true;
    thread = std::thread(&Watchdog::loop, this);
}

void Watchdog::stop() {
    std::unique_lock<std::mutex> lock(mutex);
    if(!isRunning) return;

    isRunning = false;
    stopCondition.notify_all();
    lock.unlock();
    thread.join();
}

void Watchdog::refresh() {
    std::unique_lock<std::mutex> lock(mutex);
    lastRefreshTime = std::chrono::steady_clock::now();
    stopCondition.notify_all();
}

void Watchdog::loop() {
    std::unique_lock<std::mutex> lock(mutex);
    while(isRunning) {
      if(stopCondition.wait_for(lock, std::chrono::milliseconds(interval)) == std::cv_status::timeout) {
        if(callback != nullptr) {
          isRunning = false;
          callback();
        }
      }
    }
}  

}  
