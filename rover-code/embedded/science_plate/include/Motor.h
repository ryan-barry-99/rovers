#include <iostream>

class Encoder {
public:
    // Encoder class definition goes here
};

template<typename T>
class Motor {
public:
    virtual ~Motor() {} // Virtual destructor to ensure proper cleanup

    // Constructor
    Motor(int pin, Encoder encoder) {}

    // Helper methods as pure virtual functions
    virtual bool setPower(float power) = 0;
    virtual float getPower() const = 0;
    virtual bool setPosition(T target) = 0;
    virtual T getPosition() const = 0;
    virtual bool stop() = 0;
};
