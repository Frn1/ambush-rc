// SPDX-License-Identifier: Apache-2.0
// Copyright 2021 Ricardo Quesada
// http://retro.moe/unijoysticle2

#include "sdkconfig.h"

#include <Arduino.h>
#include <Bluepad32.h>

//
// README FIRST, README FIRST, README FIRST
//
// Bluepad32 has a built-in interactive console.
// By default, it is enabled (hey, this is a great feature!).
// But it is incompatible with Arduino "Serial" class.
//
// Instead of using, "Serial" you can use Bluepad32 "Console" class instead.
// It is somewhat similar to Serial but not exactly the same.
//
// Should you want to still use "Serial", you have to disable the Bluepad32's console
// from "sdkconfig.defaults" with:
//    CONFIG_BLUEPAD32_USB_CONSOLE_ENABLE=n

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedController(ControllerPtr ctl) {
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            Console.printf("CALLBACK: Controller is connected, index=%d\n", i);
            // Additionally, you can get certain gamepad properties like:
            // Model, VID, PID, BTAddr, flags, etc.
            ControllerProperties properties = ctl->getProperties();
            Console.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName(), properties.vendor_id,
                           properties.product_id);
            myControllers[i] = ctl;
            foundEmptySlot = true;
            break;
        }
    }
    if (!foundEmptySlot) {
        Console.println("CALLBACK: Controller connected, but could not found empty slot");
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    bool foundController = false;

    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            Console.printf("CALLBACK: Controller disconnected from index=%d\n", i);
            myControllers[i] = nullptr;
            foundController = true;
            break;
        }
    }

    if (!foundController) {
        Console.println("CALLBACK: Controller disconnected, but not found in myControllers");
    }
}

void dumpGamepad(ControllerPtr ctl) {
    Console.printf(
        "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
        "misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d\n",
        ctl->index(),        // Controller Index
        ctl->dpad(),         // D-pad
        ctl->buttons(),      // bitmask of pressed buttons
        ctl->axisX(),        // (-511 - 512) left X Axis
        ctl->axisY(),        // (-511 - 512) left Y axis
        ctl->axisRX(),       // (-511 - 512) right X axis
        ctl->axisRY(),       // (-511 - 512) right Y axis
        ctl->brake(),        // (0 - 1023): brake button
        ctl->throttle(),     // (0 - 1023): throttle (AKA gas) button
        ctl->miscButtons(),  // bitmask of pressed "misc" buttons
        ctl->gyroX(),        // Gyro X
        ctl->gyroY(),        // Gyro Y
        ctl->gyroZ(),        // Gyro Z
        ctl->accelX(),       // Accelerometer X
        ctl->accelY(),       // Accelerometer Y
        ctl->accelZ()        // Accelerometer Z
    );
}

void dumpMouse(ControllerPtr ctl) {
    Console.printf("idx=%d, buttons: 0x%04x, scrollWheel=0x%04x, delta X: %4d, delta Y: %4d\n",
                   ctl->index(),        // Controller Index
                   ctl->buttons(),      // bitmask of pressed buttons
                   ctl->scrollWheel(),  // Scroll Wheel
                   ctl->deltaX(),       // (-511 - 512) left X Axis
                   ctl->deltaY()        // (-511 - 512) left Y axis
    );
}

void dumpKeyboard(ControllerPtr ctl) {
    static const char* key_names[] = {
        // clang-format off
        // To avoid having too much noise in this file, only a few keys are mapped to strings.
        // Starts with "A", which is offset 4.
        "A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L", "M", "N", "O", "P", "Q", "R", "S", "T", "U", "V",
        "W", "X", "Y", "Z", "1", "2", "3", "4", "5", "6", "7", "8", "9", "0",
        // Special keys
        "Enter", "Escape", "Backspace", "Tab", "Spacebar", "Underscore", "Equal", "OpenBracket", "CloseBracket",
        "Backslash", "Tilde", "SemiColon", "Quote", "GraveAccent", "Comma", "Dot", "Slash", "CapsLock",
        // Function keys
        "F1", "F2", "F3", "F4", "F5", "F6", "F7", "F8", "F9", "F10", "F11", "F12",
        // Cursors and others
        "PrintScreen", "ScrollLock", "Pause", "Insert", "Home", "PageUp", "Delete", "End", "PageDown",
        "RightArrow", "LeftArrow", "DownArrow", "UpArrow",
        // clang-format on
    };
    static const char* modifier_names[] = {
        // clang-format off
        // From 0xe0 to 0xe7
        "Left Control", "Left Shift", "Left Alt", "Left Meta",
        "Right Control", "Right Shift", "Right Alt", "Right Meta",
        // clang-format on
    };
    Console.printf("idx=%d, Pressed keys: ", ctl->index());
    for (int key = Keyboard_A; key <= Keyboard_UpArrow; key++) {
        if (ctl->isKeyPressed(static_cast<KeyboardKey>(key))) {
            const char* keyName = key_names[key - 4];
            Console.printf("%s,", keyName);
        }
    }
    for (int key = Keyboard_LeftControl; key <= Keyboard_RightMeta; key++) {
        if (ctl->isKeyPressed(static_cast<KeyboardKey>(key))) {
            const char* keyName = modifier_names[key - 0xe0];
            Console.printf("%s,", keyName);
        }
    }
    Console.printf("\n");
}

void dumpBalanceBoard(ControllerPtr ctl) {
    Console.printf("idx=%d,  TL=%u, TR=%u, BL=%u, BR=%u, temperature=%d\n",
                   ctl->index(),        // Controller Index
                   ctl->topLeft(),      // top-left scale
                   ctl->topRight(),     // top-right scale
                   ctl->bottomLeft(),   // bottom-left scale
                   ctl->bottomRight(),  // bottom-right scale
                   ctl->temperature()   // temperature: used to adjust the scale value's precision
    );
}

template <typename T>
T betterMap(const T x, const T in_min, const T in_max, const T out_min, const T out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

template <typename T, typename O>
O linearInterpolation(const T x, const T* x_values, const O* y_values, size_t len) {
    size_t start_index = 0;
    size_t end_index = 1;

    for (size_t i = 1; i < len - 1; i++) {
        if (x >= x_values[i]) {
            if (x < x_values[i + 1]) {
                start_index = i;
                end_index = i + 1;
                break;
            }
        }
    }

    return betterMap(x, x_values[start_index], x_values[end_index], y_values[start_index], y_values[end_index]);
}

struct MotorSpeeds {
    uint8_t left_speed;
    bool left_forward;
    uint8_t right_speed;
    bool right_forward;
};

struct MotorSpeeds calculateMotorSpeeds(int8_t x, int8_t y) {
    constexpr float left_x_values[] = {-PI, -PI / 2, 0, PI / 2, PI};
    constexpr float left_y_values[] = {0, -1, 1, 1, 0};
    constexpr float right_x_values[] = {-PI, -PI / 2, 0, PI / 2, PI};
    constexpr float right_y_values[] = {1, -1, 0, 1, 1};

    float angle = atan2f(y, x);
    float magnitude = constrain(sqrt(x * x + y * y), 0.0, 500.0) / 500.0;

    Serial.printf("%f\t%f\n", angle, magnitude);

    float left_speed = linearInterpolation(angle, left_x_values, left_y_values, sizeof(left_x_values) / sizeof(float));
    bool left_forward = true;
    if (left_speed < 0) {
        left_speed = abs(left_speed);
        left_forward = false;
    }

    float right_speed =
        linearInterpolation(angle, right_x_values, right_y_values, sizeof(right_x_values) / sizeof(float));
    bool right_forward = true;
    if (right_speed < 0) {
        right_speed = abs(right_speed);
        right_forward = false;
    }

    struct MotorSpeeds result = {
        (uint8_t)(uint8_t)round(left_speed * magnitude * 255.0),
        left_forward,
        (uint8_t)round(right_speed * magnitude * 255.0),
        right_forward,
    };
    return result;
}

constexpr uint8_t PINS_MOTOR_LEFT_FORWARD = 25;
constexpr uint8_t PINS_MOTOR_LEFT_BACKWARD = 26;
constexpr uint8_t PINS_MOTOR_RIGHT_BACKWARD = 33;
constexpr uint8_t PINS_MOTOR_RIGHT_FORWARD = 32;

void updateMotors(int32_t x, int32_t y) {
    struct MotorSpeeds motor_speeds = calculateMotorSpeeds(x, y);

    if (motor_speeds.left_forward) {
        analogWrite(PINS_MOTOR_LEFT_FORWARD, motor_speeds.left_speed);
        analogWrite(PINS_MOTOR_LEFT_BACKWARD, 0);
    } else {
        analogWrite(PINS_MOTOR_LEFT_FORWARD, 0);
        analogWrite(PINS_MOTOR_LEFT_BACKWARD, motor_speeds.left_speed);
    }

    if (motor_speeds.right_forward) {
        analogWrite(PINS_MOTOR_RIGHT_FORWARD, motor_speeds.right_speed);
        analogWrite(PINS_MOTOR_RIGHT_BACKWARD, 0);
    } else {
        analogWrite(PINS_MOTOR_RIGHT_FORWARD, 0);
        analogWrite(PINS_MOTOR_RIGHT_BACKWARD, motor_speeds.right_speed);
    }
}

void processGamepad(ControllerPtr ctl) {
    int32_t x = ctl->axisX();
    int32_t y = ctl->axisY();

    updateMotors(x, y);
}

void processBalanceBoard(ControllerPtr ctl) {
    uint16_t tl = ctl->topLeft();
    uint16_t tr = ctl->topRight();
    uint16_t bl = ctl->bottomLeft();
    uint16_t br = ctl->bottomRight();

    int32_t x = (tr + br) - (tl + bl);
    int32_t y = (tr + tl) - (br + bl);

    updateMotors(x, y);
}

void processControllers() {
    for (auto myController : myControllers) {
        if (myController && myController->isConnected() && myController->hasData()) {
            if (myController->isGamepad()) {
                processGamepad(myController);
            } else if (myController->isBalanceBoard()) {
                processBalanceBoard(myController);
            } else {
                Console.printf("Unsupported controller\n");
            }
        }
    }
}

// Arduino setup function. Runs in CPU 1
void setup() {
    Console.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t* addr = BP32.localBdAddress();
    Console.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

    // Setup the Bluepad32 callbacks
    BP32.setup(&onConnectedController, &onDisconnectedController);

    // Enables mouse / touchpad support for gamepads that support them.
    // When enabled, controllers like DualSense and DualShock4 generate two connected devices:
    // - First one: the gamepad
    // - Second one, which is a "virtual device", is a mouse.
    // By default, it is disabled.
    BP32.enableVirtualDevice(false);

    // Enables the BLE Service in Bluepad32.
    // This service allows clients, like a mobile app, to setup and see the state of Bluepad32.
    // By default, it is disabled.
    BP32.enableBLEService(true);

    pinMode(PINS_MOTOR_LEFT_BACKWARD, OUTPUT);
    pinMode(PINS_MOTOR_LEFT_FORWARD, OUTPUT);
    pinMode(PINS_MOTOR_RIGHT_BACKWARD, OUTPUT);
    pinMode(PINS_MOTOR_RIGHT_FORWARD, OUTPUT);
}

// Arduino loop function. Runs in CPU 1.
void loop() {
    // This call fetches all the controllers' data.
    // Call this function in your main loop.
    bool dataUpdated = BP32.update();
    if (dataUpdated)
        processControllers();

    // The main loop must have some kind of "yield to lower priority task" event.
    // Otherwise, the watchdog will get triggered.
    // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
    // Detailed info here:
    // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time
    vTaskDelay(1);
}
