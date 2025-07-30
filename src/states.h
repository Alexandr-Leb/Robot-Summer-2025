// --- Header Files --- //
// Header Files - External
#include <Arduino.h>

// Header Files - Internal
#include "esp_constants.h"

// --- Variables --- //
// Variables - Switch
extern bool initializeSwitchState;
extern bool resetSwitchState;

// Variables - States
enum SwitchState {Off, Initialize, Reset, Run};
extern SwitchState currentSwitchState;

// --- Function Headers --- //
void stateSetup();
void switchSetup();

// --- Functions --- //
void stateSetup() {
    switchSetup();
}

void switchSetup() {
    // Pin Setup
    pinMode(INITIALIZE_SWITCH_PIN, INPUT_PULLUP);
    pinMode(RESET_SWITCH_PIN, INPUT_PULLUP);

    // Variable Initialization
    initializeSwitchState = digitalRead(INITIALIZE_SWITCH_PIN);
    resetSwitchState = digitalRead(RESET_SWITCH_PIN);
}

void readSwitchState() {
    // Read switch states
    initializeSwitchState = digitalRead(INITIALIZE_SWITCH_PIN);
    resetSwitchState = digitalRead(RESET_SWITCH_PIN);

    // Compute states
    if (initializeSwitchState && resetSwitchState) {
        currentSwitchState = SwitchState::run;
    } else if () 
}