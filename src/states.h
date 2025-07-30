// --- Header Files --- //
// Header Files - External
#include <Arduino.h>

// Header Files - Internal
#include "esp_constants.h"

// --- Variables --- //
// Variables - Switches
extern bool initializeSwitchState;
extern bool resetSwitchState;

// Variables - States
enum SwitchState {Run, Initialize, Reset, Off};
extern SwitchState currentSwitchState;

// --- Function Headers --- //
void stateSetup();
void switchSetup();
void updateSwitchState();

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

void updateSwitchState() {
    // Read switch states
    initializeSwitchState = !(bool)digitalRead(INITIALIZE_SWITCH_PIN);
    resetSwitchState = !(bool)digitalRead(RESET_SWITCH_PIN);

    // Compute states
    if (initializeSwitchState && resetSwitchState) {
        currentSwitchState = SwitchState::Run;
    } else if (initializeSwitchState && !resetSwitchState) {
        currentSwitchState = SwitchState::Initialize;
    } else if (resetSwitchState) {
        currentSwitchState = SwitchState::Reset;
    } else {
        currentSwitchState = SwitchState::Off;
    }
}
