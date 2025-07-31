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

enum TaskState {TapeFollow, TapeFind};
extern TaskState currentTaskState;

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
