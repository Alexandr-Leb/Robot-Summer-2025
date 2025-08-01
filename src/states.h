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

enum PetState {PrePet, Pet1, Pet2, Pet3, Pet4, Pet5, Pet6, Pet7, PostPet};
extern PetState currentPetState;

enum TaskState {TapeFollow, TapeFind};
extern TaskState currentTaskState;

enum StepState_PrePet {ClearDoorway, Stop, LiftBasket, TurnArm};
extern StepState_PrePet currentStepState_PrePet;

enum StepState_Pet1 {FindRamp, ClimbRamp};
extern StepState_Pet1 currentStepState_Pet1;

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
