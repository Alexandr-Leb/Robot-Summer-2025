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

enum PetState {PrePet, Pet1, Ramp, Pet2, Pet3, Pet4, Pet5, Pet6, Debris, Pet7, PostPet};
extern PetState currentPetState;

enum TaskState {TapeFollow, TapeFind};
extern TaskState currentTaskState;

enum StepState_PrePet {ClearBucket, FindGate, ClearDoorway, TurnArm};
extern StepState_PrePet currentStepState_PrePet;

enum StepState_Pet1 {FindTarget1, LiftBasket, ArmSearchPreset1, PetSearch1, PetFound1, PetGrab1, ReturnArm1};
extern StepState_Pet1 currentStepState_Pet1;

enum StepState_Ramp {FindRamp, AlignRamp, RampCorrection, DropPet1, ClimbRamp, InchForwards};
extern StepState_Ramp currentStepState_Ramp;

enum StepState_Pet2 {FindTarget2, ArmSearchPreset2, PetSearch2, PetFound2, PetGrab2, ReturnArm2, DropPet2};
extern StepState_Pet2 currentStepState_Pet2;

enum StepState_Pet3 {FindTarget3, ArmSearchPreset3, PetSearch3, PetFound3, PetGrab3, DropPet3};
extern StepState_Pet3 currentStepState_Pet3;

enum StepState_Pet4 {FindTarget4, ArmSearchPreset4, PetSearch4, PetFound4, PetGrab4, DropPet4};
extern StepState_Pet4 currentStepState_Pet4;

enum StepState_Pet5 {FindTarget5, ArmSearchPreset5, PetSearch5, PetFound5, PetGrab5, DropPet5};
extern StepState_Pet5 currentStepState_Pet5;

enum StepState_Pet6 {FindTarget6, ArmSearchPreset6, PetSearch6, PetFound6, PetGrab6, DropPet6};
extern StepState_Pet6 currentStepState_Pet6;

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
