#include <Thread.h>
#include <ThreadController.h>
//SOURCE: https://github.com/ivanseidel/ArduinoThread
// MarvinArduino_v0.44.ino
//Packages
#include <PID_v1.h>
#include <Arduino.h>
#include <Wire.h>

//Global Variable Declarations
	
	//PINS

		//Input Pins
			const int autoSwitchInputPin = 14;

			//to use interrupts, these must be on pins 2, 3, or 18-21
			const int throttleInputPin = 20;
			const int yawInputPin = 18;
			const int aileronInputPin = 2; //control roll
			const int elevatorInputPin = 19; //control pitch

		//Output Pins 
			const int throttleOutputPin = 3;
			const int yawOutputPin = 5;
			const int aileronOutputPin = 11;
			const int elevatorOutputPin = 9;

	//STATE VARIABLES
		String RemoteScalerState = "VALUES";	//states are VALUES and ZEROS
		String AutoManualState = "MANUAL";		//states are AUTO and MANUAL

	//FLAG
		//boolean flipFlag; COULD add back in as state flag

	
	//INPUTS
		int autoSwitchValue = 0;

		volatile int throttle = 0;
		volatile int throttle_timer_start;
		volatile int last_throttle_interrupt;

		volatile int yawIn = 0;
		volatile int yaw_timer_start;
		volatile int last_yaw_interrupt;

		volatile int aileronIn = 0;
		volatile int aileron_timer_start;
		volatile int last_aileron_interrupt;

		volatile int elevatorIn = 0;
		volatile int elevator_timer_start;
		volatile int last_elevator_interrupt;

	//OUTPUTS
		float thrust = 138;
		float yawOut = 188;
		float aileronOut = 188;
		float elevatorOut = 188;

	//General
		int heightTarget = 80;
		int autoThrust = 0;
		int autoYaw = 188;
		int autoAileron = 188;
		int autoElevator = 189;

		int remoteThrust = 0;
		int remoteYaw = 0;
		int remoteAileron = 0;
		int remoteElevator = 0;

  //THREADS
    ThreadController controller = ThreadController();
    Thread remoteScalerThread = Thread();
    Thread autoManualThread = Thread();
    Thread actuatorExecuteThread = Thread();
    
void setup() {
	Serial.begin(9600);
	loadPins();
	setupThreads();
}

void loadPins(){
	//Input Pins
	pinMode(autoSwitchInputPin, INPUT);
	pinMode(throttleInputPin, INPUT);
	pinMode(yawInputPin, INPUT);
	pinMode(aileronInputPin, INPUT);
	pinMode(elevatorInputPin, INPUT);

	//Output Pins
	pinMode(throttleOutputPin, OUTPUT);
	pinMode(yawOutputPin, OUTPUT);
	pinMode(aileronOutputPin, OUTPUT);
	pinMode(elevatorOutputPin, OUTPUT);
}

void setupThreads(){
	//REMOTE SCALER MODULE
	remoteScalerThread.setInterval(25);
	remoteScalerThread.onRun(remoteScalerModule);
	controller.add(&remoteScalerThread);

	//AUTO-MANUAL MODULE
	autoManualThread.setInterval(25);
	autoManualThread.onRun(autoManualModule);
	controller.add(&autoManualThread);


	//ACTUATOR EXECUTION (not part of the subsumption architecture)
	actuatorExecuteThread.setInterval(100);
	actuatorExecuteThread.onRun(executeActuators);
	controller.add(&actuatorExecuteThread);
}
//FINISH SETUP--------------------------------------------------------------------------------------------------------------------------------------------------












//LAYER 1 ------------------------------------------------------------------------------------------------------------------------------------------------------

//REMOTE SCALER MODULE------------------------------
void remoteScalerModule() {
	if(throttle < 100){
    	remoteThrust = 138;
    } else {
    	remoteThrust = ((float) throttle / 7.854) - 5.112;
    }
    
    if(yawIn < 1000){
    	remoteYaw = 188;
    } else {
    	remoteYaw = ((float)yawIn * 0.121) + 4.804;
    }

    if(aileronIn < 1000){
    	remoteAileron = 188;
    } else {
    	remoteAileron = ((float)aileronIn * .113) + 17.96;
    }

    if(elevatorIn < 1000){
    	remoteElevator = 188;
    } else {
    	remoteElevator = ((float)elevatorIn * .114) + 16.88;
    }

    //update state
    switch(autoSwitchValue <= 1500){
    	case true:
    		RemoteScalerState = "VALUES";
    		//for this state, the general function of the module is all that is used
    		break;
    	case false:
    		RemoteScalerState = "ZEROS";
    		remoteThrust = abs(remoteThrust - 138) < 10 ? 0 : (remoteThrust - 138);
    		remoteYaw = abs(remoteYaw - 188) < 5 ? 0 : (remoteYaw - 188);
    		remoteAileron = abs(remoteAileron - 188) < 5 ? 0 : (remoteAileron - 188); 
    		remoteElevator = abs(remoteElevator - 188) < 5 ? 0 : (remoteElevator - 188); 
    }
}
//END REMOTE SCALER MODULE----------------------------


//AUTO-MANUAL MODULE----------------------------------
void autoManualModule(){
	switch(autoSwitchValue <= 1500){
    	case true:
    		AutoManualState = "MANUAL";
    		//turn PI/P controllers off
    		autoThrust = 0;
    		yawOut = remoteYaw;
    		aileronOut = remoteAileron;
    		elevatorOut = remoteElevator;
    		break;
    	case false:
    		AutoManualState = "AUTO";
    		//turn PI/P controllers on
        autoThrust = 138;
    		yawOut = autoYaw + remoteYaw;
    		aileronOut = autoAileron + remoteAileron;
    		elevatorOut = autoElevator + remoteElevator;
    }
}
//END AUTO-MANUAL MODULE------------------------------

//END LAYER 1 --------------------------------------------------------------------------------------------------------------------------------------------------





//SENSORS AND ACTUATORS----------------------------------------------------------------------------------------------------------------------------------------


//LOOP ---------------------------------------
//just calls sensor reading functions and updates sensor values
void loop(){
  readRemote(); //use remote input to update state of this module
  controller.run();
}
//END LOOP ----------------------------------


//REMOTE SENSOR -------------------------------------------------------------------------------------------------------------------------

long getPWM(int sourcePin){
	byte zeroCounter = 0;
	long pwm = 0;
	while(pwm == 0){
		pwm = pulseIn(sourcePin,HIGH,17000);
		zeroCounter++;
		if(zeroCounter == 5){
			return 0; 
		}
	}
	return pwm; 
}

void readRemote(){
	autoSwitchValue = getPWM(autoSwitchInputPin);
	throttle = (int)getPWM(throttleInputPin);
	yawIn = (int)getPWM(yawInputPin);
	aileronIn = (int)getPWM(aileronInputPin);
	elevatorIn = (int)getPWM(elevatorInputPin);
}

//END REMOTE SENSOR--------------------------------------------------------------------------------------------------------------------



//ACTUATORS----------------------------------------------------------------------------------------------------------------------------

void executeActuators(){
  // PI & P controller will go here
  thrust = remoteThrust + autoThrust;
  
  Serial.print("Thrust: ");
  Serial.println(thrust);
  Serial.print("Yaw: ");
  Serial.println(yawOut);
  Serial.print("Aileron: ");
  Serial.println(aileronOut);
  Serial.print("Elevator: ");
  Serial.println(elevatorOut);
  
  analogWrite(throttleOutputPin, (int)thrust);
  analogWrite(yawOutputPin, (int)yawOut);
  analogWrite(aileronOutputPin, (int)aileronOut);
  analogWrite(elevatorOutputPin, (int)elevatorOut);
}

/*Not being used right now
void stableAxes(){
	if(flipFlag){
		yawOut += 187;
		flipFlag = false;
	} else {
		yawOut += 188;
		flipFlag = true;
	}

	aileronOut += 188;
	elevatorOut += 190;
}
*/




