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
		String RemoteScalerState = "MANUAL";	//states are AUTO and MANUAL
	
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
		int autoYaw = 0;
		int autoAileron = 0;
		int autoElevator = 0;

		int remoteThrust = 0;
		int remoteYaw = 0;
		int remoteAileron = 0;
		int remoteElevator = 0;

  //THREADS
    ThreadController controller = ThreadController();
    Thread remoteScalerThread = Thread();
    Thread autoManualThread = Thread();
    Thread actuatorExecuteThread = Thread();



  //ACTUATOR VARIABLES -------------------------------------------------------------------------------

    //PID VARIABLES

    //HEIGHT PID
      double oldHeight = 0;
      double newHeight = 0;
      double hCheck = 0;

      double hoverHeight = 0;
      
      double pidThrust = 0;
      double hoverThrust = 0;
      double autoThrust = 0;

    //VELOCITY PID
      double velocity = 0;
      double vCheck = 0;

      double velGoal = 0;

      double autoVelocity = 0;
      
    //PIDS
		PID heightPID(&newHeight,&pidThrust,&hoverHeight,0.027,0.027,0, DIRECT);
		PID velocityPID(&velocity,&autoVelocity,&velGoal,1,0,0, DIRECT);


//END VARIABLE DECLARATIONS-------------------------------------------------------------------------------------------------------------------------------------
void setup() {
	Serial.begin(9600);
	loadPins();
	setupThreads();
	loadPIDS();
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


	//ACTUATOR EXECUTION (not part of the subsumption architecture)
	actuatorExecuteThread.setInterval(100);
	actuatorExecuteThread.onRun(executeActuators);
	controller.add(&actuatorExecuteThread);
}

void loadPIDS(){
	heightPID.SetMode(MANUAL);
	heightPID.SetOutputLimits(0,25);

	velocityPID.SetMode(MANUAL);
	velocityPID.SetOutputLimits(-3,3);

	heightPID.SetSampleTime(90);
	velocityPID.SetSampleTime(90);
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
    		RemoteScalerState = "MANUAL";
    		autoThrust = 0;
    		remoteThrust = abs(remoteThrust - 138) < 2 ? 138 : remoteThrust;
    		remoteYaw = abs(remoteYaw - 188) < 2 ? 188 : remoteYaw;
    		remoteAileron = abs(remoteAileron - 188) < 2 ? 188 : remoteAileron; 
    		remoteElevator = abs(remoteElevator - 188) < 2 ? 188 : remoteElevator;
    		break;
    	case false:
    		RemoteScalerState = "AUTO";
    		autoThrust = 138;
    		remoteThrust = 0;
    		remoteYaw = abs(remoteYaw - 188) < 5 ? 188 : remoteYaw;
    		remoteAileron = abs(remoteAileron - 188) < 5 ? 188 : remoteAileron; 
    		remoteElevator = abs(remoteElevator - 188) < 5 ? 188 : remoteElevator; 
    }
}
//END REMOTE SCALER MODULE----------------------------

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
  heightPID.Compute();
  velocityPID.Compute();
  thrust = remoteThrust + autoThrust + hoverThrust + autoVelocity;
  yawOut = remoteYaw + autoYaw;
  aileronOut = remoteAileron + autoAileron;
  elevatorOut = remoteElevator + autoElevator;

  Serial.println(thrust);
  
  analogWrite(throttleOutputPin, (int)thrust);
  analogWrite(yawOutputPin, (int)yawOut);
  analogWrite(aileronOutputPin, (int)aileronOut);
  analogWrite(elevatorOutputPin, (int)elevatorOut);
}

