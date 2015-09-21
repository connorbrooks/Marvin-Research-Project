// MarvinArduino_v0.44.ino
//Packages
#include <PID_v1.h>
#include <Arduino.h>
#include <Wire.h>

//Global Variable Declarations
	
	//PINS

		//Input Pins
			const int auxSwitchInputPin = 14;

			//to use interrupts, these must be on pins 2, 3, or 18-21
			const int throttleInputPin = 20;
			const int yawInputPin = 18;
			const int aileronInputPin = 2;
			const int elevatorInputPin = 19;

			const int heightInputPin = 12;
			const int heightInputPin2 = 4;
			const int lowFrontInputPin = 33;
			const int rightInputPin = 53;
			const int leftInputPin = 37;
			const int landSwitchInputPin = 10;

		//Output Pins 
			const int throttleOutputPin = 3;
			const int yawOutputPin = 5;
			const int aileronOutputPin = 11;
			const int elevatorOutputPin = 9;
			const int sonicTriggerPin = 7;
			const int sonicTriggerPin2 = 8;
			const int sonicTriggerPin3 = 31;
			const int sonicTriggerPin4 = 51;
			const int sonicTriggerPin5 = 35;

	//FLAG
		boolean autoOn;
		boolean landing;
		boolean takeOff;
		boolean flipFlag;
		boolean hoverHeightReached;

	//PID VARIABLES

		//HEIGHT PID
			double oldHeight;
			double newHeight;
			double hCheck;

			double baseHover;
			double hoverHeight;
			
			double autoThrust;
			double hoverThrust;
			double finalThrust;

		//VELOCITY PID
			double velocity;
			double vCheck;

			double velGoal = 0;
			double takeoffVelGoal = 3;
			double landVelGoal = -1;

			double autoVelocity;

		//FRONT PID
			double frontVelocity;

			double frontVelGoal = 0;

			double autoFront;

		//LEFT PID
			double leftVelocity;

			double leftVelGoal = 0;

			double autoLeft;

		//RIGHT PID
			double rightVelocity;

			double rightVelGoal = 0;

			double autoRight;

		//YAW PID
			//waiting on compass to implement

	//PIDS
		PID heightPID(&newHeight,&autoThrust,&hoverHeight,0.027,0.027,0, DIRECT);
		PID velocityPID(&velocity,&autoVelocity,&velGoal,1,0,0, DIRECT);
		PID frontPID(&frontVelocity,&autoFront,&frontVelGoal,0.2,0.2,0, DIRECT);
		PID leftPID(&leftVelocity,&autoLeft,&leftVelGoal,0.2,0.2,0, DIRECT);
		PID rightPID(&rightVelocity,&autoRight,&rightVelGoal,0.2,0.2,0, REVERSE);

	//INPUTS
		int switchValue;
		int landSwitchValue;

		volatile int throttle;
		volatile int throttle_timer_start;
		volatile int last_throttle_interrupt;

		volatile int yawIn;
		volatile int yaw_timer_start;
		volatile int last_yaw_interrupt;

		volatile int aileronIn;
		volatile int aileron_timer_start;
		volatile int last_aileron_interrupt;

		volatile int elevatorIn;
		volatile int elevator_timer_start;
		volatile int last_elevator_interrupt;

		double frontSensor;
		double rightSensor;
		double leftSensor;

		double frontTarget;
		double rightTarget;
		double leftTarget;

	//OUTPUTS
		float thrust;
		float yawOut;
		float aileronOut;
		float elevatorOut;

	//Raspberry Pi Comm
		unsigned char pi_In;
		unsigned char last_pi_In;
		unsigned char pi_Out;
		const byte HANDSHAKE = 33;
		const byte READY_INITIATE = 82; //'R'
		const byte READY_COMPLETE = 114; //'r'
		const byte TAKEOFF_INITIATE = 84; //'T'
		const byte TAKEOFF_COMPLETE = 116; // 't'
		const byte LANDING_INITIATE = 76; // 'L'
		const byte LANDING_COMPLETE = 108; // 'l'
		const byte HOVER_STEADY = 72; //'H'

	//General
		double startTime;
		double loopTime;
		int remoteOff;
		String mode;



void setup() {
	Serial.begin(9600);
	loadPins();
	loadPIDS();
	initializeVariables();

	//need to change interrupt number if input pins are changed
	attachInterrupt(3, throttleInterrupt, CHANGE);
	attachInterrupt(5, yawInterrupt, CHANGE);
	attachInterrupt(0, aileronInterrupt, CHANGE);
	attachInterrupt(4, elevatorInterrupt, CHANGE);

	//CHANGE GOAL HEIGHT HERE:
	baseHover = 75;

	mode = "MANUAL";
}

void loop() {
	startTime = millis();
	readRemote();

	if (switchValue < 1000){
		remoteOff += 1;
		if(remoteOff > 2){
			ground();
		}
	} else if (switchValue > 2000){
		remoteOff = 0;
		autoPilot();
	} else {
		remoteOff = 0;
		manualPilot();
		mode = "MANUAL";
	}

	analogWrite(throttleOutputPin, (int)finalThrust);
	analogWrite(yawOutputPin, (int)yawOut);
	analogWrite(aileronOutputPin, (int)aileronOut);
	analogWrite(elevatorOutputPin, (int)elevatorOut);

	//ensure loop execute in uniform amount of time
	loopTime = millis() - startTime;
	if(loopTime < 100){
		delay((100 - loopTime));
	}

}



void loadPins(){
	//Input Pins
	pinMode(auxSwitchInputPin,INPUT);
	pinMode(throttleInputPin, INPUT);
	pinMode(yawInputPin, INPUT);
	pinMode(aileronInputPin, INPUT);
	pinMode(elevatorInputPin, INPUT);
	pinMode(heightInputPin, INPUT);
	pinMode(heightInputPin2, INPUT);
	pinMode(lowFrontInputPin, INPUT);
	pinMode(rightInputPin, INPUT);
	pinMode(leftInputPin, INPUT);

	//Output Pins
	pinMode(throttleOutputPin, OUTPUT);
	pinMode(yawOutputPin, OUTPUT);
	pinMode(aileronOutputPin, OUTPUT);
	pinMode(elevatorOutputPin, OUTPUT);
	pinMode(sonicTriggerPin, OUTPUT);
	pinMode(sonicTriggerPin2, OUTPUT);
	pinMode(sonicTriggerPin3, OUTPUT);
	pinMode(sonicTriggerPin4, OUTPUT);
	pinMode(sonicTriggerPin5, OUTPUT);
}

void loadPIDS(){
	heightPID.SetMode(MANUAL);
	heightPID.SetOutputLimits(0,25);

	velocityPID.SetMode(MANUAL);
	velocityPID.SetOutputLimits(-3,3);

	frontPID.SetMode(MANUAL);
	frontPID.SetOutputLimits(-1,1);

	leftPID.SetMode(MANUAL);
	leftPID.SetOutputLimits(-1,1);

	rightPID.SetMode(MANUAL);
	rightPID.SetOutputLimits(-1,1);

	heightPID.SetSampleTime(90);
	velocityPID.SetSampleTime(90);
	frontPID.SetSampleTime(500);
	leftPID.SetSampleTime(500);
	rightPID.SetSampleTime(500);
}

void initializeVariables(){
	newHeight = 0;
	frontSensor = 0;
	rightSensor = 0;
	leftSensor = 0;

	autoOn = false;
	landing = false;
	takeOff = true;
}

//code for interrupts modified from:
//http://www.camelsoftware.com/firetail/blog/radio/reading-pwm-signals-from-a-remote-control-receiver-with-arduino/
void throttleInterrupt(){
	last_throttle_interrupt = micros();

	if(digitalRead(throttleInputPin) == HIGH){
		throttle_timer_start = micros();
	} else if(throttle_timer_start > 0){
		throttle = ((volatile int)micros() - throttle_timer_start);
		throttle_timer_start = 0;
	}
}

void yawInterrupt(){
	last_yaw_interrupt = micros();

	if(digitalRead(yawInputPin) == HIGH){
		yaw_timer_start = micros();
	} else if(yaw_timer_start > 0){
		yawIn = ((volatile int)micros() - yaw_timer_start);
		yaw_timer_start = 0;
	}
}

void aileronInterrupt(){
	last_aileron_interrupt = micros();

	if(digitalRead(aileronInputPin) == HIGH){
		aileron_timer_start = micros();
	} else if(aileron_timer_start > 0){
		aileronIn = ((volatile int)micros() - aileron_timer_start);
		aileron_timer_start = 0;
	}
}

void elevatorInterrupt(){
	last_elevator_interrupt = micros();

	if(digitalRead(elevatorInputPin) == HIGH){
		elevator_timer_start = micros();
	} else if(elevator_timer_start > 0){
		elevatorIn = ((volatile int)micros() - elevator_timer_start);
		elevator_timer_start = 0;
	}
}

long getPWM(int sourcePin1, int sourcePin2){
	byte zeroCounter1 = 0;
	byte zeroCounter2 = 0;
	long pwm1 = 0;
	long pwm2 = 0;
	do {
		pwm1 = (pwm1 == 0) ? pulseIn(sourcePin1, HIGH, 17000) : pwm1;
		pwm2 = (pwm2 == 0) ? pulseIn(sourcePin2, HIGH, 17000) : pwm2;

		if(zeroCounter1 == 2){
			pwm1 = 1;
		} else if(pwm1 == 0){
			++zeroCounter1;
		}
		if(zeroCounter2 == 2){
			pwm2 = 1;
		} else if(pwm2 == 0){
			++zeroCounter2;
		}
	} while(pwm1 == 0 || pwm2 == 0);

	switchValue = pwm1;
	landSwitchValue = pwm2;
	if(landSwitchValue > 1800){
		setMode();
	} 
}

void readRemote(){
	//consider creating a remote object for interfacing with remote
	getPWM(auxSwitchInputPin, landSwitchInputPin);
}

void readPi(){
	last_pi_In = pi_In;
	while(Serial.available() > 0){
		pi_In = Serial.read();
	}

	if(pi_In != last_pi_In){
		setMode();
	}
}

void readSensors(){
	//consider creating a sensor object for interfacing with sensors
	newHeight = verticalHeightInput(heightInputPin, heightInputPin2, sonicTriggerPin, sonicTriggerPin2);

	frontSensor = getSonicDistance(lowFrontInputPin, sonicTriggerPin3);
	rightSensor = getSonicDistance(rightInputPin, sonicTriggerPin4);
	leftSensor = getSonicDistance(leftInputPin, sonicTriggerPin5);

	velocity = newHeight - oldHeight;

	oldHeight = newHeight;
}

void setMode(){
	if(landSwitchValue > 1800 || pi_In == 'L'){
		landing = true;
		mode = "LANDING";
	} else if (pi_In == 'R'){
		mode = "READY";
	} else if (pi_In == 'T'){
		mode = "TAKEOFF";
	} else if (pi_In == 'H'){
		mode = "HOVER_STEADY";
		setSpot();
	} else {
		mode = "GROUND";
	}
}

void setSpot(){
	frontTarget = getSonicDistance(lowFrontInputPin, sonicTriggerPin3);
	rightTarget = getSonicDistance(rightInputPin, sonicTriggerPin4);
	leftTarget = getSonicDistance(leftInputPin, sonicTriggerPin5);
	//yawTarget

	frontPID.SetMode(AUTOMATIC);
	leftPID.SetMode(AUTOMATIC);
	rightPID.SetMode(AUTOMATIC);
	//yawPID
}

long getSonicDistance(int echoPin, int triggerPin){
	long distanceCM = 0;
	digitalWrite(triggerPin,HIGH);
	delayMicroseconds(300);
	digitalWrite(triggerPin,LOW);
	long duration = pulseIn(echoPin, HIGH,30000);
	long distance = (duration*34029L)/2000000L;
	distanceCM = distance;
	if(distanceCM == 0){
		distanceCM = 400;
	}   
	return (double)distanceCM; 
}

//this function uses two sensors averaged output to get input
long verticalHeightInput(int echoPin, int echoPin2, int triggerPin, int triggerPin2){
	long height1 = getSonicDistance(echoPin, triggerPin);
	long height2 = getSonicDistance(echoPin2, triggerPin2);
	long aveHeight;

	if(height1 != 400){
		if(height2 != 400){
			if((height1 - height2) > 20){
				aveHeight = height2;
			} else if((height2 - height1) > 20){
				aveHeight = height1;
			} else{
				aveHeight = (height1 + height2)/2L;
			}
		} else{
			aveHeight = height1;
		}
	} else{
		if(height2 != 400){
			aveHeight = height2;
		} else{
			aveHeight = 400;
		}
	}
	return aveHeight; 
}

void initiateHandshake(){
	pi_Out = HANDSHAKE;
	Serial.write(pi_Out);
}

void ground(){
	finalThrust = 138;
}

void autoSetup(){
	hoverHeight = baseHover;
	velocity = 0;
	hoverThrust = 180;

	velocityPID.SetMode(AUTOMATIC);

	autoOn = true;
	takeOff = true;
	flipFlag = true;

	pi_Out = READY_COMPLETE;
	Serial.write(pi_Out);
}

void takeOffSequence(){
	if(newHeight < 10){
		hoverThrust = hoverThrust + 1; //was 0.5, may need to change back
		finalThrust = hoverThrust;
	} else{
		mode = "HOVER";
		velGoal = takeoffVelGoal;
		heightPID.SetMode(AUTOMATIC);

		pi_Out = TAKEOFF_COMPLETE;
		Serial.write(pi_Out);
	}
}

void landingSequence(){
	if(newHeight < 7 && velocity > -3){
		mode = "END";
		pi_Out = LANDING_COMPLETE;
		Serial.write(pi_Out);
	} else{
		if(hoverHeight > 0){
			hoverHeight -= 0.5;
		}

		heightPID.Compute();
		velGoal = landVelGoal;

		velocityPID.Compute();
		finalThrust = autoVelocity + autoThrust + hoverThrust;
	}
}

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

void getManualOverrides(){
	//yaw
	if(yawIn > 1000){
		yawOut = ((float)yawIn * 0.121) + 4.804;
		yawOut = abs(yawOut - 188) < 5 ? 0 : (yawOut - 188);
	}
	else{
		yawOut = 0;
	}

	//aileron
	if(aileronIn > 1000){
		aileronOut = (aileronIn * .113) + 17.96;
		aileronOut = abs(aileronOut - 188) < 5 ? 0 : (aileronOut - 188);  
	} else {
		aileronOut = 0;
	}

	//elevator
	if(elevatorIn > 1000){
		elevatorOut = (elevatorIn * .114) + 16.88;
		elevatorOut = abs(elevatorOut - 188) < 5 ? 0 : (elevatorOut - 188);  
	} else {
		elevatorOut = 0;
	}
}

void convertSticks(){
	//thrust
	if(throttle>100){  
		finalThrust = ((float)throttle/7.854) - 5.112;
	}
	else{
		finalThrust = 138;
	}

	//yaw
	if(yawIn > 1000){
		yawOut = ((float)yawIn * 0.121) + 4.804;
		yawOut = abs(yawOut - 188) < 4 ? 188 : yawOut;
	}
	else{
		yawOut = 188;
	}

	//aileron
	if(aileronIn > 1000){
		aileronOut = (aileronIn * .113) + 17.96;
		aileronOut = abs(aileronOut - 188) < 2 ? 188 : aileronOut;  
	} else {
		aileronOut = 188;
	}

	//elevator
	if(elevatorIn > 1000){
		elevatorOut = (elevatorIn * .114) + 16.88;
		elevatorOut = abs(elevatorOut - 188) < 2 ? 188 : elevatorOut;  
	} else {
		elevatorOut = 188;
	}
}

void autoPilot(){
	readSensors();
	readPi();

	//safety features allows input of yaw, aileron, and elevator even during auto mode
	//input values are added to auto controls
	//leave sticks in default positioning to allow full auto
	getManualOverrides();

	if (mode == "MANUAL"){
		ground();
		initiateHandshake();
		mode = "END";
	} else if (mode == "READY"){
		autoSetup();
		mode = "END";
	} else if (mode == "TAKEOFF"){
		takeOffSequence();
	} else if (mode == "LANDING"){
		landingSequence();
	} else if (mode == "END"){
		ground();
	} else {
		//code for hovering sequences
		heightPID.Compute();
		velGoal = (newHeight - hoverHeight >= -10) ? (newHeight - hoverHeight >= 10 ? landVelGoal : 0) : takeoffVelGoal;

		if(velGoal == 0 && abs(velocity) < 2){
			velocity = 0;
		}

		velocityPID.Compute();
		finalThrust = autoVelocity + autoThrust + hoverThrust;

		//development section
		if(mode == "HOVER_STEADY"){
			frontVelocity = frontSensor - frontTarget;
			leftVelocity = leftSensor - leftTarget;
			rightVelocity = rightSensor - rightTarget;
			//yaw

			frontPID.Compute();
			leftPID.Compute();
			rightPID.Compute();

			elevatorOut += autoFront;
			aileronOut += (autoLeft + autoRight)/2;
		}

	}

	stableAxes();
}

void manualPilot(){
	autoOn = false;
	convertSticks();
}

