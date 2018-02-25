
// COMPILA SE METTO NEI PATH I FOLDER SEGUENTI
//C:\Users\Luca\Documents\Arduino\SW\libraries\i2cdevlib\Arduino; C:\Users\Luca\Documents\Arduino\SW\libraries\i2cdevlib\Arduino\I2Cdev; C:\Users\Luca\Documents\Arduino\SW\libraries\i2cdevlib\Arduino\MPU6050; C:\Users\Luca\Documents\Arduino\SW\libraries\i2cdevlib\Arduino\HMC5883L; C:\Users\Luca\Documents\Arduino\SW\ESP8266\esprosimu; C:\Users\Luca\AppData\Local\arduino15\packages\esp8266\hardware\esp8266\2.2.0\libraries; C:\Users\Luca\AppData\Local\arduino15\packages\esp8266\hardware\esp8266\2.2.0\cores\esp8266; C:\Users\Luca\Documents\Arduino\SW\libraries; C:\Users\Luca\Documents\Arduino\SW\libraries\pgmspace
#include <SoftwareSerial.h>
#include <ESP8266WiFi.h>

#include <ros.h>
#include <Wire.h>

// ricordati di verificare in ros.h
// che sia
// #include "ArduinoHardware.h"
#define SIMULATION_STEPPER_ON 0
#define SIMULATION_LDS_ON 0
#define	WEBSERVER 0
//#define DEBUG_ON 1
#define SERIAL_SPEED		 115200		//default at boot=	74880


#define OPT_COMPASS 1
#define OPT_MPU6050 1
#define OPT_BLYNK 0
//#define dbg(s)  	printf(s); 

// ///////////////////////////////////////////////////////////////////////////////
///
//		DEBUG
///
// ///////////////////////////////////////////////////////////////////////////////
#ifdef DEBUG_ON
	#define dbg(s) 	Serial.println(s); 
	#define dbg2(s,v)  	Serial.print(s);Serial.println(v);  
	#define dbgV(...) Serial.printf(__VA_ARGS__)

#else
	#define dbg(s) 
	#define dbg2(s,v)
	#define dbgV(...)
#endif


// ///////////////////////////////////////////////////////////////////////////////
///
//		OLED DISPLAY
///
// ///////////////////////////////////////////////////////////////////////////////

#pragma region OledDisplay
	//https://github.com/ThingPulse/esp8266-oled-ssd1306

	#include <TimeLib.h>
	#include <Wire.h>  // Only needed for Arduino 1.6.5 and earlier
	#include "SSD1306.h" // alias for `#include "SSD1306Wire.h"`

	// Include the UI lib
	#include "OLEDDisplayUi.h"
	#include <CircularBuffer\CircularBuffer.h>
	CircularBuffer<String, 4> displayBuffer;

	// Include custom images
	//#include "images.h"
	const char activeSymbol[] PROGMEM = {
		B00000000,
		B00000000,
		B00011000,
		B00100100,
		B01000010,
		B01000010,
		B00100100,
		B00011000
	};

	const char inactiveSymbol[] PROGMEM = {
		B00000000,
		B00000000,
		B00000000,
		B00000000,
		B00011000,
		B00011000,
		B00000000,
		B00000000
	};


	// Initialize the OLED display using Wire library
	SSD1306  display(0x3c, D2, D1);

	OLEDDisplayUi ui(&display);

	int screenW = 128;
	int screenH = 64;
	int clockCenterX = screenW / 2;
	int clockCenterY = ((screenH - 16) / 2) + 16;   // top yellow part is 16 px height
	int clockRadius = 23;

	void drawProgressBarDemo(int percent) {

		// draw the progress bar
		display.drawProgressBar(0, 32, 120, 10, percent);

		// draw the percentage as String
		display.setTextAlignment(TEXT_ALIGN_CENTER);
		display.drawString(64, 15, String(percent) + "%");
	}
	void drawTextFlowDemo(String s) {
		display.setFont(ArialMT_Plain_10);
		display.setTextAlignment(TEXT_ALIGN_LEFT);
		display.drawStringMaxWidth(0, 0, 128, s);
	}


	int displayFrameIndex = 0; // scheda corrente, incrementata dal pulsante flash 

	#define DISPLAY_BUFF_ROWS 4

	void displayCircularBuffer(void) {
		// Initialize the log buffer
		// allocate memory to store 8 lines of text and 30 chars per line.
		display.setLogBuffer(DISPLAY_BUFF_ROWS, 30);

		// distanza in pixel tra due righe; dipende dal font size
	#define INTERLINEA 11
	#define BUFFER_START_Y 13
		display.clear();
		// 4 linee x 21 caratteri
		for (uint8_t i = 0; i < DISPLAY_BUFF_ROWS; i++) {
			// Print to the screen
			display.drawString(0, BUFFER_START_Y + INTERLINEA * (i), displayBuffer[i]);
			//display.println(displayBuffer[i]);


			// Draw it to the internal screen buffer
			display.drawLogBuffer(0, 0);
			// Display it on the screen

		}
		display.display();
	}
	#define dbgD(s) displayBuffer.push(s); displayCircularBuffer();


	// disegna un cerchio e un raggio orientato secondo l'angolo rad
	void displayCompass(float rad) {
		// offset da impostare per allineare la barra sul display all'asse x del magnetometro
		// positivo in senso orario
		#define OFFSET PI/2

		#define CENTER_X 80
		#define CENTER_Y 38
		#define RADIUS 22
		display.drawCircle(CENTER_X, CENTER_Y, RADIUS);
		int dx =round( RADIUS* cos(-rad+OFFSET));
		int dy =round( RADIUS* sin(-rad+ OFFSET));
		display.drawLine(CENTER_X, CENTER_Y, CENTER_X + dx, CENTER_Y+ dy);
	}

	void setup_display() {
		// Sets the current font. Available default fonts
		// ArialMT_Plain_10, ArialMT_Plain_16, ArialMT_Plain_24
		display.setFont(ArialMT_Plain_10);


		// The ESP is capable of rendering 60fps in 80Mhz mode
		// but that won't give you much time for anything else
		// run it in 160Mhz mode or just set it to 30 fps
		ui.setTargetFPS(30);

		// Customize the active and inactive symbol
		ui.setActiveSymbol(activeSymbol);
		ui.setInactiveSymbol(inactiveSymbol);

		// You can change this to
		// TOP, LEFT, BOTTOM, RIGHT
		ui.setIndicatorPosition(TOP);

		// Defines where the first frame is located in the bar.
		ui.setIndicatorDirection(LEFT_RIGHT);

		// You can change the transition that is used
		// SLIDE_LEFT, SLIDE_RIGHT, SLIDE_UP, SLIDE_DOWN
		ui.setFrameAnimation(SLIDE_LEFT);

		// Add frames
		//ui.setFrames(frames, frameCount);

		// Add overlays
		//ui.setOverlays(overlays, overlaysCount);

		// Initialising the UI will init the display too.
		ui.init();

		display.flipScreenVertically();


		dbgD(".");
		dbgD(".");
		dbgD(".");
		dbgD(".");


		dbg("\nOled Display setup complete\n");

		display.display();

	}



#pragma endregion  // OLED DISPLAY


// ///////////////////////////////////////////////////////////////////////////////
///
//		HELPER FUNCTIONS
///
// ///////////////////////////////////////////////////////////////////////////////
#pragma region HelperFunctions

	enum systemStatus_e {
			STARTING = 0,
			WIFICONNECTED,
			ROSCONNECTED,
			SCANNING,
			ENDSCAN,
			PUBLISHING,
			ENDPUBLISH,
			SLOWPUBLISH, // la pubblicazione termina dopo l'arrivo a HOME
			SLOWSCAN	//la scansione termina dopo l'arrivo a END

		};

	systemStatus_e SystemStatus;


	void ledSeq1(int ms, int times = 3) {
		pinMode(LED_BUILTIN, OUTPUT);
		for (int i = 0; i < times; i++)
		{
			digitalWrite(LED_BUILTIN, 0); delay(ms);
			digitalWrite(LED_BUILTIN, 1); delay(ms);

		}

	}


	String ftoa(float number, uint8_t precision, uint8_t size) {
		// Based on mem,  16.07.2008
		// http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num = 1207226548/6#6

		// prints val with number of decimal places determine by precision
		// precision is a number from 0 to 6 indicating the desired decimial places
		// example: printDouble(3.1415, 2); // prints 3.14 (two decimal places)

		// Added rounding, size and overflow #
		// ftoa(343.1453, 2, 10) -> "    343.15"
		// ftoa(343.1453, 4,  7) -> "#      "
		// avenue33, April 10th, 2010

		String s = "";

		// Negative 
		if (number < 0.0) {
			s = "-";
			number = -number;
		}

		double rounding = 0.5;
		for (uint8_t i = 0; i < precision; ++i)    rounding /= 10.0;

		number += rounding;
		s += String(uint16_t(number));  // prints the integer part

		if (precision > 0) {
			s += ".";                // prints the decimal point
			uint32_t frac;
			uint32_t mult = 1;
			uint8_t padding = precision - 1;
			while (precision--)     mult *= 10;

			frac = (number - uint16_t(number)) * mult;

			uint32_t frac1 = frac;
			while (frac1 /= 10)    padding--;
			while (padding--)      s += "0";

			s += String(frac, DEC);  // prints the fractional part
		}

		if (size > 0)                // checks size
			if (s.length() > size)        return("#");
			else while (s.length() < size) s = " " + s;

			return s;
	}


	#pragma region Hearthbeat

		//const unsigned char _heartbeat_values[] = { 21,21,21,21,21,21,21,21,21,21,21,21,21,22,23,25,
		//28,34,42,54,71,92,117,145,175,203,228,246,255,254,242,220,191,157,121,87,
		//58,34,17,6,1,0,2,5,9,13,16,18,19,20,21,21,21,21,21,21,21,21,21,21,21,21,21,21 };
		const unsigned char _heartbeat_values[] = { 1,2,3,5,7,9,1,1,1,21,21,21,21,22,23,25,
		28,34,42,54,71,92,117,145,175,203,228,246,255,254,242,220,191,157,121,87,
		58,34,17,6,1,0,2,5,9,13,16,18,19,20,21,21,21,21,21,21,21,21,9,7,5,3,2,1 };
		const unsigned char _HEARTBEAT_INDEXES = 64;

		//byte ledPin = 9; //Must be PWM capable pin
		//int heartbeat_period = 500; //[ms] 60000/120 (120 beats x min, baby) 
		int heartbeat_period = 1200; //[ms] 60000/70 (70 beats x min, adult in rest) 
									 //int heartbeat_period = 1000; // slower, feels like soothing


		void heartbeat(byte pwmLedPin) {
			int index = (millis() % heartbeat_period) * _HEARTBEAT_INDEXES / heartbeat_period;
			analogWrite(pwmLedPin, _heartbeat_values[index]);
			delay(20);
		}

	#pragma endregion //Hearthbeat


	void printESPinfo() {
		dbg2("CPU freq.", ESP.getCpuFreqMHz());
		dbg2("FlashChipRealSize: ", ESP.getFlashChipRealSize());

	}
	int scanI2C()
	{
		byte error, address;
		int nDevices;

		Serial.println("I2C Scanning...");
		dbgD("I2C Scanning...");

		nDevices = 0;
		for (address = 1; address < 127; address++)
		{
			ESP.wdtFeed();
			Wire.beginTransmission(address);
			error = Wire.endTransmission();

			if (error == 0)
			{
				//dbgD("I2C found at address (dec):");
				Serial.print("I2C device found at address 0x");
				if (address < 16)
					Serial.print("0");
				dbgD("I2C found at: "+String(address));delay(1000);
				Serial.print(address, HEX);
				Serial.println("  !");
				
				nDevices++;
			}
			else if (error == 4)
			{
				Serial.print("Unknow error at address 0x");
				if (address < 16)
					Serial.print("0");
				Serial.println(address, HEX);
			}
		}
		if (nDevices == 0)
			Serial.println("\n########\nNo I2C devices found\n##############");
		else
			Serial.println("done\n");

		return nDevices;
	}
	void softReset() {
		Serial.println("Restarting...");
		//Serial.println(WiFi.status());
		//esp_wifi_wps_disable() // add this, okay
		ESP.restart();
	}
#pragma endregion	//HelperFunctions



// ///////////////////////////////////////////////////////////////////////////////
///
//		NODEMCU HW
///
// ///////////////////////////////////////////////////////////////////////////////
#pragma region NODEMCU_HW

	#pragma region NODEMCU_PIN

	/*
	At startup, pins are configured as INPUT.
	GPIO0-GPIO15 can be INPUT, OUTPUT, or INPUT_PULLUP.
	GPIO16 can be INPUT, OUTPUT, or INPUT_PULLDOWN_16.
	It is also XPD for deepSleep() (perhaps via a small capacitor.)
	Note that GPIO6-GPIO11 are typically used to interface with the flash memory
	ICs on most esp8266 modules, so these pins should not generally be used.
	*/

	//LED
	#define Pin_LaserOn					D0	// Accensione Laser
	#define PIN_FLASHBUTTON				D3	// PULSANTE FLASH
	#define Pin_LED_TOP_B				LED_BUILTIN
	// I2C
	#define ESP_PIN_SPI_SDA				D2
	#define ESP_PIN_SPI_CK				D1



	// LED EXT.
	#define ESP_PIN_LED2				D9


	#define INTERRUPT_PIN 15 // use pin 15 on ESP8266





	#define LED2_ON digitalWrite(ESP_PIN_LED2, 1);
	#define LED2_OFF digitalWrite(ESP_PIN_LED2, 0);



	#define LED_ON digitalWrite(Pin_LED_TOP_B, 0);
	#define LED_OFF digitalWrite(Pin_LED_TOP_B, 1);

	//LASER



	#define writeFast1(gpIO)	WRITE_PERI_REG(PERIPHS_GPIO_BASEADDR + 4,  (1<<gpIO));
	#define writeFast0(gpIO) 	WRITE_PERI_REG(PERIPHS_GPIO_BASEADDR + 8,  (1<<gpIO));


	#define BLINK  LED_ON; delay(5); LED_OFF;

	#pragma  endregion


	void setup_HW() {

		printESPinfo();


		//// TFT
		//tft.begin();
		//tft.setRotation(3);
		//tft.setTextSize(2);
		//clear_screen();
		//display_progress("Initialising", 5);

		// LED
		pinMode(Pin_LED_TOP_B, OUTPUT);
		pinMode(ESP_PIN_LED2, OUTPUT);

		pinMode(PIN_FLASHBUTTON, INPUT);



		Wire.begin(ESP_PIN_SPI_SDA, ESP_PIN_SPI_CK);


		int i2cDevices = scanI2C();
		if (i2cDevices > 0)
		{
			printf("\nTovate %d periferiche I2C", i2cDevices);

		#if OPT_MPU6050

				setup_MPU();

		#endif 
		#if OPT_COMPASS

				setup_compass();

		#endif 



		}
		else
		{
			printf("Impossibile continuare. Soft Reset...");
			dbgD("Impossibile continuare");

			//	ESP.restart();
		}





	}


	// memoria libera ----
	// si usa così: uint32_t free = system_get_free_heap_size();
	extern "C" {
		#include "user_interface.h"
	}

#pragma endregion	// NODEMCU_HW



// ///////////////////////////////////////////////////////////////////////////////
///
//		IMU
///
// ///////////////////////////////////////////////////////////////////////////////
#pragma region IMU
	const char DEVICE_NAME[] = "mpu6050";

	#ifndef VECTOR_STRUCT_H
	#define VECTOR_STRUCT_H
	struct Vector
	{
		float XAxis;
		float YAxis;
		float ZAxis;
	};
	struct VectorInt
	{
		float x;
		float y;
		float z;
	};
	#endif

	#if OPT_MPU6050
	#include "I2Cdev.h"


	//#include <MPU6050\MPU6050.h> // not necessary if using MotionApps include file
	//#include <MPU6050.h>



	#include "MPU6050_6Axis_MotionApps20.h"  //libreria nella cartella MPU6050dmp
	#include "helper_3dmath.h"



	// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
	// is used in I2Cdev.h


	// class default I2C address is 0x68
	// specific I2C addresses may be passed as a parameter here
	// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
	// AD0 high = 0x69
	MPU6050 mpu;		//MPU6050 mpu(0x69); // <-- use for AD0 high


	// MPU control/status vars
	bool dmpReady = false;  // set true if DMP init was successful
	uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
	uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
	uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
	uint16_t fifoCount;     // count of all bytes currently in FIFO
	uint8_t fifoBuffer[64]; // FIFO storage buffer

							//						// orientation/motion vars
							//Quaternion q;           // [w, x, y, z]         quaternion container
							//VectorInt16 aa;         // [x, y, z]            accel sensor measurements
							//VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
							//VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
							//VectorFloat gravity;    // [x, y, z]            gravity vector
	float euler[3];         // [psi, theta, phi]    Euler angle container
	float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
							//		#include <MPU6050dmp\MPU6050_6Axis_MotionApps20.h>
							// MPU control/status vars

							// orientation/motion vars
	Quaternion q;           // [w, x, y, z]         quaternion container
	VectorInt16 aa;         // [x, y, z]            accel sensor measurements
	VectorInt16 gg;         // [x, y, z]            gyro sensor measurements
	VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
	VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
	VectorFloat gravity;    // [x, y, z]            gravity vector

							// packet structure for InvenSense teapot demo
	//uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

	// ================================================================
	// ===               INTERRUPT DETECTION ROUTINE                ===
	// ================================================================

	volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
	void dmpDataReady() {
		mpuInterrupt = true;
	}
 

	void setup_MPU() { // versione per <MPU6050\MPU6050.h>
		dbgD("\nInit MPU6050...");
		Wire.begin();
		Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties



		// libreria MPU6050dmp: usa		mpu.initialize();	
		// lib. motionaxis....   usa	mpu.begin();
		mpu.initialize();		
		pinMode(INTERRUPT_PIN, INPUT);

		// verify connection
		dbgD("Testing device connections...");
		dbgD(mpu.testConnection() ? F("MPU6050 connected") : F("MPU6050 connection failed"));

		// load and configure the DMP
		dbgD("Initializing DMP...");
		devStatus = mpu.dmpInitialize();  //MPU6050_6Axis_Motion lib.

		//// supply your own gyro offsets here, scaled for min sensitivity
		//mpu.setGyroOffsetX(220);
		//mpu.setGyroOffsetY(76);
		//mpu.setGyroOffsetZ(-85);
		//mpu.setAccelOffsetZ(1788); // 1688 factory default for my test chip
		//						   // turn on the DMP, now that it's ready
								   // supply your own gyro offsets here, scaled for min sensitivity
		mpu.setXGyroOffset(220);
		mpu.setYGyroOffset(76);
		mpu.setZGyroOffset(-85);
		mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

		mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);


								   // make sure it worked (returns 0 if so)
		if (devStatus == 0) {
			// turn on the DMP, now that it's ready
			Serial.println(F("Enabling DMP..."));
			mpu.setDMPEnabled(true);

			// enable Arduino interrupt detection
			Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
			attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
			mpuIntStatus = mpu.getIntStatus();

			// set our DMP Ready flag so the main loop() function knows it's okay to use it
			dbg("DMP ready! Waiting for first interrupt...");
			dbgD("DMP ready!");
			dmpReady = true;

			// get expected DMP packet size for later comparison
			packetSize = mpu.dmpGetFIFOPacketSize();







		}
		else {
			// ERROR!
			// 1 = initial memory load failed
			// 2 = DMP configuration updates failed
			// (if it's going to break, usually the code will be 1)
			dbgD("DMP Initialization failed (code ");
			Serial.print(F("DMP Initialization failed (code "));
			Serial.print(devStatus);
			Serial.println(F(")"));
		}




		dbg("...done");

	}
	/*
	void setup_MPU() { // versione per MPU6050_6Axis_MotionApps20.h
		mpu.initialize();
		devStatus = mpu.dmpInitialize();
		mpu.setXGyroOffset(220);
		mpu.setYGyroOffset(76);
		mpu.setZGyroOffset(-85);
		mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
	}
	*/


	void printMPUraw() {
		//VectorInt16 v;
		//v = mpu.readRawAccel();

		//dbg("A\t);")
		//dbg(v.XAxis); dbg("\t");
		//dbg(v.YAxis); dbg("\t");
		//dbg(v.ZAxis); dbg("\n");

		// display quaternion values in easy matrix form: w x y z
		mpu.dmpGetQuaternion(&q, fifoBuffer);
		Serial.print("quat \tW ");
		Serial.print(q.w);
		Serial.print("\tX ");
		Serial.print(q.x);
		Serial.print("\tY ");
		Serial.print(q.y);
		Serial.print("\tZ ");
		Serial.println(q.z);

	}

	void mpu_loopTest()
	{
		#define OUTPUT_READABLE_QUATERNION

		// if programming failed, don't try to do anything
		if (!dmpReady) return;

		// wait for MPU interrupt or extra packet(s) available
		if (!mpuInterrupt && fifoCount < packetSize) return;

		// reset interrupt flag and get INT_STATUS byte
		mpuInterrupt = false;
		mpuIntStatus = mpu.getIntStatus();

		// get current FIFO count
		fifoCount = mpu.getFIFOCount();

		// check for overflow (this should never happen unless our code is too inefficient)
		if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
			// reset so we can continue cleanly
			mpu.resetFIFO();
			Serial.println(F("FIFO overflow!"));

			// otherwise, check for DMP data ready interrupt (this should happen frequently)
		}
		else if (mpuIntStatus & 0x02) 
		{
			// wait for correct available data length, should be a VERY short wait
			while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

			// read a packet from FIFO
			mpu.getFIFOBytes(fifoBuffer, packetSize);

			// track FIFO count here in case there is > 1 packet available
			// (this lets us immediately read more without waiting for an interrupt)
			fifoCount -= packetSize;

			#if 1         //def OUTPUT_READABLE_QUATERNION
				// display quaternion values in easy matrix form: w x y z
				//mpu.dmpGetQuaternion(&q, fifoBuffer);
				//Serial.print("quat\t");	Serial.print(q.w);
				//Serial.print("\t"); 	Serial.print(q.x);
				//Serial.print("\t");		Serial.print(q.y);
				//Serial.print("\t");		Serial.println(q.z);

				readAndDisplayQuat();
			#endif

			#ifdef OUTPUT_TEAPOT_OSC
				#ifndef OUTPUT_READABLE_QUATERNION

					// display quaternion values in easy matrix form: w x y z
					mpu.dmpGetQuaternion(&q, fifoBuffer);
				#endif
				// Send OSC message
				OSCMessage msg("/imuquat");
				msg.add((float)q.w);
				msg.add((float)q.x);
				msg.add((float)q.y);
				msg.add((float)q.z);

				Udp.beginPacket(outIp, outPort);
				msg.send(Udp);
				Udp.endPacket();

				msg.empty();
			#endif

			#ifdef OUTPUT_READABLE_EULER
				// display Euler angles in degrees
				mpu.dmpGetQuaternion(&q, fifoBuffer);
				mpu.dmpGetEuler(euler, &q);
				Serial.print("euler\t");
				Serial.print(euler[0] * 180 / M_PI);
				Serial.print("\t");
				Serial.print(euler[1] * 180 / M_PI);
				Serial.print("\t");
				Serial.println(euler[2] * 180 / M_PI);
			#endif

			#ifdef OUTPUT_READABLE_YAWPITCHROLL
				// display Euler angles in degrees
				mpu.dmpGetQuaternion(&q, fifoBuffer);
				mpu.dmpGetGravity(&gravity, &q);
				mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
				Serial.print("ypr\t");
				Serial.print(ypr[0] * 180 / M_PI);
				Serial.print("\t");
				Serial.print(ypr[1] * 180 / M_PI);
				Serial.print("\t");
				Serial.println(ypr[2] * 180 / M_PI);
			#endif

			#ifdef OUTPUT_READABLE_REALACCEL
				// display real acceleration, adjusted to remove gravity
				mpu.dmpGetQuaternion(&q, fifoBuffer);
				mpu.dmpGetAccel(&aa, fifoBuffer);
				mpu.dmpGetGravity(&gravity, &q);
				mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
				Serial.print("areal\t");
				Serial.print(aaReal.x);
				Serial.print("\t");
				Serial.print(aaReal.y);
				Serial.print("\t");
				Serial.println(aaReal.z);
			#endif

			#ifdef OUTPUT_READABLE_WORLDACCEL
				// display initial world-frame acceleration, adjusted to remove gravity
				// and rotated based on known orientation from quaternion
				mpu.dmpGetQuaternion(&q, fifoBuffer);
				mpu.dmpGetAccel(&aa, fifoBuffer);
				mpu.dmpGetGravity(&gravity, &q);
				mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
				mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
				Serial.print("aworld\t");
				Serial.print(aaWorld.x);
				Serial.print("\t");
				Serial.print(aaWorld.y);
				Serial.print("\t");
				Serial.println(aaWorld.z);
			#endif
		}
	}

// ///////////////////////////////////////////////////////////////////////////////
///
//		COMPASS
///
// ///////////////////////////////////////////////////////////////////////////////
#pragma region compass
	#if OPT_COMPASS
	#include <Wire.h>
	#include <HMC5883L\HMC5883L.h>
	#include <compass\MyCompass.h>  // derivato da HMC5883L
	//MyCompass_c compass;

	// compass;
		MyCompass_c compass;
		VectorInt16  mag;

		void setup_compass() {
			// Initialize Initialize HMC5883L
			dbgD("Init COMPASS...");
			//compass.initialize();
			while (!compass.begin(2)) { delay(500); dbg("."); }
			//{
			//	Serial.println("compass not found, check wiring!");
			//	delay(500);
			/**/
			// Set measurement range
			compass.setRange(hmc5883l_range_t::HMC5883L_RANGE_1_3GA);


			// Set measurement mode
			compass.setMeasurementMode(hmc5883l_mode_t::HMC5883L_CONTINOUS);

			// Set data rate
			compass.setDataRate(hmc5883l_dataRate_t::HMC5883L_DATARATE_30HZ);

			// Set number of samples averaged
			compass.setSamples(hmc5883l_samples_t::HMC5883L_SAMPLES_8);

			// Set calibration offset. See HMC5883L_calibration.ino
			compass.setOffset(0, 0);

			printf("Compass Deg.:%d", compass.getHeadingDeg());


			dbgD("...COMPASS OK");

		}
		void compassPrintRaw() {
			Vector v;
			v = compass.readRaw();
			dbg("Cmp\t");
			dbg(v.XAxis); dbg("\t");
			dbg(v.YAxis); dbg("\t");
			dbg(v.ZAxis); dbg("\n");
		}
	#endif // COMPASS  

#pragma endregion



	void mpu_acquisition_dmp()
	{
		// if programming failed, don't try to do anything
		if (!dmpReady) return;

		// wait for MPU interrupt or extra packet(s) available
		if (!mpuInterrupt && fifoCount < packetSize) return;

		// reset interrupt flag and get INT_STATUS byte
		mpuInterrupt = false;
		mpuIntStatus = mpu.getIntStatus();

		// get current FIFO count
		fifoCount = mpu.getFIFOCount();

		// check for overflow (this should never happen unless our code is too inefficient)
		if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
			// reset so we can continue cleanly
			mpu.resetFIFO();
			dbgD(F("FIFO overflow!"));

			// otherwise, check for DMP data ready interrupt (this should happen frequently)
		}
		else if (mpuIntStatus & 0x02) {
			// wait for correct available data length, should be a VERY short wait
			while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

			// read a packet from FIFO
			mpu.getFIFOBytes(fifoBuffer, packetSize);

			// track FIFO count here in case there is > 1 packet available
			// (this lets us immediately read more without waiting for an interrupt)
			fifoCount -= packetSize;

			// display quaternion values in easy matrix form: w x y z
			mpu.dmpGetQuaternion(&q, fifoBuffer);

			mpu.getMotion6(&aa.x, &aa.y, &aa.z, &gg.x, &gg.y, &gg.z);
 
			// display Euler angles in degrees
			///mpu.dmpGetEuler(euler, &q);

			// display Euler angles in degrees
			///mpu.dmpGetGravity(&gravity, &q);
			///mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

			// display real acceleration, adjusted to remove gravity
			///mpu.dmpGetAccel(&aa, fifoBuffer);
			///mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);




			// display initial world-frame acceleration, adjusted to remove gravity
			// and rotated based on known orientation from quaternion
			///mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);




		}
	}

	void mpu_acquisition_complementaryFilter()
	{

 
		// wait for MPU interrupt or extra packet(s) available
		if (!mpuInterrupt && fifoCount < packetSize) return;

		// reset interrupt flag and get INT_STATUS byte
		mpuInterrupt = false;
		mpuIntStatus = mpu.getIntStatus();

		// get current FIFO count
		fifoCount = mpu.getFIFOCount();

		// check for overflow (this should never happen unless our code is too inefficient)
		if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
			// reset so we can continue cleanly
			mpu.resetFIFO();
			dbgD(F("FIFO overflow!"));

			// otherwise, check for DMP data ready interrupt (this should happen frequently)
		}
		else if (mpuIntStatus & 0x02) {
			// wait for correct available data length, should be a VERY short wait
			while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

			// read a packet from FIFO
			mpu.getFIFOBytes(fifoBuffer, packetSize);

			// track FIFO count here in case there is > 1 packet available
			// (this lets us immediately read more without waiting for an interrupt)
			fifoCount -= packetSize;




			//mpu.dmpGetQuaternion(&q, fifoBuffer);
			mpu.getMotion6(&aa.x, &aa.y, &aa.z, &gg.x, &gg.y, &gg.z);
			//		VectorInt16 mag;  //compass raw data

		}

		// Legge compass
		compass.getHeading(&mag.x, &mag.y, &mag.z);

		// Sensor fusion
		MadgwickQuaternionUpdate2( aa.x, aa.y, aa.z, gg.x, gg.y, gg.z, mag.x, mag.y, mag.z);


	}

	#endif // MPU



#pragma region AHRS sensor fusion
	// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
	float GyroMeasError = PI * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
	float GyroMeasDrift = PI * (0.0f / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
												  // There is a tradeoff in the beta parameter between accuracy and response speed.
												  // In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
												  // However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
												  // Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
												  // By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
												  // I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense; 
												  // the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy. 
												  // In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
	float beta = sqrt(3.0f / 4.0f) * GyroMeasError;   // compute beta
	float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
	#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
	#define Ki 0.0f

	uint32_t delt_t = 0; // used to control display output rate
	uint32_t count = 0;  // used to control display output rate
	uint32_t mcount = 0; // used to control magnetometer read rate
	uint32_t MagRate;    // read rate for magnetometer data

	float pitch, yaw, roll;
	float deltat = 0.0f;        // integration interval for both filter schemes

	uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
	uint32_t Now = 0;        // used to calculate integration interval

	float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values 
	float qt[4] = { 1.0f, 0.0f, 0.0f, 0.0f };    // vector to hold quaternion
	float eInt[3] = { 0.0f, 0.0f, 0.0f };       // vector to hold integral error for Mahony method

	// Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
	// (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
	// which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
	// device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
	// The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
	// but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!
	void MadgwickQuaternionUpdate2( float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
	{
		float q1 = qt[0], q2 = qt[1], q3 = qt[2], q4 = qt[3];   // short name local variable for readability
		float norm;
		float hx, hy, _2bx, _2bz;
		float s1, s2, s3, s4;
		float qDot1, qDot2, qDot3, qDot4;
		//            float gerrx, gerry, gerrz, gbiasx, gbiasy, gbiasz;        // gyro bias error

		// Auxiliary variables to avoid repeated arithmetic
		float _2q1mx;
		float _2q1my;
		float _2q1mz;
		float _2q2mx;
		float _4bx;
		float _4bz;
		float _2q1 = 2.0f * q1;
		float _2q2 = 2.0f * q2;
		float _2q3 = 2.0f * q3;
		float _2q4 = 2.0f * q4;
		float _2q1q3 = 2.0f * q1 * q3;
		float _2q3q4 = 2.0f * q3 * q4;
		float q1q1 = q1 * q1;
		float q1q2 = q1 * q2;
		float q1q3 = q1 * q3;
		float q1q4 = q1 * q4;
		float q2q2 = q2 * q2;
		float q2q3 = q2 * q3;
		float q2q4 = q2 * q4;
		float q3q3 = q3 * q3;
		float q3q4 = q3 * q4;
		float q4q4 = q4 * q4;

		// Normalise accelerometer measurement
		norm = sqrt(ax * ax + ay * ay + az * az);
		if (norm == 0.0f) return; // handle NaN
		norm = 1.0f / norm;
		ax *= norm;
		ay *= norm;
		az *= norm;

		// Normalise magnetometer measurement
		norm = sqrt(mx * mx + my * my + mz * mz);
		if (norm == 0.0f) return; // handle NaN
		norm = 1.0f / norm;
		mx *= norm;
		my *= norm;
		mz *= norm;

		// Reference direction of Earth's magnetic field
		_2q1mx = 2.0f * q1 * mx;
		_2q1my = 2.0f * q1 * my;
		_2q1mz = 2.0f * q1 * mz;
		_2q2mx = 2.0f * q2 * mx;
		hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
		hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
		_2bx = sqrt(hx * hx + hy * hy);
		_2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;

		// Gradient decent algorithm corrective step
		s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
		s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
		s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
		s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
		norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
		norm = 1.0f / norm;
		s1 *= norm;
		s2 *= norm;
		s3 *= norm;
		s4 *= norm;

		/*            // Compute estimated gyroscope biases
		gerrx = _2q1 * s2 - _2q2 * s1 - _2q3 * s4 + _2q4 * s3;
		gerry = _2q1 * s3 + _2q2 * s4 - _2q3 * s1 - _2q4 * s2;
		gerrz = _2q1 * s4 - _2q2 * s3 + _2q3 * s2 - _2q4 * s1;

		// Compute and remove gyroscope biases
		gbiasx += gerrx * deltat * zeta;
		gbiasy += gerry * deltat * zeta;
		gbiasz += gerrz * deltat * zeta;
		gx -= gbiasx;
		gy -= gbiasy;
		gz -= gbiasz;
		*/
		// Compute rate of change of quaternion
		qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
		qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
		qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
		qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

		// Integrate to yield quaternion
		q1 += qDot1 * deltat;
		q2 += qDot2 * deltat;
		q3 += qDot3 * deltat;
		q4 += qDot4 * deltat;
		norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
		norm = 1.0f / norm;

		qt[0] = q1 * norm;
		qt[1] = q2 * norm;
		qt[2] = q3 * norm;
		qt[3] = q4 * norm;

//		Quaternion q;
		q.w = qt[0];
		q.x = qt[1];
		q.y = qt[2];
		q.z = qt[3];
		//q->w = q1 * norm;
		//q->x = q2 * norm;
		//q->y = q3 * norm;
		//q->z = q4 * norm;

//		return q;

	}

	void MadgwickQuaternionUpdate_orig(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
	{
		float q1 = qt[0], q2 = qt[1], q3 = qt[2], q4 = qt[3];   // short name local variable for readability
		float norm;
		float hx, hy, _2bx, _2bz;
		float s1, s2, s3, s4;
		float qDot1, qDot2, qDot3, qDot4;
		//            float gerrx, gerry, gerrz, gbiasx, gbiasy, gbiasz;        // gyro bias error

		// Auxiliary variables to avoid repeated arithmetic
		float _2q1mx;
		float _2q1my;
		float _2q1mz;
		float _2q2mx;
		float _4bx;
		float _4bz;
		float _2q1 = 2.0f * q1;
		float _2q2 = 2.0f * q2;
		float _2q3 = 2.0f * q3;
		float _2q4 = 2.0f * q4;
		float _2q1q3 = 2.0f * q1 * q3;
		float _2q3q4 = 2.0f * q3 * q4;
		float q1q1 = q1 * q1;
		float q1q2 = q1 * q2;
		float q1q3 = q1 * q3;
		float q1q4 = q1 * q4;
		float q2q2 = q2 * q2;
		float q2q3 = q2 * q3;
		float q2q4 = q2 * q4;
		float q3q3 = q3 * q3;
		float q3q4 = q3 * q4;
		float q4q4 = q4 * q4;

		// Normalise accelerometer measurement
		norm = sqrt(ax * ax + ay * ay + az * az);
		if (norm == 0.0f) return; // handle NaN
		norm = 1.0f / norm;
		ax *= norm;
		ay *= norm;
		az *= norm;

		// Normalise magnetometer measurement
		norm = sqrt(mx * mx + my * my + mz * mz);
		if (norm == 0.0f) return; // handle NaN
		norm = 1.0f / norm;
		mx *= norm;
		my *= norm;
		mz *= norm;

		// Reference direction of Earth's magnetic field
		_2q1mx = 2.0f * q1 * mx;
		_2q1my = 2.0f * q1 * my;
		_2q1mz = 2.0f * q1 * mz;
		_2q2mx = 2.0f * q2 * mx;
		hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
		hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
		_2bx = sqrt(hx * hx + hy * hy);
		_2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;

		// Gradient decent algorithm corrective step
		s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
		s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
		s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
		s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
		norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
		norm = 1.0f / norm;
		s1 *= norm;
		s2 *= norm;
		s3 *= norm;
		s4 *= norm;

		/*            // Compute estimated gyroscope biases
		gerrx = _2q1 * s2 - _2q2 * s1 - _2q3 * s4 + _2q4 * s3;
		gerry = _2q1 * s3 + _2q2 * s4 - _2q3 * s1 - _2q4 * s2;
		gerrz = _2q1 * s4 - _2q2 * s3 + _2q3 * s2 - _2q4 * s1;

		// Compute and remove gyroscope biases
		gbiasx += gerrx * deltat * zeta;
		gbiasy += gerry * deltat * zeta;
		gbiasz += gerrz * deltat * zeta;
		gx -= gbiasx;
		gy -= gbiasy;
		gz -= gbiasz;
		*/
		// Compute rate of change of quaternion
		qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
		qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
		qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
		qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

		// Integrate to yield quaternion
		q1 += qDot1 * deltat;
		q2 += qDot2 * deltat;
		q3 += qDot3 * deltat;
		q4 += qDot4 * deltat;
		norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
		norm = 1.0f / norm;
		qt[0] = q1 * norm;
		qt[1] = q2 * norm;
		qt[2] = q3 * norm;
		qt[3] = q4 * norm;

	}


	// Similar to Madgwick scheme but uses proportional and integral filtering on the error between estimated reference vectors and
	// measured ones. 
	void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
	{
		float q1 = qt[0], q2 = qt[1], q3 = qt[2], q4 = qt[3];   // short name local variable for readability
		float norm;
		float hx, hy, bx, bz;
		float vx, vy, vz, wx, wy, wz;
		float ex, ey, ez;
		float pa, pb, pc;

		// Auxiliary variables to avoid repeated arithmetic
		float q1q1 = q1 * q1;
		float q1q2 = q1 * q2;
		float q1q3 = q1 * q3;
		float q1q4 = q1 * q4;
		float q2q2 = q2 * q2;
		float q2q3 = q2 * q3;
		float q2q4 = q2 * q4;
		float q3q3 = q3 * q3;
		float q3q4 = q3 * q4;
		float q4q4 = q4 * q4;

		// Normalise accelerometer measurement
		norm = sqrt(ax * ax + ay * ay + az * az);
		if (norm == 0.0f) return; // handle NaN
		norm = 1.0f / norm;        // use reciprocal for division
		ax *= norm;
		ay *= norm;
		az *= norm;

		// Normalise magnetometer measurement
		norm = sqrt(mx * mx + my * my + mz * mz);
		if (norm == 0.0f) return; // handle NaN
		norm = 1.0f / norm;        // use reciprocal for division
		mx *= norm;
		my *= norm;
		mz *= norm;

		// Reference direction of Earth's magnetic field
		hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
		hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
		bx = sqrt((hx * hx) + (hy * hy));
		bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

		// Estimated direction of gravity and magnetic field
		vx = 2.0f * (q2q4 - q1q3);
		vy = 2.0f * (q1q2 + q3q4);
		vz = q1q1 - q2q2 - q3q3 + q4q4;
		wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
		wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
		wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);

		// Error is cross product between estimated direction and measured direction of gravity
		ex = (ay * vz - az * vy) + (my * wz - mz * wy);
		ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
		ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
		if (Ki > 0.0f)
		{
			eInt[0] += ex;      // accumulate integral error
			eInt[1] += ey;
			eInt[2] += ez;
		}
		else
		{
			eInt[0] = 0.0f;     // prevent integral wind up
			eInt[1] = 0.0f;
			eInt[2] = 0.0f;
		}

		// Apply feedback terms
		gx = gx + Kp * ex + Ki * eInt[0];
		gy = gy + Kp * ey + Ki * eInt[1];
		gz = gz + Kp * ez + Ki * eInt[2];

		// Integrate rate of change of quaternion
		pa = q2;
		pb = q3;
		pc = q4;
		q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
		q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
		q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
		q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);

		// Normalise quaternion
		norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
		norm = 1.0f / norm;
		qt[0] = q1 * norm;
		qt[1] = q2 * norm;
		qt[2] = q3 * norm;
		qt[3] = q4 * norm;

	}
	/*
	#pragma region complementary filter
	#include "six_axis_comp_filter.h"


		//da https://gist.github.com/savovs/cbe998c3dfea711c3413cb23b6244cd9

	// deltaTime - The time delta update period expressed in seconds. This value
	//             should be set to how often the filter is updated with new values,
	//             which should be on a regular interval. The smaller this value is,
	//             the less noise there will be on the comp. filter's output.
	//
	// tau - Max allowable time until gyro drifts too far and comp. filter
	//      shifts its weight to the accelerometer expressed in seconds. This
	//      value is usually based on the drift rate of the gyro. For example,
	//      if the gyro drifts at a rate of 0.5 degrees per second and the
	//      tolerance of max allowable drift is 1 degree, then tau should be set
	//      to 2 seconds as it will take 2 seconds to drift 1 degree. The larger
	//      this value is, the less noise there will be on the comp. filter's
	//      output. In exchange, it will drift further from the correct angular
	//      position.
		CompSixAxis CompFilter(0.1, 2);


		void compFilter(VectorInt16 aa, VectorInt16 gg) {


			////  Get all motion sensor (in this case LSM6DS3) parameters,
			////  If you're using a different sensor you'll have to replace the values
			//accelX = myIMU.readFloatAccelX();
			//accelY = myIMU.readFloatAccelY();
			//accelZ = myIMU.readFloatAccelZ();

			//gyroX = myIMU.readFloatGyroX();
			//gyroY = myIMU.readFloatGyroY();
			//gyroZ = myIMU.readFloatGyroZ();

			// Convert these values into angles using the Complementary Filter
			CompFilter.CompAccelUpdate(aa.XAxis, aa.YAxis, aa.ZAxis); // takes arguments in m/s^2
			CompFilter.CompGyroUpdate(gg.XAxis, gg.YAxis, gg.ZAxis); // takes arguments un rad/s 
			CompFilter.CompUpdate();
			CompFilter.CompStart();

			// Get angle relative to X and Y 
			//CompFilter.CompAnglesGet(&xAngle, &yAngle);

		}


	#pragma endregion

*/	

#pragma endregion



#pragma endregion



// ///////////////////////////////////////////////////////////////////////////////
///
//		BLYNK
///
// ///////////////////////////////////////////////////////////////////////////////

#pragma region Blynk
//
//#if OPT_BLINK
//
//	#include <BlynkSimpleEsp8266.h>
//	#define BLYNK_TOKEN "10cc6df79d9e4cabb0fc54f093ea7769"
//	char auth[] = BLYNK_TOKEN;
//	#define BLYNK_PRINT Serial
//	// This is called when Smartphone App is opened
//	BLYNK_APP_CONNECTED() {
//		Serial.println("Blynk App Connected.");
//	}
//	// This is called when Smartphone App is closed
//	BLYNK_APP_DISCONNECTED() {
//		Serial.println("Blynk App Disconnected.");
//	}
//
//
//
//#endif
//
#pragma endregion

#pragma region Integrazione_MPU_Display
	void readAndDisplayQuat() {
		// display quaternion values in easy matrix form: w x y z
		mpu.dmpGetQuaternion(&q, fifoBuffer);

		String s;
		s = "w: " + String(q.w, 4); displayBuffer.push(s);
		s = "x: " + String(q.x, 4); displayBuffer.push(s);
		s = "y: " + String(q.y, 4); displayBuffer.push(s);
		s = "z: " + String(q.z, 4); displayBuffer.push(s);
		displayCircularBuffer();

		#define BAR_W 100
		#define BAR_H 8
		#define min(a,b) ((a)<(b)?(a):(b))

		uint8_t zBar = min(100, round(abs(q.x) * 100));
		display.drawProgressBar(1, 0, BAR_W, BAR_H, zBar);
		display.display();
	}

	inline void displayQuat(){
		// display quaternion values in easy matrix form: w x y z
		//mpu.dmpGetQuaternion(&q, fifoBuffer);

		String s;
		//s= "w: " + String(q->w, 4); displayBuffer.push(s);
		//s= "x: " + String(q->x, 4); displayBuffer.push(s);
		//s= "y: " + String(q->y, 4); displayBuffer.push(s);
		//s= "z: " + String(q->z, 4); displayBuffer.push(s);
		s= "w: " + String(q.w, 4); displayBuffer.push(s);
		s= "x: " + String(q.x, 4); displayBuffer.push(s);
		s= "y: " + String(q.y, 4); displayBuffer.push(s);
		s= "z: " + String(q.z, 4); displayBuffer.push(s);
		displayCircularBuffer();

		//#define BAR_W 100
		//#define BAR_H 8
		//uint8_t zBar =min(100, round(abs(q.x) * 100));
		//display.drawProgressBar(1, 0, BAR_W, BAR_H, zBar);
		//display.display();
	}

	//get  Quaternion and print it on serial
	inline	void dbgImuQuat() {
		// display quaternion values in easy matrix form: w x y z
		mpu.dmpGetQuaternion(&q, fifoBuffer);

		String s;
		s = "w: " + String(q.w, 4); Serial.println(s);
		s = "x: " + String(q.x, 4); Serial.println(s);
		s = "y: " + String(q.y, 4); Serial.println(s);
		s = "z: " + String(q.z, 4); Serial.println(s);


	}

#pragma endregion


#pragma region WIFI

	//#include <WiFiManager.h>

	//WiFiManager wifiManager; // Connect to Wi-Fi

	//////////////////////
	// WiFi Definitions //
	//////////////////////

	#include <ESP8266WiFi.h>
	const char* ssid = "FASTWEB-CSRLCU";
	const char* password = "cesarini";
	#define ROS_TCP_CONNECTION_PORT 11411
	#define ROS_MASTER_URI	"192.168.0.51"

	const uint16_t port = 80;
	const char * host = ROS_MASTER_URI; // ip or dns

										//IPAddress server(192, 168, 0, 51); // ip of your ROS server
										//IPAddress ip_address;
	int status = WL_IDLE_STATUS;



	// Use WiFiClient class to create TCP connections
	WiFiClient client;

/*
	// classe usata da node handle di ROS
	class WiFiHardware {

	public:
		WiFiHardware() {};

		void init() {
			// do your initialization here. this probably includes TCP server/client setup

			//client.connect(host, ROS_TCP_CONNECTION_PORT); //client.connect(server, ROS_TCP_CONNECTION_PORT);
			//dbg2("WiFiClient connected to port ", ROS_TCP_CONNECTION_PORT);
		}

		// read a byte from the serial port. -1 = failure
		int read() {
			// implement this method so that it reads a byte from the TCP connection and returns it
			//  you may return -1 is there is an error; for example if the TCP connection is not open
			return client.read();         //will return -1 when it will works
		}

		// write data to the connection to ROS
		void write(uint8_t* data, int length) {
			// implement this so that it takes the arguments and writes or prints them to the TCP connection
			client.write(data);
			//for (int i = 0; i < length; i++)client.write(data[i]);
		}

		// returns milliseconds since start of program
		unsigned long time() {
			return millis(); // easy; did this one for you
		}
	};


*/

	// ritorna true se riesce a connettersi al wifi
	bool setup_WiFi(int maxRetries = 5)
	{











		bool connected = false;


		WiFi.begin(ssid, password);

		String mac = WiFi.macAddress();
		dbg2("\nConnecting to ", ssid);
		dbg2("MAC ADDR:", mac);

		uint8_t i = 0;
		while (WiFi.status() != WL_CONNECTED && i++ < maxRetries) delay(1000);



		if (WiFi.status() != WL_CONNECTED) {
			connected = false;
			dbg2("Could not connect to: ", ssid);


			// LED flash per 3 sec...
			unsigned long t0 = millis();
			while ((millis() - t0) < 3000)
			{
				LED_ON; delay(10); LED_OFF;	delay(500);
			}

			//while (1) {
			//	// LED flash per 3 sec...
			//	while( (millis() - t0) < 3000)
			//	{
			//		LED_ON; delay(10);LED_OFF;	delay(500);				 		


			//	}
			//	// poi faccio il reboot
			//	softReset();

			//}
		}
		else //ok connesso
		{
			connected = true;
			dbg2("Ready to use  IP: ", WiFi.localIP());

	#if SIMULATION_ON
			// led acceso per 2 secondi
			digitalWrite(LED_BUILTIN, 1);	  delay(200); 	  digitalWrite(LED_BUILTIN, 0);
			digitalWrite(LED_BUILTIN, 1);	  delay(200); 	  digitalWrite(LED_BUILTIN, 0);
			digitalWrite(LED_BUILTIN, 1);	  delay(200); 	  digitalWrite(LED_BUILTIN, 0);
			digitalWrite(LED_BUILTIN, 1);
	#endif

			SystemStatus = systemStatus_e::WIFICONNECTED;
			delay(2000);
		}

		return connected;
	}

	// rientra quando si è connesso al WiFi



	///////////////////////////////////////////////////////////////////////
	// codice da http://usemodj.com/2016/08/04/esp8266-wifi-smartconfig/
	//#include <WiFiManager\WiFiManager.h>          //https://github.com/tzapu/WiFiManager
	#include <WiFiManager.h>
	#include <DNSServer.h>

	void connectToWiFi() {
		//Local intialization. Once its business is done, there is no need to keep it around
		WiFiManager wifiManager;
		//wifiManager.resetSettings();//reset settings - for testing
		//wifiManager.setAPCallback(configModeCallback);//set callback that gets called when connecting to previous WiFi fails, and enters Access Point mode

		//fetches ssid and pass and tries to connect
		//if it does not connect it starts an access point with the specified name and goes into a blocking loop awaiting configuration
		if (!wifiManager.autoConnect("ESP", "cesarini")) {
			Serial.println("failed to connect and hit timeout");
			//reset and try again, or maybe put it to deep sleep
			ESP.reset();
			delay(1000);
		}

		//if you get here you have connected to the WiFi
		Serial.print(F("WiFi connected! IP address: "));
		Serial.println(WiFi.localIP());


		////if you used auto generated SSID, print it
		//Serial.print("SSID is:");
		//Serial.println(wifiManager.getConfigPortalSSID());
		//Serial.print("----------------------------------------");

	}


#pragma endregion





//////////////////////
///
// WEB SERVER		 
///
//////////////////////
#pragma region WEB SERVER
	#if WEBSERVER
	#include <ESP8266WebServer.h>  // ho aggiunto come path File di Origine: C:\Users\Luca\AppData\Local\arduino15\packages\esp8266\hardware\esp8266\2.2.0\libraries\ESP8266WebServer\src
	ESP8266WebServer server(80);
	#include "webpages.hpp"
	#endif
#pragma endregion





// ///////////////////////////////////////////////////////////////////////////////
///
//		ROS
///
// ///////////////////////////////////////////////////////////////////////////////

#pragma region ROS MACRO REGION


	#include <ros.h>
	#include <ros/time.h>
	#include <std_msgs/String.h>
	#include <sensor_msgs/Range.h> //ultrasound
	//	#include <tf/transform_broadcaster.h>
	#include <sensor_msgs/Imu.h>


	//--------------------------------
	//	ros::NodeHandle  nh;
	//--------------------------------
	//ros::NodeHandle_<WiFiHardware> nh;
	ros::NodeHandle nh;		// con aggiunta in Ros.h di: 	typedef NodeHandle_<Esp8266Hardware, 25, 25, 512, 1024> NodeHandle;

	//--------------------------------
	// TF 
	//--------------------------------
	#include <tf/tf.h>
	#include <tf/transform_broadcaster.h>
	tf::TransformBroadcaster broadcaster;

	//--------------------------------
	// IMU 
	//--------------------------------
	sensor_msgs::Imu imu_msg;
	//ros::Publisher imu_pub("/imu/data_raw", &imu_msg);
	ros::Publisher imu_pub("/imu", &imu_msg);

	//--------------------------------
	// COMPASS 
	//--------------------------------
	sensor_msgs::Range compass_msg;
	ros::Publisher compass_pub("/compass", &compass_msg);

	//--------------------------------
	// MAG 
	//--------------------------------
	geometry_msgs::Vector3 mag_msg;
	ros::Publisher mag_pub("/mag", &mag_msg);


	//--------------------------------
	//	geometry_msgs::TransformStamped t;
	//	tf::TransformBroadcaster broadcaster;

	//--------------------------------
	std_msgs::String str_msg;
	ros::Publisher chatter("chatter", &str_msg);
	////--------------------------------
	//sensor_msgs::Range rosmsg_range;
	//ros::Publisher pub_range("ultrasound", &rosmsg_range);

	//--------------------------------
	#define ROS_INFO(s) nh.loginfo(s);



	char imu_frame_name[] = "base_link";
	char compass_frame_name[] = "base_link";
//	char odom[] = "odom";
	//	char frameid[] = "ultrasound";
	char charVal[10];

	unsigned long range_time;
	unsigned long tf_time;




	const int adc_pin = 0;
	double x = 1.0;
	double y = 0.0;
	double theta = 1.57;
	double g_req_angular_vel_z = 0;
	double g_req_linear_vel_x = 0;
	unsigned long g_prev_command_time = 0;

	byte hbFreq = 10; //heartbeat


 




	#pragma region Setup_ROS

		//// acquisisce i parametri da parameter server
		//void readParameters(int LDSsamples, float  ldsSpeed) {
		//	if (nh.getParam("/ldsSpeed", &ldsSpeed, LDSspeed_default))
		//	{
		//		dbg2("/ldsSpeed", ldsSpeed);
		//		myLDSstepper.goRadsPerSecond(ldsSpeed);
		//	}
		//	if (nh.getParam("/LDSsamples", &LDSsamples, SCAN_SAMPLES_DEFAULT))
		//	{
		//		dbg2("/LDSsamples", LDSsamples);
		//
		//	}
		//
		//
		//}

		/// ///////////////////////////////////////////////////////////////
		/// ///////////////////////////////////////////////////////////////
		///								
		///	FUNZIONI PER CARICARE I PARAMETRI ROS DAI FILE *.YAML					
		///								
		/// ///////////////////////////////////////////////////////////////
		/// ////////////////////////////////////////////////////////////////

		/*
		Lanciare Rosserial node così:
		<node ns="robot"  name="serial_node_wifi" pkg="rosserial_python" type="serial_node.py" output="screen">
		<param name="port" value="tcp" />
		<param name="/scan_speed" value="1.6"/>
		<param name="/scan_samples" value="33"/>
		</node>
		*/
		String thisNs = "/robot";
		String thisNode = "/serial_node_wifi";

		int loadRosParameter(ros::NodeHandle* nh, const char* parName, int defaultValue) {
			String parPath = thisNs + thisNode + "/" + parName;
			int parValue = defaultValue;
			ESP.wdtFeed();
			if (nh->getParam(parPath.c_str(), &parValue))
			{
				printf("\n Loaded int   [%s]:\t %d", parName, parValue);
				char ros_info_msg[200]; sprintf(ros_info_msg, "Loaded int param  [%s]:\t %d", parName, parValue); nh->loginfo(ros_info_msg);

				return parValue;
			}
			else
			{
				char ros_info_msg[200]; sprintf(ros_info_msg, "***Warning*** int param. [%s] not found. Using default value %f ", parPath.c_str(), defaultValue); nh->logwarn(ros_info_msg);
				printf("***Warning*** Int param. [%s] not found. \n\t\t Using default value %d \n\n", parPath.c_str(), defaultValue);
				printf(parPath.c_str());
				return defaultValue;
			}
		}
		float loadRosParameter(ros::NodeHandle* nh, const char* parName, float defaultValue) {

			String parPath = thisNs + thisNode + "/" + parName;
			float parValue = defaultValue;
			ESP.wdtFeed();//necessario
			if (nh->getParam(parPath.c_str(), &parValue))
			{
				String parValueStr = ftoa(parValue, 4, 10);
				printf("Loaded float [%s]:\t %s", parName, parValueStr.c_str());  // prinf con %f non è supportato
				char ros_info_msg[200]; sprintf(ros_info_msg, "Loaded float param [%s]:\t  %s", parName, parValueStr.c_str()); nh->loginfo(ros_info_msg);
				return parValue;
			}
			else
			{
				char ros_info_msg[200]; sprintf(ros_info_msg, "***Warning*** float param. [%s] not found. Using default value %s ", parPath.c_str(), ftoa(defaultValue, 4, 10).c_str()); nh->logwarn(ros_info_msg);
				printf(" ***Warning*** float param. [%s] not found. \n\t \tUsing default value %s \n\n", parPath.c_str(), ftoa(defaultValue, 4, 10).c_str());
				return defaultValue;
			}
		}

	#if 0
			int loadRosParameter(ros::NodeHandle_<WiFiHardware>* nh, const char* parName, int defaultValue) {
				String parPath = thisNs + thisNode + "/" + parName;
				int parValue = defaultValue;
				ESP.wdtFeed();
				if (nh->getParam(parPath.c_str(), &parValue))
				{
					printf("\n Loaded int   [%s]:\t %d", parName, parValue);
					char ros_info_msg[200]; sprintf(ros_info_msg, "Loaded int param  [%s]:\t %d", parName, parValue); nh->loginfo(ros_info_msg);

					return parValue;
				}
				else
				{
					char ros_info_msg[200]; sprintf(ros_info_msg, "***Warning*** int param. [%s] not found. Using default value %f ", parPath.c_str(), defaultValue); nh->logwarn(ros_info_msg);
					printf("***Warning*** Int param. [%s] not found. \n\t\t Using default value %d \n\n", parPath.c_str(), defaultValue);
					printf(parPath.c_str());
					return defaultValue;
				}
			}
			float loadRosParameter(ros::NodeHandle_<WiFiHardware>* nh, const char* parName, float defaultValue) {

				String parPath = thisNs + thisNode + "/" + parName;
				float parValue = defaultValue;
				ESP.wdtFeed();//necessario
				if (nh->getParam(parPath.c_str(), &parValue))
				{
					String parValueStr = ftoa(parValue, 4, 10);
					printf("Loaded float [%s]:\t %s", parName, parValueStr.c_str());  // prinf con %f non è supportato
					char ros_info_msg[200]; sprintf(ros_info_msg, "Loaded float param [%s]:\t  %s", parName, parValueStr.c_str()); nh->loginfo(ros_info_msg);
					return parValue;
				}
				else
				{
					char ros_info_msg[200]; sprintf(ros_info_msg, "***Warning*** float param. [%s] not found. Using default value %s ", parPath.c_str(), ftoa(defaultValue, 4, 10).c_str()); nh->logwarn(ros_info_msg);
					printf(" ***Warning*** float param. [%s] not found. \n\t \tUsing default value %s \n\n", parPath.c_str(), ftoa(defaultValue, 4, 10).c_str());
					return defaultValue;
				}
			}

	#endif // 0





	void ROS_loadParameters() {

		//LDSsamples =SCAN_SAMPLES_DEFAULT; //con 40 funziona  devono essere minore di  SCAN_SAMPLES_MAX

		//	LDSsamples = loadRosParameter(&nh, "scan_samples", SCAN_SAMPLES_DEFAULT);
		//	scanSpeed = loadRosParameter(&nh, "scan_speed", SCAN_SPEED_SCANNING);
		//	returnSpeed = loadRosParameter(&nh, "return_speed", SCAN_SPEED_RETURNING);



	}




	/// ///////////////////////////////////////////////////////////////
	/// ////////////////////////////////////////////////////////////////
	// si connette a ROS , fa l'advertise dei vari messaggi, e carica i parametri
	void setup_ROS() {
		dbg("Connecting to Ros...");
		dbgD("Connecting to Ros...");


		IPAddress ros_core_server(192,168,0,51); 


		// Ros objects constructors   
		nh.getHardware()->setConnection(ros_core_server, ROS_TCP_CONNECTION_PORT);
		nh.initNode();
		broadcaster.init(nh);

		//nh.advertise(pub_range);
		//nh.advertise(odom_pub);
		//nh.subscribe(Sub);
		nh.advertise(chatter);
		nh.advertise(imu_pub);
		nh.advertise(mag_pub);
		nh.advertise(compass_pub);


		// recupero parametri




		// FIll Quaternion
		imu_msg.orientation.w = 0.0;
		imu_msg.orientation.x = 0.0;
		imu_msg.orientation.y = 0.0;
		imu_msg.orientation.z = 0.0;
		for (int i = 0; i<9; i++)
		{
			imu_msg.angular_velocity_covariance[i] = 0;
			imu_msg.linear_acceleration_covariance[i] = 0;
			imu_msg.orientation_covariance[i] = 0;
		}



		nh.spinOnce();
		//delay(3000);

		


		//"no more advertise admitted...");
		while (!nh.connected()) {
			if (!client.connected())
			{
				client.connect(host, ROS_TCP_CONNECTION_PORT);

			}
			//#if OPT_BLYNK
			//	//Blynk.virtualWrite(2, 250 * digitalRead(ESP_PIN_STEPPERLDS_HOME));
			//	//Blynk.virtualWrite(3, 250 * digitalRead(ESP_PIN_STEPPERLDS_END));
			//	Blynk.run();

			//#endif // OPT_BLYNK

			nh.spinOnce();
			delay(1000); LED_ON;  delay(1); LED_OFF;


		}







		SystemStatus = systemStatus_e::ROSCONNECTED;
		dbg("\nCONNESSO A ROS");
		dbgD("CONNESSO A ROS");
		nh.loginfo("CONNESSO A ROS");
		delay(5);





	//	ROS_loadParameters();




	}

	#pragma endregion	// Setup_ROS


	static float g_to_m_s2 = 9.81;                  // conversion from g to m/s^2
	static float deg_to_rad = 3.14159265359 / 180.; // conversion from degree to radiants
	static float scale_accel = 1 / 16384.;          // conversion from MPU6050 accel readings to g
	static float scale_gyro = 1 / 131.;             // conversion from MPU6050 gyro readings to degree/second




	// Pubblica imu_msg usando le variabili globali q=Orientation, aa=Linear Acceleration, gg=Angular velocity
	void publish_imu(uint32_t loopcnt) {

		/*
		This works with the standard MPU6050
		settings AFS_SEL=0 and FS_SEL=0
		which corresponds to a sensitivity of
		+-2g for the accelerometer
		and +-250 degree/second for the gyro.
		The scaling factor to get from the reading to [g] (accel) and [deg/s] (gyro)
		is 16,384 and 131. The readings are then scaled from [g] to [m/s²] and from [deg/s] to [rad/s]. The values come from the MPU6050 Datasheet Page 12-13.
		*/
		//int16_t ax, ay, az, gx, gy, gz;
		//mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
		//mpu.dmpGetQuaternion(&q, fifoBuffer);



		imu_msg.header.stamp = nh.now();
		imu_msg.header.frame_id = imu_frame_name;	// "base_link";
		imu_msg.header.seq = loopcnt;
		
		imu_msg.orientation.w = q.w;
		imu_msg.orientation.x = q.x;
		imu_msg.orientation.y = q.y;
		imu_msg.orientation.z = q.z;

		imu_msg.linear_acceleration.x = (float)aa.x * scale_accel * g_to_m_s2;
		imu_msg.linear_acceleration.y = (float)aa.y * scale_accel * g_to_m_s2;
		imu_msg.linear_acceleration.z = (float)aa.z * scale_accel * g_to_m_s2;
		imu_msg.angular_velocity.x = (float)gg.x * scale_gyro * deg_to_rad;
		imu_msg.angular_velocity.y = (float)gg.y * scale_gyro * deg_to_rad;
		imu_msg.angular_velocity.z = (float)gg.z * scale_gyro * deg_to_rad;

		//int dummy = displayQuat(&q);
		//displayQuat();

//		long  t1 = millis();

		imu_pub.publish(&imu_msg);  //ora ci mette dai 4 ai 6 ms
		//nh.spinOnce();
		//dbgD("MPU acq ms: "+String(acqTime));
//		printf("\nPublish ms:%d ", millis() - t1);

	}


	void publish_compass(uint32_t loopcnt) {

		compass_msg.header.stamp = nh.now();
		compass_msg.header.seq = loopcnt;
		compass_msg.header.frame_id = compass_frame_name;

		compass_msg.range = compass.getHeadingRad();
		compass_pub.publish(&compass_msg);
	}

	void publish_mag() {


		mag_msg.x  = mag.x;
		mag_msg.y  = mag.y;
		mag_msg.z  = mag.z;
		mag_pub.publish(&mag_msg);
	}


#pragma endregion ROS 





// ///////////////////////////////////////////////////////////////////////////////
///
//		S E T U P
///
// ///////////////////////////////////////////////////////////////////////////////

void setup() {
	Serial.begin(SERIAL_SPEED);
	Serial.setDebugOutput(false);// to enable output from printf() function.  >>http://esp8266.github.io/Arduino/versions/2.0.0/doc/reference.html#timing-and-delays
	SystemStatus = systemStatus_e::STARTING;



	dbg("\n---------   EspRosImu  ---------\n");
	setup_display();
	dbgD("===EspRosImu==");

	/// //////////////////////////////////////////////////////////////
	//  WIFI CONNECTION
	/// //////////////////////////////////////////////////////////////
	connectToWiFi();
	dbgD("WiFi connected to:");
	String ip = WiFi.localIP().toString();
	dbgD(ip);

	SystemStatus = systemStatus_e::WIFICONNECTED;

	//#if OPT_BLINK		
	//	Blynk.begin(auth, ssid, password);
	//#endif


 
	/// //////////////////////////////////////////////////////////////
	//  WEB SERVER
	/// //////////////////////////////////////////////////////////////
	#if WEBSERVER
		//	setup_WebServer(connectedToWiFi);
		start_WebServer();
		#if 0	// test webserver
			while (true)
			{
				LED_ON;
				delay(10);
				LED_OFF;
				server.handleClient();
			}

		#endif // 1
	#endif


	/// //////////////////////////////////////////////////////////////
	//  HARDWARE SETUP
	/// //////////////////////////////////////////////////////////////

	setup_HW();


	//for (int i = 0; i < 1000; i++) {		dbgImuQuat();	}

	//#if OPT_BLINK
	//	Blynk.virtualWrite(2, digitalRead(ESP_PIN_STEPPERLDS_HOME));
	//	Blynk.
	//	Blynk.run();
	//#endif

	#pragma region TEST

		#if 0
			dbg("\nMPU6050 Test loop--------");
			while (true)
			{

				BLINK

				ESP.wdtFeed();
				#if OPT_MPU6050
				while (true) // questo loop è stabile
				{
					mpu_loopTest();
				}
				//printMPUraw();
				#endif // OPT_MPU6050

				//#if OPT_COMPASS
				//	//compassPrintRaw();
				//#endif // OPT_COMPASS

			}

		#endif // 0


		#if 0 // loop infinito per verificare che non intervenga il wtd
			int i = 0;
			dbg("\nTest no crash");
			while (true)
			{
				while (nh.connected())
				{
					ESP.wdtFeed();
					dbg2("n=", i++);
					client.flush();
					ROS_INFO("scan alive");
					delay(1000);  // 500 -->dopo n-=120  LmacRxBlk:1 which mean that the WiFi input buffer of the SDK is overloaded
				}
				dbg("\n Lost ROS");
				setup_ROS(); //fa il setup dei vari nodi e legge i parametri

			}
		#endif // 0





			// test IMU
		#if 0

			dbg("TST MPU");
			long tStart = millis();
			while (millis() - tStart < 10000) {
				if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
					// reset so we can continue cleanly
					mpu.resetFIFO();
					dbg("FIFO overflow!");

					// otherwise, check for DMP data ready interrupt (this should happen frequently)
				}
				else if (mpuIntStatus & 0x02) {
					// wait for correct available data length, should be a VERY short wait
					while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

					// read a packet from FIFO
					mpu.getFIFOBytes(fifoBuffer, packetSize);

					// track FIFO count here in case there is > 1 packet available
					// (this lets us immediately read more without waiting for an interrupt)
					fifoCount -= packetSize;

					mpu.dmpGetQuaternion(&q, fifoBuffer);
					dtostrf(q.z, 4, 3, charVal);	//float to char array
					dbg("MPU")
						Serial1.print("1,quat ");
					Serial1.print(q.w);
					Serial1.print(",");
					Serial1.print(q.x);
					Serial1.print(",");
					Serial1.print(q.y);
					Serial1.print(",");
					Serial1.print(q.z);
					Serial1.println(";");

				}

 
			}

		#endif // 1



	#pragma endregion //regione dei test


	/// //////////////////////////////////////////////////////////////
	//  ROS CONNECTION
	/// //////////////////////////////////////////////////////////////
	setup_ROS(); //fa il setup dei vari nodi e legge i parametri
	SystemStatus = systemStatus_e::ROSCONNECTED;

	//int i=0;	while (1) { i++;				delay(1000);		readAndDisplayQuat();	/*dbgD(String(i));*/ }
	//while (true) {	mpu_loopTest();	}




	#if OPT_BLINK
		//Blynk.begin(auth);

		// You can also specify server:
		//Blynk.begin(auth, ssid, pass, "blynk-cloud.com", 8442);
		//Blynk.begin(auth, ssid, pass, IPAddress(192,168,1,100), 8442);


		//while (true)
		//{
		//	Blynk.virtualWrite(2, 250 * digitalRead(ESP_PIN_STEPPERLDS_HOME));
		//	Blynk.virtualWrite(3, 250 * digitalRead(ESP_PIN_STEPPERLDS_END));
		//	Blynk.run();

		//}
	#endif

	// inizia
	dbgD(" --- Starting loop ---");
	//	ESP.wdtEnable(WDTO_1S);
	ESP.wdtFeed();

	delay(1000);
}




unsigned long nextMsg_time = 0;
long publishTime = 0;		/// tempo impiegato per pubblicare
long publishStartTime = 0;  ///inizio pubblicazione
#define DISPLAY_FRAMES 3	// Numero di schede

int alfa;
void  displayFrame(int id) {

	switch (id)
	{
	case 0:	// Visualizza Quaternion 
		displayQuat(); //esegue il clear all'inizio
		display.drawString(0, 0, "sec.:       "); display.drawString(30, 0, String(millis() / 1000));
		display.drawCircle(DISPLAY_WIDTH - 3, 5, 2);
		display.display();


		break;

	case 1: //compass
		alfa = compass.getHeadingDeg();
		display.clear();
		display.drawString(0, 0, "sec.:       "); display.drawString(30, 0, String(millis() / 1000));
		display.drawCircle(DISPLAY_WIDTH - 3, 5, 2);
		display.drawCircle(DISPLAY_WIDTH - 14, 5, 2);
		display.drawString(1, 20, "Deg. " + String(alfa));
		//float angle = compass.getHeadingRad();
		displayCompass(alfa*DEG_TO_RAD);//
		display.display();

		break;

	case 2:		// non visualizza nulla per ottenere il massimo framerate 
	default:
		display.clear();
		display.drawCircle(DISPLAY_WIDTH - 3, 5, 2);
		display.drawCircle(DISPLAY_WIDTH - 12, 5, 2);
		display.drawCircle(DISPLAY_WIDTH - 21, 5, 2);
		display.display();

		break;

	}


}
long loopcnt = 0;
void loop() {


//	dbgD(".");
	while (nh.connected())
	{
		loopcnt++;
		BLINK;
		//dbgD("loop: "+String(loopcnt++));
		


		//ESP.wdtFeed();

		//int remainingTimeBudget = ui.update();
		//if (remainingTimeBudget > 0) {
		//	// You can do some work here
		//	// Don't do stuff if you are below your
		//	// time budget.
		//	delay(remainingTimeBudget);
		//}
		



		//long  t1 = millis();

		//mpu_acquisition_complementaryFilter(); // acquisisce quat. , aa, gg 
		mpu_acquisition_dmp();


		//uint32_t acqTime = millis() - t1;
		////dbgD("MPU acq ms: "+String(acqTime));
		//printf("\nMPU acq ms:%d ",acqTime);

		

		publish_imu(loopcnt); 
		#if OPT_COMPASS		
			//publish_compass(loopcnt);
			publish_mag();
		#endif

		nh.spinOnce();

		// 	displayQuat();

		
		
				// Ogni 1 secondo segnala che è vivo
				if (millis() > nextMsg_time + 500)
				{

				//	displayQuat(); //esegue il clear all'inizio
					//display.drawString(0, 0, "loop:       "); display.drawString(30,0, String(loopcnt++));
				//	float angle = compass.getHeadingRad();
					//displayCompass(angle);//


					if (!digitalRead(PIN_FLASHBUTTON)==1)
					{
						displayFrameIndex += 1; 	if (displayFrameIndex >= DISPLAY_FRAMES){	displayFrameIndex = 0;	}


					}

					displayFrame(displayFrameIndex);




					//readParameters(LDSsamples, );
					//dbg2("Loop: ", loopcnt);  //42792
					//ROS_INFO("esprosimu alive");
 


					#if OPT_COMPASS		
					//printf("\nBear° :%d", compass.getBearing());
					
					#endif
					nh.spinOnce();  //elabora eventuali callBack
					nextMsg_time = millis();
					/////////////////////////////////////////////////////////////////////

				}

		
 
		//#if OPT_BLINK
		//	Blynk.virtualWrite(2, 250 * digitalRead(ESP_PIN_STEPPERLDS_HOME));
		//	Blynk.run();
		//#endif

	}


	dbgD("########################");
	dbgD("# Lost ROS connection! #");
	dbgD("########################");

	SystemStatus = systemStatus_e::STARTING;
	setup_ROS(); //fa il setup dei vari nodi

	//ledSeq1(200, 5);

	delay(1000);

}

