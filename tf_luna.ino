#include <SoftwareSerial.h> //header file of software serial port

SoftwareSerial Serial1(2, 3); //define software serial port name as Serial1 and define pin2 as RX and pin3

/** BEGIN UTILS */

void setPinsToMode(uint8_t pins[], size_t pins_length, int mode) {
	// assert(mode == INPUT || mode == OUTPUT);

	for (size_t i = 0; i < pins_length; i++) {
		pinMode(pins[i], mode);
	}
}

void setLEDBrightness(uint8_t pin, uint8_t LED_brightness) {
	digitalWrite(pin, LED_brightness != 0 ? HIGH : LOW); // TODO actual brightness
}

int measure() {
	//if (distance >=20 and distance < 50){ledstate = HIGH;}
	// else ledState = LOW;}
	//digitalWrite(ledPin,LedState);

	//as TX
	/* For Arduinoboards with multiple serial ports like DUEboard, interpret above two pieces of code and
	directly use Serial1 serial port*/
	int dist = 0;	  //actual distance measurements of LiDAR
	int strength = 0; //signal strength of LiDAR
	float temprature = 0.f;
	int check = 0; //save check value
	int i = 0;
	int uart[9];			 //save data measured by LiDAR
	memset(uart, 0, sizeof(uart));
	const int HEADER = 0x59; //frame header of data package

	if (false && !Serial1.available()) {
		// Serial.println("!Serial1.available()");
	} else {
		uart[0] = HEADER;

		if (false && Serial1.read() != HEADER) { //assess data package frame header 0x59
			Serial.println("(depth 1) Serial1.read() != HEADER");
		} else {
			uart[0] = HEADER;

			if (Serial1.read() == HEADER) {
				Serial.println("(depth 2) Serial1.read() != HEADER");
			} else {
				//{ //assess data package frame header 0x59
				uart[1] = HEADER;

				for (i = 2; i < 9; i++) { //save data in array
					uart[i] = Serial1.read();
				}

				check = uart[0] + uart[1] + uart[2] + uart[3] + uart[4] + uart[5] + uart[6] + uart[7];

				if (false && uart[8] != (check & 0xff)) {	//verify the received data as per protocol
					Serial.println("uart[8] != (check & 0xff)");
				} else {
					dist = uart[2] + uart[3] * 256; //256		  //calculate distance value
					strength = uart[4] + uart[5] * 256;	  //calculate signal strength value
					temprature = uart[6] + uart[7] * 256; //calculate chip temprature
					temprature = temprature / 8 - 256;

					/*
					Serial.print("dist = ");
					Serial.print(dist); //output measure distance value of LiDAR
					Serial.print('\t');
	
					Serial.print("strength = ");
					Serial.print(strength); //output signal strength value
					Serial.print("\t Chip Temprature = ");
					Serial.print(temprature);
					Serial.println(" celcius degree"); //output chip temperature of Lidar
					*/
				}
			}
		}
	} 

	char output[1024];
	sprintf(output, "dist %d; strength %d; temp %d; check %d; i %d;", dist, strength, temprature, check, i);
	Serial.println(output);

	return dist;
}

int dist;	  //actual distance measurements of LiDAR
int strength; //signal strength of LiDAR
float temprature;
int check; //save check value
int i;
int uart[9];			 //save data measured by LiDAR
const int HEADER = 0x59; //frame header of data package
//D9 led1
//D8 led2
//D7 led3

int measure_v2() {
	if (Serial1.available()){
  uart[0] = HEADER;
	if (Serial1.read() == HEADER) { //assess data package frame header 0x59
		uart[0] = HEADER;
		if (Serial1.read() == HEADER){
		//{ //assess data package frame header 0x59
			uart[1] = HEADER;
			for (i = 2; i < 9; i++)
			{ //save data in array
				uart[i] = Serial1.read();
			}
			check = uart[0] + uart[1] + uart[2] + uart[3] + uart[4] + uart[5] + uart[6] + uart[7];
			if (uart[8] == (check & 0xff))
			{										  //verify the received data as per protocol
				dist = uart[2] + uart[3] * 256;//256		  //calculate distance value
				strength = uart[4] + uart[5] * 256;	  //calculate signal strength value
				temprature = uart[6] + uart[7] * 256; //calculate chip temprature
				temprature = temprature / 8 - 256;
				Serial.print("dist = ");
				Serial.print(dist); //output measure distance value of LiDAR
				Serial.print('\t');

				Serial.print("strength = ");
				Serial.print(strength); //output signal strength value
				Serial.print("\t Chip Temprature = ");
				Serial.print(temprature);
				Serial.println(" celcius degree"); //output chip temperature of Lidar
			}
		}
	}
}

/*
	int reed = Serial1.read();

	char output[1024];
	sprintf(output, "dist %d; strength %d; temp %d; check %d; i %d; reed %d;", dist, strength, temprature, check, i, reed);
	Serial.println(output);
	*/

	return dist;
}

/** END UTILS */

/** BEGIN STRUCTS & FUNCTIONS FOR MAIN PROGRAM */

/** increase these if needed but beware to not exceed the max memory */
#define MAX_LED_COUNT   16
#define MAX_RANGE_COUNT 16

struct Range {
	size_t from;   /** from specified to 0, unless a smaller range is provided */
	uint8_t LEDs[MAX_LED_COUNT];
	
	static Range find_matching_by_distance(Range ranges[], size_t ranges_length, size_t distance) {
		Range candidate;

		for (size_t i = 0; i < ranges_length; i++) {
			Range current = ranges[i];
			
			if (distance <= current.from) {
				candidate = current;
			} else if (distance > current.from) {
				break;
			} else {
				// assert(false);
			}
		}

		return candidate;
	}
};

struct Config {
	size_t pin_count;
	size_t range_count;

	uint8_t LED_pins[MAX_LED_COUNT]; /** shall match the shape of a single LED array */
	Range ranges[MAX_RANGE_COUNT];

	void init() {
		setPinsToMode(LED_pins, pin_count, OUTPUT);

		if (pin_count > MAX_LED_COUNT) {
			// throw "MAX_LED_COUNT smaller than pin_count -- increase MAX_LED_COUNT";
		}

		if (range_count > MAX_RANGE_COUNT) {
			// throw "MAX_RANGE_COUNT smaller than pin_count -- increase MAX_RANGE_COUNT";
		}
	}

	void draw(size_t distance) {
		Range matching_range = Range::find_matching_by_distance(ranges, range_count, distance);

		for (size_t i = 0; i < pin_count; i++) {
			uint8_t pin = LED_pins[i];
			uint8_t LED = matching_range.LEDs[i];
			
			setLEDBrightness(pin, LED);
		}
	}

	void undraw() {
		for (size_t i = 0; i < pin_count; i++) {
			uint8_t pin = LED_pins[i];
			setLEDBrightness(pin, 0);
		}
	}
};

/** END STRUCTS & FUNCTIONS FOR MAIN PROGRAM */

/** BEGIN CONFIGS FOR MAIN PROGRAM */

Config config1 = {
	.pin_count = 9, /** shall not exceed MAX_LED_COUNT */

	.range_count = 11, /** shall not exceed MAX_RANGE_COUNT */

	.LED_pins = {
		1, 2, 3,
		4, 5, 6,
		7, 8, 9

		/** don't forget to update pin_count */
	},

	.ranges = {
    /*  from        LEDs             */
        9000,       {   0,   0,   0,
                        0,   0,   0,
                        0,   0,   0 },
    
         100,       { 128,   0,   0,
                        0,   0,   0,
                        0,   0,   0 },
    
          90,       { 255, 128,   0,
                        0,   0,   0,
                        0,   0,   0 },
    
          80,       { 255, 255, 128,
                        0,   0,   0,
                        0,   0,   0 },
    
          70,       { 255, 255, 255,
                        0,   0, 128,
                        0,   0,   0 },
    
          60,       { 255, 255, 255,
                        0,   0, 255,
                        0,   0, 128 },
    
          50,       { 255, 255, 255,
                        0,   0, 255,
                        0, 128, 255 },
    
          45,       { 255, 255, 255,
                        0,   0, 255,
                      128, 255, 255 },
    
          40,       { 255, 255, 255,
                      128,   0, 255,
                      255, 255, 255 },
    
          35,       { 255, 255, 255,
                      255, 128, 255,
                      255, 255, 255 },
    
          30,       { 255, 255, 255,
                      255, 255, 255,
                      255, 255, 255 }

		 /** don't forget to update range_count */
	}
};

/** END CONFIGS FOR MAIN PROGRAM */

void setup() {
	Serial.begin(9600);	   //set bit rate of serial port connecting Arduino with computer
	Serial1.begin(115200); //set bit rate of serial port connecting LiDAR with Arduino

	config1.init();
}

void loop() {
	int distance = measure_v2();
	config1.draw(distance);

	/*
	int fake_distance = 3000 - (millis() % 3000);
	config1.draw(fake_distance);
	*/

	/*
	int distance = measure();

	if (millis() % 1000 < 300) {
		config1.draw(distance);
	} else {
		config1.undraw();
	}
	*/
}

