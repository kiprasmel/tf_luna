#define DEBUG

#include <SoftwareSerial.h> // header file of software serial port

SoftwareSerial DistanceSensor(2, 3); // define software serial port name as DistanceSensor and define pin2 as RX and pin3

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

/**
 * `measure` is implemented directly from the manual (+ improved for clarity)
 *
 * If the serial is (currently) available (and all other conditions pass) in the current call,
 * the distance will been updated.
 *
 * Otherwise -- the distance returned will be the `previous_valid_distance`
 * to avoid crazy jumps, because the serial is not available in every loop tick,
 * and the distance most likely hasn't changed.
 *
 */
int previous_valid_distance = 0;

int measure() {
	const int HEADER = 0x59; // frame header of data package

	int uart[9]; // save data measured by LiDAR

	if (DistanceSensor.available()) {
		uart[0] = HEADER;

		if (DistanceSensor.read() == HEADER) { // assess data package frame header 0x59
			uart[0] = HEADER;

			if (DistanceSensor.read() == HEADER) { // assess data package frame header 0x59
				uart[1] = HEADER;

				for (int i = 2; i < 9; i++) { // save data in array
					uart[i] = DistanceSensor.read();
				}

				int check = 0;
				for (int i = 0; i <= 7; i++) {
					check += uart[i];
				}

				if (uart[8] == (check & 0xff)) { // verify the received data as per protocol
					int distance      = uart[2] + uart[3] * 256; // actual distance measurements of LiDAR
					int strength      = uart[4] + uart[5] * 256; // signal strength of LiDAR
					float temperature = ((uart[6] + uart[7] * 256) / 8) - 256;

					#if defined(DEBUG) && true

						/**
						 *
						 * Since this printing happens only when the DistanceSensor sensor is available,
						 * this is still fast enough.
						 *
						 * However, if you print everytime the function "measure" is called,
						 * it will be too slow and will severely lag behind
						 *
						 * You can still disable this by **not** defining DEBUG
						 * and/or changing true to false in the #if macro condition
						 *
						 */

						char output[1024];

						sprintf(output, "distance %d; strength %d; temperature %.4f (celcius degrees); check %d;",
						                 distance,    strength,    temperature,                        check     );

						Serial.println(output);

					#endif

					previous_valid_distance = distance;
					return distance;
				}
			}
		}
	}

	return previous_valid_distance;
}

/** END UTILS */

/** BEGIN STRUCTS & FUNCTIONS FOR MAIN PROGRAM */

/**
 * Increase these if needed but beware to not exceed the max memory
 * (by chking the output when uploading)
 *
 */
#define MAX_LED_COUNT   16
#define MAX_RANGE_COUNT 16

struct Range {
	size_t from;   /** from specified to 0, unless a smaller range is provided */
	uint8_t LEDs[MAX_LED_COUNT];

	static Range find_matching_by_distance(const Range ranges[], const size_t ranges_length, size_t distance) {
		Range candidate;

		/**
		 * O(n)
		 *   where n = ranges_length
		 *
		 * Could be improved to
		 *
		 * O(log n)
		 *
		 * via binary search, but since n is extremely small,
		 * it's not even worth bothering
		 *
		*/
		for (size_t i = 0; i < ranges_length; i++) {
			Range current = ranges[i];

			if (distance <= current.from) {
				candidate = current;
			} else {
				break;
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

	void init() const {
		/**
		 * TODO FIXME INVESTIGATE BROKEN:
		 *
		 * If we call `setPinsToMode`, **all* LEDs turn on and **do not** react
		 * like they're supposed to;
		 *
		 * If we don't -- it just works! Wtf?
		 *
		*/

		/** BEGIN DISABLE */

		// setPinsToMode(LED_pins, pin_count, OUTPUT);

		/** END DISABLE */

		if (pin_count > MAX_LED_COUNT) {
			// throw "MAX_LED_COUNT smaller than pin_count -- increase MAX_LED_COUNT";
		}

		if (range_count > MAX_RANGE_COUNT) {
			// throw "MAX_RANGE_COUNT smaller than pin_count -- increase MAX_RANGE_COUNT";
		}
	}

	void draw(size_t distance) const {
		Range matching_range = Range::find_matching_by_distance(ranges, range_count, distance);

		for (size_t i = 0; i < pin_count; i++) {
			uint8_t pin = LED_pins[i];
			uint8_t LED = matching_range.LEDs[i];

			setLEDBrightness(pin, LED);
		}
	}

	/** mostly for debugging -- you probably should not need this in production */
	void undraw() const {
		for (size_t i = 0; i < pin_count; i++) {
			uint8_t pin = LED_pins[i];
			setLEDBrightness(pin, 0);
		}
	}
};

/** END STRUCTS & FUNCTIONS FOR MAIN PROGRAM */

/** BEGIN CONFIGS FOR MAIN PROGRAM */

const Config config_1 = {
    .pin_count = 4 * 3, /** shall not exceed MAX_LED_COUNT */

    .range_count = 11, /** shall not exceed MAX_RANGE_COUNT */

    .LED_pins = {
         1,  2,  3,
         4,  5,  6,
         7,  8,  9,
        10, 11, 12,

        /** don't forget to update pin_count */
    },

    .ranges = {
    /*  from        LEDs          */
        9000,       {
                        0,   0,   0,
                        0,   0,   0,
                        0,   0,   0,
                        0,   0,   0,
                    },

         200,       {
                      128,   0,   0,
                        0,   0,   0,
                        0,   0,   0,
                        0,   0,   0,
                    },

         180,       {
                      255, 128,   0,
                        0,   0,   0,
                        0,   0,   0,
                        0,   0,   0,
                    },

         160,       {
                      255, 255, 128,
                        0,   0,   0,
                        0,   0,   0,
                        0,   0,   0,
                    },

         140,       {
                      255, 255, 255,
                      128,   0,   0,
                        0,   0,   0,
                        0,   0,   0,
                    },

         120,       {
                      255, 255, 255,
                      255, 128,   0,
                        0,   0,   0,
                        0,   0,   0,
                    },

          40,       {
                      255, 255, 255,
                      255, 255, 128,
                        0,   0,   0,
                        0,   0,   0,
                    },

          30,       {
                      255, 255, 255,
                      255, 255, 255,
                      128,   0,   0,
                        0,   0,   0,
                    },

          20,       {
                      255, 255, 255,
                      255, 255, 255,
                      255, 128,   0,
                        0,   0,   0,
                    },

          10,       {
                      255, 255, 255,
                      255, 255, 255,
                      255, 255, 128,
                        0,   0,   0,
                    },

           5,       {
                      255, 255, 255,
                      255, 255, 255,
                      255, 255, 255,
                        0,   0,   0,
                    },

         /** don't forget to update range_count */
    }
};

/** END CONFIGS FOR MAIN PROGRAM */

/** BEGIN MAIN PROGRAM */

const Config& selectedConfig = config_1;

void setup() {
	Serial.begin(9600);           // set bit rate of serial port connecting Arduino with computer
	DistanceSensor.begin(115200); // set bit rate of serial port connecting LiDAR with Arduino

	selectedConfig.init();
}

void loop() {
	int distance = measure();

	selectedConfig.draw(distance);
}

/** END MAIN PROGRAM */

