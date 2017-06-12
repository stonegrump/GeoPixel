/******************************************************************************

GeoCache Hunt Project (GeoCache.cpp)

This is skeleton code provided as a project development guideline only.  You
are not required to follow this coding structure.  You are free to implement
your project however you wish.

Team Name:
Never Get Lost

Team Members:
1.
2.
3.
4.

NOTES:

You only have 32k of program space and 2k of data space.  You must
use your program and data space wisely and sparingly.  You must also be
very conscious to properly configure the digital pin usage of the boards,
else weird things will happen.

The Arduino GCC sprintf() does not support printing floats or doubles.  You should
consider using sprintf(), dtostrf(), strtok() and strtod() for message string
parsing and converting between floats and strings.

The GPS provides latitude and longitude in degrees minutes format (DDDMM.MMMM).
You will need convert it to Decimal Degrees format (DDD.DDDD).  The switch on the
GPS Shield must be set to the "Soft Serial" position, else you will not receive
any GPS messages.

*******************************************************************************

Following is the GPS Shield "GPRMC" Message Structure.  This message is received
once a second.  You must parse the message to obtain the parameters required for
the GeoCache project.  GPS provides coordinates in Degrees Minutes (DDDMM.MMMM).
The coordinates in the following GPRMC sample message, after converting to Decimal
Degrees format(DDD.DDDDDD) is latitude(23.118757) and longitude(120.274060).  By
the way, this coordinate is GlobalTop Technology in Taiwan, who designed and
manufactured the GPS Chip.

"$GPRMC,064951.000,A,2307.1256,N,12016.4438,E,0.03,165.48,260406,3.05,W,A*2C/r/n"

$GPRMC,         // GPRMC Message
064951.000,     // utc time hhmmss.sss
A,              // status A=data valid or V=data not valid
2307.1256,      // Latitude 2307.1256 (degrees minutes format dddmm.mmmm)
N,              // N/S Indicator N=north or S=south
12016.4438,     // Longitude 12016.4438 (degrees minutes format dddmm.mmmm)
E,              // E/W Indicator E=east or W=west
0.03,           // Speed over ground knots
165.48,         // Course over ground (decimal degrees format ddd.dd)
260406,         // date ddmmyy
3.05,           // Magnetic variation (decimal degrees format ddd.dd)
W,              // E=east or W=west
A               // Mode A=Autonomous D=differential E=Estimated
*2C             // checksum
/r/n            // return and newline

Following are approximate results calculated from above GPS GPRMC message
(when GPS_ON == 0) to the GEOLAT0/GEOLON0 tree location:

degMin2DecDeg() LAT 2307.1256 N = 23.118757 decimal degrees
degMin2DecDeg() LON 12016.4438 E = 120.274060 decimal degrees
calcDistance() to GEOLAT0/GEOLON0 target = 45335760 feet
calcBearing() to GEOLAT0/GEOLON0 target = 22.999655 degrees

The resulting relative target bearing to the tree is 217.519650 degrees

******************************************************************************/

/*
Configuration settings.

These defines makes it easy for you to enable/disable certain
code during the development and debugging cycle of this project.
There may not be sufficient room in the PROGRAM or DATA memory to
enable all these libraries at the same time.  You must have have
NEO_ON, GPS_ON and SDC_ON during the actual GeoCache Flag Hunt on
Finals Day
*/
#define NOT_STUPID_BRIGHT 16

#define NEO_ON	1		// NeoPixelShield
#define TRM_ON 1		// SerialTerminal
#define GPS_ON	0		// Live GPS Message (off = simulated)
#define SDC_ON 1		// SD Card

// define pin usage
#define NEO_TX	6		// NEO transmit
#define GPS_TX	7		// GPS transmit
#define GPS_RX	8		// GPS receive

// GPS message buffer
#define GPS_RX_BUFSIZ	128
char cstr[GPS_RX_BUFSIZ];

// global variables
uint8_t target = 0;		// target number
float heading = 0.0;	// target heading
float distance = 0.0;	// target distance
bool previousButtonState = false;

float lat;
float lon;
//File mapFile;			// current file to write the map data

#if GPS_ON
#include <SoftwareSerial.h>
SoftwareSerial gps(GPS_RX, GPS_TX);
#endif

#if NEO_ON
#include <Adafruit_NeoPixel.h>
Adafruit_NeoPixel strip = Adafruit_NeoPixel(40, NEO_TX, NEO_GRB + NEO_KHZ800);
#endif

#if SDC_ON
#include <SD.h>
File mapFile;			// current file to write the map data
#endif

/*
Following is a Decimal Degrees formatted waypoint for the large tree
in the parking lot just outside the front entrance of FS3B-116.
*/
#define GEOLAT0 28.594532
#define GEOLON0 -81.304437

#if GPS_ON
/*
These are GPS command messages (only a few are used).
*/
#define PMTK_AWAKE "$PMTK010,002*2D"
#define PMTK_STANDBY "$PMTK161,0*28"
#define PMTK_Q_RELEASE "$PMTK605*31"
#define PMTK_ENABLE_WAAS "$PMTK301,2*2E"
#define PMTK_ENABLE_SBAS "$PMTK313,1*2E"
#define PMTK_CMD_HOT_START "$PMTK101*32"
#define PMTK_CMD_WARM_START "$PMTK102*31"
#define PMTK_CMD_COLD_START "$PMTK103*30"
#define PMTK_CMD_FULL_COLD_START "$PMTK104*37"
#define PMTK_SET_BAUD_9600 "$PMTK251,9600*17"
#define PMTK_SET_BAUD_57600 "$PMTK251,57600*2C"
#define PMTK_SET_NMEA_UPDATE_1HZ  "$PMTK220,1000*1F"
#define PMTK_SET_NMEA_UPDATE_5HZ  "$PMTK220,200*2C"
#define PMTK_API_SET_FIX_CTL_1HZ  "$PMTK300,1000,0,0,0,0*1C"
#define PMTK_API_SET_FIX_CTL_5HZ  "$PMTK300,200,0,0,0,0*2F"
#define PMTK_SET_NMEA_OUTPUT_RMC "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
#define PMTK_SET_NMEA_OUTPUT_GGA "$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
#define PMTK_SET_NMEA_OUTPUT_RMCGGA "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
#define PMTK_SET_NMEA_OUTPUT_OFF "$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"

#endif // GPS_ON

//#include <iostream>
//#include <sstream>

/*************************************************
**** GEO FUNCTIONS - BEGIN ***********************
*************************************************/

/**************************************************
Convert Degrees Minutes (DDMM.MMMM) into Decimal Degrees (DDD.DDDD)

float degMin2DecDeg(char *cind, char *ccor)

Input:
cind = string char pointer containing the GPRMC latitude(N/S) or longitude (E/W) indicator
ccor = string char pointer containing the GPRMC latitude or longitude DDDMM.MMMM coordinate

Return:
Decimal degrees coordinate.

Mike is responsible
**************************************************/
float degMin2DecDeg(char *cind, char *ccor)
{
	float degrees = 0.0;

	//seperate Num from Char*
	double degreePart, minutePart;
	//String degreeString, minuteString;
	char degreeString[2], minuteString[7];

	for (int i = 0; i < 9; i++)
	{
		if (i < 2)
		{
			degreeString[i] = ccor[i];
		}
		else
		{			
			minuteString[i - 2] = ccor[i];
		}
	}

	//degreePart = atof(degreeString.c_str());
	//minutePart = atof(minuteString.c_str());

	degreePart = strtod(degreeString, NULL);
	minutePart = strtod(minuteString, NULL);

	//convert
	degrees = degreePart + minutePart / 60.0;

	//negate if S or W
	if (*cind == 'S' || *cind == 's' || *cind == 'W' || *cind == 'w')
	{
		degrees *= -1.0;
	}

	//Serial.println(degreeString);
	//Serial.println(minuteString);
	//Serial.println(String(degreePart, 6).c_str());
	//Serial.println(String(minutePart, 6).c_str());
	//Serial.println(String(degrees, 6).c_str());

	return(degrees);
}

/**************************************************
Calculate Great Circle Distance between to coordinates using
Haversine formula.

float calcDistance(float flat1, float flon1, float flat2, float flon2)

EARTH_RADIUS_FEET = 3959.00 radius miles * 5280 feet per mile

Input:
flat1, flon1 = first latitude and longitude coordinate in decimal degrees
flat2, flon2 = second latitude and longitude coordinate in decimal degrees

Return:
distance in feet (3959 earth radius in miles * 5280 feet per mile)

Mike is responsible//yes yes
**************************************************/
float calcDistance(float flat1, float flon1, float flat2, float flon2)
{
	float distance = 0.0;
  int radius = 6371;
  
	// add code here 
	float lat1 = flat1 * (3.14 / 180);
	float lat2 = flat2 * (3.14 / 180);

	float latting = (lat2 - lat1) * (3.14 / 180);
	float longing = (flon2 - flon1) * (3.14 / 180);

	float a = sin(latting / 2) * sin(latting / 2) +
		cos(lat1) * cos(lat2) *
		sin(longing / 2) * sin(longing / 2);

  float c = 2 * atan2(sqrt(a), sqrt(1-a));

  distance = radius * c;
	return(distance);
}

/**************************************************
Calculate Great Circle Bearing between two coordinates

float calcBearing(float flat1, float flon1, float flat2, float flon2)

Input:
flat1, flon1 = first latitude and longitude coordinate in decimal degrees
flat2, flon2 = second latitude and longitude coordinate in decimal degrees

Return:
angle in decimal degrees from magnetic north (normalize to a range of 0 to 360)
**************************************************/
float calcBearing(float flat1, float flon1, float flat2, float flon2)
{
	float bearing = 0.0;

	float flatRad1, flatRad2, flonRad1, flonRad2;
	flatRad1 = DEG_TO_RAD*flat1;
	flatRad2 = DEG_TO_RAD*flat2;
	flonRad1 = DEG_TO_RAD*flon1;
	flonRad2 = DEG_TO_RAD*flon2;

	float x, y;
	y = sin(flonRad2 - flonRad1) * cos(flatRad2);
	x = cos(flatRad1) * sin(flatRad2) - sin(flatRad1)*cos(flatRad2)*cos(flonRad2 - flonRad1);
	
	bearing = atan2(y, x);
	bearing = DEG_TO_RAD*bearing;
	bearing = fmod(bearing + 360.0f, 360.0f);

	return(bearing);
}

/*************************************************
**** GEO FUNCTIONS - END**************************
*************************************************/

#if NEO_ON
/*
Sets target number, heading and distance on NeoPixel Display

NOTE: Target number, bearing and distance parameters used
by this function do not need to be passed in, since these
parameters are in global data space.

*/
void setNeoPixel(uint8_t _target, float _heading, float _distance)
{


	float d10 = (2500.0 - distance) / 250.0;
	uint16_t secondBase = (2500 - distance) - ((uint8_t)d10 * 250);

	float curCol = (255.0f / 10.0f) * d10;

	int8_t i = 37, j = 0;
	for (i = 37, j = 0; i > 0; i -= 8, ++j) {
		if (d10 / 2 == 5)
			strip.setPixelColor(i, 255, 255, 255);
		else {
			if (j <= (d10 / 2))
				strip.setPixelColor(i, 255 - (curCol), (curCol), 0);
			else
				strip.setPixelColor(i, 0);
}
		for (int8_t x = 38, y = 0; x > 0; x -= 8, ++y) {
			if (secondBase / 50 == 5)
				strip.setPixelColor(x, 255, 255, 255);
			else {
				if (y <= (secondBase / 50))
					strip.setPixelColor(x, 255 - (secondBase), (secondBase), 0);
				else
					strip.setPixelColor(x, 0);
			}
		}
	}

	heading += 10;
	if (heading < 0 || heading > 360)
		heading = 0;

	uint8_t outerLEDs[16]{ 2,3,4,12,20,28,36,35,34,33,32, 24, 16, 8, 0, 1 };
	uint8_t innerLEDs[8]{ 10, 11, 19, 27, 26, 25, 17, 9 };
	uint8_t centerLED = 18;
	uint8_t dir = (uint8_t)(_heading / 22.5);
	for (i = 0; i < 16; ++i) {
		if (i == dir)
			strip.setPixelColor(outerLEDs[i], 255 - (curCol), curCol, 0);
		else
			strip.setPixelColor(outerLEDs[i], 64, 192, 128);
	}
	for (i = 0; i < 8; ++i) {
		if (i == (dir >> 1))
			strip.setPixelColor(innerLEDs[i], 255 - (curCol), curCol, 0);
		else
			strip.setPixelColor(innerLEDs[i], 64, 192, 128);
	}
	strip.setPixelColor(centerLED, 255 - (curCol), curCol, 0);

	for (i = 39, j = 0; i > 7; i -= 8, ++j) {
		if (j < _target) {
			strip.setPixelColor(i, 0, 255, 0);
		}
		else if (j == _target) {
			strip.setPixelColor(i, 255, 255, 0);
		}
		else {
			strip.setPixelColor(i, 255, 0, 0);
		}
	}

	strip.show();

	
	distance -= 10;
	if (distance < 0)
		distance = 2500;
}

#endif	// NEO_ON

#if GPS_ON
/*
Get valid GPS message. This function returns ONLY once a second.

NOTE: DO NOT CHANGE THIS CODE !!!

void getGPSMessage(void)

Side affects:
Message is placed in global "cstr" string buffer.

Input:
none

Return:
none

*/
void getGPSMessage(void)
{
	uint8_t x = 0, y = 0, isum = 0;

	memset(cstr, 0, sizeof(cstr));

	// get nmea string
	while (true)
	{
		if (gps.peek() != -1)
		{
			cstr[x] = gps.read();

			// if multiple inline messages, then restart
			if ((x != 0) && (cstr[x] == '$'))
			{
				x = 0;
				cstr[x] = '$';
			}

			// if complete message
			if ((cstr[0] == '$') && (cstr[x++] == '\n'))
			{
				// nul terminate string before /r/n
				cstr[x - 2] = 0;

				// if checksum not found
				if (cstr[x - 5] != '*')
				{
					x = 0;
					continue;
				}

				// convert hex checksum to binary
				isum = strtol(&cstr[x - 4], NULL, 16);

				// reverse checksum
				for (y = 1; y < (x - 5); y++) isum ^= cstr[y];

				// if invalid checksum
				if (isum != 0)
				{
					x = 0;
					continue;
				}

				// else valid message
				break;
			}
		}
	}
}

#else
/*
Get simulated GPS message once a second.

This is the same message and coordinates as described at the top of this
file.  You could edit these coordinates to point to the tree out front (GEOLAT0,
GEOLON0) to test your distance and direction calculations.  Just note that the
tree coordinates are in Decimal Degrees format, and the message coordinates are
in Degrees Minutes format.

NOTE: DO NOT CHANGE THIS CODE !!!

void getGPSMessage(void)

Side affects:
Static GPRMC message is placed in global "cstr" null terminated char string buffer.

Input:
none

Return:
none

*/
void getGPSMessage(void)
{
	static unsigned long gpsTime = 0;

	// simulate waiting for message
	while (gpsTime > millis()) delay(100);

	// do this once a second
	gpsTime = millis() + 1000;

	memcpy(cstr, "$GPRMC,064951.000,A,2307.1256,N,12016.4438,E,0.03,165.48,260406,3.05,W,A*2C", sizeof(cstr));

	return;
}

#endif	// GPS_ON

void setup(void)
{
#if TRM_ON
	// init serial interface
	Serial.begin(115200);
#endif	

#if NEO_ON
	// init NeoPixel Shield

	strip.begin();
	strip.show(); // Initialize all pixels to 'off'

	strip.setBrightness(NOT_STUPID_BRIGHT);
#endif	

#if SDC_ON
	/*
	Initialize the SecureDigitalCard and open a numbered sequenced file
	name "MyMapNN.txt" for storing your coordinates, where NN is the
	sequential number of the file.  The filename can not be more than 8
	chars in length (excluding the ".txt").
	*/
	SD.begin();
	File root = SD.open("/");
	int8_t fileCount = -1;
	while (true)
	{
		File entry = root.openNextFile();\
		if (!entry)
		{
			entry.close();
			break;
		}
		++fileCount;
		entry.close();
	}

	fileCount = fileCount % 100;
	char mapFileName[15] = "MyMapNN.txt";
	if (fileCount < 10)
		mapFileName[5] = '0';
	else
		mapFileName[5] = 48 + (fileCount / 10);
	mapFileName[6] = 48 + (fileCount % 10);
	root.close();

	mapFile = SD.open(mapFileName, FILE_WRITE);
#endif

#if GPS_ON
	// enable GPS sending GPRMC message
	gps.begin(9600);
	gps.println(PMTK_SET_NMEA_UPDATE_1HZ);
	gps.println(PMTK_API_SET_FIX_CTL_1HZ);
	gps.println(PMTK_SET_NMEA_OUTPUT_RMC);
#endif		

	// init target button here
	pinMode(2, INPUT_PULLUP);
	
	//Serial.println("Start converting");
	//char * c1 = "S";
	////char * c2 = "9999.9999";
	//char * c2 = "1234.5678";
	////char * c2 = "0043.5677";
	//degMin2DecDeg(c1, c2);
}

void loop(void)
{
	// max 1 second blocking call till GPS message received
	getGPSMessage();
	bool newButtonState = !digitalRead(2);
	// if button pressed, set new target
	if (!previousButtonState && newButtonState) {
		if (target < 4) {
			strip.setPixelColor(7, 255, 255, 255);
			strip.show();
			target++;
		}
	}
	else if (previousButtonState && !newButtonState)
	{
		strip.setPixelColor(7, 0);
		strip.show();
	}
	previousButtonState = newButtonState;

	// if GPRMC message (3rd letter = R)
	while (cstr[3] == 'R')
	{

		/*Following is the GPS Shield "GPRMC" Message Structure.This message is received
			once a second.You must parse the message to obtain the parameters required for
			the GeoCache project.GPS provides coordinates in Degrees Minutes(DDDMM.MMMM).
			The coordinates in the following GPRMC sample message, after converting to Decimal
			Degrees format(DDD.DDDDDD) is latitude(23.118757) and longitude(120.274060).By
			the way, this coordinate is GlobalTop Technology in Taiwan, who designed and
			manufactured the GPS Chip.

			"$GPRMC,064951.000,A,2307.1256,N,12016.4438,E,0.03,165.48,260406,3.05,W,A*2C/r/n"

			$GPRMC,         // GPRMC Message
			064951.000,     // utc time hhmmss.sss
			A,              // status A=data valid or V=data not valid
			2307.1256,      // Latitude 2307.1256 (degrees minutes format dddmm.mmmm)
			N,              // N/S Indicator N=north or S=south
			12016.4438,     // Longitude 12016.4438 (degrees minutes format dddmm.mmmm)
			E,              // E/W Indicator E=east or W=west
			0.03,           // Speed over ground knots
			165.48,         // Course over ground (decimal degrees format ddd.dd)
			260406,         // date ddmmyy
			3.05,           // Magnetic variation (decimal degrees format ddd.dd)
			W,              // E=east or W=west
			A               // Mode A=Autonomous D=differential E=Estimated
			* 2C             // checksum
			/ r / n            // return and newline

			Following are approximate results calculated from above GPS GPRMC message
			(when GPS_ON == 0) to the GEOLAT0 / GEOLON0 tree location :

		degMin2DecDeg() LAT 2307.1256 N = 23.118757 decimal degrees
			degMin2DecDeg() LON 12016.4438 E = 120.274060 decimal degrees
			calcDistance() to GEOLAT0 / GEOLON0 target = 45335760 feet
			calcBearing() to GEOLAT0 / GEOLON0 target = 22.999655 degrees

			The resulting relative target bearing to the tree is 217.519650 degrees

			******************************************************************************/
		// parse message parameters

		double course;
		char buffer[100];
		char sChar;
		uint8_t commaNum = 0;
		uint16_t modNum;

		for (uint16_t i = 0; i < strlen(cstr); ++i) {
			for (; cstr[i] != ','; ++i);
			++i;
			for (; cstr[i] != ','; ++i);
			++i;
			for (; cstr[i] != ','; ++i);
			++i;
			modNum = i;
			for (; cstr[i] != ','; ++i) {
				buffer[(uint8_t)fmod(i, modNum)] = cstr[i];
			}
			++i;
			sChar = cstr[i];
			i += 2;
			lat = degMin2DecDeg(&sChar, buffer);

			modNum = i;
			for (; cstr[i] != ','; ++i) {
				buffer[(uint8_t)fmod(i, modNum)] = cstr[i];
			}
			++i;
			sChar = cstr[i];
			i += 2;
			lon = degMin2DecDeg(&sChar, buffer);

		}

		// calculated destination heading
		distance = calcDistance(lat, lon, GEOLAT0, GEOLON0);

		// calculated destination distance
		heading = calcBearing(lat, lon, GEOLAT0, GEOLON0);

#if SDC_ON
		// write current position to SecureDigital then flush
		Serial.print(lat);
		Serial.print(", ");
		Serial.print(lon);
		Serial.print(", ");
		Serial.print(heading);
		Serial.print(".");
		Serial.print(distance);
		Serial.print('/n');
		//mapFile.flush();
#endif

		break;
	}

#if NEO_ON
	// set NeoPixel target display
	setNeoPixel(target, heading, distance);
#endif		

#if TRM_ON
	// print debug information to Serial Terminal
	Serial.println(cstr);
#endif		
}

bool IsButtonPressed(int8_t pin) {
	bool val;
	for (int i = 0; i < 1000; ++i) {
		val = digitalRead(pin);
		if (val == HIGH)
			return false;
	}

	return true;
}
//<<<<<<< HEAD
//}

/*
	Counts all files in a directory. This doesn't include
	files inside any of the subdirectories.
*/
//uint8_t CountDirFiles(File dir)
//{
//	if (!dir)
//		return 0;
//
//	uint8_t count = 1;
//	while (true)
//	{
//		File entry = dir.openNextFile();
//
//		if (!entry)
//			break;
//
//		++count;
//		entry.close();
//	}
//
//	count = count % 100;
//	return count;
//}
//=======
//}
//>>>>>>> e22485e95b21365ce0d76071714be86ac0138ce9
