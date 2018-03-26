/***********************************
This is our GPS library

Adafruit invests time and resources providing this open source code,
please support Adafruit and open-source hardware by purchasing
products from Adafruit!

Written by Limor Fried/Ladyada for Adafruit Industries.
BSD license, check license.txt for more information
All text above must be included in any redistribution
****************************************/
#if defined(__AVR__) && defined(USE_SW_SERIAL)
  // Only include software serial on AVR platforms (i.e. not on Due).
  #include <SoftwareSerial.h>
#endif
#include <Adafruit_GPS.h>

// how long are max NMEA lines to parse?
#define MAXLINELENGTH 120

// we double buffer: read one line in and leave one for the main program
volatile char line1[MAXLINELENGTH];
volatile char line2[MAXLINELENGTH];
// our index into filling the current line
volatile uint8_t lineidx=0;
// pointers to the double buffers
volatile char *currentline;
volatile char *lastline;
volatile bool recvdflag;
volatile bool inStandbyMode;

static const char *next_data(const char *nmea) {
  nmea = strchr(nmea, ',');
  if (!nmea) return NULL;
  nmea++;
  return (*nmea) ? nmea : NULL;
}

static bool decode_angle(const char *buffer, int32_t *angle_degree_minute, int32_t *angle_degree) {
  char degreebuff[10];
  if (!buffer || strlen(buffer) < 6 || !angle_degree_minute || !angle_degree) {
    return false;
  }
  if (buffer[4] == '.') {
    degreebuff[0] = '0';
    strncpy(degreebuff + 1, buffer, 2);
    buffer += 2;
  } else if (buffer[5] == '.') {
    strncpy(degreebuff, buffer, 3);
    buffer += 3;
  } else {
    *angle_degree_minute = 0;
    *angle_degree = 0;
    return false;
  }
  degreebuff[3] = '\0';

  long minutes;
  int32_t degree;
  degree = atol(degreebuff) * 10000000;
  strncpy(degreebuff, buffer, 2); // minutes
  buffer += 3; // skip decimal point
  strncpy(degreebuff + 2, buffer, 4);
  degreebuff[6] = '\0';
  minutes = atol(degreebuff);
  *angle_degree = degree + 50 * minutes / 3;
  *angle_degree_minute = degree + minutes * 10;
  return true;
}

bool Adafruit_GPS::parse(const char *nmea) {
  // do checksum check
  // first look if we even have one
  if (nmea[strlen(nmea)-4] != '*')
    return false;

  uint16_t sum = parseHex(nmea[strlen(nmea)-3]) * 16;
  sum += parseHex(nmea[strlen(nmea)-2]);
  // check checksum
  for (uint8_t i=2; i < (strlen(nmea)-4); i++) {
    sum ^= nmea[i];
  }
  if (sum != 0) {
    // bad checksum :(
    return false;
  }
  // look for a few common sentences
  if (strstr(nmea, "$GPGGA")) {
    return parse_GPGGA(nmea);
  } else if (strstr(nmea, "$GPRMC")) {
    return parse_GPRMC(nmea);
  } else if (strstr(nmea, "$PGTOP")) {
    return parse_PGTOP(nmea);
  } else if (strstr(nmea, "$GPGSV")) {
    return parse_GPGSV(nmea);
  }
  return false;
}

char Adafruit_GPS::read() {
  char c = 0;

  if (paused) return c;

#if defined(__AVR__) && defined(USE_SW_SERIAL)
  if(gpsSwSerial) {
    if(!gpsSwSerial->available()) return c;
    c = gpsSwSerial->read();
  } else
#endif
  {
    if(!gpsHwSerial->available()) return c;
    c = gpsHwSerial->read();
  }

  //Serial.print(c);

//  if (c == '$') {         //please don't eat the dollar sign - rdl 9/15/14
//    currentline[lineidx] = 0;
//    lineidx = 0;
//  }
  if (c == '\n') {
    currentline[lineidx] = 0;

    if (currentline == line1) {
      currentline = line2;
      lastline = line1;
    } else {
      currentline = line1;
      lastline = line2;
    }

    //Serial.println("----");
    //Serial.println((char *)lastline);
    //Serial.println("----");
    lineidx = 0;
    recvdflag = true;
  }

  currentline[lineidx++] = c;
  if (lineidx >= MAXLINELENGTH)
    lineidx = MAXLINELENGTH-1;

  return c;
}

#if defined(__AVR__) && defined(USE_SW_SERIAL)
// Constructor when using SoftwareSerial or NewSoftSerial
#if ARDUINO >= 100
Adafruit_GPS::Adafruit_GPS(SoftwareSerial *ser)
#else
Adafruit_GPS::Adafruit_GPS(NewSoftSerial *ser)
#endif
{
  common_init();     // Set everything to common state, then...
  gpsSwSerial = ser; // ...override gpsSwSerial with value passed.
}
#endif

// Constructor when using HardwareSerial
Adafruit_GPS::Adafruit_GPS(HardwareSerial *ser) {
  common_init();  // Set everything to common state, then...
  gpsHwSerial = ser; // ...override gpsHwSerial with value passed.
}

// Initialization code used by all constructor types
void Adafruit_GPS::common_init() {
#if defined(__AVR__) && defined(USE_SW_SERIAL)
  gpsSwSerial = NULL; // Set both to NULL, then override correct
#endif
  gpsHwSerial = NULL; // port pointer in corresponding constructor
  recvdflag   = false;
  paused      = false;
  lineidx     = 0;
  currentline = line1;
  lastline    = line2;

  hour = minute = seconds = year = month = day =
    fixquality = satellites = 0; // uint8_t
  lat = lon = mag = 0; // char
  fix = false; // bool
  milliseconds = 0; // uint16_t
  satellites_in_views = 0;
  geoidheight = altitude = speed = angle = magvariation = HDOP = 0.0; // float
  latitude_degree_minute = longitude_degree_minute = latitude_degree = longitude_degree = 0;
}

void Adafruit_GPS::begin(uint32_t baud)
{
#if defined(__AVR__) && defined(USE_SW_SERIAL)
  if(gpsSwSerial)
    gpsSwSerial->begin(baud);
  else
#endif
    gpsHwSerial->begin(baud);

  delay(10);
}

void Adafruit_GPS::sendCommand(const char *str) {
#if defined(__AVR__) && defined(USE_SW_SERIAL)
  if(gpsSwSerial)
    gpsSwSerial->println(str);
  else
#endif
    gpsHwSerial->println(str);
}

bool Adafruit_GPS::newNMEAreceived() {
  return recvdflag;
}

void Adafruit_GPS::pause(bool p) {
  paused = p;
}

const char *Adafruit_GPS::lastNMEA() {
  recvdflag = false;
  return (char *)lastline;
}

// read a Hex value and return the decimal equivalent
uint8_t Adafruit_GPS::parseHex(char c) {
    if (c < '0')
      return 0;
    if (c <= '9')
      return c - '0';
    if (c < 'A')
       return 0;
    if (c <= 'F')
       return (c - 'A')+10;
    // if (c > 'F')
    return 0;
}

bool Adafruit_GPS::waitForSentence(const char *wait4me, uint8_t max) {
  char str[20];

  uint8_t i=0;
  while (i < max) {
    read();

    if (newNMEAreceived()) {
      const char *nmea = lastNMEA();
      strncpy(str, nmea, 20);
      str[19] = 0;
      i++;

        if (strstr(str, wait4me))
	return true;
    }
  }

  return false;
}

bool Adafruit_GPS::LOCUS_StartLogger() {
  sendCommand(PMTK_LOCUS_STARTLOG);
  recvdflag = false;
  return waitForSentence(PMTK_LOCUS_STARTSTOPACK);
}

bool Adafruit_GPS::LOCUS_StopLogger() {
  sendCommand(PMTK_LOCUS_STOPLOG);
  recvdflag = false;
  return waitForSentence(PMTK_LOCUS_STARTSTOPACK);
}

bool Adafruit_GPS::LOCUS_ReadStatus() {
  sendCommand(PMTK_LOCUS_QUERY_STATUS);

  if (! waitForSentence("$PMTKLOG"))
    return false;

  const char *response = lastNMEA();
  uint16_t parsed[10];
  uint8_t i;

  for (i=0; i<10; i++) parsed[i] = -1;

  response = next_data(response); if (!response) return false;
  for (i=0; i<10; i++) {
    if (!response || (response[0] == 0) || (response[0] == '*'))
      break;
    response++;
    parsed[i]=0;
    while ((response[0] != ',') &&
	   (response[0] != '*') && (response[0] != 0)) {
      parsed[i] *= 10;
      char c = response[0];
      if (isDigit(c))
        parsed[i] += c - '0';
      else
        parsed[i] = c;
      response++;
    }
  }
  LOCUS_serial = parsed[0];
  LOCUS_type = parsed[1];
  if (isAlpha(parsed[2])) {
    parsed[2] = parsed[2] - 'a' + 10;
  }
  LOCUS_mode = parsed[2];
  LOCUS_config = parsed[3];
  LOCUS_interval = parsed[4];
  LOCUS_distance = parsed[5];
  LOCUS_speed = parsed[6];
  LOCUS_status = !parsed[7];
  LOCUS_records = parsed[8];
  LOCUS_percent = parsed[9];

  return true;
}

// Standby Mode Switches
bool Adafruit_GPS::standby() {
  if (inStandbyMode) {
    return false;  // Returns false if already in standby mode, so that you do not wake it up by sending commands to GPS
  }
  else {
    inStandbyMode = true;
    sendCommand(PMTK_STANDBY);
    //return waitForSentence(PMTK_STANDBY_SUCCESS);  // don't seem to be fast enough to catch the message, or something else just is not working
    return true;
  }
}

bool Adafruit_GPS::wakeup() {
  if (inStandbyMode) {
   inStandbyMode = false;
    sendCommand("");  // send byte to wake it up
    return waitForSentence(PMTK_AWAKE);
  }
  else {
      return false;  // Returns false if not in standby mode, nothing to wakeup
  }
}

bool Adafruit_GPS::parse_GPGGA(const char *nmea) {
  // found GGA
  const char *p = nmea;
  // get time
  int32_t degree;
  long minutes;
  char degreebuff[10];
  p = next_data(p); if (!p) return false;
  float timef = atof(p);
  uint32_t time = timef;
  hour = time / 10000;
  minute = (time % 10000) / 100;
  seconds = (time % 100);

  milliseconds = fmod(timef, 1.0) * 1000;

  parse_latitude_longitude(&p);

  p = next_data(p); if (!p) return false;
  if (',' != *p)
    fixquality = atoi(p);

  p = next_data(p); if (!p) return false;
  if (',' != *p)
    satellites = atoi(p);

  p = next_data(p); if (!p) return false;
  if (',' != *p)
    HDOP = atof(p);

  p = next_data(p); if (!p) return false;
  if (',' != *p)
    altitude = atof(p);

  p = next_data(p); if (!p) return false;
  p = next_data(p); if (!p) return false;
  if (',' != *p)
    geoidheight = atof(p);
  return true;
}

bool Adafruit_GPS::parse_GPRMC(const char *nmea) {
 // found RMC
  const char *p = nmea;

  // get time
  int32_t degree;
  long minutes;
  char degreebuff[10];
  p = next_data(p); if (!p) return false;
  float timef = atof(p);
  uint32_t time = timef;
  hour = time / 10000;
  minute = (time % 10000) / 100;
  seconds = (time % 100);

  milliseconds = fmod(timef, 1.0) * 1000;

  p = next_data(p); if (!p) return false;
  // Serial.println(p);
  if (p[0] == 'A')
    fix = true;
  else if (p[0] == 'V')
    fix = false;
  else
    return false;

  parse_latitude_longitude(&p);

  // speed
  p = next_data(p); if (!p) return false;
  if (',' != *p)
    speed = atof(p);

  // angle
  p = next_data(p); if (!p) return false;
  if (',' != *p)
    angle = atof(p);

  p = next_data(p); if (!p) return false;
  if (',' != *p)
  {
    uint32_t fulldate = atof(p);
    day = fulldate / 10000;
    month = (fulldate % 10000) / 100;
    year = (fulldate % 100);
  }
  // we dont parse the remaining, yet!
  return true;
}

bool Adafruit_GPS::parse_PGTOP(const char *nmea) {
  const char *p = nmea;
  p = next_data(p); if (!p) return false;
  p = next_data(p); if (!p) return false;
  int value = atoi(p);
  if (value == 1) {
    antenna = Adafruit_GPS::ExternalProblemAntenna;
  } else if (value == 2) {
    antenna = Adafruit_GPS::InternalAntenna;
  } else if (value == 3) {
    antenna = Adafruit_GPS::ExternalAntenna;
  } else {
    antenna = Adafruit_GPS::UnknownAntenna;
  }
  return true;
}

bool Adafruit_GPS::parse_GPGSV(const char *nmea) {
  // $GPGSV,4,1,14,22,87,059,12,01,82,080,23,03,69,248,34,11,67,155,15*7A
  const char *p = nmea;
  p = next_data(p); if (!p) return false;
  p = next_data(p); if (!p) return false;
  p = next_data(p); if (!p) return false;
  satellites_in_views = atoi(p);
  return true;
}

bool Adafruit_GPS::parse_latitude_longitude(const char **buffer) {
  if (!buffer) return false;
  // parse out latitude
  *buffer = next_data(*buffer); if (!*buffer) return false;
  if (',' != **buffer) {
    if (!decode_angle(*buffer, &latitude_degree_minute, &latitude_degree)) {
      return false;
    }
  }
  *buffer = next_data(*buffer); if (!*buffer) return false;
  if (',' != **buffer) {
    if ((*buffer)[0] == 'S') {
      latitude_degree = -latitude_degree;
    }
    if ((*buffer)[0] == 'N') lat = 'N';
    else if ((*buffer)[0] == 'S') lat = 'S';
    else if ((*buffer)[0] == ',') lat = 0;
    else return false;
  }

  // parse out longitude
  *buffer = next_data(*buffer); if (!*buffer) return false;
  if (',' != **buffer) {
    if (!decode_angle(*buffer, &longitude_degree_minute, &longitude_degree)) {
      return false;
    }
  }
  *buffer = next_data(*buffer); if (!*buffer) return false;
  if (',' != **buffer) {
    if ((*buffer)[0] == 'W') {
      longitude_degree = -longitude_degree;
    }
    if ((*buffer)[0] == 'W') lon = 'W';
    else if ((*buffer)[0] == 'E') lon = 'E';
    else if ((*buffer)[0] == ',') lon = 0;
    else return false;
  }
}
