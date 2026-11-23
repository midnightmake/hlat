#include <fstream>
#include <sstream>
#include <string>
#include <iostream>
#include <iomanip>
#include <vector>
#include <format>
#include <chrono>
#include <cmath>
#include <cfloat>
using namespace std;

// - - - defines - - - //

// versioning information
#define VERSION_MAJOR 0
#define VERSION_MINOR 8
#define VERSION_REVISION 2

// minimum log size for stats, although even this small is not really useful
#define MIN_LOG_SIZE 6

// mathematical constants
#define PI 3.14159265358979323846
#define EARTH_RADIUS 6371000
#define EPSILON 0.0001

// thresholds used to determine if a payload is ascending or descending
#define ASCENT_RATE_THRESHOLD 1.5
#define LANDED_RATE_THRESHOLD 1.0

// how many times the thresholds above must be reached in order for a flight stage transition to occur
#define NUM_CONSEC_ASCENTS 3
#define NUM_CONSEC_LANDS 3

// descent rate bucket parameters: max supported bin altitude, and bin size
#define MAX_SUPPORTED_ALT 50
#define DESCENT_RATE_BIN_SIZE 5
#define NUM_DESCENT_RATE_BINS ((MAX_SUPPORTED_ALT / DESCENT_RATE_BIN_SIZE) + 1)

// threshold used to determine when/if there is a gap in the logs
#define LOG_GAP_THRESHOLD 30

// bits identifying peripheral statuses
#define STATUS_CAM_BIT (1 << 0)
#define STATUS_DISK_BIT (1 << 1)
#define STATUS_IMU_BIT (1 << 2)
#define STATUS_ENV_BIT (1 << 3)

// - - - prototypes - - - //

typedef enum FLIGHT_STAGE
{
	FLIGHT_STAGE_PRELAUNCH = 0,
	FLIGHT_STAGE_ASCENT,
	FLIGHT_STAGE_DESCENT,
	FLIGHT_STAGE_LANDED
} FlightStage;

typedef struct HABSAT_LOG_LINE
{
	chrono::sys_seconds dateTime;		// date time object
	uint32_t up;						// up time
	uint32_t seq;						// sequence number
	uint32_t batt;						// battery in millivolts
	double lat;							// latitude
	double lon;							// longitude
	int alt;							// altitude
	int temp;							// temperature in C
	uint8_t status;						// peripheral status bitmask
	uint32_t boots;						// number of boots
	uint32_t images;					// number of high res images
	bool hasTimeData;					// is GPS time data present?
	bool hasLocationData;				// is GPS location data present?
	FlightStage flightStage;			// current state of flight
	uint32_t lineNum;					// line in the log, including CSV header
} HabSatLogLine;

typedef struct HABSAT_LOG
{
	vector<HabSatLogLine*> lines;
} HabSatLog;

// - - - prototypes - - - //

void analyzeLog(char *filename);
HabSatLog *parseLog(const vector<vector<string>> &table);
void generateReport(HabSatLog *log);
void freeLog(HabSatLog *log);
int getColumnIndex(const vector<vector<string>> &table, const string &header1, const string &header2 = "");

// math functions
double radians(double deg);
double haversineDistance(double lat1, double lon1, double lat2, double lon2);

// utilities
string formatSeconds(int seconds);
string commaDelimitNumber(int number);
string trim(string str);

// - - - implementations - - - //

int main(int args, char *argv[])
{
	if(args == 2)
	{
		analyzeLog(argv[1]);
	}
	else
	{
		cout << "[ERR] you must provide an input file argument; this" << endl;
		cout << "      may be either a HabSat Ops CSV or a CSV from" << endl;
		cout << "      your HabSat disk" << endl;
		cout << endl;
		cout << "usage: ./hlat.exe example.csv" << endl;
		exit(1);
	}

	// done
	return 0;
}

void analyzeLog(char *filename)
{
	string line;
	string token;
	stringstream ss;
	vector<string> tokens;
	vector<vector<string>> table;
	int lineNum;
	ifstream file(filename);
	HabSatLog *log;

	// make sure the file was opened
	if(file.is_open())
	{
		// read in each line
		lineNum = 1;
		while(getline(file, line))
		{
			// don't process empty lines
			if(!trim(line).empty())
			{
				// extract tokens on each line
				ss.clear();
				ss.str(line);
				tokens.clear();
				while(getline(ss, token, ','))
				{
					tokens.push_back(token);
				}
				table.push_back(tokens);

				// make sure column numbers match
				if(lineNum == 1 && tokens.size() < 2)
				{
					cout << "[ERR] log header information was not detected" << endl;
					exit(1);
				}
				else if(lineNum > 1 && table[0].size() != tokens.size())
				{
					cout << "[ERR] line " << lineNum << " has " << tokens.size() << " columns but expected " << table[0].size() << endl;
					exit(1);
				}
			}

			// advance line number
			++ lineNum;
		}

		// close the file and parse into a workable data structure
		file.close();
		log = parseLog(table);
		generateReport(log);
		freeLog(log);
	}
	else
	{
		// error message
		cout << "[ERR] the file '" << filename << "' could not be opened." << endl;
		exit(1);
	}
}

HabSatLog *parseLog(const vector<vector<string>> &table)
{
	HabSatLog *log = NULL;
	HabSatLogLine *line;

	// used for parsing and rendering dates and times
	string dateStr, timeStr;
	string dateTime;

	uint32_t latE7, lonE7;
	int32_t alt;

	// get locations of data we care about
	int dateIndex = getColumnIndex(table, "date");
	int timeIndex = getColumnIndex(table, "time");
	int upIndex = getColumnIndex(table, "up", "habSatUpTime");
	int seqIndex = getColumnIndex(table, "seq", "numTelemetryFrames");
	int battIndex = getColumnIndex(table, "batt_mv", "habSatBatteryMillivolts");
	int latIndex = getColumnIndex(table, "latE7", "habSatLat");
	int lonIndex = getColumnIndex(table, "lonE7", "habSatLong");
	int altIndex = getColumnIndex(table, "alt_m", "habSatAltitude");
	int tempIndex = getColumnIndex(table, "temp_c", "habSatTemperature");
	int statusIndex = getColumnIndex(table, "status", "habSatPeripherals");
	int bootsIndex = getColumnIndex(table, "boots", "habSatNumBoots");
	int imagesIndex = getColumnIndex(table, "images", "habSatNumHighResImages");

	// temporary vars
	stringstream ss;
	int i;

	// prime some variables
	log = new HabSatLog();

	// iterate through rows
	for(i = 1; i < (int)table.size(); ++ i)
	{
		// skip if this is a sequence number matching the last one
		if(i == 1 || table[i][seqIndex].compare(table[i - 1][seqIndex]) != 0)
		{
			// parse tokens
			line = new HabSatLogLine();
			line -> batt = atoi(table[i][battIndex].c_str());
			line -> temp = atoi(table[i][tempIndex].c_str());
			line -> up = atoi(table[i][upIndex].c_str());
			line -> boots = atoi(table[i][bootsIndex].c_str());
			line -> seq = atoi(table[i][seqIndex].c_str());
			line -> images = atoi(table[i][imagesIndex].c_str());
			line -> lineNum = i + 1;

			// status could be rendered in hex or not
			if(table[i][statusIndex].size() == 4) line -> status = (uint8_t)strtol(table[i][statusIndex].c_str(), NULL, 16);
			else 								  line -> status = atoi(table[i][statusIndex].c_str());

			// HabSat Ops renders these as "null", but the HabSat doesn't
			line -> hasTimeData = false;
			if(table[i][dateIndex].compare("null") != 0 && table[i][timeIndex].compare("null") != 0)
			{
				// parse raw GPS time data and make sure it's valid
				dateStr = table[i][dateIndex];
				timeStr = table[i][timeIndex];
				if(dateStr.compare("2000-00-00") != 0 && timeStr.compare("00:00:00") != 0)
				{
					ss.str("");
					ss << dateStr << " " << timeStr;
					ss >> chrono::parse("%F %T", line -> dateTime);
					line -> hasTimeData = !ss.fail();
				}
			}

			// HabSat Ops renders these as "null", but the HabSat doesn't
			line -> hasLocationData = false;
			if(table[i][latIndex].compare("null") != 0 &&
			   table[i][lonIndex].compare("null") != 0 &&
			   table[i][altIndex].compare("null") != 0)
			{
				// are these latE7/longE7 or not?
				if(table[i][latIndex].find_first_of('.') != string::npos &&
				   table[i][lonIndex].find_first_of('.') != string::npos)
				{
					// parse directly into lat and lon
					line -> lat = atof(table[i][latIndex].c_str());
					line -> lon = atof(table[i][lonIndex].c_str());
					line -> alt = atoi(table[i][altIndex].c_str());
					line -> hasLocationData = true;
				}
				else
				{
					// parse raw GPS location data and make sure it's valid
					latE7 = atoi(table[i][latIndex].c_str());
					lonE7 = atoi(table[i][lonIndex].c_str());
					alt = atoi(table[i][altIndex].c_str());
					if(latE7 != INT32_MAX && lonE7 != INT32_MAX && alt != UINT16_MAX)
					{
						// compute GPS coords
						line -> lat = (double)latE7 / 1e7;
						line -> lon = (double)lonE7 / 1e7;
						line -> alt = alt;
						line -> hasLocationData = true;
					}
				}
			}

			// by default we're in unknown flight stage
			line -> flightStage = FLIGHT_STAGE_PRELAUNCH;

			// add data to the table
			log -> lines.push_back(line);
		}
	}

	// success
	return log;
}

void generateReport(HabSatLog *log)
{
	// record analysis
	int entriesWithGPS;
	int entriesWithTime;
	int gpsBreaks;
	int timeBreaks;
	int recordGaps;

	// peripheral status tracking
	uint8_t oldStatus;
	uint32_t numCameraFailures;
	uint32_t numDiskFailures;
	uint32_t numIMUFailures;
	uint32_t numEnvFailures;

	// habsat conditions
	uint32_t minBatt, maxBatt, battAtLand;
	int32_t minTemp, maxTemp;
	uint32_t longestUpTime;

	// telemetry reporting
	uint32_t maxSeqNumber;
	uint32_t maxImgNumber;

	// navigation analysis
	uint32_t numConsecutiveRateEvents;
	double altDiff;
	double rate;
	double distance;
	double maxDistance;
	double groundTrack;
	int32_t maxAlt;
	FlightStage flightStage;
	chrono::sys_seconds launchTime;
	chrono::sys_seconds burstTime;
	chrono::sys_seconds landTime;
	double timeDiff;
	vector<double> ascentRates;
	vector<double> descentRateBins[NUM_DESCENT_RATE_BINS];
	double avgDescentRates[NUM_DESCENT_RATE_BINS];
	double minDescentRates[NUM_DESCENT_RATE_BINS];
	double maxDescentRates[NUM_DESCENT_RATE_BINS];
	int descentRateBinIndex;
	double avgDescentRate;
	double avgAscentRate;

	// temporary variables
	stringstream ss;
	int i, j;

	// prime variables
	flightStage = FLIGHT_STAGE_PRELAUNCH;
	numConsecutiveRateEvents = 0;
	maxDistance = 0.0;
	groundTrack = 0.0;
	oldStatus = 0x0f;
	numCameraFailures = 0;
	numDiskFailures = 0;
	numIMUFailures = 0;
	numEnvFailures = 0;
	minBatt = UINT32_MAX;
	maxBatt = 0;
	minTemp = INT32_MAX;
	maxTemp = INT32_MIN;
	maxAlt = INT32_MIN;
	longestUpTime = 0;
	maxSeqNumber = 0;
	maxImgNumber = 0;
	gpsBreaks = 0;
	timeBreaks = 0;
	recordGaps = 0;
	entriesWithGPS = 0;
	entriesWithTime = 0;

	// prime descent rate tracking
	for(i = 0; i < NUM_DESCENT_RATE_BINS; ++ i)
	{
		minDescentRates[i] = DBL_MAX;
		maxDescentRates[i] = DBL_MIN;
	}

	// track max altitude and entries with GPS location and time
	for(i = 0; i < (int)log -> lines.size(); ++ i)
	{
		// do we have GPS location or GPS time?
		if(log -> lines[i] -> hasLocationData) ++ entriesWithGPS;
		if(log -> lines[i] -> hasTimeData)     ++ entriesWithTime;

		// only if we have valid location data, of course
		if(log -> lines[i] -> hasLocationData)
		{
			maxAlt = max(maxAlt, log -> lines[i] -> alt);
		}
	}

	// fill in flight stage data
	for(i = 1; i < (int)log -> lines.size(); ++ i)
	{
		// compute battery range
		minBatt = min(minBatt, log -> lines[i] -> batt);
		maxBatt = max(maxBatt, log -> lines[i] -> batt);

		// compute temperature range
		minTemp = min(minTemp, log -> lines[i] -> temp);
		maxTemp = max(maxTemp, log -> lines[i] -> temp);

		// compute longest up time
		longestUpTime = max(longestUpTime, log -> lines[i] -> up);

		// count if we lost any GPS lock after having one
		if(!log -> lines[i] -> hasLocationData && log -> lines[i - 1] -> hasLocationData)
		{
			++ gpsBreaks;
		}

		// count if we lost any time lock after having one
		if(!log -> lines[i] -> hasTimeData && log -> lines[i - 1] -> hasTimeData)
		{
			++ timeBreaks;
		}

		// count if there are any gaps in the data; these are temporal gaps longer than LOG_GAP_THRESHOLD
		if(log -> lines[i] -> hasTimeData && log -> lines[i - 1] -> hasTimeData &&
		   chrono::duration<double>(log -> lines[i] -> dateTime - log -> lines[i - 1] -> dateTime).count() > LOG_GAP_THRESHOLD)
		{
			++ recordGaps;
		}

		// record highest telemetry packet sequence number and image capture number
		maxSeqNumber = max(maxSeqNumber, log -> lines[i] -> seq);
		maxImgNumber = max(maxImgNumber, log -> lines[i] -> images);

		// did we have any peripheral failures?
		if((oldStatus & STATUS_CAM_BIT)  && !(log -> lines[i] -> status & STATUS_CAM_BIT))  ++ numCameraFailures;
		if((oldStatus & STATUS_DISK_BIT) && !(log -> lines[i] -> status & STATUS_DISK_BIT)) ++ numDiskFailures;
		if((oldStatus & STATUS_IMU_BIT)  && !(log -> lines[i] -> status & STATUS_IMU_BIT))  ++ numIMUFailures;
		if((oldStatus & STATUS_ENV_BIT)  && !(log -> lines[i] -> status & STATUS_ENV_BIT))  ++ numEnvFailures;

		// only do this if the flight stage is ascending or descending
		if(flightStage == FLIGHT_STAGE_ASCENT || flightStage == FLIGHT_STAGE_DESCENT)
		{
			// compare this location with all others
			for(j = i + 1; j < (int)log -> lines.size(); ++ j)
			{
				if(log -> lines[i] -> hasLocationData && log -> lines[j] -> hasLocationData)
				{
					distance = haversineDistance(log -> lines[i] -> lat, log -> lines[i] -> lon,
												 log -> lines[j] -> lat, log -> lines[j] -> lon);
					maxDistance = max(distance, maxDistance);
				}
			}

			// also accumulate ground track
			groundTrack += haversineDistance(log -> lines[i - 0] -> lat, log -> lines[i - 0] -> lon,
											 log -> lines[i - 1] -> lat, log -> lines[i - 1] -> lon);
		}

		// make sure we have valid date/time data
		if(log -> lines[i]     -> hasTimeData && log -> lines[i]     -> hasLocationData &&
		   log -> lines[i - 1] -> hasTimeData && log -> lines[i - 1] -> hasLocationData)
		{
			// prelaunch flight stage
			if(flightStage == FLIGHT_STAGE_PRELAUNCH)
			{
				// if we're going up fast enough, consider us in the ascent stage
				altDiff = (double)log -> lines[i] -> alt - (double)log -> lines[i - 1] -> alt;
				timeDiff = chrono::duration<double>(log -> lines[i] -> dateTime - log -> lines[i - 1] -> dateTime).count();
				if(timeDiff > EPSILON && altDiff / timeDiff > ASCENT_RATE_THRESHOLD)
				{
					// we require a fixed number of these to mitigate the effects of altitude noise
					++ numConsecutiveRateEvents;
					if(numConsecutiveRateEvents >= NUM_CONSEC_ASCENTS)
					{
						flightStage = FLIGHT_STAGE_ASCENT;
						launchTime = log -> lines[i] -> dateTime;
						ascentRates.push_back(altDiff / timeDiff);
					}
				}
				else
				{
					// reset consecutive ascent rate event counter
					numConsecutiveRateEvents = 0;
				}
			}
			else if(flightStage == FLIGHT_STAGE_ASCENT)
			{
				// if we were ascending and reached maximum altitude, then now we're descending
				if(log -> lines[i] -> alt >= maxAlt)
				{
					flightStage = FLIGHT_STAGE_DESCENT;
					burstTime = log -> lines[i] -> dateTime;
					numConsecutiveRateEvents = 0;
				}

				// add to ascent rates
				altDiff = (double)log -> lines[i] -> alt - (double)log -> lines[i - 1] -> alt;
				timeDiff = chrono::duration<double>(log -> lines[i] -> dateTime - log -> lines[i - 1] -> dateTime).count();
				if(timeDiff > EPSILON)
				{
					ascentRates.push_back(altDiff / timeDiff);
				}
			}
			else if(flightStage == FLIGHT_STAGE_DESCENT)
			{
				// if we're not changing altitude much, consider us in the landed stage
				altDiff = abs((double)log -> lines[i] -> alt - (double)log -> lines[i - 1] -> alt);
				timeDiff = chrono::duration<double>(log -> lines[i] -> dateTime - log -> lines[i - 1] -> dateTime).count();

				// prevent zero time deltas
				if(timeDiff > EPSILON)
				{
					// compute rate
					rate = altDiff / timeDiff;
					if(rate < LANDED_RATE_THRESHOLD)
					{
						// we require a fixed number of these to mitigate the effects of altitude noise
						++ numConsecutiveRateEvents;
						if(numConsecutiveRateEvents >= NUM_CONSEC_LANDS)
						{
							// land
							flightStage = FLIGHT_STAGE_LANDED;
							landTime = log -> lines[i] -> dateTime;
							battAtLand = log -> lines[i] -> batt;
						}
					}
					else
					{
						// reset consecutive land rate event counter
						numConsecutiveRateEvents = 0;
					}

					// add the descent rate to the appropriate bin, indexed by altitude;
					// note that negative altitude is allowed; we'll just clamp it to zero
					if(flightStage != FLIGHT_STAGE_LANDED)
					{
						descentRateBinIndex = log -> lines[i] -> alt / (1000 * DESCENT_RATE_BIN_SIZE);
						if(descentRateBinIndex < 0)
						{
							descentRateBinIndex = 0;
						}
						if(descentRateBinIndex < NUM_DESCENT_RATE_BINS)
						{
							// insert descent rate into bin as long as it's in range
							descentRateBins[descentRateBinIndex].push_back(rate);

							// also track min and max for this bin
							minDescentRates[descentRateBinIndex] = min(minDescentRates[descentRateBinIndex], rate);
							maxDescentRates[descentRateBinIndex] = max(maxDescentRates[descentRateBinIndex], rate);
						}
					}
				}
			}
		}
	}

	// compute average ascent rate
	avgAscentRate = 0.0;
	for(i = 0; i < (int)ascentRates.size(); ++ i)
	{
		avgAscentRate += ascentRates[i];
	}
	avgAscentRate /= (double)ascentRates.size();

	// compute descent rates for each altitude bin
	for(i = 0; i < NUM_DESCENT_RATE_BINS; ++ i)
	{
		// skip empty bins
		if(descentRateBins[i].size() > 0)
		{
			// compute average inside this bin
			avgDescentRate = 0.0;
			for(j = 0; j < (int)descentRateBins[i].size(); ++ j)
			{
				avgDescentRate += descentRateBins[i][j];
			}
			avgDescentRates[i] = avgDescentRate / descentRateBins[i].size();
		}
	}

	// print intro and some basic stats
	cout << endl << endl;
	cout << "======================================================" << endl;
	cout << "HABSAT LOG ANALYSIS TOOL v" << VERSION_MAJOR << "." << VERSION_MINOR << "." << VERSION_REVISION << endl;
	cout << "======================================================" << endl;

	// skip if the log is empty
	if(log -> lines.size() >= MIN_LOG_SIZE)
	{
		// data quality
		cout << endl << endl;
		cout << "LOG QUALITY" << endl;
		cout << "------------------------------------------------------" << endl;
		cout << "Displays the total number of log entries, including" << endl;
		cout << "with missing GPS location or GPS time data. A lost GPS" << endl;
		cout << "or time lock indicates a transition from having a lock" << endl;
		cout << "to losing one. Ideally this should not happen. Record " << endl;
		cout << "gaps are gaps in entries longer than " << LOG_GAP_THRESHOLD << " seconds and   " << endl;
		cout << "can indicate missed telemetry packets or power losses," << endl;
		cout << "depending on the source of the log." << endl;
		cout << "------------------------------------------------------" << endl;
		cout << "records           : " << commaDelimitNumber(log -> lines.size()) << endl;
		cout << "records with GPS  : " << commaDelimitNumber(entriesWithGPS) << " (" << fixed << setprecision(1) << (entriesWithGPS * 100.0) / log -> lines.size() << "%)" << endl;
		cout << "records with time : " << commaDelimitNumber(entriesWithTime) << " (" << fixed << setprecision(1) << (entriesWithTime * 100.0) / log -> lines.size() << "%)" << endl;
		cout << "GPS locks lost    : " << commaDelimitNumber(gpsBreaks) << endl;
		cout << "time locks lost   : " << commaDelimitNumber(timeBreaks) << endl;
		cout << "record gaps       : " << commaDelimitNumber(recordGaps) << endl;

		// telemetry
		cout << endl << endl;
		cout << "TELEMETRY" << endl;
		cout << "------------------------------------------------------" << endl;
		cout << "Displays the highest telemetry packet sequence number" << endl;
		cout << "and the highest image capture number in the log." << endl;
		cout << "Actual number of transmitted telemetry packets or" << endl;
		cout << "image captures may vary based on the log source," << endl;
		cout << "reboots, or missed packets." << endl;
		cout << "------------------------------------------------------" << endl;
		cout << "max seq number    : " << commaDelimitNumber(maxSeqNumber) << endl;
		cout << "max img number    : " << commaDelimitNumber(maxImgNumber) << endl;

		// basic stats
		cout << endl << endl;
		cout << "HEALTH AND CONDITIONS" << endl;
		cout << "------------------------------------------------------" << endl;
		cout << "Displays minimum and maximum temperature and battery" << endl;
		cout << "voltages, including battery voltage upon landing." << endl;
		cout << "Longest up time refers to the longest uninterrupted   " << endl;
		cout << "period of time that the HabSat was online." << endl;
		cout << "------------------------------------------------------" << endl;
		cout << "min battery       : " << commaDelimitNumber(minBatt) << " mV" << endl;
		cout << "max battery       : " << commaDelimitNumber(maxBatt) << " mV" << endl;
		cout << "battery on land   : ";
		if(flightStage == FLIGHT_STAGE_LANDED) cout << commaDelimitNumber(battAtLand) << " mV" << endl;
		else 								   cout << "[landing not detected]" << endl;
		cout << "min internal temp : " << minTemp << " C" << endl;
		cout << "max internal temp : " << maxTemp << " C" << endl;
		cout << "longest up time   : " << formatSeconds(longestUpTime) << endl;

		// peripheral failures
		cout << endl << endl;
		cout << "PERIPHERAL FAILURES" << endl;
		cout << "------------------------------------------------------" << endl;
		cout << "A peripheral failure is any period of time over which" << endl;
		cout << "a peripheral is reported to be functioning improperly." << endl;
		cout << "Peripherals can be reported to fail more than once if" << endl;
		cout << "they recover and subsequently fail again." << endl;
		cout << "------------------------------------------------------" << endl;
		cout << "camera failures   : " << commaDelimitNumber(numCameraFailures) << endl;
		cout << "disk failures     : " << commaDelimitNumber(numDiskFailures) << endl;
		cout << "IMU failures      : " << commaDelimitNumber(numIMUFailures) << endl;
		cout << "env failures      : " << commaDelimitNumber(numEnvFailures) << endl;

		// flight statistics
		cout << endl << endl;
		cout << "REBOOT EVENTS" << endl;
		cout << "------------------------------------------------------" << endl;
		cout << "Displays reboot or reboot counter reset events." << endl;
		cout << "Ideally, no such events should take place in-flight." << endl;
		cout << "It is part of the normal operating procedure for at" << endl;
		cout << "least one reboot and one reset event to occur during" << endl;
		cout << "prelaunch." << endl;
		cout << "------------------------------------------------------" << endl;
		for(i = 1; i < (int)log -> lines.size(); ++ i)
		{
			if(log -> lines[i] -> boots < log -> lines[i - 1] -> boots)
			{
				if     (log -> lines[i] -> flightStage == FLIGHT_STAGE_PRELAUNCH) cout << "prelaunch ";
				else if(log -> lines[i] -> flightStage == FLIGHT_STAGE_LANDED)    cout << "post-land ";
				else 															  cout << "in-flight ";
				cout << "reset   : line " << setw(4) << commaDelimitNumber(log -> lines[i] -> lineNum) << ", ";
				if(log -> lines[i] -> hasTimeData) cout << format("{:%F %T}", log -> lines[i] -> dateTime) << " UTC";
				else 							   cout << "time unknown";
				cout << endl;
			}
			else if(log -> lines[i] -> boots > log -> lines[i - 1] -> boots)
			{
				if     (log -> lines[i] -> flightStage == FLIGHT_STAGE_PRELAUNCH) cout << "prelaunch ";
				else if(log -> lines[i] -> flightStage == FLIGHT_STAGE_LANDED)    cout << "post-land ";
				else 															  cout << "in-flight ";
				cout << "reboot  : line " << setw(4) << commaDelimitNumber(log -> lines[i] -> lineNum) << ", ";
				if(log -> lines[i] -> hasTimeData) cout << format("{:%F %T}", log -> lines[i] -> dateTime) << " UTC";
				else 							   cout << "time unknown";
				cout << endl;
			}
		}

		// make sure launch was detected
		if(flightStage >= FLIGHT_STAGE_ASCENT)
		{
			// launch and ascent analysis
			cout << endl << endl;
			cout << "LAUNCH AND ASCENT" << endl;
			cout << "------------------------------------------------------" << endl;
			cout << "Launch is detected when ascent rate exceeds " << ASCENT_RATE_THRESHOLD << " m/s" << endl;
			cout << "for " << NUM_CONSEC_ASCENTS << " consecutive entries. Ascent ends when the" << endl;
			cout << "maximum recorded altitude has been reached. Ascent" << endl;
			cout << "rate is the average ascent rate over that period." << endl;
			cout << "------------------------------------------------------" << endl;
			cout << "launch time       : " << format("{:%F %T}", launchTime) << " UTC" << endl;
			cout << "ascent rate       : " << fixed << setprecision(1) << avgAscentRate << " m/s" << endl;
			cout << "ascent duration   : " << formatSeconds(chrono::duration<double>(burstTime - launchTime).count()) << endl;

			// burst and descent analysis
			cout << endl << endl;
			cout << "BURST" << endl;
			cout << "------------------------------------------------------" << endl;
			cout << "Burst is considered to occur when the maximum recorded" << endl;
			cout << "altitude has been reached." << endl;
			cout << "------------------------------------------------------" << endl;
			cout << "burst time        : " << format("{:%F %T}", burstTime) << " UTC" << endl;
			cout << "burst altitude    : " << commaDelimitNumber(maxAlt) << " m" << endl;

			// descent profile
			cout << endl << endl;
			cout << "DESCENT PROFILE" << endl;
			cout << "------------------------------------------------------" << endl;
			cout << "Descent rate is non-linear and is reported in bins" << endl;
			cout << "grouped by altitude. Descent ends when the descent" << endl;
			cout << "rate falls below " << LANDED_RATE_THRESHOLD << " m/s for " << NUM_CONSEC_LANDS << " consecutive entries." << endl;
			cout << "Note that GPS altitude can be noisy and that outliers" << endl;
			cout << "are not removed." << endl;

			// print out descent profile table
			cout << "------------------------------------------------------" << endl;
			cout << setw(16) << left << "altitude (km) " << setw(13) << "avg (m/s)" << setw(13) << "min (m/s)" << setw(13) << "max (m/s)" << endl;
			cout << "------------------------------------------------------" << endl;
			for(i = NUM_DESCENT_RATE_BINS - 1; i >= 0; -- i)
			{
				// don't print empty bins
				if(descentRateBins[i].size() > 0)
				{
					ss.str("");
					ss << left << setw(2) << (i * DESCENT_RATE_BIN_SIZE) << " - " << left << setw(2) << ((i + 1) * DESCENT_RATE_BIN_SIZE);

					// average
					cout << left << setw(16) << fixed << ss.str() << left;
					cout << setw(13) << fixed << setprecision(1) << avgDescentRates[i];
					cout << setw(13) << fixed << setprecision(1) << minDescentRates[i];
					cout << setw(13) << fixed << setprecision(1) << maxDescentRates[i];
					cout << endl;
				}
			}

			// make sure landing was detected
			if(flightStage >= FLIGHT_STAGE_LANDED)
			{
				// flight statistics
				cout << endl << endl;
				cout << "FLIGHT AND LANDING" << endl;
				cout << "------------------------------------------------------" << endl;
				cout << "Landing is detected when descent rate falls below " << LANDED_RATE_THRESHOLD << endl;
				cout << "m/s for " << NUM_CONSEC_LANDS << " consecutive entries. Flight distance is" << endl;
				cout << "the straight-line distance between the launch and" << endl;
				cout << "landing locations. Ground track is the total length of" << endl;
				cout << "the path travelled from launch to landing." << endl;
				cout << "------------------------------------------------------" << endl;
				cout << "descent duration  : " << formatSeconds(chrono::duration<double>(landTime - burstTime).count()) << endl;
				cout << "landing time      : " << format("{:%F %T}", landTime) << " UTC" << endl;
				cout << "flight distance   : " << commaDelimitNumber(maxDistance) << " m" << endl;
				cout << "ground track      : " << commaDelimitNumber(groundTrack) << " m" << endl;
				cout << "flight duration   : " << formatSeconds(chrono::duration<double>(landTime - launchTime).count()) << endl;
			}
			else
			{
				// no landing event detected
				cout << endl << endl;
				cout << "[!] LANDING WAS NOT DETECTED [!]" << endl;
				cout << "------------------------------------------------------" << endl;
				cout << "Landing is detected when descent rate falls below " << LANDED_RATE_THRESHOLD << endl;
				cout << "m/s for " << NUM_CONSEC_LANDS << " consecutive entries, and this was not" << endl;
				cout << "detected. Report is incomplete." << endl;
				cout << "------------------------------------------------------" << endl;
			}
		}
		else
		{
			// no launch event detected
			cout << endl << endl;
			cout << "[!] LAUNCH WAS NOT DETECTED [!]" << endl;
			cout << "------------------------------------------------------" << endl;
			cout << "Launch is detected when ascent rate exceeds " << ASCENT_RATE_THRESHOLD << " m/s" << endl;
			cout << "for " << NUM_CONSEC_ASCENTS << " consecutive entries, and this was not detected." << endl;
			cout << "detected. Report is incomplete." << endl;
			cout << "------------------------------------------------------" << endl;
		}
	}
	else
	{
		// empty or near-empty log
		cout << endl << endl;
		cout << "LOG MUST CONTAIN AT LEAST " << MIN_LOG_SIZE << " RECORDS." << endl;
	}

	// outro
	cout << endl << endl;
	cout << "======================================================" << endl;
	cout << "END OF REPORT" << endl;
	cout << "======================================================" << endl;
}

void freeLog(HabSatLog *log)
{
	vector<HabSatLogLine*>::iterator i;

	for(i = log -> lines.begin(); i != log -> lines.end(); ++ i)
	{
		delete *i;
	}
	delete log;
}

int getColumnIndex(const vector<vector<string>> &table, const string &header1, const string &header2)
{
	string col;
	int result = -1;
	int i;

	// make sure enough data is present
	if(table.size() > 0)
	{
		i = 0;
		while(result == -1 && i < (int)table[0].size())
		{
			col = trim(table[0][i]);
			if((!header1.empty() && col.compare(header1) == 0) ||
			   (!header2.empty() && col.compare(header2) == 0))
			{
				result = i;
			}
			++ i;
		}
	}

	// exit if we can't find some column
	if(result == -1)
	{
		cout << "[ERR] neither column '" << header1 << "' nor '" << header1 << "' was not found." << endl;
		exit(1);
	}

	// done
	return result;
}

double radians(double deg)
{
	return deg * (PI / 180.0);
}

double haversineDistance(double lat1, double lon1, double lat2, double lon2)
{
	// compute deltas
	double latDiff = radians(lat2 - lat1);
	double lonDiff = radians(lon2 - lon1);

	// Haversine distance
	double a = sin(latDiff / 2.0) * sin(latDiff / 2.0) +
			   cos(radians(lat1)) * cos(radians(lat2)) *
			   sin(lonDiff / 2.0) * sin(lonDiff / 2.0);
	double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
	return EARTH_RADIUS * c;
}

string formatSeconds(int seconds)
{
	const int SECONDS_PER_DAY = 86400;
	const int SECONDS_PER_HOUR = 3600;
	const int SECONDS_PER_MINUTE = 60;

	stringstream s;
	int days, hours, minutes;

	days = seconds / SECONDS_PER_DAY;
	seconds %= SECONDS_PER_DAY;

	hours = seconds / SECONDS_PER_HOUR;
	seconds %= SECONDS_PER_HOUR;

	minutes = seconds / SECONDS_PER_MINUTE;
	seconds %= SECONDS_PER_MINUTE;

	if(days > 0)
	{
		s << setw(2) << setfill('0') << days << "d " << setw(2) << setfill('0') << hours << "h " << setw(2) << setfill('0') << minutes << "m " << setw(2) << setfill('0') << seconds << "s";
	}
	else if(hours > 0)
	{
		s << setw(2) << setfill('0') << hours << "h " << setw(2) << setfill('0') << minutes << "m " << setw(2) << setfill('0') << seconds << "s";
	}
	else if(minutes > 0)
	{
		s << setw(2) << setfill('0') << minutes << "m " << setw(2) << setfill('0') << seconds << "s";
	}
	else
	{
		s << seconds << " s";
	}

	return s.str();
}

string commaDelimitNumber(int number)
{
	stringstream ss;
	string str;
	string result = "";
	int i;

	ss << number;
	str = ss.str();
	for(i = str.size() - 1; i >= 0; -- i)
	{
		result = str[i] + result;
		if(((int)str.size() - i) % 3 == 0 && i > 0) result = "," + result;
	}

	return result;
}

string trim(string str)
{
    const int ASCII_WHITESPACE = 33;

    string result = "";
    int length = str.size();
    int start = 0;
    int end = length - 1;

    while(start < length && str[start] < ASCII_WHITESPACE)
    {
        start ++;
	}

    while(end > 0 && str[end] < ASCII_WHITESPACE)
    {
        end --;
	}

    if(start <= end)
    {
        result = str.substr(start, end - start + 1);
	}

    return result;
}
