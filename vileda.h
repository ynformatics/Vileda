#define FORWARD 0
#define BACKWARD 1

const double mmToTicks = 7.6923;
const double wheelbase = 250; // in mm
unsigned int batteryCapacity = 2000; // mAh

// About 1/4 of the battery voltage is fed to pin battVoltage
// Adjust these constants as needed (plot a few pairs and fit a straight-line through them)
// voltage = analogRead(battVoltage) * batteryVoltageGradient + batteryVoltageOffset;
const double batteryVoltageGradient = 0.0206;
const double batteryVoltageOffset = -0.5964;

// Bias equation to convert required velocity to motor PWM power
//motorPower = velocityBiasGradient * velocity + velocityBiasOffset;
const double velocityBiasGradient = 5.8;
const double velocityBiasOffset = 12;

const int leftBumper =    2; 
const int IN4 =           3; 
const int speaker =       4; 
const int IN3 =           5; 
const int rightBumper =   6; 
const int leftWheelEnc =  7; 
const int rightWheelEnc = 8; 
const int IN2 =           9; 
const int IN1 =           10; 
const int wheelsUp =      11; 
const int switchL =       12; 
const int switchLedL =    13; // pin 13 must be an output (because of LED on arduino) and it gets toggled when bootloading

const int battVoltage =   A0;
const int mainBrush =     A1; 
const int greenLed =      A2; 
const int redLed =        A3; 
const int switchLedM =    A4; 
const int switchLedS =    A5; 
const int switchM =       A6; // A6 and A7 are analog-input only!
const int switchS =       A7; // 

volatile  int leftEncoderCount = 0;
volatile  int rightEncoderCount = 0;
bool leftEncoderForwards = true;
bool rightEncoderForwards = true;

boolean commandComplete = false;  
byte bytes[200];
int bytePos = 0;
int wanted = 0;

int direction = FORWARD;
bool clockwise = true;

int leftEncoder = 0;
int rightEncoder = 0;
double prevLeftEncoder;
double prevRightEncoder;

double leftSetpoint;
double rightSetpoint;

double syncError;
double syncPower;
double syncSetpoint = 0;
double syncFactor = 1;
double requestPower;

double velIP;
double velSP;
double velBias;

double rightPower;
double velRightIP;
double velRightSP;
double velRightBias;
int dirRight;

double leftPower;
double velLeftIP;
double velLeftSP;
double velLeftBias;
int dirLeft;

double distance = 0;
double angle = 0;
double lastDistance = 0;

PID sync(&syncError, &syncPower, &syncSetpoint, 1.0, 0.1, 0, P_ON_E, DIRECT);
PID velPID(&velIP, &requestPower, &velSP, &velBias, 2.0, 2.0, 0, P_ON_E, DIRECT);
PID velLeftPID(&velLeftIP, &leftPower, &velLeftSP, &velLeftBias, 2.0, 2.0, 0, P_ON_E, DIRECT);  
PID velRightPID(&velRightIP, &rightPower, &velRightSP, &velRightBias, 2.0, 2.0, 0, P_ON_E, DIRECT);   

int sampleTime = 15; //  ms
unsigned long lastTime;
bool driving = false;
bool spin = false;

const double ticksToMm = 1.0 / mmToTicks;
const double ticksToDegrees = ticksToMm * 180.0 / (2 * wheelbase * PI);

double voltage = 0;
unsigned long lastBatteryReadTime = 0;


enum DriveMode {
    DM_DRIVE_DIRECT = 0,
    DM_DRIVE = 1,
    DM_DRIVE_PWM = 2
};

enum OIMode {
    OFF = 0, 
    PASSIVE = 1,
    SAFE = 2,
    FULL = 3
};
enum SensorPacketID {
    ID_GROUP_0 = 0,
    ID_GROUP_1 = 1,
    ID_GROUP_2 = 2,
    ID_GROUP_3 = 3,
    ID_GROUP_4 = 4,
    ID_GROUP_5 = 5,
    ID_GROUP_6 = 6,
    ID_GROUP_100 = 100,
    ID_GROUP_101 = 101,
    ID_GROUP_106 = 106,
    ID_GROUP_107 = 107,
    ID_BUMP_WHEELDROP = 7,
    ID_WALL = 8,
    ID_CLIFF_LEFT = 9,
    ID_CLIFF_FRONT_LEFT = 10,
    ID_CLIFF_FRONT_RIGHT = 11,
    ID_CLIFF_RIGHT = 12,
    ID_VIRTUAL_WALL = 13,
    ID_OVERCURRENTS = 14,
    ID_DIRT_DETECT_LEFT = 15,
    ID_DIRT_DETECT_RIGHT = 16,
    ID_IR_OMNI = 17,
    ID_IR_LEFT = 52,
    ID_IR_RIGHT = 53,
    ID_BUTTONS = 18,
    ID_DISTANCE = 19,
    ID_ANGLE = 20,
    ID_CHARGE_STATE = 21,
    ID_VOLTAGE = 22,
    ID_CURRENT = 23,
    ID_TEMP = 24,
    ID_CHARGE = 25,
    ID_CAPACITY = 26,
    ID_WALL_SIGNAL = 27,
    ID_CLIFF_LEFT_SIGNAL = 28,
    ID_CLIFF_FRONT_LEFT_SIGNAL = 29,
    ID_CLIFF_FRONT_RIGHT_SIGNAL = 30,
    ID_CLIFF_RIGHT_SIGNAL = 31,
    ID_CARGO_BAY_DIGITAL_INPUTS = 32,
    ID_CARGO_BAY_ANALOG_SIGNAL = 33,
    ID_CHARGE_SOURCE = 34,
    ID_OI_MODE = 35,
    ID_SONG_NUM = 36,
    ID_PLAYING = 37,
    ID_NUM_STREAM_PACKETS = 38,
    ID_VEL = 39,
    ID_RADIUS = 40,
    ID_RIGHT_VEL = 41,
    ID_LEFT_VEL = 42,
    ID_LEFT_ENC = 43,
    ID_RIGHT_ENC = 44,
    ID_LIGHT = 45,
    ID_LIGHT_LEFT = 46,
    ID_LIGHT_FRONT_LEFT = 47,
    ID_LIGHT_CENTER_LEFT = 48,
    ID_LIGHT_CENTER_RIGHT = 49,
    ID_LIGHT_FRONT_RIGHT = 50,
    ID_LIGHT_RIGHT = 51,
    ID_LEFT_MOTOR_CURRENT = 54,
    ID_RIGHT_MOTOR_CURRENT = 55,
    ID_MAIN_BRUSH_CURRENT = 56,
    ID_SIDE_BRUSH_CURRENT = 57,
    ID_STASIS = 58,
    ID_NUM = 52
};

enum OpCodes {
    OC_RESET = 7,
    OC_DRIVE = 137,
    OC_DRIVE_DIRECT = 145,
    OC_DRIVE_PWM = 146,
    OC_START = 128,
    OC_SAFE = 131,
    OC_FULL = 132,
    OC_STOP = 173,
    OC_CONTROL = 130,
    OC_LEDS = 139,
    OC_SENSORS = 142,
    OC_STREAM = 148,
    OC_TOGGLE_STREAM = 150,
    OC_BAUD = 129,
    OC_DATE = 168,
};

bool accum = false;

byte streamPacketIds[100];
int streamPacketIdsLength = 0;
bool streaming = false;
byte streamPacket[200];
int streamPacketLength = 0;

int requestedVelocity;
int requestedRadius;

int requestedVelRight;
int requestedVelLeft;

enum OIMode mode = OFF;
enum DriveMode driveMode = DM_DRIVE;

int pwmRight = 0;
int pwmLeft = 0;


void MonitorBattery();

double GetBatteryVoltage();

unsigned int GetBatteryCharge(unsigned int mV);

void ExecCommand(byte* command);

void Sensors(int id);

void DoStream(bool withChecksum);

void DrivePWM(int pwmR, int pwmL);

void DriveDirect(int velRight, int velLeft);

void Drive(int velocity, int radius);

void SampleSync();

void allStop();

void LeftMotorPower(int power, int dir);

void RightMotorPower(int power, int dir);

void setupInterupts();

void beep(int count);
