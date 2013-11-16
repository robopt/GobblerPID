///////////////////////
// GobblerPID Header //
// Edward Mead       //
///////////////////////
// asm(nop) = 41.667 ns

// wall PID values
double Kp = 10; //15
double KpInner = 15; //15
int lastError = 0;
   
int sum, senSum;
int error, deltaError;
int lastFront = 0;

/////////////
// Braking //
/////////////
int brakeThresh = 2;
const brakeThreshInner = 2;
int brakeCount = 0;
char brakeEnabled = 1;

//base brake
int BASEBRAKE = 220;
int BASEBRAKEINNER = 205;

//line PID values
double KpLine = 8;
double KdLine = 0;
double KiLine = 0;

///////////////////////
// Motor constraints //
///////////////////////
int BASE;//95
int BASEOUTER = 145;
int BASEOUTERTWO = 145;
int BASEINNER = 140;
int LINEBASE = 90; 


//base backup 
int BASERBKUP = 130; 
int BASELBKUP = 80;

//base turns
int BASETURN = 165;
int BASETURNINNER = 205;

//limitations
const int MAX = 255;
const int MIN = 40; 
const int LINEMAX = 140;

//motor variables 
int routput,loutput;

//wall follow defaults
int gWallOuter = 88, gWallInner = 34, gWallCurrent;
int wallTurnOuter = 70, wallTurnInner = 34, wallTurnCurrent; //54
int wallBrakeOuter = 34, wallBrakeInner = 25, wallBrakeCurrent;
int wallTurnInnerExit = 20;
int wallDumpApproach = 3;
int gWallClose = 110;
int gWallOuterCorner = 82;
char closeToWall = 0x00;
char turnEnabled = 0x00;

//nativation variables
int lastLine = 0;
int lastSide = 0;
const int lastThresh = 125;
int lineValueThresh = 110;
const int lineOptimum = 36;
char lineCount = 0x00;
char isSetup = 0x00;


//sensor variables
const int sensorAvgCount = 40;
const int sensorCount = 4;
const int sensorStart = 4;
int sensorIndex = 4;
int sensorAvgIndex = 0;
int s0,s1,s2,s3,s4,s5,s6,s7;
int s4total,s5total,s6total,s7total;
int s4avg,s5avg,s6avg,s7avg;
int s4a[255];
int s5a[255];
int s6a[255];
int s7a[255];

//function defs
void portInit(void);
void dumpApproach(void);
void averageSensors(void);
void forward(char speed);
void forwardP();
void forwardLine();
void right(char lspeed);
void left(char rspeed);
void leftManual(char lspeed, char rspeed);
void back(char speed);
void backSplit(char lspeed, char rspeed);
void backLeft(char rspeed, char lspeed);
void stop();
void setup();
