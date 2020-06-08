#include <SPI.h>               // Serial protocol
#include <Adafruit_GFX.h>      // Core graphics library
#include "Adafruit_HX8357.h"   // Display
#include "TouchScreen.h"       // Touchscreen
#include <SD.h>                // SD card read/write support
#include <stdlib.h>
#include "HX711.h"             // Load cell support

//----------------------------- PINS definitions -------------------------------

#define YP A3             // analog pin for touchscreen Y
#define XM A2             // analog pin for touchscreen X
#define YM 12             // digital pin for touchscreen Y
#define XP 11             // digital pin for touchscreen X

#define TFT_RST -1        // reset pin (goes unused)
#define TFT_DC 9          // data/command pin for touchscreen
#define TFT_CS 10         // chip select  pin for touchscreen SPI

#define CLK A5            // clock pin for loadcell 
#define DOUT 13           // data pin for loadcell

#define ls_top A0         // top limit switch pin
#define estop_button A1   // estop pin

#define SD_CS 4            //SD card chip select pin   
             
//-----------------------Motor speed variables---------------------------------

static long LOWSPD = 3;
static long MEDIUMSPD = 10;
static long HIGHSPD = 50;
long jog_speed = 0;

//----------------------Position variables-----------------------------

long spring_top, spring_bottom, top_limit, bottom_limit;

//-----------------Force and Displacement Calibration Factors----------

static float steps_per_mm = 245.0;    //steps per mm
static float calibration_factor = 19770.0; //determined from initial calibration

//---------------------------- SD CARD ------------------------------

File dataFile;
String dataString;

//---------------------------- LOAD CELL ----------------------------

HX711 scale;
static float ZERO = 0.1;     //variable used for scale taring 
float f;

//----------- DMM Motor Drive Serial Communication ------------------

#define Set_Origin              0x00
#define Go_Absolute_Pos         0x01
#define Make_LinearLine         0x02
#define Go_Relative_Pos         0x03
#define Is_AbsPos32             0x1b
#define General_Read            0x0e
#define Is_TrqCurrent           0x1E
#define Read_MainGain           0x18
#define Read_Drive_Status       0x09
#define Set_MainGain            0x10
#define Set_SpeedGain           0x11
#define Set_IntGain             0x12
#define Set_HighSpeed           0x14
#define Set_HighAccel           0x15
#define Set_Pos_OnRange         0x16
#define Is_MainGain             0x10
#define Is_SpeedGain            0x11
#define Is_IntGain              0x12
#define Is_TrqCons              0x13
#define Is_HighSpeed            0x14
#define Is_HighAccel            0x15
#define Is_Driver_ID            0x16
#define Is_Pos_OnRange          0x17
#define Is_Status               0x19
#define Is_Config               0x1a

char InputBuffer[256];                          //Input buffer from RS232,
char OutputBuffer[256];                         //Output buffer to RS232,
unsigned char InBfTopPointer, InBfBtmPointer;   //input buffer pointers
unsigned char OutBfTopPointer, OutBfBtmPointer; //output buffer pointers
unsigned char Read_Package_Buffer[8], Read_Num, Read_Package_Length, Global_Func;
unsigned char MotorPosition32Ready_Flag, MotorTorqueCurrentReady_Flag, MainGainRead_Flag, Drive_Status_Read_Flag;
unsigned char Driver_MainGain, Driver_SpeedGain, Driver_IntGain, Driver_TrqCons, Driver_HighSpeed, Driver_HighAccel, Driver_ReadID, Driver_Status, Driver_Config, Driver_OnRange;
long Motor_Pos32, MotorTorqueCurrent;

void move_rel32(char ID, long pos);
void ReadMotorTorqueCurrent(char ID);
void ReadMotorPosition32(char ID);
void move_abs32(char MotorID, long Pos32);
void Turn_const_speed(char ID, long spd);
void ReadPackage(void);
void Get_Function(void);
void check_for_finished_movement(char ID);
long Cal_SignValue(unsigned char One_Package[8]);
long Cal_Value(unsigned char One_Package[8]);
void Send_Package(char ID , long Displacement);
void Make_CRC_Send(unsigned char Plength, unsigned char B[8]);
void move_up(void);
void move_down(void);
void stopmotor(void);
void movelinear(long Pos32, long feedrate);
boolean done_with_movement;

//----------------------------- TOUCHSCREEN ------------------------------------------------------------

// This is calibration data for the raw touch data to the screen coordinates
#define TS_MINX 330
#define TS_MINY 215
#define TS_MAXX 800
#define TS_MAXY 865
#define MINPRESSURE 500
#define MAXPRESSURE 1000

Adafruit_HX8357 tft = Adafruit_HX8357(TFT_CS, TFT_DC, TFT_RST);   //screen object
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 297.9);              //touchscreen object

// touchscreen color definitions

#define HX8357_BLACK       0x0000  ///<   0,   0,   0
#define HX8357_NAVY        0x000F  ///<   0,   0, 123
#define HX8357_DARKGREEN   0x03E0  ///<   0, 125,   0
#define HX8357_DARKCYAN    0x03EF  ///<   0, 125, 123
#define HX8357_MAROON      0x7800  ///< 123,   0,   0
#define HX8357_PURPLE      0x780F  ///< 123,   0, 123
#define HX8357_OLIVE       0x7BE0  ///< 123, 125,   0
#define HX8357_LIGHTGREY   0xC618  ///< 198, 195, 198
#define HX8357_DARKGREY    0x7BEF  ///< 123, 125, 123
#define HX8357_BLUE        0x001F  ///<   0,   0, 255
#define HX8357_GREEN       0x07E0  ///<   0, 255,   0
#define HX8357_CYAN        0x07FF  ///<   0, 255, 255
#define HX8357_RED         0xF800  ///< 255,   0,   0
#define HX8357_MAGENTA     0xF81F  ///< 255,   0, 255
#define HX8357_YELLOW      0xFFE0  ///< 255, 255,   0
#define HX8357_WHITE       0xFFFF  ///< 255, 255, 255
#define HX8357_ORANGE      0xFD20  ///< 255, 165,   0
#define HX8357_GREENYELLOW 0xAFE5  ///< 173, 255,  41
#define HX8357_PINK        0xFC18  ///< 255, 130, 198

// UI Buttondetails
#define BUTTON_X 40
#define BUTTON_Y 40
#define BUTTON_W 80
#define BUTTON_H 80
#define BUTTON_TEXTSIZE 2
#define DISPLAY_XOFFSET 0
#define DISPLAY_TEXTOFFSET 90
#define DISPLAY_YOFFSET 20
#define TEXT_SIZE 2



enum ButtonName {
  BTN1,  BTN2,  BTN3,  BTN4,  BTN5,  BTN6,  BTN7,  BTN8,
  BTN9,  BTN10
};


//---------------------------Main Menu Button-------------------------------------------
#define MainMenu_BTN_CNT 10
Adafruit_GFX_Button MainMenuButtons[MainMenu_BTN_CNT];

char MainMenuLabels[MainMenu_BTN_CNT][10] = {"JOG", "TEST", " ", " ", " "
                                             , "FILES", " ", " ", " ", " "
                                            };
uint16_t MainMenuColors[MainMenu_BTN_CNT] = {HX8357_BLUE, HX8357_BLUE,
                                             HX8357_BLACK, HX8357_BLACK,
                                             HX8357_BLACK,
                                             HX8357_PURPLE, HX8357_BLACK,
                                             HX8357_BLACK, HX8357_BLACK,
                                             HX8357_BLACK,
                                            };

                                            
//-------------------------Jog Menu Buttons Labels and Colors-----------------------------
#define JogMenu_BTN_CNT 10
Adafruit_GFX_Button JogMenuButtons[JogMenu_BTN_CNT];
char JogMenuLabels[JogMenu_BTN_CNT][10] = {"LOW", "MED", "HIGH",
                                           " ", " ", "UP", "STOP", "DOWN",
                                           "AUTO", "BACK"
                                          };
uint16_t JogMenuColors[JogMenu_BTN_CNT] = {HX8357_BLUE, HX8357_BLUE,
                                           HX8357_BLUE,
                                           HX8357_BLACK, HX8357_BLACK,
                                           HX8357_BLUE,
                                           HX8357_RED, HX8357_BLUE,
                                           HX8357_PURPLE,
                                           HX8357_DARKGREY
                                          };


//--------------------------Test Menu Buttons Labels and Colors----------------------------
#define TestMenu_BTN_CNT 10
Adafruit_GFX_Button TestMenuButtons[JogMenu_BTN_CNT];
char TestMenuLabels[TestMenu_BTN_CNT][10] = {"SET Uy", "SET Fy", "SET N",
                                             " ", " ", " ", " ", " ", " ",
                                             "BACK"
                                            };
uint16_t TestMenuColors[TestMenu_BTN_CNT] = {HX8357_BLUE, HX8357_BLUE,
                                             HX8357_BLUE, HX8357_BLACK,
                                             HX8357_BLACK, HX8357_BLACK,
                                             HX8357_BLACK, HX8357_BLACK,
                                             HX8357_BLACK,
                                             HX8357_DARKGREY
                                            };

//-------------------------Modify Values Buttons and Colors--------------------------------
#define Mod_Value_BTN_CNT 3
Adafruit_GFX_Button Mod_Value_Buttons[Mod_Value_BTN_CNT];
char Mod_Value_Labels[Mod_Value_BTN_CNT][10] = {"+", "-", "SAVE"};
uint16_t Mod_Value_Colors[Mod_Value_BTN_CNT] = {HX8357_BLACK, HX8357_BLACK, HX8357_GREEN};


//-------------------------Max and Min values For Test Settings------------------------------
#define MAX_FORCE 50
#define MAX_DISP 10   //10 cm? Maybe? Check and measure later
#define MAX_CYCLES 7  // 10^0 =1, up to 10^7
#define MIN_FORCE 1  //1 N minimum
#define MIN_DISP 1   //1 cm minimum
#define MIN_CYCLES 0  //10^0 minimum cycles (static)

//--------------------------Presets for force,displacement,and cycle settings---------------
int FORCE = 5;
int DISP = 2;
int CYCLES = 0;

//--------------------------File Menu Variables-------------------------------------------
int num_files;
int selected_file_num;





//-------------------------Initialize buttons function------------------------------------
bool initializeButtons(
  Adafruit_GFX_Button menuButtons[],
  uint16_t menuColors[],
  char menuLabels[][10],
  int menuButtonCount) {
  tft.fillScreen(HX8357_BLACK);

  for (uint8_t row = 0; row < 5; row++)
  {
    if (isblank(menuLabels[row][0]) == 1) {   //skips button spaces that are blank
      continue;
    }
    menuButtons[row].initButton(& tft,
                                BUTTON_X,
                                BUTTON_Y + row * (BUTTON_H),
                                BUTTON_W,
                                BUTTON_H,
                                HX8357_BLACK,
                                menuColors[row],
                                HX8357_WHITE,
                                menuLabels[row], BUTTON_TEXTSIZE);
    menuButtons[row].drawButton();
  }
  for (uint8_t row = 5; row < menuButtonCount; row++)
  {
    if (isblank(menuLabels[row][0]) == 1) {   //skips button spaces that are blank
      continue;
    }
    menuButtons[row].initButton(& tft,
                                (tft.width() - BUTTON_X),
                                BUTTON_Y + (row - 5) * (BUTTON_H),
                                BUTTON_W,
                                BUTTON_H,
                                HX8357_BLACK,
                                menuColors[row],
                                HX8357_WHITE,
                                menuLabels[row], BUTTON_TEXTSIZE);
    menuButtons[row].drawButton();
  }
  return true;
}


//--------------------------Initialize modify values buttons function-----------------------
bool initialize_Mod_Value_Buttons(
  Adafruit_GFX_Button menuButtons[],
  uint16_t menuColors[],
  char menuLabels[][10],
  int menuButtonCount) {
  menuButtons[0].initButton(& tft, (tft.width() - 2 * BUTTON_W + BUTTON_X), BUTTON_Y, BUTTON_W, BUTTON_H,
                            HX8357_BLACK, menuColors[0], HX8357_WHITE,
                            menuLabels[0], 5);
  menuButtons[0].drawButton();
  menuButtons[1].initButton(& tft, (tft.width() - 2 * BUTTON_W + BUTTON_X), (BUTTON_Y + BUTTON_H),
                            BUTTON_W, BUTTON_H,
                            HX8357_BLACK, menuColors[1], HX8357_WHITE,
                            menuLabels[1], 5);
  menuButtons[1].drawButton();
  menuButtons[2].initButton(& tft, (tft.width() - 2 * BUTTON_W + BUTTON_X), BUTTON_Y + (2 * BUTTON_H), BUTTON_W, BUTTON_H,
                            HX8357_BLACK, menuColors[2], HX8357_WHITE,
                            menuLabels[2], BUTTON_TEXTSIZE);
  menuButtons[2].drawButton();
  return true;
}


//-------------------------Bmp drawing functions for EWU logo--------------------------------
#define BUFFPIXEL 40

void bmpDraw(char *filename, uint8_t x, uint16_t y) {

  File     bmpFile;
  int      bmpWidth, bmpHeight;   // W+H in pixels
  uint8_t  bmpDepth;              // Bit depth (currently must be 24)
  uint32_t bmpImageoffset;        // Start of image data in file
  uint32_t rowSize;               // Not always = bmpWidth; may have padding
  uint8_t  sdbuffer[3 * BUFFPIXEL]; // pixel buffer (R+G+B per pixel)
  uint8_t  buffidx = sizeof(sdbuffer); // Current position in sdbuffer
  boolean  goodBmp = false;       // Set to true on valid header parse
  boolean  flip    = true;        // BMP is stored bottom-to-top
  int      w, h, row, col;
  uint8_t  r, g, b;
  uint32_t pos = 0, startTime = millis();

  if ((x >= tft.width()) || (y >= tft.height())) return;

  // Open requested file on SD card
  if ((bmpFile = SD.open(filename)) == NULL) {
    Serial.print(F("File not found"));
    return;
  }

  // Parse BMP header
  if (read16(bmpFile) == 0x4D42) { // BMP signature
    Serial.print(F("File size: ")); Serial.println(read32(bmpFile));
    (void)read32(bmpFile); // Read & ignore creator bytes
    bmpImageoffset = read32(bmpFile); // Start of image data
    Serial.print(F("Image Offset: ")); Serial.println(bmpImageoffset, DEC);
    // Read DIB header
    Serial.print(F("Header size: ")); Serial.println(read32(bmpFile));
    bmpWidth  = read32(bmpFile);
    bmpHeight = read32(bmpFile);
    if (read16(bmpFile) == 1) { // # planes -- must be '1'
      bmpDepth = read16(bmpFile); // bits per pixel
      Serial.print(F("Bit Depth: ")); Serial.println(bmpDepth);
      if ((bmpDepth == 24) && (read32(bmpFile) == 0)) { // 0 = uncompressed

        goodBmp = true; // Supported BMP format -- proceed!
        Serial.print(F("Image size: "));
        Serial.print(bmpWidth);
        Serial.print('x');
        Serial.println(bmpHeight);

        // BMP rows are padded (if needed) to 4-byte boundary
        rowSize = (bmpWidth * 3 + 3) & ~3;

        // If bmpHeight is negative, image is in top-down order.
        // This is not canon but has been observed in the wild.
        if (bmpHeight < 0) {
          bmpHeight = -bmpHeight;
          flip      = false;
        }

        // Crop area to be loaded
        w = bmpWidth;
        h = bmpHeight;
        if ((x + w - 1) >= tft.width())  w = tft.width()  - x;
        if ((y + h - 1) >= tft.height()) h = tft.height() - y;

        // Set TFT address window to clipped image bounds
        tft.startWrite(); // Start TFT transaction
        tft.setAddrWindow(x, y, w, h);

        for (row = 0; row < h; row++) { // For each scanline...

          // Seek to start of scan line.  It might seem labor-
          // intensive to be doing this on every line, but this
          // method covers a lot of gritty details like cropping
          // and scanline padding.  Also, the seek only takes
          // place if the file position actually needs to change
          // (avoids a lot of cluster math in SD library).
          if (flip) // Bitmap is stored bottom-to-top order (normal BMP)
            pos = bmpImageoffset + (bmpHeight - 1 - row) * rowSize;
          else     // Bitmap is stored top-to-bottom
            pos = bmpImageoffset + row * rowSize;
          if (bmpFile.position() != pos) { // Need seek?
            tft.endWrite(); // End TFT transaction
            bmpFile.seek(pos);
            buffidx = sizeof(sdbuffer); // Force buffer reload
            tft.startWrite(); // Start new TFT transaction
          }

          for (col = 0; col < w; col++) { // For each pixel...
            // Time to read more pixel data?
            if (buffidx >= sizeof(sdbuffer)) { // Indeed
              tft.endWrite(); // End TFT transaction
              bmpFile.read(sdbuffer, sizeof(sdbuffer));
              buffidx = 0; // Set index to beginning
              tft.startWrite(); // Start new TFT transaction
            }

            // Convert pixel from BMP to TFT format, push to display
            b = sdbuffer[buffidx++];
            g = sdbuffer[buffidx++];
            r = sdbuffer[buffidx++];
            tft.pushColor(tft.color565(r, g, b));
          } // end pixel
        } // end scanline
        tft.endWrite(); // End last TFT transaction
        Serial.print(F("Loaded in "));
        Serial.print(millis() - startTime);
        Serial.println(" ms");
      } // end goodBmp
    }
  }

  bmpFile.close();
  if (!goodBmp) Serial.println(F("BMP format not recognized."));
}

uint16_t read16(File &f) {
  uint16_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read(); // MSB
  return result;
}

uint32_t read32(File &f) {
  uint32_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read();
  ((uint8_t *)&result)[2] = f.read();
  ((uint8_t *)&result)[3] = f.read(); // MSB
  return result;
}

//------------------------------E stop function----------------------------------------------------
void E_STOP() {       //estop disables motor drive. Cycle system power to reset.
  tftBottomPrint("ESTOP TRIGGERED");
  delay(1000);
  tftBottomPrint("RESET BUTTON AND RESTART");
  while(digitalRead(estop_button)==LOW){
    delay(100);
  }
}


//--------------------------Function to display files-----------------------------------------------
void displayfiles() {
  num_files=0;
  int count;
  tft.fillScreen(HX8357_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(HX8357_WHITE, HX8357_BLACK);
  File root = SD.open("/");
  uint16_t y_disp = 0;
  while (true) {
    File entry =  root.openNextFile();
    if (! entry) {
      // no more files
      break;
    }
    else if (entry.isDirectory()) {
      continue;
    }
    tft.setCursor(0, y_disp);
    tft.print(entry.name());
    entry.close();
    y_disp += 25;
    num_files++;
  }
  if (num_files<1){
    tft.setCursor(0,0);
    tft.print("NO FILES");
    MainMenuButtons[0].initButton(& tft, tft.width() - BUTTON_X, BUTTON_Y + 4 * BUTTON_H,
                                BUTTON_W, BUTTON_H, HX8357_BLACK, HX8357_DARKGREY, HX8357_WHITE, "BACK", 2);
    MainMenuButtons[0].drawButton();
    count=1;
  }
  else{
    MainMenuButtons[0].initButton(& tft, tft.width() - BUTTON_X, BUTTON_Y + 4 * BUTTON_H,
                                BUTTON_W, BUTTON_H, HX8357_BLACK, HX8357_DARKGREY, HX8357_WHITE, "BACK", 2);
    MainMenuButtons[0].drawButton();
    MainMenuButtons[1].initButton(& tft,BUTTON_X, BUTTON_Y + 4 * BUTTON_H,
                                  BUTTON_W, BUTTON_H, HX8357_BLACK, HX8357_RED, HX8357_WHITE, "DELETE", 2);
    MainMenuButtons[1].drawButton();
    count=2;
    if (num_files>1){
      MainMenuButtons[2].initButton(& tft,2*BUTTON_W, BUTTON_Y + 4 * BUTTON_H,
                                    BUTTON_W*2, BUTTON_H, HX8357_BLACK, HX8357_RED, HX8357_WHITE, "DEL ALL", 2);
      MainMenuButtons[2].drawButton();
      count=3;
    }
  }
  bool exitLoop = false;
  while (!exitLoop)
  {
    int btn =  tftButtonRelease(MainMenuButtons, count);
    switch (btn)
    {
      case BTN1:
        exitLoop = true;
        break;
      case BTN2:
        if (num_files>=1){
          selected_file_num=1;
          process_deletefile();
          displayfiles();
        }
        break;
      case BTN3:
        if (num_files>1){
          delete_all();
          tftBottomPrint("ALL FILES DELETED");
          delay(1000);
          tft.fillScreen(HX8357_BLACK);
          displayfiles();
          exitLoop=true;
        }
      default:
        break;
    }
  }
}
//--------------------------Funciton to delete all files-----------------------------------------------
void delete_all(){
  tft.fillScreen(HX8357_BLACK);
  tftSetCenterCursor("DELETE ALL FILES?",tft.width()/2,tft.height()/2);
  tft.setTextColor(HX8357_WHITE, HX8357_BLACK);
  tft.print("DELETE ALL FILES?");
  MainMenuButtons[0].initButton(& tft, tft.width() -BUTTON_W, BUTTON_Y + 4 * BUTTON_H,
                                BUTTON_W+BUTTON_X, BUTTON_H, HX8357_BLACK, HX8357_DARKGREY, HX8357_WHITE, "CANCEL", 2);
  MainMenuButtons[0].drawButton();
  MainMenuButtons[1].initButton(& tft,BUTTON_W, BUTTON_Y + 4 * BUTTON_H,
                                BUTTON_W+BUTTON_X, BUTTON_H, HX8357_BLACK, HX8357_RED, HX8357_WHITE, "YES", 2);
  MainMenuButtons[1].drawButton();
  
  bool exitLoop = false;
  int i=0;
  while (!exitLoop)
  {
    int btn =  tftButtonRelease(MainMenuButtons, 2);
    switch (btn)
    {
      case BTN1:  //CANCEL
        exitLoop = true;
        break;
      case BTN2:   //YES
         File root = SD.open("/");
         File entry = root.openNextFile();
         while (i<num_files) {
           if (entry.isDirectory() || entry.name()=="ewu.bmp") {
             continue;
           }
           else if (! entry){
             exitLoop=true;
             break;
           }
           else{
             SD.remove(entry.name());
             entry=root.openNextFile();
           }
          i++;
          }
        exitLoop=true;
        break;     
    }
  }
}

//------------------------Function to delete one file------------------------------------------------
void deletefile(String file){
  tft.fillScreen(HX8357_BLACK);
  tftSetCenterCursor(file,tft.width()/2,tft.height()/2);
  tft.setTextColor(HX8357_WHITE, HX8357_BLACK);
  tft.print(file);
  MainMenuButtons[0].initButton(& tft, tft.width() -BUTTON_W, BUTTON_Y + 4 * BUTTON_H,
                                BUTTON_W+BUTTON_X, BUTTON_H, HX8357_BLACK, HX8357_DARKGREY, HX8357_WHITE, "CANCEL", 2);
  MainMenuButtons[0].drawButton();
  MainMenuButtons[1].initButton(& tft,BUTTON_W, BUTTON_Y + 4 * BUTTON_H,
                                BUTTON_W+BUTTON_X, BUTTON_H, HX8357_BLACK, HX8357_RED, HX8357_WHITE, "DELETE", 2);
  MainMenuButtons[1].drawButton();

  bool exitLoop = false;
  while (!exitLoop){
    int btn =  tftButtonRelease(MainMenuButtons, 2);
    switch (btn)
    {
      case BTN1:  //CANCEL
        exitLoop = true;
        tftBottomPrint("CANCELLED");
        break;
      case BTN2:   //DELETE
        SD.remove(file);
        tftBottomPrint(file+" DELETED");
        delay(200);
        exitLoop=true;
        break;     
      default:
        break;
    }
  }
}

//--------------------------Function to display file menu buttons----------------------------------------------
void process_deletefile(){
  tft.setTextColor(HX8357_WHITE, HX8357_BLACK);
  uint16_t y_disp = 0;
  String file;
  int i=1;

  MainMenuButtons[0].initButton(& tft, tft.width() - BUTTON_X, BUTTON_Y + 4 * BUTTON_H,
                                BUTTON_W, BUTTON_H, HX8357_BLACK, HX8357_DARKGREY, HX8357_WHITE, "BACK", 2);
  MainMenuButtons[0].drawButton();
  MainMenuButtons[1].initButton(& tft,BUTTON_X, BUTTON_Y + 4 * BUTTON_H,
                                BUTTON_W, BUTTON_H, HX8357_BLACK, HX8357_BLUE, HX8357_WHITE, "SEL", 2);
  MainMenuButtons[1].drawButton();
  MainMenuButtons[2].initButton(& tft,BUTTON_W*2, BUTTON_Y + 4 * BUTTON_H,
                                BUTTON_W*2, BUTTON_H, HX8357_BLACK, HX8357_RED, HX8357_WHITE, "DELETE", 2);
  MainMenuButtons[2].drawButton();
  reselect:
  File root = SD.open("/");
  i=1;
  y_disp=0;
  while (i<=num_files) {
    File entry =  root.openNextFile();
    if (! entry) {
      // no more files
      break;
    }
    else if (entry.isDirectory()) {
      continue;
    }
    else if (i==selected_file_num){
      tft.setTextColor(HX8357_BLACK, HX8357_WHITE);
      file=entry.name();
    }
    else{
      tft.setTextColor(HX8357_WHITE, HX8357_BLACK);
    }
    tft.setCursor(0, y_disp);
    tft.print(entry.name());
    entry.close();
    y_disp += 25;
    i++;
  }
  
  bool exitLoop = false;
  while (!exitLoop)
  {
    int btn =  tftButtonRelease(MainMenuButtons, 3);
    switch (btn)
    {
      case BTN1:  //back
        exitLoop = true;
        break;
      case BTN2:   //SELECT
        if (selected_file_num==num_files){
          selected_file_num=1;
        }
        else {
          selected_file_num++;
        }
        goto reselect;
        break;     
      case BTN3:    //DELETE
        deletefile(file);
        exitLoop=true;
        break;
      default:
        break;
    }
  }
}

//--------------------------Function to center text on x,y coordinates----------------------------------------------
void tftSetCenterCursor(String str, int16_t xIn, int16_t yIn) {
  int16_t xText, yText;
  uint16_t w, h;

  tft.getTextBounds(str, 0, 0, &xText, &yText, &w, &h);
  tft.setCursor(xIn - w / 2, yIn - h / 2);
}

//--------------------------Function to write text to the right of x,y coordinates----------------------------------
void tftJustifyRight(String str, int16_t xIn, int16_t yIn) {
  int16_t xText, yText;
  uint16_t w, h;

  tft.getTextBounds(str, 0, 0, &xText, &yText, &w, &h);
  tft.setCursor(tft.width() - w, yIn);
}

//--------------------------Function to print text starting in upper left corner of screen------------------------------------------------------
void tftPrint(String str) {
  int16_t xText, yText;
  uint16_t w, h;

  //tft.getTextBounds(str, 0, 0, &xText, &yText, &w, &h);
  //tft.fillRect(  tft.getCursorX(),   tft.getCursorY(), w, h, HX8357_BLACK);
  tft.fillRect(0, tft.height() - 50, tft.width(), 50, HX8357_BLACK);
  tft.print(str);
}

//--------------------------Function to print to bottom of tft screen-----------------------------------------------
void tftBottomPrint(String str) {
  tft.setTextSize(TEXT_SIZE);
  tft.setTextColor(HX8357_WHITE, HX8357_BLACK);
  tftSetCenterCursor(str, tft.width() / 2,
                     (tft.height() - DISPLAY_YOFFSET));
  tftPrint(str);
}
void tftSplashPrint(String str) {
  tft.setTextSize(TEXT_SIZE);
  tft.setTextColor(HX8357_BLACK, HX8357_WHITE);
  tftSetCenterCursor(str, tft.width() / 2,
                     (tft.height() - DISPLAY_YOFFSET));
  int16_t xText, yText;
  uint16_t w, h;
  tft.fillRect(0, tft.height() - 50, tft.width(), 50, HX8357_WHITE);
  tft.print(str);
  delay(500);
}
//--------------------------Random variables----------------------------------------------------
unsigned long startTime;
String filename;

float displacement;
float time;    //time in seconds (for write to SD)
float timeh;   //time in hours   (for readout)
float force;
String testtype;
int counterStatic = 1;
int counterFatigue = 1;
unsigned long cycle_num;
bool check_for_release;

//-------------------------MAIN SETUP FUNCTION (gets executed first)----------------------------------------------------
void setup() {
  Serial.begin(38400);
  delay(5000);
  pinMode(0, INPUT_PULLUP);
  Serial1.begin(38400,SERIAL_8N2);
  pinMode(ls_top,INPUT);
  pinMode(estop_button,INPUT);
  
  
  //////////SCALE///////////////
  scale.begin(DOUT, CLK);
  delay(200);
  /////////TOUCHSCREEN/////////
  tft.begin();
  tft.setRotation(0);
  tft.setTextWrap(false);
  /////////SPLASH SCREEN EWU LOGO//////////
  tft.fillScreen(HX8357_WHITE);
  tftSplashPrint("Initializing SD Card...");
  while(!SD.begin(SD_CS)){
    delay(10);
    tftSplashPrint("Waiting for SD");
  }
  bmpDraw("img/ewu.bmp", 0, (tft.height()-320)/2);
  if (digitalRead(estop_button)==LOW){
    E_STOP();
  }
  tftSplashPrint("Initializing Scale...");
  //attachInterrupt(digitalPinToInterrupt(estop_button),E_STOP,LOW); //triggers ESTOP interrupt when pin is LOW  /////disconnected////
  motor_calibrate();
  tftSplashPrint("Motor Calibrated");
  tft.fillScreen(HX8357_BLACK);
  initializeButtons(MainMenuButtons, MainMenuColors, MainMenuLabels, MainMenu_BTN_CNT);
}


//-------------------------Function to check buttons for presses-------------------------------------------------------
int  tftButtonRelease(Adafruit_GFX_Button menuButtons[], int menuButtonCount) {
  int btn = -1;
  TSPoint p = ts.getPoint();
  int16_t px = p.x;
  int16_t py = p.y;

  if (p.z > MINPRESSURE && p.z < MAXPRESSURE)
  {
    p = ts.getPoint();
  }

  // Scale to  tft.width using the calibration #'s
  if (p.z != -1)
  {
    py = map(p.y, TS_MINY, TS_MAXY, 0, tft.height());
    px = map(p.x, TS_MINX, TS_MAXX, 0, tft.width());
  }

  // go thru all the buttons, checking if they were pressed
  for (uint8_t b = 0; b < menuButtonCount; b++)
  {
    if (menuButtons[b].contains(px, py))
    {
      menuButtons[b].press(true);  // tell the button it is pressed
    }
    else
    {
      menuButtons[b].press(false);  // tell the button it is NOT pressed
    }
  }

  // now we can ask the buttons if their state has changed
  for (uint8_t b = 0; b < menuButtonCount; b++)
  {
    if (menuButtons[b].justReleased())
    {
      if(check_for_release){
        btn=b;
      }
      menuButtons[b].drawButton();  // draw normal
      delay(100);
    }

    if (menuButtons[b].justPressed())
    {
      menuButtons[b].drawButton(true);  // draw invert!
      delay(100); // UI debouncing
      btn = b;
    }
  }
  return btn;
}


//-------------------------Function to use scale------------------------------------------------------
void startup_scale() {
  scale.begin(DOUT, CLK);
  tftBottomPrint("Waking up scale...");
  while (!scale.is_ready()){
    delay(50);
  }
  tftBottomPrint("Taring...");
  scale.set_scale(calibration_factor);
  float val=scale.get_units();
  //tare scale
  while (val>ZERO || val<-ZERO){
    scale.tare(3);
    scale.set_scale(calibration_factor);
    val=scale.get_units();
  }
  tftBottomPrint("Scale tared");
}


//-------------------------Testing Function-------------------------------------------------------
void TESTING() {
  tft.fillScreen(HX8357_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(HX8357_WHITE, HX8357_BLACK);
  if (CYCLES<1){
    testtype = "STATIC";
    create_file(testtype);  
  }
  else{
    testtype = "FATIGUE";
    create_file(testtype);
  }
  tftBottomPrint("Creating "+filename);
  
  float f=0;
  float d=0;
  float t=0;
  long num_cycles = (long)pow(10,CYCLES);
  
  long N=1;
  long cycle_start;
  long cycle_end;
  float sample_rate;
  int cycle_speed=2;                          //up in one move, down in one move -> 2
  bool broken=false;
  bool exitLoop=false;
  bool overload=false;
  tft.setCursor(0,0);
  tft.print("Force (N)  ");
  tft.setCursor(0,30);
  tft.print("Disp (mm)  ");
  tft.setCursor(0,60);
  tft.print("Cycles");
  tft.setCursor(0,90);
  tft.print("Time Elapsed (s)");
  tft.setCursor(0,120);
  tft.print("Sample Rate (Hz)");
  tft.setCursor(0,150);
  tft.print("Cycle Rate (Hz)");
  delay(300);
  dataFile = SD.open(filename, O_CREAT | O_APPEND | O_WRITE);
  dataFile.print("Time(s),Displacement(mm),Force(N),Cycles");
  dataFile.print('\n');
  dataFile.flush();
  startup_scale();
  int i=0;
  while(i<10){              //let value stabilize before taking measurements
    scale.get_units();
    Serial.println(scale.get_units());
    i++;
  }
  tftBottomPrint("TESTING");
  ReadMotorPosition32(0);
  delay(50);
  startTime = millis();
  move_abs32(0,spring_top/4);    //moves to absolute spring top position
  delay(1000);                    //waits for motor to get to position
  cycle_start=millis();


  /////////////////////////////////////move down slowly and record/////////////////////////////////////////////////////////////
  
  while (f<FORCE && d<DISP*10 && f<MAX_FORCE)
  {
    move_rel32(0,-20);
    cycle_start=millis();
    t = double(millis() - startTime) / 1000;
    f=scale.get_units();
    ReadMotorPosition32(0);
    if (f>MAX_FORCE){
      tftBottomPrint("OVERLOADED");
      break;
    }
    delay(10);
    d=double((spring_top - Motor_Pos32)/(4*steps_per_mm)); //conversion factor to get to (mm)
    
    dataFile.print(t,2);
    dataFile.print(',');
    dataFile.print(d,2);
    dataFile.print(',');
    dataFile.print(f,2);
    dataFile.print(',');
    dataFile.print(N,DEC);
    dataFile.print(',');
    dataFile.print('\n');
    dataFile.flush();           //very important

    tft.setCursor(tft.width()-80, 0);
    tft.print(f,2);
    tft.setCursor(tft.width()-80, 30);    
    tft.print(d,2);
    tft.setCursor(tft.width()-80, 60);
    tft.print(N);
    tft.setCursor(tft.width()-80, 90);
    tft.print(t,2);
    tft.setCursor(tft.width()-80,120);
    sample_rate=1000/double(cycle_end - cycle_start);
    tft.print(sample_rate);
    tft.setCursor(tft.width()-80,150);
    tft.print(sample_rate/cycle_speed, 2);
  }   

  ReadMotorPosition32(0);
  delay(50);
  spring_bottom = Motor_Pos32;  

  
  
  /////////////////////////////////////move up slowly and record/////////////////////////////////////////////////////////////
  while (Motor_Pos32 < spring_top)
  {
    move_rel32(0,20);
    cycle_start=millis();
    t = double(millis() - startTime) / 1000;
    f=scale.get_units();
    ReadMotorPosition32(0);
    delay(10);
    d=double((spring_top - Motor_Pos32)/(4*steps_per_mm)); //conversion factor to get to (mm)
    dataFile.print(t,2);
    dataFile.print(',');
    dataFile.print(d,2);
    dataFile.print(',');
    dataFile.print(f,2);
    dataFile.print(',');
    dataFile.print(N,DEC);
    dataFile.print(',');
    dataFile.print('\n');
    dataFile.flush();           //very important

    tft.setCursor(tft.width()-80, 0);
    tft.print(f,2);
    tft.setCursor(tft.width()-80, 30);    
    tft.print(d,2);
    tft.setCursor(tft.width()-80, 60);
    tft.print(N);
    tft.setCursor(tft.width()-80, 90);
    tft.print(t,2);
    tft.setCursor(tft.width()-80,120);
    cycle_end=millis();
    sample_rate=1000/double(cycle_end - cycle_start);
    tft.print(sample_rate);
    tft.setCursor(tft.width()-80,150);
    tft.print(sample_rate/cycle_speed, 2);
    
  }   
  N = N + 1;
  delay(50);
  move_abs32(0,spring_top/4);
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
  dataFile.print("Time(s),Displacement(mm),Force(N),Cycles"); 
  dataFile.print('\n');
  dataFile.flush();
  tftBottomPrint("BEGIN FATIGUE TEST");
  
  long timer = millis();        
  
  
  while(N <= num_cycles){

    
    for(i=0;i<1;i++){
      
      /*if (f<.05 && Motor_Pos32<spring_top-1000){   
        tftBottomPrint("BROKEN");
        break;
      }*/
      if (num_cycles<=0){
        tftBottomPrint("DONE");
        break; 
      }
      if (f>MAX_FORCE){
        tftBottomPrint("OVERLOADED");
        break;
      }

      move_abs32(0,(spring_top/4));
      delay(150);
      cycle_start=millis();    
      f=scale.get_units();
      t = double(millis() - startTime) / 1000;
      ReadMotorPosition32(0);
      d=double((spring_top - Motor_Pos32)/(4*steps_per_mm)); //conversion factor to get to (mm)
      dataFile.print(t,2);
      dataFile.print(',');
      dataFile.print(d,2);
      dataFile.print(',');
      dataFile.print(f,2);
      dataFile.print(',');
      dataFile.print(N,DEC);
      dataFile.print(',');
      dataFile.print('\n');
      dataFile.flush();           //very important
      tft.setCursor(tft.width()-80, 0);
      tft.print(f,2);
      tft.setCursor(tft.width()-80, 30);    
      tft.print(d,2);
      tft.setCursor(tft.width()-80, 60);
      tft.print(N);
      tft.setCursor(tft.width()-80, 90);
      tft.print(t,2);
      tft.setCursor(tft.width()-80,120);
      cycle_end=millis();
      sample_rate=1000/double(cycle_end - cycle_start);
      tft.print(sample_rate);
      tft.setCursor(tft.width()-80,150);
      tft.print(sample_rate/cycle_speed, 2);
    }
    
  

    for(i=0;i<1;i++){
      /*if (f<.05 && Motor_Pos32<spring_top-1000){ 
        tftBottomPrint("BROKEN");
        break;
      }*/
      if (num_cycles<=0){
        tftBottomPrint("DONE");
        break; 
      }
      if (f>MAX_FORCE){
        tftBottomPrint("OVERLOADED");
        break;
      }

      move_abs32(0,(spring_bottom/4));
      delay(150);
      cycle_start = millis();
     
      f=scale.get_units();
      ReadMotorPosition32(0);
      t = double(millis() - startTime) / 1000;
      d=double((spring_top - Motor_Pos32)/(4*steps_per_mm)); //conversion factor to get to (mm)
      dataFile.print(t,2);
      dataFile.print(',');
      dataFile.print(d,2);
      dataFile.print(',');
      dataFile.print(f,2);
      dataFile.print(',');
      dataFile.print(N,DEC);
      dataFile.print(',');
      dataFile.print('\n');
      dataFile.flush();           //very important
      
      tft.setCursor(tft.width()-80, 0);
      tft.print(f,2);
      tft.setCursor(tft.width()-80, 30);    
      tft.print(d,2);
      tft.setCursor(tft.width()-80, 60);
      tft.print(N);
      tft.setCursor(tft.width()-80, 90);
      tft.print(t,2);
      tft.setCursor(tft.width()-80,120);
      cycle_end=millis();
      sample_rate=1000/double(cycle_end - cycle_start);
      tft.print(sample_rate);
      tft.setCursor(tft.width()-80,150);
      tft.print(sample_rate/cycle_speed, 2);
    }
    
    /*if (f<ZERO && Motor_Pos32<spring_top-1000){   
        tftBottomPrint("BROKEN");
        break;
    }*/
    if (N>=num_cycles){
        break; 
    }
    if (f>MAX_FORCE){
        tftBottomPrint("OVERLOADED");
        break;
    }
    
 
    N = N+1;

  }
  move_abs32(0, (spring_top/4));
  tftBottomPrint("DONE");
  dataFile.close();
  
}
//-----------------------------------------------------------------------------------
bool process_auto_setup() {
  String msg = "PLACE SPRING ON SCALE";
  bool exitLoop = false;
  bool exitstatus = false;
  int i=0;
  tft.fillScreen(HX8357_BLACK);
  TestMenuButtons[0].initButton(& tft, 2 * BUTTON_W, BUTTON_Y + 3 * BUTTON_H,
                                BUTTON_W * 2, BUTTON_H, HX8357_BLACK, HX8357_GREEN, HX8357_WHITE, "OK", 3);
  TestMenuButtons[0].drawButton();
  TestMenuButtons[1].initButton(& tft, tft.width() - BUTTON_X, BUTTON_Y + 4 * BUTTON_H,
                                BUTTON_W, BUTTON_H, HX8357_BLACK, HX8357_DARKGREY, HX8357_WHITE, "BACK", 2);
  TestMenuButtons[1].drawButton();

  tftBottomPrint(msg);
  tft.setTextColor(HX8357_WHITE, HX8357_BLACK);
  while (!exitLoop)
  {
    int btn =  tftButtonRelease(TestMenuButtons, 2);
    switch (btn)
    {
      case BTN1:
        tft.fillScreen(HX8357_BLACK);
        startup_scale();
        
        while(i<10){              //let value stabilize before taking measurements
          scale.get_units();
          i++;
        }
        tftBottomPrint("SETUP IN PROGRESS");
        f=scale.get_units();
        while(f<0.05 && f<10 && Motor_Pos32 > bottom_limit){
          move_rel32(0,-3);
          ReadMotorPosition32(0);
          f=scale.get_units();
        }
        tftBottomPrint("CONTACT");
        delay(500);
        while(f>0.05 && digitalRead(ls_top) && Motor_Pos32< top_limit){
          move_rel32(0,3);
          ReadMotorPosition32(0);
          f=scale.get_units();
        }
        ReadMotorPosition32(0);
        delay(50);
        spring_top = Motor_Pos32;
        delay(50);
        tftBottomPrint("DONE");
        delay(1000);
        exitLoop = true;
        exitstatus = true;
        break;

      case BTN2:
        tftBottomPrint("Returning to Jog Menu");
        exitLoop = true;
        exitstatus = false;
        break;
    }
  }
  return exitstatus;
}

//--------------------------------------------------------------------------------
int process_mod_value_menu(String pre, int val, int val_max, int val_min, String units) {
  String msg = "";
  bool exitLoop = false;

  initialize_Mod_Value_Buttons(Mod_Value_Buttons, Mod_Value_Colors, Mod_Value_Labels, Mod_Value_BTN_CNT);
  tftBottomPrint(pre + String(val) + units);

  while (!exitLoop)
  {
    int btn =  tftButtonRelease(Mod_Value_Buttons, Mod_Value_BTN_CNT);
    switch (btn)
    {
      case BTN1:
        // +
        if (val >= val_max) {
          break;
        }
        else {
          val = val + 1;
        }
        tftBottomPrint(pre + String(val) + units);
        break;

      case BTN2:
        // -
        if (val <= val_min) {
          break;
        }
        else {
          val = val - 1;
        }
        tftBottomPrint(pre + String(val) + units);
        break;

      case BTN3:
        msg = "Saved";
        tftBottomPrint(msg);
        tft.fillRect((tft.width() - 2 * BUTTON_W), 0, BUTTON_W, 3 * BUTTON_H, HX8357_BLACK);
        exitLoop = true;
        return val;
        break;

      default:
        break;
    }
  }
}

//--------------------------------------------------------------------------------
void process_test_menu() {
  String msg = "TESTING MODE";
  bool exitLoop = false;
  bool force_set = false;
  bool disp_set = false;
  bool cycles_set = false;
  bool started = false;

  initializeButtons(TestMenuButtons, TestMenuColors, TestMenuLabels, TestMenu_BTN_CNT);

  Serial.println(msg);
  tftBottomPrint(msg);

  while (!exitLoop)
  {
    int btn =  tftButtonRelease(TestMenuButtons, TestMenu_BTN_CNT);
    if (btn >= 0 && btn < TestMenu_BTN_CNT)
    {
      Serial.print("btn = "); Serial.println(btn);
    }
    if (force_set && disp_set && cycles_set && !started) {
      TestMenuButtons[3].initButton(& tft, 2 * BUTTON_W, BUTTON_Y + 3 * BUTTON_H,
                                    BUTTON_W * 2, BUTTON_H, HX8357_BLACK, HX8357_GREEN, HX8357_WHITE, "START", 3);
      TestMenuButtons[3].drawButton();
      started = true;
    }

    switch (btn)
    {
      case BTN1:
        msg = "Set Max Displacement";
        Serial.println(msg);
        tftBottomPrint(msg);
        DISP = process_mod_value_menu("", DISP, MAX_DISP, MIN_DISP, " cm");
        
        disp_set = true;
        Serial.println("DISP SET TRUE");
        break;

      case BTN2:
        msg = "Set Max Force";
        Serial.println(msg);
        tftBottomPrint(msg);
        FORCE = process_mod_value_menu("", FORCE, MAX_FORCE, MIN_FORCE, " N");
        force_set = true;
        Serial.println("FORCE SET TRUE");
        break;

      case BTN3:
        msg = "Set Cycle Count";
        Serial.println(msg);
        tftBottomPrint(msg);
        CYCLES = process_mod_value_menu("10^", CYCLES, MAX_CYCLES, MIN_CYCLES, " cycles");
        cycles_set = true;
        Serial.println("CYCLES SET TRUE");
        break;

      case BTN10:
        msg = "Returning to Main Menu";
        Serial.println(msg);
        tftBottomPrint(msg);
        exitLoop = true;
        break;
        if (force_set && disp_set && cycles_set) {
          Serial.println("READY FOR TESTING");
        case BTN4:
          tft.fillScreen(HX8357_BLACK);
          tftBottomPrint("BEGINNING TEST");
          TESTING();          //go to testing mode and data logging loop here
          
          exitLoop = true;
          break;
        }

      default:
        break;
    }
  }
}

//--------------------------------------------------------------------------------
void motor_speed_highlight(int spd)    //from 0 to 2
{
  uint16_t color[3] = {HX8357_BLACK, HX8357_BLACK, HX8357_BLACK};
  color[spd] = HX8357_RED;
  tft.fillRect(BUTTON_W, 20 , 5, 40, color[0]);
  tft.fillRect(BUTTON_W, 20 + BUTTON_H, 5, 40, color[1]);
  tft.fillRect(BUTTON_W, 20 + 2 * (BUTTON_H), 5, 40, color[2]);
}
//--------------------------------------------------------------------------------
void process_jog_menu() {
  int i=0;
  String msg;
  bool exitLoop = false;
  boolean success;
  float val=0;
  jog_speed=LOWSPD;   //default
  initializeButtons(JogMenuButtons, JogMenuColors, JogMenuLabels, JogMenu_BTN_CNT);
  motor_speed_highlight(0);   //set to LOW by default
  startup_scale();
  while(i<10){              //let value stabilize before taking measurements
    val=scale.get_units();
    //tftBottomPrint(String(val));
    i++;
    delay(20);
  }
  tftBottomPrint("JOG MENU");
  while (!exitLoop)
  {
    ReadMotorPosition32(0);
    
    int btn =  tftButtonRelease(JogMenuButtons, JogMenu_BTN_CNT);
    switch (btn)
    {
      case BTN1:
        msg = "LOW SPEED";
        Serial.println(msg);
        tftBottomPrint(msg);
        motor_speed_highlight(0);
        jog_speed=LOWSPD;
        break;

      case BTN2:
        msg = "MEDIUM SPEED";
        Serial.println(msg);
        tftBottomPrint(msg);
        motor_speed_highlight(1);
        jog_speed=MEDIUMSPD;
        break;

      case BTN3:
        msg = "HIGH SPEED";
        Serial.println(msg);
        tftBottomPrint(msg);
        motor_speed_highlight(2);
        jog_speed=HIGHSPD;
        break;

      case BTN6:                            //UP
        msg = "MOVING UP";
        tftBottomPrint(msg);
        while(!JogMenuButtons[btn].justReleased() && digitalRead(ls_top) && Motor_Pos32 < (top_limit-1000)){
          ReadMotorPosition32(0);
          move_rel32(0,jog_speed);
          delay(20);
          btn =  tftButtonRelease(JogMenuButtons, JogMenu_BTN_CNT);
          check_for_release= true;
          delay(20);
        }
        check_for_release=false;
        
        if (Motor_Pos32>= (top_limit-1000)){
            tftBottomPrint("UPPER LIMIT REACHED");
        }
        
        break;

      case BTN8:                          //DOWN
        while(!JogMenuButtons[btn].justReleased()&& val<10 && Motor_Pos32 > (bottom_limit+1000)){
          ReadMotorPosition32(0);
          move_rel32(0,-jog_speed);
          delay(20);
          btn =  tftButtonRelease(JogMenuButtons, JogMenu_BTN_CNT);
          check_for_release= true;
          val=scale.get_units();
        }
        check_for_release= false;
        if (val>10){
            tftBottomPrint("SCALE OVERLOAD");
            delay(100);
            move_rel32(0,100);
        } 
        if (Motor_Pos32<= (bottom_limit+1000)){
            tftBottomPrint("LOWER LIMIT REACHED");
        }
       
        break;

      case BTN9:
        msg = "AUTO SETUP";
        tftBottomPrint(msg);
        success = process_auto_setup();

        tft.fillScreen(HX8357_BLACK);
        if (success == true) {
          tftBottomPrint("SETUP COMPLETE");
                               //move to origin
          delay(1000);
          tftBottomPrint("PROCEED TO TEST");
          delay(1000);
          exitLoop = true;
        }
        else {
          initializeButtons(JogMenuButtons, JogMenuColors, JogMenuLabels, JogMenu_BTN_CNT);
          motor_speed_highlight(0);   //set to LOW by default
          stopmotor();
        }
        break;

      case BTN10:
        msg = "Returning to Main Menu";
        Serial.println(msg);
        tftBottomPrint(msg);
        exitLoop = true;
        break;      
    }                 
    
  }
}

//--------------------------------------------------------------------------------


//---------------------MAIN MENU ROUTINE------------------------------------------------
void loop() {
  String msg = "MAIN MENU";
  int btn =  tftButtonRelease(MainMenuButtons, MainMenu_BTN_CNT);
  switch (btn)
  {
    case BTN1:
      msg = "ENTERING JOG MODE";
      tftBottomPrint(msg);
      process_jog_menu();

      msg = "MAIN MENU";
      initializeButtons(MainMenuButtons, MainMenuColors, MainMenuLabels, MainMenu_BTN_CNT);
      tftBottomPrint(msg);
      break;

    case BTN2:
      msg = "ENTERING TESTING MODE";
      tftBottomPrint(msg);
      process_test_menu();

      msg = "MAIN MENU";
      initializeButtons(MainMenuButtons, MainMenuColors, MainMenuLabels, MainMenu_BTN_CNT);
      tftBottomPrint(msg);
      break;

    case BTN6:
      msg = "LISTING FILES";
      tftBottomPrint(msg);
      displayfiles();

      msg = "MAIN MENU";
      initializeButtons(MainMenuButtons, MainMenuColors, MainMenuLabels, MainMenu_BTN_CNT);
      tftBottomPrint(msg);
      break;

    default:
      break;
  }
}
//------------------------------------------------------------

void create_file(String testtype) {
  if(testtype == "STATIC"){
    filename = "STATIC"+String(counterStatic)+".CSV";
    File root = SD.open("/");
    while(true){
      File entry = root.openNextFile();
      if(!entry){
        break;
      }
      else if(entry.isDirectory()){
        continue;
      }
      String entryname= entry.name();
      if(entryname == filename){
         counterStatic = counterStatic +1;
         filename = "STATIC"+String(counterStatic)+".CSV";
      }
      entry.close();
    }
  }
  
  if(testtype == "FATIGUE"){
    filename = "FATIGUE"+String(counterFatigue)+".CSV";
    File root = SD.open("/");
    while(true){
      File entry = root.openNextFile();
      if(!entry){
        break;
      }
      else if(entry.isDirectory()){
        continue;
      }
      String entryname = entry.name();
      if(entryname == filename){
        counterFatigue = counterFatigue+1;
        filename = "FATIGUE"+String(counterFatigue)+".CSV";
      }
      entry.close();
    }
  }
  dataFile = SD.open(filename, O_CREAT | O_WRITE);
  dataFile.close();
  
}

/////////////////////////// DMM SAMPLE COMMAND FUNCTIONS  ////////////////////////////////////

void move_rel32(char ID, long pos)
{
  long Pos = pos;
  char Axis_Num = ID;
  Global_Func = (char)Go_Relative_Pos;
  Send_Package(Axis_Num, Pos);
}

void ReadMotorTorqueCurrent(char ID)
{
  Global_Func = General_Read;
  Send_Package(ID , Is_TrqCurrent);
  MotorTorqueCurrentReady_Flag = 0xff;
  while (MotorTorqueCurrentReady_Flag != 0x00)
  {
    ReadPackage();
  }
}

void ReadMotorPosition32(char ID)
{
  Global_Func = (char)General_Read;
  Send_Package(ID , Is_AbsPos32);
  MotorPosition32Ready_Flag = 0xff;
  while (MotorPosition32Ready_Flag != 0x00)
  {
    ReadPackage();
  }
}

void move_abs32(char MotorID, long Pos32)
{                                            //changed to divided by 4 (for some reason it works)
  long Pos = Pos32;
  char Axis_Num = MotorID;
  Global_Func = (char)Go_Absolute_Pos;
  Send_Package(Axis_Num, Pos);
  
}

void Turn_const_speed(char ID, long spd)
{
  char Axis_Num = ID;
  Global_Func = (char)0x0a;
  Send_Package(Axis_Num, spd);
}

void stopmotor(){
  
  move_rel32(0,0);
}

//------------------------------------------------------
void motor_calibrate()
{
  int i=0;
  float val=0;
  String str;
  boolean SENSED=true;
  tftSplashPrint("Calibrating Motor...");  
  if(digitalRead(ls_top)){
    for(i=0;i<75;i++){
      move_rel32(0,10);
      delay(3);// move up ~0.25 inch
    }
  }
  scale.begin(DOUT, CLK);                
  while (!scale.is_ready()){
    delay(50);
  }
  scale.set_scale(calibration_factor);
  val=scale.get_units();
  while (val>ZERO || val<-ZERO){
    scale.tare(3);
    scale.set_scale(calibration_factor);
    val=scale.get_units();
  }
  while(i<10){              
    val=scale.get_units();
    i++;
  }
  
  while(val < 1){
    move_rel32(0,-10);
    val=scale.get_units();// move down very slowly
  }
  ReadMotorPosition32(0);
  bottom_limit = Motor_Pos32+100;

  
  for (i=0;i<4000;i++){
    move_rel32(0,10);
    delay(3);
    if (digitalRead(ls_top)==LOW){
      break;
    }
  }
  //move linear up fast ~ 6 inches
  while(digitalRead(ls_top)){
    move_rel32(0,10);
    delay(30);
  }
  ReadMotorPosition32(0);
  top_limit = Motor_Pos32-100;
  move_rel32(0,-500);
  delay(10);
  //move down linear 0.25 inches
  tftSplashPrint("CALIBRATION COMPLETE");
}

////////////////////// DYN232M SERIAL PROTOCOL PACKAGE FUNCTIONS  ///////////////////////////////

void ReadPackage(void)
{
  unsigned char c, cif;
  
  while (Serial1.available() > 0) {
    InputBuffer[InBfTopPointer] = Serial1.read(); //Load InputBuffer with received packets
    InBfTopPointer++;
  }
  while (InBfBtmPointer != InBfTopPointer)
  {
    c = InputBuffer[InBfBtmPointer];
    InBfBtmPointer++;
    cif = c & 0x80;
    if (cif == 0) {
      Read_Num = 0;
      Read_Package_Length = 0;
    }
    if (cif == 0 || Read_Num > 0) {
      Read_Package_Buffer[Read_Num] = c;
      Read_Num++;
      if (Read_Num == 2) {
        cif = c >> 5;
        cif = cif & 0x03;
        Read_Package_Length = 4 + cif;
        c = 0;
      }
      if (Read_Num == Read_Package_Length) {
        Get_Function();
        Read_Num = 0;
        Read_Package_Length = 0;
      }
    }
  }
}

void Get_Function(void) {
  char ID, ReceivedFunction_Code, CRC_Check;
  long Temp32;
  ID = Read_Package_Buffer[0] & 0x7f;
  ReceivedFunction_Code = Read_Package_Buffer[1] & 0x1f;
  CRC_Check = 0;

  for (int i = 0; i < Read_Package_Length - 1; i++) {
    CRC_Check += Read_Package_Buffer[i];
  }

  CRC_Check ^= Read_Package_Buffer[Read_Package_Length - 1];
  CRC_Check &= 0x7f;
  if (CRC_Check != 0) {
  }
  else {
    switch (ReceivedFunction_Code)
    {
      case  Is_AbsPos32:
        Motor_Pos32 = Cal_SignValue(Read_Package_Buffer);
        MotorPosition32Ready_Flag = 0x00;
        break;
      case  Is_TrqCurrent:
        MotorTorqueCurrent = Cal_SignValue(Read_Package_Buffer);
        break;
      case  Is_Status:
        Driver_Status = (char)Cal_SignValue(Read_Package_Buffer);
        Drive_Status_Read_Flag = 0x00;
        // Driver_Status=drive status byte data
        break;
      case  Is_Config:
        Temp32 = Cal_Value(Read_Package_Buffer);
        //Driver_Config = drive configuration setting
        break;
      case  Is_MainGain:
        Driver_MainGain = (char)Cal_SignValue(Read_Package_Buffer);
        Driver_MainGain = Driver_MainGain & 0x7f;
        break;
      case  Is_SpeedGain:
        Driver_SpeedGain = (char)Cal_SignValue(Read_Package_Buffer);
        Driver_SpeedGain = Driver_SpeedGain & 0x7f;
        break;
      case  Is_IntGain:
        Driver_IntGain = (char)Cal_SignValue(Read_Package_Buffer);
        Driver_IntGain = Driver_IntGain & 0x7f;
        break;
      case  Is_TrqCons:
        Driver_TrqCons = (char)Cal_SignValue(Read_Package_Buffer);
        Driver_TrqCons = Driver_TrqCons & 0x7f;
        break;
      case  Is_HighSpeed:
        Driver_HighSpeed = (char)Cal_SignValue(Read_Package_Buffer);
        Driver_HighSpeed = Driver_HighSpeed & 0x7f;
        break;
      case  Is_HighAccel:
        Driver_HighAccel = (char)Cal_SignValue(Read_Package_Buffer);
        Driver_HighAccel = Driver_HighAccel & 0x7f;
        break;
      case  Is_Driver_ID:
        Driver_ReadID = ID;
        break;
      case  Is_Pos_OnRange:
        Driver_OnRange = (char)Cal_SignValue(Read_Package_Buffer);
        Driver_OnRange = Driver_OnRange & 0x7f;
        break;
    }
  }
}

long Cal_SignValue(unsigned char One_Package[8]) //Get data with sign - signed long
{
  char Package_Length, OneChar, i;
  long Lcmd,Lcmdleadingzeros;
  
  OneChar = One_Package[1];
  OneChar = OneChar >> 5;
  OneChar = OneChar & 0x03;
  Package_Length = 4 + OneChar;
  OneChar = One_Package[2]; /*First byte 0x7f, bit 6 reprents sign */
  if(OneChar & B01000000){
    Lcmdleadingzeros = 0xFFFFFF80;
  }
  else {
    Lcmdleadingzeros = 0;
  }
  OneChar &= 0x7f;
  Lcmd = (long)OneChar; /* Sign extended to 32bits */
  Lcmd += Lcmdleadingzeros;
  for (i = 3; i < Package_Length - 1; i++)
  {
    OneChar = One_Package[i];
    OneChar &= 0x7f;
    Lcmd = Lcmd << 7;
    Lcmd += OneChar;
  }
  
  return (Lcmd); /* Lcmd : -2^27 ~ 2^27 - 1 */
}

long Cal_Value(unsigned char One_Package[8])
{
  char Package_Length, OneChar, i;
  long Lcmd;
  OneChar = One_Package[1];
  OneChar = OneChar >> 5;
  OneChar = OneChar & 0x03;
  Package_Length = 4 + OneChar;

  OneChar = One_Package[2];   /*First byte 0x7f, bit 6 reprents sign      */
  OneChar &= 0x7f;
  Lcmd = (long)OneChar;     /*Sign extended to 32bits           */
  for (i = 3; i < Package_Length - 1; i++)
  {
    OneChar = One_Package[i];
    OneChar &= 0x7f;
    Lcmd = Lcmd << 7;
    Lcmd += OneChar;
  }
  return (Lcmd);        /*Lcmd : -2^27 ~ 2^27 - 1           */
}
void Send_Package(char ID , long Displacement) {
  unsigned char B[8], Package_Length, Function_Code;
  long TempLong;
  B[1] = B[2] = B[3] = B[4] = B[5] = (unsigned char)0x80;
  B[0] = ID & 0x7f;
  Function_Code = Global_Func & 0x1f;
  TempLong = Displacement & 0x0fffffff; //Max 28bits
  B[5] += (unsigned char)TempLong & 0x0000007f;
  TempLong = TempLong >> 7;
  B[4] += (unsigned char)TempLong & 0x0000007f;
  TempLong = TempLong >> 7;
  B[3] += (unsigned char)TempLong & 0x0000007f;
  TempLong = TempLong >> 7;
  B[2] += (unsigned char)TempLong & 0x0000007f;
  Package_Length = 7;
  TempLong = Displacement;
  TempLong = TempLong >> 20;
  if (( TempLong == 0x00000000) || ( TempLong == 0xffffffff)) { //Three byte data
    B[2] = B[3];
    B[3] = B[4];
    B[4] = B[5];
    Package_Length = 6;
  }
  TempLong = Displacement;
  TempLong = TempLong >> 13;
  if (( TempLong == 0x00000000) || ( TempLong == 0xffffffff)) { //Two byte data
    B[2] = B[3];
    B[3] = B[4];
    Package_Length = 5;
  }
  TempLong = Displacement;
  TempLong = TempLong >> 6;
  if (( TempLong == 0x00000000) || ( TempLong == 0xffffffff)) { //One byte data
    B[2] = B[3];
    Package_Length = 4;
  }
  B[1] += (Package_Length - 4) * 32 + Function_Code;
  Make_CRC_Send(Package_Length, B);
}

void Make_CRC_Send(unsigned char Plength, unsigned char B[8]) {
  unsigned char Error_Check = 0;
  char RS232_HardwareShiftRegister;
  
  for (int i = 0; i < Plength - 1; i++) {
    OutputBuffer[OutBfTopPointer] = B[i];
    OutBfTopPointer++;
    Error_Check += B[i];
  }
  Error_Check = Error_Check | 0x80;
  OutputBuffer[OutBfTopPointer] = Error_Check;
  OutBfTopPointer++;
  while (OutBfBtmPointer != OutBfTopPointer) {
    RS232_HardwareShiftRegister = OutputBuffer[OutBfBtmPointer];
    Serial1.write(RS232_HardwareShiftRegister);                          
    OutBfBtmPointer++; // Change to next byte in OutputBuffer to send
  }
}
