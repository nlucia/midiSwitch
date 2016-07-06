/*
 *  midiSwitch - Arduino based midi switcher - looper
 *  Copyright (C) 2016  Nacho Lucia
 * 
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 * 
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

/*
 *   Pinout: 
 *   
 *   d0 -> MIDI in (disconnect when programming arduino via USB)
 *   d1 -> MIDI out/thru
 *   d2 -> LCD RS
 *   d3 -> LCD E (Enable)
 *   d4 -> LCD D4
 *   d5 -> LCD D5
 *   d6 -> LCD D6
 *   d7 -> LCD D7
 *   d8 -> BTN 0 Config/back/store
 *   d9 -> LCD A (backlight led anode)
 *   d10 -> BTN 1 Down
 *   d11 -> BTN 2 Up
 *   d12 -> BTN 3 Ok/select/toggle
 *   
 *   a0-a5 -> OUT relay 0-5 (active high)
 *   
 *   Buttons use internal pull-ups, connect other end to ground.
 */

const char* VERSION = "1.31";

#include <EEPROM.h>
#include <MIDI.h>
#include <SimpleTimer.h>
#include <LiquidCrystal.h>
#include <PButton.h>
#include <Fsm.h>


const byte NUM_PATCHES = 128; //number of patches
const byte NUM_CONTROLS = 6; //number of switches/loops
const byte MIDI_CC_ON_THRESHOLD = 64; //cc values >= this are considered 'on'
const int MESSAGE_DISPLAY_TIME = 1200; //ms flashed mesages are shown

const byte CONFIG_START_ADDRESS = 0; //start address of settings in eeprom
const byte EEPROM_PATCH_START_ADDRESS = 64; //start address of patches in eeprom

const byte LCD_ROWS = 2;
const byte LCD_COLS = 16;

// I/O pins settings
// LCD pins
const byte LCD_RS  = 2;
const byte LCD_EN  = 3;
const byte LCD_D4  = 4;
const byte LCD_D5  = 5;
const byte LCD_D6  = 6;
const byte LCD_D7  = 7;
const byte LCD_PWM  = 9;

// button input pins
const byte BTN_0 = 8;
const byte BTN_1 = 10;
const byte BTN_2 = 11;
const byte BTN_3 = 12;

// relay output pins
const byte RLY_0 = A0;
const byte RLY_1 = A1;
const byte RLY_2 = A2;
const byte RLY_3 = A3;
const byte RLY_4 = A4;
const byte RLY_5 = A5;

/*
 * CONSTANT / GLOBAL VARS
 */
// FSM
typedef enum {
  EVT_TO_MAIN,
  EVT_TO_EDIT_PATCH,
  EVT_TO_EDIT_PATCH_PARAMETER,
  EVT_TO_SAVE_PATCH,
  EVT_TO_SETTINGS,
  EVT_TO_EDIT_OPTION,
  EVT_TO_RESET
} eventsEnum;

// Custom characters
const byte blqcarac[][8] PROGMEM = {
  {0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f}, 
  {0x1f, 0x18, 0x10, 0x10, 0x1f, 0x1f, 0x1f, 0x1c},
  {0x1f, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x10},
  {0x1f, 0x18, 0x10, 0x10, 0x11, 0x11, 0x11, 0x10},
  {0x1f, 0x18, 0x10, 0x10, 0x11, 0x11, 0x11, 0x11},
  {0x1f, 0x3, 0x1, 0x1, 0x11, 0x11, 0x11, 0x11},
  {0x1f, 0x3, 0x1, 0x1, 0x11, 0x11, 0x11, 0x1},
  {0x1f, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1},
  {0x1f, 0x3, 0x1, 0x1, 0x1f, 0x1f, 0x1f, 0x7},
  {0x10, 0x11, 0x11, 0x11, 0x10, 0x10, 0x18, 0x1f},
  {0x1c, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f},
  {0x1c, 0x1f, 0x1f, 0x1f, 0x10, 0x10, 0x18, 0x1f},
  {0x11, 0x11, 0x11, 0x11, 0x10, 0x10, 0x18, 0x1f},
  {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1f},
  {0x7, 0x1f, 0x1f, 0x1f, 0x1, 0x1, 0x3, 0x1f},
  {0x1, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1f},
  {0x1, 0x11, 0x11, 0x11, 0x1, 0x1, 0x3, 0x1f},
  {0x11, 0x11, 0x11, 0x11, 0x1, 0x1, 0x3, 0x1f}  
};

const byte blqnums[10][4] PROGMEM = {
  {4,5,12,17},
  {0,5,0,13}, 
  {1,6,9,14},
  {1,6,11,16},
  {2,7,10,15},
  {3,8,11,16},//5
  {3,8,9,16}, 
  {4,5,0,13},
  {3,6,9,16},
  {3,6,11,16}
};

// Settings
const char* CONFIG_VERSION = "v2"; // ID of the settings block, change if struct is modified

struct settingsStruct {
  char version[2];
  
  byte midiChannel;
  byte ccRelay1, ccRelay2, ccRelay3, ccRelay4;
  byte ccLoopA, ccLoopB;
  bool toggleMode; 
  byte brightness;
} _settings;

typedef enum {
  ST_CHANNEL,
  ST_TOGGLE,
  ST_CC_R1,
  ST_CC_R2,
  ST_CC_R3,
  ST_CC_R4,
  ST_CC_LA,
  ST_CC_LB,
  ST_BRIGHTNESS,
  ST_INFO,
  
  OPTIONSENUM_LENGTH
} optionsEnum;

const char optionName00[] PROGMEM = "Canal MIDI";
const char optionName01[] PROGMEM = "Modo CC";
const char optionName02[] PROGMEM = "CC Switch 1";
const char optionName03[] PROGMEM = "CC Switch 2";
const char optionName04[] PROGMEM = "CC Switch 3";
const char optionName05[] PROGMEM = "CC Switch 4";
const char optionName06[] PROGMEM = "CC Loop A";
const char optionName07[] PROGMEM = "CC Loop B";
const char optionName08[] PROGMEM = "Brillo";
const char optionName09[] PROGMEM = "Informacion";

const char* const _settingsItemName[] PROGMEM = {
  optionName00, optionName01, optionName02, optionName03, optionName04, 
  optionName05, optionName06, optionName07, optionName08, optionName09
};

typedef void (*callback) (void);

//Global vars
byte _patch = 0;
byte _relayStatus = 0xFF; //all off

byte _editPatchPosition = 0;
bool _isPatchModified = false;
bool _isConfigModified = false;
byte _tempPatchNumber = 0;

optionsEnum _settingsItem;
byte _settingsValue;
byte _tempSettingsValue;

/*
 * Global instances
 */
SimpleTimer _timer;

PButton btn_back = PButton(BTN_0);
PButton btn_down = PButton(BTN_1);
PButton btn_up = PButton(BTN_2);
PButton btn_ok = PButton(BTN_3);

LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

MIDI_CREATE_DEFAULT_INSTANCE();

//fsm
State stateSplash(NULL, NULL);
State stateMain(&onMainEnter, NULL);
State stateEditPatch(&onEditPatchEnter, &onEditPatchExit);
State stateSavePatch(&onSavePatchEnter, &onSavePatchExit);
State stateSettings(&onSettingsEnter, NULL);
State stateEditOption(&onEditOptionEnter, &onEditOptionExit);
State stateReset(&onResetEnter, NULL);

Fsm fsm(&stateSplash); 


//buttons
void resetButtons() {
  btn_ok.resetEvents();
  btn_back.resetEvents();
  btn_up.resetEvents();
  btn_down.resetEvents();
}


/* 
 *  CHANGE STATE
 */
void goToMain() {       fsm.trigger(EVT_TO_MAIN); }
void goToEditPatch() {  fsm.trigger(EVT_TO_EDIT_PATCH); }
void goToSavePatch() {  fsm.trigger(EVT_TO_SAVE_PATCH); }
void goToSettings() {   fsm.trigger(EVT_TO_SETTINGS); }
void goToEditOption() { fsm.trigger(EVT_TO_EDIT_OPTION); }
void goToReset() {      fsm.trigger(EVT_TO_RESET); }

/*
 * STATES
 */

/* 
 *  MAIN
 */
void onMainEnter() {

  MIDI.setHandleControlChange(midiControlChangeHandler);
  MIDI.setHandleProgramChange(midiProgramChangeHandler);
  
  updateMainDisplay();

  _tempPatchNumber = _patch;
  
  btn_up.addPushEvent(increasePatch); //immediate change
  btn_up.addRepeatEvent(increaseTempPatchNumber, updatePatchNumber);
  
  btn_down.addPushEvent(decreasePatch);  
  btn_down.addRepeatEvent(decreaseTempPatchNumber, updatePatchNumber);
  
  btn_back.addShortEvent(goToEditPatch);
  btn_back.addLongEvent(goToSettings);
}

void increasePatch(){
  changePatch(incDecValue(_patch, NUM_PATCHES, +1));
  _tempPatchNumber = _patch;
}

void decreasePatch(){
  changePatch(incDecValue(_patch, NUM_PATCHES, -1));
  _tempPatchNumber = _patch;
}

void updatePatchNumber() {
  changePatch(_tempPatchNumber);
}

/*
 * EDIT PATCH
 */
void onEditPatchEnter() {
  
  updateMainDisplay();
  updateEditPatchDisplay();

  btn_up.addPushEvent(editNextRelay);
  btn_down.addPushEvent(editPrevRelay);
  
  btn_back.addShortEvent(goToMain);
  btn_back.addLongEvent(goToSavePatch);
  
  btn_ok.addPushEvent(editToggleRelay);
}

void editNextRelay(){
  changeEditPatchPosition(incDecValue(_editPatchPosition, NUM_CONTROLS, +1));
  updateEditPatchDisplay();  
}

void editPrevRelay(){
  changeEditPatchPosition(incDecValue(_editPatchPosition, NUM_CONTROLS, -1));
  updateEditPatchDisplay();   
}

void editToggleRelay() {
    toggleRelayStatus(_editPatchPosition);
}

void onEditPatchExit() {
  lcdClearEditPatchPosition();
}

/*
 * SAVE PATCH
 */
void onSavePatchEnter() {
  
  MIDI.setHandleProgramChange(midiLearnPC);
  
  btn_up.addRepeatEvent(increaseTempPatchNumber);
  btn_down.addRepeatEvent(decreaseTempPatchNumber);
  btn_back.addShortEvent(goToEditPatch);
  btn_ok.addShortEvent(saveModifiedPatch);
   
  _tempPatchNumber = _patch;
  
  blinkBigNumbers(true);
  lcd.setCursor(4,1);
  lcd.write(0x7F);
  lcd.print(F(" Guardar en"));  
}

void onSavePatchExit() {
    blinkBigNumbers(false);
}

void increaseTempPatchNumber(){
    _tempPatchNumber = incDecValue(_tempPatchNumber, NUM_PATCHES, +1);
    lcdPrintBigNumber(_tempPatchNumber);
}

void decreaseTempPatchNumber(){
    _tempPatchNumber = incDecValue(_tempPatchNumber, NUM_PATCHES, -1);
    lcdPrintBigNumber(_tempPatchNumber);
}

void saveModifiedPatch() {
    savePatch(_tempPatchNumber, _relayStatus);
    if (_tempPatchNumber != _patch) {
      changePatch(_tempPatchNumber);
    }
    goToMain();
    flashMessage(F("  Guardado  "), lcdInitStatus);
}

void midiLearnPC(byte channel, byte number) {
    _tempPatchNumber = number;
    lcdPrintBigNumber(_tempPatchNumber);
}

/*
 * SETTINGS
 */
void onSettingsEnter() {
  btn_up.addRepeatEvent(nextOption);
  btn_down.addRepeatEvent(prevOption);
  
  btn_back.addShortEvent(goToMain);
  btn_ok.addShortEvent(okEditOption);
  
  btn_back.addLongEvent(saveSettingsAndExit);

  _settingsValue = *optionValuePt(_settingsItem);

  lcdPrintSettingsDisplay(_settingsValue, _settingsItem);
}

void nextOption(){
  changeSettingsItem((optionsEnum) incDecValue(_settingsItem, OPTIONSENUM_LENGTH, +1));  
}

void prevOption(){
  changeSettingsItem((optionsEnum) incDecValue(_settingsItem, OPTIONSENUM_LENGTH, -1));  
}

void okEditOption(){
  if(_settingsItem == ST_INFO) {
    splash();
    resetButtons();
    btn_back.addShortEvent(onSettingsEnter);
    btn_ok.addShortEvent(onSettingsEnter);
  }
  else {
    goToEditOption();
  }
}

void saveSettingsAndExit() {
  if (_isConfigModified) {
    saveSettings(); 
    _isConfigModified = false;
  }
  goToMain();
  flashMessage(F("  Guardado  "), lcdInitStatus);
}

/*
 * EDIT OPTION
 */
void onEditOptionEnter() {

  btn_up.addRepeatEvent(increaseOptionValue, applySettings);
  btn_down.addRepeatEvent(decreaseOptionValue, applySettings);

  btn_ok.addShortEvent(okOptionEdit);
  btn_back.addShortEvent(cancelOptionEdit);
  
  _settingsValue = *optionValuePt(_settingsItem);
  _tempSettingsValue = _settingsValue;
  lcdPrintEditOptionDisplay(_settingsValue, _settingsItem);

  if (_settingsItem >= ST_CC_R1 && _settingsItem <= ST_CC_LB) {
    MIDI.setHandleControlChange(midiLearnCC); //midi learn CC
  } else {
    MIDI.setHandleControlChange(NULL);    
  }
}

void increaseOptionValue() {
  changeOptionValue(incOptionValue(_settingsValue, _settingsItem), _settingsItem);
  updateEditOptionDisplay();
}
void decreaseOptionValue() {
  changeOptionValue(decOptionValue(_settingsValue, _settingsItem), _settingsItem);
  updateEditOptionDisplay();
}
void cancelOptionEdit() {
  changeOptionValue(_tempSettingsValue, _settingsItem);
  applySettings();
  goToSettings();
}
void okOptionEdit() {
  _isConfigModified = _isConfigModified || (_settingsValue != _tempSettingsValue);
  goToSettings();
}
void onEditOptionExit() {
  lcdClearEditOptionDisplay();
}
void midiLearnCC(byte channel, byte number, byte value) {
  changeOptionValue(number, _settingsItem);
  updateEditOptionDisplay();
}
/*
 * RESET
 */
void onResetEnter(){
  _tempSettingsValue = false;
  lcdPrintResetDisplay(false, F("  Borrar todo!!?"));

  btn_up.addShortEvent(nextResetValue);
  btn_down.addShortEvent(nextResetValue);
 
  btn_ok.addShortEvent(confirmReset);
  btn_back.addShortEvent(goToMain);
}

void nextResetValue(){
  _tempSettingsValue = !_tempSettingsValue;
  lcdPrintResetDisplay(_tempSettingsValue,"");
}

void confirmReset() {
  if (_tempSettingsValue) { // yes
    askAgainReset();
  } else {
    goToMain(); 
  }
}
void askAgainReset() {
  _tempSettingsValue = false;
  btn_ok.addShortEvent(lastConfirmReset);
  lcdPrintResetDisplay(_tempSettingsValue, F("     Seguro?   "));
}
void lastConfirmReset() {
  if (_tempSettingsValue) { // yes
    factoryReset();
    applySettings();
    goToMain(); 
    flashMessage(F("  Borrado!  "), lcdInitStatus);
  } else {
    goToMain(); 
  }
}

void flashMessage(const String& message, callback cb){
    lcd.setCursor(4,1);
    lcd.print(message);
    _timer.setTimeout(MESSAGE_DISPLAY_TIME, cb);   
}
/*
 * TRANSITIONS
 */
void stateTransition(){
  resetButtons();
}

void initMainTransition(){
  initMainDisplay();
  resetButtons();
}

/*
 * MAIN
 */
void lcdInitBigNumbers() {
  lcd.setCursor(0,0);
  lcd.write((byte) 0);
  lcd.write(1);
  lcd.write(4);
  lcd.write(5);
  lcd.setCursor(0,1);     
  lcd.write(2);
  lcd.write(3); 
  lcd.write(6);
  lcd.write(7); 
}

void lcdInitStatus() {
  lcd.setCursor(4,1);
  lcd.print(F(" 1 2 3 4 A B"));
}

void initLcd() {
  lcd.begin(LCD_COLS, LCD_ROWS);  
  analogWrite(LCD_PWM, brightnessLevelToValue(_settings.brightness));
}

void initMainDisplay() {
    lcdInitBigNumbers();
    lcdInitStatus();
}
/*
 * SETTINGS -- LCD
 */

void lcdPrintSettingsDisplay(byte value, optionsEnum option){
  const byte ellipsisChar[8] = {0, 0, 0, 0, 0, 0, 0x15, 0};
  lcd.createChar(2, (byte*)ellipsisChar);
  //top row
  lcd.setCursor(0,0);
  lcd.print(F("Opciones     "));

  lcd.setCursor(LCD_COLS - 1 - (OPTIONSENUM_LENGTH < 10 ? 1 : 2) 
                             - (option +1 <10 ? 1 : 2),0);
  lcd.print(option + 1);
  lcd.print('/');
  lcd.print(OPTIONSENUM_LENGTH);

  //bottom row
  lcd.setCursor(0,1);
  

  lcd.print(optionText(option));
  if (settingsValueText(value, option).length() > 0) lcd.print(':');
  else lcd.write(2);
  lcd.print(F("              "));

  lcdPrintOptionValue(value, option);

  //config modified flag
  lcd.setCursor(8,0); //8 = opciones text length
  lcd.write(_isConfigModified ? 0xEB : ' ');
}

void lcdPrintEditOptionDisplay(byte value, optionsEnum option) {
  lcdPrintSettingsDisplay(value, option);
  const byte arrowsChar[8] = {0x1F, 0x1B, 0x11, 0x1F, 0x1F, 0x11, 0x1B, 0x1F};
  lcd.createChar(1, (byte*)arrowsChar);
  lcdPrintEditOptionMarker(value, option);
  lcd.blink();    
}
void lcdPrintOptionValue(byte value, optionsEnum option) {
  String valueText = settingsValueText(value, option);
  lcd.setCursor(optionText(option).length() + 1,1); //right to option name
  lcd.print(F("             ")); 
  lcd.setCursor(LCD_COLS - valueText.length(),1);

  lcd.print(valueText);
}
void lcdPrintEditOptionMarker(byte value, optionsEnum option) {
  byte pos = LCD_COLS - 1 - settingsValueText(value, option).length();

  lcd.setCursor(pos,1);
  lcd.write(1);
  lcd.setCursor(pos,1);
}
void updateEditOptionDisplay(){
  lcdPrintOptionValue(_settingsValue, _settingsItem);
  lcdPrintEditOptionMarker(_settingsValue, _settingsItem);
}
void lcdClearEditOptionDisplay() {
  lcd.noBlink();  
}

void changeSettingsItem(optionsEnum item){
  _settingsItem = item;
  lcdPrintSettingsDisplay(*optionValuePt(item), item);
}

byte* optionValuePt(optionsEnum option) {
  switch (_settingsItem) {
    case ST_CHANNEL: return &_settings.midiChannel;
    case ST_TOGGLE: return (byte*) &_settings.toggleMode;
    case ST_CC_R1: return &_settings.ccRelay1;
    case ST_CC_R2: return &_settings.ccRelay2;
    case ST_CC_R3: return &_settings.ccRelay3;
    case ST_CC_R4: return &_settings.ccRelay4;
    case ST_CC_LA: return &_settings.ccLoopA;
    case ST_CC_LB: return &_settings.ccLoopB;
    case ST_BRIGHTNESS: return &_settings.brightness;
  }
}

String optionText(optionsEnum option){
  char buffer[16];  
  strcpy_P(buffer, (char*)pgm_read_word(&(_settingsItemName[option])));
  return buffer;
}
String settingsValueText(byte value, optionsEnum option){
  char buffer[8];
//  byte value = *optionValuePt(option);
  switch (_settingsItem) {
    case ST_CHANNEL:
      if (value == 0) return F("Omni");
      break;
    case ST_TOGGLE:
      if (value) return F("Toggle");
      else return F("Normal");
/*    case ST_BRIGHTNESS:
      if (value <= 0) return F("Off");
      if (value >= 5) return F("Max");
      break; */
    case ST_INFO:
      return "";
  }  
  sprintf(buffer, "%d",value);
  return buffer;
}

void applySettings() {
  analogWrite(LCD_PWM, brightnessLevelToValue(_settings.brightness));
  MIDI.setInputChannel(_settings.midiChannel);
}

byte incDecValue(byte value, byte max, int offset) { 
  return (offset >= 0) ? ((value + offset) % (max)) 
                       : (((value + offset) < 0) ? max + offset 
                                                 : value + offset); }

byte incDecOptionValue(byte value, optionsEnum option, int offset){
  switch (option) {
    case ST_CHANNEL: return incDecValue(value, 17, offset);
    case ST_TOGGLE: return !value;
    case ST_CC_R1:
    case ST_CC_R2:
    case ST_CC_R3:
    case ST_CC_R4:
    case ST_CC_LA:
    case ST_CC_LB: return incDecValue(value, 128, offset);
    case ST_BRIGHTNESS: return incDecValue(value, 6, offset);
  } 
}

byte incOptionValue(byte value, optionsEnum option) {
  return incDecOptionValue(value, option, +1);
}
byte decOptionValue(byte value, optionsEnum option) {
  return incDecOptionValue(value, option, -1);
}

void updateOptionValue(byte value, optionsEnum option){
  *optionValuePt(option) = value;
}

void changeOptionValue(byte value, optionsEnum option) {
  _settingsValue = value;
  updateOptionValue(value, option);
}

byte brightnessLevelToValue(byte level){
  if (level == 5) return 255;
  else if (level == 0) return 0;
  return 1 << level + 3;
}
/*
 * MAIN -- LCD
 */

void lcdPrintBigNumber(byte number){
    byte tens=((number+1)/10)%10;
    byte units=(number+1)%10;
    
    byte bufferTens[8];
    byte bufferUnits[8];
    
    for (byte i=0; i<4; i++) {
      for (byte j=0; j<8; j++) {
        bufferTens[j] = pgm_read_byte( &blqcarac[pgm_read_byte( &blqnums[tens][i] )][j] );
        bufferUnits[j]= pgm_read_byte( &blqcarac[pgm_read_byte( &blqnums[units][i] )][j] );
        if (number >= 99) { // invert character
          bufferTens[j] = ~bufferTens[j];
          bufferUnits[j] = ~bufferUnits[j];
        }
      }
      lcd.createChar(i,bufferTens);
      lcd.createChar(i+4,bufferUnits);
    }
}

void lcdClearBigNumbers() {
    lcd.setCursor(0,0);
    lcd.print(F("    "));
    lcd.setCursor(0,1);
    lcd.print(F("    ")); 
}

void lcdPrintRelayStatus(byte status) {
    for (byte i=0; i<NUM_CONTROLS; i++) {
      lcd.setCursor(5+2*i,0);
      lcd.write(status >> i & 1 ? 0x10 : 0xFF);
      lcd.print(' ');
    }  
}

void updateMainDisplay(){
  lcdPrintBigNumber(_patch);
  lcdPrintRelayStatus(_relayStatus);
  lcdPrintPatchModifiedFlag(_isPatchModified);
}

void blinkBigNumbers(bool active) {
   static byte timerId;
   if (active) {
     timerId = _timer.setInterval(600, toggleBlinkBigNumbers);
   } else {
     _timer.deleteTimer(timerId);
     lcdInitBigNumbers();
   }
}

void toggleBlinkBigNumbers(){
   lcdClearBigNumbers();
   _timer.setTimeout(200, lcdInitBigNumbers);
}

void changeEditPatchPosition(byte position) {
    if (position < 0) position = NUM_CONTROLS - 1;
    else if (position >= NUM_CONTROLS) position = 0;
    _editPatchPosition = position;
}
void updateEditPatchDisplay() {
      lcdClearEditPatchPosition();
      lcd.setCursor(4+2*_editPatchPosition,1);
      lcd.write(0x7e);
}
void lcdClearEditPatchPosition() {
  lcdInitStatus();
}

void lcdPrintResetDisplay(bool reset, const String& message){
  if (message.length() > 0) {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(message);  
  }
  lcd.setCursor(1,1);
  lcd.print(F(" No        Si"));
  lcd.setCursor(reset ? 11 : 1, 1);
  lcd.write(0x7E); 
}
/*
 * CONTROL -- PATCH
 */
byte changePatch(byte number){
  if (number >= 0 && number < NUM_PATCHES) {  
    _patch = number;
    _isPatchModified = false;
    changeRelayStatus(loadPatch(number));
    updateMainDisplay();

//    _tempPatchNumber = _patch;
  }
}

byte loadPatch(byte number) {
  if (number >= 0 && number < NUM_PATCHES) {  
    byte status = EEPROM.read(EEPROM_PATCH_START_ADDRESS + number);
    return status; //EEPROM has value 0xFF if never written to
  }
}

void savePatch(byte number, byte status) {
  if (number >= 0 && number < NUM_PATCHES) {  
    EEPROM.update(EEPROM_PATCH_START_ADDRESS + number, status);
    _isPatchModified = false;
  }
}

void factoryReset() {
  resetPatchMemory();
  resetSettingsMemory();
}

void resetPatchMemory(){
  for (int i = 0; i < NUM_PATCHES; i++) {
    EEPROM.update(EEPROM_PATCH_START_ADDRESS + i, 0xFF);
  }
  _patch = 0;
  _isPatchModified = false;
  changeRelayStatus(0xFF);
}

void resetSettingsMemory(){
  for (int i = 0; i < sizeof(settingsStruct); i++) {
    EEPROM.update(CONFIG_START_ADDRESS + i, 0xFF);
  }
  loadDefaultSettings();
}

void lcdPrintPatchModifiedFlag(bool modified) {
  lcd.setCursor(4,0);
  lcd.write(_isPatchModified ? 0xEB : ' ');   
}

void loadDefaultSettings(){
  strcpy(_settings.version,CONFIG_VERSION);
  _settings.midiChannel = MIDI_CHANNEL_OMNI;
  _settings.ccRelay1 = 75;
  _settings.ccRelay2 = 76; 
  _settings.ccRelay3 = 77; 
  _settings.ccRelay4 = 78;
  _settings.ccLoopA = 82;
  _settings.ccLoopB = 83;
  _settings.toggleMode = false;
  _settings.brightness = 5;
}
/*
 * CONTROL -- STATUS
 */
void changeRelayStatus(byte status) {
  // inverted status, relays are active low 
  digitalWrite(RLY_0, status & 1 << 0); //status bit 0
  digitalWrite(RLY_1, status & 1 << 1); //status bit 1
  digitalWrite(RLY_2, status & 1 << 2);
  digitalWrite(RLY_3, status & 1 << 3);

  digitalWrite(RLY_4, status & 1 << 4);
  digitalWrite(RLY_5, status & 1 << 5);  

  _relayStatus = status;
}


void setRelayStatus(byte pos, bool active) {
  if (active) changeRelayStatus(_relayStatus & ~(1 << pos));
  else changeRelayStatus(_relayStatus | (1 << pos));
  _isPatchModified = true;
  lcdPrintRelayStatus(_relayStatus);
  lcdPrintPatchModifiedFlag(_isPatchModified);
}

void toggleRelayStatus(byte pos) {    
  changeRelayStatus(_relayStatus ^ (1 << pos));  
  _isPatchModified = true;
  lcdPrintRelayStatus(_relayStatus);
  lcdPrintPatchModifiedFlag(_isPatchModified);
}

/*
 * MIDI
 */
void midiControlChangeHandler(byte channel, byte number, byte value) {

  if (_settings.toggleMode && value >= MIDI_CC_ON_THRESHOLD) {
    if (number == _settings.ccRelay1) toggleRelayStatus(0);
    if (number == _settings.ccRelay2) toggleRelayStatus(1);
    if (number == _settings.ccRelay3) toggleRelayStatus(2);
    if (number == _settings.ccRelay4) toggleRelayStatus(3);
    if (number == _settings.ccLoopA) toggleRelayStatus(4);
    if (number == _settings.ccLoopB) toggleRelayStatus(5); 
  }
  else if (!_settings.toggleMode) {     
    if (number == _settings.ccRelay1) setRelayStatus(0, value >= MIDI_CC_ON_THRESHOLD);
    if (number == _settings.ccRelay2) setRelayStatus(1, value >= MIDI_CC_ON_THRESHOLD);
    if (number == _settings.ccRelay3) setRelayStatus(2, value >= MIDI_CC_ON_THRESHOLD);
    if (number == _settings.ccRelay4) setRelayStatus(3, value >= MIDI_CC_ON_THRESHOLD);
    if (number == _settings.ccLoopA) setRelayStatus(4, value >= MIDI_CC_ON_THRESHOLD);
    if (number == _settings.ccLoopB) setRelayStatus(5, value >= MIDI_CC_ON_THRESHOLD);            
  }
}

void midiProgramChangeHandler(byte channel, byte number) {
  changePatch(number);
}

/*
 * CONFIG
 */

void loadSettings() {
  EEPROM.get(CONFIG_START_ADDRESS, _settings);
  if (strcmp(_settings.version,CONFIG_VERSION) != 0) {
    loadDefaultSettings();
  }
}

void saveSettings() {
   EEPROM.put(CONFIG_START_ADDRESS, _settings);
}

/*
 * SPLASH
 */
void splash() {
  lcd.setCursor(0,0);
  lcd.print(F("midiSwitch v"));
  lcd.print(VERSION);
  lcd.setCursor(0,1);
  lcd.print(F("N. Lucia - 2016"));    
}

/*
 * SETUP 
 */
void setup() {
  pinMode(LCD_PWM,OUTPUT);

  pinMode(BTN_0, INPUT_PULLUP);
  pinMode(BTN_1, INPUT_PULLUP);
  pinMode(BTN_2, INPUT_PULLUP);
  pinMode(BTN_3, INPUT_PULLUP);
  
  pinMode(RLY_0, OUTPUT);
  pinMode(RLY_1, OUTPUT);
  pinMode(RLY_2, OUTPUT);
  pinMode(RLY_3, OUTPUT);
  pinMode(RLY_4, OUTPUT);
  pinMode(RLY_5, OUTPUT);

//Serial.begin(9600);
//Serial.println("INICIO");

  loadSettings();
  changePatch(_patch);
  
  initLcd();
  
  splash(); 
  _timer.setTimeout(MESSAGE_DISPLAY_TIME, goToMain); //exit splash screen


  MIDI.begin(_settings.midiChannel);  
//Serial.begin(115200); //for hairless midi testing
 
  fsm.add_transition(&stateSplash, &stateMain, EVT_TO_MAIN, &initMainTransition);
  fsm.add_transition(&stateEditPatch, &stateMain, EVT_TO_MAIN, &stateTransition);
  fsm.add_transition(&stateSavePatch, &stateMain, EVT_TO_MAIN, &stateTransition);
  fsm.add_transition(&stateSettings, &stateMain, EVT_TO_MAIN, &initMainTransition);
  fsm.add_transition(&stateReset, &stateMain, EVT_TO_MAIN, &initMainTransition);

  fsm.add_transition(&stateEditPatch, &stateSavePatch, EVT_TO_SAVE_PATCH, &stateTransition);

  fsm.add_transition(&stateMain, &stateEditPatch, EVT_TO_EDIT_PATCH, &stateTransition);
  fsm.add_transition(&stateSavePatch, &stateEditPatch, EVT_TO_EDIT_PATCH, &stateTransition);

  fsm.add_transition(&stateMain, &stateSettings, EVT_TO_SETTINGS, &stateTransition);
  fsm.add_transition(&stateEditOption, &stateSettings, EVT_TO_SETTINGS, &stateTransition);

  fsm.add_transition(&stateSettings, &stateEditOption, EVT_TO_EDIT_OPTION, &stateTransition);

  fsm.add_transition(&stateSplash, &stateReset, EVT_TO_RESET, NULL);

  if (digitalRead(BTN_0) == LOW) { //back pressed on startup
    _timer.deleteTimer(0);
    goToReset();
  }
}

void loop() {
  _timer.run();
  MIDI.read();
  btn_up.update();
  btn_down.update();
  btn_back.update();
  btn_ok.update();
}
