/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

/**
 * DWIN LCD
 */

#include "../../../inc/MarlinConfigPre.h"

#if HAS_DWIN_LCD

#include "dwin.h"
#include "..\language\dwin_multi_language.h"

#if ANY(AUTO_BED_LEVELING_BILINEAR, AUTO_BED_LEVELING_LINEAR, AUTO_BED_LEVELING_3POINT) && DISABLED(PROBE_MANUALLY)
  //#define HAS_ONESTEP_LEVELING 1
#endif

#if ANY(BABYSTEPPING, HAS_BED_PROBE, HAS_WORKSPACE_OFFSET)
  #define HAS_ZOFFSET_ITEM 1
#endif

#if !HAS_BED_PROBE && ENABLED(BABYSTEPPING)
  #define JUST_BABYSTEP 1
#endif

#if ENABLED(MIXING_EXTRUDER)
  #include "../../../feature/mixing.h"
#endif

#if ENABLED(OPTION_3DTOUCH)
  #include "../../../feature/bltouch.h"
#endif

#include <WString.h>
#include <stdio.h>
#include <string.h>

#include "../../fontutils.h"
#include "../../ultralcd.h"

#include "../../../sd/cardreader.h"

#include "../../../MarlinCore.h"
#include "../../../core/serial.h"
#include "../../../core/macros.h"
#include "../../../gcode/queue.h"

#include "../../../module/temperature.h"
#include "../../../module/printcounter.h"
#include "../../../module/motion.h"
#include "../../../module/planner.h"

#if ENABLED(EEPROM_SETTINGS)
  #include "../../../module/settings.h"
#endif

#if ENABLED(HOST_ACTION_COMMANDS)
  #include "../../../feature/host_actions.h"
#endif

#if ENABLED(FILAMENT_RUNOUT_SENSOR)
  #include "../../../feature/runout.h"
  #include "../../../feature/pause.h"
#endif

#if HAS_ONESTEP_LEVELING
  #include "../../../feature/bedlevel/bedlevel.h"
#endif

#if HAS_BED_PROBE
  #include "../../../module/probe.h"
#endif

#if EITHER(BABYSTEP_ZPROBE_OFFSET, JUST_BABYSTEP)
  #include "../../../feature/babystep.h"
#endif

#if ENABLED(POWER_LOSS_RECOVERY)
  #include "../../../feature/powerloss.h"
#endif

#ifndef MACHINE_SIZE
  #define MACHINE_SIZE "300x300x400"
#endif

#ifndef CORP_WEBSITE_C
  #define CORP_WEBSITE_C "www.zonestar3d.com"
#endif

#ifndef CORP_WEBSITE_E
  #define CORP_WEBSITE_E "www.zonestar3d.com"
#endif

#define PAUSE_HEAT

#define USE_STRING_HEADINGS

#define DWIN_FONT_MENU font8x16
#define DWIN_FONT_STAT font10x20
#define DWIN_FONT_HEAD font10x20
#define DWIN_FONT_MIX  font14x28

#define MENU_CHAR_LIMIT  24
#define STATUS_Y_START 	 360
#define STATUS_Y_END 	 DWIN_HEIGHT

#define STATUS_MIXER_Y_START 	 State_text_mix_Y
#define STATUS_MIXER_Y_END 	 STATUS_MIXER_Y_START +28

// Fan speed limit
#define FANON           255
#define FANOFF          0

// Print speed limit
#define MAX_PRINT_SPEED   999
#define MIN_PRINT_SPEED   10

// Temp limits
#if HAS_HOTEND
  #define MAX_E_TEMP    (HEATER_0_MAXTEMP - (HOTEND_OVERSHOOT))
  #define MIN_E_TEMP    HEATER_0_MINTEMP
#endif

#if HAS_HEATED_BED
  #define MIN_BED_TEMP  BED_MINTEMP
#endif

// Feedspeed limit (max feedspeed = DEFAULT_MAX_FEEDRATE * 2)
#define MIN_MAXFEEDSPEED      1
#define MIN_MAXACCELERATION   1
#define MIN_MAXJERK           0.1
#define MIN_STEP              1

#define FEEDRATE_E      (60)

// Mininum unit (0.1) : multiple (10)
#define MINUNITMULT     10

#define ENCODER_WAIT    20
#define DWIN_SCROLL_UPDATE_INTERVAL 1000
#define DWIN_REMAIN_TIME_UPDATE_INTERVAL 60000

constexpr uint16_t TROWS = 6, MROWS = TROWS - 1,        // Total rows, and other-than-Back
                   TITLE_HEIGHT = 30,                   // Title bar height
                   MLINE = 53,                          // Menu line height
                   LBLX = 55,                           // Menu item label X
                   MENU_CHR_W = 8, STAT_CHR_W = 10,
				   MENU_CHR_H = 16, STAT_CHR_H = 20;

#define MBASE(L) (49 + MLINE * (L))

#define BABY_Z_VAR TERN(HAS_BED_PROBE, probe.offset.z, dwin_zoffset)

/* Value Init */
HMI_value_t HMI_ValueStruct;
HMI_Flag_t HMI_flag = {0};
FIL_CFG FIL;
MIXER_CFG MixerCfg;
MIXER_DIS MixerDis;

millis_t dwin_heat_time = 0;

uint8_t checkkey = 0;

typedef struct {
  uint8_t now, last;
  void set(uint8_t v) { now = last = v; }
  void reset() { set(0); }
  bool changed() { bool c = (now != last); if (c) last = now; return c; }
  bool dec() { if (now) now--; return changed(); }
  bool inc(uint8_t v) { if (now < (v - 1)) now++; else now = (v - 1); return changed(); }
} select_t;

select_t select_page{0}, select_file{0}, select_print{0}, select_prepare{0}
         , select_control{0}, select_axis{0}, select_temp{0}, select_motion{0}, select_mixer{0}, select_tune{0}
         , select_PLA{0}, select_ABS{0}
         , select_speed{0}
         , select_acc{0}
         , select_jerk{0}
         , select_step{0}
		 , select_manual{0}
		 , select_auto{0}
		 , select_random{0}
		 , select_vtool{0}
		 , select_leveling{0}
		 , select_home{0}
		 , select_option{0}
		 , select_bltouch{0}
		 , select_powerdown{0}
		 , select_language{0}
         ;

uint8_t index_file     	= MROWS,
        index_prepare  	= MROWS,
        index_control  	= MROWS,
        index_leveling 	= MROWS,
        index_home	 	= MROWS,
        index_tune     	= MROWS,
		index_axismove  = MROWS,
		index_manual  	= MROWS,
		index_auto  	= MROWS,
		index_random  	= MROWS,
		index_bltouch  	= MROWS,
        index_language  = MROWS;

bool dwin_abort_flag = false; // Flag to reset feedrate, return to Home

constexpr float default_max_feedrate[]        = DEFAULT_MAX_FEEDRATE;
constexpr float default_max_acceleration[]    = DEFAULT_MAX_ACCELERATION;
constexpr float default_max_jerk[]            = { DEFAULT_XJERK, DEFAULT_YJERK, DEFAULT_ZJERK, DEFAULT_EJERK };
constexpr float default_axis_steps_per_unit[] = DEFAULT_AXIS_STEPS_PER_UNIT;

uint8_t Percentrecord = 0;
uint16_t remain_time = 0;

#if ENABLED(PAUSE_HEAT)
  #if HAS_HOTEND
    uint16_t temphot = 0;
  #endif
  #if HAS_HEATED_BED
    uint16_t tempbed = 0;
  #endif
#endif

#if HAS_ZOFFSET_ITEM
  float dwin_zoffset = 0, last_zoffset = 0;
#endif

#ifdef IIC_BL24CXX_EEPROM
#define DWIN_LANGUAGE_EEPROM_ADDRESS 0x01   // Between 0x01 and 0x63 (EEPROM_OFFSET-1)
                                            // BL24CXX::check() uses 0x00
#else
#define DWIN_LANGUAGE_EEPROM_ADDRESS 80    
#endif
/*
inline bool HMI_IsChinese() { return HMI_flag.language == DWIN_CHINESE; }
*/
void HMI_SetLanguageCache() {
  //DWIN_JPG_CacheTo1(HMI_IsChinese() ? Language_Chinese : Language_English);
  //HMI_flag.language = 2;
  DWIN_JPG_CacheToN(1,HMI_flag.language + 1);
  if(HMI_flag.language < 3) HMI_flag.Title_Menu_Bankup = 7;
  else HMI_flag.Title_Menu_Bankup = 6;
  //DWIN_JPG_CacheToN(1,8);
}

void HMI_SetLanguage() {
/*
  #if ENABLED(EEPROM_SETTINGS)
    HMI_flag.language = DWIN_ENGLISH;
	#ifdef IIC_BL24CXX_EEPROM
    BL24CXX::read(DWIN_LANGUAGE_EEPROM_ADDRESS, (uint8_t*)&HMI_flag.language, sizeof(HMI_flag.language));
	#else
	//EEPROM_READ();
	#endif
  #endif
  */
  HMI_SetLanguageCache();
}

void HMI_ToggleLanguage() {
  //HMI_flag.language = HMI_IsChinese() ? DWIN_ENGLISH : DWIN_CHINESE;
  //settings.save();
  HMI_SetLanguageCache();
  /*
  #if ENABLED(EEPROM_SETTINGS)
    #ifdef IIC_BL24CXX_EEPROM
    BL24CXX::write(DWIN_LANGUAGE_EEPROM_ADDRESS, (uint8_t*)&HMI_flag.language, sizeof(HMI_flag.language));
	#else
	uint16_t working_crc = 0;
	//EEPROM_WRITE();
	#endif
  #endif
  */
}

unsigned int GenRandomString(int length)
{
   int i;
   srand(millis());   
   for (i = 0; i < length; i++)
   {          
      return rand()%length;  
   }
   return 0;
}

uint8_t  Check_Percent_equal(){
	for(uint8_t i=0; i<MIXING_STEPPERS-1; i++) {
		if(MixerCfg.Manual_Percent[mixer.selected_vtool][i] != MixerCfg.Manual_Percent[mixer.selected_vtool][i+1]) return 1;
	}
	return 0;
}

void updata_mixer_from_vtool(){
	float ctot = 0;
	int16_t sum_mix = 0;
    MIXER_STEPPER_LOOP(i) ctot += mixer.color[mixer.selected_vtool][i];
    MIXER_STEPPER_LOOP(i) mixer.mix[i] = (int8_t)(100.0f * mixer.color[mixer.selected_vtool][i] / ctot);
	for(uint8_t i=0; i<MIXING_STEPPERS-1; i++) sum_mix += mixer.mix[i];
	mixer.mix[MIXING_STEPPERS-1] = 100 - sum_mix;
}

void recalculation_mixer_percent(){
    uint16_t sum_mix = 0;
	MIXER_STEPPER_LOOP(i) sum_mix+=mixer.mix[i];
	const float scaleMix = 100/sum_mix;
	MIXER_STEPPER_LOOP(i) mixer.mix[i] *= scaleMix;
	sum_mix = 0;
	for(uint8_t i=0; i<MIXING_STEPPERS-1; i++) sum_mix += mixer.mix[i];
	mixer.mix[MIXING_STEPPERS-1] = 100 - sum_mix;
}

void DWIN_Draw_Signed_Float(uint8_t size, uint16_t bColor, uint8_t iNum, uint8_t fNum, uint16_t x, uint16_t y, long value) {
  if (value < 0) {
    DWIN_Draw_String(false, true, size, Color_White, bColor, x - 6, y, F("-"));
    DWIN_Draw_FloatValue(true, true, 0, size, Color_White, bColor, iNum, fNum, x, y, -value);
  }
  else {
    DWIN_Draw_String(false, true, size, Color_White, bColor, x - 6, y, F(" "));
    DWIN_Draw_FloatValue(true, true, 0, size, Color_White, bColor, iNum, fNum, x, y, value);
  }
}

void ICON_Print() {
  uint16_t Coordinate_Temp;
  Coordinate_Temp = Print_X_Coordinate[HMI_flag.language];
  if (select_page.now == 0) {
    DWIN_ICON_Show(ICON, ICON_Print_1, 17, 130);
    DWIN_Draw_Rectangle(0, Color_White, 17, 130, 126, 229);
  	//DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Picture_Print, Menu_Coordinate,53,201);
  	DWIN_ICON_Show(ICON, Picture_Coordinate[HMI_flag.language][Picture_Print], Coordinate_Temp, 201);
  }
  else {
    DWIN_ICON_Show(ICON, ICON_Print_0, 17, 130);
	DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Main_Menu_Print0, Menu_Coordinate, Coordinate_Temp,201);
  }
  DWIN_UpdateLCD();
}

void ICON_Prepare() {
  uint16_t Coordinate_Temp;
  Coordinate_Temp = Prepare_X_Coordinate[HMI_flag.language];
  if (select_page.now == 1) {
    DWIN_ICON_Show(ICON, ICON_Prepare_1, 145, 130);
    DWIN_Draw_Rectangle(0, Color_White, 145, 130, 254, 229);
  	//DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Picture_Prepare, Menu_Coordinate,172,201);
  	DWIN_ICON_Show(ICON, Picture_Coordinate[HMI_flag.language][Picture_Prepare], Coordinate_Temp, 201);
  }
  else {
    DWIN_ICON_Show(ICON, ICON_Prepare_0, 145, 130);
	DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Main_Menu_Prepare0, Menu_Coordinate, Coordinate_Temp,201);
  }
  DWIN_UpdateLCD();
}

void ICON_Control() {
  uint16_t Coordinate_Temp;
  Coordinate_Temp = Control_X_Coordinate[HMI_flag.language];
  if (select_page.now == 2) {
    DWIN_ICON_Show(ICON, ICON_Control_1, 17, 246);
    DWIN_Draw_Rectangle(0, Color_White, 17, 246, 126, 345);
  	//DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Picture_Control, Menu_Coordinate,44,318);
  	DWIN_ICON_Show(ICON, Picture_Coordinate[HMI_flag.language][Picture_Control], Coordinate_Temp, 318);
  }
  else {
    DWIN_ICON_Show(ICON, ICON_Control_0, 17, 246);
	DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Main_Menu_Control0, Menu_Coordinate, Coordinate_Temp,318);
  }
  DWIN_UpdateLCD();
}

void ICON_StartInfo(bool show) {
  uint16_t Coordinate_Temp;
  Coordinate_Temp = StartInfo_X_Coordinate[HMI_flag.language];
  if (show) {
    DWIN_ICON_Show(ICON, ICON_Info_1, 145, 246);
    DWIN_Draw_Rectangle(0, Color_White, 145, 246, 254, 345);
  	//DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Picture_StartInfo, Menu_Coordinate,184,318);
  	DWIN_ICON_Show(ICON, Picture_Coordinate[HMI_flag.language][Picture_StartInfo], Coordinate_Temp, 318);
  }
  else {
    DWIN_ICON_Show(ICON, ICON_Info_0, 145, 246);
	DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Main_Menu_StartInfo0, Menu_Coordinate, Coordinate_Temp,318);
  }
  DWIN_UpdateLCD();
}

void ICON_Leveling(bool show) {
  if (show) {
    DWIN_ICON_Show(ICON, ICON_Leveling_1, 145, 246);
    DWIN_Draw_Rectangle(0, Color_White, 145, 246, 254, 345);
  /*
    if (HMI_IsChinese())
      DWIN_Frame_AreaCopy(1, 211, 447, 238, 460, 186, 318);
    else
		
      DWIN_Frame_AreaCopy(1, 84, 437, 120,  449, 182, 318);
  }
  else {
  	
    DWIN_ICON_Show(ICON, ICON_Leveling_0, 145, 246);
    if (HMI_IsChinese())
      DWIN_Frame_AreaCopy(1, 211, 405, 238, 420, 186, 318);
    else
		*/
      DWIN_Frame_AreaCopy(1, 84, 465, 120, 478, 182, 318);
  }
}

void ICON_Tune() {
  if (select_print.now == 0) {
    DWIN_ICON_Show(ICON, ICON_Setup_1, 8, 252);
    DWIN_Draw_Rectangle(0, Color_White, 8, 252, 87, 351);
    //DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Picture_Tune, Menu_Coordinate,32, 325);
    DWIN_ICON_Show(ICON, Picture_Coordinate[HMI_flag.language][Picture_Tune], Tune_X_Coordinate[HMI_flag.language], 325);
  }
  else {
    DWIN_ICON_Show(ICON, ICON_Setup_0, 8, 252);
	DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Printing_Menu_Tune, Menu_Coordinate,Tune_X_Coordinate[HMI_flag.language], 325);
  }
  DWIN_UpdateLCD();
}

void ICON_Pause() {
  if ((select_print.now == 1)) {
    DWIN_ICON_Show(ICON, ICON_Pause_1, 96, 252);
    DWIN_Draw_Rectangle(0, Color_White, 96, 252, 175, 351);
    //DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Picture_Pause, Menu_Coordinate,116, 325);
    DWIN_ICON_Show(ICON, Picture_Coordinate[HMI_flag.language][Picture_Pause], Pause_X_Coordinate[HMI_flag.language], 325);
  }
  else {
    DWIN_ICON_Show(ICON, ICON_Pause_0, 96, 252);
	DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Printing_Menu_Pause, Menu_Coordinate,Pause_X_Coordinate[HMI_flag.language], 325);
  }
  DWIN_UpdateLCD();
}

void ICON_Continue() {
  if (select_print.now == 1) {
    DWIN_ICON_Show(ICON, ICON_Continue_1, 96, 252);
    DWIN_Draw_Rectangle(0, Color_White, 96, 252, 175, 351);
    //DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Picture_Print, Menu_Coordinate,116, 325);
    DWIN_ICON_Show(ICON, Picture_Coordinate[HMI_flag.language][Picture_Continue], Continue_X_Coordinate[HMI_flag.language], 325);
  }
  else {
    DWIN_ICON_Show(ICON, ICON_Continue_0, 96, 252);
	DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Printing_Menu_Continue, Menu_Coordinate,Continue_X_Coordinate[HMI_flag.language], 325);
  }
  DWIN_UpdateLCD();
}

void ICON_Stop() {
  if (select_print.now == 2) {
    DWIN_ICON_Show(ICON, ICON_Stop_1, 184, 252);
    DWIN_Draw_Rectangle(0, Color_White, 184, 252, 263, 351);
    //DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Picture_Stop, Menu_Coordinate,208, 325);
    DWIN_ICON_Show(ICON, Picture_Coordinate[HMI_flag.language][Picture_Stop], Stop_X_Coordinate[HMI_flag.language], 325);
  }
  else {
    DWIN_ICON_Show(ICON, ICON_Stop_0, 184, 252);
	DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Printing_Menu_Stop, Menu_Coordinate,Stop_X_Coordinate[HMI_flag.language], 325);
  }
  DWIN_UpdateLCD();
}

void ICON_YESorNO(uint8_t Option){
	if (Option == false) {
    	DWIN_ICON_Show(ICON, ICON_YES_0, 26, 168);
		DWIN_ICON_Show(ICON, ICON_NO_1, 146, 168);
    	DWIN_Draw_Rectangle(0, Color_White, 26, 168, 126, 206);
  	}
	else{
		DWIN_ICON_Show(ICON, ICON_YES_1, 26, 168);
		DWIN_ICON_Show(ICON, ICON_NO_0, 146, 168);
    	DWIN_Draw_Rectangle(0, Color_White, 146, 168, 246, 206);
	}
}

void ICON_YESorNO_Powerdown(uint8_t Option){
	if (Option == false) {
    	DWIN_ICON_Show(ICON, ICON_NO_0, 26, 228);
		DWIN_ICON_Show(ICON, ICON_YES_1, 146, 228);
    	DWIN_Draw_Rectangle(0, Color_White, 26, 228, 126, 266);
  	}
	else{
		DWIN_ICON_Show(ICON, ICON_NO_1, 26, 228);
		DWIN_ICON_Show(ICON, ICON_YES_0, 146, 228);
    	DWIN_Draw_Rectangle(0, Color_White, 146, 228, 246, 266);
	}
}


inline void Clear_Title_Bar() {
  DWIN_Draw_Rectangle(1, Color_Bg_Blue, 0, 0, DWIN_WIDTH, 30);
}

inline void Draw_Title(const char * const title) {
  DWIN_Draw_String(false, false, DWIN_FONT_HEAD, Color_White, Color_Bg_Blue, 14, 4, (char*)title);
}

inline void Draw_Title(const __FlashStringHelper * title) {
  DWIN_Draw_String(false, false, DWIN_FONT_HEAD, Color_White, Color_Bg_Blue, 14, 4, (char*)title);
}

inline void Clear_Menu_Area() {
  DWIN_Draw_Rectangle(1, Color_Bg_Black, 0, 31, DWIN_WIDTH, STATUS_Y_START);
}

inline void Clear_Main_Window() {
  Clear_Title_Bar();
  Clear_Menu_Area();
}

inline void Clear_Popup_Area() {
  Clear_Title_Bar();
  DWIN_Draw_Rectangle(1, Color_Bg_Black, 0, 31, DWIN_WIDTH, DWIN_HEIGHT);
}

void Draw_Popup_Bkgd_105() {
  DWIN_Draw_Rectangle(1, Color_Bg_Window, 14, 105, 258, 374);
}

inline void Draw_More_Icon(const uint8_t line) {
  DWIN_ICON_Show(ICON, ICON_More, 226, MBASE(line) - 3);
}

inline void Draw_Menu_Cursor(const uint8_t line) {
  // DWIN_ICON_Show(ICON,ICON_Rectangle, 0, MBASE(line) - 18);
  DWIN_Draw_Rectangle(1, Rectangle_Color, 0, MBASE(line) - 18, 14, MBASE(line + 1) - 20);
}

inline void Erase_Menu_Cursor(const uint8_t line) {
  DWIN_Draw_Rectangle(1, Color_Bg_Black, 0, MBASE(line) - 18, 14, MBASE(line + 1) - 20);
}

inline void Move_Highlight(const int16_t from, const uint16_t newline) {
  Erase_Menu_Cursor(newline - from);
  Draw_Menu_Cursor(newline);
}

inline void Add_Menu_Line() {
  Move_Highlight(1, MROWS);
  DWIN_Draw_Line(Line_Color, 16, MBASE(MROWS + 1) - 20, 256, MBASE(MROWS + 1) - 19);
}

inline void Scroll_Menu(const uint8_t dir) {
  DWIN_Frame_AreaMove(1, dir, MLINE, Color_Bg_Black, 0, 31, DWIN_WIDTH, 349);
  switch (dir) {
    case DWIN_SCROLL_DOWN: Move_Highlight(-1, 0); break;
    case DWIN_SCROLL_UP:   Add_Menu_Line(); break;
  }
}

inline uint16_t nr_sd_menu_items() {
  return card.get_num_Files() + !card.flag.workDirIsRoot;
}

inline void Draw_Menu_Icon(const uint8_t line, const uint8_t icon) {
  DWIN_ICON_Show(ICON, icon, 26, MBASE(line) - 3);
}

inline void Erase_Menu_Text(const uint8_t line) {
  DWIN_Draw_Rectangle(1, Color_Bg_Black, LBLX, MBASE(line) - 14, 271, MBASE(line) + 28);
}

inline void Draw_Menu_Line(const uint8_t line, const uint8_t icon=0, const char * const label=nullptr) {
  if (label) DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, LBLX, MBASE(line) - 1, (char*)label);
  if (icon) Draw_Menu_Icon(line, icon);
  DWIN_Draw_Line(Line_Color, 16, MBASE(line) + 33, 256, MBASE(line) + 34);
}

// The "Back" label is always on the first line
inline void Draw_Back_Label() {
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Menu_Back, Menu_Coordinate,LBLX, MBASE(0));
  DWIN_UpdateLCD();
}

// Draw "Back" line at the top
inline void Draw_Back_First(const bool is_sel=true) {
  Draw_Menu_Line(0, ICON_Back);
  Draw_Back_Label();
  if (is_sel) Draw_Menu_Cursor(0);
}

inline bool Apply_Encoder(const ENCODER_DiffState &encoder_diffState, auto &valref) {
  if (encoder_diffState == ENCODER_DIFF_CW)
    valref += EncoderRate.encoderMoveValue;
  else if (encoder_diffState == ENCODER_DIFF_CCW)
    valref -= EncoderRate.encoderMoveValue;
  else if (encoder_diffState == ENCODER_DIFF_ENTER)
    return true;
  return false;
}

//
// Draw Menus
//
#define MOTION_CASE_RATE   1
#define MOTION_CASE_ACCEL  2
#define MOTION_CASE_JERK   (MOTION_CASE_ACCEL + ENABLED(HAS_CLASSIC_JERK))
#define MOTION_CASE_STEPS  (MOTION_CASE_JERK + 1)
#define MOTION_CASE_TOTAL  MOTION_CASE_STEPS

#define PREPARE_CASE_MOVE  		1
#define PREPARE_CASE_DISA  		2
#define PREPARE_CASE_HOME  		3
#define PREPARE_CASE_LEVELING  	4
#define PREPARE_CASE_POWERDOWN  5
#define PREPARE_CASE_ZOFF (PREPARE_CASE_POWERDOWN + ENABLED(HAS_ZOFFSET_ITEM))
#define PREPARE_CASE_PLA  (PREPARE_CASE_ZOFF + ENABLED(HAS_HOTEND))
#define PREPARE_CASE_ABS  (PREPARE_CASE_PLA + ENABLED(HAS_HOTEND))
#define PREPARE_CASE_COOL (PREPARE_CASE_ABS + EITHER(HAS_HOTEND, HAS_HEATED_BED))
#define PREPARE_CASE_LANG (PREPARE_CASE_COOL + 1)
#define PREPARE_CASE_TOTAL PREPARE_CASE_LANG

#define CONTROL_CASE_TEMP 	1
#define CONTROL_CASE_MOVE   2
#define CONTROL_CASE_MIXER  3
#define CONTROL_CASE_BLTOUCH (CONTROL_CASE_MIXER + ENABLED(BLTOUCH))
#define CONTROL_CASE_SAVE  (CONTROL_CASE_BLTOUCH + ENABLED(EEPROM_SETTINGS))
#define CONTROL_CASE_LOAD  (CONTROL_CASE_SAVE + ENABLED(EEPROM_SETTINGS))
#define CONTROL_CASE_RESET (CONTROL_CASE_LOAD + ENABLED(EEPROM_SETTINGS))
#define CONTROL_CASE_INFO  (CONTROL_CASE_RESET + 1)
#define CONTROL_CASE_TOTAL CONTROL_CASE_INFO

#define TUNE_CASE_SPEED 1
#define TUNE_CASE_TEMP (TUNE_CASE_SPEED + ENABLED(HAS_HOTEND))
#define TUNE_CASE_BED  (TUNE_CASE_TEMP + ENABLED(HAS_HEATED_BED))
#define TUNE_CASE_FAN  (TUNE_CASE_BED + ENABLED(HAS_FAN))
#define TUNE_CASE_ZOFF (TUNE_CASE_FAN + ENABLED(HAS_ZOFFSET_ITEM))
#define TUNE_CASE_MIXER (TUNE_CASE_ZOFF + 1)
#define TUNE_CASE_TOTAL TUNE_CASE_MIXER

#define TEMP_CASE_TEMP (0 + ENABLED(HAS_HOTEND))
#define TEMP_CASE_BED  (TEMP_CASE_TEMP + ENABLED(HAS_HEATED_BED))
#define TEMP_CASE_FAN  (TEMP_CASE_BED + ENABLED(HAS_FAN))
#define TEMP_CASE_PLA  (TEMP_CASE_FAN + ENABLED(HAS_HOTEND))
#define TEMP_CASE_ABS  (TEMP_CASE_PLA + ENABLED(HAS_HOTEND))
#define TEMP_CASE_TOTAL TEMP_CASE_ABS

#define PREHEAT_CASE_TEMP (0 + ENABLED(HAS_HOTEND))
#define PREHEAT_CASE_BED  (PREHEAT_CASE_TEMP + ENABLED(HAS_HEATED_BED))
#define PREHEAT_CASE_FAN  (PREHEAT_CASE_BED + ENABLED(HAS_FAN))
#define PREHEAT_CASE_SAVE (PREHEAT_CASE_FAN + ENABLED(EEPROM_SETTINGS))
#define PREHEAT_CASE_TOTAL PREHEAT_CASE_SAVE

#define BLTOUCH_CASE_RESET   	1
#define BLTOUCH_CASE_TEST  		2
#define BLTOUCH_CASE_STOW   	3
#define BLTOUCH_CASE_DEPLOY    	4
#define BLTOUCH_CASE_SW    		5
#define BLTOUCH_CASE_TOTAL  	BLTOUCH_CASE_SW

#define LANGUAGE_CASE_EN   1
#define LANGUAGE_CASE_SP  	2
#define LANGUAGE_CASE_RU   3
#define LANGUAGE_CASE_FR    4
#define LANGUAGE_CASE_PO    5
#define LANGUAGE_CASE_ZH  	6
#define LANGUAGE_CASE_TOTAL LANGUAGE_CASE_PO

#define MIXER_CASE_MANUAL   1
#define MIXER_CASE_AUTO  	2
#define MIXER_CASE_RANDOM   3
#define MIXER_CASE_VTOOL    4
#define MIXER_CASE_TOTAL  	MIXER_CASE_VTOOL

#if ENABLED(MIXING_EXTRUDER)
  	#if(MIXING_STEPPERS == 4) 
		#define MANUAL_CASE_EXTRUDER1   1
		#define MANUAL_CASE_EXTRUDER2  	2
		#define MANUAL_CASE_EXTRUDER3   3
		#define MANUAL_CASE_EXTRUDER4  	4
		#define MANUAL_CASE_OK  		5
		#define MANUAL_CASE_TOTAL       MANUAL_CASE_OK
	#elif(MIXING_STEPPERS == 3) 
		#define MANUAL_CASE_EXTRUDER1   1
		#define MANUAL_CASE_EXTRUDER2  	2
		#define MANUAL_CASE_EXTRUDER3   3
		#define MANUAL_CASE_OK  		4
		#define MANUAL_CASE_TOTAL       MANUAL_CASE_OK
	#elif(MIXING_STEPPERS == 2) 
		#define MANUAL_CASE_EXTRUDER1   1
		#define MANUAL_CASE_EXTRUDER2  	2
		#define MANUAL_CASE_OK  		3
		#define MANUAL_CASE_TOTAL       MANUAL_CASE_OK
	#else
	    #define MANUAL_CASE_EXTRUDER1   1
		#define MANUAL_CASE_OK  		2
		#define MANUAL_CASE_TOTAL       MANUAL_CASE_OK
	#endif
#endif

#define AUTO_CASE_ZPOS_START  	1
#define AUTO_CASE_ZPOS_END  	2
#define AUTO_CASE_VTOOL_START  	3
#define AUTO_CASE_VTOOL_END  	4
#define AUTO_CASE_TOTAL       	AUTO_CASE_VTOOL_END

#define RANDOM_CASE_ZPOS_START  1
#define RANDOM_CASE_ZPOS_END  	2
#define RANDOM_CASE_TOTAL       RANDOM_CASE_ZPOS_END


#define AXISMOVE_CASE_MOVEX   	1
#define AXISMOVE_CASE_MOVEY  	2
#define AXISMOVE_CASE_MOVEZ   	3
#if ENABLED(MIXING_EXTRUDER)
  #if(MIXING_STEPPERS == 4) 
	#define AXISMOVE_CASE_EX1  		4
	#define AXISMOVE_CASE_EX2  		5
	#define AXISMOVE_CASE_EX3  		6
	#define AXISMOVE_CASE_EX4  		7
	#define AXISMOVE_CASE_TOTAL     AXISMOVE_CASE_EX4
  #elif(MIXING_STEPPERS == 3) 
	#define AXISMOVE_CASE_EX1  		4
	#define AXISMOVE_CASE_EX2  		5
	#define AXISMOVE_CASE_EX3  		6
	#define AXISMOVE_CASE_TOTAL     AXISMOVE_CASE_EX3
  #elif(MIXING_STEPPERS == 2) 
	#define AXISMOVE_CASE_EX1  		4
	#define AXISMOVE_CASE_EX2  		5
	#define AXISMOVE_CASE_TOTAL     AXISMOVE_CASE_EX2
  #endif
#else
  #define AXISMOVE_CASE_EX1  		4
  #define AXISMOVE_CASE_TOTAL     AXISMOVE_CASE_EX1
#endif

#define LEVELING_CASE_POINT1   1
#define LEVELING_CASE_POINT2  	2
#define LEVELING_CASE_POINT3   3
#define LEVELING_CASE_POINT4    4
#define LEVELING_CASE_AUTO    	(LEVELING_CASE_POINT4 + ENABLED(LCD_BED_LEVELING))
#define LEVELING_CASE_SAVE    	(LEVELING_CASE_AUTO + ENABLED(LCD_BED_LEVELING))
#define LEVELING_CASE_TOTAL  	LEVELING_CASE_SAVE

#define HOME_CASE_ALL   1
#define HOME_CASE_X  	2
#define HOME_CASE_Y   3
#define HOME_CASE_Z    4
#define HOME_CASE_TOTAL  	HOME_CASE_Z

/*
//
// Draw Menus
//
inline void draw_move_en(const uint16_t line) {
  DWIN_Frame_AreaCopy(1, 69, 61, 102, 71, LBLX, line); 			// "Move"
}
*/
inline void DWIN_Frame_TitleCopy(uint8_t id, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) { DWIN_Frame_AreaCopy(id, x1, y1, x2, y2, 14, 8); }

inline void Item_Leveling_Save(const uint8_t row) {
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Leveling_Menu_Bed, Menu_Coordinate,LBLX, MBASE(row));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Leveling_Menu_Level, Menu_Coordinate,LBLX+Bed_X_Coordinate[HMI_flag.language], MBASE(row));
  Draw_Menu_Line(row, ICON_Leveling_Save);
}

inline void Item_Leveling_Point1(const uint8_t row) {
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Leveling_Menu_Point, Menu_Coordinate,LBLX, MBASE(row));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Leveling_Menu_1, Menu_Coordinate,LBLX+47, MBASE(row));
  Draw_Menu_Line(row, ICON_Leveling_Point1);
}

inline void Item_Control_Temp(const uint8_t row) {
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Control_Menu_Temperature, Menu_Coordinate,LBLX, MBASE(row));
  Draw_Menu_Line(row, ICON_Temperature);
  Draw_More_Icon(row);
}

inline void Item_Control_Motion(const uint8_t row) {
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Control_Menu_Motion, Menu_Coordinate,LBLX, MBASE(row));
  Draw_Menu_Line(row, ICON_Motion);
  Draw_More_Icon(row);
}

#if ENABLED(BLTOUCH)
inline void Item_Control_Save(const uint8_t row) {
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Control_Menu_Store, Menu_Coordinate,LBLX, MBASE(row));
  Draw_Menu_Line(row, ICON_WriteEEPROM);
  Draw_More_Icon(row);
}

inline void Item_Control_Load(const uint8_t row) {
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Control_Menu_Load, Menu_Coordinate,LBLX, MBASE(row));
  //DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Control_Menu_Settings, Menu_Coordinate,LBLX+Load_X_Coordinate[HMI_flag.language], MBASE(row));
  Draw_Menu_Line(row, ICON_ReadEEPROM);
}
#endif

inline void Item_Control_Reset(const uint8_t row) {
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Control_Menu_Reset, Menu_Coordinate,LBLX, MBASE(row));
  Draw_Menu_Line(row, ICON_ResumeEEPROM);
}

inline void Item_Control_Info(const uint8_t row) {
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Control_Menu_Info, Menu_Coordinate,LBLX, MBASE(row));
  Draw_Menu_Line(row, ICON_Info);
  Draw_More_Icon(row);
}

inline void Item_Language_ZH(const uint8_t row) {
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Language_Menu_ZH, Menu_Coordinate,LBLX, MBASE(row));
  Draw_Menu_Line(row, ICON_CH);
}

inline void Item_Language_EN(const uint8_t row) {
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Language_Menu_EN, Menu_Coordinate,LBLX, MBASE(row));
  Draw_Menu_Line(row, ICON_EN);
}

inline void Item_Tune_Mixer(const uint8_t row) {
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Tune_Menu_Mixer, Menu_Coordinate,LBLX, MBASE(row));
  Draw_Menu_Line(row, ICON_Mixer);
  Draw_More_Icon(row);
}

inline void Item_Tune_Speed(const uint8_t row) {
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Tune_Menu_Speed, Menu_Coordinate,LBLX, MBASE(row));
  Draw_Menu_Line(row, ICON_Setspeed);
  DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Select_Color, 3, 216, MBASE(row), feedrate_percentage);
}

inline void Item_Axis_MoveX(const uint8_t row) {
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Move_Menu_Move, Menu_Coordinate,LBLX, MBASE(row));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Move_Menu_X, Menu_Coordinate,LBLX+Move_X_Coordinate[HMI_flag.language], MBASE(row));
  Draw_Menu_Line(row, ICON_MoveX);
  DWIN_Draw_FloatValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 1, 216, MBASE(row), current_position.x * MINUNITMULT);
}

inline void Item_Axis_MoveY(const uint8_t row) {
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Move_Menu_Move, Menu_Coordinate,LBLX, MBASE(row));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Move_Menu_Y, Menu_Coordinate,LBLX+Move_X_Coordinate[HMI_flag.language], MBASE(row));
  Draw_Menu_Line(row, ICON_MoveY);
  DWIN_Draw_FloatValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 1, 216, MBASE(row), current_position.y * MINUNITMULT);
}
/*
inline void Item_Axis_MoveZ(const uint8_t row) {
  if (HMI_IsChinese())
    DWIN_Frame_AreaCopy(1, 159, 70, 200, 84, LBLX, MBASE(row));
  else
    DWIN_Frame_AreaCopy(1, 112, 104, 120, 114, LBLX, MBASE(row)); 		// "MoveZ"

  Draw_Menu_Line(row, ICON_MoveZ);
}
*/
#if ENABLED (MIXING_EXTRUDER)
	#if(MIXING_STEPPERS == 4 || MIXING_STEPPERS == 3)
	inline void Item_Axis_MoveEX3(const uint8_t row) {
    DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Move_Menu_Extruder, Menu_Coordinate,LBLX, MBASE(row));
    DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Move_Menu_3, Menu_Coordinate,LBLX+Extruder_X_Coordinate[HMI_flag.language], MBASE(row));
  	Draw_Menu_Line(row, ICON_Extruder3);
  	DWIN_Draw_Signed_Float(font8x16, Color_Bg_Black, 3, 1, 216, MBASE(row), HMI_ValueStruct.Move_E3_scale);
	}
	#endif
#endif

#if ENABLED (MIXING_EXTRUDER)
	#if(MIXING_STEPPERS == 4)
	inline void Item_Axis_MoveEX4(const uint8_t row) {
    DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Move_Menu_Extruder, Menu_Coordinate,LBLX, MBASE(row));
    DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Move_Menu_4, Menu_Coordinate,LBLX+Extruder_X_Coordinate[HMI_flag.language], MBASE(row));
  	Draw_Menu_Line(row, ICON_Extruder4);
  	DWIN_Draw_Signed_Float(font8x16, Color_Bg_Black, 3, 1, 216, MBASE(row), HMI_ValueStruct.Move_E4_scale);
	}
	#endif
#endif
/*
inline void Item_Auto_EX1_Percent(const uint8_t row) {
  if (HMI_IsChinese())
    DWIN_Frame_AreaCopy(1, 159, 70, 200, 84, LBLX, MBASE(row));
  else
    DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, LBLX, MBASE(row), GET_TEXT_F(MSG_MOVE_E1));

  Draw_Menu_Line(row, ICON_Extruder3);
  DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(row), MixerCfg.Auto_Percent[mixer.selected_vtool][0]);
}

inline void Item_Auto_EX2_Percent(const uint8_t row) {
  if (HMI_IsChinese())
    DWIN_Frame_AreaCopy(1, 159, 70, 200, 84, LBLX, MBASE(row));
  else
    DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, LBLX, MBASE(row), GET_TEXT_F(MSG_MOVE_E2));

  Draw_Menu_Line(row, ICON_Extruder3);
  DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(row), MixerCfg.Auto_Percent[mixer.selected_vtool][1]);
}

inline void Item_Auto_EX3_Percent(const uint8_t row) {
  if (HMI_IsChinese())
    DWIN_Frame_AreaCopy(1, 159, 70, 200, 84, LBLX, MBASE(row));
  else
    DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, LBLX, MBASE(row), GET_TEXT_F(MSG_MOVE_E3));

  Draw_Menu_Line(row, ICON_Extruder3);
  DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(row), MixerCfg.Auto_Percent[mixer.selected_vtool][2]);
}

inline void Item_Auto_EX4_Percent(const uint8_t row) {
  if (HMI_IsChinese())
    DWIN_Frame_AreaCopy(1, 159, 70, 200, 84, LBLX, MBASE(row));
  else
    DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, LBLX, MBASE(row), GET_TEXT_F(MSG_MOVE_E4));

  Draw_Menu_Line(row, ICON_Extruder3);
  DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(row), MixerCfg.Auto_Percent[mixer.selected_vtool][3]);
  
}
*/
/*
inline void Item_Auto_Zpos_Start(const uint8_t row) {
  if (HMI_IsChinese())
    DWIN_Frame_AreaCopy(1, 159, 70, 200, 84, LBLX, MBASE(row));
  else
    DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, LBLX, MBASE(row), GET_TEXT_F(MSG_START_Z));

  //Draw_Menu_Line(row, ICON_Zpos_Start);
  Draw_Menu_Line(row, ICON_MoveX);
  HMI_ValueStruct.Auto_Zstart_scale = MixerCfg.Auto_Zpos[ZPOS_START]*MINUNITMULT;
  DWIN_Draw_Signed_Float(font8x16, Color_Bg_Black, 3, 1, 216, MBASE(row), HMI_ValueStruct.Auto_Zstart_scale);
  //DWIN_Draw_Signed_Float(font8x16, Color_Bg_Black, 4, 1, 192, MBASE(row), HMI_ValueStruct.Move_Zstart_scale);
  //DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(row), MixerCfg.Auto_Zpos[ZPOS_START]);
}

inline void Item_Auto_Zpos_End(const uint8_t row) {
  if (HMI_IsChinese())
    DWIN_Frame_AreaCopy(1, 159, 70, 200, 84, LBLX, MBASE(row));
  else
    DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, LBLX, MBASE(row), GET_TEXT_F(MSG_END_Z));

  //Draw_Menu_Line(row, ICON_Zpos_End);
  Draw_Menu_Line(row, ICON_MoveY);
  HMI_ValueStruct.Auto_Zend_scale = MixerCfg.Auto_Zpos[ZPOS_END]*MINUNITMULT;
  DWIN_Draw_Signed_Float(font8x16, Color_Bg_Black, 3, 1, 216, MBASE(row), HMI_ValueStruct.Auto_Zend_scale);
  //DWIN_Draw_Signed_Float(font8x16, Color_Bg_Black, 4, 1, 192, MBASE(row), HMI_ValueStruct.Move_Zend_scale);
  //DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(row), MixerCfg.Auto_Zpos[ZPOS_END]);
}

inline void Item_Auto_VTool_Start(const uint8_t row) {
  if (HMI_IsChinese())
    DWIN_Frame_AreaCopy(1, 159, 70, 200, 84, LBLX, MBASE(row));
  else
    DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, LBLX, MBASE(row), GET_TEXT_F(MSG_START_VTOOL));

  //Draw_Menu_Line(row, ICON_VTool_Start);
  Draw_Menu_Line(row, ICON_MoveX);
  DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(row), MixerCfg.Auto_VTool[VTOOL_START]);
}


inline void Item_Auto_VTool_End(const uint8_t row) {
  if (HMI_IsChinese())
    DWIN_Frame_AreaCopy(1, 159, 70, 200, 84, LBLX, MBASE(row));
  else
    DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, LBLX, MBASE(row), GET_TEXT_F(MSG_END_VTOOL));

  //Draw_Menu_Line(row, ICON_VTool_End);
  Draw_Menu_Line(row, ICON_MoveY);
  DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(row), MixerCfg.Auto_VTool[VTOOL_END]);
}
*/

inline void Item_Prepare_Move(const uint8_t row) {
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Prepare_Menu_Move, Menu_Coordinate,LBLX, MBASE(row));
  Draw_Menu_Line(row, ICON_Axis);
  Draw_More_Icon(row);
}

inline void Item_Prepare_Disable(const uint8_t row) {
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Prepare_Menu_Disable_Steppers, Menu_Coordinate,LBLX, MBASE(row));
  Draw_Menu_Line(row, ICON_CloseMotor);
}

inline void Item_Prepare_Home(const uint8_t row) {
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Prepare_Menu_Auto_Home, Menu_Coordinate,LBLX, MBASE(row));
  Draw_Menu_Line(row, ICON_Homing);
  Draw_More_Icon(row);
}

inline void Item_Prepare_Leveling(const uint8_t row) {
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Prepare_Menu_Bed, Menu_Coordinate,LBLX, MBASE(row));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Prepare_Menu_Leveling, Menu_Coordinate,LBLX+Bed_X_Coordinate[HMI_flag.language], MBASE(row));
  Draw_Menu_Line(row, ICON_Leveling0);
  Draw_More_Icon(row);
}

inline void Item_Prepare_Powerdown(const uint8_t row) {
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Prepare_Menu_Power_Outage, Menu_Coordinate,LBLX, MBASE(row));
  Draw_Menu_Line(row, ICON_POWERDOWN);
}



#if HAS_ZOFFSET_ITEM
  inline void Item_Prepare_Offset(const uint8_t row) {
	#if HAS_BED_PROBE
  		DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Prepare_Menu_Z_Offset, Menu_Coordinate,LBLX, MBASE(row));
        DWIN_Draw_Signed_Float(font8x16, Color_Bg_Black, 2, 2, 202, MBASE(PREPARE_CASE_ZOFF + MROWS - index_prepare), BABY_Z_VAR * 100);
	#else
  		DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Prepare_Menu_Z_Offset, Menu_Coordinate,LBLX, MBASE(row));
		HMI_ValueStruct.offset_value = BABY_Z_VAR * 100;
        DWIN_Draw_Signed_Float(font8x16, Color_Bg_Black, 2, 2, 202, MBASE(PREPARE_CASE_ZOFF + MROWS - index_prepare), HMI_ValueStruct.offset_value);
	#endif
    Draw_Menu_Line(row, ICON_SetHome);
  }

#endif

#if HAS_HOTEND
  inline void Item_Prepare_PLA(const uint8_t row) {
  	DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Prepare_Menu_Preheat, Menu_Coordinate,LBLX, MBASE(row));
	DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Prepare_Menu_PLA, Menu_Coordinate,LBLX+Preheat_X_Coordinate[HMI_flag.language], MBASE(row));
    Draw_Menu_Line(row, ICON_PLAPreheat);
  }

  inline void Item_Prepare_ABS(const uint8_t row) {
  	DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Prepare_Menu_Preheat, Menu_Coordinate,LBLX, MBASE(row));
	DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Prepare_Menu_ABS, Menu_Coordinate,LBLX+Preheat_X_Coordinate[HMI_flag.language], MBASE(row));
    Draw_Menu_Line(row, ICON_ABSPreheat);
  }
#endif

#if HAS_PREHEAT
  inline void Item_Prepare_Cool(const uint8_t row) {
  	DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Prepare_Menu_Cooldown, Menu_Coordinate,LBLX, MBASE(row));
    Draw_Menu_Line(row, ICON_Cool);
  }
#endif

inline void Item_Prepare_Lang(const uint8_t row) {
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Prepare_Menu_Language, Menu_Coordinate,LBLX, MBASE(row));
  Draw_Menu_Line(row, ICON_Language);
  Draw_More_Icon(row);
}

inline void Draw_Prepare_Menu() {
  Clear_Main_Window();

  const int16_t pscroll = MROWS - index_prepare; // Scrolled-up lines
  #define PSCROL(L) (pscroll + (L))
  #define PVISI(L)  WITHIN(PSCROL(L), 0, MROWS)
  
  //DWIN_ICON_Show(ICON, Title_Coordinate[HMI_flag.language][Title_Prepare], 14, 7);
  DWIN_JPG_CacheToN(1,HMI_flag.Title_Menu_Bankup);
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Title_Prepare, Menu_Coordinate,14, 7);
  DWIN_JPG_CacheToN(1,HMI_flag.language+1);
  
  if (PVISI(0)) Draw_Back_First(select_prepare.now == 0);                         // < Back  
  if (PVISI(PREPARE_CASE_MOVE)) Item_Prepare_Move(PSCROL(PREPARE_CASE_MOVE));     // Move >
  if (PVISI(PREPARE_CASE_DISA)) Item_Prepare_Disable(PSCROL(PREPARE_CASE_DISA));  // Disable Stepper
  if (PVISI(PREPARE_CASE_HOME)) Item_Prepare_Home(PSCROL(PREPARE_CASE_HOME));     // Auto Home
  if (PVISI(PREPARE_CASE_LEVELING)) Item_Prepare_Leveling(PSCROL(PREPARE_CASE_LEVELING));     // Leveling
  if (PVISI(PREPARE_CASE_POWERDOWN)) Item_Prepare_Powerdown(PSCROL(PREPARE_CASE_POWERDOWN));     // Powerdown
  #if HAS_ZOFFSET_ITEM
    if (PVISI(PREPARE_CASE_ZOFF)) Item_Prepare_Offset(PSCROL(PREPARE_CASE_ZOFF)); // Edit Z-Offset / Babystep / Set Home Offset
  #endif
  #if HAS_HOTEND
    if (PVISI(PREPARE_CASE_PLA)) Item_Prepare_PLA(PSCROL(PREPARE_CASE_PLA));      // Preheat PLA
    if (PVISI(PREPARE_CASE_ABS)) Item_Prepare_ABS(PSCROL(PREPARE_CASE_ABS));      // Preheat ABS
  #endif
  #if HAS_PREHEAT
    if (PVISI(PREPARE_CASE_COOL)) Item_Prepare_Cool(PSCROL(PREPARE_CASE_COOL));   // Cooldown
  #endif
  if (PVISI(PREPARE_CASE_LANG)) Item_Prepare_Lang(PSCROL(PREPARE_CASE_LANG));     // Language CN/EN

  if (select_prepare.now) Draw_Menu_Cursor(PSCROL(select_prepare.now));
}

inline void Draw_Control_Menu() {
  Clear_Main_Window();

  #if CONTROL_CASE_TOTAL >= 6
    const int16_t cscroll = MROWS - index_control; // Scrolled-up lines
    #define CCSCROL(L) (cscroll + (L))
  #else
    #define CCSCROL(L) (L)
  #endif
  #define CCLINE(L)  MBASE(CCSCROL(L))
  #define CCVISI(L)  WITHIN(CCSCROL(L), 0, MROWS)
  
  //DWIN_ICON_Show(ICON, Title_Coordinate[HMI_flag.language][Title_Control], 14, 7);
  DWIN_JPG_CacheToN(1,HMI_flag.Title_Menu_Bankup);
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Title_Control, Menu_Coordinate,14, 7);
  DWIN_JPG_CacheToN(1,HMI_flag.language+1);
  if (CCVISI(0)) Draw_Back_First(select_control.now == 0);                         // < Back

  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Control_Menu_Temperature, Menu_Coordinate,LBLX, CCLINE(CONTROL_CASE_TEMP));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Control_Menu_Motion, Menu_Coordinate,LBLX, CCLINE(CONTROL_CASE_MOVE));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Control_Menu_Mixer, Menu_Coordinate,LBLX, CCLINE(CONTROL_CASE_MIXER));
  #if ENABLED(BLTOUCH)
	  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Control_Menu_Bltouch, Menu_Coordinate,LBLX, CCLINE(CONTROL_CASE_BLTOUCH));
	#if ENABLED(EEPROM_SETTINGS)
  	  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Control_Menu_Store, Menu_Coordinate,LBLX, CCLINE(CONTROL_CASE_SAVE));
	 // DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Control_Menu_Settings, Menu_Coordinate,LBLX+Store_X_Coordinate[HMI_flag.language], CLINE(CONTROL_CASE_SAVE));
	#endif
  #else
    #if ENABLED(EEPROM_SETTINGS)
  	  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Control_Menu_Store, Menu_Coordinate,LBLX, CCLINE(CONTROL_CASE_SAVE));
	 // DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Control_Menu_Settings, Menu_Coordinate,LBLX+Store_X_Coordinate[HMI_flag.language], CLINE(CONTROL_CASE_SAVE));
	  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Control_Menu_Load, Menu_Coordinate,LBLX, CCLINE(CONTROL_CASE_LOAD));
     // DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Control_Menu_Settings, Menu_Coordinate,LBLX+Load_X_Coordinate[HMI_flag.language], CLINE(CONTROL_CASE_LOAD));
	#endif
  #endif
  
  if (select_control.now && CCVISI(select_control.now))
    Draw_Menu_Cursor(CCSCROL(select_control.now));

  Draw_Menu_Line(CCSCROL(1),ICON_Temperature);
  Draw_More_Icon(CCSCROL(1));

  Draw_Menu_Line(CCSCROL(2),ICON_Motion);
  Draw_More_Icon(CCSCROL(2));

  Draw_Menu_Line(CCSCROL(3),ICON_Mixer);
  Draw_More_Icon(CCSCROL(3));
  #if ENABLED(BLTOUCH)
  	Draw_Menu_Line(CCSCROL(4),ICON_BLTouch);
    Draw_More_Icon(CCSCROL(4));
    #if ENABLED(EEPROM_SETTINGS)
  		Draw_Menu_Line(CCSCROL(5),ICON_WriteEEPROM);
	#endif
  #else
    #if ENABLED(EEPROM_SETTINGS)
    	Draw_Menu_Line(CCSCROL(4),ICON_WriteEEPROM);
  		Draw_Menu_Line(CCSCROL(5),ICON_ReadEEPROM);
	#endif
  #endif
}

inline void Draw_Tune_Menu() {
  Clear_Main_Window();

  //DWIN_ICON_Show(ICON, Title_Coordinate[HMI_flag.language][Title_Tune], 14, 7);
  DWIN_JPG_CacheToN(1,HMI_flag.Title_Menu_Bankup);
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Title_Tune, Menu_Coordinate,14, 7);
  DWIN_JPG_CacheToN(1,HMI_flag.language+1);
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Tune_Menu_Speed, Menu_Coordinate,LBLX, MBASE(TUNE_CASE_SPEED));
  #if HAS_HOTEND
  	DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Tune_Menu_Hotend, Menu_Coordinate,LBLX, MBASE(TUNE_CASE_TEMP));
    DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Tune_Menu_Temp, Menu_Coordinate,LBLX+Hotend_X_Coordinate[HMI_flag.language], MBASE(TUNE_CASE_TEMP));
  #endif
  #if HAS_HEATED_BED
  	DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Tune_Menu_Bed, Menu_Coordinate,LBLX, MBASE(TUNE_CASE_BED));
    DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Tune_Menu_Temp, Menu_Coordinate,LBLX+Bed_X_Coordinate[HMI_flag.language], MBASE(TUNE_CASE_BED));
  #endif
  #if HAS_FAN
    DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Tune_Menu_Fan_Speed, Menu_Coordinate,LBLX, MBASE(TUNE_CASE_FAN));
  #endif
  #if HAS_ZOFFSET_ITEM
  	DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Tune_Menu_Probe, Menu_Coordinate,LBLX, MBASE(TUNE_CASE_ZOFF));
    DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Tune_Menu_Z_Offset, Menu_Coordinate,LBLX+Probe_X_Coordinate[HMI_flag.language], MBASE(TUNE_CASE_ZOFF));
  #endif
  
  Draw_Back_First(select_tune.now == 0);
  if (select_tune.now) Draw_Menu_Cursor(select_tune.now);

  Draw_Menu_Line(TUNE_CASE_SPEED, ICON_Speed);
  DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(TUNE_CASE_SPEED), feedrate_percentage);

  #if HAS_HOTEND
    Draw_Menu_Line(TUNE_CASE_TEMP, ICON_HotendTemp);
    DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(TUNE_CASE_TEMP), thermalManager.temp_hotend[0].target);
  #endif
  
  #if HAS_HEATED_BED
    Draw_Menu_Line(TUNE_CASE_BED, ICON_BedTemp);
    DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(TUNE_CASE_BED), thermalManager.temp_bed.target);
  #endif
  
  #if HAS_FAN
    Draw_Menu_Line(TUNE_CASE_FAN, ICON_FanSpeed);
    DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(TUNE_CASE_FAN), thermalManager.fan_speed[0]);
  #endif
  
  #if HAS_ZOFFSET_ITEM
    Draw_Menu_Line(TUNE_CASE_ZOFF, ICON_Zoffset);
    DWIN_Draw_Signed_Float(font8x16, Color_Bg_Black, 2, 2, 202, MBASE(TUNE_CASE_ZOFF), BABY_Z_VAR * 100);
  #endif
}

#if 0
inline void draw_max_en(const uint16_t line) {
  DWIN_Frame_AreaCopy(1, 245, 119, 269, 129, LBLX, line);   // "Max"
}
inline void draw_max_accel_en(const uint16_t line) {
  draw_max_en(line);
  DWIN_Frame_AreaCopy(1, 1, 135, 79, 145, LBLX + 27, line); // "Acceleration"
}
inline void draw_speed_en(const uint16_t inset, const uint16_t line) {
  DWIN_Frame_AreaCopy(1, 184, 119, 224, 132, LBLX + inset, line); // "Speed"
}
inline void draw_jerk_en(const uint16_t line) {
  DWIN_Frame_AreaCopy(1, 64, 119, 106, 129, LBLX + 27, line); // "Jerk"
}
inline void draw_steps_per_mm(const uint16_t line) {
  DWIN_Frame_AreaCopy(1, 1, 151, 101, 161, LBLX, line);   // "Steps-per-mm"
}
inline void say_x(const uint16_t inset, const uint16_t line) {
  DWIN_Frame_AreaCopy(1, 95, 104, 102, 114, LBLX + inset, line); // "X"
}
inline void say_y(const uint16_t inset, const uint16_t line) {
  DWIN_Frame_AreaCopy(1, 104, 104, 110, 114, LBLX + inset, line); // "Y"
}
inline void say_z(const uint16_t inset, const uint16_t line) {
  DWIN_Frame_AreaCopy(1, 112, 104, 120, 114, LBLX + inset, line); // "Z"
}
inline void say_e(const uint16_t inset, const uint16_t line) {
  DWIN_Frame_AreaCopy(1, 237, 119, 244, 129, LBLX + inset, line); // "E"
}
#endif

inline void Draw_Motion_Menu() {
  Clear_Main_Window();

  //DWIN_ICON_Show(ICON, Title_Coordinate[HMI_flag.language][Title_Motion], 14, 7);
  DWIN_JPG_CacheToN(1,HMI_flag.Title_Menu_Bankup);
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Title_Motion, Menu_Coordinate,14, 7);
  DWIN_JPG_CacheToN(1,HMI_flag.language+1);
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Motion_Menu_Feedrate, Menu_Coordinate,LBLX, MBASE(MOTION_CASE_RATE));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Motion_Menu_Acc, Menu_Coordinate,LBLX, MBASE(MOTION_CASE_ACCEL));
  #if HAS_CLASSIC_JERK
  	DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Motion_Menu_Jerk, Menu_Coordinate,LBLX, MBASE(MOTION_CASE_JERK));
  #endif 
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Motion_Menu_Steps, Menu_Coordinate,LBLX, MBASE(MOTION_CASE_STEPS));
  
  Draw_Back_First(select_motion.now == 0);
  if (select_motion.now) Draw_Menu_Cursor(select_motion.now);

  uint8_t i = 0;
  #define _MOTION_ICON(N) Draw_Menu_Line(++i, ICON_MaxSpeed + (N) - 1)
  _MOTION_ICON(MOTION_CASE_RATE); Draw_More_Icon(i);
  _MOTION_ICON(MOTION_CASE_ACCEL); Draw_More_Icon(i);
  #if HAS_CLASSIC_JERK
    _MOTION_ICON(MOTION_CASE_JERK); Draw_More_Icon(i);
  #endif
  _MOTION_ICON(MOTION_CASE_STEPS); Draw_More_Icon(i);
}

//
// Draw Leveling Windows
//
inline void Draw_Leveling_Menu() {
  Clear_Main_Window();

  //DWIN_ICON_Show(ICON, Title_Coordinate[HMI_flag.language][Title_Leveling], 14, 7);
  DWIN_JPG_CacheToN(1,HMI_flag.Title_Menu_Bankup);
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Title_Leveling, Menu_Coordinate,14, 7);
  DWIN_JPG_CacheToN(1,HMI_flag.language+1);
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Leveling_Menu_Point, Menu_Coordinate,LBLX, MBASE(LEVELING_CASE_POINT1));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Leveling_Menu_1, Menu_Coordinate,LBLX+Point_X_Coordinate[HMI_flag.language], MBASE(LEVELING_CASE_POINT1));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Leveling_Menu_Point, Menu_Coordinate,LBLX, MBASE(LEVELING_CASE_POINT2));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Leveling_Menu_2, Menu_Coordinate,LBLX+Point_X_Coordinate[HMI_flag.language], MBASE(LEVELING_CASE_POINT2));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Leveling_Menu_Point, Menu_Coordinate,LBLX, MBASE(LEVELING_CASE_POINT3));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Leveling_Menu_3, Menu_Coordinate,LBLX+Point_X_Coordinate[HMI_flag.language], MBASE(LEVELING_CASE_POINT3));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Leveling_Menu_Point, Menu_Coordinate,LBLX, MBASE(LEVELING_CASE_POINT4));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Leveling_Menu_4, Menu_Coordinate,LBLX+Point_X_Coordinate[HMI_flag.language], MBASE(LEVELING_CASE_POINT4));
  #if ENABLED(LCD_BED_LEVELING)
  	DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Leveling_Menu_Catch, Menu_Coordinate,LBLX, MBASE(LEVELING_CASE_AUTO));
  	DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Leveling_Menu_Z_Offset, Menu_Coordinate,LBLX+Ctach_X_Coordinate[HMI_flag.language], MBASE(LEVELING_CASE_AUTO));
  #endif
  
  Draw_Back_First(select_leveling.now == 0);
  if (select_leveling.now) Draw_Menu_Cursor(select_leveling.now);

  #if ENABLED(BLTOUCH)
  	uint8_t i,j=ICON_Leveling_Point1;
  	for(i=LEVELING_CASE_POINT1; i<LEVELING_CASE_TOTAL; i++,j++) {
  		Draw_Menu_Line(i,j);
  	}
  #else
    uint8_t i,j=ICON_Leveling_Point1;
  	for(i=LEVELING_CASE_POINT1; i<LEVELING_CASE_TOTAL+1; i++,j++) {
  		Draw_Menu_Line(i,j);
  	}
  #endif
}

//
// Draw Home Windows
//
inline void Draw_Home_Menu() {
  Clear_Main_Window();

  //DWIN_ICON_Show(ICON, Title_Coordinate[HMI_flag.language][Title_Home], 14, 7);
  DWIN_JPG_CacheToN(1,HMI_flag.Title_Menu_Bankup);
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Title_Home, Menu_Coordinate,14, 7);
  DWIN_JPG_CacheToN(1,HMI_flag.language+1);
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Home_Menu_Home, Menu_Coordinate,LBLX, MBASE(HOME_CASE_ALL));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Home_Menu_All, Menu_Coordinate,LBLX+Home_X_Coordinate[HMI_flag.language], MBASE(HOME_CASE_ALL));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Home_Menu_Home, Menu_Coordinate,LBLX, MBASE(HOME_CASE_X));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Home_Menu_X, Menu_Coordinate,LBLX+Home_X_Coordinate[HMI_flag.language], MBASE(HOME_CASE_X));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Home_Menu_Home, Menu_Coordinate,LBLX, MBASE(HOME_CASE_Y));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Home_Menu_Y, Menu_Coordinate,LBLX+Home_X_Coordinate[HMI_flag.language], MBASE(HOME_CASE_Y));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Home_Menu_Home, Menu_Coordinate,LBLX, MBASE(HOME_CASE_Z));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Home_Menu_Z, Menu_Coordinate,LBLX+Home_X_Coordinate[HMI_flag.language], MBASE(HOME_CASE_Z));
  
  Draw_Back_First(select_home.now == 0);
  if (select_home.now) Draw_Menu_Cursor(select_home.now);

  uint8_t i,j=ICON_HOME_ALL;
  for(i=HOME_CASE_ALL; i<HOME_CASE_TOTAL+1; i++,j++) {
  	Draw_Menu_Line(i,j);
  }
}

//
// Draw Bltouch Windows
//
#if ENABLED(BLTOUCH)
inline void Draw_Bltouch_Menu() {
  Clear_Main_Window();

  //DWIN_ICON_Show(ICON, Title_Coordinate[HMI_flag.language][Title_BLTouch], 14, 7);
  DWIN_JPG_CacheToN(1,HMI_flag.Title_Menu_Bankup);
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Title_BLTouch, Menu_Coordinate,14, 7);
  DWIN_JPG_CacheToN(1,HMI_flag.language+1);
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, BLTouch_Menu_Reset, Menu_Coordinate,LBLX, MBASE(BLTOUCH_CASE_RESET));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, BLTouch_Menu_Test, Menu_Coordinate,LBLX, MBASE(BLTOUCH_CASE_TEST));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, BLTouch_Menu_Stow, Menu_Coordinate,LBLX, MBASE(BLTOUCH_CASE_STOW));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, BLTouch_Menu_Deploy, Menu_Coordinate,LBLX, MBASE(BLTOUCH_CASE_DEPLOY));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, BLTouch_Menu_Mode, Menu_Coordinate,LBLX, MBASE(BLTOUCH_CASE_SW));

  Draw_Back_First(select_bltouch.now == 0);
  if (select_bltouch.now) Draw_Menu_Cursor(select_bltouch.now);

  uint8_t i,j=ICON_BLTOUCH_RESET;
  for(i=BLTOUCH_CASE_RESET; i<BLTOUCH_CASE_TOTAL+1; i++,j++) {
  	Draw_Menu_Line(i,j);
  }
}
#endif

//
// Draw Language Windows
//
inline void Draw_Language_Menu() {
  Clear_Main_Window();

  //DWIN_ICON_Show(ICON, Title_Coordinate[HMI_flag.language][Title_Mixer], 14, 7);
 
  DWIN_JPG_CacheToN(1,HMI_flag.Title_Menu_Bankup);
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Title_Language, Menu_Coordinate,14, 7);
  DWIN_JPG_CacheToN(1,HMI_flag.language+1);
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Language_Menu_EN, Menu_Coordinate,LBLX, MBASE(LANGUAGE_CASE_EN));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Language_Menu_SP, Menu_Coordinate,LBLX, MBASE(LANGUAGE_CASE_SP));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Language_Menu_RU, Menu_Coordinate,LBLX, MBASE(LANGUAGE_CASE_RU));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Language_Menu_FR, Menu_Coordinate,LBLX, MBASE(LANGUAGE_CASE_FR));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Language_Menu_PO, Menu_Coordinate,LBLX, MBASE(LANGUAGE_CASE_PO));

  Draw_Back_First(select_language.now == 0);
  if (select_language.now) Draw_Menu_Cursor(select_language.now);

  uint8_t i,j=ICON_EN;
  for(i=LANGUAGE_CASE_EN; i<LANGUAGE_CASE_TOTAL+1; i++,j++) {
  	Draw_Menu_Line(i,j);
  }
}


//
// Draw Mixer Windows
//
inline void Draw_Mixer_Menu() {
  Clear_Main_Window();

  //DWIN_ICON_Show(ICON, Title_Coordinate[HMI_flag.language][Title_Mixer], 14, 7);
 
  DWIN_JPG_CacheToN(1,HMI_flag.Title_Menu_Bankup);
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Title_Mixer, Menu_Coordinate,14, 7);
  DWIN_JPG_CacheToN(1,HMI_flag.language+1);
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Mixer_Menu_Mix, Menu_Coordinate,LBLX, MBASE(MIXER_CASE_MANUAL));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Mixer_Menu_Gradient, Menu_Coordinate,LBLX, MBASE(MIXER_CASE_AUTO));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Mixer_Menu_Random, Menu_Coordinate,LBLX, MBASE(MIXER_CASE_RANDOM));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Mixer_Menu_Current, Menu_Coordinate,LBLX, MBASE(MIXER_CASE_VTOOL));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Mixer_Menu_Vtool, Menu_Coordinate,LBLX+Current_X_Coordinate[HMI_flag.language], MBASE(MIXER_CASE_VTOOL));

  Draw_Back_First(select_mixer.now == 0);
  if (select_mixer.now) Draw_Menu_Cursor(select_mixer.now);

  uint8_t i,j=ICON_Mixer_Manual;
  for(i=MIXER_CASE_MANUAL; i<MIXER_CASE_VTOOL; i++,j++) {
  	Draw_Menu_Line(i,j);
	Draw_More_Icon(i);
  }
  
  Draw_Menu_Line(i++,ICON_S_VTOOL);
  DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(i-1), mixer.selected_vtool);
}

//
// Draw Mixer_Manual Windows
//
inline void Draw_Mixer_Manual_Menu() {
  Clear_Main_Window();

  #if MANUAL_CASE_TOTAL >= 6
    const int16_t mscroll = MROWS - index_manual; // Scrolled-up lines
    #define MCSCROL(L) (mscroll + (L))
  #else
    #define MCSCROL(L) (L)
  #endif
  #define MCLINE(L)  MBASE(MCSCROL(L))
  #define MCVISI(L)  WITHIN(MCSCROL(L), 0, MROWS)

  //DWIN_ICON_Show(ICON, Title_Coordinate[HMI_flag.language][Title_MIX], 14, 7);
  DWIN_JPG_CacheToN(1,HMI_flag.Title_Menu_Bankup);
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Title_MIX, Menu_Coordinate,14, 7);
  DWIN_JPG_CacheToN(1,HMI_flag.language+1);
  if (MCVISI(0)) Draw_Back_First(select_manual.now == 0);                         // < Back

  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Mix_Menu_Extruder, Menu_Coordinate,LBLX, MCLINE(MANUAL_CASE_EXTRUDER1));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Mix_Menu_1, Menu_Coordinate,LBLX+Extruder_X_Coordinate[HMI_flag.language], MCLINE(MANUAL_CASE_EXTRUDER1));
  #if ENABLED (MIXING_EXTRUDER)
  		#if(MIXING_STEPPERS == 4)
			DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Mix_Menu_Extruder, Menu_Coordinate,LBLX, MCLINE(MANUAL_CASE_EXTRUDER2));
  			DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Mix_Menu_2, Menu_Coordinate,LBLX+Extruder_X_Coordinate[HMI_flag.language], MCLINE(MANUAL_CASE_EXTRUDER2));
			DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Mix_Menu_Extruder, Menu_Coordinate,LBLX, MCLINE(MANUAL_CASE_EXTRUDER3));
  			DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Mix_Menu_3, Menu_Coordinate,LBLX+Extruder_X_Coordinate[HMI_flag.language], MCLINE(MANUAL_CASE_EXTRUDER3));
			DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Mix_Menu_Extruder, Menu_Coordinate,LBLX, MCLINE(MANUAL_CASE_EXTRUDER4));
  			DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Mix_Menu_4, Menu_Coordinate,LBLX+Extruder_X_Coordinate[HMI_flag.language], MCLINE(MANUAL_CASE_EXTRUDER4));
			DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Mix_Menu_Comit, Menu_Coordinate,LBLX, MCLINE(MANUAL_CASE_OK));
			DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Mix_Menu_vtool, Menu_Coordinate,LBLX+Comit_X_Coordinate[HMI_flag.language], MCLINE(MANUAL_CASE_OK));
    	#elif(MIXING_STEPPERS == 3)
    		DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Mix_Menu_Extruder, Menu_Coordinate,LBLX, MCLINE(MANUAL_CASE_EXTRUDER2));
  			DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Mix_Menu_2, Menu_Coordinate,LBLX+Extruder_X_Coordinate[HMI_flag.language], MCLINE(MANUAL_CASE_EXTRUDER2));
			DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Mix_Menu_Extruder, Menu_Coordinate,LBLX, MCLINE(MANUAL_CASE_EXTRUDER3));
  			DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Mix_Menu_3, Menu_Coordinate,LBLX+Extruder_X_Coordinate[HMI_flag.language], MCLINE(MANUAL_CASE_EXTRUDER3));
			DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Mix_Menu_Comit, Menu_Coordinate,LBLX, MCLINE(MANUAL_CASE_OK));
			DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Mix_Menu_vtool, Menu_Coordinate,LBLX+Comit_X_Coordinate[HMI_flag.language], MCLINE(MANUAL_CASE_OK));
    	#elif(MIXING_STEPPERS == 2)
    		DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Mix_Menu_Extruder, Menu_Coordinate,LBLX, MCLINE(MANUAL_CASE_EXTRUDER2));
  			DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Mix_Menu_2, Menu_Coordinate,LBLX+Extruder_X_Coordinate[HMI_flag.language], MCLINE(MANUAL_CASE_EXTRUDER2));
			DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Mix_Menu_Comit, Menu_Coordinate,LBLX, MCLINE(MANUAL_CASE_OK));
			DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Mix_Menu_vtool, Menu_Coordinate,LBLX+Comit_X_Coordinate[HMI_flag.language], MCLINE(MANUAL_CASE_OK));
    	#endif
  #endif
  
  Draw_Back_First(select_manual.now == 0);
  if (select_manual.now) Draw_Menu_Cursor(select_manual.now);

  uint8_t i,j=ICON_Extruder1;
  for(i=MANUAL_CASE_EXTRUDER1; i<MANUAL_CASE_TOTAL; i++,j++) {
  	Draw_Menu_Line(i,j);
  	//DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(i), MixerCfg.Manual_Percent[mixer.selected_vtool][i-1]);
	DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(i), mixer.mix[i-1]);
  }
  Draw_Menu_Line(i++,ICON_C_VTOOL);
  DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(i-1), mixer.selected_vtool);
}

//
// Draw Mixer_Auto Windows
//
inline void Draw_Mixer_Auto_Menu() {
  Clear_Main_Window();

  #if AUTO_CASE_TOTAL >= 6
    const int16_t ascroll = MROWS - index_auto; // Scrolled-up lines
    #define ACSCROL(L) (ascroll + (L))
  #else
    #define ACSCROL(L) (L)
  #endif
  #define ACLINE(L)  MBASE(ACSCROL(L))
  #define ACVISI(L)  WITHIN(ACSCROL(L), 0, MROWS)

  //DWIN_ICON_Show(ICON, Title_Coordinate[HMI_flag.language][Title_GRADIENT], 14, 7);
  DWIN_JPG_CacheToN(1,HMI_flag.Title_Menu_Bankup);
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Title_GRADIENT, Menu_Coordinate,14, 7);
  DWIN_JPG_CacheToN(1,HMI_flag.language+1);
  if (ACVISI(0)) Draw_Back_First(select_auto.now == 0);                         // < Back

  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Gradient_Menu_Start, Menu_Coordinate,LBLX, ACLINE(AUTO_CASE_ZPOS_START));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Gradient_Menu_Z, Menu_Coordinate,LBLX+Start_X_Coordinate[HMI_flag.language], ACLINE(AUTO_CASE_ZPOS_START));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Gradient_Menu_End, Menu_Coordinate,LBLX, ACLINE(AUTO_CASE_ZPOS_END));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Gradient_Menu_Z, Menu_Coordinate,LBLX+End_X_Coordinate[HMI_flag.language], ACLINE(AUTO_CASE_ZPOS_END));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Gradient_Menu_Start, Menu_Coordinate,LBLX, ACLINE(AUTO_CASE_VTOOL_START));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Gradient_Menu_Vtool, Menu_Coordinate,LBLX+Start_X_Coordinate[HMI_flag.language], ACLINE(AUTO_CASE_VTOOL_START));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Gradient_Menu_End, Menu_Coordinate,LBLX, ACLINE(AUTO_CASE_VTOOL_END));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Gradient_Menu_Vtool, Menu_Coordinate,LBLX+End_X_Coordinate[HMI_flag.language], ACLINE(AUTO_CASE_VTOOL_END));
  
  Draw_Back_First(select_auto.now == 0);
  if (select_auto.now) Draw_Menu_Cursor(select_auto.now);

  Draw_Menu_Line(1,ICON_Zpos_Rise);
  HMI_ValueStruct.Auto_Zstart_scale = MixerCfg.Auto_Zpos[ZPOS_START]*MINUNITMULT;
  DWIN_Draw_Signed_Float(font8x16, Color_Bg_Black, 3, 1, 216, MBASE(1), HMI_ValueStruct.Auto_Zstart_scale);

  Draw_Menu_Line(2,ICON_Zpos_Drop);
  HMI_ValueStruct.Auto_Zend_scale = MixerCfg.Auto_Zpos[ZPOS_END]*MINUNITMULT;
  DWIN_Draw_Signed_Float(font8x16, Color_Bg_Black, 3, 1, 216, MBASE(2), HMI_ValueStruct.Auto_Zend_scale);

  Draw_Menu_Line(3,ICON_VTool_Rise);
  DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(3), MixerCfg.Auto_VTool[VTOOL_START]);

  Draw_Menu_Line(4,ICON_VTool_Drop);
  DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(4), MixerCfg.Auto_VTool[VTOOL_END]);
  /*
  uint8_t i,j=ICON_Extruder1;
  for(i=AUTO_CASE_VTOOL_START; i<AUTO_CASE_TOTAL; i++,j++) {
  	Draw_Menu_Line(i,j);
  	DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(i), MixerCfg.Auto_Percent[mixer.selected_vtool][i-1]);
  }
  */
  //Draw_Menu_Line(i++,ICON_S_VTOOL);
  //DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(i-1), mixer.selected_vtool);
}


//
// Draw Mixer_Random Windows
//
inline void Draw_Mixer_Random_Menu() {
  Clear_Main_Window();

  #if RANDOM_CASE_TOTAL >= 6
    const int16_t rscroll = MROWS - index_random; // Scrolled-up lines
    #define RCSCROL(L) (rscroll + (L))
  #else
    #define RCSCROL(L) (L)
  #endif
  #define RCLINE(L)  MBASE(RCSCROL(L))
  #define RCVISI(L)  WITHIN(RCSCROL(L), 0, MROWS)

  //DWIN_ICON_Show(ICON, Title_Coordinate[HMI_flag.language][Title_RANDOM], 14, 7);
  DWIN_JPG_CacheToN(1,HMI_flag.Title_Menu_Bankup);
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Title_RANDOM, Menu_Coordinate,14, 7);
  DWIN_JPG_CacheToN(1,HMI_flag.language+1);
  if (RCVISI(0)) Draw_Back_First(select_random.now == 0);                         // < Back

  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Random_Menu_Start, Menu_Coordinate,LBLX, RCLINE(RANDOM_CASE_ZPOS_START));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Random_Menu_Z, Menu_Coordinate,LBLX+Start_X_Coordinate[HMI_flag.language], RCLINE(RANDOM_CASE_ZPOS_START));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Random_Menu_End, Menu_Coordinate,LBLX, RCLINE(RANDOM_CASE_ZPOS_END));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Random_Menu_Z, Menu_Coordinate,LBLX+End_X_Coordinate[HMI_flag.language], RCLINE(RANDOM_CASE_ZPOS_END));
  
  Draw_Back_First(select_random.now == 0);
  if (select_random.now) Draw_Menu_Cursor(select_random.now);

  Draw_Menu_Line(1,ICON_Zpos_Rise);
  HMI_ValueStruct.Random_Zstart_scale = MixerCfg.Random_Zpos[ZPOS_START]*MINUNITMULT;
  DWIN_Draw_Signed_Float(font8x16, Color_Bg_Black, 3, 1, 216, MBASE(1), HMI_ValueStruct.Random_Zstart_scale);
  Draw_Menu_Line(2,ICON_Zpos_Drop);
  HMI_ValueStruct.Random_Zend_scale = MixerCfg.Random_Zpos[ZPOS_END]*MINUNITMULT;
  DWIN_Draw_Signed_Float(font8x16, Color_Bg_Black, 3, 1, 216, MBASE(2), HMI_ValueStruct.Random_Zend_scale);
}


//
// Draw Popup Windows
//
#if HAS_HOTEND || HAS_HEATED_BED
  void DWIN_Popup_Temperature(const bool toohigh) {
    Clear_Popup_Area();
    Draw_Popup_Bkgd_105();
    if (toohigh) {
      DWIN_ICON_Show(ICON, ICON_TempTooHigh, 102, 165);
	  /*
      if (HMI_IsChinese()) {
        DWIN_Frame_AreaCopy(1, 103, 371, 237, 386, 52, 285);
        DWIN_Frame_AreaCopy(1, 151, 389, 185, 402, 187, 285);
        DWIN_Frame_AreaCopy(1, 189, 389, 271, 402, 95, 310);
      }
      else {
	  	*/
        DWIN_Draw_String(false, true, font8x16, Popup_Text_Color, Color_Bg_Window, 36, 300, F("Nozzle or Bed temperature"));
        DWIN_Draw_String(false, true, font8x16, Popup_Text_Color, Color_Bg_Window, 92, 300, F("is too high"));
      //}
    }
    else {
      DWIN_ICON_Show(ICON, ICON_TempTooLow, 102, 165);
	  /*
      if (HMI_IsChinese()) {
        DWIN_Frame_AreaCopy(1, 103, 371, 270, 386, 52, 285);
        DWIN_Frame_AreaCopy(1, 189, 389, 271, 402, 95, 310);
      }
      else {
	  	*/
        DWIN_Draw_String(false, true, font8x16, Popup_Text_Color, Color_Bg_Window, 36, 300, F("Nozzle or Bed temperature"));
        DWIN_Draw_String(false, true, font8x16, Popup_Text_Color, Color_Bg_Window, 92, 300, F("is too low"));
      //}
    }
  }

#endif

inline void Draw_Popup_Bkgd_60() {
  DWIN_Draw_Rectangle(1, Color_Bg_Window, 14, 60, 258, 330);
}

#if HAS_HOTEND
  void Popup_Window_ETempTooLow() {
    Clear_Main_Window();
    Draw_Popup_Bkgd_60();
    DWIN_ICON_Show(ICON, ICON_TempTooLow, 102, 105);
	/*
    if (HMI_IsChinese()) {
      DWIN_Frame_AreaCopy(1, 103, 371, 136, 386, 69, 240);
      DWIN_Frame_AreaCopy(1, 170, 371, 270, 386, 102, 240);
      DWIN_ICON_Show(ICON, ICON_Confirm_C, 86, 280);
    }
    else {
      DWIN_Draw_String(false, true, font8x16, Popup_Text_Color, Color_Bg_Window, 20, 235, F("Nozzle is too cold"));
      DWIN_ICON_Show(ICON, ICON_Confirm_E, 86, 280);
    }
	*/
	DWIN_Draw_String(false, true, font8x16, Popup_Text_Color, Color_Bg_Window, 64, 235, F("Nozzle is too cold"));
    DWIN_ICON_Show(ICON, ICON_Confirm_E, 86, 280);
  }
#endif

void Popup_Window_Resume() {
  Clear_Popup_Area();
  Draw_Popup_Bkgd_105();
  /*
  if (HMI_IsChinese()) {
    DWIN_Frame_AreaCopy(1, 160, 338, 235, 354, 98, 135);
    DWIN_Frame_AreaCopy(1, 103, 321, 271, 335, 52, 192);
    DWIN_ICON_Show(ICON, ICON_Cancel_C,    26, 307);
    DWIN_ICON_Show(ICON, ICON_Continue_C, 146, 307);
  }
  else {
    DWIN_Draw_String(false, true, font8x16, Popup_Text_Color, Color_Bg_Window, (272 - 8 * 14) / 2, 115, F("Continue Print"));
    DWIN_Draw_String(false, true, font8x16, Popup_Text_Color, Color_Bg_Window, (272 - 8 * 22) / 2, 192, F("It looks like the last"));
    DWIN_Draw_String(false, true, font8x16, Popup_Text_Color, Color_Bg_Window, (272 - 8 * 22) / 2, 212, F("file was interrupted."));
    DWIN_ICON_Show(ICON, ICON_Cancel_E,    26, 307);
    DWIN_ICON_Show(ICON, ICON_Continue_E, 146, 307);
  }
  */
  DWIN_Draw_String(false, true, font8x16, Popup_Text_Color, Color_Bg_Window, (272 - 8 * 14) / 2, 115, F("Continue Print"));
  DWIN_Draw_String(false, true, font8x16, Popup_Text_Color, Color_Bg_Window, (272 - 8 * 22) / 2, 192, F("It looks like the last"));
  DWIN_Draw_String(false, true, font8x16, Popup_Text_Color, Color_Bg_Window, (272 - 8 * 22) / 2, 212, F("file was interrupted."));
  DWIN_ICON_Show(ICON, ICON_Cancel_E,    26, 307);
  DWIN_ICON_Show(ICON, ICON_Continue_E, 146, 307);
}

void Popup_Window_HomeAll(const bool parking/*=false*/) {
  Clear_Main_Window();
  Draw_Popup_Bkgd_60();
  //DWIN_ICON_Show(ICON, ICON_BLTouch, 101, 105);
  /*
  if (HMI_IsChinese()) {
    DWIN_Frame_AreaCopy(1, 0, 371, 33, 386, 85, 240);
    DWIN_Frame_AreaCopy(1, 203, 286, 271, 302, 118, 240);
    DWIN_Frame_AreaCopy(1, 0, 389, 150, 402, 61, 280);
  }
  else {
    DWIN_Draw_String(false, true, font8x16, Popup_Text_Color, Color_Bg_Window, (272 - 8 * (parking ? 7 : 10)) / 2, 230, parking ? F("Parking") : F("Homing XYZ"));
    DWIN_Draw_String(false, true, font8x16, Popup_Text_Color, Color_Bg_Window, (272 - 8 * 23) / 2, 260, F("Please wait until done."));
  }
  */
   DWIN_Draw_String(false, true, font8x16, Popup_Text_Color, Color_Bg_Window, (272 - 8 * (parking ? 7 : 10)) / 2, 230, parking ? F("Parking") : F("Homing XYZ"));
   DWIN_Draw_String(false, true, font8x16, Popup_Text_Color, Color_Bg_Window, (272 - 8 * 23) / 2, 260, F("Please wait until done."));
}

void Popup_Window_HomeX(const bool parking/*=false*/) {
  Clear_Main_Window();
  Draw_Popup_Bkgd_60();
  //DWIN_ICON_Show(ICON, ICON_BLTouch, 101, 105);
  /*
  if (HMI_IsChinese()) {
    DWIN_Frame_AreaCopy(1, 0, 371, 33, 386, 85, 240);
    DWIN_Frame_AreaCopy(1, 203, 286, 271, 302, 118, 240);
    DWIN_Frame_AreaCopy(1, 0, 389, 150, 402, 61, 280);
  }
  else {
  	*/
    DWIN_Draw_String(false, true, font8x16, Popup_Text_Color, Color_Bg_Window, (272 - 8 * (parking ? 7 : 10)) / 2, 230, parking ? F("Parking") : F("Homing X"));
    DWIN_Draw_String(false, true, font8x16, Popup_Text_Color, Color_Bg_Window, (272 - 8 * 23) / 2, 260, F("Please wait until done."));
  //}
}

void Popup_Window_HomeY(const bool parking/*=false*/) {
  Clear_Main_Window();
  Draw_Popup_Bkgd_60();
  //DWIN_ICON_Show(ICON, ICON_BLTouch, 101, 105);
  /*
  if (HMI_IsChinese()) {
    DWIN_Frame_AreaCopy(1, 0, 371, 33, 386, 85, 240);
    DWIN_Frame_AreaCopy(1, 203, 286, 271, 302, 118, 240);
    DWIN_Frame_AreaCopy(1, 0, 389, 150, 402, 61, 280);
  }
  else {
  	*/
    DWIN_Draw_String(false, true, font8x16, Popup_Text_Color, Color_Bg_Window, (272 - 8 * (parking ? 7 : 10)) / 2, 230, parking ? F("Parking") : F("Homing Y"));
    DWIN_Draw_String(false, true, font8x16, Popup_Text_Color, Color_Bg_Window, (272 - 8 * 23) / 2, 260, F("Please wait until done."));
 // }
}

void Popup_Window_HomeZ(const bool parking/*=false*/) {
  Clear_Main_Window();
  Draw_Popup_Bkgd_60();
  //DWIN_ICON_Show(ICON, ICON_BLTouch, 101, 105);
  /*
  if (HMI_IsChinese()) {
    DWIN_Frame_AreaCopy(1, 0, 371, 33, 386, 85, 240);
    DWIN_Frame_AreaCopy(1, 203, 286, 271, 302, 118, 240);
    DWIN_Frame_AreaCopy(1, 0, 389, 150, 402, 61, 280);
  }
  else {
  	*/
    DWIN_Draw_String(false, true, font8x16, Popup_Text_Color, Color_Bg_Window, (272 - 8 * (parking ? 7 : 10)) / 2, 230, parking ? F("Parking") : F("Homing Z"));
    DWIN_Draw_String(false, true, font8x16, Popup_Text_Color, Color_Bg_Window, (272 - 8 * 23) / 2, 260, F("Please wait until done."));
  //}
}

void Popup_Window_Leveling0() {
    Clear_Main_Window();
    Draw_Popup_Bkgd_60();
    DWIN_ICON_Show(ICON, ICON_AutoLeveling, 101, 105);
	/*
    if (HMI_IsChinese()) {
      DWIN_Frame_AreaCopy(1, 0, 371, 100, 386, 84, 240);
      DWIN_Frame_AreaCopy(1, 0, 389, 150, 402, 61, 280);
    }
    else {
		*/
      DWIN_Draw_String(false, true, font8x16, Popup_Text_Color, Color_Bg_Window, (272 - 8 * 13) / 2, 230, GET_TEXT_F(MSG_BED_LEVELING));
      DWIN_Draw_String(false, true, font8x16, Popup_Text_Color, Color_Bg_Window, (272 - 8 * 23) / 2, 260, F("Please wait until done."));
    //}
  }



#if HAS_ONESTEP_LEVELING
  void Popup_Window_Leveling() {
    Clear_Main_Window();
    Draw_Popup_Bkgd_60();
    DWIN_ICON_Show(ICON, ICON_AutoLeveling, 101, 105);
    if (HMI_IsChinese()) {
      DWIN_Frame_AreaCopy(1, 0, 371, 100, 386, 84, 240);
      DWIN_Frame_AreaCopy(1, 0, 389, 150, 402, 61, 280);
    }
    else {
      DWIN_Draw_String(false, true, font8x16, Popup_Text_Color, Color_Bg_Window, (272 - 8 * 13) / 2, 230, GET_TEXT_F(MSG_BED_LEVELING));
      DWIN_Draw_String(false, true, font8x16, Popup_Text_Color, Color_Bg_Window, (272 - 8 * 23) / 2, 260, F("Please wait until done."));
    }
  }
#endif

void Draw_Select_Highlight(const bool sel) {
  HMI_flag.select_flag = sel;
  const uint16_t c1 = sel ? Select_Color : Color_Bg_Window,
                 c2 = sel ? Color_Bg_Window : Select_Color;
  DWIN_Draw_Rectangle(0, c1, 25, 279, 126, 318);
  DWIN_Draw_Rectangle(0, c1, 24, 278, 127, 319);
  DWIN_Draw_Rectangle(0, c2, 145, 279, 246, 318);
  DWIN_Draw_Rectangle(0, c2, 144, 278, 247, 319);
}

void Popup_window_PauseOrStop() {
  Clear_Main_Window();
  Draw_Popup_Bkgd_60();
  /*
  if (HMI_IsChinese()) {
         if (select_print.now == 1) DWIN_Frame_AreaCopy(1, 237, 338, 269, 356, 98, 150);
    else if (select_print.now == 2) DWIN_Frame_AreaCopy(1, 221, 320, 253, 336, 98, 150);
    DWIN_Frame_AreaCopy(1, 220, 304, 264, 319, 130, 150);
    DWIN_ICON_Show(ICON, ICON_Confirm_C, 26, 280);
    DWIN_ICON_Show(ICON, ICON_Cancel_C, 146, 280);
  }
  else {
  	*/
         if (select_print.now == 1) DWIN_Draw_String(false, true, font12x24, Popup_Text_Color, Color_Bg_Window, (272 - 12 * 12) / 2, 150, GET_TEXT_F(MSG_PAUSE_PRINT));
    else if (select_print.now == 2) DWIN_Draw_String(false, true, font12x24, Popup_Text_Color, Color_Bg_Window, (272 - 12 * 11) / 2, 150, GET_TEXT_F(MSG_STOP_PRINT));
    DWIN_ICON_Show(ICON, ICON_Confirm_E, 26, 280);
    DWIN_ICON_Show(ICON, ICON_Cancel_E, 146, 280);
 // }
  Draw_Select_Highlight(true);
}


void DRAW_Filament_Runout_Mode(char mode){
	switch (mode){
		case DWIN_PAUSE_MODE_PAUSE_PRINT:
			Draw_Title(GET_TEXT_F(MSG_DWIN_FILAMENT_PRINT_PAUSE));
			break;
		case DWIN_PAUSE_MODE_CHANGE_FILAMENT:
			Draw_Title(GET_TEXT_F(MSG_DWIN_FILAMENT_PRINT_CHANGE));
			break;
		case DWIN_PAUSE_MODE_LOAD_FILAMENT:
			Draw_Title(GET_TEXT_F(MSG_DWIN_FILAMENT_PRINT_LOAD));
			break;
		case DWIN_PAUSE_MODE_UNLOAD_FILAMENT:
			Draw_Title(GET_TEXT_F(MSG_DWIN_FILAMENT_PRINT_UNLOAD));
			break;
		case DWIN_PAUSE_MODE_INSERT_FILAMENT:
			Draw_Title(GET_TEXT_F(MSG_DWIN_FILAMENT_PRINT_INSERT));
			break;
	    case DWIN_PAUSE_MODE_PURGE_FILAMENT:
			Draw_Title(GET_TEXT_F(MSG_DWIN_FILAMENT_PRINT_PURGE));
			break;
	    case DWIN_PAUSE_MODE_OPTION_FILAMENT:
			Draw_Title(GET_TEXT_F(MSG_DWIN_FILAMENT_PRINT_OPTION));
			break;
		case DWIN_PAUSE_MODE_RESUME_FILAMENT:
			Draw_Title(GET_TEXT_F(MSG_DWIN_FILAMENT_PRINT_RESUME));
			break;
		default:break;
	}
}

void Popup_window_Powerdown() {
  Clear_Main_Window();
  Draw_Popup_Bkgd_60();
  /*
  if (HMI_IsChinese()) {
    DWIN_Frame_AreaCopy(1, 0, 371, 33, 386, 85, 240);
    DWIN_Frame_AreaCopy(1, 203, 286, 271, 302, 118, 240);
    DWIN_Frame_AreaCopy(1, 0, 389, 150, 402, 61, 280);
  }
  else {
  */
    Draw_Title(GET_TEXT_F(MSG_DWIN_POWERDOWN_TITLE));
    DWIN_ICON_Show(ICON, ICON_POWER_DOWN, 86, 95);
  	DWIN_ICON_Show(ICON, ICON_NO_0, 26, 228);
	DWIN_ICON_Show(ICON, ICON_YES_1, 146, 228);
	DWIN_Draw_String(false, true, FIL.Font, FIL.Text_Color, FIL.Window_Color, (DWIN_WIDTH - FIL.Font_W * 17) / 2, 290, GET_TEXT_F(MSG_DWIN_POWERDOWN));
  //}
}


void Popup_window_Filament_Runout_Start(char mode) {
  Clear_Main_Window();
  Draw_Popup_Bkgd_60();
  DWIN_ICON_Show(ICON, ICON_FIL_OPTION, 86, 105);
  DRAW_Filament_Runout_Mode(mode);
  /*
  if (HMI_IsChinese()) {
    DWIN_Frame_AreaCopy(1, 0, 371, 33, 386, 85, 240);
    DWIN_Frame_AreaCopy(1, 203, 286, 271, 302, 118, 240);
    DWIN_Frame_AreaCopy(1, 0, 389, 150, 402, 61, 280);
  }
  else {
  */
    DWIN_Draw_String(false, true, FIL.Font, FIL.Text_Color, FIL.Window_Color, (DWIN_WIDTH - FIL.Font_W * 15) / 2, 240, GET_TEXT_F(MSG_DWIN_FILAMENT_CHANGE_START1));
	DWIN_Draw_String(false, true, FIL.Font, FIL.Text_Color, FIL.Window_Color, (DWIN_WIDTH - FIL.Font_W * 9) / 2, 269, GET_TEXT_F(MSG_DWIN_FILAMENT_CHANGE_START2));
  //}
}

void Popup_window_Filament_Runout_Heating(char mode) {
  Clear_Main_Window();
  Draw_Popup_Bkgd_60();
  DWIN_ICON_Show(ICON, ICON_FIL_OPTION, 86, 105);
  DRAW_Filament_Runout_Mode(mode);
  /*
  if (HMI_IsChinese()) {
    DWIN_Frame_AreaCopy(1, 0, 371, 33, 386, 85, 240);
    DWIN_Frame_AreaCopy(1, 203, 286, 271, 302, 118, 240);
    DWIN_Frame_AreaCopy(1, 0, 389, 150, 402, 61, 280);
  }
  else {
  */
    DWIN_Draw_String(false, true, FIL.Font, FIL.Text_Color, FIL.Window_Color, (DWIN_WIDTH - FIL.Font_W * 8) / 2, 240, GET_TEXT_F(MSG_DWIN_FILAMENT_CHANGE_HEATING1));
	DWIN_Draw_String(false, true, FIL.Font, FIL.Text_Color, FIL.Window_Color, (DWIN_WIDTH - FIL.Font_W * 17) / 2, 269, GET_TEXT_F(MSG_DWIN_FILAMENT_CHANGE_HEATING2));
  //}
}

void Popup_window_Filament_Runout_Insert(char mode) {
  Clear_Main_Window();
  Draw_Popup_Bkgd_60();
  //DWIN_ICON_Show(ICON, ICON_BLTouch, 101, 105);
   DRAW_Filament_Runout_Mode(mode);
  /*
  if (HMI_IsChinese()) {
    DWIN_Frame_AreaCopy(1, 0, 371, 33, 386, 85, 240);
    DWIN_Frame_AreaCopy(1, 203, 286, 271, 302, 118, 240);
    DWIN_Frame_AreaCopy(1, 0, 389, 150, 402, 61, 280);
  }
  else {
  */
  	DWIN_ICON_Show(ICON, ICON_Confirm_E, 86, 168);
  	DWIN_Draw_String(false, true, FIL.Font, FIL.Text_Color, FIL.Window_Color, (DWIN_WIDTH - FIL.Font_W * 15) / 2, 211, GET_TEXT_F(MSG_DWIN_FILAMENT_CHANGE_INSERT1));
    DWIN_Draw_String(false, true, FIL.Font, FIL.Text_Color, FIL.Window_Color, (DWIN_WIDTH - FIL.Font_W * 16) / 2, 240, GET_TEXT_F(MSG_DWIN_FILAMENT_CHANGE_INSERT2));
	DWIN_Draw_String(false, true, FIL.Font, FIL.Text_Color, FIL.Window_Color, (DWIN_WIDTH - FIL.Font_W * 12) / 2, 269, GET_TEXT_F(MSG_DWIN_FILAMENT_CHANGE_INSERT3));
  //}
}

void Popup_window_Filament_Runout_Unload(char mode) {
  Clear_Main_Window();
  Draw_Popup_Bkgd_60();
  DWIN_ICON_Show(ICON, ICON_FIL_OPTION, 86, 105);
  DRAW_Filament_Runout_Mode(mode);
  /*
  if (HMI_IsChinese()) {
    DWIN_Frame_AreaCopy(1, 0, 371, 33, 386, 85, 240);
    DWIN_Frame_AreaCopy(1, 203, 286, 271, 302, 118, 240);
    DWIN_Frame_AreaCopy(1, 0, 389, 150, 402, 61, 280);
  }
  else {
  */
    DWIN_Draw_String(false, true, FIL.Font, FIL.Text_Color, FIL.Window_Color, (DWIN_WIDTH - FIL.Font_W * 8) / 2, 240, GET_TEXT_F(MSG_DWIN_FILAMENT_CHANGE_UNLOAD1));
	DWIN_Draw_String(false, true, FIL.Font, FIL.Text_Color, FIL.Window_Color, (DWIN_WIDTH - FIL.Font_W * 16) / 2, 269, GET_TEXT_F(MSG_DWIN_FILAMENT_CHANGE_UNLOAD2));
  //}
}

void Popup_window_Filament_Runout_Load(char mode) {
  Clear_Main_Window();
  Draw_Popup_Bkgd_60();
  DWIN_ICON_Show(ICON, ICON_FIL_OPTION, 86, 105);
  DRAW_Filament_Runout_Mode(mode);
  /*
  if (HMI_IsChinese()) {
    DWIN_Frame_AreaCopy(1, 0, 371, 33, 386, 85, 240);
    DWIN_Frame_AreaCopy(1, 203, 286, 271, 302, 118, 240);
    DWIN_Frame_AreaCopy(1, 0, 389, 150, 402, 61, 280);
  }
  else {
  */
    DWIN_Draw_String(false, true, FIL.Font, FIL.Text_Color, FIL.Window_Color, (DWIN_WIDTH - FIL.Font_W * 8) / 2, 240, GET_TEXT_F(MSG_DWIN_FILAMENT_CHANGE_LOAD1));
	DWIN_Draw_String(false, true, FIL.Font, FIL.Text_Color, FIL.Window_Color, (DWIN_WIDTH - FIL.Font_W * 14) / 2, 269, GET_TEXT_F(MSG_DWIN_FILAMENT_CHANGE_LOAD2));
  //}
}

void Popup_window_Filament_Runout_Purge(char mode) {
  Clear_Main_Window();
  Draw_Popup_Bkgd_60();
  DWIN_ICON_Show(ICON, ICON_FIL_OPTION, 86, 105);
  DRAW_Filament_Runout_Mode(mode);
  /*
  if (HMI_IsChinese()) {
    DWIN_Frame_AreaCopy(1, 0, 371, 33, 386, 85, 240);
    DWIN_Frame_AreaCopy(1, 203, 286, 271, 302, 118, 240);
    DWIN_Frame_AreaCopy(1, 0, 389, 150, 402, 61, 280);
  }
  else {
  */
	DWIN_Draw_String(false, true, FIL.Font, FIL.Text_Color, FIL.Window_Color, (DWIN_WIDTH - FIL.Font_W * 8) / 2, 240, GET_TEXT_F(MSG_DWIN_FILAMENT_CHANGE_PURGE1));
	DWIN_Draw_String(false, true, FIL.Font, FIL.Text_Color, FIL.Window_Color, (DWIN_WIDTH - FIL.Font_W * 15) / 2, 269, GET_TEXT_F(MSG_DWIN_FILAMENT_CHANGE_PURGE2));
  //}
}

void Popup_window_Filament_Runout_Option(char mode) {
  Clear_Main_Window();
  Draw_Popup_Bkgd_60();
  //DWIN_ICON_Show(ICON, ICON_Leveling_0, 86, 105);
  DRAW_Filament_Runout_Mode(mode);
  /*
  if (HMI_IsChinese()) {
    DWIN_Frame_AreaCopy(1, 0, 371, 33, 386, 85, 240);
    DWIN_Frame_AreaCopy(1, 203, 286, 271, 302, 118, 240);
    DWIN_Frame_AreaCopy(1, 0, 389, 150, 402, 61, 280);
  }
  else {
  */
  	DWIN_ICON_Show(ICON, ICON_YES_0, 26, 168);
	DWIN_ICON_Show(ICON, ICON_NO_1, 146, 168);
	DWIN_Draw_String(false, true, FIL.Font, FIL.Text_Color, FIL.Window_Color, (DWIN_WIDTH - FIL.Font_W * 18) / 2, 240, GET_TEXT_F(MSG_DWIN_FILAMENT_CHANGE_OPTION));
  //}
}

void Popup_window_Filament_Runout_Resume(char mode) {
  Clear_Main_Window();
  Draw_Popup_Bkgd_60();
  DWIN_ICON_Show(ICON, ICON_Leveling_0, 86, 105);
  DRAW_Filament_Runout_Mode(mode);
  /*
  if (HMI_IsChinese()) {
    DWIN_Frame_AreaCopy(1, 0, 371, 33, 386, 85, 240);
    DWIN_Frame_AreaCopy(1, 203, 286, 271, 302, 118, 240);
    DWIN_Frame_AreaCopy(1, 0, 389, 150, 402, 61, 280);
  }
  else {
  */
	DWIN_Draw_String(false, true, FIL.Font, FIL.Text_Color, FIL.Window_Color, (DWIN_WIDTH - FIL.Font_W * 17) / 2, 240, GET_TEXT_F(MSG_DWIN_FILAMENT_CHANGE_RESUME));
  //}
}


void Draw_Printing_Screen() {
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Printing_Menu_Tune, Menu_Coordinate,Tune_X_Coordinate[HMI_flag.language], 325);
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Printing_Menu_Pause, Menu_Coordinate,Pause_X_Coordinate[HMI_flag.language], 325);
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Printing_Menu_Stop, Menu_Coordinate,Stop_X_Coordinate[HMI_flag.language], 325);
}

void Draw_Print_ProgressBar() {
  DWIN_ICON_Show(ICON, ICON_Bar, 15, 63);
  DWIN_Draw_Rectangle(1, BarFill_Color, 16 + Percentrecord * 240 / 100, 63, 256, 83);
  DWIN_Draw_IntValue(true, true, 0, font8x16, Percent_Color, BarFill_Color, 2, 117, 65, Percentrecord);
  DWIN_Draw_String(false, false, font8x16, Percent_Color, BarFill_Color, 133, 65, F("%"));
}

void Draw_Print_ProgressElapsed() {
  duration_t elapsed = print_job_timer.duration(); // print timer
  DWIN_Draw_IntValue(true, true, 1, font8x16, Color_White, Color_Bg_Black, 2, 42, 212, elapsed.value / 3600);
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, 58, 212, F(":"));
  DWIN_Draw_IntValue(true, true, 1, font8x16, Color_White, Color_Bg_Black, 2, 66, 212, (elapsed.value % 3600) / 60);
}

#if HAS_SUICIDE
void Draw_Powerdown_Machine() {		
  DWIN_Draw_Rectangle(1, Color_Bg_Black, 115, 325, 157, 353);
  if(HMI_flag.putdown_close_timer_rg >= 100)
  	DWIN_Draw_IntValue(true, true, 1, font14x28, Color_White, Color_Bg_Black, 3, 115, 325, HMI_flag.putdown_close_timer_rg);// Remaining time
  else if((HMI_flag.putdown_close_timer_rg < 100)&&(HMI_flag.putdown_close_timer_rg >= 10))
  	DWIN_Draw_IntValue(true, true, 1, font14x28, Color_White, Color_Bg_Black, 2, 122, 325, HMI_flag.putdown_close_timer_rg);// Remaining time
  else
  	DWIN_Draw_IntValue(true, true, 1, font14x28, Color_White, Color_Bg_Black, 1, 129, 325, HMI_flag.putdown_close_timer_rg);// Remaining time
}

void Draw_Freedown_Machine() {		
  DWIN_Draw_Rectangle(1, Color_Bg_Black, 230, 7, 256, 23);
  if(HMI_flag.free_close_timer_rg >= 100)
  	DWIN_Draw_IntValue(true, true, 1, font8x16, Color_White, Color_Bg_Black, 3, 230, 7, HMI_flag.free_close_timer_rg);// Remaining time
  else if((HMI_flag.free_close_timer_rg < 100)&&(HMI_flag.free_close_timer_rg >= 10))
  	DWIN_Draw_IntValue(true, true, 1, font8x16, Color_White, Color_Bg_Black, 2, 234, 7, HMI_flag.free_close_timer_rg);// Remaining time
  else
  	DWIN_Draw_IntValue(true, true, 1, font8x16, Color_White, Color_Bg_Black, 1, 238, 7, HMI_flag.free_close_timer_rg);// Remaining time
}
#endif

void Draw_Print_ProgressRemain() {
  DWIN_Draw_IntValue(true, true, 1, font8x16, Color_White, Color_Bg_Black, 2, 176, 212, remain_time / 3600);
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, 192, 212, F(":"));
  DWIN_Draw_IntValue(true, true, 1, font8x16, Color_White, Color_Bg_Black, 2, 200, 212, (remain_time % 3600) / 60);
}

void Draw_Print_ProgressZvaule() {
  DWIN_Draw_Signed_Float(DWIN_FONT_STAT, Color_Bg_Black, State_text_Zoffset_inum, State_text_Zoffset_fnum, State_text_Zoffset_X, State_text_Zoffset_Y, 100*current_position.z);
}

void Draw_Print_ProgressExtruder() {
  #if(MIXING_STEPPERS == 4)
  	DWIN_ICON_Show(ICON, ICON_Extruder1_P, 10, 98);
  	DWIN_ICON_Show(ICON, ICON_Extruder2_P, 62, 98);
  	DWIN_ICON_Show(ICON, ICON_Extruder3_P, 114, 98);
  	DWIN_ICON_Show(ICON, ICON_Extruder4_P, 166, 98);
  	DWIN_ICON_Show(ICON, ICON_VTool_P, 	218, 98);
  #elif(MIXING_STEPPERS == 3)
  	DWIN_ICON_Show(ICON, ICON_Extruder1_P, 21, 98);
  	DWIN_ICON_Show(ICON, ICON_Extruder2_P, 84, 98);
  	DWIN_ICON_Show(ICON, ICON_Extruder3_P, 147, 98);
  	DWIN_ICON_Show(ICON, ICON_VTool_P, 	210, 98);
  #elif(MIXING_STEPPERS == 2)
  	DWIN_ICON_Show(ICON, ICON_Extruder1_P, 36, 98);
  	DWIN_ICON_Show(ICON, ICON_Extruder2_P, 114, 98);
  	DWIN_ICON_Show(ICON, ICON_VTool_P, 	192, 98);
  #else
    DWIN_ICON_Show(ICON, ICON_Extruder1_P, 63, 98);
    DWIN_ICON_Show(ICON, ICON_VTool_P, 	168, 98);
  #endif
}

void Draw_Print_ProgressModel(){
    DWIN_Draw_Rectangle(1, Color_Bg_Black, 10, 455, 250, 471);
    DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, 10, 455, F("Model:"));
	if(MixerCfg.Mixer_Mode_Rg == 0) {
		DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, 58, 455, F("Mix"));
		DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 2, 91, 455, mixer.selected_vtool);
	}
	if(MixerCfg.Mixer_Mode_Rg == 1) {
		DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, 58, 455, F("Gradient"));
		DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 2, 130, 455, MixerCfg.Auto_VTool[VTOOL_START]);
		DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, 146, 455, F("--->"));
		DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 2, 178, 455, MixerCfg.Auto_VTool[VTOOL_END]);
	}
	if(MixerCfg.Mixer_Mode_Rg == 2) {
		DWIN_Draw_String(false, true, font8x16, Color_White, Color_Bg_Black, 58, 455, F("Random"));
	}
}

void Goto_PrintProcess() {
  checkkey = PrintProcess;

  Clear_Main_Window();
  Draw_Printing_Screen();
 
  //DWIN_ICON_Show(ICON, Title_Coordinate[HMI_flag.language][Title_Print], 14, 7);
  DWIN_JPG_CacheToN(1,HMI_flag.Title_Menu_Bankup);
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Title_Printing, Menu_Coordinate,14, 7);
  DWIN_JPG_CacheToN(1,HMI_flag.language+1);
  ICON_Tune();
  if (printingIsPaused()) ICON_Continue(); else ICON_Pause();
  ICON_Stop();
  
  // Copy into filebuf string before entry
  char * const name = card.longest_filename();
  const int8_t npos = _MAX(0U, DWIN_WIDTH - strlen(name) * MENU_CHR_W) / 2;
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, npos, 40, name);

  DWIN_ICON_Show(ICON, ICON_PrintTime, 17, 193);
  DWIN_ICON_Show(ICON, ICON_RemainTime, 150, 191);
  DWIN_ICON_Show(ICON, ICON_PRINT_TIME, 42, 193);
  DWIN_ICON_Show(ICON, ICON_REMAIN_TIME, 176, 193);
  
  Draw_Print_ProgressBar();
  Draw_Print_ProgressElapsed();
  Draw_Print_ProgressRemain();
  Draw_Print_ProgressZvaule();
  Draw_Print_ProgressExtruder();
  mixer.selected_vtool = MixerCfg.Vtool_Bankup;
  updata_mixer_from_vtool();
  Refresh_Percent_display();
  Draw_Print_ProgressModel();
}

void Goto_MainMenu() {
  checkkey = MainMenu;

  //Clear_Main_Window();
  DWIN_Draw_Rectangle(1, Color_Bg_Black, 0, 0, DWIN_WIDTH, DWIN_HEIGHT);
 
  //DWIN_JPG_CacheToN(1,HMI_flag.Title_Menu_Bankup);
  //DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Title_Main, Menu_Coordinate,14, 7);
  //DWIN_JPG_CacheToN(1,HMI_flag.language+1);
  DWIN_ICON_Show(ICON, ICON_LOGO, Logo_offset_X, Logo_offset_Y);

  ICON_Print();
  ICON_Prepare();
  ICON_Control();
  TERN(HAS_ONESTEP_LEVELING, ICON_Leveling, ICON_StartInfo)(select_page.now == 3);
  Draw_Status_Area(true);
  mixer.selected_vtool = MixerCfg.Vtool_Bankup;
  //Draw_Print_ProgressModel();
  #if HAS_SUICIDE
  	Draw_Freedown_Machine();
  #endif
}

inline ENCODER_DiffState get_encoder_state() {
  static millis_t Encoder_ms = 0;
  const millis_t ms = millis();
  if (PENDING(ms, Encoder_ms)) return ENCODER_DIFF_NO;
  const ENCODER_DiffState state = Encoder_ReceiveAnalyze();
  if (state != ENCODER_DIFF_NO) Encoder_ms = ms + ENCODER_WAIT;
  return state;
}

void HMI_Move_X() {
  ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
  if (encoder_diffState != ENCODER_DIFF_NO) {
    if (Apply_Encoder(encoder_diffState, HMI_ValueStruct.Move_X_scale)) {
      checkkey = AxisMove;
      EncoderRate.enabled = false;
	  //DWIN_Draw_FloatValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 1, 216, MBASE(MROWS -index_axismove + AXISMOVE_CASE_MOVEX), HMI_ValueStruct.Move_X_scale);
	  DWIN_Draw_Signed_Float(font8x16, Color_Bg_Black, 3, 1, 216, MBASE(MROWS -index_axismove + AXISMOVE_CASE_MOVEX), HMI_ValueStruct.Move_X_scale);
	  if (!planner.is_full()) {
        // Wait for planner moves to finish!
        planner.synchronize();
        planner.buffer_line(current_position, MMM_TO_MMS(HOMING_FEEDRATE_XY), active_extruder);
      }
      DWIN_UpdateLCD();
      return;
    }
    NOLESS(HMI_ValueStruct.Move_X_scale, (X_MIN_POS) * MINUNITMULT);
    NOMORE(HMI_ValueStruct.Move_X_scale, (X_MAX_POS) * MINUNITMULT);
    current_position.x = HMI_ValueStruct.Move_X_scale / 10;
    //DWIN_Draw_FloatValue(true, true, 0, font8x16, Color_White, Select_Color, 3, 1, 216, MBASE(MROWS -index_axismove + AXISMOVE_CASE_MOVEX), HMI_ValueStruct.Move_X_scale);
	DWIN_Draw_Signed_Float(font8x16, Select_Color, 3, 1, 216, MBASE(MROWS -index_axismove + AXISMOVE_CASE_MOVEX), HMI_ValueStruct.Move_X_scale);
	DWIN_UpdateLCD();
  }
}

void HMI_Move_Y() {
  ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
  if (encoder_diffState != ENCODER_DIFF_NO) {
    if (Apply_Encoder(encoder_diffState, HMI_ValueStruct.Move_Y_scale)) {
      checkkey = AxisMove;
      EncoderRate.enabled = false;
      //DWIN_Draw_FloatValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 1, 216, MBASE(MROWS -index_axismove + AXISMOVE_CASE_MOVEY), HMI_ValueStruct.Move_Y_scale);
	  DWIN_Draw_Signed_Float(font8x16, Color_Bg_Black, 3, 1, 216, MBASE(MROWS -index_axismove + AXISMOVE_CASE_MOVEY), HMI_ValueStruct.Move_Y_scale);
	  if (!planner.is_full()) {
        // Wait for planner moves to finish!
        planner.synchronize();
        planner.buffer_line(current_position, MMM_TO_MMS(HOMING_FEEDRATE_XY), active_extruder);
      }
      DWIN_UpdateLCD();
      return;
    }
    NOLESS(HMI_ValueStruct.Move_Y_scale, (Y_MIN_POS) * MINUNITMULT);
    NOMORE(HMI_ValueStruct.Move_Y_scale, (Y_MAX_POS) * MINUNITMULT);
    current_position.y = HMI_ValueStruct.Move_Y_scale / 10;
    //DWIN_Draw_FloatValue(true, true, 0, font8x16, Color_White, Select_Color, 3, 1, 216, MBASE(MROWS -index_axismove + AXISMOVE_CASE_MOVEY), HMI_ValueStruct.Move_Y_scale);
	DWIN_Draw_Signed_Float(font8x16, Select_Color, 3, 1, 216, MBASE(MROWS -index_axismove + AXISMOVE_CASE_MOVEY), HMI_ValueStruct.Move_Y_scale);
	DWIN_UpdateLCD();
  }
}

void HMI_Move_Z() {
  ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
  if (encoder_diffState != ENCODER_DIFF_NO) {
    if (Apply_Encoder(encoder_diffState, HMI_ValueStruct.Move_Z_scale)) {
      checkkey = AxisMove;
      EncoderRate.enabled = false;
      //DWIN_Draw_FloatValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 1, 216, MBASE(MROWS -index_axismove + AXISMOVE_CASE_MOVEZ), HMI_ValueStruct.Move_Z_scale);
	  DWIN_Draw_Signed_Float(font8x16, Color_Bg_Black, 3, 1, 216, MBASE(MROWS -index_axismove + AXISMOVE_CASE_MOVEZ), HMI_ValueStruct.Move_Z_scale);
	  if (!planner.is_full()) {
        // Wait for planner moves to finish!
        planner.synchronize();
        planner.buffer_line(current_position, MMM_TO_MMS(HOMING_FEEDRATE_Z), active_extruder);
      }
      DWIN_UpdateLCD();
      return;
    }
    NOLESS(HMI_ValueStruct.Move_Z_scale, Z_MIN_POS * MINUNITMULT);
    NOMORE(HMI_ValueStruct.Move_Z_scale, Z_MAX_POS * MINUNITMULT);
    current_position.z = HMI_ValueStruct.Move_Z_scale / 10;
    //DWIN_Draw_FloatValue(true, true, 0, font8x16, Color_White, Select_Color, 3, 1, 216, MBASE(MROWS -index_axismove + AXISMOVE_CASE_MOVEZ), HMI_ValueStruct.Move_Z_scale);
	DWIN_Draw_Signed_Float(font8x16, Select_Color, 3, 1, 216, MBASE(MROWS -index_axismove + AXISMOVE_CASE_MOVEZ), HMI_ValueStruct.Move_Z_scale);
	DWIN_UpdateLCD();
  }
}

void HMI_Adjust_Ext_Percent(uint8_t Extruder_Number) {
  ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
  if (encoder_diffState != ENCODER_DIFF_NO) {
    if (Apply_Encoder(encoder_diffState, MixerCfg.Manual_Percent[mixer.selected_vtool][Extruder_Number-1])) {
      checkkey = Mix_Manual;
      EncoderRate.enabled = false;
	  DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(Extruder_Number), MixerCfg.Manual_Percent[mixer.selected_vtool][Extruder_Number-1]);
      DWIN_UpdateLCD();
      return;
    }
    NOLESS(MixerCfg.Manual_Percent[mixer.selected_vtool][Extruder_Number-1], 0);
    NOMORE(MixerCfg.Manual_Percent[mixer.selected_vtool][Extruder_Number-1], 100);
    DWIN_Draw_IntValue(true, true, 0, font8x16, Select_Color, Color_Bg_Black, 3, 216, MBASE(Extruder_Number), MixerCfg.Manual_Percent[mixer.selected_vtool][Extruder_Number-1]);
	DWIN_UpdateLCD();
  }
}

void HMI_Adjust_Auto_Zpos_Start() {
	static float last_Z_scale = 0;
	ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();

	if (encoder_diffState != ENCODER_DIFF_NO) {
		if (Apply_Encoder(encoder_diffState, HMI_ValueStruct.Auto_Zstart_scale)) {
			checkkey = Mix_Auto;
			EncoderRate.enabled = false;
			last_Z_scale = HMI_ValueStruct.Auto_Zstart_scale;
			DWIN_Draw_Signed_Float(font8x16, Color_Bg_Black, 3, 1, 216, MBASE(MROWS -index_auto + AUTO_CASE_ZPOS_START), HMI_ValueStruct.Auto_Zstart_scale);
			DWIN_UpdateLCD();
			return;
	   }
		
	   if ((HMI_ValueStruct.Auto_Zstart_scale - last_Z_scale) > (Z_MAX_POS) * MINUNITMULT)
			HMI_ValueStruct.Auto_Zstart_scale = last_Z_scale + (Z_MAX_POS) * MINUNITMULT;
	   else if ((last_Z_scale - HMI_ValueStruct.Auto_Zstart_scale) > (Z_MAX_POS) * MINUNITMULT)
			HMI_ValueStruct.Auto_Zstart_scale = last_Z_scale - (Z_MAX_POS) * MINUNITMULT;
	     NOLESS(HMI_ValueStruct.Auto_Zstart_scale, 0);
		 MixerCfg.Auto_Zpos[VTOOL_START] = HMI_ValueStruct.Auto_Zstart_scale / 10;
		 DWIN_Draw_Signed_Float(font8x16, Select_Color, 3, 1, 216, MBASE(MROWS -index_auto + AUTO_CASE_ZPOS_START), HMI_ValueStruct.Auto_Zstart_scale);
		 DWIN_UpdateLCD();
	}
}

void HMI_Adjust_Auto_Zpos_End() {
	static float last_Z_scale = 0;
		ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
	
		if (encoder_diffState != ENCODER_DIFF_NO) {
			if (Apply_Encoder(encoder_diffState, HMI_ValueStruct.Auto_Zend_scale)) {
				checkkey = Mix_Auto;
				EncoderRate.enabled = false;
				last_Z_scale = HMI_ValueStruct.Auto_Zend_scale;
				DWIN_Draw_Signed_Float(font8x16, Color_Bg_Black, 3, 1, 216, MBASE(MROWS -index_auto + AUTO_CASE_ZPOS_END), HMI_ValueStruct.Auto_Zend_scale);
				DWIN_UpdateLCD();
				return;
		   }
			
		   if ((HMI_ValueStruct.Auto_Zend_scale - last_Z_scale) > (Z_MAX_POS) * MINUNITMULT)
				HMI_ValueStruct.Auto_Zend_scale = last_Z_scale + (Z_MAX_POS) * MINUNITMULT;
		   else if ((last_Z_scale - HMI_ValueStruct.Auto_Zend_scale) > (Z_MAX_POS) * MINUNITMULT)
				HMI_ValueStruct.Auto_Zend_scale = last_Z_scale - (Z_MAX_POS) * MINUNITMULT;
		     NOLESS(HMI_ValueStruct.Auto_Zend_scale, 0);
			 MixerCfg.Auto_Zpos[VTOOL_END] = HMI_ValueStruct.Auto_Zend_scale / 10;
			 DWIN_Draw_Signed_Float(font8x16, Select_Color, 3, 1, 216, MBASE(MROWS -index_auto + AUTO_CASE_ZPOS_END), HMI_ValueStruct.Auto_Zend_scale);
			 DWIN_UpdateLCD();
		}
}


void HMI_Adjust_Random_Zpos_Start() {
	static float last_Z_scale = 0;
	
	ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
	if (encoder_diffState != ENCODER_DIFF_NO) {
		if (Apply_Encoder(encoder_diffState, HMI_ValueStruct.Random_Zstart_scale)) {
			checkkey = Mix_Random;
			EncoderRate.enabled = false;
			last_Z_scale = HMI_ValueStruct.Random_Zstart_scale;
			DWIN_Draw_Signed_Float(font8x16, Color_Bg_Black, 3, 1, 216, MBASE(MROWS -index_random + RANDOM_CASE_ZPOS_START), HMI_ValueStruct.Random_Zstart_scale);
			DWIN_UpdateLCD();
			return;
	   }
		
	   if ((HMI_ValueStruct.Random_Zstart_scale - last_Z_scale) > (Z_MAX_POS) * MINUNITMULT)
			HMI_ValueStruct.Random_Zstart_scale = last_Z_scale + (Z_MAX_POS) * MINUNITMULT;
	   else if ((last_Z_scale - HMI_ValueStruct.Random_Zstart_scale) > (Z_MAX_POS) * MINUNITMULT)
			HMI_ValueStruct.Random_Zstart_scale = last_Z_scale - (Z_MAX_POS) * MINUNITMULT;
	     NOLESS(HMI_ValueStruct.Random_Zstart_scale, 0);
		 MixerCfg.Random_Zpos[VTOOL_START] = HMI_ValueStruct.Random_Zstart_scale / 10;
		 DWIN_Draw_Signed_Float(font8x16, Select_Color, 3, 1, 216, MBASE(MROWS -index_random + RANDOM_CASE_ZPOS_START), HMI_ValueStruct.Random_Zstart_scale);
		 DWIN_UpdateLCD();
	}
}

void HMI_Adjust_Random_Zpos_End() {
	static float last_Z_scale = 0;
	
		ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
		if (encoder_diffState != ENCODER_DIFF_NO) {
			if (Apply_Encoder(encoder_diffState, HMI_ValueStruct.Random_Zend_scale)) {
				checkkey = Mix_Random;
				EncoderRate.enabled = false;
				last_Z_scale = HMI_ValueStruct.Random_Zend_scale;
				DWIN_Draw_Signed_Float(font8x16, Color_Bg_Black, 3, 1, 216, MBASE(MROWS -index_random + RANDOM_CASE_ZPOS_END), HMI_ValueStruct.Random_Zend_scale);
				DWIN_UpdateLCD();
				return;
		   }
			
		   if ((HMI_ValueStruct.Random_Zend_scale - last_Z_scale) > (Z_MAX_POS) * MINUNITMULT)
				HMI_ValueStruct.Random_Zend_scale = last_Z_scale + (Z_MAX_POS) * MINUNITMULT;
		   else if ((last_Z_scale - HMI_ValueStruct.Random_Zend_scale) > (Z_MAX_POS) * MINUNITMULT)
				HMI_ValueStruct.Random_Zend_scale = last_Z_scale - (Z_MAX_POS) * MINUNITMULT;
		     NOLESS(HMI_ValueStruct.Random_Zend_scale, 0);
			 MixerCfg.Random_Zpos[VTOOL_END] = HMI_ValueStruct.Random_Zend_scale / 10;
			 DWIN_Draw_Signed_Float(font8x16, Select_Color, 3, 1, 216, MBASE(MROWS -index_random + RANDOM_CASE_ZPOS_END), HMI_ValueStruct.Random_Zend_scale);
			 DWIN_UpdateLCD();
		}
}


void HMI_Adjust_Auto_VTool_Start() {
  ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
  if (encoder_diffState != ENCODER_DIFF_NO) {
    if (Apply_Encoder(encoder_diffState, MixerCfg.Auto_VTool[VTOOL_START])) {
      checkkey = Mix_Auto;
      EncoderRate.enabled = false;
	  DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(MROWS -index_auto + AUTO_CASE_VTOOL_START), MixerCfg.Auto_VTool[VTOOL_START]);
	  mixer.selected_vtool = MixerCfg.Auto_VTool[VTOOL_START];      
	  updata_mixer_from_vtool();
	  MIXER_STEPPER_LOOP(i) MixerCfg.Start_Percent[i] = mixer.mix[i];
	  //Draw_Print_ProgressModel();
	  recovery.save(true);
	  DWIN_UpdateLCD();
      return;
    }
    NOLESS(MixerCfg.Auto_VTool[VTOOL_START], 0);
    NOMORE(MixerCfg.Auto_VTool[VTOOL_START], MixerCfg.occupy_vtool - 1);
    DWIN_Draw_IntValue(true, true, 0, font8x16, Select_Color, Color_Bg_Black, 3, 216, MBASE(MROWS -index_auto + AUTO_CASE_VTOOL_START), MixerCfg.Auto_VTool[VTOOL_START]);
	DWIN_UpdateLCD();
  }
}

void HMI_Adjust_Auto_VTool_End() {
  ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
  if (encoder_diffState != ENCODER_DIFF_NO) {
    if (Apply_Encoder(encoder_diffState, MixerCfg.Auto_VTool[VTOOL_END])) {
      checkkey = Mix_Auto;
      EncoderRate.enabled = false;
	  DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(MROWS -index_auto + AUTO_CASE_VTOOL_END), MixerCfg.Auto_VTool[VTOOL_END]);
	  mixer.selected_vtool = MixerCfg.Auto_VTool[VTOOL_END];      
	  updata_mixer_from_vtool();
	  MIXER_STEPPER_LOOP(i) MixerCfg.End_Percent[i] = mixer.mix[i]; 
	  //Draw_Print_ProgressModel();
	  recovery.save(true);
	  DWIN_UpdateLCD();
      return;
    }
    NOLESS(MixerCfg.Auto_VTool[VTOOL_END], 0);
    NOMORE(MixerCfg.Auto_VTool[VTOOL_END], MixerCfg.occupy_vtool - 1);
    DWIN_Draw_IntValue(true, true, 0, font8x16, Select_Color, Color_Bg_Black, 3, 216, MBASE(MROWS -index_auto + AUTO_CASE_VTOOL_END), MixerCfg.Auto_VTool[VTOOL_END]);
	DWIN_UpdateLCD();
  }
}

#if HAS_HOTEND
char E_Buf[50] = {0};
  	
  void HMI_Move_E1() {
    static float last_E_scale = 0;
	
    ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
    if (encoder_diffState != ENCODER_DIFF_NO) {
      if (Apply_Encoder(encoder_diffState, HMI_ValueStruct.Move_E1_scale)) {
        checkkey = AxisMove;
        EncoderRate.enabled = false;
        last_E_scale = HMI_ValueStruct.Move_E1_scale;
		DWIN_Draw_Signed_Float(font8x16, Color_Bg_Black, 3, 1, 216, MBASE(MROWS -index_axismove + AXISMOVE_CASE_EX1), HMI_ValueStruct.Move_E1_scale);
		if (!planner.is_full()) {
		  planner.synchronize(); // Wait for planner moves to finish!
          //planner.buffer_line(current_position, MMM_TO_MMS(FEEDRATE_E), active_extruder);
          ZERO(E_Buf);
		  if((HMI_flag.last_E_Coordinate - current_position.e) < 0)
          	sprintf_P(E_Buf, PSTR("T0\nG92 E0\nG1 E%.2f F200"),(current_position.e - HMI_flag.last_E_Coordinate));
		  else
		  	sprintf_P(E_Buf, PSTR("T0\nG92 E0\nG1 E-%.2f F200"),(HMI_flag.last_E_Coordinate - current_position.e));
		  queue.inject_P(E_Buf);
		  HMI_flag.last_E_Coordinate = current_position.e;
        }
        DWIN_UpdateLCD();
        return;
      }
      if ((HMI_ValueStruct.Move_E1_scale - last_E_scale) > (EXTRUDE_MAXLENGTH) * MINUNITMULT)
        HMI_ValueStruct.Move_E1_scale = last_E_scale + (EXTRUDE_MAXLENGTH) * MINUNITMULT;
      else if ((last_E_scale - HMI_ValueStruct.Move_E1_scale) > (EXTRUDE_MAXLENGTH) * MINUNITMULT)
        HMI_ValueStruct.Move_E1_scale = last_E_scale - (EXTRUDE_MAXLENGTH) * MINUNITMULT;
      current_position.e = HMI_ValueStruct.Move_E1_scale / 10;
      DWIN_Draw_Signed_Float(font8x16, Select_Color, 3, 1, 216, MBASE(MROWS -index_axismove + AXISMOVE_CASE_EX1), HMI_ValueStruct.Move_E1_scale);
      DWIN_UpdateLCD();
    }
  }
  
  //#if ENABLED (MIXING_EXTRUDER)
 	//#if((MIXING_STEPPERS == 2)||(MIXING_STEPPERS == 3)||(MIXING_STEPPERS == 4))
 	void HMI_Move_E2() {
    	static float last_E_scale = 0;
		
    	ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
    	if (encoder_diffState != ENCODER_DIFF_NO) {
      	if (Apply_Encoder(encoder_diffState, HMI_ValueStruct.Move_E2_scale)) {
        	checkkey = AxisMove;
        	EncoderRate.enabled = false;
        	last_E_scale = HMI_ValueStruct.Move_E2_scale;
			DWIN_Draw_Signed_Float(font8x16, Color_Bg_Black, 3, 1, 216, MBASE(MROWS -index_axismove + AXISMOVE_CASE_EX2), HMI_ValueStruct.Move_E2_scale);
        	if (!planner.is_full()) {
			planner.synchronize(); // Wait for planner moves to finish!
		  	//planner.buffer_line(current_position, MMM_TO_MMS(FEEDRATE_E), active_extruder);
		  	ZERO(E_Buf);
		  if((HMI_flag.last_E_Coordinate - current_position.e) < 0)
          	sprintf_P(E_Buf, PSTR("T1\nG92 E0\nG1 E%.2f F200"),(current_position.e - HMI_flag.last_E_Coordinate));
		  else
		  	sprintf_P(E_Buf, PSTR("T1\nG92 E0\nG1 E-%.2f F200"),(HMI_flag.last_E_Coordinate - current_position.e));
		  queue.inject_P(E_Buf);
		  HMI_flag.last_E_Coordinate = current_position.e;
        	}
        	DWIN_UpdateLCD();
        	return;
      	}
      	if ((HMI_ValueStruct.Move_E2_scale - last_E_scale) > (EXTRUDE_MAXLENGTH) * MINUNITMULT)
        	HMI_ValueStruct.Move_E2_scale = last_E_scale + (EXTRUDE_MAXLENGTH) * MINUNITMULT;
      	else if ((last_E_scale - HMI_ValueStruct.Move_E2_scale) > (EXTRUDE_MAXLENGTH) * MINUNITMULT)
        	HMI_ValueStruct.Move_E2_scale = last_E_scale - (EXTRUDE_MAXLENGTH) * MINUNITMULT;
      	current_position.e = HMI_ValueStruct.Move_E2_scale / 10;
      	DWIN_Draw_Signed_Float(font8x16, Select_Color, 3, 1, 216, MBASE(MROWS -index_axismove + AXISMOVE_CASE_EX2), HMI_ValueStruct.Move_E2_scale);
      	DWIN_UpdateLCD();
    	}
  	}
    //#endif
 //#endif
 
 //#if ENABLED (MIXING_EXTRUDER)
 	//#if((MIXING_STEPPERS == 3)||(MIXING_STEPPERS == 4))
 	void HMI_Move_E3() {
   	static float last_E_scale = 0;
    	ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
    	if (encoder_diffState != ENCODER_DIFF_NO) {
      	if (Apply_Encoder(encoder_diffState, HMI_ValueStruct.Move_E3_scale)) {
        	checkkey = AxisMove;
        	EncoderRate.enabled = false;
        	last_E_scale = HMI_ValueStruct.Move_E3_scale;
			DWIN_Draw_Signed_Float(font8x16, Color_Bg_Black, 3, 1, 216, MBASE(MROWS -index_axismove + AXISMOVE_CASE_EX3), HMI_ValueStruct.Move_E3_scale);
        	if (!planner.is_full()) {
          	planner.synchronize(); // Wait for planner moves to finish!
          	//planner.buffer_line(current_position, MMM_TO_MMS(FEEDRATE_E), active_extruder);
          	ZERO(E_Buf);
		  if((HMI_flag.last_E_Coordinate - current_position.e) < 0)
          	sprintf_P(E_Buf, PSTR("T2\nG92 E0\nG1 E%.2f F200"),(current_position.e - HMI_flag.last_E_Coordinate));
		  else
		  	sprintf_P(E_Buf, PSTR("T2\nG92 E0\nG1 E-%.2f F200"),(HMI_flag.last_E_Coordinate - current_position.e));
		  queue.inject_P(E_Buf);
		  HMI_flag.last_E_Coordinate = current_position.e;
        	}
        	DWIN_UpdateLCD();
        	return;
      	}
      	if ((HMI_ValueStruct.Move_E3_scale - last_E_scale) > (EXTRUDE_MAXLENGTH) * MINUNITMULT)
        	HMI_ValueStruct.Move_E3_scale = last_E_scale + (EXTRUDE_MAXLENGTH) * MINUNITMULT;
      	else if ((last_E_scale - HMI_ValueStruct.Move_E3_scale) > (EXTRUDE_MAXLENGTH) * MINUNITMULT)
        	HMI_ValueStruct.Move_E3_scale = last_E_scale - (EXTRUDE_MAXLENGTH) * MINUNITMULT;
      	current_position.e = HMI_ValueStruct.Move_E3_scale / 10;
      	DWIN_Draw_Signed_Float(font8x16, Select_Color, 3, 1, 216, MBASE(MROWS -index_axismove + AXISMOVE_CASE_EX3), HMI_ValueStruct.Move_E3_scale);
      	DWIN_UpdateLCD();
    	}
  	}
    //#endif
 //#endif
 
 //#if ENABLED (MIXING_EXTRUDER)
 	//#if(MIXING_STEPPERS == 4)
 	void HMI_Move_E4() {
    	static float last_E_scale = 0;
    	ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
    	if (encoder_diffState != ENCODER_DIFF_NO) {
      	if (Apply_Encoder(encoder_diffState, HMI_ValueStruct.Move_E4_scale)) {
        	checkkey = AxisMove;
        	EncoderRate.enabled = false;
        	last_E_scale = HMI_ValueStruct.Move_E4_scale;
			DWIN_Draw_Signed_Float(font8x16, Color_Bg_Black, 3, 1, 216, MBASE(MROWS -index_axismove + AXISMOVE_CASE_EX4), HMI_ValueStruct.Move_E4_scale);
        	if (!planner.is_full()) {
            planner.synchronize(); // Wait for planner moves to finish!
          	//planner.buffer_line(current_position, MMM_TO_MMS(FEEDRATE_E), active_extruder);
			ZERO(E_Buf);
		  if((HMI_flag.last_E_Coordinate - current_position.e) < 0)
          	sprintf_P(E_Buf, PSTR("T3\nG92 E0\nG1 E%.2f F200"),(current_position.e - HMI_flag.last_E_Coordinate));
		  else
		  	sprintf_P(E_Buf, PSTR("T3\nG92 E0\nG1 E-%.2f F200"),(HMI_flag.last_E_Coordinate - current_position.e));
		  queue.inject_P(E_Buf);
		  HMI_flag.last_E_Coordinate = current_position.e;
        	}
        	DWIN_UpdateLCD();
        	return;
      	}
      	if ((HMI_ValueStruct.Move_E4_scale - last_E_scale) > (EXTRUDE_MAXLENGTH) * MINUNITMULT)
        	HMI_ValueStruct.Move_E4_scale = last_E_scale + (EXTRUDE_MAXLENGTH) * MINUNITMULT;
      	else if ((last_E_scale - HMI_ValueStruct.Move_E4_scale) > (EXTRUDE_MAXLENGTH) * MINUNITMULT)
        	HMI_ValueStruct.Move_E4_scale = last_E_scale - (EXTRUDE_MAXLENGTH) * MINUNITMULT;
      	current_position.e = HMI_ValueStruct.Move_E4_scale / 10;
      	DWIN_Draw_Signed_Float(font8x16, Select_Color, 3, 1, 216, MBASE(MROWS -index_axismove + AXISMOVE_CASE_EX4), HMI_ValueStruct.Move_E4_scale);
      	DWIN_UpdateLCD();
    	}
  	}
	//#endif
 //#endif
#endif

#if HAS_ZOFFSET_ITEM
  bool printer_busy() { return planner.movesplanned() || printingIsActive(); }

  void HMI_Zoffset() {
    ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
    if (encoder_diffState != ENCODER_DIFF_NO) {
      uint8_t zoff_line;
      switch (HMI_ValueStruct.show_mode) {
        case -4: zoff_line = PREPARE_CASE_ZOFF + MROWS - index_prepare; break;
        default: zoff_line = TUNE_CASE_ZOFF + MROWS - index_tune;
      }
	  
      if (Apply_Encoder(encoder_diffState, HMI_ValueStruct.offset_value)) {
        EncoderRate.enabled = false;
        #if HAS_BED_PROBE
          probe.offset.z = dwin_zoffset;
          TERN_(EEPROM_SETTINGS, settings.save());
        #endif
        if (HMI_ValueStruct.show_mode == -4) {
          checkkey = Prepare;
          DWIN_Draw_Signed_Float(font8x16, Color_Bg_Black, 2, 2, 202, MBASE(zoff_line), TERN(HAS_BED_PROBE, BABY_Z_VAR * 100, HMI_ValueStruct.offset_value));
        }
        else {
          checkkey = Tune;
          DWIN_Draw_Signed_Float(font8x16, Color_Bg_Black, 2, 2, 202, MBASE(zoff_line), TERN(HAS_BED_PROBE, BABY_Z_VAR * 100, HMI_ValueStruct.offset_value));
        }
        DWIN_UpdateLCD();
        return;
      }
      NOLESS(HMI_ValueStruct.offset_value, (Z_PROBE_OFFSET_RANGE_MIN) * 100);
      NOMORE(HMI_ValueStruct.offset_value, (Z_PROBE_OFFSET_RANGE_MAX) * 100);
      last_zoffset = dwin_zoffset;
      dwin_zoffset = HMI_ValueStruct.offset_value / 100.0f;
      #if EITHER(BABYSTEP_ZPROBE_OFFSET, JUST_BABYSTEP)
        if ( (ENABLED(BABYSTEP_WITHOUT_HOMING) || all_axes_known()) && (ENABLED(BABYSTEP_ALWAYS_AVAILABLE) || printer_busy()) )
          babystep.add_mm(Z_AXIS, dwin_zoffset - last_zoffset);
      #endif
      DWIN_Draw_Signed_Float(font8x16, Select_Color, 2, 2, 202, MBASE(zoff_line), HMI_ValueStruct.offset_value);
      DWIN_UpdateLCD();
    }
  }

#endif // HAS_ZOFFSET_ITEM

#if HAS_HOTEND
  void HMI_ETemp() {
    ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
    if (encoder_diffState != ENCODER_DIFF_NO) {
      uint8_t temp_line;
      switch (HMI_ValueStruct.show_mode) {
        case -1: temp_line = TEMP_CASE_TEMP; break;
        case -2: temp_line = PREHEAT_CASE_TEMP; break;
        case -3: temp_line = PREHEAT_CASE_TEMP; break;
        default: temp_line = TUNE_CASE_TEMP + MROWS - index_tune;
      }
      if (Apply_Encoder(encoder_diffState, HMI_ValueStruct.E_Temp)) {
        EncoderRate.enabled = false;
        if (HMI_ValueStruct.show_mode == -1) { // temperature
          checkkey = TemperatureID;
          DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(temp_line), HMI_ValueStruct.E_Temp);
        }
        else if (HMI_ValueStruct.show_mode == -2) {
          checkkey = PLAPreheat;
          ui.material_preset[0].hotend_temp = HMI_ValueStruct.E_Temp;
          DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(temp_line), ui.material_preset[0].hotend_temp);
          return;
        }
        else if (HMI_ValueStruct.show_mode == -3) {
          checkkey = ABSPreheat;
          ui.material_preset[1].hotend_temp = HMI_ValueStruct.E_Temp;
          DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(temp_line), ui.material_preset[1].hotend_temp);
          return;
        }
        else { // tune
          checkkey = Tune;
          DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(temp_line), HMI_ValueStruct.E_Temp);
        }
        thermalManager.setTargetHotend(HMI_ValueStruct.E_Temp, 0);
        return;
      }
      // E_Temp limit
      NOMORE(HMI_ValueStruct.E_Temp, MAX_E_TEMP);
      NOLESS(HMI_ValueStruct.E_Temp, MIN_E_TEMP);
      // E_Temp value
      DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Select_Color, 3, 216, MBASE(temp_line), HMI_ValueStruct.E_Temp);
    }
  }
#endif // HAS_HOTEND

#if HAS_HEATED_BED
  void HMI_BedTemp() {
    ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
    if (encoder_diffState != ENCODER_DIFF_NO) {
      uint8_t bed_line;
      switch (HMI_ValueStruct.show_mode) {
        case -1: bed_line = TEMP_CASE_BED; break;
        case -2: bed_line = PREHEAT_CASE_BED; break;
        case -3: bed_line = PREHEAT_CASE_BED; break;
        default: bed_line = TUNE_CASE_BED + MROWS - index_tune;
      }
      if (Apply_Encoder(encoder_diffState, HMI_ValueStruct.Bed_Temp)) {
        EncoderRate.enabled = false;
        if (HMI_ValueStruct.show_mode == -1) {
          checkkey = TemperatureID;
          DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(bed_line), HMI_ValueStruct.Bed_Temp);
        }
        else if (HMI_ValueStruct.show_mode == -2) {
          checkkey = PLAPreheat;
          ui.material_preset[0].bed_temp = HMI_ValueStruct.Bed_Temp;
          DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(bed_line), ui.material_preset[0].bed_temp);
          return;
        }
        else if (HMI_ValueStruct.show_mode == -3) {
          checkkey = ABSPreheat;
          ui.material_preset[1].bed_temp = HMI_ValueStruct.Bed_Temp;
          DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(bed_line), ui.material_preset[1].bed_temp);
          return;
        }
        else {
          checkkey = Tune;
          DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(bed_line), HMI_ValueStruct.Bed_Temp);
        }
        thermalManager.setTargetBed(HMI_ValueStruct.Bed_Temp);
        return;
      }
      // Bed_Temp limit
      NOMORE(HMI_ValueStruct.Bed_Temp, BED_MAX_TARGET);
      NOLESS(HMI_ValueStruct.Bed_Temp, MIN_BED_TEMP);
      // Bed_Temp value
      DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Select_Color, 3, 216, MBASE(bed_line), HMI_ValueStruct.Bed_Temp);
    }
  }
#endif // HAS_HEATED_BED

#if HAS_PREHEAT
  void HMI_FanSpeed() {
    ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
    if (encoder_diffState != ENCODER_DIFF_NO) {
      uint8_t fan_line;
      switch (HMI_ValueStruct.show_mode) {
        case -1: fan_line = TEMP_CASE_FAN; break;
        case -2: fan_line = PREHEAT_CASE_FAN; break;
        case -3: fan_line = PREHEAT_CASE_FAN; break;
        default: fan_line = TUNE_CASE_FAN + MROWS - index_tune;
      }

      if (Apply_Encoder(encoder_diffState, HMI_ValueStruct.Fan_speed)) {
        EncoderRate.enabled = false;
        if (HMI_ValueStruct.show_mode == -1) {
          checkkey = TemperatureID;
          DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(fan_line), HMI_ValueStruct.Fan_speed);
        }
        else if (HMI_ValueStruct.show_mode == -2) {
          checkkey = PLAPreheat;
          ui.material_preset[0].fan_speed = HMI_ValueStruct.Fan_speed;
          DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(fan_line), ui.material_preset[0].fan_speed);
          return;
        }
        else if (HMI_ValueStruct.show_mode == -3) {
          checkkey = ABSPreheat;
          ui.material_preset[1].fan_speed = HMI_ValueStruct.Fan_speed;
          DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(fan_line), ui.material_preset[1].fan_speed);
          return;
        }
        else {
          checkkey = Tune;
          DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(fan_line), HMI_ValueStruct.Fan_speed);
        }
        thermalManager.set_fan_speed(0, HMI_ValueStruct.Fan_speed);
        return;
      }
      // Fan_speed limit
      NOMORE(HMI_ValueStruct.Fan_speed, FANON);
      NOLESS(HMI_ValueStruct.Fan_speed, FANOFF);
      // Fan_speed value
      DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Select_Color, 3, 216, MBASE(fan_line), HMI_ValueStruct.Fan_speed);
    }
  }
#endif // HAS_PREHEAT

void HMI_PrintSpeed() {
  ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
  if (encoder_diffState != ENCODER_DIFF_NO) {
    if (Apply_Encoder(encoder_diffState, HMI_ValueStruct.print_speed)) {
      checkkey = Tune;
      EncoderRate.enabled = false;
      feedrate_percentage = HMI_ValueStruct.print_speed;
      DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(select_tune.now + MROWS - index_tune), HMI_ValueStruct.print_speed);
      return;
    }
    // print_speed limit
    NOMORE(HMI_ValueStruct.print_speed, MAX_PRINT_SPEED);
    NOLESS(HMI_ValueStruct.print_speed, MIN_PRINT_SPEED);
    // print_speed value
    DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Select_Color, 3, 216, MBASE(select_tune.now + MROWS - index_tune), HMI_ValueStruct.print_speed);
  }
}

void HMI_MaxFeedspeedXYZE() {
  ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
  if (encoder_diffState != ENCODER_DIFF_NO) {
    if (Apply_Encoder(encoder_diffState, HMI_ValueStruct.Max_Feedspeed)) {
      checkkey = MaxSpeed;
      EncoderRate.enabled = false;
      if (WITHIN(HMI_flag.feedspeed_axis, X_AXIS, E_AXIS))
        planner.set_max_feedrate(HMI_flag.feedspeed_axis, HMI_ValueStruct.Max_Feedspeed);
      DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 4, 210, MBASE(select_speed.now), HMI_ValueStruct.Max_Feedspeed);
      return;
    }
    // MaxFeedspeed limit
    if (WITHIN(HMI_flag.feedspeed_axis, X_AXIS, E_AXIS))
      NOMORE(HMI_ValueStruct.Max_Feedspeed, default_max_feedrate[HMI_flag.feedspeed_axis] * 2);
    if (HMI_ValueStruct.Max_Feedspeed < MIN_MAXFEEDSPEED) HMI_ValueStruct.Max_Feedspeed = MIN_MAXFEEDSPEED;
    // MaxFeedspeed value
    DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Select_Color, 4, 210, MBASE(select_speed.now), HMI_ValueStruct.Max_Feedspeed);
  }
}

void HMI_MaxAccelerationXYZE() {
  ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
  if (encoder_diffState != ENCODER_DIFF_NO) {
    if (Apply_Encoder(encoder_diffState, HMI_ValueStruct.Max_Acceleration)) {
      checkkey = MaxAcceleration;
      EncoderRate.enabled = false;
      if (HMI_flag.acc_axis == X_AXIS) planner.set_max_acceleration(X_AXIS, HMI_ValueStruct.Max_Acceleration);
      else if (HMI_flag.acc_axis == Y_AXIS) planner.set_max_acceleration(Y_AXIS, HMI_ValueStruct.Max_Acceleration);
      else if (HMI_flag.acc_axis == Z_AXIS) planner.set_max_acceleration(Z_AXIS, HMI_ValueStruct.Max_Acceleration);
      #if HAS_HOTEND
        else if (HMI_flag.acc_axis == E_AXIS) planner.set_max_acceleration(E_AXIS, HMI_ValueStruct.Max_Acceleration);
      #endif
      DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 4, 210, MBASE(select_acc.now), HMI_ValueStruct.Max_Acceleration);
      return;
    }
    // MaxAcceleration limit
    if (WITHIN(HMI_flag.acc_axis, X_AXIS, E_AXIS))
      NOMORE(HMI_ValueStruct.Max_Acceleration, default_max_acceleration[HMI_flag.acc_axis] * 2);
    if (HMI_ValueStruct.Max_Acceleration < MIN_MAXACCELERATION) HMI_ValueStruct.Max_Acceleration = MIN_MAXACCELERATION;
    // MaxAcceleration value
    DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Select_Color, 4, 210, MBASE(select_acc.now), HMI_ValueStruct.Max_Acceleration);
  }
}

#if HAS_CLASSIC_JERK
  void HMI_MaxJerkXYZE() {
    ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
    if (encoder_diffState != ENCODER_DIFF_NO) {
      if (Apply_Encoder(encoder_diffState, HMI_ValueStruct.Max_Jerk)) {
        checkkey = MaxJerk;
        EncoderRate.enabled = false;
        if (WITHIN(HMI_flag.step_axis, X_AXIS, E_AXIS))
          planner.set_max_jerk(HMI_flag.step_axis, HMI_ValueStruct.Max_Jerk / 10);
        DWIN_Draw_FloatValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 1, 210, MBASE(select_jerk.now), HMI_ValueStruct.Max_Jerk);
        return;
      }
      // MaxJerk limit
      if (WITHIN(HMI_flag.jerk_axis, X_AXIS, E_AXIS))
        NOMORE(HMI_ValueStruct.Max_Jerk, default_max_jerk[HMI_flag.jerk_axis] * 2 * MINUNITMULT);
      NOLESS(HMI_ValueStruct.Max_Jerk, (MIN_MAXJERK) * MINUNITMULT);
      // MaxJerk value
      DWIN_Draw_FloatValue(true, true, 0, font8x16, Color_White, Select_Color, 3, 1, 210, MBASE(select_jerk.now), HMI_ValueStruct.Max_Jerk);
    }
  }
#endif // HAS_CLASSIC_JERK

void HMI_StepXYZE() {
  ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
  if (encoder_diffState != ENCODER_DIFF_NO) {
    if (Apply_Encoder(encoder_diffState, HMI_ValueStruct.Max_Step)) {
      checkkey = Step;
      EncoderRate.enabled = false;
      if (WITHIN(HMI_flag.step_axis, X_AXIS, E_AXIS))
        planner.settings.axis_steps_per_mm[HMI_flag.step_axis] = HMI_ValueStruct.Max_Step / 10;
      DWIN_Draw_FloatValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 1, 210, MBASE(select_step.now), HMI_ValueStruct.Max_Step);
      return;
    }
    // Step limit
    if (WITHIN(HMI_flag.step_axis, X_AXIS, E_AXIS))
      NOMORE(HMI_ValueStruct.Max_Step, default_axis_steps_per_unit[HMI_flag.step_axis] * 2 * MINUNITMULT);
    NOLESS(HMI_ValueStruct.Max_Step, MIN_STEP);
    // Step value
    DWIN_Draw_FloatValue(true, true, 0, font8x16, Color_White, Select_Color, 3, 1, 210, MBASE(select_step.now), HMI_ValueStruct.Max_Step);
  }
}

void X_Start_Coordinate_Calculation(uint8_t Extruder_number,uint16_t Coordinate){
	uint8_t j = 0;

	MIXER_STEPPER_LOOP(i){
		if(Coordinate >= 100) MixerDis.Extruder_X_Coordinate[i] = MixerDis.Extruder_X_Start_Coordinate[MIXING_STEPPERS]+MixerDis.Extruder_X_Start_Gap[MIXING_STEPPERS]*i, MixerDis.Extruder_Int_Number[i] = 3;
		if((Coordinate < 100)&&(Coordinate >= 10)) MixerDis.Extruder_X_Coordinate[i] = MixerDis.Extruder_X_Start_Coordinate[MIXING_STEPPERS]+7+MixerDis.Extruder_X_Start_Gap[MIXING_STEPPERS]*i, MixerDis.Extruder_Int_Number[i] = 2;
		if(Coordinate < 10) MixerDis.Extruder_X_Coordinate[i] = MixerDis.Extruder_X_Start_Coordinate[MIXING_STEPPERS]+14+MixerDis.Extruder_X_Start_Gap[MIXING_STEPPERS]*i, MixerDis.Extruder_Int_Number[i] = 1;
		j = i;
    }
	j++;
	if(Extruder_number == MIXING_STEPPERS){
		if((Coordinate < 100)&&(Coordinate >= 10)) MixerDis.VTool_X_Coordinate = MixerDis.Extruder_X_Start_Coordinate[MIXING_STEPPERS]+7+MixerDis.Extruder_X_Start_Gap[MIXING_STEPPERS]*j, MixerDis.VTool_Int_Number = 2;
		if(Coordinate < 10) MixerDis.VTool_X_Coordinate = MixerDis.Extruder_X_Start_Coordinate[MIXING_STEPPERS]+14+MixerDis.Extruder_X_Start_Gap[MIXING_STEPPERS]*j, MixerDis.VTool_Int_Number = 1;
	}
}

void Refresh_Percent_display(){
	DWIN_Draw_Rectangle(1, Color_Bg_Black, MixerDis.Area_X_Start, MixerDis.Area_Y_Start, MixerDis.Area_X_End, MixerDis.Area_Y_End);
	MIXER_STEPPER_LOOP(i){
		X_Start_Coordinate_Calculation(i,mixer.mix[i]);
	    DWIN_Draw_IntValue(true, true, 0, DWIN_FONT_MIX, Color_White, Color_Bg_Black, MixerDis.Extruder_Int_Number[i], MixerDis.Extruder_X_Coordinate[i], MixerDis.Y_Coordinate, mixer.mix[i]);
		if(MixerCfg.Mixer_Mode_Rg == 0){
			X_Start_Coordinate_Calculation(MIXING_STEPPERS,MixerCfg.Vtool_Bankup);
			DWIN_Draw_IntValue(true, true, 0, DWIN_FONT_MIX, Color_White, Color_Bg_Black, MixerDis.VTool_Int_Number, MixerDis.VTool_X_Coordinate, MixerDis.Y_Coordinate, MixerCfg.Vtool_Bankup);
		}
		else if(MixerCfg.Mixer_Mode_Rg == 1 || MixerCfg.Mixer_Mode_Rg == 2){
			X_Start_Coordinate_Calculation(MIXING_STEPPERS,MixerCfg.occupy_vtool);
			DWIN_Draw_IntValue(true, true, 0, DWIN_FONT_MIX, Color_White, Color_Bg_Black, MixerDis.VTool_Int_Number, MixerDis.VTool_X_Coordinate, MixerDis.Y_Coordinate, MixerCfg.occupy_vtool);
		}
	}
}

void update_variable() {
  #if HAS_HOTEND
    static float last_temp_hotend_target = 0, last_temp_hotend_current = 0;
  #endif
  #if HAS_HEATED_BED
    static float last_temp_bed_target = 0, last_temp_bed_current = 0;
  #endif
  #if HAS_FAN
    static uint8_t last_fan_speed = 0;
  #endif
  #if ENABLED(MIXING_EXTRUDER)
  	static uint8_t last_mixer_percent[MIXING_STEPPERS] = {0};
    static uint8_t last_vtool = 0;
  #endif

  /* Tune page temperature update */
  if (checkkey == Tune) {
    #if HAS_HOTEND
      if (last_temp_hotend_target != thermalManager.temp_hotend[0].target)
        DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(TUNE_CASE_TEMP + MROWS - index_tune), thermalManager.temp_hotend[0].target);
    #endif
    #if HAS_HEATED_BED
      if (last_temp_bed_target != thermalManager.temp_bed.target)
        DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(TUNE_CASE_BED + MROWS - index_tune), thermalManager.temp_bed.target);
    #endif
    #if HAS_FAN
      if (last_fan_speed != thermalManager.fan_speed[0]) {
        DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(TUNE_CASE_FAN + MROWS - index_tune), thermalManager.fan_speed[0]);
        last_fan_speed = thermalManager.fan_speed[0];
      }
    #endif
  }

  /* Temperature page temperature update */
  if (checkkey == TemperatureID) {
    #if HAS_HOTEND
      if (last_temp_hotend_target != thermalManager.temp_hotend[0].target)
        DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(TEMP_CASE_TEMP), thermalManager.temp_hotend[0].target);
    #endif
    #if HAS_HEATED_BED
      if (last_temp_bed_target != thermalManager.temp_bed.target)
        DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(TEMP_CASE_BED), thermalManager.temp_bed.target);
    #endif
    #if HAS_FAN
      if (last_fan_speed != thermalManager.fan_speed[0]) {
        DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(TEMP_CASE_FAN), thermalManager.fan_speed[0]);
        last_fan_speed = thermalManager.fan_speed[0];
      }
    #endif
  }

  /* Bottom temperature update */
  #if HAS_HOTEND
    if (last_temp_hotend_current != thermalManager.temp_hotend[0].celsius) {
      DWIN_Draw_IntValue(true, true, 0, DWIN_FONT_STAT, Color_White, Color_Bg_Black, State_text_extruder_num, State_text_extruder_X, State_text_extruder_Y, thermalManager.temp_hotend[0].celsius);
      last_temp_hotend_current = thermalManager.temp_hotend[0].celsius;
    }
    if (last_temp_hotend_target != thermalManager.temp_hotend[0].target) {
      DWIN_Draw_IntValue(true, true, 0, DWIN_FONT_STAT, Color_White, Color_Bg_Black, State_text_extruder_num, State_text_extruder_X + (State_text_extruder_num + 1) * STAT_CHR_W, State_text_extruder_Y, thermalManager.temp_hotend[0].target);
      last_temp_hotend_target = thermalManager.temp_hotend[0].target;
    }
  #endif
  
  #if HAS_HEATED_BED
    if (last_temp_bed_current != thermalManager.temp_bed.celsius) {
      DWIN_Draw_IntValue(true, true, 0, DWIN_FONT_STAT, Color_White, Color_Bg_Black, State_text_bed_num, State_text_bed_X, State_text_bed_Y, thermalManager.temp_bed.celsius);
      last_temp_bed_current = thermalManager.temp_bed.celsius;
    }
    if (last_temp_bed_target != thermalManager.temp_bed.target) {
      DWIN_Draw_IntValue(true, true, 0, DWIN_FONT_STAT, Color_White, Color_Bg_Black, State_text_bed_num, State_text_bed_X + (State_text_bed_num + 1) * STAT_CHR_W, State_text_bed_Y, thermalManager.temp_bed.target);
      last_temp_bed_target = thermalManager.temp_bed.target;
    }
  #endif
  
  static uint16_t last_speed = 0;
  if (last_speed != feedrate_percentage) {
    DWIN_Draw_IntValue(true, true, 0, DWIN_FONT_STAT, Color_White, Color_Bg_Black, State_text_speed_num, State_text_speed_X, State_text_speed_Y, feedrate_percentage);
    last_speed = feedrate_percentage;
  }
  #if HAS_ZOFFSET_ITEM
    if(checkkey != PrintProcess){
		if (last_zoffset != BABY_Z_VAR)
		DWIN_Draw_Signed_Float(DWIN_FONT_STAT, Color_Bg_Black, State_text_Zoffset_inum, State_text_Zoffset_fnum, State_text_Zoffset_X, State_text_Zoffset_Y, BABY_Z_VAR * 100);
	    last_zoffset = BABY_Z_VAR;
    	}
	else {
		if (last_zoffset != current_position.z)
		DWIN_Draw_Signed_Float(DWIN_FONT_STAT, Color_Bg_Black, State_text_Zoffset_inum, State_text_Zoffset_fnum, State_text_Zoffset_X, State_text_Zoffset_Y, 100*current_position.z);
	    last_zoffset = current_position.z;
		}
  #endif

  #if ENABLED(MIXING_EXTRUDER)
  if((checkkey == PrintProcess)&&(!HMI_flag.filament_runout_star)) {
	for(uint8_t j=0; j<MIXING_STEPPERS; j++)	{
	    if((last_mixer_percent[j] != mixer.mix[j])||(last_vtool != MixerCfg.Vtool_Bankup))
		{
	    	last_vtool = MixerCfg.Vtool_Bankup;
			MIXER_STEPPER_LOOP(i) last_mixer_percent[i] = mixer.mix[i];
    		Refresh_Percent_display();
			break;
	    }
	}
 }	
  #endif
}

/**
 * Read and cache the working directory.
 *
 * TODO: New code can follow the pattern of menu_media.cpp
 * and rely on Marlin caching for performance. No need to
 * cache files here.
 */

#ifndef strcasecmp_P
  #define strcasecmp_P(a, b) strcasecmp((a), (b))
#endif

inline void make_name_without_ext(char *dst, char *src, size_t maxlen=MENU_CHAR_LIMIT) {
  char * const name = card.longest_filename();
  size_t pos        = strlen(name); // index of ending nul

  // For files, remove the extension
  // which may be .gcode, .gco, or .g
  if (!card.flag.filenameIsDir)
    while (pos && src[pos] != '.') pos--; // find last '.' (stop at 0)

  size_t len = pos;   // nul or '.'
  if (len > maxlen) { // Keep the name short
    pos        = len = maxlen; // move nul down
    dst[--pos] = '.'; // insert dots
    dst[--pos] = '.';
    dst[--pos] = '.';
  }

  dst[len] = '\0';    // end it

  // Copy down to 0
  while (pos--) dst[pos] = src[pos];
}

inline void HMI_SDCardInit() { card.cdroot(); }

void MarlinUI::refresh() { /* Nothing to see here */ }

#define ICON_Folder ICON_More

#if ENABLED(SCROLL_LONG_FILENAMES)

  char shift_name[LONG_FILENAME_LENGTH + 1];
  int8_t shift_amt; // = 0
  millis_t shift_ms; // = 0

  // Init the shift name based on the highlighted item
  inline void Init_Shift_Name() {
    const bool is_subdir = !card.flag.workDirIsRoot;
    const int8_t filenum = select_file.now - 1 - is_subdir; // Skip "Back" and ".."
    const uint16_t fileCnt = card.get_num_Files();
    if (WITHIN(filenum, 0, fileCnt - 1)) {
      card.getfilename_sorted(SD_ORDER(filenum, fileCnt));
      char * const name = card.longest_filename();
      make_name_without_ext(shift_name, name, 100);
    }
  }

  inline void Init_SDItem_Shift() {
    shift_amt = 0;
    shift_ms  = select_file.now > 0 && strlen(shift_name) > MENU_CHAR_LIMIT
           ? millis() + 750UL : 0;
  }
#endif

/**
 * Display an SD item, adding a CDUP for subfolders.
 */
inline void Draw_SDItem(const uint16_t item, int16_t row=-1) {
  if (row < 0) row = item + 1 + MROWS - index_file;
  const bool is_subdir = !card.flag.workDirIsRoot;
  if (is_subdir && item == 0) {
    Draw_Menu_Line(row, ICON_Folder, "..");
    return;
  }

  card.getfilename_sorted(item - is_subdir);
  char * const name = card.longest_filename();

  #if ENABLED(SCROLL_LONG_FILENAMES)
    // Init the current selected name
    // This is used during scroll drawing
    if (item == select_file.now - 1) {
      make_name_without_ext(shift_name, name, 100);
      Init_SDItem_Shift();
    }
  #endif

  // Draw the file/folder with name aligned left
  char str[strlen(name) + 1];
  make_name_without_ext(str, name);
  Draw_Menu_Line(row, card.flag.filenameIsDir ? ICON_Folder : ICON_File, str);
}

#if ENABLED(SCROLL_LONG_FILENAMES)
  inline void Draw_SDItem_Shifted(int8_t &shift) {
    // Limit to the number of chars past the cutoff
    const size_t len = strlen(shift_name);
    NOMORE(shift, _MAX(len - MENU_CHAR_LIMIT, 0U));

    // Shorten to the available space
    const size_t lastchar = _MIN((signed)len, shift + MENU_CHAR_LIMIT);

    const char c = shift_name[lastchar];
    shift_name[lastchar] = '\0';

    const uint8_t row = select_file.now + MROWS - index_file; // skip "Back" and scroll
    Erase_Menu_Text(row);
    Draw_Menu_Line(row, 0, &shift_name[shift]);

    shift_name[lastchar] = c;
  }
#endif

// Redraw the first set of SD Files
inline void Redraw_SD_List() {
  select_file.reset();
  index_file = MROWS;

  Clear_Menu_Area(); // Leave title bar unchanged

  Draw_Back_First();

  card.mount();
  
  if (card.isMounted()) {
  //if (card.flag.mounted) {
    // As many files as will fit
    LOOP_L_N(i, _MIN(nr_sd_menu_items(), MROWS))
      Draw_SDItem(i, i+1);

    TERN_(SCROLL_LONG_FILENAMES, Init_SDItem_Shift());
  }
  else {
    DWIN_Draw_Rectangle(1, Color_Bg_Red, 10, MBASE(3) - 10, DWIN_WIDTH - 10, MBASE(4));
    DWIN_Draw_String(false, false, font16x32, Color_Yellow, Color_Bg_Red, ((DWIN_WIDTH) - 8 * 16) / 2, MBASE(3), F("No Media"));
  }
}

bool DWIN_lcd_sd_status = false;

inline void SDCard_Up() {
  card.cdup();
  Redraw_SD_List();
  DWIN_lcd_sd_status = false; // On next DWIN_Update
}

inline void SDCard_Folder(char * const dirname) {
  card.cd(dirname);
  Redraw_SD_List();
  DWIN_lcd_sd_status = false; // On next DWIN_Update
}

//
// Watch for media mount / unmount
//
void HMI_SDCardUpdate() {
  if (HMI_flag.home_flag) return;
  if (DWIN_lcd_sd_status != card.isMounted()) {
    DWIN_lcd_sd_status = card.isMounted();
    // SERIAL_ECHOLNPAIR("HMI_SDCardUpdate: ", int(DWIN_lcd_sd_status));
    if (DWIN_lcd_sd_status) {
      //if (checkkey == SelectFile)
       // Redraw_SD_List();
    }
    else {
      // clean file icon
      //if (checkkey == SelectFile) {
        //Redraw_SD_List();
      //}
      //else if (checkkey == PrintProcess || checkkey == Tune || printingIsActive()) {
      if (checkkey == PrintProcess || checkkey == Tune || printingIsActive()) {
        // TODO: Move card removed abort handling
        //       to CardReader::manage_media.
        card.flag.abort_sd_printing = true;
        wait_for_heatup = wait_for_user = false;
        dwin_abort_flag = true; // Reset feedrate, return to Home
      }
    }
    DWIN_UpdateLCD();
  }
}

//
// The status area is always on-screen, except during
// full-screen modal dialogs. (TODO: Keep alive during dialogs)
//
void Draw_Status_Area(const bool with_update) {
  // Clear the bottom area of the screen
  DWIN_Draw_Rectangle(1, Color_Bg_Black, 0, STATUS_Y_START, DWIN_WIDTH, STATUS_Y_END);
  //
  // Status Area
  //
  #if HAS_HOTEND
    DWIN_ICON_Show(ICON, ICON_HotendTemp, State_icon_extruder_X, State_icon_extruder_Y);
    DWIN_Draw_IntValue(true, true, 0, DWIN_FONT_STAT, Color_White, Color_Bg_Black, State_text_extruder_num, State_text_extruder_X, State_text_extruder_Y, thermalManager.temp_hotend[0].celsius);
    DWIN_Draw_String(false, false, DWIN_FONT_STAT, Color_White, Color_Bg_Black, State_string_extruder_X, State_string_extruder_Y, F("/"));
    DWIN_Draw_IntValue(true, true, 0, DWIN_FONT_STAT, Color_White, Color_Bg_Black, State_text_extruder_num, State_text_extruder_X + (State_text_extruder_num + 1) * STAT_CHR_W, State_text_extruder_Y, thermalManager.temp_hotend[0].target);
  #endif
  #if HOTENDS > 1
    // DWIN_ICON_Show(ICON,ICON_HotendTemp, 13, 381);
  #endif

  #if HAS_HEATED_BED
    DWIN_ICON_Show(ICON, ICON_BedTemp, State_icon_bed_X, State_icon_bed_Y);
    DWIN_Draw_IntValue(true, true, 0, DWIN_FONT_STAT, Color_White, Color_Bg_Black, State_text_bed_num, State_text_bed_X, State_text_bed_Y, thermalManager.temp_bed.celsius);
    DWIN_Draw_String(false, false, DWIN_FONT_STAT, Color_White, Color_Bg_Black, State_string_bed_X, State_string_bed_Y, F("/"));
    DWIN_Draw_IntValue(true, true, 0, DWIN_FONT_STAT, Color_White, Color_Bg_Black, State_text_bed_num, State_text_bed_X + (State_text_extruder_num + 1) * STAT_CHR_W, State_text_bed_Y, thermalManager.temp_bed.target);
  #endif

  DWIN_ICON_Show(ICON, ICON_Speed, State_icon_speed_X, State_icon_speed_Y);
  DWIN_Draw_IntValue(true, true, 0, DWIN_FONT_STAT, Color_White, Color_Bg_Black, State_text_speed_num, State_text_speed_X, State_text_speed_Y, feedrate_percentage);
  DWIN_Draw_String(false, false, DWIN_FONT_STAT, Color_White, Color_Bg_Black, State_string_speed_X, State_string_speed_Y, F("%"));
  
  #if HAS_ZOFFSET_ITEM
    DWIN_ICON_Show(ICON, ICON_Zoffset, State_icon_Zoffset_X, State_icon_Zoffset_Y);
    DWIN_Draw_Signed_Float(DWIN_FONT_STAT, Color_Bg_Black, State_text_Zoffset_inum, State_text_Zoffset_fnum, State_text_Zoffset_X, State_text_Zoffset_Y, BABY_Z_VAR * 100);
  #endif

  if (with_update) {
    DWIN_UpdateLCD();
    delay(5);
  }
}

void Draw_Mixer_Status_Area(const bool with_update) {
  //
  // Mixer Status Area
  //
  #if ENABLED(MIXING_EXTRUDER)
   	mixer.selected_vtool = MixerCfg.Vtool_Bankup;
  	updata_mixer_from_vtool();
  	Refresh_Percent_display();
  #endif

  if (with_update) {
    DWIN_UpdateLCD();
    delay(5);
  }
}


void HMI_StartFrame(const bool with_update) {
  Goto_MainMenu();
  Draw_Status_Area(with_update);
  mixer.selected_vtool = MixerCfg.Vtool_Bankup;
  //Draw_Print_ProgressModel();
}

inline void Draw_Info_Menu() {
  Clear_Main_Window();
  /*
  //DWIN_ICON_Show(ICON, Title_Coordinate[HMI_flag.language][Title_StarInfo], 14, 7);
  DWIN_JPG_CacheToN(1,HMI_flag.Title_Menu_Bankup);
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Title_StarInfo, Menu_Coordinate,14, 7);
  DWIN_JPG_CacheToN(1,HMI_flag.language+1);
  
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Info_Menu_Size, Menu_Coordinate,120, 102);
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, (DWIN_WIDTH - strlen(MACHINE_SIZE) * MENU_CHR_W) / 2, 122, (char*)MACHINE_SIZE);
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Info_Menu_Version, Menu_Coordinate,108, 175);
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, (DWIN_WIDTH - strlen(SHORT_BUILD_VERSION) * MENU_CHR_W) / 2, 195, (char*)SHORT_BUILD_VERSION);
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Info_Menu_Website, Menu_Coordinate,105, 248);
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, (DWIN_WIDTH - strlen(CORP_WEBSITE_E) * MENU_CHR_W) / 2, 268, (char*)CORP_WEBSITE_E);
  
  Draw_Back_First();
  LOOP_L_N(i, 3) {
    DWIN_ICON_Show(ICON, ICON_PrintSize + i, 26, 99 + i * 73);
    DWIN_Draw_Line(Line_Color, 16, MBASE(2) + i * 73, 256, 156 + i * 73);
  }
  */
  DWIN_JPG_CacheToN(1,HMI_flag.Title_Menu_Bankup);
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Title_StarInfo, Menu_Coordinate,14, 7);
  DWIN_JPG_CacheToN(1,HMI_flag.language+1);
  /*
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Info_Menu_Size, Menu_Coordinate,120, 102);
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, (DWIN_WIDTH - strlen(MACHINE_SIZE) * MENU_CHR_W) / 2, 122, (char*)MACHINE_SIZE);
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Info_Menu_Version, Menu_Coordinate,108, 175);
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, (DWIN_WIDTH - strlen(SHORT_BUILD_VERSION) * MENU_CHR_W) / 2, 195, (char*)SHORT_BUILD_VERSION);
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Info_Menu_Website, Menu_Coordinate,105, 248);
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, (DWIN_WIDTH - strlen(CORP_WEBSITE_E) * MENU_CHR_W) / 2, 268, (char*)CORP_WEBSITE_E);
  */
  /*
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, (DWIN_WIDTH - strlen(FIRMWARE_VERSION) * MENU_CHR_W) / 2, 96, (char*)FIRMWARE_VERSION);
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, (DWIN_WIDTH - strlen(SHORT_BUILD_VERSION) * MENU_CHR_W) / 2, 138, (char*)SHORT_BUILD_VERSION);
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, (DWIN_WIDTH - strlen(WEBSITE_URL) * MENU_CHR_W) / 2, 180, (char*)WEBSITE_URL);
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, (DWIN_WIDTH - strlen(CUSTOM_MACHINE_NAME) * MENU_CHR_W) / 2, 222, (char*)CUSTOM_MACHINE_NAME);
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, (DWIN_WIDTH - strlen(BOARD_INFO_NAME) * MENU_CHR_W) / 2, 264, (char*)BOARD_INFO_NAME);
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, (DWIN_WIDTH - strlen(STRING_DISTRIBUTION_DATE) * MENU_CHR_W) / 2, 306, (char*)STRING_DISTRIBUTION_DATE);
  */
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, 5, 96, F("Version:"));
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, 100, 96, (char*)FIRMWARE_VERSION);
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, 5, 138, F("Firmware:"));
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, 100, 138, (char*)SHORT_BUILD_VERSION);
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, 5, 180, F("Website:"));
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, 100, 180, (char*)WEBSITE_URL);
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, 5, 222, F("Model:"));
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, 100, 222, (char*)CUSTOM_MACHINE_NAME);
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, 5, 264, F("Board:"));
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, 100, 264, (char*)BOARD_INFO_NAME);
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, 5, 306, F("Date:"));
  DWIN_Draw_String(false, false, font8x16, Color_White, Color_Bg_Black, 100, 306, (char*)STRING_DISTRIBUTION_DATE);
  Draw_Back_First();
  LOOP_L_N(i, 6) {
    DWIN_Draw_Line(Line_Color, 0, 83 + (i+1) * 42, 256, 84 + (i+1) * 42);
  }
}

inline void Draw_Print_File_Menu() {
  Clear_Title_Bar();

  //DWIN_ICON_Show(ICON, Title_Coordinate[HMI_flag.language][Title_Print_File], 14, 7);
  DWIN_JPG_CacheToN(1,HMI_flag.Title_Menu_Bankup);
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Title_Print, Menu_Coordinate,14, 7);
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Title_File, Menu_Coordinate,14+Print_File_X_Coordinate[HMI_flag.language], 7);
  DWIN_JPG_CacheToN(1,HMI_flag.language+1);
  Redraw_SD_List();
}

/* Main Process */
void HMI_MainMenu() {
  ENCODER_DiffState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;

  if (encoder_diffState == ENCODER_DIFF_CW) {
    if (select_page.inc(4)) {
      switch (select_page.now) {
        case 0: ICON_Print(); break;
        case 1: ICON_Print(); ICON_Prepare(); break;
        case 2: ICON_Prepare(); ICON_Control(); break;
        case 3: ICON_Control(); TERN(HAS_ONESTEP_LEVELING, ICON_Leveling, ICON_StartInfo)(1); break;
      }
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW) {
    if (select_page.dec()) {
      switch (select_page.now) {
        case 0: ICON_Print(); ICON_Prepare(); break;
        case 1: ICON_Prepare(); ICON_Control(); break;
        case 2: ICON_Control(); TERN(HAS_ONESTEP_LEVELING, ICON_Leveling, ICON_StartInfo)(0); break;
        case 3: TERN(HAS_ONESTEP_LEVELING, ICON_Leveling, ICON_StartInfo)(1); break;
      }
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    switch (select_page.now) {
      case 0: // Print File
        checkkey = SelectFile;
		index_file = MROWS;
        Draw_Print_File_Menu();
        break;

      case 1: // Prepare
        checkkey = Prepare;
        select_prepare.reset();
        index_prepare = MROWS;
        Draw_Prepare_Menu();
        break;

      case 2: // Control
        checkkey = Control;
        select_control.reset();
        index_control = MROWS;
        Draw_Control_Menu();
        break;

      case 3: // Leveling or Info
        #if HAS_ONESTEP_LEVELING
          checkkey = Leveling;
          HMI_Leveling();
        #else
          checkkey = Info;
          Draw_Info_Menu();
        #endif
        break;
    }
  }
  DWIN_UpdateLCD();
}

// Select (and Print) File
void HMI_SelectFile() {
  ENCODER_DiffState encoder_diffState = get_encoder_state();

  const uint16_t hasUpDir = !card.flag.workDirIsRoot;

  if (encoder_diffState == ENCODER_DIFF_NO) {
    #if ENABLED(SCROLL_LONG_FILENAMES)
      if (shift_ms && select_file.now >= 1 + hasUpDir) {
        // Scroll selected filename every second
        const millis_t ms = millis();
        if (ELAPSED(ms, shift_ms)) {
          const bool was_reset = shift_amt < 0;
          shift_ms = ms + 375UL + was_reset * 250UL;  // ms per character
          int8_t shift_new = shift_amt + 1;           // Try to shift by...
          Draw_SDItem_Shifted(shift_new);             // Draw the item
          if (!was_reset && shift_new == 0)           // Was it limited to 0?
            shift_ms = 0;                             // No scrolling needed
          else if (shift_new == shift_amt)            // Scroll reached the end
            shift_new = -1;                           // Reset
          shift_amt = shift_new;                      // Set new scroll
        }
      }
    #endif
    return;
  }

  // First pause is long. Easy.
  // On reset, long pause must be after 0.

  const uint16_t fullCnt = nr_sd_menu_items();

  if (encoder_diffState == ENCODER_DIFF_CW && fullCnt) {
    if (select_file.inc(1 + fullCnt)) {
      const uint8_t itemnum = select_file.now - 1;              // -1 for "Back"
      if (TERN0(SCROLL_LONG_FILENAMES, shift_ms)) {             // If line was shifted
        Erase_Menu_Text(itemnum + MROWS - index_file);          // Erase and
        Draw_SDItem(itemnum - 1);                               // redraw
      }
	  
      if (select_file.now > MROWS && select_file.now > index_file) { // Cursor past the bottom
        index_file = select_file.now;                           // New bottom line
        Scroll_Menu(DWIN_SCROLL_UP);
        Draw_SDItem(itemnum, MROWS);                            // Draw and init the shift name
      }
      else {
        Move_Highlight(1, select_file.now + MROWS - index_file); // Just move highlight
        TERN_(SCROLL_LONG_FILENAMES, Init_Shift_Name());         // ...and init the shift name
      }
      TERN_(SCROLL_LONG_FILENAMES, Init_SDItem_Shift());
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW && fullCnt) {
    if (select_file.dec()) {
      const uint8_t itemnum = select_file.now - 1;              // -1 for "Back"
      if (TERN0(SCROLL_LONG_FILENAMES, shift_ms)) {             // If line was shifted
        Erase_Menu_Text(select_file.now + 1 + MROWS - index_file); // Erase and
        Draw_SDItem(itemnum + 1);                               // redraw
      }
	  
      if (select_file.now < index_file - MROWS) {               // Cursor past the top
        index_file--;                                           // New bottom line
        Scroll_Menu(DWIN_SCROLL_DOWN);
        if (index_file == MROWS) {
          Draw_Back_First();
          TERN_(SCROLL_LONG_FILENAMES, shift_ms = 0);
        }
        else {
          Draw_SDItem(itemnum, 0);                              // Draw the item (and init shift name)
        }
      }
      else {
        Move_Highlight(-1, select_file.now + MROWS - index_file); // Just move highlight
        TERN_(SCROLL_LONG_FILENAMES, Init_Shift_Name());        // ...and init the shift name
      }
      TERN_(SCROLL_LONG_FILENAMES, Init_SDItem_Shift());        // Reset left. Init timer.
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    if (select_file.now == 0) { // Back
      select_page.set(0);
      Goto_MainMenu();
    }
    else if (hasUpDir && select_file.now == 1) { // CD-Up
      SDCard_Up();
      goto HMI_SelectFileExit;
    }
    else {
      const uint16_t filenum = select_file.now - 1 - hasUpDir;
      card.getfilename_sorted(SD_ORDER(filenum, card.get_num_Files()));

      // Enter that folder!
      if (card.flag.filenameIsDir) {
        SDCard_Folder(card.filename);
        goto HMI_SelectFileExit;
      }

      // Reset highlight for next entry
      select_print.reset();
      select_file.reset();

      // Start choice and print SD file
      HMI_flag.heat_flag = true;
      HMI_flag.print_finish = false;
      HMI_ValueStruct.show_mode = 0;

      card.openAndPrintFile(card.filename);

      #if FAN_COUNT > 0
        // All fans on for Ender 3 v2 ?
        // The slicer should manage this for us.
        // for (uint8_t i = 0; i < FAN_COUNT; i++)
        //  thermalManager.fan_speed[i] = FANON;
      #endif

      Goto_PrintProcess();
    }
  }
HMI_SelectFileExit:
  DWIN_UpdateLCD();
}

/* Printing */
void HMI_Printing() {
  ENCODER_DiffState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;

  if (HMI_flag.done_confirm_flag) {
    if (encoder_diffState == ENCODER_DIFF_ENTER) {
	  #if HAS_SUICIDE
      	HMI_flag.putdown_close_machine = 0;
	  	HMI_flag.putdown_close_timer_rg = POWERDOWN_MACHINE_TIMER;
	  #endif
      HMI_flag.done_confirm_flag = false;
      dwin_abort_flag = true; // Reset feedrate, return to Home
    }
    return;
  }
  
  if(HMI_flag.filament_runout_star) return;
  
  // Avoid flicker by updating only the previous menu
  if (encoder_diffState == ENCODER_DIFF_CW) {
    if (select_print.inc(3)) {
      switch (select_print.now) {
        case 0: ICON_Tune(); break;
        case 1:
          ICON_Tune();
          if (printingIsPaused()) ICON_Continue(); else ICON_Pause();
          break;
        case 2:
          if (printingIsPaused()) ICON_Continue(); else ICON_Pause();
          ICON_Stop();
          break;
      }
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW) {
    if (select_print.dec()) {
      switch (select_print.now) {
        case 0:
          ICON_Tune();
          if (printingIsPaused()) ICON_Continue(); else ICON_Pause();
          break;
        case 1:
          if (printingIsPaused()) ICON_Continue(); else ICON_Pause();
          ICON_Stop();
          break;
        case 2: ICON_Stop(); break;
      }
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    switch (select_print.now) {
      case 0: // Tune
        checkkey = Tune;
		HMI_flag.Is_Mixer_Print = 1;
        HMI_ValueStruct.show_mode = 0;
        select_tune.reset();
        index_tune = MROWS;
        Draw_Tune_Menu();
        break;
      case 1: // Pause
        if (HMI_flag.pause_flag) {
          ICON_Pause();

          char cmd[40];
          cmd[0] = '\0';

          #if ENABLED(PAUSE_HEAT)
            #if HAS_HEATED_BED
              if (tempbed) sprintf_P(cmd, PSTR("M190 S%i\n"), tempbed);
            #endif
            #if HAS_HOTEND
              if (temphot) sprintf_P(&cmd[strlen(cmd)], PSTR("M109 S%i\n"), temphot);
            #endif
          #endif

          strcat_P(cmd, PSTR("M24"));
          queue.inject(cmd);
        }
        else {
          HMI_flag.select_flag = true;
          checkkey = Print_window;
          Popup_window_PauseOrStop();
        }
        break;

      case 2: // Stop
        HMI_flag.select_flag = true;
        checkkey = Print_window;
        Popup_window_PauseOrStop();
        break;

      default: break;
    }
  }
  DWIN_UpdateLCD();
}

/* Filament Runout Option window */
void HMI_Filament_Runout_Option() {
  ENCODER_DiffState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;

  if (encoder_diffState == ENCODER_DIFF_CW) {
    if (select_option.inc(2)) ICON_YESorNO(select_option.now);
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW) {
    if (select_option.dec()) ICON_YESorNO(select_option.now);
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    switch (select_option.now) {
      case 0: // say yes
        FIL.Puge_More_Yes = 1;
		FIL.Puge_More_No = 0;
        break;
      case 1: // say no
        FIL.Puge_More_Yes = 1;
        FIL.Puge_More_No = 1;
	    checkkey = PrintProcess;
        break;
      default: break;
    }
  }
  DWIN_UpdateLCD();
}

/* Powerdown window */
void HMI_Powerdown() {
  ENCODER_DiffState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;

  if (encoder_diffState == ENCODER_DIFF_CW) {
    if (select_powerdown.inc(2)) ICON_YESorNO_Powerdown(select_powerdown.now);
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW) {
    if (select_powerdown.dec()) ICON_YESorNO_Powerdown(select_powerdown.now);
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    switch (select_powerdown.now) {
      case 0: 
        checkkey = Prepare;
        select_prepare.set(PREPARE_CASE_POWERDOWN);
        index_prepare = MROWS;
        Draw_Prepare_Menu();
        break;
      case 1: 
        queue.inject_P(PSTR("M81"));
        break;
      default: break;
    }
  }
  DWIN_UpdateLCD();
}

/* Pause and Stop window */
void HMI_PauseOrStop() {
  ENCODER_DiffState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;

  if (encoder_diffState == ENCODER_DIFF_CW)
    Draw_Select_Highlight(false);
  else if (encoder_diffState == ENCODER_DIFF_CCW)
    Draw_Select_Highlight(true);
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    if (select_print.now == 1) { // pause window
      if (HMI_flag.select_flag) {
        HMI_flag.pause_action = true;
        ICON_Continue();
        #if ENABLED(POWER_LOSS_RECOVERY)
          if (recovery.enabled){
		  	recovery.save(true);
          }
        #endif
        queue.inject_P(PSTR("M25"));
      }
      else {
        // cancel pause
      }
      Goto_PrintProcess();
    }
    else if (select_print.now == 2) { // stop window
      if (HMI_flag.select_flag) {
        checkkey = Back_Main;
        if (HMI_flag.home_flag) planner.synchronize(); // Wait for planner moves to finish!
        wait_for_heatup = wait_for_user = false;       // Stop waiting for heating/user
        card.flag.abort_sd_printing = true;            // Let the main loop handle SD abort
        dwin_abort_flag = true;                        // Reset feedrate, return to Home
        #ifdef ACTION_ON_CANCEL
          host_action_cancel();
        #endif
        Popup_Window_HomeAll(true);
      }
      else
        Goto_PrintProcess(); // cancel stop
    }
  }
  DWIN_UpdateLCD();
}

inline void Draw_Move_Menu() {
  Clear_Main_Window();

   #if AXISMOVE_CASE_TOTAL >= 6
    const int16_t scroll = MROWS - index_axismove; // Scrolled-up lines
    #define CSCROL(L) (scroll + (L))
  #else
    #define CSCROL(L) (L)
  #endif
  #define CLINE(L)  MBASE(CSCROL(L))
  #define CVISI(L)  WITHIN(CSCROL(L), 0, MROWS)

  //DWIN_ICON_Show(ICON, Title_Coordinate[HMI_flag.language][Title_Move], 14, 7);
  DWIN_JPG_CacheToN(1,HMI_flag.Title_Menu_Bankup);
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Title_Move, Menu_Coordinate,14, 7);
  DWIN_JPG_CacheToN(1,HMI_flag.language+1);

  if (CVISI(0)) Draw_Back_First(select_axis.now == 0);                         // < Back
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Move_Menu_Move, Menu_Coordinate,LBLX, CLINE(AXISMOVE_CASE_MOVEX));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Move_Menu_X, Menu_Coordinate,LBLX+Move_X_Coordinate[HMI_flag.language], CLINE(AXISMOVE_CASE_MOVEX));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Move_Menu_Move, Menu_Coordinate,LBLX, CLINE(AXISMOVE_CASE_MOVEY));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Move_Menu_Y, Menu_Coordinate,LBLX+Move_X_Coordinate[HMI_flag.language], CLINE(AXISMOVE_CASE_MOVEY));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Move_Menu_Move, Menu_Coordinate,LBLX, CLINE(AXISMOVE_CASE_MOVEZ));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Move_Menu_Z, Menu_Coordinate,LBLX+Move_X_Coordinate[HMI_flag.language], CLINE(AXISMOVE_CASE_MOVEZ));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Move_Menu_Extruder, Menu_Coordinate,LBLX, CLINE(AXISMOVE_CASE_EX1));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Move_Menu_1, Menu_Coordinate,LBLX+Extruder_X_Coordinate[HMI_flag.language], CLINE(AXISMOVE_CASE_EX1));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Move_Menu_Extruder, Menu_Coordinate,LBLX, CLINE(AXISMOVE_CASE_EX2));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Move_Menu_2, Menu_Coordinate,LBLX+Extruder_X_Coordinate[HMI_flag.language], CLINE(AXISMOVE_CASE_EX2));
  
  if (select_axis.now && CVISI(select_axis.now))
    Draw_Menu_Cursor(CSCROL(select_axis.now));

  Draw_Menu_Line(1,ICON_MoveX);
  Draw_Menu_Line(2,ICON_MoveY);
  Draw_Menu_Line(3,ICON_MoveZ);
  Draw_Menu_Line(4,ICON_Extruder1);
  Draw_Menu_Line(5,ICON_Extruder2);
}


/*
inline void Draw_Move_Menu() {
  Clear_Main_Window();

  if (HMI_IsChinese()) {
    DWIN_Frame_TitleCopy(1, 192, 1, 233, 14); // "Move"
    DWIN_Frame_AreaCopy(1, 58, 118, 106, 132, LBLX, MBASE(1));
    DWIN_Frame_AreaCopy(1, 109, 118, 157, 132, LBLX, MBASE(2));
    DWIN_Frame_AreaCopy(1, 160, 118, 209, 132, LBLX, MBASE(3));
    #if HAS_HOTEND
      DWIN_Frame_AreaCopy(1, 212, 118, 253, 131, LBLX, MBASE(4));
    #endif
  }
  else {
    #ifdef USE_STRING_HEADINGS
      Draw_Title(GET_TEXT_F(MSG_MOVE_AXIS));
    #else
      DWIN_Frame_TitleCopy(1, 231, 2, 265, 12);                     // "Move"
    #endif
    draw_move_en(MBASE(1)); say_x(36, MBASE(1));                    // "Move X"
    draw_move_en(MBASE(2)); say_y(36, MBASE(2));                    // "Move Y"
    draw_move_en(MBASE(3)); say_z(36, MBASE(3));                    // "Move Z"
    #if HAS_HOTEND
      DWIN_Frame_AreaCopy(1, 123, 192, 176, 202, LBLX, MBASE(4));   // "Extruder"
    #endif
  }

  Draw_Back_First(select_axis.now == 0);
  if (select_axis.now) Draw_Menu_Cursor(select_axis.now);

  // Draw separators and icons
  LOOP_L_N(i, 3 + ENABLED(HAS_HOTEND)) Draw_Menu_Line(i + 1, ICON_MoveX + i);
}
*/
#include "../../../libs/buzzer.h"
void HMI_AudioFeedback(const bool success=true) {
  if (success) {
    buzzer.tone(100, 659);
    buzzer.tone(10, 0);
    buzzer.tone(100, 698);
  }
  else
    buzzer.tone(40, 440);
}

/* Prepare */
void HMI_Prepare() {
  ENCODER_DiffState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;

  // Avoid flicker by updating only the previous menu
  if (encoder_diffState == ENCODER_DIFF_CW) {
    if (select_prepare.inc(1 + PREPARE_CASE_TOTAL)) {
      if (select_prepare.now > MROWS && select_prepare.now > index_prepare) {
        index_prepare = select_prepare.now;

        // Scroll up and draw a blank bottom line
        Scroll_Menu(DWIN_SCROLL_UP);
        Draw_Menu_Icon(MROWS, ICON_Axis + select_prepare.now - 1);

        // Draw "More" icon for sub-menus
        if (index_prepare < 7) Draw_More_Icon(MROWS - index_prepare + 1);
		
        #if HAS_ZOFFSET_ITEM
		  if (index_prepare == PREPARE_CASE_ZOFF) Item_Prepare_Offset(MROWS);
		#endif
        #if HAS_HOTEND
		  if (index_prepare == PREPARE_CASE_PLA) Item_Prepare_PLA(MROWS);
          if (index_prepare == PREPARE_CASE_ABS) Item_Prepare_ABS(MROWS);
        #endif
        #if HAS_PREHEAT
          if (index_prepare == PREPARE_CASE_COOL) Item_Prepare_Cool(MROWS);
        #endif
        if (index_prepare == PREPARE_CASE_LANG) Item_Prepare_Lang(MROWS);
      }
      else {
        Move_Highlight(1, select_prepare.now + MROWS - index_prepare);
      }
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW) {
    if (select_prepare.dec()) {
      if (select_prepare.now < index_prepare - MROWS) {
        index_prepare--;
        Scroll_Menu(DWIN_SCROLL_DOWN);

        if (index_prepare == MROWS)
          Draw_Back_First();
        else
          Draw_Menu_Line(0, ICON_Axis + select_prepare.now - 1);

        if (index_prepare < 7) Draw_More_Icon(MROWS - index_prepare + 1);

             if (index_prepare == 6) Item_Prepare_Move(0);
        else if (index_prepare == 7) Item_Prepare_Disable(0);
        else if (index_prepare == 8) Item_Prepare_Home(0);
		else if (index_prepare == 9) Item_Prepare_Leveling(0);
		else if (index_prepare == 10) Item_Prepare_Offset(0);
      }
      else {
        Move_Highlight(-1, select_prepare.now + MROWS - index_prepare);
      }
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    switch (select_prepare.now) {
      case 0: // Back
        select_page.set(1);
        Goto_MainMenu();
        break;
      case PREPARE_CASE_MOVE: // Axis move
        checkkey = AxisMove;
        select_axis.reset();
	    index_axismove = MROWS;
        Draw_Move_Menu();
		
        //DWIN_Draw_FloatValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 1, 216, MBASE(AXISMOVE_CASE_MOVEX), current_position.x * MINUNITMULT);
        //DWIN_Draw_FloatValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 1, 216, MBASE(AXISMOVE_CASE_MOVEY), current_position.y * MINUNITMULT);
        //DWIN_Draw_FloatValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 1, 216, MBASE(AXISMOVE_CASE_MOVEZ), current_position.z * MINUNITMULT);
		DWIN_Draw_Signed_Float(font8x16, Color_Bg_Black, 3, 1, 216, MBASE(AXISMOVE_CASE_MOVEX), current_position.x * MINUNITMULT);
		DWIN_Draw_Signed_Float(font8x16, Color_Bg_Black, 3, 1, 216, MBASE(AXISMOVE_CASE_MOVEY), current_position.y * MINUNITMULT);
		DWIN_Draw_Signed_Float(font8x16, Color_Bg_Black, 3, 1, 216, MBASE(AXISMOVE_CASE_MOVEZ), current_position.z * MINUNITMULT);
		#if HAS_HOTEND
          queue.inject_P(PSTR("G92 E0"));
          current_position.e = HMI_ValueStruct.Move_E1_scale = 0;
		  current_position.e = HMI_ValueStruct.Move_E2_scale = 0;
		  //current_position.e = HMI_ValueStruct.Move_E3_scale = 0;
		  //current_position.e = HMI_ValueStruct.Move_E4_scale = 0;
          DWIN_Draw_Signed_Float(font8x16, Color_Bg_Black, 3, 1, 216, MBASE(AXISMOVE_CASE_EX1), 0);
		  DWIN_Draw_Signed_Float(font8x16, Color_Bg_Black, 3, 1, 216, MBASE(AXISMOVE_CASE_EX2), 0);
        #endif
		
        break;
      case PREPARE_CASE_DISA: // Disable steppers
        queue.inject_P(PSTR("M84"));
        break;
	  
      case PREPARE_CASE_HOME: // Homing
      	checkkey = Home;
        index_home = MROWS;
	  	select_home.reset();
        Draw_Home_Menu();
      /*
        checkkey = Last_Prepare;
        index_prepare = MROWS;
        queue.inject_P(PSTR("G28")); 	// G28 will set home_flag
        Popup_Window_HomeAll();
        */
        break;
		
	  case PREPARE_CASE_LEVELING: 		// Leveling
        checkkey = Leveling0;
        index_leveling = MROWS;
	  	select_leveling.reset();
		HMI_flag.Leveling_first_time = 1;
        Draw_Leveling_Menu();
        break;

	  case PREPARE_CASE_POWERDOWN: 		// Powerdown
        //queue.inject_P(PSTR("M81"));
        checkkey = Powerdown;
        Popup_window_Powerdown();
        break;
		
      #if HAS_ZOFFSET_ITEM
        case PREPARE_CASE_ZOFF: // Z-offset
          #if EITHER(HAS_BED_PROBE, BABYSTEPPING)
            checkkey = Homeoffset;
            HMI_ValueStruct.show_mode = -4;
            HMI_ValueStruct.offset_value = BABY_Z_VAR * 100;
            DWIN_Draw_Signed_Float(font8x16, Select_Color, 2, 2, 202, MBASE(PREPARE_CASE_ZOFF + MROWS - index_prepare), HMI_ValueStruct.offset_value);
            EncoderRate.enabled = true;
          #else
            // Apply workspace offset, making the current position 0,0,0
            queue.inject_P(PSTR("G92 X0 Y0 Z0"));
            HMI_AudioFeedback();
          #endif
          break;
      #endif
	  
      #if HAS_HOTEND
        case PREPARE_CASE_PLA: // PLA preheat
          thermalManager.setTargetHotend(ui.material_preset[0].hotend_temp, 0);
          thermalManager.setTargetBed(ui.material_preset[0].bed_temp);
          thermalManager.set_fan_speed(0, ui.material_preset[0].fan_speed);
          break;
        case PREPARE_CASE_ABS: // ABS preheat
          thermalManager.setTargetHotend(ui.material_preset[1].hotend_temp, 0);
          thermalManager.setTargetBed(ui.material_preset[1].bed_temp);
          thermalManager.set_fan_speed(0, ui.material_preset[1].fan_speed);
          break;
      #endif
      #if HAS_PREHEAT
        case PREPARE_CASE_COOL: // Cool
          TERN_(HAS_FAN, thermalManager.zero_fan_speeds());
          #if HAS_HOTEND || HAS_HEATED_BED
            thermalManager.disable_all_heaters();
          #endif
          break;
      #endif
      case PREPARE_CASE_LANG: // Toggle Language
        //HMI_ToggleLanguage();
        //Draw_Prepare_Menu();
        checkkey = Language;
        index_language = MROWS;
	  	select_language.reset();
        Draw_Language_Menu();
        break;
      default: break;
    }
  }
  DWIN_UpdateLCD();
}

void Draw_Temperature_Menu() {
  Clear_Main_Window();

  //DWIN_ICON_Show(ICON, Title_Coordinate[HMI_flag.language][Title_Temperature], 14, 7);
  DWIN_JPG_CacheToN(1,HMI_flag.Title_Menu_Bankup);
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Title_Temperature, Menu_Coordinate,14, 7);
  DWIN_JPG_CacheToN(1,HMI_flag.language+1);
  #if HAS_HOTEND
  	DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Temp_Menu_Hotend, Menu_Coordinate,LBLX, MBASE(TEMP_CASE_TEMP));
  	DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Temp_Menu_Temp, Menu_Coordinate,LBLX+Hotend_X_Coordinate[HMI_flag.language], MBASE(TEMP_CASE_TEMP));
  #endif
  #if HAS_HEATED_BED
  	DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Temp_Menu_Bed, Menu_Coordinate,LBLX, MBASE(TEMP_CASE_BED));
  	DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Temp_Menu_Temp, Menu_Coordinate,LBLX+Bed_X_Coordinate[HMI_flag.language], MBASE(TEMP_CASE_BED));
  #endif
  #if HAS_FAN
  	DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Temp_Menu_Fan_Speed, Menu_Coordinate,LBLX, MBASE(TEMP_CASE_FAN));
  #endif
  #if HAS_HOTEND
  	DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Temp_Menu_PLA, Menu_Coordinate,LBLX, MBASE(TEMP_CASE_PLA));
  	DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Temp_Menu_Preheat, Menu_Coordinate,LBLX+36, MBASE(TEMP_CASE_PLA));
    DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Temp_Menu_ABS, Menu_Coordinate,LBLX, MBASE(TEMP_CASE_ABS));
  	DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Temp_Menu_Preheat, Menu_Coordinate,LBLX+36, MBASE(TEMP_CASE_ABS));
  #endif
  
  Draw_Back_First(select_temp.now == 0);
  if (select_temp.now) Draw_Menu_Cursor(select_temp.now);

  // Draw icons and lines
  uint8_t i = 0;
  #define _TMENU_ICON(N) Draw_Menu_Line(++i, ICON_SetEndTemp + (N) - 1)
  #if HAS_HOTEND
    _TMENU_ICON(TEMP_CASE_TEMP);
    DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(i), thermalManager.temp_hotend[0].target);
  #endif
  #if HAS_HEATED_BED
    _TMENU_ICON(TEMP_CASE_BED);
    DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(i), thermalManager.temp_bed.target);
  #endif
  #if HAS_FAN
    _TMENU_ICON(TEMP_CASE_FAN);
    DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(i), thermalManager.fan_speed[0]);
  #endif
  #if HAS_HOTEND
    // PLA/ABS items have submenus
    _TMENU_ICON(TEMP_CASE_PLA);
    Draw_More_Icon(i);
    _TMENU_ICON(TEMP_CASE_ABS);
    Draw_More_Icon(i);
  #endif
}

/* Control */
void HMI_Control() {
  ENCODER_DiffState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;

  // Avoid flicker by updating only the previous menu
  if (encoder_diffState == ENCODER_DIFF_CW) {
    if (select_control.inc(1 + CONTROL_CASE_TOTAL)) {
      if (select_control.now > MROWS && select_control.now > index_control) {
        index_control = select_control.now;

		// Scroll up and draw a blank bottom line
        Scroll_Menu(DWIN_SCROLL_UP);
        Draw_Menu_Icon(MROWS, ICON_Temperature + select_control.now - 1);
		
        #if ENABLED(BLTOUCH)
			if(index_control == CONTROL_CASE_LOAD)	Item_Control_Load(MROWS);
        	else if(index_control == CONTROL_CASE_RESET)	Item_Control_Reset(MROWS);
			else if (index_control == CONTROL_CASE_INFO) Item_Control_Info(MROWS);
		#else
			if(index_control == CONTROL_CASE_RESET)	Item_Control_Reset(MROWS);
			else if (index_control == CONTROL_CASE_INFO) Item_Control_Info(MROWS);
		#endif
      }
      else {
        Move_Highlight(1, select_control.now + MROWS - index_control);
      }
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW) {
    if (select_control.dec()) {
      if (select_control.now < index_control - MROWS) {
        index_control--;
        Scroll_Menu(DWIN_SCROLL_DOWN);

        if (index_control == MROWS)
          Draw_Back_First();
        else
          Draw_Menu_Line(0, ICON_Temperature + select_control.now - 1);
		
        #if ENABLED(BLTOUCH)
			if(index_control == CONTROL_CASE_LOAD) Item_Control_Temp(0);
        	else if (index_control == CONTROL_CASE_RESET) Item_Control_Motion(0);
			else if(index_control == CONTROL_CASE_INFO) Item_Control_Mixer(0);
		#else
		    if(index_control == CONTROL_CASE_RESET) Item_Control_Temp(0);
        	else if (index_control == CONTROL_CASE_INFO) Item_Control_Motion(0);
		#endif
      }
      else {
        Move_Highlight(-1, select_control.now + MROWS - index_control);
      }
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    switch (select_control.now) {
      case 0: // Back
        select_page.set(2);
        Goto_MainMenu();
        break;
      case CONTROL_CASE_TEMP: // Temperature
        checkkey = TemperatureID;
        HMI_ValueStruct.show_mode = -1;
        select_temp.reset();
        Draw_Temperature_Menu();
        break;
      case CONTROL_CASE_MOVE: // Motion
        checkkey = Motion;
        select_motion.reset();
        Draw_Motion_Menu();
        break;
	  case CONTROL_CASE_MIXER: // Mixer
        checkkey = DMixer;
		HMI_flag.Is_Mixer_Print = 0;
        select_mixer.reset();
        Draw_Mixer_Menu();
        break;
		
	  #if ENABLED(BLTOUCH)
	  	case CONTROL_CASE_BLTOUCH: // Bltouch
        	checkkey = Bltouch;
        	select_bltouch.reset();
        	Draw_Bltouch_Menu();
        	break;
	  #endif
	  
      #if ENABLED(EEPROM_SETTINGS)
        case CONTROL_CASE_SAVE: { // Write EEPROM
          const bool success = settings.save();
          HMI_AudioFeedback(success);
        } break;
        case CONTROL_CASE_LOAD: { // Read EEPROM
          const bool success = settings.load();
          HMI_AudioFeedback(success);
        } break;
        case CONTROL_CASE_RESET: // Reset EEPROM
          settings.reset();
		  //Draw_Print_ProgressModel();
          HMI_AudioFeedback();
		  checkkey = Control;
          select_control.reset();
          index_control = MROWS;
		  Draw_Control_Menu();
          break;
      #endif
      case CONTROL_CASE_INFO: // Info
        checkkey = Info;
        Draw_Info_Menu();
        break;
      default: break;
    }
  }
  DWIN_UpdateLCD();
}

/* Language */
void HMI_Language() {
  ENCODER_DiffState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;
  
  if (encoder_diffState == ENCODER_DIFF_CW) {
    if (select_language.inc(1 + LANGUAGE_CASE_TOTAL)) {
      if (select_language.now > MROWS && select_language.now > index_language) {
        index_language = select_language.now;

		// Scroll up and draw a blank bottom line
        Scroll_Menu(DWIN_SCROLL_UP);
        Draw_Menu_Icon(MROWS, LANGUAGE_CASE_EN + select_language.now - 1);
		
		if(index_language == LANGUAGE_CASE_ZH) Item_Language_ZH(MROWS);
      }
      else {
        Move_Highlight(1, select_language.now + MROWS - index_language);
      }
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW) {
    if (select_language.dec()) {
      if (select_language.now < index_language - MROWS) {
        index_language--;
        Scroll_Menu(DWIN_SCROLL_DOWN);

        if (index_language == MROWS)
          Draw_Back_First();
        else
          Draw_Menu_Line(0, LANGUAGE_CASE_EN + select_language.now - 1);

		  if(index_language == LANGUAGE_CASE_ZH) Item_Language_EN(0);
      }
      else {
        Move_Highlight(-1, select_language.now + MROWS - index_language);
      }
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    switch (select_language.now) {
	  case 0: 										// Back
        checkkey = Prepare;
        select_prepare.set(PREPARE_CASE_LANG);
        Draw_Prepare_Menu();
        break;

	  case LANGUAGE_CASE_EN: 
	  case LANGUAGE_CASE_SP:
	  case LANGUAGE_CASE_RU:
	  	HMI_flag.Title_Menu_Bankup = 7;
        HMI_flag.language = select_language.now - 1;
		DWIN_JPG_CacheToN(1,HMI_flag.language+1);
		HMI_AudioFeedback(settings.save());
		checkkey = Prepare;
        select_prepare.set(PREPARE_CASE_LANG);
        Draw_Prepare_Menu();
        break;									
	  case LANGUAGE_CASE_FR: 
	  case LANGUAGE_CASE_PO:
	  	HMI_flag.Title_Menu_Bankup = 6;
        HMI_flag.language = select_language.now - 1;
		DWIN_JPG_CacheToN(1,HMI_flag.language+1);
		HMI_AudioFeedback(settings.save());
		checkkey = Prepare;
        select_prepare.set(PREPARE_CASE_LANG);
        Draw_Prepare_Menu();
		break; 									
	  case LANGUAGE_CASE_ZH: 
		break;
	  
	  default:break;
    } 
  }
  DWIN_UpdateLCD();
}



#if HAS_ONESTEP_LEVELING
  /* Leveling */
  void HMI_Leveling() {
    Popup_Window_Leveling();
    DWIN_UpdateLCD();
	queue.inject_P(PSTR("G28O\nG29"));
  }

#endif

/* Leveling0 */
char Level_Buf[200]={0};
void HMI_Leveling0() {
  ENCODER_DiffState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;
  
  if (encoder_diffState == ENCODER_DIFF_CW) {
    if (select_leveling.inc(1 + LEVELING_CASE_TOTAL)) {
      if (select_leveling.now > MROWS && select_leveling.now > index_leveling) {
        index_leveling = select_leveling.now;

		// Scroll up and draw a blank bottom line
        Scroll_Menu(DWIN_SCROLL_UP);
        Draw_Menu_Icon(MROWS, ICON_Leveling_Point1 + select_leveling.now - 1);
		
		if(index_leveling == LEVELING_CASE_SAVE) Item_Leveling_Save(MROWS);
      }
      else {
        Move_Highlight(1, select_leveling.now + MROWS - index_leveling);
      }
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW) {
    if (select_leveling.dec()) {
      if (select_leveling.now < index_leveling - MROWS) {
        index_leveling--;
        Scroll_Menu(DWIN_SCROLL_DOWN);

        if (index_leveling == MROWS)
          Draw_Back_First();
        else
          Draw_Menu_Line(0, ICON_Leveling_Point1 + select_leveling.now - 1);

		  if(index_leveling == LEVELING_CASE_SAVE) Item_Leveling_Point1(0);
      }
      else {
        Move_Highlight(-1, select_leveling.now + MROWS - index_leveling);
      }
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    switch (select_leveling.now) {
	  case 0: 										// Back
        checkkey = Prepare;
        select_prepare.set(PREPARE_CASE_LEVELING);
        Draw_Prepare_Menu();
        break;

	  case LEVELING_CASE_POINT1: 										
        checkkey = Leveling0;
        for(uint8_t i=LEVELING_CASE_POINT1; i<LEVELING_CASE_TOTAL+1; i++) DWIN_ICON_Show(ICON, ICON_Blank, 216, MBASE(i));
		DWIN_ICON_Show(ICON, ICON_Cursor, 216, MBASE(LEVELING_CASE_POINT1));
		if(HMI_flag.Leveling_first_time){
			HMI_flag.Leveling_first_time = 0;	
			ZERO(Level_Buf);
			sprintf_P(Level_Buf,PSTR("G28\nG91\nG1 Z%d F%d\nG90\nG1 X%d Y%d F%d\nG1 Z%d F%d"),10,1500,20,20,3000,0,500);
			queue.inject_P(Level_Buf);
		}
		else{
			ZERO(Level_Buf);
			sprintf_P(Level_Buf,PSTR("G91\nG1 Z%d F%d\nG90\nG1 X%d Y%d F%d\nG1 Z%d F%d"),10,1500,20,20,3000,0,500);
			queue.inject_P(Level_Buf);
		}
        break;
	  case LEVELING_CASE_POINT2: 										
        checkkey = Leveling0;
        for(uint8_t i=LEVELING_CASE_POINT1; i<LEVELING_CASE_TOTAL+1; i++) DWIN_ICON_Show(ICON, ICON_Blank, 216, MBASE(i));
		DWIN_ICON_Show(ICON, ICON_Cursor, 216, MBASE(LEVELING_CASE_POINT2));
		if(HMI_flag.Leveling_first_time){
			HMI_flag.Leveling_first_time = 0;
			ZERO(Level_Buf);
			sprintf_P(Level_Buf,PSTR("G28\nG91\nG1 Z%d F%d\nG90\nG1 X%d Y%d F%d\nG1 Z%d F%d"),10,1500,X_BED_SIZE-20,20,3000,0,500);
			queue.inject_P(Level_Buf);
		}
		else{
			ZERO(Level_Buf);
			sprintf_P(Level_Buf,PSTR("G91\nG1 Z%d F%d\nG90\nG1 X%d Y%d F%d\nG1 Z%d F%d"),10,1500,X_BED_SIZE-20,20,3000,0,500);
			queue.inject_P(Level_Buf);
		}
        break;
	  case LEVELING_CASE_POINT3: 										
        checkkey = Leveling0;
        for(uint8_t i=LEVELING_CASE_POINT1; i<LEVELING_CASE_TOTAL+1; i++) DWIN_ICON_Show(ICON, ICON_Blank, 216, MBASE(i));
		DWIN_ICON_Show(ICON, ICON_Cursor, 216, MBASE(LEVELING_CASE_POINT3));
		if(HMI_flag.Leveling_first_time){
			HMI_flag.Leveling_first_time = 0;
			ZERO(Level_Buf);
			sprintf_P(Level_Buf,PSTR("G28\nG91\nG1 Z%d F%d\nG90\nG1 X%d Y%d F%d\nG1 Z%d F%d"),10,1500,X_BED_SIZE-20,Y_BED_SIZE-20,3000,0,500);
			queue.inject_P(Level_Buf);
		}
		else{
			ZERO(Level_Buf);
			sprintf_P(Level_Buf,PSTR("G91\nG1 Z%d F%d\nG90\nG1 X%d Y%d F%d\nG1 Z%d F%d"),10,1500,X_BED_SIZE-20,Y_BED_SIZE-20,3000,0,500);
			queue.inject_P(Level_Buf);
		}
        break;
	  case LEVELING_CASE_POINT4: 										
        checkkey = Leveling0;
		for(uint8_t i=LEVELING_CASE_POINT1; i<LEVELING_CASE_TOTAL+1; i++) DWIN_ICON_Show(ICON, ICON_Blank, 216, MBASE(i));
		DWIN_ICON_Show(ICON, ICON_Cursor, 216, MBASE(LEVELING_CASE_POINT4));

		if(HMI_flag.Leveling_first_time){
			HMI_flag.Leveling_first_time = 0;
			ZERO(Level_Buf);
			sprintf_P(Level_Buf,PSTR("G28\nG91\nG1 Z%d F%d\nG90\nG1 X%d Y%d F%d\nG1 Z%d F%d"),10,1500,20,Y_BED_SIZE-20,3000,0,500);
			queue.inject_P(Level_Buf);
		}
		else{
			ZERO(Level_Buf);
			sprintf_P(Level_Buf,PSTR("G91\nG1 Z%d F%d\nG90\nG1 X%d Y%d F%d\nG1 Z%d F%d"),10,1500,20,Y_BED_SIZE-20,3000,0,500);
			queue.inject_P(Level_Buf);
		}
        break;
		
      	#if ENABLED(LCD_BED_LEVELING)
	  	case LEVELING_CASE_AUTO: 										
       	 checkkey = Leveling0;
			//Popup_Window_Leveling0();
    		//DWIN_UpdateLCD();
			queue.inject_P(PSTR("G28\nG29 N"));
			break;

	  	case LEVELING_CASE_SAVE: 										
        	checkkey = Last_Leveling;
			Popup_Window_Leveling0();
			queue.inject_P(PSTR("G28\nG29"));
			break;
	  	#endif
		
	  default:break;
    } 
  }
  DWIN_UpdateLCD();
}

/* Home */
void HMI_Home() {
  ENCODER_DiffState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;

  // Avoid flicker by updating only the previous menu
  if (encoder_diffState == ENCODER_DIFF_CW) {
    if (select_home.inc(1 + HOME_CASE_TOTAL)) {
		Move_Highlight(1, select_home.now);

	    for(uint8_t i=HOME_CASE_ALL; i<HOME_CASE_TOTAL+1; i++) DWIN_ICON_Show(ICON, ICON_Blank, 216, MBASE(i));
		DWIN_ICON_Show(ICON, ICON_Cursor, 216, MBASE(select_home.now));
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW) {
    if (select_home.dec()) {
		Move_Highlight(-1, select_home.now);
		for(uint8_t i=HOME_CASE_ALL; i<HOME_CASE_TOTAL+1; i++) DWIN_ICON_Show(ICON, ICON_Blank, 216, MBASE(i));
		if(select_home.now) DWIN_ICON_Show(ICON, ICON_Cursor, 216, MBASE(select_home.now));
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    switch (select_home.now) {
	  case 0: 										// Back
        checkkey = Prepare;
        select_leveling.set(PREPARE_CASE_HOME);
        Draw_Prepare_Menu();
        break;
	  case HOME_CASE_ALL: 										
        checkkey = Last_Prepare;
        index_home = MROWS;
        queue.inject_P(PSTR("G28")); 	// G28 will set home_flag
        Popup_Window_HomeAll();
        break;
	  case HOME_CASE_X: 										
        checkkey = Last_Prepare;
        index_home = MROWS;
        queue.inject_P(PSTR("G28 X0")); 	// G28 will set home_flag
        Popup_Window_HomeX();
        break;
	  case HOME_CASE_Y: 										
        checkkey = Last_Prepare;
        index_home = MROWS;
        queue.inject_P(PSTR("G28 Y0")); 	// G28 will set home_flag
        Popup_Window_HomeY();
        break;
	  case HOME_CASE_Z: 										
        checkkey = Last_Prepare;
        index_home = MROWS;
        queue.inject_P(PSTR("G28 Z0")); 	// G28 will set home_flag
        Popup_Window_HomeZ();
        break;
	  default:break;
    } 
  }
  DWIN_UpdateLCD();
}

/* Axis Move */
void HMI_AxisMove() {
  ENCODER_DiffState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;
  
  #if ENABLED(PREVENT_COLD_EXTRUSION)
    // popup window resume
    if (HMI_flag.ETempTooLow_flag) {
      if (encoder_diffState == ENCODER_DIFF_ENTER) {
        HMI_flag.ETempTooLow_flag = false;
        current_position.e = HMI_ValueStruct.Move_E1_scale = HMI_ValueStruct.Move_E2_scale = HMI_ValueStruct.Move_E3_scale = HMI_ValueStruct.Move_E4_scale = 0;
		checkkey = AxisMove;
        select_axis.reset();
	    index_axismove = MROWS;
		Draw_Move_Menu();
        //DWIN_Draw_FloatValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 1, 216, MBASE(1), HMI_ValueStruct.Move_X_scale);
        //DWIN_Draw_FloatValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 1, 216, MBASE(2), HMI_ValueStruct.Move_Y_scale);
        //DWIN_Draw_FloatValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 1, 216, MBASE(3), HMI_ValueStruct.Move_Z_scale);
        //DWIN_Draw_Signed_Float(font8x16, Color_Bg_Black, 3, 1, 216, MBASE(4), 0);
        HMI_ValueStruct.Move_X_scale = current_position.x * MINUNITMULT;
		DWIN_Draw_Signed_Float(font8x16, Color_Bg_Black, 3, 1, 216, MBASE(1), HMI_ValueStruct.Move_X_scale);
		HMI_ValueStruct.Move_Y_scale = current_position.y * MINUNITMULT;
		DWIN_Draw_Signed_Float(font8x16, Color_Bg_Black, 3, 1, 216, MBASE(2), HMI_ValueStruct.Move_Y_scale);
		HMI_ValueStruct.Move_Z_scale = current_position.z * MINUNITMULT;
		DWIN_Draw_Signed_Float(font8x16, Color_Bg_Black, 3, 1, 216, MBASE(3), HMI_ValueStruct.Move_Z_scale);
		DWIN_Draw_Signed_Float(font8x16, Color_Bg_Black, 3, 1, 216, MBASE(4), HMI_ValueStruct.Move_E1_scale);
		DWIN_Draw_Signed_Float(font8x16, Color_Bg_Black, 3, 1, 216, MBASE(5), HMI_ValueStruct.Move_E1_scale);
        DWIN_UpdateLCD();
      }
      return;
    }
  #endif

  // Avoid flicker by updating only the previous menu
  if (encoder_diffState == ENCODER_DIFF_CW) {
  	if (select_axis.inc(1 + AXISMOVE_CASE_TOTAL)) {
      if (select_axis.now > MROWS && select_axis.now > index_axismove) {
        index_axismove = select_axis.now;

		// Scroll up and draw a blank bottom line
        Scroll_Menu(DWIN_SCROLL_UP);
        Draw_Menu_Icon(MROWS, ICON_MoveX + select_axis.now - 1);

		#if ENABLED(MIXING_EXTRUDER)
	  		#if(MIXING_STEPPERS == 4)
				if(index_axismove == AXISMOVE_CASE_EX3)	Item_Axis_MoveEX3(MROWS);
				else if (index_axismove == AXISMOVE_CASE_EX4) Item_Axis_MoveEX4(MROWS);
			#elif(MIXING_STEPPERS == 3)
				if(index_axismove == AXISMOVE_CASE_EX3)	Item_Axis_MoveEX3(MROWS);
			#endif
		#endif
      }
      else {
        Move_Highlight(1, select_axis.now + MROWS - index_axismove);
      }
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW) {
  	if (select_axis.dec()) {
      if (select_axis.now < index_axismove - MROWS) {
        index_axismove--;
        Scroll_Menu(DWIN_SCROLL_DOWN);

		if (index_axismove == MROWS)
          Draw_Back_First();
        else
          Draw_Menu_Line(0, ICON_MoveX + select_axis.now - 1);

		#if ENABLED(MIXING_EXTRUDER)
	  		#if(MIXING_STEPPERS == 4)
				if(index_axismove == AXISMOVE_CASE_EX3) Item_Axis_MoveX(0);
        		else if (index_axismove == AXISMOVE_CASE_EX4) Item_Axis_MoveY(0);
			#elif(MIXING_STEPPERS == 3)
				if(index_axismove == AXISMOVE_CASE_EX3) Item_Axis_MoveX(0);
			#endif
		#endif
      }
      else {
        Move_Highlight(-1, select_axis.now + MROWS - index_axismove);
      }
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    switch (select_axis.now) {
      case 0: // Back
        checkkey = Prepare;
        select_prepare.set(1);
        index_prepare = MROWS;
        Draw_Prepare_Menu();
        break;
      case AXISMOVE_CASE_MOVEX: // X axis move
        checkkey = Move_X;
        HMI_ValueStruct.Move_X_scale = current_position.x * MINUNITMULT;
	    //DWIN_Draw_FloatValue(true, true, 0, font8x16, Color_White, Select_Color, 3, 1, 216, MBASE(MROWS -index_axismove + AXISMOVE_CASE_MOVEX), HMI_ValueStruct.Move_X_scale);
		DWIN_Draw_Signed_Float(font8x16, Select_Color, 3, 1, 216, MBASE(MROWS -index_axismove + AXISMOVE_CASE_MOVEX), HMI_ValueStruct.Move_X_scale);
	    EncoderRate.enabled = true;
        break;
      case AXISMOVE_CASE_MOVEY: // Y axis move
        checkkey = Move_Y;
        HMI_ValueStruct.Move_Y_scale = current_position.y * MINUNITMULT;
        //DWIN_Draw_FloatValue(true, true, 0, font8x16, Color_White, Select_Color, 3, 1, 216, MBASE(MROWS -index_axismove + AXISMOVE_CASE_MOVEY), HMI_ValueStruct.Move_Y_scale);
		DWIN_Draw_Signed_Float(font8x16, Select_Color, 3, 1, 216, MBASE(MROWS -index_axismove + AXISMOVE_CASE_MOVEY), HMI_ValueStruct.Move_Y_scale);
	    EncoderRate.enabled = true;
        break;
      case AXISMOVE_CASE_MOVEZ: // Z axis move
        checkkey = Move_Z;
        HMI_ValueStruct.Move_Z_scale = current_position.z * MINUNITMULT;
        //DWIN_Draw_FloatValue(true, true, 0, font8x16, Color_White, Select_Color, 3, 1, 216, MBASE(MROWS -index_axismove + AXISMOVE_CASE_MOVEZ), HMI_ValueStruct.Move_Z_scale);
		DWIN_Draw_Signed_Float(font8x16, Select_Color, 3, 1, 216, MBASE(MROWS -index_axismove + AXISMOVE_CASE_MOVEZ), HMI_ValueStruct.Move_Z_scale);
	    EncoderRate.enabled = true;
        break;
		
        #if HAS_HOTEND
         case AXISMOVE_CASE_EX1: // Extruder1
            // window tips
            #ifdef PREVENT_COLD_EXTRUSION
              if (thermalManager.temp_hotend[0].celsius < EXTRUDE_MINTEMP) {
                HMI_flag.ETempTooLow_flag = true;
                Popup_Window_ETempTooLow();
                DWIN_UpdateLCD();
                return;
              }
            #endif
            checkkey = Extruder1;
            //HMI_ValueStruct.Move_E1_scale = current_position.e * MINUNITMULT;
			//current_position.e = HMI_ValueStruct.Move_E1_scale = 0;
			HMI_flag.last_E_Coordinate = current_position.e = HMI_ValueStruct.Move_E1_scale/10;
			DWIN_Draw_Signed_Float(font8x16, Select_Color, 3, 1, 216, MBASE(MROWS -index_axismove + AXISMOVE_CASE_EX1), HMI_ValueStruct.Move_E1_scale);
			EncoderRate.enabled = true;
            break;

         #if ENABLED(MIXING_EXTRUDER)
	  		#if(MIXING_STEPPERS == 4)
				case AXISMOVE_CASE_EX2: // Extruder2
				// window tips
            	#ifdef PREVENT_COLD_EXTRUSION
				if (thermalManager.temp_hotend[0].celsius < EXTRUDE_MINTEMP) {
					HMI_flag.ETempTooLow_flag = true;
					Popup_Window_ETempTooLow();
					DWIN_UpdateLCD();
					return;
				}
            	#endif
					checkkey = Extruder2;
					//HMI_ValueStruct.Move_E2_scale = current_position.e * MINUNITMULT;
					//current_position.e = HMI_ValueStruct.Move_E1_scale = 0;
					HMI_flag.last_E_Coordinate = current_position.e = HMI_ValueStruct.Move_E2_scale/10;
					DWIN_Draw_Signed_Float(font8x16, Select_Color, 3, 1, 216, MBASE(MROWS -index_axismove + AXISMOVE_CASE_EX2), HMI_ValueStruct.Move_E2_scale);
					EncoderRate.enabled = true;
				break;
				
				case AXISMOVE_CASE_EX3: // Extruder3
				// window tips
            	#ifdef PREVENT_COLD_EXTRUSION
				if (thermalManager.temp_hotend[0].celsius < EXTRUDE_MINTEMP) {
					HMI_flag.ETempTooLow_flag = true;
					Popup_Window_ETempTooLow();
					DWIN_UpdateLCD();
					return;
				}
            	#endif
					checkkey = Extruder3;
					//HMI_ValueStruct.Move_E3_scale = current_position.e * MINUNITMULT;
					//current_position.e = HMI_ValueStruct.Move_E1_scale = 0;
					HMI_flag.last_E_Coordinate = current_position.e = HMI_ValueStruct.Move_E3_scale/10;
					DWIN_Draw_Signed_Float(font8x16, Select_Color, 3, 1, 216, MBASE(MROWS -index_axismove + AXISMOVE_CASE_EX3), HMI_ValueStruct.Move_E3_scale);
					EncoderRate.enabled = true;
				break;
				
		 		case AXISMOVE_CASE_EX4: // Extruder4
            	// window tips
            	#ifdef PREVENT_COLD_EXTRUSION
              	if (thermalManager.temp_hotend[0].celsius < EXTRUDE_MINTEMP) {
                	HMI_flag.ETempTooLow_flag = true;
                	Popup_Window_ETempTooLow();
                	DWIN_UpdateLCD();
                	return;
              	}
            	#endif
            	checkkey = Extruder4;
            	//HMI_ValueStruct.Move_E4_scale = current_position.e * MINUNITMULT;
            	//current_position.e = HMI_ValueStruct.Move_E1_scale = 0;
            	HMI_flag.last_E_Coordinate = current_position.e = HMI_ValueStruct.Move_E4_scale/10;
            	DWIN_Draw_Signed_Float(font8x16, Select_Color, 3, 1, 216, MBASE(MROWS -index_axismove + AXISMOVE_CASE_EX4), HMI_ValueStruct.Move_E4_scale);
            	EncoderRate.enabled = true;
            	break;
		  #elif(MIXING_STEPPERS == 3)
		  		case AXISMOVE_CASE_EX2: // Extruder2
				// window tips
            	#ifdef PREVENT_COLD_EXTRUSION
				if (thermalManager.temp_hotend[0].celsius < EXTRUDE_MINTEMP) {
					HMI_flag.ETempTooLow_flag = true;
					Popup_Window_ETempTooLow();
					DWIN_UpdateLCD();
					return;
				}
            	#endif
					checkkey = Extruder2;
					//HMI_ValueStruct.Move_E2_scale = current_position.e * MINUNITMULT;
					//current_position.e = HMI_ValueStruct.Move_E1_scale = 0;
					HMI_flag.last_E_Coordinate = current_position.e = HMI_ValueStruct.Move_E2_scale/10;
					DWIN_Draw_Signed_Float(font8x16, Select_Color, 3, 1, 216, MBASE(MROWS -index_axismove + AXISMOVE_CASE_EX2), HMI_ValueStruct.Move_E2_scale);
					EncoderRate.enabled = true;
				break;
				
				case AXISMOVE_CASE_EX3: // Extruder3
				// window tips
            	#ifdef PREVENT_COLD_EXTRUSION
				if (thermalManager.temp_hotend[0].celsius < EXTRUDE_MINTEMP) {
					HMI_flag.ETempTooLow_flag = true;
					Popup_Window_ETempTooLow();
					DWIN_UpdateLCD();
					return;
				}
            	#endif
					checkkey = Extruder3;
					//HMI_ValueStruct.Move_E3_scale = current_position.e * MINUNITMULT;
					//current_position.e = HMI_ValueStruct.Move_E1_scale = 0;
					HMI_flag.last_E_Coordinate = current_position.e = HMI_ValueStruct.Move_E3_scale/10;
					DWIN_Draw_Signed_Float(font8x16, Select_Color, 3, 1, 216, MBASE(MROWS -index_axismove + AXISMOVE_CASE_EX3), HMI_ValueStruct.Move_E3_scale);
					EncoderRate.enabled = true;
				break;
		  #elif(MIXING_STEPPERS == 2)
		  		case AXISMOVE_CASE_EX2: // Extruder2
				// window tips
            	#ifdef PREVENT_COLD_EXTRUSION
				if (thermalManager.temp_hotend[0].celsius < EXTRUDE_MINTEMP) {
					HMI_flag.ETempTooLow_flag = true;
					Popup_Window_ETempTooLow();
					DWIN_UpdateLCD();
					return;
				}
            	#endif
					checkkey = Extruder2;
					//HMI_ValueStruct.Move_E2_scale = current_position.e * MINUNITMULT;
					//current_position.e = HMI_ValueStruct.Move_E1_scale = 0;
					HMI_flag.last_E_Coordinate = current_position.e = HMI_ValueStruct.Move_E2_scale/10;
					DWIN_Draw_Signed_Float(font8x16, Select_Color, 3, 1, 216, MBASE(MROWS -index_axismove + AXISMOVE_CASE_EX2), HMI_ValueStruct.Move_E2_scale);
					EncoderRate.enabled = true;
				break;
		  #endif
	   #endif //end if enable(MIXING_EXTRUDER)
	 #endif //end HAS_HOTEND
    }
  }
  DWIN_UpdateLCD();
}

/* TemperatureID */
void HMI_Temperature() {
  ENCODER_DiffState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;

  // Avoid flicker by updating only the previous menu
  if (encoder_diffState == ENCODER_DIFF_CW) {
    if (select_temp.inc(1 + TEMP_CASE_TOTAL)) Move_Highlight(1, select_temp.now);
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW) {
    if (select_temp.dec()) Move_Highlight(-1, select_temp.now);
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    switch (select_temp.now) {
      case 0: // Back
        checkkey = Control;
        select_control.set(1);
        index_control = MROWS;
        Draw_Control_Menu();
        break;
      #if HAS_HOTEND
        case TEMP_CASE_TEMP: // Nozzle temperature
          checkkey = ETemp;
          HMI_ValueStruct.E_Temp = thermalManager.temp_hotend[0].target;
          DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Select_Color, 3, 216, MBASE(1), thermalManager.temp_hotend[0].target);
          EncoderRate.enabled = true;
          break;
      #endif
      #if HAS_HEATED_BED
        case TEMP_CASE_BED: // Bed temperature
          checkkey = BedTemp;
          HMI_ValueStruct.Bed_Temp = thermalManager.temp_bed.target;
          DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Select_Color, 3, 216, MBASE(2), thermalManager.temp_bed.target);
          EncoderRate.enabled = true;
          break;
      #endif
      #if HAS_FAN
        case TEMP_CASE_FAN: // Fan speed
          checkkey = FanSpeed;
          HMI_ValueStruct.Fan_speed = thermalManager.fan_speed[0];
          DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Select_Color, 3, 216, MBASE(3), thermalManager.fan_speed[0]);
          EncoderRate.enabled = true;
          break;
      #endif
      #if HAS_HOTEND
        case TEMP_CASE_PLA: { // PLA preheat setting
          checkkey = PLAPreheat;
          select_PLA.reset();
          HMI_ValueStruct.show_mode = -2;

          Clear_Main_Window();

		  //DWIN_ICON_Show(ICON, Title_Coordinate[HMI_flag.language][Title_PLA], 14, 7);
		  DWIN_JPG_CacheToN(1,HMI_flag.Title_Menu_Bankup);
		  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Title_PLA, Menu_Coordinate,14, 7);
		  DWIN_JPG_CacheToN(1,HMI_flag.language+1);
  		  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, PLA_ABS_Menu_Nozzle, Menu_Coordinate,LBLX, MBASE(PREHEAT_CASE_TEMP));
		  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, PLA_ABS_Menu_Temp, Menu_Coordinate,LBLX+58, MBASE(PREHEAT_CASE_TEMP));
		  #if HAS_HEATED_BED
		   	DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, PLA_ABS_Menu_Bed, Menu_Coordinate,LBLX, MBASE(PREHEAT_CASE_BED));
		  	DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, PLA_ABS_Menu_Temp, Menu_Coordinate,LBLX+33, MBASE(PREHEAT_CASE_BED));
		  #endif
		  #if HAS_FAN
		  	DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, PLA_ABS_Menu_Fan_Speed, Menu_Coordinate,LBLX, MBASE(PREHEAT_CASE_FAN));
		  #endif
		  #if ENABLED(EEPROM_SETTINGS)
		    DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, PLA_ABS_Menu_Store, Menu_Coordinate,LBLX, MBASE(PREHEAT_CASE_SAVE));
		  	DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, PLA_ABS_Menu_Settings, Menu_Coordinate,LBLX+50, MBASE(PREHEAT_CASE_SAVE));
		  #endif
          
          Draw_Back_First();

          uint8_t i = 0;
          Draw_Menu_Line(++i, ICON_SetEndTemp);
          DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(i), ui.material_preset[0].hotend_temp);
          #if HAS_HEATED_BED
            Draw_Menu_Line(++i, ICON_SetBedTemp);
            DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(i), ui.material_preset[0].bed_temp);
          #endif
          #if HAS_FAN
            Draw_Menu_Line(++i, ICON_FanSpeed);
            DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(i), ui.material_preset[0].fan_speed);
          #endif
          #if ENABLED(EEPROM_SETTINGS)
            Draw_Menu_Line(++i, ICON_WriteEEPROM);
          #endif
        } break;

        case TEMP_CASE_ABS: { // ABS preheat setting
          checkkey = ABSPreheat;
          select_ABS.reset();
          HMI_ValueStruct.show_mode = -3;

          Clear_Main_Window();

		  //DWIN_ICON_Show(ICON, Title_Coordinate[HMI_flag.language][Title_ABS], 14, 7);
		  DWIN_JPG_CacheToN(1,HMI_flag.Title_Menu_Bankup);
		  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Title_ABS, Menu_Coordinate,14, 7);
		  DWIN_JPG_CacheToN(1,HMI_flag.language+1);
  		  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, PLA_ABS_Menu_Nozzle, Menu_Coordinate,LBLX, MBASE(PREHEAT_CASE_TEMP));
		  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, PLA_ABS_Menu_Temp, Menu_Coordinate,LBLX+58, MBASE(PREHEAT_CASE_TEMP));
		  #if HAS_HEATED_BED
		   	DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, PLA_ABS_Menu_Bed, Menu_Coordinate,LBLX, MBASE(PREHEAT_CASE_BED));
		  	DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, PLA_ABS_Menu_Temp, Menu_Coordinate,LBLX+33, MBASE(PREHEAT_CASE_BED));
		  #endif
		  #if HAS_FAN
		  	DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, PLA_ABS_Menu_Fan_Speed, Menu_Coordinate,LBLX, MBASE(PREHEAT_CASE_FAN));
		  #endif
		  #if ENABLED(EEPROM_SETTINGS)
		    DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, PLA_ABS_Menu_Store, Menu_Coordinate,LBLX, MBASE(PREHEAT_CASE_SAVE));
		  	DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, PLA_ABS_Menu_Settings, Menu_Coordinate,LBLX+50, MBASE(PREHEAT_CASE_SAVE));
		  #endif
          
          Draw_Back_First();

          uint8_t i = 0;
          Draw_Menu_Line(++i, ICON_SetEndTemp);
          DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(i), ui.material_preset[1].hotend_temp);
          #if HAS_HEATED_BED
            Draw_Menu_Line(++i, ICON_SetBedTemp);
            DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(i), ui.material_preset[1].bed_temp);
          #endif
          #if HAS_FAN
            Draw_Menu_Line(++i, ICON_FanSpeed);
            DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(i), ui.material_preset[1].fan_speed);
          #endif
          #if ENABLED(EEPROM_SETTINGS)
            Draw_Menu_Line(++i, ICON_WriteEEPROM);
          #endif

        } break;

      #endif // HAS_HOTEND
    }
  }
  DWIN_UpdateLCD();
}

inline void Draw_Max_Speed_Menu() {
  Clear_Main_Window();

  //DWIN_ICON_Show(ICON, Title_Coordinate[HMI_flag.language][Title_FEEDRATE], 14, 7);
  DWIN_JPG_CacheToN(1,HMI_flag.Title_Menu_Bankup);
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Title_FEEDRATE, Menu_Coordinate,14, 7);
  DWIN_JPG_CacheToN(1,HMI_flag.language+1);
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Feedrate_Menu_Max, Menu_Coordinate,LBLX, MBASE(1));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Feedrate_Menu_Feedrate, Menu_Coordinate,LBLX+Max_X_Coordinate[HMI_flag.language], MBASE(1));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Feedrate_Menu_X, Menu_Coordinate,LBLX+Max_X_Coordinate[HMI_flag.language]+Feedrate_X_Coordinate[HMI_flag.language], MBASE(1));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Feedrate_Menu_Max, Menu_Coordinate,LBLX, MBASE(2));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Feedrate_Menu_Feedrate, Menu_Coordinate,LBLX+Max_X_Coordinate[HMI_flag.language], MBASE(2));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Feedrate_Menu_Y, Menu_Coordinate,LBLX+Max_X_Coordinate[HMI_flag.language]+Feedrate_X_Coordinate[HMI_flag.language], MBASE(2));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Feedrate_Menu_Max, Menu_Coordinate,LBLX, MBASE(3));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Feedrate_Menu_Feedrate, Menu_Coordinate,LBLX+Max_X_Coordinate[HMI_flag.language], MBASE(3));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Feedrate_Menu_Z, Menu_Coordinate,LBLX+Max_X_Coordinate[HMI_flag.language]+Feedrate_X_Coordinate[HMI_flag.language], MBASE(3));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Feedrate_Menu_Max, Menu_Coordinate,LBLX, MBASE(4));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Feedrate_Menu_Feedrate, Menu_Coordinate,LBLX+Max_X_Coordinate[HMI_flag.language], MBASE(4));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Feedrate_Menu_E, Menu_Coordinate,LBLX+Max_X_Coordinate[HMI_flag.language]+Feedrate_X_Coordinate[HMI_flag.language], MBASE(4));
  
  Draw_Back_First();
  LOOP_L_N(i, 3 + ENABLED(HAS_HOTEND)) Draw_Menu_Line(i + 1, ICON_MaxSpeedX + i);
  DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 4, 210, MBASE(1), planner.settings.max_feedrate_mm_s[X_AXIS]);
  DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 4, 210, MBASE(2), planner.settings.max_feedrate_mm_s[Y_AXIS]);
  DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 4, 210, MBASE(3), planner.settings.max_feedrate_mm_s[Z_AXIS]);
  #if HAS_HOTEND
    DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 4, 210, MBASE(4), planner.settings.max_feedrate_mm_s[E_AXIS]);
  #endif
}

inline void Draw_Max_Accel_Menu() {
  Clear_Main_Window();

  //DWIN_ICON_Show(ICON, Title_Coordinate[HMI_flag.language][Title_ACCEL], 14, 7);
  DWIN_JPG_CacheToN(1,HMI_flag.Title_Menu_Bankup);
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Title_ACCEL, Menu_Coordinate,14, 7);
  DWIN_JPG_CacheToN(1,HMI_flag.language+1);
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Accel_Menu_Max, Menu_Coordinate,LBLX, MBASE(1));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Accel_Menu_Accel, Menu_Coordinate,LBLX+Max_X_Coordinate[HMI_flag.language], MBASE(1));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Accel_Menu_X, Menu_Coordinate,LBLX+Max_X_Coordinate[HMI_flag.language]+Accel_X_Coordinate[HMI_flag.language], MBASE(1));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Accel_Menu_Max, Menu_Coordinate,LBLX, MBASE(2));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Accel_Menu_Accel, Menu_Coordinate,LBLX+Max_X_Coordinate[HMI_flag.language], MBASE(2));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Accel_Menu_Y, Menu_Coordinate,LBLX+Max_X_Coordinate[HMI_flag.language]+Accel_X_Coordinate[HMI_flag.language], MBASE(2));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Accel_Menu_Max, Menu_Coordinate,LBLX, MBASE(3));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Accel_Menu_Accel, Menu_Coordinate,LBLX+Max_X_Coordinate[HMI_flag.language], MBASE(3));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Accel_Menu_Z, Menu_Coordinate,LBLX+Max_X_Coordinate[HMI_flag.language]+Accel_X_Coordinate[HMI_flag.language], MBASE(3));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Accel_Menu_Max, Menu_Coordinate,LBLX, MBASE(4));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Accel_Menu_Accel, Menu_Coordinate,LBLX+Max_X_Coordinate[HMI_flag.language], MBASE(4));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Accel_Menu_E, Menu_Coordinate,LBLX+Max_X_Coordinate[HMI_flag.language]+Accel_X_Coordinate[HMI_flag.language], MBASE(4));
  
  Draw_Back_First();
  LOOP_L_N(i, 3 + ENABLED(HAS_HOTEND)) Draw_Menu_Line(i + 1, ICON_MaxAccX + i);
  DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 4, 210, MBASE(1), planner.settings.max_acceleration_mm_per_s2[X_AXIS]);
  DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 4, 210, MBASE(2), planner.settings.max_acceleration_mm_per_s2[Y_AXIS]);
  DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 4, 210, MBASE(3), planner.settings.max_acceleration_mm_per_s2[Z_AXIS]);
  #if HAS_HOTEND
    DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 4, 210, MBASE(4), planner.settings.max_acceleration_mm_per_s2[E_AXIS]);
  #endif
}

inline void Draw_Max_Jerk_Menu() {
  Clear_Main_Window();

  //DWIN_ICON_Show(ICON, Title_Coordinate[HMI_flag.language][Title_JERK], 14, 7);
  DWIN_JPG_CacheToN(1,HMI_flag.Title_Menu_Bankup);
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Title_JERK, Menu_Coordinate,14, 7);
  DWIN_JPG_CacheToN(1,HMI_flag.language+1);
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Jerk_Menu_Max, Menu_Coordinate,LBLX, MBASE(1));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Jerk_Menu_Jerk, Menu_Coordinate,LBLX+Max_X_Coordinate[HMI_flag.language], MBASE(1));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Jerk_Menu_X, Menu_Coordinate,LBLX+Max_X_Coordinate[HMI_flag.language]+Jerk_X_Coordinate[HMI_flag.language], MBASE(1));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Jerk_Menu_Max, Menu_Coordinate,LBLX, MBASE(2));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Jerk_Menu_Jerk, Menu_Coordinate,LBLX+Max_X_Coordinate[HMI_flag.language], MBASE(2));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Jerk_Menu_Y, Menu_Coordinate,LBLX+Max_X_Coordinate[HMI_flag.language]+Jerk_X_Coordinate[HMI_flag.language], MBASE(2));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Jerk_Menu_Max, Menu_Coordinate,LBLX, MBASE(3));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Jerk_Menu_Jerk, Menu_Coordinate,LBLX+Max_X_Coordinate[HMI_flag.language], MBASE(3));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Jerk_Menu_Z, Menu_Coordinate,LBLX+Max_X_Coordinate[HMI_flag.language]+Jerk_X_Coordinate[HMI_flag.language], MBASE(3));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Jerk_Menu_Max, Menu_Coordinate,LBLX, MBASE(4));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Jerk_Menu_Jerk, Menu_Coordinate,LBLX+Max_X_Coordinate[HMI_flag.language], MBASE(4));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Jerk_Menu_E, Menu_Coordinate,LBLX+Max_X_Coordinate[HMI_flag.language]+Jerk_X_Coordinate[HMI_flag.language], MBASE(4));
  
  Draw_Back_First();
  LOOP_L_N(i, 3 + ENABLED(HAS_HOTEND)) Draw_Menu_Line(i + 1, ICON_MaxSpeedJerkX + i);
  DWIN_Draw_FloatValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 1, 210, MBASE(1), planner.max_jerk[X_AXIS] * MINUNITMULT);
  DWIN_Draw_FloatValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 1, 210, MBASE(2), planner.max_jerk[Y_AXIS] * MINUNITMULT);
  DWIN_Draw_FloatValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 1, 210, MBASE(3), planner.max_jerk[Z_AXIS] * MINUNITMULT);
  #if HAS_HOTEND
    DWIN_Draw_FloatValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 1, 210, MBASE(4), planner.max_jerk[E_AXIS] * MINUNITMULT);
  #endif
}

inline void Draw_Steps_Menu() {
  Clear_Main_Window();

  //DWIN_ICON_Show(ICON, Title_Coordinate[HMI_flag.language][Title_STEP], 14, 7);
  DWIN_JPG_CacheToN(1,HMI_flag.Title_Menu_Bankup);
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Title_STEP, Menu_Coordinate,14, 7);
  DWIN_JPG_CacheToN(1,HMI_flag.language+1);
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Step_Menu_Max, Menu_Coordinate,LBLX, MBASE(1));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Step_Menu_Step, Menu_Coordinate,LBLX+Max_X_Coordinate[HMI_flag.language], MBASE(1));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Step_Menu_X, Menu_Coordinate,LBLX+Max_X_Coordinate[HMI_flag.language]+Step_X_Coordinate[HMI_flag.language], MBASE(1));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Step_Menu_Max, Menu_Coordinate,LBLX, MBASE(2));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Step_Menu_Step, Menu_Coordinate,LBLX+Max_X_Coordinate[HMI_flag.language], MBASE(2));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Step_Menu_Y, Menu_Coordinate,LBLX+Max_X_Coordinate[HMI_flag.language]+Step_X_Coordinate[HMI_flag.language], MBASE(2));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Step_Menu_Max, Menu_Coordinate,LBLX, MBASE(3));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Step_Menu_Step, Menu_Coordinate,LBLX+Max_X_Coordinate[HMI_flag.language], MBASE(3));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Step_Menu_Z, Menu_Coordinate,LBLX+Max_X_Coordinate[HMI_flag.language]+Step_X_Coordinate[HMI_flag.language], MBASE(3));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Step_Menu_Max, Menu_Coordinate,LBLX, MBASE(4));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Step_Menu_Step, Menu_Coordinate,LBLX+Max_X_Coordinate[HMI_flag.language], MBASE(4));
  DWIN_Frame_AreaCopy_Index(1,HMI_flag.language, Step_Menu_E, Menu_Coordinate,LBLX+Max_X_Coordinate[HMI_flag.language]+Step_X_Coordinate[HMI_flag.language], MBASE(4));
  
  Draw_Back_First();
  LOOP_L_N(i, 3 + ENABLED(HAS_HOTEND)) Draw_Menu_Line(i + 1, ICON_StepX + i);
  DWIN_Draw_FloatValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 1, 210, MBASE(1), planner.settings.axis_steps_per_mm[X_AXIS] * MINUNITMULT);
  DWIN_Draw_FloatValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 1, 210, MBASE(2), planner.settings.axis_steps_per_mm[Y_AXIS] * MINUNITMULT);
  DWIN_Draw_FloatValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 1, 210, MBASE(3), planner.settings.axis_steps_per_mm[Z_AXIS] * MINUNITMULT);
  #if HAS_HOTEND
    DWIN_Draw_FloatValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 1, 210, MBASE(4), planner.settings.axis_steps_per_mm[E_AXIS] * MINUNITMULT);
  #endif
}

/* Motion */
void HMI_Motion() {
  ENCODER_DiffState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;

  // Avoid flicker by updating only the previous menu
  if (encoder_diffState == ENCODER_DIFF_CW) {
    if (select_motion.inc(1 + MOTION_CASE_TOTAL)) Move_Highlight(1, select_motion.now);
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW) {
    if (select_motion.dec()) Move_Highlight(-1, select_motion.now);
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    switch (select_motion.now) {
      case 0: // Back
        checkkey = Control;
        select_control.set(CONTROL_CASE_MOVE);
        index_control = MROWS;
        Draw_Control_Menu();
        break;
      case MOTION_CASE_RATE:   // Max speed
        checkkey = MaxSpeed;
        select_speed.reset();
        Draw_Max_Speed_Menu();
        break;
      case MOTION_CASE_ACCEL:  // Max acceleration
        checkkey = MaxAcceleration;
        select_acc.reset();
        Draw_Max_Accel_Menu();
        break;
      #if HAS_CLASSIC_JERK
        case MOTION_CASE_JERK: // Max jerk
          checkkey = MaxJerk;
          select_jerk.reset();
          Draw_Max_Jerk_Menu();
         break;
      #endif
      case MOTION_CASE_STEPS:  // Steps per mm
        checkkey = Step;
        select_step.reset();
        Draw_Steps_Menu();
        break;
      default: break;
    }
  }
  DWIN_UpdateLCD();
}


/* Bltouch */
#if ENABLED(BLTOUCH)
void HMI_Option_Bltouch() {
  ENCODER_DiffState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;

  // Avoid flicker by updating only the previous menu
  if (encoder_diffState == ENCODER_DIFF_CW) {
    if (select_bltouch.inc(1 + BLTOUCH_CASE_TOTAL)) {
		Move_Highlight(1, select_bltouch.now);
		for(uint8_t i=BLTOUCH_CASE_RESET; i<BLTOUCH_CASE_TOTAL+1; i++) DWIN_ICON_Show(ICON, ICON_Blank, 216, MBASE(i));
			DWIN_ICON_Show(ICON, ICON_Cursor, 216, MBASE(select_bltouch.now));
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW) {
    if (select_bltouch.dec()) {
		Move_Highlight(-1, select_bltouch.now);
		for(uint8_t i=BLTOUCH_CASE_RESET; i<BLTOUCH_CASE_TOTAL+1; i++) DWIN_ICON_Show(ICON, ICON_Blank, 216, MBASE(i));
		if(select_bltouch.now) DWIN_ICON_Show(ICON, ICON_Cursor, 216, MBASE(select_bltouch.now));
    }
	
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    switch (select_bltouch.now) {
	  case 0: 					// Back
        checkkey = Control;
        select_control.set(CONTROL_CASE_BLTOUCH);
        index_control = MROWS;
        Draw_Control_Menu();
        break;

      case BLTOUCH_CASE_RESET: 	// Reset
        checkkey = Bltouch;
		bltouch._reset();
        break;

	  case BLTOUCH_CASE_TEST: 	// Test
        checkkey = Bltouch;
		bltouch._selftest();
        break;

	  case BLTOUCH_CASE_STOW: 	// Stow
        checkkey = Bltouch;
		bltouch._stow();
        break;
	  
	  case BLTOUCH_CASE_DEPLOY: 	// Proc
        checkkey = Bltouch;
		bltouch._deploy();
        break;
	  case BLTOUCH_CASE_SW: 	// sw
        checkkey = Bltouch;
		bltouch._set_SW_mode();
        break;
      default: break;
    }
  }
  DWIN_UpdateLCD();
}
#endif

/* DMixer */
void HMI_Mixer() {
  ENCODER_DiffState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;

  // Avoid flicker by updating only the previous menu
  if (encoder_diffState == ENCODER_DIFF_CW) {
    if (select_mixer.inc(1 + MIXER_CASE_TOTAL)) Move_Highlight(1, select_mixer.now);
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW) {
    if (select_mixer.dec()) Move_Highlight(-1, select_mixer.now);
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    switch (select_mixer.now) {
	  case 0: 					// Back
	    if(!HMI_flag.Is_Mixer_Print){
        	checkkey = Control;
        	select_control.set(CONTROL_CASE_MIXER);
        	index_control = MROWS;
        	Draw_Control_Menu();
	  	}
		else{
			checkkey = Tune;
        	HMI_ValueStruct.show_mode = 0;
			select_tune.set(TUNE_CASE_SPEED);
        	index_tune = MROWS;
        	Draw_Tune_Menu();
		}
        break;

      case MIXER_CASE_MANUAL: 	// Manual
        checkkey = Mix_Manual;
        select_manual.reset();
	    MixerCfg.Vtool_Bankup  = mixer.selected_vtool;
	  	updata_mixer_from_vtool();
		MIXER_STEPPER_LOOP(i) MixerCfg.Manual_Percent[mixer.selected_vtool][i] = mixer.mix[i];
		index_manual = MROWS;
		MixerCfg.Mixer_Mode_Rg = 0;
		recovery.save(true);
		Draw_Mixer_Manual_Menu();
		//Draw_Print_ProgressModel();
        break;
      case MIXER_CASE_AUTO:   	// Auto
        checkkey = Mix_Auto;
        select_auto.reset();
        
		mixer.selected_vtool = MixerCfg.Auto_VTool[VTOOL_START];     
		updata_mixer_from_vtool();
		MIXER_STEPPER_LOOP(i) MixerCfg.Start_Percent[i] = mixer.mix[i];
		mixer.selected_vtool = MixerCfg.Auto_VTool[VTOOL_END];     
		updata_mixer_from_vtool();
		MIXER_STEPPER_LOOP(i) MixerCfg.End_Percent[i] = mixer.mix[i];
		
	    index_auto = MROWS;
		MixerCfg.Mixer_Mode_Rg = 1;
		mixer.selected_vtool = MixerCfg.occupy_vtool;
		recovery.save(true);
		Draw_Mixer_Auto_Menu();
		//Draw_Print_ProgressModel();
        break;
      case MIXER_CASE_RANDOM:  	// Random
        checkkey = Mix_Random;
        select_random.reset();
	    index_random = MROWS;
		MixerCfg.Mixer_Mode_Rg = 2;
		mixer.selected_vtool = MixerCfg.occupy_vtool;
		recovery.save(true);
		Draw_Mixer_Random_Menu();
		//Draw_Print_ProgressModel();
		break;
	  case MIXER_CASE_VTOOL:  	// vtool
        checkkey = Mix_Vtool;
		MixerCfg.Mixer_Mode_Rg = 0;
		recovery.save(true);
        DWIN_Draw_IntValue(true, true, 0, font8x16, Select_Color, Color_Bg_Black, 3, 216, MBASE(4), mixer.selected_vtool);
	  	EncoderRate.enabled = true;
        break;
      default: break;
    }
  }
  DWIN_UpdateLCD();
}

/* Mixer_Manual */
void HMI_Mixer_Manual() {
  uint8_t i = 0;
  signed int Temp_Buff[4] = {0};
  
  ENCODER_DiffState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;

  // Avoid flicker by updating only the previous menu
  if (encoder_diffState == ENCODER_DIFF_CW) {
    if (select_manual.inc(1 + MANUAL_CASE_TOTAL)) Move_Highlight(1, select_manual.now);
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW) {
    if (select_manual.dec()) Move_Highlight(-1, select_manual.now);
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    switch (select_manual.now) {
	  case 0: 						// Back
        checkkey = DMixer;
        select_mixer.set(MIXER_CASE_MANUAL);
        Draw_Mixer_Menu();
        break;

	  #if ENABLED(MIXING_EXTRUDER)
		#if(MIXING_STEPPERS == 4)
			case MANUAL_CASE_EXTRUDER1: 	// ex1
      			checkkey = Mix_Manual_Extruder1;
				DWIN_Draw_IntValue(true, true, 0, font8x16, Select_Color, Color_Bg_Black, 3, 216, MBASE(1), MixerCfg.Manual_Percent[mixer.selected_vtool][0]);
				DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(5), mixer.selected_vtool);
				EncoderRate.enabled = true;
        		break;
      		case MANUAL_CASE_EXTRUDER2: 	// ex2
        		checkkey = Mix_Manual_Extruder2;
        		DWIN_Draw_IntValue(true, true, 0, font8x16, Select_Color, Color_Bg_Black, 3, 216, MBASE(2), MixerCfg.Manual_Percent[mixer.selected_vtool][1]);
				DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(5), mixer.selected_vtool);
				EncoderRate.enabled = true;
        		break;
      		case MANUAL_CASE_EXTRUDER3:  	// ex3
        		checkkey = Mix_Manual_Extruder3;
        		DWIN_Draw_IntValue(true, true, 0, font8x16, Select_Color, Color_Bg_Black, 3, 216, MBASE(3), MixerCfg.Manual_Percent[mixer.selected_vtool][2]);
				DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(5), mixer.selected_vtool);
				EncoderRate.enabled = true;
        		break;
	  		case MANUAL_CASE_EXTRUDER4:  	// ex4
        		checkkey = Mix_Manual_Extruder4;
        		DWIN_Draw_IntValue(true, true, 0, font8x16, Select_Color, Color_Bg_Black, 3, 216, MBASE(4), MixerCfg.Manual_Percent[mixer.selected_vtool][3]);
	  			DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(5), mixer.selected_vtool);
	  			EncoderRate.enabled = true;
        		break;
		#elif(MIXING_STEPPERS == 3)
			case MANUAL_CASE_EXTRUDER1: 	// ex1
      			checkkey = Mix_Manual_Extruder1;
				DWIN_Draw_IntValue(true, true, 0, font8x16, Select_Color, Color_Bg_Black, 3, 216, MBASE(1), MixerCfg.Manual_Percent[mixer.selected_vtool][0]);
				DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(4), mixer.selected_vtool);
				EncoderRate.enabled = true;
        		break;
      		case MANUAL_CASE_EXTRUDER2: 	// ex2
        		checkkey = Mix_Manual_Extruder2;
        		DWIN_Draw_IntValue(true, true, 0, font8x16, Select_Color, Color_Bg_Black, 3, 216, MBASE(2), MixerCfg.Manual_Percent[mixer.selected_vtool][1]);
				DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(4), mixer.selected_vtool);
				EncoderRate.enabled = true;
        		break;
      		case MANUAL_CASE_EXTRUDER3:  	// ex3
        		checkkey = Mix_Manual_Extruder3;
        		DWIN_Draw_IntValue(true, true, 0, font8x16, Select_Color, Color_Bg_Black, 3, 216, MBASE(3), MixerCfg.Manual_Percent[mixer.selected_vtool][2]);
				DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(4), mixer.selected_vtool);
				EncoderRate.enabled = true;
        		break;
		#elif(MIXING_STEPPERS == 2)
			case MANUAL_CASE_EXTRUDER1: 	// ex1
      			checkkey = Mix_Manual_Extruder1;
				DWIN_Draw_IntValue(true, true, 0, font8x16, Select_Color, Color_Bg_Black, 3, 216, MBASE(1), MixerCfg.Manual_Percent[mixer.selected_vtool][0]);
				DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(3), mixer.selected_vtool);
				EncoderRate.enabled = true;
        		break;
      		case MANUAL_CASE_EXTRUDER2: 	// ex2
        		checkkey = Mix_Manual_Extruder2;
        		DWIN_Draw_IntValue(true, true, 0, font8x16, Select_Color, Color_Bg_Black, 3, 216, MBASE(2), MixerCfg.Manual_Percent[mixer.selected_vtool][1]);
				DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(3), mixer.selected_vtool);
				EncoderRate.enabled = true;
        		break;
		#endif
	  #endif
		
	  case MANUAL_CASE_OK:  		// OK
	    checkkey = Mix_Manual;
		DWIN_Draw_IntValue(true, true, 0, font8x16, Select_Color, Color_Bg_Black, 3, 216, MBASE(MIXING_STEPPERS+1), mixer.selected_vtool);
		//MIXER_STEPPER_LOOP(i) mixer.mix[i] = MixerCfg.Manual_Percent[mixer.selected_vtool][i];
		//recalculation_mixer_percent();
		//MIXER_STEPPER_LOOP(i) {
			//mixer.color[mixer.selected_vtool][i] = mixer.mix[i];
			//DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(i+1), mixer.mix[i]);
			//DWIN_UpdateLCD();
		//}
	    #if(MIXING_STEPPERS == 4)
			if(!Check_Percent_equal()) {
				for(i=0;i<4;i++){
				mixer.color[mixer.selected_vtool][i] = mixer.mix[i] =  MixerCfg.Manual_Percent[mixer.selected_vtool][i] = 25;
    			DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(i+1), MixerCfg.Manual_Percent[mixer.selected_vtool][i]);
				}
	    	}
			else 
			{
				for(i=0;i<4;i++){
				if(i < 3) Temp_Buff[i] = 100*MixerCfg.Manual_Percent[mixer.selected_vtool][i]/(MixerCfg.Manual_Percent[mixer.selected_vtool][0]+MixerCfg.Manual_Percent[mixer.selected_vtool][1]+MixerCfg.Manual_Percent[mixer.selected_vtool][2]+MixerCfg.Manual_Percent[mixer.selected_vtool][3]);
				else Temp_Buff[i] = 100 - Temp_Buff[0]-Temp_Buff[1]-Temp_Buff[2];	
  				}

				for(i=0;i<4;i++){
				mixer.color[mixer.selected_vtool][i] = mixer.mix[i] =  MixerCfg.Manual_Percent[mixer.selected_vtool][i] = Temp_Buff[i];
    			DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(i+1), MixerCfg.Manual_Percent[mixer.selected_vtool][i]);
				}
			}
		#elif(MIXING_STEPPERS == 3)
	    	if(!Check_Percent_equal()) {
				for(i=0;i<2;i++){
				mixer.mix[i] =  MixerCfg.Manual_Percent[mixer.selected_vtool][i] = 33;
    			DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(i+1), MixerCfg.Manual_Percent[mixer.selected_vtool][i]);
				DWIN_UpdateLCD();
				}
				i++;
				mixer.mix[i] =  MixerCfg.Manual_Percent[mixer.selected_vtool][i] = 34;
    			DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(i+1), MixerCfg.Manual_Percent[mixer.selected_vtool][i]);
	    	}
			else 
			{
				for(i=0;i<3;i++){
				if(i < 2) Temp_Buff[i] = 100*MixerCfg.Manual_Percent[mixer.selected_vtool][i]/(MixerCfg.Manual_Percent[mixer.selected_vtool][0]+MixerCfg.Manual_Percent[mixer.selected_vtool][1]+MixerCfg.Manual_Percent[mixer.selected_vtool][2]);
				else Temp_Buff[i] = 100 - Temp_Buff[0]-Temp_Buff[1];	
  				}

				for(i=0;i<3;i++){
				mixer.mix[i] =  MixerCfg.Manual_Percent[mixer.selected_vtool][i] = Temp_Buff[i];
    			DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(i+1), MixerCfg.Manual_Percent[mixer.selected_vtool][i]);
				mixer.color[mixer.selected_vtool][i] = MixerCfg.Manual_Percent[mixer.selected_vtool][i];
				}
			}
		#elif(MIXING_STEPPERS == 2)
	    	if(!Check_Percent_equal()) {
				for(i=0;i<2;i++){
				mixer.mix[i] =  MixerCfg.Manual_Percent[mixer.selected_vtool][i] = 50;
    			DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(i+1), MixerCfg.Manual_Percent[mixer.selected_vtool][i]);
				}
	    	}
			else 
			{
				for(i=0;i<2;i++){
				if(i < 1) Temp_Buff[i] = 100*MixerCfg.Manual_Percent[mixer.selected_vtool][i]/(MixerCfg.Manual_Percent[mixer.selected_vtool][0]+MixerCfg.Manual_Percent[mixer.selected_vtool][1]);
				else Temp_Buff[i] = 100 - Temp_Buff[0];	
  				}

				for(i=0;i<2;i++){
				mixer.mix[i] =  MixerCfg.Manual_Percent[mixer.selected_vtool][i] = Temp_Buff[i];
    			DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(i+1), MixerCfg.Manual_Percent[mixer.selected_vtool][i]);
				mixer.color[mixer.selected_vtool][i] = MixerCfg.Manual_Percent[mixer.selected_vtool][i];
				//DWIN_UpdateLCD();
				}
			}
		#endif
      break;
      default: break;
    }
  }
  DWIN_UpdateLCD();
}

/* Mixer_Auto */
void HMI_Mixer_Auto() {
  ENCODER_DiffState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;
  
  // Avoid flicker by updating only the previous menu
  if (encoder_diffState == ENCODER_DIFF_CW) {
    if (select_auto.inc(1 + AUTO_CASE_TOTAL)) {
      if (select_auto.now > MROWS && select_auto.now > index_auto) {
        index_auto = select_auto.now;
      }
      else {
        Move_Highlight(1, select_auto.now + MROWS - index_auto);
      }
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW) {
    if (select_auto.dec()) {
      if (select_auto.now < index_auto - MROWS) {
        index_auto--;
        Scroll_Menu(DWIN_SCROLL_DOWN);
      }
      else {
        Move_Highlight(-1, select_auto.now + MROWS - index_auto);
      }
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    switch (select_auto.now) {
	  case 0: 						// Back
        checkkey = DMixer;
        select_mixer.set(MIXER_CASE_AUTO);
        Draw_Mixer_Menu();
        break;
	  case AUTO_CASE_ZPOS_START:  		// zpos_start
        checkkey = Auto_Zpos_Start;
		HMI_ValueStruct.Auto_Zstart_scale = MixerCfg.Auto_Zpos[ZPOS_START]*MINUNITMULT;
		DWIN_Draw_Signed_Float(font8x16, Select_Color, 3, 1, 216, MBASE(MROWS -index_auto + AUTO_CASE_ZPOS_START), HMI_ValueStruct.Auto_Zstart_scale);
	    EncoderRate.enabled = true;
        break;
	  case AUTO_CASE_ZPOS_END:  		// zpos_end
        checkkey = Auto_Zpos_End;
		HMI_ValueStruct.Auto_Zend_scale = MixerCfg.Auto_Zpos[ZPOS_END]*MINUNITMULT;
		DWIN_Draw_Signed_Float(font8x16, Select_Color, 3, 1, 216, MBASE(MROWS -index_auto + AUTO_CASE_ZPOS_END), HMI_ValueStruct.Auto_Zend_scale);
	    EncoderRate.enabled = true;
        break;
	  case AUTO_CASE_VTOOL_START:  		// vtool_start
        checkkey = Mix_VTool_Start;
        DWIN_Draw_IntValue(true, true, 0, font8x16, Select_Color, Color_Bg_Black, 3, 216, MBASE(MROWS -index_auto + AUTO_CASE_VTOOL_START), MixerCfg.Auto_VTool[VTOOL_START]);
		EncoderRate.enabled = true;
        break;
	  case AUTO_CASE_VTOOL_END:  		// vtool_end
        checkkey = Mix_VTool_End;
         DWIN_Draw_IntValue(true, true, 0, font8x16, Select_Color, Color_Bg_Black, 3, 216, MBASE(MROWS -index_auto + AUTO_CASE_VTOOL_END), MixerCfg.Auto_VTool[VTOOL_END]);
		EncoderRate.enabled = true;
        break;
      default: break;
    }
  }
  DWIN_UpdateLCD();
}

/* Mixer_Auto */
void HMI_Mixer_Random() {
  ENCODER_DiffState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;
  
  // Avoid flicker by updating only the previous menu
  if (encoder_diffState == ENCODER_DIFF_CW) {
    if (select_random.inc(1 + RANDOM_CASE_TOTAL)) {
      if (select_random.now > MROWS && select_random.now > index_random) {
        index_random = select_random.now;
      }
      else {
        Move_Highlight(1, select_random.now + MROWS - index_random);
      }
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW) {
    if (select_random.dec()) {
      if (select_random.now < index_random - MROWS) {
        index_random--;
        Scroll_Menu(DWIN_SCROLL_DOWN);
      }
      else {
        Move_Highlight(-1, select_random.now + MROWS - index_random);
      }
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    switch (select_random.now) {
	  case 0: 						// Back
        checkkey = DMixer;
        select_mixer.set(MIXER_CASE_RANDOM);
        Draw_Mixer_Menu();
        break;
		
	  case RANDOM_CASE_ZPOS_START:  		// zpos_start
        checkkey = Random_Zpos_Start;
		HMI_ValueStruct.Random_Zstart_scale = MixerCfg.Random_Zpos[ZPOS_START]*MINUNITMULT;
		DWIN_Draw_Signed_Float(font8x16, Select_Color, 3, 1, 216, MBASE(MROWS -index_random + RANDOM_CASE_ZPOS_START), HMI_ValueStruct.Random_Zstart_scale);
	    EncoderRate.enabled = true;
        break;
	  case RANDOM_CASE_ZPOS_END:  			// zpos_end
        checkkey = Random_Zpos_End;
		HMI_ValueStruct.Random_Zend_scale = MixerCfg.Random_Zpos[ZPOS_END]*MINUNITMULT;
		DWIN_Draw_Signed_Float(font8x16, Select_Color, 3, 1, 216, MBASE(MROWS -index_random + RANDOM_CASE_ZPOS_END), HMI_ValueStruct.Random_Zend_scale);
	    EncoderRate.enabled = true;
        break;
      default: break;
    }
  }
  DWIN_UpdateLCD();
}

/* Mixer_Vtool */
void HMI_Adjust_Mixer_Vtool() {
  ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
  if (encoder_diffState != ENCODER_DIFF_NO) {
    if (Apply_Encoder(encoder_diffState, mixer.selected_vtool)) {
      checkkey = DMixer;
      EncoderRate.enabled = false;
	  DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Color_Bg_Black, 3, 216, MBASE(4), mixer.selected_vtool);
	  MixerCfg.Vtool_Bankup  = mixer.selected_vtool;
	  recovery.save(true);
	  //Draw_Print_ProgressModel();
	  updata_mixer_from_vtool();
	  DWIN_UpdateLCD();
      return;
    }
    NOLESS(mixer.selected_vtool, 0);
    NOMORE(mixer.selected_vtool, MixerCfg.occupy_vtool - 1);
    DWIN_Draw_IntValue(true, true, 0, font8x16, Select_Color, Color_Bg_Black, 3, 216, MBASE(4), mixer.selected_vtool);
	MixerCfg.Vtool_Bankup  = mixer.selected_vtool;
	//Draw_Print_ProgressModel();
	DWIN_UpdateLCD();
  }
}

/* Info */
void HMI_Info() {
  ENCODER_DiffState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;
  if (encoder_diffState == ENCODER_DIFF_ENTER) {
    #if HAS_ONESTEP_LEVELING
      checkkey = Control;
	  select_control.reset();
      index_control = MROWS;
      //select_control.set(CONTROL_CASE_INFO);
      Draw_Control_Menu();
    #else
      select_page.set(3);
      Goto_MainMenu();
    #endif
  }
  DWIN_UpdateLCD();
}

/* Tune */
void HMI_Tune() {
  ENCODER_DiffState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;

  // Avoid flicker by updating only the previous menu
  if (encoder_diffState == ENCODER_DIFF_CW) {
    if (select_tune.inc(1 + TUNE_CASE_TOTAL)) {
      if (select_tune.now > MROWS && select_tune.now > index_tune) {
        index_tune = select_tune.now;

		// Scroll up and draw a blank bottom line
        Scroll_Menu(DWIN_SCROLL_UP);
        //Draw_Menu_Icon(MROWS, ICON_Setspeed + select_tune.now - 1);
		
        if(index_tune == TUNE_CASE_MIXER) Item_Tune_Mixer(MROWS);
      }
      else {
        Move_Highlight(1, select_tune.now + MROWS - index_tune);
      }
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW) {
    if (select_tune.dec()) {
      if (select_tune.now < index_tune - MROWS) {
        index_tune--;
        Scroll_Menu(DWIN_SCROLL_DOWN);

		if (index_tune == MROWS)
          Draw_Back_First();
        else
          Draw_Menu_Line(0, ICON_Setspeed + select_tune.now - 1);

        if(index_tune == TUNE_CASE_MIXER) Item_Tune_Speed(0);
      }
      else {
        Move_Highlight(-1, select_tune.now + MROWS - index_tune);
      }
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    switch (select_tune.now) {
      case 0: { // Back
        select_print.set(0);
        Goto_PrintProcess();
      }
      break;
      case TUNE_CASE_SPEED: // Print speed
        checkkey = PrintSpeed;
        HMI_ValueStruct.print_speed = feedrate_percentage;
        DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Select_Color, 3, 216, MBASE(TUNE_CASE_SPEED + MROWS - index_tune), feedrate_percentage);
        EncoderRate.enabled = true;
        break;
      #if HAS_HOTEND
        case TUNE_CASE_TEMP: // Nozzle temp
          checkkey = ETemp;
          HMI_ValueStruct.E_Temp = thermalManager.temp_hotend[0].target;
          DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Select_Color, 3, 216, MBASE(TUNE_CASE_TEMP + MROWS - index_tune), thermalManager.temp_hotend[0].target);
          EncoderRate.enabled = true;
          break;
      #endif
      #if HAS_HEATED_BED
        case TUNE_CASE_BED: // Bed temp
          checkkey = BedTemp;
          HMI_ValueStruct.Bed_Temp = thermalManager.temp_bed.target;
          DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Select_Color, 3, 216, MBASE(TUNE_CASE_BED + MROWS - index_tune), thermalManager.temp_bed.target);
          EncoderRate.enabled = true;
          break;
      #endif
      #if HAS_FAN
        case TUNE_CASE_FAN: // Fan speed
          checkkey = FanSpeed;
          HMI_ValueStruct.Fan_speed = thermalManager.fan_speed[0];
          DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Select_Color, 3, 216, MBASE(TUNE_CASE_FAN + MROWS - index_tune), thermalManager.fan_speed[0]);
          EncoderRate.enabled = true;
          break;
      #endif
      #if HAS_ZOFFSET_ITEM
        case TUNE_CASE_ZOFF: // Z-offset
          #if EITHER(HAS_BED_PROBE, BABYSTEPPING)
            checkkey = Homeoffset;
            HMI_ValueStruct.offset_value = BABY_Z_VAR * 100;
            DWIN_Draw_Signed_Float(font8x16, Select_Color, 2, 2, 202, MBASE(TUNE_CASE_ZOFF + MROWS - index_tune), HMI_ValueStruct.offset_value);
            EncoderRate.enabled = true;
          #else
            // Apply workspace offset, making the current position 0,0,0
            queue.inject_P(PSTR("G92 X0 Y0 Z0"));
            HMI_AudioFeedback();
          #endif
        break;
      #endif

	  case TUNE_CASE_MIXER:
	  	checkkey = DMixer;
        select_mixer.reset();
        Draw_Mixer_Menu();
	  	break;
		
      default: break;
    }
  }
  DWIN_UpdateLCD();
}

#if HAS_PREHEAT

  /* PLA Preheat */
  void HMI_PLAPreheatSetting() {
    ENCODER_DiffState encoder_diffState = get_encoder_state();
    if (encoder_diffState == ENCODER_DIFF_NO) return;

    // Avoid flicker by updating only the previous menu
    if (encoder_diffState == ENCODER_DIFF_CW) {
      if (select_PLA.inc(1 + PREHEAT_CASE_TOTAL)) Move_Highlight(1, select_PLA.now);
    }
    else if (encoder_diffState == ENCODER_DIFF_CCW) {
      if (select_PLA.dec()) Move_Highlight(-1, select_PLA.now);
    }
    else if (encoder_diffState == ENCODER_DIFF_ENTER) {
      switch (select_PLA.now) {
        case 0: // Back
          checkkey = TemperatureID;
          select_temp.now = TEMP_CASE_PLA;
          HMI_ValueStruct.show_mode = -1;
          Draw_Temperature_Menu();
          break;
        #if HAS_HOTEND
          case PREHEAT_CASE_TEMP: // Nozzle temperature
            checkkey = ETemp;
            HMI_ValueStruct.E_Temp = ui.material_preset[0].hotend_temp;
            DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Select_Color, 3, 216, MBASE(PREHEAT_CASE_TEMP), ui.material_preset[0].hotend_temp);
            EncoderRate.enabled = true;
            break;
        #endif
        #if HAS_HEATED_BED
          case PREHEAT_CASE_BED: // Bed temperature
            checkkey = BedTemp;
            HMI_ValueStruct.Bed_Temp = ui.material_preset[0].bed_temp;
            DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Select_Color, 3, 216, MBASE(PREHEAT_CASE_BED), ui.material_preset[0].bed_temp);
            EncoderRate.enabled = true;
            break;
        #endif
        #if HAS_FAN
          case PREHEAT_CASE_FAN: // Fan speed
            checkkey = FanSpeed;
            HMI_ValueStruct.Fan_speed = ui.material_preset[0].fan_speed;
            DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Select_Color, 3, 216, MBASE(PREHEAT_CASE_FAN), ui.material_preset[0].fan_speed);
            EncoderRate.enabled = true;
            break;
        #endif
        #if ENABLED(EEPROM_SETTINGS)
          case 4: { // Save PLA configuration
            const bool success = settings.save();
            HMI_AudioFeedback(success);
          } break;
        #endif
        default: break;
      }
    }
    DWIN_UpdateLCD();
  }

  /* ABS Preheat */
  void HMI_ABSPreheatSetting() {
    ENCODER_DiffState encoder_diffState = get_encoder_state();
    if (encoder_diffState == ENCODER_DIFF_NO) return;

    // Avoid flicker by updating only the previous menu
    if (encoder_diffState == ENCODER_DIFF_CW) {
      if (select_ABS.inc(1 + PREHEAT_CASE_TOTAL)) Move_Highlight(1, select_ABS.now);
    }
    else if (encoder_diffState == ENCODER_DIFF_CCW) {
      if (select_ABS.dec()) Move_Highlight(-1, select_ABS.now);
    }
    else if (encoder_diffState == ENCODER_DIFF_ENTER) {
      switch (select_ABS.now) {
        case 0: // Back
          checkkey = TemperatureID;
          select_temp.now = TEMP_CASE_ABS;
          HMI_ValueStruct.show_mode = -1;
          Draw_Temperature_Menu();
          break;
        #if HAS_HOTEND
          case PREHEAT_CASE_TEMP: // Set nozzle temperature
            checkkey = ETemp;
            HMI_ValueStruct.E_Temp = ui.material_preset[1].hotend_temp;
            DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Select_Color, 3, 216, MBASE(PREHEAT_CASE_TEMP), ui.material_preset[1].hotend_temp);
            EncoderRate.enabled = true;
            break;
        #endif
        #if HAS_HEATED_BED
          case PREHEAT_CASE_BED: // Set bed temperature
            checkkey = BedTemp;
            HMI_ValueStruct.Bed_Temp = ui.material_preset[1].bed_temp;
            DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Select_Color, 3, 216, MBASE(PREHEAT_CASE_BED), ui.material_preset[1].bed_temp);
            EncoderRate.enabled = true;
            break;
        #endif
        #if HAS_FAN
          case PREHEAT_CASE_FAN: // Set fan speed
            checkkey = FanSpeed;
            HMI_ValueStruct.Fan_speed = ui.material_preset[1].fan_speed;
            DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Select_Color, 3, 216, MBASE(PREHEAT_CASE_FAN), ui.material_preset[1].fan_speed);
            EncoderRate.enabled = true;
            break;
        #endif
        #if ENABLED(EEPROM_SETTINGS)
          case PREHEAT_CASE_SAVE: { // Save ABS configuration
            const bool success = settings.save();
            HMI_AudioFeedback(success);
          } break;
        #endif
        default: break;
      }
    }
    DWIN_UpdateLCD();
  }

#endif

/* Max Speed */
void HMI_MaxSpeed() {
  ENCODER_DiffState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;

  // Avoid flicker by updating only the previous menu
  if (encoder_diffState == ENCODER_DIFF_CW) {
    if (select_speed.inc(1 + 3 + ENABLED(HAS_HOTEND))) Move_Highlight(1, select_speed.now);
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW) {
    if (select_speed.dec()) Move_Highlight(-1, select_speed.now);
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    if (WITHIN(select_speed.now, 1, 4)) {
      checkkey = MaxSpeed_value;
      HMI_flag.feedspeed_axis = AxisEnum(select_speed.now - 1);
      HMI_ValueStruct.Max_Feedspeed = planner.settings.max_feedrate_mm_s[HMI_flag.feedspeed_axis];
      DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Select_Color, 4, 210, MBASE(select_speed.now), HMI_ValueStruct.Max_Feedspeed);
      EncoderRate.enabled = true;
    }
    else { // Back
      checkkey = Motion;
      select_motion.now = MOTION_CASE_RATE;
      Draw_Motion_Menu();
    }
  }
  DWIN_UpdateLCD();
}

/* Max Acceleration */
void HMI_MaxAcceleration() {
  ENCODER_DiffState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;

  // Avoid flicker by updating only the previous menu
  if (encoder_diffState == ENCODER_DIFF_CW) {
    if (select_acc.inc(1 + 3 + ENABLED(HAS_HOTEND))) Move_Highlight(1, select_acc.now);
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW) {
    if (select_acc.dec()) Move_Highlight(-1, select_acc.now);
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    if (WITHIN(select_acc.now, 1, 4)) {
      checkkey = MaxAcceleration_value;
      HMI_flag.acc_axis = AxisEnum(select_acc.now - 1);
      HMI_ValueStruct.Max_Acceleration = planner.settings.max_acceleration_mm_per_s2[HMI_flag.acc_axis];
      DWIN_Draw_IntValue(true, true, 0, font8x16, Color_White, Select_Color, 4, 210, MBASE(select_acc.now), HMI_ValueStruct.Max_Acceleration);
      EncoderRate.enabled = true;
    }
    else { // Back
      checkkey = Motion;
      select_motion.now = MOTION_CASE_ACCEL;
      Draw_Motion_Menu();
    }
  }
  DWIN_UpdateLCD();
}

#if HAS_CLASSIC_JERK
  /* Max Jerk */
  void HMI_MaxJerk() {
    ENCODER_DiffState encoder_diffState = get_encoder_state();
    if (encoder_diffState == ENCODER_DIFF_NO) return;

    // Avoid flicker by updating only the previous menu
    if (encoder_diffState == ENCODER_DIFF_CW) {
      if (select_jerk.inc(1 + 3 + ENABLED(HAS_HOTEND))) Move_Highlight(1, select_jerk.now);
    }
    else if (encoder_diffState == ENCODER_DIFF_CCW) {
      if (select_jerk.dec()) Move_Highlight(-1, select_jerk.now);
    }
    else if (encoder_diffState == ENCODER_DIFF_ENTER) {
      if (WITHIN(select_jerk.now, 1, 4)) {
        checkkey = MaxJerk_value;
        HMI_flag.jerk_axis = AxisEnum(select_jerk.now - 1);
        HMI_ValueStruct.Max_Jerk = planner.max_jerk[HMI_flag.jerk_axis] * MINUNITMULT;
        DWIN_Draw_FloatValue(true, true, 0, font8x16, Color_White, Select_Color, 3, 1, 210, MBASE(select_jerk.now), HMI_ValueStruct.Max_Jerk);
        EncoderRate.enabled = true;
      }
      else { // Back
        checkkey = Motion;
        select_motion.now = MOTION_CASE_JERK;
        Draw_Motion_Menu();
      }
    }
    DWIN_UpdateLCD();
  }
#endif // HAS_CLASSIC_JERK

/* Step */
void HMI_Step() {
  ENCODER_DiffState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;

  // Avoid flicker by updating only the previous menu
  if (encoder_diffState == ENCODER_DIFF_CW) {
    if (select_step.inc(1 + 3 + ENABLED(HAS_HOTEND))) Move_Highlight(1, select_step.now);
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW) {
    if (select_step.dec()) Move_Highlight(-1, select_step.now);
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    if (WITHIN(select_step.now, 1, 4)) {
      checkkey = Step_value;
      HMI_flag.step_axis = AxisEnum(select_step.now - 1);
      HMI_ValueStruct.Max_Step = planner.settings.axis_steps_per_mm[HMI_flag.step_axis] * MINUNITMULT;
      DWIN_Draw_FloatValue(true, true, 0, font8x16, Color_White, Select_Color, 3, 1, 210, MBASE(select_step.now), HMI_ValueStruct.Max_Step);
      EncoderRate.enabled = true;
    }
    else { // Back
      checkkey = Motion;
      select_motion.now = MOTION_CASE_STEPS;
      Draw_Motion_Menu();
    }
  }
  DWIN_UpdateLCD();
}

void HMI_Init() {
  HMI_SDCardInit();
  for (uint16_t t = 0; t <= 100; t += 2) {
    DWIN_ICON_Show(ICON, ICON_Bar, 15, 260);
    DWIN_Draw_Rectangle(1, Color_Bg_Black, 15 + t * 242 / 100, 260, 257, 280);
    DWIN_UpdateLCD();
    delay(20);
  }
  HMI_SetLanguage();
}

void DWIN_Update() {
  EachMomentUpdate();   // Status update
  HMI_SDCardUpdate();   // SD card update
  DWIN_HandleScreen();  // Rotary encoder update
}

void Mixer_Manual_Print(){
    mixer.selected_vtool = MixerCfg.Vtool_Bankup;
	updata_mixer_from_vtool();
	MIXER_STEPPER_LOOP(i) MixerCfg.Manual_Percent[mixer.selected_vtool][i] = mixer.mix[i];	
}

void Mixer_Auto_Print(){
	float Zpos_Rate_Vaule = 0;
	
	if(current_position.z != MixerCfg.Zpos_Buff){
		MixerCfg.Zpos_Buff = current_position.z;

	    if(MixerCfg.Zpos_Buff < MixerCfg.Auto_Zpos[ZPOS_START]) mixer.selected_vtool = MixerCfg.Auto_VTool[VTOOL_START];
		else if(MixerCfg.Zpos_Buff > MixerCfg.Auto_Zpos[ZPOS_END]) mixer.selected_vtool = MixerCfg.Auto_VTool[VTOOL_END];
		else {
			if(MixerCfg.Auto_Zpos[ZPOS_START] < MixerCfg.Auto_Zpos[ZPOS_END]) {
				Zpos_Rate_Vaule = ((abs)(MixerCfg.Zpos_Buff - MixerCfg.Auto_Zpos[ZPOS_START]))/((abs)(MixerCfg.Auto_Zpos[ZPOS_END] - MixerCfg.Auto_Zpos[ZPOS_START]));
				MIXER_STEPPER_LOOP(i){
					if(MixerCfg.Start_Percent[i] >= MixerCfg.End_Percent[i])
					{
						MixerCfg.Current_Percent[i] =(int8_t)(MixerCfg.Start_Percent[i] - (MixerCfg.Start_Percent[i] - MixerCfg.End_Percent[i]) * Zpos_Rate_Vaule);
						if(MixerCfg.Current_Percent[i] <= MixerCfg.End_Percent[i]) MixerCfg.Current_Percent[i] = MixerCfg.End_Percent[i];
					}
					else
					{
						MixerCfg.Current_Percent[i] =(int8_t)(MixerCfg.Start_Percent[i] + (MixerCfg.End_Percent[i] - MixerCfg.Start_Percent[i]) * Zpos_Rate_Vaule);
						if(MixerCfg.Current_Percent[i] >= MixerCfg.End_Percent[i]) MixerCfg.Current_Percent[i] = MixerCfg.End_Percent[i];
					}
				}
				MIXER_STEPPER_LOOP(i) mixer.mix[i] = MixerCfg.Current_Percent[i];
				recalculation_mixer_percent();
				mixer.selected_vtool = MixerCfg.occupy_vtool;
				MixerCfg.Vtool_Bankup = mixer.selected_vtool;
				MIXER_STEPPER_LOOP(i) mixer.color[mixer.selected_vtool][i] = mixer.mix[i];
			}
		}
	}
}

void Mixer_Random_Print(){
	if(current_position.z != MixerCfg.Zpos_Buff){
		MixerCfg.Zpos_Buff = current_position.z;
		if((MixerCfg.Zpos_Buff <= MixerCfg.Random_Zpos[ZPOS_END])&&(MixerCfg.Zpos_Buff >= MixerCfg.Random_Zpos[ZPOS_START])){
			//MIXER_STEPPER_LOOP(i) mixer.mix[i] = random(100);
			//recalculation_mixer_percent();
			#if(MIXING_STEPPERS == 4)
			mixer.mix[0] =  GenRandomString(100);
			mixer.mix[1] =  GenRandomString(100 - mixer.mix[0]);
			mixer.mix[2] =  GenRandomString(100 - mixer.mix[0] - mixer.mix[1]);
			mixer.mix[3] =  100 - mixer.mix[0] -mixer.mix[1] - mixer.mix[2];
			#elif(MIXING_STEPPERS == 3)
			mixer.mix[0] =  GenRandomString(100);
			mixer.mix[1] =  GenRandomString(100 - mixer.mix[0]);
			mixer.mix[2] =  100 - mixer.mix[0] - mixer.mix[1];
			#elif(MIXING_STEPPERS == 2)
			mixer.mix[0] =  GenRandomString(100);
			mixer.mix[1] =  100 - mixer.mix[0];
			#endif
			mixer.selected_vtool = MixerCfg.occupy_vtool;
			MixerCfg.Vtool_Bankup = mixer.selected_vtool;
			MIXER_STEPPER_LOOP(i) mixer.color[mixer.selected_vtool][i] = mixer.mix[i];
		}
	}
}

void EachMomentUpdate() {
  static millis_t next_rts_update_ms = 0;
  const millis_t ms = millis();
  
  if (PENDING(ms, next_rts_update_ms)) 	return;
  next_rts_update_ms = ms + DWIN_SCROLL_UPDATE_INTERVAL;

  #if HAS_SUICIDE
  	if(checkkey == MainMenu){
  		if(HMI_flag.free_close_timer_rg == 0){
			queue.inject_P(PSTR("M81")); 
			HMI_flag.free_close_timer_rg = POWERDOWN_MACHINE_TIMER;
  		}
  		else {
			HMI_flag.free_close_timer_rg--;
			//if((HMI_flag.free_close_timer_rg < 2)&&(HMI_flag.free_close_timer_rg>0)) buzzer.tone(100, 700);
			if(HMI_flag.free_close_timer_rg == 0) buzzer.tone(500, 300);
  			Draw_Freedown_Machine();
  		}
  	}else HMI_flag.free_close_timer_rg = POWERDOWN_MACHINE_TIMER;
  
  	if(HMI_flag.putdown_close_machine){
		Draw_Powerdown_Machine();
  		if(HMI_flag.putdown_close_timer_rg == 0){
			HMI_flag.putdown_close_timer_rg = POWERDOWN_MACHINE_TIMER;
  			HMI_flag.putdown_close_machine = false;
			queue.inject_P(PSTR("M81")); 
  		}
		else {
			HMI_flag.putdown_close_timer_rg--;
			//if((HMI_flag.putdown_close_timer_rg < 2)&&(HMI_flag.putdown_close_timer_rg>0)) buzzer.tone(100, 700);
			if(HMI_flag.putdown_close_timer_rg == 0) buzzer.tone(500, 300);
		}
  	}
  #endif
  
  #if ENABLED(MIXING_EXTRUDER)
  	if (MixerCfg.Mixer_Mode_Rg == 0) Mixer_Manual_Print();
  	else if (MixerCfg.Mixer_Mode_Rg == 1) Mixer_Auto_Print();
  	else if (MixerCfg.Mixer_Mode_Rg == 2) Mixer_Random_Print();
  #endif
  
  // variable update
  update_variable();
  
 #if ENABLED(POWER_LOSS_RECOVERY)
  if(current_position.z != HMI_flag.current_zpos_bankup){
		HMI_flag.current_zpos_bankup = current_position.z;
		recovery.save(true);
  }
 #endif
 
  #ifdef DEBUG_POWER_LOSS
	SERIAL_ECHOLNPAIR("HMI_flag.current_zpos_bankup:\n",HMI_flag.current_zpos_bankup);
    SERIAL_ECHOLNPAIR("HMI_flag.pause_zpos_bankup:\n",HMI_flag.pause_zpos_bankup);
  #endif
  
  if (checkkey == PrintProcess) {
    // if print done
    if (HMI_flag.print_finish && !HMI_flag.done_confirm_flag) {
      HMI_flag.print_finish = false;
      HMI_flag.done_confirm_flag = true;
	
	  #if HAS_SUICIDE
	  	HMI_flag.putdown_close_machine = true;
	  	HMI_flag.putdown_close_timer_rg = POWERDOWN_MACHINE_TIMER;
	  #endif

      TERN_(POWER_LOSS_RECOVERY, recovery.cancel());

      planner.finish_and_disable();

      // show percent bar and value
      Percentrecord = 0;
      Draw_Print_ProgressBar();

      // show print done confirm
      DWIN_Draw_Rectangle(1, Color_Bg_Black, 0, 250, DWIN_WIDTH - 1, STATUS_Y_START);
      //DWIN_ICON_Show(ICON, HMI_IsChinese() ? ICON_Confirm_C : ICON_Confirm_E, 86, 283);
      DWIN_ICON_Show(ICON,ICON_Confirm_E, 86, 283);
    }
    else if (HMI_flag.pause_flag != printingIsPaused()) {
      // print status update
      //mixer.selected_vtool = MixerCfg.Vtool_Bankup;
      HMI_flag.pause_zpos_bankup = current_position.z;
      HMI_flag.pause_flag = printingIsPaused();

      if (HMI_flag.pause_flag){
	  	if(!HMI_flag.filament_runout_star) ICON_Continue();
      }
	  else {
	  	if(!HMI_flag.filament_runout_end) ICON_Pause();
		else{
			HMI_flag.filament_runout_star = 0;
			HMI_flag.filament_runout_end = 0;
			Goto_PrintProcess();
		}
	  }
    }
  }

  // pause after homing
  if (HMI_flag.pause_action && printingIsPaused() && !planner.has_blocks_queued()) {
    HMI_flag.pause_action = false;

    #if ENABLED(PAUSE_HEAT)
      #if HAS_HEATED_BED
        tempbed = thermalManager.temp_bed.target;
      #endif
      #if HAS_HOTEND
        temphot = thermalManager.temp_hotend[0].target;
      #endif
      thermalManager.disable_all_heaters();
    #endif
    queue.inject_P(PSTR("G1 F1200 X0 Y0"));
  }

  if (card.isPrinting() && checkkey == PrintProcess) { // print process
    const uint8_t card_pct = card.percentDone();
    static uint8_t last_cardpercentValue = 101;
	
    if (last_cardpercentValue != card_pct) { // print percent
      last_cardpercentValue = card_pct;
      if (card_pct) {
        Percentrecord = card_pct;
        Draw_Print_ProgressBar();
      }
    }
    duration_t elapsed = print_job_timer.duration(); // print timer

    // Print time so far
    static uint16_t last_Printtime = 0;
    const uint16_t min = (elapsed.value % 3600) / 60;
    if (last_Printtime != min) { // 1 minute update
      last_Printtime = min;
      Draw_Print_ProgressElapsed();
    }

    // Estimate remaining time every 20 seconds
    static millis_t next_remain_time_update = 0;
    if (Percentrecord > 1 && ELAPSED(ms, next_remain_time_update) && !HMI_flag.heat_flag) {
      remain_time = (elapsed.value - dwin_heat_time) / (Percentrecord * 0.01f) - (elapsed.value - dwin_heat_time);
      next_remain_time_update += 20 * 1000UL;
      Draw_Print_ProgressRemain();
    }
  }
  else if (dwin_abort_flag && !HMI_flag.home_flag) { // Print Stop
    dwin_abort_flag = false;
    HMI_ValueStruct.print_speed = feedrate_percentage = 100;
    dwin_zoffset = TERN0(HAS_BED_PROBE, probe.offset.z);
    select_page.set(0);
    Goto_MainMenu();
  }
  #if ENABLED(POWER_LOSS_RECOVERY)
    else if (DWIN_lcd_sd_status && recovery.dwin_flag) { // resume print before power off
      static bool recovery_flag = false;

	  #ifdef DEBUG_POWER_LOSS
		SERIAL_ECHOLNPGM("Power Loss Recover...");
		SERIAL_EOL();
	  #endif

      recovery.dwin_flag = false;
      recovery_flag = true;

      auto update_selection = [&](const bool sel) {
        HMI_flag.select_flag = sel;
        const uint16_t c1 = sel ? Color_Bg_Window : Select_Color;
        DWIN_Draw_Rectangle(0, c1, 25, 306, 126, 345);
        DWIN_Draw_Rectangle(0, c1, 24, 305, 127, 346);
        const uint16_t c2 = sel ? Select_Color : Color_Bg_Window;
        DWIN_Draw_Rectangle(0, c2, 145, 306, 246, 345);
        DWIN_Draw_Rectangle(0, c2, 144, 305, 247, 346);
      };

      Popup_Window_Resume();
      update_selection(true);

      // TODO: Get the name of the current file from someplace
      //
      //(void)recovery.interrupted_file_exists();
      char * const name = card.longest_filename();
      const int8_t npos = _MAX(0U, DWIN_WIDTH - strlen(name) * (MENU_CHR_W)) / 2;
      DWIN_Draw_String(false, true, font8x16, Popup_Text_Color, Color_Bg_Window, npos, 252, name);
      DWIN_UpdateLCD();

      while (recovery_flag) {
        ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
        if (encoder_diffState != ENCODER_DIFF_NO) {
          if (encoder_diffState == ENCODER_DIFF_ENTER) {
            recovery_flag = false;
            if (HMI_flag.select_flag) break;
            TERN_(POWER_LOSS_RECOVERY, queue.inject_P(PSTR("M1000C")));
            HMI_StartFrame(true);
            return;
          }
          else
            update_selection(encoder_diffState == ENCODER_DIFF_CW);
          DWIN_UpdateLCD();
        }
      }

      select_print.set(0);
      HMI_ValueStruct.show_mode = 0;
      queue.inject_P(PSTR("M1000"));
      
	  Draw_Status_Area(true);
	  Goto_PrintProcess();
	  Draw_Mixer_Status_Area(true);
    }
  #endif
  DWIN_UpdateLCD();
}

void DWIN_HandleScreen() {
  switch (checkkey) {
    case MainMenu:        HMI_MainMenu(); break;
    case SelectFile:      HMI_SelectFile(); break;
    case Prepare:         HMI_Prepare(); break;
    case Control:         HMI_Control(); break;
    case Leveling:        break;
	case Leveling0:       HMI_Leveling0(); break;
	case Home:       	  HMI_Home(); break; 
    case PrintProcess:    HMI_Printing(); break;
    case Print_window:    HMI_PauseOrStop(); break;
    case AxisMove:        HMI_AxisMove(); break;
    case TemperatureID:   HMI_Temperature(); break;
    case Motion:          HMI_Motion(); break;
	case DMixer:          HMI_Mixer(); break;
    case Info:            HMI_Info(); break;
    case Tune:            HMI_Tune(); break;
    #if HAS_PREHEAT
      case PLAPreheat:    HMI_PLAPreheatSetting(); break;
      case ABSPreheat:    HMI_ABSPreheatSetting(); break;
    #endif
    case MaxSpeed:        HMI_MaxSpeed(); break;
    case MaxAcceleration: HMI_MaxAcceleration(); break;
    case MaxJerk:         HMI_MaxJerk(); break;
    case Step:            HMI_Step(); break;
    case Move_X:          HMI_Move_X(); break;
    case Move_Y:          HMI_Move_Y(); break;
    case Move_Z:          HMI_Move_Z(); break;
    #if HAS_HOTEND
	  #if ENABLED (MIXING_EXTRUDER)
	  	#if(MIXING_STEPPERS == 4)
      		case Extruder1:      HMI_Move_E1(); break;
	  		case Extruder2:      HMI_Move_E2(); break;
	  		case Extruder3:      HMI_Move_E3(); break;
	  		case Extruder4:      HMI_Move_E4(); break;
		#elif(MIXING_STEPPERS == 3)
      		case Extruder1:      HMI_Move_E1(); break;
	  		case Extruder2:      HMI_Move_E2(); break;
	  		case Extruder3:      HMI_Move_E3(); break;
		#elif(MIXING_STEPPERS == 2)
      		case Extruder1:      HMI_Move_E1(); break;
	  		case Extruder2:      HMI_Move_E2(); break;
		#else
			case Extruder1:      HMI_Move_E1(); break;
		#endif
	  #endif
      case ETemp:         HMI_ETemp(); break;
    #endif
	
    #if EITHER(HAS_BED_PROBE, BABYSTEPPING)
      case Homeoffset:    HMI_Zoffset(); break;
    #endif

    #if HAS_HEATED_BED
      case BedTemp:       HMI_BedTemp(); break;
    #endif
    #if HAS_PREHEAT
      case FanSpeed:      HMI_FanSpeed(); break;
    #endif
    case PrintSpeed:      HMI_PrintSpeed(); break;
    case MaxSpeed_value:  HMI_MaxFeedspeedXYZE(); break;
    case MaxAcceleration_value: HMI_MaxAccelerationXYZE(); break;
    case MaxJerk_value:   HMI_MaxJerkXYZE(); break;
    case Step_value:      HMI_StepXYZE(); break;

	#if ENABLED(MIXING_EXTRUDER)
	case Mix_Manual:      HMI_Mixer_Manual(); break;
	case Mix_Auto:        HMI_Mixer_Auto(); break;
	case Mix_Random:      HMI_Mixer_Random(); break;
	case Mix_Vtool:       HMI_Adjust_Mixer_Vtool(); break;

	#if ENABLED (MIXING_EXTRUDER)
	  	#if(MIXING_STEPPERS == 4)
			case Mix_Manual_Extruder1:   HMI_Adjust_Ext_Percent(MANUAL_CASE_EXTRUDER1);break;//HMI_Adjust_Manual_Ext1_Percent(); break;
			case Mix_Manual_Extruder2:   HMI_Adjust_Ext_Percent(MANUAL_CASE_EXTRUDER2);break;//HMI_Adjust_Manual_Ext2_Percent(); break;
			case Mix_Manual_Extruder3:   HMI_Adjust_Ext_Percent(MANUAL_CASE_EXTRUDER3);break;//HMI_Adjust_Manual_Ext3_Percent(); break;
			case Mix_Manual_Extruder4:   HMI_Adjust_Ext_Percent(MANUAL_CASE_EXTRUDER4);break;//HMI_Adjust_Manual_Ext4_Percent(); break;
			
		#elif(MIXING_STEPPERS == 3)
			case Mix_Manual_Extruder1:   HMI_Adjust_Ext_Percent(MANUAL_CASE_EXTRUDER1);break;//HMI_Adjust_Manual_Ext1_Percent(); break;
			case Mix_Manual_Extruder2:   HMI_Adjust_Ext_Percent(MANUAL_CASE_EXTRUDER2);break;//HMI_Adjust_Manual_Ext2_Percent(); break;
			case Mix_Manual_Extruder3:   HMI_Adjust_Ext_Percent(MANUAL_CASE_EXTRUDER3);break;//HMI_Adjust_Manual_Ext3_Percent(); break;
		#elif(MIXING_STEPPERS == 2)
			case Mix_Manual_Extruder1:   HMI_Adjust_Ext_Percent(MANUAL_CASE_EXTRUDER1);break;//HMI_Adjust_Manual_Ext1_Percent(); break;
			case Mix_Manual_Extruder2:   HMI_Adjust_Ext_Percent(MANUAL_CASE_EXTRUDER2);break;//HMI_Adjust_Manual_Ext2_Percent(); break;
		#else
			case Mix_Manual_Extruder1:   HMI_Adjust_Ext_Percent(MANUAL_CASE_EXTRUDER1);break;//HMI_Adjust_Manual_Ext1_Percent(); break;
		#endif
    #endif
	
	case Auto_Zpos_Start:     	 		HMI_Adjust_Auto_Zpos_Start(); break;
	case Auto_Zpos_End:   	 	 		HMI_Adjust_Auto_Zpos_End(); break;
	case Mix_VTool_Start:     	 		HMI_Adjust_Auto_VTool_Start(); break;
	case Mix_VTool_End:     	 		HMI_Adjust_Auto_VTool_End(); break;

	case Random_Zpos_Start:     	 	HMI_Adjust_Random_Zpos_Start(); break;
	case Random_Zpos_End:   	 	 	HMI_Adjust_Random_Zpos_End(); break;

	case Filament_Option:				HMI_Filament_Runout_Option(); break;

	case Powerdown:						HMI_Powerdown(); break;
	case Language:						HMI_Language(); break;

	#if ENABLED(BLTOUCH)
		case Bltouch:					HMI_Option_Bltouch(); break;
	#endif
	
	#endif
	
    default: break;
  }
}

void DWIN_CompletedHoming() {
  HMI_flag.home_flag = false;
  if (checkkey == Last_Prepare) {
  	checkkey = Home;
  	//select_prepare.now = PREPARE_CASE_HOME;
    //index_home = MROWS;
    Draw_Home_Menu();
  	if(select_home.now) DWIN_ICON_Show(ICON, ICON_MoveX, 216, MBASE(select_home.now));
  	/*
    checkkey = Prepare;
    select_prepare.now = PREPARE_CASE_HOME;
    index_prepare = MROWS;
    Draw_Prepare_Menu();
    */
  }
  else if (checkkey == Back_Main) {
    HMI_ValueStruct.print_speed = feedrate_percentage = 100;
    dwin_zoffset = TERN0(HAS_BED_PROBE, probe.offset.z);
    planner.finish_and_disable();
    Goto_MainMenu();
  }
}

void DWIN_CompletedLeveling() {
  //if (checkkey == Leveling) Goto_MainMenu();
  if (checkkey == Last_Leveling) {
  	checkkey = Prepare;
    select_prepare.set(PREPARE_CASE_LEVELING);
    Draw_Prepare_Menu();
  }
}

void DRAW_Filament_Runout_Message(char message,char mode){
	switch (message){
		case DWIN_PAUSE_MESSAGE_CHANGING:
			Popup_window_Filament_Runout_Start(mode);
			break;
		case DWIN_PAUSE_MESSAGE_WAITING:
			break;
		case DWIN_PAUSE_MESSAGE_UNLOAD:
			Popup_window_Filament_Runout_Unload(mode);
			break;
		case DWIN_PAUSE_MESSAGE_INSERT:
			Popup_window_Filament_Runout_Insert(mode);
			break;
		case DWIN_PAUSE_MESSAGE_LOAD:
			Popup_window_Filament_Runout_Load(mode);
			break;
		case DWIN_PAUSE_MESSAGE_PURGE:
			Popup_window_Filament_Runout_Purge(mode);
			break;
		case DWIN_PAUSE_MESSAGE_OPTION:
			checkkey = Filament_Option;
			Popup_window_Filament_Runout_Option(mode);
			break;
		case DWIN_PAUSE_MESSAGE_HEATING:
			Popup_window_Filament_Runout_Heating(mode);
			break;
		case DWIN_PAUSE_MESSAGE_RESUME:
			Popup_window_Filament_Runout_Resume(mode);
			break;
		default:break;
	}
}

void DWIN_Pause_Show_Message(
  const DWINPauseMessage message,
  const DWINPauseMode mode/*=DWIN_PAUSE_MODE_SAME*/
  //const uint8_t extruder/*=active_extruder*/
) {
  DRAW_Filament_Runout_Message(message,mode);
}


#endif // HAS_DWIN_LCD
