#ifndef CONFIG_STORE_H
#define CONFIG_STORE_H

#include "Configuration.h"


void Config_ResetDefault();

#ifdef OutageTest

static bool RestartFlag=false;
void OutageSave();
void OutageRead();
extern float last_position[4];
extern long last_sd_position[1];
#endif


extern float Current_z_offset;
//extern float last_z_offset[1];
extern float last_z_offset;

extern unsigned char FirstBootFlag;
void SaveFirstBootFlag();
void readFirstBootFlag();

#ifndef DISABLE_M503
void Config_PrintSettings();
#else
FORCE_INLINE void Config_PrintSettings() {}
#endif

#ifdef EEPROM_SETTINGS
void Config_StoreSettings();
void Config_RetrieveSettings();
#else
FORCE_INLINE void Config_StoreSettings() {}
FORCE_INLINE void Config_RetrieveSettings() { Config_ResetDefault(); Config_PrintSettings(); }
#endif

#endif//CONFIG_STORE_H
