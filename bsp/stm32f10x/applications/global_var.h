#ifndef __GLOBAL_VAR
#define	__GLOBAL_VAR
#include "sys_conf.h"

extern Var_Info	 g_Var_inst;

uint16 reg_map_write(uint16 reg_addr,uint16 *wr_data, uint8_t wr_cnt,uint16 User_ID);
uint16 reg_map_read(uint16 reg_addr,uint16* reg_data,uint8_t read_cnt);
uint16_t save_conf_reg(uint8_t addr_sel);
uint16_t set_load_flag(uint8_t ee_load_flag);
uint16_t sys_global_var_init(void);
uint16_t sys_local_var_init(void);
int16_t eeprom_tripple_write(uint16 reg_offset_addr,uint16 wr_data,uint16_t rd_data);

int16_t eeprom_compare_read(uint16 reg_offset_addr, uint16_t *rd_data);

uint8_t reset_runtime(uint16_t param);
uint8_t load_factory_pram(void);

#endif //__GLOBAL_VAR
