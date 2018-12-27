#include <rtthread.h>
#include <components.h>
#include "sys_conf.h"
#include "calc.h"
#include "sys_def.h" 
#include "i2c_bsp.h"
#include "local_status.h"
#include "global_var.h"
//configuration parameters


typedef enum
{
		INIT_LOAD_USR=0,
		INIT_LOAD_FACT,
		INIT_LOAD_DEBUT,
		INIT_LOAD_DEFAULT,
}init_state_em;


#define	EE_FLAG_LOAD_USR			0xdf
#define	EE_FLAG_LOAD_FACT			0x1b
#define	EE_FLAG_LOAD_DFT			0x9b
#define	EE_FLAG_LOAD_DEBUT		0xff


#define	EE_FLAG_EMPTY				0x00
#define	EE_FLAG_OK					0x01
#define	EE_FLAG_ERROR				0x02

alarm_acl_conf_st		g_alarm_acl_inst[MAX_ALARM_ACL_NUM];		//alarm check list declairation
sys_reg_st					g_sys; 																	//global parameter declairation
local_reg_st 				l_sys;																	//local status declairation
uint16_t 						test;
Var_Info	g_Var_inst;

//configuration register map declairation

const conf_reg_map_st conf_reg_map_inst[CONF_REG_MAP_NUM]=
{		// 			id			mapped registers																			 min					max					default				permission	r/w     chk_prt
		{				0,		NULL,																                     0,			  		0,					0,						0,					1,      NULL   	},
		{				1,		NULL,																                     0,			  		0,					0,						0,					1,      NULL   	},
		{				2,		NULL,														                         0,			  		0,					0,						0,					1,      NULL   	},
		{				3,		NULL,														                         0,			  		0,					0,						0,					1,      NULL   	},
		{				4,		NULL,											                               0,			  		0,					0,						0,					1,      NULL   	},
		{				5,		NULL,											                               0,			  		0,					0,						0,					1,      NULL   	},
		{				6,		NULL,											                               0,			  		0,					0,						0,					1,      NULL   	},
		{				7,		NULL,											                               0,			  		0,					0,						0,					1,      NULL   	},
		{				8,		NULL,												                             0,			  		0,					0,						0,					1,      NULL   	},
		{				9,		NULL,												                             0,			  		0,					0,						0,					1,      NULL    },
}; 



//status register map declairation
const sts_reg_map_st status_reg_map_inst[STATUS_REG_MAP_NUM]=
{		// 			id			mapped registers														default				
		{				0,			&g_sys.status.sys_info.status_reg_num,			STATUS_REG_MAP_NUM},
		{				1,			&g_sys.status.sys_info.config_reg_num,			CONF_REG_MAP_NUM},
		{				2,			&g_sys.status.sys_info.software_ver,				SOFTWARE_VER},
		{				3,			&g_sys.status.sys_info.hardware_ver,				HARDWARE_VER},
		{				4,			&g_sys.status.sys_info.serial_no[3],				SERIAL_NO_3},
		{				5,			&g_sys.status.sys_info.serial_no[2],				SERIAL_NO_2},
		{				6,			&g_sys.status.sys_info.serial_no[1],				SERIAL_NO_1},
		{				7,			&g_sys.status.sys_info.serial_no[0],				SERIAL_NO_0},
		{				8,			&g_sys.status.sys_info.man_date[1],					MAN_DATA_1},
		{				9,			&g_sys.status.sys_info.man_date[0],					MAN_DATA_0},
		{				10,			NULL,																				0},
		{				11,			NULL,																				0},
		{				12,			NULL,																				0},
		{				13,			NULL,																				0},
		{				14,			NULL,																				0},
		{				15,			NULL,																				0},
		{				16,			NULL,																				0},
		{				17,			NULL,																				0},
		{				18,			NULL,																				0},
		{				19,			NULL,																				0},
    {				20,		NULL,		                                                                0}, 
};


/**
  * @brief  get eeprom program status
  * @param  None
  * @retval 
		`EE_FLAG_OK:		configuration data valid in eeprom
		`EE_FLAG_EMPTY:	eeprom empty
  */

static init_state_em get_ee_status(void)
{
		init_state_em em_init_state;
		uint8_t ee_pflag;
		rt_thread_delay(10);											//wait for eeprom power on
		I2C_EE_BufRead(&ee_pflag,STS_EE_ADDR,1);
		switch(ee_pflag)
		{
				case(EE_FLAG_LOAD_USR):
				{
						em_init_state = INIT_LOAD_USR;
						break;
				}
			
				case(EE_FLAG_LOAD_FACT):
				{
						em_init_state = INIT_LOAD_FACT;
						break;
				}		
				case(EE_FLAG_LOAD_DFT):
				{
						em_init_state = INIT_LOAD_DEFAULT;
						break;
				}						
				default:
				{
						em_init_state = INIT_LOAD_DEBUT;
						break;
				}				
		}
		return em_init_state;
} 

/**
  * @brief 	save system configurable variables initialization
	* @param  0:load usr1 eeprom
						1:load usr2 eeprom
						2:load facotry eeprom
						3:load default eeprom
	* @retval err_cnt: mismatch read/write data count
  */

uint16_t set_load_flag(uint8_t ee_load_flag)
{
		uint8_t ee_flag;
		switch (ee_load_flag)
		{
				case (0):
				{
						ee_flag = EE_FLAG_LOAD_USR;
						break;
				}
				case (1):
				{
						ee_flag = EE_FLAG_LOAD_FACT;
						break;
				}
				case (2):
				{
						ee_flag = EE_FLAG_LOAD_DEBUT;
						break;
				}				
				default:
				{
						ee_flag = EE_FLAG_LOAD_DFT;			
						break;
				}
		}
		I2C_EE_BufWrite(&ee_flag,STS_EE_ADDR,1);
		return 1;
}


/**
  * @brief 	save system configurable variables initialization
	* @param  addr_sel:
						`0: save current configuration to usr1 eeprom address
						`1:	save current configuration to usr2 eeprom address
						`2:	save current configuration to facotry eeprom address
	* @retval err_cnt: mismatch read/write data count
  */
uint16_t save_conf_reg(uint8_t addr_sel)
{		
		uint16_t conf_reg[CONF_REG_MAP_NUM];
		uint16_t test_reg[CONF_REG_MAP_NUM];
		uint16_t i,j,err_cnt,chk_res;
		uint16_t ee_save_addr;
		uint8_t ee_flag,req;
		
		
		ee_save_addr =0;
		err_cnt = 0;
	
		switch (addr_sel)
		{
				case(0):
				{
						ee_flag = EE_FLAG_LOAD_USR;
						break;
				}
				case(1):
				{
						ee_save_addr = CONF_REG_FACT_ADDR;
						ee_flag = EE_FLAG_LOAD_FACT;
						break;
				}			
				default:
				{
						return 0xff;
				}
		}		
		
		for(i=0;i<CONF_REG_MAP_NUM;i++)																								//set configration reg with default value
		{
				conf_reg[i] = *(conf_reg_map_inst[i].reg_ptr);
		}	
		if(ee_flag == EE_FLAG_LOAD_USR)
		{
				req = 0;
				for(j=0;j<3;j++)
				{
						if(j == 0)
						{
							ee_save_addr=CONF_REG_EE1_ADDR;
						}
						else if(j == 1)
						{
							ee_save_addr=CONF_REG_EE2_ADDR;
						}
						else
						{
							ee_save_addr=CONF_REG_EE3_ADDR;
						}

						I2C_EE_BufWrite((uint8_t *)conf_reg,ee_save_addr,(CONF_REG_MAP_NUM)*2);				//save configuration data to eeprom
						rt_thread_delay(100);
						I2C_EE_BufRead((uint8_t *)test_reg,ee_save_addr,(CONF_REG_MAP_NUM)*2);
						for(i=0;i<CONF_REG_MAP_NUM;i++)
						{
								if(conf_reg[i] != test_reg[i])
								{
										err_cnt++;
								}
						}
						if(err_cnt == 0)
						{
								chk_res = checksum_u16(conf_reg,CONF_REG_MAP_NUM);													//set parameter checksum
								I2C_EE_BufWrite((uint8_t*)&chk_res,ee_save_addr+(CONF_REG_MAP_NUM*2),2);
//							
//								rt_kprintf("\nchk_res_addr = %d",ee_save_addr+(CONF_REG_MAP_NUM*2));
//								rt_kprintf("\nchk_res = %d",chk_res);
							
								I2C_EE_BufWrite(&ee_flag,STS_EE_ADDR,1);																		//set eeprom program flag
						}
						else
						{
							req++;
						}

						
				}
				if(req<2)
				{
							err_cnt = 0;
				}
				else
				{
							err_cnt = req;
				}
				
		}
		else
		{
				I2C_EE_BufWrite((uint8_t *)conf_reg,ee_save_addr,(CONF_REG_MAP_NUM)*2);				//save configuration data to eeprom
				rt_thread_delay(100);
				I2C_EE_BufRead((uint8_t *)test_reg,ee_save_addr,(CONF_REG_MAP_NUM)*2);
				for(i=0;i<CONF_REG_MAP_NUM;i++)
				{
						if(conf_reg[i] != test_reg[i])
						{
								err_cnt++;
						}
				}
				if(err_cnt == 0)
				{
						chk_res = checksum_u16(conf_reg,CONF_REG_MAP_NUM);													//set parameter checksum
						I2C_EE_BufWrite((uint8_t*)&chk_res,ee_save_addr+(CONF_REG_MAP_NUM*2),2);
						I2C_EE_BufWrite(&ee_flag,STS_EE_ADDR,1);																		//set eeprom program flag
				
				}
		}
	
		return err_cnt;
}


static uint16_t init_load_default(void)
{
		uint16_t i, ret;
		ret =1;
		for(i=0;i<CONF_REG_MAP_NUM;i++)		//initialize global variable with default values
		{
				if(conf_reg_map_inst[i].reg_ptr != NULL)
				{
						*(conf_reg_map_inst[i].reg_ptr) = conf_reg_map_inst[i].dft;
				}
		}
//		for(i=0;i<3;i++)
//		{
//				if(save_conf_reg(0) == 0)	//save configuration data into eeprom
//				{
//						break;
//				}
//				else
//				{
//						continue;
//				}
//		}
		
//				authen_init();
				ret = 1;
				g_sys.status.status_remap[0] |= 0x0001;			
		
		
		return ret;
}

 
/**
  * @brief  load system configuration data from eeprom
  * @param  None
  * @retval None
  */
  


static uint16_t conf_reg_read_ee(uint16_t addr)
{
		uint16_t reg;
		uint16_t ee_err,ret;
		reg = 0;
		ee_err = 0;
		ret = 0;
		ee_err = eeprom_compare_read(addr,&reg);

		if((conf_reg_map_inst[addr].reg_ptr != NULL)&&((reg < conf_reg_map_inst[addr].min) || (reg > conf_reg_map_inst[addr].max)))
		{
				*(conf_reg_map_inst[addr].reg_ptr) = (conf_reg_map_inst[addr].dft);	
				ret = 0;	
		}
		else
		{
				*(conf_reg_map_inst[addr].reg_ptr) = reg;
				ret = 1;
		}
		
		if((ee_err != 0)||(ret == 0))
		{
				ret = 1;
		}
		else
		{
				ret = 0;
		}
		return ret;
}


static uint16_t init_load_user_conf(void)
{		
		uint16_t i,sum,sum_reg;
		sum = 0;
	
		for(i=0;i<CONF_REG_MAP_NUM;i++)
		{			
				sum_reg = sum;
				sum += conf_reg_read_ee(i);
				if(sum != sum_reg)
				{
						rt_kprintf("Err addr:%d\n",i);
				}
		}
		
		if(sum == 0)
		{
				return(1);
		}
		else
		{
				return(0);
		}
		
}

static uint16_t init_load_factory_conf(void)
{		
		uint16_t buf_reg[CONF_REG_MAP_NUM+1];
		uint16_t i;
		uint16_t chk_res;
		uint16_t ee_load_addr;
		ee_load_addr = CONF_REG_FACT_ADDR;
		
		I2C_EE_BufRead((uint8_t *)buf_reg,ee_load_addr,(CONF_REG_MAP_NUM+1)*2);		//read eeprom data & checksum to data buffer
		rt_thread_delay(1);																												//wait for i2c opeartion comletion
		chk_res = checksum_u16(buf_reg,(CONF_REG_MAP_NUM+1));
		if(chk_res != 0)	//eeprom configuration data checksum fail
		{
				for(i=0;i<CONF_REG_MAP_NUM;i++)		//initialize global variable with default values
				{
						if(conf_reg_map_inst[i].reg_ptr != NULL)
						{
								*(conf_reg_map_inst[i].reg_ptr) = conf_reg_map_inst[i].dft;
						}
				}
				return 0;
				
		}
		else
		{
				for(i=0;i<CONF_REG_MAP_NUM;i++)
				{
						*(conf_reg_map_inst[i].reg_ptr) = buf_reg[i];
				}		
				
				return 1;
		}			
}


/**
  * @brief  initialize system status reg data
  * @param  None
  * @retval None
  */
static void init_load_status(void)
{
		uint16_t i;	
		for(i=0;i<STATUS_REG_MAP_NUM;i++)
		{
				if(status_reg_map_inst[i].reg_ptr != NULL)
				{
						*(status_reg_map_inst[i].reg_ptr) = status_reg_map_inst[i].dft;
				}
		}
		
		I2C_EE_BufRead((uint8_t *)&g_sys.status.run_time,STS_REG_EE1_ADDR,sizeof(g_sys.status.run_time));	//read legacy checksum data
}

/**
  * @brief  system configurable variables initialization
  * @param  None
  * @retval 
			1:	load default data
			2:	load eeprom data
  */
uint16_t sys_global_var_init(void)
{	
		uint16_t ret;
		init_state_em	em_init_state;

		em_init_state = get_ee_status();		//get eeprom init status
		init_load_status();
		switch(em_init_state)
		{
				case(INIT_LOAD_USR):						//load usr1 data
				{
						ret = init_load_user_conf();
						if(ret == 1)
						{
								g_sys.status.status_remap[0] |= 0x01;
								rt_kprintf("Usr conf file loaded successfully.\n");
						}
						else
						{
								g_sys.status.status_remap[0] &= 0xFFFE;
								rt_kprintf("Usr conf file loaded failed.\n");
						}
						break;
				}
				case(INIT_LOAD_FACT):						//load factory data
				{
						ret = init_load_factory_conf();
						if(ret == 1)
						{		
								save_conf_reg(0);
								set_load_flag(0);
								g_sys.status.status_remap[0] |= 0x01;
								rt_kprintf("Factory conf file loaded successfully.\n");
						}
						else
						{
								g_sys.status.status_remap[0] &= 0xFFFE;
								rt_kprintf("Factory conf file loaded failed.\n");
						}
						break;
				}		
				case(INIT_LOAD_DEBUT):												//resotre default configuration data, include reset password to default values
				{
						ret = init_load_default();
						if(ret == 1)
						{			
								g_sys.status.status_remap[0] |= 0x01;
								save_conf_reg(0);
								save_conf_reg(1);
								set_load_flag(0);
								// reset dev run time
//								reset_runtime(0xff);
								rt_kprintf("INIT_LOAD_DEBUT loaded successfully.\n");
						}
						else
						{
								g_sys.status.status_remap[0] &= 0xFFFE;
								rt_kprintf("INIT_LOAD_DEBUT loaded failed.\n");
						}
						break;
				}	
				default:												//resotre default configuration data, include reset password to default values
				{
						ret = init_load_default();
						if(ret == 1)
						{		
								g_sys.status.status_remap[0] |= 0x01;
								rt_kprintf("Default conf data load successfully.\n");
						}
						else
						{
								g_sys.status.status_remap[0] &= 0xFFFE;
								rt_kprintf("Default conf file loaded failed.\n");
						}
						break;
				}		
		}		
		//测试模式和诊断模式复位。
		g_sys.config.general.diagnose_mode_en = 0;	
		g_sys.config.general.alarm_bypass_en = 0;	
		g_sys.config.general.testing_mode_en = 0;
		
		return ret;
} 





uint16_t sys_local_var_init(void)
{
//		uint16_t i,j;
//		
//		for(i=0;i<REQ_MAX_CNT;i++)
//		{
//				for(j=0;j<REQ_MAX_LEVEL;j++)
//				{
//						l_sys.require[i][j] = 0;
//				}
//		}
//		//MAX REQ initialization
//		l_sys.require[MAX_REQ][T_REQ] = 100;
//		l_sys.require[MAX_REQ][H_REQ] = 0;
//		
//		
//		l_sys.t_fsm_state = 0;		
//
//		for(i=0;i<DO_MAX_CNT;i++)
//		{
//				l_sys.bitmap[i] = 0;
//		}		
//		
//		for(i=0;i<DO_MAX_CNT;i++)
//		{
//				l_sys.comp_timeout[i] = 0;
//		}		
//		
//		for(i=0;i<AO_MAX_CNT;i++)
//		{
//				for(j=0;j<BITMAP_MAX_CNT;j++)
//				{
//						l_sys.ao_list[i][j] = 0;
//				}
//		}		
//		
//		for(i=0;i<L_FSM_STATE_MAX_NUM;i++)
//		{
//				l_sys.l_fsm_state[i] = 0;
//		}		
//		l_sys.debug_flag = 0;
//		l_sys.debug_tiemout = DEBUG_TIMEOUT_MAX;
////		l_sys.debug_flag = 1;
////		l_sys.debug_tiemout = DEBUG_TIMEOUT_MAX;		
//		l_sys.t_fsm_signal = 0;
//		l_sys.watervalve_warmup_delay = 0;
//		l_sys.ec_fan_diff_reg = 0;
//		l_sys.ec_fan_suc_temp = 0; 
		return 1;
}


/**
  * @brief  get current system permission
  * @param  None
  * @retval current system permission level
  */
uint16_t get_sys_permission(void)
{
	return g_sys.status.general.permission_level;

}

static int16_t eeprom_singel_write(uint16 base_addr,uint16 reg_offset_addr, uint16 wr_data,uint16_t rd_data)
{
		int16_t err_no;
		uint16_t wr_data_buf;
		uint16_t cs_data,ee_rd_cheksum;
		wr_data_buf = wr_data;	

		err_no = I2C_EE_BufRead((uint8_t *)&cs_data,base_addr+(CONF_REG_MAP_NUM<<1),2);
		//计算check_sum
		ee_rd_cheksum = cs_data^rd_data^wr_data;
		// 写寄存器		
		err_no += I2C_EE_BufWrite((uint8_t *)&wr_data_buf,base_addr+(reg_offset_addr<<1),2);
		// 写校验
		err_no += I2C_EE_BufWrite((uint8_t *)&ee_rd_cheksum,base_addr+(CONF_REG_MAP_NUM<<1),2);
		
		return err_no;
}


int16_t eeprom_compare_read(uint16 reg_offset_addr, uint16_t *rd_data)
{
		uint16_t rd_buf0;
		uint16_t rd_buf1;
		uint16_t rd_buf2;
		int16_t ret;
		
		I2C_EE_BufRead((uint8_t *)&rd_buf0,CONF_REG_EE1_ADDR+(reg_offset_addr<<1),2);
		I2C_EE_BufRead((uint8_t *)&rd_buf1,CONF_REG_EE2_ADDR+(reg_offset_addr<<1),2);
		I2C_EE_BufRead((uint8_t *)&rd_buf2,CONF_REG_EE3_ADDR+(reg_offset_addr<<1),2);

		//normal situation
		if((rd_buf0 == rd_buf1)&&(rd_buf2 == rd_buf1))
		{
				if(rd_buf0 )
				*rd_data = rd_buf0;
				ret = 0;
		}
		else if((rd_buf0 == rd_buf1)||(rd_buf0 == rd_buf2)||(rd_buf1 == rd_buf2))
		{
				*rd_data = rd_buf0;
				if(rd_buf0 == rd_buf1)//buf2!= buf1
				{
						*rd_data = rd_buf0;
						eeprom_singel_write(CONF_REG_EE3_ADDR,reg_offset_addr,rd_buf0,rd_buf2);
						
				}
				else if (rd_buf0 == rd_buf2)//buf2 = buf0, buf1错
				{
						*rd_data = rd_buf2;
						eeprom_singel_write(CONF_REG_EE2_ADDR,reg_offset_addr,rd_buf2,rd_buf1);
				}
				else//(rd_buf1 == rd_buf2)
				{
						*rd_data = rd_buf1;
						eeprom_singel_write(CONF_REG_EE1_ADDR,reg_offset_addr,rd_buf1,rd_buf0);
				}
				rt_kprintf("eeprom_compare_read :reg_offset_addr_ERRO= %d \n",reg_offset_addr);	
				ret = 0;
		}	
		else//三个都错误
		{
				*rd_data = 0x7fff;
				ret = 1;
				rt_kprintf("eeprom_compare_read :reg_offset_addr_ALL_ERRO= %d \n",reg_offset_addr);
		}
		return (ret);
}

int16_t eeprom_tripple_write(uint16 reg_offset_addr,uint16 wr_data,uint16_t rd_data)
{
		int16_t err_no;
		uint16_t wr_data_buf;
		uint16_t cs_data,ee_rd_cheksum;
		wr_data_buf = wr_data;	

		err_no = eeprom_compare_read(CONF_REG_MAP_NUM, &cs_data);
		
		if(err_no == 0)
		{
				ee_rd_cheksum = cs_data^rd_data^wr_data;
		}
		else
		{
				rt_kprintf("eeprom_tripple_write : ERRO \n");
				return -1;
		}
		err_no = 0;
		
		//write data to eeprom
		err_no += I2C_EE_BufWrite((uint8_t *)&wr_data_buf,CONF_REG_EE1_ADDR+(reg_offset_addr<<1),2);
		err_no += I2C_EE_BufWrite((uint8_t *)&wr_data_buf,CONF_REG_EE2_ADDR+(reg_offset_addr<<1),2);
		err_no += I2C_EE_BufWrite((uint8_t *)&wr_data_buf,CONF_REG_EE3_ADDR+(reg_offset_addr<<1),2);			
		
		//write checksum data to eeprom
		err_no += I2C_EE_BufWrite((uint8_t *)&ee_rd_cheksum,CONF_REG_EE1_ADDR+(CONF_REG_MAP_NUM*2),2);
		err_no += I2C_EE_BufWrite((uint8_t *)&ee_rd_cheksum,CONF_REG_EE2_ADDR+(CONF_REG_MAP_NUM*2),2);
		err_no += I2C_EE_BufWrite((uint8_t *)&ee_rd_cheksum,CONF_REG_EE3_ADDR+(CONF_REG_MAP_NUM*2),2);
		
		return err_no;
}



/**
  * @brief  write register map with constraints.
  * @param  reg_addr: reg map address.
  * @param  wr_data: write data. 
	* @param  permission_flag:  
  *   This parameter can be one of the following values:
  *     @arg PERM_PRIVILEGED: write opertion can be performed dispite permission level
  *     @arg PERM_INSPECTION: write operation could only be performed when pass permission check
  * @retval 
  *   This parameter can be one of the following values:
  *     @arg 1: write operation success
  *     @arg 0: write operation fail
  */
uint16 reg_map_write(uint16 reg_addr,uint16 *wr_data, uint8_t wr_cnt,uint16 User_ID)
{
		uint16_t i;
		uint16_t err_code;
//		uint16_t sys_permission;
		uint16_t ee_wr_data,ee_rd_data;
		err_code = CPAD_ERR_NOERR;
		
		if((reg_addr+wr_cnt) > CONF_REG_MAP_NUM)	//address range check
		{
				err_code = CPAD_ERR_ADDR_OR;
			rt_kprintf("MB_SLAVE CPAD_ERR_ADDR_OR1 failed\n");
				return err_code;
		}


		for(i=0;i<wr_cnt;i++)										//permission check
		{
//			if(conf_reg_map_inst[reg_addr+i].permission > sys_permission)	
//			{
//					err_code = CPAD_ERR_PERM_OR;
//					rt_kprintf("CPAD_ERR_PERM_OR1 failed\n");
//					return err_code;	
//			}						
		}
	
		for(i=0;i<wr_cnt;i++)										//writablility check
		{
			if(conf_reg_map_inst[reg_addr+i].rw != 1)	
			{
					err_code = CPAD_ERR_WR_OR;
					rt_kprintf("CPAD_ERR_WR_OR02 failed\n");
					return err_code;	
			}						
		}		

		for(i=0;i<wr_cnt;i++)										//min_max limit check
		{
				if((*(wr_data+i)>conf_reg_map_inst[reg_addr+i].max)||(*(wr_data+i)<conf_reg_map_inst[reg_addr+i].min))		//min_max limit check
				{
						err_code = CPAD_ERR_DATA_OR;
						rt_kprintf("CPAD_ERR_WR_OR03 failed\n");
						return err_code;	
				}
				
				if(conf_reg_map_inst[reg_addr+i].chk_ptr != NULL)
				{
						if(conf_reg_map_inst[reg_addr+i].chk_ptr(*(wr_data+i))==0)
						{
								err_code = CPAD_ERR_CONFLICT_OR;
								rt_kprintf("CHK_PTR:CPAD_ERR_WR_OR failed\n");
								return err_code;	
						}
				}
		}
		
		for(i=0;i<wr_cnt;i++)										//data write
		{		
				ee_rd_data = *(conf_reg_map_inst[reg_addr+i].reg_ptr);	//buffer legacy reg data
				ee_wr_data = *(wr_data+i);															//buffer current write data
			
				*(conf_reg_map_inst[reg_addr+i].reg_ptr) = *(wr_data+i);//write data to designated register			
																										
//				I2C_EE_BufWrite((uint8_t *)&ee_wr_data,CONF_REG_EE1_ADDR+((i+reg_addr)<<1),2);//write data to eeprom
//				
//				I2C_EE_BufRead((uint8_t *)&ee_rd_cheksum,CONF_REG_EE1_ADDR+(CONF_REG_MAP_NUM*2),2);	//read legacy checksum data
//				ee_rd_cheksum = ee_rd_cheksum^ee_rd_data^ee_wr_data;
//				I2C_EE_BufWrite((uint8_t *)&ee_rd_cheksum,CONF_REG_EE1_ADDR+(CONF_REG_MAP_NUM*2),2);//write checksum data to eeprom				
				
				eeprom_tripple_write(i+reg_addr,ee_wr_data,ee_rd_data);
//				//event_Recrd
//				add_eventlog_fifo(i+reg_addr, (User_ID<<8)|g_sys.status.general.permission_level,ee_rd_data,ee_wr_data);
//			//	rt_kprintf("reg_map_write I2C_EE_BufRead ADDR \n");

		}
		return err_code;		
}
/**
  * @brief  read register map.
  * @param  reg_addr: reg map address.
	* @param  *rd_data: read data buffer ptr.
  * @retval 
  *   This parameter can be one of the following values:
  *     @arg 1: write operation success
  *     @arg 0: read operation fail
  */
uint16 reg_map_read(uint16 reg_addr,uint16* reg_data,uint8_t read_cnt)
{
		uint16_t i;
		uint16_t err_code;
		extern uint16_t   cpad_usSRegHoldBuf[100];
		err_code = CPAD_ERR_NOERR;
		if((reg_addr&0x8000) != 0)
		{
				reg_addr &= 0x7fff;
				if(reg_addr > STATUS_REG_MAP_NUM)	//address out of range
				{
						err_code = CPAD_ERR_ADDR_OR;
				}
				else
				{
						for(i=0;i<read_cnt;i++)
						{
							*(reg_data+i) = *(status_reg_map_inst[reg_addr+i].reg_ptr);//read data from designated register
						}
				}
		}
		else if((reg_addr&0x4000) != 0)
		{
				reg_addr &= 0x3fff;
				if(reg_addr > 100)	//address out of range
				{
						err_code = CPAD_ERR_ADDR_OR;
				}
				else
				{
//						for(i=0;i<read_cnt;i++)
//						{
//							*(reg_data+i) = cpad_usSRegHoldBuf[reg_addr+i];//read data from designated register
//						}
				}			
		}
		else
		{
				reg_addr = reg_addr;
				if(reg_addr > CONF_REG_MAP_NUM)	//address out of range
				{
						err_code = CPAD_ERR_ADDR_OR;
				}
				else
				{
						for(i=0;i<read_cnt;i++)
						{
								*(reg_data+i) = *(conf_reg_map_inst[reg_addr+i].reg_ptr);//read data from designated register
						}
				}		
		}	
		return err_code;
}

/**
  * @brief  show register map content.
  * @param  reg_addr: reg map address.
	* @param  *rd_data: register read count.
  * @retval none
  */
static void show_reg_map(uint16_t reg_addr, uint16_t reg_cnt)
{
		uint16_t reg_buf[32];
		uint16_t i;
		reg_map_read(reg_addr,reg_buf,reg_cnt);
		rt_kprintf("Reg map info:\n");
		for(i=0;i<reg_cnt;i++)
		{
				rt_kprintf("addr:%d;data:%x\n", (reg_addr+i)&0x3fff,reg_buf[i]);
		}
}

static uint16_t write_reg_map(uint16_t reg_addr, uint16_t data)
{
		uint16_t ret;	
		ret = reg_map_write(reg_addr,&data,1,0);
		return ret;
}

static void read_eeprom(uint16_t offset,uint16_t rd_cnt)
{
		uint8_t rd_reg[32];
		uint16_t i;
		I2C_EE_BufRead((uint8_t *)rd_reg,offset,rd_cnt);
		for(i=0;i<rd_cnt;i++)
		{
				rt_kprintf("addr:%d,data:%x\n",i+offset,rd_reg[i]);
		}
}

//uint8_t reset_runtime(uint16_t param)
//{
//		uint8_t i,req ;
//	
//		req = 1;
//		if(param == 0xff)
//		{
//				for(i=0; i < DO_MAX_CNT; i++)
//				{
//						g_sys.status.run_time[i].low = 0;
//						g_sys.status.run_time[i].high = 0;
//				}
//		}
//	
//		else
//		{
//				if( param < (DO_MAX_CNT))
//				{
//						g_sys.status.run_time[param].low = 0;
//						g_sys.status.run_time[param].high = 0;				
//				}
//				else
//				{
//					req = 0;
//				}
//		}
//		
//		if(req == 1)
//		{
//			I2C_EE_BufWrite((uint8_t *)&g_sys.status.run_time,STS_REG_EE1_ADDR,sizeof(g_sys.status.run_time));
//		}
//		
//		return(req);
//		
//}

//uint8_t load_factory_pram(void)
//{
//		uint8_t req;
//		req = 0;
//		req = init_load_factory_conf();
//		authen_init();
//		save_conf_reg(0);
//		set_load_flag(0);
//		rt_thread_delay(1000);
//		NVIC_SystemReset();
//		return(req);
//}


//void write_passward1 (uint16_t work_mode)
//{
//		
//		uint8_t pass[3]={0x12,0x34,0x56};
//		
//		write_passward(pass,work_mode,2);
//		
//		I2C_EE_BufRead((uint8_t*)&g_sys.status.sys_work_mode.work_mode,WORK_MODE_EE_ADDR,sizeof(work_mode_st));
//		
//		rt_kprintf("g_sys.status.sys_work_mode.work_mode = %d\n",g_sys.status.sys_work_mode.work_mode);	
//		rt_kprintf("g_sys.status.sys_work_mode.limit_time = %d\n",g_sys.status.sys_work_mode.limit_time);
//		rt_kprintf("g_sys.status.sys_work_mode.runing_time = %d\n",g_sys.status.sys_work_mode.runing_time);
//		rt_kprintf("g_sys.status.sys_work_mode.runing_hour = %d\n",g_sys.status.sys_work_mode.runing_hour);
//		rt_kprintf("g_sys.status.sys_work_mode.pass_word[0] = %d\n",g_sys.status.sys_work_mode.pass_word[0]);
//		rt_kprintf("g_sys.status.sys_work_mode.pass_word[1] = %d\n",g_sys.status.sys_work_mode.pass_word[1]);
//		rt_kprintf("g_sys.status.sys_work_mode.pass_word[2] = %d\n",g_sys.status.sys_work_mode.pass_word[2]);
//		
//}

//void cpad_work_mode1(uint16_t work_mode)
//{
//		
//		cpad_work_mode(work_mode,2);
//		rt_kprintf("g_sys.status.sys_work_mode.work_mode = %d\n",g_sys.status.sys_work_mode.work_mode);	
//		rt_kprintf("g_sys.status.sys_work_mode.limit_time = %d\n",g_sys.status.sys_work_mode.limit_time);
//		rt_kprintf("g_sys.status.sys_work_mode.runing_time = %d\n",g_sys.status.sys_work_mode.runing_time);
//		rt_kprintf("g_sys.status.sys_work_mode.runing_hour = %d\n",g_sys.status.sys_work_mode.runing_hour);
//		rt_kprintf("g_sys.status.sys_work_mode.pass_word[0] = %d\n",g_sys.status.sys_work_mode.pass_word[0]);
//		rt_kprintf("g_sys.status.sys_work_mode.pass_word[1] = %d\n",g_sys.status.sys_work_mode.pass_word[1]);
//		rt_kprintf("g_sys.status.sys_work_mode.pass_word[2] = %d\n",g_sys.status.sys_work_mode.pass_word[2]);
//}

//void test_fsm(void)
//{
////		I2C_EE_BufRead((uint8_t*)&g_sys.status.sys_work_mode.work_mode,WORK_MODE_EE_ADDR,sizeof(work_mode_st));
//		rt_kprintf("g_sys.status.sys_work_mode.work_mode = %d\n",g_sys.status.sys_work_mode.work_mode);	
//		rt_kprintf("g_sys.status.sys_work_mode.limit_time = %d\n",g_sys.status.sys_work_mode.limit_time);
//		rt_kprintf("g_sys.status.sys_work_mode.runing_time = %d\n",g_sys.status.sys_work_mode.runing_time);
//		rt_kprintf("g_sys.status.sys_work_mode.runing_hour = %d\n",g_sys.status.sys_work_mode.runing_hour);
//		rt_kprintf("g_sys.status.sys_work_mode.pass_word[0] = %d\n",g_sys.status.sys_work_mode.pass_word[0]);
//		rt_kprintf("g_sys.status.sys_work_mode.pass_word[1] = %d\n",g_sys.status.sys_work_mode.pass_word[1]);
//		rt_kprintf("g_sys.status.sys_work_mode.pass_word[2] = %d\n",g_sys.status.sys_work_mode.pass_word[2]);
//}

static void sys_dbg(uint16_t flag)
{
		if(flag == 1)
		{
				l_sys.debug_flag = DEBUG_ON_FLAG;
				l_sys.debug_tiemout = DEBUG_TIMEOUT_MAX;
		}
		else if(flag == 223)
		{
				l_sys.debug_flag = DEBUG_ON_FLAG;
				l_sys.debug_tiemout = DEBUG_TIMEOUT_NA;	
		}
		else
		{
				l_sys.debug_flag = DEBUG_OFF_FLAG;
				l_sys.debug_tiemout = 0;
		}
}



void eeprom_addr(void)
{
		rt_kprintf("user1 conf  start =%d ,end =%d ,size = %d\n",CONF_REG_EE1_ADDR,CONF_REG_EE2_ADDR,(CONF_REG_EE2_ADDR-CONF_REG_EE1_ADDR));
		rt_kprintf("user2 conf  start =%d ,end =%d ,size = %d\n",CONF_REG_EE2_ADDR,CONF_REG_EE3_ADDR,(CONF_REG_EE3_ADDR-CONF_REG_EE2_ADDR));
		rt_kprintf("user3 conf  start =%d ,end =%d ,size = %d\n",CONF_REG_EE3_ADDR,CONF_REG_FACT_ADDR,(CONF_REG_FACT_ADDR-CONF_REG_EE3_ADDR));
		rt_kprintf("fact  conf  start =%d ,end =%d ,size = %d\n",CONF_REG_FACT_ADDR,STS_REG_EE1_ADDR,(STS_REG_EE1_ADDR-CONF_REG_FACT_ADDR));
		rt_kprintf("status reg1 start =%d ,end =%d ,size = %d\n",STS_REG_EE1_ADDR,STS_REG_EE2_ADDR,(STS_REG_EE2_ADDR-STS_REG_EE1_ADDR));
		rt_kprintf("status reg2 start =%d ,end =%d ,size = %d\n",STS_REG_EE2_ADDR,WORK_MODE_EE_ADDR,(WORK_MODE_EE_ADDR-STS_REG_EE2_ADDR));
		rt_kprintf("work_mode   start =%d ,end =%d ,size = %d\n",WORK_MODE_EE_ADDR,AlARM_TABLE_ADDR,(AlARM_TABLE_ADDR-WORK_MODE_EE_ADDR));
//		rt_kprintf("alarm_table start =%d ,end =%d ,size = %d\n",AlARM_TABLE_ADDR, TEM_HUM_PT_ADDR ,(TEM_HUM_PT_ADDR - AlARM_TABLE_ADDR));
//		rt_kprintf("TEM_HUM_REC start =%d ,end =%d ,size = %d\n",TEM_HUM_PT_ADDR, ALARM_REC_PT_ADDR ,(ALARM_REC_PT_ADDR - TEM_HUM_PT_ADDR));
//		rt_kprintf("ALARM_REC   start =%d ,end =%d ,size = %d\n",ALARM_REC_PT_ADDR, EVENT_REC_PT_ADDR ,(EVENT_REC_PT_ADDR  - ALARM_REC_PT_ADDR));
//		rt_kprintf("EVENT_REC   start =%d ,end =%d ,size = %d\n",EVENT_REC_PT_ADDR, EE_REC_END ,(EE_REC_END  - EVENT_REC_PT_ADDR));
		
}
FINSH_FUNCTION_EXPORT(eeprom_addr, show_ee_addr_table);
FINSH_FUNCTION_EXPORT(sys_dbg, system debug switchs.);
//FINSH_FUNCTION_EXPORT(test_fsm, test_test_fasm.);
//FINSH_FUNCTION_EXPORT(write_passward1, test_test_cpad_work_mode.);
//FINSH_FUNCTION_EXPORT(cpad_work_mode1, test_test_write_passward.);
//FINSH_FUNCTION_EXPORT(reset_runtime, reset run_time eeprom & regs.);
FINSH_FUNCTION_EXPORT(show_reg_map, show registers map.);
FINSH_FUNCTION_EXPORT(write_reg_map, write data into conf registers.);
FINSH_FUNCTION_EXPORT(set_load_flag, set sys init load option.);
FINSH_FUNCTION_EXPORT(save_conf_reg, save current conf reg data.);
FINSH_FUNCTION_EXPORT(read_eeprom, read eeprom content eeprom flag.);
