#include "Kits_Delay.h"

void Delay_us(u32 nus)
{
		u8 i;
		u32 temp;
		for(i = 0;i <4;i++)
		{
				for(temp = nus; temp != 0; temp--)
				{
						;
				}
		}
}





