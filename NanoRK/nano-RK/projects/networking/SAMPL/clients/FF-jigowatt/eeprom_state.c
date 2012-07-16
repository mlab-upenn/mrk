#include <eeprom_state.h>
#include <stdint.h>
#include <power_vars.h>
#include <nrk.h>

static uint8_t eeprom_cache[EEPROM_LEVELS];

void energy_eeprom_read(void)
{
uint8_t max_eeprom_cnt,i,t,wrap;
energy_eeprom_cnt=0;
wrap=0;
max_eeprom_cnt=0;
for(i=0; i<EEPROM_LEVELS; i++ )
{
	eeprom_cache[i]=nrk_eeprom_read_byte(EEPROM_ENERGY_START+((uint16_t)i*EEPROM_ENERGY_SIZE));
	if(eeprom_cache[i]>max_eeprom_cnt) max_eeprom_cnt=eeprom_cache[i];
}


for(i=0; i<EEPROM_LEVELS; i++ )
{
	//t=nrk_eeprom_read_byte(EEPROM_ENERGY_START+((uint16_t)i*EEPROM_ENERGY_SIZE)); 
	t=eeprom_cache[i];
	if(t>energy_eeprom_cnt || t==0)
	{
		if(t==0 && max_eeprom_cnt>EEPROM_LEVELS) 
			{
			energy_eeprom_cnt=t;
			energy_eeprom_addr_index=i;
			wrap=1;
			}
		if(t!=0 ) {
			if(wrap==0 || t<EEPROM_LEVELS )
			  {
				energy_eeprom_cnt=t;
				energy_eeprom_addr_index=i;
			  }
			}
	}	
}

for(i=0; i<8; i++ )
	{
	t=nrk_eeprom_read_byte(EEPROM_ENERGY_START+(energy_eeprom_addr_index*EEPROM_ENERGY_SIZE)+1+(uint16_t)i); 
	cummulative_energy.byte[i]=t;
	}
for(i=0; i<8; i++ )
	{
	t=nrk_eeprom_read_byte(EEPROM_ENERGY_START+(energy_eeprom_addr_index*EEPROM_ENERGY_SIZE)+9+(uint16_t)i); 
	cummulative_energy2.byte[i]=t;
	}

tmp_energy.total=cummulative_energy.total;
tmp_energy2.total=cummulative_energy2.total;

}

void energy_eeprom_erase()
{

uint8_t t;
	nrk_int_disable();
	for(t=0; t<(EEPROM_ENERGY_SIZE*EEPROM_LEVELS); t++ )
		nrk_eeprom_write_byte((EEPROM_ENERGY_START+(uint16_t)t),0x00);
	//for(t=0; t<(EEPROM_ENERGY_SIZE*EEPROM_LEVELS); t+=EEPROM_ENERGY_SIZE)
	//	nrk_eeprom_write_byte((EEPROM_ENERGY_START+(uint16_t)t),0xff);
	nrk_eeprom_write_byte(EEPROM_MAGIC,0x42);
	// write default push thresholds at 20 watts
	nrk_eeprom_write_byte(EEPROM_PUSH_THRESHOLD_0,20);
	nrk_eeprom_write_byte(EEPROM_PUSH_THRESHOLD_1,20);
	nrk_int_enable();
  	cummulative_energy2.total=0;
  	cummulative_energy.total=0;
	energy_eeprom_addr_index=0;

}
	
void energy_eeprom_write(void)
{
uint8_t i;

energy_eeprom_addr_index++;
if(energy_eeprom_addr_index>=EEPROM_LEVELS) energy_eeprom_addr_index=0;
energy_eeprom_cnt++;

nrk_int_disable();
nrk_eeprom_write_byte(EEPROM_ENERGY_START+(energy_eeprom_addr_index*EEPROM_ENERGY_SIZE), energy_eeprom_cnt );
nrk_int_enable();

nrk_int_disable();
for(i=0; i<8; i++ )
	{
	nrk_eeprom_write_byte(EEPROM_ENERGY_START+(energy_eeprom_addr_index*EEPROM_ENERGY_SIZE)+1+(uint16_t)i, cummulative_energy.byte[i]);
	}
nrk_int_enable();

nrk_int_disable();
for(i=0; i<8; i++ )
	{
	nrk_eeprom_write_byte(EEPROM_ENERGY_START+(energy_eeprom_addr_index*EEPROM_ENERGY_SIZE)+9+(uint16_t)i, cummulative_energy2.byte[i]);
	}

nrk_int_enable();

}


