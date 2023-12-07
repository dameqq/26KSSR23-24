#include <avr/io.h>
#include <util/delay.h>
#include <stdbool.h>
#define F_CPU 1000000UL
#include <avr/eeprom.h>

void spiInit(void) {
	 // /SS, MOSI, SCK jako wyj�cia
	 DDRB |= (1 << DDB2) | (1 << DDB3) | (1 << DDB5);
	 // /SS w stanie wysokim
	 PORTB |= (1<< DDB2);
	 //SPI w��czone, tryb master, podzia� przez 16
	 SPCR = _BV(SPE) | _BV(MSTR) | _BV(SPR0);
}

uint8_t spiTransfer(uint8_t data) {
	 //rozpocznij transmisj�
	 
	 SPDR = data;
	 //poczekaj na koniec
	 while(!(SPSR & _BV(SPIF)));
	 //zwr�� odebran� warto��
	 return SPDR;
}

#define PAMIEC 512
uint8_t eeprom_val[PAMIEC] __attribute__((section(".eeprom")));

int main(void)
{
    uint8_t tab_ref_first[32];
	for(int i=0;i<32;i++){
		tab_ref_first[i]=0x66;
	}
	uint8_t tab_ref_second[32];
	for(int i=0;i<32;i++){
		tab_ref_second[i]=0x22;
	}


	
	DDRC  = 0b00111000; // C5 = PWR_UP | C4 = CE | C3 = TX_EN | C2 = DR | C1 = CD | C0 = AM
	PORTC = 0b00100000; // pwr_up=1 ce=0 txen=0
	DDRD = 0b11100001;	// D7,D6,D5 jako wyj�cia
	
    
	spiInit();
	// zapisz ustawien urzadzenia nadawczego
    PORTB &= ~(1<< DDB2);				//start przesylu
	spiTransfer(0b00000000);			//zapis ustawien
	spiTransfer(0b01101100);			//CH_NO[7:0]
	spiTransfer(0b00101100);			//bit[7:6] not used, AUTO_RETRAN, RX_RED_PWR, PA_PWR[1:0], HFREQ_PLL, CH_NO[8]
	spiTransfer(0b01000100);			//bit[7] not used, TX_AFW[2:0] , bit[3] not used, RX_AFW[2:0]
	spiTransfer(0b00100000);			//bit[7:6] not used, RX_PW[5:0]
	spiTransfer(0b00100000);			//bit[7:6] not used, TX_PW[5:0]
	spiTransfer(0xE7);					//RX_ADDRESS (device identity) byte 0
	spiTransfer(0xE7);					//RX_ADDRESS (device identity) byte 1
	spiTransfer(0xE7);					//RX_ADDRESS (device identity) byte 2
	spiTransfer(0xE7);					//RX_ADDRESS (device identity) byte 3
	spiTransfer(0b11011111);			//CRC_MODE,CRC_EN, XOF[2:0], UP_CLK_EN, UP_CLK_FREQ[1:0]
	PORTB |= (1<< DDB2);				//zakonczenie przesylu
	_delay_ms(10);
	
	
	PORTB &= ~(1<< DDB2);
	eeprom_write_byte(&eeprom_val[0],spiTransfer(0b00010000)); 
	eeprom_write_byte(&eeprom_val[1],spiTransfer(1)); //36
	eeprom_write_byte(&eeprom_val[2],spiTransfer(0));//0C
	eeprom_write_byte(&eeprom_val[3],spiTransfer(1));//44
	eeprom_write_byte(&eeprom_val[4],spiTransfer(0));//20
	eeprom_write_byte(&eeprom_val[5],spiTransfer(0));//20
	eeprom_write_byte(&eeprom_val[6],spiTransfer(0));//E7
	eeprom_write_byte(&eeprom_val[7],spiTransfer(0));//E7
	eeprom_write_byte(&eeprom_val[8],spiTransfer(0));//E7
	eeprom_write_byte(&eeprom_val[9],spiTransfer(0));//E7
	eeprom_write_byte(&eeprom_val[10],spiTransfer(0));
	PORTB |= (1<< DDB2);
	_delay_ms(10);
	_delay_ms(10);
	
	

	
    
    _delay_ms(250);
	_delay_ms(250);
	_delay_ms(250);
	_delay_ms(250);
	PORTC |= (1<<DDC4); // ce=1 odbi�r
		//usatwic c0,c1,c2 jako wesjcia 
	// d5/d6/d7 jako wyjscia
	while(1){
		if(PINC & (1<<PINC2)){				//carier detect
			PORTD |= (1 << DDD5);
		}else{
			PORTD &= ~(1 << DDD5);
		}
		
		
		
		if(PINC & (1<<PINC1)){				//AM
			PORTD |= (1 << DDD6);
		}else{
			PORTD &= ~(1 << DDD6);
		}
		
		
		
		if(PINC & (1<<PINC0)){				//DR
			PORTD |= (1 << DDD7);
			PORTB &= ~(1<< DDB2);
			uint8_t tablica[32];
			eeprom_write_byte(&eeprom_val[11],spiTransfer(0b00100100));
			for(int i=0;i <32; i++){
				tablica[i]=spiTransfer(0);
			}
			PORTB |= (1<< DDB2);
			
			
			
			bool flaga_ref_one = false;
			for(int i=0; i<32; i++){
				if(!(tablica[i] == tab_ref_first[i])){
					flaga_ref_one = false;
					break;
				}
				flaga_ref_one=true;
			}
			
			bool flaga_ref_two = false;
			for(int i=0; i<32; i++){
				if(!(tablica[i] == tab_ref_second[i])){
					flaga_ref_two = false;
					break;
				}
				flaga_ref_two = true;
			}
			
			
			if(flaga_ref_one == true){
				PORTD |= (1 << DDD0);
				_delay_ms(250);
				_delay_ms(250);
				_delay_ms(250);
				_delay_ms(250);
				_delay_ms(250);
				_delay_ms(250);
				_delay_ms(250);
				_delay_ms(250);
				_delay_ms(250);
				_delay_ms(250);
				_delay_ms(250);
				_delay_ms(250);
				PORTD &= ~(1 << DDD0);
				flaga_ref_one = false;
				
			}
			
			if(flaga_ref_two == true){
				PORTD |= (1 << DDD0);
				_delay_ms(250);
				_delay_ms(250);
				PORTD &= ~(1 << DDD0);
				_delay_ms(250);
				_delay_ms(250);
				PORTD |= (1 << DDD0);
				_delay_ms(250);
				_delay_ms(250);
				PORTD &= ~(1 << DDD0);
				_delay_ms(250);
				_delay_ms(250);
				PORTD |= (1 << DDD0);
				_delay_ms(250);
				_delay_ms(250);
				PORTD &= ~(1 << DDD0);
				_delay_ms(250);
				_delay_ms(250);
				PORTD |= (1 << DDD0);
				_delay_ms(250);
				_delay_ms(250);
				PORTD &= ~(1 << DDD0);
				flaga_ref_two = false;
			}
			
			
		}else{
			PORTD &= ~(1 << DDD7);
		}
	}
	
    
} 

