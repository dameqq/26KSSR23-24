#include <avr/io.h>
#define F_CPU 1000000UL
#include <util/delay.h>
#include <avr/eeprom.h>
#define PAMIEC	512



void spiInit(void) {
	 // /SS, MOSI, SCK jako wyjœcia
	 DDRB |= (1 << DDB2) | (1 << DDB3) | (1 << DDB5);
	 // /SS w stanie wysokim
	 PORTB |= (1<< DDB2);
	 //SPI w³¹czone, tryb master, podzia³ przez 16
	 SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0);
	 SPCR &= ~(1<<DORD);				//wysylanie od MSB
	 SPCR &= ~(1<<CPOL);				//tryb 0 spi
	 SPCR &= ~(1<<CPHA);

}

uint8_t spiTransfer(uint8_t data) {
	 //rozpocznij transmisjê
	 SPDR = data;
	 //poczekaj na koniec
	 while(!(SPSR & _BV(SPIF)));
	 //zwróæ odebran¹ wartoœæ
	 return SPDR;
}
uint8_t eeprom_val[PAMIEC] __attribute__((section(".eeprom")));
int main(void)
{
    
	// NRF w standby C5 = PWR C4 = CE C3 = TX_EN 
	DDRC  = 0b00111000;
	PORTC = 0b00101000;
	DDRB = 0b00000011;
	PORTB = 0b00000011;
	

	spiInit();							
	
	_delay_ms(10);
	
	
    // zapisz ustawieñ urzadzenia nadawczego
    PORTB &= ~(1<< DDB2);				//start przesylu
	spiTransfer(0b00000000);			//zapis ustawien
	spiTransfer(0b01101100);			//CH_NO[7:0]
	spiTransfer(0b00001100);			//bit[7:6] not used, AUTO_RETRAN, RX_RED_PWR, PA_PWR[1:0], HFREQ_PLL, CH_NO[8]
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
	
	// zapis przesylanej informacji
	PORTB &= ~(1<< DDB2);
	spiTransfer(0b00100000); 			//zapis wysylanych danych
	for(int i=0; i<32; i++){
		spiTransfer(0x66);
	}
	PORTB |= (1<< DDB2);	
	_delay_ms(10);
	
	
	// zapis adresu urzadzenia odbiorczego
	PORTB &= ~(1<< DDB2);
	spiTransfer(0b00100010);			//zapis adresu urzadzenia docelowego
	for(int i=0; i<32; i++){
		spiTransfer(0xE7);
	}			
	PORTB |= (1<< DDB2);	
	_delay_ms(10);
	
	
	//odczyt konfiguracji i zapis w EEPROM
	
	PORTB &= ~(1<< DDB2);
	eeprom_write_byte(&eeprom_val[0],spiTransfer(0b00010000));
	eeprom_write_byte(&eeprom_val[1],spiTransfer(1));
	eeprom_write_byte(&eeprom_val[2],spiTransfer(1));
	eeprom_write_byte(&eeprom_val[3],spiTransfer(1));
	eeprom_write_byte(&eeprom_val[4],spiTransfer(1));
	eeprom_write_byte(&eeprom_val[5],spiTransfer(1));
	eeprom_write_byte(&eeprom_val[6],spiTransfer(1));
	eeprom_write_byte(&eeprom_val[7],spiTransfer(1));
	eeprom_write_byte(&eeprom_val[8],spiTransfer(1));
	eeprom_write_byte(&eeprom_val[9],spiTransfer(1));
	eeprom_write_byte(&eeprom_val[10],spiTransfer(1));
	PORTB |= (1<< DDB2);
	_delay_ms(10);
	
	
	// odczyt  wys³ania adresu i zapisz w EEPROM

	PORTB &= ~(1<< DDB2);
	eeprom_write_byte(&eeprom_val[11],spiTransfer(0b00100011));
	eeprom_write_byte(&eeprom_val[12],spiTransfer(1));
	eeprom_write_byte(&eeprom_val[13],spiTransfer(0));
	eeprom_write_byte(&eeprom_val[14],spiTransfer(1));
	eeprom_write_byte(&eeprom_val[15],spiTransfer(0));
	PORTB |= (1<< DDB2);
	_delay_ms(10);
	
	 //zapisz najwazniejszej konfiguracji
	/*
	PORTB &= ~(1<< DDB2);
	spiTransfer(0b10001100);         //1000 pphc cccc cccc CH_NO= ccccccccc, HFREQ_PLL = h PA_PWR = pp 
	spiTransfer(0b01101100);
	PORTB |= (1<< DDB2);
	_delay_ms(10);
	
	*/
	
	while(1){
		if (!(PINB & (1<<PINB1))) {
			DDRD |= (1<<DDD7);
            PORTD |= (1<<DDD7);
			PORTB &= ~(1<< DDB2);
			spiTransfer(0b00100000); 			//zapis wysylanych danych
			for(int i=0; i<32; i++){
				spiTransfer(0x66);
			}
			PORTB |= (1<< DDB2);	
			_delay_us(10);
			
			
			PORTC |= (1<<DDC4) ; 				//inicjalizacja transmisji CE=HI
			_delay_us(20);
			PORTC &= ~(1<<DDC4) ;
			_delay_ms(250);
			_delay_ms(250);
			_delay_ms(250);
			_delay_ms(250);
			_delay_ms(250);
			_delay_ms(250);
			
			
        } else {
            PORTD &= ~(1<<DDD7);
		
        }
		
		if(!(PINB & (1<<PINB0))) {
			DDRD |= (1<<DDD6);
            PORTD |= (1<<DDD6);
			PORTB &= ~(1<< DDB2);
			spiTransfer(0b00100000); 			//zapis wysylanych danych
			for(int i=0; i<32; i++){
				spiTransfer(0x22);
			}
			PORTB |= (1<< DDB2);	
			_delay_us(10);
			
			PORTC |= (1<<DDC4) ; 				//inicjalizacja transmisji CE=HI
			_delay_us(20);
			PORTC &= ~(1<<DDC4) ;
			_delay_ms(250);
			_delay_ms(250);
			_delay_ms(250);
			_delay_ms(250);
			_delay_ms(250);
			_delay_ms(250);

		}else {
            PORTD &= ~(1<<DDD6);
		
        }
		
	}
	
	
	
	
} 