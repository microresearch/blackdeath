/*

TODO:

- porting new stuff from microbd (datagens/interpreters)

- be able to move around the cell/program space/random walks

= more manipulations and use for cellhead (so cell/prog and samplespace are closer)

//TEST//

- cleanups and speedups/test all datagens

- more feedback across datagens (so control of params with datagens)

- distortion and feedback controlled by datagens

*/

#define F_CPU 16000000UL 
#define samplerate 5000 // was 12000 // try raise samplerate!

#define true 1
#define false 0

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include "head.h"
#include <avr/sleep.h>
#include <avr/wdt.h>

#define floor(x) ((int)(x))
#define CELLLEN 32
#define starcount 4
#define NSTEPS  10000
#define dead 255                                                                   

/* For DAC */
#define CYWM_SCK		PB1 // Output
#define CYWM_MISO		PB3	// Input
#define CYWM_MOSI		PB2	// Output
#define CYWM_nSS		PB0	// Output	

#define MAX_SAM 60000 

#define BV(bit) (1<<(bit)) 
#define low(port, pin) (port &= ~_BV(pin))
#define high(port, pin) (port |= _BV(pin))
#define BET(A, B, C)  (((A>=B)&&(A<=C))?1:0)    /* a between [b,c] */

unsigned char *cells, *newcells;
int spointer, mpointer, prog;
uint32_t tick, tween, place, readhead, writehead, cellhead;
volatile uint32_t maxsamp = MAX_SAM;
volatile uint32_t lowersamp = 0;
ifss ifs;
rosstype ross;
unsigned int cttt;
volatile long dtae; // ????
long olddtae; // ????
unsigned char datagen, effect, weff, wrambank,rrambank;
unsigned char *xramptr;
unsigned char *xxramptr;
volatile unsigned char knob[6] = {0, 0, 0, 0, 0, 0};
uint8_t *swap;
volatile uint8_t swapping;
uint8_t kwhich,dist;
uint8_t susceptible = 0;                                                                   uint8_t recovered = 255;                                                                   uint8_t tau = 2;                                                                         
uint8_t k = 128;                                                                           int lenny=CELLLEN*CELLLEN;
double ax[starcount];
double ay[starcount];
double az[starcount];
double vx[starcount];
double vy[starcount];
double vz[starcount];
double x[starcount];
double y[starcount];
double z[starcount];

const uint8_t  sinewave[] PROGMEM= //256 values
{
0x80,0x83,0x86,0x89,0x8c,0x8f,0x92,0x95,0x98,0x9c,0x9f,0xa2,0xa5,0xa8,0xab,0xae,
0xb0,0xb3,0xb6,0xb9,0xbc,0xbf,0xc1,0xc4,0xc7,0xc9,0xcc,0xce,0xd1,0xd3,0xd5,0xd8,
0xda,0xdc,0xde,0xe0,0xe2,0xe4,0xe6,0xe8,0xea,0xec,0xed,0xef,0xf0,0xf2,0xf3,0xf5,
0xf6,0xf7,0xf8,0xf9,0xfa,0xfb,0xfc,0xfc,0xfd,0xfe,0xfe,0xff,0xff,0xff,0xff,0xff,
0xff,0xff,0xff,0xff,0xff,0xff,0xfe,0xfe,0xfd,0xfc,0xfc,0xfb,0xfa,0xf9,0xf8,0xf7,
0xf6,0xf5,0xf3,0xf2,0xf0,0xef,0xed,0xec,0xea,0xe8,0xe6,0xe4,0xe2,0xe0,0xde,0xdc,
0xda,0xd8,0xd5,0xd3,0xd1,0xce,0xcc,0xc9,0xc7,0xc4,0xc1,0xbf,0xbc,0xb9,0xb6,0xb3,
0xb0,0xae,0xab,0xa8,0xa5,0xa2,0x9f,0x9c,0x98,0x95,0x92,0x8f,0x8c,0x89,0x86,0x83,
0x80,0x7c,0x79,0x76,0x73,0x70,0x6d,0x6a,0x67,0x63,0x60,0x5d,0x5a,0x57,0x54,0x51,
0x4f,0x4c,0x49,0x46,0x43,0x40,0x3e,0x3b,0x38,0x36,0x33,0x31,0x2e,0x2c,0x2a,0x27,
0x25,0x23,0x21,0x1f,0x1d,0x1b,0x19,0x17,0x15,0x13,0x12,0x10,0x0f,0x0d,0x0c,0x0a,
0x09,0x08,0x07,0x06,0x05,0x04,0x03,0x03,0x02,0x01,0x01,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x02,0x03,0x03,0x04,0x05,0x06,0x07,0x08,
0x09,0x0a,0x0c,0x0d,0x0f,0x10,0x12,0x13,0x15,0x17,0x19,0x1b,0x1d,0x1f,0x21,0x23,
0x25,0x27,0x2a,0x2c,0x2e,0x31,0x33,0x36,0x38,0x3b,0x3e,0x40,0x43,0x46,0x49,0x4c,
0x4f,0x51,0x54,0x57,0x5a,0x5d,0x60,0x63,0x67,0x6a,0x6d,0x70,0x73,0x76,0x79,0x7c
};

signed char insdir,dir; 

unsigned char filterk, cpu, plague, step, hardk, fhk, instruction, instructionp, IP, controls, hardware, samp, count,qqq;


int ADConvert(short Channel);

void SPI_Write(uint8_t byte)
{
  SPDR = byte;				// Send SPI byte
  while(!(SPSR & (1<<SPIF)));	// Wait for SPI transmission complete
}

void init_DAC(void)
{
  DDRB = _BV(CYWM_SCK) | _BV(CYWM_MOSI) | _BV(CYWM_nSS);
  high(PORTB, CYWM_nSS);
  SPCR = 0b01010000;
}

void write_DAC(uint8_t data)
{
  low(PORTB, CYWM_nSS);
  SPI_Write(0b00001001);
  SPI_Write(data);
  high(PORTB, CYWM_nSS);
}

SIGNAL(TIMER1_COMPA_vect) {

  unsigned int ADresult;

  unsigned char mmmmm;

  maxsamp=(uint32_t)((knob[1]+2)*233);
  lowersamp=(uint32_t)((knob[5]+1)*234); 

  if (maxsamp>MAX_SAM || maxsamp<0) maxsamp=MAX_SAM;
  if (lowersamp>MAX_SAM || lowersamp<0 || lowersamp>=maxsamp) lowersamp=1;
  tween=maxsamp-lowersamp;
    
  if (cttt>=maxsamp)     cttt=lowersamp;   
  if (cttt<lowersamp)     cttt=lowersamp;   

  //more dtae action// modifiers are knob

  // poss mods as eg. *, /, +, - <<, >> and dtae ops


  //  mmmmm=(knob[4]>>5); // 0-7
  weff=(knob[3]>>3); // 0-31
   weff=3;
    switch(weff){
      case 0:
	writehead=cttt;
	break;
      case 1:
	writehead=cttt+dtae;
	break;
      case 2:
	writehead=cttt-dtae;
	break;
      case 3:
	writehead=cttt*dtae;
	break;
      case 4:
	writehead=cttt/dtae;
	break;
      case 5:
	writehead=cttt<<dtae;
	break;
      case 6:
	writehead=cttt>>dtae;
	break;
      case 7:
	writehead=0;
	break;
      case 8:
      writehead=maxsamp-cttt; 
      break;
      case 9:
      writehead=maxsamp-cttt-dtae; 
      break;
      case 10:
      writehead=maxsamp-(cttt*dtae); 
      break;
      case 11:
      writehead=maxsamp-(cttt/dtae); 
      break;
      case 12:
      writehead=maxsamp/(cttt/dtae); 
      break;

    case 13:
      writehead=cttt*(knob[0]>>3); // speeedups
      break;
    case 14:
      writehead=cttt/(knob[0]>>3); // slowdowns
      break;
    case 15:
      writehead=cttt>>(knob[0]>>3); // also can go between 1>4
      break;
    case 16:
      writehead=cttt+knob[0];
      break;

    case 17:
      writehead=cellhead;
      break;
    case 18:
      writehead=0; // sample remains
      break;

    case 19:
      writehead=writehead<<(knob[0]>>3);
      break;
    case 20:
      writehead=writehead<<dtae;
      break;
    case 21:
      writehead=writehead<<(knob[0]>>3);
      break;
    case 22:
      writehead=writehead-dtae;
      break;
    case 23:
      writehead=cttt>>dtae;
      break;
    case 24:
      writehead=cttt>>(knob[0]>>3);
      break;
    case 25:
      writehead=writehead+dtae;
      break;
    case 26:
      writehead=writehead/dtae;
      break;

    case 27:
      writehead+=(knob[0]>>3); 
      break;
    case 28:
      writehead+=dtae; 
      break;
    case 29:
      writehead+=(knob[0]>>3)+dtae; 
      break;
    case 30:
      writehead+=(knob[0]>>3)*dtae; 
      break;
    case 31:
      writehead*=(knob[0]>>3); 
      break;
  }


        ADMUX = 0x60; // clear existing channel selection 8 BIT                

  high(ADCSRA, ADSC); 
  loop_until_bit_is_set(ADCSRA, ADIF);

  xramptr = (unsigned char *)(0x1100+lowersamp+(writehead%tween));
    *xramptr = (unsigned char) ADCH;

    effect=(knob[2]>>3); 

    // divided as to those which change the buffer

    switch(effect){ 
    case 0:
      readhead=cttt;
      break;      
    case 1:
      readhead=maxsamp-cttt; 
      break;
    case 2:
      readhead=cttt<<(knob[0]>>3); // speeedups
      break;
    case 3:
      readhead=cttt>>(knob[0]>>3); // slowdowns
      break;
    case 4:
      readhead=cttt*(knob[0]>>3); // speeedups
      break;
    case 5:
      readhead=cttt/(knob[0]>>3); // speeedups
      break;
    case 6:
      readhead=cttt+dtae;
      break;
    case 7:
      readhead=cttt-dtae;
      break;
    case 8:
      readhead=cttt/(dtae+1);
      break;
    case 9:
      readhead=cttt*dtae+1;
      break;
    case 10:
      readhead=maxsamp-(dtae<<2);
      break;
    case 11:
      readhead=readhead<<(knob[0]>>3);
      break;
    case 12:
      readhead=readhead^dtae;
      break;
    case 13:
      readhead=readhead<<dtae;
      break;
    case 14:
      readhead=readhead>>dtae;
      break;
    case 15:
      readhead=readhead+(dtae<<(knob[0]>>3)); 
      break;

      // mods

    case 16:
      xxramptr = (unsigned char *)(0x1101+cttt); // just loops // - also as delay
      *xramptr = *xxramptr;
      readhead=maxsamp-(cttt<<(knob[0]>>3));
      break;      
    case 17:
      *xramptr=(dtae%255);
      readhead= knob[0]*233;
      break;
    case 18:
      *xramptr&=dtae;
      readhead&=dtae<<2;
      break;
    case 19:
      *xramptr |=dtae;
      readhead|=dtae<<2;
      break;
    case 20:
      *xramptr*=dtae;
      readhead+=dtae<<2;
      break;
    case 21:
      *xramptr=dtae;
      readhead/=dtae<<2;
      break;
    case 22:
      *xramptr=dtae;
      readhead*=dtae<<2;
      break;

    case 24:
      xxramptr = (unsigned char *)(0x1100+(maxsamp-cttt)); // just loops backwards
      *xramptr |= *xxramptr; // also other logical opps
      readhead=cttt* *xxramptr;
      break;
    case 25:
      xxramptr = (unsigned char *)(0x1100+(maxsamp-dtae)); // just loops backwards
      *xramptr = *xxramptr; // also other logical opps
      readhead=cttt* *xxramptr;
      break;
    case 26:
      xxramptr = (unsigned char *)(0x1100+(maxsamp-cttt)); // just loops backwards
      *xramptr &= *xxramptr; // also other logical opps
      readhead=cttt/ *xxramptr;
      break;
    case 27:
      xxramptr = (unsigned char *)(0x1100+(maxsamp-cttt)); // just loops backwards
      *xramptr ^= *xxramptr; // also other logical opps
			 readhead=cttt+ (*xxramptr<<2);
      break;
    case 28:
      xxramptr = (unsigned char *)(0x1100+((cttt*(knob[0]>>3))%maxsamp));
      *xramptr = *xxramptr;
      readhead=*xxramptr<<(knob[0]>>3);
      break;
    case 29:
      *xramptr=dtae;      readhead=cttt*dtae;
      break;
    case 30:
      *xramptr=dtae<<(knob[0]>>3);       readhead=cttt-dtae;
      break;
    case 31:
      *xramptr=dtae<<(knob[0]>>3); 
      readhead=*(unsigned char *)(0x1100+(cellhead));
  }

  xramptr = (unsigned char *)(0x1100+lowersamp+(readhead%tween));

  low(PORTB, CYWM_nSS);
  SPDR = 0b00001001;				// Send SPI byte
  while(!(SPSR & (1<<SPIF)));	// Wait for SPI transmission complete

  SPDR = *xramptr ;				// Send SPI byte
  while(!(SPSR & (1<<SPIF)));	// Wait for SPI transmission complete

  high(PORTB, CYWM_nSS);

  cttt++; 
}

// instructions for plague CPUS!

unsigned char ostack[20], stack[20], omem;

/* BIOTA: two dimensional memory map */

unsigned char btdir,dcdir;

unsigned char btempty(unsigned char* cells, unsigned char IP){
  // turn around
  if (btdir==0) btdir=1;
  else if (btdir==1) btdir=0;
  else if (btdir==2) btdir=3;
  else if (btdir==3) btdir=2;
  return IP;
}

unsigned char btoutp(unsigned char* cells, unsigned char IP){
  dtae=cells[omem];
  return IP;
}

unsigned char btstraight(unsigned char* cells, unsigned char IP){
  if (dcdir==0) omem+=1;
  else if (dcdir==1) omem-=1;
  else if (dcdir==2) omem+=16;
  else if (dcdir==3) omem-=16;

  if (cells[omem]==0) 
    { // change dir
  if (btdir==0) btdir=1;
  else if (btdir==1) btdir=0;
  else if (btdir==2) btdir=3;
  else if (btdir==3) btdir=2;
    }
}

unsigned char btbackup(unsigned char* cells, unsigned char IP){
  if (dcdir==0) omem-=1;
  else if (dcdir==1) omem+=1;
  else if (dcdir==2) omem-=16;
  else if (dcdir==3) omem+=16;
  if (cells[omem]==0) 
    {
  if (btdir==0) btdir=1;
  else if (btdir==1) btdir=0;
  else if (btdir==2) btdir=3;
  else if (btdir==3) btdir=2;
    }
  return IP;
}

unsigned char btturn(unsigned char* cells, unsigned char IP){
  if (dcdir==0) omem+=16;
  else if (dcdir==1) omem-=16;
  else if (dcdir==2) omem+=1;
  else if (dcdir==3) omem-=1;
  return IP;
}

unsigned char btunturn(unsigned char* cells, unsigned char IP){
  if (dcdir==0) omem-=16;
  else if (dcdir==1) omem+=16;
  else if (dcdir==2) omem-=1;
  else if (dcdir==3) omem+=1;
  return IP;
}

unsigned char btg(unsigned char* cells, unsigned char IP){
  unsigned char x;
  while (x<20 && cells[omem]!=0){
    if (dcdir==0) omem+=1;
    else if (dcdir==1) omem-=1;
    else if (dcdir==2) omem+=16;
    else if (dcdir==3) omem-=16;
  }
  return IP;
}

unsigned char btclear(unsigned char* cells, unsigned char IP){
  if (cells[omem]==0){
  if (btdir==0) btdir=1;
  else if (btdir==1) btdir=0;
  else if (btdir==2) btdir=3;
  else if (btdir==3) btdir=2;
  }
  else cells[omem]=0;
  return IP;
}

unsigned char btdup(unsigned char* cells, unsigned char IP){
  if (cells[omem]==0 || cells[omem-1]!=0){
  if (btdir==0) btdir=1;
  else if (btdir==1) btdir=0;
  else if (btdir==2) btdir=3;
  else if (btdir==3) btdir=2;
  }
  else cells[omem-1]=cells[omem];
  return IP;
}

unsigned char clock, count;

// reddeath

//1- the plague within (12 midnight) - all the cells infect

unsigned char redplague(unsigned char* cells, unsigned char IP){
  if (clock==12){
    clock=12;
    cells[IP+1]=cells[IP];
    if (IP==255) clock=13;
    return IP+1;
  }
  else return IP+insdir;
}

//2- death - one by one fall dead
unsigned char reddeath(unsigned char* cells, unsigned char IP){
  if (clock==13){
    clock=13;
    cells[IP+count]=((unsigned char *)(0x1100+cttt));
    count++;
    return IP; // just keeps on going
  }
  else return IP+insdir;
}

//3- clock every hour - instruction counter or IP -some kind of TICK
unsigned char redclock(unsigned char* cells, unsigned char IP){
  clock++;
  if (clock%60==0) {
    dtae^=255;
    return IP; // everyone stops
  }
  else return IP+insdir;
}

//4- seven rooms: divide cellspace into 7 - 7 layers with filter each
unsigned char redrooms(unsigned char* cells, unsigned char IP){
  /* need to change

  switch(IP%7){
  case 0:
    //blue
      sbi(DDRB,PB1);
      TCCR1B=  (1<<WGM12) |(1<<CS10); // no divider
      filterk=8;
    break;
  case 1:
    //purple
      sbi(DDRB,PB1);
      TCCR1B=  (1<<WGM12) |(1<<CS10); // no divider
    break;
  case 2:
    //green
      sbi(DDRB,PB1);
      TCCR1B=  (1<<WGM12) |(1<<CS11); // divide by 8
      filterk=8;
    break;
  case 3:
    //orange
      sbi(DDRB,PB1);
      TCCR1B=  (1<<WGM12) |(1<<CS11); // divide by 8
    break;
  case 4:
    //white
      sbi(DDRB,PB1);
      TCCR1B=  (1<<WGM12) |(1<<CS11)|(1<<CS10); // divide by 64
    break;
  case 5:
    //violet
      sbi(DDRB,PB1);
      TCCR1B=  (1<<WGM12) |(1<<CS12); //256
    break;
  case 6:
    // black
    cbi(DDRB,PB1); //filter off
    }*/
  return IP+insdir;
}

  //5- unmasking (change neighbouring cells)

unsigned char redunmask(unsigned char* cells, unsigned char IP){
  cells[IP-1]^=255;
  cells[IP+1]^=255;
return IP+insdir;
}
  //6- the prince (omem) - the output! walking through 7 rooms 

unsigned char redprospero(unsigned char* cells, unsigned char IP){

  unsigned char dirrr;
  // prince/omem moves at random through rooms
  dirrr=*((unsigned char *)(0x1100+cttt))%4;
  if (dirrr==0) omem=omem+1;
  else if (dirrr==1) omem=omem-1;
  else if (dirrr==2) omem=omem+16;
  else if (dirrr==3) omem=omem-16;

  // output
  dtae=cells[omem]; 
  return IP+insdir;
}

  //7- the outside - the input!
unsigned char redoutside(unsigned char* cells, unsigned char IP){

  // input sample to cell (which one neighbour to omem)
  cells[omem+1]=*((unsigned char *)(0x1100+cttt));

  // output to filter 
  //  (*filtermod[qqq]) ((int)cells[omem]);
  return IP+insdir;
}

// plague

unsigned char ploutp(unsigned char* cells, unsigned char IP){
  dtae=cells[IP+1]+cells[IP-1];
  return IP+insdir;
}

unsigned char plenclose(unsigned char* cells, unsigned char IP){
  cells[IP]=255; cells[IP+1]=255;
  return IP+2;
}

unsigned char plinfect(unsigned char* cells, unsigned char IP){

  if (cells[IP]<128) {
    cells[IP+1]= cells[IP];
    cells[IP-1]= cells[IP];
  }
  return IP+insdir;
}

unsigned char pldie(unsigned char* cells, unsigned char IP){
  cells[IP-1]=0; cells[IP+1]=0;
  return IP+insdir;
}

unsigned char plwalk(unsigned char* cells, unsigned char IP){
  // changing direction
  if (dir<0 && (cells[IP]%0x03)==1) dir=1;
  else if (dir>1 && (cells[IP]%0x03)==0) dir=-1;
  else
    // changing pace
    insdir=(int)dir*cells[IP]>>4;
  
  return IP+insdir;
}

// redcode

unsigned char rdmov(unsigned char* cells, unsigned char IP){
  cells[(IP+cells[IP+2])]=cells[(IP+cells[IP+1])];
  return IP+=3;
}

unsigned char rdadd(unsigned char* cells, unsigned char IP){
  cells[(IP+cells[IP+2])]=cells[(IP+cells[IP+2])]+cells[(IP+cells[IP+1])];
  return IP+=3;
}

unsigned char rdsub(unsigned char* cells, unsigned char IP){
  cells[(IP+cells[IP+2])]=cells[(IP+cells[IP+2])]-cells[(IP+cells[IP+1])];
  return IP+=3;
}

unsigned char rdjmp(unsigned char* cells, unsigned char IP){
  IP=IP+cells[IP+1];
  return IP;
}

unsigned char rdjmz(unsigned char* cells, unsigned char IP){
  if (cells[(IP+cells[IP+2])]==0) IP=cells[IP+1];
  else IP+=3;
  return IP;
}

unsigned char rdjmg(unsigned char* cells, unsigned char IP){
  if (cells[(IP+cells[IP+2])]>=0) IP=cells[IP+1];
  else IP+=3;
  return IP;
}

unsigned char rddjz(unsigned char* cells, unsigned char IP){
  unsigned char x;
  x=(IP+cells[IP+2]);
  cells[x]=cells[x]-1;
  if (cells[x]==0) IP=cells[IP+1];
  else IP+=3;
  return IP;
}

unsigned char rddat(unsigned char* cells, unsigned char IP){
  IP+=3;
  return IP;
}

unsigned char rdcmp(unsigned char* cells, unsigned char IP){
  if (cells[(IP+cells[IP+2])]!=cells[(IP+cells[IP+1])]) IP+=6;
  else IP+=3;
  return IP;
}

unsigned char rdoutp(unsigned char* cells, unsigned char IP){
  dtae=cells[(IP+2)]; 
  IP+=3;
  return IP;
}

// SIR: inc if , die if, recover if, getinfected if 

unsigned char SIRoutp(unsigned char* cells, unsigned char IP){
  dtae=cells[(IP+1)]+cells[IP-1];
  return IP+insdir;
}

unsigned char SIRincif(unsigned char* cells, unsigned char IP){
  if ((cells[(IP+1)]>0 && cells[(IP+1)]<128)) cells[IP]++;
  return IP+insdir;
}

unsigned char SIRdieif(unsigned char* cells, unsigned char IP){
  
  if ((cells[(IP+1)]>0 && cells[(IP+1)]<128)) {
    if (rand()%10 < 4) cells[IP] = dead;       
  }
  return IP+insdir;
}

unsigned char SIRrecif(unsigned char* cells, unsigned char IP){
  if (cells[(IP+1)] >= 128) cells[IP] = recovered;                                             
  return IP+insdir;
}

unsigned char SIRinfif(unsigned char* cells, unsigned char IP){

  if (cells[(IP+1)] == 0) {   
                                                 
    if ((cells[IP-1]>0 && cells[IP-1]<128) ||
	(cells[(IP+1)]>0 && cells[(IP+1)]<128))
	{
	if (rand()%10 < 4) cells[IP] = 1;       
	}
}
  return IP+insdir;
}

// brainfuck

unsigned char cycle;

unsigned char bfinc(unsigned char* cells, unsigned char IP){
  omem++; 
  return IP++;
}

unsigned char bfdec(unsigned char* cells, unsigned char IP){
  omem--; 
  return IP++;
}

unsigned char bfincm(unsigned char* cells, unsigned char IP){
  cells[omem]++; 
  return IP++;
}

unsigned char bfdecm(unsigned char* cells, unsigned char IP){
  cells[omem]--; 
  return IP++;
}

unsigned char bfoutp(unsigned char* cells, unsigned char IP){
  dtae=cells[omem]; 
  return IP++;
}

unsigned char bfin(unsigned char* cells, unsigned char IP){
  cells[omem] = *((unsigned char *)(0x1100+cttt));
  return IP++;
}

unsigned char bfbrac1(unsigned char* cells, unsigned char IP){
  cycle++; 
  if(cycle>=20) cycle=0; 
  ostack[cycle] = IP; 
  return IP++;
}

unsigned char bfbrac2(unsigned char* cells, unsigned char IP){
  int i;
  if(cells[omem] != 0) i = ostack[cycle]-1; 
  cycle--; 
  if(cycle<-1) cycle=20;
  return i;
}

// first attempt

unsigned char finc(unsigned char* cells, unsigned char IP){
  omem++; 
  return IP+insdir;
}

unsigned char fin1(unsigned char* cells, unsigned char IP){
  omem=*((unsigned char *)(0x1100+cttt));
  return IP+insdir;
}

unsigned char fdec(unsigned char* cells, unsigned char IP){
  omem--; 
  return IP+insdir;
}

unsigned char fincm(unsigned char* cells, unsigned char IP){
  cells[omem]++; 
  return IP+insdir;
}

unsigned char fdecm(unsigned char* cells, unsigned char IP){
  cells[omem]--; 
  return IP+insdir;
}

unsigned char outp(unsigned char* cells, unsigned char IP){
  dtae=cells[omem];
  return IP+insdir;
}

unsigned char outpp(unsigned char* cells, unsigned char IP){
  dtae=omem;
  return IP+insdir;
}

unsigned char plus(unsigned char* cells, unsigned char IP){
  cells[IP]+=1;
  return IP+insdir;
}

unsigned char minus(unsigned char* cells, unsigned char IP){
  cells[IP]-=1;
  return IP+insdir;
}

unsigned char bitshift1(unsigned char* cells, unsigned char IP){
  cells[IP]=cells[IP]<<1;
  return IP+insdir;
}

unsigned char bitshift2(unsigned char* cells, unsigned char IP){
  cells[IP]=cells[IP]<<2;
  return IP+insdir;
}

unsigned char bitshift3(unsigned char* cells, unsigned char IP){
  cells[IP]=cells[IP]<<3;
  return IP+insdir;
}

unsigned char branch(unsigned char* cells, unsigned char IP){
  if (cells[IP+1]==0) IP=cells[omem];
  return IP+insdir;
}

unsigned char jump(unsigned char* cells, unsigned char IP){
  if (cells[(IP+1)]<128) return IP+cells[(IP+1)];
  else return IP+insdir;
}

unsigned char infect(unsigned char* cells, unsigned char IP){
  int x=IP-1;
  if (x<0) x=MAX_SAM;
  if (cells[x]<128) cells[(IP+1)]= cells[IP];
  return IP+insdir;
}

unsigned char store(unsigned char* cells, unsigned char IP){
  cells[IP]=cells[cells[IP+1]];
  return IP+insdir;
}

unsigned char skip(unsigned char* cells, unsigned char IP){
  return IP+insdir;
}

unsigned char direction(unsigned char* cells, unsigned char IP){
  if (dir<0) dir=1;
  else dir=-1;
  return IP+insdir;
}

unsigned char die(unsigned char* cells, unsigned char IP){
  return IP+insdir;
}

unsigned char writesamp(unsigned char* cells, unsigned char IP){
  cells[IP]=*((unsigned char *)(0x1100+cttt));
  return IP+insdir;
}

/// back to older 

void ioinit(void)
{
  DDRD=0xFC; // all output except pd0/pd1 as switches IN
  PORTD= 0x03; // all pullups
  DDRE=0xE4; // 0x01 for PE2 as switch out + PE7/6/5 for ram bank
  PORTE=0xFF;
}

void adc_init(void)
{
  //ADCSRA |= (1 << ADPS1) | (1<< ADPS0) ; // seems now to hum = /8
  ADCSRA = (1 << ADPS2) | (1<< ADPS0) ; //= 32 - very clean
  ADMUX |= (1 << REFS0); // Set ADC reference to AVCC
  ADCSRA |= (1 << ADEN) ;
  DDRF = 0x00; // c0 for top two/three??? as OUTPUT
  PORTF = 0x00; 
}

void enable_external_ram (void) __attribute__ ((naked)) __attribute__ ((section (".init1")));

void enable_external_ram(void)
{
  MCUCR |= _BV(SRE); 
}

void initross(rosstype* ross){
  ross->h = 0.01;
  ross->a = 0.2;
  ross->b = 0.2;
  ross->c = 5.7;
  ross->lx0 = 0.1;
  ross->ly0 = 0;
  ross->lz0 = 0;
  ross->step=1;
}

void runross(rosstype* ross){
  double lx0,ly0,lz0,lx1,ly1,lz1;
  double h,a,b,c;
  int x;

  h = ross->h;
  a = ross->a;
  b = ross->b;
  c = ross->c;
  lx0 = ross->lx0;
  ly0 = ross->ly0;
  lz0 = ross->lz0;
  lx1 = lx0 + h * (-ly0 - lz0);
  ly1 = ly0 + h * (lx0 + (a * ly0));
  lz1 = lz0 + h * (b + (lx0*lz0) - (c * lz0));;
  ross->lx0 = lx1;
  ross->ly0 = ly1;
  ross->lz0 = lz1;
  //  ross->intx=(signed int)((float)lx1*1024)%1024;
  //  ross->inty=(signed int)((float)ly1*1024)%1024;
  ross->intz=(signed int)((float)lz1*1024)%1024;
}

void runbrain(void){
  unsigned char *xxx; int kl=0;

  // should also be able to move these around

  spointer++; 
  if (spointer==0) spointer=MAX_SAM;
  if (mpointer==0) mpointer=MAX_SAM;
  if (spointer>MAX_SAM) spointer=0;
  if (mpointer>MAX_SAM) mpointer=0;

  xxx=((unsigned char *)(0x1100+spointer));
  prog=*xxx%8;
  switch(prog) {
    /* Increment pointer value */
  case 0:
    xxx=((unsigned char *)(0x1100+mpointer));
    *xxx=*xxx+1;
    break;
    /* Decrement pointer value */
  case 1:
    xxx=((unsigned char *)(0x1100+mpointer));
    *xxx=*xxx-1;
    break;
    /* Increment pointer */
  case 2:
    mpointer++;
    break;
    /* Decrement pointer */
  case 3:
    mpointer--;
    break;
    /* Print current pointer value */
  case 4:
    xxx=((unsigned char *)(0x1100+mpointer));
    dtae=*xxx;
    break;
    /* Read value and store in current pointer */
  case 5:
    xxx=((unsigned char *)(0x1100+mpointer));
    *xxx=dtae;
    break;
    /* Start loop */
  case 6:
    xxx=((unsigned char *)(0x1100+mpointer));
    if (*xxx == 0) {
      /* Find matching ] */
      spointer++;
      if (spointer>MAX_SAM) spointer=0;

      /* If kl == 0 and space[pointer] == ']' we found
       * the matching brace */

      xxx=((unsigned char *)(0x1100+spointer));
      while (kl > 0 || (*xxx)%8 != 7)
	{
	  if ((*xxx)%8 == 6) kl++;
	  if ((*xxx)%8 == 7) kl--;
	  /* Go in right direction */
	  spointer++;
	  if (spointer>MAX_SAM) spointer=0;
	  xxx=((unsigned char *)(0x1100+spointer));
	}
    }
    break;
    /* End loop */
  case 7:
    xxx=((unsigned char *)(0x1100+mpointer));
    if (*xxx != 0) { // a fix
      /* Find matching [ */
      spointer--;
      if (spointer==0) spointer=MAX_SAM;
      xxx=((unsigned char *)(0x1100+spointer));

      while (kl > 0 || (*xxx)%8 != 6) {
	if ((*xxx)%8 == 6) kl--;
	if ((*xxx)%8 == 7) kl++;
	/* Go left */
	spointer--;
	if (spointer==0) spointer=MAX_SAM;
	xxx=((unsigned char *)(0x1100+spointer));
      }
      spointer--;
      if (spointer==0) spointer=MAX_SAM;
    }
    break;
  }
}

void initifs(ifss* ifs){
  int i,iter;
  int column = 6, row = 4;
  ifs->step=1;         
  ifs->ifscount=0;
  ifs->prob[0]=0.0;
  ifs->prob[1]=0.85; 
  ifs->prob[2]=0.92; 
  ifs->prob[3]=0.99; 
  ifs->prob[4]=1.0; 
  ifs->p1.x=0.1;
  ifs->p1.y=0.1;         
                         
  for (iter=0;iter<row;iter++){
    for (i=0;i<column;i++){
      iter=rand()%row;
      i=rand()%column;
      ifs->coeff[iter][i]=((double)rand()/(double)(RAND_MAX));
      if (((double)rand()/(double)(RAND_MAX))>0.5) ifs->coeff[iter][i]= ifs->coeff[iter][i]-1;
      ifs->prob[iter]=((double)rand()/(double)(RAND_MAX));
    }
  }
}

void runifs(ifss* ifs){
  double random_num;
  int iter,i,it;
  int column = 6, row = 4;

  ifs->ifscount++;
  random_num = (double)rand()/(double)(RAND_MAX);
  for(i = 0; i < row; i++){
    if ( BET(random_num,ifs->prob[i],ifs->prob[i+1]) ){
      ifs->p2.x = ifs->coeff[i][0]*ifs->p1.x + ifs->coeff[i][1]*ifs->p1.y + ifs->coeff[i][4];
      ifs->p2.y = ifs->coeff[i][2]*ifs->p1.x + ifs->coeff[i][3]*ifs->p1.y + ifs->coeff[i][5];
      break;
    }
  }
  ifs->p1=ifs->p2;  
					
  if (ifs->p2.x>0.0)
    ifs->returnvalx=(int)((ifs->p2.x)*1024);
  if (ifs->p2.y>0.0)
    ifs->returnvaly=(int)((ifs->p2.y)*1024);

  if (ifs->ifscount>NSTEPS) {
    iter=rand()%row;
    i=rand()%column;
    ifs->coeff[iter][i]=((double)rand()/(double)(RAND_MAX));
    if (((double)rand()/(double)(RAND_MAX))>0.5) ifs->coeff[iter][i]= ifs->coeff[iter][i]-1;
    ifs->prob[iter]=((double)rand()/(double)(RAND_MAX));
    ifs->p1.x=0.5;
    ifs->p1.y=0.5;
  }
}

unsigned char *table;

// second automata based on dewdney

void inittable(unsigned char r, unsigned char k, int rule){
  int max, z, summ;

  free(table);
  max = (k-1)*((r*2)+1);
  table= (unsigned char *)malloc(max+1);
  for (z=max;z>=0;z--){
    summ=0;
    while ((rule-pow(k,z))>=0) {
      summ++;
      rule=rule-pow(k,z);
    }
    if (summ>=1) {
      table[z]=summ;
    }
    else table[z]=0;
  }
}

int runcell1d(unsigned char *cells,unsigned char radius, unsigned char k){

  static unsigned char l=0; unsigned char cell; int sum,ssum,z,zz;

  l++; ssum=0;
  l%=CELLLEN;
  for (cell = 0; cell < CELLLEN; cell++){ 
    sum=0;
    // sum of cells in radius 
    for (z=-radius;z<radius;z++){
      zz=cell+z;
      if (zz>=CELLLEN) zz=zz-CELLLEN;
      if (zz<0) zz=CELLLEN+zz;
      sum+=cells[zz+(l*CELLLEN)]%k;
    }
    cells[cell+(((l+1)%CELLLEN)*CELLLEN)]= table[sum]; 
    ssum+=sum;
  }
  return ssum;
}

void initcell(unsigned char *cells, uint8_t param, int redhead, uint8_t topval){
  int y;
  unsigned char *ptr;
  
  for (y=0;y<lenny;y++){
    //    cells[y] = malloc(sizeof(uint8_t *) * CELLLEN);
    //    newcells[y] = malloc(sizeof(uint8_t *) * CELLLEN);
    if (param==2){
      ptr = (unsigned char *)(0x1100+(redhead+y)%MAX_SAM); // ??? go over limit?
      cells[y]=*ptr;
    }
	
    else if (param==1) cells[y]=rand()%topval; //*ptr;  
    else if (param==3) cells[y]=rand()%(topval-2) + 2;            
    else if (param==0) cells[y]=0;
  }
}

int runhodge(unsigned char *acells, unsigned char *anewcells, uint8_t q,uint8_t k1,uint8_t k2, uint8_t g){

  long sum=0, numill=0, numinf=0;
  static int x=CELLLEN+1;

  //  for (x=CELLLEN+1;x<(lenny-CELLLEN-1);x++){

  sum=acells[x]+acells[x-1]+acells[x+1]+acells[x-CELLLEN]+acells[x+CELLLEN]+acells[x-CELLLEN-1]+acells[x-CELLLEN+1]+acells[x+CELLLEN-1]+acells[x+CELLLEN+1];

  if (acells[x-1]==(q-1)) numill++; else if (acells[x-1]>0) numinf++;
  if (acells[x+1]==(q-1)) numill++; else if (acells[x+1]>0) numinf++;
  if (acells[x-CELLLEN]==(q-1)) numill++; else if (acells[x-CELLLEN]>0) numinf++;
  if (acells[x+CELLLEN]==(q-1)) numill++; else if (acells[x+CELLLEN]>0) numinf++;
  if (acells[x-CELLLEN-1]==q) numill++; else if (acells[x-CELLLEN-1]>0) numinf++;
  if (acells[x-CELLLEN+1]==q) numill++; else if (acells[x-CELLLEN+1]>0) numinf++;
  if (acells[x+CELLLEN-1]==q) numill++; else if (acells[x+CELLLEN-1]>0) numinf++;
  if (acells[x+CELLLEN+1]==q) numill++; else if (acells[x+CELLLEN+1]>0) numinf++;

  /* Healthy cell: */
  if(acells[x] == 0)
    anewcells[x] = floor(numinf / k1) + floor(numill / k2);
  /* Infected cell: */
  else if(acells[x] < q - 1)
    anewcells[x] = floor(sum / (numinf + 1)) + g;
  /* Ill cell: */
  else
    anewcells[x] = 0;

  /* Bound next state to sane limit. */
  if(anewcells[x] > q - 1)
    anewcells[x] = q - 1;

  x++;
  if (x>(lenny-CELLLEN-1)) {
    x=CELLLEN+1;
    swap = cells; cells = newcells; newcells = swap;
  }


  return sum;
    
}
                                                                                         
unsigned char prob( unsigned char p ){                                                       if (rand()%10 < p) return true;                                                     
  else return false;                                                                     
}                                                                                        
                                                                                         
unsigned char sick( unsigned char patient ){
  if ((patient > 0) && (patient < k)) return true;                                       
  return false;                                                                          
}                                                                                        

int runSIR(unsigned char *cells, unsigned char *newcells){
  uint8_t cell;
  int x,sum=0;
  for (x=CELLLEN;x<(lenny-CELLLEN);x++){
    cell = cells[x];
    newcells[x]=cell;
    if (cell >= k) newcells[x] = recovered;                                                 else if (sick(cell)){
      newcells[x]++;                                                       
      sum++;
    }
    else if (cell == susceptible) {                                                    
      if (sick(cells[x-CELLLEN]) || sick(cells[x+CELLLEN]) ||                  
	  sick(cells[x-1]) || sick(cells[x+1])) {                  
	if (prob(tau)) newcells[x] = 1;       
      }
    }

    

  }

  //  swap = cells; cells = newcells; newcells = swap;
  return sum;
}


int runlife(unsigned char *cells, unsigned char *newcells){

  unsigned int x,sum,ssum;

  for (x=CELLLEN+1;x<(lenny-CELLLEN-1);x++){
    sum=cells[x]%2+cells[x-1]%2+cells[x+1]%2+cells[x-CELLLEN]%2+cells[x+CELLLEN]%2+cells[x-CELLLEN-1]%2+cells[x-CELLLEN+1]%2+cells[x+CELLLEN-1]%2+cells[x+CELLLEN+1]%2;
    ssum+=sum;
    sum=sum-cells[x]%2;
    if (sum==3 || (sum+(cells[x]%2)==3)) newcells[x]=255;
    else newcells[x]=0;
  }

  //    swap = cells; cells = newcells; newcells = swap;
  return ssum;
}

int runcell(unsigned char *cellu, unsigned char sizeLow, unsigned char sizeHigh, unsigned char rule){

  static unsigned char l=0; unsigned char cell, state, res;
  
  l++;
  l%=CELLLEN;

  for (cell = 1; cell < CELLLEN; cell++){ 
    if ((cell > sizeLow) && (cell < sizeHigh)){
      state = 0;
      if (cellu[cell + 1+ (l*CELLLEN)]>1)
	state |= 0x4;
      if (cellu[cell+(CELLLEN*l)]>1)
	state |= 0x2;
      if (cellu[cell - 1 +(CELLLEN*l)]>1)
	state |= 0x1;
                        
      if ((rule >> state) & 1){
	res += 1; 
	cellu[cell+(((l+1)%CELLLEN)*CELLLEN)] = 255;
      }
      else{
	cellu[cell+(((l+1)%CELLLEN)*CELLLEN)] = 0;
      } 
    }
    else
      cellu[cell+((l+1%CELLLEN)*CELLLEN)] = 0;
  }
  return res;
}

void initorbit(void){
  int i;

  for (i=0;i<starcount;i++){

    vx[i]=10-(((float) random() / (float) 0x7fffffff)*20); 
    vy[i]=10-(((float) random() / (float) 0x7fffffff)*20);
    vz[i]=10-(((float) random() / (float) 0x7fffffff)*20);

    x[i]=random()%1024;
    y[i]=random()%1024;
    z[i]=random()%1024;
  }

}

int runorbit(int accelerate){

  int i,j;
  double  d1,d2,d3,dis,dis2;

  for (i=0;i<starcount;i++){
    ax[i]=ay[i]=az[i]=0;
    for (j=0;j<starcount;j++){
      if (i!=j){ // else do nothing
	d1=(x[j]-x[i]);
	d2=(y[j]-y[i]);
	d3=(z[j]-z[i]);
	dis=d1*d1+d2*d2+d3*d3; 
	if (dis!=0) dis2=sqrt(dis);
	//      force=1000/dis2;
	ax[i]=ax[i]+accelerate*(x[j]-x[i])/dis2;
	ay[i]=ay[i]+accelerate*(y[j]-y[i])/dis2;
      }
    }
  }

  for (i=0;i<starcount;i++){
    vx[i]=vx[i]+ax[i];
    vy[i]=vy[i]+ay[i];
    vz[i]=vz[i]+az[i];
  }

  for (i=0;i<starcount;i++){
    x[i]=x[i]+vx[i];
    y[i]=y[i]+vy[i];
    z[i]=z[i]+vz[i];
  }

  return x[0];

}


int main(void)
{
  unsigned char x=0;
  uint8_t step=1;
  int scale;
  uint8_t k1,k2,q,g,kn;
  uint8_t low, high, rule, accelerate;
  uint8_t celln[lenny];
  uint8_t cellx[lenny];
  cells=celln;
  newcells=cellx;

  // 7 sets

  unsigned char (*instructionsetfirst[])(unsigned char* cells, unsigned char IP) = {outpp,finc,fdec,fincm,fdecm,fin1,outp,plus,minus,bitshift1,bitshift2,bitshift3,branch,jump,infect,store,writesamp,skip,direction,die}; // 20 instructions

  unsigned char (*instructionsetplague[])(unsigned char* cells, unsigned char IP) = {writesamp, ploutp, plenclose, plinfect, pldie, plwalk}; // 8 

  unsigned char (*instructionsetbf[])(unsigned char* cells, unsigned char IP) = {bfinc,bfdec,bfincm,bfdecm,bfoutp,bfin,bfbrac1,bfbrac2}; // 9

  unsigned char (*instructionsetSIR[])(unsigned char* cells, unsigned char IP) = {SIRoutp,SIRincif,SIRdieif,SIRrecif,SIRinfif}; // 6

  unsigned char (*instructionsetredcode[])(unsigned char* cells, unsigned char IP) = {rdmov,rdadd,rdsub,rdjmp,rdjmz,rdjmg,rddjz,rddat,rdcmp,rdoutp}; // 11

  unsigned char (*instructionsetbiota[])(unsigned char* cells, unsigned char IP) = {btempty,btoutp,btstraight,btbackup,btturn,btunturn,btg,btclear,btdup}; // 10

  unsigned char (*instructionsetreddeath[])(unsigned char* cells, unsigned char IP) = {redplague,reddeath,redclock,redrooms,redunmask,redprospero,redoutside}; // 7

  instructionp=0; insdir=1; dir=1; btdir=0; dcdir=0;


  adc_init();
  ioinit();
  //  uart0_init();
  init_DAC();

  ADMUX = 0x60; // clear existing channel selection                
  high(ADCSRA, ADSC); 
  loop_until_bit_is_set(ADCSRA, ADIF);
  //x=ADConvert(0);
  srand(ADCH);


  q=(rand()%100)+2;
  k1=(rand()%7)+1;   k2=(rand()%7)+1;
  g=rand()%100;
  spointer=0; mpointer=0;
  cttt=0;
  accelerate=4;
  rule=39;low=1;high=32;
  dtae=1;

  TIFR  |= _BV(OCF1A);	
  TCCR1A = 0;              // no pwm
  TCCR1B = _BV(WGM12) | _BV(CS10); // no clock div, CTC mode
  tick = F_CPU/samplerate; // make it go off 1ce per sample, no more than 22khz
  OCR1A = tick;
  OCR1B = 1;
  TIMSK |= _BV(OCIE1A);   // turn it on  

  // datagens

  initross(&ross);
  initifs(&ifs);
  initcell(cells,3,0,q+1);
  inittable(3,2,41); //radius,states(k),rule
  initorbit();

  rule=39;

  unsigned char *ptr,*ptrr;
  ptr = (unsigned char *)(0x1100);

  sei();
  //  wdt_enable(WDTO_1S);
  
  for(;;){
    step=(knob[4]>>4)+1;
    scale=knob[0]%16;
    //scale=4;
    cellhead=(knob[4]+1)*234; // or random walk through 
    // and if we have feedback? knob[x]=dtae
    // how to switch feedback?

    ptrr=ptr+cellhead;
    kn=knob[0]>>3;
    x++;

    if (x%step==0){
	kn=25;
      switch(kn){ // to 32
      case 0:
	dtae=olddtae<<scale;
	break;
      case 1:
	runbrain();
	dtae=spointer<<scale;
	break;
      case 2:
	runifs(&ifs);
	dtae=(ifs.returnvalx+1)<<scale;
	break;
      case 3:
	dtae=runcell1d(cells,3,2)<<scale;
	break;
      case 4:
	dtae=runcell1d(ptrr,3,2)<<scale;
	break;
      case 5:
	inittable(3,2,knob[4]); //radius,states(k),rule
	dtae=runcell1d(cells,3,2)<<scale;
	break;
      case 6:
	dtae=runhodge(cells,newcells,q,k1,k2,g)<<scale;
	break;
      case 7:
	dtae=runhodge(ptrr,newcells,q,k1,k2,g)<<scale;
	break;
      case 8:
	dtae=runhodge(cells,newcells,q,k1,k2,knob[4]>>4)<<scale;
	break;
      case 9:
	dtae=runhodge(ptrr,newcells,knob[4],k1,k2,g)<<scale;
	break;
      case 10:
	dtae=runhodge(ptrr,newcells,q,k1,knob[4]>>6,g)<<scale;
	break;
      case 11:
	dtae=runhodge(cells,newcells,q,knob[4]>>6,k2,g)<<scale;
	break;
      case 12:
	runross(&ross);
	dtae=ross.intz<<scale;
	break;
      case 13:
	dtae=runSIR(cells, newcells)<<scale; 
	swap = cells; cells = newcells; newcells = swap;
	if (dtae==0)   initcell(cells,3,0,q+1);
	break;
      case 14:
	dtae=runSIR(ptrr, newcells)<<scale;
	swap = ptrr; ptrr = newcells; newcells = swap;
	break;
      case 15:
	dtae=runlife(cells, newcells)<<scale;
	swap = cells; cells = newcells; newcells = swap;
	break;
      case 16:
	dtae=runlife(ptrr, newcells)<<scale;
	swap = ptrr; ptrr = newcells; newcells = swap;
	break;
      case 17: 
	dtae=runcell(cells, low, high, rule)<<scale;
	break;
      case 18:
	dtae=runcell(ptrr, low, high, knob[4])<<scale;
	break;
      case 19:
	dtae=runorbit(knob[4])<<scale;
	break;
      case 20:
	dtae=*(ptrr)<<scale;
	break;
      case 21:
	dtae+=(knob[0]>>3);
	break;
      case 22:
	ptrr=ptr+dtae;
	dtae=*(ptrr)<<scale;
	break;
      case 23:
	  ptrr=(int *)0x005d;
	  dtae=*ptrr<<scale;
	break;
      case 24:
	dtae=pgm_read_byte(&sinewave[x])<<scale; // frequency
	break;
      case 25:
	instruction=*(ptr+((cellhead+instructionp)%MAX_SAM));
	instructionp=(*instructionsetfirst[instruction%20]) (ptrr, instructionp); // mistake before as was instruction%INSTLEN in last instance
	//      insdir=dir*(IP%16)+1; // prev mistake as just got exponentially larger
	insdir=dir;
	break;
      case 26:
	    instruction=cells[instructionp];
	    instructionp=(*instructionsetbf[instruction%9]) (cells, instructionp);
	    insdir=dir;
	    break;
      case 27:
	    instruction=cells[instructionp];
	    instructionp=(*instructionsetSIR[instruction%6]) (cells, instructionp);
	    insdir=dir;
	    break;
      case 28:
	    instruction=cells[instructionp];
	    instructionp=(*instructionsetredcode[instruction%11]) (cells, instructionp); 
	    insdir=dir;
	    break;
      case 29:
	    instruction=cells[instructionp];
	    OCR0A=instruction;
	    instructionp+=insdir;
	    break;
      case 30:
	    instruction=cells[instructionp];
	    instructionp=(*instructionsetreddeath[instruction%7]) (cells, instructionp); 
	    insdir=dir;
	    break;
      case 31:
	    //la biota
	    instruction=cells[instructionp];
	    instructionp=(*instructionsetbiota[instruction%10]) (cells, instructionp); 
	    if (btdir==0) instructionp+=1;
	    else if (btdir==1) instructionp-=1;
	    else if (btdir==2) instructionp+=16;
	    else if (btdir==3) instructionp-=16;
      } 
      olddtae=dtae; 

    }

      cli();
      ADMUX = 0x61+kwhich;                
  high(ADCSRA, ADSC); 
  loop_until_bit_is_set(ADCSRA, ADIF);

  knob[kwhich]=ADCH;

  kwhich++;
  kwhich%=7;

  //6 knob[4] 2-distortion choice/switchings (distort1/2/apply datagens/applyADC etc), other?

  if ((PIND & 0x02) == 0x00) PORTD=(PORTD&0x43)+0x80;    // straight out - SW5 PD7 HIGH
  else {
    PORTD=(PORTD&0x43)+0x28;   // distortion through - SW1/3 = PD3/5 HIGH

  }
  if ((PIND & 0x01) == 0x00) {
    high(PORTD,PD6);   // feedback - SW4 = PD6 
    low(PORTE,PE2);       // and disconnect PE0 - preamp to ADC = now PE2
  }

  else 
    {
      low(PORTD, PD6);
      high(PORTE,PE2); // and connect PE0 - preamp to ADC = now PE2
    }

  sei();


  }
}


