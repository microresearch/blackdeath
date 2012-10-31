/*

/// model so far is kind of OK...

crash!!!

- function pointers instead of switch with more manipulations...
- test/redo SIR, hodge, cel, life

///

- new scheme:

///
             -0-sample effect/what else?

-3-wtae/scale              -2-rtae/scale

-----------------------------

-5-start                  -1-end          -

             -4-grain size

///

- cleanups and speedups/test all datagens (see what changes are from micro)

//new features on knobs????//

knob5 and knob1 bottom left/right set to ZERO means dtae determines
sample length

knob4-bottommid <16 puts cellhead to dtae

*/
#define samplerate 8000
#define F_CPU 16000000UL 
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

#define MAX_SAM 50000 

#define BV(bit) (1<<(bit)) 
#define low(port, pin) (port &= ~_BV(pin))
#define high(port, pin) (port |= _BV(pin))
#define BET(A, B, C)  (((A>=B)&&(A<=C))?1:0)    /* a between [b,c] */

unsigned char *cells, *newcells;
int spointer, mpointer, prog;
uint32_t tick, tween, place, readhead, writehead, cellhead;
uint32_t maxsamp = MAX_SAM;
uint32_t lsamp = 0;
uint32_t lowersamp = 0;
ifss ifs;
rosstype ross;
volatile uint32_t rtae; // ????
volatile uint32_t wtae; // ????

//long oldrtae; // ????
//long oldwtae;
unsigned char datagen, effect, weff, wrambank,rrambank;
volatile unsigned char *xramptr;
volatile unsigned char knob[6] = {0, 0, 0, 0, 0, 0};

uint8_t *swap;
volatile uint8_t swapping;
uint8_t kwhich,dist, oldknob ,oldkn,oldknn,kn,knn;
uint8_t susceptible = 0;                                                                   uint8_t recovered = 255;                                                                   uint8_t tau = 2;  


unsigned int chunk;
uint32_t grainsize;

  uint8_t low, high, rule, accelerate;
  uint8_t celln[CELLLEN*CELLLEN];
  uint8_t cellx[CELLLEN*CELLLEN];
  uint8_t scaler,scalew; 
  uint8_t k1,k2,q,g;

                                                                       
uint8_t k = 128;
int lenny=CELLLEN*CELLLEN;
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

signed char insdir,dir,dirr,insdirr; 

unsigned char filterk, cpu, plague, hardk, fhk, instruction, IP, controls, hardware, samp, count,qqq;

unsigned int instructionp, instructionpr;

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

volatile unsigned char flagg;
volatile unsigned int i;

SIGNAL(TIMER1_COMPA_vect) {
  unsigned char modrr, tempi; 

  grainsize=(knob[4]>>2)+1;
  //  grainsize=(grainsize++)%10;
  if (flagg==1){
      maxsamp=(uint32_t)((knob[1]+2)*233);
      lowersamp=(uint32_t)((knob[5]+1)*234);
      if (maxsamp>MAX_SAM || maxsamp<0) maxsamp=MAX_SAM;
      if (lowersamp>MAX_SAM || lowersamp<0 || lowersamp>=maxsamp) lowersamp=1;
      flagg=0; 
      tween=maxsamp-lowersamp;
  }

  i++;
  if (i>=grainsize) {
    chunk+=grainsize;
    if (chunk>=tween) chunk=tween-chunk;
    if (chunk>tween) chunk=0;
    lsamp=0x1100+lowersamp+chunk;
    i=0;
  }
  
  modrr=knob[0]>>5;
  switch(modrr){
  case 0:
  ADMUX = 0x60; // clear existing channel selection 8 BIT                
  high(ADCSRA, ADSC); 
  loop_until_bit_is_set(ADCSRA, ADIF);
  xramptr = (unsigned char *)(lsamp+(wtae%grainsize));
  *xramptr = (unsigned char) ADCH;
    break;
  case 1:
  ADMUX = 0x60; // clear existing channel selection 8 BIT                
  high(ADCSRA, ADSC); 
  loop_until_bit_is_set(ADCSRA, ADIF);
  xramptr = (unsigned char *)(lsamp+(wtae%grainsize));
  *xramptr = (unsigned char) ADCH<<rtae;
    break;
  case 2:
  ADMUX = 0x60; // clear existing channel selection 8 BIT                
  high(ADCSRA, ADSC); 
  loop_until_bit_is_set(ADCSRA, ADIF);
  xramptr = (unsigned char *)(lsamp+(wtae%grainsize));
  *xramptr = (unsigned char) ADCH|rtae;
    break;
  case 3:
  xramptr = (unsigned char *)(lsamp+(wtae%grainsize));
  *xramptr = (unsigned char) *(unsigned char *)(lsamp+(rtae%grainsize));
  break;
  case 4:
  ADMUX = 0x60; // clear existing channel selection 8 BIT                
  high(ADCSRA, ADSC); 
  loop_until_bit_is_set(ADCSRA, ADIF);
  xramptr = (unsigned char *)(lsamp+(wtae%grainsize));
  *xramptr = (unsigned char) ADCH^rtae;
    break;
  case 5:
  ADMUX = 0x60; // clear existing channel selection 8 BIT                
  high(ADCSRA, ADSC); 
  loop_until_bit_is_set(ADCSRA, ADIF);
  xramptr = (unsigned char *)(lsamp+(wtae%grainsize));
  *xramptr = (unsigned char) ADCH&rtae;
  break;
  case 6:
  xramptr = (unsigned char *)(lsamp+(wtae%grainsize));
  *xramptr = (unsigned char) rtae;
    break;
  case 7:
    xramptr = (unsigned char *)(lsamp+(wtae%grainsize));
    // leave as it is!
  }
  xramptr = (unsigned char *)(lsamp+(rtae%grainsize));
  low(PORTB, CYWM_nSS);
  SPDR = 0b00001001;				// Send SPI byte
  while(!(SPSR & (1<<SPIF)));	// Wait for SPI transmission complete
  SPDR = *xramptr ;				// Send SPI byte
  while(!(SPSR & (1<<SPIF)));	// Wait for SPI transmission complete
  high(PORTB, CYWM_nSS);
}

unsigned char ostack[20], stack[20], omem;

/* BIOTA: two dimensional memory map */

unsigned char btdir,dcdir,btdirr,dcdirr;

unsigned char btempty(unsigned char* cells, unsigned char IP){
  // turn around
  if (btdir==0) btdir=1;
  else if (btdir==1) btdir=0;
  else if (btdir==2) btdir=3;
  else if (btdir==3) btdir=2;
  return IP;
}

unsigned char btoutpw(unsigned char* cells, unsigned char IP){
  wtae=cells[omem];
  return IP;
}

unsigned char btoutpr(unsigned char* cells, unsigned char IP){
  rtae=cells[omem];
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
  return IP;
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
    x++;
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
    cells[IP+count]=*((unsigned char *)(0x1100+(wtae%MAX_SAM))); //????
    count++;
    return IP; // just keeps on going
  }
  else return IP+insdir;
}

//3- clock every hour - instruction counter or IP -some kind of TICK
unsigned char redclockw(unsigned char* cells, unsigned char IP){
  clock++;
  if (clock%60==0) {
    wtae^=255;
    return IP; // everyone stops
  }
  else return IP+insdir;
}

unsigned char redclockr(unsigned char* cells, unsigned char IP){
  clock++;
  if (clock%60==0) {
    rtae^=255;
    return IP; // everyone stops
  }
  else return IP+insdir;
}


//4- seven rooms: divide cellspace into 7 - 7 layers with filter each
unsigned char redrooms(unsigned char* cells, unsigned char IP){
  /* need to change for blackdeath

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

unsigned char redprosperow(unsigned char* cells, unsigned char IP){

  unsigned char dirrr;
  // prince/omem moves at random through rooms
  dirrr=*((unsigned char *)(0x1100+(wtae%MAX_SAM)))%4;
  if (dirrr==0) omem=omem+1;
  else if (dirrr==1) omem=omem-1;
  else if (dirrr==2) omem=omem+16;
  else if (dirrr==3) omem=omem-16;

  // output
  wtae=cells[omem]; 
  return IP+insdir;
}

unsigned char redprosperor(unsigned char* cells, unsigned char IP){

  unsigned char dirrr;
  // prince/omem moves at random through rooms
  dirrr=*((unsigned char *)(0x1100+(wtae%MAX_SAM)))%4;
  if (dirrr==0) omem=omem+1;
  else if (dirrr==1) omem=omem-1;
  else if (dirrr==2) omem=omem+16;
  else if (dirrr==3) omem=omem-16;

  // output
  rtae=cells[omem]; 
  return IP+insdir;
}


  //7- the outside - the input!
unsigned char redoutside(unsigned char* cells, unsigned char IP){

  // input sample to cell (which one neighbour to omem)
  cells[omem+1]=*((unsigned char *)(0x1100+(wtae%MAX_SAM)));

  // output to filter 
  //  (*filtermod[qqq]) ((int)cells[omem]);
  return IP+insdir;
}

// plague

unsigned char ploutpw(unsigned char* cells, unsigned char IP){
  wtae=cells[IP+1]+cells[IP-1];
  return IP+insdir;
}

unsigned char ploutpr(unsigned char* cells, unsigned char IP){
  rtae=cells[IP+1]+cells[IP-1];
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

unsigned char rdoutpw(unsigned char* cells, unsigned char IP){
  wtae=cells[(IP+2)]; 
  IP+=3;
  return IP;
}

unsigned char rdoutpr(unsigned char* cells, unsigned char IP){
  rtae=cells[(IP+2)]; 
  IP+=3;
  return IP;
}


// SIR: inc if , die if, recover if, getinfected if 

unsigned char SIRoutpw(unsigned char* cells, unsigned char IP){
  wtae=cells[(IP+1)]+cells[IP-1];
  return IP+insdir;
}

unsigned char SIRoutpr(unsigned char* cells, unsigned char IP){
  rtae=cells[(IP+1)]+cells[IP-1];
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

unsigned char bfoutpw(unsigned char* cells, unsigned char IP){
  wtae=cells[omem]; 
  return IP++;
}

unsigned char bfoutpr(unsigned char* cells, unsigned char IP){
  rtae=cells[omem]; 
  return IP++;
}


unsigned char bfin(unsigned char* cells, unsigned char IP){
  cells[omem] = *((unsigned char *)(0x1100+(wtae%MAX_SAM)));
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
  omem=*((unsigned char *)(0x1100+(wtae%MAX_SAM)));
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


unsigned char outpw(unsigned char* cells, unsigned char IP){
  wtae=cells[omem];
  return IP+insdir;
}

unsigned char outppw(unsigned char* cells, unsigned char IP){
  wtae=omem;
  return IP+insdir;
}

unsigned char outpr(unsigned char* cells, unsigned char IP){
  rtae=cells[omem];
  return IP+insdir;
}

unsigned char outppr(unsigned char* cells, unsigned char IP){
  rtae=omem;
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
  cells[IP]=*((unsigned char *)(0x1100+(wtae%MAX_SAM)));
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
  //  ADCSRA |= (1 << ADPS1) | (1<< ADPS0) ; // seems now to hum = /8
  // ADCSRA = (1 << ADPS2) | (1<< ADPS0) ; //= 32 - very clean
      ADCSRA = (1 << ADPS2); // /16
  //  ADCSRA = (1 << ADPS2) | (1<< ADPS1);// /64
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

  // ops: ++,-,&,|,division, *, bitshifts << >>

unsigned char inc(unsigned char* cellies, unsigned int ink, unsigned int param){
  return ink+=param;}

unsigned char dec(unsigned char* cellies, unsigned int ink, unsigned int param){
  return ink-=param;}

unsigned char andy(unsigned char* cellies, unsigned int ink, unsigned int param){
  return ink&=param;}

unsigned char orry(unsigned char* cellies, unsigned int ink, unsigned int param){
  return ink|=param;}

unsigned char excy(unsigned char* cellies, unsigned int ink, unsigned int param){
  return ink^=param;}

unsigned char divvy(unsigned char* cellies, unsigned int ink, unsigned int param){
  return ink/=param;}

unsigned char starry(unsigned char* cellies, unsigned int ink, unsigned int param){
  return ink*=param;}

unsigned char lefty(unsigned char* cellies, unsigned int ink, unsigned int param){
  return ink<<=param;}

unsigned char righty(unsigned char* cellies, unsigned int ink, unsigned int param){
  return ink>>=param;}


unsigned char hodge(unsigned char* cellies, unsigned int wr, unsigned int param){
  int sum=0, numill=0, numinf=0;
  unsigned char q,k1,k2,g;
  static unsigned char x=CELLLEN+1;
  static unsigned char flag=0;
  unsigned char *newcells, *cells;

  if ((flag&0x01)==0) {
    cells=cellies; newcells=&cells[chunk/2];
  }
  else {
    cells=&cells[chunk/2]; newcells=cellies;
  }      

  q=cells[0];k1=cells[1];k2=cells[2];g=cells[3];
  if (k1==0) k1=1;
  if (k2==0) k2=1;

  sum=cells[x]+cells[x-1]+cells[x+1]+cells[x-CELLLEN]+cells[x+CELLLEN]+cells[x-CELLLEN-1]+cells[x-CELLLEN+1]+cells[x+CELLLEN-1]+cells[x+CELLLEN+1];

  if (cells[x-1]==(q-1)) numill++; else if (cells[x-1]>0) numinf++;
  if (cells[x+1]==(q-1)) numill++; else if (cells[x+1]>0) numinf++;
  if (cells[x-CELLLEN]==(q-1)) numill++; else if (cells[x-CELLLEN]>0) numinf++;
  if (cells[x+CELLLEN]==(q-1)) numill++; else if (cells[x+CELLLEN]>0) numinf++;
  if (cells[x-CELLLEN-1]==q) numill++; else if (cells[x-CELLLEN-1]>0) numinf++;
  if (cells[x-CELLLEN+1]==q) numill++; else if (cells[x-CELLLEN+1]>0) numinf++;
  if (cells[x+CELLLEN-1]==q) numill++; else if (cells[x+CELLLEN-1]>0) numinf++;
  if (cells[x+CELLLEN+1]==q) numill++; else if (cells[x+CELLLEN+1]>0) numinf++;

  if(cells[x] == 0)
    newcells[x%128] = floor(numinf / k1) + floor(numill / k2);
  else if(cells[x] < q - 1)
    newcells[x%128] = floor(sum / (numinf + 1)) + g;
  else
    newcells[x%128] = 0;

  if(newcells[x%128] > q - 1)
    newcells[x%128] = q - 1;

  x++;
  if (x>((chunk/2)-CELLLEN-1)) {
    x=CELLLEN+1;
    flag^=0x01;
  }
  return sum;
}

unsigned char cel(unsigned char* cells, unsigned int wr, unsigned int param){

  static unsigned char l=0; unsigned char cell, state, res;
  unsigned char rule=cells[0];
  res=0;
  l++;
  l%=CELLLEN;

  for (cell = 1; cell < CELLLEN; cell++){ 
      state = 0;
      if (cells[cell + 1+ (l*CELLLEN)]>128)
	state |= 0x4;
      if (cells[cell+(CELLLEN*l)]>128)
	state |= 0x2;
      if (cells[cell - 1 +(CELLLEN*l)]>128)
	state |= 0x1;
                     
      if ((rule >> state) & 1){
	res += 1; 
	cells[cell+(((l+1)%CELLLEN)*CELLLEN)] = 255;
      }
      else{
	cells[cell+(((l+1)%CELLLEN)*CELLLEN)] = 0;
      } 
  }
}

unsigned char SIR(unsigned char* cellies, unsigned int wr, unsigned int param){
  unsigned char cell,x,sum=0;
  static unsigned char flag=0;
  unsigned char *newcells, *cells;
  unsigned char kk=cellies[0], p=cellies[1];

  if ((flag&0x01)==0) {
    cells=cellies; newcells=&cells[chunk/2];
  }
  else {
    cells=&cells[chunk/2]; newcells=cellies;
  }      


  for (x=tween;x<((chunk/2)-CELLLEN);x++){
    cell = cells[x];
    newcells[x]=cell;
    if (cell >= kk) newcells[x] = recovered;                                                 else if ((cell>0 && cell<kk)){
      newcells[x]++;                                                       
    }
    else if (cell == susceptible) {   
      sum++;                                    
      if ( (cells[x-CELLLEN]>0 && cells[x-CELLLEN]<kk) ||
	   (cells[x+CELLLEN]>0 && cells[x+CELLLEN]<kk) ||
	   (cells[x-1]>0 && cells[x-1]<kk) ||
	   (cells[x+1]>0 && cells[x+1]<kk))
	{
	if (rand()%10 < p) newcells[x] = 1;       
      }
    }
  }
  flag^=0x01;
  return sum;
}

unsigned char life(unsigned char* cellies, unsigned int wr, unsigned int param){
  unsigned char x, sum;

  static unsigned char flag=0;
  unsigned char *newcells, *cells;

  if ((flag&0x01)==0) {
    cells=cellies; newcells=&cells[chunk/2];
  }
  else {
    cells=&cells[chunk/2]; newcells=cellies;
  }      

  for (x=CELLLEN+1;x<((chunk/2)-CELLLEN-1);x++){
    sum=cells[x]%2+cells[x-1]%2+cells[x+1]%2+cells[x-CELLLEN]%2+cells[x+CELLLEN]%2+cells[x-CELLLEN-1]%2+cells[x-CELLLEN+1]%2+cells[x+CELLLEN-1]%2+cells[x+CELLLEN+1]%2;
    sum=sum-cells[x]%2;
    if (sum==3 || (sum+(cells[x]%2)==3)) newcells[x]=255;
    else newcells[x]=0;
  }
  
  // swapping 
  flag^=0x01;
  return sum;
}


int main(void)
{
  unsigned char x=0, distie, dist,feedb;
  uint8_t stepr=1, stepw=1;

  // 7 sets

  unsigned char (*instructionsetfirstw[])(unsigned char* cells, unsigned char IP) = {outppw,finc,fdec,fincm,fdecm,fin1,outpw,plus,minus,bitshift1,bitshift2,bitshift3,branch,jump,infect,store,writesamp,skip,direction,die}; // 20 instructions

  unsigned char (*instructionsetplaguew[])(unsigned char* cells, unsigned char IP) = {writesamp, ploutpw, plenclose, plinfect, pldie, plwalk}; // 6

  unsigned char (*instructionsetbfw[])(unsigned char* cells, unsigned char IP) = {bfinc,bfdec,bfincm,bfdecm,bfoutpw,bfin,bfbrac1,bfbrac2}; // 8

  unsigned char (*instructionsetSIRw[])(unsigned char* cells, unsigned char IP) = {SIRoutpw,SIRincif,SIRdieif,SIRrecif,SIRinfif}; // 5

  unsigned char (*instructionsetredcodew[])(unsigned char* cells, unsigned char IP) = {rdmov,rdadd,rdsub,rdjmp,rdjmz,rdjmg,rddjz,rddat,rdcmp,rdoutpw}; // 10

  unsigned char (*instructionsetbiotaw[])(unsigned char* cells, unsigned char IP) = {btempty,btoutpw,btstraight,btbackup,btturn,btunturn,btg,btclear,btdup}; // 9

  unsigned char (*instructionsetreddeathw[])(unsigned char* cells, unsigned char IP) = {redplague,reddeath,redclockw,redrooms,redunmask,redprosperow,redoutside}; // 7

  // *2

  unsigned char (*instructionsetfirstr[])(unsigned char* cells, unsigned char IP) = {outppr,finc,fdec,fincm,fdecm,fin1,outpr,plus,minus,bitshift1,bitshift2,bitshift3,branch,jump,infect,store,writesamp,skip,direction,die}; // 20 instructions

  unsigned char (*instructionsetplaguer[])(unsigned char* cells, unsigned char IP) = {writesamp, ploutpr, plenclose, plinfect, pldie, plwalk}; // 6

  unsigned char (*instructionsetbfr[])(unsigned char* cells, unsigned char IP) = {bfinc,bfdec,bfincm,bfdecm,bfoutpr,bfin,bfbrac1,bfbrac2}; // 8

  unsigned char (*instructionsetSIRr[])(unsigned char* cells, unsigned char IP) = {SIRoutpr,SIRincif,SIRdieif,SIRrecif,SIRinfif}; // 5

  unsigned char (*instructionsetredcoder[])(unsigned char* cells, unsigned char IP) = {rdmov,rdadd,rdsub,rdjmp,rdjmz,rdjmg,rddjz,rddat,rdcmp,rdoutpr}; // 10

  unsigned char (*instructionsetbiotar[])(unsigned char* cells, unsigned char IP) = {btempty,btoutpr,btstraight,btbackup,btturn,btunturn,btg,btclear,btdup}; // 9

  unsigned char (*instructionsetreddeathr[])(unsigned char* cells, unsigned char IP) = {redplague,reddeath,redclockr,redrooms,redunmask,redprosperor,redoutside}; // 7

  unsigned char (*wplag[])(unsigned char* cells, unsigned int wt, unsigned int p) = {hodge,inc,dec,andy,orry,excy,divvy,starry,lefty,righty,cel,SIR,life};
  unsigned char (*rplag[])(unsigned char* cells, unsigned int rt, unsigned int p) = {hodge,inc,dec,andy,orry,excy,divvy,starry,lefty,righty,cel,SIR,life}; //13 so far

  // so port all the datagens, test, + usual suspects:

  instructionp=0; insdir=1; dir=1; btdir=0; dcdir=0;
  instructionpr=0; insdirr=1; dirr=1; btdirr=0; dcdirr=0;


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
  accelerate=4;
  rule=39;low=1;high=32;
  wtae=1;rtae=1;

  TIFR  |= _BV(OCF1A);	
  TCCR1A = 0;              // no pwm
  TCCR1B = _BV(WGM12) | _BV(CS10); // no clock div, CTC mode
  tick = F_CPU/samplerate; // make it go off 1ce per sample, no more than 22khz
  OCR1A = tick;
  OCR1B = 1;
  TIMSK |= _BV(OCIE1A);   // turn it on  

  // datagens

  //  initross(&ross);
  //  initifs(&ifs);
  //  initcell(cells,3,0,q+1);
  //  inittable(3,2,41); //radius,states(k),rule
  //  initorbit();

  rule=39;

  sei();
  //  wdt_enable(WDTO_1S);
  
  for(;;){

    scalew=knob[3]%8;
    scaler=knob[2]%8;
    kn=knob[3]>>4;
    knn=knob[2]>>4;
    kn=1;knn=11;
    wtae=(*wplag[kn])(lsamp,wtae,scalew);
    rtae=(*rplag[knn])(lsamp,rtae,scaler);

    cli();
    ADMUX = 0x61+kwhich;                
    high(ADCSRA, ADSC); 
    loop_until_bit_is_set(ADCSRA, ADIF);
    knob[kwhich]=ADCH;
    sei();
    if (oldkn!=knob[5] || oldknn!=knob[1]) flagg=1;
    oldkn=knob[5]; oldknn=knob[1];
    kwhich++;
    kwhich%=7;
   
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


}

}



