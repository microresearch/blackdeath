/*

blackdeath samhain 2012 code base with more granular processing:

///CONTROLS///
                     TOP    
                     -0-write_effect/stepread

-3-writehead/scale              -2-readhead/scale

-5-start                        -1-end

                     -4-grainsize/stepwrite

///

*/
#define samplerate 5000
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
int spointerr, mpointerr;
uint32_t tick, tween, place, readhead, writehead, cellhead;
uint32_t maxsamp = MAX_SAM;
uint32_t lsamp = 0,llsamp=0;
uint32_t lowersamp = 0;
ifss ifs,ifsr;
rosstype ross,rossr;
volatile unsigned char rtae; // ????
volatile unsigned char wtae; // ????

//long oldrtae; // ????
//long oldwtae;
unsigned char datagen, effect, weff, wrambank,rrambank;
volatile unsigned char *xramptr,*xxramptr,*ptr;
volatile unsigned char knob[6] = {0, 0, 0, 0, 0, 0};

uint8_t *swap;
volatile uint8_t swapping;
uint8_t kwhich,dist, oldknob ,oldkn,oldknn,kn,knn;
uint8_t susceptible = 0;                                                                   uint8_t recovered = 255;                                                                   uint8_t tau = 2;  


volatile unsigned int chunk,rchunk;
uint8_t grainsize,rgrainsize;

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
unsigned int modrrr,modrr;
unsigned char i,ir,tmp;

SIGNAL(TIMER1_COMPA_vect) {

  
  grainsize=((knob[4]>>3)<<modrrr);
  if (knob[4]<8) grainsize=wtae+1;
  if (flagg==1){
    maxsamp=(uint32_t)((knob[1]+2)*233);
    //    if (knob[1]==0) maxsamp=(uint32_t)wtae*233;
    lowersamp=(uint32_t)((knob[5]+1)*234);
    //    if (knob[5]==0) lowersamp=(uint32_t)wtae*234;
    if (maxsamp>MAX_SAM || maxsamp<0) maxsamp=MAX_SAM;
    if (lowersamp>MAX_SAM || lowersamp<0 || lowersamp>=maxsamp) lowersamp=1;
    flagg=0; 
    tween=maxsamp-lowersamp;
    }
    llsamp=0x1100+lowersamp;
 
  i++; ir++;
   if (i>=grainsize) {
    chunk+=grainsize;
    // want to have wrapping window so need  
    // but then also %tween later and is lots of math
    if (chunk>=tween) chunk=chunk-tween;
    i=0;
    lsamp=llsamp+chunk;
    }

   /*
  if (ir>=rgrainsize) {
    rchunk+=rgrainsize;
    if (rchunk>=tween) rchunk=rchunk-tween;
    if (rchunk>tween) rchunk=0;
    rlsamp=0x1100+lowersamp+rchunk;
    ir=0;
    }*/
  //  modrr=0;  
   modrr=knob[0]>>4; //16
  switch(modrr){
  case 0:
  ADMUX = 0x60; // clear existing channel selection 8 BIT                
  high(ADCSRA, ADSC); 
  loop_until_bit_is_set(ADCSRA, ADIF);
  xramptr = (unsigned char *)(llsamp+((uint32_t)chunk+(uint32_t)(wtae%grainsize))%tween);
  *xramptr = (unsigned char) ADCH;
    break;
  case 1:
  ADMUX = 0x60; // clear existing channel selection 8 BIT                
  high(ADCSRA, ADSC); 
  loop_until_bit_is_set(ADCSRA, ADIF);
  //  xramptr = (unsigned char *)(lsamp+(uint32_t)(wtae%grainsize));
  xramptr = (unsigned char *)(llsamp+((uint32_t)chunk+(uint32_t)(wtae%grainsize))%tween);
  *xramptr = (unsigned char) ADCH&rtae;
    break;
  case 2:
  ADMUX = 0x60; // clear existing channel selection 8 BIT                
  high(ADCSRA, ADSC); 
  loop_until_bit_is_set(ADCSRA, ADIF);
  //  xramptr = (unsigned char *)(lsamp+(uint32_t)(wtae%grainsize));
  xramptr = (unsigned char *)(llsamp+((uint32_t)chunk+(uint32_t)(wtae%grainsize))%tween);
  *xramptr = (unsigned char) ADCH^rtae;
    break;
  case 3:
  ADMUX = 0x60; // clear existing channel selection 8 BIT                
  high(ADCSRA, ADSC); 
  loop_until_bit_is_set(ADCSRA, ADIF);
  //  xramptr = (unsigned char *)(lsamp+(uint32_t)(wtae%grainsize));
  xramptr = (unsigned char *)(llsamp+((uint32_t)chunk+(uint32_t)(wtae%grainsize))%tween);
  *xramptr = (unsigned char) ADCH|rtae;
    break;
  case 4:
    //  xramptr = (unsigned char *)(lsamp+(uint32_t)(wtae%grainsize));
  xramptr = (unsigned char *)(llsamp+((uint32_t)chunk+(uint32_t)(wtae%grainsize))%tween);
  *xramptr = (unsigned char) wtae*rtae;
  break;
  case 5:
    //  xramptr = (unsigned char *)(lsamp+(uint32_t)(wtae%grainsize));
  xramptr = (unsigned char *)(llsamp+((uint32_t)chunk+(uint32_t)(wtae%grainsize))%tween);
  *xramptr = (unsigned char) wtae;
    break;
  case 6:
    //  xramptr = (unsigned char *)(lsamp+(uint32_t)(wtae%grainsize));
  xramptr = (unsigned char *)(llsamp+((uint32_t)chunk+(uint32_t)(wtae%grainsize))%tween);
  *xramptr = (unsigned char) rtae;
    break;
  case 7:
    // copy what is at rtae and wtae
  xxramptr = (unsigned char *)(llsamp+((uint32_t)chunk+(uint32_t)(rtae%grainsize))%tween);
  xramptr = (unsigned char *)(llsamp+((uint32_t)chunk+(uint32_t)(wtae%grainsize))%tween);
  *xramptr = *xxramptr ;
  case 8:
  xramptr = (unsigned char *)(llsamp+((uint32_t)chunk+(uint32_t)(rtae%grainsize))%tween);
  xxramptr = (unsigned char *)(llsamp+((uint32_t)chunk+(uint32_t)(wtae%grainsize))%tween);
  *xramptr = *xxramptr;
  *xxramptr=rtae;
    break;
  case 9:
    // write to one and read from other
  ADMUX = 0x60; // clear existing channel selection 8 BIT                
  high(ADCSRA, ADSC); 
  loop_until_bit_is_set(ADCSRA, ADIF);
  xramptr = (unsigned char *)(llsamp+((uint32_t)chunk+(uint32_t)(rtae%grainsize))%tween);
  xxramptr = (unsigned char *)(llsamp+((uint32_t)chunk+(uint32_t)(wtae%grainsize))%tween);
  *xramptr = *xxramptr;
  *xxramptr=ADCH;
    break;
  case 10:
    // straight swap
  xxramptr = (unsigned char *)(llsamp+((uint32_t)chunk+(uint32_t)(rtae%grainsize))%tween);
  xramptr = (unsigned char *)(llsamp+((uint32_t)chunk+(uint32_t)(wtae%grainsize))%tween);
  tmp=*xramptr;
  *xramptr = *xxramptr ;
  *xxramptr = tmp;
  break;
  case 11:
    // variation on ignore chunk and swap with chunk
    xxramptr = (unsigned char *)(llsamp+(rtae%tween));
    xramptr = (unsigned char *)(llsamp+((uint32_t)chunk+(uint32_t)(wtae%grainsize))%tween);
    tmp=*xramptr;
    *xramptr = *xxramptr ;
    *xxramptr = tmp;
    break;
  case 12:
    // variation on ignore chunk and swap with chunk
    xxramptr = (unsigned char *)(llsamp+(wtae%tween));
    xramptr = (unsigned char *)(llsamp+((uint32_t)chunk+(uint32_t)(rtae%grainsize))%tween);
    tmp=*xramptr;
    *xramptr = *xxramptr ;
    *xxramptr = tmp;
    break;
  case 13:
    xxramptr = (unsigned char *)(llsamp+(rtae%tween));
    xramptr = (unsigned char *)(llsamp+((uint32_t)chunk+(uint32_t)(wtae%grainsize))%tween);
    *xramptr = *xxramptr ;
    break;
  case 14:
  ADMUX = 0x60; // clear existing channel selection 8 BIT                
  high(ADCSRA, ADSC); 
  loop_until_bit_is_set(ADCSRA, ADIF);
  xramptr = (unsigned char *)(llsamp+(rtae%tween));
  xxramptr = (unsigned char *)(llsamp+((uint32_t)chunk+(uint32_t)(wtae%grainsize))%tween);
  *xramptr = *xxramptr;
  *xxramptr=ADCH;
    break;
  case 15:
    //    xramptr = (unsigned char *)(lsamp+(uint32_t)(wtae%grainsize));
  xramptr = (unsigned char *)(llsamp+((uint32_t)chunk+(uint32_t)(wtae%grainsize))%tween);
    // leave as it is!

    }

  // how do we determine read one!

  //    xramptr = (unsigned char *)(lsamp+(uint32_t)(rtae%grainsize));
  xramptr = (unsigned char *)(llsamp+((uint32_t)chunk+(uint32_t)(rtae%grainsize))%tween);

    //xramptr = (unsigned char *)(0x1100+((MAX_SAM-(wtae%MAX_SAM))));
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
  int iter,i;
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

void runbrainw(void){
  unsigned char *xxx,ss; int kl=0;

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
    wtae=*xxx;
    break;
    /* Read value and store in current pointer */
  case 5:
    xxx=((unsigned char *)(0x1100+mpointer));
    *xxx=wtae;
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
      ss=0;
      xxx=((unsigned char *)(0x1100+spointer));
      while ((kl > 0 || (*xxx)%8 != 7) && ss<254)
	{
	  ss++;
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
      ss=0;
      while ((kl > 0 || (*xxx)%8 != 6) && ss<254) {
	ss++;
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

void runbrainr(void){
  unsigned char *xxx,ss; int kl=0;

  // should also be able to move these around

  spointerr++; 
  if (spointerr==0) spointerr=MAX_SAM;
  if (mpointerr==0) mpointerr=MAX_SAM;
  if (spointerr>MAX_SAM) spointerr=0;
  if (mpointerr>MAX_SAM) mpointerr=0;

  xxx=((unsigned char *)(0x1100+spointerr));
  prog=*xxx%8;
  switch(prog) {
    /* Increment pointer value */
  case 0:
    xxx=((unsigned char *)(0x1100+mpointerr));
    *xxx=*xxx+1;
    break;
    /* Decrement pointer value */
  case 1:
    xxx=((unsigned char *)(0x1100+mpointerr));
    *xxx=*xxx-1;
    break;
    /* Increment pointer */
  case 2:
    mpointerr++;
    break;
    /* Decrement pointer */
  case 3:
    mpointerr--;
    break;
    /* Print current pointer value */
  case 4:
    xxx=((unsigned char *)(0x1100+mpointerr));
    rtae=*xxx;
    break;
    /* Read value and store in current pointer */
  case 5:
    xxx=((unsigned char *)(0x1100+mpointerr));
    *xxx=rtae;
    break;
    /* Start loop */
  case 6:
    xxx=((unsigned char *)(0x1100+mpointerr));
    if (*xxx == 0) {
      /* Find matching ] */
      spointerr++;
      if (spointerr>MAX_SAM) spointerr=0;

      /* If kl == 0 and space[pointer] == ']' we found
       * the matching brace */
      ss=0;
      xxx=((unsigned char *)(0x1100+spointerr));
      while ((kl > 0 || (*xxx)%8 != 7) && ss<254)
	{
	  ss++;
	  if ((*xxx)%8 == 6) kl++;
	  if ((*xxx)%8 == 7) kl--;
	  /* Go in right direction */
	  spointerr++;
	  if (spointerr>MAX_SAM) spointerr=0;
	  xxx=((unsigned char *)(0x1100+spointerr));
	}
    }
    break;
    /* End loop */
  case 7:
    xxx=((unsigned char *)(0x1100+mpointerr));
    if (*xxx != 0) { // a fix
      /* Find matching [ */
      spointerr--;
      if (spointerr==0) spointerr=MAX_SAM;
      xxx=((unsigned char *)(0x1100+spointerr));
      ss=0;
      while ((kl > 0 || (*xxx)%8 != 6) && ss<254) {
	ss++;
	if ((*xxx)%8 == 6) kl--;
	if ((*xxx)%8 == 7) kl++;
	/* Go left */
	spointerr--;
	if (spointerr==0) spointerr=MAX_SAM;
	xxx=((unsigned char *)(0x1100+spointerr));
      }
      spointerr--;
      if (spointerr==0) spointerr=MAX_SAM;
    }
    break;
  }
}




unsigned char btempty(unsigned char* cells, unsigned char IP){
  // turn around
  if (btdir==0) btdir=1;
  else if (btdir==1) btdir=0;
  else if (btdir==2) btdir=3;
  else if (btdir==3) btdir=2;
  return IP;
}

unsigned char btoutpw(unsigned char* cells, unsigned char IP){
  wtae=cells[omem]; // 0-255
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
  // need to change for blackdeath

  switch(IP%7){
  case 0:
    //blue
    modrrr=0;
     break;
  case 1:
    //purple
    modrrr=1;
     break;
  case 2:
    //green
    modrrr=2;
     break;
  case 3:
    //orange
    modrrr=3;
     break;
  case 4:
    //white
    modrrr=4;
     break;
  case 5:
    //violet
    modrrr=5;
    break;
  case 6:
    // black
    modrrr=6;
  }
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
  //     ADCSRA = (1 << ADPS2) | (1<< ADPS0) ; //= 32 - very clean
          ADCSRA = (1 << ADPS2); // divide/16
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

  // ops: ++,-,&,|,division, *, bitshifts << >>

unsigned char inc(unsigned char* cellies, unsigned int ink, unsigned int param){
  return ink+param;}

unsigned char dec(unsigned char* cellies, unsigned int ink, unsigned int param){
  return ink-param;}

unsigned char inca(unsigned char* cellies, unsigned int ink, unsigned int param){
  modrr=0;
  return ink+param;}

unsigned char deca(unsigned char* cellies, unsigned int ink, unsigned int param){
  modrr=0;
  return ink-param;}

unsigned char incb(unsigned char* cellies, unsigned int ink, unsigned int param){
  modrr=1;
  return ink+param;}

unsigned char decb(unsigned char* cellies, unsigned int ink, unsigned int param){
  modrr=1;
  return ink-param;}

unsigned char incc(unsigned char* cellies, unsigned int ink, unsigned int param){
  modrr=2;
  return ink+param;}

unsigned char decc(unsigned char* cellies, unsigned int ink, unsigned int param){
  modrr=2;
  return ink-param;}

unsigned char incd(unsigned char* cellies, unsigned int ink, unsigned int param){
  modrr=3;
  return ink+param;}

unsigned char decd(unsigned char* cellies, unsigned int ink, unsigned int param){
  modrr=3;
  return ink-param;}

//

unsigned char andyw(unsigned char* cellies, unsigned int ink, unsigned int param){
  return ink&=rtae;}

unsigned char orryw(unsigned char* cellies, unsigned int ink, unsigned int param){
  return ink|=rtae;}

unsigned char excyw(unsigned char* cellies, unsigned int ink, unsigned int param){
  return ink^=rtae;}

unsigned char divvyw(unsigned char* cellies, unsigned int ink, unsigned int param){
  return ink/rtae;}

unsigned char starryw(unsigned char* cellies, unsigned int ink, unsigned int param){
  return ink*rtae;}

unsigned char leftyw(unsigned char* cellies, unsigned int ink, unsigned int param){
  return ink<<rtae;}

unsigned char rightyw(unsigned char* cellies, unsigned int ink, unsigned int param){
  return ink>>rtae;}

///

unsigned char andyr(unsigned char* cellies, unsigned int ink, unsigned int param){
  return ink&=wtae;}

unsigned char orryr(unsigned char* cellies, unsigned int ink, unsigned int param){
  return ink|=wtae;}

unsigned char excyr(unsigned char* cellies, unsigned int ink, unsigned int param){
  return ink^=wtae;}

unsigned char divvyr(unsigned char* cellies, unsigned int ink, unsigned int param){
  return ink/wtae;}

unsigned char starryr(unsigned char* cellies, unsigned int ink, unsigned int param){
  return ink*wtae;}

unsigned char leftyr(unsigned char* cellies, unsigned int ink, unsigned int param){
  return ink<<wtae;}

unsigned char rightyr(unsigned char* cellies, unsigned int ink, unsigned int param){
  return ink>>wtae;}


unsigned char rossy(unsigned char* cellies, unsigned int ink, unsigned int param){
  runross(&ross);
  return ross.intz<<param;
}

unsigned char ifsy(unsigned char* cellies, unsigned int ink, unsigned int param){
  runifs(&ifs);
  return (ifs.returnvalx+1)<<param;
}

unsigned char rossyr(unsigned char* cellies, unsigned int ink, unsigned int param){
  runross(&rossr);
  return rossr.intz<<param;
}

unsigned char ifsyr(unsigned char* cellies, unsigned int ink, unsigned int param){
  runifs(&ifsr);
  return (ifsr.returnvalx+1)<<param;
}


unsigned char iffsy(unsigned char* cellies, unsigned int ink, unsigned int param){
  return(ink*ink)<<param;
}

unsigned char swappy(unsigned char* cellies, unsigned int ink, unsigned int param){
  if (ink==wtae) return rtae;
  else return wtae;
}

unsigned char brainy(unsigned char* cellies, unsigned int ink, unsigned int param){
  runbrainw();
  return spointer<<param;
}

unsigned char brainyr(unsigned char* cellies, unsigned int ink, unsigned int param){
  runbrainr();
  return spointerr<<param;
}


unsigned char sine(unsigned char* cellies, unsigned int ink, unsigned int param){
  ink+=param;
  return(pgm_read_byte(&sinewave[ink%255])<<param); // frequency
}

unsigned char insl1(unsigned char* cellies, unsigned int ink, unsigned int param){
  instruction=*(unsigned char *)(cellies+(instructionpr%grainsize));
  instructionpr=(*instructionsetplaguew[instruction%6]) (cellies, instructionpr);
  insdirr=dirr;
  if (cells[instructionpr]==255 && dirr<0) dirr=1;
  else if (cells[instructionpr]==255 && dirr>0) dirr=-1; // barrier
  return rtae;
}

unsigned char insl2(unsigned char* cellies, unsigned int ink, unsigned int param){
      instruction=*(unsigned char *)(cellies+(instructionpr%grainsize));
      instructionpr=(*instructionsetfirstr[instruction%20]) ((unsigned char*)cellies, instructionpr);
      insdirr=dirr;
      return rtae;
}

unsigned char insl3(unsigned char* cellies, unsigned int ink, unsigned int param){
      instruction=*(unsigned char *)(cellies+(instructionpr%grainsize));
      instructionpr=(*instructionsetbfr[instruction%8]) ((unsigned char *)cellies, instructionpr);
      insdirr=dirr;
      return rtae;
}

unsigned char insl4(unsigned char* cellies, unsigned int ink, unsigned int param){
      instruction=*(unsigned char *)(cellies+(instructionpr%grainsize));
      instructionpr=(*instructionsetSIRr[instruction%5]) ((unsigned char *)cellies, instructionpr);
      insdirr=dirr;
      return rtae;
}

unsigned char insl5(unsigned char* cellies, unsigned int ink, unsigned int param){
      instruction=*(unsigned char *)(cellies+(instructionpr%grainsize));
      instructionpr=(*instructionsetredcoder[instruction%10]) ((unsigned char *)lsamp, instructionpr);
      insdirr=dirr;
      return rtae;
}

unsigned char insl6(unsigned char* cellies, unsigned int ink, unsigned int param){
      instruction=*(unsigned char *)(cellies+(instructionpr%grainsize));
      instructionpr=(*instructionsetreddeathr[instruction%7]) ((unsigned char *)lsamp, instructionpr);
      insdirr=dirr;
      return rtae;
}

unsigned char insl7(unsigned char* cellies, unsigned int ink, unsigned int param){
      instruction=*(unsigned char *)(cellies+(instructionpr%grainsize));
      instructionpr=(*instructionsetbiotar[instruction%9]) ((unsigned char *)cellies, instructionpr); 
      if (btdir==0) instructionpr+=1;
      else if (btdir==1) instructionpr-=1;
      else if (btdir==2) instructionpr+=16;
      else if (btdir==3) instructionpr-=16;
      return rtae;
}

unsigned char ins1(unsigned char* cellies, unsigned int ink, unsigned int param){
  instruction=*(unsigned char *)(cellies+(instructionp%grainsize));
  instructionp=(*instructionsetplaguew[instruction%6]) (cellies, instructionp);
  insdir=dir;
  if (cells[instructionp]==255 && dir<0) dir=1;
  else if (cells[instructionp]==255 && dir>0) dir=-1; // barrier
  return wtae;
}

unsigned char ins2(unsigned char* cellies, unsigned int ink, unsigned int param){
      instruction=*(unsigned char *)(cellies+(instructionp%grainsize));
      instructionp=(*instructionsetfirstr[instruction%20]) ((unsigned char*)cellies, instructionp);
      insdir=dir;
      return wtae;
}

unsigned char ins3(unsigned char* cellies, unsigned int ink, unsigned int param){
      instruction=*(unsigned char *)(cellies+(instructionp%grainsize));
      instructionp=(*instructionsetbfr[instruction%8]) ((unsigned char *)cellies, instructionp);
      insdir=dir;
      return wtae;
}

unsigned char ins4(unsigned char* cellies, unsigned int ink, unsigned int param){
      instruction=*(unsigned char *)(cellies+(instructionp%grainsize));
      instructionp=(*instructionsetSIRr[instruction%5]) ((unsigned char *)cellies, instructionp);
      insdir=dir;
      return wtae;
}


unsigned char ins5(unsigned char* cellies, unsigned int ink, unsigned int param){
      instruction=*(unsigned char *)(cellies+(instructionp%grainsize));
      instructionp=(*instructionsetredcoder[instruction%10]) ((unsigned char *)cellies, instructionp);
      insdir=dir;
      return wtae;
}

unsigned char ins6(unsigned char* cellies, unsigned int ink, unsigned int param){
      instruction=*(unsigned char *)(cellies+(instructionp%grainsize));
      instructionp=(*instructionsetreddeathr[instruction%7]) ((unsigned char *)cellies, instructionp);
      insdir=dir;
      return wtae;
}

unsigned char ins7(unsigned char* cellies, unsigned int ink, unsigned int param){
      instruction=*(unsigned char *)(cellies+(instructionp%grainsize));
      instructionp=(*instructionsetbiotaw[instruction%9]) ((unsigned char *)cellies, instructionp); 
      if (btdir==0) instructionp+=1;
      else if (btdir==1) instructionp-=1;
      else if (btdir==2) instructionp+=16;
      else if (btdir==3) instructionp-=16;
      return wtae;
}


unsigned char worm(unsigned char* cellies, unsigned int ink, unsigned int param){

  if (*(cellies)%0x01) ink+=16;
  else if (*(cellies)%0x02) ink-=16;
  else if (*(cellies)%0x04) ink+=1;
  else if (*(cellies)%0x08) ink-=1;
  return ink<<param;
}

unsigned char back(unsigned char* cellies, unsigned int ink, unsigned int param){
  return (chunk-ink);
}

unsigned char munge(unsigned char* cellies, unsigned int ink, unsigned int param){
  *(cellies)+=*(cellies+1);
  return (ink++)<<param;
}

unsigned char coded(unsigned char* cellies, unsigned int ink, unsigned int param){
  ptr=(int *)0x005d;
  return(*ptr<<param);
  }

unsigned char wredo(unsigned char* cellies, unsigned int ink, unsigned int param){
  grainsize=ink;
  return ink+param;
}

unsigned char non(unsigned char* cellies, unsigned int ink, unsigned int param){
  modrr=0;
  return 0;
}

unsigned char hodge(unsigned char* cellies, unsigned int wr, unsigned int param){
  int sum=0, numill=0, numinf=0;
  unsigned char q,k1,k2,g;
  static unsigned char x;
  static unsigned char flag=0;
  unsigned char *newcells, *cells;
  x=chunk;
  if ((flag&0x01)==0) {
    cells=cellies; newcells=&cells[chunk/2];
  }
  else {
    cells=cellies[chunk/2]; newcells=cellies;
  }      

  q=cells[0];k1=cells[1];k2=cells[2];g=cells[3];
  if (k1==0) k1=1;
  if (k2==0) k2=1;

  sum=cells[x]+cells[x-1]+cells[x+1]+cells[x-chunk]+cells[x+chunk]+cells[x-chunk-1]+cells[x-chunk+1]+cells[x+chunk-1]+cells[x+chunk+1];

  if (cells[x-1]==(q-1)) numill++; else if (cells[x-1]>0) numinf++;
  if (cells[x+1]==(q-1)) numill++; else if (cells[x+1]>0) numinf++;
  if (cells[x-chunk]==(q-1)) numill++; else if (cells[x-chunk]>0) numinf++;
  if (cells[x+chunk]==(q-1)) numill++; else if (cells[x+chunk]>0) numinf++;
  if (cells[x-chunk-1]==q) numill++; else if (cells[x-chunk-1]>0) numinf++;
  if (cells[x-chunk+1]==q) numill++; else if (cells[x-chunk+1]>0) numinf++;
  if (cells[x+chunk-1]==q) numill++; else if (cells[x+chunk-1]>0) numinf++;
  if (cells[x+chunk+1]==q) numill++; else if (cells[x+chunk+1]>0) numinf++;

  if(cells[x] == 0)
    newcells[x%128] = floor(numinf / k1) + floor(numill / k2);
  else if(cells[x] < q - 1)
    newcells[x%128] = floor(sum / (numinf + 1)) + g;
  else
    newcells[x%128] = 0;

  if(newcells[x%128] > q - 1)
    newcells[x%128] = q - 1;

  x++;
  if (x>((chunk/2)-chunk-1)) {
    x=chunk+1;
    flag^=0x01;
  }
  return sum;
}

unsigned char celr(unsigned char* cels, unsigned int ink, unsigned int param){

  static unsigned char l=0; unsigned char cell, state, res;
  unsigned char *cells=(unsigned char*)(0x1500+(CELLLEN*CELLLEN));
  unsigned char rule=cels[1];
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
  return res;
}

unsigned char SIRr(unsigned char* cellies, unsigned int ink, unsigned int param){
  unsigned char cell,x,sum=0;
  static unsigned char flag=0;
  unsigned char *newcells, *cells;
  unsigned char kk=cellies[2], p=cellies[3];

  if ((flag&0x01)==0) {
    cells=(unsigned char*)0x1500; newcells=0x1600;
  }
  else {
    //    cells=&cells[256]; newcells=cellies;
    cells=(unsigned char*)0x1600; newcells=0x1500;
  }      


  for (x=CELLLEN;x<(256-CELLLEN);x++){
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


unsigned char lifer(unsigned char* cellies, unsigned int ink, unsigned int param){
  unsigned char x, sum;

  static unsigned char flag=0;
  unsigned char *newcells, *cells;

  if ((flag&0x01)==0) {
    cells=(unsigned char*)0x1500; newcells=(unsigned char*)0x1580;
  }
  else {
    cells=(unsigned char*)0x1580; newcells=(unsigned char*)0x1500;
  }      

  for (x=CELLLEN+1;x<(128-CELLLEN-1);x++){
    sum=cells[x]%2+cells[x-1]%2+cells[x+1]%2+cells[x-CELLLEN]%2+cells[x+CELLLEN]%2+cells[x-CELLLEN-1]%2+cells[x-CELLLEN+1]%2+cells[x+CELLLEN-1]%2+cells[x+CELLLEN+1]%2;
    sum=sum-cells[x]%2;
    if (sum==3 || (sum+(cells[x]%2)==3)) newcells[x]=255;
    else newcells[x]=0;
  }
  
  // swapping 
  flag^=0x01;
  return sum;
}

unsigned char cel(unsigned char* cels, unsigned int ink, unsigned int param){

  static unsigned char l=0; unsigned char cell, state, res;
  unsigned char *cells=(unsigned char*)(0x1100+(CELLLEN*CELLLEN));
  unsigned char rule=cels[0];
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
  return res;
}

unsigned char SIR(unsigned char* cellies, unsigned int ink, unsigned int param){
  unsigned char cell,x,sum=0;
  static unsigned char flag=0;
  unsigned char *newcells, *cells;
  unsigned char kk=cellies[0], p=cellies[1];

  if ((flag&0x01)==0) {
    cells=(unsigned char*)0x1100; newcells=0x1200;
  }
  else {
    //    cells=&cells[256]; newcells=cellies;
    cells=(unsigned char*)0x1200; newcells=0x1100;

  }      


  for (x=CELLLEN;x<(256-CELLLEN);x++){
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


unsigned char life(unsigned char* cellies, unsigned int ink, unsigned int param){
  unsigned char x, sum;

  static unsigned char flag=0;
  unsigned char *newcells, *cells;

  if ((flag&0x01)==0) {
    cells=(unsigned char*)0x1100; newcells=(unsigned char*)0x1180;
  }
  else {
    cells=(unsigned char*)0x1180; newcells=(unsigned char*)0x1100;
  }      

  for (x=CELLLEN+1;x<(128-CELLLEN-1);x++){
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


  //  unsigned char (*wplag[])(unsigned char* cells, unsigned int wt, unsigned int p) = {hodgea,hodgeb,hodgec,hodged,inca,incb,incc,incd,deca,decb,decc,decd,cela,celb,celc,celd,lifea,lifeb,lifec,lifed,rossya,rossyb,rossyc,rossyd,ifsya,ifsyb,ifsyc,ifsyd,nona,nonb,nonc,nond}; // reduce to 8 x 4(as mods of modrr)

  unsigned char (*rplag[])(unsigned char* cells, unsigned int rt, unsigned int p) = {hodge,inc,dec,andyr,orryr,excyr,divvyr,starryr,leftyr,rightyr,iffsy, swappy, celr,SIRr,lifer,rossyr,ifsyr,brainyr,insl1,sine,insl2,insl3,insl4,insl5,insl6,insl7,worm,back,munge,coded,wredo,non}; //32!

unsigned char (*wplag[])(unsigned char* cells, unsigned int rt, unsigned int p) = {hodge,inc,dec,andyw,orryw,excyw,divvyw,starryw,leftyw,rightyw,iffsy, swappy, cel,SIR,life,rossy,ifsy,brainy,ins1,sine,ins2,ins3,ins4,ins5,ins6,ins7,worm,back,munge,coded,wredo,non}; //32!

  instructionp=0; insdir=1; dir=1; btdir=0; dcdir=0;
  instructionpr=0; insdirr=1; dirr=1; btdirr=0; dcdirr=0;
  modrrr=0;

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
  spointerr=0; mpointerr=0;
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

  initross(&ross);
  initifs(&ifs);
  initross(&rossr);
  initifs(&ifsr);


  sei();
  
  for(;;){
    x++;
    stepr=knob[0]%8;
    stepw=knob[4]%8;
    scalew=knob[3]%8;
    scaler=knob[2]%8;
    kn=knob[3]>>3;
    knn=knob[2]>>3;

    if ((x%stepw)==0) wtae=(*wplag[kn])((unsigned char*)lsamp,wtae,scalew); // does lsamp overflow?
    if ((x%stepr)==0) rtae=(*rplag[knn])((unsigned char*)lsamp,rtae,scaler);
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

  if ((PIND & 0x01) == 0x01) {
      low(PORTD, PD6);
      high(PORTE,PE2); // and connect PE0 - preamp to ADC = now PE2
  }
  else 
    {
    high(PORTD,PD6);   // feedback - SW4 = PD6 
    low(PORTE,PE2);       // and disconnect PE0 - preamp to ADC = now PE2

    }
}

}



