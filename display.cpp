

// code for the TFT display
#include "analyze_fft256iq.h"
#include "analyze_rms.h"
#include <Metro.h>
#include "display.h"
#include <Audio.h>
#include "font_regular.h"
#include "font_ArialBold.h"
#include <ILI9341_t3.h>
#define pos_spec 			196 				// position of spectrum display, has to be < 124
#define pos_x_tunestep 		99
#define pos_y_tunestep 		119 			
#define pos_x_frequency 	15 			
#define pos_y_frequency 	6  	
#define pos_y_mainmenu		45	
#define pos_x_smeter 		5
#define pos_y_smeter 		80
#define pos_x_dbm			190
#define s_w 10
#define notchpos 			196
#define notchL 				4
#define notchColour 		RED
#define spectrum_span 		96.000
#define bin_width	 		375
#define font_width 			24
#define modeUSB 			1
#define modeLSB 			2
#define WATERFALL_TOP  		200
#define WATERFALL_BOTTOM  	300
#define MAX_PIXEL			240					// horizontal max pixel
#define MIN_BW_CW     		1000

int pos_centre_f = 135; 						// Rx above IF, IF = 6000 	horizontal max pixel
int pos_centre_bins = 127;

uint8_t sch = 0;
bool freq_flag = 0;
extern ILI9341_t3 tft;
extern AudioAnalyzeFFT256IQ  	myFFT;
extern AudioAnalyzePeak			adc_input_voltage;
extern AudioAnalyzePeak			AGCMeter;
extern AudioMixer4          	MixGain;    // Summer (add inputs)
extern int RFgain_Auto;
extern const int8_t debug_pin  = 0;
uint8_t factor = 1;
#define AGC_THERSHOLD_UPPER		0.5		    // just a window of deisred agc non action
#define AGC_THERSHOLD_LOWER		0.4   		// dB gap
#define AGC_GAIN_FACTOR_UP		1.05		// Slow gain up 43 dB/sec FAST 
#define AGC_GAIN_FACTOR_DWN		0.95 		// Fast gain down 
#define AGC_GAIN_MAX			2048		// max is 2^15
#define AGC_GAIN_MIN			0.00004		// 1/2^15
#define AGC_FAST_FACTOR			1.5			// 50% Faster than MID
#define AGC_MEDIUM_FACTOR		1.3			// 20% Faster than MID
#define AGC_SLOW_FACTOR			1				
#define MEDIUM 					2			// definitions of AGC mode
#define FAST 					3
#define SLOW 					1 
#define OFF 					0
#define TUNING 					0
#define DBM_CALC  				33
#define AGC_TIMER				5 			// *** Noise does not make sanse to sample faster then newer values are available
#define RFG_TMR			  		333			// 
#define SPECTRUM_UPDATE		   50	
//*******************************************
// PRE-AMP+MIX		POST-AMP
// 18dB				35dB
//*******************************************
#define TOTOAL_AMP		   	   45.0			// FRONT end 51dB - 6dB SGTL5000
#define ADC_MAX_PP		   	   2.1		    // 2.82Vpp ADC maximum input, depends on Op Amp

float	initial_gain = 1.0;
extern int Menu;
extern int Menu2; 
extern int AGC_Mode;						// Switch for AGC control
extern uint8_t scope;
extern AudioControlSGTL5000 audioShield;  

extern float  RFgain_automatic;
float uv, rms_sample, dbuv, s_m, v, old_rms_sample, agc_peak;
static float dbm;
char string[7];  
int le, bwhelp, left, lsb; 
uint8_t pixelold[MAX_PIXEL] = {0};
uint8_t pixelnew[MAX_PIXEL];
int oldnotchF = 10000;
uint8_t digits_old [] = {9,9,9,9,9,9,9,9,9,9};

extern const int16_t menu_t_high = 16;
extern const int16_t pos_x_menu = 100;
extern const int16_t pos_x_menu_value_room = 80;
extern const int16_t pos_x_menu_value_start = 180;
extern const int16_t pos_y_menu = 304;

// Timers
Metro dBm_calc =  					Metro(DBM_CALC); 
Metro RFG_update =  				Metro(RFG_TMR);
Metro AGC_tmr_rms_sample = 			Metro(AGC_TIMER); 
Metro lcd_upd =						Metro(SPECTRUM_UPDATE);  	


int i;

const uint16_t gradient[] = {
  0x0
  , 0x1
  , 0x2
  , 0x3
  , 0x4
  , 0x5
  , 0x6
  , 0x7
  , 0x8
  , 0x9
  , 0x10
  , 0x1F
  , 0x11F
  , 0x19F
  , 0x23F
  , 0x2BF
  , 0x33F
  , 0x3BF
  , 0x43F
  , 0x4BF
  , 0x53F
  , 0x5BF
  , 0x63F
  , 0x6BF
  , 0x73F
  , 0x7FE
  , 0x7FA
  , 0x7F5
  , 0x7F0
  , 0x7EB
  , 0x7E6
  , 0x7E2
  , 0x17E0
  , 0x3FE0
  , 0x67E0
  , 0x8FE0
  , 0xB7E0
  , 0xD7E0
  , 0xFFE0
  , 0xFFC0
  , 0xFF80
  , 0xFF20
  , 0xFEE0
  , 0xFE80
  , 0xFE40
  , 0xFDE0
  , 0xFDA0
  , 0xFD40
  , 0xFD00
  , 0xFCA0
  , 0xFC60
  , 0xFC00
  , 0xFBC0
  , 0xFB60
  , 0xFB20
  , 0xFAC0
  , 0xFA80
  , 0xFA20
  , 0xF9E0
  , 0xF980
  , 0xF940
  , 0xF8E0
  , 0xF8A0
  , 0xF840
  , 0xF800
  , 0xF802
  , 0xF804
  , 0xF806
  , 0xF808
  , 0xF80A
  , 0xF80C
  , 0xF80E
  , 0xF810
  , 0xF812
  , 0xF814
  , 0xF816
  , 0xF818
  , 0xF81A
  , 0xF81C
  , 0xF81E
  , 0xF81E
  , 0xF81E
  , 0xF81E
  , 0xF83E
  , 0xF83E
  , 0xF83E
  , 0xF83E
  , 0xF85E
  , 0xF85E
  , 0xF85E
  , 0xF85E
  , 0xF87E
  , 0xF87E
  , 0xF83E
  , 0xF83E
  , 0xF83E
  , 0xF83E
  , 0xF85E
  , 0xF85E
  , 0xF85E
  , 0xF85E
  , 0xF87E
  , 0xF87E
  , 0xF87E
  , 0xF87E
  , 0xF87E
  , 0xF87E
  , 0xF87E
  , 0xF87E
  , 0xF87E
  , 0xF87E
  , 0xF87E
  , 0xF87E
  , 0xF87E
  , 0xF88F
  , 0xF88F
  , 0xF88F
};



void init_display(void) {
  tft.begin();
  tft.setRotation( 2 );
  tft.fillScreen(BLACK);  
}

//****************************************************************************************
// print band name at the bottom line
//
//****************************************************************************************
void show_rfg(String rfg) {  // show band  
 
  tft.fillRect(50, pos_y_mainmenu, 45, 9,BLACK);
  tft.setCursor(54, pos_y_mainmenu);
  tft.print(rfg);
  tft.print("dB");
  tft.setTextColor(WHITE);
}
//****************************************************************************************
// show selction during menu change wih some rect
//
//****************************************************************************************
void highlight_rfg(uint8_t on_off) {
 	//void drawRoundRect(int16_t x0, int16_t y0, int16_t w, int16_t h, int16_t radius, uint16_t color);
 if (on_off)
  tft.drawRoundRect(50-1, pos_y_mainmenu-1, 47, 11,2,RED);
 else
  tft.drawRoundRect(50-1, pos_y_mainmenu-1, 47, 11,2,BLACK);

}
//****************************************************************************************
// print band name at the bottom line
//
//****************************************************************************************
void show_agc(uint8_t agc) 
{  

		tft.setTextColor(WHITE);
		tft.fillRect(1, pos_y_mainmenu, 44, 9,BLACK);		
		tft.setCursor(4, pos_y_mainmenu);
		switch (AGC_Mode) 
			{
			case FAST: tft.print("AGC-F"); break;
			case MEDIUM: tft.print("AGC-M"); break;
			case SLOW: tft.print("AGC-S"); break;
			case OFF: tft.print("NOAGC"); break;
			tft.setTextColor(WHITE);			
			}
	
}
//****************************************************************************************
// show selction during menu change wih some rect
//
//****************************************************************************************
void highlight_agc(uint8_t on_off) {
 if (on_off)
  tft.drawRoundRect(0, pos_y_mainmenu-1, 47, 11,2,RED);
 else
  tft.drawRoundRect(0, pos_y_mainmenu-1, 47, 11,2,BLACK);
}
//****************************************************************************************
// show selction during menu change wih some rect
//
//****************************************************************************************
void highlight_bw(uint8_t on_off) {
	if (on_off)
		tft.drawRoundRect(99, pos_y_mainmenu-1, 47, 11,2,RED);
	else
		tft.drawRoundRect(99, pos_y_mainmenu-1, 47, 11,2,BLACK);
}
//****************************************************************************************
// Setup Smeter
//
//
//****************************************************************************************

void setup_display(void) {
  
  // initialize the LCD display
  tft.fillScreen(BLACK); //BLACK);
  tft.setCursor(0, 119);
  tft.setTextColor(WHITE);
  tft.setTextWrap(false);  
  tft.setFont(Arial_8_Bold);
  // draw adc_input_voltage layout
  tft.fillRect(pos_x_smeter, pos_y_smeter-3, 2, 2, WHITE);
  tft.fillRect(pos_x_smeter+8*s_w, pos_y_smeter-3, 2, 2, WHITE);
  tft.fillRect(pos_x_smeter+2*s_w, pos_y_smeter-3, 2, 2, WHITE);
  tft.fillRect(pos_x_smeter+4*s_w, pos_y_smeter-3, 2, 2, WHITE);
  tft.fillRect(pos_x_smeter+6*s_w, pos_y_smeter-3, 2, 2, WHITE);
  tft.fillRect(pos_x_smeter+7*s_w, pos_y_smeter-4, 2, 3, WHITE);
  tft.fillRect(pos_x_smeter+3*s_w, pos_y_smeter-4, 2, 3, WHITE);
  tft.fillRect(pos_x_smeter+5*s_w, pos_y_smeter-4, 2, 3, WHITE);
  tft.fillRect(pos_x_smeter+s_w, pos_y_smeter-4, 2, 3, WHITE);
  tft.fillRect(pos_x_smeter+9*s_w, pos_y_smeter-4, 2, 3, WHITE);
  tft.fillRect(pos_x_smeter+11*s_w, pos_y_smeter-4, 2, 3, WHITE);
  tft.fillRect(pos_x_smeter+13*s_w, pos_y_smeter-4, 2, 3, WHITE);
  tft.fillRect(pos_x_smeter+15*s_w, pos_y_smeter-4, 2, 3, WHITE);
  tft.setCursor(pos_x_smeter - 4, pos_y_smeter - 14);
  tft.setTextColor(WHITE);
  tft.setTextWrap(true);
  tft.print("S 1");
  tft.setCursor(pos_x_smeter + 28, pos_y_smeter - 14);
  tft.print("3");
  tft.setCursor(pos_x_smeter + 48, pos_y_smeter - 14);
  tft.print("5");
  tft.setCursor(pos_x_smeter + 68, pos_y_smeter - 14);
  tft.print("7");
  tft.setCursor(pos_x_smeter + 88, pos_y_smeter - 14);
  tft.print("9");
  tft.setCursor(pos_x_smeter + 120, pos_y_smeter - 14);
  tft.print(" 30");
  
} // end void setupdisplay


//****************************************************************************************
// draw the spectrum display
// this version draws 1/10 of the spectrum per call but we run it 10x the speed
// this allows other stuff to run without blocking for so long
// Also check al timing critical tasks for AGC
//****************************************************************************************
void show_spectrum(float line_gain, float LPFcoeff, int M,long int FU, long int FL ) { 
      static uint8_t first_time_full=0, startx=0, endx; 
	  static uint16_t p=WATERFALL_TOP, cnt=WATERFALL_BOTTOM;	  
	  static float agc_avg=0;
	  float voltage;
	  int bin_cnt, i;
      endx=startx+MAX_PIXEL;
      float scale = 1.75;
	  char rfg[7];  
      float avg,wtf = 0.0;
	  static uint32_t timer = 0;
	  uint8_t clipping, not_clipping;
	  uint16_t bw;

	// ScrollAreaDefinition(uint16_t TopFixedArea, uint16_t VerticalScrollingArea, uint16_t BottomFixedArea)
	// summ must be 320
     tft.ScrollAreaDefinition(WATERFALL_TOP,WATERFALL_BOTTOM-WATERFALL_TOP,20); 
	
	// Draw spectrum display 256 Point to 240 pixels		
	if (lcd_upd.check()==1)
	  {	
      for (uint8_t x = startx; x < endx; x++  ) 
	   {
		if ((x > 2) && (x < 237)) {
			avg = myFFT.output[(x)*256/240]*0.8  + 
			myFFT.output[(x-1)*256/240]*0.05 + 			
			myFFT.output[(x+1)*256/240]*0.05 + 
			myFFT.output[(x+2)*256/240]*0.03 +
			myFFT.output[(x-3)*256/240]*0.02 +
			myFFT.output[(x+3)*256/240]*0.02;}
		else
			avg = myFFT.output[(x)*256/240]; 		
			wtf = abs(avg);
			uint8_t value = map(20*log10f(wtf), 0,116 , 1, 117);
			wtf= gradient[value];				
			tft.drawPixel(x,p,wtf);						
			//
			// Spectrum scope
			//
			if (RFgain_Auto)
				pixelnew[x] = LPFcoeff *scale* 10*log10f (abs(avg))-RFgain_automatic/4+ (1 - LPFcoeff) * pixelold[x];	
			else
				pixelnew[x] = LPFcoeff *scale* 10*log10f (abs(avg))-line_gain/4+ (1 - LPFcoeff) * pixelold[x];	
			if (( pos_spec-pixelnew[x] ) < pos_y_smeter+10)
				pixelnew[x]=pos_spec-pos_y_smeter-10;		
   		
            if (pixelold[x] != pixelnew[x]) 
			  {               
				tft.drawFastVLine (x, pos_y_smeter+9, pos_spec-pos_y_smeter-10, BLACK); // delete line					
				tft.drawFastVLine (x, pos_spec-pixelnew[x], pixelnew[x]+1,BLUE);
				tft.drawPixel(x, pos_spec-1-pixelnew[x], LBLUE); // write new pixel
				pixelold[x] = pixelnew[x]; 		
				}
	   }
		
	// ok, got full line drawn, start new
	if(startx ==MAX_PIXEL)
		startx=0;			

	if (cnt ==  WATERFALL_TOP)
		cnt=WATERFALL_BOTTOM;

	if( first_time_full)
		{	
		tft.setScroll(cnt--);	
		if (p == WATERFALL_TOP)
			p = WATERFALL_BOTTOM;
		p = p-1;
		}
	else
		p = p+1;		
	if (p == WATERFALL_BOTTOM-1)
		first_time_full =1;	
				
  	}
	
    // ******************************************************************************************************
	// Colect an calculate AGC values, take rms_sample every 2ms and calc average	
	// reads all AGC_tmr_rms_sample the peak values and feeds values 
	// through a low pass.
	// ******************************************************************************************************	
	if (AGC_tmr_rms_sample.check()==1)
	{ 	
	AGC_tmr_rms_sample.reset();	
	if (AGCMeter.available() && (AGC_Mode)){	
	
		agc_peak = AGCMeter.read();
		switch (AGC_Mode) {
			case SLOW: factor=AGC_SLOW_FACTOR; break; 
			case MEDIUM: factor=AGC_MEDIUM_FACTOR; break; 
			case FAST: factor=AGC_FAST_FACTOR; break; 
			default: factor = 1;
			}
		// Low pass
		agc_avg = agc_avg*0.2+agc_peak*0.8;					
		// reduce gain				
		if (agc_avg > AGC_THERSHOLD_UPPER) 
		{  
			factor = (factor+agc_avg-AGC_THERSHOLD_UPPER)*(factor+agc_avg-AGC_THERSHOLD_UPPER);
			if (factor > 2) factor = 2;
			initial_gain = initial_gain*AGC_GAIN_FACTOR_DWN/factor;
			if (initial_gain < AGC_GAIN_MIN ) 
				initial_gain = AGC_GAIN_MIN;
			if (initial_gain != AGC_GAIN_MIN)
				{
				MixGain.gain(0,initial_gain);       	// Adjust AGC gain					
				MixGain.gain(1,initial_gain);       	// Adjust AGC gain	
				}				
		}
		//increment gain
		if (agc_avg < AGC_THERSHOLD_LOWER) 		
			{
			initial_gain = initial_gain*AGC_GAIN_FACTOR_UP*factor;
			if (initial_gain > AGC_GAIN_MAX ) 
				initial_gain = AGC_GAIN_MAX;
			if (initial_gain != AGC_GAIN_MAX)
				{
				MixGain.gain(0,initial_gain);       	// Adjust AGC gain					
				MixGain.gain(1,initial_gain);       	// Adjust AGC gain	
				}				
			}				
		}
	}	
	// ******************************************************************************************************
	// Colect an calculate S-Meter values, from spectrum	
	// make a average of values to avoid to nervous display
	// through a low pass.	
	// Use data only for the dBm and Smeter display not for the RFGain controll
	// Tukey Windowing	a=0,25     a=0,5    a=0,75
	// SCALLOP Loss     2,96       2,24     1,73
	// -6dB BW bins     1,38       1,57     1,8
	// ******************************************************************************************************	 
	#if 0
		Serial.println("Spectrum bins at LSB:");	
		Serial.println(myFFT.output[143]);	
		Serial.println(myFFT.output[142]);	
		Serial.println(myFFT.output[141]);	
		Serial.println("Spectrum bins at USB:");
		Serial.println(myFFT.output[111]);
		Serial.println(myFFT.output[112]);
		Serial.println(myFFT.output[113]);		
		Serial.println(myFFT.output[114]);
		Serial.println(myFFT.output[115]);
		Serial.println(myFFT.output[116]);			
		Serial.println(myFFT.output[117]);
		Serial.println(myFFT.output[118]);
		Serial.println(myFFT.output[119]);		
		Serial.println(myFFT.output[120]);		
		Serial.println(rms_sample);
	#endif
		 
	 switch (M) {
				//***************************************************
		case 0: //AM
		case 3: //DSB    
		case 4: //StereoAM   		
		bw = (FU + FL);
		// Calcuate amplitude based on bandwidth
  	    bin_cnt = (int)(bw / (1.57 * bin_width));	
		if (!bin_cnt) 
			bin_cnt=1;
		i = bin_cnt;
		pos_centre_bins = 127;
		rms_sample=0;
		while (bin_cnt--)
			{
			rms_sample = rms_sample + myFFT.output[pos_centre_bins+bin_cnt]*myFFT.output[pos_centre_bins+bin_cnt];
			rms_sample = rms_sample + myFFT.output[pos_centre_bins-bin_cnt]*myFFT.output[pos_centre_bins-bin_cnt];
			}
		rms_sample = sqrtf(rms_sample/i);			
		break;  
				//***************************************************
		case 2: //LSB Mode   
	    bw = (FL);
		// Calcuate amplitude based on bandwidth
  	    bin_cnt = (int)(bw / (1.57 * bin_width))-2;	
		if (!bin_cnt) 
			bin_cnt=1;
		i = bin_cnt;		
		pos_centre_bins = 143-3;
		rms_sample=0;
		while (bin_cnt--)
			rms_sample = rms_sample + myFFT.output[pos_centre_bins-bin_cnt]*myFFT.output[pos_centre_bins-bin_cnt];			
		rms_sample = sqrtf(rms_sample/i);		
		break;
				 //***************************************************
		case 1:  //USB Mode
		bw = (FU);
		// Calcuate amplitude based on bandwidth
  	    bin_cnt = (int)(bw / (1.57 * bin_width))-2;	
		if (!bin_cnt) 
			bin_cnt=1;
		i = bin_cnt;		
		pos_centre_bins = 111+3;
		rms_sample=0;
		while (bin_cnt--)
			rms_sample = rms_sample + myFFT.output[pos_centre_bins+bin_cnt]*myFFT.output[pos_centre_bins+bin_cnt];			
		rms_sample = sqrtf(rms_sample/i);		
		break;
		
		case 5:  //CW
		bw = (FU-MIN_BW_CW);
		// Calcuate amplitude based on bandwidth
  	    bin_cnt = (int)(bw / (1.57 * bin_width))-1;	
		if (bin_cnt<1) 
			bin_cnt=1;
		i = bin_cnt;		
		pos_centre_bins = 143-3;
		rms_sample=0;
		while (bin_cnt--)
			rms_sample = rms_sample + myFFT.output[pos_centre_bins+bin_cnt]*myFFT.output[pos_centre_bins+bin_cnt];			
		rms_sample = sqrtf(rms_sample/i);		

		break;
		
		default: 
		rms_sample =1;
		break;
		}
		
	// Show S-Meter Data to Display, slower update but faster then dbm numbers	
	if (dBm_calc.check()==1) 
		{  	
		uv=((rms_sample*0.6)+(old_rms_sample*0.4))*1010152/32768;	
		old_rms_sample = rms_sample;		
		// correction due to scalop loss = 2,24
		if (!RFgain_Auto)
			{		
			dbuv = 20.0*log10f(uv)-line_gain*1.5-TOTOAL_AMP+2.24+17;	
			dbm = dbuv -107;
			}
		else 
			{			
			dbuv = 20.0*log10f(uv)-RFgain_automatic*1.5-TOTOAL_AMP+2.24+17;	
			dbm = dbuv -107;
			}		
		if (dbm<-130) dbm=-130;				
		s_m = 1.0 + (14.1 + dbuv)/6.0;
		if (s_m <0.0) s_m=0.0;
		if (s_m>9.0)
		{
        dbuv = dbuv - 34.0;
        s_m = 9.0;
		}
		else dbuv = 0;		

		// Draw S-meter bar
		tft.drawFastHLine(pos_x_smeter, pos_y_smeter, s_m*s_w+1, BLUE);
		tft.drawFastHLine(pos_x_smeter+s_m*s_w+1, pos_y_smeter, (9*s_w+1)-s_m*s_w+1, BLACK);
		tft.drawFastHLine(pos_x_smeter, pos_y_smeter+1, s_m*s_w+1, BLUE);
		tft.drawFastHLine(pos_x_smeter+s_m*s_w+1, pos_y_smeter+1, (9*s_w+1)-s_m*s_w+1, BLACK);
		tft.drawFastHLine(pos_x_smeter, pos_y_smeter+2, s_m*s_w+1, BLUE);
		tft.drawFastHLine(pos_x_smeter+s_m*s_w+1, pos_y_smeter+2, (9*s_w+1)-s_m*s_w+1, BLACK);
		tft.drawFastHLine(pos_x_smeter, pos_y_smeter+3, s_m*s_w+1, BLUE);
		tft.drawFastHLine(pos_x_smeter+s_m*s_w+1, pos_y_smeter+3, (9*s_w+1)-s_m*s_w+1, BLACK);
	
		if(dbuv>45) dbuv=45;
		tft.drawFastHLine(pos_x_smeter+9*s_w+1, pos_y_smeter, (dbuv/7.5)*s_w+1, RED);
		tft.drawFastHLine(pos_x_smeter+9*s_w+(dbuv/7.5)*s_w+1, pos_y_smeter, (6*s_w+1)-(dbuv/7.5)*s_w, BLACK);
		tft.drawFastHLine(pos_x_smeter+9*s_w+1, pos_y_smeter+1, (dbuv/7.5)*s_w+1, RED);
		tft.drawFastHLine(pos_x_smeter+9*s_w+(dbuv/7.5)*s_w+1, pos_y_smeter+1, (6*s_w+1)-(dbuv/7.5)*s_w, BLACK);
		tft.drawFastHLine(pos_x_smeter+9*s_w+1, pos_y_smeter+2, (dbuv/7.5)*s_w+1, RED);
		tft.drawFastHLine(pos_x_smeter+9*s_w+(dbuv/7.5)*s_w+1, pos_y_smeter+2, (6*s_w+1)-(dbuv/7.5)*s_w, BLACK);    
		tft.drawFastHLine(pos_x_smeter+9*s_w+1, pos_y_smeter+3, (dbuv/7.5)*s_w+1, RED);
		tft.drawFastHLine(pos_x_smeter+9*s_w+(dbuv/7.5)*s_w+1, pos_y_smeter+3, (6*s_w+1)-(dbuv/7.5)*s_w, BLACK);    
				
    }	 
	
	//******************************************************
	//* update the dBm number display
	//* and calc the RFG Gain
	//******************************************************	
	if (RFG_update.check()==1) 
		{ 	
		// reset this timer else 
		RFG_update.reset();	 
		voltage = adc_input_voltage.read();
		/*
		Serial.println("Voltage:");	
		Serial.println(voltage);	
		
		float gain_temp=1;
		
		if(!RFgain_Auto)							 
			gain_temp = pow(10,line_gain*1.5/20);
		else	
			gain_temp = pow(10,RFgain_automatic*1.5/20);		
	    */
			//******************************************************
			//* Check for Clipping, use the automatic RFgain or manual 
			//******************************************************			 
			clipping = 0;
		    not_clipping = 0;			 
			if ((float)(voltage*2) > (ADC_MAX_PP*0.5)) 
				{
				clipping = 1; 
				tft.setTextColor(RED);
				}
			if ((float)(voltage*2) < (ADC_MAX_PP*0.25)) 
				{	
				not_clipping = 1;
				tft.setTextColor(WHITE);
				}					
			// hier Anzeige dBm, 
			// First the colour is changed by the clipping detector
			tft.setCursor(pos_x_dbm, pos_y_smeter-8);
			tft.fillRect(pos_x_dbm, pos_y_smeter-8, 240-pos_x_dbm, 8,BLACK);
			sprintf(string,"%04.0fdBm",dbm);
			tft.print(string);
			tft.setTextColor(WHITE);		
				
			// Display automatic Gain Value
			if (RFgain_Auto)
				{ 
				tft.setTextColor(BLUE);		  
				sprintf(rfg,"%2.1f",(float)RFgain_automatic*1.5);
				show_rfg(rfg);
				tft.setTextColor(WHITE);				
				}				
				
		//*************************************************************
		// Automatic RF (SGTL) Pre Amplifier Gain Controll
		//*************************************************************	
		if (RFgain_Auto)
		{	
		  if (clipping && (RFgain_automatic >0))      // did clipping almost occur?
		  {			
			  if (RFgain_automatic)       // yes - is this NOT zero?
			  {
				RFgain_automatic--;;    // decrease gain one step, 1.5dB
				if (RFgain_automatic < 0)        
				  RFgain_automatic = 0;        
				timer = 0;  // reset the adjustment timer
				audioShield.lineInLevel(RFgain_automatic,RFgain_automatic);  
				Serial.print("clipping");					
			  }			
		  }
		  if (not_clipping && (RFgain_automatic <15))     // no clipping occurred
		  {		
				RFgain_automatic += 1;    // increase gain by one step, 1.5dB
				timer = 0;  // reset the timer to prevent this from executing too often
				if (RFgain_automatic > 15)      
					RFgain_automatic = 15;      
				audioShield.lineInLevel(RFgain_automatic,RFgain_automatic);

			} 	
		}			
	}
 }


//****************************************************************************************
// This shows bandwidht under the spectrum diagram, use full information.
// Here CW must be added.
//
//
//****************************************************************************************
void show_bandwidth (int M, long int FU, long int FL ) {

   tft.drawFastHLine(0,pos_spec+1,240, BLUE); // erase old indicator
   tft.drawFastHLine(0,pos_spec+2,240, BLUE); // erase old indicator 
   tft.drawFastHLine(0,pos_spec+3,240, BLUE); // erase old indicator
   tft.drawFastHLine(0,pos_spec,240, BLUE); // erase old indicator
   
   tft.fillRect(150, pos_y_mainmenu, 45, 8,BLACK);
   tft.setTextColor(WHITE);
   tft.setCursor(154, pos_y_mainmenu);
     
   bwhelp = FU /100;
   int leU = bwhelp*24/spectrum_span;
   bwhelp = FL /100;
   int leL = bwhelp*24/spectrum_span;
   float kHz;
   // on CW we use the uper for the LP fc and the lower for the HP fc
   if (M != 5)
	    kHz = (FU + FL) / 1000.0;   
   else
	    kHz = (FU - MIN_BW_CW) / 1000.0;   
   
      
   switch (M) {
   case 0: //AM
       tft.print("AM"); 
 	   pos_centre_f = 120;
       break;
   case 3: //DSB    
       tft.print("DSB"); 
 	   pos_centre_f = 120;
       break;
   case 4: //StereoAM     
       tft.print("SAM"); 
 	   pos_centre_f = 120;
       break;  
   case 2: //LSB      
       tft.print("LSB"); 
 	   pos_centre_f = 135;
       break;
   case 1:  //USB
       tft.print("USB"); 
 	   pos_centre_f = 105;
       break;
   case 5: //CW     
       tft.print("CW"); 
 	   pos_centre_f = 133;
       break;
  default: 
 	  break;
} // end switch
      //print bandwidth ! 
	  tft.fillRect(100, pos_y_mainmenu, 45, 8,BLACK);	
      tft.setCursor(104, pos_y_mainmenu);	 
	  if (kHz < 10)
		sprintf(string,"%02.1fk",kHz);
	  else 
		 sprintf(string,"%02.0fk",kHz);
      tft.print(string);
      tft.setTextColor(WHITE); // set text color to white for other print routines not to get confused ;-)
      // draw upper sideband indicator
      tft.drawFastHLine(pos_centre_f, pos_spec+1, leU, WHITE);
      tft.drawFastHLine(pos_centre_f, pos_spec+2, leU, WHITE);
      tft.drawFastHLine(pos_centre_f, pos_spec+3, leU, WHITE);
      tft.drawFastHLine(pos_centre_f, pos_spec, leU, WHITE);
      // draw lower sideband indicator   
      left = pos_centre_f - leL; 
      tft.drawFastHLine(left+1, pos_spec+1, leL, WHITE);
      tft.drawFastHLine(left+1, pos_spec+2, leL, WHITE);
      tft.drawFastHLine(left+1,pos_spec+3, leL, WHITE);
      tft.drawFastHLine(left+1,pos_spec, leL, WHITE);
	  
	  //tft.fillRect(0,pos_spec,240,2,BLUE);
	  tft.fillRect(pos_centre_f  + 240/spectrum_span * 10, pos_spec, 2, 3, WHITE); // erase old string
	  tft.fillRect(pos_centre_f  + 240/spectrum_span * 20, pos_spec, 2, 4, WHITE); // erase old string
	  tft.fillRect(pos_centre_f  + 240/spectrum_span * 30, pos_spec, 2, 3, WHITE); // erase old string
	  tft.fillRect(pos_centre_f  + 240/spectrum_span * 40, pos_spec, 2, 3, WHITE); // erase old string
	  tft.fillRect(pos_centre_f  - 240/spectrum_span * 10, pos_spec, 2, 3, WHITE); // erase old string
	  tft.fillRect(pos_centre_f  - 240/spectrum_span * 20, pos_spec, 2, 4, WHITE); // erase old string
	  tft.fillRect(pos_centre_f  - 240/spectrum_span * 30, pos_spec, 2, 3, WHITE); // erase old string	
	  tft.fillRect(pos_centre_f  - 240/spectrum_span * 40, pos_spec, 2, 4, WHITE); // erase old string
	  tft.fillRect(pos_centre_f  - 240/spectrum_span * 50, pos_spec, 2, 3, WHITE); // erase old string
	//  tft.fillRect(pos_centre_f  - 240/spectrum_span * 60, pos_spec, 2, 4, WHITE); // erase old string
}  

//****************************************************************************************
// print tune step is showed a tthe bottom line.
// #define TUNE_STEP1   100000    	// 100
// #define TUNE_STEP2   1000  		// 1khz
// #define TUNE_STEP3   100  		// 1khz
// #define TUNE_STEP4   10    		// 10Hz
//
//
//****************************************************************************************
void show_tunestep(int tunestep) {  // show band
if (tunestep==1)
	{
	tft.fillRect(pos_x_frequency + font_width * (8) + 14 ,pos_y_frequency+font_width+7, 20,3,GREEN); // delete old digit
	tft.fillRect(pos_x_frequency + font_width * (7) + 14 ,pos_y_frequency+font_width+7, 20,3,BLACK); // delete old digit
	tft.fillRect(pos_x_frequency + font_width * (6) + 14 ,pos_y_frequency+font_width+7, 20,3,BLACK); // delete old digit
	tft.fillRect(pos_x_frequency + font_width * (5) + 7 ,pos_y_frequency+font_width+7, 20,3,BLACK); // delete old digit
	tft.fillRect(pos_x_frequency + font_width * (3) + 7 ,pos_y_frequency+font_width+7, 20,3,BLACK); // delete old digit
	}
if (tunestep==10)
	{
	tft.fillRect(pos_x_frequency + font_width * (8) + 14 ,pos_y_frequency+font_width+7, 20,3,BLACK); // delete old digit
	tft.fillRect(pos_x_frequency + font_width * (7) + 14 ,pos_y_frequency+font_width+7, 20,3,GREEN); // delete old digit
	tft.fillRect(pos_x_frequency + font_width * (6) + 14 ,pos_y_frequency+font_width+7, 20,3,BLACK); // delete old digit
	tft.fillRect(pos_x_frequency + font_width * (5) + 7 ,pos_y_frequency+font_width+7, 20,3,BLACK); // delete old digit
	tft.fillRect(pos_x_frequency + font_width * (3) + 7 ,pos_y_frequency+font_width+7, 20,3,BLACK); // delete old digit
	}
if (tunestep==100)
	{
	tft.fillRect(pos_x_frequency + font_width * (8) + 14 ,pos_y_frequency+font_width+7, 20,3,BLACK); // delete old digit
	tft.fillRect(pos_x_frequency + font_width * (6) + 14 ,pos_y_frequency+font_width+7, 20,3,GREEN); // delete old digit
	tft.fillRect(pos_x_frequency + font_width * (7) + 14 ,pos_y_frequency+font_width+7, 20,3,BLACK); // delete old digit
 	tft.fillRect(pos_x_frequency + font_width * (5) + 7 ,pos_y_frequency+font_width+7, 20,3,BLACK); // delete old digit
	tft.fillRect(pos_x_frequency + font_width * (3) + 7 ,pos_y_frequency+font_width+7, 20,3,BLACK); // delete old digit
	}
if (tunestep==1000){	
	tft.fillRect(pos_x_frequency + font_width * (8) + 14 ,pos_y_frequency+font_width+7, 20,3,BLACK); // delete old digit
	tft.fillRect(pos_x_frequency + font_width * (5) + 7 ,pos_y_frequency+font_width+7, 20,3,GREEN); // delete old digit
	tft.fillRect(pos_x_frequency + font_width * (6) + 14 ,pos_y_frequency+font_width+7, 20,3,BLACK); // delete old digit
	tft.fillRect(pos_x_frequency + font_width * (7) + 14 ,pos_y_frequency+font_width+7, 20,3,BLACK); // delete old digit
	tft.fillRect(pos_x_frequency + font_width * (3) + 7 ,pos_y_frequency+font_width+7, 20,3,BLACK); // delete old digit
	}
if (tunestep==100000){
		tft.fillRect(pos_x_frequency + font_width * (8) + 14 ,pos_y_frequency+font_width+7, 20,3,BLACK); // delete old digit
	tft.fillRect(pos_x_frequency + font_width * (3) + 7 ,pos_y_frequency+font_width+7, 20,3,GREEN); // delete old digit
	tft.fillRect(pos_x_frequency + font_width * (5) + 7 ,pos_y_frequency+font_width+7, 20,3,BLACK); // delete old digit
	tft.fillRect(pos_x_frequency + font_width * (6) + 14 ,pos_y_frequency+font_width+7, 20,3,BLACK); // delete old digit
	tft.fillRect(pos_x_frequency + font_width * (7) + 14 ,pos_y_frequency+font_width+7, 20,3,BLACK); // delete old digit
	 
}
}

//****************************************************************************************
// show yellow arrow in spectrum diagram
// to much code, needs to be adapted
//
//
//****************************************************************************************
void show_notch(int notchF, int MODE) {
  // pos_centre_f is the x position of the Rx centre
  // pos is the y position of the spectrum display 
  // notch display should be at x = pos_centre_f +- notch frequency and y = 20 
  // LSB: 
   switch (MODE) 
   {
          case 0: //AM
        
        	  pos_centre_f = 120;
              break;
          case 3: //DSB    
        
        	  pos_centre_f = 120;
              break;
          case 4: //StereoAM     
        
        	  pos_centre_f = 120;
              break;  
          case 2: //LSB      
        
        	  pos_centre_f = 135;
              break;
          case 1:  //USB
        
        	  pos_centre_f = 105;
              break;
         default: 
        	  break;
   }
		  pos_centre_f+=1; // = pos_centre_f + 1;
          // delete old indicator
          tft.drawFastVLine(pos_centre_f + 1 + 240/spectrum_span * oldnotchF / 1000, pos_spec, notchL, BLUE);
          tft.drawFastVLine(pos_centre_f + 240/spectrum_span * oldnotchF / 1000, pos_spec, notchL, BLUE);
          tft.drawFastVLine(pos_centre_f -1 + 240/spectrum_span * oldnotchF / 1000, pos_spec, notchL, BLUE);    
          tft.drawFastVLine(pos_centre_f + 240/spectrum_span * oldnotchF / 1000, pos_spec+notchL + 4, 2, BLUE);

          tft.drawFastVLine(pos_centre_f +1 - 240/spectrum_span * oldnotchF / 1000, pos_spec, notchL, BLUE);
          tft.drawFastVLine(pos_centre_f - 240/spectrum_span * oldnotchF / 1000, pos_spec, notchL, BLUE);
          tft.drawFastVLine(pos_centre_f -1 - 240/spectrum_span * oldnotchF / 1000, pos_spec, notchL, BLUE);   
          tft.drawFastVLine(pos_centre_f - 240/spectrum_span * oldnotchF / 1000, pos_spec+notchL + 4, 2, BLUE);
     

      if (notchF >= 400 || notchF <= -400) {
          // draw new indicator according to mode
      switch (MODE)  {
          case 2: //modeLSB:
          tft.drawFastVLine(pos_centre_f + 1 - 240/spectrum_span * notchF / -1000, pos_spec, notchL, notchColour);
          tft.drawFastVLine(pos_centre_f - 240/spectrum_span * notchF / -1000, pos_spec, notchL, notchColour);
          tft.drawFastVLine(pos_centre_f -1 - 240/spectrum_span * notchF / -1000, pos_spec, notchL, notchColour);     
          tft.drawFastVLine(pos_centre_f - 240/spectrum_span * notchF / -1000, pos_spec+notchL + 4, 2, notchColour);
          break;
          case 1: //modeUSB:
          tft.drawFastVLine(pos_centre_f +1 + 240/spectrum_span * notchF / 1000, pos_spec, notchL, notchColour);
          tft.drawFastVLine(pos_centre_f + 240/spectrum_span * notchF / 1000, pos_spec, notchL, notchColour);
          tft.drawFastVLine(pos_centre_f -1 + 240/spectrum_span * notchF / 1000, pos_spec, notchL, notchColour);    
          tft.drawFastVLine(pos_centre_f + 240/spectrum_span * notchF / 1000, pos_spec+notchL + 4, 2, notchColour);
          break;
          case 0: // modeAM:
          tft.drawFastVLine(pos_centre_f + 1 + 240/spectrum_span * notchF / 1000, pos_spec, notchL, notchColour);
          tft.drawFastVLine(pos_centre_f + 240/spectrum_span * notchF / 1000, pos_spec, notchL, notchColour);
          tft.drawFastVLine(pos_centre_f - 1 + 240/spectrum_span * notchF / 1000, pos_spec, notchL, notchColour);      
          tft.drawFastVLine(pos_centre_f + 240/spectrum_span * notchF / 1000, pos_spec+notchL + 4, 2, notchColour);

          tft.drawFastVLine(pos_centre_f + 1 - 240/spectrum_span * notchF / 1000, pos_spec, notchL, notchColour);
          tft.drawFastVLine(pos_centre_f - 240/spectrum_span * notchF / 1000, pos_spec, notchL, notchColour);
          tft.drawFastVLine(pos_centre_f - 1 - 240/spectrum_span * notchF / 1000, pos_spec, notchL, notchColour);   
          tft.drawFastVLine(pos_centre_f - 240/spectrum_span * notchF / 1000, pos_spec+notchL + 4, 2, notchColour);
          break;
          case 3: //modeDSB:
          case 4: //modeStereoAM:
          if (notchF <=-400) {
          tft.drawFastVLine(pos_centre_f + 1 - 240/spectrum_span * notchF / -1000, notchpos, notchL, notchColour);
          tft.drawFastVLine(pos_centre_f - 240/spectrum_span * notchF / -1000, notchpos, notchL, notchColour);
          tft.drawFastVLine(pos_centre_f - 1 - 240/spectrum_span * notchF / -1000, notchpos, notchL, notchColour);
          tft.drawFastVLine(pos_centre_f - 240/spectrum_span * notchF / -1000, notchpos+notchL + 4, 2, notchColour);
          }
          if (notchF >=400) {
          tft.drawFastVLine(pos_centre_f + 1 + 240/spectrum_span * notchF / 1000, notchpos, notchL, notchColour);
          tft.drawFastVLine(pos_centre_f + 240/spectrum_span * notchF / 1000, notchpos, notchL, notchColour);
           tft.drawFastVLine(pos_centre_f - 1 + 240/spectrum_span * notchF / 1000, notchpos, notchL, notchColour);    
          tft.drawFastVLine(pos_centre_f + 240/spectrum_span * notchF / 1000, notchpos+notchL + 4, 2, notchColour);
          }
          break;
		}
      }
      oldnotchF = notchF;
      pos_centre_f-=1; // = pos_centre_f - 1;
	  
  } // end void show_notch

//****************************************************************************************
// exctract digits
//
//
//****************************************************************************************
int ExtractDigit(long int n, int k) {
        switch (k) {
              case 0: return n%10;
              case 1: return n/10%10;
              case 2: return n/100%10;
              case 3: return n/1000%10;
              case 4: return n/10000%10;
              case 5: return n/100000%10;
              case 6: return n/1000000%10;
              case 7: return n/10000000%10;
              case 8: return n/100000000%10;
			  default: return n;
        }
}
//****************************************************************************************
// show frequency
//
//
//****************************************************************************************
void show_frequency(uint32_t freq) { // show frequency
   
    tft.setTextColor(GREEN);
    uint8_t zaehler, high=30;
    uint8_t digits[10];
    zaehler = 8;
    tft.setFont(Digital7_32);
	
          while (zaehler--) {
              digits[zaehler] = ExtractDigit (freq, zaehler);
			}
		tft.setTextSize(2);

		zaehler = 8;
        while (zaehler--) { // counts from 7 to 0
              if (zaehler < 6) sch = 7; // (khz)
              if (zaehler < 3) sch = 14; // (Hz)
          if (digits[zaehler] != digits_old[zaehler] || !freq_flag) { // digit has changed (or frequency is displayed for the first time after power on)
              if (zaehler == 7) {
                     sch = 0;
                     tft.setCursor(pos_x_frequency + font_width * (8-zaehler) + sch,pos_y_frequency); // set print position
                     tft.fillRect(pos_x_frequency + font_width * (8-zaehler) + sch,pos_y_frequency, font_width,high,BLACK); // delete old digit
					 //tft.setTextColor(((((0x0C)&0xF8)<<8)|(((0x0C)&0xFC)<<3)|((0x0C)>>3)));
					 //tft.print(8);
					 //tft.setTextColor(GREEN);
                     if (digits[7] != 0) tft.print(digits[zaehler]); // write new digit in white
              }
              if (zaehler == 6) {
                            sch = 0;
                            tft.setCursor(pos_x_frequency + font_width * (8-zaehler) + sch,pos_y_frequency); // set print position
                            tft.fillRect(pos_x_frequency + font_width * (8-zaehler) + sch,pos_y_frequency, font_width,high,BLACK); // delete old digit
							//tft.setTextColor(((((0x0C)&0xF8)<<8)|(((0x0C)&0xFC)<<3)|((0x0C)>>3)));
							//tft.print(8);
							//tft.setTextColor(GREEN);
                            if (digits[6]!=0 || digits[7] != 0) tft.print(digits[zaehler]); // write new digit in white
              }
               if (zaehler == 5) {
                            sch = 7;
                            tft.setCursor(pos_x_frequency + font_width * (8-zaehler) + sch,pos_y_frequency); // set print position
                            tft.fillRect(pos_x_frequency + font_width * (8-zaehler) + sch,pos_y_frequency, font_width,high,BLACK); // delete old digit
							//tft.setTextColor(((((0x0C)&0xF8)<<8)|(((0x0C)&0xFC)<<3)|((0x0C)>>3)));
							//tft.print(8);
							//tft.setTextColor(GREEN);
                            if (digits[5] != 0 || digits[6]!=0 || digits[7] != 0) tft.print(digits[zaehler]); // write new digit in white
              }
              
              if (zaehler < 5) { 
              // print the digit
              tft.setCursor(pos_x_frequency + font_width * (8-zaehler) + sch,pos_y_frequency); // set print position
              tft.fillRect(pos_x_frequency + font_width * (8-zaehler) + sch,pos_y_frequency, font_width,high,BLACK); // delete old digit
			  //tft.setTextColor(((((0x0C)&0xF8)<<8)|(((0x0C)&0xFC)<<3)|((0x0C)>>3)));
			  //tft.print(8);
			  //tft.setTextColor(GREEN);
              tft.print(digits[zaehler]); // write new digit in white
              }
              digits_old[zaehler] = digits[zaehler]; 
        }
        }
	  //Reset Font type
	  tft.setFont(Arial_8_Bold);
      tft.setTextSize(1);
	  
      if (digits[7] == 0 && digits[6] == 0)
		      // fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
              tft.fillRect(pos_x_frequency + font_width * 3,pos_y_frequency + 26, 3, 3, BLACK);
      else    tft.fillRect(pos_x_frequency + font_width * 3,pos_y_frequency + 26, 3, 3, GREEN);
		
		tft.fillRect(pos_x_frequency + font_width * 7 -16, pos_y_frequency + 26, 3, 3, GREEN);
      if (!freq_flag) 
            tft.setCursor(pos_x_frequency + font_width * 9 + 16,pos_y_frequency + 7); // set print position      
      freq_flag = 1;
      tft.setTextColor(WHITE);

} // END VOID SHOW-FREQUENCY



