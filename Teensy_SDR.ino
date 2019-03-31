/* Teensy SDR

 
 * the Teensy audio shield is used to capture and generate 16 bit audio
 * audio processing is done by the Teensy 3.2
 * simple UI runs on a 160x128 color TFT display - AdaFruit or Banggood knockoff which has a different LCD controller
 * Copyright (C) 2014, 2015  Rich Heslip rheslip@hotmail.com
 * History:
 * 4/14 initial version by R Heslip VE3MKC
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#define ver "v 2018.12.30"
#include <Time.h>
#include <TimeLib.h>
#include <Metro.h>
#include <Audio.h>
#include <Wire.h>
//#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#include <EEPROM.h>
#include <si5351.h>
#include <Bounce.h>
#include "font_Arial.h"
#include "font_ArialBold.h"
#include <ILI9341_t3.h>
#include <SPI.h>
#include "analyze_fft256iq.h" 
#include "AM_demod.h"
#include "AM_sync_demod.h"
#include "display.h"
#include "iir_18_96ks.h"
#include "iir_highpass_96ks.h"
#include "iir_23_96ks.h"
#include "iir_26_96ks.h"
#include "iir_28_96ks.h"
#include "iir_33_96ks.h"
#include "iir_36_96ks.h"
#include "iir_40_96ks.h"
#include "iir_44_96ks.h"
#include "iir_54_96ks.h"
#include "iir_60_96ks.h"
#include "iir_65_96ks.h"
#include "iir_80_96ks.h"
#include "iir_110_96ks.h"
//#include <TeensyAudioPlotter.h>

#define BLACK   ILI9341_BLACK
#define WHITE   ILI9341_WHITE
#define RED     ILI9341_RED
#define GREEN   ILI9341_GREEN
#define YELLOW  ILI9341_YELLOW
#define BLUE    ILI9341_BLUE
#define LGREEN	0x03E0
#define LBLUE   0xAF9F
#define AUDIO_SAMPLE_RATE_EXACT 96000

//void sinewave(void); 			
void setI2SFreq(int freq);			
void tune_SAM_PLL(void);		
extern void setup_display(void);
extern void init_display(void);
extern void show_spectrum(float line_gain, float LPFcoeff, int M, long int FU, long int FL );// spectrum display draw
extern void show_waterfall(void); 								// waterfall display
extern void show_rfg(String rfg);  								// show band
extern void show_agc(uint8_t agc); 
extern void highlight_agc(uint8_t on_off);
extern void highlight_rfg(uint8_t on_off);
extern void highlight_bw(uint8_t on_off);
extern void show_frequency(uint32_t freq);  					// agcshow frequency
extern void show_tunestep (int tunestep);
extern void show_bandwidth (int M, long int FU, long int FL);
extern void show_notch (int notchF, int MODE);
						
time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}
// band selection stuff
struct band {
  uint32_t freq; 		// frequency in Hz
  String name; 			// name of band
  int mode; 			// sideband & bandwidth
  int bandwidthU;
  int bandwidthL;
  int RFgain; 
};

// Oscillator 
long calibration_factor = 9983199;
long calibration_constant = 3500;
unsigned long long hilfsf;

#define F_MAX 36000000
#define F_MIN 100000
#define BAND_LW  0   
#define BAND_MW1   1
#define BAND_90M  2
#define BAND_75M  3   
#define BAND_60M  4
#define BAND_49M  5
#define BAND_41M  6
#define BAND_31M  7
#define BAND_25M  8
#define BAND_22M  9
#define BAND_19M  10
#define BAND_16M  11
#define BAND_15M  12
#define BAND_13M  13
#define BAND_10M  14
#define FIRST_BAND BAND_LW
#define LAST_BAND  BAND_10M
#define NUM_BANDS  15
// radio operation mode defines used for filter selections etc
#define modeAM 0
#define modeUSB 1
#define modeLSB 2
#define modeDSB 3
#define modeSAM 4
#define modeCW 5
#define firstmode modeAM
#define lastmode modeCW
#define startmode modeSAM
#define MAX_BANDWIDTH 8000
#define MIN_BANDWIDTH 1400
#define MAX_BW_CW     2500
#define MIN_BW_CW     1000

uint32_t 		IF_FREQ = 6000;     // IF Oscillator frequency 96ks

// f, band, mode, bandwidth, RFgain
struct band bands[NUM_BANDS] = {
  251000-IF_FREQ,"LW", modeAM, 3600,3600,120,
  1215000-IF_FREQ,"MW1",  modeSAM, 3600,3600,100,
  3500000-IF_FREQ,"90M",  modeAM, 3600,3600,120,
  3782000-IF_FREQ,"75M",  modeUSB, 2800,2800,130,
  4750000-IF_FREQ,"60M",  modeAM, 3600,3600,130,
  5960000-IF_FREQ,"49M",  modeSAM, 3600,3600,130,
  7330000-IF_FREQ,"41M",  modeLSB, 3600,3600,130,
  9420000-IF_FREQ,"31M",  modeAM, 3600,3600,130,
  11735000-IF_FREQ,"25M", modeAM, 3600,3600,130,
  13570000-IF_FREQ,"22M", modeUSB, 2800,2800,130,
  15140000-IF_FREQ,"19M", modeAM, 3600,3600,150,
  17480000-IF_FREQ,"16M", modeAM, 3600,3600,150,
  18900000-IF_FREQ,"15M", modeUSB, 2800,2800,150,
  21450000-IF_FREQ,"13M", modeUSB, 2800,2800,150,
  28300000-IF_FREQ,"10M", modeUSB, 2800,2800,150
};
#define STARTUP_BAND  BAND_41M    
int band;

#define Si_5351_clock  SI5351_CLK2
#define Si_5351_crystal 27000000

//***************************************************************************
// SPI connections for 2.8" display
//***************************************************************************
#define TFT_DC          20
#define TFT_CS          21
#define TFT_RST         35  // 255 = unused. connect to 3.3V
#define TFT_MOSI        7
#define TFT_SCLK        14
#define TFT_MISO        12
ILI9341_t3 tft = ILI9341_t3(TFT_CS, TFT_DC, TFT_RST, TFT_MOSI, TFT_SCLK, TFT_MISO);
const int8_t BandDOWNSW 		= 39;   // band selector down
const int8_t ModeSW				= 33;  	// Switch form Volume Encoder
const int8_t MenuSW 			= 8;    // Switch from Menu Encoder 
const int8_t BandUPSW 			= 37; 	// band selector up 
const int8_t TuneSW 			= 38;   // switch tune step
const int8_t MainMenuScrollSW 	= 30; 	// Switch form Menu EncTune
const int8_t Bnd1 		= 26; 	// band selection pins 
const int8_t Bnd2		= 27; 	// band selection pins 
const int8_t Off_switch	= 24;   // Sense switch off device
const int8_t On_set		= 25;	// hold switched on
//Encoder
Encoder EncMenu(4,5);			// Optical Encoder connections, Freq.
Encoder EncTune(16,17);			// Optical Encoder connections, Menu.
Encoder EncVolume(2,1);			// Optical Encoder connections, Freq.
// Estebans defs

bool 		AM_pass = 1; 		// 1 = Am demodulation on; 0 = off

// definitions for cursor positions display setup
const int16_t pos_x_date = 80;
const int16_t pos_y_date = 304; 		
const int16_t pos_x_time = 0;
const int16_t pos_y_time = 304;		
// Constants from display.c
const int16_t pos_x_frequency  = 15; 
const int16_t pos_y_frequency = 6; 
const int16_t menu_t_high = 16;
const int16_t pos_x_menu = 100;
const int16_t pos_x_menu_value_room = 80;
const int16_t pos_x_menu_value_start = 180;
const int16_t pos_y_menu = 304;

// position of spectrum display, has to be < 124 --> also to be defined in display.cpp
#define 	pos 		50 		
#define  	BACKLIGHT  	0  		// backlight control signal
#define USE_EEPROM				// not use EEPROM
#define AUDIO_STATS   			// shows audio library CPU utilization etc on serial console

// clock generator
Si5351 si5351;
#define MASTER_CLK_MULT  4  	// QSD frontend requires 4x clock

// various timers
Metro five_sec=Metro(5000); 	// Set up a 5 second Metro
Metro one_sec = Metro(1000);
Metro ms_100 = Metro(97); 		// Set up a 100ms Metro
Metro ms_300 = Metro(297); 		// Set up a 100ms Metro
Metro ms_50 = Metro(53);  		// Set up a 50ms Metro for polling switches
Metro ms_10 = Metro(11);

#define TUNE_STEP1   100000    	// 100k
#define TUNE_STEP2   1000  		// 1khz
#define TUNE_STEP3   100  		// 100hz
#define TUNE_STEP4   10    		// 10Hz
#define TUNE_STEP5   1    		// 1Hz
#define first_tunehelp 2
#define last_tunehelp 5
int tunehelp = 1;
int tunestep = TUNE_STEP4;
// Menus !
#define TUNING 				0
#define BASS 				1
#define TREBLE 				2
#define NOTCH 				3
#define CALIBRATIONCONSTANT 4
#define IQADJUST 			5
#define IQPHASEADJUST 		6
#define SAVETOEEPROM 		7
#define LOADFROMEEPROM 		8
#define TIMEADJUST 			9
#define DATEADJUST 			10
#define CALIBRATIONFACTOR 	11
#define FFTWINDOW 			12
#define LPFSPECTRUM 		13
#define RESETCODEC 			14
#define SNAP 				15
#define BW_step 			100
#define first_menu 			TUNING
#define last_menu 			SNAP
#define start_menu 			TUNING
int Menu;
String menu_text = "Tuning";

// Menu2 !
#define BWADJUST 		3
#define RFGAIN 			2
#define AGC 			1
#define first_menu2 	TUNING
#define last_menu2 		BWADJUST 					
#define FAST 			3	
#define MEDIUM 			2
#define SLOW 			1 
#define OFF 			0

int Menu2; 
float vol;
float bass = 0.3;
float bass_help = bass * 100;
int AGC_Mode;
int RFgain_Auto =0;
float  RFgain_automatic=7;
float treble = 0.8;
float treble_help = treble *100; 
int notchF = 399; 							// if 200, notch is off, if >= 400, notch is on
int notchQ = 10;
float LPFcoeff = 0.4; 						// used for low pass filtering the spectrum display
float LPFcoeffhelp = LPFcoeff * 100;
int passbandBW = 0;
int cnt;
int average_bin1, average_bin2, average_bin3;
int maxbin = 0;								// Values for SAM
int Lbin = 0;
int Ubin = 0;
int links = 0;
int rechts = 0;
float delta = 0;
int average_delta;
float binBW = AUDIO_SAMPLE_RATE_EXACT / 256.0;
float bin1 = 0;
float bin2 = 0;
float bin3 = 0;
int posbin = 127; 							// for Rx 
int helpmin; 								// definitions for time and date adjust - Menu
int helphour;
int helpday;
int helpmonth;
int helpyear;
int helpsec;
uint8_t hour10_old;
uint8_t hour1_old;
uint8_t minute10_old;
uint8_t minute1_old;
uint8_t second10_old;
uint8_t second1_old;
bool timeflag = 0;
float Q_gain;
float I_gain;
float I_help;
float Q_help;
float Q_in_I;
float Q_in_I_help;
float I_in_Q;
float I_in_Q_help;
float IQcounter;

#define  INITIAL_VOLUME 0.8  // 0-1.0 output volume on startup
int Window_FFT = 7; // BlackNutt
String FFT_STRING;

//**************************************************************/
// Setup Tx Variables
//
//
//**************************************************************/
const int MicIn = AUDIO_INPUT_MIC;
#define  MIC_GAIN      30      // mic gain in db 
bool TxOn = 0;

/* ############################################################
 *  Signalflow
 *  
 * ############################################################
 */
 
AudioInputI2S    	  audioinput;  // Audio Shield: mic or line-in
AudioMixer4      	  audio_in_I;
AudioMixer4      	  audio_in_Q;
AMDemod          	  AM;
AMDemodSync			  SAM;
AudioFilterBiquad     biquad_I, biquad_I2;
AudioFilterBiquad     biquad_Q, biquad_Q2;
AudioSynthWaveformSineHires SinLO, CosLO;
AudioFilterBiquad     AM_highpass;
// IIR filters (= biquads)
AudioFilterBiquad     biquad1;
AudioFilterBiquad     biquad3;
AudioAnalyzeFFT256IQ  myFFT; 			// Spectrum Display complex FFT
AudioMixer4           USB;
AudioMixer4           LSB;
AudioMixer4			  DSB;
AudioMixer4           ModeMixer;
AudioMixer4			  MixerAM;
AudioMixer4           MixGain;
AudioEffectMultiply	  MIX_I, MIX_Q;
AudioAnalyzePeak	  adc_input_voltage;
AudioAnalyzePeak	  AGCMeter;
AudioOutputI2S        audioOutput;   	// Audio Shield: headphones & line-out
AudioControlSGTL5000 audioShield;  		// Create an object to control the audio shield.

// Create Audio connections to build a software defined Radio Receiver
// audio input and IQ amplitude and phase correction
//AudioConnection z0(audioinput, 0, PrintAudio1, 0);
AudioConnection c5(audioinput, 1, audio_in_I, 0);
AudioConnection c6(audioinput, 0, audio_in_Q, 0);
AudioConnection c7(audio_in_I, 0, audio_in_Q, 1);
AudioConnection c8(audio_in_Q, 0, audio_in_I, 1); 
AudioConnection c23(audio_in_I, 0, myFFT, 0);
AudioConnection c24(audio_in_Q, 0, myFFT, 1);
AudioConnection n01(audio_in_I, 0, myFFT, 0);
AudioConnection n02(audio_in_Q, 0, myFFT, 1);
AudioConnection n1(audio_in_I, 0, biquad_I, 0);
AudioConnection n2(audio_in_Q, 0, biquad_Q, 0);
AudioConnection n1b(biquad_I, 0, biquad_I2, 0);
AudioConnection n2b(biquad_Q, 0, biquad_Q2, 0);
AudioConnection c21(biquad_I2, 0, AM, 0);
AudioConnection c22(biquad_Q2, 0, AM, 1);
AudioConnection c222(biquad_Q2, 0, SAM, 1);
AudioConnection c223(biquad_I2, 0, SAM, 0);
AudioConnection n5b(AM,0,MixerAM,0);
AudioConnection n5c(SAM,0,MixerAM,1);
AudioConnection c211(MixerAM, 0 , AM_highpass,0 );
AudioConnection n3(biquad_I2, 0, MIX_I, 0);
AudioConnection n4(biquad_Q2, 0, MIX_Q, 0);
AudioConnection n4b(biquad_Q2 ,0, DSB, 0);
AudioConnection n4c(biquad_I2, 0, DSB, 1);
AudioConnection n5(SinLO, 0, MIX_I, 1);
AudioConnection n6(CosLO, 0, MIX_Q, 1);
AudioConnection n7(MIX_I, 0, USB, 0);
AudioConnection n71(MIX_Q, 0, USB, 1);
AudioConnection n8(MIX_I, 0, LSB, 0);
AudioConnection n81(MIX_Q, 0, LSB, 1);
AudioConnection c28a(USB, 0, ModeMixer, 0); // right channel
AudioConnection c28b(LSB, 0, ModeMixer, 1); // left channel
AudioConnection c28c(AM_highpass, 0, ModeMixer, 2); // left channel
AudioConnection c28d(DSB, 0, ModeMixer, 3); // left channel
AudioConnection c29a(ModeMixer, 0, biquad1 , 0);
AudioConnection c29b(biquad1, 0, biquad3, 0);
AudioConnection c30a(biquad3,0, MixGain, 0);
AudioConnection c31c(MixGain,0, AGCMeter, 0);
AudioConnection c40(audioinput,0, adc_input_voltage, 0);
AudioConnection c32(MixGain,0, audioOutput, 0);

//TeensyAudioPlotter plotter;

//Changes pins_teensy.c
// 		*config = PORT_PCR_SRE | PORT_PCR_MUX(1);
//		*config = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
// 		*config = PORT_PCR_MUX(1) | PORT_PCR_PE | PORT_PCR_PS ;
// 		*config = PORT_PCR_MUX(1) | PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_PFE;
//Changes to sgtl5000.c
//     	return write(CHIP_ANA_ADC_CTRL, 0x0100 |(left << 4) | right);
//Changes si5351.c
// 		Wire.setClock(400000);
//---------------------------------------------------------------------------------------------------------

void setup() 
{
	Q_gain=1000;
	vol=0.8;
	I_gain=1000;
	I_help = 1.0;
	Q_help = 1.0;
	Q_in_I = 0.000;
	Q_in_I_help = 0;
	I_in_Q = 0.000;
	I_in_Q_help = 0;
	IQcounter = 0;
	AGC_Mode=2;
	RFgain_Auto=1;
	cnt=0;
	Menu = TUNING;
	Menu2 = TUNING;
	setSyncProvider(getTeensy3Time);
	pinMode(ModeSW, INPUT_PULLUP);  		// USB/LSB switch
	pinMode(BandUPSW, INPUT_PULLUP);  	
	pinMode(BandDOWNSW, INPUT_PULLUP);  	 
	pinMode(TuneSW, INPUT_PULLUP);  		// tuning rate = high
	pinMode(MenuSW, INPUT_PULLUP);  	

	pinMode(MainMenuScrollSW, INPUT_PULLUP);  		
	pinMode(Bnd1, OUTPUT);  				// LPF switches
	pinMode(Bnd2, OUTPUT);  				
	pinMode(On_set, OUTPUT); 		 
	
	digitalWrite (On_set, HIGH);			// Hold switch on
	pinMode(Off_switch, INPUT_PULLUP);  	
	AudioMemory(50);	
	init_display();
	// here startup message
	tft.setCursor(pos_x_time,pos_y_time);
	tft.setTextColor(RED);
	tft.setTextSize(1);
	tft.print (ver); 
	tft.setCursor(40, 120);
	tft.setFont(Arial_20_Bold);
	tft.print("SSR 2");
	tft.setCursor(40, 220);
	tft.setFont(Arial_14_Bold);
	tft.setTextSize(3);
	tft.print("by dj7jbh");
	delay (2000);  
	
	// get saved frequencies, modes and calibration from EEPROM
	eepromload(); 
	tft.setCursor(120,pos_y_time);
	tft.setFont(Arial_8_Bold);
	tft.setTextSize(3);
	tft.print("Load EEPROM...");
	delay (1000);  
	// Set display
	// Clear all
	tft.fillRect(0, 0, 240, 320,BLACK);
	setup_display();
	tft.setTextSize(1);
	tft.setTextColor(WHITE);	
	tft.setFont(Arial_8_Bold);

  	Serial.begin(115200); 			// debug console
	#ifdef DEBUG
	// show that if not console connected we stuck here
	tft.setCursor(0, 105);
    tft.print ("#define DEBUG");
	while (!Serial) ;				// wait for connection
	Serial.println("initializing");
	#endif   
	
	band = STARTUP_BAND;
	// set up initial band and frequency
	// show_band(bands[band].name);
	show_tunestep(tunestep);  
	#ifdef DEBUG
	while (!Serial) ; // wait for connection
	Serial.println("start audioshield init");
	#endif 
	// Enable the audio shield and set the output volume.
	audioShield.enable();
	audioShield.dacVolumeRamp();
	audioShield.volume(INITIAL_VOLUME);
	// enables the DAP chain of the codec post audio processing before the headphone out
	audioShield.audioPostProcessorEnable(); 	
	//dacVolume(0.5) reduces resolution to 15 bits while dacVolume(0.125) reduces it to 13, and so on.
	audioShield.dacVolume(0.75);	
    audioShield.eqSelect (2); 
	// (float bass, float treble) in % -100 to +100
    audioShield.eqBands (bass, treble); 
    // RX mode uses line ins
	audioShield.inputSelect(AUDIO_INPUT_LINEIN); 
    // ++ very important to get rid of the near 0 Hz noise
	audioShield.adcHighPassFilterEnable();
	// 
	audioShield.muteLineout();

    setup_gain();
	// turn on amplifier
	MixGain.gain(0,1);       	// Adjust AGC gain
	MixGain.gain(1,1);       	// Adjust AGC gain	
    setup_mode(bands[band].mode);
	
    // set up the audio chain for reception
    setup_RX(bands[band].mode, bands[band].bandwidthU, bands[band].bandwidthL); 
    FFT_WINDOW_SET (Window_FFT);
    FFT_STRING = FFT_WINDOW_STRING (Window_FFT);  
    // Set up VFO
	si5351.init(SI5351_CRYSTAL_LOAD_8PF);
	//
    si5351.drive_strength(SI5351_CLK2, SI5351_DRIVE_4MA); 
    // Using CLK2 PLLB ist to be used, for CLK0 there must be PLLA
    si5351.set_pll(SI5351_PLL_FIXED, SI5351_PLLB);
    setfreq();
    delay(3);
	// Show AGC
	show_agc(AGC_Mode);		
	// Clear bottom command line
	tft.fillRect(0, pos_y_menu-4, 240, menu_t_high+4, BLACK); // erase old string	
	AudioNoInterrupts();   // Disable Audio while reconfiguring filters  
	
	// SSB demodulation Oscillators
	SinLO.amplitude(1.0);
	SinLO.frequency(6000.0);
	SinLO.phase(90);
	CosLO.amplitude(1.0);
	CosLO.frequency(6000.0);
	CosLO.phase(0);
	// Set the sampling frequency
	setI2SFreq(96000);
	delay(10); 
	AudioInterrupts();
    #ifdef DEBUG
    while (!Serial) ; // wait for connection
    Serial.println("init done");
    #endif 
	
	Wire.setClock(400000);
	//plotter.addLogger(PrintAudio1);

} // end void SETUP



void reset_codec ()
{
  AudioNoInterrupts();
  audioShield.disable();
  delay(10);
  audioShield.enable();
  delay(100);
  audioShield.inputSelect(AUDIO_INPUT_LINEIN);
  audioShield.lineInLevel((float)bands[band].RFgain/10, (float) bands[band].RFgain/10);
  audioShield.lineOutLevel(24); // 13 loudest, 31 lowest
  audioShield.audioPostProcessorEnable(); // enables the DAP chain of the codec post audio processing before the headphone out
  audioShield.eqSelect (2); // Tone Control
  audioShield.eqBands (bass, treble);
  audioShield.dacVolume(0.75);	
  audioShield.enhanceBassEnable();
  audioShield.volume(INITIAL_VOLUME);  
  AudioInterrupts();

}

/* #########################################################################
 *  
 * void set frequency and set up the filters
 *  
 * ######################################################################### 
 */
void setfreq () {
   

   hilfsf = bands[band].freq;
   hilfsf = (hilfsf*10000000) / calibration_factor;
   si5351.set_freq((uint32_t)(hilfsf*MASTER_CLK_MULT), SI5351_PLL_FIXED, SI5351_CLK2);
   
   int MO = bands[band].mode;
   
   if ((MO == modeAM)||(MO == modeSAM) || (MO == modeDSB))
		IF_FREQ = 0;
   if (MO == modeLSB)
		IF_FREQ = 6000.0;
   if (MO == modeUSB)
		IF_FREQ = -6000.0;		
	if (MO == modeCW)
		IF_FREQ = 6000.0;	
   
   show_frequency(bands[band].freq + IF_FREQ);  // frequency we are listening to, Rx above IF
 
//***************************************************************************
// Bandpass Filter switch
// Bnd2 Bnd1  Frequency Range
// 0	0     700khz  1.6MHz
// 0	1     1.6Mhz  4.16MHz
// 1	0     4.16MHz 11MHz
// 1    1     11MHz   29MHz
//***************************************************************************
   if ((bands[band].freq + IF_FREQ) < 1600000)  { 
     digitalWrite (Bnd1, LOW); digitalWrite (Bnd2, LOW);  
     } // end if
   if (((bands[band].freq + IF_FREQ) > 1600000) && ((bands[band].freq + IF_FREQ) < 4160000)) {
      digitalWrite (Bnd1, HIGH); digitalWrite (Bnd2, LOW);  
     } // end if
   if (((bands[band].freq + IF_FREQ) > 4160000) && ((bands[band].freq + IF_FREQ) < 11000000)) {
      digitalWrite (Bnd1, LOW); digitalWrite (Bnd2, HIGH);  
     } // end if
   if ((bands[band].freq + IF_FREQ) > 11000000) { 
      digitalWrite (Bnd1, HIGH); digitalWrite (Bnd2, HIGH);  
    } // end if   
 
} //end void setfreq

void clearname() {
        
     tft.fillRect(0, 104, 160, 10,BLACK);  
	 tft.setCursor(0, 105);
} // end void clearname
/* #########################################################################
 *  
 * save data to eeprom
 *  
 * ######################################################################### 
 */
void EEPROMSAVE() {
	 struct config_t {
	 long calibration_factor;
	 long calibration_constant;
	 uint32_t freq[NUM_BANDS];
	 int mode[NUM_BANDS];
	 int bwu[NUM_BANDS];
	 int bwl[NUM_BANDS];
	 int rfg[NUM_BANDS];
	 int band;
	 float I_ampl;
	 float Q_in_I;
	 float I_in_Q;
	 int Window_FFT;
	 float LPFcoeff;	 
	 int agc_m;
	 int gain_Auto;
	 int taps;
	 int tapsssb;
 } E; 

	E.calibration_factor = calibration_factor;
	E.band = band;
	E.calibration_constant = calibration_constant;
	for (int i=0; i< (NUM_BANDS); i++) 
	E.freq[i] = bands[i].freq;
	for (int i=0; i< (NUM_BANDS); i++)
	E.mode[i] = bands[i].mode;
	for (int i=0; i< (NUM_BANDS); i++)
	E.bwu[i] = bands[i].bandwidthU;
	for (int i=0; i< (NUM_BANDS); i++)
	E.bwl[i] = bands[i].bandwidthL;
	for (int i=0; i< (NUM_BANDS); i++)
	E.rfg[i] = bands[i].RFgain;
	E.I_ampl = I_help;
	E.Q_in_I = Q_in_I_help;
	E.I_in_Q = I_in_Q_help;
	E.Window_FFT = Window_FFT;
	E.LPFcoeff = LPFcoeff;
	E.agc_m = AGC_Mode;
	E.gain_Auto = RFgain_Auto;
	
	// Write dont work with overclock
	 
	 __disable_irq( );
    SMC_PMCTRL = SMC_PMCTRL_RUNM(0); // exit HSRUN mode
    while (SMC_PMSTAT == SMC_PMSTAT_HSRUN) ; // wait for !HSRUN
    __enable_irq( );
	
	eeprom_write_block (&E,0,sizeof(E));
	
	__disable_irq( );
    SMC_PMCTRL = SMC_PMCTRL_RUNM(3); // enter HSRUN mode
    while (SMC_PMSTAT != SMC_PMSTAT_HSRUN) ; // wait for HSRUN
    __enable_irq( );

} // end void eeProm SAVE 
/* #########################################################################
 *  STARTUP_BAND
 * read data to eeprom
 *  
 * ######################################################################### 
 */
void eepromload() {
#ifdef USE_EEPROM
	 struct config_t {
	 long calibration_factor;
	 long calibration_constant;
	 uint32_t freq[NUM_BANDS];
	 int mode[NUM_BANDS];
	 int bwu[NUM_BANDS];
	 int bwl[NUM_BANDS];
	 int rfg[NUM_BANDS];
	 int band;
	 float I_ampl;
	 float Q_in_I;
	 float I_in_Q;
	 int Window_FFT;
	 float LPFcoeff;	 
	 int agc_m;
	 int gain_Auto;
	 int taps;
	 int tapsssb;
	 
 } E; 

	eeprom_read_block(&E,0,sizeof(E));

	calibration_factor = E.calibration_factor;
	calibration_constant = E.calibration_constant;
	for (int i=0; i< (NUM_BANDS); i++) 
	bands[i].freq = E.freq[i];

	for (int i=0; i< (NUM_BANDS); i++)
	bands[i].mode = E.mode[i];

	for (int i=0; i< (NUM_BANDS); i++)
	bands[i].bandwidthU = E.bwu[i];

	for (int i=0; i< (NUM_BANDS); i++)
	bands[i].bandwidthL = E.bwl[i];

	for (int i=0; i< (NUM_BANDS); i++)
	bands[i].RFgain = E.rfg[i];
	band = E.band;

	I_help = E.I_ampl;
	Q_in_I_help = E.Q_in_I;
	I_in_Q_help = E.I_in_Q;
	Window_FFT = E.Window_FFT;
	LPFcoeff = E.LPFcoeff;
	AGC_Mode =E.agc_m;
	RFgain_Auto=E.gain_Auto ;
	
	#ifdef DEBUG	
	   Serial.print ("eeprom loaded"); 
	#endif
	
#endif //USE_EEPROM
} // end void eeProm LOAD 

void loop() 
{
  static uint8_t modesw_state=0;
  static uint8_t MainMenuScrollsw_state=0;
  static uint8_t Bandupsw_state=0;
  static uint8_t switchoffsw_state=0;
  static uint8_t Banddownsw_state=0;
  static uint8_t tunesw_state = 0, menusw_state=0; 
  static uint8_t turns = 64;
  static long encoder_pos=0, 
			  last_encoder_pos=0, 
			  encoder_pos1=0, 
			  last_encoder_pos1=0,
			  encoder_pos2=0, 
			  last_encoder_pos2=0;
  long 	encoder_change,  // EncTune = Frequency
		encoder_change1, // Menu 
		encoder_change2; // Volume


  char rfg[7];  
  
  encoder_pos1=EncTune.read();
  encoder_pos=EncMenu.read();
  encoder_pos2=EncVolume.read();
 
  // Switch off Receiver, but prior to that save data.
  if (!digitalRead(Off_switch))
  {	   
		tft.setTextColor(WHITE);
		tft.fillRect(pos_x_menu, pos_y_menu, pos_x_menu_value_room, menu_t_high, BLACK); // erase old string
        tft.setCursor(pos_x_menu, pos_y_menu);           
        tft.print ("Saving to EEPROM..."); 
        delay (1000);       
        EEPROMSAVE();  // save to EEPROM	

		delay (10);   
		digitalWrite (On_set, LOW);		
  }
    
  //************************************************************
  // Check if EncTune encoder changed
  //************************************************************  
  encoder_change1=encoder_pos1-last_encoder_pos1;

  if (encoder_pos1 != last_encoder_pos1) {
 
	if (Menu2 == TUNING ) {

		// do only send freq control data if freq changed.
		if (encoder_change1)
		{
		bands[band].freq+=encoder_change1*0.5*tunestep;  // EncTune the master vfo 
		if (bands[band].freq > F_MAX) bands[band].freq = F_MAX;
		if (bands[band].freq < F_MIN) bands[band].freq = F_MIN;
		setfreq();
		last_encoder_pos1 = encoder_pos1;
		}	
	    // this is SAM mode, when you EncTune in 1kHz steps and frequency is automatically tuned to carrier
        if (bands[band].mode == modeSAM) 
			{ // temporarily switched off!          
			setup_RX(bands[band].mode, bands[band].bandwidthU, bands[band].bandwidthL); 
			tune_SAM_PLL();
			}	

	} // end Encoder EncTune
    //***Write Menu Change to***	
	if (Menu2 == AGC) {    
			if ((encoder_change1 > 5)||(encoder_change1 < -5)) 
				{				
				//truncate change
				if (encoder_change1 > 5 ) encoder_change1 = 1;	     
				if (encoder_change1 < -5 ) encoder_change1 = -1;	   
				// display at Bottom
				AGC_Mode = AGC_Mode + encoder_change1;
				if ((AGC_Mode > FAST)) AGC_Mode = OFF;	
				if ((AGC_Mode < OFF)) AGC_Mode = FAST;					
               
				show_agc(AGC_Mode);	
				last_encoder_pos1 = encoder_pos1;
				}
			// NO AGF desired
			if (!AGC_Mode) {
			MixGain.gain(0,1);       	// Adjust AGC gain
			MixGain.gain(1,1);       	// Adjust AGC gain			
			}
		} // end Menu2 == AGC	
  	//***Write Menu Change to***
	if (Menu2 == RFGAIN) {
	  if (encoder_change1){
      bands[band].RFgain = bands[band].RFgain + encoder_change1;
      if (bands[band].RFgain < 0) 
		  bands[band].RFgain = 0;
      if ((bands[band].RFgain > 150))
		{
		  RFgain_Auto =1;
		  bands[band].RFgain = 150;
		}
      else
		{	
	    RFgain_Auto =0;
		audioShield.lineInLevel((float)bands[band].RFgain/10, (float)bands[band].RFgain/10);  
		sprintf(rfg,"%2.1f",(float)(bands[band].RFgain*0.15));
		show_rfg(rfg);
		} // end RFGAIN
	if (RFgain_Auto)
		{			
		show_rfg(rfg);		
		}		
		last_encoder_pos1 = encoder_pos1;
	  }
  }   
	//***Write Menu Change to***
	if (Menu2 == BWADJUST) {

    bands[band].bandwidthL = bands[band].bandwidthL + encoder_change1*BW_step*0.5; 
    bands[band].bandwidthU = bands[band].bandwidthU + encoder_change1*BW_step*0.5; 
	    //LSB
		if (bands[band].mode == modeLSB)
		{
		bands[band].bandwidthU = 0;
		if(bands[band].bandwidthL < MIN_BANDWIDTH) 
			bands[band].bandwidthL = MIN_BANDWIDTH;    
		if (bands[band].bandwidthL > MAX_BANDWIDTH) 
			bands[band].bandwidthL = MAX_BANDWIDTH;		
		//USB
		} else if (bands[band].mode == modeUSB)
		{
        bands[band].bandwidthL = 0;
		if(bands[band].bandwidthU < MIN_BANDWIDTH) 
			bands[band].bandwidthU = MIN_BANDWIDTH;
		if (bands[band].bandwidthU > MAX_BANDWIDTH) 
			bands[band].bandwidthU = MAX_BANDWIDTH;
		//AM, DSB, SAM
		} else if ((bands[band].mode == modeAM) || (bands[band].mode == modeSAM) || (bands[band].mode == modeDSB))
		{
		if(bands[band].bandwidthU < MIN_BANDWIDTH) 
			bands[band].bandwidthU = MIN_BANDWIDTH;
		if (bands[band].bandwidthU > MAX_BANDWIDTH) 
			bands[band].bandwidthU = MAX_BANDWIDTH;
		if(bands[band].bandwidthL < MIN_BANDWIDTH) 
			bands[band].bandwidthL = MIN_BANDWIDTH;    
		if (bands[band].bandwidthL > MAX_BANDWIDTH) 
			bands[band].bandwidthL = MAX_BANDWIDTH;		
		//CW
		}else if (bands[band].mode == modeCW)
		{
		if(bands[band].bandwidthU < (MIN_BW_CW+BW_step)) 
			bands[band].bandwidthU = MIN_BW_CW+BW_step;		
		if(bands[band].bandwidthU > (MAX_BW_CW)) 
			bands[band].bandwidthU = MAX_BW_CW;		
		bands[band].bandwidthL = MIN_BW_CW;
		
		}
			
    setup_RX(bands[band].mode, bands[band].bandwidthU, bands[band].bandwidthL); 
	show_bandwidth(bands[band].mode, bands[band].bandwidthU, bands[band].bandwidthL); 
   
	last_encoder_pos1 = encoder_pos1;
    } // END BWadjust
} 	// end Encoder !
	
   
  //************************************************************
  // Check if Menu encoder changed
  //************************************************************
  encoder_change=encoder_pos-last_encoder_pos;	 
  
  if (encoder_pos != last_encoder_pos) {
	if (encoder_change > 2)
	  encoder_change =1;
	if (encoder_change < -2)
	  encoder_change =-1;
 
 	  Serial.print("Encoder Pos");
      Serial.println(encoder_pos);	
  //***Write Menu Change to***
  if (Menu == NOTCH) { 
    notchF = notchF + encoder_change*10;
    if(notchF < -11000) notchF = -11000;
    if (notchF > 11000) notchF = 11000;
    if (bands[band].mode == modeLSB && notchF >= -200) notchF = -200;     
    if (bands[band].mode == modeUSB && notchF < 200) notchF = 200;     
    show_notch(notchF, bands[band].mode); // draw notch indicator into spectrum display
    if (notchF < 400 && notchF > -400) { // notch indicator is in centre position, which means OFF
           tft.fillRect(pos_x_menu_value_start, pos_y_menu, pos_x_menu_value_room, menu_t_high,BLACK);
           tft.setCursor(pos_x_menu_value_start, pos_y_menu);
           tft.print ("OFF");
           tft.setTextColor(WHITE);  
    }
    else {
      // here real notch 
            tft.fillRect(pos_x_menu_value_start, pos_y_menu, pos_x_menu_value_room, menu_t_high,BLACK);
            tft.setCursor(pos_x_menu_value_start, pos_y_menu);
            tft.setTextColor(GREEN);
            tft.print (notchF); 
			tft.print ("Hz");     // Linkwitz-Riley filter, 48 dB/octave
            tft.setTextColor(WHITE);
            setup_RX(bands[band].mode, bands[band].bandwidthU, bands[band].bandwidthL);			
			
    } // end ELSE
   } // END NOTCH
  //***Write Menu Change to***
  if (Menu == TIMEADJUST) {
      helpmin = minute(); helphour = hour();
      helpmin = helpmin + encoder_change;
      if (helpmin > 59) { 
        helpmin = 0; helphour = helphour +1;}
       if (helpmin < 0) {
        helpmin = 59; helphour = helphour -1; }
       if (helphour < 0) helphour = 23; 
      if (helphour > 23) helphour = 0;
      helpmonth = month(); helpyear = year(); helpday = day();
      setTime (helphour, helpmin, 0, helpday, helpmonth, helpyear);      
      Teensy3Clock.set(now()); // set the RTC  
    } // end TIMEADJUST
  //***Write Menu Change to***
  if (Menu == BASS) {     
                 bass_help = bass_help + encoder_change;
                  if (bass_help < -100) bass_help = -100;
                  if (bass_help > 100) bass_help = 100;
                  bass = bass_help / 100;
				tft.fillRect(pos_x_menu_value_start, pos_y_menu, pos_x_menu_value_room, menu_t_high,BLACK);
				tft.setCursor(pos_x_menu_value_start, pos_y_menu);
				tft.print (bass_help);
				audioShield.eqBands (bass, treble); // (float bass, float treble) in % -100 to +100
    } // end Menu == BASS
  //***Write Menu Change to***
  if (Menu == SNAP) {
        if (turns == 64) turns = 5;
           tft.fillRect(pos_x_menu_value_start, pos_y_menu, pos_x_menu_value_room, menu_t_high,BLACK);
           tft.setCursor(pos_x_menu_value_start, pos_y_menu);
           tft.print ("Nr.:");
           turns = turns - encoder_change;    	 	
           tft.print (turns); //, " steps till snap!"); 
        if (turns == 0) {
           turns = 10;
           tft.fillRect(pos_x_menu_value_start, pos_y_menu, pos_x_menu_value_room, menu_t_high,BLACK);
           tft.setCursor(pos_x_menu_value_start, pos_y_menu);
           tft.setTextColor(WHITE);          
           tft.print("Carrier !");            
           last_encoder_pos = encoder_pos;
           encoder_change = 0;
       
		   tune_SAM_PLL();        
		}
   } // end if Menu == SNAP
  //***Write Menu Change to***
  if (Menu == TREBLE) {     
                 treble_help = treble_help + encoder_change;
                  if (treble_help < -100) treble_help = -100;
                  if (treble_help > 100) treble_help = 100;
                  treble = treble_help / 100;
				tft.fillRect(pos_x_menu_value_start, pos_y_menu, pos_x_menu_value_room, menu_t_high,BLACK);
				tft.setCursor(pos_x_menu_value_start, pos_y_menu);
				tft.print (treble_help);
              audioShield.eqBands (bass, treble); // (float bass, float treble) in % -100 to +100
    } // end Menu== TREBLE
  //***Write Menu Change to***
  if (Menu == DATEADJUST) {
      helpyear = year(); helpmonth = month();
      helpday = day();
      helpday = helpday + encoder_change;
      if (helpday < 1) {helpday=31; helpmonth=helpmonth-1;}
      if (helpday > 31) {helpmonth = helpmonth +1; helpday=1;}
      if (helpmonth < 1) {helpmonth = 12; helpyear = helpyear-1;}
      if (helpmonth > 12) {helpmonth = 1; helpyear = helpyear+1;}
      helphour=hour(); helpmin=minute(); helpsec=second(); 
      setTime (helphour, helpmin, helpsec, helpday, helpmonth, helpyear);      
      Teensy3Clock.set(now()); // set the RTC
      displayDate();
          } // end DATEADJUST
   //***Write Menu Change to***
  if (Menu == CALIBRATIONFACTOR) {
      calibration_factor = calibration_factor + encoder_change;
        tft.fillRect(pos_x_menu_value_start, pos_y_menu, pos_x_menu_value_room, menu_t_high,BLACK);
        tft.setCursor(pos_x_menu_value_start, pos_y_menu);        
        tft.print (calibration_factor);
	    // if CLCK2 use PLLB
        si5351.set_pll(SI5351_PLL_FIXED, SI5351_PLLB);
        setfreq();
        } // end CALIBRATIONFACTOR   
  //***Write Menu Change to***  
  if (Menu == CALIBRATIONCONSTANT) {
      calibration_constant = calibration_constant + encoder_change*10;
       tft.fillRect(pos_x_menu_value_start, pos_y_menu, pos_x_menu_value_room, menu_t_high,BLACK);
       tft.setCursor(pos_x_menu_value_start, pos_y_menu);       
       tft.print (calibration_constant);
       si5351.set_correction(calibration_constant); 
       //si5351.init(SI5351_CRYSTAL_LOAD_10PF, Si_5351_crystal, calibration_constant);
       si5351.set_pll(SI5351_PLL_FIXED, SI5351_PLLB);
       setfreq();
      } // end CALIBRATIONCONSTANT
  //***Write Menu Change to***
  if (Menu == LOADFROMEEPROM) {
        tft.fillRect(pos_x_menu_value_start, pos_y_menu, pos_x_menu_value_room, menu_t_high,BLACK);
        tft.setCursor(pos_x_menu_value_start, pos_y_menu);
        tft.print ("Turn...");
        turns = turns - encoder_change;
        tft.print (turns); //, " steps till load!"); 
        if (turns == 0) {
        turns = 10;
        tft.fillRect(pos_x_menu_value_start, pos_y_menu, pos_x_menu_value_room, menu_t_high,BLACK);
        tft.setCursor(pos_x_menu_value_start, pos_y_menu);
        tft.print ("Loaded!"); 
        delay (2000);
        last_encoder_pos = encoder_pos;
        encoder_change = 0;
        tft.fillRect(pos_x_menu_value_start, pos_y_menu, pos_x_menu_value_room, menu_t_high,BLACK);
        tft.setCursor(pos_x_menu_value_start, pos_y_menu);
        Menu = TUNING;
        show_tunestep(tunestep);
        eepromload(); // load from EEPROM
        setup_gain();
        setup_mode(bands[band].mode);
		setfreq();
        setup_RX(bands[band].mode, bands[band].bandwidthU, bands[band].bandwidthL);  // set up the audio chain for new mode
        }
              } // end Load from EEPROM
  //***Write Menu Change to***
  if (Menu == SAVETOEEPROM) {
        tft.fillRect(pos_x_menu_value_start, pos_y_menu, pos_x_menu_value_room, menu_t_high,BLACK);
        tft.setCursor(pos_x_menu_value_start, pos_y_menu);
        tft.print ("Turn..");
        turns = turns - encoder_change;       
        tft.print (turns); //, " steps till save!"); 
        if (turns == 0) {
        turns = 10;        
        tft.fillRect(pos_x_menu_value_start, pos_y_menu, pos_x_menu_value_room, menu_t_high,BLACK);
        tft.setCursor(pos_x_menu_value_start, pos_y_menu);
        tft.print ("Saved!"); 
        delay (2000);
        last_encoder_pos = encoder_pos;
        encoder_change = 0;
        tft.fillRect(pos_x_menu_value_start, pos_y_menu, pos_x_menu_value_room, menu_t_high,BLACK);
        tft.setCursor(pos_x_menu_value_start, pos_y_menu);
        Menu = TUNING;
        show_tunestep(tunestep);
        EEPROMSAVE();  // save to EEPROM
                  }
              } // end Save to EEPROM
  //***Write Menu Change to***
  if (Menu == IQADJUST) {
    I_gain = I_help * 1000;
    I_gain=I_gain+encoder_change;
    if (I_gain > 1500) I_gain=1500;
    if (I_gain <1) I_gain=1;
    I_help = I_gain/1000;
    tft.fillRect(pos_x_menu_value_start, pos_y_menu, pos_x_menu_value_room, menu_t_high,BLACK);
    tft.setCursor(pos_x_menu_value_start, pos_y_menu);
    tft.print (I_gain);
    AudioNoInterrupts();   		// Disable Audio while reconfiguring filters    
    audio_in_I.gain(0,I_help); 	// sets I gain on input I
    AudioInterrupts();
    } // END IQadjust
  //***Write Menu Change to***
  if (Menu == LPFSPECTRUM) {
    LPFcoeffhelp = LPFcoeff * 100;
    LPFcoeffhelp = LPFcoeffhelp + encoder_change;
    if (LPFcoeffhelp > 100) LPFcoeffhelp = 100;
    if (LPFcoeffhelp < 1) LPFcoeffhelp = 1;
    LPFcoeff = LPFcoeffhelp / 100;
	tft.fillRect(pos_x_menu_value_start, pos_y_menu, pos_x_menu_value_room, menu_t_high,BLACK);
    tft.setCursor(pos_x_menu_value_start, pos_y_menu);
    tft.print (LPFcoeffhelp);
    } // END LPFSPECTRUM
    //***Write Menu Change to***
    if (Menu == RESETCODEC) {   
    tft.fillRect(pos_x_menu_value_start, pos_y_menu, pos_x_menu_value_room, menu_t_high,BLACK);
    tft.setCursor(pos_x_menu_value_start, pos_y_menu);
	if (encoder_change){
		reset_codec();		
		tft.print ("RESETED");
	 }
	}
  //***Write Menu Change to***
  if (Menu == IQPHASEADJUST) {
    // get values of I_in_Q / Q_in_I
	// phase adjustment mixes a little amount of I into Q or vice versa before passing the Hilbert Filters
    if (I_in_Q != 0) {
      IQcounter = - I_in_Q_help * 1000;
    } else IQcounter = Q_in_I_help * 1000;
    IQcounter = IQcounter + encoder_change;
    if (IQcounter < 0) {
    I_in_Q = - IQcounter;
    Q_in_I = 0;
    }
    else {
      Q_in_I = IQcounter;
      I_in_Q = 0;
      }  
    Q_in_I_help = Q_in_I / 1000;
    I_in_Q_help = I_in_Q / 1000;
    tft.fillRect(pos_x_menu_value_start, pos_y_menu, pos_x_menu_value_room, menu_t_high,BLACK);
    tft.setCursor(pos_x_menu_value_start, pos_y_menu);
    if (I_in_Q != 0)
      tft.print (-I_in_Q);
    else 
	  tft.print (Q_in_I);
      AudioNoInterrupts();   // Disable Audio while reconfiguring filters
      audio_in_I.gain(1,Q_in_I_help); // sets I gain on input I
      audio_in_Q.gain(1,I_in_Q_help);	
	  //Q Phase corr: Q = Q +(1-winkel² * 0,5)
	  //              Q = Q - I *  (Winkel)	   
	  audio_in_I.gain(0,(I_help*(1-Q_in_I_help*Q_in_I_help*0.5))); 	// sets I gain on input I
      audio_in_Q.gain(0,(1-I_in_Q_help*I_in_Q_help*0.5)); 	// sets I gain on input I			
      AudioInterrupts();
    } // END IQadjust
  //***Write Menu Change to***
  if (Menu == FFTWINDOW) {
      //wind = Window_FFT * 4;
      Window_FFT = Window_FFT + encoder_change;
      //Window_FFT = wind / 4;
      if (Window_FFT < 0) Window_FFT = 0;
      if (Window_FFT > 10) Window_FFT = 10;

     FFT_WINDOW_SET (Window_FFT);
     FFT_STRING = FFT_WINDOW_STRING (Window_FFT);
     tft.fillRect(pos_x_menu_value_start, pos_y_menu, pos_x_menu_value_room, menu_t_high,BLACK);
     tft.setCursor(pos_x_menu_value_start, pos_y_menu);
     tft.print (FFT_STRING);
       } // end FFTWINDOW
    last_encoder_pos = encoder_pos;

  }
  
  //************************************************************
  // Check if Vol encoder changed
  //************************************************************

	encoder_change2=encoder_pos2-last_encoder_pos2;	  
	if (encoder_change2) {		
		if (encoder_change2>2)
			encoder_change2 =1;
		if (encoder_change2 <-2)
			encoder_change2 =-1;

		
		vol = 100*vol+encoder_change2;
		vol = vol/100;
		// only send new data to codec if volume changed
		if (vol > 1)
			vol=1;
		if (vol < 0)
			vol=0;
		audioShield.volume(vol); // Volume pot does adjust analog gain of headphone amp
		audioShield.lineOutLevel((1.0 - vol) * 18 + 13, (1.0 - vol) * 18 + 13); // 13 loudest, 31 lowest
		audioShield.dacVolume(0.75);	
		tft.fillRect(pos_x_menu, pos_y_menu, pos_x_menu_value_room, menu_t_high, BLACK); // erase old string
		tft.setTextColor(WHITE);
        tft.setCursor(pos_x_menu, pos_y_menu);        
        tft.print("[VOL   ]");	
        tft.fillRect(pos_x_menu_value_start, pos_y_menu, pos_x_menu_value_room, menu_t_high,BLACK);
		tft.setCursor(pos_x_menu_value_start, pos_y_menu);      
        tft.print(vol);		
		last_encoder_pos2=encoder_pos2;
		
	}
   
  
   // print time every 0.1 second
   if (ms_100.check() == 1) {
	   displayClock();	   
   if (bands[band].mode == modeSAM)
	  tune_SAM_PLL();
   }
   //******************************************************
   // every 300 ms, adjust show frequency difference
   //******************************************************
   /*
   if (ms_300.check() == 1) {		
	// show delta to carrier
   if ((bands[band].mode == modeSAM) && (Menu == start_menu))
		{
		tft.fillRect(pos_x_menu, pos_y_menu, pos_x_menu_value_room, menu_t_high, BLACK); // erase old string
		tft.setTextColor(WHITE);
        tft.setCursor(pos_x_menu, pos_y_menu);        
        tft.print("[DeltaF]");		
		tft.fillRect(pos_x_menu_value_start, pos_y_menu, pos_x_menu_value_room, menu_t_high,BLACK);
        tft.setCursor(pos_x_menu_value_start, pos_y_menu);       
		float freq_diff = AUDIO_SAMPLE_RATE_EXACT*(SAM.read())/(2*PI);
		char string[7];  
		sprintf(string,"%06.00fHz",freq_diff);
		tft.print (freq_diff); 
		}	
	}  
   */
   //******************************************************
   // every 50 ms, adjust the volume and check the switches
   //******************************************************
   if (ms_50.check() == 1) {     
		
    // Draw Spectrum Display	
   show_spectrum(bands[band].RFgain/10, LPFcoeff, bands[band].mode, bands[band].bandwidthU, bands[band].bandwidthL );
   if (!digitalRead(MenuSW) ) {
   if (menusw_state == 0) { // switch was pressed - falling edge
         tft.fillRect(pos_x_menu, pos_y_menu, pos_x_menu_value_room, menu_t_high, BLACK); // erase old string
         if(++Menu > last_menu) Menu = first_menu;		
		
          //***Show Menu Change Item ***
          if (Menu == TIMEADJUST) {
           tft.fillRect(pos_x_menu, pos_y_menu, pos_x_menu_value_room, menu_t_high, BLACK); // erase old string
		   tft.setTextColor(WHITE);
           tft.setCursor(pos_x_menu, pos_y_menu);        
           tft.print("[TIME  ]");		
           } 
		 //***Show Menu Change Item ***		 
         if (Menu == SNAP) {
           tft.fillRect(pos_x_menu, pos_y_menu, pos_x_menu_value_room, menu_t_high, BLACK); // erase old string
           tft.setTextColor(WHITE);
           tft.setCursor(pos_x_menu, pos_y_menu);
           tft.print("[Snap ?]"); 
		   tft.fillRect(pos_x_menu_value_start, pos_y_menu, pos_x_menu_value_room, menu_t_high,BLACK);
           tft.setCursor(pos_x_menu_value_start, pos_y_menu);
           } 		   
		 //***Show Menu Change Item ***		   
        if (Menu == TREBLE) {
           tft.fillRect(pos_x_menu, pos_y_menu, pos_x_menu_value_room, menu_t_high, BLACK); // erase old string
           tft.setTextColor(WHITE);
           tft.setCursor(pos_x_menu, pos_y_menu);
           tft.print("[TRBL  ]"); 		
           tft.fillRect(pos_x_menu_value_start, pos_y_menu, pos_x_menu_value_room, menu_t_high,BLACK);
           tft.setCursor(pos_x_menu_value_start, pos_y_menu);
           tft.print (treble_help); 
           }
		  //***Show Menu Change Item ***
		 if (Menu == BASS) {
           tft.fillRect(pos_x_menu, pos_y_menu, pos_x_menu_value_room, menu_t_high, BLACK); // erase old string
           tft.setTextColor(WHITE);
           tft.setCursor(pos_x_menu, pos_y_menu);           
           tft.print("[BASS  ]"); 
           tft.fillRect(pos_x_menu_value_start, pos_y_menu, pos_x_menu_value_room, menu_t_high,BLACK);
           tft.setCursor(pos_x_menu_value_start, pos_y_menu);
           tft.print (bass_help); 
           }	
		 //***Show Menu Change Item ***
          if (Menu == DATEADJUST) {
           tft.fillRect(pos_x_menu, pos_y_menu, pos_x_menu_value_room, menu_t_high, BLACK); // erase old string
		   tft.setTextColor(WHITE);
           tft.setCursor(pos_x_menu, pos_y_menu);        
           tft.print("[DATE  ]");		
           //displayDate();
         } 
		  //***Show Menu Change Item ***
          if (Menu == NOTCH) {
           tft.fillRect(pos_x_menu, pos_y_menu, pos_x_menu_value_room, menu_t_high, BLACK); // erase old string
		   tft.setTextColor(WHITE);
           tft.setCursor(pos_x_menu, pos_y_menu);        
           tft.print("[NOTCH  ]");		
            if (notchF < 400 && notchF > -400) { // notch indicator is in centre position, which means OFF
            tft.fillRect(pos_x_menu_value_start, pos_y_menu, pos_x_menu_value_room, menu_t_high,BLACK);
			tft.setCursor(pos_x_menu_value_start, pos_y_menu);
            tft.print ("OFF");
              }
            else {
            // here real notch 
            tft.fillRect(pos_x_menu_value_start, pos_y_menu, pos_x_menu_value_room, menu_t_high,BLACK);
			tft.setCursor(pos_x_menu_value_start, pos_y_menu);
            tft.setTextColor(GREEN);
            tft.print (notchF); tft.print ("Hz");     // Linkwitz-Riley filter, 48 dB/octave
            tft.setTextColor(WHITE);
                } // end ELSE
           }    
		   //***Show Menu Change Item ***	   
          if (Menu == CALIBRATIONFACTOR) {
           tft.fillRect(pos_x_menu, pos_y_menu, pos_x_menu_value_room, menu_t_high, BLACK); // erase old string
		   tft.setTextColor(WHITE);
           tft.setCursor(pos_x_menu, pos_y_menu);        
           tft.print("[CALIB ]");		
           tft.fillRect(pos_x_menu_value_start, pos_y_menu, pos_x_menu_value_room, menu_t_high,BLACK);
		   tft.setCursor(pos_x_menu_value_start, pos_y_menu);
           tft.print (calibration_factor); 
           } 
           //***Show Menu Change Item ***
           if (Menu == CALIBRATIONCONSTANT)  {
           tft.fillRect(pos_x_menu, pos_y_menu, pos_x_menu_value_room, menu_t_high, BLACK); // erase old string
		   tft.setTextColor(WHITE);
           tft.setCursor(pos_x_menu, pos_y_menu);        
           tft.print("[CAL K ]");		
           tft.fillRect(pos_x_menu_value_start, pos_y_menu, pos_x_menu_value_room, menu_t_high,BLACK);
		   tft.setCursor(pos_x_menu_value_start, pos_y_menu);
           tft.print (calibration_constant); 
           } 
           //***Show Menu Change Item ***       
           if (Menu == LOADFROMEEPROM) {
           tft.fillRect(pos_x_menu, pos_y_menu, pos_x_menu_value_room, menu_t_high, BLACK); // erase old string
		   tft.setTextColor(WHITE);
           tft.setCursor(pos_x_menu, pos_y_menu);        
           tft.print("[LOAD ?]");		
           } 
           //***Show Menu Change Item ***     
           if (Menu == SAVETOEEPROM) {
           tft.fillRect(pos_x_menu, pos_y_menu, pos_x_menu_value_room, menu_t_high, BLACK); // erase old string
		   tft.setTextColor(WHITE);
           tft.setCursor(pos_x_menu, pos_y_menu);        
           tft.print("[SAVE  ]");		
		   tft.fillRect(pos_x_menu_value_start, pos_y_menu, pos_x_menu_value_room, menu_t_high,BLACK);
           } 
		   //***Show Menu Change Item ***
           if (Menu == IQADJUST) {
               tft.fillRect(pos_x_menu, pos_y_menu, pos_x_menu_value_room, menu_t_high, BLACK); // erase old string
		   tft.setTextColor(WHITE);
           tft.setCursor(pos_x_menu, pos_y_menu);        
           tft.print("[IQ-AMP]");		
            tft.fillRect(pos_x_menu_value_start, pos_y_menu, pos_x_menu_value_room, menu_t_high,BLACK);
			tft.setCursor(pos_x_menu_value_start, pos_y_menu);
           tft.print (I_help * 1000);
           }
		   //***Show Menu Change Item ***
           if (Menu == LPFSPECTRUM) {
           tft.fillRect(pos_x_menu, pos_y_menu, pos_x_menu_value_room, menu_t_high, BLACK); // erase old string
		   tft.setTextColor(WHITE);
           tft.setCursor(pos_x_menu, pos_y_menu);        
           tft.print("[LPF SP]");		
            tft.fillRect(pos_x_menu_value_start, pos_y_menu, pos_x_menu_value_room, menu_t_high,BLACK);
			tft.setCursor(pos_x_menu_value_start, pos_y_menu);
           tft.print (LPFcoeff * 100); 
           }
		   //***Show Menu Change Item ***
		   if (Menu == RESETCODEC) {
           tft.fillRect(pos_x_menu, pos_y_menu, pos_x_menu_value_room, menu_t_high, BLACK); // erase old string
		   tft.setTextColor(WHITE);
           tft.setCursor(pos_x_menu, pos_y_menu);        
           tft.print("[RSTCOD]");		
           tft.fillRect(pos_x_menu_value_start, pos_y_menu, pos_x_menu_value_room, menu_t_high,BLACK);
		   tft.setCursor(pos_x_menu_value_start, pos_y_menu);    	  
           }
		   //***Show Menu Change Item ***
           if (Menu == IQPHASEADJUST) {
           tft.fillRect(pos_x_menu, pos_y_menu, pos_x_menu_value_room, menu_t_high, BLACK); // erase old string
		   tft.setTextColor(WHITE);
           tft.setCursor(pos_x_menu, pos_y_menu);        
           tft.print("[IQ-PH ]");		
           tft.fillRect(pos_x_menu_value_start, pos_y_menu, pos_x_menu_value_room, menu_t_high,BLACK);
			tft.setCursor(pos_x_menu_value_start, pos_y_menu);
           if (I_in_Q_help != 0)
           tft.print (-I_in_Q_help*1000);
           else tft.print (Q_in_I_help*1000);
           }
		  //***Show Menu Change Item ***
          if (Menu == FFTWINDOW) {
           tft.fillRect(pos_x_menu, pos_y_menu, pos_x_menu_value_room, menu_t_high, BLACK); // erase old string
		   tft.setTextColor(WHITE);
           tft.setCursor(pos_x_menu, pos_y_menu);        
           tft.print("[FFT-W ]");		
           FFT_STRING = FFT_WINDOW_STRING (Window_FFT);
            tft.fillRect(pos_x_menu_value_start, pos_y_menu, pos_x_menu_value_room, menu_t_high,BLACK);
			tft.setCursor(pos_x_menu_value_start, pos_y_menu);
           tft.print (FFT_STRING);
           }
        menusw_state=1; // flag switch is pressed
       }
    }
    else menusw_state=0; // flag switch not pressed
    if (!digitalRead(TuneSW)) {
    if (tunesw_state == 0) { // switch was pressed - falling edge
         tunehelp = tunehelp +1;
         if(tunehelp > last_tunehelp) tunehelp = first_tunehelp;
        //   if (tunehelp == 1)  {tunestep=TUNE_STEP1; }
           if (tunehelp == 2)  {tunestep=TUNE_STEP2; }
           if (tunehelp == 3)  {tunestep=TUNE_STEP3; }
		   if (tunehelp == 4)  {tunestep=TUNE_STEP4; }
		   if (tunehelp == 5)  {tunestep=TUNE_STEP5; }
        show_tunestep(tunestep);			
		tunesw_state=1; // flag switch is pressed
		}
		// ESC
		tunesw_state=1; // flag switch is pressed		
    }
    else tunesw_state=0; // flag switch not pressed
    if (!digitalRead(ModeSW) ) {
       if (modesw_state==0) { // switch was pressed - falling edge
       if(++bands[band].mode > lastmode) bands[band].mode=firstmode; // cycle thru radio modes 
         if (bands[band].mode == modeUSB && notchF <=-400) notchF = notchF *-1; // this flips the notch filter round, when you go from LSB --> USB and vice versa
         if (bands[band].mode == modeLSB && notchF >=400) notchF = notchF *-1;
         setup_mode(bands[band].mode);
		 setfreq();
         setup_RX(bands[band].mode, bands[band].bandwidthU, bands[band].bandwidthL);  // set up the audio chain for new mode                            
         show_bandwidth(bands[band].mode, bands[band].bandwidthU, bands[band].bandwidthL);
         modesw_state=1; // flag switch is pressed
       }
    }
    else modesw_state=0; // flag switch not pressed; end ModeSW
    if (!digitalRead(MainMenuScrollSW)) {
       if (MainMenuScrollsw_state==0) { // switch was pressed - falling edge   
	    
         if(++Menu2 > last_menu2) 
			 Menu2 = first_menu2;
		 
		 //***Show Menu Change Item ***
         if (Menu2 == RFGAIN){  
		    highlight_rfg(1);
			//show_rfg(rfg);
           } 
		 else
			{
			//show_rfg(rfg);
			highlight_rfg(0);
			}
              		
		 //***Show Menu Change Item ***
         if (Menu2 == BWADJUST){			
				highlight_bw(1);
				show_bandwidth(bands[band].mode, bands[band].bandwidthU, bands[band].bandwidthL); 
           } 
		   else
			{
			   highlight_bw(0);
			   show_bandwidth(bands[band].mode, bands[band].bandwidthU, bands[band].bandwidthL); 
			}
			   
		//***Show Menu Change Item ***
		if (Menu2 == AGC) {      
			highlight_agc(1);
			show_agc(AGC_Mode);
           }
		   else
		   {
			highlight_agc(0);
			show_agc(AGC_Mode);
		   }
		//***Show Menu Change Item ***
        if (Menu2 == TUNING) {           
		   tft.fillRect(pos_x_menu, pos_y_menu, pos_x_menu_value_room, menu_t_high, BLACK); // erase old string
           tft.setTextColor(WHITE);
           show_tunestep(tunestep);
           }		
         MainMenuScrollsw_state=1; // flag switch is pressed
       }
    }
    else MainMenuScrollsw_state=0; // flag swlitch not pressed    
    
    if (!digitalRead(BandUPSW)) {
       if (Bandupsw_state==0) { // switch was pressed - falling edge
         if(++band > LAST_BAND) band=FIRST_BAND; // cycle thru radio band   
         setup_mode(bands[band].mode);    	 
         setup_RX(bands[band].mode, bands[band].bandwidthU, bands[band].bandwidthL);  // set up the audio chain for new mode
         setfreq();
         Bandupsw_state=1; // flag switch is pressed
       }
    }
    else Bandupsw_state=0; // flag switch not pressed      
	
	if (!digitalRead(BandDOWNSW)) {
       if (Banddownsw_state==0) { // switch was pressed - falling edge
         if(--band < FIRST_BAND) band=LAST_BAND; // cycle thru radio bands     
         setup_mode(bands[band].mode);    	 
         setup_RX(bands[band].mode, bands[band].bandwidthU, bands[band].bandwidthL);  // set up the audio chain for new mode
         setfreq();
         Banddownsw_state=1; // flag switch is pressed
       }
    }
    else Banddownsw_state=0; // flag switch not pressed    		
}
#if 1
    if (five_sec.check() == 1)
    {		 
	//plotter.step();  
	//if (plotter.newBlock())
	//	{		
		  Serial.print("Proc = ");
		  Serial.print(AudioProcessorUsage());
		  Serial.print(" (");    
		  Serial.print(AudioProcessorUsageMax());
		  Serial.print("),  Mem = ");
		  Serial.print(AudioMemoryUsage());
		  Serial.print(" (");    
		  Serial.print(AudioMemoryUsageMax());
		  Serial.println(")");
		  AudioProcessorUsageMaxReset();
		  AudioMemoryUsageMaxReset();     
		  Serial.print("Sampling Rate");
		  Serial.println(AUDIO_SAMPLE_RATE_EXACT);	
		  Serial.print("F_PLL");
		  Serial.println(F_PLL);

	//	}
	// plotter.done();
	}
#endif
} // end void loop ()

/* #########################################################################
 *  
 * set the FFT window function
 *  
 * ######################################################################### 
 */
void FFT_WINDOW_SET (int FFT) {  
  switch (FFT) {
        case 0: myFFT.windowFunction(AudioWindowHanning256);
                break; 
        case 1: myFFT.windowFunction(AudioWindowHamming256);                
                break;
        case 2: myFFT.windowFunction(AudioWindowBartlett256);               
                break;
        case 3: myFFT.windowFunction(AudioWindowBlackman256);               
                break;
        case 4: myFFT.windowFunction(AudioWindowNuttall256);                
                break;
        case 5: myFFT.windowFunction(AudioWindowBlackmanHarris256);              
                break;
        case 6: myFFT.windowFunction(AudioWindowBlackmanNuttall256);               
                break;
        case 7: myFFT.windowFunction(AudioWindowFlattop256);               
                break;
        case 8: myFFT.windowFunction(AudioWindowWelch256);               
                break;
        case 9: myFFT.windowFunction(AudioWindowCosine256);               
                break;
        case 10:myFFT.windowFunction(AudioWindowTukey256);               
                break;
        default: myFFT.windowFunction(AudioWindowHanning256);
                break;       
      } //end switch
} // end FFT WINDOW SET

String FFT_WINDOW_STRING (int FFT) {
  switch (FFT) {
        case 0: return "Hann"; break; 
        case 1: return "Hamming";
                break;
        case 2: return "Bartlet";
                break;
        case 3: return "Blackman";
                break;
        case 4: return "Nuttal";
                break;
        case 5: return "BlackHar";
                break;
        case 6: return "BlackNut";
                break;
        case 7: return "BlackNut";
                break;
        case 8: return "Welch";
                break;
        case 9: return "Cosine";
                break;
        case 10: return "Tukey";
                break;
		default: return "default"; break;        
      } //end switch
    } // end FFT WINDOW SET


/* #########################################################################
 *  
 * void setup_gain
 * use I/Q amplitude correction  
 *  
 * ######################################################################### 
 */
void setup_gain() {
  AudioNoInterrupts();
  // set gain values for audio_in_I and audio_in_Q
  // phase adjustment mixes a little amount of I into Q or vice versa before passing the Hilbert Filters
  audio_in_I.gain(0, I_help); 		// take values from EEPROM
  audio_in_I.gain(1, Q_in_I_help);  // take values from EEPROM
  audio_in_I.gain(2,0); 			//not used
  audio_in_I.gain(3,0); 			//not used
  audio_in_Q.gain(0, 1); 			// always 1, only I gain is calibrated
  audio_in_Q.gain(1, I_in_Q_help); 	// take values from EEPROM
  audio_in_Q.gain(2,0); 			//not used
  audio_in_Q.gain(3,0); 			//not used
  AudioInterrupts();
} // end void setup_gain();


/* #########################################################################
 *  
 *  void setup_mode
 *  
 *  set up radio for RX modes - USB, LSB
 *  USB/LSB Mixer with 3 Inputs 
 *  0= I, 1= Q, 2= AM
 * ######################################################################### 
 */
void setup_mode(int MO) {
  tft.fillRect(pos_x_menu, pos_y_menu, pos_x_menu_value_room, menu_t_high, BLACK); // erase old string       
  tft.fillRect(pos_x_menu_value_start, pos_y_menu, pos_x_menu_value_room, menu_t_high,BLACK);
  AudioNoInterrupts();
  switch (MO)  {
    case modeLSB:
		AM.passthrough(0); // switches off SAM and AM demodulator for the single sideband demodulation modes  
     	SAM.passthrough(0); 
        if (bands[band].bandwidthU != 0) 
			bands[band].bandwidthL = bands[band].bandwidthU;
        bands[band].bandwidthU = 0;
        USB.gain(0, 0);
        USB.gain(1, 0); // 
        LSB.gain(0, 1);
        LSB.gain(1, -1); 
		DSB.gain(0,0);
		DSB.gain(1,0);
		MixerAM.gain(0,0);
		MixerAM.gain(1,0);
        ModeMixer.gain(0, 0); 
        ModeMixer.gain(1, 1); 
		ModeMixer.gain(2, 0); 
		ModeMixer.gain(3, 0);  	
		
		if (notchF > -399) notchF= -399;;
    break;
    case modeUSB:
		AM.passthrough(0); // switches off SAM and AM demodulator for the single sideband demodulation modes  
     	SAM.passthrough(0); 
        if (bands[band].bandwidthL != 0) 
			bands[band].bandwidthU = bands[band].bandwidthL;
        bands[band].bandwidthL = 0;
        USB.gain(0, 1);
        USB.gain(1, 1); 
        LSB.gain(0, 0);
        LSB.gain(1, 0);
		DSB.gain(0,0);
		DSB.gain(1,0);
		MixerAM.gain(0,0);
		MixerAM.gain(1,0);
        ModeMixer.gain(0, 1); 
        ModeMixer.gain(1, 0); 
		ModeMixer.gain(2, 0); 
		ModeMixer.gain(3, 0);  	
		
		if (notchF < 399) notchF= 399;
    break;
    case modeAM:
		AM.passthrough(AM_pass); // switches on the AM demodulator 		
     	SAM.passthrough(0); 
		AM_highpass.setCoefficients(0, IIR_highpass_96k_Coeffs_0); 
        if (bands[band].bandwidthU >= bands[band].bandwidthL) 
            bands[band].bandwidthL = bands[band].bandwidthU;
            else bands[band].bandwidthU = bands[band].bandwidthL;
        USB.gain(0, 0);
        USB.gain(1, 0);
        LSB.gain(0, 1);
        LSB.gain(1, 1);
		DSB.gain(0,0);
		DSB.gain(1,0);
		MixerAM.gain(0,1);
		MixerAM.gain(1,0);
        ModeMixer.gain(0, 0);
        ModeMixer.gain(1, 0);    
		ModeMixer.gain(2, 1); 		
		ModeMixer.gain(3, 0);  	
    break;
	 case modeSAM:
	    SAM.passthrough(AM_pass); // switches on the AM demodulator 
		AM.passthrough(0); // switches on the AM demodulator 		    
        if (bands[band].bandwidthU >= bands[band].bandwidthL) 
            bands[band].bandwidthL = bands[band].bandwidthU;
            else bands[band].bandwidthU = bands[band].bandwidthL;
		AM_highpass.setCoefficients(0, IIR_highpass_96k_Coeffs_0); 
        USB.gain(0, 0);
        USB.gain(1, 0); // off
        LSB.gain(0, 0);
        LSB.gain(1, 0);		
		DSB.gain(0,0);
		DSB.gain(1,0);
		MixerAM.gain(0,0);
		MixerAM.gain(1,1);
		ModeMixer.gain(0, 0);
        ModeMixer.gain(1, 0);    
		ModeMixer.gain(2, 1);  
		ModeMixer.gain(3, 0);  	
    break;
    case modeDSB:
		AM.passthrough(0); // switches off SAM and AM demodulator for the single sideband demodulation modes  
     	SAM.passthrough(0); 
        if (bands[band].bandwidthU >= bands[band].bandwidthL) 
            bands[band].bandwidthL = bands[band].bandwidthU;
            else bands[band].bandwidthU = bands[band].bandwidthL;          
        USB.gain(0, 0);
        USB.gain(1, 0); 
        LSB.gain(0, 0);
        LSB.gain(1,0 );  
		DSB.gain(0,0.5);
		DSB.gain(1,0.5);
		MixerAM.gain(0,0);
		MixerAM.gain(1,0);
        ModeMixer.gain(0, 0); 
        ModeMixer.gain(1, 0);   
		ModeMixer.gain(2, 0);  		
		ModeMixer.gain(3, 1);  		
    break;
	 case modeCW:
		AM.passthrough(0); // switches off SAM and AM demodulator for the single sideband demodulation modes  
     	SAM.passthrough(0); 

        if (bands[band].bandwidthU == 0) 
			bands[band].bandwidthU = MIN_BW_CW+BW_step;
      			
        USB.gain(0, 0);
        USB.gain(1, 0); // 
        LSB.gain(0, 1);
        LSB.gain(1, -1); 
		DSB.gain(0,0);
		DSB.gain(1,0);
		MixerAM.gain(0,0);
		MixerAM.gain(1,0);
        ModeMixer.gain(0, 0); 
        ModeMixer.gain(1, 1); 
		ModeMixer.gain(2, 0); 
		ModeMixer.gain(3, 0);  	
  }
   AudioInterrupts();
} // end void setup_mode


/* #########################################################################
 *  
 *  void setup_RX
 *  
 *  set up filters: bandwidths etc
 * ######################################################################### 
 */
 void setup_RX(int MO, int bwu, int bwl)
{
  char rfg[7];
  show_notch(notchF, MO);
  //FreqConv.direction(1); // receive freq is higher than zeroband
  //FreqConv.passthrough(freqconv_sw); // do not passthrough audio, but convert by IF
  AudioNoInterrupts();   // Disable Audio while reconfiguring filters

  // if Auto it is showen by the other routine.
  if (!RFgain_Auto)
	  {
	  audioShield.lineInLevel((float)bands[band].RFgain/10, (float) bands[band].RFgain/10);
	  sprintf(rfg,"%2.1f",(float)(bands[band].RFgain*0.15));
	  show_rfg(rfg);
	  }
	else
	  {
	  //get last current value of amplifier
	  audioShield.lineInLevel(RFgain_automatic,RFgain_automatic);
	  }
	biquad_I.setCoefficients(0, IIR_60_96k_Coeffs_0);
	biquad_I.setCoefficients(1, IIR_60_96k_Coeffs_1);
	biquad_I.setCoefficients(2, IIR_60_96k_Coeffs_2);
	biquad_I.setCoefficients(3, IIR_60_96k_Coeffs_3); 	

	biquad_Q.setCoefficients(0, IIR_60_96k_Coeffs_0);
	biquad_Q.setCoefficients(1, IIR_60_96k_Coeffs_1);
	biquad_Q.setCoefficients(2, IIR_60_96k_Coeffs_2);
	biquad_Q.setCoefficients(3, IIR_60_96k_Coeffs_3); 	

	biquad_I2.setCoefficients(0, IIR_60_96k_Coeffs_0);
	biquad_I2.setCoefficients(1, IIR_60_96k_Coeffs_1);
	biquad_I2.setCoefficients(2, IIR_60_96k_Coeffs_2);
	//biquad_I2.setCoefficients(3, IIR_60_96k_Coeffs_3); 	

	biquad_Q2.setCoefficients(0, IIR_60_96k_Coeffs_0);
	biquad_Q2.setCoefficients(1, IIR_60_96k_Coeffs_1);
	biquad_Q2.setCoefficients(2, IIR_60_96k_Coeffs_2);
	//biquad_Q2.setCoefficients(3, IIR_60_96k_Coeffs_3); 	
		
			
	int BW = bwl;  
	if ((MO == modeAM) || (MO == modeSAM))	  
		{	  			
		AM_highpass.setCoefficients(0, IIR_highpass_96k_Coeffs_0); 
		//setHighpass
    
	   	// Receiver Bandwidth Filter
            if (BW < 1301) {             
                biquad1.setCoefficients(0, IIR_26_96k_Coeffs_0);
                biquad1.setCoefficients(1, IIR_26_96k_Coeffs_1);
                biquad1.setCoefficients(2, IIR_26_96k_Coeffs_2);
                biquad1.setCoefficients(3, IIR_26_96k_Coeffs_3); 		
            } else
            if (BW < 1601) {
                biquad1.setCoefficients(0, IIR_33_96k_Coeffs_0);
                biquad1.setCoefficients(1, IIR_33_96k_Coeffs_1);
                biquad1.setCoefficients(2, IIR_33_96k_Coeffs_2);
                biquad1.setCoefficients(3, IIR_33_96k_Coeffs_3);		
            } else
            if (BW < 1801) {         
                biquad1.setCoefficients(0, IIR_36_96k_Coeffs_0);
                biquad1.setCoefficients(1, IIR_36_96k_Coeffs_1);
                biquad1.setCoefficients(2, IIR_36_96k_Coeffs_2);
                biquad1.setCoefficients(3, IIR_36_96k_Coeffs_3);               
            } else
            if (BW < 2301) {             
                biquad1.setCoefficients(0, IIR_44_96k_Coeffs_0);
                biquad1.setCoefficients(1, IIR_44_96k_Coeffs_1);
                biquad1.setCoefficients(2, IIR_44_96k_Coeffs_2);
                biquad1.setCoefficients(3, IIR_44_96k_Coeffs_3);			
            } else
            if (BW < 2601) {               
                biquad1.setCoefficients(0, IIR_54_96k_Coeffs_0);
                biquad1.setCoefficients(1, IIR_54_96k_Coeffs_1);
                biquad1.setCoefficients(2, IIR_54_96k_Coeffs_2);
                biquad1.setCoefficients(3, IIR_54_96k_Coeffs_3);   			    				
            }else        
		    if (BW < 3001) {               
                biquad1.setCoefficients(0, IIR_60_96k_Coeffs_0);
                biquad1.setCoefficients(1, IIR_60_96k_Coeffs_1);
                biquad1.setCoefficients(2, IIR_60_96k_Coeffs_2);
                biquad1.setCoefficients(3, IIR_60_96k_Coeffs_3);   			    				
            }else        
            if (BW < 3601) {            
                biquad1.setCoefficients(0, IIR_65_96k_Coeffs_0);
                biquad1.setCoefficients(1, IIR_65_96k_Coeffs_1);
                biquad1.setCoefficients(2, IIR_65_96k_Coeffs_2);
                biquad1.setCoefficients(3, IIR_65_96k_Coeffs_3);		
            } else
            if (BW < 4401) {         
                biquad1.setCoefficients(0, IIR_80_96k_Coeffs_0);
                biquad1.setCoefficients(1, IIR_80_96k_Coeffs_1);
                biquad1.setCoefficients(2, IIR_80_96k_Coeffs_2);
                biquad1.setCoefficients(3, IIR_80_96k_Coeffs_3);		
            }
            else
            if (BW < 6500) { 
                biquad1.setCoefficients(0, IIR_110_96k_Coeffs_0);
                biquad1.setCoefficients(1, IIR_110_96k_Coeffs_1);
                biquad1.setCoefficients(2, IIR_110_96k_Coeffs_2);
                biquad1.setCoefficients(3, IIR_110_96k_Coeffs_3);		          	
            }
            else {              
                biquad1.setCoefficients(0, IIR_110_96k_Coeffs_0);
                biquad1.setCoefficients(1, IIR_110_96k_Coeffs_1);
                biquad1.setCoefficients(2, IIR_110_96k_Coeffs_2);
                biquad1.setCoefficients(3, IIR_110_96k_Coeffs_3); 						
            }		
	}  
    if ((MO == modeLSB) || (MO == modeUSB)|| (MO == modeDSB))	  
		{ 
		  //AM.passthrough(0); // switches off AM demodulator for the single sideband demodulation modes       	
          if (bwu >= bwl) BW = bwu;
            else BW = bwl;    	
						    
			if (BW < 1801) {
                biquad1.setCoefficients(0, IIR_18_96k_Coeffs_0);
                biquad1.setCoefficients(1, IIR_18_96k_Coeffs_1);
                biquad1.setCoefficients(2, IIR_18_96k_Coeffs_2);
                biquad1.setCoefficients(3, IIR_18_96k_Coeffs_3);			
            } else
            if (BW < 2301) {
                biquad1.setCoefficients(0, IIR_23_96k_Coeffs_0);
                biquad1.setCoefficients(1, IIR_23_96k_Coeffs_1);
                biquad1.setCoefficients(2, IIR_23_96k_Coeffs_2);
                biquad1.setCoefficients(3, IIR_23_96k_Coeffs_3);	
            } else          
			 if (BW < 2601) {
                biquad1.setCoefficients(0, IIR_28_96k_Coeffs_0);
                biquad1.setCoefficients(1, IIR_28_96k_Coeffs_1);
                biquad1.setCoefficients(2, IIR_28_96k_Coeffs_2);
                biquad1.setCoefficients(3, IIR_28_96k_Coeffs_3);		
            }else
            if (BW < 2801) {
                biquad1.setCoefficients(0, IIR_28_96k_Coeffs_0);
                biquad1.setCoefficients(1, IIR_28_96k_Coeffs_1);
                biquad1.setCoefficients(2, IIR_28_96k_Coeffs_2);
                biquad1.setCoefficients(3, IIR_28_96k_Coeffs_3);		
            }else
			if (BW < 3301) {
                biquad1.setCoefficients(0, IIR_33_96k_Coeffs_0);
                biquad1.setCoefficients(1, IIR_33_96k_Coeffs_1);
                biquad1.setCoefficients(2, IIR_33_96k_Coeffs_2);
                biquad1.setCoefficients(3, IIR_33_96k_Coeffs_3);				
            } else
            if (BW < 4001) {
                biquad1.setCoefficients(0, IIR_40_96k_Coeffs_0);
                biquad1.setCoefficients(1, IIR_40_96k_Coeffs_1);
                biquad1.setCoefficients(2, IIR_40_96k_Coeffs_2);
                biquad1.setCoefficients(3, IIR_40_96k_Coeffs_3);			
            } else
            if (BW < 4401) {
                biquad1.setCoefficients(0, IIR_44_96k_Coeffs_0);
                biquad1.setCoefficients(1, IIR_44_96k_Coeffs_1);
                biquad1.setCoefficients(2, IIR_44_96k_Coeffs_2);
                biquad1.setCoefficients(3, IIR_44_96k_Coeffs_3);			
            } else
            if (BW < 5401) {
                biquad1.setCoefficients(0, IIR_54_96k_Coeffs_0);
                biquad1.setCoefficients(1, IIR_54_96k_Coeffs_1);
                biquad1.setCoefficients(2, IIR_54_96k_Coeffs_2);
                biquad1.setCoefficients(3, IIR_54_96k_Coeffs_3);
            } else
			if (BW < 6001) {
                biquad1.setCoefficients(0, IIR_60_96k_Coeffs_0);
                biquad1.setCoefficients(1, IIR_60_96k_Coeffs_1);
                biquad1.setCoefficients(2, IIR_60_96k_Coeffs_2);
                biquad1.setCoefficients(3, IIR_60_96k_Coeffs_3);
            } else
            if (BW < 6500) {
                biquad1.setCoefficients(0, IIR_65_96k_Coeffs_0);
                biquad1.setCoefficients(1, IIR_65_96k_Coeffs_1);
                biquad1.setCoefficients(2, IIR_65_96k_Coeffs_2);
                biquad1.setCoefficients(3, IIR_65_96k_Coeffs_3);
            } else
            if (BW < 8000) {
                biquad1.setCoefficients(0, IIR_80_96k_Coeffs_0);
                biquad1.setCoefficients(1, IIR_80_96k_Coeffs_1);
                biquad1.setCoefficients(2, IIR_80_96k_Coeffs_2);
                biquad1.setCoefficients(3, IIR_80_96k_Coeffs_3);		
            } else
             {
                biquad1.setCoefficients(0, IIR_110_96k_Coeffs_0); 
                biquad1.setCoefficients(1, IIR_110_96k_Coeffs_1);
                biquad1.setCoefficients(2, IIR_110_96k_Coeffs_2);
                biquad1.setCoefficients(3, IIR_110_96k_Coeffs_3); 					
			 }		             
    } // end else (mode = SSB, DSB)
	// need a high pass here
    if (MO == modeCW)	  
		{ 
		    //AM.passthrough(0); // switches off AM demodulator for the single sideband demodulation modes       	
            BW = bwu;						    
			if (BW < 1801) {
                biquad1.setCoefficients(0, IIR_18_96k_Coeffs_0);
                biquad1.setCoefficients(1, IIR_18_96k_Coeffs_1);
                biquad1.setCoefficients(2, IIR_18_96k_Coeffs_2);
                biquad1.setCoefficients(3, IIR_18_96k_Coeffs_3);			
            } else
            if (BW < 2301) {
                biquad1.setCoefficients(0, IIR_23_96k_Coeffs_0);
                biquad1.setCoefficients(1, IIR_23_96k_Coeffs_1);
                biquad1.setCoefficients(2, IIR_23_96k_Coeffs_2);
                biquad1.setCoefficients(3, IIR_23_96k_Coeffs_3);	
            } 
    } 
	// draw bandwidth indicator under spectrum display
    show_bandwidth(MO, bwu, bwl); 	
    if (bwu < 400) bwu = 400;
    if (bwl < 400) bwl = 400;
	
	float fac = 1;
	
    if ((MO == modeAM)||(MO == modeSAM)) 
	{
      fac = 2.0;	
      biquad3.setLowpass(0, bwu*fac, 0.54);
      biquad3.setLowpass(1, bwu*fac, 1.3);
      biquad3.setLowpass(2, bwu*fac, 0.54);
      biquad3.setLowpass(3, bwu*fac, 1.3);  
	}
    // Linkwitz-Riley filter, 48 dB/octave
    // USB path
    fac = 1.0;
    if (MO == modeUSB) 	{   
	  biquad3.setLowpass(0, bwu*fac, 0.54);
      biquad3.setLowpass(1, bwu*fac, 1.3);
      biquad3.setLowpass(2, bwu*fac, 0.54);
      biquad3.setLowpass(3, bwu*fac, 1.3);	  
	}
	if (MO == modeLSB)  {     
	  biquad3.setLowpass(0, bwl*fac, 0.54);
      biquad3.setLowpass(1, bwl*fac, 1.3);
      biquad3.setLowpass(2, bwl*fac, 0.54);
      biquad3.setLowpass(3, bwl*fac, 1.3);  	
	  }	 
	if (MO == modeDSB)  {     
	  biquad3.setLowpass(0, BW*fac, 0.54);
      biquad3.setLowpass(1, BW*fac, 1.3);
      biquad3.setLowpass(2, BW*fac, 0.54);
      biquad3.setLowpass(3, BW*fac, 1.3);
	  }	     
	if (MO == modeCW)  {   
	  bwl = MIN_BW_CW;	
	  biquad3.setLowpass(0, bwu*fac, 0.54);
      biquad3.setLowpass(1, bwu*fac, 1.3);
      biquad3.setLowpass(2, bwu*fac, 0.54);
      biquad3.setLowpass(3, bwu*fac, 1.3);
	  biquad3.setHighpass(0, bwl*fac, 0.54);
      biquad3.setHighpass(1, bwl*fac, 1.3);
      biquad3.setHighpass(2, bwl*fac, 0.54);
      biquad3.setHighpass(3, bwl*fac, 1.3);	  
	  }	     
	
	  // notchfilter switching is implemented only HERE
      if (notchF >= 400 || notchF <=-400) {
      switch (MO) {
          case modeUSB:
              if (notchF >= 400) { 
              biquad3.setNotch(0, notchF, notchQ);
              biquad3.setNotch(1, notchF, notchQ);
              biquad3.setNotch(2, notchF, notchQ);
              biquad3.setNotch(3, notchF, notchQ);
              } else
              {
              biquad3.setNotch(0, notchF * -1, notchQ);
              biquad3.setNotch(1, notchF * -1, notchQ);
              biquad3.setNotch(2, notchF * -1, notchQ);
              biquad3.setNotch(3, notchF * -1, notchQ);
              }
          break;
          case modeLSB:
              if (notchF <= -400) { 
              biquad3.setNotch(0, notchF * -1, notchQ);
              biquad3.setNotch(1, notchF * -1, notchQ);
              biquad3.setNotch(2, notchF * -1, notchQ);
              biquad3.setNotch(3, notchF * -1, notchQ);
              }
              else {
              biquad3.setNotch(0, notchF, notchQ);
              biquad3.setNotch(1, notchF, notchQ);
              biquad3.setNotch(2, notchF, notchQ);
              biquad3.setNotch(3, notchF, notchQ);
              }
          break;
          case modeAM:
                if (notchF <= -400) {
                     biquad3.setNotch(0, notchF * -1, notchQ);
                     biquad3.setNotch(1, notchF * -1, notchQ);
                     biquad3.setNotch(2, notchF * -1, notchQ);
                     biquad3.setNotch(3, notchF * -1, notchQ);
                }
                else {
                     biquad3.setNotch(0, notchF, notchQ);
                     biquad3.setNotch(1, notchF, notchQ);
                     biquad3.setNotch(2, notchF, notchQ);
                     biquad3.setNotch(3, notchF, notchQ);
                }
          break;
          case modeDSB:        
                if (notchF <= -400) {
                      biquad3.setNotch(0, notchF * -1, notchQ); // LSB path
                      biquad3.setNotch(1, notchF * -1, notchQ); // LSB path
                      biquad3.setNotch(2, notchF * -1, notchQ); // LSB path
                      biquad3.setNotch(3, notchF * -1, notchQ); // LSB path          
                }
                else if (notchF >= 400){     
                      biquad3.setNotch(0, notchF, notchQ);
                      biquad3.setNotch(1, notchF, notchQ);
                      biquad3.setNotch(2, notchF, notchQ);
                      biquad3.setNotch(3, notchF, notchQ);                    
                }
          break;
		  case modeSAM:
                if (notchF <= -400) {
                     biquad3.setNotch(0, notchF * -1, notchQ);
                     biquad3.setNotch(1, notchF * -1, notchQ);
                     biquad3.setNotch(2, notchF * -1, notchQ);
                     biquad3.setNotch(3, notchF * -1, notchQ);
                }
                else {
                     biquad3.setNotch(0, notchF, notchQ);
                     biquad3.setNotch(1, notchF, notchQ);
                     biquad3.setNotch(2, notchF, notchQ);
                     biquad3.setNotch(3, notchF, notchQ);
                }
          break;
		  
		}
    }  
 AudioInterrupts();
}

void setup_TX(int MO, int bwu, int bwl){
	
	AudioNoInterrupts();   // Disable Audio while reconfiguring filters
	//FreqConv.passthrough(0); // pass passthrough audio, dont convert

	audioShield.inputSelect(MicIn); // SSB TX mode uses mic in
	audioShield.micGain(MIC_GAIN);  // have  to adjust mic gain after selecting mic in. The input number is in decibels, from 0 to 63. 
	audioShield.autoVolumeControl(0,1,0,-40.0,12.0,70.0); 
    audioShield.autoVolumeEnable();
	audioShield.lineOutLevel(13);//audioShield.lineOutLevel 13=3,16Vpp...311,16Vpp
	if (TxOn)
	{}
	AudioInterrupts(); 
}
/* #########################################################################
 *  
 * void display time
 *  
 *  
 * ######################################################################### 
 */
void displayClock() {

      uint8_t hour10 = hour ()/10%10;
      uint8_t hour1 = hour()%10;
      uint8_t minute10 = minute()/10%10;
      uint8_t minute1 = minute()%10;
      uint8_t second10 = second()/10%10;
      uint8_t second1 = second()%10;
      uint8_t time_pos_shift = 6;
      uint8_t dp = 5; 

   
      if (!timeflag) {
      tft.setCursor(pos_x_time + 2 * time_pos_shift, pos_y_time);
      tft.print(":");
      tft.setCursor(pos_x_time + 4 * time_pos_shift + dp, pos_y_time);
      tft.print(":");
      
      }
      if (hour10 != hour10_old || !timeflag) {
             tft.setCursor(pos_x_time, pos_y_time);
             tft.fillRect(pos_x_time, pos_y_time, time_pos_shift, 8, BLACK);             
             if (hour10) tft.print(hour10);  // do not display, if zero   
        }
      if (hour1 != hour1_old || !timeflag) {
             tft.setCursor(pos_x_time + time_pos_shift, pos_y_time);
             tft.fillRect(pos_x_time  + time_pos_shift, pos_y_time, time_pos_shift, 8, BLACK);             
             tft.print(hour1);  // always display   
        }
      if (minute1 != minute1_old || !timeflag) {
             tft.setCursor(pos_x_time + 3 * time_pos_shift + dp, pos_y_time);
             tft.fillRect(pos_x_time  + 3 * time_pos_shift + dp, pos_y_time, time_pos_shift, 8, BLACK);             
             tft.print(minute1);  // always display   
        }
      if (minute10 != minute10_old || !timeflag) {
             tft.setCursor(pos_x_time + 2 * time_pos_shift + dp, pos_y_time);
             tft.fillRect(pos_x_time  + 2 * time_pos_shift + dp, pos_y_time, time_pos_shift, 8, BLACK);             
             tft.print(minute10);  // always display   
        }
      if (second10 != second10_old || !timeflag) {
             tft.setCursor(pos_x_time + 4 * time_pos_shift + 2*dp, pos_y_time);
             tft.fillRect(pos_x_time  + 4 * time_pos_shift + 2*dp, pos_y_time, time_pos_shift, 8, BLACK);             
             tft.print(second10);  // always display   
        }
      if (second1 != second1_old || !timeflag) {
             tft.setCursor(pos_x_time + 5 * time_pos_shift + 2*dp, pos_y_time);
             tft.fillRect(pos_x_time  + 5 * time_pos_shift + 2*dp, pos_y_time, time_pos_shift, 8, BLACK);             
             tft.print(second1);  // always display   
        }
      hour1_old = hour1;
      hour10_old = hour10;
      minute1_old = minute1;
      minute10_old = minute10;
      second1_old = second1;
      second10_old = second10;
      timeflag = 1;


} // end void displayTime
/* #########################################################################
 *  
 * voiy display date
 *  
 * ######################################################################### 
 */
void displayDate() {
  char string99 [14]; 
  tft.fillRect(pos_x_date, pos_y_date, 50, 8, BLACK); // erase old string
  tft.setTextColor(WHITE);
  tft.setCursor(pos_x_date, pos_y_date);
  sprintf(string99,"%02d.%02d.%02d", day(),month(),year()-2000);
  tft.print(string99);
}

// ****************************************************************************
// set samplerate code by Frank Boesing
// ****************************************************************************
void setI2SFreq(int freq) {
  typedef struct {
    uint8_t mult;
    uint16_t div;
  } tmclk;

  const int numfreqs = 14;
  const int samplefreqs[numfreqs] = { 8000, 11025, 16000, 22050, 32000, 44100, 44117.64706 , 48000, 88200, 44117.64706 * 2, 96000, 176400, 44117.64706 * 4, 192000};

#if (F_PLL==16000000)
  const tmclk clkArr[numfreqs] = {{16, 125}, {148, 839}, {32, 125}, {145, 411}, {64, 125}, {151, 214}, {12, 17}, {96, 125}, {151, 107}, {24, 17}, {192, 125}, {127, 45}, {48, 17}, {255, 83} };
#elif (F_PLL==72000000)
  const tmclk clkArr[numfreqs] = {{32, 1125}, {49, 1250}, {64, 1125}, {49, 625}, {128, 1125}, {98, 625}, {8, 51}, {64, 375}, {196, 625}, {16, 51}, {128, 375}, {249, 397}, {32, 51}, {185, 271} };
#elif (F_PLL==96000000)
  const tmclk clkArr[numfreqs] = {{8, 375}, {73, 2483}, {16, 375}, {147, 2500}, {32, 375}, {147, 1250}, {2, 17}, {16, 125}, {147, 625}, {4, 17}, {32, 125}, {151, 321}, {8, 17}, {64, 125} };
#elif (F_PLL==120000000)
  const tmclk clkArr[numfreqs] = {{32, 1875}, {89, 3784}, {64, 1875}, {147, 3125}, {128, 1875}, {205, 2179}, {8, 85}, {64, 625}, {89, 473}, {16, 85}, {128, 625}, {178, 473}, {32, 85}, {145, 354} };
#elif (F_PLL==144000000)
  const tmclk clkArr[numfreqs] = {{16, 1125}, {49, 2500}, {32, 1125}, {49, 1250}, {64, 1125}, {49, 625}, {4, 51}, {32, 375}, {98, 625}, {8, 51}, {64, 375}, {196, 625}, {16, 51}, {128, 375} };
#elif (F_PLL==180000000)
  const tmclk clkArr[numfreqs] = {{46, 4043}, {49, 3125}, {73, 3208}, {98, 3125}, {183, 4021}, {196, 3125}, {16, 255}, {128, 1875}, {107, 853}, {32, 255}, {219, 1604}, {214, 853}, {64, 255}, {219, 802} };
#elif (F_PLL==192000000)
  const tmclk clkArr[numfreqs] = {{4, 375}, {37, 2517}, {8, 375}, {73, 2483}, {16, 375}, {147, 2500}, {1, 17}, {8, 125}, {147, 1250}, {2, 17}, {16, 125}, {147, 625}, {4, 17}, {32, 125} };
#elif (F_PLL==216000000)
  const tmclk clkArr[numfreqs] = {{32, 3375}, {49, 3750}, {64, 3375}, {49, 1875}, {128, 3375}, {98, 1875}, {8, 153}, {64, 1125}, {196, 1875}, {16, 153}, {128, 1125}, {226, 1081}, {32, 153}, {147, 646} };
#elif (F_PLL==240000000)
  const tmclk clkArr[numfreqs] = {{16, 1875}, {29, 2466}, {32, 1875}, {89, 3784}, {64, 1875}, {147, 3125}, {4, 85}, {32, 625}, {205, 2179}, {8, 85}, {64, 625}, {89, 473}, {16, 85}, {128, 625} };
#endif

  for (int f = 0; f < numfreqs; f++) {
    if ( freq == samplefreqs[f] ) {
      while (I2S0_MCR & I2S_MCR_DUF) ;
      I2S0_MDR = I2S_MDR_FRACT((clkArr[f].mult - 1)) | I2S_MDR_DIVIDE((clkArr[f].div - 1));
      return;
    }
  }
}


// continousliy retune the oscillator to carrier frequency
//
//
void tune_SAM_PLL(void)
{			
/*
const int loop=16;
			
			// estimate frequ of carrier by three-point-interpolation of bins
			// so range is +/- 800 Hz
			// Hanning 2*
			// Blackmann 2.8*
			// Hamming 1.85
			bin1 = myFFT.output[posbin-1];
			bin2 = myFFT.output[posbin];
			bin3 = myFFT.output[posbin+1];
			if (!cnt)
				{
				average_bin1=0;
				average_bin2=0;
				average_bin3=0;
				average_delta=0;
				}
			cnt++;			
			average_bin1=bin1+average_bin1;
			average_bin2=bin2+average_bin2;
			average_bin3=bin3+average_bin3;		
			delta = binBW * ((-average_bin3 + average_bin1 )) / (average_bin1 + average_bin2  + average_bin3 );
			average_delta = delta+average_delta;				
			if (cnt > loop)	{							
				cnt=0;						
				average_delta=average_delta/(loop-1);		
				// added low pass to retune slowly 			
				bands[band].freq = bands[band].freq - 4.2*average_delta;
				setfreq();		
				Serial.print("dF = ");
				Serial.print(average_delta);				
				}
	*/		
	
}