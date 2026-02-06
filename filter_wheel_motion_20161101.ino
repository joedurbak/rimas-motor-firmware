/*
 * Author: John Capone
 * Arduino online community and code by Jordan Wheeler used as references
 * jicapone@astro.umd.edu
 * 609-915-6213
 * 
 * Purpose: This code is intended to use the Arduino as a stepper motor controller for filter wheels. Step rate ramping is done by the Arduino controller.
 * 
 * To Do:
 *      - replace "attachInterrupt" with custom ISRs to prevent loss of serial data, etc.
 *      - resonant frequency avoidance
 *          - pass a lookup table from PC
 *      - determine which global variables must be volatile, removing this qualifier from other global variables
 */

/////////////////////////////////////
/* * * PREPROCESSOR DIRECTIVES * * */
/////////////////////////////////////

// libraries
#include <avr/interrupt.h>

// options
//#define HOLD_TRQ  // uncommenting this will maintain current through motor coils even when not in motion

//////////////////////////////
/* * * GLOBAL CONSTANTS * * */
//////////////////////////////

// timer constants
const float SYS_CLOCK = 16e6;                       // arduino system clock speed [Hz]
const float IPRESCAL = (1.0/256.0);                 // inverted prescaler value
const float START_FRAC = 0.1;                       // start of ramp is this fraction of user specified maximum rate
const float MAX_RATE = 30000;                       // maximum step frequency [Hz] is (clock speed) / (prescaler) / (1) / (2) -> 16e6 Hz / 256 / 1 / 2 = 31250 Hz
const float MIN_RATE = 1;                           // for 16 bit timers: minimum step frequency [Hz] is (clock speed) / (prescaler) / (2^# of register bits) / (2) -> 16e6 Hz / 256 / 65536 / 2 = ~0.5 Hz (NOTE: must be divided by 2 because step is on low/high transition -> every other interrupt)
const unsigned long HOLD_DELAY = 10;                // time to maintain coil current after a move is completed [ms]

// serial constants
const unsigned long BAUD_RATE = 57600;              // set baud rate [bits/s]
const int MAX_CMD_LEN = 20;                         // approximate max character length of a command
const unsigned int READ_DELAY = MAX_CMD_LEN*10/(BAUD_RATE/1000)+1;                   // wait so the serial buffer can fill [ms]; minimum of 1 ms. Note: 10 bits per serial char

// command constants
const char MOT_CHARS[] = {'0','1','2','3'};         // characters for motor numbers
const unsigned int N_MOT = 4;                       // number of small motors controlled by arduino
const char SEP = ':';                               // delimiter characer -> colon (:)
const char EOT = '\n';                              // end of transmission character
const char EOS = 0x00;                              // end of string character is ASCII code for NULL
const char ACK = 0x06;                              // positive acknowledge ASCII code
const char NAK = 0x15;                              // negative acknowledge ASCII code
const boolean DIR_POS = true;                       // positive direction
const boolean DIR_NEG = false;                      // negative direction
const unsigned long FR_STEPS = -1;                  // maximum unsigned long for # steps -> free run
const long NO_RAMP = 0;                             // 0 acceleration -> no ramp -> start at maximum step rate

// motor constants
const boolean ENBL_OFF = HIGH;                      // SMALL motor is off when enable pin is high
const boolean STEP_DFLT = LOW;                      // step pin should be low before enabling motor

// switch constants
const int SWITCH_INT[] = {0,1,3,2};                 // interrupt numbers corresponding to switch pins (pins --> 2, 3, 20, 21 - 18 and 19 are used by Serial1)
const boolean SWITCH_ACTIVE = LOW;                 // level for activated switch, ************************ may change later ************************
const unsigned char SWITCH_MODE = CHANGE;           // when the interrupt is triggered: LOW, CHANGE, RISING, FALLING
const float SWITCH_LIMIT = 2000;

// pin constants
const int SWITCH_PIN[] = {2,3,20,21};               // pin numbers for position switch switch interrupts
const int ENBL_PIN[] = {22,23,24,25};               // pin numbers for motor enables (note: active low -> LOW==enabled)
const int DIR_PIN[] = {26,28,30,32};                // pin numbers for motor directions (note: HIGH==CCW, LOW==CW)
const int STEP_PIN[] = {27,29,31,33};               // pin numbers for motor steps (note: step triggered by rising edge, pin must be high/low for at least 1 us)

//////////////////////////////
/* * * GLOBAL VARIABLES * * */
//////////////////////////////

// timing variables
volatile unsigned long serial_time = 0;                     // serial start time [ms]
volatile unsigned long hold_time[] = {0,0,0,0};             // hold current start time [ms]

// timer interrupt variables
volatile unsigned int tcnt[] = {0,0,0,0};
volatile boolean toggle[] = {LOW,LOW,LOW,LOW};

// switch variables
volatile unsigned long switch_count[] = {0,0,0,0};          // counter for number of switch triggers detected
volatile boolean switch_stop[] = {false,false,false,false}; // true if switch stop conditions are met
volatile boolean switch_hold[] = {false,false,false,false}; // true if holding before reversing direction for return
volatile boolean returning[] = {false,false,false,false};   // true if motor is returning to target position

// command storage
volatile boolean mot_state[] = {false,false,false,false};   // on or off for each motor [false = off, true = on]
volatile boolean direction[] = {false,false,false,false};   // direction of each motor [false = cw, true = ccw]
volatile boolean switch_move[] = {false,false,false,false}; // motor is performing either a switch move or a relative move (if this variable is false)
volatile unsigned long start_steps[] = {0,0,0,0};           // original number of steps for each motor [steps]
volatile float min_rate[] = {MIN_RATE,MIN_RATE,MIN_RATE,MIN_RATE}; // starting rate for motor
volatile float max_rate[] = {MIN_RATE,MIN_RATE,MIN_RATE,MIN_RATE};  // max rate for each motor [steps/s]
volatile float ramp[] = {0,0,0,0};                          // acceleration/deceleration ramp of each motor [steps/s^2]
volatile unsigned long npos[] = {0,0,0,0};                  // number of positions to move during switch move
volatile boolean e_stop[] = {false,false,false,false};       // false = normal opperation, true = emergency stop
volatile int emerg_mult[] = {-1,-1,-1,-1};                  // default ramp slope multiplier for emergency stop (-2 -> 2x deceleration)

// move variables
volatile float ramp_mult[] = {0,0,0,0};                     // ramp multiplier. negative = deceleration, zero = constant rate, positive = acceleration
volatile float step_rate[] = {MIN_RATE,MIN_RATE,MIN_RATE,MIN_RATE};                     // current step rate for each motor [steps/s]
volatile boolean holding[] = {false,false,false,false};     // is the motor maintaining a holding current following a move?

// counters
volatile unsigned long acc_steps[] = {0,0,0,0};             // to hold the number of steps for which the motor accelerated [steps]
volatile unsigned long remaining_steps[] = {0,0,0,0};       // remaining number of steps for each motor [steps]
volatile unsigned long steps_taken[] = {0,0,0,0};           // total number of steps taken --> independent counter [steps]

void setup(){
    
    ///////////////////////////////
    /* * * setup pin outputs * * */
    ///////////////////////////////
    int i = 0;
    
    // Arduino motors
    for(i = 0; i < N_MOT; i++){
        pinMode(DIR_PIN[i], OUTPUT);
        digitalWrite(DIR_PIN[i], LOW);            // set to low for now
        pinMode(STEP_PIN[i], OUTPUT);
        digitalWrite(STEP_PIN[i], STEP_DFLT);     // set to low for now
        pinMode(ENBL_PIN[i], OUTPUT);
        #ifdef HOLD_TRQ
            digitalWrite(ENBL_PIN[i], !ENBL_OFF); // active low = enabled (motors on)
        #else
            digitalWrite(ENBL_PIN[i], ENBL_OFF);  // active high = disabled (motors off)
        #endif
    }

    //////////////////////////////
    /* * * setup pin inputs * * */
    //////////////////////////////
    for(i = 0; i < N_MOT; i++){
        pinMode(SWITCH_PIN[i], INPUT);    // INPUT sets up pin as input pin with internal pull-up resistor disabled. To use pull-up resistor, change to INPUT_PULLUP.
    }
    
    ////////////////////////////////////////
    /* * * open serial communications * * */
    ////////////////////////////////////////
    
    //Serial2.begin(BAUD_RATE);
    Serial.begin(BAUD_RATE);
    
    /////////////////////////////////////////////
    /* * * MOTOR#1: setup TIMER1 registers * * */
    /////////////////////////////////////////////
    
    // turn off all interrupts
    TIMSK1 &= ~((1<<ICIE1) | (1<<OCIE1B) | (1<<OCIE1A) | (1<<TOIE1));
        
    // set normal mode and disable compares
    TCCR1A &= ~((1<<COM1A1) | (1<<COM1A0) | (1<<COM1B1) | (1<<COM1B0) | (1<<WGM11) | (1<<WGM10));
    TCCR1B &= ~((1<<ICNC1) | (1<<ICES1) | (1<<WGM13) | (1<<WGM12));
    TCCR1C &= ~((1<<FOC1A) | (1<<FOC1B));
    
    // use system clk, prescaling by 1/256
    TCCR1B &= ~((1<<CS11) | (1<<CS10));
    TCCR1B |= (1<<CS12);
    
    /////////////////////////////////////////////
    /* * * MOTOR#2: setup TIMER3 registers * * */
    /////////////////////////////////////////////
    
    // turn off all interrupts
    TIMSK3 &= ~((1<<ICIE3) | (1<<OCIE3B) | (1<<OCIE3A) | (1<<TOIE3));
    
    // set normal mode and disable compares
    TCCR3A &= ~((1<<COM3A1) | (1<<COM3A0) | (1<<COM3B1) | (1<<COM3B0) | (1<<WGM31) | (1<<WGM30));
    TCCR3B &= ~((1<<ICNC3) | (1<<ICES3) | (1<<WGM33) | (1<<WGM32));
    TCCR3C &= ~((1<<FOC3A) | (1<<FOC3B));
    
    // use system clk, prescaling by 1/256
    TCCR3B &= ~((1<<CS31) | (1<<CS30));
    TCCR3B |= (1<<CS32);
    
    /////////////////////////////////////////////
    /* * * MOTOR#3: setup TIMER4 registers * * */
    /////////////////////////////////////////////
    
    // turn off all interrupts
    TIMSK4 &= ~((1<<ICIE4) | (1<<OCIE4B) | (1<<OCIE4A) | (1<<TOIE4));
    
    // set normal mode and disable compares
    TCCR4A &= ~((1<<COM4A1) | (1<<COM4A0) | (1<<COM4B1) | (1<<COM4B0) | (1<<WGM41) | (1<<WGM40));
    TCCR4B &= ~((1<<ICNC4) | (1<<ICES4) | (1<<WGM43) | (1<<WGM42));
    TCCR4C &= ~((1<<FOC4A) | (1<<FOC4B));
    
    // use system clk, prescaling by 1/256
    TCCR4B &= ~((1<<CS41) | (1<<CS40));
    TCCR4B |= (1<<CS42);
    
    /////////////////////////////////////////////
    /* * * MOTOR#4: setup TIMER5 registers * * */
    /////////////////////////////////////////////
    
    // turn off all interrupts
    TIMSK5 &= ~((1<<ICIE5) | (1<<OCIE5B) | (1<<OCIE5A) | (1<<TOIE5));
    
    // set normal mode and disable compares
    TCCR5A &= ~((1<<COM5A1) | (1<<COM5A0) | (1<<COM5B1) | (1<<COM5B0) | (1<<WGM51) | (1<<WGM50));
    TCCR5B &= ~((1<<ICNC5) | (1<<ICES5) | (1<<WGM53) | (1<<WGM52));
    TCCR5C &= ~((1<<FOC5A) | (1<<FOC5B));
    
    // use system clk, prescaling by 1/256
    TCCR5B &= ~((1<<CS51) | (1<<CS50));
    TCCR5B |= (1<<CS52);
    
    // set reload for step frequency of 1008.0645161290323 (->0.000992 s period) -> HIGH/LOW change at 2016.1290322580646 Hz (->0.000496 s half-period).  need reload every 31 cycles.  16 bit overflow register -> max value of 65536.  65536 - 31 = 65505 for reload value.
    for(i = 0; i < N_MOT; i++){
        tcnt[i] = sixteen_bit_reset(MIN_RATE);
        update_TCNT(i);
    }   
    
    //////////////////////////////////////
    /* * * switch: setup interrupts * * */
    //////////////////////////////////////
    //  Note: unlike timer interrupts, this used the Arduino library function "attachInterrupt()" which sets the registers and creates the ISR from a normal function.
    
    // adruino switch interrupts
    attachInterrupt(SWITCH_INT[0], switch_trigger_0, SWITCH_MODE);
    attachInterrupt(SWITCH_INT[1], switch_trigger_1, SWITCH_MODE);
    attachInterrupt(SWITCH_INT[2], switch_trigger_2, SWITCH_MODE);
    attachInterrupt(SWITCH_INT[3], switch_trigger_3, SWITCH_MODE);
    
    // ensure interrupts are activated
    interrupts();
    
}

void loop(){
    
    ///////////////////////////////////////////////
    /* * * poll serial port for new commands * * */
    ///////////////////////////////////////////////
    //if(Serial2.available() > 0){
    if(Serial.available() > 0){
        if(serial_time == 0)
            serial_time = millis(); // set serial_time if not already set
        else if(serial_ready()){ // if enough serial_time has elapsed, retrieve the data
            if(!get_command()){
                com_fail();
            }
            else{
                com_success();
            }
        }
    }

    ///////////////////////////////////////////////////////
    /* * * check for motors holding following a move * * */
    ///////////////////////////////////////////////////////
    for(int i = 0; i < N_MOT; i++){
        if(holding[i]){   // waiting to end move
            //Serial.print("[DEBUG] holding:"); Serial.print(i); Serial.print(EOT);
            if(millis() - hold_time[i] >= HOLD_DELAY){
                holding[i] = false;
                set_mot_state(i);   // turn off motor current
            }
        }
        else if(switch_hold[i]){  // waiting to reverse move
            //Serial.println("DEBUG: switch_hold");
            // is the switch pin hight or low?
            boolean sw_state = digitalRead(SWITCH_PIN[i]);
            if(sw_state){ // if switch is still active, then end the move.
                switch_stop[i] = true;
                steps_on(i);    // restart steps to end move
            }
            else if(millis() - hold_time[i] >= HOLD_DELAY){
                switch_hold[i] = false;
                returning[i] = true;
                direction[i] = !direction[i];   // reverse direction direction variable
                set_mot_dir(i); // pass direction to motor
                steps_on(i);    // restart steps
            }
        }
    }
    
}

// update TCNT register for motor #mot
void update_TCNT(int mot){
    switch(mot){
        case 0:
            TCNT1 = tcnt[0];
            break;
        case 1:
            TCNT3 = tcnt[1];
            break;
        case 2:
            TCNT4 = tcnt[2];
            break;
        case 3:
            TCNT5 = tcnt[3];
            break;
    }
}

// general ISR function for 16-bit timers
void timer_isr(int mot){
    //Serial.print("mot_state[2]:"); Serial.print(mot_state[2]); 
    //Serial.print(", e_stop[2]:");    Serial.print(e_stop[2]);
    //Serial.print(", switch_move[2]:");    Serial.print(switch_move[2]);
    //Serial.print(", remaining_steps[2]:");    Serial.print(remaining_steps[2]);
    //Serial.print(EOT); 
   
    if(mot_state[mot]){        
        // stopping procedure
        if(e_stop[mot]){
            
            if(ramp[mot] == NO_RAMP){
                reset_mot(mot);
            }
            else if(step_rate[mot] == min_rate[mot]){
                reset_mot(mot);
            }
            else{
                update_TCNT(mot);
                digitalWrite(STEP_PIN[mot], toggle[mot] == 0 ? HIGH : LOW);
                //toggle[mot] = ~toggle[mot];
                toggle[mot] = !toggle[mot];
                if(toggle[mot]){
                    steps_taken[mot]++;
                }
            }
        }
        
        // switch move procedure
        else if(switch_move[mot]){
            if(!switch_stop[mot]){
                update_TCNT(mot);
                digitalWrite(STEP_PIN[mot], toggle[mot] == 0 ? HIGH : LOW);
                //toggle[mot] = ~toggle[mot];
                toggle[mot] = !toggle[mot];
                if(toggle[mot]){
                    if(!returning[mot]){
                        if((remaining_steps[mot] != FR_STEPS) && (remaining_steps[mot] != 0)){
                            remaining_steps[mot]--;
                        }
                        steps_taken[mot]++;
                    }
                    else{
                        steps_taken[mot]--; // moving in opposite direction
                    }
                }
            }
            else{
                reset_mot(mot);
            }
        }
        
        // relative move procedure
        else{
            if(remaining_steps[mot] != 0){
                update_TCNT(mot);
                digitalWrite(STEP_PIN[mot], toggle[mot] == 0 ? HIGH : LOW);
                //Serial.print("Point0: toggle[2]:"); Serial.print(toggle[2]); Serial.print(EOT);
                //toggle[mot] = ~toggle[mot];
                toggle[mot] = !toggle[mot];
                
                //Serial.print("Point1: toggle[2]:"); Serial.print(toggle[2]); Serial.print(EOT);
                if(toggle[mot]){
                    if(remaining_steps[mot] != FR_STEPS){
                        remaining_steps[mot]--;
                    }
                    steps_taken[mot]++;
                }
                //Serial.print("[DEBUG]"); Serial.print( steps_taken[mot] ); Serial.print(EOT);
            }
            else{
                reset_mot(mot);
            }
        }
        
        // update step rate after each step
        if(toggle[mot]){
            update_reset(mot);
        }
    }
    
}

// ISR for TIMER1 for motor 1 - 16 bit register
ISR(TIMER1_OVF_vect){
    timer_isr(0);
}

// ISR for TIMER3  for motor 2 - 16 bit register
ISR(TIMER3_OVF_vect) {
    timer_isr(1);
}

// ISR for TIMER4  for motor 3 - 16 bit register
ISR(TIMER4_OVF_vect) {
    timer_isr(2);
}

// ISR for TIMER5  for motor 4 - 16 bit register
ISR(TIMER5_OVF_vect) {
    timer_isr(3);
}

// function for all switch ISRs
void switch_trigger(int mot){

    // only perform actions if motor is active
    if(mot_state[mot]){

        // is the switch pin hight or low?
        boolean sw_state = digitalRead(SWITCH_PIN[mot]);
        //Serial.print("DEBUG: sw_state: "); Serial.print( sw_state); Serial.print(", steps_taken[mot]"); Serial.print(steps_taken[mot]); Serial.print(EOT);
        
        sw_state = sw_state==SWITCH_ACTIVE; // correct polarity

        // tell user that switch state changed
        char qresp[] = {'Q',MOT_CHARS[mot],'S','_',EOS};
        if(sw_state)
            qresp[3] = 'H';
        else
            qresp[3] = 'L';
        response_telegram(qresp);

        // handle switch change to high
        if(sw_state){

            // during a switch move, this is a position on a filter wheel.
            if(switch_move[mot]){

                // increment the switch trigger counter
                switch_count[mot]++;
                
                // if the end position's trigger has been detected
                if(switch_count[mot] == npos[mot]){
                    if((remaining_steps[mot] == 0) || (ramp[mot] == 0)){  // if there are no steps remaining, then motor is decelerated and can safely stop
                        switch_stop[mot] = true;
                    }
                    else{   // if the motor is at the position or has overstepped the target position, begin free run operation
                        remaining_steps[mot] = FR_STEPS;
                    }
                }
                else if(switch_count[mot] == npos[mot] + 1){  // if the motor has overstepped the end position, then motor has returned and should stop
                    switch_stop[mot] = true;
                }

            }

            // during relative move, this is a limit switch.  execute emergency stop
            else{
                e_stop[mot] = true;
            }

        }

    }
    
}

// "ISR" for switch trigger, Arduino function sets up true ISR
void switch_trigger_0(){
    Serial.print("[DEBUG: T0_Step0("); Serial.print(steps_taken[0]); Serial.print(")]");
    Serial.print("[DEBUG: T0_Step1("); Serial.print(steps_taken[1]); Serial.print(")]");
    Serial.print(EOT);
    //switch_trigger(0);
    if (steps_taken[0] > SWITCH_LIMIT ) switch_trigger(0);
}

// "ISR" for switch trigger, Arduino function sets up true ISR
void switch_trigger_1(){
    Serial.print("[DEBUG: T1_Step0("); Serial.print(steps_taken[0]); Serial.print(")]");
    Serial.print("[DEBUG: T1_Step1("); Serial.print(steps_taken[1]); Serial.print(")]");
    Serial.print(EOT);
    //switch_trigger(1);
    if (steps_taken[1] > SWITCH_LIMIT ) switch_trigger(1); 
}

// "ISR" for switch trigger, Arduino function sets up true ISR
void switch_trigger_2(){
    //switch_trigger(2);
    if (steps_taken[2] > SWITCH_LIMIT ) switch_trigger(2);
}

// "ISR" for switch trigger, Arduino function sets up true ISR
void switch_trigger_3(){
    //switch_trigger(3);
    if (steps_taken[3] > SWITCH_LIMIT ) switch_trigger(3);
}

// print function for Serial2 which enforces efficiency
void serial_print(char *str){
    int i = 0;
    //while(str[i] != EOS){
    while(str[i] != EOS && str[i] != ACK ){
        //Serial2.write(str[i]);
        Serial.write(str[i]);
        i++;
    }

    /*
    i = 0;
    while(str[i] != EOS && str[i] != ACK ){
        Serial.print(str[i],HEX); Serial.print(EOT);
        i++;
    }
    */
    
}

// return the length of a string
int str_len(char *str){
    int len = 0;
    while(str[len] != EOS){
        len++;
    }
    return len;
}

// correctly format response telegram to Serial2
void response_telegram(char *msg){
    int len = str_len(msg);
    char fmsg[len+2];   // formated message
    for(int i = 0; i < len; i++){
        fmsg[i] = msg[i];
    }
    fmsg[len] = EOT;
    fmsg[len+1] = EOS;
    serial_print(fmsg);
}

// successful communication acknowledgement
void com_success(){
    char msg[] = {ACK, EOT, EOS};
    serial_print(msg);
    Serial.print("[DEBUG]com_success\n");
}

// failed communication acknowledgement
void com_fail(){
    char msg[] = {NAK, EOT, EOS};
    serial_print(msg);
    Serial.print("[DEBUG]com_fail\n");
}

/*
 * function to determine if the serial data is ready to read or not
 *  this way, you don't need to stop the arduino from running the other motors
 *  use the millis() function to do a passive count
 */
boolean serial_ready(){
    boolean success = false;
    if((millis() - serial_time) > READ_DELAY){
        success = true;
        serial_time = 0;
    }
    return success;
}

// check whether c is a digit (0-9), return True or False
boolean is_digit(char c){
    if(c>=48 && c<=57)   // decimal 48 == ASCII 0, decimal 57 == ASCII 9
        return true;
    else
        return false;
}

// convert digits from string to unsigned long int
unsigned long str2ulong(char* str, int len){
    if(len <= 0){ return 0; }
    unsigned long i = 0;
    unsigned long n = 0;
    unsigned long l = 1;
    unsigned long q = 0;
    for(i = len-1; i != -1; i--){
        q = str[i] - '0';
        n = n + q * l;
        l = l * 10;
    }
    return n;
}

// convert digits from string to int
int str2int(char* str, int len){
    unsigned long n = str2ulong(str, len);
    return (int) n;
}

// calculate integer powers of 10
int pow10(int exp){
    int x = 1;  // 10^0
    for(int i = 0; i < exp; i++)
        x *= 10;
    return x;
}

// convert int to string         *** remember to free !!! ***
char* int2str(int val){
    // get length of final string
    int i = 10;
    int j = 1;
    while(val % i != val){
        i *= 10;
        j++;
    }
    char* str = (char*) malloc((j+1) * sizeof(char)); // +1 for EOS
    if(str == NULL){
        return NULL;
    }
    else{
        for(i = 0; i < j; i++){
            str[j-i-1] = ((val%pow10(i+1) - val%pow10(i))/pow10(i)) + 48;   // 48 is decimal ASCII value for 0
        }
        str[j] = EOS;
        return str;
    }
}

// convert long to string         *** remember to free !!! ***
char* long2str(long val){
    // get length of final string
    long i = 10;
    long j = 1;
    while(val % i != val){
        i *= 10;
        j++;
    }
    char* str = (char*) malloc((j+1) * sizeof(char)); // +1 for EOS
    if(str == NULL){
        return NULL;
    }
    else{
        for(i = 0; i < j; i++){
            str[j-i-1] = ((val%pow10(i+1) - val%pow10(i))/pow10(i)) + 48;   // 48 is decimal ASCII value for 0
        }
        str[j] = EOS;
        return str;
    }
}

/* 
 * once the serial is ready, fetch the data and parse it
 *
 *  Check communications:
 *      [C]|[check code]|[EOT]
 *
 *  Move:
 *      [M]|Motor # [0-3]|Motor State [1]|Motor Direction [+,-]|Type of Move [R,S]|# Steps [uint]|Delimiter [:]|Max. Rate [uint]|Delimiter [:]|Ramp Slope [uint]|Delimiter [:] (optional)|# Positions [uint] (optional)|[EOT]
 *          * type of move: R --> relative move, S --> switch move
 *          * if number of steps is 0 --> free run
 *          * if Ramp Slope == 0 -> no ramp (start at max. rate)
 *          * may specify the number of position to move for a switch move.  if not specified, defaults to 1 position.
 *
 *  Emergency Stop:
 *      [M]|Motor # [0-3]|Motor State [0]|[EOT]
 *
 *  Get Motor Completed Steps:
 *      [Q]|Motor # [0-3]|[C]|[EOT]
 *  Get Motor Direction:
 *      [Q]|Motor # [0-3]|[D]|[EOT]
 *  Get Motor Step Rate:
 *      [Q]|Motor # [0-3]|[R]|[EOT]
 *  Get Switch State:
 *      [Q]|Motor # [0-3]|[S]|[EOT]
 *
 *  Set Emergency Ramp Multiplier:
 *      [V]|Motor # [0-3]|[E]|Multiplier [uint]|[EOT]
 */
boolean get_command(){
    
    boolean success = false;
    int mot = -1; // holder for motor number
    int i = 0;
    //int k = Serial2.available();
    int k = Serial.available();
    
    if(k > 0){ // double check that there is stuff to read
        
        char cmd_str[k];
        for (i = 0; i < k; i++){
            //cmd_str[i] = Serial2.read();
            cmd_str[i] = Serial.read();
            //Serial.print(cmd_str[i], HEX); Serial.print(EOT);
        }
        
        Serial.print("[Command Recv("); Serial.print(k); Serial.print(")]");
        Serial.print(cmd_str);
        Serial.print(EOT);
        
        // current command index
        int cmd_ind = 0;
        
        /*
         * (1a) comms check command
         */
        if(cmd_str[cmd_ind] == 'C'){
            int istart = cmd_ind;
            while(cmd_str[cmd_ind] != EOT){
                cmd_ind++;
            }
            cmd_str[cmd_ind] = EOS;
            response_telegram(&cmd_str[istart]); // echo code back to user
            success = true;
        }
        
        /* 
         * (1b) motion modification command
         */
        else if(cmd_str[cmd_ind] == 'M'){            
            cmd_ind++;

            // get motor number and check that it is valid
            if(!is_digit(cmd_str[cmd_ind]))
                goto get_command_bailout;
            mot = cmd_str[cmd_ind] - '0'; // char to int conversion
            if(mot < 0 || mot > 3)
                goto get_command_bailout;
            cmd_ind++;
            Serial.print("Motor ID     :"); Serial.print(mot); Serial.print(EOT);

            /*
             *  (2a) emergency stop command
             */
            if(cmd_str[cmd_ind] == '0'){
                if(mot_state[mot]){ // do only if motor is on
                    e_stop[mot] = true;
                }
                success = true; // return successful parsing either way
                goto get_command_bailout;
            }   // close emergency stop code block
            
            /* 
             * (2b) move command
             */
            else if(cmd_str[cmd_ind] == '1'){
                Serial.print("State        :"); Serial.print(cmd_str[cmd_ind]); Serial.print(EOT);

                cmd_ind++;

                // get move direction
                if(cmd_str[cmd_ind] == '+'){
                    direction[mot] = DIR_POS; // positive == ccw
                }
                else if(cmd_str[cmd_ind] == '-'){
                    direction[mot] = DIR_NEG; // negative == cw
                }
                else{
                    goto get_command_bailout;
                }
                cmd_ind++;
                Serial.print("Direction    :"); Serial.print(direction[mot]); Serial.print(EOT);
                
                // get move type
                if(cmd_str[cmd_ind] == 'R'){  // R == relative move
                    switch_move[mot] = false;
                }
                else if(cmd_str[cmd_ind] == 'S'){ // S == switch move
                    switch_move[mot] = true;
                }
                else{
                    goto get_command_bailout;
                }
                cmd_ind++;
                Serial.print("Switch       :"); Serial.print(switch_move[mot]); Serial.print(EOT);

                // get number of steps
                int istart;
                if(cmd_str[cmd_ind] - '0' == 0){  // check for free run
                    start_steps[mot] = remaining_steps[mot] = FR_STEPS;
                    cmd_ind++;  // advance to delimeter character
                }
                else{
                    istart = cmd_ind;
                    while(cmd_str[cmd_ind] != SEP){
                        if(!is_digit(cmd_str[cmd_ind]))  // only ints accepted
                            goto get_command_bailout;
                        cmd_ind++;
                    }
                    start_steps[mot] = remaining_steps[mot] = str2ulong(&cmd_str[istart], cmd_ind-istart);    // store number of steps to take
                }
                cmd_ind++;  // skip delimiter character
                Serial.print("# of steps   :"); Serial.print(start_steps[mot]); Serial.print(EOT);

                // get maximum step rate [steps/s]
                istart = cmd_ind;
                while(cmd_str[cmd_ind] != SEP){
                    if(!is_digit(cmd_str[cmd_ind]))  // only ints accepted
                        goto get_command_bailout;
                    cmd_ind++;
                }
                max_rate[mot] = str2ulong(&cmd_str[istart], cmd_ind-istart);    // store maximum step rate
                cmd_ind++;  // skip delimiter character
                Serial.print("Step max rate:"); Serial.print(max_rate[mot]); Serial.print(EOT);             

                // get ramp slope [steps/s^2]
                istart = cmd_ind;
                while(cmd_str[cmd_ind] != SEP && cmd_str[cmd_ind] != EOT){
                    if(!is_digit(cmd_str[cmd_ind]))  // only ints accepted
                        goto get_command_bailout;
                    cmd_ind++;
                }
                ramp[mot] = str2ulong(&cmd_str[istart], cmd_ind-istart);    // store acceleration ramp
                Serial.print("Ramp Slope   :"); Serial.print(ramp[mot]); Serial.print(EOT);

                // get number of positions to move, defaults to 1
                if(cmd_str[cmd_ind] != EOT){
                    cmd_ind++;  // skip delimiter character
                    
                    istart = cmd_ind;
                    while(cmd_str[cmd_ind] != EOT){
                        if(!is_digit(cmd_str[cmd_ind]))  // only ints accepted
                            goto get_command_bailout;
                        cmd_ind++;
                    }
                    npos[mot] = str2ulong(&cmd_str[istart], cmd_ind-istart);    // store number positions
                }
                else if(switch_move[mot])
                    npos[mot] = 1;    // default number positions to move
                Serial.print("# of position:"); Serial.print(npos[mot]); Serial.print(EOT);
                
                init_mot(mot); // start motor
                success = true; // got to the end

            } // close move command code block

            else
                goto get_command_bailout;

        } // close motion modification command code block

        /* 
         * (1c) user query command
         */
        else if(cmd_str[cmd_ind] == 'Q'){
            
            cmd_ind++;
            
            // get motor number and check that it is valid
            if(!is_digit(cmd_str[cmd_ind]))
                goto get_command_bailout;
            mot = cmd_str[cmd_ind] - '0'; // char to int conversion
            if(mot < 0 || mot > 3)
                goto get_command_bailout;
            cmd_ind++;

            // determine request
            switch(cmd_str[cmd_ind]){
                // controller returns # completed steps
                case 'C':{
                    char *var_str = long2str(steps_taken[mot]);
                    int len = str_len(var_str);
                    char qresp[len+4];
                    qresp[0] = 'Q'; qresp[1] = MOT_CHARS[mot]; qresp[2] = 'C';
                    for(int j=0; j<len; j++){
                        qresp[j+3] = var_str[j];
                    }
                    qresp[len+3] = EOS;
                    response_telegram(qresp);
                    free(var_str);
                    success = true;
                    break;
                }
                // controller returns motor direction
                case 'D':{
                    char qresp[] = {'Q',MOT_CHARS[mot],'D','_',EOS};
                    if(direction[mot] == DIR_POS)
                        qresp[3] = '+';
                    else
                        qresp[3] = '-';
                    response_telegram(qresp);
                    success = true;
                    break;
                }
                // controller returns step rate
                case 'R':{
                    char *var_str = long2str(step_rate[mot]);
                    int len = str_len(var_str);
                    char qresp[len+4];
                    qresp[0] = 'Q'; qresp[1] = MOT_CHARS[mot]; qresp[2] = 'R';
                    for(int j=0; j<len; j++){
                        qresp[j+3] = var_str[j];
                    }
                    qresp[len+3] = EOS;
                    response_telegram(qresp);
                    free(var_str);
                    success = true;
                    break;
                }
                // controller returns switch state
                case 'S':{
                    boolean sw_state = digitalRead(SWITCH_PIN[mot]);
                    char qresp[] = {'Q',MOT_CHARS[mot],'S','_',EOS};
                    if(sw_state)
                        qresp[3] = 'H';
                    else
                        qresp[3] = 'L';
                    response_telegram(qresp);
                    success = true;
                    break;
                }
            }

        }   // close query code block
        
        /* 
         * (1d) check for variable assignment
         */
        else if(cmd_str[cmd_ind] == 'V'){
            
            cmd_ind++;
            
            // get motor number and check that it is valid
            if(!is_digit(cmd_str[cmd_ind]))
                goto get_command_bailout;
            mot = cmd_str[cmd_ind] - '0'; // char to int conversion
            if(mot < 0 || mot > 3)
                goto get_command_bailout;
            cmd_ind++;

            // determine variable
            int istart;
            switch(cmd_str[cmd_ind]){
                case 'E':   // emergency ramp multiplier *** user specifies w/o sign - always negative ***
                    cmd_ind++;
                    istart = cmd_ind;
                    while(cmd_str[cmd_ind] != EOT){
                        if(!is_digit(cmd_str[cmd_ind])){  // only ints accepted
                            goto get_command_bailout;
                        }
                        cmd_ind++;
                    }
                    emerg_mult[mot] = -1 * str2ulong(&cmd_str[istart], cmd_ind-istart); // store emergency ramp multiplier
                    success = true;
                    break;
            }

        }   // close variable assignment code block
        
    }   // close buffer check code block
    
    get_command_bailout:;
    //Serial.print("[Complete command analysis]\n");
    //Serial2.flush(); // clean out Serial2           ************** CHECK THIS - does it clear buffer? **************
    Serial.flush(); // clean out Serial2           ************** CHECK THIS - does it clear buffer? **************
    return success;
    
}

// reset initial conditions for all motors
void reinit(){
    
    int i = 0;
    
    serial_time = 0;
    for(i = 0; i < N_MOT; i++){
       reset_mot(i);
    }
    
}

// reset a motor to initial conditions
void reset_mot(int mot){

    // turn off motor interrupt
    steps_off(mot);
    
    // telegram that run is complete - return number of steps taken
    char *tstr = long2str(steps_taken[mot]);
    int len = str_len(tstr);
    char msg[len+6];
    msg[0] = 'E'; msg[1] = 'O'; msg[2] = 'R'; msg[3] = MOT_CHARS[mot]; msg[4] = SEP; msg[len+5] = EOS; 
    for(int j=0; j<len; j++){
        msg[j+5] = tstr[j];
    }
    free(tstr);
    response_telegram(msg);
    
    // timer interrupt variables
    tcnt[mot] = sixteen_bit_reset(MIN_RATE);
    toggle[mot] = LOW;
    
    // switch variables
    switch_count[mot] = 0;
    switch_stop[mot] = false;
    switch_hold[mot] = false;
    returning[mot] = false;
    
    // command storage
    mot_state[mot] = false;
    switch_move[mot] = false;
    start_steps[mot] = 0;
    min_rate[mot] = MIN_RATE;
    max_rate[mot] = MIN_RATE;
    ramp[mot] = 0;
    npos[mot] = 0;
    e_stop[mot] = false;
    
    // move variables
    ramp_mult[mot] = 0;
    step_rate[mot] = MIN_RATE;
    
    // counters
    acc_steps[mot] = 0;
    remaining_steps[mot] = 0;
    steps_taken[mot] = 0;
    
    // additional housekeeping
    holding[mot] = true;    // keep current on to hold motor in place following completed move
    hold_time[mot] = millis();  // set hold start time
    digitalWrite(STEP_PIN[mot], STEP_DFLT); // set to low for now
    
}

// initialize a motor for operation
void init_mot(int mot){
    
    // start at min rate unless acceleration == 0 steps/s^2
    if(((int) ramp[mot]) != 0){
        min_rate[mot] = max_rate[mot] * START_FRAC;
        if(min_rate[mot] < MIN_RATE){
            min_rate[mot] = MIN_RATE;
        }
        step_rate[mot] = min_rate[mot];
    }
    else{
        min_rate[mot] = step_rate[mot] = max_rate[mot];
    }
    tcnt[mot] = sixteen_bit_reset(step_rate[mot]);
    
    // turn motor on
    mot_state[mot] = true;
    set_mot_state(mot);
    
    // set direction
    set_mot_dir(mot);
    
    // turn on interrupt
    steps_on(mot);
    
}

// calculate reset value for 16-bit register
int sixteen_bit_reset(float rate){
    return (int) (65536.0 - SYS_CLOCK * IPRESCAL / (2.0 * rate));
}

// turn motor on/off
void set_mot_state(int mot){
    #ifdef HOLD_TRQ
        digitalWrite(ENBL_PIN[mot],!ENBL_OFF); // false == LOW == on
    #else
        digitalWrite(ENBL_PIN[mot],!mot_state[mot]); // true == HIGH == off, false == LOW == on
    #endif
    char qresp[] = {'Q',MOT_CHARS[mot],'M','_',EOS};
    if(mot_state[mot])
        qresp[3] = '1';
    else
        qresp[3] = '0';
    response_telegram(qresp);
}

// set motor direction
void set_mot_dir(int mot){
    digitalWrite(DIR_PIN[mot],direction[mot]); // true == ccw, false == cw
    char qresp[] = {'Q',MOT_CHARS[mot],'D','_',EOS};
    if(direction[mot] == DIR_POS)
        qresp[3] = '+';
    else
        qresp[3] = '-';
    response_telegram(qresp);
}

// enable a motor's stepping interrupt
void steps_on(int mot){
    
    switch(mot){
        case 0:
            TIMSK1 |= (1<<TOIE1);
            break;
        case 1:
            TIMSK3 |= (1<<TOIE3);
            break;
        case 2:
            TIMSK4 |= (1<<TOIE4);
            break;
        case 3:
            TIMSK5 |= (1<<TOIE5);
            break;
    }
    
}

// disable a motor's stepping interrupt
void steps_off(int mot){
    
    switch(mot){
        case 0:
            TIMSK1 &= ~(1<<TOIE1);
            break;
        case 1:
            TIMSK3 &= ~(1<<TOIE3);
            break;
        case 2:
            TIMSK4 &= ~(1<<TOIE4);
            break;
        case 3:
            TIMSK5 &= ~(1<<TOIE5);
            break;
    }
    
}

// change reset value for acceleration and deceleration
void update_reset(int mot){
    
    // time since last update [s] is 1/(last step rate [Hz])
    float dt = 1.0 / ((float) step_rate[mot]);
    
    // Case 1: emergency stop
    if(e_stop[mot]){
        ramp_mult[mot] = emerg_mult[mot];
    }
    
    // Case 2: overstepped switch move
    else if(switch_move[mot] && (switch_count[mot] == npos[mot])){
        if(step_rate[mot] > min_rate[mot]){ // still decelerating
            ramp_mult[mot] = -1;
        }
        else if(!returning[mot]){ // reached turning point - need to hold before reversing direction
            steps_off(mot);
            switch_hold[mot] = true;
            hold_time[mot] = millis();  // set hold start time
        }
        else{   // hold at turning point completed - return to target position at minimum step rate
            step_rate[mot] = min_rate[mot];   // ensure motor is stepping at minimum rate
            ramp_mult[mot] = 0;
        }
    }
    
    /* 
     * Case 3: acceleration if:
     *  1) more than 1/2 of planned steps left
     *              AND
     *  2) the step rate is less than the specified maximum
     */
    else if(remaining_steps[mot] >= start_steps[mot]/2 && step_rate[mot] < max_rate[mot]){
        ramp_mult[mot] = 1;
        acc_steps[mot] = start_steps[mot] - remaining_steps[mot];   // update number of acceleration steps
    }
    
    /* 
     * Case 4: deceleration if:
     *  1) fewer than number of steps to accelerate
     */
    else if(remaining_steps[mot] <= acc_steps[mot]){
        ramp_mult[mot] = -1;
    }
    
    // Case 5: constant step rate
    else{
        ramp_mult[mot] = 0;
    }
    
    // [new rate] = [last rate] + [ramp multiplier] * [ramp] * [time since last update in s]
    float temp_rate = step_rate[mot] + ramp_mult[mot] * ramp[mot] * dt;
    
    // perform limit checks
    if(temp_rate < min_rate[mot]){
        temp_rate = min_rate[mot];
    }
    else if(temp_rate > max_rate[mot]){
        temp_rate = max_rate[mot];
    }
    
    // store new step rate
    step_rate[mot] = temp_rate;
    tcnt[mot] = sixteen_bit_reset(step_rate[mot]);
    
}
