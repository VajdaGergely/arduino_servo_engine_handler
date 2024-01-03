//libraries
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <IRremote.h>


//technical parameters
#define STEP 5
#define DELAY_SEC 50
#define SERVO_COUNT 8
#define VALVE_COUNT 6

//values of settings
#define OFF 0
#define ON 1
#define CW 1
#define CC 0

#define NORMAL_WAY 1
#define REVERSE_WAY -1

#define SETTINGS 0
#define MOVE 1

//buttons of Remote Controller  **************************************
#define BTN_TEST  0x322C      //play
#define BTN_TEST2  0x1322C

#define BTN_FUNC  0x3254      //setup
#define BTN_FUNC2  0x13254

#define BTN_OK    0x325C      //ok
#define BTN_OK2    0x1325C

#define BTN_CMD   0x32C9      //select
#define BTN_CMD2   0x132C9

#define BTN_BACK  0x325E      //source
#define BTN_BACK2  0x1325E

#define BTN_RESET 0x320C      //power button
#define BTN_RESET2 0x1320C

#define BTN_NEXT  0x325B      //rigth arrow
#define BTN_NEXT2  0x1325B
#define BTN_PREV  0x325A      //left arrow
#define BTN_PREV2  0x1325A

#define BTN_UP    0x3258      //up arrow
#define BTN_UP2    0x13258
#define BTN_DOWN  0x3259      //down arrow
#define BTN_DOWN2  0x13259

#define BTN_ZERO  0x3200
#define BTN_ZERO2  0x13200

#define BTN_ONE   0x3201
#define BTN_ONE2   0x13201

#define BTN_TWO   0x3202
#define BTN_TWO2   0x13202

#define BTN_THREE 0x3203
#define BTN_THREE2 0x13203

#define BTN_FOUR  0x3204
#define BTN_FOUR2  0x13204

#define BTN_FIVE  0x3205
#define BTN_FIVE2  0x13205

#define BTN_SIX   0x3206
#define BTN_SIX2   0x13206

#define BTN_SEVEN 0x3207
#define BTN_SEVEN2 0x13207

#define BTN_EIGHT 0x3208
#define BTN_EIGHT2 0x13208

#define BTN_NINE  0x3209
#define BTN_NINE2  0x13209

#define BTN_REC   0x32F3      //rec button
#define BTN_REC2   0x132F3


//servo groups  ****************************************
#define FIRST_LVL_SERVOS  0x000000000000000000111111
#define SECOND_LVL_SERVOS 0x000000000000111111000000
#define THIRD_LVL_SERVOS  0x000000111111000000000000
#define FORTH_LVL_SERVOS  0x111111000000000000000000

#define FIRST_LVL_FRONT_SERVOS  0x000000000000000000100001
#define FIRST_LVL_MIDDLE_SERVOS 0x000000000000000000010010
#define FIRST_LVL_BACK_SERVOS   0x000000000000000000001100
#define FIRST_LVL_LEFT_SERVOS   0x000000000000000000111000
#define FIRST_LVL_RIGHT_SERVOS  0x000000000000000000000111

#define SECOND_LVL_FRONT_SERVOS  0x000000000000100001000000
#define SECOND_LVL_MIDDLE_SERVOS 0x000000000000010010000000
#define SECOND_LVL_BACK_SERVOS   0x000000000000001100000000
#define SECOND_LVL_LEFT_SERVOS   0x000000000000111000000000
#define SECOND_LVL_RIGHT_SERVOS  0x000000000000000111000000

#define THIRD_LVL_FRONT_SERVOS  0x000000100001000000000000
#define THIRD_LVL_MIDDLE_SERVOS 0x000000010010000000000000
#define THIRD_LVL_BACK_SERVOS   0x000000001100000000000000
#define THIRD_LVL_LEFT_SERVOS   0x000000111000000000000000
#define THIRD_LVL_RIGHT_SERVOS  0x000000000111000000000000

#define FORTH_LVL_FRONT_SERVOS  0x100001000000000000000000
#define FORTH_LVL_MIDDLE_SERVOS 0x010010000000000000000000
#define FORTH_LVL_BACK_SERVOS   0x001100000000000000000000
#define FORTH_LVL_LEFT_SERVOS   0x111000000000000000000000
#define FORTH_LVL_RIGHT_SERVOS  0x000111000000000000000000


//Commands  *************************************

//servo commands
#define CMD_SERVO_1_ON		1
#define CMD_SERVO_1_OFF		2
#define CMD_SERVO_1_CC		3
#define CMD_SERVO_1_CW		4
#define CMD_SERVO_2_ON		5
#define CMD_SERVO_2_OFF		6
#define CMD_SERVO_2_CC		7
#define CMD_SERVO_2_CW		8
#define CMD_SERVO_3_ON		9
#define CMD_SERVO_3_OFF		10
#define CMD_SERVO_3_CC		11
#define CMD_SERVO_3_CW		12
#define CMD_SERVO_4_ON		13
#define CMD_SERVO_4_OFF		14
#define CMD_SERVO_4_CC		15
#define CMD_SERVO_4_CW		16
#define CMD_SERVO_5_ON		17
#define CMD_SERVO_5_OFF		18
#define CMD_SERVO_5_CC		19
#define CMD_SERVO_5_CW		20
#define CMD_SERVO_6_ON		21
#define CMD_SERVO_6_OFF		22
#define CMD_SERVO_6_CC		23
#define CMD_SERVO_6_CW		24
#define CMD_SERVO_7_ON		25
#define CMD_SERVO_7_OFF		26
#define CMD_SERVO_7_CC		27
#define CMD_SERVO_7_CW		28
#define CMD_SERVO_8_ON		29
#define CMD_SERVO_8_OFF		30
#define CMD_SERVO_8_CC		31
#define CMD_SERVO_8_CW		32
#define CMD_SERVO_9_ON		33
#define CMD_SERVO_9_OFF		34
#define CMD_SERVO_9_CC		35
#define CMD_SERVO_9_CW		36
#define CMD_SERVO_10_ON		37
#define CMD_SERVO_10_OFF		38
#define CMD_SERVO_10_CC		39
#define CMD_SERVO_10_CW		40
#define CMD_SERVO_11_ON		41
#define CMD_SERVO_11_OFF		42
#define CMD_SERVO_11_CC		43
#define CMD_SERVO_11_CW		44
#define CMD_SERVO_12_ON		45
#define CMD_SERVO_12_OFF		46
#define CMD_SERVO_12_CC		47
#define CMD_SERVO_12_CW		48
#define CMD_SERVO_13_ON		49
#define CMD_SERVO_13_OFF		50
#define CMD_SERVO_13_CC		51
#define CMD_SERVO_13_CW		52
#define CMD_SERVO_14_ON		53
#define CMD_SERVO_14_OFF		54
#define CMD_SERVO_14_CC		55
#define CMD_SERVO_14_CW		56
#define CMD_SERVO_15_ON		57
#define CMD_SERVO_15_OFF		58
#define CMD_SERVO_15_CC		59
#define CMD_SERVO_15_CW		60
#define CMD_SERVO_16_ON		61
#define CMD_SERVO_16_OFF		62
#define CMD_SERVO_16_CC		63
#define CMD_SERVO_16_CW		64
#define CMD_SERVO_17_ON		65
#define CMD_SERVO_17_OFF		66
#define CMD_SERVO_17_CC		67
#define CMD_SERVO_17_CW		68
#define CMD_SERVO_18_ON		69
#define CMD_SERVO_18_OFF		70
#define CMD_SERVO_18_CC		71
#define CMD_SERVO_18_CW		72
#define CMD_SERVO_19_ON		73
#define CMD_SERVO_19_OFF		74
#define CMD_SERVO_19_CC		75
#define CMD_SERVO_19_CW		76
#define CMD_SERVO_20_ON		77
#define CMD_SERVO_20_OFF		78
#define CMD_SERVO_20_CC		79
#define CMD_SERVO_20_CW		80
#define CMD_SERVO_21_ON		81
#define CMD_SERVO_21_OFF		82
#define CMD_SERVO_21_CC		83
#define CMD_SERVO_21_CW		84
#define CMD_SERVO_22_ON		85
#define CMD_SERVO_22_OFF		86
#define CMD_SERVO_22_CC		87
#define CMD_SERVO_22_CW		88
#define CMD_SERVO_23_ON		89
#define CMD_SERVO_23_OFF		90
#define CMD_SERVO_23_CC		91
#define CMD_SERVO_23_CW		92
#define CMD_SERVO_24_ON		93
#define CMD_SERVO_24_OFF		94
#define CMD_SERVO_24_CC		95
#define CMD_SERVO_24_CW		96


//valve control commands
#define CMD_VALVE_1_ON    500
#define CMD_VALVE_1_OFF    501
#define CMD_VALVE_2_ON    502
#define CMD_VALVE_2_OFF    503
#define CMD_VALVE_3_ON    504
#define CMD_VALVE_3_OFF    505
#define CMD_VALVE_4_ON    506
#define CMD_VALVE_4_OFF    507
#define CMD_VALVE_5_ON    508
#define CMD_VALVE_5_OFF    509
#define CMD_VALVE_6_ON    510
#define CMD_VALVE_6_OFF    511
#define CMD_VALVE_7_ON    512
#define CMD_VALVE_7_OFF    513


//servo group commands
#define CMD_FIRST_LVL_SERVOS    600
#define CMD_SECOND_LVL_SERVOS    601
#define CMD_THIRD_LVL_SERVOS    602
#define CMD_FORTH_LVL_SERVOS    603
#define CMD_FIRST_LVL_FRONT_SERVOS    604
#define CMD_FIRST_LVL_MIDDLE_SERVOS    605
#define CMD_FIRST_LVL_BACK_SERVOS    606
#define CMD_FIRST_LVL_LEFT_SERVOS    607
#define CMD_FIRST_LVL_RIGHT_SERVOS    608
#define CMD_SECOND_LVL_FRONT_SERVOS    609
#define CMD_SECOND_LVL_MIDDLE_SERVOS    610
#define CMD_SECOND_LVL_BACK_SERVOS    611
#define CMD_SECOND_LVL_LEFT_SERVOS    612
#define CMD_SECOND_LVL_RIGHT_SERVOS    613
#define CMD_THIRD_LVL_FRONT_SERVOS    614
#define CMD_THIRD_LVL_MIDDLE_SERVOS    615
#define CMD_THIRD_LVL_BACK_SERVOS    616
#define CMD_THIRD_LVL_LEFT_SERVOS    617
#define CMD_THIRD_LVL_RIGHT_SERVOS    618
#define CMD_FORTH_LVL_FRONT_SERVOS    619
#define CMD_FORTH_LVL_MIDDLE_SERVOS    620
#define CMD_FORTH_LVL_BACK_SERVOS    621
#define CMD_FORTH_LVL_LEFT_SERVOS    622
#define CMD_FORTH_LVL_RIGHT_SERVOS    623


//Ezekbõl késõbb annyi db kell, ahány szervo van, mert egyedi értékek lesznek náluk.
//#define SERVOMIN  95
//#define SERVOMAX  460

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

//Buffer
struct CmdBuffer
{
  unsigned int mode: 1; //mode flag
  unsigned int menu_state1: 1; //menu state flag
  unsigned int c1 : 5;  //1st command, value intervallum: [0 - 31]
  unsigned int c2 : 5;  //2nd command, value intervallum: [0 - 31]
  unsigned int c3 : 5;  //2nd command, value intervallum: [0 - 31]
  unsigned int c4 : 5;  //2nd command, value intervallum: [0 - 31]
};

/*
c1, c2, c3, c4 potential values

0..9  ->  0..9
31  ->  NULL
*/

//servo engines
//Servo servos[SERVO_COUNT];

struct CmdBuffer cmd_buffer;

//settings (32x bitfield per each) for servos settings (size: 4 byte per each)
unsigned long states;   //bitfield of 32 servos' state
unsigned long directions; //bitfield of 32 servos' direction

struct Servo
{
  unsigned int min;
  unsigned int max;
  unsigned int pos;
  
};

//real time servo positions (24 db 0-180ig tartó értéket rögzítünk benne)
Servo servos[SERVO_COUNT];

//valves
unsigned char valves;



//IR receiver
decode_results results;
decode_results temp_results;

IRrecv irrecv(2);

void setup()
{
  Serial.begin(9600);
  irrecv.enableIRIn();
  
  //init servos
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  
  //90 fokra állítani õket???
  for(char i = 0; i < SERVO_COUNT; i++)
  {
    servos[i].min = 95;   //ezeket késõbb konstansként vesszük majd fel, szervónként külön külön...
    servos[i].max = 460;  //ezeket késõbb konstansként vesszük majd fel, szervónként külön külön...
    //valami ehhez hasonló hasznos lenne!!!
    servos[i].pos = 280;              //be kell majd állítani az alap pozíciókat
    pwm.setPWM(i, 0, servos[i].pos);
  }
  
  //init valves (szelepek vezérlése)
  valves = 0;
  for(char i = 0; i < 7; i++)
  {
    pinMode(i + 3, OUTPUT);
  }
  
  //init settings
  reset_settings();
  
  //init delay
  delay(500);
}

void loop()
{
  if (irrecv.decode(&results))
  {
    Serial.println("-------------------------");
    Serial.println(results.value, HEX);
    
    if(results.value == BTN_TEST || results.value == BTN_TEST2)
    {
      Serial.println("TEST");
      settings_dump();  //print settings' values to serial monitor
    }
    else if(results.value == BTN_FUNC || results.value == BTN_FUNC2)
    {
      Serial.println("FUNC");
      change_mode();  //switch SETTINGS mode and MOVE mode
    }
    else if(cmd_buffer.mode) //MOVE MODE
    {
      if(results.value == BTN_UP || results.value == BTN_UP2) //moving normal way
      {
        Serial.println("UP");
        move_servos_to_rel_pos(NORMAL_WAY);
      }
      else if(results.value == BTN_DOWN || results.value == BTN_DOWN2) //moving reverse way
      {
        Serial.println("DOWN");
        move_servos_to_rel_pos(REVERSE_WAY);
      }
      else if(results.value == BTN_REC || results.value == BTN_REC2)
      {
        Serial.println("REC");
        print_servo_positions();
      }
    }
    else if(!cmd_buffer.mode) //SETTINGS MODE
    {
      if(!cmd_buffer.menu_state1)  //section 0
      {
        switch(results.value)
        {
          case BTN_RESET:
          case BTN_RESET2:
            Serial.println("RESET");
            reset_settings(); //reset all settings
            break;
          case BTN_CMD:
          case BTN_CMD2:
            Serial.println("COMMAND");
            cmd_buffer.menu_state1 = 1;
            break;
        }
      }
      else              //section 1
      {
        if(results.value == BTN_OK || results.value == BTN_OK2)
        {
          Serial.println("OK");
          numbered_command();
          cmd_buffer.menu_state1 = 0;
        }
        else if(results.value == BTN_BACK || results.value == BTN_BACK2)
        {
          Serial.println("BACK");
          clear_buffer();
          cmd_buffer.menu_state1 = 0;
        }
        
        switch(results.value)
        {
          case BTN_ZERO:
          case BTN_ZERO2:
            Serial.println("0");
            digit_to_buffer(0);
            break;
          case BTN_ONE:
          case BTN_ONE2:
            Serial.println("1");
            digit_to_buffer(1);
            break;
          case BTN_TWO:
          case BTN_TWO2:
            Serial.println("2");
            digit_to_buffer(2);
            break;
          case BTN_THREE:
          case BTN_THREE2:
            Serial.println("3");
            digit_to_buffer(3);
            break;
          case BTN_FOUR:
          case BTN_FOUR2:
            Serial.println("4");
            digit_to_buffer(4);
            break;
          case BTN_FIVE:
          case BTN_FIVE2:
            Serial.println("5");
            digit_to_buffer(5);
            break;
          case BTN_SIX:
          case BTN_SIX2:
            Serial.println("6");
            digit_to_buffer(6);
            break;
          case BTN_SEVEN:
          case BTN_SEVEN2:
            Serial.println("7");
            digit_to_buffer(7);
            break;
          case BTN_EIGHT:
          case BTN_EIGHT2:
            Serial.println("8");
            digit_to_buffer(8);
            break;
          case BTN_NINE:
          case BTN_NINE2:
            Serial.println("9");
            digit_to_buffer(9);
            break;
        }
      }
    }
    irrecv.resume();
  }
}

//*********************       SMALL UTILITY FUNCTIONS       *****************************************

void settings_dump()
{
  //print
  Serial.println("------------------------------------");
  Serial.print("mode: ");
  if(!cmd_buffer.mode)
  {
    Serial.println("SETTINGS");
  }
  else
  {
    Serial.println("MOVE");
  }
  Serial.print("buffer: [");
  Serial.print(cmd_buffer.c1);
  Serial.print("] [");
  Serial.print(cmd_buffer.c2);
  Serial.print("] [");
  Serial.print(cmd_buffer.c3);
  Serial.print("] [");
  Serial.print(cmd_buffer.c4);
  Serial.println("]");
  
  unsigned long temp_states = states;
  unsigned long temp_directions = directions;
  
  for(unsigned char i = 0; i < SERVO_COUNT; i++)
  {
    
    Serial.print("servo ");
    Serial.print(i + 1);
    Serial.print(". [");
    if(temp_states & 0x00000001)
    {
      Serial.print("ON");
    }
    else
    {
      Serial.print("OFF");
    }
    temp_states >>= 1;
    Serial.print("] [");
    if(temp_directions & 0x00000001)
    {
      Serial.print("CW");
    }
    else
    {
      Serial.print("CC");
    }
    temp_directions >>= 1;
    Serial.println("]");
  }
  Serial.println("------------------------------------");
}

void change_mode()
{
  if(cmd_buffer.mode)
  {
    cmd_buffer.mode = false;
  }
  else
  {
    cmd_buffer.mode = true;
  }
}

void print_servo_positions()
{
  Serial.println("---------------------------");
  for(char i = 0; i < SERVO_COUNT; i++)
  {
    Serial.print(i + 1);
    Serial.print(". servo position: ");
    Serial.println(servos[i].pos);
    Serial.println();
  }
  Serial.println("---------------------------");
}

void reset_settings()
{
  cmd_buffer.mode = 0;
  cmd_buffer.menu_state1 = 0;
  clear_buffer();
  
  states = 0;
  directions = 0;
  valves = 0;
}

void clear_buffer()
{
  cmd_buffer.c1 = 31;
  cmd_buffer.c2 = 31;
  cmd_buffer.c3 = 31;
  cmd_buffer.c4 = 31;
}

void digit_to_buffer(unsigned char digit)
{
  if(cmd_buffer.c1 == 31)
  {
    cmd_buffer.c1 = digit;
  }
  else if(cmd_buffer.c2 == 31)
  {
    cmd_buffer.c2 = digit;
  }
  else if(cmd_buffer.c3 == 31)
  {
    cmd_buffer.c3 = digit;
  }
  else if(cmd_buffer.c4 == 31)
  {
    cmd_buffer.c4 = digit;
  }
}


//*********************       SETTINGS FOCUSED FUNCTIONS       *****************************************

void numbered_command()
{
  //csekkoljuk, hogy megfelelõ e commandnak a sorszáma
  //megfelelõ sorrendben nyomkodtuk e a gombokat
  //a végén ha végrehajtottuk a parancsot, akkor a cmd_buffer tartalmát nullázni kell
  //4 jegyû legyen a cmd_buffer
  
  //create command number from digits
  unsigned int cmd_num = 0;
  if(cmd_buffer.c1 != 31)
  {
    cmd_num += cmd_buffer.c1 * 1000;
    if(cmd_buffer.c2 != 31)
    {
      cmd_num += cmd_buffer.c2 * 100;
      if(cmd_buffer.c3 != 31)
      {
        cmd_num += cmd_buffer.c3 * 10;
        if(cmd_buffer.c4 != 31)
        {
          cmd_num += cmd_buffer.c4;
        }
      }
    }
  }
  
  //search command by command number
  Serial.println(cmd_num);
  switch(cmd_num)
  {
    //servo controlling
    case CMD_SERVO_1_ON: modify_bitfield(&states, 1, ON); break;
    case CMD_SERVO_1_OFF: modify_bitfield(&states, 1, OFF); break;
    case CMD_SERVO_1_CC: modify_bitfield(&directions, 1, CC); break;
    case CMD_SERVO_1_CW: modify_bitfield(&directions, 1, CW); break;
    case CMD_SERVO_2_ON: modify_bitfield(&states, 2, ON); break;
    case CMD_SERVO_2_OFF: modify_bitfield(&states, 2, OFF); break;
    case CMD_SERVO_2_CC: modify_bitfield(&directions, 2, CC); break;
    case CMD_SERVO_2_CW: modify_bitfield(&directions, 2, CW); break;
    case CMD_SERVO_3_ON: modify_bitfield(&states, 3, ON); break;
    case CMD_SERVO_3_OFF: modify_bitfield(&states, 3, OFF); break;
    case CMD_SERVO_3_CC: modify_bitfield(&directions, 3, CC); break;
    case CMD_SERVO_3_CW: modify_bitfield(&directions, 3, CW); break;
    case CMD_SERVO_4_ON: modify_bitfield(&states, 4, ON); break;
    case CMD_SERVO_4_OFF: modify_bitfield(&states, 4, OFF); break;
    case CMD_SERVO_4_CC: modify_bitfield(&directions, 4, CC); break;
    case CMD_SERVO_4_CW: modify_bitfield(&directions, 4, CW); break;
    case CMD_SERVO_5_ON: modify_bitfield(&states, 5, ON); break;
    case CMD_SERVO_5_OFF: modify_bitfield(&states, 5, OFF); break;
    case CMD_SERVO_5_CC: modify_bitfield(&directions, 5, CC); break;
    case CMD_SERVO_5_CW: modify_bitfield(&directions, 5, CW); break;
    case CMD_SERVO_6_ON: modify_bitfield(&states, 6, ON); break;
    case CMD_SERVO_6_OFF: modify_bitfield(&states, 6, OFF); break;
    case CMD_SERVO_6_CC: modify_bitfield(&directions, 6, CC); break;
    case CMD_SERVO_6_CW: modify_bitfield(&directions, 6, CW); break;
    case CMD_SERVO_7_ON: modify_bitfield(&states, 7, ON); break;
    case CMD_SERVO_7_OFF: modify_bitfield(&states, 7, OFF); break;
    case CMD_SERVO_7_CC: modify_bitfield(&directions, 7, CC); break;
    case CMD_SERVO_7_CW: modify_bitfield(&directions, 7, CW); break;
    case CMD_SERVO_8_ON: modify_bitfield(&states, 8, ON); break;
    case CMD_SERVO_8_OFF: modify_bitfield(&states, 8, OFF); break;
    case CMD_SERVO_8_CC: modify_bitfield(&directions, 8, CC); break;
    case CMD_SERVO_8_CW: modify_bitfield(&directions, 8, CW); break;
    case CMD_SERVO_9_ON: modify_bitfield(&states, 9, ON); break;
    case CMD_SERVO_9_OFF: modify_bitfield(&states, 9, OFF); break;
    case CMD_SERVO_9_CC: modify_bitfield(&directions, 9, CC); break;
    case CMD_SERVO_9_CW: modify_bitfield(&directions, 9, CW); break;
    case CMD_SERVO_10_ON: modify_bitfield(&states, 10, ON); break;
    case CMD_SERVO_10_OFF: modify_bitfield(&states, 10, OFF); break;
    case CMD_SERVO_10_CC: modify_bitfield(&directions, 10, CC); break;
    case CMD_SERVO_10_CW: modify_bitfield(&directions, 10, CW); break;
    case CMD_SERVO_11_ON: modify_bitfield(&states, 11, ON); break;
    case CMD_SERVO_11_OFF: modify_bitfield(&states, 11, OFF); break;
    case CMD_SERVO_11_CC: modify_bitfield(&directions, 11, CC); break;
    case CMD_SERVO_11_CW: modify_bitfield(&directions, 11, CW); break;
    case CMD_SERVO_12_ON: modify_bitfield(&states, 12, ON); break;
    case CMD_SERVO_12_OFF: modify_bitfield(&states, 12, OFF); break;
    case CMD_SERVO_12_CC: modify_bitfield(&directions, 12, CC); break;
    case CMD_SERVO_12_CW: modify_bitfield(&directions, 12, CW); break;
    case CMD_SERVO_13_ON: modify_bitfield(&states, 13, ON); break;
    case CMD_SERVO_13_OFF: modify_bitfield(&states, 13, OFF); break;
    case CMD_SERVO_13_CC: modify_bitfield(&directions, 13, CC); break;
    case CMD_SERVO_13_CW: modify_bitfield(&directions, 13, CW); break;
    case CMD_SERVO_14_ON: modify_bitfield(&states, 14, ON); break;
    case CMD_SERVO_14_OFF: modify_bitfield(&states, 14, OFF); break;
    case CMD_SERVO_14_CC: modify_bitfield(&directions, 14, CC); break;
    case CMD_SERVO_14_CW: modify_bitfield(&directions, 14, CW); break;
    case CMD_SERVO_15_ON: modify_bitfield(&states, 15, ON); break;
    case CMD_SERVO_15_OFF: modify_bitfield(&states, 15, OFF); break;
    case CMD_SERVO_15_CC: modify_bitfield(&directions, 15, CC); break;
    case CMD_SERVO_15_CW: modify_bitfield(&directions, 15, CW); break;
    case CMD_SERVO_16_ON: modify_bitfield(&states, 16, ON); break;
    case CMD_SERVO_16_OFF: modify_bitfield(&states, 16, OFF); break;
    case CMD_SERVO_16_CC: modify_bitfield(&directions, 16, CC); break;
    case CMD_SERVO_16_CW: modify_bitfield(&directions, 16, CW); break;
    case CMD_SERVO_17_ON: modify_bitfield(&states, 17, ON); break;
    case CMD_SERVO_17_OFF: modify_bitfield(&states, 17, OFF); break;
    case CMD_SERVO_17_CC: modify_bitfield(&directions, 17, CC); break;
    case CMD_SERVO_17_CW: modify_bitfield(&directions, 17, CW); break;
    case CMD_SERVO_18_ON: modify_bitfield(&states, 18, ON); break;
    case CMD_SERVO_18_OFF: modify_bitfield(&states, 18, OFF); break;
    case CMD_SERVO_18_CC: modify_bitfield(&directions, 18, CC); break;
    case CMD_SERVO_18_CW: modify_bitfield(&directions, 18, CW); break;
    case CMD_SERVO_19_ON: modify_bitfield(&states, 19, ON); break;
    case CMD_SERVO_19_OFF: modify_bitfield(&states, 19, OFF); break;
    case CMD_SERVO_19_CC: modify_bitfield(&directions, 19, CC); break;
    case CMD_SERVO_19_CW: modify_bitfield(&directions, 19, CW); break;
    case CMD_SERVO_20_ON: modify_bitfield(&states, 20, ON); break;
    case CMD_SERVO_20_OFF: modify_bitfield(&states, 20, OFF); break;
    case CMD_SERVO_20_CC: modify_bitfield(&directions, 20, CC); break;
    case CMD_SERVO_20_CW: modify_bitfield(&directions, 20, CW); break;
    case CMD_SERVO_21_ON: modify_bitfield(&states, 21, ON); break;
    case CMD_SERVO_21_OFF: modify_bitfield(&states, 21, OFF); break;
    case CMD_SERVO_21_CC: modify_bitfield(&directions, 21, CC); break;
    case CMD_SERVO_21_CW: modify_bitfield(&directions, 21, CW); break;
    case CMD_SERVO_22_ON: modify_bitfield(&states, 22, ON); break;
    case CMD_SERVO_22_OFF: modify_bitfield(&states, 22, OFF); break;
    case CMD_SERVO_22_CC: modify_bitfield(&directions, 22, CC); break;
    case CMD_SERVO_22_CW: modify_bitfield(&directions, 22, CW); break;
    case CMD_SERVO_23_ON: modify_bitfield(&states, 23, ON); break;
    case CMD_SERVO_23_OFF: modify_bitfield(&states, 23, OFF); break;
    case CMD_SERVO_23_CC: modify_bitfield(&directions, 23, CC); break;
    case CMD_SERVO_23_CW: modify_bitfield(&directions, 23, CW); break;
    case CMD_SERVO_24_ON: modify_bitfield(&states, 24, ON); break;
    case CMD_SERVO_24_OFF: modify_bitfield(&states, 24, OFF); break;
    case CMD_SERVO_24_CC: modify_bitfield(&directions, 24, CC); break;
    case CMD_SERVO_24_CW: modify_bitfield(&directions, 24, CW); break;
    
    //valve controlling
    case CMD_VALVE_1_ON : turn_on_valve(1); break;
    case CMD_VALVE_1_OFF : turn_off_valve(1); break;
    case CMD_VALVE_2_ON : turn_on_valve(2); break;
    case CMD_VALVE_2_OFF : turn_off_valve(2); break;
    case CMD_VALVE_3_ON : turn_on_valve(3); break;
    case CMD_VALVE_3_OFF : turn_off_valve(3); break;
    case CMD_VALVE_4_ON : turn_on_valve(4); break;
    case CMD_VALVE_4_OFF : turn_off_valve(4); break;
    case CMD_VALVE_5_ON : turn_on_valve(5); break;
    case CMD_VALVE_5_OFF : turn_off_valve(5); break;
    case CMD_VALVE_6_ON : turn_on_valve(6); break;
    case CMD_VALVE_6_OFF : turn_off_valve(6); break;
    case CMD_VALVE_7_ON : turn_on_valve(7); break;
    case CMD_VALVE_7_OFF : turn_off_valve(7); break;
    
    //servo group commands
    case CMD_FIRST_LVL_SERVOS:    states = FIRST_LVL_SERVOS; break;
    case CMD_SECOND_LVL_SERVOS:    states = SECOND_LVL_SERVOS; break;
    case CMD_THIRD_LVL_SERVOS:    states = THIRD_LVL_SERVOS; break;
    case CMD_FORTH_LVL_SERVOS:    states = FORTH_LVL_SERVOS; break;
    case CMD_FIRST_LVL_FRONT_SERVOS:    states = FIRST_LVL_FRONT_SERVOS; break;
    case CMD_FIRST_LVL_MIDDLE_SERVOS:    states = FIRST_LVL_MIDDLE_SERVOS; break;
    case CMD_FIRST_LVL_BACK_SERVOS:    states = FIRST_LVL_BACK_SERVOS; break;
    case CMD_FIRST_LVL_LEFT_SERVOS:    states = FIRST_LVL_LEFT_SERVOS; break;
    case CMD_FIRST_LVL_RIGHT_SERVOS:    states = FIRST_LVL_RIGHT_SERVOS; break;
    case CMD_SECOND_LVL_FRONT_SERVOS:    states = SECOND_LVL_FRONT_SERVOS; break;
    case CMD_SECOND_LVL_MIDDLE_SERVOS:    states = SECOND_LVL_MIDDLE_SERVOS; break;
    case CMD_SECOND_LVL_BACK_SERVOS:    states = SECOND_LVL_BACK_SERVOS; break;
    case CMD_SECOND_LVL_LEFT_SERVOS:    states = SECOND_LVL_LEFT_SERVOS; break;
    case CMD_SECOND_LVL_RIGHT_SERVOS:    states = SECOND_LVL_RIGHT_SERVOS; break;
    case CMD_THIRD_LVL_FRONT_SERVOS:    states = THIRD_LVL_FRONT_SERVOS; break;
    case CMD_THIRD_LVL_MIDDLE_SERVOS:    states = THIRD_LVL_MIDDLE_SERVOS; break;
    case CMD_THIRD_LVL_BACK_SERVOS:    states = THIRD_LVL_BACK_SERVOS; break;
    case CMD_THIRD_LVL_LEFT_SERVOS:    states = THIRD_LVL_LEFT_SERVOS; break;
    case CMD_THIRD_LVL_RIGHT_SERVOS:    states = THIRD_LVL_RIGHT_SERVOS; break;
    case CMD_FORTH_LVL_FRONT_SERVOS:    states = FORTH_LVL_FRONT_SERVOS; break;
    case CMD_FORTH_LVL_MIDDLE_SERVOS:    states = FORTH_LVL_MIDDLE_SERVOS; break;
    case CMD_FORTH_LVL_BACK_SERVOS:    states = FORTH_LVL_BACK_SERVOS; break;
    case CMD_FORTH_LVL_LEFT_SERVOS:    states = FORTH_LVL_LEFT_SERVOS; break;
    case CMD_FORTH_LVL_RIGHT_SERVOS:    states = FORTH_LVL_RIGHT_SERVOS; break;
  }
  clear_buffer();
}

//ezeket eltároljuk majd valami változóban is???
//szelep vezérlés, bekapcs
void turn_on_valve(char num)
{
  digitalWrite(num, HIGH);
  delay(DELAY_SEC);
}

//szelep vezérlés, kikapcs
void turn_off_valve(char num)
{
  digitalWrite(num, LOW);
  delay(DELAY_SEC);
}

//change settings (for using bitfield variables)
void modify_bitfield(unsigned long *bitfield, char n, bool value)
{
  unsigned long hi_bits = *bitfield; //all the bits before and inculding the one that we will modify
  unsigned long low_bits = *bitfield; //the remaining bits at the end
  
  hi_bits >>= n + 1;
  hi_bits <<= 1;
  if(value)
  {
    hi_bits++;
  }
  hi_bits <<= n;
  
  low_bits <<= ((31 - n) +1);
  low_bits >>= ((31 - n) +1);
  
  hi_bits += low_bits;
  
  *bitfield = hi_bits;
}


//*********************       CORE MOVEMENT FUNCTIONS       *****************************************

//move servos to relative positions (can handle MAX 32 servo)
void move_servos_to_rel_pos(char movement_direction)
{
  //use to iterate through states' and directions' bits
  unsigned long temp_states = states;
  unsigned long temp_directions = directions;

  //iterate through all servos
  for(char i = 0; i < SERVO_COUNT; i++)
  {
    //check state (ON / OFF)  (1 -> ON) (0 -> OFF)
    if(temp_states & 0x1) //is last bit one (true)?
    {
      //check direction
      if(!(temp_directions & 0x1)) //0 is CC
      {
        //read last position, increment it with STEP, and make movement
        //servos[i].write(servos[i].read() + (STEP * movement_direction));
        
        if((servos[i].pos + (STEP * movement_direction)) <= servos[i].max)
        {
          servos[i].pos += STEP * movement_direction;
          Serial.print("Position: ");
          Serial.println(servos[i].pos);
        }
        else
        {
          Serial.print("Position: ");
          Serial.print(servos[i].pos);
          Serial.print("    ");
          Serial.println("Reached the maximum!");
        }
      }
      else  //1 is CW
      {
        //read last position, decrement it with STEP, and make movement
        //servos[i].write(servos[i].read() - (STEP * movement_direction));
        if((servos[i].pos - (STEP * movement_direction)) >= servos[i].min)
        {
          servos[i].pos -= STEP * movement_direction;
          Serial.print("Position: ");
          Serial.println(servos[i].pos);
        }
        else
        {
          Serial.print("Position: ");
          Serial.print(servos[i].pos);
          Serial.print("    ");
          Serial.println("Reached the minimum!!!!");
        }
      }
      pwm.setPWM(i, 0, servos[i].pos);
    }
    temp_states >>= 1; //shift state by 1
    temp_directions >>= 1; //shift direction by 1
  }
  //delay for Arduino rules
  delay(DELAY_SEC);
}
