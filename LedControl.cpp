
#include "LedControl.h"

//the opcodes for the MAX7221 and MAX7219
#define OP_NOOP   0
#define OP_DIGIT0 1
#define OP_DIGIT1 2
#define OP_DIGIT2 3
#define OP_DIGIT3 4
#define OP_DIGIT4 5
#define OP_DIGIT5 6
#define OP_DIGIT6 7
#define OP_DIGIT7 8
#define OP_DECODEMODE  9
#define OP_INTENSITY   10
#define OP_SCANLIMIT   11
#define OP_SHUTDOWN    12
#define OP_DISPLAYTEST 15

LedControl::LedControl(int dataPin, int clkPin, int csPin, int numDevices) {
    SPI_MOSI=dataPin;
    SPI_CLK=clkPin;
    SPI_CS=csPin;
    if(numDevices<=0 || numDevices>8 )
        numDevices=8;
    maxDevices=numDevices;
    pinMode(SPI_MOSI,OUTPUT);
    pinMode(SPI_CLK,OUTPUT);
    pinMode(SPI_CS,OUTPUT);
    digitalWrite(SPI_CS,HIGH);
    SPI_MOSI=dataPin;
    for(int i=0;i<64;i++) 
        status[i]=0x00;
    for(int i=0;i<maxDevices;i++) {
        spiTransfer(i,OP_DISPLAYTEST,0);
        //scanlimit is set to max on startup
        setScanLimit(i,7);
        //decode is done in source
        spiTransfer(i,OP_DECODEMODE,0);
        clearDisplay(i);
        //we go into shutdown-mode on startup
        shutdown(i,true);
    }
}

int LedControl::getDeviceCount() {
    return maxDevices;
}

void LedControl::shutdown(int addr, bool b) {
    if(addr<0 || addr>=maxDevices)
        return;
    if(b)
        spiTransfer(addr, OP_SHUTDOWN,0);
    else
        spiTransfer(addr, OP_SHUTDOWN,1);
}

void LedControl::setScanLimit(int addr, int limit) {
    if(addr<0 || addr>=maxDevices)
        return;
    if(limit>=0 && limit<8)
        spiTransfer(addr, OP_SCANLIMIT,limit);
}

void LedControl::setIntensity(int addr, int intensity) {
    if(addr<0 || addr>=maxDevices)
        return;
    if(intensity>=0 && intensity<16)	
        spiTransfer(addr, OP_INTENSITY,intensity);
}

void LedControl::clearDisplay(int addr) {
    int offset;

    if(addr<0 || addr>=maxDevices)
        return;
    offset=addr*8;
    for(int i=0;i<8;i++) {
        status[offset+i]=0;
        spiTransfer(addr, i+1,status[offset+i]);
    }
}

void LedControl::setLed(int addr, int row, int column, boolean state) {
    int offset;
    byte val=0x00;

    if(addr<0 || addr>=maxDevices)
        return;
    if(row<0 || row>7 || column<0 || column>7)
        return;
    offset=addr*8;
    val=B10000000 >> column;
    if(state)
        status[offset+row]=status[offset+row]|val;
    else {
        val=~val;
        status[offset+row]=status[offset+row]&val;
    }
    spiTransfer(addr, row+1,status[offset+row]);
}

void LedControl::setRow(int addr, int row, byte value) {
    int offset;
    if(addr<0 || addr>=maxDevices)
        return;
    if(row<0 || row>7)
        return;
    offset=addr*8;
    status[offset+row]=value;
    spiTransfer(addr, row+1,status[offset+row]);
}

void LedControl::setColumn(int addr, int col, byte value) {
    byte val;

    if(addr<0 || addr>=maxDevices)
        return;
    if(col<0 || col>7) 
        return;
    for(int row=0;row<8;row++) {
        val=value >> (7-row);
        val=val & 0x01;
        setLed(addr,row,col,val);
    }
}

void LedControl::setDigit(int addr, int digit, byte value, boolean dp) {
    int offset;
    byte v;

    if(addr<0 || addr>=maxDevices)
        return;
    if(digit<0 || digit>7 || value>15)
        return;
    offset=addr*8;
    v=pgm_read_byte_near(charTable + value); 
    if(dp)
        v|=B10000000;
    status[offset+digit]=v;
    spiTransfer(addr, digit+1,v);
}

void LedControl::setChar(int addr, int digit, char value, boolean dp) {
    int offset;
    byte index,v;

    if(addr<0 || addr>=maxDevices)
        return;
    if(digit<0 || digit>7)
        return;
    offset=addr*8;
    index=(byte)value;
    if(index >127) {
        //no defined beyond index 127, so we use the space char
        index=32;
    }
    v=pgm_read_byte_near(charTable + index); 
    if(dp)
        v|=B10000000;
    status[offset+digit]=v;
    spiTransfer(addr, digit+1,v);
}

void LedControl::spiTransfer(int addr, volatile byte opcode, volatile byte data) {
    //Create an array with the data to shift out
    int offset=addr*2;
    int maxbytes=maxDevices*2;

    for(int i=0;i<maxbytes;i++)
        spidata[i]=(byte)0;
    //put our device data into the array
    spidata[offset+1]=opcode;
    spidata[offset]=data;
    //enable the line 
    digitalWrite(SPI_CS,LOW);
    //Now shift out the data 
    for(int i=maxbytes;i>0;i--)
        shiftOut(SPI_MOSI,SPI_CLK,MSBFIRST,spidata[i-1]);
    //latch the data onto the display
    digitalWrite(SPI_CS,HIGH);
}    

Algorithm:
#include "LedControl.h"
//DIN=MOSI= master - output slave-input
#define DIN   12
#define CS    11
#define CLK   10

#define LEFT_BUTTON_PIN   8
#define RIGHT_BUTTON_PIN  7

#define BIT_SET(a,b) ((a) |= (1ULL<<(b)))
#define BIT_CLEAR(a,b) ((a) &= ~(1ULL<<(b)))
#define BIT_CHECK(a,b) ((a) & (1ULL<<(b)))

#define INITIAL_X 4
#define INITIAL_Y 1

//Genereaza obiectul ce controleaza driverul matricii de leduri
LedControl ledControl=LedControl(DIN,CLK,CS);

byte initialMatrix[8]= 
            {0b11111111,
             0b11111111,
             0b11111111,
             0b00000000,
             0b00000000,
             0b00000000,
             0b00000000,
             0b00011100};

byte matrix[8]= 
            {0b11111111,
             0b11111111,
             0b11111111,
             0b00000000,
             0b00000000,
             0b00000000,
             0b00000000,
             0b00011100};

int ballX = INITIAL_X, ballY = INITIAL_Y;

void setup() 
{
//serial monitor debugging
  Serial.begin(9600);
//initializare matrice de leduri
  ledControl.shutdown(0,false);
  ledControl.setIntensity(0,8);
  ledControl.clearDisplay(0);
//initializare tastatura
  pinMode(LEFT_BUTTON_PIN , INPUT);
  pinMode(RIGHT_BUTTON_PIN, INPUT);
}
//starile mingii
enum StareMinge
{
  InitialPosition = 0,
  UrcaSt,
  UrcaDr,
  LiftingStraight,
  CoboaraSt,
  CoboaraDr,
  Coboara,
  GameOver
};
//variabila ce opereaza starile mingii
StareMinge stareMinge = InitialPosition;
long ballMovementDelta = millis();
long debouncingDelta = millis();
void loop()
{
//citire butoane singular(evitare citiri multiple) 
  if (millis() - debouncingDelta >= 120) 
  {
    debouncingDelta = millis();
    if(digitalRead(LEFT_BUTTON_PIN)== HIGH && matrix[7] < 0b11100000)
      matrix[7] <<= 1;
      
    if(digitalRead(RIGHT_BUTTON_PIN)== HIGH && matrix[7] > 0b00000111)
      matrix[7] >>= 1;
  
    if(stareMinge == GameOver && digitalRead(RIGHT_BUTTON_PIN)== HIGH && digitalRead(LEFT_BUTTON_PIN)== HIGH) // RESET GAME
      stareMinge = InitialPosition;
  }
  
  if (millis() - ballMovementDelta >= 400) 
  {
    ballMovementDelta = millis();
    //pt verificarea coordonatelor mingii
    int newX = -1, newY = -1;

    Serial.print("X:");
    Serial.print(ballX);
    Serial.print("Y:");
    Serial.println(ballY);
    
    switch(stareMinge)
    {
      case InitialPosition :
	  //la reset sau pornire se incarca matricea initiala
        Serial.println("InitialPosition");
        for(int i = 0; i <= 7; ++i)
          matrix[i] = initialMatrix[i];
		  //setezi pozitia initiala
        newX = INITIAL_X, newY = INITIAL_Y;
        stareMinge = LiftingStraight;
        break;
        
      case UrcaSt :

        Serial.println("UrcaSt");
      
        if(ballX - 1 < 0) // if it hits left wall
        {
          stareMinge = UrcaDr;
          break;
        }
        
        if(ballY + 1 > 7)
        {
          stareMinge = CoboaraSt;
          break;
        }
        
        if(BIT_CHECK(matrix[7 - ballY - 1], 7 - ballX + 1) != 0) //if there is a block
        {
          BIT_CLEAR(matrix[7 - ballY - 1], 7 - ballX + 1); //clear the block
          stareMinge = CoboaraSt;
          break;     
        }
        
        newX = ballX - 1; newY = ballY + 1;
        break;

      case UrcaDr :

        Serial.println("UrcaDr");
        
        if(ballX + 1 > 7)
        {
          stareMinge = UrcaSt;
          break;
        }
        
        if(ballY + 1 > 7)
        {
          stareMinge = CoboaraDr;
          break;
        }
        
        if(BIT_CHECK(matrix[7 - ballY - 1], 7 - ballX - 1) != 0) //if there is a block
        {
          BIT_CLEAR(matrix[7 - ballY - 1], 7 - ballX - 1); //clear the block
          stareMinge = CoboaraSt;
          break;
        }
        
        newX = ballX + 1; newY = ballY + 1;
        break;

     case CoboaraSt :
        Serial.println("CoboaraSt");
        
        if(ballX - 1 < 0)
        {
          stareMinge = CoboaraDr;
          break;
        }
        
        if(ballY - 1 < 0)
        {
          stareMinge = GameOver;
          break;
        }
        
        if(BIT_CHECK(matrix[7 - ballY + 1], 7 - ballX + 1) != 0) //if there is the slide
        {
          if(ballY == 1)//if there is the slide
          {
            if(BIT_CHECK(matrix[7 - ballY + 1], 7 - ballX ) != 0 && 
              BIT_CHECK(matrix[7 - ballY + 1], 7 - ballX - 1) != 0 &&
              BIT_CHECK(matrix[7 - ballY + 1], 7 - ballX + 1) != 0) 
            {
              stareMinge = LiftingStraight;
              break;   
            }
            else
            {
              stareMinge = UrcaSt;
              break;  
            } 
          }
          else //if there is a block underneath
          {
            BIT_CLEAR(matrix[7 - ballY + 1], 7 - ballX + 1); //clear the block
          } 
          
        }
        
        newX = ballX - 1; newY = ballY - 1;
        break;

    case CoboaraDr :

        Serial.println("CoboaraDr");
        
        if(ballX + 1 > 7)
        {
          stareMinge = CoboaraSt;
          break;
        }
        
        if(ballY - 1 < 0)
        {
          stareMinge = GameOver;
          break;
        }
        
        if(BIT_CHECK(matrix[7 - ballY + 1], 7 - ballX - 1) != 0) 
        {
          if(ballY == 1)//if there is the slide
          {
            Serial.println("Slider");

            if(BIT_CHECK(matrix[7 - ballY + 1], 7 - ballX ) != 0 && 
              BIT_CHECK(matrix[7 - ballY + 1], 7 - ballX - 1) != 0 &&
              BIT_CHECK(matrix[7 - ballY + 1], 7 - ballX + 1) != 0) 
            {
              stareMinge = LiftingStraight;
              break;   
            }
            else
            {            
              stareMinge = UrcaDr;
              break; 
            }
          }
          else //if there is a block underneath
          {
            Serial.println("Block underneath");
            BIT_CLEAR(matrix[7 - ballY - 1], 7 - ballX - 1); //clear the block
          }    
        }
        
        newX = ballX + 1; newY = ballY - 1;
        break;

       case LiftingStraight :

          Serial.println("LiftingStraight");
           //daca mingea a atins marginea superioara
          if(ballY + 1 > 7)
          {
            stareMinge = Coboara;
            break;
          }
          //se face peste tot, schimbare sistem de coordonate
          if(BIT_CHECK(matrix[7 - ballY - 1], 7 - ballX ) != 0) 
          {
              BIT_CLEAR(matrix[7 - ballY - 1], 7 - ballX); //clear the block
              stareMinge = Coboara;
              break;
          }
          //daca nu e block si nici margine superioara, migea urca
          newY = ballY + 1; newX = ballX;
          break;

        case Coboara :

          Serial.println("Coboara");

          if(ballY - 1 < 0)
          {
            stareMinge = GameOver;
            break;
          }
          
          if(BIT_CHECK(matrix[7 - ballY + 1], 7 - ballX ) != 0 && 
              BIT_CHECK(matrix[7 - ballY + 1], 7 - ballX - 1) != 0 &&
              BIT_CHECK(matrix[7 - ballY + 1], 7 - ballX + 1) != 0) 
          {
            stareMinge = LiftingStraight;
            break;   
          }
          else if(BIT_CHECK(matrix[7 - ballY + 1], 7 - ballX - 1) != 0)
          {
            stareMinge = UrcaDr;
            break; 
          }else if(BIT_CHECK(matrix[7 - ballY + 1], 7 - ballX + 1) != 0)
          {
            stareMinge = UrcaSt;
            break; 
          }
                    
          newY = ballY - 1; newX = ballX;
          break;

        case GameOver:
          Serial.println("GameOver");
          matrix[0]= 0b10001110;
          matrix[1]= 0b10001010;
          matrix[2]= 0b11101110;
          matrix[3]= 0b00000000;
          matrix[4]= 0b01101110;
          matrix[5]= 0b01000100;
          matrix[6]= 0b11000100;
          matrix[7]= 0b00000000;
             break;        
    }
    if(newX != -1 && newY != -1) //if new position is available
      drawBall(newX, newY);
  }

  drawMatrix();
}

void drawBall(byte newX, byte newY)
{//se sterge mingea din pozitia curenta
  BIT_CLEAR(matrix[7 - ballY], 7 - ballX);
//se deseneaza mingea pe noua pozitie 
 BIT_SET(matrix[7 - newY], 7 - newX);
  ledControl.setRow(0,7 - newY,matrix[7 - newY]);
//se actualizeaza pozitia actuala
  ballX = newX;
  ballY = newY;
}

void drawMatrix()
{ 
  for(int i = 0; i<=7; ++i)
   ledControl.setRow(0,i,matrix[i]);
}

