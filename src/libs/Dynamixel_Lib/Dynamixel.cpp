#include "Dynamixel.h"

void Dynamixelclass::begin(HardwareSerial &Serial, uint32_t baudRate, int8_t directionPINOUT){
    DynamixelSerial = &Serial;
    DynamixelSerial->begin(baudRate);
    directionPIN = directionPINOUT;
    pinMode(directionPIN, OUTPUT);
    delay(1000);
} 

unsigned char Dynamixelclass::ping(unsigned char MOTOR_ID){
    unsigned char pingArr[10]={0xFF, 0xFF, 0xFD, 0x00, MOTOR_ID, 0x03, 0x00, 0x01, 0, 0};
    unsigned short len = sizeof(pingArr)-2;
    unsigned short crc = update_crc(pingArr, len); // MInus two, because the the CRC_L and CRC_H are not included
    unsigned char CRC_L = (crc & 0x00FF);
    unsigned char CRC_H = (crc>>8) & 0x00FF;
    bool check = false;
    
    pingArr[8]=CRC_L;
    pingArr[9]=CRC_H;

    unsigned char *rArr;
    rArr = sendNreadPacket(pingArr, sizeof(pingArr));
    unsigned char value = rArr[8];
    return value;  // return value equal to error, if any error occurs.
}

short Dynamixelclass::degreesToBits(float degrees) {

	return short(degrees * 4096 / 360);
}

/****************** Here are "get" functions ******************/
/******************                          ******************/
/******************                          ******************/

int32_t Dynamixelclass::getPosition(unsigned char MOTOR_ID){
    //clearSerialBuffer();
    unsigned char Arr[14]={0xFF, 0xFF, 0xFD, 0x00, MOTOR_ID, 0x07, 0x00, 0x02, 0x84, 0x00, 0x04, 0x00, 0, 0};
    unsigned short len = sizeof(Arr)-2;
    unsigned short crc = update_crc(Arr, len); // MInus two, because the the CRC_L and CRC_H are not included
    unsigned char CRC_L = (crc & 0x00FF);
    unsigned char CRC_H = (crc>>8) & 0x00FF;

    Arr[12]=CRC_L;
    Arr[13]=CRC_H;

    unsigned char *rArr;
    rArr = sendNreadPacket(Arr, sizeof(Arr)); 
	int32_t result =(rArr[9] | rArr[10] << 8 | rArr[11] << 16 | rArr[12] << 24);  //Bitwize or for 4bit  
    //clearSerialBuffer();
    return result; 
}

float Dynamixelclass::getPositionDegree(unsigned char MOTOR_ID){
    //Converting from raw data to degrees (360/4095)
    float posd;
  posd = (float)getPosition(MOTOR_ID)*0.088;
  return posd;
}

int32_t Dynamixelclass::getVelocity(unsigned char MOTOR_ID){
    //clearSerialBuffer();
    unsigned char Arr[14]={0xFF, 0xFF, 0xFD, 0x00, MOTOR_ID, 0x07, 0x00, 0x02, 0x80, 0x00, 0x02, 0x00, 0, 0};
    unsigned short len = sizeof(Arr)-2;
    unsigned short crc = update_crc(Arr, len); // MInus two, because the the CRC_L and CRC_H are not included
    unsigned char CRC_L = (crc & 0x00FF);
    unsigned char CRC_H = (crc>>8) & 0x00FF;

    Arr[12]=CRC_L;
    Arr[13]=CRC_H;

    unsigned char *rArr;
    rArr = sendNreadPacket(Arr, sizeof(Arr));  
	int32_t result =(rArr[9] | rArr[10] << 8 );  //Bitwize or for 2 bit
    return result; 
}

int32_t Dynamixelclass::getPWM(unsigned char MOTOR_ID){
    //clearSerialBuffer();
    unsigned char Arr[14]={0xFF, 0xFF, 0xFD, 0x00, MOTOR_ID, 0x07, 0x00, 0x02, 0x7C, 0x00, 0x02, 0x00, 0, 0};
    unsigned short len = sizeof(Arr)-2;
    unsigned short crc = update_crc(Arr, len); // MInus two, because the the CRC_L and CRC_H are not included
    unsigned char CRC_L = (crc & 0x00FF);
    unsigned char CRC_H = (crc>>8) & 0x00FF;

    Arr[12]=CRC_L;
    Arr[13]=CRC_H;

    unsigned char *rArr;
    rArr = sendNreadPacket(Arr, sizeof(Arr));
	int32_t result =(rArr[9] | rArr[10] << 8 );  //Bitwize or for 2 bit
    return result; 
}

int32_t Dynamixelclass::getGain(unsigned char MOTOR_ID, char setControllerGain){
    //clearSerialBuffer();
    char controllerGain = 0x00;
    switch (setControllerGain)
    {
    case 'p':
        controllerGain = 0x54;
        break;
    case 'i':
        controllerGain = 0x52;
        break;
    case 'd':
        controllerGain = 0x50;
        break;
    default:
        controllerGain = 0x00;
        break;
    }
    unsigned char Arr[14]={0xFF, 0xFF, 0xFD, 0x00, MOTOR_ID, 0x07, 0x00, 0x02, controllerGain, 0x00, 0x02, 0x00, 0, 0};
    unsigned short len = sizeof(Arr)-2;
    unsigned short crc = update_crc(Arr, len); // MInus two, because the the CRC_L and CRC_H are not included
    unsigned char CRC_L = (crc & 0x00FF);
    unsigned char CRC_H = (crc>>8) & 0x00FF;

    Arr[12]=CRC_L;
    Arr[13]=CRC_H;

    unsigned char *rArr;
    rArr = sendNreadPacket(Arr, sizeof(Arr));  
	int32_t result =(rArr[9] | rArr[10] << 8 );  //Bitwize or for 2 bit
    return result; 
}

int32_t  Dynamixelclass::getMovingstatus(unsigned char MOTOR_ID){
    //clearSerialBuffer();
    unsigned char Arr[14]={0xFF, 0xFF, 0xFD, 0x00, MOTOR_ID, 0x07, 0x00, 0x02, 0x7B, 0x00, 0x02, 0x00, 0, 0}; // der mangler lige at blive tjekke om det her er korrekt
    unsigned short len = sizeof(Arr)-2;
    unsigned short crc = update_crc(Arr, len); // MInus two, because the the CRC_L and CRC_H are not included
    unsigned char CRC_L = (crc & 0x00FF);
    unsigned char CRC_H = (crc>>8) & 0x00FF;

    Arr[12]=CRC_L;
    Arr[13]=CRC_H;

    unsigned char *rArr;
    rArr = sendNreadPacket(Arr, sizeof(Arr));
	int32_t result =(rArr[9] | rArr[10] << 8 );  //Bitwize or for 2 bit
    return result; 
}

bool Dynamixelclass::getMoving(unsigned char MOTOR_ID){
    //clearSerialBuffer();
    unsigned char Arr[14]={0xFF, 0xFF, 0xFD, 0x00, MOTOR_ID, 0x07, 0x00, 0x02, 0x7A, 0x00, 0x01, 0x00, 0, 0};
    unsigned short len = sizeof(Arr)-2;
    unsigned short crc = update_crc(Arr, len); // MInus two, because the the CRC_L and CRC_H are not included
    unsigned char CRC_L = (crc & 0x00FF);
    unsigned char CRC_H = (crc>>8) & 0x00FF;

    Arr[12]=CRC_L;
    Arr[13]=CRC_H;

    unsigned char *rArr;
    rArr = sendNreadPacket(Arr, sizeof(Arr));
	bool result =(bool)rArr[9];  //Bitwize or for 2 bit
    return result; 
}

/****************** Here are "set" functions ******************/
/******************                          ******************/
/******************                          ******************/

void Dynamixelclass::setGain(unsigned char MOTOR_ID, unsigned short setVal, unsigned char setIntruction, char setControllerGain){
    char controllerGain = 0x00;
    switch (setControllerGain)
    {
    case 'p':
        controllerGain = 0x54;
        break;
    case 'i':
        controllerGain = 0x52;
        break;
    case 'd':
        controllerGain = 0x50;
        break;
    default:
        controllerGain = 0x00;
        break;
    }
    signed short val = setVal;
    unsigned char val_L = (val & 0x00FF);
    unsigned char val_H = (val>>8) & 0x00FF;

    unsigned char Arr[14]={0xFF, 0xFF, 0xFD, 0x00, MOTOR_ID, 0x07, 0x00, setIntruction, controllerGain, 0x00, val_L, val_H, 0, 0};
    unsigned short len = sizeof(Arr)-2;
    unsigned short crc = update_crc(Arr, len); // MInus two, because the the CRC_L and CRC_H are not included
    unsigned char CRC_L = (crc & 0x00FF);
    unsigned char CRC_H = (crc>>8) & 0x00FF;

    Arr[12]=CRC_L;
    Arr[13]=CRC_H;

    clearSerialBuffer();
    sendPacket(Arr, sizeof(Arr));
}

void Dynamixelclass::setPosition(unsigned char MOTOR_ID, signed short setVal, unsigned char setIntruction){
    signed short val = setVal;
    val %= 4096;
   // unsigned char val_LL = (val & 0x00FF);
   // unsigned char val_L = (val>>8) & 0x00FF;
   // unsigned char val_H = (val>>16) & 0x00FF;
   // unsigned char val_HH = (val>>24) & 0x00FF;
    unsigned char val_LL = (val & 0xFF);
    unsigned char val_L =  (val & 0xFF00) >> 8;
    unsigned char val_H =  (val & 0xFF0000) >> 16;
    unsigned char val_HH =  (val & 0xFF000000) >> 24;

    unsigned char Arr[16]={0xFF, 0xFF, 0xFD, 0x00, MOTOR_ID, 0x09, 0x00, setIntruction, 0x74, 0x00, val_LL, val_L, val_H, val_HH, 0, 0};
    unsigned short len = sizeof(Arr)-2;
    unsigned short crc = update_crc(Arr, len); // MInus two, because the the CRC_L and CRC_H are not included
    unsigned char CRC_L = (crc & 0x00FF);
    unsigned char CRC_H = (crc>>8) & 0x00FF;

    Arr[14]=CRC_L;
    Arr[15]=CRC_H;

    //clearSerialBuffer(); // Denne linje er mega vigtig for at sikre man kun læser vædierne én gang
    sendPacket(Arr, sizeof(Arr));
}

void Dynamixelclass::setPositionDegree(unsigned char MOTOR_ID, float setVal, unsigned char setIntruction){
    //short value = (short)setVal / 0.088;
    //short value = short(setVal * 4096 / 360);
    setPosition(MOTOR_ID, short(setVal * 4095 / 360), setIntruction);
}

void Dynamixelclass::setPWM(unsigned char MOTOR_ID, unsigned short setVal, unsigned char setIntruction){
    unsigned short val = setVal;
    unsigned char val_H = (val & 0x00FF);
    unsigned char val_L = (val>>8) & 0x00FF;

    unsigned char Arr[14]={0xFF, 0xFF, 0xFD, 0x00, MOTOR_ID, 0x07, 0x00, setIntruction, 0x64, 0x00, val_L, val_H, 0, 0};
    unsigned short len = sizeof(Arr)-2;
    unsigned short crc = update_crc(Arr, len); // MInus two, because the the CRC_L and CRC_H are not included
    unsigned char CRC_L = (crc & 0x00FF);
    unsigned char CRC_H = (crc>>8) & 0x00FF;

    Arr[12]=CRC_L;
    Arr[13]=CRC_H;

    clearSerialBuffer();
    sendPacket(Arr, sizeof(Arr));
}

void Dynamixelclass::setOperationMode(unsigned char MOTOR_ID, unsigned short setVal, unsigned char setIntruction){
    unsigned char Arr[13]={0xFF, 0xFF, 0xFD, 0x00, MOTOR_ID, 0x06, 0x00, setIntruction, 0x0B, 0x00, setVal, 0, 0};
    unsigned short len = sizeof(Arr)-2;
    unsigned short crc = update_crc(Arr, len); // MInus two, because the the CRC_L and CRC_H are not included
    unsigned char CRC_L = (crc & 0x00FF);
    unsigned char CRC_H = (crc>>8) & 0x00FF;

    Arr[11]=CRC_L;
    Arr[12]=CRC_H;

    clearSerialBuffer();
    sendPacket(Arr, sizeof(Arr));

                //0= Current Control Mode
                //1= velocity Control Mode
                //3= Position Control Mode (default)
                //4= Extended Position Control Mode
                //5= Current based posistion Control Mode
                //16= PWM control Control Mode
}

void Dynamixelclass::setAccelerationProfile(unsigned char MOTOR_ID, unsigned short setVal, unsigned char setIntruction){
   unsigned short val = setVal;
    val %= 4096;
    unsigned char val_LL = (val & 0xFF);
    unsigned char val_L =  (val & 0xFF00) >> 8;
    unsigned char val_H =  (val & 0xFF0000) >> 16;
    unsigned char val_HH =  (val & 0xFF000000) >> 24;

    unsigned char Arr[16]={0xFF, 0xFF, 0xFD, 0x00, MOTOR_ID, 0x09, 0x00, setIntruction, 0x6C, 0x00, val_LL, val_L, val_H, val_HH, 0, 0};
    unsigned short len = sizeof(Arr)-2;
    unsigned short crc = update_crc(Arr, len); // MInus two, because the the CRC_L and CRC_H are not included
    unsigned char CRC_L = (crc & 0x00FF);
    unsigned char CRC_H = (crc>>8) & 0x00FF;

    Arr[14]=CRC_L;
    Arr[15]=CRC_H;

    clearSerialBuffer();
    sendPacket(Arr, sizeof(Arr));
}

void Dynamixelclass::setVelocityProfile(unsigned char MOTOR_ID, unsigned short setVal, unsigned char setIntruction){
    unsigned short val = setVal;
    val %= 4096;
    unsigned char val_LL = (val & 0xFF);
    unsigned char val_L =  (val & 0xFF00) >> 8;
    unsigned char val_H =  (val & 0xFF0000) >> 16;
    unsigned char val_HH =  (val & 0xFF000000) >> 24;

    unsigned char Arr[16]={0xFF, 0xFF, 0xFD, 0x00, MOTOR_ID, 0x09, 0x00, setIntruction, 0x70, 0x00, val_LL, val_L, val_H, val_HH, 0, 0};
    unsigned short len = sizeof(Arr)-2;
    unsigned short crc = update_crc(Arr, len); // MInus two, because the the CRC_L and CRC_H are not included
    unsigned char CRC_L = (crc & 0x00FF);
    unsigned char CRC_H = (crc>>8) & 0x00FF;

    Arr[14]=CRC_L;
    Arr[15]=CRC_H;

    clearSerialBuffer();
    sendPacket(Arr, sizeof(Arr));
}
void Dynamixelclass::setEnableTorque(unsigned char MOTOR_ID, unsigned char setVal, unsigned char setIntruction){
    unsigned char Arr[13]={0xFF, 0xFF, 0xFD, 0x00, MOTOR_ID, 0x06, 0x00, setIntruction, 0x40, 0x00, setVal, 0, 0};
    unsigned short len = sizeof(Arr)-2;
    unsigned short crc = update_crc(Arr, len); // MInus two, because the the CRC_L and CRC_H are not included
    unsigned char CRC_L = (crc & 0x00FF);
    unsigned char CRC_H = (crc>>8) & 0x00FF;

    Arr[11]=CRC_L;
    Arr[12]=CRC_H;

    clearSerialBuffer();
    sendPacket(Arr, sizeof(Arr));
}

void Dynamixelclass::setAction(unsigned char MOTOR_ID){
    unsigned char Arr[10]={0xFF, 0xFF, 0xFD, 0x00, MOTOR_ID, 0x03, 0x00, 0x05, 0, 0};
    unsigned short len = sizeof(Arr)-2;
    unsigned short crc = update_crc(Arr, len); // MInus two, because the the CRC_L and CRC_H are not included
    unsigned char CRC_L = (crc & 0x00FF);
    unsigned char CRC_H = (crc>>8) & 0x00FF;

    Arr[8]=CRC_L;
    Arr[9]=CRC_H;

    clearSerialBuffer();
    sendPacket(Arr, sizeof(Arr));
}

void Dynamixelclass::setStatusReturnLevel(unsigned char MOTOR_ID, unsigned short setVal, unsigned char setIntruction){
    unsigned char Arr[13]={0xFF, 0xFF, 0xFD, 0x00, MOTOR_ID, 0x06, 0x00, setIntruction, 0x44, 0x00, setVal, 0, 0};
    unsigned short len = sizeof(Arr)-2;
    unsigned short crc = update_crc(Arr, len); // MInus two, because the the CRC_L and CRC_H are not included
    unsigned char CRC_L = (crc & 0x00FF);
    unsigned char CRC_H = (crc>>8) & 0x00FF;

    Arr[11]=CRC_L;
    Arr[12]=CRC_H;

    clearSerialBuffer();
    sendPacket(Arr, sizeof(Arr));

            /*
            setVal = 0x00 -> Returns PING Instuction only
            setVal = 0x01 -> Returns PING and READ Instuctions only
            setVal = 0x02 -> Returns all Instructions
            */
}

void Dynamixelclass::setMaxPosition(unsigned char MOTOR_ID, unsigned short setVal, unsigned char setIntruction){
    unsigned short val = setVal;
    val %= 4096;
    unsigned char val_LL = (val & 0xFF);
    unsigned char val_L =  (val & 0xFF00) >> 8;
    unsigned char val_H =  (val & 0xFF0000) >> 16;
    unsigned char val_HH =  (val & 0xFF000000) >> 24;

    unsigned char Arr[16]={0xFF, 0xFF, 0xFD, 0x00, MOTOR_ID, 0x09, 0x00, setIntruction, 0x30, 0x00, val_LL, val_L, val_H, val_HH, 0, 0};
    unsigned short len = sizeof(Arr)-2;
    unsigned short crc = update_crc(Arr, len); // MInus two, because the the CRC_L and CRC_H are not included
    unsigned char CRC_L = (crc & 0x00FF);
    unsigned char CRC_H = (crc>>8) & 0x00FF;

    Arr[14]=CRC_L;
    Arr[15]=CRC_H;

    clearSerialBuffer();
    sendPacket(Arr, sizeof(Arr));
}

void Dynamixelclass::setMinPosition(unsigned char MOTOR_ID, unsigned short setVal, unsigned char setIntruction){
    unsigned short val = setVal;
    val %= 4096;
    unsigned char val_LL = (val & 0xFF);
    unsigned char val_L =  (val & 0xFF00) >> 8;
    unsigned char val_H =  (val & 0xFF0000) >> 16;
    unsigned char val_HH =  (val & 0xFF000000) >> 24;

    unsigned char Arr[16]={0xFF, 0xFF, 0xFD, 0x00, MOTOR_ID, 0x09, 0x00, setIntruction, 0x34, 0x00, val_LL, val_L, val_H, val_HH, 0, 0};
    unsigned short len = sizeof(Arr)-2;
    unsigned short crc = update_crc(Arr, len); // MInus two, because the the CRC_L and CRC_H are not included
    unsigned char CRC_L = (crc & 0x00FF);
    unsigned char CRC_H = (crc>>8) & 0x00FF;

    Arr[14]=CRC_L;
    Arr[15]=CRC_H;

    clearSerialBuffer();
    sendPacket(Arr, sizeof(Arr));
}

/****************** Here are private functions ******************/
/******************                            ******************/
/******************                            ******************/

void Dynamixelclass::clearSerialBuffer(void){
    while (DynamixelSerial->read() != -1);  // Clear RX buffer;
}

double Dynamixelclass::rad2deg(double rad){
    double deg = 0;
    rad * 180/pi;
    return deg;
 }

double Dynamixelclass::deg2rad(double deg){
    double rad = 0;
    deg * pi / 180;
    return rad;
 }

void Dynamixelclass::sendPacket(unsigned char *arr, int arrSIZE){
    digitalWrite(directionPIN, HIGH);
    delayMicroseconds(500);
    DynamixelSerial->write(arr, arrSIZE);
    DynamixelSerial->flush();
    noInterrupts();
    delayMicroseconds(500);
    digitalWrite(directionPIN, LOW);
    interrupts();
    delay(20);
}

 unsigned char* Dynamixelclass::sendNreadPacket(unsigned char *arr, int arrSIZE){
    
    unsigned char incomingbyte;
    unsigned char len = 0;
    digitalWrite(directionPIN, HIGH);
    delayMicroseconds(500);
    DynamixelSerial->write(arr, arrSIZE);
    DynamixelSerial->flush();
    //noInterrupts();
    delayMicroseconds(500);
    digitalWrite(directionPIN, LOW);
   // interrupts();

    if (DynamixelSerial->available()) {
    // read the incoming byte:
        if(DynamixelSerial->read() == 0xFF){
            if(DynamixelSerial->read() == 0xFF && DynamixelSerial->peek() == 0xFD){    // check that there are  "0xFF" and "0xFD" header data
                ReturnPacket[0] = 0xFF;
                ReturnPacket[1] = 0xFF;
                ReturnPacket[2] = DynamixelSerial->read(); //0xFD
                ReturnPacket[3] = DynamixelSerial->read(); //0x00
                ReturnPacket[4] = DynamixelSerial->read(); //MOTOR_ID
                ReturnPacket[5] = DynamixelSerial->read(); //LEN_L
                ReturnPacket[6] = DynamixelSerial->read(); //LEN_H
                len = (ReturnPacket[6] << 8) + ReturnPacket[5]; // Lenght of remaining status packet

                for(int i=7; i<7+(int)len; i++){
                    incomingbyte = DynamixelSerial->read();        //Save incomingbyte
                    ReturnPacket[i]=incomingbyte;          //Save data in ReturnPacket array
                }
            }
        }
    }
    return ReturnPacket; 
}

void Dynamixelclass::inverseKinematics(double x, double y, double z){
    double L1 = 0.0528;
    double L2 = 0.21988;
    double L3 = 0.22368;

    signed short Arr[3]; 

    //Theta 1
    double theta1a=((atan2(y,x))); 
    double theta1b=theta1a+pi;
    double theta1c = theta1a;
    double theta1d = theta1b;
    
    Arr[0] = (rad2deg(theta1a) * (4095 / 360))+2047;
    setPosition(01, Arr[0], 04);
    delay(10);
    double r = sqrt((x*x)+(y*y));
    double c = sqrt((z-L1)*(z-L1)+(r*r));
    double a=atan2((z-L1),r);
    double A=acos(((c*c)+(L2*L2)-(L3*L3))/(2*c*L2));
    
    //Theta 2
    double theta2a = -(a+A);
    double theta2b = +(a+A)-pi;
    double theta2c = A-a;
    double theta2d = a-A-pi;
    Arr[1] = (theta2a * (4095 / (2*pi)))+3073;
    setPosition(02, Arr[1], 04);
    delay(10);
    //Theta 3
    double theta3 = acos(((L3*L3)+(L2*L2)-(c*c))/(2*L3*L2));
    double theta3a = pi-theta3;
    double theta3b = -pi+theta3;
    double theta3c = -pi+theta3;
    double theta3d = pi-theta3; 

    Arr[2] = (theta3a * (4095 / (2*pi)))+2047;
    setPosition(03, Arr[2], 04);
delay(10);
    delay(10);
    setAction(0xfe);
}
/*
double Dynamixelclass::atan2(double numerator, double denominator){ //(y,x)
    double value = 0;
    if (numerator > 0){
        value = atan((denominator / numerator));
    }
    if (numerator < 0 && denominator >= 0){
        value = (atan(denominator / numerator)) + pi;
    }
    if (numerator < 0 && denominator < 0){
        value = (atan(denominator / numerator)) - pi;
    }
    if (numerator == 0 && denominator > 0){
        value = +pi/2;
    }
    if (numerator == 0 && denominator < 0){
        value = -pi/2;
    }
    if (numerator == 0 && denominator == 0){
        value = 0;
    }
    return value;
}
*/
unsigned short Dynamixelclass::update_crc(unsigned char *data_blk_ptr, unsigned short data_blk_size){
    unsigned short crc_accum = 0;
    unsigned short i, j;
    unsigned short crc_table[256] = {
        0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
        0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
        0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
        0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
        0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
        0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
        0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
        0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
        0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
        0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
        0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
        0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
        0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
        0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
        0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
        0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
        0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
        0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
        0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
        0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
        0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
        0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
        0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
        0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
        0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
        0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
        0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
        0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
        0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
        0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
        0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
        0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
    };

    for(j = 0; j < data_blk_size; j++)
    {
        i = ((unsigned short)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
        crc_accum = (crc_accum << 8) ^ crc_table[i];
    }

    return crc_accum;
  }
