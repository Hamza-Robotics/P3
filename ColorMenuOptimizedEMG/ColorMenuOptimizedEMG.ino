#include <EMG_menu.h>
#include <EMG.h>

//Det er fordi jeg elskser alle farver...
#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

EMGclass xbee;
EMGmenu menuClass;


  int upButton = 52;
  int downButton = 53;
  int selectButton = 51;
  int menu = 0;
  int y = 0;    
  bool MainMenu = true;
  bool Point1State = false;
  bool Point2State = false;
  bool Point3State = false;
  bool Point4State = false;
  int sensorMin = 1023; 
  int sensorMax = 0;
  int sensorValue = 0;
  int PointerY[5]={35,70,105,140,170};


void setup() {
  Serial.begin(9600);
  xbee.begin(Serial1, 115200);
  menuClass.reset();
  menuClass.begin(0x9341);
  pinMode(upButton, INPUT_PULLUP);
  pinMode(downButton, INPUT_PULLUP);
  pinMode(selectButton, INPUT_PULLUP);
  menuClass.fillScreen(BLACK);
  callibration();
  menuClass.fillScreen(BLACK);
  updateMenu();
  
}

//Print Pointer
  void printPointer(){
    menuClass.SetParam(WHITE,3,40,PointerY[y]);
    menuClass.print("->",0);
  }
      
  void updateMenu(){
    if (MainMenu == true){
    PrintMainMenu();  
    };
    switch (menu){
    case 0:
    menuClass.fillRect(40,0,33,240,BLACK);
    printPointer();
    break;
    case 1:
    menuClass.fillRect(40,0,33,240,BLACK);
    printPointer();
    break;
    case 2:
    menuClass.fillRect(40,0,33,240,BLACK);
    printPointer();

    break;
    case 3:
    menuClass.fillRect(40,0,33,240,BLACK);
    printPointer();
    
    break;
    case 4:
    menuClass.fillRect(40,0,33,240,BLACK);
    printPointer();
    
    break;
    }
  }
void execute() {
  switch (menu) {
    case 0:
    if (MainMenu == false){
      
      if(Point1State == false){
      menuClass.SetParam(RED,3,70,PointerY[y]);
      menuClass.print(" Point 1 ",0);
      PointState = true; 
      } 
      else if(Poin1tState == true){
      menuClass.SetParam(GREEN,3,70,PointerY[y]);
      menuClass.print(" Point 1 ",0);
      PointState = false;
      }
    }
      break;
    case 1:
    if (MainMenu == false){
      
      if(Point2State == false){
      menuClass.SetParam(RED,3,70,PointerY[y]);
      menuClass.print(" Point 2 ",0);
      PointState = true; 
      } 
      else if(Point2State == true){
      menuClass.SetParam(GREEN,3,70,PointerY[y]);
      menuClass.print(" Point 2 ",0);
      PointState = false;
      }
    }
      break;

    case 2:
    if (MainMenu == false){
      
      if(Point3State == false){
      menuClass.SetParam(RED,3,70,PointerY[y]);
      menuClass.print(" Point 3 ",0);
      PointState = true;
      }
      else if(Point3State == true){
      menuClass.SetParam(GREEN,3,70,PointerY[y])
      menuClass.print(" Point 3 ",0);
      PointState = false; 
      }
    }
      break;

    case 3:
    if (MainMenu == false){
      
      if(Point4State == false){
      menuClass.SetParam(RED,3,70,PointerY[y]);
      menuClass.print(" Point 4 ",0);
      PointState = true; 
      } 
      else if(Point4State == true){
      menuClass.SetParam(GREEN,3,70,printPointer[y]);
      menuClass.print(" Point 4 ",0);
      PointState = false;
      }
    }
      break;
    case 4:
    menuClass.fillScreen(BLACK);
    if(MainMenu == true){
    PrintSubMenu();
    MainMenu = false;  
    }
    else{
    menuClass.fillScreen(BLACK);
    PrintMainMenu();
    MainMenu = true;
    }
      break;
  }
}
void callibration(){
    while (millis() < 5000) 
    {
    menuClass.SetParam(GREEN,3,0,35);
    menuClass.print(" Shake ur head along X-axis cunt ",0);
    xbee.updateData();
    sensorValue = xbee.getAccX();
    Serial.println(sensorValue);
    // record the maximum sensor value
    if (sensorValue > sensorMax) {
    sensorMax = sensorValue;
    }
    // record the minimum sensor value
    if (sensorValue < sensorMin) {
    sensorMin = sensorValue;
    }
  
  sensorValue = map(sensorValue, sensorMin, sensorMax, 0, 255);
    menuClass.SetParam(GREEN,2,0,70);
    menuClass.print(sensorValue,0);
    }
}
void loop() {
  if (!digitalRead(downButton)){
    menu++;
    y++;
    if (menu>4){
     menu=0;
     y = 0;
    }
    updateMenu();
    delay(200);
    //while (!digitalRead(downButton));
  }
  if (!digitalRead(upButton)){
    menu--;
    y--;
    if (menu<0){
      menu=4;
      y = 4;
    }
    updateMenu();
    delay(200);
    //while (!digitalRead(upButton));
  }
  if (!digitalRead(selectButton)){
    execute();
    ///updateMenu();
    delay(200);
    while (!digitalRead(selectButton));
  }
}
