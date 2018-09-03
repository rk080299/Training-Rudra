#include <Wire.h>
#include <HMC5883L.h>
#include<TinyGPS++.h>
HMC5883L compass;
TinyGPSPlus gps;
#include <Servo.h> //metaldetector start
Servo serArm;
Servo serCam;
#define capPin A5
#define pulsePin A4
long sumExpect=0; 
long ignor=0;   
long diff=0;        
long pTime=0;
long buzPeriod=0; //metaldetector end
int i=1;
int flag;
const int ptrigg = 12; //ultrasonic
const int pecho = 13;  //ultrasonic
float la; //latitude
float lo; //longitude
float hd; //heading in degrees
long int turnCount=0;
float stla;
float stlo;
float endla;
float endlo;
const int pwm[4]={2,6,8,9};
const int m[4][2]={24,26,34,36,38,40,42,44};

void setup() {
  for(int i=0;i<4;i++)
    pinMode(pwm[i],OUTPUT);
  for(int i=0;i<4;i++)
  { for(int j=0;j<2;j++)
    { pinMode(m[i][j],OUTPUT); }}
   serArm.attach(10);
   serArm.writeMicroseconds(1500);
   serCam.attach(4);
   serCam.writeMicroseconds(1500);
   Wire.begin();
   Serial1.begin(9600);
   Serial.begin(9600); 
   compass = HMC5883L();
   compass.setRange(HMC5883L_RANGE_1_3GA);
   compass.setMeasurementMode(HMC5883L_CONTINOUS);
   //entry
   //entry of starting and destination latitudes and longitudes x1,y1,x2,y2 
   //entry of comments to be
   float x1=0.0;
   float y1=0.0;
   float x2=12.0;
   float y2=14.0;
   //end of comments to be
   float dist=distance(x1,y1,x2,y2);
   float b=dist/0.3;
   int c=(int)b;
   if(b>c)
   turnCount=c+1;
   else
   turnCount=c;
   Serial.print("TURNCOUNT");
   Serial.println(turnCount);
   stla=x1-.025;
   stlo=y1-.105;
   endla=x2+.025;
   endlo=y2+.105;
}

void forward()
{   Serial.println("FORWARD");
    pwmslow();
    for(int i=0;i<4;i++)
    { digitalWrite(m[i][0],HIGH);
      digitalWrite(m[i][1],LOW); }
}

void backward()
{   Serial.println("BACKWARD");
    pwmslow();
    for(int i=0;i<4;i++)
    { digitalWrite(m[i][0],HIGH);
      digitalWrite(m[i][1],LOW); }
}

void left()
{   Serial.println("LEFT");
    pwmslow();
    digitalWrite(m[0][0],LOW);
    digitalWrite(m[0][1],HIGH);
    digitalWrite(m[1][0],LOW);
    digitalWrite(m[1][1],HIGH);
    digitalWrite(m[2][0],HIGH);
    digitalWrite(m[2][1],LOW);
    digitalWrite(m[3][0],HIGH);
    digitalWrite(m[3][1],LOW);
}

void right()
{   Serial.println("RIGHT");
    pwmslow();
    digitalWrite(m[0][0],HIGH);
    digitalWrite(m[0][1],LOW);
    digitalWrite(m[1][0],HIGH);
    digitalWrite(m[1][1],LOW);
    digitalWrite(m[2][0],LOW);
    digitalWrite(m[2][1],HIGH);
    digitalWrite(m[3][0],LOW);
    digitalWrite(m[3][1],HIGH);
}

void stopp()
{   Serial.println("STOP");
    for(int i=0;i<4;i++)
    analogWrite(pwm[i],0);
}

void pwmcon()
{ Serial.println("SPEED FAST");
  for(int i=0;i<4;i++)
  analogWrite(pwm[i],100);
}

void pwmslow()
{ Serial.println("SPEED SLOW");
  for(int i=0;i<4;i++)
  analogWrite(pwm[i],50);
}

void obstacle(int i)
{
  long range=ultra();
  if(range<30 && range>=0)
  { 
    stopp();
    if(i%2==0)
    motionLR();
    else
    motionRL();
  }  
}

void mine(int i)
{
    stopp();
    gpscode();
    sendData(la,lo); //send the values of gps
    cameraOn(); //call camera question
    if(i%2==0)
    motionLR();
    else
    motionRL();

    //serCam.writeMicroseconds(750);
}
 
void motionLR()
{  int cc=0;
do{ cc++;
    dirAnti();
    forward();
    delay(1000);
    dirClock();
    }while(check()==1);
    forward();
    delay(3000);
    dirClock();
    while(cc>0)
    {
    forward();
    delay(1000);
    cc--;
    }
    dirAnti();
    afterMotion(); 
} 

void motionRL()
{ int cc=0;
do{ cc++;
    dirClock();
    forward();
    delay(1000);
    dirAnti();
    }while(check()==1);
    forward();
    delay(3000);
    dirAnti();
    while(cc>0)
    {
    forward();
    delay(1000);
    cc--;
    }
    dirClock();
    afterMotion();   
}

void afterMotion()
{
  gpscode();
  double b=bear(la,lo,endla,endlo);
  double h=compasscode();
  comeToRightPlace(endla,endlo,b,h);
}

void gpscode()
{ if(gps.encode(Serial1.read()))
    {  la=(gps.location.lat(),6);
        lo=(gps.location.lng(),6);
       } }

float compasscode()
{
   Vector raw = compass.readNormalize();
   float heading = atan2(raw.YAxis, raw.XAxis);
   float decAngle =-0.01978;
   heading += decAngle;
   if (heading < 0)
    heading += 2*PI;
   if (heading > 2*PI )
    heading -= 2*PI;
   hd = heading * 180/PI;
   hd=map(hd,360,0,0,360);
   return(hd);
}

void align(float a)
{ //a=initial heading
  hd=compasscode();
  char ch=turncheck(hd,a);
  if(ch=='L')
  {
    right();
    while(!(abs(hd-a)<=3))
    {
     hd=compasscode(); 
    }
    stopp();
    forward();
  }
  else if(ch=='R')
  {
    left();
    while(!(abs(hd-a)<=3))
    {
     hd=compasscode(); 
    }
    stopp();
    forward();
  }
  else
  {
    forward();
  }  
}

char turncheck(float hd, float a)
{
  if((hd>=270 && hd<=360) && (a>=0 && a<=90))
    return ('L');
  else if(a>hd)
    return('L');
  else if(hd>a)
   return('R');
  else 
   return('N');
}

float distance(float sla,float slo,float dla,float dlo)
{  
    sla=sla*0.0174533;
    slo=slo*0.0174533;
    dla=dla*0.0174533;
    dlo=dlo*0.0174533;
    float dist_calc = 0;
    float diflat = 0;
    float diflon = 0;
    diflat = dla - sla; 
    diflon = dlo - dla; 
    float distan = (sin(diflat / 2.0) * sin(diflat / 2.0));
    dist_calc = cos(sla);
    dist_calc *= cos(dla);
    dist_calc *= sin(diflon / 2.0);
    dist_calc *= sin(diflon / 2.0);
    distan += dist_calc;
    distan = (2 * atan2(sqrt(distan), sqrt(1.0 - distan)));
    distan *= 6371000.0;
    return(distan);
}

void dirAnti()
{ Serial.println("ROTATE ANTICLOCKWISE 90");
  float initialH=compasscode();
  Serial.print("initialH");
  Serial.println(initialH);
  float finalH=initialH-90.0;
  if(finalH<0)
   finalH+=360.0;
  Serial.print("FinalH");
  Serial.println(finalH);
  Serial.println((initialH-finalH));
  Serial.println(abs(initialH-finalH));
  while(!((abs(initialH-finalH))<=3))
  {
   left(); 
   initialH=compasscode();
  }
  Serial.println("stop");
  stopp();
}

void dirClock()
{ Serial.println("ROTATE CLOCKWISE 90");
  float initialH=compasscode();
  Serial.print("initialH");
  Serial.println(initialH);
  float finalH=initialH+90.0;
  if(finalH>360)
   finalH-=360.0;
  Serial.print("FinalH");
  Serial.println(finalH);
  Serial.println((initialH-finalH));
  Serial.println(abs(initialH-finalH));
  while(((abs(initialH-finalH))>=3))
  {
   right(); 
   initialH=compasscode();
   Serial.println(initialH);
  }
  Serial.println("stop");
  stopp();
}

long ultra() {
  long duration, cm;
  pinMode(ptrigg, OUTPUT);
  digitalWrite(ptrigg, LOW);
  delayMicroseconds(2);
  digitalWrite(ptrigg, HIGH);
  delayMicroseconds(5);
  digitalWrite(ptrigg, LOW); 
  pinMode(pecho, INPUT);
  duration = pulseIn(pecho, HIGH);
  cm = duration/29/2;
  return(cm);
}

void serA()
{
  mys.writeMicroseconds(1900);
  delay(10);
  mys.writeMicroseconds(1500);
  delay(10);
}

/*void serC()
{
  mys.writeMicroseconds(1000);
  delay(10);
  mys.writeMicroseconds(750);
  delay(10);
}*/

int check() 
{
  int minval=1023;
  int maxval=0;
  long unsigned int sum=0;
  for (int i=0; i<256; i++)
  {
    pinMode(capPin,OUTPUT);
    digitalWrite(capPin,LOW);
    delayMicroseconds(20);
    pinMode(capPin,INPUT);
    applyPulses();
    int val = analogRead(capPin); 
    minval = min(val,minval);
    maxval = max(val,maxval);
    sum+=val;   
    long unsigned int cTime=millis();
    char buzState=0;
    if (cTime<pTime+10)
    {
      if (diff>0)
        buzState=1;
      else if(diff<0)
        buzState=2;
    }
    if (cTime>pTime+buzPeriod)
    {
      if (diff>0)
      buzState=1;
      else if (diff<0)
      buzState=2;
      pTime=cTime;   
    }
    if (buzPeriod>300)
    buzState=0;

    if (buzState==0)
    {
    //Serial.println("notfound");
    return 0;
    }  
    else if (buzState==1 || buzState==2)
    {
     // Serial.println("found");
      return 1;
    }
  }
  sum-=minval; 
  sum-=maxval;
  
  if (sumExpect==0) 
  sumExpect=sum<<6;
  long int avgsum=(sumExpect+32)>>6; 
  diff=sum-avgsum;
  if (abs(diff)<avgsum>>10)
  {
    sumExpect=sumExpect+sum-avgsum;
    ignor=0;
  } 
  else 
    ignor++;
  if (ignor>64)
  { 
    sumExpect=sum<<6;
    ignor=0;
  }
  if (diff==0) 
    buzPeriod=1000000;
  else 
  buzPeriod=avgsum/(2*abs(diff));    
}

void applyPulses()
{
    for (int i=0;i<3;i++) 
    {
      digitalWrite(pulsePin,HIGH); 
      delayMicroseconds(3);
      digitalWrite(pulsePin,LOW);  
      delayMicroseconds(3);
    }
}

double bear(double sla,double slo,double dla,double dlo)
{
  sla*=(PI/180);
  slo*=(PI/180);
  dla*=(PI/180);
  dlo*=(PI/180);
  double br=atan2(((sin(dlo-slo))*(cos(dla))),(((cos(sla))*(sin(dla)))-((sin(sla))*(cos(dla))*(cos(dlo-slo)))));
  br*=(180/PI);
  return br;
}

void comeToRightPlace(float lati,float longi,float br,float hd)
{ hd=compasscode();
  if(br<hd)
  {
   left();
   comeToRightPlace(lati,longi,br,hd); 
  }
  else if(br>hd)
  {
   right();
   comeToRightPlace(lati,longi,br,hd);
  }
  else
  stopp();
  
  gpscode();
  while(la!=lati || lo!=longi)
  {
    forward();
  }
  stopp();
}

void comeToRightAngle(float br,float hd)
{ hd=compasscode();
  if(abs(br-hd)<=3)
  { stopp();
    Serial.println("reached");
  }
  else if(br<hd)
  {
   left();
   comeToRightAngle(br,hd); 
  }
  else if(br>hd)
  {
   right();
   comeToRightAngle(br,hd);
  }
  stopp();
}

/*void loop() {
  float head; ser(); 
  int p=check();
  if(p==1) //mine detected
  mine(i);
  if(i<=turnCount)
  { 
   head=compasscode();
   if(i%2!=0)//left to right ODD
   {
     while(endla!=stla && endlo!=stlo) //1m
     {Serial.println("left to right 1m");
      forward();
      gpscode();
      stla=la;
      stlo=lo;
      align(head);
      pwmcon(); 
     }
     stopp();
     dirAnti();
     stla=endla;
     stlo=endlo;
     endla+=.0517;
     endlo+=.2105;
     while(endla!=stla && endlo!=stlo) //.3m
     {Serial.println("FORWARD 0.3m");
      forward();
      gpscode();
      stla=la;
      stlo=lo;
     }
     stopp(); 
     stla=endla;
     stlo=endlo;
     endla-=.05; 
     endlo-=.21;
     i++;
   }
   else //right to left EVEN
   {
    while(endla!=stla && endlo!=stlo) //1m
     {Serial.println("right to left 1m");
      forward();
      gpscode();
      stla=la;
      stlo=lo;
      align(head);
      pwmcon(); 
     }
     stopp();
     dirClock();
     stla=endla;
     stlo=endlo;
     endla+=.0517;
     endlo+=.2105;
      while(endla!=stla && endlo!=stlo) //.3m
     {Serial.println("BACKWARD 0.3m");
      forward();
      gpscode();
      stla=la;
      stlo=lo;
     }
     stopp();
     stla=endla;
     stlo=endlo;
     endla+=.05; 
     endlo+=.21;
     i++;
   }
  }
}
*/

void loop() {
  float head; serA(); obstacle(); double b;
  int p=check(); 
  if(p==1) //mine detected
    {
     serCam.writeMicroseconds(1000);
     mine(i);
    }
  if(flag>4)
   flag=1;
  if(i<=turnCount)
  { 
   if(i%2!=0 && flag==1)//left to right ODD
   { head=compasscode();
     forward();
     gpscode();
     stla=la;
     stlo=lo;
     align(head);
     if(abs(endla-stla)<=0.000002 && abs(endlo-stlo)<=0.000002) //1m
      {
       stopp();
       dirAnti();
       flag=2;
       stla=endla;
       stlo=endlo;
       endla+=.0517;
       endlo+=.2105;
       b=bear(stla,stlo,endla,endlo);
       head=compasscode();
       comeToRightAngle(b,head);
      }
   }
   if(i%2!=0 && flag==2)//forward ODD
   { head=compasscode();
     forward();
     gpscode();
     stla=la;
     stlo=lo;
     align(head);
     if(abs(endla-stla)<=0.000002 && abs(endlo-stlo)<=0.000002) //.3m
      {
       stopp();
       dirAnti();
       flag=3;
       stla=endla;
       stlo=endlo;
       endla-=.05;
       endlo-=.21;
       b=bear(stla,stlo,endla,endlo);
       head=compasscode();
       comeToRightAngle(b,head);
       i++;
      }
   }
   if(i%2==0 && flag==3)//right to left EVEN
   { head=compasscode();
     forward();
     gpscode();
     stla=la;
     stlo=lo;
     align(head);
     if(abs(endla-stla)<=0.000002 && abs(endlo-stlo)<=0.000002) //1m
      {
       stopp();
       dirClock();
       flag=4;
       stla=endla;
       stlo=endlo;
       endla+=.0517;
       endlo+=.2105;
       b=bear(stla,stlo,endla,endlo);
       head=compasscode();
       comeToRightAngle(b,head);
      }
   }
    if(i%2==0 && flag==4)//forward EVEN
   { head=compasscode();
     forward();
     gpscode();
     stla=la;
     stlo=lo;
     align(head);
     if(abs(endla-stla)<=0.000002 && abs(endlo-stlo)<=0.000002) //.3m
      {
       stopp();
       dirClock();
       flag=1;
       stla=endla;
       stlo=endlo;
       endla+=.05;
       endlo+=.21;
       b=bear(stla,stlo,endla,endlo);
       head=compasscode();
       comeToRightAngle(b,head);
       i++;
      }
   } 
 }
}

