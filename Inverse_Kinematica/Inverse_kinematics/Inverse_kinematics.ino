

#define pi 3.14
int linearActuator1[4]={37,39,9,A0};                 //pin1,pin2,pwm,potPin
int linearActuator2[4]={41,43,10,A1};
int initialPotVal[2],finalPotVal[2];    //velocitcoordinates[1],potValues
double velocity[2];
double actuatorLength[2],coordinates[2],theta,fi;    //0-top ,1-bottom,0-  coordinates[0]coordinate,1- coordinates[1]coordinate
double actuatorLimits[4];
double l[10];  //link lengths
double triangleParameters[6]={};
double fixedangles[7];
double factor=0.017453;
void setup()
{
 startPins();
 initialisefixedparameters();
 initialisecoordinates();
 Serial.begin(9600);
}

void loop()
{
 
 char input;   //static
 
 if(Serial.available()>0)
 {
   input=Serial.read();
  switch(input)
  {
    case 'w':coordinates[0]=coordinates[0]+10; break; //X++
    case 's':coordinates[0]=coordinates[0]-10; break;
      case 'i':coordinates[1]=coordinates[1]+10; break; //Y++
    case 'j':coordinates[1]=coordinates[1]-10; break;
      case 'e':stopactuators();
  }
  input='k';
   maths1();
  maths2();
 } 
  initialPotVal[0]=analogRead(linearActuator1[3]);
  initialPotVal[1]=analogRead(linearActuator2[3]);
  
  finalPotVal[0]=map(actuatorLength[0],actuatorLimits[0],actuatorLimits[1],0,1023);
  finalPotVal[1]=map(actuatorLength[1],actuatorLimits[2],actuatorLimits[3],0,1023);
  Serial.print("ACi 1: ");
  Serial.println(initialPotVal[0]);
  Serial.print("ACi 2: ");
  Serial.println(initialPotVal[1]);  

  Serial.print("X : ");
  Serial.println(coordinates[0]);
  Serial.print("Y : ");
  Serial.println(coordinates[1]);

  Serial.print("theta : ");
  Serial.println(theta/factor);
  Serial.print("fi : ");
  Serial.println(fi/factor);
  
  Serial.print("ALPHA : ");
  Serial.println(actuatorLength[0]);
  Serial.print("BETA : ");
  Serial.println(actuatorLength[1]);
  
   
  Serial.print("ACf 1: ");
  Serial.println(finalPotVal[0]);
  Serial.print("ACf 2: ");
  Serial.println(finalPotVal[1]);
  setvelocity();
  attainconfiguration();
 
}

void initialisefixedparameters()
{
 l[1]=515;    l[2]=330;   l[3]=342;
              l[5]=150;   l[6]=327.112;
 l[7]=80.316; l[8]=344.33;  l[9]=20;

 triangleParameters[0]=50;   //a
 triangleParameters[1]=70.7; //b
 triangleParameters[2]=50;   //c
 triangleParameters[3]=43.53*factor;//A
 triangleParameters[4]=90*factor;   //B
 triangleParameters[5]=46.44*factor;

 actuatorLimits[0]=242;
 actuatorLimits[1]=342;
 actuatorLimits[2]=342;
 actuatorLimits[3]=452;
 infixangles();
 
 fixedangles[2]=(21.66*factor);//R    
 fixedangles[3]=(45*factor);   //F1
 fixedangles[4]=43.53*factor;
 fixedangles[5]=90*factor;;
 fixedangles[6]=46.44*factor;;
 //4,5,6 are ABC only
}


void initialisecoordinates()
{
 
 theta=(60*factor);
 fi=(89*factor);
 maths2();
 
 initialPotVal[0]=analogRead(linearActuator1[3]);
 initialPotVal[1]=analogRead(linearActuator2[3]);
 finalPotVal[0]=map(actuatorLength[0],actuatorLimits[0],actuatorLimits[1],0,1023);
 finalPotVal[1]=map(actuatorLength[1],actuatorLimits[2],actuatorLimits[3],0,1023);
 
 setvelocity();
 attainconfiguration();

 
 coordinates[0]=((l[1]*cos(theta))+(l[2]*cos((theta-fi))));
 coordinates[1]=((l[1]*sin(theta))+(l[2]*sin((theta-fi))));

 //coordinates[0]=((l[1]*cos(theta))*l[2]*cos((theta-fi)));
 //coordinates[1]=((l[1]*sin(theta))*l[2]*sin((theta-fi)));
 
}

void infixangles()
{
 fixedangles[1]=acos(l[3]/l[8]); //P
 fixedangles[0]=atan(l[9]/l[6]); //Z  
}

void moveforward(int actuator)
{
  (actuator==1) ? (digitalWrite(linearActuator1[0],1), digitalWrite(linearActuator1[1],0)) : (digitalWrite(linearActuator2[0],1) ,digitalWrite(linearActuator2[1],0));
}


void movebackward(int actuator){
  
  (actuator==1) ? ( digitalWrite(linearActuator1[0],0), digitalWrite(linearActuator1[1],1)) : (digitalWrite(linearActuator2[0],0), digitalWrite(linearActuator2[1],1));
}



double mod(double value){
  
  if(value<=0)
    value=(-1)*(value);
 
 return value; 
}





void maths1(){
  
 fi=acos((pow(coordinates[0],2)+pow(coordinates[1],2)-pow(l[1],2)-pow(l[2],2))/(2*(l[1])*(l[2])));
 theta=atan(((coordinates[1])*(l[1]+(l[2]*cos(fi)))+(coordinates[0]*(l[2])*(sin(fi))) )/( (coordinates[0])*(l[1]+((l[2])*(cos(fi))))-(coordinates[1]*(l[2])*(sin(fi)))));
}


void maths2(){
  //alpha
  
  actuatorLength[0]=pow( (pow(l[6],2) + pow(l[7],2) - 2*(l[6])*(l[7])*cos(fixedangles[2]+theta-fixedangles[0])  ),0.5);   
  Serial.println(" //////////////////////////////////MATHS//////////////////////");
  Serial.print("Alpha : ");
  Serial.println(actuatorLength[0]);
  double Q=(pi-triangleParameters[3]-theta-triangleParameters[4]-fixedangles[3]+fi);
  Serial.print("Q : ");
  Serial.println(Q/factor);
//  Serial.println("######investigating Q###############");
//  Serial.print("pi : ");
//  Serial.println(pi);
//  Serial.print("A : ");
//  Serial.println(triangleParameters[3]/factor);
//  Serial.print("theta : ");
//  Serial.println(theta/factor);
//  Serial.print("B : ");
//  Serial.println(triangleParameters[4]/factor);
//  Serial.print("F1 : ");
//  Serial.println(fixedangles[3]/factor);
//  Serial.print("fi : ");
//  Serial.println(fi/factor);
//  Serial.println("###########################################");
  double lambda=pow((pow(triangleParameters[1],2)+pow(l[5],2)-2*(triangleParameters[1])*(l[5])*cos(Q)),0.5);
  Serial.print("Lambda : ");
  Serial.println(lambda);
  double delta=acos((pow(triangleParameters[1],2)+pow(lambda,2)-pow(l[5],2))/(2*(triangleParameters[1])*(lambda)));
  Serial.print("Delta : ");
  Serial.println(delta/factor);
  double gama=(pi-delta-fixedangles[1]+theta+fixedangles[3]-fixedangles[6]);
  Serial.println("---------------------------------");
  Serial.print("P : ");
  Serial.println(fixedangles[1]/factor);
  Serial.print("F1 : ");
  Serial.println(fixedangles[3]/factor);
  Serial.print("C : ");
  Serial.println(fixedangles[6]/factor);
  Serial.print("Gama : ");
  Serial.println("--------------------------------");
  Serial.println(gama/factor);
  //beta
  actuatorLength[1]=pow((pow(lambda,2)+pow(l[8],2))-2*(lambda)*(l[8])*cos(gama),0.5);
  Serial.print("Beta : ");
  Serial.println(actuatorLength[1]);
  Serial.println("///////////////////MATHS OVER/////////////////////////////");
} 

void stopactuators(){
  
  for (int i=0;i<2;i++){
   digitalWrite(linearActuator1[i],LOW);
   digitalWrite(linearActuator2[i],LOW);}
}

void startPins(){  //function to intialise the pins
 
  for(int i=0;i<3;i++)
  {
      pinMode(linearActuator1[i],OUTPUT);
      pinMode(linearActuator2[i],OUTPUT);
  }
  pinMode(linearActuator1[3],INPUT);
  pinMode(linearActuator2[3],INPUT);
}

void setvelocity(){

 if(mod(Decision(0))>=mod(Decision(1))){ 
     velocity[0]=150; 
     velocity[1]=150*(double)(mod(Decision(1)))/(mod(Decision(0))+mod(Decision(1)));
     }
 
 else{ 
    velocity[1]=150; 
    velocity[0]=150*(double)(mod(Decision(0)))/( mod(Decision(1))+mod(Decision(0))); 
    }
   
 Serial.println("__________________________________");
 Serial.print("mod(f0-i0) : ");
 Serial.println(mod(Decision(0)));
 Serial.print("mod(f1-i1 : ");
 Serial.println(mod(Decision(1)));
 Serial.print("vel1 : ");
 Serial.println(velocity[0]);
 Serial.print("vel2 : ");
 Serial.println(velocity[1]);
// Serial.println("__________________________________");
 analogWrite(linearActuator1[2],velocity[0]); 
 analogWrite(linearActuator2[2],velocity[1]); 
}

void attainconfiguration()
{
  if(finalPotVal[0] >0 && finalPotVal[1]>0){ 

       if((mod(Decision(0))<10) || (mod(Decision(1))<10)){
         stopactuators();
         }
   else{
        
        if(Decision(0) >0  && Decision(1)>0){
         
         Serial.println("FORWARD");
         moveforward(1); //actuator1 forward....
         moveforward(2);
         }
        
        else if(Decision(0) >0  && Decision(1) <0){
          
          Serial.println("FORWARD BACKWARD");
          moveforward(1);
          movebackward(2);
          }
        
        else if(Decision(0) <0  && Decision(1) >0){
          
          Serial.println("BACKWARD FORWARD");
          movebackward(1);
          moveforward(2);
          }
        
        else if( Decision(0)< 0  && Decision(1) <0){
          
         Serial.println("BACKWARD BACKWARD");
         movebackward(1);
         movebackward(2);
         }
      }
  }
   
  else{ 
     stopactuators();
     Serial.println("STOP");
      }
  Serial.println("__________________________________");
}


double Decision(int a){
  return (finalPotVal[a] - initialPotVal[a]);
}
