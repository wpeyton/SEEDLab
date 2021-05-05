float theta;   //radians
float thetaDesired; //radians
float Va;      //volts
int T = 5;     //sample time in ms
float totalError;
float prevTime = 0;
float Kp = 26.8;    //proportional gain
float Ki = 5.7;     //integral gain

void setup(){ 

}

void loop(){
  PI_Controller();
}

void PI_Controller(){

  float currentTime = millis(); //returns ms
  int deltaTime = currentTime - prevTime; //time interval 

  if (deltaTime >= T){

    float error = thetaDesired - theta;
    totalError += error;

    Va = Kp*error + (Ki*T)*totalError; //PI controller

    prevTime = currentTime;
  } 
}
