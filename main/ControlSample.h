#include <BasicLinearAlgebra.h>
#include <Arduino.h>

using namespace BLA;

struct ControlState {
  float K;          
  float L;          
  float r;          
  BLA::Matrix<4, 1> x_hat;      
  BLA::Matrix<3, 1> w_hat;      
  BLA::Matrix<2, 1> y_alpha;    
  BLA::Matrix<2, 1> y_alpha_hat; 
  unsigned long timestamp;       
};


struct ControlSystem {
  BLA::Matrix<4, 1> x_hat; 
  BLA::Matrix<4, 1> x; 
  BLA::Matrix<4, 1> y_hat; 
  BLA::Matrix<2, 1> u;
};



ControlSystem myControl;
ControlState myStorage;  


BLA::Matrix<4, 4> A_1 = {1, 1, 1, 1,  
                         1, 1, 1, 1,
                         1, 1, 1, 1,
                         1, 1, 1, 1}; 

BLA::Matrix<4, 1> B_1 = {1, 1, 1, 1};

BLA::Matrix<4, 3> C_1 = {1, 1, 1,
                         1, 1, 1,
                         1, 1, 1,
                         1, 1, 1};

BLA::Matrix<1, 4> omega_1 = {1, 1, 1, 1};
BLA::Matrix<1, 4> omega_2 = {1, 1, 1, 1};

BLA::Matrix<3, 3> A_w = {1, 1, 1,
                         1, 1, 1,
                         1, 1, 1};

BLA::Matrix<3, 2> B_w = {1, 1,
                         1, 1,
                         1, 1}; 

BLA::Matrix<2, 3> c_1 = {1, 1, 1,
                         1, 1, 1};
BLA::Matrix<2, 3> c_2 = {1, 1, 1,
                         1, 1, 1};
BLA::Matrix<3, 4> c_3 = {1, 1, 1, 1,
                         1, 1, 1, 1,
                         1, 1, 1, 1};
BLA::Matrix<3, 4> c_4 = {1, 1, 1, 1,
                         1, 1, 1, 1,
                         1, 1, 1, 1};
                         
BLA::Matrix<3, 1> b_1 = {1, 1, 1};
BLA::Matrix<3, 1> b_2 = {1, 1, 1};

BLA::Matrix<4, 3> P = {1, 1, 1,
                       1, 1, 1, 
                       1, 1, 1,
                       1, 1, 1};

BLA::Matrix<4, 2> Q = {1, 1,
                       1, 1,
                       1, 1,
                       1, 1};


BLA::Matrix<3, 1> w_hat;       
BLA::Matrix<3, 1> w_hat_dot;
BLA::Matrix<2, 1> e_w;
BLA::Matrix<3, 1> y_g;
BLA::Matrix<3, 1> y_a;
BLA::Matrix<2, 1> y_alpha;
BLA::Matrix<2, 1> y_alpha_hat; 

const float dt = 0.01; 

void updateControlState();
void calculateControlParameters();
void saveControlState();
void printStoredState();


void setup() {
  Serial.begin(115200);
  delay(1000); 
  
  Serial.println("=== Control State Storage System ===");
  
  myControl.x.Fill(0);
  myControl.u.Fill(0);
  myControl.x_hat.Fill(0);
  myControl.y_hat.Fill(0);
  
  w_hat.Fill(0);
  y_alpha.Fill(0);
  y_alpha_hat.Fill(0);
  
  myStorage.K = 0.0;
  myStorage.L = 0.0;
  myStorage.r = 0.0;
  myStorage.x_hat.Fill(0);
  myStorage.w_hat.Fill(0);
  myStorage.y_alpha.Fill(0);
  myStorage.y_alpha_hat.Fill(0);
  myStorage.timestamp = 0;
  
  Serial.println("Initialized!");
  Serial.println();
}


void loop() {
  updateControlState();
  
  calculateControlParameters();
  
  saveControlState();

  static int printCounter = 0;
  if (++printCounter >= 100) {  
    printStoredState();
    printCounter = 0;
  }
  
  delay(10); 
}

void updateControlState() {
  y_g = c_3 * myControl.x + b_1;
  y_a = c_4 * myControl.x + b_2;
  
  e_w = c_1 * y_g + c_2 * y_a;

  w_hat_dot = A_w * w_hat + B_w * e_w;
  w_hat = w_hat + w_hat_dot * dt;
  
  myControl.y_hat = C_1 * w_hat;
  
  myControl.x_hat = P * w_hat + Q * e_w;

  y_alpha(0) = (omega_1 * myControl.x)(0); 
  y_alpha(1) = (omega_2 * myControl.x)(0);

  y_alpha_hat(0) = (omega_1 * myControl.x_hat)(0);
  y_alpha_hat(1) = (omega_2 * myControl.x_hat)(0);
}


void calculateControlParameters() {
  myStorage.K = 0.5 * w_hat(0) + 0.3 * w_hat(1);
  myStorage.L = 0.4 * w_hat(1) + 0.2 * w_hat(2);
  myStorage.r = 1.0;
  
  myControl.u(0) = myStorage.K;
  myControl.u(1) = myStorage.L;
}


void saveControlState() {

  myStorage.x_hat = myControl.x_hat;
  
  myStorage.w_hat = w_hat;
  
  myStorage.y_alpha = y_alpha;
  myStorage.y_alpha_hat = y_alpha_hat;
  
  myStorage.timestamp = millis();
}

//Print the stored ControlState

void printStoredState() {
  Serial.println("========== Stored Control State ==========");
  Serial.print("Timestamp: "); 
  Serial.print(myStorage.timestamp); 
  Serial.println(" ms\n");
  

  Serial.println("--- Control Parameters ---");
  Serial.print("K = "); Serial.println(myStorage.K, 4);
  Serial.print("L = "); Serial.println(myStorage.L, 4);
  Serial.print("r = "); Serial.println(myStorage.r, 4);
  Serial.println();
  

  Serial.println("--- State Estimate x_hat (4x1) ---");
  for (int i = 0; i < 4; i++) {
    
    Serial.print("x_hat("); Serial.print(i); Serial.print(") = ");
    Serial.println(myStorage.x_hat(i), 4);
  }
  Serial.println();

  Serial.println("--- Observer State w_hat (3x1) ---");
  for (int i = 0; i < 3; i++) {
    Serial.print("w_hat("); Serial.print(i); Serial.print(") = ");
    Serial.println(myStorage.w_hat(i), 4);
  }
  Serial.println();

  Serial.println("--- Output y_alpha (2x1) ---");
  Serial.print("y_alpha(0) = "); Serial.println(myStorage.y_alpha(0), 4);
  Serial.print("y_alpha(1) = "); Serial.println(myStorage.y_alpha(1), 4);
  Serial.println();
  
  Serial.println("--- Output Estimate y_alpha_hat (2x1) ---");
  Serial.print("y_alpha_hat(0) = "); Serial.println(myStorage.y_alpha_hat(0), 4);
  Serial.print("y_alpha_hat(1) = "); Serial.println(myStorage.y_alpha_hat(1), 4);
  Serial.println();
  
  Serial.println("==========================================\n");
}