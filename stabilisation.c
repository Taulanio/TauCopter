#include <AP_Common.h>

#include <AP_Math.h>

#include <AP_Param.h>

#include <AP_Progmem.h>

#include <AP_ADC.h>

#include <AP_InertialSensor.h>

#include <AP_HAL.h>

#include <AP_HAL_AVR.h>

#include <PID.h>
// ���������� ��� Arduino ��� ������ � ��������� � ��������������� ���������� ��� ����������


const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;

// ������������� ��������� � �������������

AP_InertialSensor_MPU6000 ins;

// ������ �������

#define MOTOR_FL 2 // �������� ����� 

#define MOTOR_FR 0 // �������� ������

#define MOTOR_BL 1 // ������ �����

#define MOTOR_BR 3 // ������ ������



long map(long x, long in_min, long in_max, long out_min, long out_max)

{

 return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

}

#define wrap_180(x) (x < -180 ? x+360 : (x > 180 ? x - 360: x))

// ������ �������� ���-����������

PID pids[6];

#define PID_PITCH_RATE 0

#define PID_ROLL_RATE 1

#define PID_PITCH_STAB 2

#define PID_ROLL_STAB 3

#define PID_YAW_RATE 4

#define PID_YAW_STAB 5

void setup() 

{

 // ����������� ������� � ����� ���-������� � �������� 490 ���

 hal.rcout->set_freq(0xF, 490);

 hal.rcout->enable_mask(0xFF);

 // ������������ ���

 pids[PID_PITCH_RATE].kP(0.7);

 pids[PID_PITCH_RATE].kI(1);

 pids[PID_PITCH_RATE].imax(50);

 pids[PID_ROLL_RATE].kP(0.7);

 pids[PID_ROLL_RATE].kI(1);

 pids[PID_ROLL_RATE].imax(50);

 pids[PID_YAW_RATE].kP(2.7);

 pids[PID_YAW_RATE].kI(1);

 pids[PID_YAW_RATE].imax(50);

 pids[PID_PITCH_STAB].kP(4.5);

 pids[PID_ROLL_STAB].kP(4.5);

 pids[PID_YAW_STAB].kP(10);



 hal.gpio->pinMode(40, GPIO_OUTPUT);

 hal.gpio->write(40, 1);

 

 // ���������� ��������� MPU6050

 ins.init(AP_InertialSensor::COLD_START, 

 NULL);


 hal.scheduler->suspend_timer_procs(); 

 ins.dmp_init();

 hal.scheduler->resume_timer_procs();

 hal.uartA->begin(57600); // ��� �����, �� ����������������� �����

 

}

AP_InertialSensor::RATE_100HZ,

// ������ ��� ���������������� �����

char buf[255];

int buf_offset = 0;

//�������� �������

uint8_t verify_chksum(char *str, char *chk) {

 uint8_t nc = 0;

 for(int i=0; i<strlen(str); i++) 

 nc = (nc + str[i]) << 1;

 

 long chkl = strtol(chk, NULL, 16); 

 if(chkl == (long)nc) 

 return true;

 return false;

}

void loop() 

{

 static float yaw_target = 0; 

 // ������ 5���� ������������ ����� ������ ����������

 while (ins.num_samples_available() == 0);

 

 static uint32_t lastPkt = 0;

 static int16_t channels[8] = {0,0,0,0,0,0,0,0};

 

 int bytesAvail = hal.console->available();

 if(bytesAvail > 0) {

 while(bytesAvail > 0) { 

 char c = (char)hal.console->read(); 

 if(c == '\n') {  

 buf[buf_offset] = '\0'; 

 

 char *str = strtok(buf, "*"); // � ������ ������������ �������� ����� �����, �������, ��������

 char *chk = strtok(NULL, "*"); // ���������� ��������

 

 if(verify_chksum(str,chk)) { 

 char *ch = strtok(str, ","); 

 channels[0] = (uint16_t)strtol(ch, NULL, 10); // ������� ��������

 for(int i=1; i<4; i++) { 

 char *ch = strtok(NULL, ",");

 channels[i] = (uint16_t)strtol(ch, NULL, 10); 

 }

 

 lastPkt = hal.scheduler->millis(); 

 } else {

 hal.console->printf("Invalid chksum\n");

 }

 

 buf_offset = 0;

 }

 else if(c != '\r') {

 buf[buf_offset++] = c; 

 }

 bytesAvail --;

 }

 }



 if(hal.scheduler->millis() - lastPkt > 500) 

 channels[2] = 0;

 long rcthr, rcyaw, rcpit, rcroll; 

 // ������ �������� ������� � ����������� 

 rcthr = channels[2];

 rcyaw = channels[3];

 rcpit = channels[1];

 rcroll = channels[0];

 

 // ����� MPU6050 �� �������� ���������� � ������������

 ins.update();

 float roll,pitch,yaw; 

 ins.quaternion.to_euler(&roll, &pitch, &yaw);
 
 // �������� ������������ ����� �����, ������� � �������� ��������� � �������

 roll = ToDeg(roll) ;

 pitch = ToDeg(pitch) ;

 yaw = ToDeg(yaw) ;

 

 // ����� MPU6050 �� �������� ���������

 Vector3f gyro = ins.get_gyro();

 float gyroPitch = ToDeg(gyro.y), gyroRoll = ToDeg(gyro.x), gyroYaw = 

ToDeg(gyro.z);

 

 

 if(rcthr > RC_THR_MIN + 100) { 

 // ������������ �� ��� ���������

 float pitch_stab_output = 

constrain(pids[PID_PITCH_STAB].get_pid((float)rcpit - pitch, 1), -250, 250); 

 float roll_stab_output = constrain(pids[PID_ROLL_STAB].get_pid((float)rcroll 

- roll, 1), -250, 250);

 float yaw_stab_output = 

constrain(pids[PID_YAW_STAB].get_pid(wrap_180(yaw_target - yaw), 1), -360, 360);

 

 

(overwriting yaw stab output)

 if(abs(rcyaw ) > 5) {

 yaw_stab_output = rcyaw;

 yaw_target = yaw; // ���������� �������� ���� �������� 

 }

 

 // �������� ���

 long pitch_output = (long) 

constrain(pids[PID_PITCH_RATE].get_pid(pitch_stab_output - gyroPitch, 1), - 500, 

500); 

 long roll_output = (long) 

constrain(pids[PID_ROLL_RATE].get_pid(roll_stab_output - gyroRoll, 1), -500, 

500); 

 long yaw_output = (long) 

constrain(pids[PID_ROLL_RATE].get_pid(yaw_stab_output - gyroYaw, 1), -500, 500); 

 // ����������� �������� ���-���������� �� ���������

 hal.rcout->write(MOTOR_FL, rcthr + roll_output + pitch_output - yaw_output);

 hal.rcout->write(MOTOR_BL, rcthr + roll_output - pitch_output + yaw_output);

 hal.rcout->write(MOTOR_FR, rcthr - roll_output + pitch_output + yaw_output);

 hal.rcout->write(MOTOR_BR, rcthr - roll_output - pitch_output - yaw_output);

 } else {

 // ����������� ������. ����������� �������� ��� ������� 1000����

 hal.rcout->write(MOTOR_FL, 1000);

 hal.rcout->write(MOTOR_BL, 1000);

 hal.rcout->write(MOTOR_FR, 1000);

 hal.rcout->write(MOTOR_BR, 1000);

 



 yaw_target = yaw;

 

 // ����� ������������ �������� ��� ��� ���������� �� �����

 for(int i=0; i<6; i++)

 pids[i].reset_I();

 }

}

AP_HAL_MAIN();

