// Final Proj Code By KMG

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Bool.h"
#include "math.h"
#include <ctime>


#define FRONT 0 
#define MID 29
#define LEFT 90
#define BACK 180
#define RIGHT 270

#define INIT_DIS 0.3 // 초기화 할 때 사용하는 거리
#define TDIS 0.27 // Front의 거리와 비교하여 앞으로 갈 수 있는지 판단하는 값 
#define TURN_TDIS 0.32 // 막혀있는 곳이 나왔는지 판단하는 거리 값
#define RANGE_CHECK 0.22 // 로봇이 주변에 사물이 있는지 판단하는 거리 값


float distance[360]; // 모든 방향의 거리를 저장하는 배열
float q0, q1, q2, q3, ang_yaw;
float max_distance;

int max_index, size;
float starting_ang, ending_ang,max;

int cur_ang; // 현재 각도를 저장하는 변수
bool block_flag = false; // 현재 올바른 방향이 아닌 reverse 방향인 것에 대한 상태 유무를 판단하는 값
bool INIT = false;
int right_range; // 왼쪽 최대 범위를 저장하는 변수
int left_range; // 오른쪽 최대 범위를 저장하는 변수

// bool turnaround = false;

void laser_scan_Callback(const sensor_msgs::LaserScan::ConstPtr &msg)
{

    for (int i = 0; i < 360; i++)
    {
        // Handling exception for distances
        if ((msg->ranges[i]) <= 0)
        {
            distance[i] = 5;
        }
        else
        {
            distance[i] = msg->ranges[i];
        }
    }
}

bool is_turnaround(){ // turnaround를 해야 하는 상황인지 판단하는 함수
        int temp_dis_r = 0; // 정면 기준 오른쪽 90도의 사물이 있는지 판단
        int temp_dis_l = 0; // 정면 기준 왼쪽으로 90도의 사물이 있는지 판단
        for(int i = 0;i < LEFT ;i++){
        if(i < 15){
            temp_dis_r += distance[i]/15;
            if(temp_dis_r > 0.45){
                return false;
            }
        }
        else{
            if(distance[i] > TURN_TDIS)
            {
                return false;
            }
        }
    }
    for(int i = RIGHT; i < 360;i++){
        if(i >= 345){
            temp_dis_l += distance[i]/15;
            if(temp_dis_l > 0.45){
                return false;
            }
        }
        else{
            if(distance[i] > TURN_TDIS)
            {
                return false;
            }
        }
    }
    return true;
}

bool is_safe(){ // 현재 정면 기준 0-MID ~ 0+MID에 사물이 있는지 판단
    bool f = true; 
    for(int i = 0;i < MID;i++){ // 0 ~ 0 + MID 까지 사물이 있는지 판단하는 부분
        if(distance[i] < RANGE_CHECK)
        {
            return false;
        }
    }
    for(int i = 360-MID; i < 360;i++){ // 360 -MID ~  360 까지 사물이 있는지 판단하는 부분
        if(distance[i] < RANGE_CHECK)
        {
            return false;
        }
    }
    return f;
}

int get_min_dis(){ // 정면 기준 오른쪽 90 과 왼쪽 90의 거리의 평균을 비교하여 방향을 정함 
    float min = 5.1;
    int min_index = -1;
    for(int i = 0;i < MID ;i++){
        if(min > distance[i]){
            min = distance[i];
            min_index = i;
        }
    }
    for(int i = 360-MID; i < 360;i++){
        if(min > distance[i]){
            min = distance[i];
            min_index = i;
        }
    }
    if(min_index < MID)
        return 1; // left
    else
        return 0; // right
}


void debug(){
    printf("\n--------------------------------------\n");
    printf("[INFO FRONT     ] :: %f\n", distance[FRONT]);
    printf("[INFO RIGHT     ] :: %f\n", distance[RIGHT]);
    printf("[INFO LEFT      ] :: %f\n", distance[LEFT]);
    printf("[INFO RANGE     ] :: %d ~ %d \n", left_range, right_range);
    printf("[INFO CUR ANG   ] :: %d\n", cur_ang);
    printf("[INFO BLOCK FLAG]  :: %d \n", block_flag ? 1 : 0);
    printf("--------------------------------------\n");
}

void imu_Callback(const sensor_msgs::Imu::ConstPtr &imu_msg)
{
    q0 = imu_msg->orientation.x;
    q1 = imu_msg->orientation.y;
    q2 = imu_msg->orientation.z;
    q3 = imu_msg->orientation.w;
    ang_yaw = atan2(2 * ((q0 * q1) + (q2 * q3)), 1 - 2 * ((q1 * q1) + (q2 * q2))) * 180 / M_PI;
}

bool is_reverse_turn(){ // 진행 반대 방향일 때, 현재 오른쪽이나 왼쪽으로 회전 가능 유무를 판단하는 함수
    bool l_f = true;
    bool r_f = true;
    for(int i = LEFT-MID;i < LEFT+MID ;i++){ // 왼쪽 부분에 대한 각도를 판단하는 함수, TURN_TDIS보다 적은 거리가 있으면 회전 불가
        if(distance[i] < TURN_TDIS)
        {
            l_f = false;
            break;
        }
    }
    for(int i = RIGHT - MID; i < RIGHT+MID;i++){ // 오른쪽 부분에 대한 각도를 판단하는 함수, TURN_TDIS보다 적은 거리가 있으면 회전 불가
        if(distance[i] < TURN_TDIS)
        {
            r_f = false;
            break;
        }
    }
    
    return l_f || r_f; // 둘 중 하나라도 True가 있다면 회전 가능
}

int get_average(){ // 오른쪽과 왼쪽 90도 거리를 더하여 더 열려있는 곳을 판단하는 함수
    /*
        1 : left
        2 : right
    */
    float left_dis= 0;
    float right_dis = 0;
    for(int i=0;i<90;i++){
        if(distance[i] == 5) // 거리 측정 불가인 곳은 배제
            continue;
        left_dis += distance[i]; // 왼쪽 90도 거리들의 총합
    }
    for(int i =270;i<360;i++){
        if(distance[i] == 5) // 거리 측정 불가인 곳은 배제
            continue;
        right_dis += distance[i]; // 오른쪽 90도 거리들의 총합
    }
    int res = (left_dis > right_dis) ? 1 : 2; // 합이 더 높은 곳에 대해서 1과 2로 판단하여 반환
    return res;

}


int get_direction(int left, int right){ // 현재 어느 방향인지를 반환하는 함수
    if(left == 90 &&  right == 270){ // front
        return 0; // both
    }
    else if(left == 45 && right == 225){ // left-front
        return 1; // left
    }
    else if(left == 0 && right == 180){ // left
        return 2; // right
    }
    else if(left == 135 && right == 315){ // right-front
        return 3; 
    }
    else if(left == 180 &&  right == 0){ // right
        return 4; 
    }
    else if(left == 315 && right == 135){ // left-back
        return 5; 
    }
    else if(left == 225 && right == 45){ // right-back
        return 6; 
    }
    else if(left == 270 && right == 90){ // turn around
        return 7; 
    }
}


void turn_around(geometry_msgs::Twist &kmg_velocity,ros::Publisher &final_velocity) { // 180도 회전을 하는 함수
    printf("[INFO TURN AROUND]\n");
    clock_t start = clock();
    float tmp_ang = 0;
    while(tmp_ang < 3.14){ // 시간으로 판단하여 180도를 돌게함
        kmg_velocity.angular.z  = -0.8; 
        kmg_velocity.linear.x = 0;
        clock_t end = clock();
        tmp_ang = 0.8 * (end-start); // 시간을 추가하는 부분
        final_velocity.publish(kmg_velocity);
        ros::Duration(4).sleep(); // 4초 정도의 sleep을 통해 로봇이 회전하도록 하는 부분
    }
}

void turn_left_90(geometry_msgs::Twist &kmg_velocity, ros::Publisher &final_velocity) { // 왼쪽으로 90도 돌게하는 함수
    printf("[INFO LEFT]\n");
    clock_t start = clock();
    float tmp_ang = 0;
    while(tmp_ang < 1.57){  // 시간으로 판단하여 90도를 돌게함
        kmg_velocity.angular.z  = 0.8;
        kmg_velocity.linear.x = 0;
        clock_t end = clock();
        tmp_ang = 0.8 * (end-start); // 시간을 추가하는 부분
        final_velocity.publish(kmg_velocity);
        ros::Duration(2).sleep(); // 2초 정도의 sleep을 통해 로봇이 회전하도록 하는 부분
    }
}

void turn_left_45(geometry_msgs::Twist &kmg_velocity, ros::Publisher &final_velocity) {// 왼쪽으로 45도 돌게하는 함수
    printf("[INFO LEFT]\n"); 
    clock_t start = clock();
    float tmp_ang = 0;
    while(tmp_ang < 0.78){ // 시간으로 판단하여 45도를 돌게함
        kmg_velocity.angular.z  = 0.8;
        kmg_velocity.linear.x = 0;
        clock_t end = clock();
        tmp_ang = 0.8 * (end-start); //시간을 추가하는 부분
        final_velocity.publish(kmg_velocity);
        ros::Duration(1).sleep(); // 1초 정도의 sleep을 통해 로봇이 회전하도록 하는 부분
    }
}

void turn_right_90(geometry_msgs::Twist &kmg_velocity, ros::Publisher &final_velocity) {// 오른쪽으로 90도 돌게하는 함수
    printf("[INFO RIGHT]\n");
    clock_t start = clock();
    float tmp_ang = 0;
    while(tmp_ang < 1.57){ // 시간으로 판단하여 90도를 돌게함
        kmg_velocity.angular.z  = -0.8;
        kmg_velocity.linear.x = 0;
        clock_t end = clock();
        tmp_ang = 0.8 * (end-start); //시간을 추가하는 부분
        final_velocity.publish(kmg_velocity);
        ros::Duration(2).sleep();  // 2초 정도의 sleep을 통해 로봇이 회전하도록 하는 부분
    }
}

void turn_right_45(geometry_msgs::Twist &kmg_velocity, ros::Publisher &final_velocity) { // 오른쪽으로 45도 돌게하는 함수
    printf("[INFO LEFT]\n");
    clock_t start = clock();
    float tmp_ang = 0;
    while(tmp_ang < 0.78){ // 시간으로 판단하여 45도를 돌게함
        kmg_velocity.angular.z  = -0.8;
        kmg_velocity.linear.x = 0;
        clock_t end = clock();
        tmp_ang = 0.8 * (end-start); //시간을 추가하는 부분
        final_velocity.publish(kmg_velocity);
        ros::Duration(1).sleep();// 1초 정도의 sleep을 통해 로봇이 회전하도록 하는 부분
    }
}


void cal_range(const int &angle){ // 현재 로봇이 정면기준으로 왼쪽 90도와 오른쪽 90도의 방향 각도를 계산하는 부분
    left_range = ((left_range + angle) + 360) % 360;
    right_range = ((right_range + angle) + 360) % 360;
}

void move(geometry_msgs::Twist &kmg_velocity, ros::Publisher &final_velocity){ // 로봇이 실제로 움직이도록 함수를 실행하는 부분
    int dir = get_average(); // 현재 왼쪽과 오른쪽 중 어떤 곳이 더 회전하기 적합한지 판단하여 저장하는 부분
    int get_dir = get_direction(left_range, right_range); // 로봇의 현재 방향을 총 8개의 방향 중 어디인지 판단하여 저장하는 부분
    if(get_dir == 0){ // 정면인 경우 : 왼쪽으로 45 혹은 오른쪽으로 45 회전 가능
        if ( dir == 1){ // 왼쪽이 더 회전하기 적합
            turn_left_45(kmg_velocity, final_velocity); // 왼쪽으로 45도 회전
            cur_ang += -45; // 현재 각도를 변경
        }
        else if (dir == 2){ // 오른쪽이 더 회전하기 적합
            turn_right_45(kmg_velocity, final_velocity); //오른쪽으로 45도 회전
            cur_ang += 45; // 현재 각도를 변경
        }
        cal_range(cur_ang); // 다시 각도를 저장하는 변수의 범위를 업데이트
    }
    else if(get_dir == 1){ // 왼쪽 45도인 경우 : 왼쪽으로 45도 혹은 오른쪽으로 45도 회전 가능
        if ( dir == 1){ // 왼쪽이 더 회전하기 적합
            turn_left_45(kmg_velocity, final_velocity); // 왼쪽으로 45도 회전
            cur_ang += -45; // 현재 각도를 변경
        }
        else if (dir == 2){ // 오른쪽이 더 회전하기 적합
            turn_right_45(kmg_velocity, final_velocity); // 오른쪽으로 45도 회전
            cur_ang += 45; // 현재 각도를 변경
        }
        cal_range(cur_ang); // 다시 각도를 저장하는 변수의 범위를 업데이트
    }
    else if(get_dir == 2){ // 왼쪽 90도인 경우 : 오른쪽으로 90도를 회전하여 정면으로 방향을 변경
        turn_right_90(kmg_velocity, final_velocity); // 오른쪽으로 90도 회전
        cur_ang += 90; // 현재 각도를 변경
        cal_range(cur_ang); // 다시 각도를 저장하는 변수의 범위를 업데이트
    }
    else if(get_dir == 3){ // 오른쪽 45도인 경우 : 왼쪽으로 45도 회전 혹은 오른쪽으로 45도 회전 가능
        if ( dir == 1){ // 왼쪽이 더 회전하기 적합
            turn_left_45(kmg_velocity, final_velocity); // 왼쪽으로 45도 회전
            cur_ang += -45; // 현재 각도 갱신
        }
        else if (dir == 2){ // 오른쪽이 더 회전하기 적합
            turn_right_45(kmg_velocity, final_velocity); // 오른쪽으로 45도 회전
            cur_ang += 45; // 현재 각도 갱신
        }
        cal_range(cur_ang);// 다시 각도를 저장하는 변수의 범위를 업데이트
    }
    else if(get_dir == 4){ // 오른쪽 90도 인 경우 : 왼쪽으로 90도를 회전하여 정면으로 방향을 변경
        turn_left_90(kmg_velocity, final_velocity); // 왼쪽 90도 호전
        cur_ang += -90; // 현재 각도 갱신
        cal_range(cur_ang); // 다시 각도를 저장하는 변수의 범위를 업데이트
    }
    else if(get_dir == 5 || get_dir == 6 || get_dir == 7) // 5,6,7은 정면 기준 뒤의 방향을 보고 있으므로 block_flag를 true로 해줌
    {
        block_flag = true;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "final_project_sub1");
    ros::NodeHandle nh;
    ros::Subscriber final_lidar = nh.subscribe("/scan", 100, laser_scan_Callback);
    ros::Subscriber final_imu = nh.subscribe("/imu",100,imu_Callback);
    ros::Publisher final_power = nh.advertise<std_msgs::Bool>("/motor_power", 100);
    ros::Publisher final_velocity = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
    sensor_msgs::LaserScan ls;
    geometry_msgs::Twist kmg_velocity;
    std_msgs::Bool kmg_power;
    ros::Rate loop_rate(10);

    float ava_dis[180] = {0,};
    float before_yaw = 0;
    bool flag = true;


    while (ros::ok())
    {
        ros::spinOnce();

        int max = 0;
        for(int i =0;i<360;i++){
            if(max < distance[i])
                max = distance[i];
        } // 초기 max 값의 오류를 해결하고자 max를 저장하는 부분

        if(max == 0) continue; //max 부분이 0인 경우는 continue

        /*
            조건
            1. 4 방향의 거리가 모두 INIT_DIS보다 작은 경우
        */
        if(distance[FRONT] < INIT_DIS && distance[RIGHT] < INIT_DIS && distance[LEFT] < INIT_DIS && distance[BACK] < INIT_DIS) // 초기화를 하는 부분
        {
            // 각 변수들을 초기화
            left_range = LEFT; // 현재 정면 기준 왼쪽 최대 범위를 90으로 초기화
            right_range = RIGHT; // 현재 정면 기준 오른쪽 최대 범위를 270으로 초기화
            INIT = true; // INIT flag를 true로 변경
            block_flag = false; 
            cur_ang = 0;

            printf("[INFO] :: INIT  \n");
            kmg_velocity.angular.z =0;
            kmg_velocity.linear.x = 0;

            final_velocity.publish(kmg_velocity);
            ros::Duration(2).sleep();
        }
        /*
            조건
            1. block_flag 가 0인 경우 -> 올바른 방향
            2. INIT이 된 경우
            3. is_turnaround() -> true :: 전방향이 막혀서 뒤를 돌아야 하는 경우
        */
        else if( !block_flag && is_turnaround() &&  INIT){ // TURN AROUND , BLOCK
            block_flag = true;
            turn_around(kmg_velocity, final_velocity); // 180도 회전 하는 함수 실행
            cur_ang += BACK; // 현재 각도를 180도 추가하여 갱신
            cal_range(cur_ang); // 범위를 업데이트
            cur_ang =0; // 업데이트 후에는 현재 각도를 다시 0으로 초기화
        }
        /*
            조건
            1. 현재 방향이 뒤를 바라보고 있는 경우
            2. INIT이 된 경우
        */
        else if((get_direction(left_range,right_range) >4 || block_flag) && INIT){ // reverse
            if(is_reverse_turn()){ // 왼쪽 혹은 오른쪽으로 회전할 수 있는지 여부 판단
                int dir = get_average(); // 왼쪽 오른쪽 범위 중 적절한 곳을 판단
                if(dir == 1){ // 왼쪽으로 회전
                    turn_left_90(kmg_velocity,final_velocity);
                    cur_ang += -90;
                }
                else if (dir ==2){ //오른쪽으로 회전
                    turn_right_90(kmg_velocity, final_velocity);
                    cur_ang += 90;
                }
                cal_range(cur_ang);
                cur_ang =0;
                block_flag = false; // block_flag를 다시 올바른 방향으로 재설정
            }
            else{ // 회전을 할 수 없는 경우 계속 직진
            // else if(is_safe()){ // reverse front
                kmg_velocity.angular.z =0;
                kmg_velocity.linear.x = 0.5;
                final_velocity.publish(kmg_velocity);
            }
        }
        else if(is_safe() && distance[FRONT] > TDIS && !block_flag && INIT){ // FRONT 
            kmg_velocity.angular.z =0;
            kmg_velocity.linear.x = 1;
            final_velocity.publish(kmg_velocity);
            
        }
        else if( ( !is_safe() || distance[FRONT] <= TDIS) && !block_flag && INIT){ // TURN 
            move(kmg_velocity, final_velocity);
            cur_ang = 0;
        }
        
        debug();
        loop_rate.sleep();
    }
}