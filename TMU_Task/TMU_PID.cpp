#include <iostream>
#include <vector>
#include <cmath>
const float pi = 3.14159;

using namespace std;

float TranslationalPID(float err_P, float err_I, float err_D){

    //Controller Parameters
    float kp = 0.5;
    float ki = 0.01;
    float kd = 0.1;

    float velocity = kp*err_P + ki*err_I + kd*err_D;

    return velocity;
}

float RotationalPID(float err_P, float err_I, float err_D){

    //Controller Parameters
    float kp = 1;
    float ki = 0.05;
    float kd = 0.01;

    float velocity = kp*err_P + ki*err_I + kd*err_D;

    return velocity;
}

int main(){

    //Time Parameters
    float dt = 0.1;             // (s)
    int t_max = 10;             // (s)
    float turn_time = 0.6;      // (s) // proportion between 0 and 1 to adjust when car turn happens

    //Car Initial Conditions
    float x_car = 5;            // (m)
    float y_car = 15;           // (m)
    float vx_car = 5;           // (m/s)
    float vy_car = -1;          // (m/s)
    float vx_car_2 = 5;         // (m/s)
    float vy_car_2 = 2;        // (m/s)

    //Drone Initial Conditions
    float x_drone = 0;          // (m)
    float y_drone = 0;          // (m)
    float theta_drone = pi/2;   // (rad)

    //Other initializations
    float v_forward;            // (m/s)
    float v_theta;              // (rad/s)
    
    float P_err_translation;
    float I_err_translation = 0;
    float D_err_translation;
    float prev_err_translation = 0;

    float P_err_rotation;
    float I_err_rotation = 0;
    float D_err_rotation;
    float prev_err_rotation = 0;

    float xy_dist;              // (m)
    float theta_dist;           // (rad)

    vector<float> x_car_array;
    vector<float> y_car_array;
    vector<float> x_drone_array;
    vector<float> y_drone_array;
    vector<float> t_array;


    //Loop each timestep
    for(float t=0; t<t_max; t+=dt){

        //Calculate xy distance and angle
        theta_dist = atan((y_car-y_drone)/(x_car-x_drone)) - theta_drone;
        theta_dist = fmod(theta_dist + pi, 2*pi) - pi;                      //circle minus operator to never turn long way
        xy_dist = sqrt(pow(x_car-x_drone, 2) + pow(y_car-y_drone, 2));

        //Calculate rotational error
        P_err_rotation = theta_dist;
        I_err_rotation += P_err_rotation*dt;
        D_err_rotation = (P_err_rotation - prev_err_rotation)/dt;

        //Calculate translational error
        P_err_translation = xy_dist;
        I_err_translation += P_err_translation*dt;
        D_err_translation = (P_err_translation - prev_err_translation)/dt;

        //Calculate control velocities
        v_theta = RotationalPID(P_err_rotation, I_err_rotation, D_err_rotation);
        v_forward = TranslationalPID(P_err_translation, I_err_translation, D_err_translation);
       
        //Update drone pose for next loop
        theta_drone += v_theta*dt;
        x_drone += v_forward*cos(theta_drone)*dt;
        y_drone += v_forward*sin(theta_drone)*dt;

        //Append to all arrays for recording
        x_car_array.push_back(x_car);
        y_car_array.push_back(y_car);
        x_drone_array.push_back(x_drone);
        y_drone_array.push_back(y_drone);
        t_array.push_back(t);

        //update car pose for next loop, turning to second segment
        if(t<t_max*turn_time){         
            x_car += vx_car*dt;
            y_car += vy_car*dt;
        }
        else{
            x_car += vx_car_2*dt;
            y_car += vy_car_2*dt;
        }


    }

    //Print out arrays
    cout << "\n" << "t"<< endl;
    for(int i=0; i<t_array.size(); i++){
        cout << t_array[i] << endl;
    }
    cout << "\n" << "x_car"<< endl;
    for(int i=0; i<x_car_array.size(); i++){
        cout << x_car_array[i] << endl;
    }

    cout << "\n" << "y_car"<< endl;
    for(int i=0; i<y_car_array.size(); i++){
        cout << y_car_array[i] << endl;
    }
    cout << "\n" << "x_drone"<< endl;
    for(int i=0; i<x_drone_array.size(); i++){
        cout << x_drone_array[i] << endl;
    }
    cout << "\n" << "y_drone"<< endl;
    for(int i=0; i<y_drone_array.size(); i++){
        cout << y_drone_array[i] << endl;
    }
    
    cout << "------------ PID Control complete --------------" <<endl;

    return 0;
}

