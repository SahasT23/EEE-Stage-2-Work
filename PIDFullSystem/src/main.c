#include "../simulation/simulation.h"
#include <stdio.h>
#include <windows.h>

#define SIM_TIME 10.0f    // Seconds
#define DT 0.01f         // 10ms time step

int main() {
    FILE* csv = fopen("sim_data.csv", "w");
    BuggyState buggy;
    PIDController speed_pid, steer_pid;
    
    // Initialize systems
    init_simulation(&buggy);
    PID_Init(&speed_pid, 0.8f, 0.2f, 0.05f);
    PID_Init(&steer_pid, 1.2f, 0.1f, 0.2f);
    
    fprintf(csv, "Time,Position,Speed,L_Ind,R_Ind,PWM,Steering\n");
    
    for(float t=0; t<SIM_TIME; t+=DT) {
        // Calculate steering error (difference in inductance)
        float error = buggy.L_inductance - buggy.R_inductance;
        float steering = PID_Update(&steer_pid, error, DT);
        
        // Speed control (maintain 100 mm/s)
        float speed_error = 100.0f - buggy.speed;
        buggy.pwm_duty = PID_Update(&speed_pid, speed_error, DT);
        
        // Update physics
        update_buggy(&buggy, steering, DT);
        
        // Log data
        fprintf(csv, "%.2f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f\n",
                t, buggy.position, buggy.speed,
                buggy.L_inductance, buggy.R_inductance,
                buggy.pwm_duty, steering);
    }
    
    fclose(csv);
    printf("Simulation complete. Data saved to sim_data.csv\n");
    
    // Automatically plot results
    system("python ../scripts/plot_results.py");
    return 0;
}