# Line-Follower
#include "coppeliasim_client.h"  // Include our header

// Global client instance for socket communication
SocketClient client;

// ----------------------
// Forward declarations (these will move to header gradually)
// ----------------------
void* control_loop(void* arg);          // Only control_loop remains

/**
 * @brief Establishes connection to the CoppeliaSim server
 * @param c Pointer to SocketClient structure
 * @param ip IP address of the server (typically "127.0.0.1" for localhost)
 * @param port Port number of the server (typically 50002)
 * @return 1 if connection successful, 0 if failed
 */

int connect_to_server(SocketClient* c, const char* ip, int port) {
#ifdef _WIN32
    // Initialize Winsock on Windows
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
        printf("WSAStartup failed\n");
        return 0;
    }
#endif
    
    // Create TCP socket
    c->sock = socket(AF_INET, SOCK_STREAM, 0);
    if (c->sock < 0) {
        printf("Socket creation failed\n");
        return 0;
    }

    // Setup server address structure
    struct sockaddr_in serv_addr;
    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(port);
    inet_pton(AF_INET, ip, &serv_addr.sin_addr);

    // Attempt to connect to server
    if (connect(c->sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
        printf("Connection failed\n");
        CLOSESOCKET(c->sock);
#ifdef _WIN32
        WSACleanup();
#endif
        return 0;
    }

    c->running = true;

    // Start the receive thread to handle incoming sensor data
#ifdef _WIN32
    c->recv_thread = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)receive_loop, c, 0, NULL);
#else
    pthread_create(&c->recv_thread, NULL, receive_loop, c);
#endif

    return 1;
}
/*===============================================================================================*/
/**
 * @brief Main control loop thread for robot behavior
 * @param arg Pointer to SocketClient structure (cast from void*)
 * @return NULL when thread exits
 * 
 * This is where you should implement your robot's control logic.
 * The function runs continuously while the client is connected.
 * 
 * Available functions for control:
 * - set_motor(c, left_speed, right_speed): Control motor speeds
 * - Access sensor data via: c->sensor_values[index] and c->sensor_count
 */
void* control_loop(void* arg) {
    SocketClient* c = (SocketClient*)arg;

    // ---- PID constants (tune these) ----
    float Kp = 0.3f;
    float Ki = 0.0f;  // start with 0 for simplicity
    float Kd = 0.6f;

    // ---- PID state variables ----
    float error = 0, previousError = 0, integral = 0, derivative = 0;
    float PIDvalue;
    float baseSpeed = 3.5f;  // base forward speed

    // ---- Weighted sensor positions ----
    float weights[5] = { -2, -1, 0, 1, 2 };
    float sensor_threshold = 200.0f; // threshold for white line detection

    while (c->running) {
        int sensorCount = c->sensor_count;
        if (sensorCount < 5) {
            printf("Waiting for sensor data... (only %d sensors found)\n", sensorCount);
            SLEEP(100);
            continue;
        }

        // ---- Read sensor values ----
        float numerator = 0.0f, denominator = 0.0f;
        int on_line_count = 0;

        for (int i = 0; i < 5; i++) {
            float value = c->sensor_values[i]; // white line: high value
            numerator += weights[i] * value;
            denominator += value;

            if (value > sensor_threshold)
                on_line_count++;
        }

        float position = 0.0f;
        if (denominator != 0) {
            position = numerator / denominator;
        } else {
            // Lost line scenario: spin to search
            if (previousError > 0) position = -500.0f;
            else position = 500.0f;
        }

        // ---- PID computation ----
        error = 0 - position;
        integral += error;
        derivative = error - previousError;
        PIDvalue = (Kp * error) + (Ki * integral) + (Kd * derivative);
        previousError = error;

        // ---- Limit PID output ----
        if (PIDvalue > 1.5f) PIDvalue = 1.5f;
        if (PIDvalue < -1.5f) PIDvalue = -1.5f;

        // ---- Differential drive control ----
        float leftSpeed = baseSpeed - PIDvalue;
        float rightSpeed = baseSpeed + PIDvalue;

        // ---- Lost line recovery ----
        if (on_line_count == 0) {
            if (previousError > 0) {
                leftSpeed = -baseSpeed * 0.5f;
                rightSpeed = baseSpeed * 0.5f;
            } else {
                leftSpeed = baseSpeed * 0.5f;
                rightSpeed = -baseSpeed * 0.5f;
            }
        }

        // ---- Clamp speeds for simulation ----
        if (leftSpeed > 4.0f) leftSpeed = 4.0f;
        if (leftSpeed < 0.5f) leftSpeed = 0.5f;
        if (rightSpeed > 4.0f) rightSpeed = 4.0f;
        if (rightSpeed < 0.5f) rightSpeed = 0.5f;

        // ---- Set motor speeds ----
        set_motor(c, leftSpeed, rightSpeed);

        // ---- Control loop delay ----
        SLEEP(20);
    }
    return NULL;
}


/*===============================================================================================*/

/**
 * @brief Main function - Entry point of the program
 * @return 0 if successful, -1 if connection failed
 * 
 * This function:
 * 1. Connects to the CoppeliaSim server
 * 2. Starts the control thread for robot behavior
 * 3. Continuously displays sensor data
 * 4. Handles cleanup when program exits
 */
int main() {
    if (!connect_to_server(&client, "127.0.0.1", 50002)) {
        printf("Failed to connect to CoppeliaSim server. Make sure:\n");
        printf("1. CoppeliaSim is running\n");
        printf("2. The simulation scene is loaded\n");
        printf("3. The ZMQ remote API is enabled on port 50002\n");
        return -1;
    }
    
    printf("Successfully connected to CoppeliaSim server!\n");
    printf("Starting control thread...\n");
    
    // Start the control thread for robot behavior
#ifdef _WIN32
    client.control_thread = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)control_loop, &client, 0, NULL);
#else
    pthread_create(&client.control_thread, NULL, control_loop, &client);
#endif

    // Main loop: Display sensor data continuously
    printf("Monitoring sensor data... (Press Ctrl+C to exit)\n");
    while (1) {
        if (client.sensor_count > 0) {
            printf("Sensors (%d): ", client.sensor_count);
            for (int i = 0; i < client.sensor_count; i++) {
                printf("%.3f ", client.sensor_values[i]);
            }
            printf("\n");
        } else {
            printf("Waiting for sensor data...\n");
        }
        
        SLEEP(200);  // Update display every 200ms
    }
    printf("Disconnecting...\n");
    disconnect(&client);
    return 0;
}
