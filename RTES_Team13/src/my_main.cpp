// We would like to thank for the 

#include "mbed.h"
#include <vector>
#include "drivers/LCD_DISCO_F429ZI.h"
#include "drivers/TS_DISCO_F429ZI.h"
#include <iostream>
#include <cmath>

#define PI 3.14159


// Define control register addresses and configurations --> Gyroscope!
// Control Register 1
#define CTRL_REG1 0x20
#define CTRL_REG1_CONFIG 0b01'10'1'1'1'1

// Control Register 4
#define CTRL_REG4 0x23
#define CTRL_REG4_CONFIG 0b0'0'01'0'00'0

// Control Register 3
#define CTRL_REG3 0x22
#define CTRL_REG3_CONFIG 0b0'0'0'0'1'000

// Output Register --> X axis
#define OUT_X_L 0x28

// Define Flag bits for the EventFlags object
#define SPI_FLAG 1
#define DATA_READY_FLAG 2
#define LEARNING_FLAG 4
#define UNLOCK_FLAG 8
#define RESET_FLAG 16

// Scaling Factor for data conversion dps --> rps (make sure its the right vale?!)
#define SCALING_FACTOR (17.5f * 0.0174532925199432957692236907684886f / 1000.0f)

// Window Size for Moving Average Window
#define WINDOW_SIZE 10


#define FONT_SIZE 16

LCD_DISCO_F429ZI lcd; // LCD object
TS_DISCO_F429ZI ts; // Touch screen object

// EventFlags Object Declaration
EventFlags flags;

// DTW threshold
float threshold = 0.0f;

// Displayed message
char display_buffer[50];

uint8_t write_buf[32], read_buf[32];
// Key Remove
bool key = false;

const int button1_x = 60;
const int button1_y = 130;
const int button1_width = 120;
const int button1_height = 50;
const char *button1_label = "RECORD";
const int button2_x = 60;
const int button2_y = 210;
const int button2_width = 120;
const int button2_height = 50;
const char *button2_label = "UNLOCK";
const int dark_x = 20;
const int dark_y = 40;
const int dark_width = 80;
const int dark_height = 50;
const char *dark_label = "Dark";
const int light_x = 130;
const int light_y = 40;
const int light_width = 80;
const int light_height = 50;
const char *light_label = "Light";
const int text_x = 5;
const int text_y = 300;
const char *text_0 = "NO KEY RECORDED";
const char *text_1 = "LOCKED";

// Array to store 2 (X,Y,Z Gyroscope) Data Sequences
float tr_accx[30];
float tr_accy[30];
float tr_accz[30];

float te_accx[30];
float te_accy[30];
float te_accz[30];

// Index to tell the gyro_read where to store the readings
bool mode = false;

// Initialize the function that will be used
void display();
void main_thread();
void touch_screen_thread();
void unlocking_phase();
void learning_phase();

// Callback function for SPI Transfer Completion
void spi_cb(int event) {
    flags.set(SPI_FLAG);
}

// Callback function for Data Ready Interrupt
void data_cb() {
    flags.set(DATA_READY_FLAG);
}

// Variable Definitions for Filters
    uint16_t raw_gx, raw_gy, raw_gz; // raw gyro values
    float gx, gy, gz; // converted gyro values
    // float filtered_gx = 0.0f, filtered_gy = 0.0f, filtered_gz = 0.0f; //lpf filtered values    
    // float high_pass_gx = 0.0f, high_pass_gy = 0.0f, high_pass_gz = 0.0f; //hpf filtered values

// Moving Average Filter Buffer --> only to be used with the Moving Average Filter!
float window_gx[WINDOW_SIZE] = {0}, window_gy[WINDOW_SIZE] = {0}, window_gz[WINDOW_SIZE] = {0};
int window_index = 0;

// Callback function: Set Flags for interuppt: data_ready/button pressed
void button_press()
{
    flags.set(RESET_FLAG);
}
void onGyroDataReady()
{
    flags.set(DATA_READY_FLAG);
}

// Display Infos
void display(char* display_buffer){

            const int text_x = 5;
            const int text_y = 300;
            lcd.SetTextColor(LCD_COLOR_BLACK);                  // Set the color to the background color
            lcd.FillRect(0, text_y, lcd.GetXSize(), FONT_SIZE); // Clear a specific line
            lcd.SetTextColor(LCD_COLOR_BLUE);                   // Reset the text color
            lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE);
}

//Decide if a touch screen is inside a button
bool is_touch_inside_button(int touch_x, int touch_y, int button_x, int button_y, int button_width, int button_height)
{
    return (touch_x >= button_x && touch_x <= button_x + button_width &&
            touch_y >= button_y && touch_y <= button_y + button_height);
}

//Draw button on the screen
void draw_button(int x, int y, int width, int height, const char *label)
{
    lcd.SetTextColor(0xff800080);
    lcd.FillRect(x, y, width, height);
    if(x == 60){
        lcd.DisplayStringAt(x + width / 2 - strlen(label)* 19, y + height / 2 - 8, (uint8_t *)label, CENTER_MODE);
    }
    else if(x == 20){
        lcd.DisplayStringAt(x + width / 2 - strlen(label)* 28, y + height / 2 - 8, (uint8_t *)label, CENTER_MODE);
    }
    else if(x == 130){
        lcd.DisplayStringAt(x + width / 2 - strlen(label)* 23, y + height / 2 - 8, (uint8_t *)label, CENTER_MODE);
    } 
}

// Calculate the min of 3 elements for DTW
float MIN(float &a, float &b ,float &c){
  if (a < b){
    if (a < c)return a;
    else return c;
  }
  else {
    if (b < c)return b;
    else return c;
  }
}

// Calculate the Norm for 3-dims data (used in DTW)
float NORM( float dx, float dy, float dz){
  return sqrt(dx*dx + dy*dy + dz*dz);
}

// Calculate the DTW threshold(the similarity threshold between 2 sequences of gesture data)
float DTW_THRESHOLD() {
    // Display initial message
    sprintf(display_buffer, "DTW...");
    display(display_buffer);

    const int n = 30; // Assuming both sequences have length 30 for simplicity
    float scale = 0.5;
    float dir;
    float DTW[n][n];
    float dx, dy, dz;

    // Initialize boundary conditions
    for (int i = 0; i < n; ++i) {
        if (i == 0) {
            dx = tr_accx[i] - te_accx[i];
            dy = tr_accy[i] - te_accy[i];
            dz = tr_accz[i] - te_accz[i];
            dir = (tr_accx[i] * te_accx[i] + tr_accy[i] * te_accy[i] + tr_accz[i] * te_accz[i]) /
                  (NORM(tr_accx[i], tr_accy[i], tr_accz[i]) * NORM(te_accx[i], te_accy[i], te_accz[i]) + 0.000001);
            DTW[i][i] = (1 - scale * dir) * NORM(dx, dy, dz);
        } else {
            dx = tr_accx[i] - te_accx[0];
            dy = tr_accy[i] - te_accy[0];
            dz = tr_accz[i] - te_accz[0];
            dir = (tr_accx[i] * te_accx[0] + tr_accy[i] * te_accy[0] + tr_accz[i] * te_accz[0]) /
                  (NORM(tr_accx[i], tr_accy[i], tr_accz[i]) * NORM(te_accx[0], te_accy[0], te_accz[0]) + 0.000001);
            DTW[i][0] = (1 - scale * dir) * NORM(dx, dy, dz) + DTW[i-1][0];

            dx = tr_accx[0] - te_accx[i];
            dy = tr_accy[0] - te_accy[i];
            dz = tr_accz[0] - te_accz[i];
            dir = (tr_accx[0] * te_accx[i] + tr_accy[0] * te_accy[i] + tr_accz[0] * te_accz[i]) /
                  (NORM(tr_accx[0], tr_accy[0], tr_accz[0]) * NORM(te_accx[i], te_accy[i], te_accz[i]) + 0.000001);
            DTW[0][i] = (1 - scale * dir) * NORM(dx, dy, dz) + DTW[0][i-1];
        }
    }

    // Fill the DTW matrix
    for (int i = 1; i < n; ++i) {
        for (int j = 1; j < n; ++j) {
            dx = tr_accx[i] - te_accx[j];
            dy = tr_accy[i] - te_accy[j];
            dz = tr_accz[i] - te_accz[j];
            dir = (tr_accx[i] * te_accx[j] + tr_accy[i] * te_accy[j] + tr_accz[i] * te_accz[j]) /
                  (NORM(tr_accx[i], tr_accy[i], tr_accz[i]) * NORM(te_accx[j], te_accy[j], te_accz[j]) + 0.000001);
            DTW[i][j] = (1 - scale * dir) * NORM(dx, dy, dz) + MIN(DTW[i-1][j], DTW[i][j-1], DTW[i-1][j-1]);
        }
    }

    // Backtrack to find the cost function
    int i = n - 1, j = n - 1;
    float mean = 0;
    while (i > 0 || j > 0) {
        dx = tr_accx[i] - te_accx[j];
        dy = tr_accy[i] - te_accy[j];
        dz = tr_accz[i] - te_accz[j];
        dir = (tr_accx[i] * te_accx[j] + tr_accy[i] * te_accy[j] + tr_accz[i] * te_accz[j]) /
              (NORM(tr_accx[i], tr_accy[i], tr_accz[i]) * NORM(te_accx[j], te_accy[j], te_accz[j]) + 0.000001);
        float cost = (1 - scale * dir) * NORM(dx, dy, dz);

        if (i == 0 && j == 0) {
            mean += cost;
            break;
        } else if (i == 0) {
            mean += cost + DTW[i][j-1];
            --j;
        } else if (j == 0) {
            mean += cost + DTW[i-1][j];
            --i;
        } else if (DTW[i-1][j] <= DTW[i][j-1] && DTW[i-1][j] <= DTW[i-1][j-1]) {
            mean += cost + DTW[i-1][j];
            --i;
        } else if (DTW[i][j-1] <= DTW[i-1][j] && DTW[i][j-1] <= DTW[i-1][j-1]) {
            mean += cost + DTW[i][j-1];
            --j;
        } else {
            mean += cost + DTW[i-1][j-1];
            --i;
            --j;
        }
    }

    // Calculate the mean as the threshold
    return mean / n;
}

// Display Main Menu
void main_UI(){
    
    lcd.Clear(LCD_COLOR_BLACK);

    draw_button(dark_x, dark_y, dark_width, dark_height, dark_label);

    draw_button(light_x, light_y, light_width, light_height, light_label);

    // Draw button 1
    draw_button(button1_x, button1_y, button1_width, button1_height, button1_label);

    // Draw button 2
    draw_button(button2_x, button2_y, button2_width, button2_height, button2_label);
}

// Display unlock animation smile face
void draw_unlock(){

    // Clear the screen and set the background color
    lcd.Clear(LCD_COLOR_BLACK);

    // Draw the face (yellow circle)
    lcd.SetTextColor(LCD_COLOR_YELLOW);
    lcd.FillCircle(120, 120, 80);

    // Draw the eyes (black circles)
    lcd.SetTextColor(LCD_COLOR_BLACK);
    lcd.FillCircle(90, 90, 10);  // Left eye
    lcd.FillCircle(150, 90, 10); // Right eye

    // Draw the smile (red arc)
    lcd.SetTextColor(LCD_COLOR_RED);

    // Arc parameters
    int centerX = 120;  // Center of the face
    int centerY = 130;  // Slightly below the center for the smile
    int radius = 50;    // Radius of the smile
    int startAngle = 30;  // Start of the arc (in degrees)
    int endAngle = 150;   // End of the arc (in degrees)

    for (int angle = startAngle; angle <= endAngle; angle++) {
        // Convert angle to radians
        float rad = angle * PI / 180;

        // Calculate the arc's points
        int x = centerX + radius * cos(rad);
        int y = centerY + radius * sin(rad);

        // Draw each point on the arc
        lcd.DrawPixel(x, y, LCD_COLOR_RED);
    }
    
    auto start_time = std::chrono::high_resolution_clock::now();

    while (1) {
        thread_sleep_for(1000);  // Empty Loop
        if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - start_time).count() >= 0.5) {
                return;}
    }
}

// Display unlock animation sad face


void draw_unlock_fail() {
    // Clear the screen and set the background color
    lcd.Clear(LCD_COLOR_BLACK);

    // Draw the face (yellow circle)
    lcd.SetTextColor(LCD_COLOR_YELLOW);
    lcd.FillCircle(120, 120, 80);

    // Draw the eyes (small black circles)
    lcd.SetTextColor(LCD_COLOR_BLACK);
    lcd.FillCircle(95, 100, 10);  // Left eye
    lcd.FillCircle(145, 100, 10); // Right eye

    // Draw the frown (red arc)
    lcd.SetTextColor(LCD_COLOR_RED);

    // Arc parameters for a frown
    int centerX = 120;  // Center of the face
    int centerY = 80;  // Slightly below the center for the smile
    int radius = 50;    // Radius of the frown
    int startAngle = -30;  // Start of the arc (in degrees)
    int endAngle = -150;   // End of the arc (in degrees)

    for (int angle = startAngle; angle <= endAngle; angle++) {
        // Convert angle to radians
        float rad = angle * PI / 180;

        // Calculate the arc's points
        int x = centerX + radius * cos(rad);
        int y = centerY + radius * sin(rad);

        // Draw each point on the arc
        lcd.DrawPixel(x, y, LCD_COLOR_RED);
    }

    auto start_time = std::chrono::high_resolution_clock::now();

    while (1) {
        thread_sleep_for(1000);  // Empty Loop
        if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - start_time).count() >= 0.5) {
                return;}
    }
}


// Display loading animation
void loading_screen(){

    // Clear the screen
    lcd.Clear(LCD_COLOR_BLACK);

    // Set initial text color for the lines
    lcd.SetTextColor(LCD_COLOR_WHITE);

    uint16_t centerX = lcd.GetXSize() / 2;
    uint16_t centerY = lcd.GetYSize() / 2;
    uint16_t radius = 80;

    auto start_time = std::chrono::high_resolution_clock::now();

    while (1 ) {
        for (int angle = 0; angle < 360; angle += 10) {
            // Convert angle to radians
            float rad = angle * PI / 180;

            // Calculate end point of the line
            uint16_t endX = centerX + radius * cos(rad);
            uint16_t endY = centerY + radius * sin(rad);

            // Draw the line
            lcd.DrawLine(centerX, centerY, endX, endY);

            thread_sleep_for(50);

            // Erase the line for the next frame
            lcd.SetTextColor(LCD_COLOR_BLACK);  // Set color to background color
            lcd.DrawLine(centerX, centerY, endX, endY);

            lcd.SetTextColor(LCD_COLOR_WHITE);  // Reset color to line color
            if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - start_time).count() >= 0.5) {
                return;}
        }
    }
}


int main() {

    main_UI();

    
    // Create the gyroscope thread
    Thread key_saving;
    key_saving.start(callback(main_thread));

    // Create the touch screen thread
    Thread touch_thread;
    touch_thread.start(callback(touch_screen_thread));
    // STEP 4: Loop for Data Collection and Filtering!
    while (1) {
        
        ThisThread::sleep_for(100ms);

        }

    }

// Function to read the gyroscope data
void read_gyro() {

    // SPI
    SPI spi(PF_9, PF_8, PF_7, PC_1, use_gpio_ssel);

    
    // Interrupt
    InterruptIn int2(PA_2, PullDown);
    InterruptIn button(PA_0, PullDown);
    int2.rise(&data_cb);
    button.rise(&button_press);

    // SPI Data transmission format and frequency
    spi.format(8, 3);
    spi.frequency(1'000'000);

    // STEP 3: GYRO Configuration!
    // 1. Control Register 1 
    write_buf[0] = CTRL_REG1;
    write_buf[1] = CTRL_REG1_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, &spi_cb);
    flags.wait_all(SPI_FLAG);

    // 2. Control Register 4
    write_buf[0] = CTRL_REG4;
    write_buf[1] = CTRL_REG4_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, &spi_cb);
    flags.wait_all(SPI_FLAG);

    // 3. Control Register 3
    write_buf[0] = CTRL_REG3;
    write_buf[1] = CTRL_REG3_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, &spi_cb);
    flags.wait_all(SPI_FLAG);

    // dUMMY BYTE for write_buf[1] --> Placeholder Value!
    // We have to send an address and a value for write operation!
    // We have the address but have to send a placeholder value as well!
    write_buf[1] = 0xFF;
    
    int counter = 0;
    for (counter = 0; counter < 30; counter++) {
                // Read GYRO Data using SPI transfer --> 6 bytes!
        write_buf[0] = OUT_X_L | 0x80 | 0x40;
        spi.transfer(write_buf, 7, read_buf, 7, spi_cb);
        flags.wait_all(SPI_FLAG);

        // Extract raw 16-bit gyroscope data for X, Y, Z
        raw_gx = (read_buf[2] << 8) | read_buf[1];
        raw_gy = (read_buf[4] << 8) | read_buf[3];
        raw_gz = (read_buf[6] << 8) | read_buf[5];

        // Convert raw data to radians per second!
        gx = raw_gx * SCALING_FACTOR;
        gy = raw_gy * SCALING_FACTOR;
        gz = raw_gz * SCALING_FACTOR;

        //  Moving Average FIR
        window_gx[window_index] = gx;
        window_gy[window_index] = gy;
        window_gz[window_index] = gz;
        float avg_gx = 0.0f, avg_gy = 0.0f, avg_gz = 0.0f;
        for (int i = 0; i < WINDOW_SIZE; i++) {
            avg_gx += window_gx[i];
            avg_gy += window_gy[i];
            avg_gz += window_gz[i];
        }
        avg_gx /= WINDOW_SIZE;
        avg_gy /= WINDOW_SIZE;
        avg_gz /= WINDOW_SIZE;
        window_index = (window_index + 1) % WINDOW_SIZE;
        printf("Moving Average -> gx: %4.5f, gy: %4.5f, gz: %4.5f\n", avg_gx, avg_gy, avg_gz);
        printf(">Moving Average X axis-> gx: %4.5f|g\n", avg_gx);
        printf(">Moving Average Y axis-> gy: %4.5f|g\n", avg_gy);
        printf(">Moving Average Z axis-> gz: %4.5f|g\n", avg_gz);
        if(mode){        
            tr_accx[counter] = avg_gx;
            tr_accy[counter] = avg_gy;
            tr_accz[counter] = avg_gz;
        }else{
            te_accx[counter] = avg_gx;
            te_accy[counter] = avg_gy;
            te_accz[counter] = avg_gz;  
        }


        thread_sleep_for(100);
    }
}


// Training a gesture
void  training_phase(){
    if(!key){
        sprintf(display_buffer, "Training Phase!");
        display(display_buffer);
        ThisThread::sleep_for(500ms);
        sprintf(display_buffer, "Start in 2s");
        display(display_buffer);
        ThisThread::sleep_for(1s);
        sprintf(display_buffer, "Start in 1s");
        display(display_buffer);
        ThisThread::sleep_for(1s);
        
        mode = true; // record reference gesture (#1)

        sprintf(display_buffer, "Recording !!!");
        display(display_buffer);
        read_gyro();
        sprintf(display_buffer, "Record Finished!");
        display(display_buffer);
        ThisThread::sleep_for(500ms);
        // tr_accx = result.first;
        // tr_accy  = result.second.first;
        // tr_accz  = result.second.second;

        sprintf(display_buffer, "Repeat Gesture!");
        display(display_buffer);
        ThisThread::sleep_for(500ms);
        sprintf(display_buffer, "Start in 2s");
        display(display_buffer);
        ThisThread::sleep_for(1s);
        sprintf(display_buffer, "Start in 1s");
        display(display_buffer);
        ThisThread::sleep_for(1s);
        sprintf(display_buffer, "Recording !!!");
        display(display_buffer);
        mode = false; //record gesture #2

        read_gyro();
        sprintf(display_buffer, "Repeat Finished!");
        display(display_buffer);
        // auto result2 = read_gyro();
        // te_accx = result2.first;
        // te_accy  = result2.second.first;
        // te_accz  = result2.second.second;
        sprintf(display_buffer, "Computing!!");
        display(display_buffer);
        loading_screen();
        threshold = DTW_THRESHOLD(); 
        //print learning completed
        main_UI();
        sprintf(display_buffer, "Learning Completed!");
        display(display_buffer);
        key = true;
    }else{
        return;
    }
}

// Unlocking
void unlocking_phase(){
    flags.clear(UNLOCK_FLAG);
    if(key){

        // tr_accx = result.first;
        // tr_accy  = result.second.first;
        // tr_accz  = result.second.second;

        // auto result2 = read_gyro();
        // te_accx = result2.first;
        // te_accy  = result2.second.first;
        // te_accz  = result2.second.second;
        // remind user start recording gesture in 1s!
        sprintf(display_buffer, "Unlock Phase!");
        display(display_buffer);
        ThisThread::sleep_for(500ms);
        sprintf(display_buffer, "Start in 2s");
        display(display_buffer);
        ThisThread::sleep_for(1s);
        sprintf(display_buffer, "Start in 1s");
        display(display_buffer);
        ThisThread::sleep_for(1s);
        mode = false;
        sprintf(display_buffer, "Recording !!!");
        display(display_buffer);
        read_gyro();
        sprintf(display_buffer, "Record Finished!");
        display(display_buffer);
        sprintf(display_buffer, "Computing!!");
        display(display_buffer);
        loading_screen();
        float var = DTW_THRESHOLD();

        if ( (var >= 0 && var <= threshold + 5) ) {

            draw_unlock();
            sprintf(display_buffer, "UNLOCK: SUCCESS!");
            display(display_buffer);
            thread_sleep_for(1000);
            main_UI();


        }else{
            draw_unlock_fail();
            sprintf(display_buffer, "UNLOCK: FAILED!");
            display(display_buffer);
            thread_sleep_for(1000);
            main_UI();
        }
    }else{
        sprintf(display_buffer, "Please set a key!");
        display(display_buffer);
    }
}

// Reset key(gesture)
void reset_key(){
    if(key){
        key = false; // Removed key
        sprintf(display_buffer, "Key removed!");
        display(display_buffer);
        thread_sleep_for(250);
        key = false;
    }else{
        sprintf(display_buffer, "Please set a key!");
        display(display_buffer);
        thread_sleep_for(250);
    }
}

// Main thread(train, unlock, reset key)
void main_thread(){
    
    while(1){
        auto flag_check = flags.wait_any(LEARNING_FLAG | UNLOCK_FLAG | RESET_FLAG);

        if(flag_check & LEARNING_FLAG){
            flags.clear(LEARNING_FLAG);
            training_phase();

        }else if(flag_check & UNLOCK_FLAG){
            flags.clear(UNLOCK_FLAG);
            unlocking_phase();
        }else if(flag_check & RESET_FLAG & key){
            flags.clear(RESET_FLAG);
            reset_key();
        }else{
            sprintf(display_buffer, "Please set a key!");
            display(display_buffer);
        }

        ThisThread::sleep_for(500ms);
    }

}


// Touch screen thread
void touch_screen_thread()
{   
    // Add your touch screen initialization and handling code here
    TS_StateTypeDef ts_state;

    if (ts.Init(lcd.GetXSize(), lcd.GetYSize()) != TS_OK)
    {
        printf("Failed to initialize the touch screen!\r\n");
        return;
    }

    // // initialize a string display_buffer that can be draw on the LCD to dispaly the status
    // char display_buffer[50];

    while (1)
    {
        ts.GetState(&ts_state);
        if (ts_state.TouchDetected)
        {
            int touch_x = ts_state.X;
            int touch_y = ts_state.Y;

            if (is_touch_inside_button(touch_x, touch_y, dark_x, dark_y, 190, dark_height)){

                flags.set(UNLOCK_FLAG);

            }

            // if (is_touch_inside_button(touch_x, touch_y, light_x, light_y, light_width, light_height)){
                
            // }

            // Check if the touch is inside record button
            if (is_touch_inside_button(touch_x, touch_y, 140, button2_y, light_width, light_height))
            {
                lcd.Clear(LCD_COLOR_WHITE);

                draw_button(dark_x, dark_y, dark_width, dark_height, dark_label);

                draw_button(light_x, light_y, light_width, light_height, light_label);

                // Draw button 1
                draw_button(button1_x, button1_y, button1_width, button1_height, button1_label);

                // Draw button 2
                draw_button(button2_x, button2_y, button2_width, button2_height, button2_label);
            }

            if (is_touch_inside_button(touch_x, touch_y, button2_x, button2_y, dark_width, dark_height))
            {
                lcd.Clear(LCD_COLOR_BLACK);

                draw_button(dark_x, dark_y, dark_width, dark_height, dark_label);

                draw_button(light_x, light_y, light_width, light_height, light_label);

                // Draw button 1
                draw_button(button1_x, button1_y, button1_width, button1_height, button1_label);

                // Draw button 2
                draw_button(button2_x, button2_y, button2_width, button2_height, button2_label);
            }

            // Check if the touch is inside unlock button
            if (is_touch_inside_button(touch_x, touch_y, button1_x, button1_y, button2_width, button2_height))
            {
                flags.set(LEARNING_FLAG);
            }
        }
        ThisThread::sleep_for(10ms);
    }
}
