# IncEnc_board_lib  

AMT102-CAN変換基盤からのデータを処理するライブラリです  
CANデータの受け取り処理はmain関数内でしてください。このメンバ関数でデータ変換ができます。
```main.cpp

#include "mbed.h"
#include "RotaryEncoder.h"
#include "NoMutexCAN.h"
#include <chrono>
#include <cstdint>

using namespace std::chrono;

#define CONFIRM_TIME 1000ms
#define BLINK_INTERVAL 150ms
#define PAUSE_INTERVAL 1000ms
#define ID_PAGE_ADDRESS 0x0800F000
#define ANGLE_PAGE_ADDRESS 0x0800F800

uint32_t stored_id = *((uint32_t*)ID_PAGE_ADDRESS);
int64_t stored_angle = *((int64_t*)ANGLE_PAGE_ADDRESS);

DigitalOut can_led(PA_0);
DigitalOut id_indicator_led(PA_1);
DigitalOut angle_change_led(PA_2);
InterruptIn id_set_button(PA_8, PullDown);
STM32_encoder encoder(PA_6, PA_7); 
// BufferedSerial pc(USBTX, USBRX, 9600);
BufferedSerial debug_uart(PA_9, PA_10, 9600);
// CAN can(PA_11, PA_12, 1000000);
NoMutexCAN can(PA_11, PA_12, 1000000);
CANMessage send_msg, receive_msg;

Timer id_set_timer, can_check_timer;
Timeout blink_event;

volatile int new_id_counter = 0;
int blink_count = 0;
uint16_t tderr_cnt = 0;
bool is_id_set_mode = false;
int64_t angle, tmp = 0;

void id_set_isr();
void blink_handler();
int flash_write(uint32_t write_addr, uint32_t num);
void check_can_bus_status();

int main(){
    // printf("main function start\r\n"):
    can.reset();
    id_set_button.rise(id_set_isr);
    encoder.start();
    encoder.reset();
    can_check_timer.start();
    
    blink_handler();

    angle_change_led = false;

    while(true){
        angle = encoder.get_angle();

        if(can.read(receive_msg)){
            if (receive_msg.id == (0x400 + stored_id) && receive_msg.data[0] == 0xff) {
                NVIC_SystemReset();
            }
        }

        angle_change_led = (tmp != angle);
        tmp = angle;

        if(is_id_set_mode) {
            id_indicator_led = id_set_button;
            if(id_set_timer.elapsed_time() > CONFIRM_TIME){
                if (flash_write(ID_PAGE_ADDRESS, new_id_counter) == 1) {
                    NVIC_SystemReset();
                } else {
                    is_id_set_mode = false;
                    blink_handler();
                }
            }
        }

        send_msg.id = 0x400 + stored_id;
        
        tderr_cnt = can.tderror();
        // tderr_cnt = 130; // test
        can_led = (tderr_cnt < 255);

        for(int i = 0; i < 8; i++){
            send_msg.data[i] = (uint8_t)(angle >> 8 * (7 - i));
        }     
        can.write(send_msg);

        printf("%2d, %lld\r\n", stored_id, angle);
        printf("\r\n%d, %d, ", stored_id, tderr_cnt);
        for(int i = 0; i < 8; i++) printf("%03d ", send_msg.data[i]);
        
        ThisThread::sleep_for(10ms);
    }
}

void id_set_isr(){
    if(!is_id_set_mode){
        // 最初のプレスでID設定モードを開始
        is_id_set_mode = true;
        new_id_counter = 1;
        id_set_timer.start();
        id_set_timer.reset();
        blink_event.detach();
    } else {
        // ID設定モード中ならIDをカウントアップ
        new_id_counter++;
        id_set_timer.reset(); // タイマーリセットで猶予時間を延長
    }
}

void blink_handler(){
    if (stored_id == 0 || stored_id > 255) return;

    if(blink_count < (stored_id * 2)) {
        id_indicator_led = !id_indicator_led;
        blink_count++;
        blink_event.attach(blink_handler, BLINK_INTERVAL);
    } else {
        id_indicator_led = false;
        blink_count = 0;
        blink_event.attach(blink_handler, PAUSE_INTERVAL);
    }
}

int flash_write(uint32_t write_addr, uint32_t num){
    uint32_t page_error = 0;
    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef erase_init;
    erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
    erase_init.PageAddress = write_addr;
    erase_init.NbPages = 1;
    if (HAL_OK != HAL_FLASHEx_Erase(&erase_init, &page_error)) return -1;
    if (HAL_OK != HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, write_addr, num)) return -1;
    HAL_FLASH_Lock();
    return 1;
}

// #include "mbed.h"

// // BufferedSerial pc(PA_9, PA_10, 9600);
// InterruptIn id_set_button(PA_8, PullDown);
// DigitalOut id_indicator_led(PA_1);

// void handler();

// int main(){
//     id_set_button.rise(handler);
//     while(true){
//         ThisThread::sleep_for(10ms);
//     }
// }

// void handler(){
//     id_indicator_led = true;
// }
```
