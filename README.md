# IncEnc_board_lib  
AMT102-CAN変換基盤からのデータを処理するライブラリです  
CANデータの受け取り処理はmain関数内でしてください。このメンバ関数でデータ変換ができます。
  
注意  
もしencoder_reset_all()ですべてのnodeにリセットコマンドを送信できない不具合が起きた場合、IncEnc_board.cpp内のコメントアウトした部分を戻してあげるとすべてにリセットがかかると思います　だたこれは_can.write(msg)を確実に行うための応急処置的なもののため、main関数の処理中にThisThread::sleep_for()が入ってしまうことに注意してください

  
  
サンプルコード  
  
main.cpp
~~~main.cpp
#include "mbed.h"
#include "IncEnc_board.h"

CAN can(PD_0, PD_1, 1000000);
UnbufferedSerial pc(USBTX, USBRX, 9600);
IncEnc_board encoder(can, 1);

int main() {
    encoder.encoder_reset_all();
    while(true) {
        int64_t received_angle;
        encoder.conv_data_all(&received_angle);
        
        if(pc.readable()){
            char key = 0;
            pc.read(&key, 1);
            switch(key){
                case 'r': 
                    printf("Sending reset command to node 1...\r\n");
                    encoder.encoder_reset_all();
                    break;
            }
        }
        printf("Received data: %lld\r\n", received_angle);
        
        ThisThread::sleep_for(1ms);
    }
}
~~~  
  
  
以下は変換基盤に書き込んでいるプログラムです。  
不具合があった際の参考にしてください  
  
main.cpp(embedded)  
~~~main.cpp
#include "mbed.h"
#include "rotaryencoder/STM32_encoder/STM32_encoder.h"
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

#define CONFIRM_TIME 1000ms
#define BLINK_INTERVAL 150ms
#define PAUSE_INTERVAL 1000ms
#define ID_PAGE_ADDRESS 0x0800F000
#define ANGLE_PAGE_ADDRESS 0x0800F800

#ifndef M_PI
#define M_PI 3.14159f
#endif

// 1パルスあたりの移動角度 [deg/pulse] (例: 200分解能×4倍率 = 800pulse/回転 -> 360/800 = 0.45 deg)
constexpr float DEG_PER_PULSE = 360.0f / (200.0f * 4.0f);

uint32_t stored_id = *((uint32_t*)ID_PAGE_ADDRESS);
int64_t stored_angle = *((int64_t*)ANGLE_PAGE_ADDRESS);
typedef union{
    uint8_t _msg_buf[8];
    int64_t _msg_ang;
} ang2can;
ang2can conv;

mbed::DigitalOut can_led(PA_0);
mbed::DigitalOut id_indicator_led(PA_1);
mbed::DigitalOut angle_change_led(PA_3);//25年版からこの1行を変化させるだけでも問題ない
mbed::InterruptIn id_set_button(PA_8, PullDown); // DigitalIn
STM32_encoder encoder(PA_6, PA_7); 
// mbed::BufferedSerial pc(USBTX, USBRX, 9600);
//mbed::BufferedSerial debug_uart(PA_9, PA_10, 9600);
mbed::CAN can(PA_11, PA_12, 1000000);
mbed::CANMessage send_msg, receive_msg;
constexpr size_t sizedata=sizeof(send_msg.data);

mbed::Timer id_set_timer, can_check_timer,v_Timer;
mbed::Timeout blink_event;

volatile int new_id_counter = 0;
int blink_count = 0;
uint16_t tderr_cnt = 0;
bool is_id_set_mode = false;
bool is_v_mode=false,prev_v=false;
int64_t angle = 0 , prev_angle = 0 , tmp = 0;
float vel=0.f;
constexpr size_t sizedata_v=sizeof(vel);

void id_set_isr();
void blink_handler();
int flash_write(uint32_t write_addr, uint32_t num);

int main(){
    // printf("main function start\r\n");
    can.reset();
    can.filter(0x400 + stored_id, 0x780);
    id_set_timer.start();
    can_check_timer.start();
    v_Timer.start();
    id_set_button.rise(id_set_isr);
    encoder.start();
    encoder.reset();
    
    blink_handler();

    angle_change_led = false;
    int64_t prev_count;
    while(true){
        angle = encoder.get_angle();
        int64_t current_count = encoder.get_count();

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

        if(can.read(receive_msg)){
            if(receive_msg.id == (0x400 + stored_id)) {
                if(receive_msg.data[0] == 0xff){
                    NVIC_SystemReset();
                    angle = 0;
                }else if(receive_msg.data[0]==1){
                    is_v_mode=true;
                }else if(receive_msg.data[0]==2){
                    is_v_mode=false;
                    prev_v=false;
                }
            }
        }

        if (is_v_mode) {
            if (prev_v) {
                float dt = std::chrono::duration<float>{v_Timer.elapsed_time()}.count();
                v_Timer.reset();

                if (dt > 1e-4) {
                    // 1. まず 64bit 整数同士で差分（10ms間の経過パルス数）を計算
                    int64_t diff_count = current_count - prev_count;

                    // 2. 差分（高々数千程度の小さい値）になってから float に変換して速度計算
                    float raw_vel = (static_cast<float>(diff_count) * DEG_PER_PULSE) / dt;

                    // // 3. ローパスフィルターを通す（ノイズ除去）
                    // vel = vel + 0.2f * (raw_vel - vel);
                    vel=raw_vel;
                }

                std::memset(send_msg.data, 0, 8);
                std::memcpy(send_msg.data, &vel, sizeof(float));
            } else {
                std::memset(send_msg.data, 0, 8);
                prev_v = true;
                prev_count=current_count;
                v_Timer.reset();
            }
            prev_count = current_count;
        }
        can.write(send_msg);

        // printf("%2d, %lld\r\n", stored_id, angle);
        // printf("\r\n%d, %d, ", stored_id, tderr_cnt);
        // for(int i = 0; i < 8; i++) printf("%03d ", send_msg.data[i]);
        
        ThisThread::sleep_for(10ms); // 1000Hz bad, 100Hz ok
    }
}

void id_set_isr(){
    if(!is_id_set_mode){
        // 最初のプレスでID設定モードを開始
        is_id_set_mode = true;
        new_id_counter = 1;
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


~~~
  
内部クロックの使用を明示することも忘れないでください  
このファイルはmain.cppと同じ階層に配置しておくだけでいいです  
  
mbed_app.json  
~~~.json
{
    "target_overrides": {
        "NUCLEO-F303K8": {
            "target.clock_source": "HSI", 
            "target.restrict_size": "0xF000"
        }
    }
}
}
~~~
