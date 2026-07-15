#pragma once

#include "mbed.h"
#include "CANManager.h"
class IncEnc_board :public CANReceiver{
public:
    IncEnc_board(mbed::CAN &can, int all_node_num);
    bool encoder_reset_node(int node);
    bool encoder_reset_all();
    void conv_data_node(int64_t* angle, uint8_t node);
    void conv_data_all(int64_t* angles);
    

private:
    bool handle_message(const mbed::CANMessage &msg) override;
    //void data_control();
    int _all_node_num;
    mbed::CAN &_can;
    rtos::Mutex _data_mutex;
    std::vector<CANMessage> _msg_buffer;
    uint8_t _new_data_mask;
};
