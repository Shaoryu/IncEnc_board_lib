#include "IncEnc_board.h"

IncEnc_board::IncEnc_board(CAN &can, int all_node_num)
    : _can(can), _all_node_num(all_node_num){
    _msg_buffer.assign(_all_node_num,0);
    _can.frequency(1e6);
    _can.mode(CAN::Normal);
    //data_control();
}

bool IncEnc_board::encoder_reset_node(int node){
    CANMessage msg;
    bool result = false;
    msg.id   = 0x400 + node;
    msg.len  = 1;
    msg.data[0] = 0xff;
    return _can.write(msg)==1;
}

bool IncEnc_board::encoder_reset_all(){
    int reset_cnt = 0;
    bool result = false;
    for(int id = 1; id <= _all_node_num; id++){
        this->encoder_reset_node(id);
        ThisThread::sleep_for(10ms); // 適宜変更をお願いします
    }
    return true;
}

void IncEnc_board::conv_data_node(int64_t *angle, uint8_t node){
    const int index = node - 1;
    angle[index] = 0;
    for (int i = 0; i < 8; i++) {
        angle[index] |= (int64_t)_msg_buffer[index].data[i] << (8 * (7 - i));
    }
    
}

void IncEnc_board::conv_data_all(int64_t *angles){
    for(int node = 1; node <= _all_node_num; node++) this->conv_data_node(angles, node);
}

bool IncEnc_board::handle_message(const CANMessage &msg){
    int id_idx = msg.id - 0x401;
    if (id_idx >= 0 && id_idx < _all_node_num) {
        _data_mutex.lock();
        _msg_buffer[id_idx] = msg;
        _data_mutex.unlock();
        return true;
    }
    return false;
}
