#include <stdio.h>
#include <stdlib.h>

#include <iterator>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

class CSVRow {
    public:
        std::string const& operator[](std::size_t index) const {
            return m_data[index];
        }
        double get_double_at(int index) {
            return atof(m_data[index].c_str());
        }
        double get_int_at(int index) {
            return atoi(m_data[index].c_str());
        }
        double get_long_at(int index) {
            return atol(m_data[index].c_str());
        }
        std::size_t size() const {
            return m_data.size();
        }
        void readNextRow(std::istream& str) {
            std::string line;
            std::getline(str, line);

            std::stringstream lineStream(line);
            std::string cell;

            m_data.clear();
            while(std::getline(lineStream, cell, ',')) {
                m_data.push_back(cell);
            }
            // This checks for a trailing comma with no data after it.
            if (!lineStream && cell.empty()) {
                // If there was a trailing comma then add an empty element.
                m_data.push_back("");
            }
        }

        void parse_to_rl(rl& _rl){
            _rl.rqstr_id = get_int_at(0);
            _rl.rqstr_vel_x = get_double_at(4);
            _rl.rqstr_vel_y = get_double_at(5);
            _rl.rqstr_vel_z = get_double_at(6);
            _rl.rqstr_pos_x = get_double_at(14);
            _rl.rqstr_pos_y = get_double_at(15);
            _rl.rqstr_pos_z = get_double_at(16);

            _rl.rspdr_id = get_int_at(1);

            _rl.dist = get_double_at(2);
            _rl.dist_dot = get_double_at(3);
            _rl.dist_str = m_data[2];

            _rl.uwb_time_ms = get_long_at(7);

            _rl.init_rl_est_x = get_double_at(11);
            _rl.init_rl_est_y = get_double_at(12);
            _rl.init_rl_est_z = get_double_at(13);

            _rl.self_vel_x = get_double_at(8);
            _rl.self_vel_y = get_double_at(9);
            _rl.self_vel_z = get_double_at(10);
            _rl.self_pos_x = get_double_at(17);
            _rl.self_pos_y = get_double_at(18);
            _rl.self_pos_z = get_double_at(19);
        }

    void parse_to_local_pos_sp(local_pos_sp& _local_pos_sp){
        _local_pos_sp.timestamp_us = get_long_at(0);
        _local_pos_sp.x = get_double_at(1);
        _local_pos_sp.y = get_double_at(2);
        _local_pos_sp.z = get_double_at(3);
        _local_pos_sp.yaw = get_double_at(4);

        _local_pos_sp.vx = get_double_at(5);
        _local_pos_sp.vy = get_double_at(6);
        _local_pos_sp.vz = get_double_at(7);

        _local_pos_sp.acc_x = get_double_at(8);
        _local_pos_sp.acc_y = get_double_at(9);
        _local_pos_sp.acc_z = get_double_at(10);
    }
    private:
        std::vector<std::string>    m_data;
};

std::istream& operator>>(std::istream& str, CSVRow& data) {
    data.readNextRow(str);
    return str;
}
