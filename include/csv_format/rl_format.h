struct rl {
    int rqstr_id;
    double rqstr_vel_x;
    double rqstr_vel_y;
    double rqstr_vel_z;
    double rqstr_pos_x;
    double rqstr_pos_y;
    double rqstr_pos_z;

    int rspdr_id;

    double dist;
    double dist_dot;
    std::string dist_str;

    long uwb_time_ms;

    double init_rl_est_x;
    double init_rl_est_y;
    double init_rl_est_z;

    double self_vel_x;
    double self_vel_y;
    double self_vel_z;
    double self_pos_x;
    double self_pos_y;
    double self_pos_z;
};
