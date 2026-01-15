from pyvesc.protocol.base import VESCMessage
from pyvesc.VESC.messages import VedderCmd


pre_v3_33_fields = [('temp_mos1', 'h', 10),
                    ('temp_mos2', 'h', 10),
                    ('temp_mos3', 'h', 10),
                    ('temp_mos4', 'h', 10),
                    ('temp_mos5', 'h', 10),
                    ('temp_mos6', 'h', 10),
                    ('temp_pcb',  'h', 10),
                    ('current_motor', 'i', 100),
                    ('current_in',  'i', 100),
                    ('duty_now',    'h', 1000),
                    ('rpm',         'i', 1),
                    ('v_in',        'h', 10),
                    ('amp_hours',   'i', 10000),
                    ('amp_hours_charged', 'i', 10000),
                    ('watt_hours',  'i', 10000),
                    ('watt_hours_charged', 'i', 10000),
                    ('tachometer', 'i', 1),
                    ('tachometer_abs', 'i', 1),
                    ('mc_fault_code', 'c', 0)]


class GetVersion(metaclass=VESCMessage):
    """ Gets version fields
    """
    id = VedderCmd.COMM_FW_VERSION

    fields = [
            ('comm_fw_version', 'b', 0),
            ('fw_version_major', 'b', 0),
            ('fw_version_minor', 'b', 0)
    ]

    def __str__(self):
        return f"{self.comm_fw_version}.{self.fw_version_major}.{self.fw_version_minor}"


class GetValues(metaclass=VESCMessage):
    """ Gets internal sensor data
    """
    id = VedderCmd.COMM_GET_VALUES

    fields = [
        ('temp_fet', 'h', 10),
        ('temp_motor', 'h', 10),
        ('avg_motor_current', 'i', 100),
        ('avg_input_current', 'i', 100),
        ('avg_id', 'i', 100),
        ('avg_iq', 'i', 100),
        ('duty_cycle_now', 'h', 1000),
        ('rpm', 'i', 1),
        ('v_in', 'h', 10),
        ('amp_hours', 'i', 10000),
        ('amp_hours_charged', 'i', 10000),
        ('watt_hours', 'i', 10000),
        ('watt_hours_charged', 'i', 10000),
        ('tachometer', 'i', 1),
        ('tachometer_abs', 'i', 1),
        ('mc_fault_code', 'c', 0),
        ('pid_pos_now', 'i', 1000000),
        ('app_controller_id', 'c', 0),
        ('time_ms', 'i', 1),
    ]


class GetRotorPosition(metaclass=VESCMessage):
    """ Gets rotor position data
    
    Must be set to DISP_POS_MODE_ENCODER or DISP_POS_MODE_PID_POS (Mode 3 or 
    Mode 4). This is set by SetRotorPositionMode (id=21).
    """
    id = VedderCmd.COMM_ROTOR_POSITION

    fields = [
            ('rotor_pos', 'i', 100000)
    ]

class GetIMUData(metaclass=VESCMessage):
    """ Gets IMU data
    """
    id = VedderCmd.COMM_GET_IMU_DATA

    fields = [
        # ('packet_id', 'c', 0),
        ('mask', 'h', 0),
        # ('rpy_x', 'i', 0),
        # ('rpy_y', 'i', 0),
        # ('rpy_z', 'i', 0),
        # ('acc_x', 'i', 0),
        # ('acc_y', 'i', 0),
        # ('acc_z', 'i', 0),
        # ('gyro_x', 'i', 0),
        # ('gyro_y', 'i', 0),
        # ('gyro_z', 'i', 0),
        # ('mag_x', 'i', 0),
        # ('mag_y', 'i', 0),
        # ('mag_z', 'i', 0),
        ('quad_w', 'i', 0),
        ('quad_x', 'i', 0),
        ('quad_y', 'i', 0),
        ('quad_z', 'i', 0),
        ('app_controller_id', 'c', 0),
    ]

class GetMcConf(metaclass=VESCMessage):
    """
    Parses the response payload of COMM_GET_MCCONF.
    Matches confgenerator_deserialize_mcconf exactly (order, types, scales).
    
    After unpacking, for each field with scale > 0, do: real_value = raw_value / scale
    """
    id = VedderCmd.COMM_GET_MCCONF

    fields = [
        # signature (uint32_t)
        ('signature', 'I', 0),
        # pwm_mode, comm_mode, motor_type, sensor_mode (enums as uint8_t)
        ('pwm_mode', 'B', 0),
        ('comm_mode', 'B', 0),
        ('motor_type', 'B', 0),
        ('sensor_mode', 'B', 0),

        # Limits
        ('l_current_max', 'f', 0),
        ('l_current_min', 'f', 0),
        ('l_in_current_max', 'f', 0),
        ('l_in_current_min', 'f', 0),
        ('l_in_current_map_start', 'h', 10000),
        ('l_in_current_map_filter', 'h', 10000),
        ('l_abs_current_max', 'f', 0),
        ('l_min_erpm', 'f', 0),
        ('l_max_erpm', 'f', 0),
        ('l_erpm_start', 'h', 10000),
        ('l_max_erpm_fbrake', 'f', 0),
        ('l_max_erpm_fbrake_cc', 'f', 0),
        ('l_min_vin', 'h', 10),
        ('l_max_vin', 'h', 10),
        ('l_battery_cut_start', 'h', 10),
        ('l_battery_cut_end', 'h', 10),
        ('l_battery_regen_cut_start', 'h', 10),
        ('l_battery_regen_cut_end', 'h', 10),
        ('l_slow_abs_current', 'B', 0),         # bool as byte
        ('l_temp_fet_start', 'B', 0),           # °C
        ('l_temp_fet_end', 'B', 0),
        ('l_temp_motor_start', 'B', 0),
        ('l_temp_motor_end', 'B', 0),
        ('l_temp_accel_dec', 'h', 10000),
        ('l_min_duty', 'h', 10000),
        ('l_max_duty', 'h', 10000),
        ('l_watt_max', 'f', 0),
        ('l_watt_min', 'f', 0),
        ('l_current_max_scale', 'h', 10000),
        ('l_current_min_scale', 'h', 10000),
        ('l_duty_start', 'h', 10000),

        # Sensorless
        ('sl_min_erpm', 'f', 0),
        ('sl_min_erpm_cycle_int_limit', 'f', 0),
        ('sl_max_fullbreak_current_dir_change', 'f', 0),
        ('sl_cycle_int_limit', 'h', 10),
        ('sl_phase_advance_at_br', 'h', 10000),
        ('sl_cycle_int_rpm_br', 'f', 0),
        ('sl_bemf_coupling_k', 'f', 0),

        # Hall sensor
        ('hall_table_0', 'b', 0),
        ('hall_table_1', 'b', 0),
        ('hall_table_2', 'b', 0),
        ('hall_table_3', 'b', 0),
        ('hall_table_4', 'b', 0),
        ('hall_table_5', 'b', 0),
        ('hall_table_6', 'b', 0),
        ('hall_table_7', 'b', 0),
        ('hall_sl_erpm', 'f', 0),

        # FOC
        ('foc_current_kp', 'f', 0),
        ('foc_current_ki', 'f', 0),
        ('foc_f_zv', 'f', 0),
        ('foc_dt_us', 'f', 0),
        ('foc_encoder_inverted', 'B', 0),
        ('foc_encoder_offset', 'f', 0),
        ('foc_encoder_ratio', 'f', 0),
        ('foc_sensor_mode', 'B', 0),
        ('foc_pll_kp', 'f', 0),
        ('foc_pll_ki', 'f', 0),
        ('foc_motor_l', 'f', 0),
        ('foc_motor_ld_lq_diff', 'f', 0),
        ('foc_motor_r', 'f', 0),
        ('foc_motor_flux_linkage', 'f', 0),
        ('foc_observer_gain', 'f', 0),
        ('foc_observer_gain_slow', 'f', 0),
        ('foc_observer_offset', 'h', 1000),
        ('foc_duty_dowmramp_kp', 'f', 0),
        ('foc_duty_dowmramp_ki', 'f', 0),
        ('foc_start_curr_dec', 'h', 10000),
        ('foc_start_curr_dec_rpm', 'f', 0),
        ('foc_openloop_rpm', 'f', 0),
        ('foc_openloop_rpm_low', 'h', 1000),
        ('foc_sl_openloop_hyst', 'h', 100),
        ('foc_sl_openloop_time_lock', 'h', 100),
        ('foc_sl_openloop_time_ramp', 'h', 100),
        ('foc_sl_openloop_time', 'h', 100),
        ('foc_sl_openloop_boost_q', 'h', 100),
        ('foc_sl_openloop_max_q', 'h', 100),
        ('foc_hall_table_0', 'B', 0),
        ('foc_hall_table_1', 'B', 0),
        ('foc_hall_table_2', 'B', 0),
        ('foc_hall_table_3', 'B', 0),
        ('foc_hall_table_4', 'B', 0),
        ('foc_hall_table_5', 'B', 0),
        ('foc_hall_table_6', 'B', 0),
        ('foc_hall_table_7', 'B', 0),
        ('foc_hall_interp_erpm', 'f', 0),
        ('foc_sl_erpm_start', 'f', 0),
        ('foc_sl_erpm', 'f', 0),
        ('foc_control_sample_mode', 'B', 0),
        ('foc_current_sample_mode', 'B', 0),
        ('foc_sat_comp_mode', 'B', 0),
        ('foc_sat_comp', 'h', 1000),
        ('foc_temp_comp', 'B', 0),
        ('foc_temp_comp_base_temp', 'h', 100),
        ('foc_current_filter_const', 'h', 10000),
        ('foc_cc_decoupling', 'B', 0),
        ('foc_observer_type', 'B', 0),
        ('foc_hfi_amb_mode', 'B', 0),
        ('foc_hfi_amb_current', 'h', 10),
        ('foc_hfi_amb_tres', 'B', 0),
        ('foc_hfi_voltage_start', 'h', 10),
        ('foc_hfi_voltage_run', 'h', 10),
        ('foc_hfi_voltage_max', 'h', 10),
        ('foc_hfi_gain', 'h', 1000),
        ('foc_hfi_max_err', 'h', 1000),
        ('foc_hfi_hyst', 'h', 100),
        ('foc_sl_erpm_hfi', 'f', 0),
        ('foc_hfi_reset_erpm', 'f', 0),
        ('foc_hfi_start_samples', 'H', 0),
        ('foc_hfi_obs_ovr_sec', 'f', 0),
        ('foc_hfi_samples', 'B', 0),
        ('foc_offsets_cal_mode', 'B', 0),
        ('foc_offsets_current_0', 'f', 0),
        ('foc_offsets_current_1', 'f', 0),
        ('foc_offsets_current_2', 'f', 0),
        ('foc_offsets_voltage_0', 'h', 10000),
        ('foc_offsets_voltage_1', 'h', 10000),
        ('foc_offsets_voltage_2', 'h', 10000),
        ('foc_offsets_voltage_undriven_0', 'h', 10000),
        ('foc_offsets_voltage_undriven_1', 'h', 10000),
        ('foc_offsets_voltage_undriven_2', 'h', 10000),
        ('foc_phase_filter_enable', 'B', 0),
        ('foc_phase_filter_disable_fault', 'B', 0),
        ('foc_phase_filter_max_erpm', 'f', 0),
        ('foc_mtpa_mode', 'B', 0),
        ('foc_fw_current_max', 'f', 0),
        ('foc_fw_duty_start', 'h', 10000),
        ('foc_fw_ramp_time', 'h', 1000),
        ('foc_fw_q_current_factor', 'h', 10000),
        ('foc_speed_soure', 'B', 0),
        ('foc_short_ls_on_zero_duty', 'B', 0),
        ('foc_overmod_factor', 'h', 10000),
        ('sp_pid_loop_rate', 'B', 0),

        # Speed PID
        ('s_pid_kp', 'f', 0),
        ('s_pid_ki', 'f', 0),
        ('s_pid_kd', 'f', 0),
        ('s_pid_kd_filter', 'h', 10000),
        ('s_pid_min_erpm', 'f', 0),
        ('s_pid_allow_braking', 'B', 0),
        ('s_pid_ramp_erpms_s', 'f', 0),
        ('s_pid_speed_source', 'B', 0),

        # Pos PID
        ('p_pid_kp', 'f', 0),
        ('p_pid_ki', 'f', 0),
        ('p_pid_kd', 'f', 0),
        ('p_pid_kd_proc', 'f', 0),
        ('p_pid_kd_filter', 'h', 10000),
        ('p_pid_ang_div', 'f', 0),
        ('p_pid_gain_dec_angle', 'h', 10),
        ('p_pid_offset', 'f', 0),

        # Current controller
        ('cc_startup_boost_duty', 'h', 10000),
        ('cc_min_current', 'f', 0),
        ('cc_gain', 'f', 0),
        ('cc_ramp_step_max', 'h', 10000),

        # Misc
        ('m_fault_stop_time_ms', 'i', 0),
        ('m_duty_ramp_step', 'h', 10000),
        ('m_current_backoff_gain', 'f', 0),
        ('m_encoder_counts', 'I', 0),
        ('m_encoder_sin_amp', 'h', 1000),
        ('m_encoder_cos_amp', 'h', 1000),
        ('m_encoder_sin_offset', 'h', 1000),
        ('m_encoder_cos_offset', 'h', 1000),
        ('m_encoder_sincos_filter_constant', 'h', 1000),
        ('m_encoder_sincos_phase_correction', 'h', 1000),
        ('m_sensor_port_mode', 'B', 0),
        ('m_invert_direction', 'B', 0),
        ('m_drv8301_oc_mode', 'B', 0),
        ('m_drv8301_oc_adj', 'b', 0),
        ('m_bldc_f_sw_min', 'f', 0),
        ('m_bldc_f_sw_max', 'f', 0),
        ('m_dc_f_sw', 'f', 0),
        ('m_ntc_motor_beta', 'f', 0),
        ('m_out_aux_mode', 'B', 0),
        ('m_motor_temp_sens_type', 'B', 0),
        ('m_ptc_motor_coeff', 'f', 0),
        ('m_ntcx_ptcx_res', 'h', 0.1),          # note: 0.1 → multiply instead of divide!
        ('m_ntcx_ptcx_temp_base', 'h', 10),
        ('m_hall_extra_samples', 'B', 0),
        ('m_batt_filter_const', 'B', 0),
        ('si_motor_poles', 'B', 0),
        ('si_gear_ratio', 'f', 0),
        ('si_wheel_diameter', 'f', 0),
        ('si_battery_type', 'B', 0),
        ('si_battery_cells', 'B', 0),
        ('si_battery_ah', 'f', 0),
        ('si_motor_nl_current', 'f', 0),

        # BMS config (comment out if your firmware version causes overrun)
        ('bms_type', 'B', 0),
        ('bms_limit_mode', 'B', 0),
        ('bms_t_limit_start', 'B', 0),
        ('bms_t_limit_end', 'B', 0),
        ('bms_soc_limit_start', 'h', 1000),
        ('bms_soc_limit_end', 'h', 1000),
        ('bms_vmin_limit_start', 'h', 1000),
        ('bms_vmin_limit_end', 'h', 1000),
        ('bms_vmax_limit_start', 'h', 1000),
        ('bms_vmax_limit_end', 'h', 1000),
        ('bms_fwd_can_mode', 'B', 0),
    ]