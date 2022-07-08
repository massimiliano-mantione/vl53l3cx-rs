#![allow(dead_code, mutable_transmutes, non_camel_case_types, non_snake_case, non_upper_case_globals, unused_assignments, unused_mut)]
//#![register_tool(c2rust)]
//#![feature(register_tool)]
extern "C" {
    fn memset(
        _: *mut libc::c_void,
        _: libc::c_int,
        _: libc::c_ulong,
    ) -> *mut libc::c_void;
    fn memcpy(
        _: *mut libc::c_void,
        _: *const libc::c_void,
        _: libc::c_ulong,
    ) -> *mut libc::c_void;
}
pub type CP_STATUS = libc::c_int;
pub type FixPoint1616_t = u32;
pub type VL53LX_Error = i8;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_static_nvm_managed_t {
    pub i2c_slave__device_address: u8,
    pub ana_config__vhv_ref_sel_vddpix: u8,
    pub ana_config__vhv_ref_sel_vquench: u8,
    pub ana_config__reg_avdd1v2_sel: u8,
    pub ana_config__fast_osc__trim: u8,
    pub osc_measured__fast_osc__frequency: u16,
    pub vhv_config__timeout_macrop_loop_bound: u8,
    pub vhv_config__count_thresh: u8,
    pub vhv_config__offset: u8,
    pub vhv_config__init: u8,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_customer_nvm_managed_t {
    pub global_config__spad_enables_ref_0: u8,
    pub global_config__spad_enables_ref_1: u8,
    pub global_config__spad_enables_ref_2: u8,
    pub global_config__spad_enables_ref_3: u8,
    pub global_config__spad_enables_ref_4: u8,
    pub global_config__spad_enables_ref_5: u8,
    pub global_config__ref_en_start_select: u8,
    pub ref_spad_man__num_requested_ref_spads: u8,
    pub ref_spad_man__ref_location: u8,
    pub algo__crosstalk_compensation_plane_offset_kcps: u16,
    pub algo__crosstalk_compensation_x_plane_gradient_kcps: i16,
    pub algo__crosstalk_compensation_y_plane_gradient_kcps: i16,
    pub ref_spad_char__total_rate_target_mcps: u16,
    pub algo__part_to_part_range_offset_mm: i16,
    pub mm_config__inner_offset_mm: i16,
    pub mm_config__outer_offset_mm: i16,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_static_config_t {
    pub dss_config__target_total_rate_mcps: u16,
    pub debug__ctrl: u8,
    pub test_mode__ctrl: u8,
    pub clk_gating__ctrl: u8,
    pub nvm_bist__ctrl: u8,
    pub nvm_bist__num_nvm_words: u8,
    pub nvm_bist__start_address: u8,
    pub host_if__status: u8,
    pub pad_i2c_hv__config: u8,
    pub pad_i2c_hv__extsup_config: u8,
    pub gpio_hv_pad__ctrl: u8,
    pub gpio_hv_mux__ctrl: u8,
    pub gpio__tio_hv_status: u8,
    pub gpio__fio_hv_status: u8,
    pub ana_config__spad_sel_pswidth: u8,
    pub ana_config__vcsel_pulse_width_offset: u8,
    pub ana_config__fast_osc__config_ctrl: u8,
    pub sigma_estimator__effective_pulse_width_ns: u8,
    pub sigma_estimator__effective_ambient_width_ns: u8,
    pub sigma_estimator__sigma_ref_mm: u8,
    pub algo__crosstalk_compensation_valid_height_mm: u8,
    pub spare_host_config__static_config_spare_0: u8,
    pub spare_host_config__static_config_spare_1: u8,
    pub algo__range_ignore_threshold_mcps: u16,
    pub algo__range_ignore_valid_height_mm: u8,
    pub algo__range_min_clip: u8,
    pub algo__consistency_check__tolerance: u8,
    pub spare_host_config__static_config_spare_2: u8,
    pub sd_config__reset_stages_msb: u8,
    pub sd_config__reset_stages_lsb: u8,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_general_config_t {
    pub gph_config__stream_count_update_value: u8,
    pub global_config__stream_divider: u8,
    pub system__interrupt_config_gpio: u8,
    pub cal_config__vcsel_start: u8,
    pub cal_config__repeat_rate: u16,
    pub global_config__vcsel_width: u8,
    pub phasecal_config__timeout_macrop: u8,
    pub phasecal_config__target: u8,
    pub phasecal_config__override: u8,
    pub dss_config__roi_mode_control: u8,
    pub system__thresh_rate_high: u16,
    pub system__thresh_rate_low: u16,
    pub dss_config__manual_effective_spads_select: u16,
    pub dss_config__manual_block_select: u8,
    pub dss_config__aperture_attenuation: u8,
    pub dss_config__max_spads_limit: u8,
    pub dss_config__min_spads_limit: u8,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_timing_config_t {
    pub mm_config__timeout_macrop_a_hi: u8,
    pub mm_config__timeout_macrop_a_lo: u8,
    pub mm_config__timeout_macrop_b_hi: u8,
    pub mm_config__timeout_macrop_b_lo: u8,
    pub range_config__timeout_macrop_a_hi: u8,
    pub range_config__timeout_macrop_a_lo: u8,
    pub range_config__vcsel_period_a: u8,
    pub range_config__timeout_macrop_b_hi: u8,
    pub range_config__timeout_macrop_b_lo: u8,
    pub range_config__vcsel_period_b: u8,
    pub range_config__sigma_thresh: u16,
    pub range_config__min_count_rate_rtn_limit_mcps: u16,
    pub range_config__valid_phase_low: u8,
    pub range_config__valid_phase_high: u8,
    pub system__intermeasurement_period: u32,
    pub system__fractional_enable: u8,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_dynamic_config_t {
    pub system__grouped_parameter_hold_0: u8,
    pub system__thresh_high: u16,
    pub system__thresh_low: u16,
    pub system__enable_xtalk_per_quadrant: u8,
    pub system__seed_config: u8,
    pub sd_config__woi_sd0: u8,
    pub sd_config__woi_sd1: u8,
    pub sd_config__initial_phase_sd0: u8,
    pub sd_config__initial_phase_sd1: u8,
    pub system__grouped_parameter_hold_1: u8,
    pub sd_config__first_order_select: u8,
    pub sd_config__quantifier: u8,
    pub roi_config__user_roi_centre_spad: u8,
    pub roi_config__user_roi_requested_global_xy_size: u8,
    pub system__sequence_config: u8,
    pub system__grouped_parameter_hold: u8,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_system_control_t {
    pub power_management__go1_power_force: u8,
    pub system__stream_count_ctrl: u8,
    pub firmware__enable: u8,
    pub system__interrupt_clear: u8,
    pub system__mode_start: u8,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_system_results_t {
    pub result__interrupt_status: u8,
    pub result__range_status: u8,
    pub result__report_status: u8,
    pub result__stream_count: u8,
    pub result__dss_actual_effective_spads_sd0: u16,
    pub result__peak_signal_count_rate_mcps_sd0: u16,
    pub result__ambient_count_rate_mcps_sd0: u16,
    pub result__sigma_sd0: u16,
    pub result__phase_sd0: u16,
    pub result__final_crosstalk_corrected_range_mm_sd0: u16,
    pub result__peak_signal_count_rate_crosstalk_corrected_mcps_sd0: u16,
    pub result__mm_inner_actual_effective_spads_sd0: u16,
    pub result__mm_outer_actual_effective_spads_sd0: u16,
    pub result__avg_signal_count_rate_mcps_sd0: u16,
    pub result__dss_actual_effective_spads_sd1: u16,
    pub result__peak_signal_count_rate_mcps_sd1: u16,
    pub result__ambient_count_rate_mcps_sd1: u16,
    pub result__sigma_sd1: u16,
    pub result__phase_sd1: u16,
    pub result__final_crosstalk_corrected_range_mm_sd1: u16,
    pub result__spare_0_sd1: u16,
    pub result__spare_1_sd1: u16,
    pub result__spare_2_sd1: u16,
    pub result__spare_3_sd1: u8,
    pub result__thresh_info: u8,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_core_results_t {
    pub result_core__ambient_window_events_sd0: u32,
    pub result_core__ranging_total_events_sd0: u32,
    pub result_core__signal_total_events_sd0: i32,
    pub result_core__total_periods_elapsed_sd0: u32,
    pub result_core__ambient_window_events_sd1: u32,
    pub result_core__ranging_total_events_sd1: u32,
    pub result_core__signal_total_events_sd1: i32,
    pub result_core__total_periods_elapsed_sd1: u32,
    pub result_core__spare_0: u8,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_debug_results_t {
    pub phasecal_result__reference_phase: u16,
    pub phasecal_result__vcsel_start: u8,
    pub ref_spad_char_result__num_actual_ref_spads: u8,
    pub ref_spad_char_result__ref_location: u8,
    pub vhv_result__coldboot_status: u8,
    pub vhv_result__search_result: u8,
    pub vhv_result__latest_setting: u8,
    pub result__osc_calibrate_val: u16,
    pub ana_config__powerdown_go1: u8,
    pub ana_config__ref_bg_ctrl: u8,
    pub ana_config__regdvdd1v2_ctrl: u8,
    pub ana_config__osc_slow_ctrl: u8,
    pub test_mode__status: u8,
    pub firmware__system_status: u8,
    pub firmware__mode_status: u8,
    pub firmware__secondary_mode_status: u8,
    pub firmware__cal_repeat_rate_counter: u16,
    pub gph__system__thresh_high: u16,
    pub gph__system__thresh_low: u16,
    pub gph__system__enable_xtalk_per_quadrant: u8,
    pub gph__spare_0: u8,
    pub gph__sd_config__woi_sd0: u8,
    pub gph__sd_config__woi_sd1: u8,
    pub gph__sd_config__initial_phase_sd0: u8,
    pub gph__sd_config__initial_phase_sd1: u8,
    pub gph__sd_config__first_order_select: u8,
    pub gph__sd_config__quantifier: u8,
    pub gph__roi_config__user_roi_centre_spad: u8,
    pub gph__roi_config__user_roi_requested_global_xy_size: u8,
    pub gph__system__sequence_config: u8,
    pub gph__gph_id: u8,
    pub system__interrupt_set: u8,
    pub interrupt_manager__enables: u8,
    pub interrupt_manager__clear: u8,
    pub interrupt_manager__status: u8,
    pub mcu_to_host_bank__wr_access_en: u8,
    pub power_management__go1_reset_status: u8,
    pub pad_startup_mode__value_ro: u8,
    pub pad_startup_mode__value_ctrl: u8,
    pub pll_period_us: u32,
    pub interrupt_scheduler__data_out: u32,
    pub nvm_bist__complete: u8,
    pub nvm_bist__status: u8,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_nvm_copy_data_t {
    pub identification__model_id: u8,
    pub identification__module_type: u8,
    pub identification__revision_id: u8,
    pub identification__module_id: u16,
    pub ana_config__fast_osc__trim_max: u8,
    pub ana_config__fast_osc__freq_set: u8,
    pub ana_config__vcsel_trim: u8,
    pub ana_config__vcsel_selion: u8,
    pub ana_config__vcsel_selion_max: u8,
    pub protected_laser_safety__lock_bit: u8,
    pub laser_safety__key: u8,
    pub laser_safety__key_ro: u8,
    pub laser_safety__clip: u8,
    pub laser_safety__mult: u8,
    pub global_config__spad_enables_rtn_0: u8,
    pub global_config__spad_enables_rtn_1: u8,
    pub global_config__spad_enables_rtn_2: u8,
    pub global_config__spad_enables_rtn_3: u8,
    pub global_config__spad_enables_rtn_4: u8,
    pub global_config__spad_enables_rtn_5: u8,
    pub global_config__spad_enables_rtn_6: u8,
    pub global_config__spad_enables_rtn_7: u8,
    pub global_config__spad_enables_rtn_8: u8,
    pub global_config__spad_enables_rtn_9: u8,
    pub global_config__spad_enables_rtn_10: u8,
    pub global_config__spad_enables_rtn_11: u8,
    pub global_config__spad_enables_rtn_12: u8,
    pub global_config__spad_enables_rtn_13: u8,
    pub global_config__spad_enables_rtn_14: u8,
    pub global_config__spad_enables_rtn_15: u8,
    pub global_config__spad_enables_rtn_16: u8,
    pub global_config__spad_enables_rtn_17: u8,
    pub global_config__spad_enables_rtn_18: u8,
    pub global_config__spad_enables_rtn_19: u8,
    pub global_config__spad_enables_rtn_20: u8,
    pub global_config__spad_enables_rtn_21: u8,
    pub global_config__spad_enables_rtn_22: u8,
    pub global_config__spad_enables_rtn_23: u8,
    pub global_config__spad_enables_rtn_24: u8,
    pub global_config__spad_enables_rtn_25: u8,
    pub global_config__spad_enables_rtn_26: u8,
    pub global_config__spad_enables_rtn_27: u8,
    pub global_config__spad_enables_rtn_28: u8,
    pub global_config__spad_enables_rtn_29: u8,
    pub global_config__spad_enables_rtn_30: u8,
    pub global_config__spad_enables_rtn_31: u8,
    pub roi_config__mode_roi_centre_spad: u8,
    pub roi_config__mode_roi_xy_size: u8,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_prev_shadow_system_results_t {
    pub prev_shadow_result__interrupt_status: u8,
    pub prev_shadow_result__range_status: u8,
    pub prev_shadow_result__report_status: u8,
    pub prev_shadow_result__stream_count: u8,
    pub prev_shadow_result__dss_actual_effective_spads_sd0: u16,
    pub prev_shadow_result__peak_signal_count_rate_mcps_sd0: u16,
    pub prev_shadow_result__ambient_count_rate_mcps_sd0: u16,
    pub prev_shadow_result__sigma_sd0: u16,
    pub prev_shadow_result__phase_sd0: u16,
    pub prev_shadow_result__final_crosstalk_corrected_range_mm_sd0: u16,
    pub psr__peak_signal_count_rate_crosstalk_corrected_mcps_sd0: u16,
    pub prev_shadow_result__mm_inner_actual_effective_spads_sd0: u16,
    pub prev_shadow_result__mm_outer_actual_effective_spads_sd0: u16,
    pub prev_shadow_result__avg_signal_count_rate_mcps_sd0: u16,
    pub prev_shadow_result__dss_actual_effective_spads_sd1: u16,
    pub prev_shadow_result__peak_signal_count_rate_mcps_sd1: u16,
    pub prev_shadow_result__ambient_count_rate_mcps_sd1: u16,
    pub prev_shadow_result__sigma_sd1: u16,
    pub prev_shadow_result__phase_sd1: u16,
    pub prev_shadow_result__final_crosstalk_corrected_range_mm_sd1: u16,
    pub prev_shadow_result__spare_0_sd1: u16,
    pub prev_shadow_result__spare_1_sd1: u16,
    pub prev_shadow_result__spare_2_sd1: u16,
    pub prev_shadow_result__spare_3_sd1: u16,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_prev_shadow_core_results_t {
    pub prev_shadow_result_core__ambient_window_events_sd0: u32,
    pub prev_shadow_result_core__ranging_total_events_sd0: u32,
    pub prev_shadow_result_core__signal_total_events_sd0: i32,
    pub prev_shadow_result_core__total_periods_elapsed_sd0: u32,
    pub prev_shadow_result_core__ambient_window_events_sd1: u32,
    pub prev_shadow_result_core__ranging_total_events_sd1: u32,
    pub prev_shadow_result_core__signal_total_events_sd1: i32,
    pub prev_shadow_result_core__total_periods_elapsed_sd1: u32,
    pub prev_shadow_result_core__spare_0: u8,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_patch_debug_t {
    pub result__debug_status: u8,
    pub result__debug_stage: u8,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_gph_general_config_t {
    pub gph__system__thresh_rate_high: u16,
    pub gph__system__thresh_rate_low: u16,
    pub gph__system__interrupt_config_gpio: u8,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_gph_static_config_t {
    pub gph__dss_config__roi_mode_control: u8,
    pub gph__dss_config__manual_effective_spads_select: u16,
    pub gph__dss_config__manual_block_select: u8,
    pub gph__dss_config__max_spads_limit: u8,
    pub gph__dss_config__min_spads_limit: u8,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_gph_timing_config_t {
    pub gph__mm_config__timeout_macrop_a_hi: u8,
    pub gph__mm_config__timeout_macrop_a_lo: u8,
    pub gph__mm_config__timeout_macrop_b_hi: u8,
    pub gph__mm_config__timeout_macrop_b_lo: u8,
    pub gph__range_config__timeout_macrop_a_hi: u8,
    pub gph__range_config__timeout_macrop_a_lo: u8,
    pub gph__range_config__vcsel_period_a: u8,
    pub gph__range_config__vcsel_period_b: u8,
    pub gph__range_config__timeout_macrop_b_hi: u8,
    pub gph__range_config__timeout_macrop_b_lo: u8,
    pub gph__range_config__sigma_thresh: u16,
    pub gph__range_config__min_count_rate_rtn_limit_mcps: u16,
    pub gph__range_config__valid_phase_low: u8,
    pub gph__range_config__valid_phase_high: u8,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_fw_internal_t {
    pub firmware__internal_stream_count_div: u8,
    pub firmware__internal_stream_counter_val: u8,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_patch_results_t {
    pub dss_calc__roi_ctrl: u8,
    pub dss_calc__spare_1: u8,
    pub dss_calc__spare_2: u8,
    pub dss_calc__spare_3: u8,
    pub dss_calc__spare_4: u8,
    pub dss_calc__spare_5: u8,
    pub dss_calc__spare_6: u8,
    pub dss_calc__spare_7: u8,
    pub dss_calc__user_roi_spad_en_0: u8,
    pub dss_calc__user_roi_spad_en_1: u8,
    pub dss_calc__user_roi_spad_en_2: u8,
    pub dss_calc__user_roi_spad_en_3: u8,
    pub dss_calc__user_roi_spad_en_4: u8,
    pub dss_calc__user_roi_spad_en_5: u8,
    pub dss_calc__user_roi_spad_en_6: u8,
    pub dss_calc__user_roi_spad_en_7: u8,
    pub dss_calc__user_roi_spad_en_8: u8,
    pub dss_calc__user_roi_spad_en_9: u8,
    pub dss_calc__user_roi_spad_en_10: u8,
    pub dss_calc__user_roi_spad_en_11: u8,
    pub dss_calc__user_roi_spad_en_12: u8,
    pub dss_calc__user_roi_spad_en_13: u8,
    pub dss_calc__user_roi_spad_en_14: u8,
    pub dss_calc__user_roi_spad_en_15: u8,
    pub dss_calc__user_roi_spad_en_16: u8,
    pub dss_calc__user_roi_spad_en_17: u8,
    pub dss_calc__user_roi_spad_en_18: u8,
    pub dss_calc__user_roi_spad_en_19: u8,
    pub dss_calc__user_roi_spad_en_20: u8,
    pub dss_calc__user_roi_spad_en_21: u8,
    pub dss_calc__user_roi_spad_en_22: u8,
    pub dss_calc__user_roi_spad_en_23: u8,
    pub dss_calc__user_roi_spad_en_24: u8,
    pub dss_calc__user_roi_spad_en_25: u8,
    pub dss_calc__user_roi_spad_en_26: u8,
    pub dss_calc__user_roi_spad_en_27: u8,
    pub dss_calc__user_roi_spad_en_28: u8,
    pub dss_calc__user_roi_spad_en_29: u8,
    pub dss_calc__user_roi_spad_en_30: u8,
    pub dss_calc__user_roi_spad_en_31: u8,
    pub dss_calc__user_roi_0: u8,
    pub dss_calc__user_roi_1: u8,
    pub dss_calc__mode_roi_0: u8,
    pub dss_calc__mode_roi_1: u8,
    pub sigma_estimator_calc__spare_0: u8,
    pub vhv_result__peak_signal_rate_mcps: u16,
    pub vhv_result__signal_total_events_ref: u32,
    pub phasecal_result__phase_output_ref: u16,
    pub dss_result__total_rate_per_spad: u16,
    pub dss_result__enabled_blocks: u8,
    pub dss_result__num_requested_spads: u16,
    pub mm_result__inner_intersection_rate: u16,
    pub mm_result__outer_complement_rate: u16,
    pub mm_result__total_offset: u16,
    pub xtalk_calc__xtalk_for_enabled_spads: u32,
    pub xtalk_result__avg_xtalk_user_roi_kcps: u32,
    pub xtalk_result__avg_xtalk_mm_inner_roi_kcps: u32,
    pub xtalk_result__avg_xtalk_mm_outer_roi_kcps: u32,
    pub range_result__accum_phase: u32,
    pub range_result__offset_corrected_range: u16,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_shadow_system_results_t {
    pub shadow_phasecal_result__vcsel_start: u8,
    pub shadow_result__interrupt_status: u8,
    pub shadow_result__range_status: u8,
    pub shadow_result__report_status: u8,
    pub shadow_result__stream_count: u8,
    pub shadow_result__dss_actual_effective_spads_sd0: u16,
    pub shadow_result__peak_signal_count_rate_mcps_sd0: u16,
    pub shadow_result__ambient_count_rate_mcps_sd0: u16,
    pub shadow_result__sigma_sd0: u16,
    pub shadow_result__phase_sd0: u16,
    pub shadow_result__final_crosstalk_corrected_range_mm_sd0: u16,
    pub shr__peak_signal_count_rate_crosstalk_corrected_mcps_sd0: u16,
    pub shadow_result__mm_inner_actual_effective_spads_sd0: u16,
    pub shadow_result__mm_outer_actual_effective_spads_sd0: u16,
    pub shadow_result__avg_signal_count_rate_mcps_sd0: u16,
    pub shadow_result__dss_actual_effective_spads_sd1: u16,
    pub shadow_result__peak_signal_count_rate_mcps_sd1: u16,
    pub shadow_result__ambient_count_rate_mcps_sd1: u16,
    pub shadow_result__sigma_sd1: u16,
    pub shadow_result__phase_sd1: u16,
    pub shadow_result__final_crosstalk_corrected_range_mm_sd1: u16,
    pub shadow_result__spare_0_sd1: u16,
    pub shadow_result__spare_1_sd1: u16,
    pub shadow_result__spare_2_sd1: u16,
    pub shadow_result__spare_3_sd1: u8,
    pub shadow_result__thresh_info: u8,
    pub shadow_phasecal_result__reference_phase_hi: u8,
    pub shadow_phasecal_result__reference_phase_lo: u8,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_shadow_core_results_t {
    pub shadow_result_core__ambient_window_events_sd0: u32,
    pub shadow_result_core__ranging_total_events_sd0: u32,
    pub shadow_result_core__signal_total_events_sd0: i32,
    pub shadow_result_core__total_periods_elapsed_sd0: u32,
    pub shadow_result_core__ambient_window_events_sd1: u32,
    pub shadow_result_core__ranging_total_events_sd1: u32,
    pub shadow_result_core__signal_total_events_sd1: i32,
    pub shadow_result_core__total_periods_elapsed_sd1: u32,
    pub shadow_result_core__spare_0: u8,
}
pub type VL53LX_WaitMethod = u8;
pub type VL53LX_DeviceState = u8;
pub type VL53LX_DeviceZonePreset = u8;
pub type VL53LX_DevicePresetModes = u8;
pub type VL53LX_DeviceMeasurementModes = u8;
pub type VL53LX_OffsetCalibrationMode = u8;
pub type VL53LX_OffsetCorrectionMode = u8;
pub type VL53LX_DeviceDmaxMode = u8;
pub type VL53LX_DeviceInterruptPolarity = u8;
pub type VL53LX_DeviceGpioMode = u8;
pub type VL53LX_DeviceError = u8;
pub type VL53LX_DeviceReportStatus = u8;
pub type VL53LX_DeviceDssMode = u8;
pub type VL53LX_HistAlgoSelect = u8;
pub type VL53LX_HistTargetOrder = u8;
pub type VL53LX_HistAmbEstMethod = u8;
pub type VL53LX_DeviceConfigLevel = u8;
pub type VL53LX_DeviceResultsLevel = u8;
pub type VL53LX_DeviceTestMode = u8;
pub type VL53LX_DeviceSscArray = u8;
pub type VL53LX_ZoneConfig_BinConfig_select = u8;
pub type VL53LX_GPIO_Interrupt_Mode = u8;
pub type VL53LX_TuningParms = u16;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_dmax_calibration_data_t {
    pub ref__actual_effective_spads: u16,
    pub ref__peak_signal_count_rate_mcps: u16,
    pub ref__distance_mm: u16,
    pub ref_reflectance_pc: u16,
    pub coverglass_transmission: u16,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_hist_gen3_dmax_config_t {
    pub signal_thresh_sigma: u8,
    pub ambient_thresh_sigma: u8,
    pub min_ambient_thresh_events: i32,
    pub signal_total_events_limit: i32,
    pub target_reflectance_for_dmax_calc: [u16; 5],
    pub max_effective_spads: u16,
    pub dss_config__target_total_rate_mcps: u16,
    pub dss_config__aperture_attenuation: u8,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_histogram_config_t {
    pub histogram_config__spad_array_selection: u8,
    pub histogram_config__low_amb_even_bin_0_1: u8,
    pub histogram_config__low_amb_even_bin_2_3: u8,
    pub histogram_config__low_amb_even_bin_4_5: u8,
    pub histogram_config__low_amb_odd_bin_0_1: u8,
    pub histogram_config__low_amb_odd_bin_2_3: u8,
    pub histogram_config__low_amb_odd_bin_4_5: u8,
    pub histogram_config__mid_amb_even_bin_0_1: u8,
    pub histogram_config__mid_amb_even_bin_2_3: u8,
    pub histogram_config__mid_amb_even_bin_4_5: u8,
    pub histogram_config__mid_amb_odd_bin_0_1: u8,
    pub histogram_config__mid_amb_odd_bin_2: u8,
    pub histogram_config__mid_amb_odd_bin_3_4: u8,
    pub histogram_config__mid_amb_odd_bin_5: u8,
    pub histogram_config__user_bin_offset: u8,
    pub histogram_config__high_amb_even_bin_0_1: u8,
    pub histogram_config__high_amb_even_bin_2_3: u8,
    pub histogram_config__high_amb_even_bin_4_5: u8,
    pub histogram_config__high_amb_odd_bin_0_1: u8,
    pub histogram_config__high_amb_odd_bin_2_3: u8,
    pub histogram_config__high_amb_odd_bin_4_5: u8,
    pub histogram_config__amb_thresh_low: u16,
    pub histogram_config__amb_thresh_high: u16,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_hist_post_process_config_t {
    pub hist_algo_select: VL53LX_HistAlgoSelect,
    pub hist_target_order: VL53LX_HistTargetOrder,
    pub filter_woi0: u8,
    pub filter_woi1: u8,
    pub hist_amb_est_method: VL53LX_HistAmbEstMethod,
    pub ambient_thresh_sigma0: u8,
    pub ambient_thresh_sigma1: u8,
    pub ambient_thresh_events_scaler: u16,
    pub min_ambient_thresh_events: i32,
    pub noise_threshold: u16,
    pub signal_total_events_limit: i32,
    pub sigma_estimator__sigma_ref_mm: u8,
    pub sigma_thresh: u16,
    pub range_offset_mm: i16,
    pub gain_factor: u16,
    pub valid_phase_low: u8,
    pub valid_phase_high: u8,
    pub algo__consistency_check__phase_tolerance: u8,
    pub algo__consistency_check__event_sigma: u8,
    pub algo__consistency_check__event_min_spad_count: u16,
    pub algo__consistency_check__min_max_tolerance: u16,
    pub algo__crosstalk_compensation_enable: u8,
    pub algo__crosstalk_compensation_plane_offset_kcps: u32,
    pub algo__crosstalk_compensation_x_plane_gradient_kcps: i16,
    pub algo__crosstalk_compensation_y_plane_gradient_kcps: i16,
    pub algo__crosstalk_detect_min_valid_range_mm: i16,
    pub algo__crosstalk_detect_max_valid_range_mm: i16,
    pub algo__crosstalk_detect_max_valid_rate_kcps: u16,
    pub algo__crosstalk_detect_max_sigma_mm: u16,
    pub algo__crosstalk_detect_event_sigma: u8,
    pub algo__crosstalk_detect_min_max_tolerance: u16,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_histogram_bin_data_t {
    pub cfg_device_state: VL53LX_DeviceState,
    pub rd_device_state: VL53LX_DeviceState,
    pub zone_id: u8,
    pub time_stamp: u32,
    pub VL53LX_p_019: u8,
    pub VL53LX_p_020: u8,
    pub VL53LX_p_021: u8,
    pub number_of_ambient_bins: u8,
    pub bin_seq: [u8; 6],
    pub bin_rep: [u8; 6],
    pub bin_data: [i32; 24],
    pub result__interrupt_status: u8,
    pub result__range_status: u8,
    pub result__report_status: u8,
    pub result__stream_count: u8,
    pub result__dss_actual_effective_spads: u16,
    pub phasecal_result__reference_phase: u16,
    pub phasecal_result__vcsel_start: u8,
    pub cal_config__vcsel_start: u8,
    pub vcsel_width: u16,
    pub VL53LX_p_005: u8,
    pub VL53LX_p_015: u16,
    pub total_periods_elapsed: u32,
    pub peak_duration_us: u32,
    pub woi_duration_us: u32,
    pub min_bin_value: i32,
    pub max_bin_value: i32,
    pub zero_distance_phase: u16,
    pub number_of_ambient_samples: u8,
    pub ambient_events_sum: i32,
    pub VL53LX_p_028: i32,
    pub roi_config__user_roi_centre_spad: u8,
    pub roi_config__user_roi_requested_global_xy_size: u8,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_xtalk_histogram_shape_t {
    pub zone_id: u8,
    pub time_stamp: u32,
    pub VL53LX_p_019: u8,
    pub VL53LX_p_020: u8,
    pub VL53LX_p_021: u8,
    pub bin_data: [u32; 12],
    pub phasecal_result__reference_phase: u16,
    pub phasecal_result__vcsel_start: u8,
    pub cal_config__vcsel_start: u8,
    pub vcsel_width: u16,
    pub VL53LX_p_015: u16,
    pub zero_distance_phase: u16,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_xtalk_histogram_data_t {
    pub xtalk_shape: VL53LX_xtalk_histogram_shape_t,
    pub xtalk_hist_removed: VL53LX_histogram_bin_data_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_ll_version_t {
    pub ll_revision: u32,
    pub ll_major: u8,
    pub ll_minor: u8,
    pub ll_build: u8,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_refspadchar_config_t {
    pub device_test_mode: u8,
    pub VL53LX_p_005: u8,
    pub timeout_us: u32,
    pub target_count_rate_mcps: u16,
    pub min_count_rate_limit_mcps: u16,
    pub max_count_rate_limit_mcps: u16,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_xtalkextract_config_t {
    pub dss_config__target_total_rate_mcps: u16,
    pub phasecal_config_timeout_us: u32,
    pub mm_config_timeout_us: u32,
    pub range_config_timeout_us: u32,
    pub num_of_samples: u8,
    pub algo__crosstalk_extract_min_valid_range_mm: i16,
    pub algo__crosstalk_extract_max_valid_range_mm: i16,
    pub algo__crosstalk_extract_max_valid_rate_kcps: u16,
    pub algo__crosstalk_extract_max_sigma_mm: u16,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_offsetcal_config_t {
    pub dss_config__target_total_rate_mcps: u16,
    pub phasecal_config_timeout_us: u32,
    pub range_config_timeout_us: u32,
    pub mm_config_timeout_us: u32,
    pub pre_num_of_samples: u8,
    pub mm1_num_of_samples: u8,
    pub mm2_num_of_samples: u8,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_zonecal_config_t {
    pub dss_config__target_total_rate_mcps: u16,
    pub phasecal_config_timeout_us: u32,
    pub mm_config_timeout_us: u32,
    pub range_config_timeout_us: u32,
    pub phasecal_num_of_samples: u16,
    pub zone_num_of_samples: u16,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_ssc_config_t {
    pub array_select: VL53LX_DeviceSscArray,
    pub VL53LX_p_005: u8,
    pub vcsel_start: u8,
    pub vcsel_width: u8,
    pub timeout_us: u32,
    pub rate_limit_mcps: u16,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_xtalk_config_t {
    pub algo__crosstalk_compensation_plane_offset_kcps: u32,
    pub algo__crosstalk_compensation_x_plane_gradient_kcps: i16,
    pub algo__crosstalk_compensation_y_plane_gradient_kcps: i16,
    pub nvm_default__crosstalk_compensation_plane_offset_kcps: u32,
    pub nvm_default__crosstalk_compensation_x_plane_gradient_kcps: i16,
    pub nvm_default__crosstalk_compensation_y_plane_gradient_kcps: i16,
    pub global_crosstalk_compensation_enable: u8,
    pub histogram_mode_crosstalk_margin_kcps: i16,
    pub lite_mode_crosstalk_margin_kcps: i16,
    pub crosstalk_range_ignore_threshold_mult: u8,
    pub crosstalk_range_ignore_threshold_rate_mcps: u16,
    pub algo__crosstalk_detect_min_valid_range_mm: i16,
    pub algo__crosstalk_detect_max_valid_range_mm: i16,
    pub algo__crosstalk_detect_max_valid_rate_kcps: u16,
    pub algo__crosstalk_detect_max_sigma_mm: u16,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_tuning_parm_storage_t {
    pub tp_tuning_parm_version: u16,
    pub tp_tuning_parm_key_table_version: u16,
    pub tp_tuning_parm_lld_version: u16,
    pub tp_init_phase_rtn_lite_long: u8,
    pub tp_init_phase_rtn_lite_med: u8,
    pub tp_init_phase_rtn_lite_short: u8,
    pub tp_init_phase_ref_lite_long: u8,
    pub tp_init_phase_ref_lite_med: u8,
    pub tp_init_phase_ref_lite_short: u8,
    pub tp_init_phase_rtn_hist_long: u8,
    pub tp_init_phase_rtn_hist_med: u8,
    pub tp_init_phase_rtn_hist_short: u8,
    pub tp_init_phase_ref_hist_long: u8,
    pub tp_init_phase_ref_hist_med: u8,
    pub tp_init_phase_ref_hist_short: u8,
    pub tp_consistency_lite_phase_tolerance: u8,
    pub tp_phasecal_target: u8,
    pub tp_cal_repeat_rate: u16,
    pub tp_lite_min_clip: u8,
    pub tp_lite_long_sigma_thresh_mm: u16,
    pub tp_lite_med_sigma_thresh_mm: u16,
    pub tp_lite_short_sigma_thresh_mm: u16,
    pub tp_lite_long_min_count_rate_rtn_mcps: u16,
    pub tp_lite_med_min_count_rate_rtn_mcps: u16,
    pub tp_lite_short_min_count_rate_rtn_mcps: u16,
    pub tp_lite_sigma_est_pulse_width_ns: u8,
    pub tp_lite_sigma_est_amb_width_ns: u8,
    pub tp_lite_sigma_ref_mm: u8,
    pub tp_lite_seed_cfg: u8,
    pub tp_timed_seed_cfg: u8,
    pub tp_lite_quantifier: u8,
    pub tp_lite_first_order_select: u8,
    pub tp_dss_target_lite_mcps: u16,
    pub tp_dss_target_histo_mcps: u16,
    pub tp_dss_target_histo_mz_mcps: u16,
    pub tp_dss_target_timed_mcps: u16,
    pub tp_dss_target_very_short_mcps: u16,
    pub tp_phasecal_timeout_lite_us: u32,
    pub tp_phasecal_timeout_hist_long_us: u32,
    pub tp_phasecal_timeout_hist_med_us: u32,
    pub tp_phasecal_timeout_hist_short_us: u32,
    pub tp_phasecal_timeout_mz_long_us: u32,
    pub tp_phasecal_timeout_mz_med_us: u32,
    pub tp_phasecal_timeout_mz_short_us: u32,
    pub tp_phasecal_timeout_timed_us: u32,
    pub tp_mm_timeout_lite_us: u32,
    pub tp_mm_timeout_histo_us: u32,
    pub tp_mm_timeout_mz_us: u32,
    pub tp_mm_timeout_timed_us: u32,
    pub tp_mm_timeout_lpa_us: u32,
    pub tp_range_timeout_lite_us: u32,
    pub tp_range_timeout_histo_us: u32,
    pub tp_range_timeout_mz_us: u32,
    pub tp_range_timeout_timed_us: u32,
    pub tp_range_timeout_lpa_us: u32,
    pub tp_phasecal_patch_power: u32,
    pub tp_hist_merge: u8,
    pub tp_reset_merge_threshold: u32,
    pub tp_hist_merge_max_size: u8,
    pub tp_uwr_enable: u8,
    pub tp_uwr_med_z_1_min: i16,
    pub tp_uwr_med_z_1_max: i16,
    pub tp_uwr_med_z_2_min: i16,
    pub tp_uwr_med_z_2_max: i16,
    pub tp_uwr_med_z_3_min: i16,
    pub tp_uwr_med_z_3_max: i16,
    pub tp_uwr_med_z_4_min: i16,
    pub tp_uwr_med_z_4_max: i16,
    pub tp_uwr_med_z_5_min: i16,
    pub tp_uwr_med_z_5_max: i16,
    pub tp_uwr_med_corr_z_1_rangea: i16,
    pub tp_uwr_med_corr_z_1_rangeb: i16,
    pub tp_uwr_med_corr_z_2_rangea: i16,
    pub tp_uwr_med_corr_z_2_rangeb: i16,
    pub tp_uwr_med_corr_z_3_rangea: i16,
    pub tp_uwr_med_corr_z_3_rangeb: i16,
    pub tp_uwr_med_corr_z_4_rangea: i16,
    pub tp_uwr_med_corr_z_4_rangeb: i16,
    pub tp_uwr_med_corr_z_5_rangea: i16,
    pub tp_uwr_med_corr_z_5_rangeb: i16,
    pub tp_uwr_lng_z_1_min: i16,
    pub tp_uwr_lng_z_1_max: i16,
    pub tp_uwr_lng_z_2_min: i16,
    pub tp_uwr_lng_z_2_max: i16,
    pub tp_uwr_lng_z_3_min: i16,
    pub tp_uwr_lng_z_3_max: i16,
    pub tp_uwr_lng_z_4_min: i16,
    pub tp_uwr_lng_z_4_max: i16,
    pub tp_uwr_lng_z_5_min: i16,
    pub tp_uwr_lng_z_5_max: i16,
    pub tp_uwr_lng_corr_z_1_rangea: i16,
    pub tp_uwr_lng_corr_z_1_rangeb: i16,
    pub tp_uwr_lng_corr_z_2_rangea: i16,
    pub tp_uwr_lng_corr_z_2_rangeb: i16,
    pub tp_uwr_lng_corr_z_3_rangea: i16,
    pub tp_uwr_lng_corr_z_3_rangeb: i16,
    pub tp_uwr_lng_corr_z_4_rangea: i16,
    pub tp_uwr_lng_corr_z_4_rangeb: i16,
    pub tp_uwr_lng_corr_z_5_rangea: i16,
    pub tp_uwr_lng_corr_z_5_rangeb: i16,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_optical_centre_t {
    pub x_centre: u8,
    pub y_centre: u8,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_user_zone_t {
    pub x_centre: u8,
    pub y_centre: u8,
    pub width: u8,
    pub height: u8,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_zone_config_t {
    pub max_zones: u8,
    pub active_zones: u8,
    pub multizone_hist_cfg: VL53LX_histogram_config_t,
    pub user_zones: [VL53LX_user_zone_t; 5],
    pub bin_config: [u8; 5],
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_GPIO_interrupt_config_t {
    pub intr_mode_distance: VL53LX_GPIO_Interrupt_Mode,
    pub intr_mode_rate: VL53LX_GPIO_Interrupt_Mode,
    pub intr_new_measure_ready: u8,
    pub intr_no_target: u8,
    pub intr_combined_mode: u8,
    pub threshold_distance_high: u16,
    pub threshold_distance_low: u16,
    pub threshold_rate_high: u16,
    pub threshold_rate_low: u16,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_low_power_auto_data_t {
    pub vhv_loop_bound: u8,
    pub is_low_power_auto_mode: u8,
    pub low_power_auto_range_count: u8,
    pub saved_interrupt_config: u8,
    pub saved_vhv_init: u8,
    pub saved_vhv_timeout: u8,
    pub first_run_phasecal_result: u8,
    pub dss__total_rate_per_spad_mcps: u32,
    pub dss__required_spads: u16,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_smudge_corrector_config_t {
    pub smudge_corr_enabled: u8,
    pub smudge_corr_apply_enabled: u8,
    pub smudge_corr_single_apply: u8,
    pub smudge_margin: u16,
    pub noise_margin: u32,
    pub user_xtalk_offset_limit: u32,
    pub user_xtalk_offset_limit_hi: u8,
    pub sample_limit: u32,
    pub single_xtalk_delta: u32,
    pub averaged_xtalk_delta: u32,
    pub smudge_corr_clip_limit: u32,
    pub smudge_corr_ambient_threshold: u32,
    pub scaler_calc_method: u8,
    pub x_gradient_scaler: i16,
    pub y_gradient_scaler: i16,
    pub user_scaler_set: u8,
    pub nodetect_ambient_threshold: u32,
    pub nodetect_sample_limit: u32,
    pub nodetect_xtalk_offset: u32,
    pub nodetect_min_range_mm: u16,
    pub max_smudge_factor: u32,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_smudge_corrector_internals_t {
    pub current_samples: u32,
    pub required_samples: u32,
    pub accumulator: u64,
    pub nodetect_counter: u32,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_smudge_corrector_data_t {
    pub smudge_corr_valid: u8,
    pub smudge_corr_clipped: u8,
    pub single_xtalk_delta_flag: u8,
    pub averaged_xtalk_delta_flag: u8,
    pub sample_limit_exceeded_flag: u8,
    pub gradient_zero_flag: u8,
    pub new_xtalk_applied_flag: u8,
    pub algo__crosstalk_compensation_plane_offset_kcps: u32,
    pub algo__crosstalk_compensation_x_plane_gradient_kcps: i16,
    pub algo__crosstalk_compensation_y_plane_gradient_kcps: i16,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_range_data_t {
    pub range_id: u8,
    pub time_stamp: u32,
    pub VL53LX_p_012: u8,
    pub VL53LX_p_019: u8,
    pub VL53LX_p_023: u8,
    pub VL53LX_p_024: u8,
    pub VL53LX_p_013: u8,
    pub VL53LX_p_025: u8,
    pub width: u16,
    pub VL53LX_p_029: u8,
    pub fast_osc_frequency: u16,
    pub zero_distance_phase: u16,
    pub VL53LX_p_004: u16,
    pub total_periods_elapsed: u32,
    pub peak_duration_us: u32,
    pub woi_duration_us: u32,
    pub VL53LX_p_016: u32,
    pub VL53LX_p_017: u32,
    pub VL53LX_p_010: i32,
    pub peak_signal_count_rate_mcps: u16,
    pub avg_signal_count_rate_mcps: u16,
    pub ambient_count_rate_mcps: u16,
    pub total_rate_per_spad_mcps: u16,
    pub VL53LX_p_009: u32,
    pub VL53LX_p_002: u16,
    pub VL53LX_p_026: u16,
    pub VL53LX_p_011: u16,
    pub VL53LX_p_027: u16,
    pub min_range_mm: i16,
    pub median_range_mm: i16,
    pub max_range_mm: i16,
    pub range_status: u8,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_range_results_t {
    pub cfg_device_state: VL53LX_DeviceState,
    pub rd_device_state: VL53LX_DeviceState,
    pub zone_id: u8,
    pub stream_count: u8,
    pub VL53LX_p_022: [i16; 5],
    pub wrap_dmax_mm: i16,
    pub device_status: u8,
    pub max_results: u8,
    pub active_results: u8,
    pub VL53LX_p_003: [VL53LX_range_data_t; 4],
    pub xmonitor: VL53LX_range_data_t,
    pub smudge_corrector_data: VL53LX_smudge_corrector_data_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_xtalk_range_data_t {
    pub no_of_samples: u8,
    pub rate_per_spad_kcps_sum: u32,
    pub rate_per_spad_kcps_avg: u32,
    pub signal_total_events_sum: i32,
    pub signal_total_events_avg: i32,
    pub sigma_mm_sum: u32,
    pub sigma_mm_avg: u32,
    pub median_phase_sum: u32,
    pub median_phase_avg: u32,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_xtalk_range_results_t {
    pub cal_status: VL53LX_Error,
    pub num_of_samples_status: u8,
    pub zero_samples_status: u8,
    pub max_sigma_status: u8,
    pub max_results: u8,
    pub active_results: u8,
    pub VL53LX_p_003: [VL53LX_xtalk_range_data_t; 5],
    pub central_histogram_sum: VL53LX_histogram_bin_data_t,
    pub central_histogram_avg: VL53LX_histogram_bin_data_t,
    pub central_histogram__window_start: u8,
    pub central_histogram__window_end: u8,
    pub histogram_avg_1: [VL53LX_histogram_bin_data_t; 5],
    pub histogram_avg_2: [VL53LX_histogram_bin_data_t; 5],
    pub xtalk_avg: [VL53LX_histogram_bin_data_t; 5],
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_offset_range_data_t {
    pub preset_mode: u8,
    pub dss_config__roi_mode_control: u8,
    pub dss_config__manual_effective_spads_select: u16,
    pub no_of_samples: u8,
    pub effective_spads: u32,
    pub peak_rate_mcps: u32,
    pub VL53LX_p_002: u32,
    pub median_range_mm: i32,
    pub range_mm_offset: i32,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_offset_range_results_t {
    pub cal_distance_mm: i16,
    pub cal_reflectance_pc: u16,
    pub cal_status: VL53LX_Error,
    pub cal_report: u8,
    pub max_results: u8,
    pub active_results: u8,
    pub VL53LX_p_003: [VL53LX_offset_range_data_t; 3],
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_additional_offset_cal_data_t {
    pub result__mm_inner_actual_effective_spads: u16,
    pub result__mm_outer_actual_effective_spads: u16,
    pub result__mm_inner_peak_signal_count_rtn_mcps: u16,
    pub result__mm_outer_peak_signal_count_rtn_mcps: u16,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_per_vcsel_period_offset_cal_data_t {
    pub short_a_offset_mm: i16,
    pub short_b_offset_mm: i16,
    pub medium_a_offset_mm: i16,
    pub medium_b_offset_mm: i16,
    pub long_a_offset_mm: i16,
    pub long_b_offset_mm: i16,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_object_data_t {
    pub VL53LX_p_016: u32,
    pub VL53LX_p_017: u32,
    pub VL53LX_p_011: u16,
    pub range_status: u8,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_zone_objects_t {
    pub cfg_device_state: VL53LX_DeviceState,
    pub rd_device_state: VL53LX_DeviceState,
    pub zone_id: u8,
    pub stream_count: u8,
    pub max_objects: u8,
    pub active_objects: u8,
    pub VL53LX_p_003: [VL53LX_object_data_t; 4],
    pub xmonitor: VL53LX_object_data_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_zone_results_t {
    pub max_zones: u8,
    pub active_zones: u8,
    pub VL53LX_p_003: [VL53LX_zone_objects_t; 5],
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_zone_hist_info_t {
    pub rd_device_state: VL53LX_DeviceState,
    pub number_of_ambient_bins: u8,
    pub result__dss_actual_effective_spads: u16,
    pub VL53LX_p_005: u8,
    pub total_periods_elapsed: u32,
    pub ambient_events_sum: i32,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_zone_histograms_t {
    pub max_zones: u8,
    pub active_zones: u8,
    pub VL53LX_p_003: [VL53LX_zone_hist_info_t; 5],
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_zone_calibration_data_t {
    pub no_of_samples: u32,
    pub effective_spads: u32,
    pub peak_rate_mcps: u32,
    pub VL53LX_p_011: u32,
    pub VL53LX_p_002: u32,
    pub median_range_mm: i32,
    pub range_mm_offset: i32,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_zone_calibration_results_t {
    pub struct_version: u32,
    pub preset_mode: VL53LX_DevicePresetModes,
    pub zone_preset: VL53LX_DeviceZonePreset,
    pub cal_distance_mm: i16,
    pub cal_reflectance_pc: u16,
    pub phasecal_result__reference_phase: u16,
    pub zero_distance_phase: u16,
    pub cal_status: VL53LX_Error,
    pub max_zones: u8,
    pub active_zones: u8,
    pub VL53LX_p_003: [VL53LX_zone_calibration_data_t; 5],
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_cal_peak_rate_map_t {
    pub cal_distance_mm: i16,
    pub cal_reflectance_pc: u16,
    pub max_samples: u16,
    pub width: u16,
    pub height: u16,
    pub peak_rate_mcps: [u16; 25],
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_zone_private_dyn_cfg_t {
    pub expected_stream_count: u8,
    pub expected_gph_id: u8,
    pub dss_mode: u8,
    pub dss_requested_effective_spad_count: u16,
    pub seed_cfg: u8,
    pub initial_phase_seed: u8,
    pub roi_config__user_roi_centre_spad: u8,
    pub roi_config__user_roi_requested_global_xy_size: u8,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_zone_private_dyn_cfgs_t {
    pub max_zones: u8,
    pub active_zones: u8,
    pub VL53LX_p_003: [VL53LX_zone_private_dyn_cfg_t; 5],
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_xtalk_calibration_results_t {
    pub algo__crosstalk_compensation_plane_offset_kcps: u32,
    pub algo__crosstalk_compensation_x_plane_gradient_kcps: i16,
    pub algo__crosstalk_compensation_y_plane_gradient_kcps: i16,
    pub algo__xtalk_cpo_HistoMerge_kcps: [u32; 6],
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_hist_xtalk_extract_data_t {
    pub sample_count: u32,
    pub pll_period_mm: u32,
    pub peak_duration_us_sum: u32,
    pub effective_spad_count_sum: u32,
    pub zero_distance_phase_sum: u32,
    pub zero_distance_phase_avg: u32,
    pub event_scaler_sum: i32,
    pub event_scaler_avg: i32,
    pub signal_events_sum: i32,
    pub xtalk_rate_kcps_per_spad: u32,
    pub xtalk_start_phase: i32,
    pub xtalk_end_phase: i32,
    pub xtalk_width_phase: i32,
    pub target_start_phase: i32,
    pub target_end_phase: i32,
    pub target_width_phase: i32,
    pub effective_width: i32,
    pub event_scaler: i32,
    pub VL53LX_p_012: u8,
    pub VL53LX_p_013: u8,
    pub target_start: u8,
    pub max_shape_value: i32,
    pub bin_data_sums: [i32; 12],
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_gain_calibration_data_t {
    pub standard_ranging_gain_factor: u16,
    pub histogram_ranging_gain_factor: u16,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_ll_driver_state_t {
    pub cfg_device_state: VL53LX_DeviceState,
    pub cfg_stream_count: u8,
    pub cfg_internal_stream_count: u8,
    pub cfg_internal_stream_count_val: u8,
    pub cfg_gph_id: u8,
    pub cfg_timing_status: u8,
    pub cfg_zone_id: u8,
    pub rd_device_state: VL53LX_DeviceState,
    pub rd_stream_count: u8,
    pub rd_internal_stream_count: u8,
    pub rd_internal_stream_count_val: u8,
    pub rd_gph_id: u8,
    pub rd_timing_status: u8,
    pub rd_zone_id: u8,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_LLDriverData_t {
    pub wait_method: u8,
    pub preset_mode: VL53LX_DevicePresetModes,
    pub zone_preset: VL53LX_DeviceZonePreset,
    pub measurement_mode: VL53LX_DeviceMeasurementModes,
    pub offset_calibration_mode: VL53LX_OffsetCalibrationMode,
    pub offset_correction_mode: VL53LX_OffsetCorrectionMode,
    pub dmax_mode: VL53LX_DeviceDmaxMode,
    pub phasecal_config_timeout_us: u32,
    pub mm_config_timeout_us: u32,
    pub range_config_timeout_us: u32,
    pub inter_measurement_period_ms: u32,
    pub dss_config__target_total_rate_mcps: u16,
    pub fw_ready_poll_duration_ms: u32,
    pub fw_ready: u8,
    pub debug_mode: u8,
    pub version: VL53LX_ll_version_t,
    pub ll_state: VL53LX_ll_driver_state_t,
    pub gpio_interrupt_config: VL53LX_GPIO_interrupt_config_t,
    pub customer: VL53LX_customer_nvm_managed_t,
    pub cal_peak_rate_map: VL53LX_cal_peak_rate_map_t,
    pub add_off_cal_data: VL53LX_additional_offset_cal_data_t,
    pub fmt_dmax_cal: VL53LX_dmax_calibration_data_t,
    pub cust_dmax_cal: VL53LX_dmax_calibration_data_t,
    pub gain_cal: VL53LX_gain_calibration_data_t,
    pub mm_roi: VL53LX_user_zone_t,
    pub optical_centre: VL53LX_optical_centre_t,
    pub zone_cfg: VL53LX_zone_config_t,
    pub tuning_parms: VL53LX_tuning_parm_storage_t,
    pub rtn_good_spads: [u8; 32],
    pub refspadchar: VL53LX_refspadchar_config_t,
    pub ssc_cfg: VL53LX_ssc_config_t,
    pub histpostprocess: VL53LX_hist_post_process_config_t,
    pub dmax_cfg: VL53LX_hist_gen3_dmax_config_t,
    pub xtalk_extract_cfg: VL53LX_xtalkextract_config_t,
    pub xtalk_cfg: VL53LX_xtalk_config_t,
    pub offsetcal_cfg: VL53LX_offsetcal_config_t,
    pub zonecal_cfg: VL53LX_zonecal_config_t,
    pub stat_nvm: VL53LX_static_nvm_managed_t,
    pub hist_cfg: VL53LX_histogram_config_t,
    pub stat_cfg: VL53LX_static_config_t,
    pub gen_cfg: VL53LX_general_config_t,
    pub tim_cfg: VL53LX_timing_config_t,
    pub dyn_cfg: VL53LX_dynamic_config_t,
    pub sys_ctrl: VL53LX_system_control_t,
    pub sys_results: VL53LX_system_results_t,
    pub nvm_copy_data: VL53LX_nvm_copy_data_t,
    pub hist_data: VL53LX_histogram_bin_data_t,
    pub hist_xtalk: VL53LX_histogram_bin_data_t,
    pub xtalk_shapes: VL53LX_xtalk_histogram_data_t,
    pub xtalk_results: VL53LX_xtalk_range_results_t,
    pub xtalk_cal: VL53LX_xtalk_calibration_results_t,
    pub xtalk_extract: VL53LX_hist_xtalk_extract_data_t,
    pub offset_results: VL53LX_offset_range_results_t,
    pub core_results: VL53LX_core_results_t,
    pub dbg_results: VL53LX_debug_results_t,
    pub smudge_correct_config: VL53LX_smudge_corrector_config_t,
    pub smudge_corrector_internals: VL53LX_smudge_corrector_internals_t,
    pub low_power_auto_data: VL53LX_low_power_auto_data_t,
    pub wArea1: [u8; 1536],
    pub wArea2: [u8; 512],
    pub per_vcsel_cal_data: VL53LX_per_vcsel_period_offset_cal_data_t,
    pub bin_rec_pos: u8,
    pub pos_before_next_recom: u8,
    pub multi_bins_rec: [[[i32; 24]; 2]; 6],
    pub PreviousRangeMilliMeter: [i16; 4],
    pub PreviousRangeStatus: [u8; 4],
    pub PreviousExtendedRange: [u8; 4],
    pub PreviousRangeActiveResults: u8,
    pub PreviousStreamCount: u8,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_LLDriverResults_t {
    pub range_results: VL53LX_range_results_t,
    pub zone_dyn_cfgs: VL53LX_zone_private_dyn_cfgs_t,
    pub zone_results: VL53LX_zone_results_t,
    pub zone_hists: VL53LX_zone_histograms_t,
    pub zone_cal: VL53LX_zone_calibration_results_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_calibration_data_t {
    pub struct_version: u32,
    pub customer: VL53LX_customer_nvm_managed_t,
    pub fmt_dmax_cal: VL53LX_dmax_calibration_data_t,
    pub cust_dmax_cal: VL53LX_dmax_calibration_data_t,
    pub add_off_cal_data: VL53LX_additional_offset_cal_data_t,
    pub optical_centre: VL53LX_optical_centre_t,
    pub xtalkhisto: VL53LX_xtalk_histogram_data_t,
    pub gain_cal: VL53LX_gain_calibration_data_t,
    pub cal_peak_rate_map: VL53LX_cal_peak_rate_map_t,
    pub per_vcsel_cal_data: VL53LX_per_vcsel_period_offset_cal_data_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_xtalk_debug_data_t {
    pub customer: VL53LX_customer_nvm_managed_t,
    pub xtalk_extract_cfg: VL53LX_xtalkextract_config_t,
    pub xtalk_cfg: VL53LX_xtalk_config_t,
    pub hist_data: VL53LX_histogram_bin_data_t,
    pub xtalk_shapes: VL53LX_xtalk_histogram_data_t,
    pub xtalk_results: VL53LX_xtalk_range_results_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_offset_debug_data_t {
    pub customer: VL53LX_customer_nvm_managed_t,
    pub fmt_dmax_cal: VL53LX_dmax_calibration_data_t,
    pub cust_dmax_cal: VL53LX_dmax_calibration_data_t,
    pub add_off_cal_data: VL53LX_additional_offset_cal_data_t,
    pub offset_results: VL53LX_offset_range_results_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_tuning_parameters_t {
    pub vl53lx_tuningparm_version: u16,
    pub vl53lx_tuningparm_key_table_version: u16,
    pub vl53lx_tuningparm_lld_version: u16,
    pub vl53lx_tuningparm_hist_algo_select: u8,
    pub vl53lx_tuningparm_hist_target_order: u8,
    pub vl53lx_tuningparm_hist_filter_woi_0: u8,
    pub vl53lx_tuningparm_hist_filter_woi_1: u8,
    pub vl53lx_tuningparm_hist_amb_est_method: u8,
    pub vl53lx_tuningparm_hist_amb_thresh_sigma_0: u8,
    pub vl53lx_tuningparm_hist_amb_thresh_sigma_1: u8,
    pub vl53lx_tuningparm_hist_min_amb_thresh_events: i32,
    pub vl53lx_tuningparm_hist_amb_events_scaler: u16,
    pub vl53lx_tuningparm_hist_noise_threshold: u16,
    pub vl53lx_tuningparm_hist_signal_total_events_limit: i32,
    pub vl53lx_tuningparm_hist_sigma_est_ref_mm: u8,
    pub vl53lx_tuningparm_hist_sigma_thresh_mm: u16,
    pub vl53lx_tuningparm_hist_gain_factor: u16,
    pub vl53lx_tuningparm_consistency_hist_phase_tolerance: u8,
    pub vl53lx_tuningparm_consistency_hist_min_max_tolerance_mm: u16,
    pub vl53lx_tuningparm_consistency_hist_event_sigma: u8,
    pub vl53lx_tuningparm_consistency_hist_event_sigma_min_spad_limit: u16,
    pub vl53lx_tuningparm_initial_phase_rtn_histo_long_range: u8,
    pub vl53lx_tuningparm_initial_phase_rtn_histo_med_range: u8,
    pub vl53lx_tuningparm_initial_phase_rtn_histo_short_range: u8,
    pub vl53lx_tuningparm_initial_phase_ref_histo_long_range: u8,
    pub vl53lx_tuningparm_initial_phase_ref_histo_med_range: u8,
    pub vl53lx_tuningparm_initial_phase_ref_histo_short_range: u8,
    pub vl53lx_tuningparm_xtalk_detect_min_valid_range_mm: i16,
    pub vl53lx_tuningparm_xtalk_detect_max_valid_range_mm: i16,
    pub vl53lx_tuningparm_xtalk_detect_max_sigma_mm: u16,
    pub vl53lx_tuningparm_xtalk_detect_min_max_tolerance: u16,
    pub vl53lx_tuningparm_xtalk_detect_max_valid_rate_kcps: u16,
    pub vl53lx_tuningparm_xtalk_detect_event_sigma: u8,
    pub vl53lx_tuningparm_hist_xtalk_margin_kcps: i16,
    pub vl53lx_tuningparm_consistency_lite_phase_tolerance: u8,
    pub vl53lx_tuningparm_phasecal_target: u8,
    pub vl53lx_tuningparm_lite_cal_repeat_rate: u16,
    pub vl53lx_tuningparm_lite_ranging_gain_factor: u16,
    pub vl53lx_tuningparm_lite_min_clip_mm: u8,
    pub vl53lx_tuningparm_lite_long_sigma_thresh_mm: u16,
    pub vl53lx_tuningparm_lite_med_sigma_thresh_mm: u16,
    pub vl53lx_tuningparm_lite_short_sigma_thresh_mm: u16,
    pub vl53lx_tuningparm_lite_long_min_count_rate_rtn_mcps: u16,
    pub vl53lx_tuningparm_lite_med_min_count_rate_rtn_mcps: u16,
    pub vl53lx_tuningparm_lite_short_min_count_rate_rtn_mcps: u16,
    pub vl53lx_tuningparm_lite_sigma_est_pulse_width: u8,
    pub vl53lx_tuningparm_lite_sigma_est_amb_width_ns: u8,
    pub vl53lx_tuningparm_lite_sigma_ref_mm: u8,
    pub vl53lx_tuningparm_lite_rit_mult: u8,
    pub vl53lx_tuningparm_lite_seed_config: u8,
    pub vl53lx_tuningparm_lite_quantifier: u8,
    pub vl53lx_tuningparm_lite_first_order_select: u8,
    pub vl53lx_tuningparm_lite_xtalk_margin_kcps: i16,
    pub vl53lx_tuningparm_initial_phase_rtn_lite_long_range: u8,
    pub vl53lx_tuningparm_initial_phase_rtn_lite_med_range: u8,
    pub vl53lx_tuningparm_initial_phase_rtn_lite_short_range: u8,
    pub vl53lx_tuningparm_initial_phase_ref_lite_long_range: u8,
    pub vl53lx_tuningparm_initial_phase_ref_lite_med_range: u8,
    pub vl53lx_tuningparm_initial_phase_ref_lite_short_range: u8,
    pub vl53lx_tuningparm_timed_seed_config: u8,
    pub vl53lx_tuningparm_dmax_cfg_signal_thresh_sigma: u8,
    pub vl53lx_tuningparm_dmax_cfg_reflectance_array_0: u16,
    pub vl53lx_tuningparm_dmax_cfg_reflectance_array_1: u16,
    pub vl53lx_tuningparm_dmax_cfg_reflectance_array_2: u16,
    pub vl53lx_tuningparm_dmax_cfg_reflectance_array_3: u16,
    pub vl53lx_tuningparm_dmax_cfg_reflectance_array_4: u16,
    pub vl53lx_tuningparm_vhv_loopbound: u8,
    pub vl53lx_tuningparm_refspadchar_device_test_mode: u8,
    pub vl53lx_tuningparm_refspadchar_vcsel_period: u8,
    pub vl53lx_tuningparm_refspadchar_phasecal_timeout_us: u32,
    pub vl53lx_tuningparm_refspadchar_target_count_rate_mcps: u16,
    pub vl53lx_tuningparm_refspadchar_min_countrate_limit_mcps: u16,
    pub vl53lx_tuningparm_refspadchar_max_countrate_limit_mcps: u16,
    pub vl53lx_tuningparm_xtalk_extract_num_of_samples: u8,
    pub vl53lx_tuningparm_xtalk_extract_min_filter_thresh_mm: i16,
    pub vl53lx_tuningparm_xtalk_extract_max_filter_thresh_mm: i16,
    pub vl53lx_tuningparm_xtalk_extract_dss_rate_mcps: u16,
    pub vl53lx_tuningparm_xtalk_extract_phasecal_timeout_us: u32,
    pub vl53lx_tuningparm_xtalk_extract_max_valid_rate_kcps: u16,
    pub vl53lx_tuningparm_xtalk_extract_sigma_threshold_mm: u16,
    pub vl53lx_tuningparm_xtalk_extract_dss_timeout_us: u32,
    pub vl53lx_tuningparm_xtalk_extract_bin_timeout_us: u32,
    pub vl53lx_tuningparm_offset_cal_dss_rate_mcps: u16,
    pub vl53lx_tuningparm_offset_cal_phasecal_timeout_us: u32,
    pub vl53lx_tuningparm_offset_cal_mm_timeout_us: u32,
    pub vl53lx_tuningparm_offset_cal_range_timeout_us: u32,
    pub vl53lx_tuningparm_offset_cal_pre_samples: u8,
    pub vl53lx_tuningparm_offset_cal_mm1_samples: u8,
    pub vl53lx_tuningparm_offset_cal_mm2_samples: u8,
    pub vl53lx_tuningparm_zone_cal_dss_rate_mcps: u16,
    pub vl53lx_tuningparm_zone_cal_phasecal_timeout_us: u32,
    pub vl53lx_tuningparm_zone_cal_dss_timeout_us: u32,
    pub vl53lx_tuningparm_zone_cal_phasecal_num_samples: u16,
    pub vl53lx_tuningparm_zone_cal_range_timeout_us: u32,
    pub vl53lx_tuningparm_zone_cal_zone_num_samples: u16,
    pub vl53lx_tuningparm_spadmap_vcsel_period: u8,
    pub vl53lx_tuningparm_spadmap_vcsel_start: u8,
    pub vl53lx_tuningparm_spadmap_rate_limit_mcps: u16,
    pub vl53lx_tuningparm_lite_dss_config_target_total_rate_mcps: u16,
    pub vl53lx_tuningparm_ranging_dss_config_target_total_rate_mcps: u16,
    pub vl53lx_tuningparm_mz_dss_config_target_total_rate_mcps: u16,
    pub vl53lx_tuningparm_timed_dss_config_target_total_rate_mcps: u16,
    pub vl53lx_tuningparm_lite_phasecal_config_timeout_us: u32,
    pub vl53lx_tuningparm_ranging_long_phasecal_config_timeout_us: u32,
    pub vl53lx_tuningparm_ranging_med_phasecal_config_timeout_us: u32,
    pub vl53lx_tuningparm_ranging_short_phasecal_config_timeout_us: u32,
    pub vl53lx_tuningparm_mz_long_phasecal_config_timeout_us: u32,
    pub vl53lx_tuningparm_mz_med_phasecal_config_timeout_us: u32,
    pub vl53lx_tuningparm_mz_short_phasecal_config_timeout_us: u32,
    pub vl53lx_tuningparm_timed_phasecal_config_timeout_us: u32,
    pub vl53lx_tuningparm_lite_mm_config_timeout_us: u32,
    pub vl53lx_tuningparm_ranging_mm_config_timeout_us: u32,
    pub vl53lx_tuningparm_mz_mm_config_timeout_us: u32,
    pub vl53lx_tuningparm_timed_mm_config_timeout_us: u32,
    pub vl53lx_tuningparm_lite_range_config_timeout_us: u32,
    pub vl53lx_tuningparm_ranging_range_config_timeout_us: u32,
    pub vl53lx_tuningparm_mz_range_config_timeout_us: u32,
    pub vl53lx_tuningparm_timed_range_config_timeout_us: u32,
    pub vl53lx_tuningparm_dynxtalk_smudge_margin: u16,
    pub vl53lx_tuningparm_dynxtalk_noise_margin: u32,
    pub vl53lx_tuningparm_dynxtalk_xtalk_offset_limit: u32,
    pub vl53lx_tuningparm_dynxtalk_xtalk_offset_limit_hi: u8,
    pub vl53lx_tuningparm_dynxtalk_sample_limit: u32,
    pub vl53lx_tuningparm_dynxtalk_single_xtalk_delta: u32,
    pub vl53lx_tuningparm_dynxtalk_averaged_xtalk_delta: u32,
    pub vl53lx_tuningparm_dynxtalk_clip_limit: u32,
    pub vl53lx_tuningparm_dynxtalk_scaler_calc_method: u8,
    pub vl53lx_tuningparm_dynxtalk_xgradient_scaler: i16,
    pub vl53lx_tuningparm_dynxtalk_ygradient_scaler: i16,
    pub vl53lx_tuningparm_dynxtalk_user_scaler_set: u8,
    pub vl53lx_tuningparm_dynxtalk_smudge_cor_single_apply: u8,
    pub vl53lx_tuningparm_dynxtalk_xtalk_amb_threshold: u32,
    pub vl53lx_tuningparm_dynxtalk_nodetect_amb_threshold_kcps: u32,
    pub vl53lx_tuningparm_dynxtalk_nodetect_sample_limit: u32,
    pub vl53lx_tuningparm_dynxtalk_nodetect_xtalk_offset_kcps: u32,
    pub vl53lx_tuningparm_dynxtalk_nodetect_min_range_mm: u16,
    pub vl53lx_tuningparm_lowpowerauto_vhv_loop_bound: u8,
    pub vl53lx_tuningparm_lowpowerauto_mm_config_timeout_us: u32,
    pub vl53lx_tuningparm_lowpowerauto_range_config_timeout_us: u32,
    pub vl53lx_tuningparm_very_short_dss_rate_mcps: u16,
    pub vl53lx_tuningparm_phasecal_patch_power: u32,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_spad_rate_data_t {
    pub spad_type: u8,
    pub VL53LX_p_020: u16,
    pub rate_data: [u16; 256],
    pub no_of_values: u16,
    pub fractional_bits: u8,
    pub error_status: u8,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_additional_data_t {
    pub preset_mode: VL53LX_DevicePresetModes,
    pub zone_preset: VL53LX_DeviceZonePreset,
    pub measurement_mode: VL53LX_DeviceMeasurementModes,
    pub offset_calibration_mode: VL53LX_OffsetCalibrationMode,
    pub offset_correction_mode: VL53LX_OffsetCorrectionMode,
    pub dmax_mode: VL53LX_DeviceDmaxMode,
    pub phasecal_config_timeout_us: u32,
    pub mm_config_timeout_us: u32,
    pub range_config_timeout_us: u32,
    pub inter_measurement_period_ms: u32,
    pub dss_config__target_total_rate_mcps: u16,
    pub VL53LX_p_006: VL53LX_histogram_bin_data_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_Version_t {
    pub revision: u32,
    pub major: u8,
    pub minor: u8,
    pub build: u8,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_DeviceInfo_t {
    pub ProductType: u8,
    pub ProductRevisionMajor: u8,
    pub ProductRevisionMinor: u8,
}
pub type VL53LX_DistanceModes = u8;
pub type VL53LX_OffsetCorrectionModes = u8;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_DeviceParameters_t {
    pub DistanceMode: VL53LX_DistanceModes,
    pub MeasurementTimingBudgetMicroSeconds: u32,
}
pub type VL53LX_SmudgeCorrectionModes = u8;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_TargetRangeData_t {
    pub RangeMaxMilliMeter: i16,
    pub RangeMinMilliMeter: i16,
    pub SignalRateRtnMegaCps: FixPoint1616_t,
    pub AmbientRateRtnMegaCps: FixPoint1616_t,
    pub SigmaMilliMeter: FixPoint1616_t,
    pub RangeMilliMeter: i16,
    pub RangeStatus: u8,
    pub ExtendedRange: u8,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_MultiRangingData_t {
    pub TimeStamp: u32,
    pub StreamCount: u8,
    pub NumberOfObjectsFound: u8,
    pub RangeData: [VL53LX_TargetRangeData_t; 4],
    pub HasXtalkValueChanged: u8,
    pub EffectiveSpadRtnCount: u16,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_UserRoi_t {
    pub TopLeftX: u8,
    pub TopLeftY: u8,
    pub BotRightX: u8,
    pub BotRightY: u8,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_CustomerNvmManaged_t {
    pub global_config__spad_enables_ref_0: u8,
    pub global_config__spad_enables_ref_1: u8,
    pub global_config__spad_enables_ref_2: u8,
    pub global_config__spad_enables_ref_3: u8,
    pub global_config__spad_enables_ref_4: u8,
    pub global_config__spad_enables_ref_5: u8,
    pub global_config__ref_en_start_select: u8,
    pub ref_spad_man__num_requested_ref_spads: u8,
    pub ref_spad_man__ref_location: u8,
    pub algo__crosstalk_compensation_plane_offset_kcps: u32,
    pub algo__crosstalk_compensation_x_plane_gradient_kcps: i16,
    pub algo__crosstalk_compensation_y_plane_gradient_kcps: i16,
    pub ref_spad_char__total_rate_target_mcps: u16,
    pub algo__part_to_part_range_offset_mm: i16,
    pub mm_config__inner_offset_mm: i16,
    pub mm_config__outer_offset_mm: i16,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_CalibrationData_t {
    pub struct_version: u32,
    pub customer: VL53LX_CustomerNvmManaged_t,
    pub add_off_cal_data: VL53LX_additional_offset_cal_data_t,
    pub optical_centre: VL53LX_optical_centre_t,
    pub xtalkhisto: VL53LX_xtalk_histogram_data_t,
    pub gain_cal: VL53LX_gain_calibration_data_t,
    pub cal_peak_rate_map: VL53LX_cal_peak_rate_map_t,
    pub per_vcsel_cal_data: VL53LX_per_vcsel_period_offset_cal_data_t,
    pub algo__xtalk_cpo_HistoMerge_kcps: [u32; 6],
}
pub type VL53LX_AdditionalData_t = VL53LX_additional_data_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_DevData_t {
    pub LLData: VL53LX_LLDriverData_t,
    pub llresults: VL53LX_LLDriverResults_t,
    pub CurrentParameters: VL53LX_DeviceParameters_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_Dev_t {
    pub Data: VL53LX_DevData_t,
    pub i2c_slave_address: u8,
    pub comms_type: u8,
    pub comms_speed_khz: u16,
    pub new_data_ready_poll_duration_ms: u32,
}
pub type VL53LX_DEV = *mut VL53LX_Dev_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_decoded_nvm_fmt_range_data_t {
    pub result__actual_effective_rtn_spads: u16,
    pub ref_spad_array__num_requested_ref_spads: u8,
    pub ref_spad_array__ref_location: u8,
    pub result__peak_signal_count_rate_rtn_mcps: u16,
    pub result__ambient_count_rate_rtn_mcps: u16,
    pub result__peak_signal_count_rate_ref_mcps: u16,
    pub result__ambient_count_rate_ref_mcps: u16,
    pub measured_distance_mm: u16,
    pub measured_distance_stdev_mm: u16,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_hist_gen3_dmax_private_data_t {
    pub VL53LX_p_037: u32,
    pub VL53LX_p_063: u8,
    pub VL53LX_p_064: u8,
    pub VL53LX_p_065: u16,
    pub VL53LX_p_066: u16,
    pub VL53LX_p_067: u16,
    pub VL53LX_p_038: u16,
    pub VL53LX_p_009: u32,
    pub VL53LX_p_033: u32,
    pub VL53LX_p_034: u16,
    pub VL53LX_p_004: u16,
    pub VL53LX_p_028: u32,
    pub VL53LX_p_035: u32,
    pub VL53LX_p_036: i16,
    pub VL53LX_p_022: i16,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_hist_gen4_algo_filtered_data_t {
    pub VL53LX_p_019: u8,
    pub VL53LX_p_020: u8,
    pub VL53LX_p_021: u8,
    pub VL53LX_p_007: [i32; 24],
    pub VL53LX_p_032: [i32; 24],
    pub VL53LX_p_001: [i32; 24],
    pub VL53LX_p_053: [i32; 24],
    pub VL53LX_p_054: [i32; 24],
    pub VL53LX_p_040: [u8; 24],
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_hist_gen3_algo_private_data_t {
    pub VL53LX_p_019: u8,
    pub VL53LX_p_020: u8,
    pub VL53LX_p_021: u8,
    pub VL53LX_p_030: u8,
    pub VL53LX_p_039: u8,
    pub VL53LX_p_028: i32,
    pub VL53LX_p_031: i32,
    pub VL53LX_p_040: [u8; 24],
    pub VL53LX_p_041: [u8; 24],
    pub VL53LX_p_042: [u8; 24],
    pub VL53LX_p_052: [i32; 24],
    pub VL53LX_p_043: [i32; 24],
    pub VL53LX_p_018: [i32; 24],
    pub VL53LX_p_044: u8,
    pub VL53LX_p_045: u8,
    pub VL53LX_p_046: u8,
    pub VL53LX_p_003: [VL53LX_hist_pulse_data_t; 8],
    pub VL53LX_p_006: VL53LX_histogram_bin_data_t,
    pub VL53LX_p_047: VL53LX_histogram_bin_data_t,
    pub VL53LX_p_048: VL53LX_histogram_bin_data_t,
    pub VL53LX_p_049: VL53LX_histogram_bin_data_t,
    pub VL53LX_p_050: VL53LX_histogram_bin_data_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_hist_pulse_data_t {
    pub VL53LX_p_012: u8,
    pub VL53LX_p_019: u8,
    pub VL53LX_p_023: u8,
    pub VL53LX_p_024: u8,
    pub VL53LX_p_013: u8,
    pub VL53LX_p_025: u8,
    pub VL53LX_p_051: u8,
    pub VL53LX_p_016: i32,
    pub VL53LX_p_017: i32,
    pub VL53LX_p_010: i32,
    pub VL53LX_p_026: u32,
    pub VL53LX_p_011: u32,
    pub VL53LX_p_027: u32,
    pub VL53LX_p_002: u16,
}
pub type VL53LX_Tuning_t = libc::c_uint;
pub const VL53LX_TUNING_MAX_TUNABLE_KEY: VL53LX_Tuning_t = 11;
pub const VL53LX_TUNING_ZERO_DISTANCE_OFFSET_NON_LINEAR_FACTOR: VL53LX_Tuning_t = 10;
pub const VL53LX_TUNING_XTALK_FULL_ROI_DEFAULT_OFFSET: VL53LX_Tuning_t = 9;
pub const VL53LX_TUNING_XTALK_FULL_ROI_BIN_SUM_MARGIN: VL53LX_Tuning_t = 8;
pub const VL53LX_TUNING_SIMPLE_OFFSET_CALIBRATION_REPEAT: VL53LX_Tuning_t = 7;
pub const VL53LX_TUNING_XTALK_FULL_ROI_TARGET_DISTANCE_MM: VL53LX_Tuning_t = 6;
pub const VL53LX_TUNING_MAX_SIMPLE_OFFSET_CALIBRATION_SAMPLE_NUMBER: VL53LX_Tuning_t = 5;
pub const VL53LX_TUNING_MIN_AMBIENT_DMAX_VALID: VL53LX_Tuning_t = 4;
pub const VL53LX_TUNING_SINGLE_TARGET_XTALK_SAMPLE_NUMBER: VL53LX_Tuning_t = 3;
pub const VL53LX_TUNING_SINGLE_TARGET_XTALK_TARGET_DISTANCE_MM: VL53LX_Tuning_t = 2;
pub const VL53LX_TUNING_PROXY_MIN: VL53LX_Tuning_t = 1;
pub const VL53LX_TUNING_VERSION: VL53LX_Tuning_t = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_decoded_nvm_fmt_info_t {
    pub nvm__fmt__fgc: [libc::c_char; 19],
    pub nvm__fmt__test_program_major: u8,
    pub nvm__fmt__test_program_minor: u8,
    pub nvm__fmt__map_major: u8,
    pub nvm__fmt__map_minor: u8,
    pub nvm__fmt__year: u8,
    pub nvm__fmt__month: u8,
    pub nvm__fmt__day: u8,
    pub nvm__fmt__module_date_phase: u8,
    pub nvm__fmt__time: u16,
    pub nvm__fmt__tester_id: u8,
    pub nvm__fmt__site_id: u8,
    pub nvm__ews__test_program_major: u8,
    pub nvm__ews__test_program_minor: u8,
    pub nvm__ews__probe_card_major: u8,
    pub nvm__ews__probe_card_minor: u8,
    pub nvm__ews__tester_id: u8,
    pub nvm__ews__lot: [libc::c_char; 8],
    pub nvm__ews__wafer: u8,
    pub nvm__ews__xcoord: u8,
    pub nvm__ews__ycoord: u8,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_decoded_nvm_ews_info_t {
    pub nvm__ews__test_program_major: u8,
    pub nvm__ews__test_program_minor: u8,
    pub nvm__ews__probe_card_major: u8,
    pub nvm__ews__probe_card_minor: u8,
    pub nvm__ews__tester_id: u8,
    pub nvm__ews__lot: [libc::c_char; 8],
    pub nvm__ews__wafer: u8,
    pub nvm__ews__xcoord: u8,
    pub nvm__ews__ycoord: u8,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_decoded_nvm_data_t {
    pub nvm__identification_model_id: u8,
    pub nvm__identification_module_type: u8,
    pub nvm__identification_revision_id: u8,
    pub nvm__identification_module_id: u16,
    pub nvm__i2c_valid: u8,
    pub nvm__i2c_device_address_ews: u8,
    pub nvm__ews__fast_osc_frequency: u16,
    pub nvm__ews__fast_osc_trim_max: u8,
    pub nvm__ews__fast_osc_freq_set: u8,
    pub nvm__ews__slow_osc_calibration: u16,
    pub nvm__fmt__fast_osc_frequency: u16,
    pub nvm__fmt__fast_osc_trim_max: u8,
    pub nvm__fmt__fast_osc_freq_set: u8,
    pub nvm__fmt__slow_osc_calibration: u16,
    pub nvm__vhv_config_unlock: u8,
    pub nvm__ref_selvddpix: u8,
    pub nvm__ref_selvquench: u8,
    pub nvm__regavdd1v2_sel: u8,
    pub nvm__regdvdd1v2_sel: u8,
    pub nvm__vhv_timeout__macrop: u8,
    pub nvm__vhv_loop_bound: u8,
    pub nvm__vhv_count_threshold: u8,
    pub nvm__vhv_offset: u8,
    pub nvm__vhv_init_enable: u8,
    pub nvm__vhv_init_value: u8,
    pub nvm__laser_safety_vcsel_trim_ll: u8,
    pub nvm__laser_safety_vcsel_selion_ll: u8,
    pub nvm__laser_safety_vcsel_selion_max_ll: u8,
    pub nvm__laser_safety_mult_ll: u8,
    pub nvm__laser_safety_clip_ll: u8,
    pub nvm__laser_safety_vcsel_trim_ld: u8,
    pub nvm__laser_safety_vcsel_selion_ld: u8,
    pub nvm__laser_safety_vcsel_selion_max_ld: u8,
    pub nvm__laser_safety_mult_ld: u8,
    pub nvm__laser_safety_clip_ld: u8,
    pub nvm__laser_safety_lock_byte: u8,
    pub nvm__laser_safety_unlock_byte: u8,
    pub nvm__ews__spad_enables_rtn: [u8; 32],
    pub nvm__ews__spad_enables_ref__loc1: [u8; 6],
    pub nvm__ews__spad_enables_ref__loc2: [u8; 6],
    pub nvm__ews__spad_enables_ref__loc3: [u8; 6],
    pub nvm__fmt__spad_enables_rtn: [u8; 32],
    pub nvm__fmt__spad_enables_ref__loc1: [u8; 6],
    pub nvm__fmt__spad_enables_ref__loc2: [u8; 6],
    pub nvm__fmt__spad_enables_ref__loc3: [u8; 6],
    pub nvm__fmt__roi_config__mode_roi_centre_spad: u8,
    pub nvm__fmt__roi_config__mode_roi_x_size: u8,
    pub nvm__fmt__roi_config__mode_roi_y_size: u8,
    pub nvm__fmt__ref_spad_apply__num_requested_ref_spad: u8,
    pub nvm__fmt__ref_spad_man__ref_location: u8,
    pub nvm__fmt__mm_config__inner_offset_mm: u16,
    pub nvm__fmt__mm_config__outer_offset_mm: u16,
    pub nvm__fmt__algo_part_to_part_range_offset_mm: u16,
    pub nvm__fmt__algo__crosstalk_compensation_plane_offset_kcps: u16,
    pub nvm__fmt__algo__crosstalk_compensation_x_plane_gradient_kcps: u16,
    pub nvm__fmt__algo__crosstalk_compensation_y_plane_gradient_kcps: u16,
    pub nvm__fmt__spare__host_config__nvm_config_spare_0: u8,
    pub nvm__fmt__spare__host_config__nvm_config_spare_1: u8,
    pub nvm__customer_space_programmed: u8,
    pub nvm__cust__i2c_device_address: u8,
    pub nvm__cust__ref_spad_apply__num_requested_ref_spad: u8,
    pub nvm__cust__ref_spad_man__ref_location: u8,
    pub nvm__cust__mm_config__inner_offset_mm: u16,
    pub nvm__cust__mm_config__outer_offset_mm: u16,
    pub nvm__cust__algo_part_to_part_range_offset_mm: u16,
    pub nvm__cust__algo__crosstalk_compensation_plane_offset_kcps: u16,
    pub nvm__cust__algo__crosstalk_compensation_x_plane_gradient_kcps: u16,
    pub nvm__cust__algo__crosstalk_compensation_y_plane_gradient_kcps: u16,
    pub nvm__cust__spare__host_config__nvm_config_spare_0: u8,
    pub nvm__cust__spare__host_config__nvm_config_spare_1: u8,
    pub fmt_optical_centre: VL53LX_optical_centre_t,
    pub fmt_peak_rate_map: VL53LX_cal_peak_rate_map_t,
    pub fmt_add_offset_data: VL53LX_additional_offset_cal_data_t,
    pub fmt_range_data: [VL53LX_decoded_nvm_fmt_range_data_t; 4],
    pub fmt_info: VL53LX_decoded_nvm_fmt_info_t,
    pub ews_info: VL53LX_decoded_nvm_ews_info_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VL53LX_xtalk_algo_data_t {
    pub VL53LX_p_061: [u32; 4],
    pub VL53LX_p_059: i16,
    pub VL53LX_p_060: i16,
    pub VL53LX_p_056: VL53LX_histogram_bin_data_t,
    pub VL53LX_p_057: VL53LX_histogram_bin_data_t,
    pub VL53LX_p_058: u32,
    pub VL53LX_p_062: [u32; 12],
}
#[no_mangle]
pub unsafe extern "C" fn unimplemented() -> libc::c_int {
    return 0 as libc::c_int;
}
static mut BDTable: [i32; 11] = [
    0x7 as libc::c_int,
    -(30 as libc::c_int),
    600 as libc::c_int,
    50 as libc::c_int,
    8 as libc::c_int,
    10 as libc::c_int,
    600 as libc::c_int,
    3 as libc::c_int,
    24 as libc::c_int,
    50 as libc::c_int,
    9 as libc::c_int,
];
#[no_mangle]
pub unsafe extern "C" fn VL53LX_GetVersion(
    mut pVersion: *mut VL53LX_Version_t,
) -> VL53LX_Error {
    let mut Status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    (*pVersion).major = 1 as libc::c_int as u8;
    (*pVersion).minor = 2 as libc::c_int as u8;
    (*pVersion).build = 8 as libc::c_int as u8;
    (*pVersion).revision = 2578 as libc::c_int as u32;
    return Status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_GetProductRevision(
    mut Dev: VL53LX_DEV,
    mut pProductRevisionMajor: *mut u8,
    mut pProductRevisionMinor: *mut u8,
) -> VL53LX_Error {
    let mut Status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut revision_id: u8 = 0;
    let mut pLLData: *mut VL53LX_LLDriverData_t = 0 as *mut VL53LX_LLDriverData_t;
    pLLData = &mut (*Dev).Data.LLData;
    revision_id = (*pLLData).nvm_copy_data.identification__revision_id;
    *pProductRevisionMajor = 1 as libc::c_int as u8;
    *pProductRevisionMinor = ((revision_id as libc::c_int & 0xf0 as libc::c_int)
        >> 4 as libc::c_int) as u8;
    return Status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_GetDeviceInfo(
    mut Dev: VL53LX_DEV,
    mut pVL53LX_DeviceInfo: *mut VL53LX_DeviceInfo_t,
) -> VL53LX_Error {
    let mut Status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut revision_id: u8 = 0;
    let mut pLLData: *mut VL53LX_LLDriverData_t = 0 as *mut VL53LX_LLDriverData_t;
    pLLData = &mut (*Dev).Data.LLData;
    (*pVL53LX_DeviceInfo)
        .ProductType = (*pLLData).nvm_copy_data.identification__module_type;
    revision_id = (*pLLData).nvm_copy_data.identification__revision_id;
    (*pVL53LX_DeviceInfo).ProductRevisionMajor = 1 as libc::c_int as u8;
    (*pVL53LX_DeviceInfo)
        .ProductRevisionMinor = ((revision_id as libc::c_int & 0xf0 as libc::c_int)
        >> 4 as libc::c_int) as u8;
    return Status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_GetUID(
    mut Dev: VL53LX_DEV,
    mut pUid: *mut u64,
) -> VL53LX_Error {
    let mut Status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut fmtdata: [u8; 8] = [0; 8];
    Status = VL53LX_read_nvm_raw_data(
        Dev,
        (0x1f8 as libc::c_int >> 2 as libc::c_int) as u8,
        (8 as libc::c_int >> 2 as libc::c_int) as u8,
        fmtdata.as_mut_ptr(),
    );
    memcpy(
        pUid as *mut libc::c_void,
        fmtdata.as_mut_ptr() as *const libc::c_void,
        ::std::mem::size_of::<u64>() as libc::c_ulong,
    );
    return Status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_SetDeviceAddress(
    mut Dev: VL53LX_DEV,
    mut DeviceAddress: u8,
) -> VL53LX_Error {
    let mut Status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    let mut pdata: *mut VL53LX_static_nvm_managed_t = &mut (*pdev).stat_nvm;
    Status = VL53LX_WrByte(
        Dev,
        0x1 as libc::c_int as u16,
        (DeviceAddress as libc::c_int / 2 as libc::c_int) as u8,
    );
    (*pdata)
        .i2c_slave__device_address = (DeviceAddress as libc::c_int / 2 as libc::c_int
        & 0x7f as libc::c_int) as u8;
    return Status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_DataInit(mut Dev: VL53LX_DEV) -> VL53LX_Error {
    let mut Status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdev: *mut VL53LX_LLDriverData_t = 0 as *mut VL53LX_LLDriverData_t;
    let mut measurement_mode: u8 = 0;
    if Status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        Status = VL53LX_data_init(Dev, 1 as libc::c_int as u8);
    }
    if Status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        Status = SetPresetModeL3CX(
            Dev,
            2 as libc::c_int as VL53LX_DistanceModes,
            1000 as libc::c_int as u32,
        );
    }
    if Status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        Status = VL53LX_SetMeasurementTimingBudgetMicroSeconds(
            Dev,
            33333 as libc::c_int as u32,
        );
    }
    if Status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        pdev = &mut (*Dev).Data.LLData;
        memset(
            &mut (*pdev).per_vcsel_cal_data
                as *mut VL53LX_per_vcsel_period_offset_cal_data_t as *mut libc::c_void,
            0 as libc::c_int,
            ::std::mem::size_of::<VL53LX_per_vcsel_period_offset_cal_data_t>()
                as libc::c_ulong,
        );
    }
    if Status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        Status = VL53LX_set_dmax_mode(Dev, 2 as libc::c_int as VL53LX_DeviceDmaxMode);
    }
    if Status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        Status = VL53LX_SmudgeCorrectionEnable(
            Dev,
            0 as libc::c_int as VL53LX_SmudgeCorrectionModes,
        );
    }
    measurement_mode = 0x20 as libc::c_int as VL53LX_DeviceMeasurementModes;
    (*Dev).Data.LLData.measurement_mode = measurement_mode;
    (*Dev)
        .Data
        .CurrentParameters
        .DistanceMode = 2 as libc::c_int as VL53LX_DistanceModes;
    return Status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_WaitDeviceBooted(mut Dev: VL53LX_DEV) -> VL53LX_Error {
    let mut Status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    Status = VL53LX_poll_for_boot_completion(Dev, 500 as libc::c_int as u32);
    return Status;
}
unsafe extern "C" fn ComputeDevicePresetMode(
    mut DistanceMode: VL53LX_DistanceModes,
    mut pDevicePresetMode: *mut VL53LX_DevicePresetModes,
) -> VL53LX_Error {
    let mut Status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut DistIdx: u8 = 0;
    let mut RangingModes: [VL53LX_DevicePresetModes; 3] = [
        33 as libc::c_int as VL53LX_DevicePresetModes,
        30 as libc::c_int as VL53LX_DevicePresetModes,
        27 as libc::c_int as VL53LX_DevicePresetModes,
    ];
    match DistanceMode as libc::c_int {
        1 => {
            DistIdx = 0 as libc::c_int as u8;
        }
        2 => {
            DistIdx = 1 as libc::c_int as u8;
        }
        _ => {
            DistIdx = 2 as libc::c_int as u8;
        }
    }
    *pDevicePresetMode = RangingModes[DistIdx as usize];
    return Status;
}
unsafe extern "C" fn SetPresetModeL3CX(
    mut Dev: VL53LX_DEV,
    mut DistanceMode: VL53LX_DistanceModes,
    mut inter_measurement_period_ms: u32,
) -> VL53LX_Error {
    let mut Status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut device_preset_mode: VL53LX_DevicePresetModes = 0;
    let mut measurement_mode: u8 = 0;
    let mut dss_config__target_total_rate_mcps: u16 = 0 as libc::c_int as u16;
    let mut phasecal_config_timeout_us: u32 = 0 as libc::c_int as u32;
    let mut mm_config_timeout_us: u32 = 0 as libc::c_int as u32;
    let mut lld_range_config_timeout_us: u32 = 0 as libc::c_int as u32;
    measurement_mode = 0x20 as libc::c_int as VL53LX_DeviceMeasurementModes;
    Status = ComputeDevicePresetMode(DistanceMode, &mut device_preset_mode);
    if Status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        Status = VL53LX_get_preset_mode_timing_cfg(
            Dev,
            device_preset_mode,
            &mut dss_config__target_total_rate_mcps,
            &mut phasecal_config_timeout_us,
            &mut mm_config_timeout_us,
            &mut lld_range_config_timeout_us,
        );
    }
    if Status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        Status = VL53LX_set_preset_mode(
            Dev,
            device_preset_mode,
            dss_config__target_total_rate_mcps,
            phasecal_config_timeout_us,
            mm_config_timeout_us,
            lld_range_config_timeout_us,
            inter_measurement_period_ms,
        );
    }
    if Status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        (*Dev).Data.LLData.measurement_mode = measurement_mode;
    }
    return Status;
}
unsafe extern "C" fn IsL4(mut Dev: VL53LX_DEV) -> libc::c_int {
    let mut devL4: libc::c_int = 0 as libc::c_int;
    let mut pDev: *mut VL53LX_LLDriverData_t = 0 as *mut VL53LX_LLDriverData_t;
    pDev = &mut (*Dev).Data.LLData;
    if (*pDev).nvm_copy_data.identification__module_type as libc::c_int
        == 0xaa as libc::c_int
        && (*pDev).nvm_copy_data.identification__model_id as libc::c_int
            == 0xeb as libc::c_int
    {
        devL4 = 1 as libc::c_int;
    }
    return devL4;
}
unsafe extern "C" fn CheckValidRectRoi(mut ROI: VL53LX_UserRoi_t) -> VL53LX_Error {
    let mut Status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    if ROI.TopLeftX as libc::c_int > 15 as libc::c_int
        || ROI.TopLeftY as libc::c_int > 15 as libc::c_int
        || ROI.BotRightX as libc::c_int > 15 as libc::c_int
        || ROI.BotRightY as libc::c_int > 15 as libc::c_int
    {
        Status = -(4 as libc::c_int) as VL53LX_Error;
    }
    if ROI.TopLeftX as libc::c_int > ROI.BotRightX as libc::c_int
        || (ROI.TopLeftY as libc::c_int) < ROI.BotRightY as libc::c_int
    {
        Status = -(4 as libc::c_int) as VL53LX_Error;
    }
    return Status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_SetDistanceMode(
    mut Dev: VL53LX_DEV,
    mut DistanceMode: VL53LX_DistanceModes,
) -> VL53LX_Error {
    let mut Status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut inter_measurement_period_ms: u32 = 0;
    let mut TimingBudget: u32 = 0 as libc::c_int as u32;
    let mut MmTimeoutUs: u32 = 0 as libc::c_int as u32;
    let mut PhaseCalTimeoutUs: u32 = 0 as libc::c_int as u32;
    if DistanceMode as libc::c_int
        != 1 as libc::c_int as VL53LX_DistanceModes as libc::c_int
        && DistanceMode as libc::c_int
            != 2 as libc::c_int as VL53LX_DistanceModes as libc::c_int
        && DistanceMode as libc::c_int
            != 3 as libc::c_int as VL53LX_DistanceModes as libc::c_int
    {
        return -(4 as libc::c_int) as VL53LX_Error;
    }
    if IsL4(Dev) != 0
        && DistanceMode as libc::c_int
            == 1 as libc::c_int as VL53LX_DistanceModes as libc::c_int
    {
        return -(4 as libc::c_int) as VL53LX_Error;
    }
    inter_measurement_period_ms = (*Dev).Data.LLData.inter_measurement_period_ms;
    if Status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        Status = VL53LX_get_timeouts_us(
            Dev,
            &mut PhaseCalTimeoutUs,
            &mut MmTimeoutUs,
            &mut TimingBudget,
        );
    }
    if Status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        Status = SetPresetModeL3CX(Dev, DistanceMode, inter_measurement_period_ms);
    }
    if Status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        (*Dev).Data.CurrentParameters.DistanceMode = DistanceMode;
    }
    if Status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        Status = VL53LX_set_timeouts_us(
            Dev,
            PhaseCalTimeoutUs,
            MmTimeoutUs,
            TimingBudget,
        );
        if Status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
            (*Dev).Data.LLData.range_config_timeout_us = TimingBudget;
        }
    }
    return Status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_GetDistanceMode(
    mut Dev: VL53LX_DEV,
    mut pDistanceMode: *mut VL53LX_DistanceModes,
) -> VL53LX_Error {
    let mut Status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    *pDistanceMode = (*Dev).Data.CurrentParameters.DistanceMode;
    return Status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_SetMeasurementTimingBudgetMicroSeconds(
    mut Dev: VL53LX_DEV,
    mut MeasurementTimingBudgetMicroSeconds: u32,
) -> VL53LX_Error {
    let mut Status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut TimingGuard: u32 = 0;
    let mut divisor: u32 = 0;
    let mut TimingBudget: u32 = 0 as libc::c_int as u32;
    let mut MmTimeoutUs: u32 = 0 as libc::c_int as u32;
    let mut PhaseCalTimeoutUs: u32 = 0 as libc::c_int as u32;
    let mut FDAMaxTimingBudgetUs: u32 = 550000 as libc::c_int as u32;
    if MeasurementTimingBudgetMicroSeconds > 10000000 as libc::c_int as libc::c_uint {
        Status = -(4 as libc::c_int) as VL53LX_Error;
    }
    if Status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        Status = VL53LX_get_timeouts_us(
            Dev,
            &mut PhaseCalTimeoutUs,
            &mut MmTimeoutUs,
            &mut TimingBudget,
        );
    }
    TimingGuard = 1700 as libc::c_int as u32;
    divisor = 6 as libc::c_int as u32;
    if IsL4(Dev) != 0 {
        FDAMaxTimingBudgetUs = 200000 as libc::c_int as u32;
    }
    if MeasurementTimingBudgetMicroSeconds <= TimingGuard {
        Status = -(4 as libc::c_int) as VL53LX_Error;
    } else {
        TimingBudget = MeasurementTimingBudgetMicroSeconds.wrapping_sub(TimingGuard);
    }
    if Status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        if TimingBudget > FDAMaxTimingBudgetUs {
            Status = -(4 as libc::c_int) as VL53LX_Error;
        } else {
            TimingBudget = (TimingBudget as libc::c_uint).wrapping_div(divisor)
                as u32 as u32;
            Status = VL53LX_set_timeouts_us(
                Dev,
                PhaseCalTimeoutUs,
                MmTimeoutUs,
                TimingBudget,
            );
        }
        if Status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
            (*Dev).Data.LLData.range_config_timeout_us = TimingBudget;
        }
    }
    if Status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        (*Dev)
            .Data
            .CurrentParameters
            .MeasurementTimingBudgetMicroSeconds = MeasurementTimingBudgetMicroSeconds;
    }
    return Status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_GetMeasurementTimingBudgetMicroSeconds(
    mut Dev: VL53LX_DEV,
    mut pMeasurementTimingBudgetMicroSeconds: *mut u32,
) -> VL53LX_Error {
    let mut Status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut MmTimeoutUs: u32 = 0 as libc::c_int as u32;
    let mut RangeTimeoutUs: u32 = 0 as libc::c_int as u32;
    let mut PhaseCalTimeoutUs: u32 = 0 as libc::c_int as u32;
    *pMeasurementTimingBudgetMicroSeconds = 0 as libc::c_int as u32;
    if Status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        Status = VL53LX_get_timeouts_us(
            Dev,
            &mut PhaseCalTimeoutUs,
            &mut MmTimeoutUs,
            &mut RangeTimeoutUs,
        );
    }
    if Status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        *pMeasurementTimingBudgetMicroSeconds = (6 as libc::c_int as libc::c_uint)
            .wrapping_mul(RangeTimeoutUs)
            .wrapping_add(1700 as libc::c_int as libc::c_uint);
    }
    return Status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_SetUserROI(
    mut Dev: VL53LX_DEV,
    mut pRoi: *mut VL53LX_UserRoi_t,
) -> VL53LX_Error {
    let mut Status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut zone_cfg: VL53LX_zone_config_t = VL53LX_zone_config_t {
        max_zones: 0,
        active_zones: 0,
        multizone_hist_cfg: VL53LX_histogram_config_t {
            histogram_config__spad_array_selection: 0,
            histogram_config__low_amb_even_bin_0_1: 0,
            histogram_config__low_amb_even_bin_2_3: 0,
            histogram_config__low_amb_even_bin_4_5: 0,
            histogram_config__low_amb_odd_bin_0_1: 0,
            histogram_config__low_amb_odd_bin_2_3: 0,
            histogram_config__low_amb_odd_bin_4_5: 0,
            histogram_config__mid_amb_even_bin_0_1: 0,
            histogram_config__mid_amb_even_bin_2_3: 0,
            histogram_config__mid_amb_even_bin_4_5: 0,
            histogram_config__mid_amb_odd_bin_0_1: 0,
            histogram_config__mid_amb_odd_bin_2: 0,
            histogram_config__mid_amb_odd_bin_3_4: 0,
            histogram_config__mid_amb_odd_bin_5: 0,
            histogram_config__user_bin_offset: 0,
            histogram_config__high_amb_even_bin_0_1: 0,
            histogram_config__high_amb_even_bin_2_3: 0,
            histogram_config__high_amb_even_bin_4_5: 0,
            histogram_config__high_amb_odd_bin_0_1: 0,
            histogram_config__high_amb_odd_bin_2_3: 0,
            histogram_config__high_amb_odd_bin_4_5: 0,
            histogram_config__amb_thresh_low: 0,
            histogram_config__amb_thresh_high: 0,
        },
        user_zones: [VL53LX_user_zone_t {
            x_centre: 0,
            y_centre: 0,
            width: 0,
            height: 0,
        }; 5],
        bin_config: [0; 5],
    };
    let mut x_centre: u8 = 0;
    let mut y_centre: u8 = 0;
    let mut width: u8 = 0;
    let mut height: u8 = 0;
    Status = CheckValidRectRoi(*pRoi);
    if Status as libc::c_int != 0 as libc::c_int as VL53LX_Error as libc::c_int {
        return -(4 as libc::c_int) as VL53LX_Error;
    }
    x_centre = (((*pRoi).BotRightX as libc::c_int + (*pRoi).TopLeftX as libc::c_int
        + 1 as libc::c_int) / 2 as libc::c_int) as u8;
    y_centre = (((*pRoi).TopLeftY as libc::c_int + (*pRoi).BotRightY as libc::c_int
        + 1 as libc::c_int) / 2 as libc::c_int) as u8;
    width = ((*pRoi).BotRightX as libc::c_int - (*pRoi).TopLeftX as libc::c_int)
        as u8;
    height = ((*pRoi).TopLeftY as libc::c_int - (*pRoi).BotRightY as libc::c_int)
        as u8;
    zone_cfg.max_zones = 1 as libc::c_int as u8;
    zone_cfg.active_zones = 0 as libc::c_int as u8;
    zone_cfg.user_zones[0 as libc::c_int as usize].x_centre = x_centre;
    zone_cfg.user_zones[0 as libc::c_int as usize].y_centre = y_centre;
    zone_cfg.user_zones[0 as libc::c_int as usize].width = width;
    zone_cfg.user_zones[0 as libc::c_int as usize].height = height;
    if (width as libc::c_int) < 3 as libc::c_int
        || (height as libc::c_int) < 3 as libc::c_int
    {
        Status = -(4 as libc::c_int) as VL53LX_Error;
    } else {
        Status = VL53LX_set_zone_config(Dev, &mut zone_cfg);
    }
    return Status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_GetUserROI(
    mut Dev: VL53LX_DEV,
    mut pRoi: *mut VL53LX_UserRoi_t,
) -> VL53LX_Error {
    let mut Status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut zone_cfg: VL53LX_zone_config_t = VL53LX_zone_config_t {
        max_zones: 0,
        active_zones: 0,
        multizone_hist_cfg: VL53LX_histogram_config_t {
            histogram_config__spad_array_selection: 0,
            histogram_config__low_amb_even_bin_0_1: 0,
            histogram_config__low_amb_even_bin_2_3: 0,
            histogram_config__low_amb_even_bin_4_5: 0,
            histogram_config__low_amb_odd_bin_0_1: 0,
            histogram_config__low_amb_odd_bin_2_3: 0,
            histogram_config__low_amb_odd_bin_4_5: 0,
            histogram_config__mid_amb_even_bin_0_1: 0,
            histogram_config__mid_amb_even_bin_2_3: 0,
            histogram_config__mid_amb_even_bin_4_5: 0,
            histogram_config__mid_amb_odd_bin_0_1: 0,
            histogram_config__mid_amb_odd_bin_2: 0,
            histogram_config__mid_amb_odd_bin_3_4: 0,
            histogram_config__mid_amb_odd_bin_5: 0,
            histogram_config__user_bin_offset: 0,
            histogram_config__high_amb_even_bin_0_1: 0,
            histogram_config__high_amb_even_bin_2_3: 0,
            histogram_config__high_amb_even_bin_4_5: 0,
            histogram_config__high_amb_odd_bin_0_1: 0,
            histogram_config__high_amb_odd_bin_2_3: 0,
            histogram_config__high_amb_odd_bin_4_5: 0,
            histogram_config__amb_thresh_low: 0,
            histogram_config__amb_thresh_high: 0,
        },
        user_zones: [VL53LX_user_zone_t {
            x_centre: 0,
            y_centre: 0,
            width: 0,
            height: 0,
        }; 5],
        bin_config: [0; 5],
    };
    let mut TopLeftX: u8 = 0;
    let mut TopLeftY: u8 = 0;
    let mut BotRightX: u8 = 0;
    let mut BotRightY: u8 = 0;
    VL53LX_get_zone_config(Dev, &mut zone_cfg);
    TopLeftX = (2 as libc::c_int
        * zone_cfg.user_zones[0 as libc::c_int as usize].x_centre as libc::c_int
        - zone_cfg.user_zones[0 as libc::c_int as usize].width as libc::c_int
        >> 1 as libc::c_int) as u8;
    TopLeftY = (2 as libc::c_int
        * zone_cfg.user_zones[0 as libc::c_int as usize].y_centre as libc::c_int
        + zone_cfg.user_zones[0 as libc::c_int as usize].height as libc::c_int
        >> 1 as libc::c_int) as u8;
    BotRightX = (2 as libc::c_int
        * zone_cfg.user_zones[0 as libc::c_int as usize].x_centre as libc::c_int
        + zone_cfg.user_zones[0 as libc::c_int as usize].width as libc::c_int
        >> 1 as libc::c_int) as u8;
    BotRightY = (2 as libc::c_int
        * zone_cfg.user_zones[0 as libc::c_int as usize].y_centre as libc::c_int
        - zone_cfg.user_zones[0 as libc::c_int as usize].height as libc::c_int
        >> 1 as libc::c_int) as u8;
    (*pRoi).TopLeftX = TopLeftX;
    (*pRoi).TopLeftY = TopLeftY;
    (*pRoi).BotRightX = BotRightX;
    (*pRoi).BotRightY = BotRightY;
    return Status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_StartMeasurement(mut Dev: VL53LX_DEV) -> VL53LX_Error {
    let mut Status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut DeviceMeasurementMode: u8 = 0;
    let mut i: u8 = 0;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    VL53LX_load_patch(Dev);
    i = 0 as libc::c_int as u8;
    while (i as libc::c_int) < 4 as libc::c_int {
        (*pdev).PreviousRangeMilliMeter[i as usize] = 0 as libc::c_int as i16;
        (*pdev).PreviousRangeStatus[i as usize] = 255 as libc::c_int as u8;
        (*pdev).PreviousExtendedRange[i as usize] = 0 as libc::c_int as u8;
        i = i.wrapping_add(1);
    }
    (*pdev).PreviousStreamCount = 0 as libc::c_int as u8;
    (*pdev).PreviousRangeActiveResults = 0 as libc::c_int as u8;
    DeviceMeasurementMode = (*Dev).Data.LLData.measurement_mode;
    if Status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        Status = VL53LX_init_and_start_range(
            Dev,
            DeviceMeasurementMode,
            6 as libc::c_int as VL53LX_DeviceConfigLevel,
        );
    }
    return Status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_StopMeasurement(mut Dev: VL53LX_DEV) -> VL53LX_Error {
    let mut Status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    Status = VL53LX_stop_range(Dev);
    VL53LX_unload_patch(Dev);
    return Status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_ClearInterruptAndStartMeasurement(
    mut Dev: VL53LX_DEV,
) -> VL53LX_Error {
    let mut Status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut DeviceMeasurementMode: u8 = 0;
    DeviceMeasurementMode = (*Dev).Data.LLData.measurement_mode;
    Status = VL53LX_clear_interrupt_and_enable_next_range(Dev, DeviceMeasurementMode);
    return Status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_GetMeasurementDataReady(
    mut Dev: VL53LX_DEV,
    mut pMeasurementDataReady: *mut u8,
) -> VL53LX_Error {
    let mut Status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    Status = VL53LX_is_new_data_ready(Dev, pMeasurementDataReady);
    return Status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_WaitMeasurementDataReady(
    mut Dev: VL53LX_DEV,
) -> VL53LX_Error {
    let mut Status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    Status = VL53LX_poll_for_range_completion(Dev, 2000 as libc::c_int as u32);
    return Status;
}
unsafe extern "C" fn ConvertStatusHisto(mut FilteredRangeStatus: u8) -> u8 {
    let mut RangeStatus: u8 = 0;
    match FilteredRangeStatus as libc::c_int {
        5 => {
            RangeStatus = 4 as libc::c_int as u8;
        }
        6 => {
            RangeStatus = 1 as libc::c_int as u8;
        }
        19 => {
            RangeStatus = 6 as libc::c_int as u8;
        }
        7 => {
            RangeStatus = 7 as libc::c_int as u8;
        }
        23 => {
            RangeStatus = 12 as libc::c_int as u8;
        }
        20 => {
            RangeStatus = 7 as libc::c_int as u8;
        }
        22 => {
            RangeStatus = 11 as libc::c_int as u8;
        }
        9 => {
            RangeStatus = 0 as libc::c_int as u8;
        }
        _ => {
            RangeStatus = 255 as libc::c_int as u8;
        }
    }
    return RangeStatus;
}
unsafe extern "C" fn SetTargetData(
    mut Dev: VL53LX_DEV,
    mut active_results: u8,
    mut streamcount: u8,
    mut iteration: u8,
    mut device_status: u8,
    mut presults_data: *mut VL53LX_range_data_t,
    mut pRangeData: *mut VL53LX_TargetRangeData_t,
) -> VL53LX_Error {
    let mut Status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    let mut tp: *mut VL53LX_tuning_parm_storage_t = &mut (*pdev).tuning_parms;
    let mut sequency: u8 = 0;
    let mut FilteredRangeStatus: u8 = 0;
    let mut AmbientRate: FixPoint1616_t = 0;
    let mut SignalRate: FixPoint1616_t = 0;
    let mut TempFix1616: FixPoint1616_t = 0;
    let mut Range: i16 = 0;
    let mut RangeDiff: i16 = 0;
    let mut RangeMillimeterInit: i16 = 0;
    let mut ExtendedRangeEnabled: i32 = 0 as libc::c_int;
    let mut uwr_status: u8 = 0;
    let mut AddOffset: i16 = 0;
    FilteredRangeStatus = ((*presults_data).range_status as libc::c_int
        & 0x1f as libc::c_int) as u8;
    SignalRate = ((*presults_data).peak_signal_count_rate_mcps as u32)
        << 9 as libc::c_int;
    (*pRangeData).SignalRateRtnMegaCps = SignalRate;
    AmbientRate = ((*presults_data).ambient_count_rate_mcps as u32)
        << 9 as libc::c_int;
    (*pRangeData).AmbientRateRtnMegaCps = AmbientRate;
    TempFix1616 = ((*presults_data).VL53LX_p_002 as u32) << 9 as libc::c_int;
    (*pRangeData).SigmaMilliMeter = TempFix1616;
    (*pRangeData).RangeMilliMeter = (*presults_data).median_range_mm;
    (*pRangeData).RangeMaxMilliMeter = (*presults_data).max_range_mm;
    (*pRangeData).RangeMinMilliMeter = (*presults_data).min_range_mm;
    match device_status as libc::c_int {
        17 | 2 | 1 | 3 => {
            (*pRangeData).RangeStatus = 5 as libc::c_int as u8;
        }
        13 => {
            (*pRangeData).RangeStatus = 13 as libc::c_int as u8;
        }
        _ => {
            (*pRangeData).RangeStatus = 0 as libc::c_int as u8;
        }
    }
    if (*pRangeData).RangeStatus as libc::c_int == 0 as libc::c_int
        && active_results as libc::c_int == 0 as libc::c_int
    {
        (*pRangeData).RangeStatus = 255 as libc::c_int as u8;
        (*pRangeData).SignalRateRtnMegaCps = 0 as libc::c_int as FixPoint1616_t;
        (*pRangeData).SigmaMilliMeter = 0 as libc::c_int as FixPoint1616_t;
        (*pRangeData).RangeMilliMeter = 8191 as libc::c_int as i16;
        (*pRangeData).RangeMaxMilliMeter = 8191 as libc::c_int as i16;
        (*pRangeData).RangeMinMilliMeter = 8191 as libc::c_int as i16;
    }
    if (*pRangeData).RangeStatus as libc::c_int == 0 as libc::c_int {
        (*pRangeData).RangeStatus = ConvertStatusHisto(FilteredRangeStatus);
    }
    VL53LX_get_tuning_parm(
        Dev,
        (0x8000 as libc::c_int + 145 as libc::c_int) as VL53LX_TuningParms,
        &mut ExtendedRangeEnabled,
    );
    sequency = (streamcount as libc::c_int % 2 as libc::c_int) as u8;
    uwr_status = 0 as libc::c_int as u8;
    RangeMillimeterInit = (*pRangeData).RangeMilliMeter;
    AddOffset = 0 as libc::c_int as i16;
    (*pRangeData).ExtendedRange = 0 as libc::c_int as u8;
    if active_results as libc::c_int != 1 as libc::c_int
        || (*pdev).PreviousRangeActiveResults as libc::c_int != 1 as libc::c_int
    {
        ExtendedRangeEnabled = 0 as libc::c_int;
    }
    if ExtendedRangeEnabled != 0
        && ((*pRangeData).RangeStatus as libc::c_int == 7 as libc::c_int
            || (*pRangeData).RangeStatus as libc::c_int == 4 as libc::c_int)
        && ((*pdev).PreviousRangeStatus[iteration as usize] as libc::c_int
            == 7 as libc::c_int
            || (*pdev).PreviousRangeStatus[iteration as usize] as libc::c_int
                == 4 as libc::c_int
            || (*pdev).PreviousRangeStatus[iteration as usize] as libc::c_int
                == 0 as libc::c_int
                && (*pdev).PreviousExtendedRange[iteration as usize] as libc::c_int
                    == 1 as libc::c_int)
    {
        if (*pdev).PreviousStreamCount as libc::c_int
            == (*pdev).hist_data.result__stream_count as libc::c_int - 1 as libc::c_int
            || (*pdev).PreviousStreamCount as libc::c_int
                == (*pdev).hist_data.result__stream_count as libc::c_int
                    + 127 as libc::c_int
        {
            RangeDiff = ((*pRangeData).RangeMilliMeter as libc::c_int
                - (*pdev).PreviousRangeMilliMeter[iteration as usize] as libc::c_int)
                as i16;
            uwr_status = 1 as libc::c_int as u8;
            match (*pdev).preset_mode as libc::c_int {
                33 => {
                    uwr_status = 0 as libc::c_int as u8;
                }
                30 => {
                    if RangeDiff as libc::c_int > (*tp).tp_uwr_med_z_1_min as libc::c_int
                        && (RangeDiff as libc::c_int)
                            < (*tp).tp_uwr_med_z_1_max as libc::c_int
                        && sequency as libc::c_int == 1 as libc::c_int
                    {
                        AddOffset = (*tp).tp_uwr_med_corr_z_1_rangeb;
                    } else if (RangeDiff as libc::c_int)
                            < -((*tp).tp_uwr_med_z_1_min as libc::c_int)
                            && RangeDiff as libc::c_int
                                > -((*tp).tp_uwr_med_z_1_max as libc::c_int)
                            && sequency as libc::c_int == 0 as libc::c_int
                        {
                        AddOffset = (*tp).tp_uwr_med_corr_z_1_rangea;
                    } else if RangeDiff as libc::c_int
                            > (*tp).tp_uwr_med_z_2_min as libc::c_int
                            && (RangeDiff as libc::c_int)
                                < (*tp).tp_uwr_med_z_2_max as libc::c_int
                            && sequency as libc::c_int == 0 as libc::c_int
                        {
                        AddOffset = (*tp).tp_uwr_med_corr_z_2_rangea;
                    } else if (RangeDiff as libc::c_int)
                            < -((*tp).tp_uwr_med_z_2_min as libc::c_int)
                            && RangeDiff as libc::c_int
                                > -((*tp).tp_uwr_med_z_2_max as libc::c_int)
                            && sequency as libc::c_int == 1 as libc::c_int
                        {
                        AddOffset = (*tp).tp_uwr_med_corr_z_2_rangeb;
                    } else if RangeDiff as libc::c_int
                            > (*tp).tp_uwr_med_z_3_min as libc::c_int
                            && (RangeDiff as libc::c_int)
                                < (*tp).tp_uwr_med_z_3_max as libc::c_int
                            && sequency as libc::c_int == 1 as libc::c_int
                        {
                        AddOffset = (*tp).tp_uwr_med_corr_z_3_rangeb;
                    } else if (RangeDiff as libc::c_int)
                            < -((*tp).tp_uwr_med_z_3_min as libc::c_int)
                            && RangeDiff as libc::c_int
                                > -((*tp).tp_uwr_med_z_3_max as libc::c_int)
                            && sequency as libc::c_int == 0 as libc::c_int
                        {
                        AddOffset = (*tp).tp_uwr_med_corr_z_3_rangea;
                    } else if RangeDiff as libc::c_int
                            > (*tp).tp_uwr_med_z_4_min as libc::c_int
                            && (RangeDiff as libc::c_int)
                                < (*tp).tp_uwr_med_z_4_max as libc::c_int
                            && sequency as libc::c_int == 0 as libc::c_int
                        {
                        AddOffset = (*tp).tp_uwr_med_corr_z_4_rangea;
                    } else if (RangeDiff as libc::c_int)
                            < -((*tp).tp_uwr_med_z_4_min as libc::c_int)
                            && RangeDiff as libc::c_int
                                > -((*tp).tp_uwr_med_z_4_max as libc::c_int)
                            && sequency as libc::c_int == 1 as libc::c_int
                        {
                        AddOffset = (*tp).tp_uwr_med_corr_z_4_rangeb;
                    } else if (RangeDiff as libc::c_int)
                            < (*tp).tp_uwr_med_z_5_max as libc::c_int
                            && RangeDiff as libc::c_int
                                > (*tp).tp_uwr_med_z_5_min as libc::c_int
                        {
                        AddOffset = (*tp).tp_uwr_med_corr_z_5_rangea;
                    } else {
                        uwr_status = 0 as libc::c_int as u8;
                    }
                }
                27 => {
                    if RangeDiff as libc::c_int > (*tp).tp_uwr_lng_z_1_min as libc::c_int
                        && (RangeDiff as libc::c_int)
                            < (*tp).tp_uwr_lng_z_1_max as libc::c_int
                        && sequency as libc::c_int == 0 as libc::c_int
                    {
                        AddOffset = (*tp).tp_uwr_lng_corr_z_1_rangea;
                    } else if (RangeDiff as libc::c_int)
                            < -((*tp).tp_uwr_lng_z_1_min as libc::c_int)
                            && RangeDiff as libc::c_int
                                > -((*tp).tp_uwr_lng_z_1_max as libc::c_int)
                            && sequency as libc::c_int == 1 as libc::c_int
                        {
                        AddOffset = (*tp).tp_uwr_lng_corr_z_1_rangeb;
                    } else if RangeDiff as libc::c_int
                            > (*tp).tp_uwr_lng_z_2_min as libc::c_int
                            && (RangeDiff as libc::c_int)
                                < (*tp).tp_uwr_lng_z_2_max as libc::c_int
                            && sequency as libc::c_int == 1 as libc::c_int
                        {
                        AddOffset = (*tp).tp_uwr_lng_corr_z_2_rangeb;
                    } else if (RangeDiff as libc::c_int)
                            < -((*tp).tp_uwr_lng_z_2_min as libc::c_int)
                            && RangeDiff as libc::c_int
                                > -((*tp).tp_uwr_lng_z_2_max as libc::c_int)
                            && sequency as libc::c_int == 0 as libc::c_int
                        {
                        AddOffset = (*tp).tp_uwr_lng_corr_z_2_rangea;
                    } else if (RangeDiff as libc::c_int)
                            < (*tp).tp_uwr_lng_z_3_max as libc::c_int
                            && RangeDiff as libc::c_int
                                > (*tp).tp_uwr_lng_z_3_min as libc::c_int
                        {
                        AddOffset = (*tp).tp_uwr_lng_corr_z_3_rangea;
                    } else {
                        uwr_status = 0 as libc::c_int as u8;
                    }
                }
                _ => {
                    uwr_status = 0 as libc::c_int as u8;
                }
            }
        }
        if uwr_status != 0 {
            let ref mut fresh0 = (*pRangeData).RangeMilliMeter;
            *fresh0 = (*fresh0 as libc::c_int + AddOffset as libc::c_int) as i16;
            let ref mut fresh1 = (*pRangeData).RangeMinMilliMeter;
            *fresh1 = (*fresh1 as libc::c_int + AddOffset as libc::c_int) as i16;
            let ref mut fresh2 = (*pRangeData).RangeMaxMilliMeter;
            *fresh2 = (*fresh2 as libc::c_int + AddOffset as libc::c_int) as i16;
            (*pRangeData).ExtendedRange = 1 as libc::c_int as u8;
            (*pRangeData).RangeStatus = 0 as libc::c_int as u8;
        }
    }
    (*pdev).PreviousRangeMilliMeter[iteration as usize] = RangeMillimeterInit;
    (*pdev).PreviousRangeStatus[iteration as usize] = (*pRangeData).RangeStatus;
    (*pdev).PreviousExtendedRange[iteration as usize] = (*pRangeData).ExtendedRange;
    (*pdev).PreviousRangeActiveResults = active_results;
    Range = (*pRangeData).RangeMilliMeter;
    if (*pRangeData).RangeStatus as libc::c_int == 0 as libc::c_int
        && (Range as libc::c_int) < 0 as libc::c_int
    {
        if (Range as libc::c_int)
            < BDTable[VL53LX_TUNING_PROXY_MIN as libc::c_int as usize]
        {
            (*pRangeData).RangeStatus = 14 as libc::c_int as u8;
        } else {
            (*pRangeData).RangeMilliMeter = 0 as libc::c_int as i16;
        }
    }
    return Status;
}
unsafe extern "C" fn SetMeasurementData(
    mut Dev: VL53LX_DEV,
    mut presults: *mut VL53LX_range_results_t,
    mut pMultiRangingData: *mut VL53LX_MultiRangingData_t,
) -> VL53LX_Error {
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    let mut i: u8 = 0;
    let mut iteration: u8 = 0;
    let mut pRangeData: *mut VL53LX_TargetRangeData_t = 0
        as *mut VL53LX_TargetRangeData_t;
    let mut presults_data: *mut VL53LX_range_data_t = 0 as *mut VL53LX_range_data_t;
    let mut Status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut ActiveResults: u8 = 0;
    (*pMultiRangingData).NumberOfObjectsFound = (*presults).active_results;
    (*pMultiRangingData)
        .HasXtalkValueChanged = (*presults).smudge_corrector_data.new_xtalk_applied_flag;
    (*pMultiRangingData).TimeStamp = 0 as libc::c_int as u32;
    (*pMultiRangingData).StreamCount = (*presults).stream_count;
    ActiveResults = (*presults).active_results;
    if (ActiveResults as libc::c_int) < 1 as libc::c_int {
        iteration = 1 as libc::c_int as u8;
    } else {
        iteration = ActiveResults;
    }
    i = 0 as libc::c_int as u8;
    while (i as libc::c_int) < iteration as libc::c_int {
        pRangeData = &mut *((*pMultiRangingData).RangeData)
            .as_mut_ptr()
            .offset(i as isize) as *mut VL53LX_TargetRangeData_t;
        presults_data = &mut *((*presults).VL53LX_p_003).as_mut_ptr().offset(i as isize)
            as *mut VL53LX_range_data_t;
        if Status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
            Status = SetTargetData(
                Dev,
                ActiveResults,
                (*pMultiRangingData).StreamCount,
                i,
                (*presults).device_status,
                presults_data,
                pRangeData,
            );
        }
        (*pMultiRangingData).EffectiveSpadRtnCount = (*presults_data).VL53LX_p_004;
        i = i.wrapping_add(1);
    }
    (*pdev).PreviousStreamCount = (*pdev).hist_data.result__stream_count;
    i = iteration;
    while (i as libc::c_int) < 4 as libc::c_int {
        (*pdev).PreviousRangeMilliMeter[i as usize] = 0 as libc::c_int as i16;
        (*pdev).PreviousRangeStatus[i as usize] = 255 as libc::c_int as u8;
        (*pdev).PreviousExtendedRange[i as usize] = 0 as libc::c_int as u8;
        i = i.wrapping_add(1);
    }
    return Status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_GetMultiRangingData(
    mut Dev: VL53LX_DEV,
    mut pMultiRangingData: *mut VL53LX_MultiRangingData_t,
) -> VL53LX_Error {
    let mut Status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    let mut presults: *mut VL53LX_range_results_t = ((*pdev).wArea1).as_mut_ptr()
        as *mut VL53LX_range_results_t;
    memset(
        pMultiRangingData as *mut libc::c_void,
        0xff as libc::c_int,
        ::std::mem::size_of::<VL53LX_MultiRangingData_t>() as libc::c_ulong,
    );
    Status = VL53LX_get_device_results(
        Dev,
        2 as libc::c_int as VL53LX_DeviceResultsLevel,
        presults,
    );
    Status = SetMeasurementData(Dev, presults, pMultiRangingData);
    return Status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_GetAdditionalData(
    mut Dev: VL53LX_DEV,
    mut pAdditionalData: *mut VL53LX_AdditionalData_t,
) -> VL53LX_Error {
    let mut Status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    Status = VL53LX_get_additional_data(Dev, pAdditionalData);
    return Status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_SetTuningParameter(
    mut Dev: VL53LX_DEV,
    mut TuningParameterId: u16,
    mut TuningParameterValue: i32,
) -> VL53LX_Error {
    let mut Status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    if TuningParameterId as libc::c_int
        == (0x8000 as libc::c_int + 134 as libc::c_int) as VL53LX_TuningParms
            as libc::c_int
    {
        return -(4 as libc::c_int) as VL53LX_Error;
    }
    if TuningParameterId as libc::c_int >= 32768 as libc::c_int {
        Status = VL53LX_set_tuning_parm(Dev, TuningParameterId, TuningParameterValue);
    } else if (TuningParameterId as libc::c_int)
            < VL53LX_TUNING_MAX_TUNABLE_KEY as libc::c_int
        {
        BDTable[TuningParameterId as usize] = TuningParameterValue;
    } else {
        Status = -(4 as libc::c_int) as VL53LX_Error;
    }
    return Status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_GetTuningParameter(
    mut Dev: VL53LX_DEV,
    mut TuningParameterId: u16,
    mut pTuningParameterValue: *mut i32,
) -> VL53LX_Error {
    let mut Status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    if TuningParameterId as libc::c_int >= 32768 as libc::c_int {
        Status = VL53LX_get_tuning_parm(Dev, TuningParameterId, pTuningParameterValue);
    } else if (TuningParameterId as libc::c_int)
            < VL53LX_TUNING_MAX_TUNABLE_KEY as libc::c_int
        {
        *pTuningParameterValue = BDTable[TuningParameterId as usize];
    } else {
        Status = -(4 as libc::c_int) as VL53LX_Error;
    }
    return Status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_PerformRefSpadManagement(
    mut Dev: VL53LX_DEV,
) -> VL53LX_Error {
    let mut Status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut RawStatus: VL53LX_Error = 0;
    let mut dcrbuffer: [u8; 24] = [0; 24];
    let mut commbuf: *mut u8 = 0 as *mut u8;
    let mut numloc: [u8; 2] = [
        5 as libc::c_int as u8,
        3 as libc::c_int as u8,
    ];
    let mut pdev: *mut VL53LX_LLDriverData_t = 0 as *mut VL53LX_LLDriverData_t;
    let mut pc: *mut VL53LX_customer_nvm_managed_t = 0
        as *mut VL53LX_customer_nvm_managed_t;
    let mut DistanceMode: VL53LX_DistanceModes = 0;
    pdev = &mut (*Dev).Data.LLData;
    pc = &mut (*pdev).customer;
    if Status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        DistanceMode = (*Dev).Data.CurrentParameters.DistanceMode;
        Status = VL53LX_run_ref_spad_char(Dev, &mut RawStatus);
        if Status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
            Status = VL53LX_SetDistanceMode(Dev, DistanceMode);
        }
    }
    if Status as libc::c_int == -(29 as libc::c_int) as VL53LX_Error as libc::c_int {
        Status = VL53LX_read_nvm_raw_data(
            Dev,
            (0xa0 as libc::c_int >> 2 as libc::c_int) as u8,
            (24 as libc::c_int >> 2 as libc::c_int) as u8,
            dcrbuffer.as_mut_ptr(),
        );
        if Status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
            Status = VL53LX_WriteMulti(
                Dev,
                0x14 as libc::c_int as u16,
                numloc.as_mut_ptr(),
                2 as libc::c_int as u32,
            );
        }
        if Status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
            (*pc)
                .ref_spad_man__num_requested_ref_spads = numloc[0 as libc::c_int
                as usize];
            (*pc).ref_spad_man__ref_location = numloc[1 as libc::c_int as usize];
        }
        commbuf = &mut *dcrbuffer.as_mut_ptr().offset(16 as libc::c_int as isize)
            as *mut u8;
        if Status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
            Status = VL53LX_WriteMulti(
                Dev,
                0xd as libc::c_int as u16,
                commbuf,
                6 as libc::c_int as u32,
            );
        }
        if Status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
            (*pc)
                .global_config__spad_enables_ref_0 = *commbuf
                .offset(0 as libc::c_int as isize);
            (*pc)
                .global_config__spad_enables_ref_1 = *commbuf
                .offset(1 as libc::c_int as isize);
            (*pc)
                .global_config__spad_enables_ref_2 = *commbuf
                .offset(2 as libc::c_int as isize);
            (*pc)
                .global_config__spad_enables_ref_3 = *commbuf
                .offset(3 as libc::c_int as isize);
            (*pc)
                .global_config__spad_enables_ref_4 = *commbuf
                .offset(4 as libc::c_int as isize);
            (*pc)
                .global_config__spad_enables_ref_5 = *commbuf
                .offset(5 as libc::c_int as isize);
        }
    }
    return Status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_SmudgeCorrectionEnable(
    mut Dev: VL53LX_DEV,
    mut Mode: VL53LX_SmudgeCorrectionModes,
) -> VL53LX_Error {
    let mut Status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut s1: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut s2: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut s3: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    match Mode as libc::c_int {
        0 => {
            s1 = VL53LX_dynamic_xtalk_correction_disable(Dev);
            s2 = VL53LX_dynamic_xtalk_correction_apply_disable(Dev);
            s3 = VL53LX_dynamic_xtalk_correction_single_apply_disable(Dev);
        }
        1 => {
            s1 = VL53LX_dynamic_xtalk_correction_enable(Dev);
            s2 = VL53LX_dynamic_xtalk_correction_apply_enable(Dev);
            s3 = VL53LX_dynamic_xtalk_correction_single_apply_disable(Dev);
        }
        2 => {
            s1 = VL53LX_dynamic_xtalk_correction_enable(Dev);
            s2 = VL53LX_dynamic_xtalk_correction_apply_enable(Dev);
            s3 = VL53LX_dynamic_xtalk_correction_single_apply_enable(Dev);
        }
        3 => {
            s1 = VL53LX_dynamic_xtalk_correction_enable(Dev);
            s2 = VL53LX_dynamic_xtalk_correction_apply_disable(Dev);
            s3 = VL53LX_dynamic_xtalk_correction_single_apply_disable(Dev);
        }
        _ => {
            Status = -(4 as libc::c_int) as VL53LX_Error;
        }
    }
    if Status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        Status = s1;
        if Status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
            Status = s2;
        }
        if Status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
            Status = s3;
        }
    }
    return Status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_SetXTalkCompensationEnable(
    mut Dev: VL53LX_DEV,
    mut XTalkCompensationEnable: u8,
) -> VL53LX_Error {
    let mut Status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    if XTalkCompensationEnable as libc::c_int == 0 as libc::c_int {
        Status = VL53LX_disable_xtalk_compensation(Dev);
    } else {
        Status = VL53LX_enable_xtalk_compensation(Dev);
    }
    return Status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_GetXTalkCompensationEnable(
    mut Dev: VL53LX_DEV,
    mut pXTalkCompensationEnable: *mut u8,
) -> VL53LX_Error {
    let mut Status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    VL53LX_get_xtalk_compensation_enable(Dev, pXTalkCompensationEnable);
    return Status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_PerformXTalkCalibration(
    mut Dev: VL53LX_DEV,
) -> VL53LX_Error {
    let mut Status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut UStatus: VL53LX_Error = 0;
    let mut CalDistanceMm: i16 = 0;
    let mut xtalk: VL53LX_xtalk_calibration_results_t = VL53LX_xtalk_calibration_results_t {
        algo__crosstalk_compensation_plane_offset_kcps: 0,
        algo__crosstalk_compensation_x_plane_gradient_kcps: 0,
        algo__crosstalk_compensation_y_plane_gradient_kcps: 0,
        algo__xtalk_cpo_HistoMerge_kcps: [0; 6],
    };
    let mut caldata: VL53LX_CalibrationData_t = VL53LX_CalibrationData_t {
        struct_version: 0,
        customer: VL53LX_CustomerNvmManaged_t {
            global_config__spad_enables_ref_0: 0,
            global_config__spad_enables_ref_1: 0,
            global_config__spad_enables_ref_2: 0,
            global_config__spad_enables_ref_3: 0,
            global_config__spad_enables_ref_4: 0,
            global_config__spad_enables_ref_5: 0,
            global_config__ref_en_start_select: 0,
            ref_spad_man__num_requested_ref_spads: 0,
            ref_spad_man__ref_location: 0,
            algo__crosstalk_compensation_plane_offset_kcps: 0,
            algo__crosstalk_compensation_x_plane_gradient_kcps: 0,
            algo__crosstalk_compensation_y_plane_gradient_kcps: 0,
            ref_spad_char__total_rate_target_mcps: 0,
            algo__part_to_part_range_offset_mm: 0,
            mm_config__inner_offset_mm: 0,
            mm_config__outer_offset_mm: 0,
        },
        add_off_cal_data: VL53LX_additional_offset_cal_data_t {
            result__mm_inner_actual_effective_spads: 0,
            result__mm_outer_actual_effective_spads: 0,
            result__mm_inner_peak_signal_count_rtn_mcps: 0,
            result__mm_outer_peak_signal_count_rtn_mcps: 0,
        },
        optical_centre: VL53LX_optical_centre_t {
            x_centre: 0,
            y_centre: 0,
        },
        xtalkhisto: VL53LX_xtalk_histogram_data_t {
            xtalk_shape: VL53LX_xtalk_histogram_shape_t {
                zone_id: 0,
                time_stamp: 0,
                VL53LX_p_019: 0,
                VL53LX_p_020: 0,
                VL53LX_p_021: 0,
                bin_data: [0; 12],
                phasecal_result__reference_phase: 0,
                phasecal_result__vcsel_start: 0,
                cal_config__vcsel_start: 0,
                vcsel_width: 0,
                VL53LX_p_015: 0,
                zero_distance_phase: 0,
            },
            xtalk_hist_removed: VL53LX_histogram_bin_data_t {
                cfg_device_state: 0,
                rd_device_state: 0,
                zone_id: 0,
                time_stamp: 0,
                VL53LX_p_019: 0,
                VL53LX_p_020: 0,
                VL53LX_p_021: 0,
                number_of_ambient_bins: 0,
                bin_seq: [0; 6],
                bin_rep: [0; 6],
                bin_data: [0; 24],
                result__interrupt_status: 0,
                result__range_status: 0,
                result__report_status: 0,
                result__stream_count: 0,
                result__dss_actual_effective_spads: 0,
                phasecal_result__reference_phase: 0,
                phasecal_result__vcsel_start: 0,
                cal_config__vcsel_start: 0,
                vcsel_width: 0,
                VL53LX_p_005: 0,
                VL53LX_p_015: 0,
                total_periods_elapsed: 0,
                peak_duration_us: 0,
                woi_duration_us: 0,
                min_bin_value: 0,
                max_bin_value: 0,
                zero_distance_phase: 0,
                number_of_ambient_samples: 0,
                ambient_events_sum: 0,
                VL53LX_p_028: 0,
                roi_config__user_roi_centre_spad: 0,
                roi_config__user_roi_requested_global_xy_size: 0,
            },
        },
        gain_cal: VL53LX_gain_calibration_data_t {
            standard_ranging_gain_factor: 0,
            histogram_ranging_gain_factor: 0,
        },
        cal_peak_rate_map: VL53LX_cal_peak_rate_map_t {
            cal_distance_mm: 0,
            cal_reflectance_pc: 0,
            max_samples: 0,
            width: 0,
            height: 0,
            peak_rate_mcps: [0; 25],
        },
        per_vcsel_cal_data: VL53LX_per_vcsel_period_offset_cal_data_t {
            short_a_offset_mm: 0,
            short_b_offset_mm: 0,
            medium_a_offset_mm: 0,
            medium_b_offset_mm: 0,
            long_a_offset_mm: 0,
            long_b_offset_mm: 0,
        },
        algo__xtalk_cpo_HistoMerge_kcps: [0; 6],
    };
    let mut pLLData: *mut VL53LX_LLDriverData_t = 0 as *mut VL53LX_LLDriverData_t;
    let mut i: libc::c_int = 0;
    let mut pPlaneOffsetKcps: *mut u32 = 0 as *mut u32;
    let mut Margin: u32 = BDTable[VL53LX_TUNING_XTALK_FULL_ROI_BIN_SUM_MARGIN
        as libc::c_int as usize] as u32;
    let mut DefaultOffset: u32 = BDTable[VL53LX_TUNING_XTALK_FULL_ROI_DEFAULT_OFFSET
        as libc::c_int as usize] as u32;
    let mut pLLDataPlaneOffsetKcps: *mut u32 = 0 as *mut u32;
    let mut sum: u32 = 0 as libc::c_int as u32;
    let mut binok: u8 = 0 as libc::c_int as u8;
    pPlaneOffsetKcps = &mut caldata
        .customer
        .algo__crosstalk_compensation_plane_offset_kcps;
    pLLData = &mut (*Dev).Data.LLData;
    pLLDataPlaneOffsetKcps = &mut (*pLLData)
        .xtalk_cal
        .algo__crosstalk_compensation_plane_offset_kcps;
    CalDistanceMm = BDTable[VL53LX_TUNING_XTALK_FULL_ROI_TARGET_DISTANCE_MM
        as libc::c_int as usize] as i16;
    Status = VL53LX_run_hist_xtalk_extraction(Dev, CalDistanceMm, &mut UStatus);
    VL53LX_GetCalibrationData(Dev, &mut caldata);
    i = 0 as libc::c_int;
    while i < 12 as libc::c_int {
        sum = (sum as libc::c_uint)
            .wrapping_add(caldata.xtalkhisto.xtalk_shape.bin_data[i as usize])
            as u32 as u32;
        if caldata.xtalkhisto.xtalk_shape.bin_data[i as usize]
            > 0 as libc::c_int as libc::c_uint
        {
            binok = binok.wrapping_add(1);
        }
        i += 1;
    }
    if UStatus as libc::c_int == -(23 as libc::c_int) as VL53LX_Error as libc::c_int
        || sum > (1024 as libc::c_int as libc::c_uint).wrapping_add(Margin)
        || sum < (1024 as libc::c_int as libc::c_uint).wrapping_sub(Margin)
        || (binok as libc::c_int) < 3 as libc::c_int
    {
        *pPlaneOffsetKcps = DefaultOffset;
        *pLLDataPlaneOffsetKcps = DefaultOffset;
        caldata
            .xtalkhisto
            .xtalk_shape
            .bin_data[0 as libc::c_int as usize] = 307 as libc::c_int as u32;
        caldata
            .xtalkhisto
            .xtalk_shape
            .bin_data[1 as libc::c_int as usize] = 410 as libc::c_int as u32;
        caldata
            .xtalkhisto
            .xtalk_shape
            .bin_data[2 as libc::c_int as usize] = 410 as libc::c_int as u32;
        caldata
            .xtalkhisto
            .xtalk_shape
            .bin_data[3 as libc::c_int as usize] = 307 as libc::c_int as u32;
        i = 4 as libc::c_int;
        while i < 12 as libc::c_int {
            caldata
                .xtalkhisto
                .xtalk_shape
                .bin_data[i as usize] = 0 as libc::c_int as u32;
            i += 1;
        }
        i = 0 as libc::c_int;
        while i < 6 as libc::c_int {
            caldata
                .algo__xtalk_cpo_HistoMerge_kcps[i
                as usize] = DefaultOffset
                .wrapping_add(DefaultOffset.wrapping_mul(i as libc::c_uint));
            i += 1;
        }
        VL53LX_SetCalibrationData(Dev, &mut caldata);
    }
    if Status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        Status = VL53LX_get_current_xtalk_settings(Dev, &mut xtalk);
        Status = VL53LX_set_tuning_parm(
            Dev,
            (0x8000 as libc::c_int + 134 as libc::c_int) as VL53LX_TuningParms,
            xtalk.algo__crosstalk_compensation_plane_offset_kcps as i32,
        );
    }
    return Status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_SetOffsetCorrectionMode(
    mut Dev: VL53LX_DEV,
    mut OffsetCorrectionMode: VL53LX_OffsetCorrectionModes,
) -> VL53LX_Error {
    let mut Status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut offset_cor_mode: VL53LX_OffsetCorrectionMode = 0;
    if OffsetCorrectionMode as libc::c_int
        == 3 as libc::c_int as VL53LX_OffsetCorrectionModes as libc::c_int
    {
        offset_cor_mode = 3 as libc::c_int as VL53LX_OffsetCorrectionMode;
    } else {
        offset_cor_mode = 1 as libc::c_int as VL53LX_OffsetCorrectionMode;
        if OffsetCorrectionMode as libc::c_int
            != 1 as libc::c_int as VL53LX_OffsetCorrectionModes as libc::c_int
        {
            Status = -(4 as libc::c_int) as VL53LX_Error;
        }
    }
    if Status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        Status = VL53LX_set_offset_correction_mode(Dev, offset_cor_mode);
    }
    return Status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_PerformOffsetSimpleCalibration(
    mut Dev: VL53LX_DEV,
    mut CalDistanceMilliMeter: i32,
) -> VL53LX_Error {
    let mut Status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut sum_ranging: i32 = 0;
    let mut offset_meas: u8 = 0;
    let mut Max: i16 = 0;
    let mut UnderMax: i16 = 0;
    let mut OverMax: i16 = 0;
    let mut Repeat: i16 = 0;
    let mut total_count: i32 = 0;
    let mut inloopcount: i32 = 0;
    let mut IncRounding: i32 = 0;
    let mut meanDistance_mm: i16 = 0;
    let mut offset: i16 = 0;
    let mut RangingMeasurementData: VL53LX_MultiRangingData_t = VL53LX_MultiRangingData_t {
        TimeStamp: 0,
        StreamCount: 0,
        NumberOfObjectsFound: 0,
        RangeData: [VL53LX_TargetRangeData_t {
            RangeMaxMilliMeter: 0,
            RangeMinMilliMeter: 0,
            SignalRateRtnMegaCps: 0,
            AmbientRateRtnMegaCps: 0,
            SigmaMilliMeter: 0,
            RangeMilliMeter: 0,
            RangeStatus: 0,
            ExtendedRange: 0,
        }; 4],
        HasXtalkValueChanged: 0,
        EffectiveSpadRtnCount: 0,
    };
    let mut pdev: *mut VL53LX_LLDriverData_t = 0 as *mut VL53LX_LLDriverData_t;
    let mut goodmeas: u8 = 0;
    let mut SmudgeStatus: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut smudge_corr_en: u8 = 0;
    let mut pRange: *mut VL53LX_TargetRangeData_t = 0 as *mut VL53LX_TargetRangeData_t;
    pdev = &mut (*Dev).Data.LLData;
    smudge_corr_en = (*pdev).smudge_correct_config.smudge_corr_enabled;
    SmudgeStatus = VL53LX_dynamic_xtalk_correction_disable(Dev);
    (*pdev).customer.algo__part_to_part_range_offset_mm = 0 as libc::c_int as i16;
    (*pdev).customer.mm_config__inner_offset_mm = 0 as libc::c_int as i16;
    (*pdev).customer.mm_config__outer_offset_mm = 0 as libc::c_int as i16;
    memset(
        &mut (*pdev).per_vcsel_cal_data as *mut VL53LX_per_vcsel_period_offset_cal_data_t
            as *mut libc::c_void,
        0 as libc::c_int,
        ::std::mem::size_of::<VL53LX_per_vcsel_period_offset_cal_data_t>()
            as libc::c_ulong,
    );
    Repeat = BDTable[VL53LX_TUNING_SIMPLE_OFFSET_CALIBRATION_REPEAT as libc::c_int
        as usize] as i16;
    Max = BDTable[VL53LX_TUNING_MAX_SIMPLE_OFFSET_CALIBRATION_SAMPLE_NUMBER
        as libc::c_int as usize] as i16;
    UnderMax = (1 as libc::c_int + Max as libc::c_int / 2 as libc::c_int) as i16;
    OverMax = (Max as libc::c_int + Max as libc::c_int / 2 as libc::c_int) as i16;
    sum_ranging = 0 as libc::c_int;
    total_count = 0 as libc::c_int;
    while Repeat as libc::c_int > 0 as libc::c_int
        && Status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int
    {
        Status = VL53LX_StartMeasurement(Dev);
        if Status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
            VL53LX_WaitMeasurementDataReady(Dev);
            VL53LX_GetMultiRangingData(Dev, &mut RangingMeasurementData);
            VL53LX_ClearInterruptAndStartMeasurement(Dev);
        }
        inloopcount = 0 as libc::c_int;
        offset_meas = 0 as libc::c_int as u8;
        while Status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int
            && inloopcount < Max as libc::c_int
            && (offset_meas as libc::c_int) < OverMax as libc::c_int
        {
            Status = VL53LX_WaitMeasurementDataReady(Dev);
            if Status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
                Status = VL53LX_GetMultiRangingData(Dev, &mut RangingMeasurementData);
            }
            pRange = &mut *(RangingMeasurementData.RangeData)
                .as_mut_ptr()
                .offset(0 as libc::c_int as isize) as *mut VL53LX_TargetRangeData_t;
            goodmeas = ((*pRange).RangeStatus as libc::c_int == 0 as libc::c_int)
                as libc::c_int as u8;
            if Status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int
                && goodmeas as libc::c_int != 0
            {
                sum_ranging += (*pRange).RangeMilliMeter as libc::c_int;
                inloopcount += 1;
            }
            Status = VL53LX_ClearInterruptAndStartMeasurement(Dev);
            offset_meas = offset_meas.wrapping_add(1);
        }
        total_count += inloopcount;
        if inloopcount < UnderMax as libc::c_int {
            Status = -(24 as libc::c_int) as VL53LX_Error;
        }
        VL53LX_StopMeasurement(Dev);
        Repeat -= 1;
    }
    if SmudgeStatus as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int
        && smudge_corr_en as libc::c_int == 1 as libc::c_int
    {
        SmudgeStatus = VL53LX_dynamic_xtalk_correction_enable(Dev);
    }
    if sum_ranging < 0 as libc::c_int
        || sum_ranging > total_count * 0xffff as libc::c_int
    {
        Status = -(32 as libc::c_int) as VL53LX_Error;
    }
    if Status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int
        && total_count > 0 as libc::c_int
    {
        IncRounding = total_count / 2 as libc::c_int;
        meanDistance_mm = ((sum_ranging + IncRounding) / total_count) as i16;
        offset = (CalDistanceMilliMeter as i16 as libc::c_int
            - meanDistance_mm as libc::c_int) as i16;
        (*pdev)
            .customer
            .algo__part_to_part_range_offset_mm = 0 as libc::c_int as i16;
        (*pdev).customer.mm_config__inner_offset_mm = offset;
        (*pdev).customer.mm_config__outer_offset_mm = offset;
        Status = VL53LX_set_customer_nvm_managed(Dev, &mut (*pdev).customer);
    }
    return Status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_PerformOffsetZeroDistanceCalibration(
    mut Dev: VL53LX_DEV,
) -> VL53LX_Error {
    let mut Status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut sum_ranging: i32 = 0;
    let mut offset_meas: u8 = 0;
    let mut Max: i16 = 0;
    let mut UnderMax: i16 = 0;
    let mut OverMax: i16 = 0;
    let mut Repeat: i16 = 0;
    let mut total_count: i32 = 0;
    let mut inloopcount: i32 = 0;
    let mut IncRounding: i32 = 0;
    let mut meanDistance_mm: i16 = 0;
    let mut offset: i16 = 0;
    let mut ZeroDistanceOffset: i16 = 0;
    let mut RangingMeasurementData: VL53LX_MultiRangingData_t = VL53LX_MultiRangingData_t {
        TimeStamp: 0,
        StreamCount: 0,
        NumberOfObjectsFound: 0,
        RangeData: [VL53LX_TargetRangeData_t {
            RangeMaxMilliMeter: 0,
            RangeMinMilliMeter: 0,
            SignalRateRtnMegaCps: 0,
            AmbientRateRtnMegaCps: 0,
            SigmaMilliMeter: 0,
            RangeMilliMeter: 0,
            RangeStatus: 0,
            ExtendedRange: 0,
        }; 4],
        HasXtalkValueChanged: 0,
        EffectiveSpadRtnCount: 0,
    };
    let mut pdev: *mut VL53LX_LLDriverData_t = 0 as *mut VL53LX_LLDriverData_t;
    let mut goodmeas: u8 = 0;
    let mut SmudgeStatus: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut smudge_corr_en: u8 = 0;
    let mut pRange: *mut VL53LX_TargetRangeData_t = 0 as *mut VL53LX_TargetRangeData_t;
    pdev = &mut (*Dev).Data.LLData;
    smudge_corr_en = (*pdev).smudge_correct_config.smudge_corr_enabled;
    SmudgeStatus = VL53LX_dynamic_xtalk_correction_disable(Dev);
    (*pdev).customer.algo__part_to_part_range_offset_mm = 0 as libc::c_int as i16;
    (*pdev).customer.mm_config__inner_offset_mm = 50 as libc::c_int as i16;
    (*pdev).customer.mm_config__outer_offset_mm = 50 as libc::c_int as i16;
    memset(
        &mut (*pdev).per_vcsel_cal_data as *mut VL53LX_per_vcsel_period_offset_cal_data_t
            as *mut libc::c_void,
        0 as libc::c_int,
        ::std::mem::size_of::<VL53LX_per_vcsel_period_offset_cal_data_t>()
            as libc::c_ulong,
    );
    ZeroDistanceOffset = BDTable[VL53LX_TUNING_ZERO_DISTANCE_OFFSET_NON_LINEAR_FACTOR
        as libc::c_int as usize] as i16;
    Repeat = BDTable[VL53LX_TUNING_SIMPLE_OFFSET_CALIBRATION_REPEAT as libc::c_int
        as usize] as i16;
    Max = BDTable[VL53LX_TUNING_MAX_SIMPLE_OFFSET_CALIBRATION_SAMPLE_NUMBER
        as libc::c_int as usize] as i16;
    UnderMax = (1 as libc::c_int + Max as libc::c_int / 2 as libc::c_int) as i16;
    OverMax = (Max as libc::c_int + Max as libc::c_int / 2 as libc::c_int) as i16;
    sum_ranging = 0 as libc::c_int;
    total_count = 0 as libc::c_int;
    while Repeat as libc::c_int > 0 as libc::c_int
        && Status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int
    {
        Status = VL53LX_StartMeasurement(Dev);
        if Status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
            VL53LX_WaitMeasurementDataReady(Dev);
            VL53LX_GetMultiRangingData(Dev, &mut RangingMeasurementData);
            VL53LX_ClearInterruptAndStartMeasurement(Dev);
        }
        inloopcount = 0 as libc::c_int;
        offset_meas = 0 as libc::c_int as u8;
        while Status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int
            && inloopcount < Max as libc::c_int
            && (offset_meas as libc::c_int) < OverMax as libc::c_int
        {
            Status = VL53LX_WaitMeasurementDataReady(Dev);
            if Status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
                Status = VL53LX_GetMultiRangingData(Dev, &mut RangingMeasurementData);
            }
            pRange = &mut *(RangingMeasurementData.RangeData)
                .as_mut_ptr()
                .offset(0 as libc::c_int as isize) as *mut VL53LX_TargetRangeData_t;
            goodmeas = ((*pRange).RangeStatus as libc::c_int == 0 as libc::c_int)
                as libc::c_int as u8;
            if Status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int
                && goodmeas as libc::c_int != 0
            {
                sum_ranging = sum_ranging + (*pRange).RangeMilliMeter as libc::c_int;
                inloopcount += 1;
            }
            Status = VL53LX_ClearInterruptAndStartMeasurement(Dev);
            offset_meas = offset_meas.wrapping_add(1);
        }
        total_count += inloopcount;
        if inloopcount < UnderMax as libc::c_int {
            Status = -(24 as libc::c_int) as VL53LX_Error;
        }
        VL53LX_StopMeasurement(Dev);
        Repeat -= 1;
    }
    if SmudgeStatus as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int
        && smudge_corr_en as libc::c_int == 1 as libc::c_int
    {
        SmudgeStatus = VL53LX_dynamic_xtalk_correction_enable(Dev);
    }
    if sum_ranging < 0 as libc::c_int
        || sum_ranging > total_count * 0xffff as libc::c_int
    {
        Status = -(32 as libc::c_int) as VL53LX_Error;
    }
    if Status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int
        && total_count > 0 as libc::c_int
    {
        IncRounding = total_count / 2 as libc::c_int;
        meanDistance_mm = ((sum_ranging + IncRounding) / total_count) as i16;
        offset = (50 as libc::c_int - meanDistance_mm as libc::c_int
            + ZeroDistanceOffset as libc::c_int) as i16;
        (*pdev)
            .customer
            .algo__part_to_part_range_offset_mm = 0 as libc::c_int as i16;
        (*pdev).customer.mm_config__inner_offset_mm = offset;
        (*pdev).customer.mm_config__outer_offset_mm = offset;
        Status = VL53LX_set_customer_nvm_managed(Dev, &mut (*pdev).customer);
    }
    return Status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_SetCalibrationData(
    mut Dev: VL53LX_DEV,
    mut pCalibrationData: *mut VL53LX_CalibrationData_t,
) -> VL53LX_Error {
    let mut Status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pC: *mut VL53LX_CustomerNvmManaged_t = 0 as *mut VL53LX_CustomerNvmManaged_t;
    let mut cal_data: VL53LX_calibration_data_t = VL53LX_calibration_data_t {
        struct_version: 0,
        customer: VL53LX_customer_nvm_managed_t {
            global_config__spad_enables_ref_0: 0,
            global_config__spad_enables_ref_1: 0,
            global_config__spad_enables_ref_2: 0,
            global_config__spad_enables_ref_3: 0,
            global_config__spad_enables_ref_4: 0,
            global_config__spad_enables_ref_5: 0,
            global_config__ref_en_start_select: 0,
            ref_spad_man__num_requested_ref_spads: 0,
            ref_spad_man__ref_location: 0,
            algo__crosstalk_compensation_plane_offset_kcps: 0,
            algo__crosstalk_compensation_x_plane_gradient_kcps: 0,
            algo__crosstalk_compensation_y_plane_gradient_kcps: 0,
            ref_spad_char__total_rate_target_mcps: 0,
            algo__part_to_part_range_offset_mm: 0,
            mm_config__inner_offset_mm: 0,
            mm_config__outer_offset_mm: 0,
        },
        fmt_dmax_cal: VL53LX_dmax_calibration_data_t {
            ref__actual_effective_spads: 0,
            ref__peak_signal_count_rate_mcps: 0,
            ref__distance_mm: 0,
            ref_reflectance_pc: 0,
            coverglass_transmission: 0,
        },
        cust_dmax_cal: VL53LX_dmax_calibration_data_t {
            ref__actual_effective_spads: 0,
            ref__peak_signal_count_rate_mcps: 0,
            ref__distance_mm: 0,
            ref_reflectance_pc: 0,
            coverglass_transmission: 0,
        },
        add_off_cal_data: VL53LX_additional_offset_cal_data_t {
            result__mm_inner_actual_effective_spads: 0,
            result__mm_outer_actual_effective_spads: 0,
            result__mm_inner_peak_signal_count_rtn_mcps: 0,
            result__mm_outer_peak_signal_count_rtn_mcps: 0,
        },
        optical_centre: VL53LX_optical_centre_t {
            x_centre: 0,
            y_centre: 0,
        },
        xtalkhisto: VL53LX_xtalk_histogram_data_t {
            xtalk_shape: VL53LX_xtalk_histogram_shape_t {
                zone_id: 0,
                time_stamp: 0,
                VL53LX_p_019: 0,
                VL53LX_p_020: 0,
                VL53LX_p_021: 0,
                bin_data: [0; 12],
                phasecal_result__reference_phase: 0,
                phasecal_result__vcsel_start: 0,
                cal_config__vcsel_start: 0,
                vcsel_width: 0,
                VL53LX_p_015: 0,
                zero_distance_phase: 0,
            },
            xtalk_hist_removed: VL53LX_histogram_bin_data_t {
                cfg_device_state: 0,
                rd_device_state: 0,
                zone_id: 0,
                time_stamp: 0,
                VL53LX_p_019: 0,
                VL53LX_p_020: 0,
                VL53LX_p_021: 0,
                number_of_ambient_bins: 0,
                bin_seq: [0; 6],
                bin_rep: [0; 6],
                bin_data: [0; 24],
                result__interrupt_status: 0,
                result__range_status: 0,
                result__report_status: 0,
                result__stream_count: 0,
                result__dss_actual_effective_spads: 0,
                phasecal_result__reference_phase: 0,
                phasecal_result__vcsel_start: 0,
                cal_config__vcsel_start: 0,
                vcsel_width: 0,
                VL53LX_p_005: 0,
                VL53LX_p_015: 0,
                total_periods_elapsed: 0,
                peak_duration_us: 0,
                woi_duration_us: 0,
                min_bin_value: 0,
                max_bin_value: 0,
                zero_distance_phase: 0,
                number_of_ambient_samples: 0,
                ambient_events_sum: 0,
                VL53LX_p_028: 0,
                roi_config__user_roi_centre_spad: 0,
                roi_config__user_roi_requested_global_xy_size: 0,
            },
        },
        gain_cal: VL53LX_gain_calibration_data_t {
            standard_ranging_gain_factor: 0,
            histogram_ranging_gain_factor: 0,
        },
        cal_peak_rate_map: VL53LX_cal_peak_rate_map_t {
            cal_distance_mm: 0,
            cal_reflectance_pc: 0,
            max_samples: 0,
            width: 0,
            height: 0,
            peak_rate_mcps: [0; 25],
        },
        per_vcsel_cal_data: VL53LX_per_vcsel_period_offset_cal_data_t {
            short_a_offset_mm: 0,
            short_b_offset_mm: 0,
            medium_a_offset_mm: 0,
            medium_b_offset_mm: 0,
            long_a_offset_mm: 0,
            long_b_offset_mm: 0,
        },
    };
    let mut x: u32 = 0;
    let mut xtalk: VL53LX_xtalk_calibration_results_t = VL53LX_xtalk_calibration_results_t {
        algo__crosstalk_compensation_plane_offset_kcps: 0,
        algo__crosstalk_compensation_x_plane_gradient_kcps: 0,
        algo__crosstalk_compensation_y_plane_gradient_kcps: 0,
        algo__xtalk_cpo_HistoMerge_kcps: [0; 6],
    };
    cal_data
        .struct_version = ((*pCalibrationData).struct_version)
        .wrapping_sub(0x20 as libc::c_int as libc::c_uint);
    memcpy(
        &mut cal_data.add_off_cal_data as *mut VL53LX_additional_offset_cal_data_t
            as *mut libc::c_void,
        &mut (*pCalibrationData).add_off_cal_data
            as *mut VL53LX_additional_offset_cal_data_t as *const libc::c_void,
        ::std::mem::size_of::<VL53LX_additional_offset_cal_data_t>() as libc::c_ulong,
    );
    memcpy(
        &mut cal_data.optical_centre as *mut VL53LX_optical_centre_t
            as *mut libc::c_void,
        &mut (*pCalibrationData).optical_centre as *mut VL53LX_optical_centre_t
            as *const libc::c_void,
        ::std::mem::size_of::<VL53LX_optical_centre_t>() as libc::c_ulong,
    );
    memcpy(
        &mut cal_data.xtalkhisto as *mut VL53LX_xtalk_histogram_data_t
            as *mut libc::c_void,
        &mut (*pCalibrationData).xtalkhisto as *mut VL53LX_xtalk_histogram_data_t
            as *const libc::c_void,
        ::std::mem::size_of::<VL53LX_xtalk_histogram_data_t>() as libc::c_ulong,
    );
    memcpy(
        &mut cal_data.gain_cal as *mut VL53LX_gain_calibration_data_t
            as *mut libc::c_void,
        &mut (*pCalibrationData).gain_cal as *mut VL53LX_gain_calibration_data_t
            as *const libc::c_void,
        ::std::mem::size_of::<VL53LX_gain_calibration_data_t>() as libc::c_ulong,
    );
    memcpy(
        &mut cal_data.cal_peak_rate_map as *mut VL53LX_cal_peak_rate_map_t
            as *mut libc::c_void,
        &mut (*pCalibrationData).cal_peak_rate_map as *mut VL53LX_cal_peak_rate_map_t
            as *const libc::c_void,
        ::std::mem::size_of::<VL53LX_cal_peak_rate_map_t>() as libc::c_ulong,
    );
    memcpy(
        &mut cal_data.per_vcsel_cal_data
            as *mut VL53LX_per_vcsel_period_offset_cal_data_t as *mut libc::c_void,
        &mut (*pCalibrationData).per_vcsel_cal_data
            as *mut VL53LX_per_vcsel_period_offset_cal_data_t as *const libc::c_void,
        ::std::mem::size_of::<VL53LX_per_vcsel_period_offset_cal_data_t>()
            as libc::c_ulong,
    );
    pC = &mut (*pCalibrationData).customer;
    x = (*pC).algo__crosstalk_compensation_plane_offset_kcps;
    cal_data
        .customer
        .algo__crosstalk_compensation_plane_offset_kcps = (x
        & 0xffff as libc::c_int as libc::c_uint) as u16;
    cal_data
        .customer
        .global_config__spad_enables_ref_0 = (*pC).global_config__spad_enables_ref_0;
    cal_data
        .customer
        .global_config__spad_enables_ref_1 = (*pC).global_config__spad_enables_ref_1;
    cal_data
        .customer
        .global_config__spad_enables_ref_2 = (*pC).global_config__spad_enables_ref_2;
    cal_data
        .customer
        .global_config__spad_enables_ref_3 = (*pC).global_config__spad_enables_ref_3;
    cal_data
        .customer
        .global_config__spad_enables_ref_4 = (*pC).global_config__spad_enables_ref_4;
    cal_data
        .customer
        .global_config__spad_enables_ref_5 = (*pC).global_config__spad_enables_ref_5;
    cal_data
        .customer
        .global_config__ref_en_start_select = (*pC).global_config__ref_en_start_select;
    cal_data
        .customer
        .ref_spad_man__num_requested_ref_spads = (*pC)
        .ref_spad_man__num_requested_ref_spads;
    cal_data.customer.ref_spad_man__ref_location = (*pC).ref_spad_man__ref_location;
    cal_data
        .customer
        .algo__crosstalk_compensation_x_plane_gradient_kcps = (*pC)
        .algo__crosstalk_compensation_x_plane_gradient_kcps;
    cal_data
        .customer
        .algo__crosstalk_compensation_y_plane_gradient_kcps = (*pC)
        .algo__crosstalk_compensation_y_plane_gradient_kcps;
    cal_data
        .customer
        .ref_spad_char__total_rate_target_mcps = (*pC)
        .ref_spad_char__total_rate_target_mcps;
    cal_data
        .customer
        .algo__part_to_part_range_offset_mm = (*pC).algo__part_to_part_range_offset_mm;
    cal_data.customer.mm_config__inner_offset_mm = (*pC).mm_config__inner_offset_mm;
    cal_data.customer.mm_config__outer_offset_mm = (*pC).mm_config__outer_offset_mm;
    Status = VL53LX_set_part_to_part_data(Dev, &mut cal_data);
    if !(Status as libc::c_int != 0 as libc::c_int as VL53LX_Error as libc::c_int) {
        Status = VL53LX_get_current_xtalk_settings(Dev, &mut xtalk);
        if !(Status as libc::c_int != 0 as libc::c_int as VL53LX_Error as libc::c_int) {
            xtalk.algo__crosstalk_compensation_plane_offset_kcps = x;
            Status = VL53LX_set_tuning_parm(
                Dev,
                (0x8000 as libc::c_int + 134 as libc::c_int) as VL53LX_TuningParms,
                x as i32,
            );
            memcpy(
                &mut *(xtalk.algo__xtalk_cpo_HistoMerge_kcps)
                    .as_mut_ptr()
                    .offset(0 as libc::c_int as isize) as *mut u32
                    as *mut libc::c_void,
                &mut *((*pCalibrationData).algo__xtalk_cpo_HistoMerge_kcps)
                    .as_mut_ptr()
                    .offset(0 as libc::c_int as isize) as *mut u32
                    as *const libc::c_void,
                ::std::mem::size_of::<[u32; 6]>() as libc::c_ulong,
            );
            Status = VL53LX_set_current_xtalk_settings(Dev, &mut xtalk);
        }
    }
    return Status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_GetCalibrationData(
    mut Dev: VL53LX_DEV,
    mut pCalibrationData: *mut VL53LX_CalibrationData_t,
) -> VL53LX_Error {
    let mut Status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut cal_data: VL53LX_calibration_data_t = VL53LX_calibration_data_t {
        struct_version: 0,
        customer: VL53LX_customer_nvm_managed_t {
            global_config__spad_enables_ref_0: 0,
            global_config__spad_enables_ref_1: 0,
            global_config__spad_enables_ref_2: 0,
            global_config__spad_enables_ref_3: 0,
            global_config__spad_enables_ref_4: 0,
            global_config__spad_enables_ref_5: 0,
            global_config__ref_en_start_select: 0,
            ref_spad_man__num_requested_ref_spads: 0,
            ref_spad_man__ref_location: 0,
            algo__crosstalk_compensation_plane_offset_kcps: 0,
            algo__crosstalk_compensation_x_plane_gradient_kcps: 0,
            algo__crosstalk_compensation_y_plane_gradient_kcps: 0,
            ref_spad_char__total_rate_target_mcps: 0,
            algo__part_to_part_range_offset_mm: 0,
            mm_config__inner_offset_mm: 0,
            mm_config__outer_offset_mm: 0,
        },
        fmt_dmax_cal: VL53LX_dmax_calibration_data_t {
            ref__actual_effective_spads: 0,
            ref__peak_signal_count_rate_mcps: 0,
            ref__distance_mm: 0,
            ref_reflectance_pc: 0,
            coverglass_transmission: 0,
        },
        cust_dmax_cal: VL53LX_dmax_calibration_data_t {
            ref__actual_effective_spads: 0,
            ref__peak_signal_count_rate_mcps: 0,
            ref__distance_mm: 0,
            ref_reflectance_pc: 0,
            coverglass_transmission: 0,
        },
        add_off_cal_data: VL53LX_additional_offset_cal_data_t {
            result__mm_inner_actual_effective_spads: 0,
            result__mm_outer_actual_effective_spads: 0,
            result__mm_inner_peak_signal_count_rtn_mcps: 0,
            result__mm_outer_peak_signal_count_rtn_mcps: 0,
        },
        optical_centre: VL53LX_optical_centre_t {
            x_centre: 0,
            y_centre: 0,
        },
        xtalkhisto: VL53LX_xtalk_histogram_data_t {
            xtalk_shape: VL53LX_xtalk_histogram_shape_t {
                zone_id: 0,
                time_stamp: 0,
                VL53LX_p_019: 0,
                VL53LX_p_020: 0,
                VL53LX_p_021: 0,
                bin_data: [0; 12],
                phasecal_result__reference_phase: 0,
                phasecal_result__vcsel_start: 0,
                cal_config__vcsel_start: 0,
                vcsel_width: 0,
                VL53LX_p_015: 0,
                zero_distance_phase: 0,
            },
            xtalk_hist_removed: VL53LX_histogram_bin_data_t {
                cfg_device_state: 0,
                rd_device_state: 0,
                zone_id: 0,
                time_stamp: 0,
                VL53LX_p_019: 0,
                VL53LX_p_020: 0,
                VL53LX_p_021: 0,
                number_of_ambient_bins: 0,
                bin_seq: [0; 6],
                bin_rep: [0; 6],
                bin_data: [0; 24],
                result__interrupt_status: 0,
                result__range_status: 0,
                result__report_status: 0,
                result__stream_count: 0,
                result__dss_actual_effective_spads: 0,
                phasecal_result__reference_phase: 0,
                phasecal_result__vcsel_start: 0,
                cal_config__vcsel_start: 0,
                vcsel_width: 0,
                VL53LX_p_005: 0,
                VL53LX_p_015: 0,
                total_periods_elapsed: 0,
                peak_duration_us: 0,
                woi_duration_us: 0,
                min_bin_value: 0,
                max_bin_value: 0,
                zero_distance_phase: 0,
                number_of_ambient_samples: 0,
                ambient_events_sum: 0,
                VL53LX_p_028: 0,
                roi_config__user_roi_centre_spad: 0,
                roi_config__user_roi_requested_global_xy_size: 0,
            },
        },
        gain_cal: VL53LX_gain_calibration_data_t {
            standard_ranging_gain_factor: 0,
            histogram_ranging_gain_factor: 0,
        },
        cal_peak_rate_map: VL53LX_cal_peak_rate_map_t {
            cal_distance_mm: 0,
            cal_reflectance_pc: 0,
            max_samples: 0,
            width: 0,
            height: 0,
            peak_rate_mcps: [0; 25],
        },
        per_vcsel_cal_data: VL53LX_per_vcsel_period_offset_cal_data_t {
            short_a_offset_mm: 0,
            short_b_offset_mm: 0,
            medium_a_offset_mm: 0,
            medium_b_offset_mm: 0,
            long_a_offset_mm: 0,
            long_b_offset_mm: 0,
        },
    };
    let mut pC: *mut VL53LX_CustomerNvmManaged_t = 0 as *mut VL53LX_CustomerNvmManaged_t;
    let mut pC2: *mut VL53LX_customer_nvm_managed_t = 0
        as *mut VL53LX_customer_nvm_managed_t;
    let mut xtalk: VL53LX_xtalk_calibration_results_t = VL53LX_xtalk_calibration_results_t {
        algo__crosstalk_compensation_plane_offset_kcps: 0,
        algo__crosstalk_compensation_x_plane_gradient_kcps: 0,
        algo__crosstalk_compensation_y_plane_gradient_kcps: 0,
        algo__xtalk_cpo_HistoMerge_kcps: [0; 6],
    };
    let mut tmp: u32 = 0;
    Status = VL53LX_get_part_to_part_data(Dev, &mut cal_data);
    (*pCalibrationData)
        .struct_version = (cal_data.struct_version)
        .wrapping_add(0x20 as libc::c_int as libc::c_uint);
    memcpy(
        &mut (*pCalibrationData).add_off_cal_data
            as *mut VL53LX_additional_offset_cal_data_t as *mut libc::c_void,
        &mut cal_data.add_off_cal_data as *mut VL53LX_additional_offset_cal_data_t
            as *const libc::c_void,
        ::std::mem::size_of::<VL53LX_additional_offset_cal_data_t>() as libc::c_ulong,
    );
    memcpy(
        &mut (*pCalibrationData).optical_centre as *mut VL53LX_optical_centre_t
            as *mut libc::c_void,
        &mut cal_data.optical_centre as *mut VL53LX_optical_centre_t
            as *const libc::c_void,
        ::std::mem::size_of::<VL53LX_optical_centre_t>() as libc::c_ulong,
    );
    memcpy(
        &mut (*pCalibrationData).xtalkhisto as *mut VL53LX_xtalk_histogram_data_t
            as *mut libc::c_void,
        &mut cal_data.xtalkhisto as *mut VL53LX_xtalk_histogram_data_t
            as *const libc::c_void,
        ::std::mem::size_of::<VL53LX_xtalk_histogram_data_t>() as libc::c_ulong,
    );
    memcpy(
        &mut (*pCalibrationData).gain_cal as *mut VL53LX_gain_calibration_data_t
            as *mut libc::c_void,
        &mut cal_data.gain_cal as *mut VL53LX_gain_calibration_data_t
            as *const libc::c_void,
        ::std::mem::size_of::<VL53LX_gain_calibration_data_t>() as libc::c_ulong,
    );
    memcpy(
        &mut (*pCalibrationData).cal_peak_rate_map as *mut VL53LX_cal_peak_rate_map_t
            as *mut libc::c_void,
        &mut cal_data.cal_peak_rate_map as *mut VL53LX_cal_peak_rate_map_t
            as *const libc::c_void,
        ::std::mem::size_of::<VL53LX_cal_peak_rate_map_t>() as libc::c_ulong,
    );
    memcpy(
        &mut (*pCalibrationData).per_vcsel_cal_data
            as *mut VL53LX_per_vcsel_period_offset_cal_data_t as *mut libc::c_void,
        &mut cal_data.per_vcsel_cal_data
            as *mut VL53LX_per_vcsel_period_offset_cal_data_t as *const libc::c_void,
        ::std::mem::size_of::<VL53LX_per_vcsel_period_offset_cal_data_t>()
            as libc::c_ulong,
    );
    pC = &mut (*pCalibrationData).customer;
    pC2 = &mut cal_data.customer;
    (*pC).global_config__spad_enables_ref_0 = (*pC2).global_config__spad_enables_ref_0;
    (*pC).global_config__spad_enables_ref_1 = (*pC2).global_config__spad_enables_ref_1;
    (*pC).global_config__spad_enables_ref_2 = (*pC2).global_config__spad_enables_ref_2;
    (*pC).global_config__spad_enables_ref_3 = (*pC2).global_config__spad_enables_ref_3;
    (*pC).global_config__spad_enables_ref_4 = (*pC2).global_config__spad_enables_ref_4;
    (*pC).global_config__spad_enables_ref_5 = (*pC2).global_config__spad_enables_ref_5;
    (*pC).global_config__ref_en_start_select = (*pC2).global_config__ref_en_start_select;
    (*pC)
        .ref_spad_man__num_requested_ref_spads = (*pC2)
        .ref_spad_man__num_requested_ref_spads;
    (*pC).ref_spad_man__ref_location = (*pC2).ref_spad_man__ref_location;
    (*pC)
        .algo__crosstalk_compensation_x_plane_gradient_kcps = (*pC2)
        .algo__crosstalk_compensation_x_plane_gradient_kcps;
    (*pC)
        .algo__crosstalk_compensation_y_plane_gradient_kcps = (*pC2)
        .algo__crosstalk_compensation_y_plane_gradient_kcps;
    (*pC)
        .ref_spad_char__total_rate_target_mcps = (*pC2)
        .ref_spad_char__total_rate_target_mcps;
    (*pC).algo__part_to_part_range_offset_mm = (*pC2).algo__part_to_part_range_offset_mm;
    (*pC).mm_config__inner_offset_mm = (*pC2).mm_config__inner_offset_mm;
    (*pC).mm_config__outer_offset_mm = (*pC2).mm_config__outer_offset_mm;
    (*pC)
        .algo__crosstalk_compensation_plane_offset_kcps = (*pC2)
        .algo__crosstalk_compensation_plane_offset_kcps as u32;
    Status = VL53LX_get_current_xtalk_settings(Dev, &mut xtalk);
    if !(Status as libc::c_int != 0 as libc::c_int as VL53LX_Error as libc::c_int) {
        tmp = xtalk.algo__crosstalk_compensation_plane_offset_kcps;
        (*pC).algo__crosstalk_compensation_plane_offset_kcps = tmp;
        tmp = xtalk.algo__crosstalk_compensation_x_plane_gradient_kcps as u32;
        (*pC).algo__crosstalk_compensation_x_plane_gradient_kcps = tmp as i16;
        tmp = xtalk.algo__crosstalk_compensation_y_plane_gradient_kcps as u32;
        (*pC).algo__crosstalk_compensation_y_plane_gradient_kcps = tmp as i16;
        memcpy(
            &mut *((*pCalibrationData).algo__xtalk_cpo_HistoMerge_kcps)
                .as_mut_ptr()
                .offset(0 as libc::c_int as isize) as *mut u32 as *mut libc::c_void,
            &mut *(xtalk.algo__xtalk_cpo_HistoMerge_kcps)
                .as_mut_ptr()
                .offset(0 as libc::c_int as isize) as *mut u32
                as *const libc::c_void,
            ::std::mem::size_of::<[u32; 6]>() as libc::c_ulong,
        );
    }
    return Status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_PerformOffsetPerVcselCalibration(
    mut Dev: VL53LX_DEV,
    mut CalDistanceMilliMeter: i32,
) -> VL53LX_Error {
    let mut Status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut sum_ranging_range_A: i32 = 0;
    let mut sum_ranging_range_B: i32 = 0;
    let mut offset_meas_range_A: u8 = 0;
    let mut offset_meas_range_B: u8 = 0;
    let mut Max: i16 = 0;
    let mut UnderMax: i16 = 0;
    let mut OverMax: i16 = 0;
    let mut Repeat: i16 = 0;
    let mut inloopcount: i32 = 0;
    let mut IncRounding: i32 = 0;
    let mut meanDistance_mm: i16 = 0;
    let mut RangingMeasurementData: VL53LX_MultiRangingData_t = VL53LX_MultiRangingData_t {
        TimeStamp: 0,
        StreamCount: 0,
        NumberOfObjectsFound: 0,
        RangeData: [VL53LX_TargetRangeData_t {
            RangeMaxMilliMeter: 0,
            RangeMinMilliMeter: 0,
            SignalRateRtnMegaCps: 0,
            AmbientRateRtnMegaCps: 0,
            SigmaMilliMeter: 0,
            RangeMilliMeter: 0,
            RangeStatus: 0,
            ExtendedRange: 0,
        }; 4],
        HasXtalkValueChanged: 0,
        EffectiveSpadRtnCount: 0,
    };
    let mut pdev: *mut VL53LX_LLDriverData_t = 0 as *mut VL53LX_LLDriverData_t;
    let mut goodmeas: u8 = 0;
    let mut currentDist: VL53LX_DistanceModes = 0;
    let mut DistMode: [VL53LX_DistanceModes; 3] = [
        1 as libc::c_int as VL53LX_DistanceModes,
        2 as libc::c_int as VL53LX_DistanceModes,
        3 as libc::c_int as VL53LX_DistanceModes,
    ];
    let mut offsetA: [i16; 3] = [
        0 as libc::c_int as i16,
        0 as libc::c_int as i16,
        0 as libc::c_int as i16,
    ];
    let mut offsetB: [i16; 3] = [
        0 as libc::c_int as i16,
        0 as libc::c_int as i16,
        0 as libc::c_int as i16,
    ];
    let mut SmudgeStatus: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut smudge_corr_en: u8 = 0;
    let mut ics: u8 = 0;
    let mut pRange: *mut VL53LX_TargetRangeData_t = 0 as *mut VL53LX_TargetRangeData_t;
    pdev = &mut (*Dev).Data.LLData;
    smudge_corr_en = (*pdev).smudge_correct_config.smudge_corr_enabled;
    SmudgeStatus = VL53LX_dynamic_xtalk_correction_disable(Dev);
    (*pdev).customer.algo__part_to_part_range_offset_mm = 0 as libc::c_int as i16;
    (*pdev).customer.mm_config__inner_offset_mm = 0 as libc::c_int as i16;
    (*pdev).customer.mm_config__outer_offset_mm = 0 as libc::c_int as i16;
    (*pdev).customer.mm_config__outer_offset_mm = 0 as libc::c_int as i16;
    memset(
        &mut (*pdev).per_vcsel_cal_data as *mut VL53LX_per_vcsel_period_offset_cal_data_t
            as *mut libc::c_void,
        0 as libc::c_int,
        ::std::mem::size_of::<VL53LX_per_vcsel_period_offset_cal_data_t>()
            as libc::c_ulong,
    );
    Repeat = 0 as libc::c_int as i16;
    if IsL4(Dev) != 0 {
        Repeat = 1 as libc::c_int as i16;
    }
    Max = (2 as libc::c_int
        * BDTable[VL53LX_TUNING_MAX_SIMPLE_OFFSET_CALIBRATION_SAMPLE_NUMBER
            as libc::c_int as usize]) as i16;
    UnderMax = (1 as libc::c_int + Max as libc::c_int / 2 as libc::c_int) as i16;
    OverMax = (Max as libc::c_int + Max as libc::c_int / 2 as libc::c_int) as i16;
    Status = VL53LX_GetDistanceMode(Dev, &mut currentDist);
    while (Repeat as libc::c_int) < 3 as libc::c_int
        && Status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int
    {
        Status = VL53LX_SetDistanceMode(Dev, DistMode[Repeat as usize]);
        Status = VL53LX_StartMeasurement(Dev);
        if Status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
            VL53LX_WaitMeasurementDataReady(Dev);
            VL53LX_GetMultiRangingData(Dev, &mut RangingMeasurementData);
            VL53LX_ClearInterruptAndStartMeasurement(Dev);
        }
        inloopcount = 0 as libc::c_int;
        offset_meas_range_A = 0 as libc::c_int as u8;
        sum_ranging_range_A = 0 as libc::c_int;
        offset_meas_range_B = 0 as libc::c_int as u8;
        sum_ranging_range_B = 0 as libc::c_int;
        while Status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int
            && inloopcount < Max as libc::c_int && inloopcount < OverMax as libc::c_int
        {
            Status = VL53LX_WaitMeasurementDataReady(Dev);
            if Status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
                Status = VL53LX_GetMultiRangingData(Dev, &mut RangingMeasurementData);
            }
            pRange = &mut *(RangingMeasurementData.RangeData)
                .as_mut_ptr()
                .offset(0 as libc::c_int as isize) as *mut VL53LX_TargetRangeData_t;
            goodmeas = ((*pRange).RangeStatus as libc::c_int == 0 as libc::c_int)
                as libc::c_int as u8;
            ics = (*pdev).ll_state.cfg_internal_stream_count;
            if Status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int
                && goodmeas as libc::c_int != 0
            {
                if ics as libc::c_int & 0x1 as libc::c_int != 0 {
                    sum_ranging_range_A += (*pRange).RangeMilliMeter as libc::c_int;
                    offset_meas_range_A = offset_meas_range_A.wrapping_add(1);
                } else {
                    sum_ranging_range_B += (*pRange).RangeMilliMeter as libc::c_int;
                    offset_meas_range_B = offset_meas_range_B.wrapping_add(1);
                }
                inloopcount = offset_meas_range_A as libc::c_int
                    + offset_meas_range_B as libc::c_int;
            }
            Status = VL53LX_ClearInterruptAndStartMeasurement(Dev);
        }
        if inloopcount < UnderMax as libc::c_int {
            Status = -(24 as libc::c_int) as VL53LX_Error;
        }
        VL53LX_StopMeasurement(Dev);
        if sum_ranging_range_A < 0 as libc::c_int
            || sum_ranging_range_B < 0 as libc::c_int
            || sum_ranging_range_A
                > offset_meas_range_A as i32 * 0xffff as libc::c_int
            || sum_ranging_range_B
                > offset_meas_range_B as i32 * 0xffff as libc::c_int
        {
            Status = -(32 as libc::c_int) as VL53LX_Error;
        }
        if Status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int
            && offset_meas_range_A as libc::c_int > 0 as libc::c_int
        {
            IncRounding = offset_meas_range_A as libc::c_int / 2 as libc::c_int;
            meanDistance_mm = ((sum_ranging_range_A + IncRounding)
                / offset_meas_range_A as libc::c_int) as i16;
            offsetA[Repeat
                as usize] = (CalDistanceMilliMeter as i16 as libc::c_int
                - meanDistance_mm as libc::c_int) as i16;
        }
        if Status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int
            && offset_meas_range_B as libc::c_int > 0 as libc::c_int
        {
            IncRounding = offset_meas_range_B as libc::c_int / 2 as libc::c_int;
            meanDistance_mm = ((sum_ranging_range_B + IncRounding)
                / offset_meas_range_B as libc::c_int) as i16;
            offsetB[Repeat
                as usize] = (CalDistanceMilliMeter as i16 as libc::c_int
                - meanDistance_mm as libc::c_int) as i16;
        }
        Repeat += 1;
    }
    if SmudgeStatus as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int
        && smudge_corr_en as libc::c_int == 1 as libc::c_int
    {
        SmudgeStatus = VL53LX_dynamic_xtalk_correction_enable(Dev);
    }
    if Status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        (*pdev)
            .per_vcsel_cal_data
            .short_a_offset_mm = offsetA[0 as libc::c_int as usize];
        (*pdev)
            .per_vcsel_cal_data
            .short_b_offset_mm = offsetB[0 as libc::c_int as usize];
        (*pdev)
            .per_vcsel_cal_data
            .medium_a_offset_mm = offsetA[1 as libc::c_int as usize];
        (*pdev)
            .per_vcsel_cal_data
            .medium_b_offset_mm = offsetB[1 as libc::c_int as usize];
        (*pdev).per_vcsel_cal_data.long_a_offset_mm = offsetA[2 as libc::c_int as usize];
        (*pdev).per_vcsel_cal_data.long_b_offset_mm = offsetB[2 as libc::c_int as usize];
    }
    VL53LX_SetDistanceMode(Dev, currentDist);
    return Status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_GetOpticalCenter(
    mut Dev: VL53LX_DEV,
    mut pOpticalCenterX: *mut FixPoint1616_t,
    mut pOpticalCenterY: *mut FixPoint1616_t,
) -> VL53LX_Error {
    let mut Status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut CalibrationData: VL53LX_calibration_data_t = VL53LX_calibration_data_t {
        struct_version: 0,
        customer: VL53LX_customer_nvm_managed_t {
            global_config__spad_enables_ref_0: 0,
            global_config__spad_enables_ref_1: 0,
            global_config__spad_enables_ref_2: 0,
            global_config__spad_enables_ref_3: 0,
            global_config__spad_enables_ref_4: 0,
            global_config__spad_enables_ref_5: 0,
            global_config__ref_en_start_select: 0,
            ref_spad_man__num_requested_ref_spads: 0,
            ref_spad_man__ref_location: 0,
            algo__crosstalk_compensation_plane_offset_kcps: 0,
            algo__crosstalk_compensation_x_plane_gradient_kcps: 0,
            algo__crosstalk_compensation_y_plane_gradient_kcps: 0,
            ref_spad_char__total_rate_target_mcps: 0,
            algo__part_to_part_range_offset_mm: 0,
            mm_config__inner_offset_mm: 0,
            mm_config__outer_offset_mm: 0,
        },
        fmt_dmax_cal: VL53LX_dmax_calibration_data_t {
            ref__actual_effective_spads: 0,
            ref__peak_signal_count_rate_mcps: 0,
            ref__distance_mm: 0,
            ref_reflectance_pc: 0,
            coverglass_transmission: 0,
        },
        cust_dmax_cal: VL53LX_dmax_calibration_data_t {
            ref__actual_effective_spads: 0,
            ref__peak_signal_count_rate_mcps: 0,
            ref__distance_mm: 0,
            ref_reflectance_pc: 0,
            coverglass_transmission: 0,
        },
        add_off_cal_data: VL53LX_additional_offset_cal_data_t {
            result__mm_inner_actual_effective_spads: 0,
            result__mm_outer_actual_effective_spads: 0,
            result__mm_inner_peak_signal_count_rtn_mcps: 0,
            result__mm_outer_peak_signal_count_rtn_mcps: 0,
        },
        optical_centre: VL53LX_optical_centre_t {
            x_centre: 0,
            y_centre: 0,
        },
        xtalkhisto: VL53LX_xtalk_histogram_data_t {
            xtalk_shape: VL53LX_xtalk_histogram_shape_t {
                zone_id: 0,
                time_stamp: 0,
                VL53LX_p_019: 0,
                VL53LX_p_020: 0,
                VL53LX_p_021: 0,
                bin_data: [0; 12],
                phasecal_result__reference_phase: 0,
                phasecal_result__vcsel_start: 0,
                cal_config__vcsel_start: 0,
                vcsel_width: 0,
                VL53LX_p_015: 0,
                zero_distance_phase: 0,
            },
            xtalk_hist_removed: VL53LX_histogram_bin_data_t {
                cfg_device_state: 0,
                rd_device_state: 0,
                zone_id: 0,
                time_stamp: 0,
                VL53LX_p_019: 0,
                VL53LX_p_020: 0,
                VL53LX_p_021: 0,
                number_of_ambient_bins: 0,
                bin_seq: [0; 6],
                bin_rep: [0; 6],
                bin_data: [0; 24],
                result__interrupt_status: 0,
                result__range_status: 0,
                result__report_status: 0,
                result__stream_count: 0,
                result__dss_actual_effective_spads: 0,
                phasecal_result__reference_phase: 0,
                phasecal_result__vcsel_start: 0,
                cal_config__vcsel_start: 0,
                vcsel_width: 0,
                VL53LX_p_005: 0,
                VL53LX_p_015: 0,
                total_periods_elapsed: 0,
                peak_duration_us: 0,
                woi_duration_us: 0,
                min_bin_value: 0,
                max_bin_value: 0,
                zero_distance_phase: 0,
                number_of_ambient_samples: 0,
                ambient_events_sum: 0,
                VL53LX_p_028: 0,
                roi_config__user_roi_centre_spad: 0,
                roi_config__user_roi_requested_global_xy_size: 0,
            },
        },
        gain_cal: VL53LX_gain_calibration_data_t {
            standard_ranging_gain_factor: 0,
            histogram_ranging_gain_factor: 0,
        },
        cal_peak_rate_map: VL53LX_cal_peak_rate_map_t {
            cal_distance_mm: 0,
            cal_reflectance_pc: 0,
            max_samples: 0,
            width: 0,
            height: 0,
            peak_rate_mcps: [0; 25],
        },
        per_vcsel_cal_data: VL53LX_per_vcsel_period_offset_cal_data_t {
            short_a_offset_mm: 0,
            short_b_offset_mm: 0,
            medium_a_offset_mm: 0,
            medium_b_offset_mm: 0,
            long_a_offset_mm: 0,
            long_b_offset_mm: 0,
        },
    };
    *pOpticalCenterX = 0 as libc::c_int as FixPoint1616_t;
    *pOpticalCenterY = 0 as libc::c_int as FixPoint1616_t;
    Status = VL53LX_get_part_to_part_data(Dev, &mut CalibrationData);
    if Status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        *pOpticalCenterX = (CalibrationData.optical_centre.x_centre as u32)
            << 12 as libc::c_int;
        *pOpticalCenterY = (CalibrationData.optical_centre.y_centre as u32)
            << 12 as libc::c_int;
    }
    return Status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_run_ref_spad_char(
    mut Dev: VL53LX_DEV,
    mut pcal_status: *mut VL53LX_Error,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    let mut comms_buffer: [u8; 6] = [0; 6];
    let mut prefspadchar: *mut VL53LX_refspadchar_config_t = &mut (*pdev).refspadchar;
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_enable_powerforce(Dev);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_set_ref_spad_char_config(
            Dev,
            (*prefspadchar).VL53LX_p_005,
            (*prefspadchar).timeout_us,
            (*prefspadchar).target_count_rate_mcps,
            (*prefspadchar).max_count_rate_limit_mcps,
            (*prefspadchar).min_count_rate_limit_mcps,
            (*pdev).stat_nvm.osc_measured__fast_osc__frequency,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_run_device_test(Dev, (*prefspadchar).device_test_mode);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_ReadMulti(
            Dev,
            0xd9 as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
            2 as libc::c_int as u32,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        (*pdev)
            .dbg_results
            .ref_spad_char_result__num_actual_ref_spads = comms_buffer[0 as libc::c_int
            as usize];
        (*pdev)
            .dbg_results
            .ref_spad_char_result__ref_location = comms_buffer[1 as libc::c_int
            as usize];
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_WriteMulti(
            Dev,
            0x14 as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
            2 as libc::c_int as u32,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        (*pdev)
            .customer
            .ref_spad_man__num_requested_ref_spads = comms_buffer[0 as libc::c_int
            as usize];
        (*pdev)
            .customer
            .ref_spad_man__ref_location = comms_buffer[1 as libc::c_int as usize];
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_ReadMulti(
            Dev,
            0xac as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
            6 as libc::c_int as u32,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_WriteMulti(
            Dev,
            0xd as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
            6 as libc::c_int as u32,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        (*pdev)
            .customer
            .global_config__spad_enables_ref_0 = comms_buffer[0 as libc::c_int as usize];
        (*pdev)
            .customer
            .global_config__spad_enables_ref_1 = comms_buffer[1 as libc::c_int as usize];
        (*pdev)
            .customer
            .global_config__spad_enables_ref_2 = comms_buffer[2 as libc::c_int as usize];
        (*pdev)
            .customer
            .global_config__spad_enables_ref_3 = comms_buffer[3 as libc::c_int as usize];
        (*pdev)
            .customer
            .global_config__spad_enables_ref_4 = comms_buffer[4 as libc::c_int as usize];
        (*pdev)
            .customer
            .global_config__spad_enables_ref_5 = comms_buffer[5 as libc::c_int as usize];
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        match (*pdev).sys_results.result__range_status as libc::c_int {
            14 => {
                status = -(28 as libc::c_int) as VL53LX_Error;
            }
            15 => {
                status = -(29 as libc::c_int) as VL53LX_Error;
            }
            16 => {
                status = -(30 as libc::c_int) as VL53LX_Error;
            }
            _ => {}
        }
    }
    *pcal_status = status;
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_get_and_avg_xtalk_samples(
    mut Dev: VL53LX_DEV,
    mut num_of_samples: u8,
    mut measurement_mode: u8,
    mut xtalk_filter_thresh_max_mm: i16,
    mut xtalk_filter_thresh_min_mm: i16,
    mut xtalk_max_valid_rate_kcps: u16,
    mut xtalk_result_id: u8,
    mut xtalk_histo_id: u8,
    mut pXR: *mut VL53LX_xtalk_range_results_t,
    mut psum_histo: *mut VL53LX_histogram_bin_data_t,
    mut pavg_histo: *mut VL53LX_histogram_bin_data_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    let mut prs: *mut VL53LX_range_results_t = ((*pdev).wArea1).as_mut_ptr()
        as *mut VL53LX_range_results_t;
    let mut prange_data: *mut VL53LX_range_data_t = 0 as *mut VL53LX_range_data_t;
    let mut pxtalk_range_data: *mut VL53LX_xtalk_range_data_t = 0
        as *mut VL53LX_xtalk_range_data_t;
    let mut i: u8 = 0 as libc::c_int as u8;
    let mut j: u8 = 0 as libc::c_int as u8;
    let mut zone_id: u8 = 0 as libc::c_int as u8;
    let mut final_zone: u8 = ((*pdev).zone_cfg.active_zones as libc::c_int
        + 1 as libc::c_int) as u8;
    let mut valid_result: u8 = 0;
    let mut smudge_corr_en: u8 = 0 as libc::c_int as u8;
    smudge_corr_en = (*pdev).smudge_correct_config.smudge_corr_enabled;
    status = VL53LX_dynamic_xtalk_correction_disable(Dev);
    VL53LX_load_patch(Dev);
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_init_and_start_range(
            Dev,
            measurement_mode,
            5 as libc::c_int as VL53LX_DeviceConfigLevel,
        );
    }
    i = 0 as libc::c_int as u8;
    while i as libc::c_int <= final_zone as libc::c_int * num_of_samples as libc::c_int {
        if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
            status = VL53LX_wait_for_range_completion(Dev);
        }
        if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
            status = VL53LX_get_device_results(
                Dev,
                2 as libc::c_int as VL53LX_DeviceResultsLevel,
                prs,
            );
        }
        if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int
            && (*pdev).ll_state.rd_device_state as libc::c_int
                != 6 as libc::c_int as VL53LX_DeviceState as libc::c_int
        {
            zone_id = ((*pdev).ll_state.rd_zone_id as libc::c_int
                + xtalk_result_id as libc::c_int) as u8;
            prange_data = &mut *((*prs).VL53LX_p_003)
                .as_mut_ptr()
                .offset(0 as libc::c_int as isize) as *mut VL53LX_range_data_t;
            if (*prs).active_results as libc::c_int > 1 as libc::c_int {
                j = 1 as libc::c_int as u8;
                while (j as libc::c_int) < (*prs).active_results as libc::c_int {
                    if ((*prs).VL53LX_p_003[j as usize].median_range_mm as libc::c_int)
                        < (*prange_data).median_range_mm as libc::c_int
                    {
                        prange_data = &mut *((*prs).VL53LX_p_003)
                            .as_mut_ptr()
                            .offset(j as isize) as *mut VL53LX_range_data_t;
                    }
                    j = j.wrapping_add(1);
                }
            }
            pxtalk_range_data = &mut *((*pXR).VL53LX_p_003)
                .as_mut_ptr()
                .offset(zone_id as isize) as *mut VL53LX_xtalk_range_data_t;
            if (*prs).active_results as libc::c_int > 0 as libc::c_int
                && ((*prange_data).median_range_mm as libc::c_int)
                    < xtalk_filter_thresh_max_mm as libc::c_int
                && (*prange_data).median_range_mm as libc::c_int
                    > xtalk_filter_thresh_min_mm as libc::c_int
                && (*prange_data).VL53LX_p_009
                    < (xtalk_max_valid_rate_kcps as libc::c_int * 16 as libc::c_int)
                        as u32
            {
                valid_result = 1 as libc::c_int as u8;
            } else {
                valid_result = 0 as libc::c_int as u8;
            }
            if valid_result as libc::c_int == 1 as libc::c_int {
                let ref mut fresh3 = (*pxtalk_range_data).no_of_samples;
                *fresh3 = (*fresh3).wrapping_add(1);
                let ref mut fresh4 = (*pxtalk_range_data).rate_per_spad_kcps_sum;
                *fresh4 = (*fresh4 as libc::c_uint)
                    .wrapping_add((*prange_data).VL53LX_p_009) as u32 as u32;
                let ref mut fresh5 = (*pxtalk_range_data).signal_total_events_sum;
                *fresh5 += (*prange_data).VL53LX_p_010;
                let ref mut fresh6 = (*pxtalk_range_data).sigma_mm_sum;
                *fresh6 = (*fresh6 as libc::c_uint)
                    .wrapping_add((*prange_data).VL53LX_p_002 as u32) as u32
                    as u32;
                let ref mut fresh7 = (*pxtalk_range_data).median_phase_sum;
                *fresh7 = (*fresh7 as libc::c_uint)
                    .wrapping_add((*prange_data).VL53LX_p_011 as u32) as u32
                    as u32;
            }
            if valid_result as libc::c_int == 1 as libc::c_int
                && zone_id as libc::c_int >= 4 as libc::c_int
            {
                status = VL53LX_sum_histogram_data(&mut (*pdev).hist_data, psum_histo);
                if ((*prange_data).VL53LX_p_012 as libc::c_int)
                    < (*pXR).central_histogram__window_start as libc::c_int
                {
                    (*pXR).central_histogram__window_start = (*prange_data).VL53LX_p_012;
                }
                if (*prange_data).VL53LX_p_013 as libc::c_int
                    > (*pXR).central_histogram__window_end as libc::c_int
                {
                    (*pXR).central_histogram__window_end = (*prange_data).VL53LX_p_013;
                }
            }
        }
        if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
            status = VL53LX_clear_interrupt_and_enable_next_range(Dev, measurement_mode);
        }
        i = i.wrapping_add(1);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_stop_range(Dev);
    }
    VL53LX_unload_patch(Dev);
    i = 0 as libc::c_int as u8;
    while (i as libc::c_int)
        < (*pdev).zone_cfg.active_zones as libc::c_int + 1 as libc::c_int
    {
        pxtalk_range_data = &mut *((*pXR).VL53LX_p_003)
            .as_mut_ptr()
            .offset((i as libc::c_int + xtalk_result_id as libc::c_int) as isize)
            as *mut VL53LX_xtalk_range_data_t;
        if (*pxtalk_range_data).no_of_samples as libc::c_int > 0 as libc::c_int {
            (*pxtalk_range_data)
                .rate_per_spad_kcps_avg = ((*pxtalk_range_data).rate_per_spad_kcps_sum)
                .wrapping_div((*pxtalk_range_data).no_of_samples as u32);
            (*pxtalk_range_data)
                .signal_total_events_avg = (*pxtalk_range_data).signal_total_events_sum
                / (*pxtalk_range_data).no_of_samples as i32;
            (*pxtalk_range_data)
                .sigma_mm_avg = ((*pxtalk_range_data).sigma_mm_sum)
                .wrapping_div((*pxtalk_range_data).no_of_samples as u32);
            (*pxtalk_range_data)
                .median_phase_avg = ((*pxtalk_range_data).median_phase_sum)
                .wrapping_div((*pxtalk_range_data).no_of_samples as u32);
        } else {
            (*pxtalk_range_data)
                .rate_per_spad_kcps_avg = (*pxtalk_range_data).rate_per_spad_kcps_sum;
            (*pxtalk_range_data)
                .signal_total_events_avg = (*pxtalk_range_data).signal_total_events_sum;
            (*pxtalk_range_data).sigma_mm_avg = (*pxtalk_range_data).sigma_mm_sum;
            (*pxtalk_range_data)
                .median_phase_avg = (*pxtalk_range_data).median_phase_sum;
        }
        i = i.wrapping_add(1);
    }
    memcpy(
        pavg_histo as *mut libc::c_void,
        &mut (*pdev).hist_data as *mut VL53LX_histogram_bin_data_t
            as *const libc::c_void,
        ::std::mem::size_of::<VL53LX_histogram_bin_data_t>() as libc::c_ulong,
    );
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        pxtalk_range_data = &mut *((*pXR).VL53LX_p_003)
            .as_mut_ptr()
            .offset(xtalk_histo_id as isize) as *mut VL53LX_xtalk_range_data_t;
        status = VL53LX_avg_histogram_data(
            (*pxtalk_range_data).no_of_samples,
            psum_histo,
            pavg_histo,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        if smudge_corr_en as libc::c_int == 1 as libc::c_int {
            status = VL53LX_dynamic_xtalk_correction_enable(Dev);
        }
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_run_phasecal_average(
    mut Dev: VL53LX_DEV,
    mut measurement_mode: u8,
    mut phasecal_result__vcsel_start: u8,
    mut phasecal_num_of_samples: u16,
    mut prange_results: *mut VL53LX_range_results_t,
    mut pphasecal_result__reference_phase: *mut u16,
    mut pzero_distance_phase: *mut u16,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    let mut i: u16 = 0 as libc::c_int as u16;
    let mut m: u16 = 0 as libc::c_int as u16;
    let mut samples: u32 = 0 as libc::c_int as u32;
    let mut period: u32 = 0 as libc::c_int as u32;
    let mut VL53LX_p_014: u32 = 0 as libc::c_int as u32;
    let mut phasecal_result__reference_phase: u32 = 0 as libc::c_int as u32;
    let mut zero_distance_phase: u32 = 0 as libc::c_int as u32;
    VL53LX_load_patch(Dev);
    m = 0 as libc::c_int as u16;
    while (m as libc::c_int) < phasecal_num_of_samples as libc::c_int {
        if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
            status = VL53LX_init_and_start_range(
                Dev,
                measurement_mode,
                5 as libc::c_int as VL53LX_DeviceConfigLevel,
            );
        }
        i = 0 as libc::c_int as u16;
        while i as libc::c_int <= 1 as libc::c_int {
            if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
                status = VL53LX_wait_for_range_completion(Dev);
            }
            if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
                status = VL53LX_get_device_results(
                    Dev,
                    2 as libc::c_int as VL53LX_DeviceResultsLevel,
                    prange_results,
                );
            }
            if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
                status = VL53LX_clear_interrupt_and_enable_next_range(
                    Dev,
                    measurement_mode,
                );
            }
            i = i.wrapping_add(1);
        }
        if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
            status = VL53LX_stop_range(Dev);
        }
        if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
            status = VL53LX_WaitUs(Dev, 1000 as libc::c_int);
        }
        if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
            samples = samples.wrapping_add(1);
            period = (2048 as libc::c_int as libc::c_uint)
                .wrapping_mul(
                    VL53LX_decode_vcsel_period((*pdev).hist_data.VL53LX_p_005)
                        as u32,
                );
            VL53LX_p_014 = period;
            VL53LX_p_014 = (VL53LX_p_014 as libc::c_uint)
                .wrapping_add(
                    (*pdev).hist_data.phasecal_result__reference_phase as u32,
                ) as u32 as u32;
            VL53LX_p_014 = (VL53LX_p_014 as libc::c_uint)
                .wrapping_add(
                    (2048 as libc::c_int as libc::c_uint)
                        .wrapping_mul(phasecal_result__vcsel_start as u32),
                ) as u32 as u32;
            VL53LX_p_014 = (VL53LX_p_014 as libc::c_uint)
                .wrapping_sub(
                    (2048 as libc::c_int as libc::c_uint)
                        .wrapping_mul(
                            (*pdev).hist_data.cal_config__vcsel_start as u32,
                        ),
                ) as u32 as u32;
            if period != 0 as libc::c_int as libc::c_uint {
                VL53LX_p_014 = VL53LX_p_014.wrapping_rem(period);
            } else {
                status = -(15 as libc::c_int) as VL53LX_Error;
                VL53LX_p_014 = 0 as libc::c_int as u32;
            }
            phasecal_result__reference_phase = (phasecal_result__reference_phase
                as libc::c_uint)
                .wrapping_add(
                    (*pdev).hist_data.phasecal_result__reference_phase as u32,
                ) as u32 as u32;
            zero_distance_phase = (zero_distance_phase as libc::c_uint)
                .wrapping_add(VL53LX_p_014) as u32 as u32;
        }
        m = m.wrapping_add(1);
    }
    VL53LX_unload_patch(Dev);
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int
        && samples > 0 as libc::c_int as libc::c_uint
    {
        phasecal_result__reference_phase = (phasecal_result__reference_phase
            as libc::c_uint)
            .wrapping_add(samples >> 1 as libc::c_int) as u32 as u32;
        phasecal_result__reference_phase = (phasecal_result__reference_phase
            as libc::c_uint)
            .wrapping_div(samples) as u32 as u32;
        zero_distance_phase = (zero_distance_phase as libc::c_uint)
            .wrapping_add(samples >> 1 as libc::c_int) as u32 as u32;
        zero_distance_phase = (zero_distance_phase as libc::c_uint).wrapping_div(samples)
            as u32 as u32;
        *pphasecal_result__reference_phase = phasecal_result__reference_phase
            as u16;
        *pzero_distance_phase = zero_distance_phase as u16;
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_run_device_test(
    mut Dev: VL53LX_DEV,
    mut device_test_mode: VL53LX_DeviceTestMode,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    let mut comms_buffer: [u8; 2] = [0; 2];
    let mut gpio_hv_mux__ctrl: u8 = 0 as libc::c_int as u8;
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_RdByte(
            Dev,
            0x30 as libc::c_int as u16,
            &mut gpio_hv_mux__ctrl,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        (*pdev).stat_cfg.gpio_hv_mux__ctrl = gpio_hv_mux__ctrl;
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_start_test(Dev, device_test_mode);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_wait_for_test_completion(Dev);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_ReadMulti(
            Dev,
            0x89 as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
            2 as libc::c_int as u32,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        (*pdev)
            .sys_results
            .result__range_status = comms_buffer[0 as libc::c_int as usize];
        (*pdev)
            .sys_results
            .result__report_status = comms_buffer[1 as libc::c_int as usize];
    }
    let ref mut fresh8 = (*pdev).sys_results.result__range_status;
    *fresh8 = (*fresh8 as libc::c_int & 0x1f as libc::c_int) as u8;
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
            status = VL53LX_clear_interrupt(Dev);
        }
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_start_test(Dev, 0 as libc::c_int as u8);
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_hist_xtalk_extract_data_init(
    mut pxtalk_data: *mut VL53LX_hist_xtalk_extract_data_t,
) {
    let mut lb: i32 = 0 as libc::c_int;
    (*pxtalk_data).sample_count = 0 as libc::c_uint;
    (*pxtalk_data).pll_period_mm = 0 as libc::c_uint;
    (*pxtalk_data).peak_duration_us_sum = 0 as libc::c_uint;
    (*pxtalk_data).effective_spad_count_sum = 0 as libc::c_uint;
    (*pxtalk_data).zero_distance_phase_sum = 0 as libc::c_uint;
    (*pxtalk_data).zero_distance_phase_avg = 0 as libc::c_uint;
    (*pxtalk_data).event_scaler_sum = 0 as libc::c_uint as i32;
    (*pxtalk_data).event_scaler_avg = 4096 as libc::c_uint as i32;
    (*pxtalk_data).signal_events_sum = 0 as libc::c_int;
    (*pxtalk_data).xtalk_rate_kcps_per_spad = 0 as libc::c_uint;
    (*pxtalk_data).VL53LX_p_012 = 0 as libc::c_uint as u8;
    (*pxtalk_data).VL53LX_p_013 = 0 as libc::c_uint as u8;
    (*pxtalk_data).target_start = 0 as libc::c_uint as u8;
    lb = 0 as libc::c_int;
    while lb < 12 as libc::c_int {
        (*pxtalk_data).bin_data_sums[lb as usize] = 0 as libc::c_int;
        lb += 1;
    }
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_hist_xtalk_extract_update(
    mut target_distance_mm: i16,
    mut target_width_oversize: u16,
    mut phist_bins: *mut VL53LX_histogram_bin_data_t,
    mut pxtalk_data: *mut VL53LX_hist_xtalk_extract_data_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    status = VL53LX_hist_xtalk_extract_calc_window(
        target_distance_mm,
        target_width_oversize,
        phist_bins,
        pxtalk_data,
    );
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_hist_xtalk_extract_calc_event_sums(phist_bins, pxtalk_data);
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_hist_xtalk_extract_fini(
    mut phist_bins: *mut VL53LX_histogram_bin_data_t,
    mut pxtalk_data: *mut VL53LX_hist_xtalk_extract_data_t,
    mut pxtalk_cal: *mut VL53LX_xtalk_calibration_results_t,
    mut pxtalk_shape: *mut VL53LX_xtalk_histogram_shape_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pX: *mut VL53LX_xtalk_calibration_results_t = pxtalk_cal;
    if (*pxtalk_data).sample_count > 0 as libc::c_int as libc::c_uint {
        (*pxtalk_data).event_scaler_avg = (*pxtalk_data).event_scaler_sum;
        let ref mut fresh9 = (*pxtalk_data).event_scaler_avg;
        *fresh9 = (*fresh9 as libc::c_uint)
            .wrapping_add((*pxtalk_data).sample_count >> 1 as libc::c_int) as i32
            as i32;
        let ref mut fresh10 = (*pxtalk_data).event_scaler_avg;
        *fresh10 = (*fresh10 as libc::c_uint).wrapping_div((*pxtalk_data).sample_count)
            as i32 as i32;
        status = VL53LX_hist_xtalk_extract_calc_rate_per_spad(pxtalk_data);
        if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
            (*pxtalk_data)
                .zero_distance_phase_avg = (*pxtalk_data).zero_distance_phase_sum;
            let ref mut fresh11 = (*pxtalk_data).zero_distance_phase_avg;
            *fresh11 = (*fresh11 as libc::c_uint)
                .wrapping_add((*pxtalk_data).sample_count >> 1 as libc::c_int)
                as u32 as u32;
            let ref mut fresh12 = (*pxtalk_data).zero_distance_phase_avg;
            *fresh12 = (*fresh12 as libc::c_uint)
                .wrapping_div((*pxtalk_data).sample_count) as u32 as u32;
            status = VL53LX_hist_xtalk_extract_calc_shape(pxtalk_data, pxtalk_shape);
            (*pxtalk_shape)
                .phasecal_result__vcsel_start = (*phist_bins)
                .phasecal_result__vcsel_start;
            (*pxtalk_shape)
                .cal_config__vcsel_start = (*phist_bins).cal_config__vcsel_start;
            (*pxtalk_shape).vcsel_width = (*phist_bins).vcsel_width;
            (*pxtalk_shape).VL53LX_p_015 = (*phist_bins).VL53LX_p_015;
        }
        if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
            (*pX)
                .algo__crosstalk_compensation_plane_offset_kcps = (*pxtalk_data)
                .xtalk_rate_kcps_per_spad;
            (*pX)
                .algo__crosstalk_compensation_x_plane_gradient_kcps = 0 as libc::c_uint
                as i16;
            (*pX)
                .algo__crosstalk_compensation_y_plane_gradient_kcps = 0 as libc::c_uint
                as i16;
        }
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_run_hist_xtalk_extraction(
    mut Dev: VL53LX_DEV,
    mut cal_distance_mm: i16,
    mut pcal_status: *mut VL53LX_Error,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    let mut pX: *mut VL53LX_xtalkextract_config_t = &mut (*pdev).xtalk_extract_cfg;
    let mut pC: *mut VL53LX_xtalk_config_t = &mut (*pdev).xtalk_cfg;
    let mut pXC: *mut VL53LX_xtalk_calibration_results_t = &mut (*pdev).xtalk_cal;
    let mut smudge_corr_en: u8 = 0 as libc::c_int as u8;
    let mut i: u8 = 0 as libc::c_int as u8;
    let mut k: i8 = 0 as libc::c_int as i8;
    let mut nbloops: u8 = 0;
    let mut initMergeSize: i32 = 0 as libc::c_int;
    let mut MergeEnabled: i32 = 0 as libc::c_int;
    let mut deltaXtalk: u32 = 0;
    let mut stepXtalk: u32 = 0;
    let mut XtalkMin: u32 = 0;
    let mut XtalkMax: u32 = 0;
    let mut measurement_mode: u8 = 0x20 as libc::c_int
        as VL53LX_DeviceMeasurementModes;
    let mut MaxId: i8 = 0;
    let mut histo_merge_nb: u8 = 0;
    let mut wait_for_accumulation: u8 = 0;
    let mut prange_results: *mut VL53LX_range_results_t = ((*pdev).wArea1).as_mut_ptr()
        as *mut VL53LX_range_results_t;
    let mut Very1stRange: u8 = 0 as libc::c_int as u8;
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_set_preset_mode(
            Dev,
            27 as libc::c_int as VL53LX_DevicePresetModes,
            (*pX).dss_config__target_total_rate_mcps,
            (*pX).phasecal_config_timeout_us,
            (*pX).mm_config_timeout_us,
            (*pX).range_config_timeout_us,
            100 as libc::c_int as u32,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_disable_xtalk_compensation(Dev);
    }
    smudge_corr_en = (*pdev).smudge_correct_config.smudge_corr_enabled;
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_dynamic_xtalk_correction_disable(Dev);
    }
    VL53LX_load_patch(Dev);
    VL53LX_get_tuning_parm(
        Dev,
        (0x8000 as libc::c_int + 143 as libc::c_int) as VL53LX_TuningParms,
        &mut initMergeSize,
    );
    VL53LX_get_tuning_parm(
        Dev,
        (0x8000 as libc::c_int + 141 as libc::c_int) as VL53LX_TuningParms,
        &mut MergeEnabled,
    );
    memset(
        &mut (*pdev).xtalk_cal as *mut VL53LX_xtalk_calibration_results_t
            as *mut libc::c_void,
        0 as libc::c_int,
        ::std::mem::size_of::<VL53LX_xtalk_calibration_results_t>() as libc::c_ulong,
    );
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_init_and_start_range(
            Dev,
            measurement_mode,
            5 as libc::c_int as VL53LX_DeviceConfigLevel,
        );
    }
    MaxId = ((*pdev).tuning_parms.tp_hist_merge_max_size as libc::c_int
        - 1 as libc::c_int) as i8;
    nbloops = (if MergeEnabled == 0 as libc::c_int {
        1 as libc::c_int
    } else {
        2 as libc::c_int
    }) as u8;
    k = 0 as libc::c_int as i8;
    while (k as libc::c_int) < nbloops as libc::c_int {
        VL53LX_hist_xtalk_extract_data_init(&mut (*pdev).xtalk_extract);
        VL53LX_set_tuning_parm(
            Dev,
            (0x8000 as libc::c_int + 143 as libc::c_int) as VL53LX_TuningParms,
            k as libc::c_int * MaxId as libc::c_int + 1 as libc::c_int,
        );
        i = 0 as libc::c_int as u8;
        while i as libc::c_int <= (*pX).num_of_samples as libc::c_int {
            if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
                status = VL53LX_wait_for_range_completion(Dev);
            }
            if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
                status = VL53LX_get_device_results(
                    Dev,
                    2 as libc::c_int as VL53LX_DeviceResultsLevel,
                    prange_results,
                );
            }
            Very1stRange = ((*pdev).ll_state.rd_device_state as libc::c_int
                == 6 as libc::c_int as VL53LX_DeviceState as libc::c_int) as libc::c_int
                as u8;
            VL53LX_compute_histo_merge_nb(Dev, &mut histo_merge_nb);
            wait_for_accumulation = (k as libc::c_int != 0 as libc::c_int
                && MergeEnabled != 0
                && status as libc::c_int
                    == 0 as libc::c_int as VL53LX_Error as libc::c_int
                && (histo_merge_nb as libc::c_int)
                    < (*pdev).tuning_parms.tp_hist_merge_max_size as libc::c_int)
                as libc::c_int as u8;
            if wait_for_accumulation != 0 {
                i = 0 as libc::c_int as u8;
            } else if status as libc::c_int
                    == 0 as libc::c_int as VL53LX_Error as libc::c_int
                    && Very1stRange == 0
                {
                status = VL53LX_hist_xtalk_extract_update(
                    cal_distance_mm,
                    4 as libc::c_int as u16,
                    &mut (*pdev).hist_data,
                    &mut (*pdev).xtalk_extract,
                );
            }
            if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
                status = VL53LX_clear_interrupt_and_enable_next_range(
                    Dev,
                    measurement_mode,
                );
            }
            i = i.wrapping_add(1);
        }
        if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
            status = VL53LX_hist_xtalk_extract_fini(
                &mut (*pdev).hist_data,
                &mut (*pdev).xtalk_extract,
                &mut (*pdev).xtalk_cal,
                &mut (*pdev).xtalk_shapes.xtalk_shape,
            );
        }
        if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
            (*pXC)
                .algo__xtalk_cpo_HistoMerge_kcps[(k as libc::c_int
                * MaxId as libc::c_int)
                as usize] = (*pXC).algo__crosstalk_compensation_plane_offset_kcps;
        }
        k += 1;
    }
    VL53LX_stop_range(Dev);
    VL53LX_set_tuning_parm(
        Dev,
        (0x8000 as libc::c_int + 143 as libc::c_int) as VL53LX_TuningParms,
        initMergeSize,
    );
    VL53LX_unload_patch(Dev);
    if status as libc::c_int != 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = -(23 as libc::c_int) as VL53LX_Error;
    } else if MergeEnabled == 1 as libc::c_int && MaxId as libc::c_int > 0 as libc::c_int
        {
        XtalkMin = (*pdev)
            .xtalk_cal
            .algo__xtalk_cpo_HistoMerge_kcps[0 as libc::c_int as usize];
        XtalkMax = (*pdev).xtalk_cal.algo__xtalk_cpo_HistoMerge_kcps[MaxId as usize];
        (*pdev).xtalk_cal.algo__crosstalk_compensation_plane_offset_kcps = XtalkMin;
        if XtalkMax > XtalkMin {
            deltaXtalk = XtalkMax.wrapping_sub(XtalkMin);
            stepXtalk = deltaXtalk.wrapping_div(MaxId as libc::c_uint);
            k = 1 as libc::c_int as i8;
            while (k as libc::c_int) < MaxId as libc::c_int {
                (*pdev)
                    .xtalk_cal
                    .algo__xtalk_cpo_HistoMerge_kcps[k
                    as usize] = XtalkMin
                    .wrapping_add(stepXtalk.wrapping_mul(k as libc::c_uint));
                k += 1;
            }
        } else {
            status = -(23 as libc::c_int) as VL53LX_Error;
        }
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        (*pC)
            .algo__crosstalk_compensation_x_plane_gradient_kcps = (*pXC)
            .algo__crosstalk_compensation_x_plane_gradient_kcps;
        (*pC)
            .algo__crosstalk_compensation_y_plane_gradient_kcps = (*pXC)
            .algo__crosstalk_compensation_y_plane_gradient_kcps;
        (*pC)
            .algo__crosstalk_compensation_plane_offset_kcps = (*pXC)
            .algo__crosstalk_compensation_plane_offset_kcps;
    }
    (*pdev).xtalk_results.cal_status = status;
    *pcal_status = (*pdev).xtalk_results.cal_status;
    status = VL53LX_enable_xtalk_compensation(Dev);
    if smudge_corr_en as libc::c_int == 1 as libc::c_int {
        status = VL53LX_dynamic_xtalk_correction_enable(Dev);
    }
    return status;
}
unsafe extern "C" fn select_offset_per_vcsel(
    mut pdev: *mut VL53LX_LLDriverData_t,
    mut poffset: *mut i16,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut tA: i16 = 0;
    let mut tB: i16 = 0;
    let mut isc: u8 = 0;
    match (*pdev).preset_mode as libc::c_int {
        33 => {
            tA = (*pdev).per_vcsel_cal_data.short_a_offset_mm;
            tB = (*pdev).per_vcsel_cal_data.short_b_offset_mm;
        }
        30 => {
            tA = (*pdev).per_vcsel_cal_data.medium_a_offset_mm;
            tB = (*pdev).per_vcsel_cal_data.medium_b_offset_mm;
        }
        27 => {
            tA = (*pdev).per_vcsel_cal_data.long_a_offset_mm;
            tB = (*pdev).per_vcsel_cal_data.long_b_offset_mm;
        }
        _ => {
            tA = (*pdev).per_vcsel_cal_data.long_a_offset_mm;
            tB = (*pdev).per_vcsel_cal_data.long_b_offset_mm;
            status = -(4 as libc::c_int) as VL53LX_Error;
            *poffset = 0 as libc::c_int as i16;
        }
    }
    isc = (*pdev).ll_state.cfg_internal_stream_count;
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        *poffset = (if isc as libc::c_int & 0x1 as libc::c_int != 0 {
            tA as libc::c_int
        } else {
            tB as libc::c_int
        }) as i16;
    }
    return status;
}
unsafe extern "C" fn vl53lx_diff_histo_stddev(
    mut pdev: *mut VL53LX_LLDriverData_t,
    mut pdata: *mut VL53LX_histogram_bin_data_t,
    mut timing: u8,
    mut HighIndex: u8,
    mut prev_pos: u8,
    mut pdiff_histo_stddev: *mut i32,
) {
    let mut bin: u16 = 0 as libc::c_int as u16;
    let mut total_rate_pre: i32 = 0 as libc::c_int;
    let mut total_rate_cur: i32 = 0 as libc::c_int;
    let mut PrevBin: i32 = 0;
    let mut CurrBin: i32 = 0;
    total_rate_pre = 0 as libc::c_int;
    total_rate_cur = 0 as libc::c_int;
    bin = (timing as libc::c_int * 4 as libc::c_int) as u16;
    while (bin as libc::c_int) < HighIndex as libc::c_int {
        total_rate_pre
            += (*pdev).multi_bins_rec[prev_pos as usize][timing as usize][bin as usize];
        total_rate_cur += (*pdata).bin_data[bin as usize];
        bin = bin.wrapping_add(1);
    }
    if total_rate_pre != 0 as libc::c_int && total_rate_cur != 0 as libc::c_int {
        bin = (timing as libc::c_int * 4 as libc::c_int) as u16;
        while (bin as libc::c_int) < HighIndex as libc::c_int {
            PrevBin = (*pdev)
                .multi_bins_rec[prev_pos as usize][timing as usize][bin as usize];
            PrevBin = PrevBin * 1000 as libc::c_int / total_rate_pre;
            CurrBin = (*pdata).bin_data[bin as usize] * 1000 as libc::c_int
                / total_rate_cur;
            *pdiff_histo_stddev += (PrevBin - CurrBin) * (PrevBin - CurrBin);
            bin = bin.wrapping_add(1);
        }
    }
}
unsafe extern "C" fn vl53lx_histo_merge(
    mut Dev: VL53LX_DEV,
    mut pdata: *mut VL53LX_histogram_bin_data_t,
) {
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    let mut bin: u16 = 0 as libc::c_int as u16;
    let mut i: u8 = 0 as libc::c_int as u8;
    let mut TuningBinRecSize: i32 = 0 as libc::c_int;
    let mut recom_been_reset: u8 = 0 as libc::c_int as u8;
    let mut timing: u8 = 0 as libc::c_int as u8;
    let mut rmt: i32 = 0 as libc::c_int;
    let mut diff_histo_stddev: i32 = 0 as libc::c_int;
    let mut HighIndex: u8 = 0;
    let mut prev_pos: u8 = 0;
    let mut BuffSize: u8 = 24 as libc::c_int as u8;
    let mut pos: u8 = 0;
    VL53LX_get_tuning_parm(
        Dev,
        (0x8000 as libc::c_int + 143 as libc::c_int) as VL53LX_TuningParms,
        &mut TuningBinRecSize,
    );
    VL53LX_get_tuning_parm(
        Dev,
        (0x8000 as libc::c_int + 142 as libc::c_int) as VL53LX_TuningParms,
        &mut rmt,
    );
    if (*pdev).pos_before_next_recom as libc::c_int == 0 as libc::c_int {
        timing = (1 as libc::c_int
            - (*pdata).result__stream_count as libc::c_int % 2 as libc::c_int)
            as u8;
        diff_histo_stddev = 0 as libc::c_int;
        HighIndex = (BuffSize as libc::c_int - timing as libc::c_int * 4 as libc::c_int)
            as u8;
        if (*pdev).bin_rec_pos as libc::c_int > 0 as libc::c_int {
            prev_pos = ((*pdev).bin_rec_pos as libc::c_int - 1 as libc::c_int)
                as u8;
        } else {
            prev_pos = (TuningBinRecSize - 1 as libc::c_int) as u8;
        }
        if (*pdev)
            .multi_bins_rec[prev_pos
            as usize][timing as usize][4 as libc::c_int as usize] > 0 as libc::c_int
        {
            vl53lx_diff_histo_stddev(
                pdev,
                pdata,
                timing,
                HighIndex,
                prev_pos,
                &mut diff_histo_stddev,
            );
        }
        if diff_histo_stddev >= rmt {
            memset(
                ((*pdev).multi_bins_rec).as_mut_ptr() as *mut libc::c_void,
                0 as libc::c_int,
                ::std::mem::size_of::<[[[i32; 24]; 2]; 6]>() as libc::c_ulong,
            );
            (*pdev).bin_rec_pos = 0 as libc::c_int as u8;
            recom_been_reset = 1 as libc::c_int as u8;
            if timing as libc::c_int == 0 as libc::c_int {
                (*pdev).pos_before_next_recom = 6 as libc::c_int as u8;
            } else {
                (*pdev)
                    .pos_before_next_recom = (6 as libc::c_int + 1 as libc::c_int)
                    as u8;
            }
        } else {
            pos = (*pdev).bin_rec_pos;
            i = 0 as libc::c_int as u8;
            while (i as libc::c_int) < BuffSize as libc::c_int {
                (*pdev)
                    .multi_bins_rec[pos
                    as usize][timing
                    as usize][i as usize] = (*pdata).bin_data[i as usize];
                i = i.wrapping_add(1);
            }
        }
        if (*pdev).bin_rec_pos as libc::c_int == TuningBinRecSize - 1 as libc::c_int
            && timing as libc::c_int == 1 as libc::c_int
        {
            (*pdev).bin_rec_pos = 0 as libc::c_int as u8;
        } else if timing as libc::c_int == 1 as libc::c_int {
            let ref mut fresh13 = (*pdev).bin_rec_pos;
            *fresh13 = (*fresh13).wrapping_add(1);
        }
        if !(recom_been_reset as libc::c_int == 1 as libc::c_int
            && timing as libc::c_int == 0 as libc::c_int)
            && (*pdev).pos_before_next_recom as libc::c_int == 0 as libc::c_int
        {
            bin = 0 as libc::c_int as u16;
            while (bin as libc::c_int) < BuffSize as libc::c_int {
                (*pdata).bin_data[bin as usize] = 0 as libc::c_int;
                bin = bin.wrapping_add(1);
            }
            bin = 0 as libc::c_int as u16;
            while (bin as libc::c_int) < BuffSize as libc::c_int {
                i = 0 as libc::c_int as u8;
                while (i as libc::c_int) < TuningBinRecSize {
                    let ref mut fresh14 = (*pdata).bin_data[bin as usize];
                    *fresh14
                        += (*pdev)
                            .multi_bins_rec[i as usize][timing as usize][bin as usize];
                    i = i.wrapping_add(1);
                }
                bin = bin.wrapping_add(1);
            }
        }
    } else {
        let ref mut fresh15 = (*pdev).pos_before_next_recom;
        *fresh15 = (*fresh15).wrapping_sub(1);
        if (*pdev).pos_before_next_recom as libc::c_int == 255 as libc::c_int {
            (*pdev).pos_before_next_recom = 0 as libc::c_int as u8;
        }
    };
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_load_patch(mut Dev: VL53LX_DEV) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut patch_tuning: i32 = 0 as libc::c_int;
    let mut comms_buffer: [u8; 256] = [0; 256];
    let mut patch_power: u32 = 0;
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_WrByte(
            Dev,
            0x85 as libc::c_int as u16,
            0 as libc::c_int as u8,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        VL53LX_enable_powerforce(Dev);
    }
    VL53LX_get_tuning_parm(
        Dev,
        (0x8000 as libc::c_int + 140 as libc::c_int) as VL53LX_TuningParms,
        &mut patch_tuning,
    );
    match patch_tuning {
        0 => {
            patch_power = 0 as libc::c_int as u32;
        }
        1 => {
            patch_power = 0x10 as libc::c_int as u32;
        }
        2 => {
            patch_power = 0x20 as libc::c_int as u32;
        }
        3 => {
            patch_power = 0x40 as libc::c_int as u32;
        }
        _ => {
            patch_power = 0 as libc::c_int as u32;
        }
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        comms_buffer[0 as libc::c_int as usize] = 0x29 as libc::c_int as u8;
        comms_buffer[1 as libc::c_int as usize] = 0xc9 as libc::c_int as u8;
        comms_buffer[2 as libc::c_int as usize] = 0xe as libc::c_int as u8;
        comms_buffer[3 as libc::c_int as usize] = 0x40 as libc::c_int as u8;
        comms_buffer[4 as libc::c_int as usize] = 0x28 as libc::c_int as u8;
        comms_buffer[5 as libc::c_int as usize] = patch_power as u8;
        status = VL53LX_WriteMulti(
            Dev,
            0x476 as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
            6 as libc::c_int as u32,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        comms_buffer[0 as libc::c_int as usize] = 0x3 as libc::c_int as u8;
        comms_buffer[1 as libc::c_int as usize] = 0x6d as libc::c_int as u8;
        comms_buffer[2 as libc::c_int as usize] = 0x3 as libc::c_int as u8;
        comms_buffer[3 as libc::c_int as usize] = 0x6f as libc::c_int as u8;
        comms_buffer[4 as libc::c_int as usize] = 0x7 as libc::c_int as u8;
        comms_buffer[5 as libc::c_int as usize] = 0x29 as libc::c_int as u8;
        status = VL53LX_WriteMulti(
            Dev,
            0x496 as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
            6 as libc::c_int as u32,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        comms_buffer[0 as libc::c_int as usize] = 0 as libc::c_int as u8;
        comms_buffer[1 as libc::c_int as usize] = 0x7 as libc::c_int as u8;
        status = VL53LX_WriteMulti(
            Dev,
            0x472 as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
            2 as libc::c_int as u32,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        comms_buffer[0 as libc::c_int as usize] = 0 as libc::c_int as u8;
        comms_buffer[1 as libc::c_int as usize] = 0x7 as libc::c_int as u8;
        status = VL53LX_WriteMulti(
            Dev,
            0x474 as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
            2 as libc::c_int as u32,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_WrByte(
            Dev,
            0x470 as libc::c_int as u16,
            0x1 as libc::c_int as u8,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_WrByte(
            Dev,
            0x85 as libc::c_int as u16,
            0x1 as libc::c_int as u8,
        );
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_unload_patch(mut Dev: VL53LX_DEV) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_WrByte(
            Dev,
            0x85 as libc::c_int as u16,
            0 as libc::c_int as u8,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        VL53LX_disable_powerforce(Dev);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_WrByte(
            Dev,
            0x470 as libc::c_int as u16,
            0 as libc::c_int as u8,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_WrByte(
            Dev,
            0x85 as libc::c_int as u16,
            0x1 as libc::c_int as u8,
        );
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_get_version(
    mut Dev: VL53LX_DEV,
    mut pdata: *mut VL53LX_ll_version_t,
) -> VL53LX_Error {
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    VL53LX_init_version(Dev);
    memcpy(
        pdata as *mut libc::c_void,
        &mut (*pdev).version as *mut VL53LX_ll_version_t as *const libc::c_void,
        ::std::mem::size_of::<VL53LX_ll_version_t>() as libc::c_ulong,
    );
    return 0 as libc::c_int as VL53LX_Error;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_data_init(
    mut Dev: VL53LX_DEV,
    mut read_p2p_data: u8,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    let mut pres: *mut VL53LX_LLDriverResults_t = &mut (*Dev).Data.llresults;
    let mut pobjects: *mut VL53LX_zone_objects_t = 0 as *mut VL53LX_zone_objects_t;
    let mut i: u8 = 0 as libc::c_int as u8;
    VL53LX_init_ll_driver_state(Dev, 98 as libc::c_int as VL53LX_DeviceState);
    (*pres).range_results.max_results = 4 as libc::c_int as u8;
    (*pres).range_results.active_results = 0 as libc::c_int as u8;
    (*pres).zone_results.max_zones = 5 as libc::c_int as u8;
    (*pres).zone_results.active_zones = 0 as libc::c_int as u8;
    i = 0 as libc::c_int as u8;
    while (i as libc::c_int) < 5 as libc::c_int {
        pobjects = &mut *((*pres).zone_results.VL53LX_p_003)
            .as_mut_ptr()
            .offset(i as isize) as *mut VL53LX_zone_objects_t;
        (*pobjects).xmonitor.VL53LX_p_016 = 0 as libc::c_int as u32;
        (*pobjects).xmonitor.VL53LX_p_017 = 0 as libc::c_int as u32;
        (*pobjects).xmonitor.VL53LX_p_011 = 0 as libc::c_int as u16;
        (*pobjects).xmonitor.range_status = 0 as libc::c_int as VL53LX_DeviceError;
        i = i.wrapping_add(1);
    }
    (*pres).zone_hists.max_zones = 5 as libc::c_int as u8;
    (*pres).zone_hists.active_zones = 0 as libc::c_int as u8;
    (*pres).zone_cal.max_zones = 5 as libc::c_int as u8;
    (*pres).zone_cal.active_zones = 0 as libc::c_int as u8;
    i = 0 as libc::c_int as u8;
    while (i as libc::c_int) < 5 as libc::c_int {
        (*pres)
            .zone_cal
            .VL53LX_p_003[i as usize]
            .no_of_samples = 0 as libc::c_int as u32;
        (*pres)
            .zone_cal
            .VL53LX_p_003[i as usize]
            .effective_spads = 0 as libc::c_int as u32;
        (*pres)
            .zone_cal
            .VL53LX_p_003[i as usize]
            .peak_rate_mcps = 0 as libc::c_int as u32;
        (*pres).zone_cal.VL53LX_p_003[i as usize].median_range_mm = 0 as libc::c_int;
        (*pres).zone_cal.VL53LX_p_003[i as usize].range_mm_offset = 0 as libc::c_int;
        i = i.wrapping_add(1);
    }
    (*pdev).wait_method = 0 as libc::c_int as VL53LX_WaitMethod;
    (*pdev).preset_mode = 30 as libc::c_int as VL53LX_DevicePresetModes;
    (*pdev).zone_preset = 0 as libc::c_int as VL53LX_DeviceZonePreset;
    (*pdev).measurement_mode = 0 as libc::c_int as VL53LX_DeviceMeasurementModes;
    (*pdev).offset_calibration_mode = 1 as libc::c_int as VL53LX_OffsetCalibrationMode;
    (*pdev).offset_correction_mode = 1 as libc::c_int as VL53LX_OffsetCorrectionMode;
    (*pdev).dmax_mode = 1 as libc::c_int as VL53LX_DeviceDmaxMode;
    (*pdev).phasecal_config_timeout_us = 1000 as libc::c_int as u32;
    (*pdev).mm_config_timeout_us = 2000 as libc::c_int as u32;
    (*pdev).range_config_timeout_us = 13000 as libc::c_int as u32;
    (*pdev).inter_measurement_period_ms = 100 as libc::c_int as u32;
    (*pdev).dss_config__target_total_rate_mcps = 0xa00 as libc::c_int as u16;
    (*pdev).debug_mode = 0 as libc::c_int as u8;
    (*pdev).offset_results.max_results = 3 as libc::c_int as u8;
    (*pdev).offset_results.active_results = 0 as libc::c_int as u8;
    (*pdev).gain_cal.standard_ranging_gain_factor = 2011 as libc::c_int as u16;
    (*pdev).gain_cal.histogram_ranging_gain_factor = 1987 as libc::c_int as u16;
    VL53LX_init_version(Dev);
    memset(
        ((*pdev).multi_bins_rec).as_mut_ptr() as *mut libc::c_void,
        0 as libc::c_int,
        ::std::mem::size_of::<[[[i32; 24]; 2]; 6]>() as libc::c_ulong,
    );
    (*pdev).bin_rec_pos = 0 as libc::c_int as u8;
    (*pdev).pos_before_next_recom = 0 as libc::c_int as u8;
    if read_p2p_data as libc::c_int > 0 as libc::c_int
        && status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int
    {
        status = VL53LX_read_p2p_data(Dev);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_init_refspadchar_config_struct(&mut (*pdev).refspadchar);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_init_ssc_config_struct(&mut (*pdev).ssc_cfg);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_init_xtalk_config_struct(
            &mut (*pdev).customer,
            &mut (*pdev).xtalk_cfg,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_init_xtalk_extract_config_struct(&mut (*pdev).xtalk_extract_cfg);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_init_offset_cal_config_struct(&mut (*pdev).offsetcal_cfg);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_init_zone_cal_config_struct(&mut (*pdev).zonecal_cfg);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_init_hist_post_process_config_struct(
            (*pdev).xtalk_cfg.global_crosstalk_compensation_enable,
            &mut (*pdev).histpostprocess,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_init_hist_gen3_dmax_config_struct(&mut (*pdev).dmax_cfg);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_init_tuning_parm_storage_struct(&mut (*pdev).tuning_parms);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_set_preset_mode(
            Dev,
            (*pdev).preset_mode,
            (*pdev).dss_config__target_total_rate_mcps,
            (*pdev).phasecal_config_timeout_us,
            (*pdev).mm_config_timeout_us,
            (*pdev).range_config_timeout_us,
            (*pdev).inter_measurement_period_ms,
        );
    }
    VL53LX_init_histogram_bin_data_struct(
        0 as libc::c_int,
        24 as libc::c_int as u16,
        &mut (*pdev).hist_data,
    );
    VL53LX_init_histogram_bin_data_struct(
        0 as libc::c_int,
        24 as libc::c_int as u16,
        &mut (*pdev).hist_xtalk,
    );
    VL53LX_init_xtalk_bin_data_struct(
        0 as libc::c_int as u32,
        12 as libc::c_int as u16,
        &mut (*pdev).xtalk_shapes.xtalk_shape,
    );
    VL53LX_xtalk_cal_data_init(Dev);
    VL53LX_dynamic_xtalk_correction_data_init(Dev);
    VL53LX_low_power_auto_data_init(Dev);
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_read_p2p_data(mut Dev: VL53LX_DEV) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    let mut pHP: *mut VL53LX_hist_post_process_config_t = &mut (*pdev).histpostprocess;
    let mut pN: *mut VL53LX_customer_nvm_managed_t = &mut (*pdev).customer;
    let mut pCD: *mut VL53LX_additional_offset_cal_data_t = &mut (*pdev)
        .add_off_cal_data;
    let mut fmt_rrd: VL53LX_decoded_nvm_fmt_range_data_t = VL53LX_decoded_nvm_fmt_range_data_t {
        result__actual_effective_rtn_spads: 0,
        ref_spad_array__num_requested_ref_spads: 0,
        ref_spad_array__ref_location: 0,
        result__peak_signal_count_rate_rtn_mcps: 0,
        result__ambient_count_rate_rtn_mcps: 0,
        result__peak_signal_count_rate_ref_mcps: 0,
        result__ambient_count_rate_ref_mcps: 0,
        measured_distance_mm: 0,
        measured_distance_stdev_mm: 0,
    };
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_get_static_nvm_managed(Dev, &mut (*pdev).stat_nvm);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_get_customer_nvm_managed(Dev, &mut (*pdev).customer);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_get_nvm_copy_data(Dev, &mut (*pdev).nvm_copy_data);
        if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
            VL53LX_copy_rtn_good_spads_to_buffer(
                &mut (*pdev).nvm_copy_data,
                &mut *((*pdev).rtn_good_spads)
                    .as_mut_ptr()
                    .offset(0 as libc::c_int as isize),
            );
        }
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        (*pHP)
            .algo__crosstalk_compensation_plane_offset_kcps = (*pN)
            .algo__crosstalk_compensation_plane_offset_kcps as u32;
        (*pHP)
            .algo__crosstalk_compensation_x_plane_gradient_kcps = (*pN)
            .algo__crosstalk_compensation_x_plane_gradient_kcps;
        (*pHP)
            .algo__crosstalk_compensation_y_plane_gradient_kcps = (*pN)
            .algo__crosstalk_compensation_y_plane_gradient_kcps;
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_read_nvm_optical_centre(Dev, &mut (*pdev).optical_centre);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_read_nvm_cal_peak_rate_map(Dev, &mut (*pdev).cal_peak_rate_map);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_read_nvm_additional_offset_cal_data(
            Dev,
            &mut (*pdev).add_off_cal_data,
        );
        if (*pCD).result__mm_inner_peak_signal_count_rtn_mcps as libc::c_int
            == 0 as libc::c_int
            && (*pCD).result__mm_outer_peak_signal_count_rtn_mcps as libc::c_int
                == 0 as libc::c_int
        {
            (*pCD)
                .result__mm_inner_peak_signal_count_rtn_mcps = 0x80 as libc::c_int
                as u16;
            (*pCD)
                .result__mm_outer_peak_signal_count_rtn_mcps = 0x180 as libc::c_int
                as u16;
            VL53LX_calc_mm_effective_spads(
                (*pdev).nvm_copy_data.roi_config__mode_roi_centre_spad,
                (*pdev).nvm_copy_data.roi_config__mode_roi_xy_size,
                0xc7 as libc::c_int as u8,
                0xff as libc::c_int as u8,
                &mut *((*pdev).rtn_good_spads)
                    .as_mut_ptr()
                    .offset(0 as libc::c_int as isize),
                0x38 as libc::c_int as u16,
                &mut (*pCD).result__mm_inner_actual_effective_spads,
                &mut (*pCD).result__mm_outer_actual_effective_spads,
            );
        }
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_read_nvm_fmt_range_results_data(
            Dev,
            0x1ac as libc::c_int as u16,
            &mut fmt_rrd,
        );
        if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
            (*pdev)
                .fmt_dmax_cal
                .ref__actual_effective_spads = fmt_rrd
                .result__actual_effective_rtn_spads;
            (*pdev)
                .fmt_dmax_cal
                .ref__peak_signal_count_rate_mcps = fmt_rrd
                .result__peak_signal_count_rate_rtn_mcps;
            (*pdev).fmt_dmax_cal.ref__distance_mm = fmt_rrd.measured_distance_mm;
            if (*pdev).cal_peak_rate_map.cal_reflectance_pc as libc::c_int
                != 0 as libc::c_int
            {
                (*pdev)
                    .fmt_dmax_cal
                    .ref_reflectance_pc = (*pdev).cal_peak_rate_map.cal_reflectance_pc;
            } else {
                (*pdev)
                    .fmt_dmax_cal
                    .ref_reflectance_pc = 0x14 as libc::c_int as u16;
            }
            (*pdev)
                .fmt_dmax_cal
                .coverglass_transmission = 0x100 as libc::c_int as u16;
        }
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_RdWord(
            Dev,
            0xde as libc::c_int as u16,
            &mut (*pdev).dbg_results.result__osc_calibrate_val,
        );
    }
    if ((*pdev).stat_nvm.osc_measured__fast_osc__frequency as libc::c_int)
        < 0x1000 as libc::c_int
    {
        (*pdev)
            .stat_nvm
            .osc_measured__fast_osc__frequency = 0xbccc as libc::c_int as u16;
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_get_mode_mitigation_roi(Dev, &mut (*pdev).mm_roi);
    }
    if (*pdev).optical_centre.x_centre as libc::c_int == 0 as libc::c_int
        && (*pdev).optical_centre.y_centre as libc::c_int == 0 as libc::c_int
    {
        (*pdev)
            .optical_centre
            .x_centre = (((*pdev).mm_roi.x_centre as libc::c_int) << 4 as libc::c_int)
            as u8;
        (*pdev)
            .optical_centre
            .y_centre = (((*pdev).mm_roi.y_centre as libc::c_int) << 4 as libc::c_int)
            as u8;
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_set_part_to_part_data(
    mut Dev: VL53LX_DEV,
    mut pcal_data: *mut VL53LX_calibration_data_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    let mut pC: *mut VL53LX_xtalk_config_t = &mut (*pdev).xtalk_cfg;
    let mut pHP: *mut VL53LX_hist_post_process_config_t = &mut (*pdev).histpostprocess;
    let mut pN: *mut VL53LX_customer_nvm_managed_t = &mut (*pdev).customer;
    let mut tempu32: u32 = 0;
    if (*pcal_data).struct_version != 0xecab0102 as libc::c_uint {
        status = -(4 as libc::c_int) as VL53LX_Error;
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        memcpy(
            &mut (*pdev).customer as *mut VL53LX_customer_nvm_managed_t
                as *mut libc::c_void,
            &mut (*pcal_data).customer as *mut VL53LX_customer_nvm_managed_t
                as *const libc::c_void,
            ::std::mem::size_of::<VL53LX_customer_nvm_managed_t>() as libc::c_ulong,
        );
        memcpy(
            &mut (*pdev).add_off_cal_data as *mut VL53LX_additional_offset_cal_data_t
                as *mut libc::c_void,
            &mut (*pcal_data).add_off_cal_data
                as *mut VL53LX_additional_offset_cal_data_t as *const libc::c_void,
            ::std::mem::size_of::<VL53LX_additional_offset_cal_data_t>() as libc::c_ulong,
        );
        memcpy(
            &mut (*pdev).fmt_dmax_cal as *mut VL53LX_dmax_calibration_data_t
                as *mut libc::c_void,
            &mut (*pcal_data).fmt_dmax_cal as *mut VL53LX_dmax_calibration_data_t
                as *const libc::c_void,
            ::std::mem::size_of::<VL53LX_dmax_calibration_data_t>() as libc::c_ulong,
        );
        memcpy(
            &mut (*pdev).cust_dmax_cal as *mut VL53LX_dmax_calibration_data_t
                as *mut libc::c_void,
            &mut (*pcal_data).cust_dmax_cal as *mut VL53LX_dmax_calibration_data_t
                as *const libc::c_void,
            ::std::mem::size_of::<VL53LX_dmax_calibration_data_t>() as libc::c_ulong,
        );
        memcpy(
            &mut (*pdev).xtalk_shapes as *mut VL53LX_xtalk_histogram_data_t
                as *mut libc::c_void,
            &mut (*pcal_data).xtalkhisto as *mut VL53LX_xtalk_histogram_data_t
                as *const libc::c_void,
            ::std::mem::size_of::<VL53LX_xtalk_histogram_data_t>() as libc::c_ulong,
        );
        memcpy(
            &mut (*pdev).gain_cal as *mut VL53LX_gain_calibration_data_t
                as *mut libc::c_void,
            &mut (*pcal_data).gain_cal as *mut VL53LX_gain_calibration_data_t
                as *const libc::c_void,
            ::std::mem::size_of::<VL53LX_gain_calibration_data_t>() as libc::c_ulong,
        );
        memcpy(
            &mut (*pdev).cal_peak_rate_map as *mut VL53LX_cal_peak_rate_map_t
                as *mut libc::c_void,
            &mut (*pcal_data).cal_peak_rate_map as *mut VL53LX_cal_peak_rate_map_t
                as *const libc::c_void,
            ::std::mem::size_of::<VL53LX_cal_peak_rate_map_t>() as libc::c_ulong,
        );
        memcpy(
            &mut (*pdev).per_vcsel_cal_data
                as *mut VL53LX_per_vcsel_period_offset_cal_data_t as *mut libc::c_void,
            &mut (*pcal_data).per_vcsel_cal_data
                as *mut VL53LX_per_vcsel_period_offset_cal_data_t as *const libc::c_void,
            ::std::mem::size_of::<VL53LX_per_vcsel_period_offset_cal_data_t>()
                as libc::c_ulong,
        );
        (*pC)
            .algo__crosstalk_compensation_plane_offset_kcps = (*pN)
            .algo__crosstalk_compensation_plane_offset_kcps as u32;
        (*pC)
            .algo__crosstalk_compensation_x_plane_gradient_kcps = (*pN)
            .algo__crosstalk_compensation_x_plane_gradient_kcps;
        (*pC)
            .algo__crosstalk_compensation_y_plane_gradient_kcps = (*pN)
            .algo__crosstalk_compensation_y_plane_gradient_kcps;
        (*pHP)
            .algo__crosstalk_compensation_plane_offset_kcps = VL53LX_calc_crosstalk_plane_offset_with_margin(
            (*pC).algo__crosstalk_compensation_plane_offset_kcps,
            (*pC).histogram_mode_crosstalk_margin_kcps,
        );
        (*pHP)
            .algo__crosstalk_compensation_x_plane_gradient_kcps = (*pC)
            .algo__crosstalk_compensation_x_plane_gradient_kcps;
        (*pHP)
            .algo__crosstalk_compensation_y_plane_gradient_kcps = (*pC)
            .algo__crosstalk_compensation_y_plane_gradient_kcps;
        if (*pC).global_crosstalk_compensation_enable as libc::c_int == 0 as libc::c_int
        {
            (*pN)
                .algo__crosstalk_compensation_plane_offset_kcps = 0 as libc::c_int
                as u16;
            (*pN)
                .algo__crosstalk_compensation_x_plane_gradient_kcps = 0 as libc::c_int
                as i16;
            (*pN)
                .algo__crosstalk_compensation_y_plane_gradient_kcps = 0 as libc::c_int
                as i16;
        } else {
            tempu32 = VL53LX_calc_crosstalk_plane_offset_with_margin(
                (*pC).algo__crosstalk_compensation_plane_offset_kcps,
                (*pC).lite_mode_crosstalk_margin_kcps,
            );
            if tempu32 > 0xffff as libc::c_int as libc::c_uint {
                tempu32 = 0xffff as libc::c_int as u32;
            }
            (*pN).algo__crosstalk_compensation_plane_offset_kcps = tempu32 as u16;
        }
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_get_part_to_part_data(
    mut Dev: VL53LX_DEV,
    mut pcal_data: *mut VL53LX_calibration_data_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    let mut pC: *mut VL53LX_xtalk_config_t = &mut (*pdev).xtalk_cfg;
    let mut pCN: *mut VL53LX_customer_nvm_managed_t = &mut (*pcal_data).customer;
    (*pcal_data).struct_version = 0xecab0102 as libc::c_uint;
    memcpy(
        &mut (*pcal_data).customer as *mut VL53LX_customer_nvm_managed_t
            as *mut libc::c_void,
        &mut (*pdev).customer as *mut VL53LX_customer_nvm_managed_t
            as *const libc::c_void,
        ::std::mem::size_of::<VL53LX_customer_nvm_managed_t>() as libc::c_ulong,
    );
    if (*pC).algo__crosstalk_compensation_plane_offset_kcps
        > 0xffff as libc::c_int as libc::c_uint
    {
        (*pCN)
            .algo__crosstalk_compensation_plane_offset_kcps = 0xffff as libc::c_int
            as u16;
    } else {
        (*pCN)
            .algo__crosstalk_compensation_plane_offset_kcps = (*pC)
            .algo__crosstalk_compensation_plane_offset_kcps as u16;
    }
    (*pCN)
        .algo__crosstalk_compensation_x_plane_gradient_kcps = (*pC)
        .algo__crosstalk_compensation_x_plane_gradient_kcps;
    (*pCN)
        .algo__crosstalk_compensation_y_plane_gradient_kcps = (*pC)
        .algo__crosstalk_compensation_y_plane_gradient_kcps;
    memcpy(
        &mut (*pcal_data).fmt_dmax_cal as *mut VL53LX_dmax_calibration_data_t
            as *mut libc::c_void,
        &mut (*pdev).fmt_dmax_cal as *mut VL53LX_dmax_calibration_data_t
            as *const libc::c_void,
        ::std::mem::size_of::<VL53LX_dmax_calibration_data_t>() as libc::c_ulong,
    );
    memcpy(
        &mut (*pcal_data).cust_dmax_cal as *mut VL53LX_dmax_calibration_data_t
            as *mut libc::c_void,
        &mut (*pdev).cust_dmax_cal as *mut VL53LX_dmax_calibration_data_t
            as *const libc::c_void,
        ::std::mem::size_of::<VL53LX_dmax_calibration_data_t>() as libc::c_ulong,
    );
    memcpy(
        &mut (*pcal_data).add_off_cal_data as *mut VL53LX_additional_offset_cal_data_t
            as *mut libc::c_void,
        &mut (*pdev).add_off_cal_data as *mut VL53LX_additional_offset_cal_data_t
            as *const libc::c_void,
        ::std::mem::size_of::<VL53LX_additional_offset_cal_data_t>() as libc::c_ulong,
    );
    memcpy(
        &mut (*pcal_data).optical_centre as *mut VL53LX_optical_centre_t
            as *mut libc::c_void,
        &mut (*pdev).optical_centre as *mut VL53LX_optical_centre_t
            as *const libc::c_void,
        ::std::mem::size_of::<VL53LX_optical_centre_t>() as libc::c_ulong,
    );
    memcpy(
        &mut (*pcal_data).xtalkhisto as *mut VL53LX_xtalk_histogram_data_t
            as *mut libc::c_void,
        &mut (*pdev).xtalk_shapes as *mut VL53LX_xtalk_histogram_data_t
            as *const libc::c_void,
        ::std::mem::size_of::<VL53LX_xtalk_histogram_data_t>() as libc::c_ulong,
    );
    memcpy(
        &mut (*pcal_data).gain_cal as *mut VL53LX_gain_calibration_data_t
            as *mut libc::c_void,
        &mut (*pdev).gain_cal as *mut VL53LX_gain_calibration_data_t
            as *const libc::c_void,
        ::std::mem::size_of::<VL53LX_gain_calibration_data_t>() as libc::c_ulong,
    );
    memcpy(
        &mut (*pcal_data).cal_peak_rate_map as *mut VL53LX_cal_peak_rate_map_t
            as *mut libc::c_void,
        &mut (*pdev).cal_peak_rate_map as *mut VL53LX_cal_peak_rate_map_t
            as *const libc::c_void,
        ::std::mem::size_of::<VL53LX_cal_peak_rate_map_t>() as libc::c_ulong,
    );
    memcpy(
        &mut (*pcal_data).per_vcsel_cal_data
            as *mut VL53LX_per_vcsel_period_offset_cal_data_t as *mut libc::c_void,
        &mut (*pdev).per_vcsel_cal_data as *mut VL53LX_per_vcsel_period_offset_cal_data_t
            as *const libc::c_void,
        ::std::mem::size_of::<VL53LX_per_vcsel_period_offset_cal_data_t>()
            as libc::c_ulong,
    );
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_set_inter_measurement_period_ms(
    mut Dev: VL53LX_DEV,
    mut inter_measurement_period_ms: u32,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    if (*pdev).dbg_results.result__osc_calibrate_val as libc::c_int == 0 as libc::c_int {
        status = -(15 as libc::c_int) as VL53LX_Error;
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        (*pdev).inter_measurement_period_ms = inter_measurement_period_ms;
        (*pdev)
            .tim_cfg
            .system__intermeasurement_period = inter_measurement_period_ms
            .wrapping_mul((*pdev).dbg_results.result__osc_calibrate_val as u32);
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_get_inter_measurement_period_ms(
    mut Dev: VL53LX_DEV,
    mut pinter_measurement_period_ms: *mut u32,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    if (*pdev).dbg_results.result__osc_calibrate_val as libc::c_int == 0 as libc::c_int {
        status = -(15 as libc::c_int) as VL53LX_Error;
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        *pinter_measurement_period_ms = ((*pdev).tim_cfg.system__intermeasurement_period)
            .wrapping_div((*pdev).dbg_results.result__osc_calibrate_val as u32);
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_set_timeouts_us(
    mut Dev: VL53LX_DEV,
    mut phasecal_config_timeout_us: u32,
    mut mm_config_timeout_us: u32,
    mut range_config_timeout_us: u32,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    if (*pdev).stat_nvm.osc_measured__fast_osc__frequency as libc::c_int
        == 0 as libc::c_int
    {
        status = -(15 as libc::c_int) as VL53LX_Error;
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        (*pdev).phasecal_config_timeout_us = phasecal_config_timeout_us;
        (*pdev).mm_config_timeout_us = mm_config_timeout_us;
        (*pdev).range_config_timeout_us = range_config_timeout_us;
        status = VL53LX_calc_timeout_register_values(
            phasecal_config_timeout_us,
            mm_config_timeout_us,
            range_config_timeout_us,
            (*pdev).stat_nvm.osc_measured__fast_osc__frequency,
            &mut (*pdev).gen_cfg,
            &mut (*pdev).tim_cfg,
        );
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_get_timeouts_us(
    mut Dev: VL53LX_DEV,
    mut pphasecal_config_timeout_us: *mut u32,
    mut pmm_config_timeout_us: *mut u32,
    mut prange_config_timeout_us: *mut u32,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    let mut macro_period_us: u32 = 0 as libc::c_int as u32;
    let mut timeout_encoded: u16 = 0 as libc::c_int as u16;
    if (*pdev).stat_nvm.osc_measured__fast_osc__frequency as libc::c_int
        == 0 as libc::c_int
    {
        status = -(15 as libc::c_int) as VL53LX_Error;
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        macro_period_us = VL53LX_calc_macro_period_us(
            (*pdev).stat_nvm.osc_measured__fast_osc__frequency,
            (*pdev).tim_cfg.range_config__vcsel_period_a,
        );
        *pphasecal_config_timeout_us = VL53LX_calc_timeout_us(
            (*pdev).gen_cfg.phasecal_config__timeout_macrop as u32,
            macro_period_us,
        );
        timeout_encoded = (*pdev).tim_cfg.mm_config__timeout_macrop_a_hi as u16;
        timeout_encoded = (((timeout_encoded as libc::c_int) << 8 as libc::c_int)
            + (*pdev).tim_cfg.mm_config__timeout_macrop_a_lo as u16 as libc::c_int)
            as u16;
        *pmm_config_timeout_us = VL53LX_calc_decoded_timeout_us(
            timeout_encoded,
            macro_period_us,
        );
        timeout_encoded = (*pdev).tim_cfg.range_config__timeout_macrop_a_hi as u16;
        timeout_encoded = (((timeout_encoded as libc::c_int) << 8 as libc::c_int)
            + (*pdev).tim_cfg.range_config__timeout_macrop_a_lo as u16
                as libc::c_int) as u16;
        *prange_config_timeout_us = VL53LX_calc_decoded_timeout_us(
            timeout_encoded,
            macro_period_us,
        );
        (*pdev).phasecal_config_timeout_us = *pphasecal_config_timeout_us;
        (*pdev).mm_config_timeout_us = *pmm_config_timeout_us;
        (*pdev).range_config_timeout_us = *prange_config_timeout_us;
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_set_user_zone(
    mut Dev: VL53LX_DEV,
    mut puser_zone: *mut VL53LX_user_zone_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    VL53LX_encode_row_col(
        (*puser_zone).y_centre,
        (*puser_zone).x_centre,
        &mut (*pdev).dyn_cfg.roi_config__user_roi_centre_spad,
    );
    VL53LX_encode_zone_size(
        (*puser_zone).width,
        (*puser_zone).height,
        &mut (*pdev).dyn_cfg.roi_config__user_roi_requested_global_xy_size,
    );
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_get_user_zone(
    mut Dev: VL53LX_DEV,
    mut puser_zone: *mut VL53LX_user_zone_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    VL53LX_decode_row_col(
        (*pdev).dyn_cfg.roi_config__user_roi_centre_spad,
        &mut (*puser_zone).y_centre,
        &mut (*puser_zone).x_centre,
    );
    VL53LX_decode_zone_size(
        (*pdev).dyn_cfg.roi_config__user_roi_requested_global_xy_size,
        &mut (*puser_zone).width,
        &mut (*puser_zone).height,
    );
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_get_mode_mitigation_roi(
    mut Dev: VL53LX_DEV,
    mut pmm_roi: *mut VL53LX_user_zone_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    let mut x: u8 = 0 as libc::c_int as u8;
    let mut y: u8 = 0 as libc::c_int as u8;
    let mut xy_size: u8 = 0 as libc::c_int as u8;
    VL53LX_decode_row_col(
        (*pdev).nvm_copy_data.roi_config__mode_roi_centre_spad,
        &mut y,
        &mut x,
    );
    (*pmm_roi).x_centre = x;
    (*pmm_roi).y_centre = y;
    xy_size = (*pdev).nvm_copy_data.roi_config__mode_roi_xy_size;
    (*pmm_roi).height = (xy_size as libc::c_int >> 4 as libc::c_int) as u8;
    (*pmm_roi).width = (xy_size as libc::c_int & 0xf as libc::c_int) as u8;
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_init_zone_config_histogram_bins(
    mut pdata: *mut VL53LX_zone_config_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut i: u8 = 0;
    i = 0 as libc::c_int as u8;
    while (i as libc::c_int) < (*pdata).max_zones as libc::c_int {
        (*pdata)
            .bin_config[i
            as usize] = 1 as libc::c_int as VL53LX_ZoneConfig_BinConfig_select;
        i = i.wrapping_add(1);
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_set_zone_config(
    mut Dev: VL53LX_DEV,
    mut pzone_cfg: *mut VL53LX_zone_config_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    memcpy(
        &mut (*pdev).zone_cfg.user_zones as *mut [VL53LX_user_zone_t; 5]
            as *mut libc::c_void,
        &mut (*pzone_cfg).user_zones as *mut [VL53LX_user_zone_t; 5]
            as *const libc::c_void,
        ::std::mem::size_of::<[VL53LX_user_zone_t; 5]>() as libc::c_ulong,
    );
    (*pdev).zone_cfg.max_zones = (*pzone_cfg).max_zones;
    (*pdev).zone_cfg.active_zones = (*pzone_cfg).active_zones;
    status = VL53LX_init_zone_config_histogram_bins(&mut (*pdev).zone_cfg);
    if (*pzone_cfg).active_zones as libc::c_int == 0 as libc::c_int {
        (*pdev).gen_cfg.global_config__stream_divider = 0 as libc::c_int as u8;
    } else if ((*pzone_cfg).active_zones as libc::c_int) < 5 as libc::c_int {
        (*pdev)
            .gen_cfg
            .global_config__stream_divider = ((*pzone_cfg).active_zones as libc::c_int
            + 1 as libc::c_int) as u8;
    } else {
        (*pdev)
            .gen_cfg
            .global_config__stream_divider = (5 as libc::c_int + 1 as libc::c_int)
            as u8;
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_get_zone_config(
    mut Dev: VL53LX_DEV,
    mut pzone_cfg: *mut VL53LX_zone_config_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    memcpy(
        pzone_cfg as *mut libc::c_void,
        &mut (*pdev).zone_cfg as *mut VL53LX_zone_config_t as *const libc::c_void,
        ::std::mem::size_of::<VL53LX_zone_config_t>() as libc::c_ulong,
    );
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_get_preset_mode_timing_cfg(
    mut Dev: VL53LX_DEV,
    mut device_preset_mode: VL53LX_DevicePresetModes,
    mut pdss_config__target_total_rate_mcps: *mut u16,
    mut pphasecal_config_timeout_us: *mut u32,
    mut pmm_config_timeout_us: *mut u32,
    mut prange_config_timeout_us: *mut u32,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    match device_preset_mode as libc::c_int {
        27 => {
            *pdss_config__target_total_rate_mcps = (*pdev)
                .tuning_parms
                .tp_dss_target_histo_mcps;
            *pphasecal_config_timeout_us = (*pdev)
                .tuning_parms
                .tp_phasecal_timeout_hist_long_us;
            *pmm_config_timeout_us = (*pdev).tuning_parms.tp_mm_timeout_histo_us;
            *prange_config_timeout_us = (*pdev).tuning_parms.tp_range_timeout_histo_us;
        }
        30 => {
            *pdss_config__target_total_rate_mcps = (*pdev)
                .tuning_parms
                .tp_dss_target_histo_mcps;
            *pphasecal_config_timeout_us = (*pdev)
                .tuning_parms
                .tp_phasecal_timeout_hist_med_us;
            *pmm_config_timeout_us = (*pdev).tuning_parms.tp_mm_timeout_histo_us;
            *prange_config_timeout_us = (*pdev).tuning_parms.tp_range_timeout_histo_us;
        }
        33 => {
            *pdss_config__target_total_rate_mcps = (*pdev)
                .tuning_parms
                .tp_dss_target_histo_mcps;
            *pphasecal_config_timeout_us = (*pdev)
                .tuning_parms
                .tp_phasecal_timeout_hist_short_us;
            *pmm_config_timeout_us = (*pdev).tuning_parms.tp_mm_timeout_histo_us;
            *prange_config_timeout_us = (*pdev).tuning_parms.tp_range_timeout_histo_us;
        }
        _ => {
            status = -(4 as libc::c_int) as VL53LX_Error;
        }
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_set_preset_mode(
    mut Dev: VL53LX_DEV,
    mut device_preset_mode: VL53LX_DevicePresetModes,
    mut dss_config__target_total_rate_mcps: u16,
    mut phasecal_config_timeout_us: u32,
    mut mm_config_timeout_us: u32,
    mut range_config_timeout_us: u32,
    mut inter_measurement_period_ms: u32,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    let mut pres: *mut VL53LX_LLDriverResults_t = &mut (*Dev).Data.llresults;
    let mut phistpostprocess: *mut VL53LX_hist_post_process_config_t = &mut (*pdev)
        .histpostprocess;
    let mut pstatic: *mut VL53LX_static_config_t = &mut (*pdev).stat_cfg;
    let mut phistogram: *mut VL53LX_histogram_config_t = &mut (*pdev).hist_cfg;
    let mut pgeneral: *mut VL53LX_general_config_t = &mut (*pdev).gen_cfg;
    let mut ptiming: *mut VL53LX_timing_config_t = &mut (*pdev).tim_cfg;
    let mut pdynamic: *mut VL53LX_dynamic_config_t = &mut (*pdev).dyn_cfg;
    let mut psystem: *mut VL53LX_system_control_t = &mut (*pdev).sys_ctrl;
    let mut pzone_cfg: *mut VL53LX_zone_config_t = &mut (*pdev).zone_cfg;
    let mut ptuning_parms: *mut VL53LX_tuning_parm_storage_t = &mut (*pdev).tuning_parms;
    (*pdev).preset_mode = device_preset_mode;
    (*pdev).mm_config_timeout_us = mm_config_timeout_us;
    (*pdev).range_config_timeout_us = range_config_timeout_us;
    (*pdev).inter_measurement_period_ms = inter_measurement_period_ms;
    VL53LX_init_ll_driver_state(Dev, 3 as libc::c_int as VL53LX_DeviceState);
    match device_preset_mode as libc::c_int {
        27 => {
            status = VL53LX_preset_mode_histogram_long_range(
                phistpostprocess,
                pstatic,
                phistogram,
                pgeneral,
                ptiming,
                pdynamic,
                psystem,
                ptuning_parms,
                pzone_cfg,
            );
        }
        30 => {
            status = VL53LX_preset_mode_histogram_medium_range(
                phistpostprocess,
                pstatic,
                phistogram,
                pgeneral,
                ptiming,
                pdynamic,
                psystem,
                ptuning_parms,
                pzone_cfg,
            );
        }
        33 => {
            status = VL53LX_preset_mode_histogram_short_range(
                phistpostprocess,
                pstatic,
                phistogram,
                pgeneral,
                ptiming,
                pdynamic,
                psystem,
                ptuning_parms,
                pzone_cfg,
            );
        }
        _ => {
            status = -(4 as libc::c_int) as VL53LX_Error;
        }
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        (*pstatic)
            .dss_config__target_total_rate_mcps = dss_config__target_total_rate_mcps;
        (*pdev).dss_config__target_total_rate_mcps = dss_config__target_total_rate_mcps;
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_set_timeouts_us(
            Dev,
            phasecal_config_timeout_us,
            mm_config_timeout_us,
            range_config_timeout_us,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_set_inter_measurement_period_ms(
            Dev,
            inter_measurement_period_ms,
        );
    }
    V53L1_init_zone_results_structure(
        ((*pdev).zone_cfg.active_zones as libc::c_int + 1 as libc::c_int) as u8,
        &mut (*pres).zone_results,
    );
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_enable_xtalk_compensation(
    mut Dev: VL53LX_DEV,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut tempu32: u32 = 0;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    let mut pC: *mut VL53LX_xtalk_config_t = &mut (*pdev).xtalk_cfg;
    let mut pHP: *mut VL53LX_hist_post_process_config_t = &mut (*pdev).histpostprocess;
    let mut pN: *mut VL53LX_customer_nvm_managed_t = &mut (*pdev).customer;
    tempu32 = VL53LX_calc_crosstalk_plane_offset_with_margin(
        (*pC).algo__crosstalk_compensation_plane_offset_kcps,
        (*pC).lite_mode_crosstalk_margin_kcps,
    );
    if tempu32 > 0xffff as libc::c_int as libc::c_uint {
        tempu32 = 0xffff as libc::c_int as u32;
    }
    (*pN).algo__crosstalk_compensation_plane_offset_kcps = tempu32 as u16;
    (*pN)
        .algo__crosstalk_compensation_x_plane_gradient_kcps = (*pC)
        .algo__crosstalk_compensation_x_plane_gradient_kcps;
    (*pN)
        .algo__crosstalk_compensation_y_plane_gradient_kcps = (*pC)
        .algo__crosstalk_compensation_y_plane_gradient_kcps;
    (*pHP)
        .algo__crosstalk_compensation_plane_offset_kcps = VL53LX_calc_crosstalk_plane_offset_with_margin(
        (*pC).algo__crosstalk_compensation_plane_offset_kcps,
        (*pC).histogram_mode_crosstalk_margin_kcps,
    );
    (*pHP)
        .algo__crosstalk_compensation_x_plane_gradient_kcps = (*pC)
        .algo__crosstalk_compensation_x_plane_gradient_kcps;
    (*pHP)
        .algo__crosstalk_compensation_y_plane_gradient_kcps = (*pC)
        .algo__crosstalk_compensation_y_plane_gradient_kcps;
    (*pC).global_crosstalk_compensation_enable = 0x1 as libc::c_int as u8;
    (*pHP)
        .algo__crosstalk_compensation_enable = (*pC)
        .global_crosstalk_compensation_enable;
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        (*pC)
            .crosstalk_range_ignore_threshold_rate_mcps = VL53LX_calc_range_ignore_threshold(
            (*pC).algo__crosstalk_compensation_plane_offset_kcps,
            (*pC).algo__crosstalk_compensation_x_plane_gradient_kcps,
            (*pC).algo__crosstalk_compensation_y_plane_gradient_kcps,
            (*pC).crosstalk_range_ignore_threshold_mult,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_set_customer_nvm_managed(Dev, &mut (*pdev).customer);
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_get_xtalk_compensation_enable(
    mut Dev: VL53LX_DEV,
    mut pcrosstalk_compensation_enable: *mut u8,
) {
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    *pcrosstalk_compensation_enable = (*pdev)
        .xtalk_cfg
        .global_crosstalk_compensation_enable;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_disable_xtalk_compensation(
    mut Dev: VL53LX_DEV,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    let mut pHP: *mut VL53LX_hist_post_process_config_t = &mut (*pdev).histpostprocess;
    let mut pN: *mut VL53LX_customer_nvm_managed_t = &mut (*pdev).customer;
    (*pN).algo__crosstalk_compensation_plane_offset_kcps = 0 as libc::c_int as u16;
    (*pN)
        .algo__crosstalk_compensation_x_plane_gradient_kcps = 0 as libc::c_int
        as i16;
    (*pN)
        .algo__crosstalk_compensation_y_plane_gradient_kcps = 0 as libc::c_int
        as i16;
    (*pdev).xtalk_cfg.global_crosstalk_compensation_enable = 0 as libc::c_int as u8;
    (*pHP)
        .algo__crosstalk_compensation_enable = (*pdev)
        .xtalk_cfg
        .global_crosstalk_compensation_enable;
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        (*pdev)
            .xtalk_cfg
            .crosstalk_range_ignore_threshold_rate_mcps = 0 as libc::c_int as u16;
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_set_customer_nvm_managed(Dev, &mut (*pdev).customer);
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_init_and_start_range(
    mut Dev: VL53LX_DEV,
    mut measurement_mode: u8,
    mut device_config_level: VL53LX_DeviceConfigLevel,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    let mut pres: *mut VL53LX_LLDriverResults_t = &mut (*Dev).Data.llresults;
    let mut buffer: [u8; 256] = [0; 256];
    let mut pstatic_nvm: *mut VL53LX_static_nvm_managed_t = &mut (*pdev).stat_nvm;
    let mut pcustomer_nvm: *mut VL53LX_customer_nvm_managed_t = &mut (*pdev).customer;
    let mut pstatic: *mut VL53LX_static_config_t = &mut (*pdev).stat_cfg;
    let mut pgeneral: *mut VL53LX_general_config_t = &mut (*pdev).gen_cfg;
    let mut ptiming: *mut VL53LX_timing_config_t = &mut (*pdev).tim_cfg;
    let mut pdynamic: *mut VL53LX_dynamic_config_t = &mut (*pdev).dyn_cfg;
    let mut psystem: *mut VL53LX_system_control_t = &mut (*pdev).sys_ctrl;
    let mut pstate: *mut VL53LX_ll_driver_state_t = &mut (*pdev).ll_state;
    let mut pN: *mut VL53LX_customer_nvm_managed_t = &mut (*pdev).customer;
    let mut pbuffer: *mut u8 = &mut *buffer
        .as_mut_ptr()
        .offset(0 as libc::c_int as isize) as *mut u8;
    let mut i: u16 = 0 as libc::c_int as u16;
    let mut i2c_index: u16 = 0 as libc::c_int as u16;
    let mut i2c_buffer_offset_bytes: u16 = 0 as libc::c_int as u16;
    let mut i2c_buffer_size_bytes: u16 = 0 as libc::c_int as u16;
    (*pdev).measurement_mode = measurement_mode;
    (*psystem)
        .system__mode_start = ((*psystem).system__mode_start as libc::c_int
        & 0xf as libc::c_int | measurement_mode as libc::c_int) as u8;
    status = VL53LX_set_user_zone(
        Dev,
        &mut *((*pdev).zone_cfg.user_zones)
            .as_mut_ptr()
            .offset((*pdev).ll_state.cfg_zone_id as isize),
    );
    if (*pdev).zone_cfg.active_zones as libc::c_int > 0 as libc::c_int {
        status = VL53LX_set_zone_dss_config(
            Dev,
            &mut *((*pres).zone_dyn_cfgs.VL53LX_p_003)
                .as_mut_ptr()
                .offset((*pdev).ll_state.cfg_zone_id as isize),
        );
    }
    if (*pdev).sys_ctrl.system__mode_start as libc::c_int & 0x2 as libc::c_int
        == 0 as libc::c_int
        && (*pdev).xtalk_cfg.global_crosstalk_compensation_enable as libc::c_int
            == 0x1 as libc::c_int
    {
        (*pdev)
            .stat_cfg
            .algo__range_ignore_threshold_mcps = (*pdev)
            .xtalk_cfg
            .crosstalk_range_ignore_threshold_rate_mcps;
    }
    if (*pdev).low_power_auto_data.low_power_auto_range_count as libc::c_int
        == 0xff as libc::c_int
    {
        (*pdev)
            .low_power_auto_data
            .low_power_auto_range_count = 0 as libc::c_int as u8;
    }
    if (*pdev).low_power_auto_data.is_low_power_auto_mode as libc::c_int
        == 1 as libc::c_int
        && (*pdev).low_power_auto_data.low_power_auto_range_count as libc::c_int
            == 0 as libc::c_int
    {
        (*pdev)
            .low_power_auto_data
            .saved_interrupt_config = (*pdev).gen_cfg.system__interrupt_config_gpio;
        (*pdev)
            .gen_cfg
            .system__interrupt_config_gpio = ((1 as libc::c_int) << 5 as libc::c_int)
            as u8;
        if (*pdev).dyn_cfg.system__sequence_config as libc::c_int
            & (0x20 as libc::c_int | 0x40 as libc::c_int) == 0 as libc::c_int
        {
            (*pN)
                .algo__part_to_part_range_offset_mm = (((*pN).mm_config__outer_offset_mm
                as libc::c_int) << 2 as libc::c_int) as i16;
        } else {
            (*pN).algo__part_to_part_range_offset_mm = 0 as libc::c_int as i16;
        }
        if (device_config_level as libc::c_int)
            < 5 as libc::c_int as VL53LX_DeviceConfigLevel as libc::c_int
        {
            device_config_level = 5 as libc::c_int as VL53LX_DeviceConfigLevel;
        }
    }
    if (*pdev).low_power_auto_data.is_low_power_auto_mode as libc::c_int
        == 1 as libc::c_int
        && (*pdev).low_power_auto_data.low_power_auto_range_count as libc::c_int
            == 1 as libc::c_int
    {
        (*pdev)
            .gen_cfg
            .system__interrupt_config_gpio = (*pdev)
            .low_power_auto_data
            .saved_interrupt_config;
        device_config_level = 6 as libc::c_int as VL53LX_DeviceConfigLevel;
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_save_cfg_data(Dev);
    }
    match device_config_level as libc::c_int {
        6 => {
            i2c_index = 0x1 as libc::c_int as u16;
        }
        5 => {
            i2c_index = 0xd as libc::c_int as u16;
        }
        4 => {
            i2c_index = 0x24 as libc::c_int as u16;
        }
        3 => {
            i2c_index = 0x44 as libc::c_int as u16;
        }
        2 => {
            i2c_index = 0x5a as libc::c_int as u16;
        }
        1 => {
            i2c_index = 0x71 as libc::c_int as u16;
        }
        _ => {
            i2c_index = 0x83 as libc::c_int as u16;
        }
    }
    i2c_buffer_size_bytes = (0x83 as libc::c_int + 5 as libc::c_int
        - i2c_index as libc::c_int) as u16;
    pbuffer = &mut *buffer.as_mut_ptr().offset(0 as libc::c_int as isize)
        as *mut u8;
    i = 0 as libc::c_int as u16;
    while (i as libc::c_int) < i2c_buffer_size_bytes as libc::c_int {
        let fresh16 = pbuffer;
        pbuffer = pbuffer.offset(1);
        *fresh16 = 0 as libc::c_int as u8;
        i = i.wrapping_add(1);
    }
    if device_config_level as libc::c_int
        >= 6 as libc::c_int as VL53LX_DeviceConfigLevel as libc::c_int
        && status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int
    {
        i2c_buffer_offset_bytes = (0x1 as libc::c_int - i2c_index as libc::c_int)
            as u16;
        status = VL53LX_i2c_encode_static_nvm_managed(
            pstatic_nvm,
            11 as libc::c_int as u16,
            &mut *buffer.as_mut_ptr().offset(i2c_buffer_offset_bytes as isize),
        );
    }
    if device_config_level as libc::c_int
        >= 5 as libc::c_int as VL53LX_DeviceConfigLevel as libc::c_int
        && status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int
    {
        i2c_buffer_offset_bytes = (0xd as libc::c_int - i2c_index as libc::c_int)
            as u16;
        status = VL53LX_i2c_encode_customer_nvm_managed(
            pcustomer_nvm,
            23 as libc::c_int as u16,
            &mut *buffer.as_mut_ptr().offset(i2c_buffer_offset_bytes as isize),
        );
    }
    if device_config_level as libc::c_int
        >= 4 as libc::c_int as VL53LX_DeviceConfigLevel as libc::c_int
        && status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int
    {
        i2c_buffer_offset_bytes = (0x24 as libc::c_int - i2c_index as libc::c_int)
            as u16;
        status = VL53LX_i2c_encode_static_config(
            pstatic,
            32 as libc::c_int as u16,
            &mut *buffer.as_mut_ptr().offset(i2c_buffer_offset_bytes as isize),
        );
    }
    if device_config_level as libc::c_int
        >= 3 as libc::c_int as VL53LX_DeviceConfigLevel as libc::c_int
        && status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int
    {
        i2c_buffer_offset_bytes = (0x44 as libc::c_int - i2c_index as libc::c_int)
            as u16;
        status = VL53LX_i2c_encode_general_config(
            pgeneral,
            22 as libc::c_int as u16,
            &mut *buffer.as_mut_ptr().offset(i2c_buffer_offset_bytes as isize),
        );
    }
    if device_config_level as libc::c_int
        >= 2 as libc::c_int as VL53LX_DeviceConfigLevel as libc::c_int
        && status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int
    {
        i2c_buffer_offset_bytes = (0x5a as libc::c_int - i2c_index as libc::c_int)
            as u16;
        status = VL53LX_i2c_encode_timing_config(
            ptiming,
            23 as libc::c_int as u16,
            &mut *buffer.as_mut_ptr().offset(i2c_buffer_offset_bytes as isize),
        );
    }
    if device_config_level as libc::c_int
        >= 1 as libc::c_int as VL53LX_DeviceConfigLevel as libc::c_int
        && status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int
    {
        i2c_buffer_offset_bytes = (0x71 as libc::c_int - i2c_index as libc::c_int)
            as u16;
        if (*psystem).system__mode_start as libc::c_int
            & 0x20 as libc::c_int as VL53LX_DeviceMeasurementModes as libc::c_int
            == 0x20 as libc::c_int as VL53LX_DeviceMeasurementModes as libc::c_int
        {
            (*pdynamic)
                .system__grouped_parameter_hold_0 = ((*pstate).cfg_gph_id as libc::c_int
                | 0x1 as libc::c_int) as u8;
            (*pdynamic)
                .system__grouped_parameter_hold_1 = ((*pstate).cfg_gph_id as libc::c_int
                | 0x1 as libc::c_int) as u8;
            (*pdynamic).system__grouped_parameter_hold = (*pstate).cfg_gph_id;
        }
        status = VL53LX_i2c_encode_dynamic_config(
            pdynamic,
            18 as libc::c_int as u16,
            &mut *buffer.as_mut_ptr().offset(i2c_buffer_offset_bytes as isize),
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        i2c_buffer_offset_bytes = (0x83 as libc::c_int - i2c_index as libc::c_int)
            as u16;
        status = VL53LX_i2c_encode_system_control(
            psystem,
            5 as libc::c_int as u16,
            &mut *buffer.as_mut_ptr().offset(i2c_buffer_offset_bytes as isize),
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_WriteMulti(
            Dev,
            i2c_index,
            buffer.as_mut_ptr(),
            i2c_buffer_size_bytes as u32,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_update_ll_driver_rd_state(Dev);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_update_ll_driver_cfg_state(Dev);
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_stop_range(mut Dev: VL53LX_DEV) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    let mut pres: *mut VL53LX_LLDriverResults_t = &mut (*Dev).Data.llresults;
    (*pdev)
        .sys_ctrl
        .system__mode_start = ((*pdev).sys_ctrl.system__mode_start as libc::c_int
        & 0xf as libc::c_int
        | 0x80 as libc::c_int as VL53LX_DeviceMeasurementModes as libc::c_int)
        as u8;
    status = VL53LX_set_system_control(Dev, &mut (*pdev).sys_ctrl);
    (*pdev)
        .sys_ctrl
        .system__mode_start = ((*pdev).sys_ctrl.system__mode_start as libc::c_int
        & 0xf as libc::c_int) as u8;
    VL53LX_init_ll_driver_state(Dev, 3 as libc::c_int as VL53LX_DeviceState);
    V53L1_init_zone_results_structure(
        ((*pdev).zone_cfg.active_zones as libc::c_int + 1 as libc::c_int) as u8,
        &mut (*pres).zone_results,
    );
    V53L1_init_zone_dss_configs(Dev);
    if (*pdev).low_power_auto_data.is_low_power_auto_mode as libc::c_int
        == 1 as libc::c_int
    {
        VL53LX_low_power_auto_data_stop_range(Dev);
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_get_measurement_results(
    mut Dev: VL53LX_DEV,
    mut device_results_level: VL53LX_DeviceResultsLevel,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    let mut buffer: [u8; 256] = [0; 256];
    let mut psystem_results: *mut VL53LX_system_results_t = &mut (*pdev).sys_results;
    let mut pcore_results: *mut VL53LX_core_results_t = &mut (*pdev).core_results;
    let mut pdebug_results: *mut VL53LX_debug_results_t = &mut (*pdev).dbg_results;
    let mut i2c_index: u16 = 0x88 as libc::c_int as u16;
    let mut i2c_buffer_offset_bytes: u16 = 0 as libc::c_int as u16;
    let mut i2c_buffer_size_bytes: u16 = 0 as libc::c_int as u16;
    match device_results_level as libc::c_int {
        2 => {
            i2c_buffer_size_bytes = (0xd6 as libc::c_int + 56 as libc::c_int
                - i2c_index as libc::c_int) as u16;
        }
        1 => {
            i2c_buffer_size_bytes = (0xb4 as libc::c_int + 33 as libc::c_int
                - i2c_index as libc::c_int) as u16;
        }
        _ => {
            i2c_buffer_size_bytes = 44 as libc::c_int as u16;
        }
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_ReadMulti(
            Dev,
            i2c_index,
            buffer.as_mut_ptr(),
            i2c_buffer_size_bytes as u32,
        );
    }
    if device_results_level as libc::c_int
        >= 2 as libc::c_int as VL53LX_DeviceResultsLevel as libc::c_int
        && status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int
    {
        i2c_buffer_offset_bytes = (0xd6 as libc::c_int - i2c_index as libc::c_int)
            as u16;
        status = VL53LX_i2c_decode_debug_results(
            56 as libc::c_int as u16,
            &mut *buffer.as_mut_ptr().offset(i2c_buffer_offset_bytes as isize),
            pdebug_results,
        );
    }
    if device_results_level as libc::c_int
        >= 1 as libc::c_int as VL53LX_DeviceResultsLevel as libc::c_int
        && status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int
    {
        i2c_buffer_offset_bytes = (0xb4 as libc::c_int - i2c_index as libc::c_int)
            as u16;
        status = VL53LX_i2c_decode_core_results(
            33 as libc::c_int as u16,
            &mut *buffer.as_mut_ptr().offset(i2c_buffer_offset_bytes as isize),
            pcore_results,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        i2c_buffer_offset_bytes = 0 as libc::c_int as u16;
        status = VL53LX_i2c_decode_system_results(
            44 as libc::c_int as u16,
            &mut *buffer.as_mut_ptr().offset(i2c_buffer_offset_bytes as isize),
            psystem_results,
        );
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_get_device_results(
    mut Dev: VL53LX_DEV,
    mut device_results_level: VL53LX_DeviceResultsLevel,
    mut prange_results: *mut VL53LX_range_results_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    let mut pres: *mut VL53LX_LLDriverResults_t = &mut (*Dev).Data.llresults;
    let mut presults: *mut VL53LX_range_results_t = &mut (*pres).range_results;
    let mut pobjects: *mut VL53LX_zone_objects_t = &mut *((*pres)
        .zone_results
        .VL53LX_p_003)
        .as_mut_ptr()
        .offset(0 as libc::c_int as isize) as *mut VL53LX_zone_objects_t;
    let mut pstate: *mut VL53LX_ll_driver_state_t = &mut (*pdev).ll_state;
    let mut pzone_cfg: *mut VL53LX_zone_config_t = &mut (*pdev).zone_cfg;
    let mut phist_info: *mut VL53LX_zone_hist_info_t = &mut *((*pres)
        .zone_hists
        .VL53LX_p_003)
        .as_mut_ptr()
        .offset(0 as libc::c_int as isize) as *mut VL53LX_zone_hist_info_t;
    let mut dmax_cal: VL53LX_dmax_calibration_data_t = VL53LX_dmax_calibration_data_t {
        ref__actual_effective_spads: 0,
        ref__peak_signal_count_rate_mcps: 0,
        ref__distance_mm: 0,
        ref_reflectance_pc: 0,
        coverglass_transmission: 0,
    };
    let mut pdmax_cal: *mut VL53LX_dmax_calibration_data_t = &mut dmax_cal;
    let mut pHP: *mut VL53LX_hist_post_process_config_t = &mut (*pdev).histpostprocess;
    let mut pC: *mut VL53LX_xtalk_config_t = &mut (*pdev).xtalk_cfg;
    let mut pL: *mut VL53LX_low_power_auto_data_t = &mut (*pdev).low_power_auto_data;
    let mut pHD: *mut VL53LX_histogram_bin_data_t = &mut (*pdev).hist_data;
    let mut pN: *mut VL53LX_customer_nvm_managed_t = &mut (*pdev).customer;
    let mut pZH: *mut VL53LX_zone_histograms_t = &mut (*pres).zone_hists;
    let mut pXCR: *mut VL53LX_xtalk_calibration_results_t = &mut (*pdev).xtalk_cal;
    let mut tmp8: u8 = 0;
    let mut zid: u8 = 0;
    let mut i: u8 = 0;
    let mut histo_merge_nb: u8 = 0;
    let mut idx: u8 = 0;
    let mut pdata: *mut VL53LX_range_data_t = 0 as *mut VL53LX_range_data_t;
    if (*pdev).sys_ctrl.system__mode_start as libc::c_int & 0x2 as libc::c_int
        == 0x2 as libc::c_int
    {
        status = VL53LX_get_histogram_bin_data(Dev, &mut (*pdev).hist_data);
        if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int
            && (*pHD).number_of_ambient_bins as libc::c_int == 0 as libc::c_int
        {
            zid = (*pdev).ll_state.rd_zone_id;
            status = VL53LX_hist_copy_and_scale_ambient_info(
                &mut *((*pZH).VL53LX_p_003).as_mut_ptr().offset(zid as isize),
                &mut (*pdev).hist_data,
            );
        }
        if !(status as libc::c_int != 0 as libc::c_int as VL53LX_Error as libc::c_int) {
            VL53LX_compute_histo_merge_nb(Dev, &mut histo_merge_nb);
            if histo_merge_nb as libc::c_int == 0 as libc::c_int {
                histo_merge_nb = 1 as libc::c_int as u8;
            }
            idx = (histo_merge_nb as libc::c_int - 1 as libc::c_int) as u8;
            if (*pdev).tuning_parms.tp_hist_merge as libc::c_int == 1 as libc::c_int {
                (*pC)
                    .algo__crosstalk_compensation_plane_offset_kcps = (*pXCR)
                    .algo__xtalk_cpo_HistoMerge_kcps[idx as usize];
            }
            (*pHP).gain_factor = (*pdev).gain_cal.histogram_ranging_gain_factor;
            (*pHP)
                .algo__crosstalk_compensation_plane_offset_kcps = VL53LX_calc_crosstalk_plane_offset_with_margin(
                (*pC).algo__crosstalk_compensation_plane_offset_kcps,
                (*pC).histogram_mode_crosstalk_margin_kcps,
            );
            (*pHP)
                .algo__crosstalk_compensation_x_plane_gradient_kcps = (*pC)
                .algo__crosstalk_compensation_x_plane_gradient_kcps;
            (*pHP)
                .algo__crosstalk_compensation_y_plane_gradient_kcps = (*pC)
                .algo__crosstalk_compensation_y_plane_gradient_kcps;
            (*pdev).dmax_cfg.ambient_thresh_sigma = (*pHP).ambient_thresh_sigma1;
            (*pdev)
                .dmax_cfg
                .min_ambient_thresh_events = (*pHP).min_ambient_thresh_events;
            (*pdev)
                .dmax_cfg
                .signal_total_events_limit = (*pHP).signal_total_events_limit;
            (*pdev)
                .dmax_cfg
                .dss_config__target_total_rate_mcps = (*pdev)
                .stat_cfg
                .dss_config__target_total_rate_mcps;
            (*pdev)
                .dmax_cfg
                .dss_config__aperture_attenuation = (*pdev)
                .gen_cfg
                .dss_config__aperture_attenuation;
            (*pHP)
                .algo__crosstalk_detect_max_valid_range_mm = (*pC)
                .algo__crosstalk_detect_max_valid_range_mm;
            (*pHP)
                .algo__crosstalk_detect_min_valid_range_mm = (*pC)
                .algo__crosstalk_detect_min_valid_range_mm;
            (*pHP)
                .algo__crosstalk_detect_max_valid_rate_kcps = (*pC)
                .algo__crosstalk_detect_max_valid_rate_kcps;
            (*pHP)
                .algo__crosstalk_detect_max_sigma_mm = (*pC)
                .algo__crosstalk_detect_max_sigma_mm;
            VL53LX_copy_rtn_good_spads_to_buffer(
                &mut (*pdev).nvm_copy_data,
                &mut *((*pdev).rtn_good_spads)
                    .as_mut_ptr()
                    .offset(0 as libc::c_int as isize),
            );
            match (*pdev).offset_correction_mode as libc::c_int {
                1 => {
                    tmp8 = (*pdev).gen_cfg.dss_config__aperture_attenuation;
                    VL53LX_hist_combine_mm1_mm2_offsets(
                        (*pN).mm_config__inner_offset_mm,
                        (*pN).mm_config__outer_offset_mm,
                        (*pdev).nvm_copy_data.roi_config__mode_roi_centre_spad,
                        (*pdev).nvm_copy_data.roi_config__mode_roi_xy_size,
                        (*pHD).roi_config__user_roi_centre_spad,
                        (*pHD).roi_config__user_roi_requested_global_xy_size,
                        &mut (*pdev).add_off_cal_data,
                        &mut *((*pdev).rtn_good_spads)
                            .as_mut_ptr()
                            .offset(0 as libc::c_int as isize),
                        tmp8 as u16,
                        &mut (*pHP).range_offset_mm,
                    );
                }
                3 => {
                    select_offset_per_vcsel(pdev, &mut (*pHP).range_offset_mm);
                    let ref mut fresh17 = (*pHP).range_offset_mm;
                    *fresh17 = (*fresh17 as libc::c_int * 4 as libc::c_int) as i16;
                }
                _ => {
                    (*pHP).range_offset_mm = 0 as libc::c_int as i16;
                }
            }
            if !(status as libc::c_int
                != 0 as libc::c_int as VL53LX_Error as libc::c_int)
            {
                VL53LX_calc_max_effective_spads(
                    (*pHD).roi_config__user_roi_centre_spad,
                    (*pHD).roi_config__user_roi_requested_global_xy_size,
                    &mut *((*pdev).rtn_good_spads)
                        .as_mut_ptr()
                        .offset(0 as libc::c_int as isize),
                    (*pdev).gen_cfg.dss_config__aperture_attenuation as u16,
                    &mut (*pdev).dmax_cfg.max_effective_spads,
                );
                status = VL53LX_get_dmax_calibration_data(
                    Dev,
                    (*pdev).dmax_mode,
                    pdmax_cal,
                );
                if !(status as libc::c_int
                    != 0 as libc::c_int as VL53LX_Error as libc::c_int)
                {
                    status = VL53LX_ipp_hist_process_data(
                        Dev,
                        pdmax_cal,
                        &mut (*pdev).dmax_cfg,
                        &mut (*pdev).histpostprocess,
                        &mut (*pdev).hist_data,
                        &mut (*pdev).xtalk_shapes,
                        ((*pdev).wArea1).as_mut_ptr(),
                        ((*pdev).wArea2).as_mut_ptr(),
                        &mut histo_merge_nb,
                        presults,
                    );
                    if (*pdev).tuning_parms.tp_hist_merge as libc::c_int
                        == 1 as libc::c_int
                        && histo_merge_nb as libc::c_int > 1 as libc::c_int
                    {
                        i = 0 as libc::c_int as u8;
                        while (i as libc::c_int) < 4 as libc::c_int {
                            pdata = &mut *((*presults).VL53LX_p_003)
                                .as_mut_ptr()
                                .offset(i as isize) as *mut VL53LX_range_data_t;
                            let ref mut fresh18 = (*pdata).VL53LX_p_016;
                            *fresh18 = (*fresh18 as libc::c_uint)
                                .wrapping_div(histo_merge_nb as libc::c_uint) as u32
                                as u32;
                            let ref mut fresh19 = (*pdata).VL53LX_p_017;
                            *fresh19 = (*fresh19 as libc::c_uint)
                                .wrapping_div(histo_merge_nb as libc::c_uint) as u32
                                as u32;
                            let ref mut fresh20 = (*pdata).VL53LX_p_010;
                            *fresh20 /= histo_merge_nb as libc::c_int;
                            let ref mut fresh21 = (*pdata).peak_signal_count_rate_mcps;
                            *fresh21 = (*fresh21 as libc::c_int
                                / histo_merge_nb as libc::c_int) as u16;
                            let ref mut fresh22 = (*pdata).avg_signal_count_rate_mcps;
                            *fresh22 = (*fresh22 as libc::c_int
                                / histo_merge_nb as libc::c_int) as u16;
                            let ref mut fresh23 = (*pdata).ambient_count_rate_mcps;
                            *fresh23 = (*fresh23 as libc::c_int
                                / histo_merge_nb as libc::c_int) as u16;
                            let ref mut fresh24 = (*pdata).VL53LX_p_009;
                            *fresh24 = (*fresh24 as libc::c_uint)
                                .wrapping_div(histo_merge_nb as libc::c_uint) as u32
                                as u32;
                            i = i.wrapping_add(1);
                        }
                    }
                    if !(status as libc::c_int
                        != 0 as libc::c_int as VL53LX_Error as libc::c_int)
                    {
                        status = VL53LX_hist_wrap_dmax(
                            &mut (*pdev).histpostprocess,
                            &mut (*pdev).hist_data,
                            &mut (*presults).wrap_dmax_mm,
                        );
                        if !(status as libc::c_int
                            != 0 as libc::c_int as VL53LX_Error as libc::c_int)
                        {
                            zid = (*pdev).ll_state.rd_zone_id;
                            status = VL53LX_hist_phase_consistency_check(
                                Dev,
                                &mut *((*pZH).VL53LX_p_003)
                                    .as_mut_ptr()
                                    .offset(zid as isize),
                                &mut *((*pres).zone_results.VL53LX_p_003)
                                    .as_mut_ptr()
                                    .offset(zid as isize),
                                presults,
                            );
                            if !(status as libc::c_int
                                != 0 as libc::c_int as VL53LX_Error as libc::c_int)
                            {
                                zid = (*pdev).ll_state.rd_zone_id;
                                status = VL53LX_hist_xmonitor_consistency_check(
                                    Dev,
                                    &mut *((*pZH).VL53LX_p_003)
                                        .as_mut_ptr()
                                        .offset(zid as isize),
                                    &mut *((*pres).zone_results.VL53LX_p_003)
                                        .as_mut_ptr()
                                        .offset(zid as isize),
                                    &mut (*presults).xmonitor,
                                );
                                if !(status as libc::c_int
                                    != 0 as libc::c_int as VL53LX_Error as libc::c_int)
                                {
                                    zid = (*pdev).ll_state.rd_zone_id;
                                    (*pZH).max_zones = 5 as libc::c_int as u8;
                                    (*pZH)
                                        .active_zones = ((*pdev).zone_cfg.active_zones
                                        as libc::c_int + 1 as libc::c_int) as u8;
                                    (*pHD).zone_id = zid;
                                    if (zid as libc::c_int)
                                        < (*pres).zone_results.max_zones as libc::c_int
                                    {
                                        phist_info = &mut *((*pZH).VL53LX_p_003)
                                            .as_mut_ptr()
                                            .offset(zid as isize) as *mut VL53LX_zone_hist_info_t;
                                        (*phist_info).rd_device_state = (*pHD).rd_device_state;
                                        (*phist_info)
                                            .number_of_ambient_bins = (*pHD).number_of_ambient_bins;
                                        (*phist_info)
                                            .result__dss_actual_effective_spads = (*pHD)
                                            .result__dss_actual_effective_spads;
                                        (*phist_info).VL53LX_p_005 = (*pHD).VL53LX_p_005;
                                        (*phist_info)
                                            .total_periods_elapsed = (*pHD).total_periods_elapsed;
                                        (*phist_info)
                                            .ambient_events_sum = (*pHD).ambient_events_sum;
                                    }
                                    if !(status as libc::c_int
                                        != 0 as libc::c_int as VL53LX_Error as libc::c_int)
                                    {
                                        VL53LX_hist_copy_results_to_sys_and_core(
                                            &mut (*pdev).hist_data,
                                            presults,
                                            &mut (*pdev).sys_results,
                                            &mut (*pdev).core_results,
                                        );
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        if (*pzone_cfg).active_zones as libc::c_int > 0 as libc::c_int {
            if (*pstate).rd_device_state as libc::c_int
                != 6 as libc::c_int as VL53LX_DeviceState as libc::c_int
            {
                if status as libc::c_int
                    == 0 as libc::c_int as VL53LX_Error as libc::c_int
                {
                    status = VL53LX_dynamic_zone_update(Dev, presults);
                }
            }
            i = 0 as libc::c_int as u8;
            while (i as libc::c_int) < 5 as libc::c_int {
                (*pzone_cfg)
                    .bin_config[i
                    as usize] = (if (*pdev).ll_state.cfg_internal_stream_count
                    as libc::c_int & 0x1 as libc::c_int != 0
                {
                    3 as libc::c_int as VL53LX_ZoneConfig_BinConfig_select as libc::c_int
                } else {
                    1 as libc::c_int as VL53LX_ZoneConfig_BinConfig_select as libc::c_int
                }) as u8;
                i = i.wrapping_add(1);
            }
            if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
                status = VL53LX_multizone_hist_bins_update(Dev);
            }
        }
        if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
            status = VL53LX_dynamic_xtalk_correction_corrector(Dev);
        }
        if (*pdev).tuning_parms.tp_hist_merge as libc::c_int == 1 as libc::c_int {
            (*pC)
                .algo__crosstalk_compensation_plane_offset_kcps = (*pXCR)
                .algo__xtalk_cpo_HistoMerge_kcps[0 as libc::c_int as usize];
        }
    } else {
        if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
            status = VL53LX_get_measurement_results(Dev, device_results_level);
        }
        if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
            VL53LX_copy_sys_and_core_results_to_range_results(
                (*pdev).gain_cal.standard_ranging_gain_factor as i32,
                &mut (*pdev).sys_results,
                &mut (*pdev).core_results,
                presults,
            );
        }
        if (*pL).is_low_power_auto_mode as libc::c_int == 1 as libc::c_int {
            if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int
                && (*pL).low_power_auto_range_count as libc::c_int == 0 as libc::c_int
            {
                status = VL53LX_low_power_auto_setup_manual_calibration(Dev);
                (*pL).low_power_auto_range_count = 1 as libc::c_int as u8;
            } else if status as libc::c_int
                    == 0 as libc::c_int as VL53LX_Error as libc::c_int
                    && (*pL).low_power_auto_range_count as libc::c_int
                        == 1 as libc::c_int
                {
                (*pL).low_power_auto_range_count = 2 as libc::c_int as u8;
            }
            if (*pL).low_power_auto_range_count as libc::c_int != 0xff as libc::c_int
                && status as libc::c_int
                    == 0 as libc::c_int as VL53LX_Error as libc::c_int
            {
                status = VL53LX_low_power_auto_update_DSS(Dev);
            }
        }
    }
    (*presults).cfg_device_state = (*pdev).ll_state.cfg_device_state;
    (*presults).rd_device_state = (*pdev).ll_state.rd_device_state;
    (*presults).zone_id = (*pdev).ll_state.rd_zone_id;
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        (*pres).zone_results.max_zones = 5 as libc::c_int as u8;
        (*pres)
            .zone_results
            .active_zones = ((*pdev).zone_cfg.active_zones as libc::c_int
            + 1 as libc::c_int) as u8;
        zid = (*pdev).ll_state.rd_zone_id;
        if (zid as libc::c_int) < (*pres).zone_results.max_zones as libc::c_int {
            pobjects = &mut *((*pres).zone_results.VL53LX_p_003)
                .as_mut_ptr()
                .offset(zid as isize) as *mut VL53LX_zone_objects_t;
            (*pobjects).cfg_device_state = (*presults).cfg_device_state;
            (*pobjects).rd_device_state = (*presults).rd_device_state;
            (*pobjects).zone_id = (*presults).zone_id;
            (*pobjects).stream_count = (*presults).stream_count;
            (*pobjects).xmonitor.VL53LX_p_016 = (*presults).xmonitor.VL53LX_p_016;
            (*pobjects).xmonitor.VL53LX_p_017 = (*presults).xmonitor.VL53LX_p_017;
            (*pobjects).xmonitor.VL53LX_p_011 = (*presults).xmonitor.VL53LX_p_011;
            (*pobjects).xmonitor.range_status = (*presults).xmonitor.range_status;
            (*pobjects).max_objects = (*presults).max_results;
            (*pobjects).active_objects = (*presults).active_results;
            i = 0 as libc::c_int as u8;
            while (i as libc::c_int) < (*presults).active_results as libc::c_int {
                (*pobjects)
                    .VL53LX_p_003[i as usize]
                    .VL53LX_p_016 = (*presults).VL53LX_p_003[i as usize].VL53LX_p_016;
                (*pobjects)
                    .VL53LX_p_003[i as usize]
                    .VL53LX_p_017 = (*presults).VL53LX_p_003[i as usize].VL53LX_p_017;
                (*pobjects)
                    .VL53LX_p_003[i as usize]
                    .VL53LX_p_011 = (*presults).VL53LX_p_003[i as usize].VL53LX_p_011;
                (*pobjects)
                    .VL53LX_p_003[i as usize]
                    .range_status = (*presults).VL53LX_p_003[i as usize].range_status;
                i = i.wrapping_add(1);
            }
        }
    }
    memcpy(
        prange_results as *mut libc::c_void,
        presults as *const libc::c_void,
        ::std::mem::size_of::<VL53LX_range_results_t>() as libc::c_ulong,
    );
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_check_ll_driver_rd_state(Dev);
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_clear_interrupt_and_enable_next_range(
    mut Dev: VL53LX_DEV,
    mut measurement_mode: u8,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_init_and_start_range(
            Dev,
            measurement_mode,
            3 as libc::c_int as VL53LX_DeviceConfigLevel,
        );
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_get_histogram_bin_data(
    mut Dev: VL53LX_DEV,
    mut pdata: *mut VL53LX_histogram_bin_data_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    let mut pres: *mut VL53LX_LLDriverResults_t = &mut (*Dev).Data.llresults;
    let mut pzone_dyn_cfg: *mut VL53LX_zone_private_dyn_cfg_t = 0
        as *mut VL53LX_zone_private_dyn_cfg_t;
    let mut pstat_nvm: *mut VL53LX_static_nvm_managed_t = &mut (*pdev).stat_nvm;
    let mut pstat_cfg: *mut VL53LX_static_config_t = &mut (*pdev).stat_cfg;
    let mut pgen_cfg: *mut VL53LX_general_config_t = &mut (*pdev).gen_cfg;
    let mut ptim_cfg: *mut VL53LX_timing_config_t = &mut (*pdev).tim_cfg;
    let mut presults: *mut VL53LX_range_results_t = &mut (*pres).range_results;
    let mut buffer: [u8; 256] = [0; 256];
    let mut pbuffer: *mut u8 = &mut *buffer
        .as_mut_ptr()
        .offset(0 as libc::c_int as isize) as *mut u8;
    let mut bin_23_0: u8 = 0 as libc::c_int as u8;
    let mut bin: u16 = 0 as libc::c_int as u16;
    let mut i2c_buffer_offset_bytes: u16 = 0 as libc::c_int as u16;
    let mut encoded_timeout: u16 = 0 as libc::c_int as u16;
    let mut pll_period_us: u32 = 0 as libc::c_int as u32;
    let mut periods_elapsed_tmp: u32 = 0 as libc::c_int as u32;
    let mut i: u8 = 0 as libc::c_int as u8;
    let mut hist_merge: i32 = 0 as libc::c_int;
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_ReadMulti(
            Dev,
            0x88 as libc::c_int as u16,
            pbuffer,
            (0xda as libc::c_int - 0x88 as libc::c_int + 1 as libc::c_int) as u32,
        );
    }
    (*pdata).result__interrupt_status = *pbuffer.offset(0 as libc::c_int as isize);
    (*pdata).result__range_status = *pbuffer.offset(1 as libc::c_int as isize);
    (*pdata).result__report_status = *pbuffer.offset(2 as libc::c_int as isize);
    (*pdata).result__stream_count = *pbuffer.offset(3 as libc::c_int as isize);
    (*pdata)
        .result__dss_actual_effective_spads = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(4 as libc::c_int as isize),
    );
    i2c_buffer_offset_bytes = (0xd6 as libc::c_int - 0x88 as libc::c_int) as u16;
    pbuffer = &mut *buffer.as_mut_ptr().offset(i2c_buffer_offset_bytes as isize)
        as *mut u8;
    (*pdata)
        .phasecal_result__reference_phase = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer,
    );
    i2c_buffer_offset_bytes = (0xd8 as libc::c_int - 0x88 as libc::c_int) as u16;
    (*pdata).phasecal_result__vcsel_start = buffer[i2c_buffer_offset_bytes as usize];
    (*pdev)
        .dbg_results
        .phasecal_result__reference_phase = (*pdata).phasecal_result__reference_phase;
    (*pdev)
        .dbg_results
        .phasecal_result__vcsel_start = (*pdata).phasecal_result__vcsel_start;
    i2c_buffer_offset_bytes = (0xd9 as libc::c_int - 0x88 as libc::c_int) as u16;
    bin_23_0 = ((buffer[i2c_buffer_offset_bytes as usize] as libc::c_int)
        << 2 as libc::c_int) as u8;
    i2c_buffer_offset_bytes = (0xda as libc::c_int - 0x88 as libc::c_int) as u16;
    bin_23_0 = (bin_23_0 as libc::c_int
        + buffer[i2c_buffer_offset_bytes as usize] as libc::c_int) as u8;
    i2c_buffer_offset_bytes = (0xd5 as libc::c_int - 0x88 as libc::c_int) as u16;
    buffer[i2c_buffer_offset_bytes as usize] = bin_23_0;
    i2c_buffer_offset_bytes = (0x8e as libc::c_int - 0x88 as libc::c_int) as u16;
    pbuffer = &mut *buffer.as_mut_ptr().offset(i2c_buffer_offset_bytes as isize)
        as *mut u8;
    bin = 0 as libc::c_int as u16;
    while (bin as libc::c_int) < 24 as libc::c_int {
        (*pdata)
            .bin_data[bin
            as usize] = VL53LX_i2c_decode_uint32_t(3 as libc::c_int as u16, pbuffer)
            as i32;
        pbuffer = pbuffer.offset(3 as libc::c_int as isize);
        bin = bin.wrapping_add(1);
    }
    VL53LX_get_tuning_parm(
        Dev,
        (0x8000 as libc::c_int + 141 as libc::c_int) as VL53LX_TuningParms,
        &mut hist_merge,
    );
    if (*pdata).result__stream_count as libc::c_int == 0 as libc::c_int {
        memset(
            ((*pdev).multi_bins_rec).as_mut_ptr() as *mut libc::c_void,
            0 as libc::c_int,
            ::std::mem::size_of::<[[[i32; 24]; 2]; 6]>() as libc::c_ulong,
        );
        (*pdev).bin_rec_pos = 0 as libc::c_int as u8;
        (*pdev).pos_before_next_recom = 0 as libc::c_int as u8;
    }
    if hist_merge == 1 as libc::c_int {
        vl53lx_histo_merge(Dev, pdata);
    }
    (*pdata).zone_id = (*pdev).ll_state.rd_zone_id;
    (*pdata).VL53LX_p_019 = 0 as libc::c_int as u8;
    (*pdata).VL53LX_p_020 = 24 as libc::c_int as u8;
    (*pdata).VL53LX_p_021 = 24 as libc::c_int as u8;
    (*pdata).cal_config__vcsel_start = (*pgen_cfg).cal_config__vcsel_start;
    (*pdata)
        .vcsel_width = (((*pgen_cfg).global_config__vcsel_width as u16
        as libc::c_int) << 4 as libc::c_int) as u16;
    let ref mut fresh25 = (*pdata).vcsel_width;
    *fresh25 = (*fresh25 as libc::c_int
        + (*pstat_cfg).ana_config__vcsel_pulse_width_offset as u16 as libc::c_int)
        as u16;
    (*pdata).VL53LX_p_015 = (*pstat_nvm).osc_measured__fast_osc__frequency;
    VL53LX_hist_get_bin_sequence_config(Dev, pdata);
    if (*pdev).ll_state.rd_timing_status as libc::c_int == 0 as libc::c_int {
        encoded_timeout = ((((*ptim_cfg).range_config__timeout_macrop_a_hi
            as libc::c_int) << 8 as libc::c_int)
            + (*ptim_cfg).range_config__timeout_macrop_a_lo as libc::c_int) as u16;
        (*pdata).VL53LX_p_005 = (*ptim_cfg).range_config__vcsel_period_a;
    } else {
        encoded_timeout = ((((*ptim_cfg).range_config__timeout_macrop_b_hi
            as libc::c_int) << 8 as libc::c_int)
            + (*ptim_cfg).range_config__timeout_macrop_b_lo as libc::c_int) as u16;
        (*pdata).VL53LX_p_005 = (*ptim_cfg).range_config__vcsel_period_b;
    }
    (*pdata).number_of_ambient_bins = 0 as libc::c_int as u8;
    i = 0 as libc::c_int as u8;
    while (i as libc::c_int) < 6 as libc::c_int {
        if (*pdata).bin_seq[i as usize] as libc::c_int & 0x7 as libc::c_int
            == 0x7 as libc::c_int
        {
            (*pdata)
                .number_of_ambient_bins = ((*pdata).number_of_ambient_bins as libc::c_int
                + 0x4 as libc::c_int) as u8;
        }
        i = i.wrapping_add(1);
    }
    (*pdata).total_periods_elapsed = VL53LX_decode_timeout(encoded_timeout);
    pll_period_us = VL53LX_calc_pll_period_us((*pdata).VL53LX_p_015);
    periods_elapsed_tmp = ((*pdata).total_periods_elapsed)
        .wrapping_add(1 as libc::c_int as libc::c_uint);
    (*pdata)
        .peak_duration_us = VL53LX_duration_maths(
        pll_period_us,
        (*pdata).vcsel_width as u32,
        2048 as libc::c_int as u32,
        periods_elapsed_tmp,
    );
    (*pdata).woi_duration_us = 0 as libc::c_int as u32;
    VL53LX_hist_calc_zero_distance_phase(pdata);
    VL53LX_hist_estimate_ambient_from_ambient_bins(pdata);
    (*pdata).cfg_device_state = (*pdev).ll_state.cfg_device_state;
    (*pdata).rd_device_state = (*pdev).ll_state.rd_device_state;
    pzone_dyn_cfg = &mut *((*pres).zone_dyn_cfgs.VL53LX_p_003)
        .as_mut_ptr()
        .offset((*pdata).zone_id as isize) as *mut VL53LX_zone_private_dyn_cfg_t;
    (*pdata)
        .roi_config__user_roi_centre_spad = (*pzone_dyn_cfg)
        .roi_config__user_roi_centre_spad;
    (*pdata)
        .roi_config__user_roi_requested_global_xy_size = (*pzone_dyn_cfg)
        .roi_config__user_roi_requested_global_xy_size;
    (*presults).device_status = 0 as libc::c_int as VL53LX_DeviceError;
    match (*pdata).result__range_status as libc::c_int & 0x1f as libc::c_int {
        1 | 2 | 3 | 13 | 17 => {
            (*presults)
                .device_status = ((*pdata).result__range_status as libc::c_int
                & 0x1f as libc::c_int) as u8;
            status = -(6 as libc::c_int) as VL53LX_Error;
        }
        _ => {}
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_copy_sys_and_core_results_to_range_results(
    mut gain_factor: i32,
    mut psys: *mut VL53LX_system_results_t,
    mut pcore: *mut VL53LX_core_results_t,
    mut presults: *mut VL53LX_range_results_t,
) {
    let mut i: u8 = 0 as libc::c_int as u8;
    let mut pdata: *mut VL53LX_range_data_t = 0 as *mut VL53LX_range_data_t;
    let mut range_mm: i32 = 0 as libc::c_int;
    let mut tmpu32: u32 = 0 as libc::c_int as u32;
    let mut rpscr_crosstalk_corrected_mcps_sd0: u16 = 0;
    let mut rmmo_effective_spads_sd0: u16 = 0;
    let mut rmmi_effective_spads_sd0: u16 = 0;
    (*presults).zone_id = 0 as libc::c_int as u8;
    (*presults).stream_count = (*psys).result__stream_count;
    (*presults).wrap_dmax_mm = 0 as libc::c_int as i16;
    (*presults).max_results = 4 as libc::c_int as u8;
    (*presults).active_results = 1 as libc::c_int as u8;
    rpscr_crosstalk_corrected_mcps_sd0 = (*psys)
        .result__peak_signal_count_rate_crosstalk_corrected_mcps_sd0;
    rmmo_effective_spads_sd0 = (*psys).result__mm_outer_actual_effective_spads_sd0;
    rmmi_effective_spads_sd0 = (*psys).result__mm_inner_actual_effective_spads_sd0;
    i = 0 as libc::c_int as u8;
    while (i as libc::c_int) < 5 as libc::c_int {
        (*presults).VL53LX_p_022[i as usize] = 0 as libc::c_int as i16;
        i = i.wrapping_add(1);
    }
    pdata = &mut *((*presults).VL53LX_p_003)
        .as_mut_ptr()
        .offset(0 as libc::c_int as isize) as *mut VL53LX_range_data_t;
    i = 0 as libc::c_int as u8;
    while (i as libc::c_int) < 2 as libc::c_int {
        (*pdata).range_id = i;
        (*pdata).time_stamp = 0 as libc::c_int as u32;
        if (*psys).result__stream_count as libc::c_int == 0 as libc::c_int
            && (*psys).result__range_status as libc::c_int & 0x1f as libc::c_int
                == 9 as libc::c_int as VL53LX_DeviceError as libc::c_int
        {
            (*pdata).range_status = 19 as libc::c_int as VL53LX_DeviceError;
        } else {
            (*pdata)
                .range_status = ((*psys).result__range_status as libc::c_int
                & 0x1f as libc::c_int) as u8;
        }
        (*pdata).VL53LX_p_012 = 0 as libc::c_int as u8;
        (*pdata).VL53LX_p_019 = 0 as libc::c_int as u8;
        (*pdata).VL53LX_p_023 = 0 as libc::c_int as u8;
        (*pdata).VL53LX_p_024 = 0 as libc::c_int as u8;
        (*pdata).VL53LX_p_013 = 0 as libc::c_int as u8;
        (*pdata).VL53LX_p_025 = 0 as libc::c_int as u8;
        match i as libc::c_int {
            0 => {
                if (*psys).result__report_status as libc::c_int
                    == 7 as libc::c_int as VL53LX_DeviceReportStatus as libc::c_int
                {
                    (*pdata).VL53LX_p_004 = rmmi_effective_spads_sd0;
                } else if (*psys).result__report_status as libc::c_int
                        == 8 as libc::c_int as VL53LX_DeviceReportStatus as libc::c_int
                    {
                    (*pdata).VL53LX_p_004 = rmmo_effective_spads_sd0;
                } else {
                    (*pdata)
                        .VL53LX_p_004 = (*psys).result__dss_actual_effective_spads_sd0;
                }
                (*pdata)
                    .peak_signal_count_rate_mcps = rpscr_crosstalk_corrected_mcps_sd0;
                (*pdata)
                    .avg_signal_count_rate_mcps = (*psys)
                    .result__avg_signal_count_rate_mcps_sd0;
                (*pdata)
                    .ambient_count_rate_mcps = (*psys)
                    .result__ambient_count_rate_mcps_sd0;
                tmpu32 = ((*psys).result__sigma_sd0 as u32) << 5 as libc::c_int;
                if tmpu32 > 0xffff as libc::c_int as libc::c_uint {
                    tmpu32 = 0xffff as libc::c_int as u32;
                }
                (*pdata).VL53LX_p_002 = tmpu32 as u16;
                (*pdata).VL53LX_p_011 = (*psys).result__phase_sd0;
                range_mm = (*psys).result__final_crosstalk_corrected_range_mm_sd0
                    as i32;
                range_mm *= gain_factor;
                range_mm += 0x400 as libc::c_int;
                range_mm /= 0x800 as libc::c_int;
                (*pdata).median_range_mm = range_mm as i16;
                (*pdata).VL53LX_p_017 = (*pcore).result_core__ranging_total_events_sd0;
                (*pdata).VL53LX_p_010 = (*pcore).result_core__signal_total_events_sd0;
                (*pdata)
                    .total_periods_elapsed = (*pcore)
                    .result_core__total_periods_elapsed_sd0;
                (*pdata).VL53LX_p_016 = (*pcore).result_core__ambient_window_events_sd0;
            }
            1 => {
                (*pdata).VL53LX_p_004 = (*psys).result__dss_actual_effective_spads_sd1;
                (*pdata)
                    .peak_signal_count_rate_mcps = (*psys)
                    .result__peak_signal_count_rate_mcps_sd1;
                (*pdata).avg_signal_count_rate_mcps = 0xffff as libc::c_int as u16;
                (*pdata)
                    .ambient_count_rate_mcps = (*psys)
                    .result__ambient_count_rate_mcps_sd1;
                tmpu32 = ((*psys).result__sigma_sd1 as u32) << 5 as libc::c_int;
                if tmpu32 > 0xffff as libc::c_int as libc::c_uint {
                    tmpu32 = 0xffff as libc::c_int as u32;
                }
                (*pdata).VL53LX_p_002 = tmpu32 as u16;
                (*pdata).VL53LX_p_011 = (*psys).result__phase_sd1;
                range_mm = (*psys).result__final_crosstalk_corrected_range_mm_sd1
                    as i32;
                range_mm *= gain_factor;
                range_mm += 0x400 as libc::c_int;
                range_mm /= 0x800 as libc::c_int;
                (*pdata).median_range_mm = range_mm as i16;
                (*pdata).VL53LX_p_017 = (*pcore).result_core__ranging_total_events_sd1;
                (*pdata).VL53LX_p_010 = (*pcore).result_core__signal_total_events_sd1;
                (*pdata)
                    .total_periods_elapsed = (*pcore)
                    .result_core__total_periods_elapsed_sd1;
                (*pdata).VL53LX_p_016 = (*pcore).result_core__ambient_window_events_sd1;
            }
            _ => {}
        }
        (*pdata).VL53LX_p_026 = (*pdata).VL53LX_p_011;
        (*pdata).VL53LX_p_027 = (*pdata).VL53LX_p_011;
        (*pdata).min_range_mm = (*pdata).median_range_mm;
        (*pdata).max_range_mm = (*pdata).median_range_mm;
        pdata = pdata.offset(1);
        i = i.wrapping_add(1);
    }
    (*presults).device_status = 0 as libc::c_int as VL53LX_DeviceError;
    match (*psys).result__range_status as libc::c_int & 0x1f as libc::c_int {
        1 | 2 | 3 | 13 | 17 => {
            (*presults)
                .device_status = ((*psys).result__range_status as libc::c_int
                & 0x1f as libc::c_int) as u8;
            (*presults)
                .VL53LX_p_003[0 as libc::c_int as usize]
                .range_status = 0 as libc::c_int as VL53LX_DeviceError;
        }
        _ => {}
    };
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_set_zone_dss_config(
    mut Dev: VL53LX_DEV,
    mut pzone_dyn_cfg: *mut VL53LX_zone_private_dyn_cfg_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    let mut pstate: *mut VL53LX_ll_driver_state_t = &mut (*pdev).ll_state;
    if (*pstate).cfg_device_state as libc::c_int
        == 5 as libc::c_int as VL53LX_DeviceState as libc::c_int
    {
        (*pdev).gen_cfg.dss_config__roi_mode_control = 0x2 as libc::c_int as u8;
        (*pdev)
            .gen_cfg
            .dss_config__manual_effective_spads_select = (*pzone_dyn_cfg)
            .dss_requested_effective_spad_count;
    } else {
        (*pdev).gen_cfg.dss_config__roi_mode_control = 0x1 as libc::c_int as u8;
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_set_dmax_mode(
    mut Dev: VL53LX_DEV,
    mut dmax_mode: VL53LX_DeviceDmaxMode,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    (*pdev).dmax_mode = dmax_mode;
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_get_dmax_mode(
    mut Dev: VL53LX_DEV,
    mut pdmax_mode: *mut VL53LX_DeviceDmaxMode,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    *pdmax_mode = (*pdev).dmax_mode;
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_get_dmax_calibration_data(
    mut Dev: VL53LX_DEV,
    mut dmax_mode: VL53LX_DeviceDmaxMode,
    mut pdmax_cal: *mut VL53LX_dmax_calibration_data_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    match dmax_mode as libc::c_int {
        2 => {
            memcpy(
                pdmax_cal as *mut libc::c_void,
                &mut (*pdev).cust_dmax_cal as *mut VL53LX_dmax_calibration_data_t
                    as *const libc::c_void,
                ::std::mem::size_of::<VL53LX_dmax_calibration_data_t>() as libc::c_ulong,
            );
        }
        1 => {
            memcpy(
                pdmax_cal as *mut libc::c_void,
                &mut (*pdev).fmt_dmax_cal as *mut VL53LX_dmax_calibration_data_t
                    as *const libc::c_void,
                ::std::mem::size_of::<VL53LX_dmax_calibration_data_t>() as libc::c_ulong,
            );
        }
        _ => {
            status = -(4 as libc::c_int) as VL53LX_Error;
        }
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_set_offset_correction_mode(
    mut Dev: VL53LX_DEV,
    mut offset_cor_mode: VL53LX_OffsetCorrectionMode,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    (*pdev).offset_correction_mode = offset_cor_mode;
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_get_offset_correction_mode(
    mut Dev: VL53LX_DEV,
    mut poffset_cor_mode: *mut VL53LX_OffsetCorrectionMode,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    *poffset_cor_mode = (*pdev).offset_correction_mode;
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_get_tuning_debug_data(
    mut Dev: VL53LX_DEV,
    mut ptun_data: *mut VL53LX_tuning_parameters_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    let mut pHP: *mut VL53LX_hist_post_process_config_t = &mut (*pdev).histpostprocess;
    let mut pXC: *mut VL53LX_xtalkextract_config_t = &mut (*pdev).xtalk_extract_cfg;
    (*ptun_data).vl53lx_tuningparm_version = (*pdev).tuning_parms.tp_tuning_parm_version;
    (*ptun_data)
        .vl53lx_tuningparm_key_table_version = (*pdev)
        .tuning_parms
        .tp_tuning_parm_key_table_version;
    (*ptun_data)
        .vl53lx_tuningparm_lld_version = (*pdev).tuning_parms.tp_tuning_parm_lld_version;
    (*ptun_data).vl53lx_tuningparm_hist_algo_select = (*pHP).hist_algo_select;
    (*ptun_data).vl53lx_tuningparm_hist_target_order = (*pHP).hist_target_order;
    (*ptun_data).vl53lx_tuningparm_hist_filter_woi_0 = (*pHP).filter_woi0;
    (*ptun_data).vl53lx_tuningparm_hist_filter_woi_1 = (*pHP).filter_woi1;
    (*ptun_data).vl53lx_tuningparm_hist_amb_est_method = (*pHP).hist_amb_est_method;
    (*ptun_data)
        .vl53lx_tuningparm_hist_amb_thresh_sigma_0 = (*pHP).ambient_thresh_sigma0;
    (*ptun_data)
        .vl53lx_tuningparm_hist_amb_thresh_sigma_1 = (*pHP).ambient_thresh_sigma1;
    (*ptun_data)
        .vl53lx_tuningparm_hist_min_amb_thresh_events = (*pHP).min_ambient_thresh_events;
    (*ptun_data)
        .vl53lx_tuningparm_hist_amb_events_scaler = (*pHP).ambient_thresh_events_scaler;
    (*ptun_data).vl53lx_tuningparm_hist_noise_threshold = (*pHP).noise_threshold;
    (*ptun_data)
        .vl53lx_tuningparm_hist_signal_total_events_limit = (*pHP)
        .signal_total_events_limit;
    (*ptun_data)
        .vl53lx_tuningparm_hist_sigma_est_ref_mm = (*pHP).sigma_estimator__sigma_ref_mm;
    (*ptun_data).vl53lx_tuningparm_hist_sigma_thresh_mm = (*pHP).sigma_thresh;
    (*ptun_data)
        .vl53lx_tuningparm_hist_gain_factor = (*pdev)
        .gain_cal
        .histogram_ranging_gain_factor;
    (*ptun_data)
        .vl53lx_tuningparm_consistency_hist_phase_tolerance = (*pHP)
        .algo__consistency_check__phase_tolerance;
    (*ptun_data)
        .vl53lx_tuningparm_consistency_hist_min_max_tolerance_mm = (*pHP)
        .algo__consistency_check__min_max_tolerance;
    (*ptun_data)
        .vl53lx_tuningparm_consistency_hist_event_sigma = (*pHP)
        .algo__consistency_check__event_sigma;
    (*ptun_data)
        .vl53lx_tuningparm_consistency_hist_event_sigma_min_spad_limit = (*pHP)
        .algo__consistency_check__event_min_spad_count;
    (*ptun_data)
        .vl53lx_tuningparm_initial_phase_rtn_histo_long_range = (*pdev)
        .tuning_parms
        .tp_init_phase_rtn_hist_long;
    (*ptun_data)
        .vl53lx_tuningparm_initial_phase_rtn_histo_med_range = (*pdev)
        .tuning_parms
        .tp_init_phase_rtn_hist_med;
    (*ptun_data)
        .vl53lx_tuningparm_initial_phase_rtn_histo_short_range = (*pdev)
        .tuning_parms
        .tp_init_phase_rtn_hist_short;
    (*ptun_data)
        .vl53lx_tuningparm_initial_phase_ref_histo_long_range = (*pdev)
        .tuning_parms
        .tp_init_phase_ref_hist_long;
    (*ptun_data)
        .vl53lx_tuningparm_initial_phase_ref_histo_med_range = (*pdev)
        .tuning_parms
        .tp_init_phase_ref_hist_med;
    (*ptun_data)
        .vl53lx_tuningparm_initial_phase_ref_histo_short_range = (*pdev)
        .tuning_parms
        .tp_init_phase_ref_hist_short;
    (*ptun_data)
        .vl53lx_tuningparm_xtalk_detect_min_valid_range_mm = (*pdev)
        .xtalk_cfg
        .algo__crosstalk_detect_min_valid_range_mm;
    (*ptun_data)
        .vl53lx_tuningparm_xtalk_detect_max_valid_range_mm = (*pdev)
        .xtalk_cfg
        .algo__crosstalk_detect_max_valid_range_mm;
    (*ptun_data)
        .vl53lx_tuningparm_xtalk_detect_max_sigma_mm = (*pdev)
        .xtalk_cfg
        .algo__crosstalk_detect_max_sigma_mm;
    (*ptun_data)
        .vl53lx_tuningparm_xtalk_detect_min_max_tolerance = (*pHP)
        .algo__crosstalk_detect_min_max_tolerance;
    (*ptun_data)
        .vl53lx_tuningparm_xtalk_detect_max_valid_rate_kcps = (*pdev)
        .xtalk_cfg
        .algo__crosstalk_detect_max_valid_rate_kcps;
    (*ptun_data)
        .vl53lx_tuningparm_xtalk_detect_event_sigma = (*pHP)
        .algo__crosstalk_detect_event_sigma;
    (*ptun_data)
        .vl53lx_tuningparm_hist_xtalk_margin_kcps = (*pdev)
        .xtalk_cfg
        .histogram_mode_crosstalk_margin_kcps;
    (*ptun_data)
        .vl53lx_tuningparm_consistency_lite_phase_tolerance = (*pdev)
        .tuning_parms
        .tp_consistency_lite_phase_tolerance;
    (*ptun_data)
        .vl53lx_tuningparm_phasecal_target = (*pdev).tuning_parms.tp_phasecal_target;
    (*ptun_data)
        .vl53lx_tuningparm_lite_cal_repeat_rate = (*pdev)
        .tuning_parms
        .tp_cal_repeat_rate;
    (*ptun_data)
        .vl53lx_tuningparm_lite_ranging_gain_factor = (*pdev)
        .gain_cal
        .standard_ranging_gain_factor;
    (*ptun_data)
        .vl53lx_tuningparm_lite_min_clip_mm = (*pdev).tuning_parms.tp_lite_min_clip;
    (*ptun_data)
        .vl53lx_tuningparm_lite_long_sigma_thresh_mm = (*pdev)
        .tuning_parms
        .tp_lite_long_sigma_thresh_mm;
    (*ptun_data)
        .vl53lx_tuningparm_lite_med_sigma_thresh_mm = (*pdev)
        .tuning_parms
        .tp_lite_med_sigma_thresh_mm;
    (*ptun_data)
        .vl53lx_tuningparm_lite_short_sigma_thresh_mm = (*pdev)
        .tuning_parms
        .tp_lite_short_sigma_thresh_mm;
    (*ptun_data)
        .vl53lx_tuningparm_lite_long_min_count_rate_rtn_mcps = (*pdev)
        .tuning_parms
        .tp_lite_long_min_count_rate_rtn_mcps;
    (*ptun_data)
        .vl53lx_tuningparm_lite_med_min_count_rate_rtn_mcps = (*pdev)
        .tuning_parms
        .tp_lite_med_min_count_rate_rtn_mcps;
    (*ptun_data)
        .vl53lx_tuningparm_lite_short_min_count_rate_rtn_mcps = (*pdev)
        .tuning_parms
        .tp_lite_short_min_count_rate_rtn_mcps;
    (*ptun_data)
        .vl53lx_tuningparm_lite_sigma_est_pulse_width = (*pdev)
        .tuning_parms
        .tp_lite_sigma_est_pulse_width_ns;
    (*ptun_data)
        .vl53lx_tuningparm_lite_sigma_est_amb_width_ns = (*pdev)
        .tuning_parms
        .tp_lite_sigma_est_amb_width_ns;
    (*ptun_data)
        .vl53lx_tuningparm_lite_sigma_ref_mm = (*pdev).tuning_parms.tp_lite_sigma_ref_mm;
    (*ptun_data)
        .vl53lx_tuningparm_lite_rit_mult = (*pdev)
        .xtalk_cfg
        .crosstalk_range_ignore_threshold_mult;
    (*ptun_data)
        .vl53lx_tuningparm_lite_seed_config = (*pdev).tuning_parms.tp_lite_seed_cfg;
    (*ptun_data)
        .vl53lx_tuningparm_lite_quantifier = (*pdev).tuning_parms.tp_lite_quantifier;
    (*ptun_data)
        .vl53lx_tuningparm_lite_first_order_select = (*pdev)
        .tuning_parms
        .tp_lite_first_order_select;
    (*ptun_data)
        .vl53lx_tuningparm_lite_xtalk_margin_kcps = (*pdev)
        .xtalk_cfg
        .lite_mode_crosstalk_margin_kcps;
    (*ptun_data)
        .vl53lx_tuningparm_initial_phase_rtn_lite_long_range = (*pdev)
        .tuning_parms
        .tp_init_phase_rtn_lite_long;
    (*ptun_data)
        .vl53lx_tuningparm_initial_phase_rtn_lite_med_range = (*pdev)
        .tuning_parms
        .tp_init_phase_rtn_lite_med;
    (*ptun_data)
        .vl53lx_tuningparm_initial_phase_rtn_lite_short_range = (*pdev)
        .tuning_parms
        .tp_init_phase_rtn_lite_short;
    (*ptun_data)
        .vl53lx_tuningparm_initial_phase_ref_lite_long_range = (*pdev)
        .tuning_parms
        .tp_init_phase_ref_lite_long;
    (*ptun_data)
        .vl53lx_tuningparm_initial_phase_ref_lite_med_range = (*pdev)
        .tuning_parms
        .tp_init_phase_ref_lite_med;
    (*ptun_data)
        .vl53lx_tuningparm_initial_phase_ref_lite_short_range = (*pdev)
        .tuning_parms
        .tp_init_phase_ref_lite_short;
    (*ptun_data)
        .vl53lx_tuningparm_timed_seed_config = (*pdev).tuning_parms.tp_timed_seed_cfg;
    (*ptun_data)
        .vl53lx_tuningparm_dmax_cfg_signal_thresh_sigma = (*pdev)
        .dmax_cfg
        .signal_thresh_sigma;
    (*ptun_data)
        .vl53lx_tuningparm_dmax_cfg_reflectance_array_0 = (*pdev)
        .dmax_cfg
        .target_reflectance_for_dmax_calc[0 as libc::c_int as usize];
    (*ptun_data)
        .vl53lx_tuningparm_dmax_cfg_reflectance_array_1 = (*pdev)
        .dmax_cfg
        .target_reflectance_for_dmax_calc[1 as libc::c_int as usize];
    (*ptun_data)
        .vl53lx_tuningparm_dmax_cfg_reflectance_array_2 = (*pdev)
        .dmax_cfg
        .target_reflectance_for_dmax_calc[2 as libc::c_int as usize];
    (*ptun_data)
        .vl53lx_tuningparm_dmax_cfg_reflectance_array_3 = (*pdev)
        .dmax_cfg
        .target_reflectance_for_dmax_calc[3 as libc::c_int as usize];
    (*ptun_data)
        .vl53lx_tuningparm_dmax_cfg_reflectance_array_4 = (*pdev)
        .dmax_cfg
        .target_reflectance_for_dmax_calc[4 as libc::c_int as usize];
    (*ptun_data)
        .vl53lx_tuningparm_vhv_loopbound = (*pdev)
        .stat_nvm
        .vhv_config__timeout_macrop_loop_bound;
    (*ptun_data)
        .vl53lx_tuningparm_refspadchar_device_test_mode = (*pdev)
        .refspadchar
        .device_test_mode;
    (*ptun_data)
        .vl53lx_tuningparm_refspadchar_vcsel_period = (*pdev).refspadchar.VL53LX_p_005;
    (*ptun_data)
        .vl53lx_tuningparm_refspadchar_phasecal_timeout_us = (*pdev)
        .refspadchar
        .timeout_us;
    (*ptun_data)
        .vl53lx_tuningparm_refspadchar_target_count_rate_mcps = (*pdev)
        .refspadchar
        .target_count_rate_mcps;
    (*ptun_data)
        .vl53lx_tuningparm_refspadchar_min_countrate_limit_mcps = (*pdev)
        .refspadchar
        .min_count_rate_limit_mcps;
    (*ptun_data)
        .vl53lx_tuningparm_refspadchar_max_countrate_limit_mcps = (*pdev)
        .refspadchar
        .max_count_rate_limit_mcps;
    (*ptun_data).vl53lx_tuningparm_xtalk_extract_num_of_samples = (*pXC).num_of_samples;
    (*ptun_data)
        .vl53lx_tuningparm_xtalk_extract_min_filter_thresh_mm = (*pXC)
        .algo__crosstalk_extract_min_valid_range_mm;
    (*ptun_data)
        .vl53lx_tuningparm_xtalk_extract_max_filter_thresh_mm = (*pXC)
        .algo__crosstalk_extract_max_valid_range_mm;
    (*ptun_data)
        .vl53lx_tuningparm_xtalk_extract_dss_rate_mcps = (*pXC)
        .dss_config__target_total_rate_mcps;
    (*ptun_data)
        .vl53lx_tuningparm_xtalk_extract_phasecal_timeout_us = (*pXC)
        .phasecal_config_timeout_us;
    (*ptun_data)
        .vl53lx_tuningparm_xtalk_extract_max_valid_rate_kcps = (*pXC)
        .algo__crosstalk_extract_max_valid_rate_kcps;
    (*ptun_data)
        .vl53lx_tuningparm_xtalk_extract_sigma_threshold_mm = (*pXC)
        .algo__crosstalk_extract_max_sigma_mm;
    (*ptun_data)
        .vl53lx_tuningparm_xtalk_extract_dss_timeout_us = (*pXC).mm_config_timeout_us;
    (*ptun_data)
        .vl53lx_tuningparm_xtalk_extract_bin_timeout_us = (*pXC).range_config_timeout_us;
    (*ptun_data)
        .vl53lx_tuningparm_offset_cal_dss_rate_mcps = (*pdev)
        .offsetcal_cfg
        .dss_config__target_total_rate_mcps;
    (*ptun_data)
        .vl53lx_tuningparm_offset_cal_phasecal_timeout_us = (*pdev)
        .offsetcal_cfg
        .phasecal_config_timeout_us;
    (*ptun_data)
        .vl53lx_tuningparm_offset_cal_mm_timeout_us = (*pdev)
        .offsetcal_cfg
        .mm_config_timeout_us;
    (*ptun_data)
        .vl53lx_tuningparm_offset_cal_range_timeout_us = (*pdev)
        .offsetcal_cfg
        .range_config_timeout_us;
    (*ptun_data)
        .vl53lx_tuningparm_offset_cal_pre_samples = (*pdev)
        .offsetcal_cfg
        .pre_num_of_samples;
    (*ptun_data)
        .vl53lx_tuningparm_offset_cal_mm1_samples = (*pdev)
        .offsetcal_cfg
        .mm1_num_of_samples;
    (*ptun_data)
        .vl53lx_tuningparm_offset_cal_mm2_samples = (*pdev)
        .offsetcal_cfg
        .mm2_num_of_samples;
    (*ptun_data)
        .vl53lx_tuningparm_zone_cal_dss_rate_mcps = (*pdev)
        .zonecal_cfg
        .dss_config__target_total_rate_mcps;
    (*ptun_data)
        .vl53lx_tuningparm_zone_cal_phasecal_timeout_us = (*pdev)
        .zonecal_cfg
        .phasecal_config_timeout_us;
    (*ptun_data)
        .vl53lx_tuningparm_zone_cal_dss_timeout_us = (*pdev)
        .zonecal_cfg
        .mm_config_timeout_us;
    (*ptun_data)
        .vl53lx_tuningparm_zone_cal_phasecal_num_samples = (*pdev)
        .zonecal_cfg
        .phasecal_num_of_samples;
    (*ptun_data)
        .vl53lx_tuningparm_zone_cal_range_timeout_us = (*pdev)
        .zonecal_cfg
        .range_config_timeout_us;
    (*ptun_data)
        .vl53lx_tuningparm_zone_cal_zone_num_samples = (*pdev)
        .zonecal_cfg
        .zone_num_of_samples;
    (*ptun_data).vl53lx_tuningparm_spadmap_vcsel_period = (*pdev).ssc_cfg.VL53LX_p_005;
    (*ptun_data).vl53lx_tuningparm_spadmap_vcsel_start = (*pdev).ssc_cfg.vcsel_start;
    (*ptun_data)
        .vl53lx_tuningparm_spadmap_rate_limit_mcps = (*pdev).ssc_cfg.rate_limit_mcps;
    (*ptun_data)
        .vl53lx_tuningparm_lite_dss_config_target_total_rate_mcps = (*pdev)
        .tuning_parms
        .tp_dss_target_lite_mcps;
    (*ptun_data)
        .vl53lx_tuningparm_ranging_dss_config_target_total_rate_mcps = (*pdev)
        .tuning_parms
        .tp_dss_target_histo_mcps;
    (*ptun_data)
        .vl53lx_tuningparm_mz_dss_config_target_total_rate_mcps = (*pdev)
        .tuning_parms
        .tp_dss_target_histo_mz_mcps;
    (*ptun_data)
        .vl53lx_tuningparm_timed_dss_config_target_total_rate_mcps = (*pdev)
        .tuning_parms
        .tp_dss_target_timed_mcps;
    (*ptun_data)
        .vl53lx_tuningparm_lite_phasecal_config_timeout_us = (*pdev)
        .tuning_parms
        .tp_phasecal_timeout_lite_us;
    (*ptun_data)
        .vl53lx_tuningparm_ranging_long_phasecal_config_timeout_us = (*pdev)
        .tuning_parms
        .tp_phasecal_timeout_hist_long_us;
    (*ptun_data)
        .vl53lx_tuningparm_ranging_med_phasecal_config_timeout_us = (*pdev)
        .tuning_parms
        .tp_phasecal_timeout_hist_med_us;
    (*ptun_data)
        .vl53lx_tuningparm_ranging_short_phasecal_config_timeout_us = (*pdev)
        .tuning_parms
        .tp_phasecal_timeout_hist_short_us;
    (*ptun_data)
        .vl53lx_tuningparm_mz_long_phasecal_config_timeout_us = (*pdev)
        .tuning_parms
        .tp_phasecal_timeout_mz_long_us;
    (*ptun_data)
        .vl53lx_tuningparm_mz_med_phasecal_config_timeout_us = (*pdev)
        .tuning_parms
        .tp_phasecal_timeout_mz_med_us;
    (*ptun_data)
        .vl53lx_tuningparm_mz_short_phasecal_config_timeout_us = (*pdev)
        .tuning_parms
        .tp_phasecal_timeout_mz_short_us;
    (*ptun_data)
        .vl53lx_tuningparm_timed_phasecal_config_timeout_us = (*pdev)
        .tuning_parms
        .tp_phasecal_timeout_timed_us;
    (*ptun_data)
        .vl53lx_tuningparm_lite_mm_config_timeout_us = (*pdev)
        .tuning_parms
        .tp_mm_timeout_lite_us;
    (*ptun_data)
        .vl53lx_tuningparm_ranging_mm_config_timeout_us = (*pdev)
        .tuning_parms
        .tp_mm_timeout_histo_us;
    (*ptun_data)
        .vl53lx_tuningparm_mz_mm_config_timeout_us = (*pdev)
        .tuning_parms
        .tp_mm_timeout_mz_us;
    (*ptun_data)
        .vl53lx_tuningparm_timed_mm_config_timeout_us = (*pdev)
        .tuning_parms
        .tp_mm_timeout_timed_us;
    (*ptun_data)
        .vl53lx_tuningparm_lite_range_config_timeout_us = (*pdev)
        .tuning_parms
        .tp_range_timeout_lite_us;
    (*ptun_data)
        .vl53lx_tuningparm_ranging_range_config_timeout_us = (*pdev)
        .tuning_parms
        .tp_range_timeout_histo_us;
    (*ptun_data)
        .vl53lx_tuningparm_mz_range_config_timeout_us = (*pdev)
        .tuning_parms
        .tp_range_timeout_mz_us;
    (*ptun_data)
        .vl53lx_tuningparm_timed_range_config_timeout_us = (*pdev)
        .tuning_parms
        .tp_range_timeout_timed_us;
    (*ptun_data)
        .vl53lx_tuningparm_dynxtalk_smudge_margin = (*pdev)
        .smudge_correct_config
        .smudge_margin;
    (*ptun_data)
        .vl53lx_tuningparm_dynxtalk_noise_margin = (*pdev)
        .smudge_correct_config
        .noise_margin;
    (*ptun_data)
        .vl53lx_tuningparm_dynxtalk_xtalk_offset_limit = (*pdev)
        .smudge_correct_config
        .user_xtalk_offset_limit;
    (*ptun_data)
        .vl53lx_tuningparm_dynxtalk_xtalk_offset_limit_hi = (*pdev)
        .smudge_correct_config
        .user_xtalk_offset_limit_hi;
    (*ptun_data)
        .vl53lx_tuningparm_dynxtalk_sample_limit = (*pdev)
        .smudge_correct_config
        .sample_limit;
    (*ptun_data)
        .vl53lx_tuningparm_dynxtalk_single_xtalk_delta = (*pdev)
        .smudge_correct_config
        .single_xtalk_delta;
    (*ptun_data)
        .vl53lx_tuningparm_dynxtalk_averaged_xtalk_delta = (*pdev)
        .smudge_correct_config
        .averaged_xtalk_delta;
    (*ptun_data)
        .vl53lx_tuningparm_dynxtalk_clip_limit = (*pdev)
        .smudge_correct_config
        .smudge_corr_clip_limit;
    (*ptun_data)
        .vl53lx_tuningparm_dynxtalk_scaler_calc_method = (*pdev)
        .smudge_correct_config
        .scaler_calc_method;
    (*ptun_data)
        .vl53lx_tuningparm_dynxtalk_xgradient_scaler = (*pdev)
        .smudge_correct_config
        .x_gradient_scaler;
    (*ptun_data)
        .vl53lx_tuningparm_dynxtalk_ygradient_scaler = (*pdev)
        .smudge_correct_config
        .y_gradient_scaler;
    (*ptun_data)
        .vl53lx_tuningparm_dynxtalk_user_scaler_set = (*pdev)
        .smudge_correct_config
        .user_scaler_set;
    (*ptun_data)
        .vl53lx_tuningparm_dynxtalk_smudge_cor_single_apply = (*pdev)
        .smudge_correct_config
        .smudge_corr_single_apply;
    (*ptun_data)
        .vl53lx_tuningparm_dynxtalk_xtalk_amb_threshold = (*pdev)
        .smudge_correct_config
        .smudge_corr_ambient_threshold;
    (*ptun_data)
        .vl53lx_tuningparm_dynxtalk_nodetect_amb_threshold_kcps = (*pdev)
        .smudge_correct_config
        .nodetect_ambient_threshold;
    (*ptun_data)
        .vl53lx_tuningparm_dynxtalk_nodetect_sample_limit = (*pdev)
        .smudge_correct_config
        .nodetect_sample_limit;
    (*ptun_data)
        .vl53lx_tuningparm_dynxtalk_nodetect_xtalk_offset_kcps = (*pdev)
        .smudge_correct_config
        .nodetect_xtalk_offset;
    (*ptun_data)
        .vl53lx_tuningparm_dynxtalk_nodetect_min_range_mm = (*pdev)
        .smudge_correct_config
        .nodetect_min_range_mm;
    (*ptun_data)
        .vl53lx_tuningparm_lowpowerauto_vhv_loop_bound = (*pdev)
        .low_power_auto_data
        .vhv_loop_bound;
    (*ptun_data)
        .vl53lx_tuningparm_lowpowerauto_mm_config_timeout_us = (*pdev)
        .tuning_parms
        .tp_mm_timeout_lpa_us;
    (*ptun_data)
        .vl53lx_tuningparm_lowpowerauto_range_config_timeout_us = (*pdev)
        .tuning_parms
        .tp_range_timeout_lpa_us;
    (*ptun_data)
        .vl53lx_tuningparm_very_short_dss_rate_mcps = (*pdev)
        .tuning_parms
        .tp_dss_target_very_short_mcps;
    (*ptun_data)
        .vl53lx_tuningparm_phasecal_patch_power = (*pdev)
        .tuning_parms
        .tp_phasecal_patch_power;
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_get_tuning_parm(
    mut Dev: VL53LX_DEV,
    mut tuning_parm_key: VL53LX_TuningParms,
    mut ptuning_parm_value: *mut i32,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    let mut pHP: *mut VL53LX_hist_post_process_config_t = &mut (*pdev).histpostprocess;
    let mut pXC: *mut VL53LX_xtalkextract_config_t = &mut (*pdev).xtalk_extract_cfg;
    match tuning_parm_key as libc::c_int {
        32768 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_tuning_parm_version as i32;
        }
        32769 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_tuning_parm_key_table_version
                as i32;
        }
        32770 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_tuning_parm_lld_version
                as i32;
        }
        32771 => {
            *ptuning_parm_value = (*pHP).hist_algo_select as i32;
        }
        32772 => {
            *ptuning_parm_value = (*pHP).hist_target_order as i32;
        }
        32773 => {
            *ptuning_parm_value = (*pHP).filter_woi0 as i32;
        }
        32774 => {
            *ptuning_parm_value = (*pHP).filter_woi1 as i32;
        }
        32775 => {
            *ptuning_parm_value = (*pHP).hist_amb_est_method as i32;
        }
        32776 => {
            *ptuning_parm_value = (*pHP).ambient_thresh_sigma0 as i32;
        }
        32777 => {
            *ptuning_parm_value = (*pHP).ambient_thresh_sigma1 as i32;
        }
        32778 => {
            *ptuning_parm_value = (*pHP).min_ambient_thresh_events;
        }
        32779 => {
            *ptuning_parm_value = (*pHP).ambient_thresh_events_scaler as i32;
        }
        32780 => {
            *ptuning_parm_value = (*pHP).noise_threshold as i32;
        }
        32781 => {
            *ptuning_parm_value = (*pHP).signal_total_events_limit;
        }
        32782 => {
            *ptuning_parm_value = (*pHP).sigma_estimator__sigma_ref_mm as i32;
        }
        32783 => {
            *ptuning_parm_value = (*pHP).sigma_thresh as i32;
        }
        32784 => {
            *ptuning_parm_value = (*pdev).gain_cal.histogram_ranging_gain_factor
                as i32;
        }
        32785 => {
            *ptuning_parm_value = (*pHP).algo__consistency_check__phase_tolerance
                as i32;
        }
        32786 => {
            *ptuning_parm_value = (*pHP).algo__consistency_check__min_max_tolerance
                as i32;
        }
        32787 => {
            *ptuning_parm_value = (*pHP).algo__consistency_check__event_sigma as i32;
        }
        32788 => {
            *ptuning_parm_value = (*pHP).algo__consistency_check__event_min_spad_count
                as i32;
        }
        32789 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_init_phase_rtn_hist_long
                as i32;
        }
        32790 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_init_phase_rtn_hist_med
                as i32;
        }
        32791 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_init_phase_rtn_hist_short
                as i32;
        }
        32792 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_init_phase_ref_hist_long
                as i32;
        }
        32793 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_init_phase_ref_hist_med
                as i32;
        }
        32794 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_init_phase_ref_hist_short
                as i32;
        }
        32795 => {
            *ptuning_parm_value = (*pdev)
                .xtalk_cfg
                .algo__crosstalk_detect_min_valid_range_mm as i32;
        }
        32796 => {
            *ptuning_parm_value = (*pdev)
                .xtalk_cfg
                .algo__crosstalk_detect_max_valid_range_mm as i32;
        }
        32797 => {
            *ptuning_parm_value = (*pdev).xtalk_cfg.algo__crosstalk_detect_max_sigma_mm
                as i32;
        }
        32798 => {
            *ptuning_parm_value = (*pHP).algo__crosstalk_detect_min_max_tolerance
                as i32;
        }
        32799 => {
            *ptuning_parm_value = (*pdev)
                .xtalk_cfg
                .algo__crosstalk_detect_max_valid_rate_kcps as i32;
        }
        32800 => {
            *ptuning_parm_value = (*pHP).algo__crosstalk_detect_event_sigma as i32;
        }
        32801 => {
            *ptuning_parm_value = (*pdev).xtalk_cfg.histogram_mode_crosstalk_margin_kcps
                as i32;
        }
        32802 => {
            *ptuning_parm_value = (*pdev)
                .tuning_parms
                .tp_consistency_lite_phase_tolerance as i32;
        }
        32803 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_phasecal_target as i32;
        }
        32804 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_cal_repeat_rate as i32;
        }
        32805 => {
            *ptuning_parm_value = (*pdev).gain_cal.standard_ranging_gain_factor
                as i32;
        }
        32806 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_lite_min_clip as i32;
        }
        32807 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_lite_long_sigma_thresh_mm
                as i32;
        }
        32808 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_lite_med_sigma_thresh_mm
                as i32;
        }
        32809 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_lite_short_sigma_thresh_mm
                as i32;
        }
        32810 => {
            *ptuning_parm_value = (*pdev)
                .tuning_parms
                .tp_lite_long_min_count_rate_rtn_mcps as i32;
        }
        32811 => {
            *ptuning_parm_value = (*pdev)
                .tuning_parms
                .tp_lite_med_min_count_rate_rtn_mcps as i32;
        }
        32812 => {
            *ptuning_parm_value = (*pdev)
                .tuning_parms
                .tp_lite_short_min_count_rate_rtn_mcps as i32;
        }
        32813 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_lite_sigma_est_pulse_width_ns
                as i32;
        }
        32814 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_lite_sigma_est_amb_width_ns
                as i32;
        }
        32815 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_lite_sigma_ref_mm as i32;
        }
        32816 => {
            *ptuning_parm_value = (*pdev).xtalk_cfg.crosstalk_range_ignore_threshold_mult
                as i32;
        }
        32817 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_lite_seed_cfg as i32;
        }
        32818 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_lite_quantifier as i32;
        }
        32819 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_lite_first_order_select
                as i32;
        }
        32820 => {
            *ptuning_parm_value = (*pdev).xtalk_cfg.lite_mode_crosstalk_margin_kcps
                as i32;
        }
        32821 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_init_phase_rtn_lite_long
                as i32;
        }
        32822 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_init_phase_rtn_lite_med
                as i32;
        }
        32823 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_init_phase_rtn_lite_short
                as i32;
        }
        32824 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_init_phase_ref_lite_long
                as i32;
        }
        32825 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_init_phase_ref_lite_med
                as i32;
        }
        32826 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_init_phase_ref_lite_short
                as i32;
        }
        32827 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_timed_seed_cfg as i32;
        }
        32828 => {
            *ptuning_parm_value = (*pdev).dmax_cfg.signal_thresh_sigma as i32;
        }
        32829 => {
            *ptuning_parm_value = (*pdev)
                .dmax_cfg
                .target_reflectance_for_dmax_calc[0 as libc::c_int as usize] as i32;
        }
        32830 => {
            *ptuning_parm_value = (*pdev)
                .dmax_cfg
                .target_reflectance_for_dmax_calc[1 as libc::c_int as usize] as i32;
        }
        32831 => {
            *ptuning_parm_value = (*pdev)
                .dmax_cfg
                .target_reflectance_for_dmax_calc[2 as libc::c_int as usize] as i32;
        }
        32832 => {
            *ptuning_parm_value = (*pdev)
                .dmax_cfg
                .target_reflectance_for_dmax_calc[3 as libc::c_int as usize] as i32;
        }
        32833 => {
            *ptuning_parm_value = (*pdev)
                .dmax_cfg
                .target_reflectance_for_dmax_calc[4 as libc::c_int as usize] as i32;
        }
        32834 => {
            *ptuning_parm_value = (*pdev).stat_nvm.vhv_config__timeout_macrop_loop_bound
                as i32;
        }
        32835 => {
            *ptuning_parm_value = (*pdev).refspadchar.device_test_mode as i32;
        }
        32836 => {
            *ptuning_parm_value = (*pdev).refspadchar.VL53LX_p_005 as i32;
        }
        32837 => {
            *ptuning_parm_value = (*pdev).refspadchar.timeout_us as i32;
        }
        32838 => {
            *ptuning_parm_value = (*pdev).refspadchar.target_count_rate_mcps as i32;
        }
        32839 => {
            *ptuning_parm_value = (*pdev).refspadchar.min_count_rate_limit_mcps
                as i32;
        }
        32840 => {
            *ptuning_parm_value = (*pdev).refspadchar.max_count_rate_limit_mcps
                as i32;
        }
        32841 => {
            *ptuning_parm_value = (*pXC).num_of_samples as i32;
        }
        32842 => {
            *ptuning_parm_value = (*pXC).algo__crosstalk_extract_min_valid_range_mm
                as i32;
        }
        32843 => {
            *ptuning_parm_value = (*pXC).algo__crosstalk_extract_max_valid_range_mm
                as i32;
        }
        32844 => {
            *ptuning_parm_value = (*pXC).dss_config__target_total_rate_mcps as i32;
        }
        32845 => {
            *ptuning_parm_value = (*pXC).phasecal_config_timeout_us as i32;
        }
        32846 => {
            *ptuning_parm_value = (*pXC).algo__crosstalk_extract_max_valid_rate_kcps
                as i32;
        }
        32847 => {
            *ptuning_parm_value = (*pXC).algo__crosstalk_extract_max_sigma_mm as i32;
        }
        32848 => {
            *ptuning_parm_value = (*pXC).mm_config_timeout_us as i32;
        }
        32849 => {
            *ptuning_parm_value = (*pXC).range_config_timeout_us as i32;
        }
        32850 => {
            *ptuning_parm_value = (*pdev)
                .offsetcal_cfg
                .dss_config__target_total_rate_mcps as i32;
        }
        32851 => {
            *ptuning_parm_value = (*pdev).offsetcal_cfg.phasecal_config_timeout_us
                as i32;
        }
        32852 => {
            *ptuning_parm_value = (*pdev).offsetcal_cfg.mm_config_timeout_us as i32;
        }
        32853 => {
            *ptuning_parm_value = (*pdev).offsetcal_cfg.range_config_timeout_us
                as i32;
        }
        32854 => {
            *ptuning_parm_value = (*pdev).offsetcal_cfg.pre_num_of_samples as i32;
        }
        32855 => {
            *ptuning_parm_value = (*pdev).offsetcal_cfg.mm1_num_of_samples as i32;
        }
        32856 => {
            *ptuning_parm_value = (*pdev).offsetcal_cfg.mm2_num_of_samples as i32;
        }
        32857 => {
            *ptuning_parm_value = (*pdev).zonecal_cfg.dss_config__target_total_rate_mcps
                as i32;
        }
        32858 => {
            *ptuning_parm_value = (*pdev).zonecal_cfg.phasecal_config_timeout_us
                as i32;
        }
        32859 => {
            *ptuning_parm_value = (*pdev).zonecal_cfg.mm_config_timeout_us as i32;
        }
        32860 => {
            *ptuning_parm_value = (*pdev).zonecal_cfg.phasecal_num_of_samples as i32;
        }
        32861 => {
            *ptuning_parm_value = (*pdev).zonecal_cfg.range_config_timeout_us as i32;
        }
        32862 => {
            *ptuning_parm_value = (*pdev).zonecal_cfg.zone_num_of_samples as i32;
        }
        32863 => {
            *ptuning_parm_value = (*pdev).ssc_cfg.VL53LX_p_005 as i32;
        }
        32864 => {
            *ptuning_parm_value = (*pdev).ssc_cfg.vcsel_start as i32;
        }
        32865 => {
            *ptuning_parm_value = (*pdev).ssc_cfg.rate_limit_mcps as i32;
        }
        32866 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_dss_target_lite_mcps
                as i32;
        }
        32867 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_dss_target_histo_mcps
                as i32;
        }
        32868 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_dss_target_histo_mz_mcps
                as i32;
        }
        32869 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_dss_target_timed_mcps
                as i32;
        }
        32870 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_phasecal_timeout_lite_us
                as i32;
        }
        32871 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_phasecal_timeout_hist_long_us
                as i32;
        }
        32872 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_phasecal_timeout_hist_med_us
                as i32;
        }
        32873 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_phasecal_timeout_hist_short_us
                as i32;
        }
        32874 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_phasecal_timeout_mz_long_us
                as i32;
        }
        32875 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_phasecal_timeout_mz_med_us
                as i32;
        }
        32876 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_phasecal_timeout_mz_short_us
                as i32;
        }
        32877 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_phasecal_timeout_timed_us
                as i32;
        }
        32878 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_mm_timeout_lite_us as i32;
        }
        32879 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_mm_timeout_histo_us as i32;
        }
        32880 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_mm_timeout_mz_us as i32;
        }
        32881 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_mm_timeout_timed_us as i32;
        }
        32882 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_range_timeout_lite_us
                as i32;
        }
        32883 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_range_timeout_histo_us
                as i32;
        }
        32884 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_range_timeout_mz_us as i32;
        }
        32885 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_range_timeout_timed_us
                as i32;
        }
        32886 => {
            *ptuning_parm_value = (*pdev).smudge_correct_config.smudge_margin as i32;
        }
        32887 => {
            *ptuning_parm_value = (*pdev).smudge_correct_config.noise_margin as i32;
        }
        32888 => {
            *ptuning_parm_value = (*pdev).smudge_correct_config.user_xtalk_offset_limit
                as i32;
        }
        32889 => {
            *ptuning_parm_value = (*pdev)
                .smudge_correct_config
                .user_xtalk_offset_limit_hi as i32;
        }
        32890 => {
            *ptuning_parm_value = (*pdev).smudge_correct_config.sample_limit as i32;
        }
        32891 => {
            *ptuning_parm_value = (*pdev).smudge_correct_config.single_xtalk_delta
                as i32;
        }
        32892 => {
            *ptuning_parm_value = (*pdev).smudge_correct_config.averaged_xtalk_delta
                as i32;
        }
        32893 => {
            *ptuning_parm_value = (*pdev).smudge_correct_config.smudge_corr_clip_limit
                as i32;
        }
        32894 => {
            *ptuning_parm_value = (*pdev).smudge_correct_config.scaler_calc_method
                as i32;
        }
        32895 => {
            *ptuning_parm_value = (*pdev).smudge_correct_config.x_gradient_scaler
                as i32;
        }
        32896 => {
            *ptuning_parm_value = (*pdev).smudge_correct_config.y_gradient_scaler
                as i32;
        }
        32897 => {
            *ptuning_parm_value = (*pdev).smudge_correct_config.user_scaler_set
                as i32;
        }
        32898 => {
            *ptuning_parm_value = (*pdev).smudge_correct_config.smudge_corr_single_apply
                as i32;
        }
        32899 => {
            *ptuning_parm_value = (*pdev)
                .smudge_correct_config
                .smudge_corr_ambient_threshold as i32;
        }
        32900 => {
            *ptuning_parm_value = (*pdev)
                .smudge_correct_config
                .nodetect_ambient_threshold as i32;
        }
        32901 => {
            *ptuning_parm_value = (*pdev).smudge_correct_config.nodetect_sample_limit
                as i32;
        }
        32902 => {
            *ptuning_parm_value = (*pdev).smudge_correct_config.nodetect_xtalk_offset
                as i32;
        }
        32903 => {
            *ptuning_parm_value = (*pdev).smudge_correct_config.nodetect_min_range_mm
                as i32;
        }
        32904 => {
            *ptuning_parm_value = (*pdev).low_power_auto_data.vhv_loop_bound as i32;
        }
        32905 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_mm_timeout_lpa_us as i32;
        }
        32906 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_range_timeout_lpa_us
                as i32;
        }
        32907 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_dss_target_very_short_mcps
                as i32;
        }
        32908 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_phasecal_patch_power
                as i32;
        }
        32909 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_hist_merge as i32;
        }
        32910 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_reset_merge_threshold
                as i32;
        }
        32911 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_hist_merge_max_size as i32;
        }
        32912 => {
            *ptuning_parm_value = (*pdev).smudge_correct_config.max_smudge_factor
                as i32;
        }
        32913 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_uwr_enable as i32;
        }
        32914 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_uwr_med_z_1_min as i32;
        }
        32915 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_uwr_med_z_1_max as i32;
        }
        32916 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_uwr_med_z_2_min as i32;
        }
        32917 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_uwr_med_z_2_max as i32;
        }
        32918 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_uwr_med_z_3_min as i32;
        }
        32919 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_uwr_med_z_3_max as i32;
        }
        32920 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_uwr_med_z_4_min as i32;
        }
        32921 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_uwr_med_z_4_max as i32;
        }
        32922 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_uwr_med_z_5_min as i32;
        }
        32923 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_uwr_med_z_5_max as i32;
        }
        32924 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_uwr_med_corr_z_1_rangea
                as i32;
        }
        32925 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_uwr_med_corr_z_1_rangeb
                as i32;
        }
        32926 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_uwr_med_corr_z_2_rangea
                as i32;
        }
        32927 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_uwr_med_corr_z_2_rangeb
                as i32;
        }
        32928 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_uwr_med_corr_z_3_rangea
                as i32;
        }
        32929 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_uwr_med_corr_z_3_rangeb
                as i32;
        }
        32930 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_uwr_med_corr_z_4_rangea
                as i32;
        }
        32931 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_uwr_med_corr_z_4_rangeb
                as i32;
        }
        32932 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_uwr_med_corr_z_5_rangea
                as i32;
        }
        32933 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_uwr_med_corr_z_5_rangeb
                as i32;
        }
        32934 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_uwr_lng_z_1_min as i32;
        }
        32935 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_uwr_lng_z_1_max as i32;
        }
        32936 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_uwr_lng_z_2_min as i32;
        }
        32937 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_uwr_lng_z_2_max as i32;
        }
        32938 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_uwr_lng_z_3_min as i32;
        }
        32939 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_uwr_lng_z_3_max as i32;
        }
        32940 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_uwr_lng_z_4_min as i32;
        }
        32941 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_uwr_lng_z_4_max as i32;
        }
        32942 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_uwr_lng_z_5_min as i32;
        }
        32943 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_uwr_lng_z_5_max as i32;
        }
        32944 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_uwr_lng_corr_z_1_rangea
                as i32;
        }
        32945 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_uwr_lng_corr_z_1_rangeb
                as i32;
        }
        32946 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_uwr_lng_corr_z_2_rangea
                as i32;
        }
        32947 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_uwr_lng_corr_z_2_rangeb
                as i32;
        }
        32948 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_uwr_lng_corr_z_3_rangea
                as i32;
        }
        32949 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_uwr_lng_corr_z_3_rangeb
                as i32;
        }
        32950 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_uwr_lng_corr_z_4_rangea
                as i32;
        }
        32951 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_uwr_lng_corr_z_4_rangeb
                as i32;
        }
        32952 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_uwr_lng_corr_z_5_rangea
                as i32;
        }
        32953 => {
            *ptuning_parm_value = (*pdev).tuning_parms.tp_uwr_lng_corr_z_5_rangeb
                as i32;
        }
        _ => {
            *ptuning_parm_value = 0x7fffffff as libc::c_int;
            status = -(4 as libc::c_int) as VL53LX_Error;
        }
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_set_tuning_parm(
    mut Dev: VL53LX_DEV,
    mut tuning_parm_key: VL53LX_TuningParms,
    mut tuning_parm_value: i32,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    let mut pHP: *mut VL53LX_hist_post_process_config_t = &mut (*pdev).histpostprocess;
    let mut pXC: *mut VL53LX_xtalkextract_config_t = &mut (*pdev).xtalk_extract_cfg;
    match tuning_parm_key as libc::c_int {
        32768 => {
            (*pdev).tuning_parms.tp_tuning_parm_version = tuning_parm_value as u16;
        }
        32769 => {
            (*pdev)
                .tuning_parms
                .tp_tuning_parm_key_table_version = tuning_parm_value as u16;
            if tuning_parm_value as u16 as libc::c_int
                != 14 as libc::c_int as u16 as libc::c_int
            {
                status = -(27 as libc::c_int) as VL53LX_Error;
            }
        }
        32770 => {
            (*pdev)
                .tuning_parms
                .tp_tuning_parm_lld_version = tuning_parm_value as u16;
        }
        32771 => {
            (*pHP).hist_algo_select = tuning_parm_value as VL53LX_HistAlgoSelect;
        }
        32772 => {
            (*pHP).hist_target_order = tuning_parm_value as VL53LX_HistTargetOrder;
        }
        32773 => {
            (*pHP).filter_woi0 = tuning_parm_value as u8;
        }
        32774 => {
            (*pHP).filter_woi1 = tuning_parm_value as u8;
        }
        32775 => {
            (*pHP).hist_amb_est_method = tuning_parm_value as VL53LX_HistAmbEstMethod;
        }
        32776 => {
            (*pHP).ambient_thresh_sigma0 = tuning_parm_value as u8;
        }
        32777 => {
            (*pHP).ambient_thresh_sigma1 = tuning_parm_value as u8;
        }
        32778 => {
            (*pHP).min_ambient_thresh_events = tuning_parm_value;
        }
        32779 => {
            (*pHP).ambient_thresh_events_scaler = tuning_parm_value as u16;
        }
        32780 => {
            (*pHP).noise_threshold = tuning_parm_value as u16;
        }
        32781 => {
            (*pHP).signal_total_events_limit = tuning_parm_value;
        }
        32782 => {
            (*pHP).sigma_estimator__sigma_ref_mm = tuning_parm_value as u8;
        }
        32783 => {
            (*pHP).sigma_thresh = tuning_parm_value as u16;
        }
        32784 => {
            (*pdev)
                .gain_cal
                .histogram_ranging_gain_factor = tuning_parm_value as u16;
        }
        32785 => {
            (*pHP)
                .algo__consistency_check__phase_tolerance = tuning_parm_value as u8;
        }
        32786 => {
            (*pHP)
                .algo__consistency_check__min_max_tolerance = tuning_parm_value
                as u16;
        }
        32787 => {
            (*pHP).algo__consistency_check__event_sigma = tuning_parm_value as u8;
        }
        32788 => {
            (*pHP)
                .algo__consistency_check__event_min_spad_count = tuning_parm_value
                as u16;
        }
        32789 => {
            (*pdev)
                .tuning_parms
                .tp_init_phase_rtn_hist_long = tuning_parm_value as u8;
        }
        32790 => {
            (*pdev)
                .tuning_parms
                .tp_init_phase_rtn_hist_med = tuning_parm_value as u8;
        }
        32791 => {
            (*pdev)
                .tuning_parms
                .tp_init_phase_rtn_hist_short = tuning_parm_value as u8;
        }
        32792 => {
            (*pdev)
                .tuning_parms
                .tp_init_phase_ref_hist_long = tuning_parm_value as u8;
        }
        32793 => {
            (*pdev)
                .tuning_parms
                .tp_init_phase_ref_hist_med = tuning_parm_value as u8;
        }
        32794 => {
            (*pdev)
                .tuning_parms
                .tp_init_phase_ref_hist_short = tuning_parm_value as u8;
        }
        32795 => {
            (*pdev)
                .xtalk_cfg
                .algo__crosstalk_detect_min_valid_range_mm = tuning_parm_value
                as i16;
        }
        32796 => {
            (*pdev)
                .xtalk_cfg
                .algo__crosstalk_detect_max_valid_range_mm = tuning_parm_value
                as i16;
        }
        32797 => {
            (*pdev)
                .xtalk_cfg
                .algo__crosstalk_detect_max_sigma_mm = tuning_parm_value as u16;
        }
        32798 => {
            (*pHP)
                .algo__crosstalk_detect_min_max_tolerance = tuning_parm_value
                as u16;
        }
        32799 => {
            (*pdev)
                .xtalk_cfg
                .algo__crosstalk_detect_max_valid_rate_kcps = tuning_parm_value
                as u16;
        }
        32800 => {
            (*pHP).algo__crosstalk_detect_event_sigma = tuning_parm_value as u8;
        }
        32801 => {
            (*pdev)
                .xtalk_cfg
                .histogram_mode_crosstalk_margin_kcps = tuning_parm_value as i16;
        }
        32802 => {
            (*pdev)
                .tuning_parms
                .tp_consistency_lite_phase_tolerance = tuning_parm_value as u8;
        }
        32803 => {
            (*pdev).tuning_parms.tp_phasecal_target = tuning_parm_value as u8;
        }
        32804 => {
            (*pdev).tuning_parms.tp_cal_repeat_rate = tuning_parm_value as u16;
        }
        32805 => {
            (*pdev)
                .gain_cal
                .standard_ranging_gain_factor = tuning_parm_value as u16;
        }
        32806 => {
            (*pdev).tuning_parms.tp_lite_min_clip = tuning_parm_value as u8;
        }
        32807 => {
            (*pdev)
                .tuning_parms
                .tp_lite_long_sigma_thresh_mm = tuning_parm_value as u16;
        }
        32808 => {
            (*pdev)
                .tuning_parms
                .tp_lite_med_sigma_thresh_mm = tuning_parm_value as u16;
        }
        32809 => {
            (*pdev)
                .tuning_parms
                .tp_lite_short_sigma_thresh_mm = tuning_parm_value as u16;
        }
        32810 => {
            (*pdev)
                .tuning_parms
                .tp_lite_long_min_count_rate_rtn_mcps = tuning_parm_value as u16;
        }
        32811 => {
            (*pdev)
                .tuning_parms
                .tp_lite_med_min_count_rate_rtn_mcps = tuning_parm_value as u16;
        }
        32812 => {
            (*pdev)
                .tuning_parms
                .tp_lite_short_min_count_rate_rtn_mcps = tuning_parm_value as u16;
        }
        32813 => {
            (*pdev)
                .tuning_parms
                .tp_lite_sigma_est_pulse_width_ns = tuning_parm_value as u8;
        }
        32814 => {
            (*pdev)
                .tuning_parms
                .tp_lite_sigma_est_amb_width_ns = tuning_parm_value as u8;
        }
        32815 => {
            (*pdev).tuning_parms.tp_lite_sigma_ref_mm = tuning_parm_value as u8;
        }
        32816 => {
            (*pdev)
                .xtalk_cfg
                .crosstalk_range_ignore_threshold_mult = tuning_parm_value as u8;
        }
        32817 => {
            (*pdev).tuning_parms.tp_lite_seed_cfg = tuning_parm_value as u8;
        }
        32818 => {
            (*pdev).tuning_parms.tp_lite_quantifier = tuning_parm_value as u8;
        }
        32819 => {
            (*pdev)
                .tuning_parms
                .tp_lite_first_order_select = tuning_parm_value as u8;
        }
        32820 => {
            (*pdev)
                .xtalk_cfg
                .lite_mode_crosstalk_margin_kcps = tuning_parm_value as i16;
        }
        32821 => {
            (*pdev)
                .tuning_parms
                .tp_init_phase_rtn_lite_long = tuning_parm_value as u8;
        }
        32822 => {
            (*pdev)
                .tuning_parms
                .tp_init_phase_rtn_lite_med = tuning_parm_value as u8;
        }
        32823 => {
            (*pdev)
                .tuning_parms
                .tp_init_phase_rtn_lite_short = tuning_parm_value as u8;
        }
        32824 => {
            (*pdev)
                .tuning_parms
                .tp_init_phase_ref_lite_long = tuning_parm_value as u8;
        }
        32825 => {
            (*pdev)
                .tuning_parms
                .tp_init_phase_ref_lite_med = tuning_parm_value as u8;
        }
        32826 => {
            (*pdev)
                .tuning_parms
                .tp_init_phase_ref_lite_short = tuning_parm_value as u8;
        }
        32827 => {
            (*pdev).tuning_parms.tp_timed_seed_cfg = tuning_parm_value as u8;
        }
        32828 => {
            (*pdev).dmax_cfg.signal_thresh_sigma = tuning_parm_value as u8;
        }
        32829 => {
            (*pdev)
                .dmax_cfg
                .target_reflectance_for_dmax_calc[0 as libc::c_int
                as usize] = tuning_parm_value as u16;
        }
        32830 => {
            (*pdev)
                .dmax_cfg
                .target_reflectance_for_dmax_calc[1 as libc::c_int
                as usize] = tuning_parm_value as u16;
        }
        32831 => {
            (*pdev)
                .dmax_cfg
                .target_reflectance_for_dmax_calc[2 as libc::c_int
                as usize] = tuning_parm_value as u16;
        }
        32832 => {
            (*pdev)
                .dmax_cfg
                .target_reflectance_for_dmax_calc[3 as libc::c_int
                as usize] = tuning_parm_value as u16;
        }
        32833 => {
            (*pdev)
                .dmax_cfg
                .target_reflectance_for_dmax_calc[4 as libc::c_int
                as usize] = tuning_parm_value as u16;
        }
        32834 => {
            (*pdev)
                .stat_nvm
                .vhv_config__timeout_macrop_loop_bound = tuning_parm_value as u8;
        }
        32835 => {
            (*pdev).refspadchar.device_test_mode = tuning_parm_value as u8;
        }
        32836 => {
            (*pdev).refspadchar.VL53LX_p_005 = tuning_parm_value as u8;
        }
        32837 => {
            (*pdev).refspadchar.timeout_us = tuning_parm_value as u32;
        }
        32838 => {
            (*pdev).refspadchar.target_count_rate_mcps = tuning_parm_value as u16;
        }
        32839 => {
            (*pdev)
                .refspadchar
                .min_count_rate_limit_mcps = tuning_parm_value as u16;
        }
        32840 => {
            (*pdev)
                .refspadchar
                .max_count_rate_limit_mcps = tuning_parm_value as u16;
        }
        32841 => {
            (*pXC).num_of_samples = tuning_parm_value as u8;
        }
        32842 => {
            (*pXC)
                .algo__crosstalk_extract_min_valid_range_mm = tuning_parm_value
                as i16;
        }
        32843 => {
            (*pXC)
                .algo__crosstalk_extract_max_valid_range_mm = tuning_parm_value
                as i16;
        }
        32844 => {
            (*pXC).dss_config__target_total_rate_mcps = tuning_parm_value as u16;
        }
        32845 => {
            (*pXC).phasecal_config_timeout_us = tuning_parm_value as u32;
        }
        32846 => {
            (*pXC)
                .algo__crosstalk_extract_max_valid_rate_kcps = tuning_parm_value
                as u16;
        }
        32847 => {
            (*pXC).algo__crosstalk_extract_max_sigma_mm = tuning_parm_value as u16;
        }
        32848 => {
            (*pXC).mm_config_timeout_us = tuning_parm_value as u32;
        }
        32849 => {
            (*pXC).range_config_timeout_us = tuning_parm_value as u32;
        }
        32850 => {
            (*pdev)
                .offsetcal_cfg
                .dss_config__target_total_rate_mcps = tuning_parm_value as u16;
        }
        32851 => {
            (*pdev)
                .offsetcal_cfg
                .phasecal_config_timeout_us = tuning_parm_value as u32;
        }
        32852 => {
            (*pdev).offsetcal_cfg.mm_config_timeout_us = tuning_parm_value as u32;
        }
        32853 => {
            (*pdev)
                .offsetcal_cfg
                .range_config_timeout_us = tuning_parm_value as u32;
        }
        32854 => {
            (*pdev).offsetcal_cfg.pre_num_of_samples = tuning_parm_value as u8;
        }
        32855 => {
            (*pdev).offsetcal_cfg.mm1_num_of_samples = tuning_parm_value as u8;
        }
        32856 => {
            (*pdev).offsetcal_cfg.mm2_num_of_samples = tuning_parm_value as u8;
        }
        32857 => {
            (*pdev)
                .zonecal_cfg
                .dss_config__target_total_rate_mcps = tuning_parm_value as u16;
        }
        32858 => {
            (*pdev)
                .zonecal_cfg
                .phasecal_config_timeout_us = tuning_parm_value as u32;
        }
        32859 => {
            (*pdev).zonecal_cfg.mm_config_timeout_us = tuning_parm_value as u32;
        }
        32860 => {
            (*pdev).zonecal_cfg.phasecal_num_of_samples = tuning_parm_value as u16;
        }
        32861 => {
            (*pdev).zonecal_cfg.range_config_timeout_us = tuning_parm_value as u32;
        }
        32862 => {
            (*pdev).zonecal_cfg.zone_num_of_samples = tuning_parm_value as u16;
        }
        32863 => {
            (*pdev).ssc_cfg.VL53LX_p_005 = tuning_parm_value as u8;
        }
        32864 => {
            (*pdev).ssc_cfg.vcsel_start = tuning_parm_value as u8;
        }
        32865 => {
            (*pdev).ssc_cfg.rate_limit_mcps = tuning_parm_value as u16;
        }
        32866 => {
            (*pdev).tuning_parms.tp_dss_target_lite_mcps = tuning_parm_value as u16;
        }
        32867 => {
            (*pdev)
                .tuning_parms
                .tp_dss_target_histo_mcps = tuning_parm_value as u16;
        }
        32868 => {
            (*pdev)
                .tuning_parms
                .tp_dss_target_histo_mz_mcps = tuning_parm_value as u16;
        }
        32869 => {
            (*pdev)
                .tuning_parms
                .tp_dss_target_timed_mcps = tuning_parm_value as u16;
        }
        32870 => {
            (*pdev)
                .tuning_parms
                .tp_phasecal_timeout_lite_us = tuning_parm_value as u32;
        }
        32871 => {
            (*pdev)
                .tuning_parms
                .tp_phasecal_timeout_hist_long_us = tuning_parm_value as u32;
        }
        32872 => {
            (*pdev)
                .tuning_parms
                .tp_phasecal_timeout_hist_med_us = tuning_parm_value as u32;
        }
        32873 => {
            (*pdev)
                .tuning_parms
                .tp_phasecal_timeout_hist_short_us = tuning_parm_value as u32;
        }
        32874 => {
            (*pdev)
                .tuning_parms
                .tp_phasecal_timeout_mz_long_us = tuning_parm_value as u32;
        }
        32875 => {
            (*pdev)
                .tuning_parms
                .tp_phasecal_timeout_mz_med_us = tuning_parm_value as u32;
        }
        32876 => {
            (*pdev)
                .tuning_parms
                .tp_phasecal_timeout_mz_short_us = tuning_parm_value as u32;
        }
        32877 => {
            (*pdev)
                .tuning_parms
                .tp_phasecal_timeout_timed_us = tuning_parm_value as u32;
        }
        32878 => {
            (*pdev).tuning_parms.tp_mm_timeout_lite_us = tuning_parm_value as u32;
        }
        32879 => {
            (*pdev).tuning_parms.tp_mm_timeout_histo_us = tuning_parm_value as u32;
        }
        32880 => {
            (*pdev).tuning_parms.tp_mm_timeout_mz_us = tuning_parm_value as u32;
        }
        32881 => {
            (*pdev).tuning_parms.tp_mm_timeout_timed_us = tuning_parm_value as u32;
        }
        32882 => {
            (*pdev)
                .tuning_parms
                .tp_range_timeout_lite_us = tuning_parm_value as u32;
        }
        32883 => {
            (*pdev)
                .tuning_parms
                .tp_range_timeout_histo_us = tuning_parm_value as u32;
        }
        32884 => {
            (*pdev).tuning_parms.tp_range_timeout_mz_us = tuning_parm_value as u32;
        }
        32885 => {
            (*pdev)
                .tuning_parms
                .tp_range_timeout_timed_us = tuning_parm_value as u32;
        }
        32886 => {
            (*pdev).smudge_correct_config.smudge_margin = tuning_parm_value as u16;
        }
        32887 => {
            (*pdev).smudge_correct_config.noise_margin = tuning_parm_value as u32;
        }
        32888 => {
            (*pdev)
                .smudge_correct_config
                .user_xtalk_offset_limit = tuning_parm_value as u32;
        }
        32889 => {
            (*pdev)
                .smudge_correct_config
                .user_xtalk_offset_limit_hi = tuning_parm_value as u8;
        }
        32890 => {
            (*pdev).smudge_correct_config.sample_limit = tuning_parm_value as u32;
        }
        32891 => {
            (*pdev)
                .smudge_correct_config
                .single_xtalk_delta = tuning_parm_value as u32;
        }
        32892 => {
            (*pdev)
                .smudge_correct_config
                .averaged_xtalk_delta = tuning_parm_value as u32;
        }
        32893 => {
            (*pdev)
                .smudge_correct_config
                .smudge_corr_clip_limit = tuning_parm_value as u32;
        }
        32894 => {
            (*pdev)
                .smudge_correct_config
                .scaler_calc_method = tuning_parm_value as u8;
        }
        32895 => {
            (*pdev)
                .smudge_correct_config
                .x_gradient_scaler = tuning_parm_value as i16;
        }
        32896 => {
            (*pdev)
                .smudge_correct_config
                .y_gradient_scaler = tuning_parm_value as i16;
        }
        32897 => {
            (*pdev).smudge_correct_config.user_scaler_set = tuning_parm_value as u8;
        }
        32898 => {
            (*pdev)
                .smudge_correct_config
                .smudge_corr_single_apply = tuning_parm_value as u8;
        }
        32899 => {
            (*pdev)
                .smudge_correct_config
                .smudge_corr_ambient_threshold = tuning_parm_value as u32;
        }
        32900 => {
            (*pdev)
                .smudge_correct_config
                .nodetect_ambient_threshold = tuning_parm_value as u32;
        }
        32901 => {
            (*pdev)
                .smudge_correct_config
                .nodetect_sample_limit = tuning_parm_value as u32;
        }
        32902 => {
            (*pdev)
                .smudge_correct_config
                .nodetect_xtalk_offset = tuning_parm_value as u32;
        }
        32903 => {
            (*pdev)
                .smudge_correct_config
                .nodetect_min_range_mm = tuning_parm_value as u16;
        }
        32904 => {
            (*pdev).low_power_auto_data.vhv_loop_bound = tuning_parm_value as u8;
        }
        32905 => {
            (*pdev).tuning_parms.tp_mm_timeout_lpa_us = tuning_parm_value as u32;
        }
        32906 => {
            (*pdev).tuning_parms.tp_range_timeout_lpa_us = tuning_parm_value as u32;
        }
        32907 => {
            (*pdev)
                .tuning_parms
                .tp_dss_target_very_short_mcps = tuning_parm_value as u16;
        }
        32908 => {
            (*pdev)
                .tuning_parms
                .tp_phasecal_patch_power = tuning_parm_value as u16 as u32;
        }
        32909 => {
            (*pdev)
                .tuning_parms
                .tp_hist_merge = tuning_parm_value as u16 as u8;
        }
        32910 => {
            (*pdev)
                .tuning_parms
                .tp_reset_merge_threshold = tuning_parm_value as u16 as u32;
        }
        32911 => {
            (*pdev)
                .tuning_parms
                .tp_hist_merge_max_size = tuning_parm_value as u16 as u8;
        }
        32912 => {
            (*pdev)
                .smudge_correct_config
                .max_smudge_factor = tuning_parm_value as u32;
        }
        32913 => {
            (*pdev).tuning_parms.tp_uwr_enable = tuning_parm_value as u8;
        }
        32914 => {
            (*pdev).tuning_parms.tp_uwr_med_z_1_min = tuning_parm_value as i16;
        }
        32915 => {
            (*pdev).tuning_parms.tp_uwr_med_z_1_max = tuning_parm_value as i16;
        }
        32916 => {
            (*pdev).tuning_parms.tp_uwr_med_z_2_min = tuning_parm_value as i16;
        }
        32917 => {
            (*pdev).tuning_parms.tp_uwr_med_z_2_max = tuning_parm_value as i16;
        }
        32918 => {
            (*pdev).tuning_parms.tp_uwr_med_z_3_min = tuning_parm_value as i16;
        }
        32919 => {
            (*pdev).tuning_parms.tp_uwr_med_z_3_max = tuning_parm_value as i16;
        }
        32920 => {
            (*pdev).tuning_parms.tp_uwr_med_z_4_min = tuning_parm_value as i16;
        }
        32921 => {
            (*pdev).tuning_parms.tp_uwr_med_z_4_max = tuning_parm_value as i16;
        }
        32922 => {
            (*pdev).tuning_parms.tp_uwr_med_z_5_min = tuning_parm_value as i16;
        }
        32923 => {
            (*pdev).tuning_parms.tp_uwr_med_z_5_max = tuning_parm_value as i16;
        }
        32924 => {
            (*pdev)
                .tuning_parms
                .tp_uwr_med_corr_z_1_rangea = tuning_parm_value as i16;
        }
        32925 => {
            (*pdev)
                .tuning_parms
                .tp_uwr_med_corr_z_1_rangeb = tuning_parm_value as i16;
        }
        32926 => {
            (*pdev)
                .tuning_parms
                .tp_uwr_med_corr_z_2_rangea = tuning_parm_value as i16;
        }
        32927 => {
            (*pdev)
                .tuning_parms
                .tp_uwr_med_corr_z_2_rangeb = tuning_parm_value as i16;
        }
        32928 => {
            (*pdev)
                .tuning_parms
                .tp_uwr_med_corr_z_3_rangea = tuning_parm_value as i16;
        }
        32929 => {
            (*pdev)
                .tuning_parms
                .tp_uwr_med_corr_z_3_rangeb = tuning_parm_value as i16;
        }
        32930 => {
            (*pdev)
                .tuning_parms
                .tp_uwr_med_corr_z_4_rangea = tuning_parm_value as i16;
        }
        32931 => {
            (*pdev)
                .tuning_parms
                .tp_uwr_med_corr_z_4_rangeb = tuning_parm_value as i16;
        }
        32932 => {
            (*pdev)
                .tuning_parms
                .tp_uwr_med_corr_z_5_rangea = tuning_parm_value as i16;
        }
        32933 => {
            (*pdev)
                .tuning_parms
                .tp_uwr_med_corr_z_5_rangeb = tuning_parm_value as i16;
        }
        32934 => {
            (*pdev).tuning_parms.tp_uwr_lng_z_1_min = tuning_parm_value as i16;
        }
        32935 => {
            (*pdev).tuning_parms.tp_uwr_lng_z_1_max = tuning_parm_value as i16;
        }
        32936 => {
            (*pdev).tuning_parms.tp_uwr_lng_z_2_min = tuning_parm_value as i16;
        }
        32937 => {
            (*pdev).tuning_parms.tp_uwr_lng_z_2_max = tuning_parm_value as i16;
        }
        32938 => {
            (*pdev).tuning_parms.tp_uwr_lng_z_3_min = tuning_parm_value as i16;
        }
        32939 => {
            (*pdev).tuning_parms.tp_uwr_lng_z_3_max = tuning_parm_value as i16;
        }
        32940 => {
            (*pdev).tuning_parms.tp_uwr_lng_z_4_min = tuning_parm_value as i16;
        }
        32941 => {
            (*pdev).tuning_parms.tp_uwr_lng_z_4_max = tuning_parm_value as i16;
        }
        32942 => {
            (*pdev).tuning_parms.tp_uwr_lng_z_5_min = tuning_parm_value as i16;
        }
        32943 => {
            (*pdev).tuning_parms.tp_uwr_lng_z_5_max = tuning_parm_value as i16;
        }
        32944 => {
            (*pdev)
                .tuning_parms
                .tp_uwr_lng_corr_z_1_rangea = tuning_parm_value as i16;
        }
        32945 => {
            (*pdev)
                .tuning_parms
                .tp_uwr_lng_corr_z_1_rangeb = tuning_parm_value as i16;
        }
        32946 => {
            (*pdev)
                .tuning_parms
                .tp_uwr_lng_corr_z_2_rangea = tuning_parm_value as i16;
        }
        32947 => {
            (*pdev)
                .tuning_parms
                .tp_uwr_lng_corr_z_2_rangeb = tuning_parm_value as i16;
        }
        32948 => {
            (*pdev)
                .tuning_parms
                .tp_uwr_lng_corr_z_3_rangea = tuning_parm_value as i16;
        }
        32949 => {
            (*pdev)
                .tuning_parms
                .tp_uwr_lng_corr_z_3_rangeb = tuning_parm_value as i16;
        }
        32950 => {
            (*pdev)
                .tuning_parms
                .tp_uwr_lng_corr_z_4_rangea = tuning_parm_value as i16;
        }
        32951 => {
            (*pdev)
                .tuning_parms
                .tp_uwr_lng_corr_z_4_rangeb = tuning_parm_value as i16;
        }
        32952 => {
            (*pdev)
                .tuning_parms
                .tp_uwr_lng_corr_z_5_rangea = tuning_parm_value as i16;
        }
        32953 => {
            (*pdev)
                .tuning_parms
                .tp_uwr_lng_corr_z_5_rangeb = tuning_parm_value as i16;
        }
        _ => {
            status = -(4 as libc::c_int) as VL53LX_Error;
        }
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_dynamic_xtalk_correction_enable(
    mut Dev: VL53LX_DEV,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    (*pdev).smudge_correct_config.smudge_corr_enabled = 1 as libc::c_int as u8;
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_dynamic_xtalk_correction_disable(
    mut Dev: VL53LX_DEV,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    (*pdev).smudge_correct_config.smudge_corr_enabled = 0 as libc::c_int as u8;
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_dynamic_xtalk_correction_apply_disable(
    mut Dev: VL53LX_DEV,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    (*pdev)
        .smudge_correct_config
        .smudge_corr_apply_enabled = 0 as libc::c_int as u8;
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_dynamic_xtalk_correction_single_apply_enable(
    mut Dev: VL53LX_DEV,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    (*pdev).smudge_correct_config.smudge_corr_single_apply = 1 as libc::c_int as u8;
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_dynamic_xtalk_correction_single_apply_disable(
    mut Dev: VL53LX_DEV,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    (*pdev).smudge_correct_config.smudge_corr_single_apply = 0 as libc::c_int as u8;
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_dynamic_xtalk_correction_apply_enable(
    mut Dev: VL53LX_DEV,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    (*pdev)
        .smudge_correct_config
        .smudge_corr_apply_enabled = 1 as libc::c_int as u8;
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_get_current_xtalk_settings(
    mut Dev: VL53LX_DEV,
    mut pxtalk: *mut VL53LX_xtalk_calibration_results_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut i: u8 = 0;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    (*pxtalk)
        .algo__crosstalk_compensation_plane_offset_kcps = (*pdev)
        .xtalk_cfg
        .algo__crosstalk_compensation_plane_offset_kcps;
    (*pxtalk)
        .algo__crosstalk_compensation_x_plane_gradient_kcps = (*pdev)
        .xtalk_cfg
        .algo__crosstalk_compensation_x_plane_gradient_kcps;
    (*pxtalk)
        .algo__crosstalk_compensation_y_plane_gradient_kcps = (*pdev)
        .xtalk_cfg
        .algo__crosstalk_compensation_y_plane_gradient_kcps;
    i = 0 as libc::c_int as u8;
    while (i as libc::c_int) < 6 as libc::c_int {
        (*pxtalk)
            .algo__xtalk_cpo_HistoMerge_kcps[i
            as usize] = (*pdev).xtalk_cal.algo__xtalk_cpo_HistoMerge_kcps[i as usize];
        i = i.wrapping_add(1);
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_set_current_xtalk_settings(
    mut Dev: VL53LX_DEV,
    mut pxtalk: *mut VL53LX_xtalk_calibration_results_t,
) -> VL53LX_Error {
    let mut i: u8 = 0;
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    (*pdev)
        .xtalk_cfg
        .algo__crosstalk_compensation_plane_offset_kcps = (*pxtalk)
        .algo__crosstalk_compensation_plane_offset_kcps;
    (*pdev)
        .xtalk_cfg
        .algo__crosstalk_compensation_x_plane_gradient_kcps = (*pxtalk)
        .algo__crosstalk_compensation_x_plane_gradient_kcps;
    (*pdev)
        .xtalk_cfg
        .algo__crosstalk_compensation_y_plane_gradient_kcps = (*pxtalk)
        .algo__crosstalk_compensation_y_plane_gradient_kcps;
    i = 0 as libc::c_int as u8;
    while (i as libc::c_int) < 6 as libc::c_int {
        (*pdev)
            .xtalk_cal
            .algo__xtalk_cpo_HistoMerge_kcps[i
            as usize] = (*pxtalk).algo__xtalk_cpo_HistoMerge_kcps[i as usize];
        i = i.wrapping_add(1);
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_decode_calibration_data_buffer(
    mut buf_size: u16,
    mut pbuffer: *mut u8,
    mut pdata: *mut VL53LX_calibration_data_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    if ::std::mem::size_of::<VL53LX_calibration_data_t>() as libc::c_ulong
        > buf_size as libc::c_ulong
    {
        return -(10 as libc::c_int) as VL53LX_Error;
    }
    memcpy(
        pdata as *mut libc::c_void,
        pbuffer as *const libc::c_void,
        ::std::mem::size_of::<VL53LX_calibration_data_t>() as libc::c_ulong,
    );
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_get_nvm_debug_data(
    mut Dev: VL53LX_DEV,
    mut pdata: *mut VL53LX_decoded_nvm_data_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    status = VL53LX_read_nvm(Dev, 0 as libc::c_int as u8, pdata);
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_get_histogram_debug_data(
    mut Dev: VL53LX_DEV,
    mut pdata: *mut VL53LX_histogram_bin_data_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    memcpy(
        pdata as *mut libc::c_void,
        &mut (*pdev).hist_data as *mut VL53LX_histogram_bin_data_t
            as *const libc::c_void,
        ::std::mem::size_of::<VL53LX_histogram_bin_data_t>() as libc::c_ulong,
    );
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_get_additional_data(
    mut Dev: VL53LX_DEV,
    mut pdata: *mut VL53LX_additional_data_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    (*pdata).preset_mode = (*pdev).preset_mode;
    (*pdata).zone_preset = (*pdev).zone_preset;
    (*pdata).measurement_mode = (*pdev).measurement_mode;
    (*pdata).offset_calibration_mode = (*pdev).offset_calibration_mode;
    (*pdata).offset_correction_mode = (*pdev).offset_correction_mode;
    (*pdata).dmax_mode = (*pdev).dmax_mode;
    (*pdata).phasecal_config_timeout_us = (*pdev).phasecal_config_timeout_us;
    (*pdata).mm_config_timeout_us = (*pdev).mm_config_timeout_us;
    (*pdata).range_config_timeout_us = (*pdev).range_config_timeout_us;
    (*pdata).inter_measurement_period_ms = (*pdev).inter_measurement_period_ms;
    (*pdata)
        .dss_config__target_total_rate_mcps = (*pdev).dss_config__target_total_rate_mcps;
    status = VL53LX_get_histogram_debug_data(Dev, &mut (*pdata).VL53LX_p_006);
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_get_xtalk_debug_data(
    mut Dev: VL53LX_DEV,
    mut pdata: *mut VL53LX_xtalk_debug_data_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    memcpy(
        &mut (*pdata).customer as *mut VL53LX_customer_nvm_managed_t
            as *mut libc::c_void,
        &mut (*pdev).customer as *mut VL53LX_customer_nvm_managed_t
            as *const libc::c_void,
        ::std::mem::size_of::<VL53LX_customer_nvm_managed_t>() as libc::c_ulong,
    );
    memcpy(
        &mut (*pdata).xtalk_cfg as *mut VL53LX_xtalk_config_t as *mut libc::c_void,
        &mut (*pdev).xtalk_cfg as *mut VL53LX_xtalk_config_t as *const libc::c_void,
        ::std::mem::size_of::<VL53LX_xtalk_config_t>() as libc::c_ulong,
    );
    memcpy(
        &mut (*pdata).hist_data as *mut VL53LX_histogram_bin_data_t as *mut libc::c_void,
        &mut (*pdev).hist_data as *mut VL53LX_histogram_bin_data_t
            as *const libc::c_void,
        ::std::mem::size_of::<VL53LX_histogram_bin_data_t>() as libc::c_ulong,
    );
    memcpy(
        &mut (*pdata).xtalk_shapes as *mut VL53LX_xtalk_histogram_data_t
            as *mut libc::c_void,
        &mut (*pdev).xtalk_shapes as *mut VL53LX_xtalk_histogram_data_t
            as *const libc::c_void,
        ::std::mem::size_of::<VL53LX_xtalk_histogram_data_t>() as libc::c_ulong,
    );
    memcpy(
        &mut (*pdata).xtalk_results as *mut VL53LX_xtalk_range_results_t
            as *mut libc::c_void,
        &mut (*pdev).xtalk_results as *mut VL53LX_xtalk_range_results_t
            as *const libc::c_void,
        ::std::mem::size_of::<VL53LX_xtalk_range_results_t>() as libc::c_ulong,
    );
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_get_offset_debug_data(
    mut Dev: VL53LX_DEV,
    mut pdata: *mut VL53LX_offset_debug_data_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    memcpy(
        &mut (*pdata).customer as *mut VL53LX_customer_nvm_managed_t
            as *mut libc::c_void,
        &mut (*pdev).customer as *mut VL53LX_customer_nvm_managed_t
            as *const libc::c_void,
        ::std::mem::size_of::<VL53LX_customer_nvm_managed_t>() as libc::c_ulong,
    );
    memcpy(
        &mut (*pdata).fmt_dmax_cal as *mut VL53LX_dmax_calibration_data_t
            as *mut libc::c_void,
        &mut (*pdev).fmt_dmax_cal as *mut VL53LX_dmax_calibration_data_t
            as *const libc::c_void,
        ::std::mem::size_of::<VL53LX_dmax_calibration_data_t>() as libc::c_ulong,
    );
    memcpy(
        &mut (*pdata).cust_dmax_cal as *mut VL53LX_dmax_calibration_data_t
            as *mut libc::c_void,
        &mut (*pdev).cust_dmax_cal as *mut VL53LX_dmax_calibration_data_t
            as *const libc::c_void,
        ::std::mem::size_of::<VL53LX_dmax_calibration_data_t>() as libc::c_ulong,
    );
    memcpy(
        &mut (*pdata).add_off_cal_data as *mut VL53LX_additional_offset_cal_data_t
            as *mut libc::c_void,
        &mut (*pdev).add_off_cal_data as *mut VL53LX_additional_offset_cal_data_t
            as *const libc::c_void,
        ::std::mem::size_of::<VL53LX_additional_offset_cal_data_t>() as libc::c_ulong,
    );
    memcpy(
        &mut (*pdata).offset_results as *mut VL53LX_offset_range_results_t
            as *mut libc::c_void,
        &mut (*pdev).offset_results as *mut VL53LX_offset_range_results_t
            as *const libc::c_void,
        ::std::mem::size_of::<VL53LX_offset_range_results_t>() as libc::c_ulong,
    );
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_init_refspadchar_config_struct(
    mut pdata: *mut VL53LX_refspadchar_config_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    (*pdata).device_test_mode = 8 as libc::c_int as u8;
    (*pdata).VL53LX_p_005 = 11 as libc::c_int as u8;
    (*pdata).timeout_us = 1000 as libc::c_int as u32;
    (*pdata).target_count_rate_mcps = 2560 as libc::c_int as u16;
    (*pdata).min_count_rate_limit_mcps = 1280 as libc::c_int as u16;
    (*pdata).max_count_rate_limit_mcps = 5120 as libc::c_int as u16;
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_init_ssc_config_struct(
    mut pdata: *mut VL53LX_ssc_config_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    (*pdata).array_select = 0 as libc::c_int as VL53LX_DeviceSscArray;
    (*pdata).VL53LX_p_005 = 18 as libc::c_int as u8;
    (*pdata).vcsel_start = 15 as libc::c_int as u8;
    (*pdata).vcsel_width = 0x2 as libc::c_int as u8;
    (*pdata).timeout_us = 36000 as libc::c_int as u32;
    (*pdata).rate_limit_mcps = 12 as libc::c_int as u16;
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_init_xtalk_config_struct(
    mut pnvm: *mut VL53LX_customer_nvm_managed_t,
    mut pdata: *mut VL53LX_xtalk_config_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    (*pdata)
        .algo__crosstalk_compensation_plane_offset_kcps = (*pnvm)
        .algo__crosstalk_compensation_plane_offset_kcps as u32;
    (*pdata)
        .algo__crosstalk_compensation_x_plane_gradient_kcps = (*pnvm)
        .algo__crosstalk_compensation_x_plane_gradient_kcps;
    (*pdata)
        .algo__crosstalk_compensation_y_plane_gradient_kcps = (*pnvm)
        .algo__crosstalk_compensation_y_plane_gradient_kcps;
    (*pdata)
        .nvm_default__crosstalk_compensation_plane_offset_kcps = (*pnvm)
        .algo__crosstalk_compensation_plane_offset_kcps as u32;
    (*pdata)
        .nvm_default__crosstalk_compensation_x_plane_gradient_kcps = (*pnvm)
        .algo__crosstalk_compensation_x_plane_gradient_kcps;
    (*pdata)
        .nvm_default__crosstalk_compensation_y_plane_gradient_kcps = (*pnvm)
        .algo__crosstalk_compensation_y_plane_gradient_kcps;
    (*pdata).histogram_mode_crosstalk_margin_kcps = 0 as libc::c_int as i16;
    (*pdata).lite_mode_crosstalk_margin_kcps = 0 as libc::c_int as i16;
    (*pdata).crosstalk_range_ignore_threshold_mult = 64 as libc::c_int as u8;
    if (*pdata).algo__crosstalk_compensation_plane_offset_kcps
        == 0 as libc::c_int as libc::c_uint
        && (*pdata).algo__crosstalk_compensation_x_plane_gradient_kcps as libc::c_int
            == 0 as libc::c_int
        && (*pdata).algo__crosstalk_compensation_y_plane_gradient_kcps as libc::c_int
            == 0 as libc::c_int
    {
        (*pdata).global_crosstalk_compensation_enable = 0 as libc::c_int as u8;
    } else {
        (*pdata).global_crosstalk_compensation_enable = 0x1 as libc::c_int as u8;
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int
        && (*pdata).global_crosstalk_compensation_enable as libc::c_int
            == 0x1 as libc::c_int
    {
        (*pdata)
            .crosstalk_range_ignore_threshold_rate_mcps = VL53LX_calc_range_ignore_threshold(
            (*pdata).algo__crosstalk_compensation_plane_offset_kcps,
            (*pdata).algo__crosstalk_compensation_x_plane_gradient_kcps,
            (*pdata).algo__crosstalk_compensation_y_plane_gradient_kcps,
            (*pdata).crosstalk_range_ignore_threshold_mult,
        );
    } else {
        (*pdata)
            .crosstalk_range_ignore_threshold_rate_mcps = 0 as libc::c_int as u16;
    }
    (*pdata).algo__crosstalk_detect_min_valid_range_mm = -(50 as libc::c_int) as i16;
    (*pdata).algo__crosstalk_detect_max_valid_range_mm = 50 as libc::c_int as i16;
    (*pdata).algo__crosstalk_detect_max_valid_rate_kcps = 400 as libc::c_int as u16;
    (*pdata).algo__crosstalk_detect_max_sigma_mm = 140 as libc::c_int as u16;
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_init_xtalk_extract_config_struct(
    mut pdata: *mut VL53LX_xtalkextract_config_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    (*pdata).dss_config__target_total_rate_mcps = 5120 as libc::c_int as u16;
    (*pdata).mm_config_timeout_us = 2000 as libc::c_int as u32;
    (*pdata).num_of_samples = 7 as libc::c_int as u8;
    (*pdata).phasecal_config_timeout_us = 15000 as libc::c_int as u32;
    (*pdata).range_config_timeout_us = 10000 as libc::c_int as u32;
    (*pdata)
        .algo__crosstalk_extract_min_valid_range_mm = -(70 as libc::c_int) as i16;
    (*pdata).algo__crosstalk_extract_max_valid_range_mm = 70 as libc::c_int as i16;
    (*pdata)
        .algo__crosstalk_extract_max_valid_rate_kcps = 640 as libc::c_int as u16;
    (*pdata).algo__crosstalk_extract_max_sigma_mm = 140 as libc::c_int as u16;
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_init_offset_cal_config_struct(
    mut pdata: *mut VL53LX_offsetcal_config_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    (*pdata).dss_config__target_total_rate_mcps = 2560 as libc::c_int as u16;
    (*pdata).phasecal_config_timeout_us = 15000 as libc::c_int as u32;
    (*pdata).range_config_timeout_us = 13000 as libc::c_int as u32;
    (*pdata).mm_config_timeout_us = 13000 as libc::c_int as u32;
    (*pdata).pre_num_of_samples = 8 as libc::c_int as u8;
    (*pdata).mm1_num_of_samples = 40 as libc::c_int as u8;
    (*pdata).mm2_num_of_samples = 9 as libc::c_int as u8;
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_init_zone_cal_config_struct(
    mut pdata: *mut VL53LX_zonecal_config_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    (*pdata).dss_config__target_total_rate_mcps = 5120 as libc::c_int as u16;
    (*pdata).phasecal_config_timeout_us = 15000 as libc::c_int as u32;
    (*pdata).range_config_timeout_us = 1000 as libc::c_int as u32;
    (*pdata).mm_config_timeout_us = 2000 as libc::c_int as u32;
    (*pdata).phasecal_num_of_samples = 16 as libc::c_int as u16;
    (*pdata).zone_num_of_samples = 8 as libc::c_int as u16;
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_init_hist_post_process_config_struct(
    mut xtalk_compensation_enable: u8,
    mut pdata: *mut VL53LX_hist_post_process_config_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    (*pdata).hist_algo_select = 4 as libc::c_int as u8;
    (*pdata).hist_target_order = 1 as libc::c_int as u8;
    (*pdata).filter_woi0 = 1 as libc::c_int as u8;
    (*pdata).filter_woi1 = 2 as libc::c_int as u8;
    (*pdata).hist_amb_est_method = 1 as libc::c_int as u8;
    (*pdata).ambient_thresh_sigma0 = 80 as libc::c_int as u8;
    (*pdata).ambient_thresh_sigma1 = 100 as libc::c_int as u8;
    (*pdata).ambient_thresh_events_scaler = 4157 as libc::c_int as u16;
    (*pdata).min_ambient_thresh_events = 16 as libc::c_int;
    (*pdata).noise_threshold = 50 as libc::c_int as u16;
    (*pdata).signal_total_events_limit = 100 as libc::c_int;
    (*pdata).sigma_estimator__sigma_ref_mm = 1 as libc::c_int as u8;
    (*pdata).sigma_thresh = 180 as libc::c_int as u16;
    (*pdata).range_offset_mm = 0 as libc::c_int as i16;
    (*pdata).gain_factor = 1987 as libc::c_int as u16;
    (*pdata).valid_phase_low = 0x8 as libc::c_int as u8;
    (*pdata).valid_phase_high = 0x88 as libc::c_int as u8;
    (*pdata).algo__consistency_check__phase_tolerance = 8 as libc::c_int as u8;
    (*pdata).algo__consistency_check__event_sigma = 0 as libc::c_int as u8;
    (*pdata)
        .algo__consistency_check__event_min_spad_count = 2048 as libc::c_int as u16;
    (*pdata).algo__consistency_check__min_max_tolerance = 0 as libc::c_int as u16;
    (*pdata).algo__crosstalk_compensation_enable = xtalk_compensation_enable;
    (*pdata).algo__crosstalk_detect_min_valid_range_mm = -(50 as libc::c_int) as i16;
    (*pdata).algo__crosstalk_detect_max_valid_range_mm = 50 as libc::c_int as i16;
    (*pdata).algo__crosstalk_detect_max_valid_rate_kcps = 400 as libc::c_int as u16;
    (*pdata).algo__crosstalk_detect_max_sigma_mm = 140 as libc::c_int as u16;
    (*pdata).algo__crosstalk_detect_event_sigma = 80 as libc::c_int as u8;
    (*pdata).algo__crosstalk_detect_min_max_tolerance = 50 as libc::c_int as u16;
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_init_dmax_calibration_data_struct(
    mut pdata: *mut VL53LX_dmax_calibration_data_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    (*pdata).ref__actual_effective_spads = 0x5f2d as libc::c_int as u16;
    (*pdata).ref__peak_signal_count_rate_mcps = 0x844 as libc::c_int as u16;
    (*pdata).ref__distance_mm = 0x8a5 as libc::c_int as u16;
    (*pdata).ref_reflectance_pc = 0x14 as libc::c_int as u16;
    (*pdata).coverglass_transmission = 0x100 as libc::c_int as u16;
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_init_tuning_parm_storage_struct(
    mut pdata: *mut VL53LX_tuning_parm_storage_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    (*pdata).tp_tuning_parm_version = 30 as libc::c_int as u16;
    (*pdata).tp_tuning_parm_key_table_version = 14 as libc::c_int as u16;
    (*pdata).tp_tuning_parm_lld_version = 12180 as libc::c_int as u16;
    (*pdata).tp_init_phase_rtn_lite_long = 14 as libc::c_int as u8;
    (*pdata).tp_init_phase_rtn_lite_med = 10 as libc::c_int as u8;
    (*pdata).tp_init_phase_rtn_lite_short = 6 as libc::c_int as u8;
    (*pdata).tp_init_phase_ref_lite_long = 14 as libc::c_int as u8;
    (*pdata).tp_init_phase_ref_lite_med = 10 as libc::c_int as u8;
    (*pdata).tp_init_phase_ref_lite_short = 6 as libc::c_int as u8;
    (*pdata).tp_init_phase_rtn_hist_long = 9 as libc::c_int as u8;
    (*pdata).tp_init_phase_rtn_hist_med = 5 as libc::c_int as u8;
    (*pdata).tp_init_phase_rtn_hist_short = 3 as libc::c_int as u8;
    (*pdata).tp_init_phase_ref_hist_long = 6 as libc::c_int as u8;
    (*pdata).tp_init_phase_ref_hist_med = 6 as libc::c_int as u8;
    (*pdata).tp_init_phase_ref_hist_short = 6 as libc::c_int as u8;
    (*pdata).tp_consistency_lite_phase_tolerance = 2 as libc::c_int as u8;
    (*pdata).tp_phasecal_target = 33 as libc::c_int as u8;
    (*pdata).tp_cal_repeat_rate = 0 as libc::c_int as u16;
    (*pdata).tp_lite_min_clip = 0 as libc::c_int as u8;
    (*pdata).tp_lite_long_sigma_thresh_mm = 60 as libc::c_int as u16;
    (*pdata).tp_lite_med_sigma_thresh_mm = 60 as libc::c_int as u16;
    (*pdata).tp_lite_short_sigma_thresh_mm = 60 as libc::c_int as u16;
    (*pdata).tp_lite_long_min_count_rate_rtn_mcps = 128 as libc::c_int as u16;
    (*pdata).tp_lite_med_min_count_rate_rtn_mcps = 128 as libc::c_int as u16;
    (*pdata).tp_lite_short_min_count_rate_rtn_mcps = 128 as libc::c_int as u16;
    (*pdata).tp_lite_sigma_est_pulse_width_ns = 8 as libc::c_int as u8;
    (*pdata).tp_lite_sigma_est_amb_width_ns = 16 as libc::c_int as u8;
    (*pdata).tp_lite_sigma_ref_mm = 1 as libc::c_int as u8;
    (*pdata).tp_lite_seed_cfg = 2 as libc::c_int as u8;
    (*pdata).tp_timed_seed_cfg = 1 as libc::c_int as u8;
    (*pdata).tp_lite_quantifier = 2 as libc::c_int as u8;
    (*pdata).tp_lite_first_order_select = 0 as libc::c_int as u8;
    (*pdata).tp_uwr_enable = 1 as libc::c_int as u8;
    (*pdata).tp_uwr_med_z_1_min = 2000 as libc::c_int as i16;
    (*pdata).tp_uwr_med_z_1_max = 2750 as libc::c_int as i16;
    (*pdata).tp_uwr_med_z_2_min = 250 as libc::c_int as i16;
    (*pdata).tp_uwr_med_z_2_max = 1000 as libc::c_int as i16;
    (*pdata).tp_uwr_med_z_3_min = 1250 as libc::c_int as i16;
    (*pdata).tp_uwr_med_z_3_max = 1750 as libc::c_int as i16;
    (*pdata).tp_uwr_med_z_4_min = 1250 as libc::c_int as i16;
    (*pdata).tp_uwr_med_z_4_max = 1750 as libc::c_int as i16;
    (*pdata).tp_uwr_med_z_5_min = -(200 as libc::c_int) as i16;
    (*pdata).tp_uwr_med_z_5_max = 200 as libc::c_int as i16;
    (*pdata).tp_uwr_med_corr_z_1_rangea = 2300 as libc::c_int as i16;
    (*pdata).tp_uwr_med_corr_z_1_rangeb = 0 as libc::c_int as i16;
    (*pdata).tp_uwr_med_corr_z_2_rangea = 2300 as libc::c_int as i16;
    (*pdata).tp_uwr_med_corr_z_2_rangeb = 3050 as libc::c_int as i16;
    (*pdata).tp_uwr_med_corr_z_3_rangea = 4600 as libc::c_int as i16;
    (*pdata).tp_uwr_med_corr_z_3_rangeb = 3050 as libc::c_int as i16;
    (*pdata).tp_uwr_med_corr_z_4_rangea = 4600 as libc::c_int as i16;
    (*pdata).tp_uwr_med_corr_z_4_rangeb = 6200 as libc::c_int as i16;
    (*pdata).tp_uwr_med_corr_z_5_rangea = 0 as libc::c_int as i16;
    (*pdata).tp_uwr_med_corr_z_5_rangeb = 0 as libc::c_int as i16;
    (*pdata).tp_uwr_lng_z_1_min = 250 as libc::c_int as i16;
    (*pdata).tp_uwr_lng_z_1_max = 1250 as libc::c_int as i16;
    (*pdata).tp_uwr_lng_z_2_min = 3250 as libc::c_int as i16;
    (*pdata).tp_uwr_lng_z_2_max = 4500 as libc::c_int as i16;
    (*pdata).tp_uwr_lng_z_3_min = -(200 as libc::c_int) as i16;
    (*pdata).tp_uwr_lng_z_3_max = 200 as libc::c_int as i16;
    (*pdata).tp_uwr_lng_z_4_min = 0 as libc::c_int as i16;
    (*pdata).tp_uwr_lng_z_4_max = 0 as libc::c_int as i16;
    (*pdata).tp_uwr_lng_z_5_min = 0 as libc::c_int as i16;
    (*pdata).tp_uwr_lng_z_5_max = 0 as libc::c_int as i16;
    (*pdata).tp_uwr_lng_corr_z_1_rangea = 3850 as libc::c_int as i16;
    (*pdata).tp_uwr_lng_corr_z_1_rangeb = 4600 as libc::c_int as i16;
    (*pdata).tp_uwr_lng_corr_z_2_rangea = 3850 as libc::c_int as i16;
    (*pdata).tp_uwr_lng_corr_z_2_rangeb = 0 as libc::c_int as i16;
    (*pdata).tp_uwr_lng_corr_z_3_rangea = 0 as libc::c_int as i16;
    (*pdata).tp_uwr_lng_corr_z_3_rangeb = 0 as libc::c_int as i16;
    (*pdata).tp_uwr_lng_corr_z_4_rangea = 0 as libc::c_int as i16;
    (*pdata).tp_uwr_lng_corr_z_4_rangeb = 0 as libc::c_int as i16;
    (*pdata).tp_uwr_lng_corr_z_5_rangea = 0 as libc::c_int as i16;
    (*pdata).tp_uwr_lng_corr_z_5_rangeb = 0 as libc::c_int as i16;
    (*pdata).tp_dss_target_lite_mcps = 2560 as libc::c_int as u16;
    (*pdata).tp_dss_target_histo_mcps = 5120 as libc::c_int as u16;
    (*pdata).tp_dss_target_histo_mz_mcps = 5120 as libc::c_int as u16;
    (*pdata).tp_dss_target_timed_mcps = 2560 as libc::c_int as u16;
    (*pdata).tp_phasecal_timeout_lite_us = 1000 as libc::c_int as u32;
    (*pdata).tp_phasecal_timeout_hist_long_us = 15000 as libc::c_int as u32;
    (*pdata).tp_phasecal_timeout_hist_med_us = 9000 as libc::c_int as u32;
    (*pdata).tp_phasecal_timeout_hist_short_us = 6000 as libc::c_int as u32;
    (*pdata).tp_phasecal_timeout_mz_long_us = 15000 as libc::c_int as u32;
    (*pdata).tp_phasecal_timeout_mz_med_us = 9000 as libc::c_int as u32;
    (*pdata).tp_phasecal_timeout_mz_short_us = 6000 as libc::c_int as u32;
    (*pdata).tp_phasecal_timeout_timed_us = 1000 as libc::c_int as u32;
    (*pdata).tp_mm_timeout_lite_us = 2000 as libc::c_int as u32;
    (*pdata).tp_mm_timeout_histo_us = 2000 as libc::c_int as u32;
    (*pdata).tp_mm_timeout_mz_us = 2000 as libc::c_int as u32;
    (*pdata).tp_mm_timeout_timed_us = 2000 as libc::c_int as u32;
    (*pdata).tp_range_timeout_lite_us = 63000 as libc::c_int as u32;
    (*pdata).tp_range_timeout_histo_us = 2500 as libc::c_int as u32;
    (*pdata).tp_range_timeout_mz_us = 2500 as libc::c_int as u32;
    (*pdata).tp_range_timeout_timed_us = 13000 as libc::c_int as u32;
    (*pdata).tp_mm_timeout_lpa_us = 1 as libc::c_int as u32;
    (*pdata).tp_range_timeout_lpa_us = 8000 as libc::c_int as u32;
    (*pdata).tp_dss_target_very_short_mcps = 10240 as libc::c_int as u16;
    (*pdata).tp_phasecal_patch_power = 0 as libc::c_int as u32;
    (*pdata).tp_hist_merge = 1 as libc::c_int as u8;
    (*pdata).tp_reset_merge_threshold = 15000 as libc::c_int as u32;
    (*pdata).tp_hist_merge_max_size = 6 as libc::c_int as u8;
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_init_hist_gen3_dmax_config_struct(
    mut pdata: *mut VL53LX_hist_gen3_dmax_config_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    (*pdata).dss_config__target_total_rate_mcps = 0x1400 as libc::c_int as u16;
    (*pdata).dss_config__aperture_attenuation = 0x38 as libc::c_int as u8;
    (*pdata).signal_thresh_sigma = 32 as libc::c_int as u8;
    (*pdata).ambient_thresh_sigma = 0x70 as libc::c_int as u8;
    (*pdata).min_ambient_thresh_events = 16 as libc::c_int;
    (*pdata).signal_total_events_limit = 100 as libc::c_int;
    (*pdata).max_effective_spads = 0xffff as libc::c_int as u16;
    (*pdata)
        .target_reflectance_for_dmax_calc[0 as libc::c_int
        as usize] = 15 as libc::c_int as u16;
    (*pdata)
        .target_reflectance_for_dmax_calc[1 as libc::c_int
        as usize] = 52 as libc::c_int as u16;
    (*pdata)
        .target_reflectance_for_dmax_calc[2 as libc::c_int
        as usize] = 200 as libc::c_int as u16;
    (*pdata)
        .target_reflectance_for_dmax_calc[3 as libc::c_int
        as usize] = 364 as libc::c_int as u16;
    (*pdata)
        .target_reflectance_for_dmax_calc[4 as libc::c_int
        as usize] = 400 as libc::c_int as u16;
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_preset_mode_standard_ranging(
    mut pstatic: *mut VL53LX_static_config_t,
    mut phistogram: *mut VL53LX_histogram_config_t,
    mut pgeneral: *mut VL53LX_general_config_t,
    mut ptiming: *mut VL53LX_timing_config_t,
    mut pdynamic: *mut VL53LX_dynamic_config_t,
    mut psystem: *mut VL53LX_system_control_t,
    mut ptuning_parms: *mut VL53LX_tuning_parm_storage_t,
    mut pzone_cfg: *mut VL53LX_zone_config_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    (*pstatic).dss_config__target_total_rate_mcps = 0xa00 as libc::c_int as u16;
    (*pstatic).debug__ctrl = 0 as libc::c_int as u8;
    (*pstatic).test_mode__ctrl = 0 as libc::c_int as u8;
    (*pstatic).clk_gating__ctrl = 0 as libc::c_int as u8;
    (*pstatic).nvm_bist__ctrl = 0 as libc::c_int as u8;
    (*pstatic).nvm_bist__num_nvm_words = 0 as libc::c_int as u8;
    (*pstatic).nvm_bist__start_address = 0 as libc::c_int as u8;
    (*pstatic).host_if__status = 0 as libc::c_int as u8;
    (*pstatic).pad_i2c_hv__config = 0 as libc::c_int as u8;
    (*pstatic).pad_i2c_hv__extsup_config = 0 as libc::c_int as u8;
    (*pstatic).gpio_hv_pad__ctrl = 0 as libc::c_int as u8;
    (*pstatic)
        .gpio_hv_mux__ctrl = (0x10 as libc::c_int as VL53LX_DeviceInterruptPolarity
        as libc::c_int | 0x1 as libc::c_int as VL53LX_DeviceGpioMode as libc::c_int)
        as u8;
    (*pstatic).gpio__tio_hv_status = 0x2 as libc::c_int as u8;
    (*pstatic).gpio__fio_hv_status = 0 as libc::c_int as u8;
    (*pstatic).ana_config__spad_sel_pswidth = 0x2 as libc::c_int as u8;
    (*pstatic).ana_config__vcsel_pulse_width_offset = 0x8 as libc::c_int as u8;
    (*pstatic).ana_config__fast_osc__config_ctrl = 0 as libc::c_int as u8;
    (*pstatic)
        .sigma_estimator__effective_pulse_width_ns = (*ptuning_parms)
        .tp_lite_sigma_est_pulse_width_ns;
    (*pstatic)
        .sigma_estimator__effective_ambient_width_ns = (*ptuning_parms)
        .tp_lite_sigma_est_amb_width_ns;
    (*pstatic).sigma_estimator__sigma_ref_mm = (*ptuning_parms).tp_lite_sigma_ref_mm;
    (*pstatic)
        .algo__crosstalk_compensation_valid_height_mm = 0x1 as libc::c_int as u8;
    (*pstatic).spare_host_config__static_config_spare_0 = 0 as libc::c_int as u8;
    (*pstatic).spare_host_config__static_config_spare_1 = 0 as libc::c_int as u8;
    (*pstatic).algo__range_ignore_threshold_mcps = 0 as libc::c_int as u16;
    (*pstatic).algo__range_ignore_valid_height_mm = 0xff as libc::c_int as u8;
    (*pstatic).algo__range_min_clip = (*ptuning_parms).tp_lite_min_clip;
    (*pstatic)
        .algo__consistency_check__tolerance = (*ptuning_parms)
        .tp_consistency_lite_phase_tolerance;
    (*pstatic).spare_host_config__static_config_spare_2 = 0 as libc::c_int as u8;
    (*pstatic).sd_config__reset_stages_msb = 0 as libc::c_int as u8;
    (*pstatic).sd_config__reset_stages_lsb = 0 as libc::c_int as u8;
    (*pgeneral).gph_config__stream_count_update_value = 0 as libc::c_int as u8;
    (*pgeneral).global_config__stream_divider = 0 as libc::c_int as u8;
    (*pgeneral).system__interrupt_config_gpio = 0x20 as libc::c_int as u8;
    (*pgeneral).cal_config__vcsel_start = 0xb as libc::c_int as u8;
    (*pgeneral).cal_config__repeat_rate = (*ptuning_parms).tp_cal_repeat_rate;
    (*pgeneral).global_config__vcsel_width = 0x2 as libc::c_int as u8;
    (*pgeneral).phasecal_config__timeout_macrop = 0xd as libc::c_int as u8;
    (*pgeneral).phasecal_config__target = (*ptuning_parms).tp_phasecal_target;
    (*pgeneral).phasecal_config__override = 0 as libc::c_int as u8;
    (*pgeneral).dss_config__roi_mode_control = 1 as libc::c_int as VL53LX_DeviceDssMode;
    (*pgeneral).system__thresh_rate_high = 0 as libc::c_int as u16;
    (*pgeneral).system__thresh_rate_low = 0 as libc::c_int as u16;
    (*pgeneral)
        .dss_config__manual_effective_spads_select = 0x8c00 as libc::c_int as u16;
    (*pgeneral).dss_config__manual_block_select = 0 as libc::c_int as u8;
    (*pgeneral).dss_config__aperture_attenuation = 0x38 as libc::c_int as u8;
    (*pgeneral).dss_config__max_spads_limit = 0xff as libc::c_int as u8;
    (*pgeneral).dss_config__min_spads_limit = 0x1 as libc::c_int as u8;
    (*ptiming).mm_config__timeout_macrop_a_hi = 0 as libc::c_int as u8;
    (*ptiming).mm_config__timeout_macrop_a_lo = 0x1a as libc::c_int as u8;
    (*ptiming).mm_config__timeout_macrop_b_hi = 0 as libc::c_int as u8;
    (*ptiming).mm_config__timeout_macrop_b_lo = 0x20 as libc::c_int as u8;
    (*ptiming).range_config__timeout_macrop_a_hi = 0x1 as libc::c_int as u8;
    (*ptiming).range_config__timeout_macrop_a_lo = 0xcc as libc::c_int as u8;
    (*ptiming).range_config__vcsel_period_a = 0xb as libc::c_int as u8;
    (*ptiming).range_config__timeout_macrop_b_hi = 0x1 as libc::c_int as u8;
    (*ptiming).range_config__timeout_macrop_b_lo = 0xf5 as libc::c_int as u8;
    (*ptiming).range_config__vcsel_period_b = 0x9 as libc::c_int as u8;
    (*ptiming).range_config__sigma_thresh = (*ptuning_parms).tp_lite_med_sigma_thresh_mm;
    (*ptiming)
        .range_config__min_count_rate_rtn_limit_mcps = (*ptuning_parms)
        .tp_lite_med_min_count_rate_rtn_mcps;
    (*ptiming).range_config__valid_phase_low = 0x8 as libc::c_int as u8;
    (*ptiming).range_config__valid_phase_high = 0x78 as libc::c_int as u8;
    (*ptiming).system__intermeasurement_period = 0 as libc::c_int as u32;
    (*ptiming).system__fractional_enable = 0 as libc::c_int as u8;
    (*phistogram).histogram_config__low_amb_even_bin_0_1 = 0x7 as libc::c_int as u8;
    (*phistogram)
        .histogram_config__low_amb_even_bin_2_3 = 0x21 as libc::c_int as u8;
    (*phistogram)
        .histogram_config__low_amb_even_bin_4_5 = 0x43 as libc::c_int as u8;
    (*phistogram).histogram_config__low_amb_odd_bin_0_1 = 0x10 as libc::c_int as u8;
    (*phistogram).histogram_config__low_amb_odd_bin_2_3 = 0x32 as libc::c_int as u8;
    (*phistogram).histogram_config__low_amb_odd_bin_4_5 = 0x54 as libc::c_int as u8;
    (*phistogram).histogram_config__mid_amb_even_bin_0_1 = 0x7 as libc::c_int as u8;
    (*phistogram)
        .histogram_config__mid_amb_even_bin_2_3 = 0x21 as libc::c_int as u8;
    (*phistogram)
        .histogram_config__mid_amb_even_bin_4_5 = 0x43 as libc::c_int as u8;
    (*phistogram).histogram_config__mid_amb_odd_bin_0_1 = 0x10 as libc::c_int as u8;
    (*phistogram).histogram_config__mid_amb_odd_bin_2 = 0x2 as libc::c_int as u8;
    (*phistogram).histogram_config__mid_amb_odd_bin_3_4 = 0x43 as libc::c_int as u8;
    (*phistogram).histogram_config__mid_amb_odd_bin_5 = 0x5 as libc::c_int as u8;
    (*phistogram).histogram_config__user_bin_offset = 0 as libc::c_int as u8;
    (*phistogram)
        .histogram_config__high_amb_even_bin_0_1 = 0x7 as libc::c_int as u8;
    (*phistogram)
        .histogram_config__high_amb_even_bin_2_3 = 0x21 as libc::c_int as u8;
    (*phistogram)
        .histogram_config__high_amb_even_bin_4_5 = 0x43 as libc::c_int as u8;
    (*phistogram)
        .histogram_config__high_amb_odd_bin_0_1 = 0x10 as libc::c_int as u8;
    (*phistogram)
        .histogram_config__high_amb_odd_bin_2_3 = 0x32 as libc::c_int as u8;
    (*phistogram)
        .histogram_config__high_amb_odd_bin_4_5 = 0x54 as libc::c_int as u8;
    (*phistogram).histogram_config__amb_thresh_low = 0xffff as libc::c_int as u16;
    (*phistogram).histogram_config__amb_thresh_high = 0xffff as libc::c_int as u16;
    (*phistogram).histogram_config__spad_array_selection = 0 as libc::c_int as u8;
    (*pzone_cfg).max_zones = 5 as libc::c_int as u8;
    (*pzone_cfg).active_zones = 0 as libc::c_int as u8;
    (*pzone_cfg)
        .user_zones[0 as libc::c_int as usize]
        .height = 0xf as libc::c_int as u8;
    (*pzone_cfg)
        .user_zones[0 as libc::c_int as usize]
        .width = 0xf as libc::c_int as u8;
    (*pzone_cfg)
        .user_zones[0 as libc::c_int as usize]
        .x_centre = 0x8 as libc::c_int as u8;
    (*pzone_cfg)
        .user_zones[0 as libc::c_int as usize]
        .y_centre = 0x8 as libc::c_int as u8;
    (*pdynamic).system__grouped_parameter_hold_0 = 0x1 as libc::c_int as u8;
    (*pdynamic).system__thresh_high = 0 as libc::c_int as u16;
    (*pdynamic).system__thresh_low = 0 as libc::c_int as u16;
    (*pdynamic).system__enable_xtalk_per_quadrant = 0 as libc::c_int as u8;
    (*pdynamic).system__seed_config = (*ptuning_parms).tp_lite_seed_cfg;
    (*pdynamic).sd_config__woi_sd0 = 0xb as libc::c_int as u8;
    (*pdynamic).sd_config__woi_sd1 = 0x9 as libc::c_int as u8;
    (*pdynamic)
        .sd_config__initial_phase_sd0 = (*ptuning_parms).tp_init_phase_rtn_lite_med;
    (*pdynamic)
        .sd_config__initial_phase_sd1 = (*ptuning_parms).tp_init_phase_ref_lite_med;
    (*pdynamic).system__grouped_parameter_hold_1 = 0x1 as libc::c_int as u8;
    (*pdynamic)
        .sd_config__first_order_select = (*ptuning_parms).tp_lite_first_order_select;
    (*pdynamic).sd_config__quantifier = (*ptuning_parms).tp_lite_quantifier;
    (*pdynamic).roi_config__user_roi_centre_spad = 0xc7 as libc::c_int as u8;
    (*pdynamic)
        .roi_config__user_roi_requested_global_xy_size = 0xff as libc::c_int as u8;
    (*pdynamic)
        .system__sequence_config = (0x1 as libc::c_int | 0x2 as libc::c_int
        | 0x8 as libc::c_int | 0x10 as libc::c_int | 0x40 as libc::c_int
        | 0x80 as libc::c_int) as u8;
    (*pdynamic).system__grouped_parameter_hold = 0x2 as libc::c_int as u8;
    (*psystem).system__stream_count_ctrl = 0 as libc::c_int as u8;
    (*psystem).firmware__enable = 0x1 as libc::c_int as u8;
    (*psystem).system__interrupt_clear = 0x1 as libc::c_int as u8;
    (*psystem)
        .system__mode_start = (0x1 as libc::c_int
        | (0 as libc::c_int) << 2 as libc::c_int
        | 0x20 as libc::c_int as VL53LX_DeviceMeasurementModes as libc::c_int)
        as u8;
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_preset_mode_histogram_ranging(
    mut phistpostprocess: *mut VL53LX_hist_post_process_config_t,
    mut pstatic: *mut VL53LX_static_config_t,
    mut phistogram: *mut VL53LX_histogram_config_t,
    mut pgeneral: *mut VL53LX_general_config_t,
    mut ptiming: *mut VL53LX_timing_config_t,
    mut pdynamic: *mut VL53LX_dynamic_config_t,
    mut psystem: *mut VL53LX_system_control_t,
    mut ptuning_parms: *mut VL53LX_tuning_parm_storage_t,
    mut pzone_cfg: *mut VL53LX_zone_config_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    status = VL53LX_preset_mode_standard_ranging(
        pstatic,
        phistogram,
        pgeneral,
        ptiming,
        pdynamic,
        psystem,
        ptuning_parms,
        pzone_cfg,
    );
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        (*pstatic)
            .dss_config__target_total_rate_mcps = 0x1400 as libc::c_int as u16;
        VL53LX_init_histogram_config_structure(
            7 as libc::c_int as u8,
            0 as libc::c_int as u8,
            1 as libc::c_int as u8,
            2 as libc::c_int as u8,
            3 as libc::c_int as u8,
            4 as libc::c_int as u8,
            0 as libc::c_int as u8,
            1 as libc::c_int as u8,
            2 as libc::c_int as u8,
            3 as libc::c_int as u8,
            4 as libc::c_int as u8,
            5 as libc::c_int as u8,
            phistogram,
        );
        VL53LX_init_histogram_multizone_config_structure(
            7 as libc::c_int as u8,
            0 as libc::c_int as u8,
            1 as libc::c_int as u8,
            2 as libc::c_int as u8,
            3 as libc::c_int as u8,
            4 as libc::c_int as u8,
            0 as libc::c_int as u8,
            1 as libc::c_int as u8,
            2 as libc::c_int as u8,
            3 as libc::c_int as u8,
            4 as libc::c_int as u8,
            5 as libc::c_int as u8,
            &mut (*pzone_cfg).multizone_hist_cfg,
        );
        (*ptiming).range_config__vcsel_period_a = 0x9 as libc::c_int as u8;
        (*ptiming).range_config__vcsel_period_b = 0xb as libc::c_int as u8;
        (*pdynamic).sd_config__woi_sd0 = 0x9 as libc::c_int as u8;
        (*pdynamic).sd_config__woi_sd1 = 0xb as libc::c_int as u8;
        (*ptiming).mm_config__timeout_macrop_a_hi = 0 as libc::c_int as u8;
        (*ptiming).mm_config__timeout_macrop_a_lo = 0x20 as libc::c_int as u8;
        (*ptiming).mm_config__timeout_macrop_b_hi = 0 as libc::c_int as u8;
        (*ptiming).mm_config__timeout_macrop_b_lo = 0x1a as libc::c_int as u8;
        (*ptiming).range_config__timeout_macrop_a_hi = 0 as libc::c_int as u8;
        (*ptiming).range_config__timeout_macrop_a_lo = 0x28 as libc::c_int as u8;
        (*ptiming).range_config__timeout_macrop_b_hi = 0 as libc::c_int as u8;
        (*ptiming).range_config__timeout_macrop_b_lo = 0x21 as libc::c_int as u8;
        (*pgeneral).phasecal_config__timeout_macrop = 0xf5 as libc::c_int as u8;
        (*phistpostprocess).valid_phase_low = 0x8 as libc::c_int as u8;
        (*phistpostprocess).valid_phase_high = 0x88 as libc::c_int as u8;
        VL53LX_copy_hist_cfg_to_static_cfg(
            phistogram,
            pstatic,
            pgeneral,
            ptiming,
            pdynamic,
        );
        (*pdynamic)
            .system__sequence_config = (0x1 as libc::c_int | 0x2 as libc::c_int
            | 0x8 as libc::c_int | 0x10 as libc::c_int | 0x80 as libc::c_int) as u8;
        (*psystem)
            .system__mode_start = (0x2 as libc::c_int
            | (0x1 as libc::c_int) << 2 as libc::c_int
            | 0x20 as libc::c_int as VL53LX_DeviceMeasurementModes as libc::c_int)
            as u8;
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_preset_mode_histogram_long_range(
    mut phistpostprocess: *mut VL53LX_hist_post_process_config_t,
    mut pstatic: *mut VL53LX_static_config_t,
    mut phistogram: *mut VL53LX_histogram_config_t,
    mut pgeneral: *mut VL53LX_general_config_t,
    mut ptiming: *mut VL53LX_timing_config_t,
    mut pdynamic: *mut VL53LX_dynamic_config_t,
    mut psystem: *mut VL53LX_system_control_t,
    mut ptuning_parms: *mut VL53LX_tuning_parm_storage_t,
    mut pzone_cfg: *mut VL53LX_zone_config_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    status = VL53LX_preset_mode_histogram_ranging(
        phistpostprocess,
        pstatic,
        phistogram,
        pgeneral,
        ptiming,
        pdynamic,
        psystem,
        ptuning_parms,
        pzone_cfg,
    );
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        VL53LX_init_histogram_config_structure(
            7 as libc::c_int as u8,
            0 as libc::c_int as u8,
            1 as libc::c_int as u8,
            2 as libc::c_int as u8,
            3 as libc::c_int as u8,
            4 as libc::c_int as u8,
            0 as libc::c_int as u8,
            1 as libc::c_int as u8,
            2 as libc::c_int as u8,
            3 as libc::c_int as u8,
            4 as libc::c_int as u8,
            5 as libc::c_int as u8,
            phistogram,
        );
        VL53LX_init_histogram_multizone_config_structure(
            7 as libc::c_int as u8,
            0 as libc::c_int as u8,
            1 as libc::c_int as u8,
            2 as libc::c_int as u8,
            3 as libc::c_int as u8,
            4 as libc::c_int as u8,
            0 as libc::c_int as u8,
            1 as libc::c_int as u8,
            2 as libc::c_int as u8,
            3 as libc::c_int as u8,
            4 as libc::c_int as u8,
            5 as libc::c_int as u8,
            &mut (*pzone_cfg).multizone_hist_cfg,
        );
        VL53LX_copy_hist_cfg_to_static_cfg(
            phistogram,
            pstatic,
            pgeneral,
            ptiming,
            pdynamic,
        );
        (*ptiming).range_config__vcsel_period_a = 0x9 as libc::c_int as u8;
        (*ptiming).range_config__vcsel_period_b = 0xb as libc::c_int as u8;
        (*ptiming).mm_config__timeout_macrop_a_hi = 0 as libc::c_int as u8;
        (*ptiming).mm_config__timeout_macrop_a_lo = 0x21 as libc::c_int as u8;
        (*ptiming).mm_config__timeout_macrop_b_hi = 0 as libc::c_int as u8;
        (*ptiming).mm_config__timeout_macrop_b_lo = 0x1b as libc::c_int as u8;
        (*ptiming).range_config__timeout_macrop_a_hi = 0 as libc::c_int as u8;
        (*ptiming).range_config__timeout_macrop_a_lo = 0x29 as libc::c_int as u8;
        (*ptiming).range_config__timeout_macrop_b_hi = 0 as libc::c_int as u8;
        (*ptiming).range_config__timeout_macrop_b_lo = 0x22 as libc::c_int as u8;
        (*pgeneral).cal_config__vcsel_start = 0x9 as libc::c_int as u8;
        (*pgeneral).phasecal_config__timeout_macrop = 0xf5 as libc::c_int as u8;
        (*pdynamic).sd_config__woi_sd0 = 0x9 as libc::c_int as u8;
        (*pdynamic).sd_config__woi_sd1 = 0xb as libc::c_int as u8;
        (*pdynamic)
            .sd_config__initial_phase_sd0 = (*ptuning_parms).tp_init_phase_rtn_hist_long;
        (*pdynamic)
            .sd_config__initial_phase_sd1 = (*ptuning_parms).tp_init_phase_ref_hist_long;
        (*phistpostprocess).valid_phase_low = 0x8 as libc::c_int as u8;
        (*phistpostprocess).valid_phase_high = 0x88 as libc::c_int as u8;
        (*pdynamic)
            .system__sequence_config = (0x1 as libc::c_int | 0x2 as libc::c_int
            | 0x8 as libc::c_int | 0x10 as libc::c_int | 0x80 as libc::c_int) as u8;
        (*psystem)
            .system__mode_start = (0x2 as libc::c_int
            | (0x1 as libc::c_int) << 2 as libc::c_int
            | 0x20 as libc::c_int as VL53LX_DeviceMeasurementModes as libc::c_int)
            as u8;
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_preset_mode_histogram_medium_range(
    mut phistpostprocess: *mut VL53LX_hist_post_process_config_t,
    mut pstatic: *mut VL53LX_static_config_t,
    mut phistogram: *mut VL53LX_histogram_config_t,
    mut pgeneral: *mut VL53LX_general_config_t,
    mut ptiming: *mut VL53LX_timing_config_t,
    mut pdynamic: *mut VL53LX_dynamic_config_t,
    mut psystem: *mut VL53LX_system_control_t,
    mut ptuning_parms: *mut VL53LX_tuning_parm_storage_t,
    mut pzone_cfg: *mut VL53LX_zone_config_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    status = VL53LX_preset_mode_histogram_ranging(
        phistpostprocess,
        pstatic,
        phistogram,
        pgeneral,
        ptiming,
        pdynamic,
        psystem,
        ptuning_parms,
        pzone_cfg,
    );
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        VL53LX_init_histogram_config_structure(
            7 as libc::c_int as u8,
            0 as libc::c_int as u8,
            1 as libc::c_int as u8,
            1 as libc::c_int as u8,
            2 as libc::c_int as u8,
            2 as libc::c_int as u8,
            0 as libc::c_int as u8,
            1 as libc::c_int as u8,
            2 as libc::c_int as u8,
            1 as libc::c_int as u8,
            2 as libc::c_int as u8,
            3 as libc::c_int as u8,
            phistogram,
        );
        VL53LX_init_histogram_multizone_config_structure(
            7 as libc::c_int as u8,
            0 as libc::c_int as u8,
            1 as libc::c_int as u8,
            1 as libc::c_int as u8,
            2 as libc::c_int as u8,
            2 as libc::c_int as u8,
            0 as libc::c_int as u8,
            1 as libc::c_int as u8,
            2 as libc::c_int as u8,
            1 as libc::c_int as u8,
            2 as libc::c_int as u8,
            3 as libc::c_int as u8,
            &mut (*pzone_cfg).multizone_hist_cfg,
        );
        VL53LX_copy_hist_cfg_to_static_cfg(
            phistogram,
            pstatic,
            pgeneral,
            ptiming,
            pdynamic,
        );
        (*ptiming).range_config__vcsel_period_a = 0x5 as libc::c_int as u8;
        (*ptiming).range_config__vcsel_period_b = 0x7 as libc::c_int as u8;
        (*ptiming).mm_config__timeout_macrop_a_hi = 0 as libc::c_int as u8;
        (*ptiming).mm_config__timeout_macrop_a_lo = 0x36 as libc::c_int as u8;
        (*ptiming).mm_config__timeout_macrop_b_hi = 0 as libc::c_int as u8;
        (*ptiming).mm_config__timeout_macrop_b_lo = 0x28 as libc::c_int as u8;
        (*ptiming).range_config__timeout_macrop_a_hi = 0 as libc::c_int as u8;
        (*ptiming).range_config__timeout_macrop_a_lo = 0x44 as libc::c_int as u8;
        (*ptiming).range_config__timeout_macrop_b_hi = 0 as libc::c_int as u8;
        (*ptiming).range_config__timeout_macrop_b_lo = 0x33 as libc::c_int as u8;
        (*pgeneral).cal_config__vcsel_start = 0x5 as libc::c_int as u8;
        (*pgeneral).phasecal_config__timeout_macrop = 0xf5 as libc::c_int as u8;
        (*pdynamic).sd_config__woi_sd0 = 0x5 as libc::c_int as u8;
        (*pdynamic).sd_config__woi_sd1 = 0x7 as libc::c_int as u8;
        (*pdynamic)
            .sd_config__initial_phase_sd0 = (*ptuning_parms).tp_init_phase_rtn_hist_med;
        (*pdynamic)
            .sd_config__initial_phase_sd1 = (*ptuning_parms).tp_init_phase_ref_hist_med;
        (*phistpostprocess).valid_phase_low = 0x8 as libc::c_int as u8;
        (*phistpostprocess).valid_phase_high = 0x48 as libc::c_int as u8;
        (*pdynamic)
            .system__sequence_config = (0x1 as libc::c_int | 0x2 as libc::c_int
            | 0x8 as libc::c_int | 0x10 as libc::c_int | 0x80 as libc::c_int) as u8;
        (*psystem)
            .system__mode_start = (0x2 as libc::c_int
            | (0x1 as libc::c_int) << 2 as libc::c_int
            | 0x20 as libc::c_int as VL53LX_DeviceMeasurementModes as libc::c_int)
            as u8;
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_preset_mode_histogram_short_range(
    mut phistpostprocess: *mut VL53LX_hist_post_process_config_t,
    mut pstatic: *mut VL53LX_static_config_t,
    mut phistogram: *mut VL53LX_histogram_config_t,
    mut pgeneral: *mut VL53LX_general_config_t,
    mut ptiming: *mut VL53LX_timing_config_t,
    mut pdynamic: *mut VL53LX_dynamic_config_t,
    mut psystem: *mut VL53LX_system_control_t,
    mut ptuning_parms: *mut VL53LX_tuning_parm_storage_t,
    mut pzone_cfg: *mut VL53LX_zone_config_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    status = VL53LX_preset_mode_histogram_ranging(
        phistpostprocess,
        pstatic,
        phistogram,
        pgeneral,
        ptiming,
        pdynamic,
        psystem,
        ptuning_parms,
        pzone_cfg,
    );
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        VL53LX_init_histogram_config_structure(
            7 as libc::c_int as u8,
            7 as libc::c_int as u8,
            0 as libc::c_int as u8,
            1 as libc::c_int as u8,
            1 as libc::c_int as u8,
            1 as libc::c_int as u8,
            0 as libc::c_int as u8,
            1 as libc::c_int as u8,
            1 as libc::c_int as u8,
            1 as libc::c_int as u8,
            2 as libc::c_int as u8,
            2 as libc::c_int as u8,
            phistogram,
        );
        VL53LX_init_histogram_multizone_config_structure(
            7 as libc::c_int as u8,
            7 as libc::c_int as u8,
            0 as libc::c_int as u8,
            1 as libc::c_int as u8,
            1 as libc::c_int as u8,
            1 as libc::c_int as u8,
            0 as libc::c_int as u8,
            1 as libc::c_int as u8,
            1 as libc::c_int as u8,
            1 as libc::c_int as u8,
            2 as libc::c_int as u8,
            2 as libc::c_int as u8,
            &mut (*pzone_cfg).multizone_hist_cfg,
        );
        VL53LX_copy_hist_cfg_to_static_cfg(
            phistogram,
            pstatic,
            pgeneral,
            ptiming,
            pdynamic,
        );
        (*ptiming).range_config__vcsel_period_a = 0x3 as libc::c_int as u8;
        (*ptiming).range_config__vcsel_period_b = 0x5 as libc::c_int as u8;
        (*ptiming).mm_config__timeout_macrop_a_hi = 0 as libc::c_int as u8;
        (*ptiming).mm_config__timeout_macrop_a_lo = 0x52 as libc::c_int as u8;
        (*ptiming).mm_config__timeout_macrop_b_hi = 0 as libc::c_int as u8;
        (*ptiming).mm_config__timeout_macrop_b_lo = 0x37 as libc::c_int as u8;
        (*ptiming).range_config__timeout_macrop_a_hi = 0 as libc::c_int as u8;
        (*ptiming).range_config__timeout_macrop_a_lo = 0x66 as libc::c_int as u8;
        (*ptiming).range_config__timeout_macrop_b_hi = 0 as libc::c_int as u8;
        (*ptiming).range_config__timeout_macrop_b_lo = 0x44 as libc::c_int as u8;
        (*pgeneral).cal_config__vcsel_start = 0x3 as libc::c_int as u8;
        (*pgeneral).phasecal_config__timeout_macrop = 0xf5 as libc::c_int as u8;
        (*pdynamic).sd_config__woi_sd0 = 0x3 as libc::c_int as u8;
        (*pdynamic).sd_config__woi_sd1 = 0x5 as libc::c_int as u8;
        (*pdynamic)
            .sd_config__initial_phase_sd0 = (*ptuning_parms)
            .tp_init_phase_rtn_hist_short;
        (*pdynamic)
            .sd_config__initial_phase_sd1 = (*ptuning_parms)
            .tp_init_phase_ref_hist_short;
        (*phistpostprocess).valid_phase_low = 0x8 as libc::c_int as u8;
        (*phistpostprocess).valid_phase_high = 0x28 as libc::c_int as u8;
        (*pdynamic)
            .system__sequence_config = (0x1 as libc::c_int | 0x2 as libc::c_int
            | 0x8 as libc::c_int | 0x10 as libc::c_int | 0x20 as libc::c_int
            | 0x80 as libc::c_int) as u8;
        (*psystem)
            .system__mode_start = (0x2 as libc::c_int
            | (0x1 as libc::c_int) << 2 as libc::c_int
            | 0x20 as libc::c_int as VL53LX_DeviceMeasurementModes as libc::c_int)
            as u8;
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_copy_hist_cfg_to_static_cfg(
    mut phistogram: *mut VL53LX_histogram_config_t,
    mut pstatic: *mut VL53LX_static_config_t,
    mut pgeneral: *mut VL53LX_general_config_t,
    mut ptiming: *mut VL53LX_timing_config_t,
    mut pdynamic: *mut VL53LX_dynamic_config_t,
) {
    (*pstatic)
        .sigma_estimator__effective_pulse_width_ns = (*phistogram)
        .histogram_config__high_amb_even_bin_0_1;
    (*pstatic)
        .sigma_estimator__effective_ambient_width_ns = (*phistogram)
        .histogram_config__high_amb_even_bin_2_3;
    (*pstatic)
        .sigma_estimator__sigma_ref_mm = (*phistogram)
        .histogram_config__high_amb_even_bin_4_5;
    (*pstatic)
        .algo__crosstalk_compensation_valid_height_mm = (*phistogram)
        .histogram_config__high_amb_odd_bin_0_1;
    (*pstatic)
        .spare_host_config__static_config_spare_0 = (*phistogram)
        .histogram_config__high_amb_odd_bin_2_3;
    (*pstatic)
        .spare_host_config__static_config_spare_1 = (*phistogram)
        .histogram_config__high_amb_odd_bin_4_5;
    (*pstatic)
        .algo__range_ignore_threshold_mcps = ((((*phistogram)
        .histogram_config__mid_amb_even_bin_0_1 as u16 as libc::c_int)
        << 8 as libc::c_int)
        + (*phistogram).histogram_config__mid_amb_even_bin_2_3 as u16
            as libc::c_int) as u16;
    (*pstatic)
        .algo__range_ignore_valid_height_mm = (*phistogram)
        .histogram_config__mid_amb_even_bin_4_5;
    (*pstatic)
        .algo__range_min_clip = (*phistogram).histogram_config__mid_amb_odd_bin_0_1;
    (*pstatic)
        .algo__consistency_check__tolerance = (*phistogram)
        .histogram_config__mid_amb_odd_bin_2;
    (*pstatic)
        .spare_host_config__static_config_spare_2 = (*phistogram)
        .histogram_config__mid_amb_odd_bin_3_4;
    (*pstatic)
        .sd_config__reset_stages_msb = (*phistogram).histogram_config__mid_amb_odd_bin_5;
    (*pstatic)
        .sd_config__reset_stages_lsb = (*phistogram).histogram_config__user_bin_offset;
    (*ptiming)
        .range_config__sigma_thresh = ((((*phistogram)
        .histogram_config__low_amb_even_bin_0_1 as u16 as libc::c_int)
        << 8 as libc::c_int)
        + (*phistogram).histogram_config__low_amb_even_bin_2_3 as u16
            as libc::c_int) as u16;
    (*ptiming)
        .range_config__min_count_rate_rtn_limit_mcps = ((((*phistogram)
        .histogram_config__low_amb_even_bin_4_5 as u16 as libc::c_int)
        << 8 as libc::c_int)
        + (*phistogram).histogram_config__low_amb_odd_bin_0_1 as u16 as libc::c_int)
        as u16;
    (*ptiming)
        .range_config__valid_phase_low = (*phistogram)
        .histogram_config__low_amb_odd_bin_2_3;
    (*ptiming)
        .range_config__valid_phase_high = (*phistogram)
        .histogram_config__low_amb_odd_bin_4_5;
    (*pdynamic).system__thresh_high = (*phistogram).histogram_config__amb_thresh_low;
    (*pdynamic).system__thresh_low = (*phistogram).histogram_config__amb_thresh_high;
    (*pdynamic)
        .system__enable_xtalk_per_quadrant = (*phistogram)
        .histogram_config__spad_array_selection;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_copy_hist_bins_to_static_cfg(
    mut phistogram: *mut VL53LX_histogram_config_t,
    mut pstatic: *mut VL53LX_static_config_t,
    mut ptiming: *mut VL53LX_timing_config_t,
) {
    (*pstatic)
        .sigma_estimator__effective_pulse_width_ns = (*phistogram)
        .histogram_config__high_amb_even_bin_0_1;
    (*pstatic)
        .sigma_estimator__effective_ambient_width_ns = (*phistogram)
        .histogram_config__high_amb_even_bin_2_3;
    (*pstatic)
        .sigma_estimator__sigma_ref_mm = (*phistogram)
        .histogram_config__high_amb_even_bin_4_5;
    (*pstatic)
        .algo__crosstalk_compensation_valid_height_mm = (*phistogram)
        .histogram_config__high_amb_odd_bin_0_1;
    (*pstatic)
        .spare_host_config__static_config_spare_0 = (*phistogram)
        .histogram_config__high_amb_odd_bin_2_3;
    (*pstatic)
        .spare_host_config__static_config_spare_1 = (*phistogram)
        .histogram_config__high_amb_odd_bin_4_5;
    (*pstatic)
        .algo__range_ignore_threshold_mcps = ((((*phistogram)
        .histogram_config__mid_amb_even_bin_0_1 as u16 as libc::c_int)
        << 8 as libc::c_int)
        + (*phistogram).histogram_config__mid_amb_even_bin_2_3 as u16
            as libc::c_int) as u16;
    (*pstatic)
        .algo__range_ignore_valid_height_mm = (*phistogram)
        .histogram_config__mid_amb_even_bin_4_5;
    (*pstatic)
        .algo__range_min_clip = (*phistogram).histogram_config__mid_amb_odd_bin_0_1;
    (*pstatic)
        .algo__consistency_check__tolerance = (*phistogram)
        .histogram_config__mid_amb_odd_bin_2;
    (*pstatic)
        .spare_host_config__static_config_spare_2 = (*phistogram)
        .histogram_config__mid_amb_odd_bin_3_4;
    (*pstatic)
        .sd_config__reset_stages_msb = (*phistogram).histogram_config__mid_amb_odd_bin_5;
    (*ptiming)
        .range_config__sigma_thresh = ((((*phistogram)
        .histogram_config__low_amb_even_bin_0_1 as u16 as libc::c_int)
        << 8 as libc::c_int)
        + (*phistogram).histogram_config__low_amb_even_bin_2_3 as u16
            as libc::c_int) as u16;
    (*ptiming)
        .range_config__min_count_rate_rtn_limit_mcps = ((((*phistogram)
        .histogram_config__low_amb_even_bin_4_5 as u16 as libc::c_int)
        << 8 as libc::c_int)
        + (*phistogram).histogram_config__low_amb_odd_bin_0_1 as u16 as libc::c_int)
        as u16;
    (*ptiming)
        .range_config__valid_phase_low = (*phistogram)
        .histogram_config__low_amb_odd_bin_2_3;
    (*ptiming)
        .range_config__valid_phase_high = (*phistogram)
        .histogram_config__low_amb_odd_bin_4_5;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_init_version(mut Dev: VL53LX_DEV) {
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    (*pdev).version.ll_major = 1 as libc::c_int as u8;
    (*pdev).version.ll_minor = 1 as libc::c_int as u8;
    (*pdev).version.ll_build = 1 as libc::c_int as u8;
    (*pdev).version.ll_revision = 0 as libc::c_int as u32;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_init_ll_driver_state(
    mut Dev: VL53LX_DEV,
    mut device_state: VL53LX_DeviceState,
) {
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    let mut pstate: *mut VL53LX_ll_driver_state_t = &mut (*pdev).ll_state;
    (*pstate).cfg_device_state = device_state;
    (*pstate).cfg_stream_count = 0 as libc::c_int as u8;
    (*pstate).cfg_gph_id = 0x2 as libc::c_int as u8;
    (*pstate).cfg_timing_status = 0 as libc::c_int as u8;
    (*pstate).cfg_zone_id = 0 as libc::c_int as u8;
    (*pstate).rd_device_state = device_state;
    (*pstate).rd_stream_count = 0 as libc::c_int as u8;
    (*pstate).rd_gph_id = 0x2 as libc::c_int as u8;
    (*pstate).rd_timing_status = 0 as libc::c_int as u8;
    (*pstate).rd_zone_id = 0 as libc::c_int as u8;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_update_ll_driver_rd_state(
    mut Dev: VL53LX_DEV,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    let mut pstate: *mut VL53LX_ll_driver_state_t = &mut (*pdev).ll_state;
    if (*pdev).sys_ctrl.system__mode_start as libc::c_int & 0xf0 as libc::c_int
        == 0 as libc::c_int
    {
        (*pstate).rd_device_state = 3 as libc::c_int as VL53LX_DeviceState;
        (*pstate).rd_stream_count = 0 as libc::c_int as u8;
        (*pstate).rd_internal_stream_count = 0 as libc::c_int as u8;
        (*pstate).rd_internal_stream_count_val = 0 as libc::c_int as u8;
        (*pstate).rd_gph_id = 0x2 as libc::c_int as u8;
        (*pstate).rd_timing_status = 0 as libc::c_int as u8;
        (*pstate).rd_zone_id = 0 as libc::c_int as u8;
    } else {
        if (*pstate).rd_stream_count as libc::c_int == 0xff as libc::c_int {
            (*pstate).rd_stream_count = 0x80 as libc::c_int as u8;
        } else {
            let ref mut fresh26 = (*pstate).rd_stream_count;
            *fresh26 = (*fresh26).wrapping_add(1);
        }
        status = VL53LX_update_internal_stream_counters(
            Dev,
            (*pstate).rd_stream_count,
            &mut (*pstate).rd_internal_stream_count,
            &mut (*pstate).rd_internal_stream_count_val,
        );
        let ref mut fresh27 = (*pstate).rd_gph_id;
        *fresh27 = (*fresh27 as libc::c_int ^ 0x2 as libc::c_int) as u8;
        match (*pstate).rd_device_state as libc::c_int {
            3 => {
                if (*pdev).dyn_cfg.system__grouped_parameter_hold as libc::c_int
                    & 0x2 as libc::c_int > 0 as libc::c_int
                {
                    (*pstate).rd_device_state = 6 as libc::c_int as VL53LX_DeviceState;
                } else if (*pstate).rd_zone_id as libc::c_int
                        >= (*pdev).zone_cfg.active_zones as libc::c_int
                    {
                    (*pstate).rd_device_state = 8 as libc::c_int as VL53LX_DeviceState;
                } else {
                    (*pstate).rd_device_state = 7 as libc::c_int as VL53LX_DeviceState;
                }
                (*pstate).rd_stream_count = 0 as libc::c_int as u8;
                (*pstate).rd_internal_stream_count = 0 as libc::c_int as u8;
                (*pstate).rd_internal_stream_count_val = 0 as libc::c_int as u8;
                (*pstate).rd_timing_status = 0 as libc::c_int as u8;
                (*pstate).rd_zone_id = 0 as libc::c_int as u8;
            }
            6 => {
                (*pstate).rd_stream_count = 0 as libc::c_int as u8;
                (*pstate).rd_internal_stream_count = 0 as libc::c_int as u8;
                (*pstate).rd_internal_stream_count_val = 0 as libc::c_int as u8;
                (*pstate).rd_zone_id = 0 as libc::c_int as u8;
                if (*pstate).rd_zone_id as libc::c_int
                    >= (*pdev).zone_cfg.active_zones as libc::c_int
                {
                    (*pstate).rd_device_state = 8 as libc::c_int as VL53LX_DeviceState;
                } else {
                    (*pstate).rd_device_state = 7 as libc::c_int as VL53LX_DeviceState;
                }
            }
            7 => {
                let ref mut fresh28 = (*pstate).rd_zone_id;
                *fresh28 = (*fresh28).wrapping_add(1);
                if (*pstate).rd_zone_id as libc::c_int
                    >= (*pdev).zone_cfg.active_zones as libc::c_int
                {
                    (*pstate).rd_device_state = 8 as libc::c_int as VL53LX_DeviceState;
                } else {
                    (*pstate).rd_device_state = 7 as libc::c_int as VL53LX_DeviceState;
                }
            }
            8 => {
                (*pstate).rd_zone_id = 0 as libc::c_int as u8;
                let ref mut fresh29 = (*pstate).rd_timing_status;
                *fresh29 = (*fresh29 as libc::c_int ^ 0x1 as libc::c_int) as u8;
                if (*pstate).rd_zone_id as libc::c_int
                    >= (*pdev).zone_cfg.active_zones as libc::c_int
                {
                    (*pstate).rd_device_state = 8 as libc::c_int as VL53LX_DeviceState;
                } else {
                    (*pstate).rd_device_state = 7 as libc::c_int as VL53LX_DeviceState;
                }
            }
            _ => {
                (*pstate).rd_device_state = 3 as libc::c_int as VL53LX_DeviceState;
                (*pstate).rd_stream_count = 0 as libc::c_int as u8;
                (*pstate).rd_internal_stream_count = 0 as libc::c_int as u8;
                (*pstate).rd_internal_stream_count_val = 0 as libc::c_int as u8;
                (*pstate).rd_gph_id = 0x2 as libc::c_int as u8;
                (*pstate).rd_timing_status = 0 as libc::c_int as u8;
                (*pstate).rd_zone_id = 0 as libc::c_int as u8;
            }
        }
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_check_ll_driver_rd_state(
    mut Dev: VL53LX_DEV,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    let mut pres: *mut VL53LX_LLDriverResults_t = &mut (*Dev).Data.llresults;
    let mut pstate: *mut VL53LX_ll_driver_state_t = &mut (*pdev).ll_state;
    let mut psys_results: *mut VL53LX_system_results_t = &mut (*pdev).sys_results;
    let mut phist_data: *mut VL53LX_histogram_bin_data_t = &mut (*pdev).hist_data;
    let mut pZ: *mut VL53LX_zone_private_dyn_cfgs_t = &mut (*pres).zone_dyn_cfgs;
    let mut device_range_status: u8 = 0 as libc::c_int as u8;
    let mut device_stream_count: u8 = 0 as libc::c_int as u8;
    let mut device_gph_id: u8 = 0 as libc::c_int as u8;
    let mut histogram_mode: u8 = 0 as libc::c_int as u8;
    let mut expected_stream_count: u8 = 0 as libc::c_int as u8;
    let mut expected_gph_id: u8 = 0 as libc::c_int as u8;
    device_range_status = ((*psys_results).result__range_status as libc::c_int
        & 0x1f as libc::c_int) as u8;
    device_stream_count = (*psys_results).result__stream_count;
    histogram_mode = ((*pdev).sys_ctrl.system__mode_start as libc::c_int
        & 0x2 as libc::c_int == 0x2 as libc::c_int) as libc::c_int as u8;
    device_gph_id = (((*psys_results).result__interrupt_status as libc::c_int
        & 0x20 as libc::c_int) >> 4 as libc::c_int) as u8;
    if histogram_mode != 0 {
        device_gph_id = (((*phist_data).result__interrupt_status as libc::c_int
            & 0x20 as libc::c_int) >> 4 as libc::c_int) as u8;
    }
    if (*pdev).sys_ctrl.system__mode_start as libc::c_int
        & 0x20 as libc::c_int as VL53LX_DeviceMeasurementModes as libc::c_int
        == 0x20 as libc::c_int as VL53LX_DeviceMeasurementModes as libc::c_int
    {
        if (*pstate).rd_device_state as libc::c_int
            == 6 as libc::c_int as VL53LX_DeviceState as libc::c_int
        {
            if histogram_mode as libc::c_int == 0 as libc::c_int {
                if device_range_status as libc::c_int
                    != 18 as libc::c_int as VL53LX_DeviceError as libc::c_int
                {
                    status = -(17 as libc::c_int) as VL53LX_Error;
                }
            }
        } else {
            if (*pstate).rd_stream_count as libc::c_int
                != device_stream_count as libc::c_int
            {
                status = -(18 as libc::c_int) as VL53LX_Error;
            }
            if (*pstate).rd_gph_id as libc::c_int != device_gph_id as libc::c_int {
                status = -(19 as libc::c_int) as VL53LX_Error;
            }
            expected_stream_count = (*pZ)
                .VL53LX_p_003[(*pstate).rd_zone_id as usize]
                .expected_stream_count;
            expected_gph_id = (*pZ)
                .VL53LX_p_003[(*pstate).rd_zone_id as usize]
                .expected_gph_id;
            if expected_stream_count as libc::c_int != device_stream_count as libc::c_int
            {
                if !((*pdev).zone_cfg.active_zones as libc::c_int == 0 as libc::c_int
                    && device_stream_count as libc::c_int == 255 as libc::c_int)
                {
                    status = -(20 as libc::c_int) as VL53LX_Error;
                }
            }
            if expected_gph_id as libc::c_int != device_gph_id as libc::c_int {
                status = -(21 as libc::c_int) as VL53LX_Error;
            }
        }
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_update_ll_driver_cfg_state(
    mut Dev: VL53LX_DEV,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    let mut pres: *mut VL53LX_LLDriverResults_t = &mut (*Dev).Data.llresults;
    let mut pstate: *mut VL53LX_ll_driver_state_t = &mut (*pdev).ll_state;
    let mut pZ: *mut VL53LX_zone_private_dyn_cfgs_t = &mut (*pres).zone_dyn_cfgs;
    let mut prev_cfg_zone_id: u8 = 0;
    let mut prev_cfg_gph_id: u8 = 0;
    let mut prev_cfg_stream_count: u8 = 0;
    if (*pdev).sys_ctrl.system__mode_start as libc::c_int & 0xf0 as libc::c_int
        == 0 as libc::c_int
    {
        (*pstate).cfg_device_state = 3 as libc::c_int as VL53LX_DeviceState;
        (*pstate).cfg_stream_count = 0 as libc::c_int as u8;
        (*pstate).cfg_internal_stream_count = 0 as libc::c_int as u8;
        (*pstate).cfg_internal_stream_count_val = 0 as libc::c_int as u8;
        (*pstate).cfg_gph_id = 0x2 as libc::c_int as u8;
        (*pstate).cfg_timing_status = 0 as libc::c_int as u8;
        (*pstate).cfg_zone_id = 0 as libc::c_int as u8;
        prev_cfg_zone_id = 0 as libc::c_int as u8;
        prev_cfg_gph_id = 0 as libc::c_int as u8;
        prev_cfg_stream_count = 0 as libc::c_int as u8;
    } else {
        prev_cfg_gph_id = (*pstate).cfg_gph_id;
        prev_cfg_zone_id = (*pstate).cfg_zone_id;
        prev_cfg_stream_count = (*pstate).cfg_stream_count;
        if (*pstate).cfg_stream_count as libc::c_int == 0xff as libc::c_int {
            (*pstate).cfg_stream_count = 0x80 as libc::c_int as u8;
        } else {
            let ref mut fresh30 = (*pstate).cfg_stream_count;
            *fresh30 = (*fresh30).wrapping_add(1);
        }
        status = VL53LX_update_internal_stream_counters(
            Dev,
            (*pstate).cfg_stream_count,
            &mut (*pstate).cfg_internal_stream_count,
            &mut (*pstate).cfg_internal_stream_count_val,
        );
        let ref mut fresh31 = (*pstate).cfg_gph_id;
        *fresh31 = (*fresh31 as libc::c_int ^ 0x2 as libc::c_int) as u8;
        match (*pstate).cfg_device_state as libc::c_int {
            3 => {
                (*pstate).cfg_zone_id = 1 as libc::c_int as u8;
                if (*pstate).cfg_zone_id as libc::c_int
                    > (*pdev).zone_cfg.active_zones as libc::c_int
                {
                    (*pstate).cfg_zone_id = 0 as libc::c_int as u8;
                    let ref mut fresh32 = (*pstate).cfg_timing_status;
                    *fresh32 = (*fresh32 as libc::c_int ^ 0x1 as libc::c_int) as u8;
                }
                (*pstate).cfg_stream_count = 1 as libc::c_int as u8;
                if (*pdev).gen_cfg.global_config__stream_divider as libc::c_int
                    == 0 as libc::c_int
                {
                    (*pstate).cfg_internal_stream_count = 1 as libc::c_int as u8;
                    (*pstate)
                        .cfg_internal_stream_count_val = 0 as libc::c_int as u8;
                } else {
                    (*pstate).cfg_internal_stream_count = 0 as libc::c_int as u8;
                    (*pstate)
                        .cfg_internal_stream_count_val = 1 as libc::c_int as u8;
                }
                (*pstate).cfg_device_state = 4 as libc::c_int as VL53LX_DeviceState;
            }
            4 => {
                let ref mut fresh33 = (*pstate).cfg_zone_id;
                *fresh33 = (*fresh33).wrapping_add(1);
                if (*pstate).cfg_zone_id as libc::c_int
                    > (*pdev).zone_cfg.active_zones as libc::c_int
                {
                    (*pstate).cfg_zone_id = 0 as libc::c_int as u8;
                    let ref mut fresh34 = (*pstate).cfg_timing_status;
                    *fresh34 = (*fresh34 as libc::c_int ^ 0x1 as libc::c_int) as u8;
                    if (*pdev).zone_cfg.active_zones as libc::c_int > 0 as libc::c_int {
                        (*pstate)
                            .cfg_device_state = 5 as libc::c_int as VL53LX_DeviceState;
                    }
                }
            }
            5 => {
                let ref mut fresh35 = (*pstate).cfg_zone_id;
                *fresh35 = (*fresh35).wrapping_add(1);
                if (*pstate).cfg_zone_id as libc::c_int
                    > (*pdev).zone_cfg.active_zones as libc::c_int
                {
                    (*pstate).cfg_zone_id = 0 as libc::c_int as u8;
                    let ref mut fresh36 = (*pstate).cfg_timing_status;
                    *fresh36 = (*fresh36 as libc::c_int ^ 0x1 as libc::c_int) as u8;
                }
            }
            _ => {
                (*pstate).cfg_device_state = 3 as libc::c_int as VL53LX_DeviceState;
                (*pstate).cfg_stream_count = 0 as libc::c_int as u8;
                (*pstate).cfg_internal_stream_count = 0 as libc::c_int as u8;
                (*pstate).cfg_internal_stream_count_val = 0 as libc::c_int as u8;
                (*pstate).cfg_gph_id = 0x2 as libc::c_int as u8;
                (*pstate).cfg_timing_status = 0 as libc::c_int as u8;
                (*pstate).cfg_zone_id = 0 as libc::c_int as u8;
            }
        }
    }
    if (*pdev).zone_cfg.active_zones as libc::c_int == 0 as libc::c_int {
        (*pZ)
            .VL53LX_p_003[prev_cfg_zone_id as usize]
            .expected_stream_count = (prev_cfg_stream_count as libc::c_int
            - 1 as libc::c_int) as u8;
        (*pZ)
            .VL53LX_p_003[(*pstate).rd_zone_id as usize]
            .expected_gph_id = (prev_cfg_gph_id as libc::c_int ^ 0x2 as libc::c_int)
            as u8;
    } else {
        (*pZ)
            .VL53LX_p_003[prev_cfg_zone_id as usize]
            .expected_stream_count = prev_cfg_stream_count;
        (*pZ).VL53LX_p_003[prev_cfg_zone_id as usize].expected_gph_id = prev_cfg_gph_id;
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_copy_rtn_good_spads_to_buffer(
    mut pdata: *mut VL53LX_nvm_copy_data_t,
    mut pbuffer: *mut u8,
) {
    *pbuffer
        .offset(0 as libc::c_int as isize) = (*pdata).global_config__spad_enables_rtn_0;
    *pbuffer
        .offset(1 as libc::c_int as isize) = (*pdata).global_config__spad_enables_rtn_1;
    *pbuffer
        .offset(2 as libc::c_int as isize) = (*pdata).global_config__spad_enables_rtn_2;
    *pbuffer
        .offset(3 as libc::c_int as isize) = (*pdata).global_config__spad_enables_rtn_3;
    *pbuffer
        .offset(4 as libc::c_int as isize) = (*pdata).global_config__spad_enables_rtn_4;
    *pbuffer
        .offset(5 as libc::c_int as isize) = (*pdata).global_config__spad_enables_rtn_5;
    *pbuffer
        .offset(6 as libc::c_int as isize) = (*pdata).global_config__spad_enables_rtn_6;
    *pbuffer
        .offset(7 as libc::c_int as isize) = (*pdata).global_config__spad_enables_rtn_7;
    *pbuffer
        .offset(8 as libc::c_int as isize) = (*pdata).global_config__spad_enables_rtn_8;
    *pbuffer
        .offset(9 as libc::c_int as isize) = (*pdata).global_config__spad_enables_rtn_9;
    *pbuffer
        .offset(
            10 as libc::c_int as isize,
        ) = (*pdata).global_config__spad_enables_rtn_10;
    *pbuffer
        .offset(
            11 as libc::c_int as isize,
        ) = (*pdata).global_config__spad_enables_rtn_11;
    *pbuffer
        .offset(
            12 as libc::c_int as isize,
        ) = (*pdata).global_config__spad_enables_rtn_12;
    *pbuffer
        .offset(
            13 as libc::c_int as isize,
        ) = (*pdata).global_config__spad_enables_rtn_13;
    *pbuffer
        .offset(
            14 as libc::c_int as isize,
        ) = (*pdata).global_config__spad_enables_rtn_14;
    *pbuffer
        .offset(
            15 as libc::c_int as isize,
        ) = (*pdata).global_config__spad_enables_rtn_15;
    *pbuffer
        .offset(
            16 as libc::c_int as isize,
        ) = (*pdata).global_config__spad_enables_rtn_16;
    *pbuffer
        .offset(
            17 as libc::c_int as isize,
        ) = (*pdata).global_config__spad_enables_rtn_17;
    *pbuffer
        .offset(
            18 as libc::c_int as isize,
        ) = (*pdata).global_config__spad_enables_rtn_18;
    *pbuffer
        .offset(
            19 as libc::c_int as isize,
        ) = (*pdata).global_config__spad_enables_rtn_19;
    *pbuffer
        .offset(
            20 as libc::c_int as isize,
        ) = (*pdata).global_config__spad_enables_rtn_20;
    *pbuffer
        .offset(
            21 as libc::c_int as isize,
        ) = (*pdata).global_config__spad_enables_rtn_21;
    *pbuffer
        .offset(
            22 as libc::c_int as isize,
        ) = (*pdata).global_config__spad_enables_rtn_22;
    *pbuffer
        .offset(
            23 as libc::c_int as isize,
        ) = (*pdata).global_config__spad_enables_rtn_23;
    *pbuffer
        .offset(
            24 as libc::c_int as isize,
        ) = (*pdata).global_config__spad_enables_rtn_24;
    *pbuffer
        .offset(
            25 as libc::c_int as isize,
        ) = (*pdata).global_config__spad_enables_rtn_25;
    *pbuffer
        .offset(
            26 as libc::c_int as isize,
        ) = (*pdata).global_config__spad_enables_rtn_26;
    *pbuffer
        .offset(
            27 as libc::c_int as isize,
        ) = (*pdata).global_config__spad_enables_rtn_27;
    *pbuffer
        .offset(
            28 as libc::c_int as isize,
        ) = (*pdata).global_config__spad_enables_rtn_28;
    *pbuffer
        .offset(
            29 as libc::c_int as isize,
        ) = (*pdata).global_config__spad_enables_rtn_29;
    *pbuffer
        .offset(
            30 as libc::c_int as isize,
        ) = (*pdata).global_config__spad_enables_rtn_30;
    *pbuffer
        .offset(
            31 as libc::c_int as isize,
        ) = (*pdata).global_config__spad_enables_rtn_31;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_init_system_results(
    mut pdata: *mut VL53LX_system_results_t,
) {
    (*pdata).result__interrupt_status = 0xff as libc::c_int as u8;
    (*pdata).result__range_status = 0xff as libc::c_int as u8;
    (*pdata).result__report_status = 0xff as libc::c_int as u8;
    (*pdata).result__stream_count = 0xff as libc::c_int as u8;
    (*pdata).result__dss_actual_effective_spads_sd0 = 0xffff as libc::c_int as u16;
    (*pdata).result__peak_signal_count_rate_mcps_sd0 = 0xffff as libc::c_int as u16;
    (*pdata).result__ambient_count_rate_mcps_sd0 = 0xffff as libc::c_int as u16;
    (*pdata).result__sigma_sd0 = 0xffff as libc::c_int as u16;
    (*pdata).result__phase_sd0 = 0xffff as libc::c_int as u16;
    (*pdata)
        .result__final_crosstalk_corrected_range_mm_sd0 = 0xffff as libc::c_int
        as u16;
    (*pdata)
        .result__peak_signal_count_rate_crosstalk_corrected_mcps_sd0 = 0xffff
        as libc::c_int as u16;
    (*pdata)
        .result__mm_inner_actual_effective_spads_sd0 = 0xffff as libc::c_int as u16;
    (*pdata)
        .result__mm_outer_actual_effective_spads_sd0 = 0xffff as libc::c_int as u16;
    (*pdata).result__avg_signal_count_rate_mcps_sd0 = 0xffff as libc::c_int as u16;
    (*pdata).result__dss_actual_effective_spads_sd1 = 0xffff as libc::c_int as u16;
    (*pdata).result__peak_signal_count_rate_mcps_sd1 = 0xffff as libc::c_int as u16;
    (*pdata).result__ambient_count_rate_mcps_sd1 = 0xffff as libc::c_int as u16;
    (*pdata).result__sigma_sd1 = 0xffff as libc::c_int as u16;
    (*pdata).result__phase_sd1 = 0xffff as libc::c_int as u16;
    (*pdata)
        .result__final_crosstalk_corrected_range_mm_sd1 = 0xffff as libc::c_int
        as u16;
    (*pdata).result__spare_0_sd1 = 0xffff as libc::c_int as u16;
    (*pdata).result__spare_1_sd1 = 0xffff as libc::c_int as u16;
    (*pdata).result__spare_2_sd1 = 0xffff as libc::c_int as u16;
    (*pdata).result__spare_3_sd1 = 0xff as libc::c_int as u8;
}
#[no_mangle]
pub unsafe extern "C" fn V53L1_init_zone_results_structure(
    mut active_zones: u8,
    mut pdata: *mut VL53LX_zone_results_t,
) {
    let mut z: u8 = 0 as libc::c_int as u8;
    let mut pobjects: *mut VL53LX_zone_objects_t = 0 as *mut VL53LX_zone_objects_t;
    (*pdata).max_zones = 5 as libc::c_int as u8;
    (*pdata).active_zones = active_zones;
    z = 0 as libc::c_int as u8;
    while (z as libc::c_int) < (*pdata).max_zones as libc::c_int {
        pobjects = &mut *((*pdata).VL53LX_p_003).as_mut_ptr().offset(z as isize)
            as *mut VL53LX_zone_objects_t;
        (*pobjects).cfg_device_state = 3 as libc::c_int as VL53LX_DeviceState;
        (*pobjects).rd_device_state = 3 as libc::c_int as VL53LX_DeviceState;
        (*pobjects).max_objects = 4 as libc::c_int as u8;
        (*pobjects).active_objects = 0 as libc::c_int as u8;
        z = z.wrapping_add(1);
    }
}
#[no_mangle]
pub unsafe extern "C" fn V53L1_init_zone_dss_configs(mut Dev: VL53LX_DEV) {
    let mut pres: *mut VL53LX_LLDriverResults_t = &mut (*Dev).Data.llresults;
    let mut z: u8 = 0 as libc::c_int as u8;
    let mut max_zones: u8 = 5 as libc::c_int as u8;
    let mut pdata: *mut VL53LX_zone_private_dyn_cfgs_t = &mut (*pres).zone_dyn_cfgs;
    z = 0 as libc::c_int as u8;
    while (z as libc::c_int) < max_zones as libc::c_int {
        (*pdata).VL53LX_p_003[z as usize].dss_mode = 0x1 as libc::c_int as u8;
        (*pdata)
            .VL53LX_p_003[z as usize]
            .dss_requested_effective_spad_count = 0 as libc::c_int as u16;
        z = z.wrapping_add(1);
    }
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_init_histogram_config_structure(
    mut even_bin0: u8,
    mut even_bin1: u8,
    mut even_bin2: u8,
    mut even_bin3: u8,
    mut even_bin4: u8,
    mut even_bin5: u8,
    mut odd_bin0: u8,
    mut odd_bin1: u8,
    mut odd_bin2: u8,
    mut odd_bin3: u8,
    mut odd_bin4: u8,
    mut odd_bin5: u8,
    mut pdata: *mut VL53LX_histogram_config_t,
) {
    (*pdata)
        .histogram_config__low_amb_even_bin_0_1 = (((even_bin1 as libc::c_int)
        << 4 as libc::c_int) + even_bin0 as libc::c_int) as u8;
    (*pdata)
        .histogram_config__low_amb_even_bin_2_3 = (((even_bin3 as libc::c_int)
        << 4 as libc::c_int) + even_bin2 as libc::c_int) as u8;
    (*pdata)
        .histogram_config__low_amb_even_bin_4_5 = (((even_bin5 as libc::c_int)
        << 4 as libc::c_int) + even_bin4 as libc::c_int) as u8;
    (*pdata)
        .histogram_config__low_amb_odd_bin_0_1 = (((odd_bin1 as libc::c_int)
        << 4 as libc::c_int) + odd_bin0 as libc::c_int) as u8;
    (*pdata)
        .histogram_config__low_amb_odd_bin_2_3 = (((odd_bin3 as libc::c_int)
        << 4 as libc::c_int) + odd_bin2 as libc::c_int) as u8;
    (*pdata)
        .histogram_config__low_amb_odd_bin_4_5 = (((odd_bin5 as libc::c_int)
        << 4 as libc::c_int) + odd_bin4 as libc::c_int) as u8;
    (*pdata)
        .histogram_config__mid_amb_even_bin_0_1 = (*pdata)
        .histogram_config__low_amb_even_bin_0_1;
    (*pdata)
        .histogram_config__mid_amb_even_bin_2_3 = (*pdata)
        .histogram_config__low_amb_even_bin_2_3;
    (*pdata)
        .histogram_config__mid_amb_even_bin_4_5 = (*pdata)
        .histogram_config__low_amb_even_bin_4_5;
    (*pdata)
        .histogram_config__mid_amb_odd_bin_0_1 = (*pdata)
        .histogram_config__low_amb_odd_bin_0_1;
    (*pdata).histogram_config__mid_amb_odd_bin_2 = odd_bin2;
    (*pdata)
        .histogram_config__mid_amb_odd_bin_3_4 = (((odd_bin4 as libc::c_int)
        << 4 as libc::c_int) + odd_bin3 as libc::c_int) as u8;
    (*pdata).histogram_config__mid_amb_odd_bin_5 = odd_bin5;
    (*pdata).histogram_config__user_bin_offset = 0 as libc::c_int as u8;
    (*pdata)
        .histogram_config__high_amb_even_bin_0_1 = (*pdata)
        .histogram_config__low_amb_even_bin_0_1;
    (*pdata)
        .histogram_config__high_amb_even_bin_2_3 = (*pdata)
        .histogram_config__low_amb_even_bin_2_3;
    (*pdata)
        .histogram_config__high_amb_even_bin_4_5 = (*pdata)
        .histogram_config__low_amb_even_bin_4_5;
    (*pdata)
        .histogram_config__high_amb_odd_bin_0_1 = (*pdata)
        .histogram_config__low_amb_odd_bin_0_1;
    (*pdata)
        .histogram_config__high_amb_odd_bin_2_3 = (*pdata)
        .histogram_config__low_amb_odd_bin_2_3;
    (*pdata)
        .histogram_config__high_amb_odd_bin_4_5 = (*pdata)
        .histogram_config__low_amb_odd_bin_4_5;
    (*pdata).histogram_config__amb_thresh_low = 0xffff as libc::c_int as u16;
    (*pdata).histogram_config__amb_thresh_high = 0xffff as libc::c_int as u16;
    (*pdata).histogram_config__spad_array_selection = 0 as libc::c_int as u8;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_init_histogram_multizone_config_structure(
    mut even_bin0: u8,
    mut even_bin1: u8,
    mut even_bin2: u8,
    mut even_bin3: u8,
    mut even_bin4: u8,
    mut even_bin5: u8,
    mut odd_bin0: u8,
    mut odd_bin1: u8,
    mut odd_bin2: u8,
    mut odd_bin3: u8,
    mut odd_bin4: u8,
    mut odd_bin5: u8,
    mut pdata: *mut VL53LX_histogram_config_t,
) {
    (*pdata)
        .histogram_config__low_amb_even_bin_0_1 = (((even_bin1 as libc::c_int)
        << 4 as libc::c_int) + even_bin0 as libc::c_int) as u8;
    (*pdata)
        .histogram_config__low_amb_even_bin_2_3 = (((even_bin3 as libc::c_int)
        << 4 as libc::c_int) + even_bin2 as libc::c_int) as u8;
    (*pdata)
        .histogram_config__low_amb_even_bin_4_5 = (((even_bin5 as libc::c_int)
        << 4 as libc::c_int) + even_bin4 as libc::c_int) as u8;
    (*pdata)
        .histogram_config__low_amb_odd_bin_0_1 = (*pdata)
        .histogram_config__low_amb_even_bin_0_1;
    (*pdata)
        .histogram_config__low_amb_odd_bin_2_3 = (*pdata)
        .histogram_config__low_amb_even_bin_2_3;
    (*pdata)
        .histogram_config__low_amb_odd_bin_4_5 = (*pdata)
        .histogram_config__low_amb_even_bin_4_5;
    (*pdata)
        .histogram_config__mid_amb_even_bin_0_1 = (*pdata)
        .histogram_config__low_amb_even_bin_0_1;
    (*pdata)
        .histogram_config__mid_amb_even_bin_2_3 = (*pdata)
        .histogram_config__low_amb_even_bin_2_3;
    (*pdata)
        .histogram_config__mid_amb_even_bin_4_5 = (*pdata)
        .histogram_config__low_amb_even_bin_4_5;
    (*pdata)
        .histogram_config__mid_amb_odd_bin_0_1 = (*pdata)
        .histogram_config__low_amb_odd_bin_0_1;
    (*pdata).histogram_config__mid_amb_odd_bin_2 = odd_bin2;
    (*pdata)
        .histogram_config__mid_amb_odd_bin_3_4 = (((odd_bin4 as libc::c_int)
        << 4 as libc::c_int) + odd_bin3 as libc::c_int) as u8;
    (*pdata).histogram_config__mid_amb_odd_bin_5 = odd_bin5;
    (*pdata).histogram_config__user_bin_offset = 0 as libc::c_int as u8;
    (*pdata)
        .histogram_config__high_amb_even_bin_0_1 = (((odd_bin1 as libc::c_int)
        << 4 as libc::c_int) + odd_bin0 as libc::c_int) as u8;
    (*pdata)
        .histogram_config__high_amb_even_bin_2_3 = (((odd_bin3 as libc::c_int)
        << 4 as libc::c_int) + odd_bin2 as libc::c_int) as u8;
    (*pdata)
        .histogram_config__high_amb_even_bin_4_5 = (((odd_bin5 as libc::c_int)
        << 4 as libc::c_int) + odd_bin4 as libc::c_int) as u8;
    (*pdata)
        .histogram_config__high_amb_odd_bin_0_1 = (*pdata)
        .histogram_config__high_amb_even_bin_0_1;
    (*pdata)
        .histogram_config__high_amb_odd_bin_2_3 = (*pdata)
        .histogram_config__high_amb_even_bin_2_3;
    (*pdata)
        .histogram_config__high_amb_odd_bin_4_5 = (*pdata)
        .histogram_config__high_amb_even_bin_4_5;
    (*pdata).histogram_config__amb_thresh_low = 0xffff as libc::c_int as u16;
    (*pdata).histogram_config__amb_thresh_high = 0xffff as libc::c_int as u16;
    (*pdata).histogram_config__spad_array_selection = 0 as libc::c_int as u8;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_init_xtalk_bin_data_struct(
    mut bin_value: u32,
    mut VL53LX_p_021: u16,
    mut pdata: *mut VL53LX_xtalk_histogram_shape_t,
) {
    let mut i: u16 = 0 as libc::c_int as u16;
    (*pdata).zone_id = 0 as libc::c_int as u8;
    (*pdata).time_stamp = 0 as libc::c_int as u32;
    (*pdata).VL53LX_p_019 = 0 as libc::c_int as u8;
    (*pdata).VL53LX_p_020 = 12 as libc::c_int as u8;
    (*pdata).VL53LX_p_021 = VL53LX_p_021 as u8;
    (*pdata).phasecal_result__reference_phase = 0 as libc::c_int as u16;
    (*pdata).phasecal_result__vcsel_start = 0 as libc::c_int as u8;
    (*pdata).cal_config__vcsel_start = 0 as libc::c_int as u8;
    (*pdata).vcsel_width = 0 as libc::c_int as u16;
    (*pdata).VL53LX_p_015 = 0 as libc::c_int as u16;
    (*pdata).zero_distance_phase = 0 as libc::c_int as u16;
    i = 0 as libc::c_int as u16;
    while (i as libc::c_int) < 12 as libc::c_int {
        if (i as libc::c_int) < VL53LX_p_021 as libc::c_int {
            (*pdata).bin_data[i as usize] = bin_value;
        } else {
            (*pdata).bin_data[i as usize] = 0 as libc::c_int as u32;
        }
        i = i.wrapping_add(1);
    }
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_i2c_encode_uint16_t(
    mut ip_value: u16,
    mut count: u16,
    mut pbuffer: *mut u8,
) {
    let mut i: u16 = 0 as libc::c_int as u16;
    let mut VL53LX_p_003: u16 = 0 as libc::c_int as u16;
    VL53LX_p_003 = ip_value;
    i = 0 as libc::c_int as u16;
    while (i as libc::c_int) < count as libc::c_int {
        *pbuffer
            .offset(
                (count as libc::c_int - i as libc::c_int - 1 as libc::c_int) as isize,
            ) = (VL53LX_p_003 as libc::c_int & 0xff as libc::c_int) as u8;
        VL53LX_p_003 = (VL53LX_p_003 as libc::c_int >> 8 as libc::c_int) as u16;
        i = i.wrapping_add(1);
    }
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_i2c_decode_uint16_t(
    mut count: u16,
    mut pbuffer: *mut u8,
) -> u16 {
    let mut value: u16 = 0 as libc::c_int as u16;
    loop {
        let fresh37 = count;
        count = count.wrapping_sub(1);
        if !(fresh37 as libc::c_int > 0 as libc::c_int) {
            break;
        }
        let fresh38 = pbuffer;
        pbuffer = pbuffer.offset(1);
        value = ((value as libc::c_int) << 8 as libc::c_int
            | *fresh38 as u16 as libc::c_int) as u16;
    }
    return value;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_i2c_encode_int16_t(
    mut ip_value: i16,
    mut count: u16,
    mut pbuffer: *mut u8,
) {
    let mut i: u16 = 0 as libc::c_int as u16;
    let mut VL53LX_p_003: i16 = 0 as libc::c_int as i16;
    VL53LX_p_003 = ip_value;
    i = 0 as libc::c_int as u16;
    while (i as libc::c_int) < count as libc::c_int {
        *pbuffer
            .offset(
                (count as libc::c_int - i as libc::c_int - 1 as libc::c_int) as isize,
            ) = (VL53LX_p_003 as libc::c_int & 0xff as libc::c_int) as u8;
        VL53LX_p_003 = (VL53LX_p_003 as libc::c_int >> 8 as libc::c_int) as i16;
        i = i.wrapping_add(1);
    }
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_i2c_decode_int16_t(
    mut count: u16,
    mut pbuffer: *mut u8,
) -> i16 {
    let mut value: i16 = 0 as libc::c_int as i16;
    if *pbuffer as libc::c_int >= 0x80 as libc::c_int {
        value = 0xffff as libc::c_int as i16;
    }
    loop {
        let fresh39 = count;
        count = count.wrapping_sub(1);
        if !(fresh39 as libc::c_int > 0 as libc::c_int) {
            break;
        }
        let fresh40 = pbuffer;
        pbuffer = pbuffer.offset(1);
        value = ((value as libc::c_int) << 8 as libc::c_int
            | *fresh40 as i16 as libc::c_int) as i16;
    }
    return value;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_i2c_encode_uint32_t(
    mut ip_value: u32,
    mut count: u16,
    mut pbuffer: *mut u8,
) {
    let mut i: u16 = 0 as libc::c_int as u16;
    let mut VL53LX_p_003: u32 = 0 as libc::c_int as u32;
    VL53LX_p_003 = ip_value;
    i = 0 as libc::c_int as u16;
    while (i as libc::c_int) < count as libc::c_int {
        *pbuffer
            .offset(
                (count as libc::c_int - i as libc::c_int - 1 as libc::c_int) as isize,
            ) = (VL53LX_p_003 & 0xff as libc::c_int as libc::c_uint) as u8;
        VL53LX_p_003 = VL53LX_p_003 >> 8 as libc::c_int;
        i = i.wrapping_add(1);
    }
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_i2c_decode_uint32_t(
    mut count: u16,
    mut pbuffer: *mut u8,
) -> u32 {
    let mut value: u32 = 0 as libc::c_int as u32;
    loop {
        let fresh41 = count;
        count = count.wrapping_sub(1);
        if !(fresh41 as libc::c_int > 0 as libc::c_int) {
            break;
        }
        let fresh42 = pbuffer;
        pbuffer = pbuffer.offset(1);
        value = value << 8 as libc::c_int | *fresh42 as u32;
    }
    return value;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_i2c_decode_with_mask(
    mut count: u16,
    mut pbuffer: *mut u8,
    mut bit_mask: u32,
    mut down_shift: u32,
    mut offset: u32,
) -> u32 {
    let mut value: u32 = 0 as libc::c_int as u32;
    loop {
        let fresh43 = count;
        count = count.wrapping_sub(1);
        if !(fresh43 as libc::c_int > 0 as libc::c_int) {
            break;
        }
        let fresh44 = pbuffer;
        pbuffer = pbuffer.offset(1);
        value = value << 8 as libc::c_int | *fresh44 as u32;
    }
    value = value & bit_mask;
    if down_shift > 0 as libc::c_int as libc::c_uint {
        value = value >> down_shift;
    }
    value = value.wrapping_add(offset);
    return value;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_i2c_encode_int32_t(
    mut ip_value: i32,
    mut count: u16,
    mut pbuffer: *mut u8,
) {
    let mut i: u16 = 0 as libc::c_int as u16;
    let mut VL53LX_p_003: i32 = 0 as libc::c_int;
    VL53LX_p_003 = ip_value;
    i = 0 as libc::c_int as u16;
    while (i as libc::c_int) < count as libc::c_int {
        *pbuffer
            .offset(
                (count as libc::c_int - i as libc::c_int - 1 as libc::c_int) as isize,
            ) = (VL53LX_p_003 & 0xff as libc::c_int) as u8;
        VL53LX_p_003 = VL53LX_p_003 >> 8 as libc::c_int;
        i = i.wrapping_add(1);
    }
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_i2c_decode_int32_t(
    mut count: u16,
    mut pbuffer: *mut u8,
) -> i32 {
    let mut value: i32 = 0 as libc::c_int;
    if *pbuffer as libc::c_int >= 0x80 as libc::c_int {
        value = 0xffffffff as libc::c_uint as i32;
    }
    loop {
        let fresh45 = count;
        count = count.wrapping_sub(1);
        if !(fresh45 as libc::c_int > 0 as libc::c_int) {
            break;
        }
        let fresh46 = pbuffer;
        pbuffer = pbuffer.offset(1);
        value = value << 8 as libc::c_int | *fresh46 as i32;
    }
    return value;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_start_test(
    mut Dev: VL53LX_DEV,
    mut test_mode__ctrl: u8,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_WrByte(Dev, 0x27 as libc::c_int as u16, test_mode__ctrl);
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_set_firmware_enable_register(
    mut Dev: VL53LX_DEV,
    mut value: u8,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    (*pdev).sys_ctrl.firmware__enable = value;
    status = VL53LX_WrByte(
        Dev,
        0x85 as libc::c_int as u16,
        (*pdev).sys_ctrl.firmware__enable,
    );
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_enable_firmware(mut Dev: VL53LX_DEV) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    status = VL53LX_set_firmware_enable_register(Dev, 0x1 as libc::c_int as u8);
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_disable_firmware(mut Dev: VL53LX_DEV) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    status = VL53LX_set_firmware_enable_register(Dev, 0 as libc::c_int as u8);
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_set_powerforce_register(
    mut Dev: VL53LX_DEV,
    mut value: u8,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    (*pdev).sys_ctrl.power_management__go1_power_force = value;
    status = VL53LX_WrByte(
        Dev,
        0x83 as libc::c_int as u16,
        (*pdev).sys_ctrl.power_management__go1_power_force,
    );
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_enable_powerforce(mut Dev: VL53LX_DEV) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    status = VL53LX_set_powerforce_register(Dev, 0x1 as libc::c_int as u8);
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_disable_powerforce(mut Dev: VL53LX_DEV) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    status = VL53LX_set_powerforce_register(Dev, 0 as libc::c_int as u8);
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_clear_interrupt(mut Dev: VL53LX_DEV) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    (*pdev).sys_ctrl.system__interrupt_clear = 0x1 as libc::c_int as u8;
    status = VL53LX_WrByte(
        Dev,
        0x86 as libc::c_int as u16,
        (*pdev).sys_ctrl.system__interrupt_clear,
    );
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_force_shadow_stream_count_to_zero(
    mut Dev: VL53LX_DEV,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_disable_firmware(Dev);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_WrByte(
            Dev,
            0xfb3 as libc::c_int as u16,
            0 as libc::c_int as u8,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_enable_firmware(Dev);
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_calc_macro_period_us(
    mut fast_osc_frequency: u16,
    mut VL53LX_p_005: u8,
) -> u32 {
    let mut pll_period_us: u32 = 0 as libc::c_int as u32;
    let mut VL53LX_p_030: u8 = 0 as libc::c_int as u8;
    let mut macro_period_us: u32 = 0 as libc::c_int as u32;
    pll_period_us = VL53LX_calc_pll_period_us(fast_osc_frequency);
    VL53LX_p_030 = VL53LX_decode_vcsel_period(VL53LX_p_005);
    macro_period_us = ((256 as libc::c_int + 2048 as libc::c_int) as u32)
        .wrapping_mul(pll_period_us);
    macro_period_us = macro_period_us >> 6 as libc::c_int;
    macro_period_us = macro_period_us.wrapping_mul(VL53LX_p_030 as u32);
    macro_period_us = macro_period_us >> 6 as libc::c_int;
    return macro_period_us;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_calc_range_ignore_threshold(
    mut central_rate: u32,
    mut x_gradient: i16,
    mut y_gradient: i16,
    mut rate_mult: u8,
) -> u16 {
    let mut range_ignore_thresh_int: i32 = 0 as libc::c_int;
    let mut range_ignore_thresh_kcps: u16 = 0 as libc::c_int as u16;
    let mut central_rate_int: i32 = 0 as libc::c_int;
    let mut x_gradient_int: i16 = 0 as libc::c_int as i16;
    let mut y_gradient_int: i16 = 0 as libc::c_int as i16;
    central_rate_int = central_rate as i32 * ((1 as libc::c_int) << 4 as libc::c_int)
        / 1000 as libc::c_int;
    if (x_gradient as libc::c_int) < 0 as libc::c_int {
        x_gradient_int = (x_gradient as libc::c_int * -(1 as libc::c_int)) as i16;
    }
    if (y_gradient as libc::c_int) < 0 as libc::c_int {
        y_gradient_int = (y_gradient as libc::c_int * -(1 as libc::c_int)) as i16;
    }
    range_ignore_thresh_int = 8 as libc::c_int * x_gradient_int as libc::c_int
        * 4 as libc::c_int
        + 8 as libc::c_int * y_gradient_int as libc::c_int * 4 as libc::c_int;
    range_ignore_thresh_int = range_ignore_thresh_int / 1000 as libc::c_int;
    range_ignore_thresh_int = range_ignore_thresh_int + central_rate_int;
    range_ignore_thresh_int = rate_mult as i32 * range_ignore_thresh_int;
    range_ignore_thresh_int = (range_ignore_thresh_int
        + ((1 as libc::c_int) << 4 as libc::c_int))
        / ((1 as libc::c_int) << 5 as libc::c_int);
    if range_ignore_thresh_int > 0xffff as libc::c_int {
        range_ignore_thresh_kcps = 0xffff as libc::c_int as u16;
    } else {
        range_ignore_thresh_kcps = range_ignore_thresh_int as u16;
    }
    return range_ignore_thresh_kcps;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_calc_timeout_mclks(
    mut timeout_us: u32,
    mut macro_period_us: u32,
) -> u32 {
    let mut timeout_mclks: u32 = 0 as libc::c_int as u32;
    if macro_period_us == 0 as libc::c_int as libc::c_uint {
        timeout_mclks = 0 as libc::c_int as u32;
    } else {
        timeout_mclks = (timeout_us << 12 as libc::c_int)
            .wrapping_add(macro_period_us >> 1 as libc::c_int)
            .wrapping_div(macro_period_us);
    }
    return timeout_mclks;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_calc_encoded_timeout(
    mut timeout_us: u32,
    mut macro_period_us: u32,
) -> u16 {
    let mut timeout_mclks: u32 = 0 as libc::c_int as u32;
    let mut timeout_encoded: u16 = 0 as libc::c_int as u16;
    timeout_mclks = VL53LX_calc_timeout_mclks(timeout_us, macro_period_us);
    timeout_encoded = VL53LX_encode_timeout(timeout_mclks);
    return timeout_encoded;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_calc_timeout_us(
    mut timeout_mclks: u32,
    mut macro_period_us: u32,
) -> u32 {
    let mut timeout_us: u32 = 0 as libc::c_int as u32;
    let mut tmp: u64 = 0 as libc::c_int as u64;
    tmp = (timeout_mclks as u64).wrapping_mul(macro_period_us as u64);
    tmp = (tmp as libc::c_ulong).wrapping_add(0x800 as libc::c_int as libc::c_ulong)
        as u64 as u64;
    tmp = tmp >> 12 as libc::c_int;
    timeout_us = tmp as u32;
    return timeout_us;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_calc_crosstalk_plane_offset_with_margin(
    mut plane_offset_kcps: u32,
    mut margin_offset_kcps: i16,
) -> u32 {
    let mut plane_offset_with_margin: u32 = 0 as libc::c_int as u32;
    let mut plane_offset_kcps_temp: i32 = 0 as libc::c_int;
    plane_offset_kcps_temp = plane_offset_kcps as i32
        + margin_offset_kcps as i32;
    if plane_offset_kcps_temp < 0 as libc::c_int {
        plane_offset_kcps_temp = 0 as libc::c_int;
    } else if plane_offset_kcps_temp > 0x3ffff as libc::c_int {
        plane_offset_kcps_temp = 0x3ffff as libc::c_int;
    }
    plane_offset_with_margin = plane_offset_kcps_temp as u32;
    return plane_offset_with_margin;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_calc_decoded_timeout_us(
    mut timeout_encoded: u16,
    mut macro_period_us: u32,
) -> u32 {
    let mut timeout_mclks: u32 = 0 as libc::c_int as u32;
    let mut timeout_us: u32 = 0 as libc::c_int as u32;
    timeout_mclks = VL53LX_decode_timeout(timeout_encoded);
    timeout_us = VL53LX_calc_timeout_us(timeout_mclks, macro_period_us);
    return timeout_us;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_encode_timeout(mut timeout_mclks: u32) -> u16 {
    let mut encoded_timeout: u16 = 0 as libc::c_int as u16;
    let mut ls_byte: u32 = 0 as libc::c_int as u32;
    let mut ms_byte: u16 = 0 as libc::c_int as u16;
    if timeout_mclks > 0 as libc::c_int as libc::c_uint {
        ls_byte = timeout_mclks.wrapping_sub(1 as libc::c_int as libc::c_uint);
        while ls_byte & 0xffffff00 as libc::c_uint > 0 as libc::c_int as libc::c_uint {
            ls_byte = ls_byte >> 1 as libc::c_int;
            ms_byte = ms_byte.wrapping_add(1);
        }
        encoded_timeout = (((ms_byte as libc::c_int) << 8 as libc::c_int)
            + (ls_byte & 0xff as libc::c_int as libc::c_uint) as u16 as libc::c_int)
            as u16;
    }
    return encoded_timeout;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_decode_timeout(
    mut encoded_timeout: u16,
) -> u32 {
    let mut timeout_macro_clks: u32 = 0 as libc::c_int as u32;
    timeout_macro_clks = (((encoded_timeout as libc::c_int & 0xff as libc::c_int)
        as u32)
        << ((encoded_timeout as libc::c_int & 0xff00 as libc::c_int) >> 8 as libc::c_int)
            as u32)
        .wrapping_add(1 as libc::c_int as libc::c_uint);
    return timeout_macro_clks;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_calc_timeout_register_values(
    mut phasecal_config_timeout_us: u32,
    mut mm_config_timeout_us: u32,
    mut range_config_timeout_us: u32,
    mut fast_osc_frequency: u16,
    mut pgeneral: *mut VL53LX_general_config_t,
    mut ptiming: *mut VL53LX_timing_config_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut macro_period_us: u32 = 0 as libc::c_int as u32;
    let mut timeout_mclks: u32 = 0 as libc::c_int as u32;
    let mut timeout_encoded: u16 = 0 as libc::c_int as u16;
    if fast_osc_frequency as libc::c_int == 0 as libc::c_int {
        status = -(15 as libc::c_int) as VL53LX_Error;
    } else {
        macro_period_us = VL53LX_calc_macro_period_us(
            fast_osc_frequency,
            (*ptiming).range_config__vcsel_period_a,
        );
        timeout_mclks = VL53LX_calc_timeout_mclks(
            phasecal_config_timeout_us,
            macro_period_us,
        );
        if timeout_mclks > 0xff as libc::c_int as libc::c_uint {
            timeout_mclks = 0xff as libc::c_int as u32;
        }
        (*pgeneral).phasecal_config__timeout_macrop = timeout_mclks as u8;
        timeout_encoded = VL53LX_calc_encoded_timeout(
            mm_config_timeout_us,
            macro_period_us,
        );
        (*ptiming)
            .mm_config__timeout_macrop_a_hi = ((timeout_encoded as libc::c_int
            & 0xff00 as libc::c_int) >> 8 as libc::c_int) as u8;
        (*ptiming)
            .mm_config__timeout_macrop_a_lo = (timeout_encoded as libc::c_int
            & 0xff as libc::c_int) as u8;
        timeout_encoded = VL53LX_calc_encoded_timeout(
            range_config_timeout_us,
            macro_period_us,
        );
        (*ptiming)
            .range_config__timeout_macrop_a_hi = ((timeout_encoded as libc::c_int
            & 0xff00 as libc::c_int) >> 8 as libc::c_int) as u8;
        (*ptiming)
            .range_config__timeout_macrop_a_lo = (timeout_encoded as libc::c_int
            & 0xff as libc::c_int) as u8;
        macro_period_us = VL53LX_calc_macro_period_us(
            fast_osc_frequency,
            (*ptiming).range_config__vcsel_period_b,
        );
        timeout_encoded = VL53LX_calc_encoded_timeout(
            mm_config_timeout_us,
            macro_period_us,
        );
        (*ptiming)
            .mm_config__timeout_macrop_b_hi = ((timeout_encoded as libc::c_int
            & 0xff00 as libc::c_int) >> 8 as libc::c_int) as u8;
        (*ptiming)
            .mm_config__timeout_macrop_b_lo = (timeout_encoded as libc::c_int
            & 0xff as libc::c_int) as u8;
        timeout_encoded = VL53LX_calc_encoded_timeout(
            range_config_timeout_us,
            macro_period_us,
        );
        (*ptiming)
            .range_config__timeout_macrop_b_hi = ((timeout_encoded as libc::c_int
            & 0xff00 as libc::c_int) >> 8 as libc::c_int) as u8;
        (*ptiming)
            .range_config__timeout_macrop_b_lo = (timeout_encoded as libc::c_int
            & 0xff as libc::c_int) as u8;
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_encode_vcsel_period(
    mut VL53LX_p_030: u8,
) -> u8 {
    let mut vcsel_period_reg: u8 = 0 as libc::c_int as u8;
    vcsel_period_reg = ((VL53LX_p_030 as libc::c_int >> 1 as libc::c_int)
        - 1 as libc::c_int) as u8;
    return vcsel_period_reg;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_decode_unsigned_integer(
    mut pbuffer: *mut u8,
    mut no_of_bytes: u8,
) -> u32 {
    let mut i: u8 = 0 as libc::c_int as u8;
    let mut decoded_value: u32 = 0 as libc::c_int as u32;
    i = 0 as libc::c_int as u8;
    while (i as libc::c_int) < no_of_bytes as libc::c_int {
        decoded_value = (decoded_value << 8 as libc::c_int)
            .wrapping_add(*pbuffer.offset(i as isize) as u32);
        i = i.wrapping_add(1);
    }
    return decoded_value;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_encode_unsigned_integer(
    mut ip_value: u32,
    mut no_of_bytes: u8,
    mut pbuffer: *mut u8,
) {
    let mut i: u8 = 0 as libc::c_int as u8;
    let mut VL53LX_p_003: u32 = 0 as libc::c_int as u32;
    VL53LX_p_003 = ip_value;
    i = 0 as libc::c_int as u8;
    while (i as libc::c_int) < no_of_bytes as libc::c_int {
        *pbuffer
            .offset(
                (no_of_bytes as libc::c_int - i as libc::c_int - 1 as libc::c_int)
                    as isize,
            ) = (VL53LX_p_003 & 0xff as libc::c_int as libc::c_uint) as u8;
        VL53LX_p_003 = VL53LX_p_003 >> 8 as libc::c_int;
        i = i.wrapping_add(1);
    }
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_hist_copy_and_scale_ambient_info(
    mut pidata: *mut VL53LX_zone_hist_info_t,
    mut podata: *mut VL53LX_histogram_bin_data_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut evts: i64 = 0 as libc::c_int as i64;
    let mut tmpi: i64 = 0 as libc::c_int as i64;
    let mut tmpo: i64 = 0 as libc::c_int as i64;
    if (*pidata).result__dss_actual_effective_spads as libc::c_int == 0 as libc::c_int {
        status = -(15 as libc::c_int) as VL53LX_Error;
    } else if (*pidata).number_of_ambient_bins as libc::c_int > 0 as libc::c_int
            && (*podata).number_of_ambient_bins as libc::c_int == 0 as libc::c_int
        {
        tmpo = 1 as libc::c_int as libc::c_long
            + (*podata).total_periods_elapsed as i64;
        tmpo *= (*podata).result__dss_actual_effective_spads as i64;
        tmpi = 1 as libc::c_int as libc::c_long
            + (*pidata).total_periods_elapsed as i64;
        tmpi *= (*pidata).result__dss_actual_effective_spads as i64;
        evts = tmpo * (*pidata).ambient_events_sum as i64;
        evts += tmpi / 2 as libc::c_int as libc::c_long;
        if tmpi != 0 as libc::c_int as libc::c_long {
            evts = evts / tmpi;
        }
        (*podata).ambient_events_sum = evts as i32;
        (*podata).VL53LX_p_028 = (*podata).ambient_events_sum;
        let ref mut fresh47 = (*podata).VL53LX_p_028;
        *fresh47 += (*pidata).number_of_ambient_bins as i32 / 2 as libc::c_int;
        let ref mut fresh48 = (*podata).VL53LX_p_028;
        *fresh48 /= (*pidata).number_of_ambient_bins as i32;
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_hist_get_bin_sequence_config(
    mut Dev: VL53LX_DEV,
    mut pdata: *mut VL53LX_histogram_bin_data_t,
) {
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    let mut amb_thresh_low: i32 = 0 as libc::c_int;
    let mut amb_thresh_high: i32 = 0 as libc::c_int;
    let mut i: u8 = 0 as libc::c_int as u8;
    amb_thresh_low = 1024 as libc::c_int
        * (*pdev).hist_cfg.histogram_config__amb_thresh_low as i32;
    amb_thresh_high = 1024 as libc::c_int
        * (*pdev).hist_cfg.histogram_config__amb_thresh_high as i32;
    if (*pdev).ll_state.rd_stream_count as libc::c_int & 0x1 as libc::c_int
        == 0 as libc::c_int
    {
        (*pdata)
            .bin_seq[5 as libc::c_int
            as usize] = ((*pdev).hist_cfg.histogram_config__mid_amb_even_bin_4_5
            as libc::c_int >> 4 as libc::c_int) as u8;
        (*pdata)
            .bin_seq[4 as libc::c_int
            as usize] = ((*pdev).hist_cfg.histogram_config__mid_amb_even_bin_4_5
            as libc::c_int & 0xf as libc::c_int) as u8;
        (*pdata)
            .bin_seq[3 as libc::c_int
            as usize] = ((*pdev).hist_cfg.histogram_config__mid_amb_even_bin_2_3
            as libc::c_int >> 4 as libc::c_int) as u8;
        (*pdata)
            .bin_seq[2 as libc::c_int
            as usize] = ((*pdev).hist_cfg.histogram_config__mid_amb_even_bin_2_3
            as libc::c_int & 0xf as libc::c_int) as u8;
        (*pdata)
            .bin_seq[1 as libc::c_int
            as usize] = ((*pdev).hist_cfg.histogram_config__mid_amb_even_bin_0_1
            as libc::c_int >> 4 as libc::c_int) as u8;
        (*pdata)
            .bin_seq[0 as libc::c_int
            as usize] = ((*pdev).hist_cfg.histogram_config__mid_amb_even_bin_0_1
            as libc::c_int & 0xf as libc::c_int) as u8;
        if (*pdata).ambient_events_sum > amb_thresh_high {
            (*pdata)
                .bin_seq[5 as libc::c_int
                as usize] = ((*pdev).hist_cfg.histogram_config__high_amb_even_bin_4_5
                as libc::c_int >> 4 as libc::c_int) as u8;
            (*pdata)
                .bin_seq[4 as libc::c_int
                as usize] = ((*pdev).hist_cfg.histogram_config__high_amb_even_bin_4_5
                as libc::c_int & 0xf as libc::c_int) as u8;
            (*pdata)
                .bin_seq[3 as libc::c_int
                as usize] = ((*pdev).hist_cfg.histogram_config__high_amb_even_bin_2_3
                as libc::c_int >> 4 as libc::c_int) as u8;
            (*pdata)
                .bin_seq[2 as libc::c_int
                as usize] = ((*pdev).hist_cfg.histogram_config__high_amb_even_bin_2_3
                as libc::c_int & 0xf as libc::c_int) as u8;
            (*pdata)
                .bin_seq[1 as libc::c_int
                as usize] = ((*pdev).hist_cfg.histogram_config__high_amb_even_bin_0_1
                as libc::c_int >> 4 as libc::c_int) as u8;
            (*pdata)
                .bin_seq[0 as libc::c_int
                as usize] = ((*pdev).hist_cfg.histogram_config__high_amb_even_bin_0_1
                as libc::c_int & 0xf as libc::c_int) as u8;
        }
        if (*pdata).ambient_events_sum < amb_thresh_low {
            (*pdata)
                .bin_seq[5 as libc::c_int
                as usize] = ((*pdev).hist_cfg.histogram_config__low_amb_even_bin_4_5
                as libc::c_int >> 4 as libc::c_int) as u8;
            (*pdata)
                .bin_seq[4 as libc::c_int
                as usize] = ((*pdev).hist_cfg.histogram_config__low_amb_even_bin_4_5
                as libc::c_int & 0xf as libc::c_int) as u8;
            (*pdata)
                .bin_seq[3 as libc::c_int
                as usize] = ((*pdev).hist_cfg.histogram_config__low_amb_even_bin_2_3
                as libc::c_int >> 4 as libc::c_int) as u8;
            (*pdata)
                .bin_seq[2 as libc::c_int
                as usize] = ((*pdev).hist_cfg.histogram_config__low_amb_even_bin_2_3
                as libc::c_int & 0xf as libc::c_int) as u8;
            (*pdata)
                .bin_seq[1 as libc::c_int
                as usize] = ((*pdev).hist_cfg.histogram_config__low_amb_even_bin_0_1
                as libc::c_int >> 4 as libc::c_int) as u8;
            (*pdata)
                .bin_seq[0 as libc::c_int
                as usize] = ((*pdev).hist_cfg.histogram_config__low_amb_even_bin_0_1
                as libc::c_int & 0xf as libc::c_int) as u8;
        }
    } else {
        (*pdata)
            .bin_seq[5 as libc::c_int
            as usize] = ((*pdev).hist_cfg.histogram_config__mid_amb_odd_bin_5
            as libc::c_int & 0xf as libc::c_int) as u8;
        (*pdata)
            .bin_seq[4 as libc::c_int
            as usize] = ((*pdev).hist_cfg.histogram_config__mid_amb_odd_bin_3_4
            as libc::c_int & 0xf as libc::c_int) as u8;
        (*pdata)
            .bin_seq[3 as libc::c_int
            as usize] = ((*pdev).hist_cfg.histogram_config__mid_amb_odd_bin_3_4
            as libc::c_int >> 4 as libc::c_int) as u8;
        (*pdata)
            .bin_seq[2 as libc::c_int
            as usize] = ((*pdev).hist_cfg.histogram_config__mid_amb_odd_bin_2
            as libc::c_int & 0xf as libc::c_int) as u8;
        (*pdata)
            .bin_seq[1 as libc::c_int
            as usize] = ((*pdev).hist_cfg.histogram_config__mid_amb_odd_bin_0_1
            as libc::c_int >> 4 as libc::c_int) as u8;
        (*pdata)
            .bin_seq[0 as libc::c_int
            as usize] = ((*pdev).hist_cfg.histogram_config__mid_amb_odd_bin_0_1
            as libc::c_int & 0xf as libc::c_int) as u8;
        if (*pdata).ambient_events_sum > amb_thresh_high {
            (*pdata)
                .bin_seq[5 as libc::c_int
                as usize] = ((*pdev).hist_cfg.histogram_config__high_amb_odd_bin_4_5
                as libc::c_int >> 4 as libc::c_int) as u8;
            (*pdata)
                .bin_seq[4 as libc::c_int
                as usize] = ((*pdev).hist_cfg.histogram_config__high_amb_odd_bin_4_5
                as libc::c_int & 0xf as libc::c_int) as u8;
            (*pdata)
                .bin_seq[3 as libc::c_int
                as usize] = ((*pdev).hist_cfg.histogram_config__high_amb_odd_bin_2_3
                as libc::c_int >> 4 as libc::c_int) as u8;
            (*pdata)
                .bin_seq[2 as libc::c_int
                as usize] = ((*pdev).hist_cfg.histogram_config__high_amb_odd_bin_2_3
                as libc::c_int & 0xf as libc::c_int) as u8;
            (*pdata)
                .bin_seq[1 as libc::c_int
                as usize] = ((*pdev).hist_cfg.histogram_config__high_amb_odd_bin_0_1
                as libc::c_int >> 4 as libc::c_int) as u8;
            (*pdata)
                .bin_seq[0 as libc::c_int
                as usize] = ((*pdev).hist_cfg.histogram_config__high_amb_odd_bin_0_1
                as libc::c_int & 0xf as libc::c_int) as u8;
        }
        if (*pdata).ambient_events_sum < amb_thresh_low {
            (*pdata)
                .bin_seq[5 as libc::c_int
                as usize] = ((*pdev).hist_cfg.histogram_config__low_amb_odd_bin_4_5
                as libc::c_int >> 4 as libc::c_int) as u8;
            (*pdata)
                .bin_seq[4 as libc::c_int
                as usize] = ((*pdev).hist_cfg.histogram_config__low_amb_odd_bin_4_5
                as libc::c_int & 0xf as libc::c_int) as u8;
            (*pdata)
                .bin_seq[3 as libc::c_int
                as usize] = ((*pdev).hist_cfg.histogram_config__low_amb_odd_bin_2_3
                as libc::c_int >> 4 as libc::c_int) as u8;
            (*pdata)
                .bin_seq[2 as libc::c_int
                as usize] = ((*pdev).hist_cfg.histogram_config__low_amb_odd_bin_2_3
                as libc::c_int & 0xf as libc::c_int) as u8;
            (*pdata)
                .bin_seq[1 as libc::c_int
                as usize] = ((*pdev).hist_cfg.histogram_config__low_amb_odd_bin_0_1
                as libc::c_int >> 4 as libc::c_int) as u8;
            (*pdata)
                .bin_seq[0 as libc::c_int
                as usize] = ((*pdev).hist_cfg.histogram_config__low_amb_odd_bin_0_1
                as libc::c_int & 0xf as libc::c_int) as u8;
        }
    }
    i = 0 as libc::c_int as u8;
    while (i as libc::c_int) < 6 as libc::c_int {
        (*pdata).bin_rep[i as usize] = 1 as libc::c_int as u8;
        i = i.wrapping_add(1);
    }
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_hist_phase_consistency_check(
    mut Dev: VL53LX_DEV,
    mut phist_prev: *mut VL53LX_zone_hist_info_t,
    mut prange_prev: *mut VL53LX_zone_objects_t,
    mut prange_curr: *mut VL53LX_range_results_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    let mut lc: u8 = 0 as libc::c_int as u8;
    let mut p: u8 = 0 as libc::c_int as u8;
    let mut phase_delta: u16 = 0 as libc::c_int as u16;
    let mut phase_tolerance: u16 = 0 as libc::c_int as u16;
    let mut events_delta: i32 = 0 as libc::c_int;
    let mut events_tolerance: i32 = 0 as libc::c_int;
    let mut event_sigma: u8 = 0;
    let mut event_min_spad_count: u16 = 0;
    let mut min_max_tolerance: u16 = 0;
    let mut pht: u8 = 0;
    let mut range_status: VL53LX_DeviceError = 0 as libc::c_int as VL53LX_DeviceError;
    event_sigma = (*pdev).histpostprocess.algo__consistency_check__event_sigma;
    event_min_spad_count = (*pdev)
        .histpostprocess
        .algo__consistency_check__event_min_spad_count;
    min_max_tolerance = (*pdev)
        .histpostprocess
        .algo__consistency_check__min_max_tolerance;
    pht = (*pdev).histpostprocess.algo__consistency_check__phase_tolerance;
    phase_tolerance = pht as u16;
    phase_tolerance = ((phase_tolerance as libc::c_int) << 8 as libc::c_int) as u16;
    if (*prange_prev).rd_device_state as libc::c_int
        != 7 as libc::c_int as VL53LX_DeviceState as libc::c_int
        && (*prange_prev).rd_device_state as libc::c_int
            != 8 as libc::c_int as VL53LX_DeviceState as libc::c_int
    {
        return status;
    }
    if phase_tolerance as libc::c_int == 0 as libc::c_int {
        return status;
    }
    lc = 0 as libc::c_int as u8;
    while (lc as libc::c_int) < (*prange_curr).active_results as libc::c_int {
        if (*prange_curr).VL53LX_p_003[lc as usize].range_status as libc::c_int
            == 9 as libc::c_int as VL53LX_DeviceError as libc::c_int
            || (*prange_curr).VL53LX_p_003[lc as usize].range_status as libc::c_int
                == 19 as libc::c_int as VL53LX_DeviceError as libc::c_int
        {
            if (*prange_prev).active_objects as libc::c_int == 0 as libc::c_int {
                (*prange_curr)
                    .VL53LX_p_003[lc as usize]
                    .range_status = 23 as libc::c_int as VL53LX_DeviceError;
            } else {
                (*prange_curr)
                    .VL53LX_p_003[lc as usize]
                    .range_status = 7 as libc::c_int as VL53LX_DeviceError;
            }
            p = 0 as libc::c_int as u8;
            while (p as libc::c_int) < (*prange_prev).active_objects as libc::c_int {
                if (*prange_curr).VL53LX_p_003[lc as usize].VL53LX_p_011 as libc::c_int
                    > (*prange_prev).VL53LX_p_003[p as usize].VL53LX_p_011 as libc::c_int
                {
                    phase_delta = ((*prange_curr).VL53LX_p_003[lc as usize].VL53LX_p_011
                        as libc::c_int
                        - (*prange_prev).VL53LX_p_003[p as usize].VL53LX_p_011
                            as libc::c_int) as u16;
                } else {
                    phase_delta = ((*prange_prev).VL53LX_p_003[p as usize].VL53LX_p_011
                        as libc::c_int
                        - (*prange_curr).VL53LX_p_003[lc as usize].VL53LX_p_011
                            as libc::c_int) as u16;
                }
                if (phase_delta as libc::c_int) < phase_tolerance as libc::c_int {
                    if status as libc::c_int
                        == 0 as libc::c_int as VL53LX_Error as libc::c_int
                    {
                        status = VL53LX_hist_events_consistency_check(
                            event_sigma,
                            event_min_spad_count,
                            phist_prev,
                            &mut *((*prange_prev).VL53LX_p_003)
                                .as_mut_ptr()
                                .offset(p as isize),
                            &mut *((*prange_curr).VL53LX_p_003)
                                .as_mut_ptr()
                                .offset(lc as isize),
                            &mut events_tolerance,
                            &mut events_delta,
                            &mut range_status,
                        );
                    }
                    if status as libc::c_int
                        == 0 as libc::c_int as VL53LX_Error as libc::c_int
                        && range_status as libc::c_int
                            == 9 as libc::c_int as VL53LX_DeviceError as libc::c_int
                    {
                        status = VL53LX_hist_merged_pulse_check(
                            min_max_tolerance as i16,
                            &mut *((*prange_curr).VL53LX_p_003)
                                .as_mut_ptr()
                                .offset(lc as isize),
                            &mut range_status,
                        );
                    }
                    (*prange_curr).VL53LX_p_003[lc as usize].range_status = range_status;
                }
                p = p.wrapping_add(1);
            }
        }
        lc = lc.wrapping_add(1);
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_hist_events_consistency_check(
    mut event_sigma: u8,
    mut min_effective_spad_count: u16,
    mut phist_prev: *mut VL53LX_zone_hist_info_t,
    mut prange_prev: *mut VL53LX_object_data_t,
    mut prange_curr: *mut VL53LX_range_data_t,
    mut pevents_tolerance: *mut i32,
    mut pevents_delta: *mut i32,
    mut prange_status: *mut VL53LX_DeviceError,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut tmpp: i64 = 0 as libc::c_int as i64;
    let mut tmpc: i64 = 0 as libc::c_int as i64;
    let mut events_scaler: i64 = 0 as libc::c_int as i64;
    let mut events_scaler_sq: i64 = 0 as libc::c_int as i64;
    let mut c_signal_events: i64 = 0 as libc::c_int as i64;
    let mut c_sig_noise_sq: i64 = 0 as libc::c_int as i64;
    let mut c_amb_noise_sq: i64 = 0 as libc::c_int as i64;
    let mut p_amb_noise_sq: i64 = 0 as libc::c_int as i64;
    let mut p_signal_events: i32 = 0 as libc::c_int;
    let mut noise_sq_sum: u32 = 0 as libc::c_int as u32;
    if event_sigma as libc::c_int == 0 as libc::c_int {
        *prange_status = 9 as libc::c_int as VL53LX_DeviceError;
        return status;
    }
    tmpp = 1 as libc::c_int as libc::c_long
        + (*phist_prev).total_periods_elapsed as i64;
    tmpp *= (*phist_prev).result__dss_actual_effective_spads as i64;
    tmpc = 1 as libc::c_int as libc::c_long
        + (*prange_curr).total_periods_elapsed as i64;
    tmpc *= (*prange_curr).VL53LX_p_004 as i64;
    events_scaler = tmpp * 4096 as libc::c_int as libc::c_long;
    if tmpc != 0 as libc::c_int as libc::c_long {
        events_scaler += tmpc / 2 as libc::c_int as libc::c_long;
        events_scaler = events_scaler / tmpc;
    }
    events_scaler_sq = events_scaler * events_scaler;
    events_scaler_sq += 2048 as libc::c_int as libc::c_long;
    events_scaler_sq /= 4096 as libc::c_int as libc::c_long;
    c_signal_events = (*prange_curr).VL53LX_p_017 as i64;
    c_signal_events -= (*prange_curr).VL53LX_p_016 as i64;
    c_signal_events *= events_scaler;
    c_signal_events += 2048 as libc::c_int as libc::c_long;
    c_signal_events /= 4096 as libc::c_int as libc::c_long;
    c_sig_noise_sq = events_scaler_sq;
    c_sig_noise_sq *= (*prange_curr).VL53LX_p_017 as i64;
    c_sig_noise_sq += 2048 as libc::c_int as libc::c_long;
    c_sig_noise_sq /= 4096 as libc::c_int as libc::c_long;
    c_amb_noise_sq = events_scaler_sq;
    c_amb_noise_sq *= (*prange_curr).VL53LX_p_016 as i64;
    c_amb_noise_sq += 2048 as libc::c_int as libc::c_long;
    c_amb_noise_sq /= 4096 as libc::c_int as libc::c_long;
    c_amb_noise_sq += 2 as libc::c_int as libc::c_long;
    c_amb_noise_sq /= 4 as libc::c_int as libc::c_long;
    p_amb_noise_sq = (*prange_prev).VL53LX_p_016 as i64;
    p_amb_noise_sq += 2 as libc::c_int as libc::c_long;
    p_amb_noise_sq /= 4 as libc::c_int as libc::c_long;
    noise_sq_sum = ((*prange_prev).VL53LX_p_017)
        .wrapping_add(c_sig_noise_sq as u32)
        .wrapping_add(p_amb_noise_sq as u32)
        .wrapping_add(c_amb_noise_sq as u32);
    *pevents_tolerance = VL53LX_isqrt(
        noise_sq_sum.wrapping_mul(16 as libc::c_int as libc::c_uint),
    ) as i32;
    *pevents_tolerance *= event_sigma as i32;
    *pevents_tolerance += 32 as libc::c_int;
    *pevents_tolerance /= 64 as libc::c_int;
    p_signal_events = (*prange_prev).VL53LX_p_017 as i32;
    p_signal_events -= (*prange_prev).VL53LX_p_016 as i32;
    if c_signal_events as i32 > p_signal_events {
        *pevents_delta = c_signal_events as i32 - p_signal_events;
    } else {
        *pevents_delta = p_signal_events - c_signal_events as i32;
    }
    if *pevents_delta > *pevents_tolerance
        && (*prange_curr).VL53LX_p_004 as libc::c_int
            > min_effective_spad_count as libc::c_int
    {
        *prange_status = 20 as libc::c_int as VL53LX_DeviceError;
    } else {
        *prange_status = 9 as libc::c_int as VL53LX_DeviceError;
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_hist_merged_pulse_check(
    mut min_max_tolerance_mm: i16,
    mut pdata: *mut VL53LX_range_data_t,
    mut prange_status: *mut VL53LX_DeviceError,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut delta_mm: i16 = 0 as libc::c_int as i16;
    if (*pdata).max_range_mm as libc::c_int > (*pdata).min_range_mm as libc::c_int {
        delta_mm = ((*pdata).max_range_mm as libc::c_int
            - (*pdata).min_range_mm as libc::c_int) as i16;
    } else {
        delta_mm = ((*pdata).min_range_mm as libc::c_int
            - (*pdata).max_range_mm as libc::c_int) as i16;
    }
    if min_max_tolerance_mm as libc::c_int > 0 as libc::c_int
        && delta_mm as libc::c_int > min_max_tolerance_mm as libc::c_int
    {
        *prange_status = 22 as libc::c_int as VL53LX_DeviceError;
    } else {
        *prange_status = 9 as libc::c_int as VL53LX_DeviceError;
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_hist_xmonitor_consistency_check(
    mut Dev: VL53LX_DEV,
    mut phist_prev: *mut VL53LX_zone_hist_info_t,
    mut prange_prev: *mut VL53LX_zone_objects_t,
    mut prange_curr: *mut VL53LX_range_data_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    let mut events_delta: i32 = 0 as libc::c_int;
    let mut events_tolerance: i32 = 0 as libc::c_int;
    let mut event_sigma: u8 = 0;
    let mut min_spad_count: u16 = 0;
    event_sigma = (*pdev).histpostprocess.algo__crosstalk_detect_event_sigma;
    min_spad_count = (*pdev)
        .histpostprocess
        .algo__consistency_check__event_min_spad_count;
    if (*prange_curr).range_status as libc::c_int
        == 9 as libc::c_int as VL53LX_DeviceError as libc::c_int
        || (*prange_curr).range_status as libc::c_int
            == 19 as libc::c_int as VL53LX_DeviceError as libc::c_int
        || (*prange_curr).range_status as libc::c_int
            == 20 as libc::c_int as VL53LX_DeviceError as libc::c_int
    {
        if (*prange_prev).xmonitor.range_status as libc::c_int
            == 9 as libc::c_int as VL53LX_DeviceError as libc::c_int
            || (*prange_prev).xmonitor.range_status as libc::c_int
                == 19 as libc::c_int as VL53LX_DeviceError as libc::c_int
            || (*prange_prev).xmonitor.range_status as libc::c_int
                == 20 as libc::c_int as VL53LX_DeviceError as libc::c_int
        {
            (*prange_curr).range_status = 9 as libc::c_int as VL53LX_DeviceError;
            status = VL53LX_hist_events_consistency_check(
                event_sigma,
                min_spad_count,
                phist_prev,
                &mut (*prange_prev).xmonitor,
                prange_curr,
                &mut events_tolerance,
                &mut events_delta,
                &mut (*prange_curr).range_status,
            );
        }
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_hist_wrap_dmax(
    mut phistpostprocess: *mut VL53LX_hist_post_process_config_t,
    mut pcurrent: *mut VL53LX_histogram_bin_data_t,
    mut pwrap_dmax_mm: *mut i16,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pll_period_mm: u32 = 0 as libc::c_int as u32;
    let mut wrap_dmax_phase: u32 = 0 as libc::c_int as u32;
    let mut range_mm: u32 = 0 as libc::c_int as u32;
    *pwrap_dmax_mm = 0 as libc::c_int as i16;
    if (*pcurrent).VL53LX_p_015 as libc::c_int != 0 as libc::c_int {
        pll_period_mm = VL53LX_calc_pll_period_mm((*pcurrent).VL53LX_p_015);
        wrap_dmax_phase = ((*phistpostprocess).valid_phase_high as u32)
            << 8 as libc::c_int;
        range_mm = wrap_dmax_phase.wrapping_mul(pll_period_mm);
        range_mm = range_mm
            .wrapping_add(((1 as libc::c_int) << 14 as libc::c_int) as libc::c_uint)
            >> 15 as libc::c_int;
        *pwrap_dmax_mm = range_mm as i16;
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_hist_combine_mm1_mm2_offsets(
    mut mm1_offset_mm: i16,
    mut mm2_offset_mm: i16,
    mut encoded_mm_roi_centre: u8,
    mut encoded_mm_roi_size: u8,
    mut encoded_zone_centre: u8,
    mut encoded_zone_size: u8,
    mut pcal_data: *mut VL53LX_additional_offset_cal_data_t,
    mut pgood_spads: *mut u8,
    mut aperture_attenuation: u16,
    mut prange_offset_mm: *mut i16,
) {
    let mut max_mm_inner_effective_spads: u16 = 0 as libc::c_int as u16;
    let mut max_mm_outer_effective_spads: u16 = 0 as libc::c_int as u16;
    let mut mm_inner_effective_spads: u16 = 0 as libc::c_int as u16;
    let mut mm_outer_effective_spads: u16 = 0 as libc::c_int as u16;
    let mut scaled_mm1_peak_rate_mcps: u32 = 0 as libc::c_int as u32;
    let mut scaled_mm2_peak_rate_mcps: u32 = 0 as libc::c_int as u32;
    let mut tmp0: i32 = 0 as libc::c_int;
    let mut tmp1: i32 = 0 as libc::c_int;
    VL53LX_calc_mm_effective_spads(
        encoded_mm_roi_centre,
        encoded_mm_roi_size,
        0xc7 as libc::c_int as u8,
        0xff as libc::c_int as u8,
        pgood_spads,
        aperture_attenuation,
        &mut max_mm_inner_effective_spads,
        &mut max_mm_outer_effective_spads,
    );
    if !(max_mm_inner_effective_spads as libc::c_int == 0 as libc::c_int
        || max_mm_outer_effective_spads as libc::c_int == 0 as libc::c_int)
    {
        VL53LX_calc_mm_effective_spads(
            encoded_mm_roi_centre,
            encoded_mm_roi_size,
            encoded_zone_centre,
            encoded_zone_size,
            pgood_spads,
            aperture_attenuation,
            &mut mm_inner_effective_spads,
            &mut mm_outer_effective_spads,
        );
        scaled_mm1_peak_rate_mcps = (*pcal_data)
            .result__mm_inner_peak_signal_count_rtn_mcps as u32;
        scaled_mm1_peak_rate_mcps = (scaled_mm1_peak_rate_mcps as libc::c_uint)
            .wrapping_mul(mm_inner_effective_spads as u32) as u32 as u32;
        scaled_mm1_peak_rate_mcps = (scaled_mm1_peak_rate_mcps as libc::c_uint)
            .wrapping_div(max_mm_inner_effective_spads as u32) as u32
            as u32;
        scaled_mm2_peak_rate_mcps = (*pcal_data)
            .result__mm_outer_peak_signal_count_rtn_mcps as u32;
        scaled_mm2_peak_rate_mcps = (scaled_mm2_peak_rate_mcps as libc::c_uint)
            .wrapping_mul(mm_outer_effective_spads as u32) as u32 as u32;
        scaled_mm2_peak_rate_mcps = (scaled_mm2_peak_rate_mcps as libc::c_uint)
            .wrapping_div(max_mm_outer_effective_spads as u32) as u32
            as u32;
        tmp0 = mm1_offset_mm as i32 * scaled_mm1_peak_rate_mcps as i32;
        tmp0 += mm2_offset_mm as i32 * scaled_mm2_peak_rate_mcps as i32;
        tmp1 = scaled_mm1_peak_rate_mcps as i32
            + scaled_mm2_peak_rate_mcps as i32;
        if tmp1 != 0 as libc::c_int {
            tmp0 = tmp0 * 4 as libc::c_int / tmp1;
        }
    }
    *prange_offset_mm = tmp0 as i16;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_hist_xtalk_extract_calc_window(
    mut target_distance_mm: i16,
    mut target_width_oversize: u16,
    mut phist_bins: *mut VL53LX_histogram_bin_data_t,
    mut pxtalk_data: *mut VL53LX_hist_xtalk_extract_data_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    (*pxtalk_data).pll_period_mm = VL53LX_calc_pll_period_mm((*phist_bins).VL53LX_p_015);
    if (*pxtalk_data).pll_period_mm == 0 as libc::c_int as libc::c_uint {
        (*pxtalk_data).pll_period_mm = 1 as libc::c_int as u32;
    }
    (*pxtalk_data)
        .xtalk_width_phase = (*phist_bins).vcsel_width as i32 * 128 as libc::c_int;
    (*pxtalk_data)
        .target_width_phase = (*pxtalk_data).xtalk_width_phase
        + target_width_oversize as i32 * 128 as libc::c_int;
    (*pxtalk_data)
        .xtalk_start_phase = (*phist_bins).zero_distance_phase as i32
        - (*pxtalk_data).xtalk_width_phase / 2 as libc::c_int;
    (*pxtalk_data)
        .xtalk_end_phase = (*pxtalk_data).xtalk_start_phase
        + (*pxtalk_data).xtalk_width_phase;
    if (*pxtalk_data).xtalk_start_phase < 0 as libc::c_int {
        (*pxtalk_data).xtalk_start_phase = 0 as libc::c_int;
    }
    (*pxtalk_data)
        .VL53LX_p_012 = ((*pxtalk_data).xtalk_start_phase / 2048 as libc::c_int)
        as u8;
    (*pxtalk_data)
        .VL53LX_p_013 = (((*pxtalk_data).xtalk_end_phase + 2047 as libc::c_int)
        / 2048 as libc::c_int) as u8;
    (*pxtalk_data)
        .target_start_phase = target_distance_mm as i32 * 2048 as libc::c_int
        * 16 as libc::c_int;
    let ref mut fresh49 = (*pxtalk_data).target_start_phase;
    *fresh49 += (*pxtalk_data).pll_period_mm as i32 / 2 as libc::c_int;
    let ref mut fresh50 = (*pxtalk_data).target_start_phase;
    *fresh50 /= (*pxtalk_data).pll_period_mm as i32;
    let ref mut fresh51 = (*pxtalk_data).target_start_phase;
    *fresh51 += (*phist_bins).zero_distance_phase as i32;
    let ref mut fresh52 = (*pxtalk_data).target_start_phase;
    *fresh52 -= (*pxtalk_data).target_width_phase / 2 as libc::c_int;
    (*pxtalk_data)
        .target_end_phase = (*pxtalk_data).target_start_phase
        + (*pxtalk_data).target_width_phase;
    if (*pxtalk_data).target_start_phase < 0 as libc::c_int {
        (*pxtalk_data).target_start_phase = 0 as libc::c_int;
    }
    (*pxtalk_data)
        .target_start = ((*pxtalk_data).target_start_phase / 2048 as libc::c_int)
        as u8;
    if (*pxtalk_data).VL53LX_p_013 as libc::c_int
        > (*pxtalk_data).target_start as libc::c_int - 1 as libc::c_int
    {
        (*pxtalk_data)
            .VL53LX_p_013 = ((*pxtalk_data).target_start as libc::c_int
            - 1 as libc::c_int) as u8;
    }
    (*pxtalk_data)
        .effective_width = 2048 as libc::c_int
        * ((*pxtalk_data).VL53LX_p_013 as i32 + 1 as libc::c_int);
    let ref mut fresh53 = (*pxtalk_data).effective_width;
    *fresh53 -= (*pxtalk_data).xtalk_start_phase;
    if (*pxtalk_data).effective_width > (*pxtalk_data).xtalk_width_phase {
        (*pxtalk_data).effective_width = (*pxtalk_data).xtalk_width_phase;
    }
    if (*pxtalk_data).effective_width < 1 as libc::c_int {
        (*pxtalk_data).effective_width = 1 as libc::c_int;
    }
    (*pxtalk_data).event_scaler = (*pxtalk_data).xtalk_width_phase * 1000 as libc::c_int;
    let ref mut fresh54 = (*pxtalk_data).event_scaler;
    *fresh54 += (*pxtalk_data).effective_width / 2 as libc::c_int;
    let ref mut fresh55 = (*pxtalk_data).event_scaler;
    *fresh55 /= (*pxtalk_data).effective_width;
    if (*pxtalk_data).event_scaler < 1000 as libc::c_int {
        (*pxtalk_data).event_scaler = 1000 as libc::c_int;
    }
    if (*pxtalk_data).event_scaler > 4000 as libc::c_int {
        (*pxtalk_data).event_scaler = 4000 as libc::c_int;
    }
    let ref mut fresh56 = (*pxtalk_data).event_scaler_sum;
    *fresh56 += (*pxtalk_data).event_scaler;
    let ref mut fresh57 = (*pxtalk_data).peak_duration_us_sum;
    *fresh57 = (*fresh57 as libc::c_uint).wrapping_add((*phist_bins).peak_duration_us)
        as u32 as u32;
    let ref mut fresh58 = (*pxtalk_data).effective_spad_count_sum;
    *fresh58 = (*fresh58 as libc::c_uint)
        .wrapping_add((*phist_bins).result__dss_actual_effective_spads as u32)
        as u32 as u32;
    let ref mut fresh59 = (*pxtalk_data).zero_distance_phase_sum;
    *fresh59 = (*fresh59 as libc::c_uint)
        .wrapping_add((*phist_bins).zero_distance_phase as u32) as u32
        as u32;
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_hist_xtalk_extract_calc_event_sums(
    mut phist_bins: *mut VL53LX_histogram_bin_data_t,
    mut pxtalk_data: *mut VL53LX_hist_xtalk_extract_data_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut lb: u8 = 0 as libc::c_int as u8;
    let mut i: u8 = 0 as libc::c_int as u8;
    lb = (*pxtalk_data).VL53LX_p_012;
    while lb as libc::c_int <= (*pxtalk_data).VL53LX_p_013 as libc::c_int {
        i = ((lb as libc::c_int + (*phist_bins).number_of_ambient_bins as libc::c_int
            + (*phist_bins).VL53LX_p_021 as libc::c_int)
            % (*phist_bins).VL53LX_p_021 as libc::c_int) as u8;
        let ref mut fresh60 = (*pxtalk_data).signal_events_sum;
        *fresh60 += (*phist_bins).bin_data[i as usize];
        let ref mut fresh61 = (*pxtalk_data).signal_events_sum;
        *fresh61 -= (*phist_bins).VL53LX_p_028;
        lb = lb.wrapping_add(1);
    }
    lb = 0 as libc::c_int as u8;
    while (lb as libc::c_int) < 12 as libc::c_int
        && (lb as libc::c_int) < (*phist_bins).VL53LX_p_021 as libc::c_int
    {
        i = ((lb as libc::c_int + (*phist_bins).number_of_ambient_bins as libc::c_int
            + (*phist_bins).VL53LX_p_021 as libc::c_int)
            % (*phist_bins).VL53LX_p_021 as libc::c_int) as u8;
        let ref mut fresh62 = (*pxtalk_data).bin_data_sums[lb as usize];
        *fresh62 += (*phist_bins).bin_data[i as usize];
        let ref mut fresh63 = (*pxtalk_data).bin_data_sums[lb as usize];
        *fresh63 -= (*phist_bins).VL53LX_p_028;
        lb = lb.wrapping_add(1);
    }
    let ref mut fresh64 = (*pxtalk_data).sample_count;
    *fresh64 = (*fresh64 as libc::c_uint).wrapping_add(1 as libc::c_int as libc::c_uint)
        as u32 as u32;
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_hist_xtalk_extract_calc_rate_per_spad(
    mut pxtalk_data: *mut VL53LX_hist_xtalk_extract_data_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut tmp64_0: u64 = 0 as libc::c_int as u64;
    let mut tmp64_1: u64 = 0 as libc::c_int as u64;
    let mut xtalk_per_spad: u64 = 0 as libc::c_int as u64;
    tmp64_1 = ((*pxtalk_data).effective_spad_count_sum as u64)
        .wrapping_mul((*pxtalk_data).peak_duration_us_sum as u64);
    if (*pxtalk_data).signal_events_sum < 0 as libc::c_int {
        (*pxtalk_data).signal_events_sum = 0 as libc::c_int;
        tmp64_0 = ((*pxtalk_data).sample_count as u64)
            .wrapping_mul((*pxtalk_data).event_scaler_avg as u64)
            .wrapping_mul(256 as libc::c_uint as libc::c_ulong) << 9 as libc::c_uint;
        if tmp64_0 > 0 as libc::c_int as libc::c_ulong {
            (*pxtalk_data)
                .signal_events_sum = (50 as libc::c_uint as libc::c_ulong)
                .wrapping_mul(tmp64_1)
                .wrapping_div(tmp64_0) as i32;
        }
    }
    tmp64_0 = ((*pxtalk_data).signal_events_sum as u64)
        .wrapping_mul((*pxtalk_data).sample_count as u64)
        .wrapping_mul((*pxtalk_data).event_scaler_avg as u64)
        .wrapping_mul(256 as libc::c_uint as libc::c_ulong) << 9 as libc::c_uint;
    if tmp64_1 > 0 as libc::c_uint as libc::c_ulong {
        tmp64_0 = tmp64_0.wrapping_add(tmp64_1 >> 1 as libc::c_uint);
        xtalk_per_spad = tmp64_0.wrapping_div(tmp64_1);
    } else {
        xtalk_per_spad = tmp64_0;
    }
    (*pxtalk_data).xtalk_rate_kcps_per_spad = xtalk_per_spad as u32;
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_hist_xtalk_extract_calc_shape(
    mut pxtalk_data: *mut VL53LX_hist_xtalk_extract_data_t,
    mut pxtalk_shape: *mut VL53LX_xtalk_histogram_shape_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut lb: i32 = 0 as libc::c_int;
    let mut total_events: u64 = 0 as libc::c_uint as u64;
    let mut tmp64_0: u64 = 0 as libc::c_uint as u64;
    let mut remaining_area: i32 = 1024 as libc::c_int;
    (*pxtalk_shape).VL53LX_p_019 = 0 as libc::c_int as u8;
    (*pxtalk_shape).VL53LX_p_020 = 12 as libc::c_int as u8;
    (*pxtalk_shape).VL53LX_p_021 = 12 as libc::c_int as u8;
    (*pxtalk_shape)
        .zero_distance_phase = (*pxtalk_data).zero_distance_phase_avg as u16;
    (*pxtalk_shape)
        .phasecal_result__reference_phase = ((*pxtalk_data).zero_distance_phase_avg
        as u16 as libc::c_int + 3 as libc::c_int * 2048 as libc::c_int) as u16;
    if (*pxtalk_data).signal_events_sum > 0 as libc::c_int {
        total_events = ((*pxtalk_data).signal_events_sum as u64)
            .wrapping_mul((*pxtalk_data).event_scaler_avg as u64);
    } else {
        total_events = 1 as libc::c_int as u64;
    }
    if total_events == 0 as libc::c_int as libc::c_ulong {
        total_events = 1 as libc::c_int as u64;
    }
    remaining_area = 1024 as libc::c_int;
    (*pxtalk_data).max_shape_value = 0 as libc::c_int;
    lb = 0 as libc::c_int;
    while lb < 12 as libc::c_int {
        if lb < (*pxtalk_data).VL53LX_p_012 as i32
            || lb > (*pxtalk_data).VL53LX_p_013 as i32
            || (*pxtalk_data).bin_data_sums[lb as usize] < 0 as libc::c_int
        {
            if remaining_area > 0 as libc::c_int && remaining_area < 1024 as libc::c_int
            {
                if remaining_area > (*pxtalk_data).max_shape_value {
                    (*pxtalk_shape)
                        .bin_data[lb
                        as usize] = (*pxtalk_data).max_shape_value as u32;
                    remaining_area -= (*pxtalk_data).max_shape_value;
                } else {
                    (*pxtalk_shape).bin_data[lb as usize] = remaining_area as u32;
                    remaining_area = 0 as libc::c_int;
                }
            } else {
                (*pxtalk_shape).bin_data[lb as usize] = 0 as libc::c_int as u32;
            }
        } else {
            tmp64_0 = ((*pxtalk_data).bin_data_sums[lb as usize] as u64)
                .wrapping_mul(1024 as libc::c_uint as libc::c_ulong)
                .wrapping_mul(1000 as libc::c_uint as libc::c_ulong);
            tmp64_0 = (tmp64_0 as libc::c_ulong)
                .wrapping_add(total_events >> 1 as libc::c_int) as u64 as u64;
            tmp64_0 = tmp64_0.wrapping_div(total_events);
            if tmp64_0 > 0xffff as libc::c_uint as libc::c_ulong {
                tmp64_0 = 0xffff as libc::c_uint as u64;
            }
            (*pxtalk_shape).bin_data[lb as usize] = tmp64_0 as u32;
            if (*pxtalk_shape).bin_data[lb as usize] as i32
                > (*pxtalk_data).max_shape_value
            {
                (*pxtalk_data)
                    .max_shape_value = (*pxtalk_shape).bin_data[lb as usize] as i32;
            }
            remaining_area -= (*pxtalk_shape).bin_data[lb as usize] as i32;
        }
        lb += 1;
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_hist_xtalk_shape_model(
    mut events_per_bin: u16,
    mut pulse_centre: u16,
    mut pulse_width: u16,
    mut pxtalk_shape: *mut VL53LX_xtalk_histogram_shape_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut phase_start: u32 = 0 as libc::c_int as u32;
    let mut phase_stop: u32 = 0 as libc::c_int as u32;
    let mut phase_bin: u32 = 0 as libc::c_int as u32;
    let mut bin_start: u32 = 0 as libc::c_int as u32;
    let mut bin_stop: u32 = 0 as libc::c_int as u32;
    let mut lb: u32 = 0 as libc::c_int as u32;
    let mut VL53LX_p_018: u16 = 0 as libc::c_int as u16;
    (*pxtalk_shape).VL53LX_p_019 = 0 as libc::c_int as u8;
    (*pxtalk_shape).VL53LX_p_020 = 12 as libc::c_int as u8;
    (*pxtalk_shape).VL53LX_p_021 = 12 as libc::c_int as u8;
    (*pxtalk_shape).zero_distance_phase = pulse_centre;
    (*pxtalk_shape)
        .phasecal_result__reference_phase = (pulse_centre as libc::c_int
        + 3 as libc::c_int * 2048 as libc::c_int) as u16;
    if pulse_centre as libc::c_int > pulse_width as libc::c_int >> 1 as libc::c_int {
        phase_start = (pulse_centre as u32)
            .wrapping_sub(pulse_width as u32 >> 1 as libc::c_int);
    } else {
        phase_start = 0 as libc::c_int as u32;
    }
    phase_stop = (pulse_centre as u32)
        .wrapping_add(pulse_width as u32 >> 1 as libc::c_int);
    bin_start = phase_start.wrapping_div(2048 as libc::c_int as libc::c_uint);
    bin_stop = phase_stop.wrapping_div(2048 as libc::c_int as libc::c_uint);
    lb = 0 as libc::c_int as u32;
    while lb < 12 as libc::c_int as libc::c_uint {
        VL53LX_p_018 = 0 as libc::c_int as u16;
        if lb == bin_start && lb == bin_stop {
            VL53LX_p_018 = VL53LX_hist_xtalk_shape_model_interp(
                events_per_bin,
                phase_stop.wrapping_sub(phase_start),
            );
        } else if lb > bin_start && lb < bin_stop {
            VL53LX_p_018 = events_per_bin;
        } else if lb == bin_start {
            phase_bin = lb
                .wrapping_add(1 as libc::c_int as libc::c_uint)
                .wrapping_mul(2048 as libc::c_int as libc::c_uint);
            VL53LX_p_018 = VL53LX_hist_xtalk_shape_model_interp(
                events_per_bin,
                phase_bin.wrapping_sub(phase_start),
            );
        } else if lb == bin_stop {
            phase_bin = lb.wrapping_mul(2048 as libc::c_int as libc::c_uint);
            VL53LX_p_018 = VL53LX_hist_xtalk_shape_model_interp(
                events_per_bin,
                phase_stop.wrapping_sub(phase_bin),
            );
        }
        (*pxtalk_shape).bin_data[lb as usize] = VL53LX_p_018 as u32;
        lb = lb.wrapping_add(1);
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_hist_xtalk_shape_model_interp(
    mut events_per_bin: u16,
    mut phase_delta: u32,
) -> u16 {
    let mut VL53LX_p_018: u32 = 0 as libc::c_int as u32;
    VL53LX_p_018 = (events_per_bin as u32).wrapping_mul(phase_delta);
    VL53LX_p_018 = (VL53LX_p_018 as libc::c_uint)
        .wrapping_add(1024 as libc::c_int as libc::c_uint) as u32 as u32;
    VL53LX_p_018 = (VL53LX_p_018 as libc::c_uint)
        .wrapping_div(2048 as libc::c_int as libc::c_uint) as u32 as u32;
    if VL53LX_p_018 > 0xffff as libc::c_uint {
        VL53LX_p_018 = 0xffff as libc::c_uint;
    }
    return VL53LX_p_018 as u16;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_spad_number_to_byte_bit_index(
    mut spad_number: u8,
    mut pbyte_index: *mut u8,
    mut pbit_index: *mut u8,
    mut pbit_mask: *mut u8,
) {
    *pbyte_index = (spad_number as libc::c_int >> 3 as libc::c_int) as u8;
    *pbit_index = (spad_number as libc::c_int & 0x7 as libc::c_int) as u8;
    *pbit_mask = ((0x1 as libc::c_int) << *pbit_index as libc::c_int) as u8;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_encode_row_col(
    mut row: u8,
    mut col: u8,
    mut pspad_number: *mut u8,
) {
    if row as libc::c_int > 7 as libc::c_int {
        *pspad_number = (128 as libc::c_int + ((col as libc::c_int) << 3 as libc::c_int)
            + (15 as libc::c_int - row as libc::c_int)) as u8;
    } else {
        *pspad_number = (((15 as libc::c_int - col as libc::c_int) << 3 as libc::c_int)
            + row as libc::c_int) as u8;
    };
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_decode_zone_size(
    mut encoded_xy_size: u8,
    mut pwidth: *mut u8,
    mut pheight: *mut u8,
) {
    *pheight = (encoded_xy_size as libc::c_int >> 4 as libc::c_int) as u8;
    *pwidth = (encoded_xy_size as libc::c_int & 0xf as libc::c_int) as u8;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_encode_zone_size(
    mut width: u8,
    mut height: u8,
    mut pencoded_xy_size: *mut u8,
) {
    *pencoded_xy_size = (((height as libc::c_int) << 4 as libc::c_int)
        + width as libc::c_int) as u8;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_decode_zone_limits(
    mut encoded_xy_centre: u8,
    mut encoded_xy_size: u8,
    mut px_ll: *mut i16,
    mut py_ll: *mut i16,
    mut px_ur: *mut i16,
    mut py_ur: *mut i16,
) {
    let mut x_centre: u8 = 0 as libc::c_int as u8;
    let mut y_centre: u8 = 0 as libc::c_int as u8;
    let mut width: u8 = 0 as libc::c_int as u8;
    let mut height: u8 = 0 as libc::c_int as u8;
    VL53LX_decode_row_col(encoded_xy_centre, &mut y_centre, &mut x_centre);
    VL53LX_decode_zone_size(encoded_xy_size, &mut width, &mut height);
    *px_ll = (x_centre as i16 as libc::c_int
        - (width as i16 as libc::c_int + 1 as libc::c_int) / 2 as libc::c_int)
        as i16;
    if (*px_ll as libc::c_int) < 0 as libc::c_int {
        *px_ll = 0 as libc::c_int as i16;
    }
    *px_ur = (*px_ll as libc::c_int + width as i16 as libc::c_int) as i16;
    if *px_ur as libc::c_int > 16 as libc::c_int - 1 as libc::c_int {
        *px_ur = (16 as libc::c_int - 1 as libc::c_int) as i16;
    }
    *py_ll = (y_centre as i16 as libc::c_int
        - (height as i16 as libc::c_int + 1 as libc::c_int) / 2 as libc::c_int)
        as i16;
    if (*py_ll as libc::c_int) < 0 as libc::c_int {
        *py_ll = 0 as libc::c_int as i16;
    }
    *py_ur = (*py_ll as libc::c_int + height as i16 as libc::c_int) as i16;
    if *py_ur as libc::c_int > 16 as libc::c_int - 1 as libc::c_int {
        *py_ur = (16 as libc::c_int - 1 as libc::c_int) as i16;
    }
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_is_aperture_location(
    mut row: u8,
    mut col: u8,
) -> u8 {
    let mut is_aperture: u8 = 0 as libc::c_int as u8;
    let mut mod_row: u8 = (row as libc::c_int % 4 as libc::c_int) as u8;
    let mut mod_col: u8 = (col as libc::c_int % 4 as libc::c_int) as u8;
    if mod_row as libc::c_int == 0 as libc::c_int
        && mod_col as libc::c_int == 2 as libc::c_int
    {
        is_aperture = 1 as libc::c_int as u8;
    }
    if mod_row as libc::c_int == 2 as libc::c_int
        && mod_col as libc::c_int == 0 as libc::c_int
    {
        is_aperture = 1 as libc::c_int as u8;
    }
    return is_aperture;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_calc_max_effective_spads(
    mut encoded_zone_centre: u8,
    mut encoded_zone_size: u8,
    mut pgood_spads: *mut u8,
    mut aperture_attenuation: u16,
    mut pmax_effective_spads: *mut u16,
) {
    let mut x: i16 = 0 as libc::c_int as i16;
    let mut y: i16 = 0 as libc::c_int as i16;
    let mut zone_x_ll: i16 = 0 as libc::c_int as i16;
    let mut zone_y_ll: i16 = 0 as libc::c_int as i16;
    let mut zone_x_ur: i16 = 0 as libc::c_int as i16;
    let mut zone_y_ur: i16 = 0 as libc::c_int as i16;
    let mut spad_number: u8 = 0 as libc::c_int as u8;
    let mut byte_index: u8 = 0 as libc::c_int as u8;
    let mut bit_index: u8 = 0 as libc::c_int as u8;
    let mut bit_mask: u8 = 0 as libc::c_int as u8;
    let mut is_aperture: u8 = 0 as libc::c_int as u8;
    VL53LX_decode_zone_limits(
        encoded_zone_centre,
        encoded_zone_size,
        &mut zone_x_ll,
        &mut zone_y_ll,
        &mut zone_x_ur,
        &mut zone_y_ur,
    );
    *pmax_effective_spads = 0 as libc::c_int as u16;
    y = zone_y_ll;
    while y as libc::c_int <= zone_y_ur as libc::c_int {
        x = zone_x_ll;
        while x as libc::c_int <= zone_x_ur as libc::c_int {
            VL53LX_encode_row_col(y as u8, x as u8, &mut spad_number);
            VL53LX_spad_number_to_byte_bit_index(
                spad_number,
                &mut byte_index,
                &mut bit_index,
                &mut bit_mask,
            );
            if *pgood_spads.offset(byte_index as isize) as libc::c_int
                & bit_mask as libc::c_int > 0 as libc::c_int
            {
                is_aperture = VL53LX_is_aperture_location(y as u8, x as u8);
                if is_aperture as libc::c_int > 0 as libc::c_int {
                    *pmax_effective_spads = (*pmax_effective_spads as libc::c_int
                        + aperture_attenuation as libc::c_int) as u16;
                } else {
                    *pmax_effective_spads = (*pmax_effective_spads as libc::c_int
                        + 0x100 as libc::c_int) as u16;
                }
            }
            x += 1;
        }
        y += 1;
    }
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_calc_mm_effective_spads(
    mut encoded_mm_roi_centre: u8,
    mut encoded_mm_roi_size: u8,
    mut encoded_zone_centre: u8,
    mut encoded_zone_size: u8,
    mut pgood_spads: *mut u8,
    mut aperture_attenuation: u16,
    mut pmm_inner_effective_spads: *mut u16,
    mut pmm_outer_effective_spads: *mut u16,
) {
    let mut x: i16 = 0 as libc::c_int as i16;
    let mut y: i16 = 0 as libc::c_int as i16;
    let mut mm_x_ll: i16 = 0 as libc::c_int as i16;
    let mut mm_y_ll: i16 = 0 as libc::c_int as i16;
    let mut mm_x_ur: i16 = 0 as libc::c_int as i16;
    let mut mm_y_ur: i16 = 0 as libc::c_int as i16;
    let mut zone_x_ll: i16 = 0 as libc::c_int as i16;
    let mut zone_y_ll: i16 = 0 as libc::c_int as i16;
    let mut zone_x_ur: i16 = 0 as libc::c_int as i16;
    let mut zone_y_ur: i16 = 0 as libc::c_int as i16;
    let mut spad_number: u8 = 0 as libc::c_int as u8;
    let mut byte_index: u8 = 0 as libc::c_int as u8;
    let mut bit_index: u8 = 0 as libc::c_int as u8;
    let mut bit_mask: u8 = 0 as libc::c_int as u8;
    let mut is_aperture: u8 = 0 as libc::c_int as u8;
    let mut spad_attenuation: u16 = 0 as libc::c_int as u16;
    VL53LX_decode_zone_limits(
        encoded_mm_roi_centre,
        encoded_mm_roi_size,
        &mut mm_x_ll,
        &mut mm_y_ll,
        &mut mm_x_ur,
        &mut mm_y_ur,
    );
    VL53LX_decode_zone_limits(
        encoded_zone_centre,
        encoded_zone_size,
        &mut zone_x_ll,
        &mut zone_y_ll,
        &mut zone_x_ur,
        &mut zone_y_ur,
    );
    *pmm_inner_effective_spads = 0 as libc::c_int as u16;
    *pmm_outer_effective_spads = 0 as libc::c_int as u16;
    y = zone_y_ll;
    while y as libc::c_int <= zone_y_ur as libc::c_int {
        x = zone_x_ll;
        while x as libc::c_int <= zone_x_ur as libc::c_int {
            VL53LX_encode_row_col(y as u8, x as u8, &mut spad_number);
            VL53LX_spad_number_to_byte_bit_index(
                spad_number,
                &mut byte_index,
                &mut bit_index,
                &mut bit_mask,
            );
            if *pgood_spads.offset(byte_index as isize) as libc::c_int
                & bit_mask as libc::c_int > 0 as libc::c_int
            {
                is_aperture = VL53LX_is_aperture_location(y as u8, x as u8);
                if is_aperture as libc::c_int > 0 as libc::c_int {
                    spad_attenuation = aperture_attenuation;
                } else {
                    spad_attenuation = 0x100 as libc::c_int as u16;
                }
                if x as libc::c_int >= mm_x_ll as libc::c_int
                    && x as libc::c_int <= mm_x_ur as libc::c_int
                    && y as libc::c_int >= mm_y_ll as libc::c_int
                    && y as libc::c_int <= mm_y_ur as libc::c_int
                {
                    *pmm_inner_effective_spads = (*pmm_inner_effective_spads
                        as libc::c_int + spad_attenuation as libc::c_int) as u16;
                } else {
                    *pmm_outer_effective_spads = (*pmm_outer_effective_spads
                        as libc::c_int + spad_attenuation as libc::c_int) as u16;
                }
            }
            x += 1;
        }
        y += 1;
    }
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_hist_copy_results_to_sys_and_core(
    mut pbins: *mut VL53LX_histogram_bin_data_t,
    mut phist: *mut VL53LX_range_results_t,
    mut psys: *mut VL53LX_system_results_t,
    mut pcore: *mut VL53LX_core_results_t,
) {
    let mut i: u8 = 0 as libc::c_int as u8;
    let mut pdata: *mut VL53LX_range_data_t = 0 as *mut VL53LX_range_data_t;
    VL53LX_init_system_results(psys);
    (*psys).result__interrupt_status = (*pbins).result__interrupt_status;
    (*psys).result__range_status = (*phist).active_results;
    (*psys).result__report_status = (*pbins).result__report_status;
    (*psys).result__stream_count = (*pbins).result__stream_count;
    pdata = &mut *((*phist).VL53LX_p_003).as_mut_ptr().offset(0 as libc::c_int as isize)
        as *mut VL53LX_range_data_t;
    i = 0 as libc::c_int as u8;
    while (i as libc::c_int) < (*phist).active_results as libc::c_int {
        match i as libc::c_int {
            0 => {
                (*psys).result__dss_actual_effective_spads_sd0 = (*pdata).VL53LX_p_004;
                (*psys)
                    .result__peak_signal_count_rate_mcps_sd0 = (*pdata)
                    .peak_signal_count_rate_mcps;
                (*psys)
                    .result__avg_signal_count_rate_mcps_sd0 = (*pdata)
                    .avg_signal_count_rate_mcps;
                (*psys)
                    .result__ambient_count_rate_mcps_sd0 = (*pdata)
                    .ambient_count_rate_mcps;
                (*psys).result__sigma_sd0 = (*pdata).VL53LX_p_002;
                (*psys).result__phase_sd0 = (*pdata).VL53LX_p_011;
                (*psys)
                    .result__final_crosstalk_corrected_range_mm_sd0 = (*pdata)
                    .median_range_mm as u16;
                (*psys).result__phase_sd1 = (*pdata).zero_distance_phase;
                (*pcore).result_core__ranging_total_events_sd0 = (*pdata).VL53LX_p_017;
                (*pcore).result_core__signal_total_events_sd0 = (*pdata).VL53LX_p_010;
                (*pcore)
                    .result_core__total_periods_elapsed_sd0 = (*pdata)
                    .total_periods_elapsed;
                (*pcore).result_core__ambient_window_events_sd0 = (*pdata).VL53LX_p_016;
            }
            1 => {
                (*psys).result__dss_actual_effective_spads_sd1 = (*pdata).VL53LX_p_004;
                (*psys)
                    .result__peak_signal_count_rate_mcps_sd1 = (*pdata)
                    .peak_signal_count_rate_mcps;
                (*psys)
                    .result__ambient_count_rate_mcps_sd1 = (*pdata)
                    .ambient_count_rate_mcps;
                (*psys).result__sigma_sd1 = (*pdata).VL53LX_p_002;
                (*psys).result__phase_sd1 = (*pdata).VL53LX_p_011;
                (*psys)
                    .result__final_crosstalk_corrected_range_mm_sd1 = (*pdata)
                    .median_range_mm as u16;
                (*pcore).result_core__ranging_total_events_sd1 = (*pdata).VL53LX_p_017;
                (*pcore).result_core__signal_total_events_sd1 = (*pdata).VL53LX_p_010;
                (*pcore)
                    .result_core__total_periods_elapsed_sd1 = (*pdata)
                    .total_periods_elapsed;
                (*pcore).result_core__ambient_window_events_sd1 = (*pdata).VL53LX_p_016;
            }
            _ => {}
        }
        pdata = pdata.offset(1);
        i = i.wrapping_add(1);
    }
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_sum_histogram_data(
    mut phist_input: *mut VL53LX_histogram_bin_data_t,
    mut phist_output: *mut VL53LX_histogram_bin_data_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut i: u8 = 0 as libc::c_int as u8;
    let mut smallest_bin_num: u8 = 0 as libc::c_int as u8;
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        if (*phist_output).VL53LX_p_021 as libc::c_int
            >= (*phist_input).VL53LX_p_021 as libc::c_int
        {
            smallest_bin_num = (*phist_input).VL53LX_p_021;
        } else {
            smallest_bin_num = (*phist_output).VL53LX_p_021;
        }
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        i = 0 as libc::c_int as u8;
        while (i as libc::c_int) < smallest_bin_num as libc::c_int {
            let ref mut fresh65 = (*phist_output).bin_data[i as usize];
            *fresh65 += (*phist_input).bin_data[i as usize];
            i = i.wrapping_add(1);
        }
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        let ref mut fresh66 = (*phist_output).VL53LX_p_028;
        *fresh66 += (*phist_input).VL53LX_p_028;
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_avg_histogram_data(
    mut no_of_samples: u8,
    mut phist_sum: *mut VL53LX_histogram_bin_data_t,
    mut phist_avg: *mut VL53LX_histogram_bin_data_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut i: u8 = 0 as libc::c_int as u8;
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        i = 0 as libc::c_int as u8;
        while (i as libc::c_int) < (*phist_sum).VL53LX_p_021 as libc::c_int {
            if no_of_samples as libc::c_int > 0 as libc::c_int {
                (*phist_avg)
                    .bin_data[i
                    as usize] = (*phist_sum).bin_data[i as usize]
                    / no_of_samples as i32;
            } else {
                (*phist_avg).bin_data[i as usize] = (*phist_sum).bin_data[i as usize];
            }
            i = i.wrapping_add(1);
        }
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        if no_of_samples as libc::c_int > 0 as libc::c_int {
            (*phist_avg)
                .VL53LX_p_028 = (*phist_sum).VL53LX_p_028 / no_of_samples as i32;
        } else {
            (*phist_avg).VL53LX_p_028 = (*phist_sum).VL53LX_p_028;
        }
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_save_cfg_data(mut Dev: VL53LX_DEV) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    let mut pres: *mut VL53LX_LLDriverResults_t = &mut (*Dev).Data.llresults;
    let mut pzone_dyn_cfg: *mut VL53LX_zone_private_dyn_cfg_t = 0
        as *mut VL53LX_zone_private_dyn_cfg_t;
    let mut pdynamic: *mut VL53LX_dynamic_config_t = &mut (*pdev).dyn_cfg;
    pzone_dyn_cfg = &mut *((*pres).zone_dyn_cfgs.VL53LX_p_003)
        .as_mut_ptr()
        .offset((*pdev).ll_state.cfg_zone_id as isize)
        as *mut VL53LX_zone_private_dyn_cfg_t;
    (*pzone_dyn_cfg).expected_stream_count = (*pdev).ll_state.cfg_stream_count;
    (*pzone_dyn_cfg).expected_gph_id = (*pdev).ll_state.cfg_gph_id;
    (*pzone_dyn_cfg)
        .roi_config__user_roi_centre_spad = (*pdynamic).roi_config__user_roi_centre_spad;
    (*pzone_dyn_cfg)
        .roi_config__user_roi_requested_global_xy_size = (*pdynamic)
        .roi_config__user_roi_requested_global_xy_size;
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_dynamic_zone_update(
    mut Dev: VL53LX_DEV,
    mut presults: *mut VL53LX_range_results_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    let mut pres: *mut VL53LX_LLDriverResults_t = &mut (*Dev).Data.llresults;
    let mut pZ: *mut VL53LX_zone_private_dyn_cfgs_t = &mut (*pres).zone_dyn_cfgs;
    let mut zone_id: u8 = (*pdev).ll_state.rd_zone_id;
    let mut i: u8 = 0;
    let mut max_total_rate_per_spads: u16 = 0;
    let mut target_rate: u16 = (*pdev).stat_cfg.dss_config__target_total_rate_mcps;
    let mut temp: u32 = 0xffff as libc::c_int as u32;
    (*pZ)
        .VL53LX_p_003[zone_id as usize]
        .dss_requested_effective_spad_count = 0 as libc::c_int as u16;
    max_total_rate_per_spads = (*presults)
        .VL53LX_p_003[0 as libc::c_int as usize]
        .total_rate_per_spad_mcps;
    i = 1 as libc::c_int as u8;
    while (i as libc::c_int) < (*presults).active_results as libc::c_int {
        if (*presults).VL53LX_p_003[i as usize].total_rate_per_spad_mcps as libc::c_int
            > max_total_rate_per_spads as libc::c_int
        {
            max_total_rate_per_spads = (*presults)
                .VL53LX_p_003[i as usize]
                .total_rate_per_spad_mcps;
        }
        i = i.wrapping_add(1);
    }
    if max_total_rate_per_spads as libc::c_int == 0 as libc::c_int {
        temp = 0xffff as libc::c_int as u32;
    } else {
        temp = ((target_rate as libc::c_int) << 14 as libc::c_int) as u32;
        temp = temp.wrapping_div(max_total_rate_per_spads as libc::c_uint);
        if temp > 0xffff as libc::c_int as libc::c_uint {
            temp = 0xffff as libc::c_int as u32;
        }
    }
    (*pZ)
        .VL53LX_p_003[zone_id as usize]
        .dss_requested_effective_spad_count = temp as u16;
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_multizone_hist_bins_update(
    mut Dev: VL53LX_DEV,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    let mut pstate: *mut VL53LX_ll_driver_state_t = &mut (*pdev).ll_state;
    let mut pzone_cfg: *mut VL53LX_zone_config_t = &mut (*pdev).zone_cfg;
    let mut phist_cfg: *mut VL53LX_histogram_config_t = &mut (*pdev).hist_cfg;
    let mut pmulti_hist: *mut VL53LX_histogram_config_t = &mut (*pzone_cfg)
        .multizone_hist_cfg;
    let mut next_range_is_odd_timing: u8 = ((*pstate).cfg_stream_count
        as libc::c_int % 2 as libc::c_int) as u8;
    if (*pzone_cfg).bin_config[(*pdev).ll_state.cfg_zone_id as usize] as libc::c_int
        == 1 as libc::c_int as VL53LX_ZoneConfig_BinConfig_select as libc::c_int
    {
        if next_range_is_odd_timing == 0 {
            (*phist_cfg)
                .histogram_config__low_amb_even_bin_0_1 = (*pmulti_hist)
                .histogram_config__low_amb_even_bin_0_1;
            (*phist_cfg)
                .histogram_config__low_amb_even_bin_2_3 = (*pmulti_hist)
                .histogram_config__low_amb_even_bin_2_3;
            (*phist_cfg)
                .histogram_config__low_amb_even_bin_4_5 = (*pmulti_hist)
                .histogram_config__low_amb_even_bin_4_5;
        }
        if next_range_is_odd_timing != 0 {
            (*phist_cfg)
                .histogram_config__low_amb_odd_bin_0_1 = (*pmulti_hist)
                .histogram_config__low_amb_even_bin_0_1;
            (*phist_cfg)
                .histogram_config__low_amb_odd_bin_2_3 = (*pmulti_hist)
                .histogram_config__low_amb_even_bin_2_3;
            (*phist_cfg)
                .histogram_config__low_amb_odd_bin_4_5 = (*pmulti_hist)
                .histogram_config__low_amb_even_bin_4_5;
        }
    } else if (*pzone_cfg).bin_config[(*pdev).ll_state.cfg_zone_id as usize]
            as libc::c_int
            == 2 as libc::c_int as VL53LX_ZoneConfig_BinConfig_select as libc::c_int
        {
        if next_range_is_odd_timing == 0 {
            (*phist_cfg)
                .histogram_config__low_amb_even_bin_0_1 = (*pmulti_hist)
                .histogram_config__mid_amb_even_bin_0_1;
            (*phist_cfg)
                .histogram_config__low_amb_even_bin_2_3 = (*pmulti_hist)
                .histogram_config__mid_amb_even_bin_2_3;
            (*phist_cfg)
                .histogram_config__low_amb_even_bin_4_5 = (*pmulti_hist)
                .histogram_config__mid_amb_even_bin_4_5;
        }
        if next_range_is_odd_timing != 0 {
            (*phist_cfg)
                .histogram_config__low_amb_odd_bin_0_1 = (*pmulti_hist)
                .histogram_config__mid_amb_even_bin_0_1;
            (*phist_cfg)
                .histogram_config__low_amb_odd_bin_2_3 = (*pmulti_hist)
                .histogram_config__mid_amb_even_bin_2_3;
            (*phist_cfg)
                .histogram_config__low_amb_odd_bin_4_5 = (*pmulti_hist)
                .histogram_config__mid_amb_even_bin_4_5;
        }
    } else if (*pzone_cfg).bin_config[(*pdev).ll_state.cfg_zone_id as usize]
            as libc::c_int
            == 3 as libc::c_int as VL53LX_ZoneConfig_BinConfig_select as libc::c_int
        {
        if next_range_is_odd_timing == 0 {
            (*phist_cfg)
                .histogram_config__low_amb_even_bin_0_1 = (*pmulti_hist)
                .histogram_config__high_amb_even_bin_0_1;
            (*phist_cfg)
                .histogram_config__low_amb_even_bin_2_3 = (*pmulti_hist)
                .histogram_config__high_amb_even_bin_2_3;
            (*phist_cfg)
                .histogram_config__low_amb_even_bin_4_5 = (*pmulti_hist)
                .histogram_config__high_amb_even_bin_4_5;
        }
        if next_range_is_odd_timing != 0 {
            (*phist_cfg)
                .histogram_config__low_amb_odd_bin_0_1 = (*pmulti_hist)
                .histogram_config__high_amb_even_bin_0_1;
            (*phist_cfg)
                .histogram_config__low_amb_odd_bin_2_3 = (*pmulti_hist)
                .histogram_config__high_amb_even_bin_2_3;
            (*phist_cfg)
                .histogram_config__low_amb_odd_bin_4_5 = (*pmulti_hist)
                .histogram_config__high_amb_even_bin_4_5;
        }
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        VL53LX_copy_hist_bins_to_static_cfg(
            phist_cfg,
            &mut (*pdev).stat_cfg,
            &mut (*pdev).tim_cfg,
        );
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_update_internal_stream_counters(
    mut Dev: VL53LX_DEV,
    mut external_stream_count: u8,
    mut pinternal_stream_count: *mut u8,
    mut pinternal_stream_count_val: *mut u8,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut stream_divider: u8 = 0;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    stream_divider = (*pdev).gen_cfg.global_config__stream_divider;
    if stream_divider as libc::c_int == 0 as libc::c_int {
        *pinternal_stream_count = external_stream_count;
    } else if *pinternal_stream_count_val as libc::c_int
            == stream_divider as libc::c_int - 1 as libc::c_int
        {
        if *pinternal_stream_count as libc::c_int == 0xff as libc::c_int {
            *pinternal_stream_count = 0x80 as libc::c_int as u8;
        } else {
            *pinternal_stream_count = (*pinternal_stream_count as libc::c_int
                + 1 as libc::c_int) as u8;
        }
        *pinternal_stream_count_val = 0 as libc::c_int as u8;
    } else {
        *pinternal_stream_count_val = (*pinternal_stream_count_val as libc::c_int
            + 1 as libc::c_int) as u8;
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_set_histogram_multizone_initial_bin_config(
    mut pzone_cfg: *mut VL53LX_zone_config_t,
    mut phist_cfg: *mut VL53LX_histogram_config_t,
    mut pmulti_hist: *mut VL53LX_histogram_config_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    if (*pzone_cfg).bin_config[0 as libc::c_int as usize] as libc::c_int
        == 1 as libc::c_int as VL53LX_ZoneConfig_BinConfig_select as libc::c_int
    {
        (*phist_cfg)
            .histogram_config__low_amb_even_bin_0_1 = (*pmulti_hist)
            .histogram_config__low_amb_even_bin_0_1;
        (*phist_cfg)
            .histogram_config__low_amb_even_bin_2_3 = (*pmulti_hist)
            .histogram_config__low_amb_even_bin_2_3;
        (*phist_cfg)
            .histogram_config__low_amb_even_bin_4_5 = (*pmulti_hist)
            .histogram_config__low_amb_even_bin_4_5;
        (*phist_cfg)
            .histogram_config__low_amb_odd_bin_0_1 = (*pmulti_hist)
            .histogram_config__low_amb_even_bin_0_1;
        (*phist_cfg)
            .histogram_config__low_amb_odd_bin_2_3 = (*pmulti_hist)
            .histogram_config__low_amb_even_bin_2_3;
        (*phist_cfg)
            .histogram_config__low_amb_odd_bin_4_5 = (*pmulti_hist)
            .histogram_config__low_amb_even_bin_4_5;
    } else if (*pzone_cfg).bin_config[0 as libc::c_int as usize] as libc::c_int
            == 2 as libc::c_int as VL53LX_ZoneConfig_BinConfig_select as libc::c_int
        {
        (*phist_cfg)
            .histogram_config__low_amb_even_bin_0_1 = (*pmulti_hist)
            .histogram_config__mid_amb_even_bin_0_1;
        (*phist_cfg)
            .histogram_config__low_amb_even_bin_2_3 = (*pmulti_hist)
            .histogram_config__mid_amb_even_bin_2_3;
        (*phist_cfg)
            .histogram_config__low_amb_even_bin_4_5 = (*pmulti_hist)
            .histogram_config__mid_amb_even_bin_4_5;
        (*phist_cfg)
            .histogram_config__low_amb_odd_bin_0_1 = (*pmulti_hist)
            .histogram_config__mid_amb_even_bin_0_1;
        (*phist_cfg)
            .histogram_config__low_amb_odd_bin_2_3 = (*pmulti_hist)
            .histogram_config__mid_amb_even_bin_2_3;
        (*phist_cfg)
            .histogram_config__low_amb_odd_bin_4_5 = (*pmulti_hist)
            .histogram_config__mid_amb_even_bin_4_5;
    } else if (*pzone_cfg).bin_config[0 as libc::c_int as usize] as libc::c_int
            == 3 as libc::c_int as VL53LX_ZoneConfig_BinConfig_select as libc::c_int
        {
        (*phist_cfg)
            .histogram_config__low_amb_even_bin_0_1 = (*pmulti_hist)
            .histogram_config__high_amb_even_bin_0_1;
        (*phist_cfg)
            .histogram_config__low_amb_even_bin_2_3 = (*pmulti_hist)
            .histogram_config__high_amb_even_bin_2_3;
        (*phist_cfg)
            .histogram_config__low_amb_even_bin_4_5 = (*pmulti_hist)
            .histogram_config__high_amb_even_bin_4_5;
        (*phist_cfg)
            .histogram_config__low_amb_odd_bin_0_1 = (*pmulti_hist)
            .histogram_config__high_amb_even_bin_0_1;
        (*phist_cfg)
            .histogram_config__low_amb_odd_bin_2_3 = (*pmulti_hist)
            .histogram_config__high_amb_even_bin_2_3;
        (*phist_cfg)
            .histogram_config__low_amb_odd_bin_4_5 = (*pmulti_hist)
            .histogram_config__high_amb_even_bin_4_5;
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_encode_GPIO_interrupt_config(
    mut pintconf: *mut VL53LX_GPIO_interrupt_config_t,
) -> u8 {
    let mut system__interrupt_config: u8 = 0;
    system__interrupt_config = (*pintconf).intr_mode_distance;
    system__interrupt_config = (system__interrupt_config as libc::c_int
        | ((*pintconf).intr_mode_rate as libc::c_int) << 2 as libc::c_int) as u8;
    system__interrupt_config = (system__interrupt_config as libc::c_int
        | ((*pintconf).intr_new_measure_ready as libc::c_int) << 5 as libc::c_int)
        as u8;
    system__interrupt_config = (system__interrupt_config as libc::c_int
        | ((*pintconf).intr_no_target as libc::c_int) << 6 as libc::c_int) as u8;
    system__interrupt_config = (system__interrupt_config as libc::c_int
        | ((*pintconf).intr_combined_mode as libc::c_int) << 7 as libc::c_int)
        as u8;
    return system__interrupt_config;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_decode_GPIO_interrupt_config(
    mut system__interrupt_config: u8,
) -> VL53LX_GPIO_interrupt_config_t {
    let mut intconf: VL53LX_GPIO_interrupt_config_t = VL53LX_GPIO_interrupt_config_t {
        intr_mode_distance: 0,
        intr_mode_rate: 0,
        intr_new_measure_ready: 0,
        intr_no_target: 0,
        intr_combined_mode: 0,
        threshold_distance_high: 0,
        threshold_distance_low: 0,
        threshold_rate_high: 0,
        threshold_rate_low: 0,
    };
    intconf
        .intr_mode_distance = (system__interrupt_config as libc::c_int
        & 0x3 as libc::c_int) as VL53LX_GPIO_Interrupt_Mode;
    intconf
        .intr_mode_rate = (system__interrupt_config as libc::c_int >> 2 as libc::c_int
        & 0x3 as libc::c_int) as VL53LX_GPIO_Interrupt_Mode;
    intconf
        .intr_new_measure_ready = (system__interrupt_config as libc::c_int
        >> 5 as libc::c_int & 0x1 as libc::c_int) as u8;
    intconf
        .intr_no_target = (system__interrupt_config as libc::c_int >> 6 as libc::c_int
        & 0x1 as libc::c_int) as u8;
    intconf
        .intr_combined_mode = (system__interrupt_config as libc::c_int
        >> 7 as libc::c_int & 0x1 as libc::c_int) as u8;
    intconf.threshold_rate_low = 0 as libc::c_int as u16;
    intconf.threshold_rate_high = 0 as libc::c_int as u16;
    intconf.threshold_distance_low = 0 as libc::c_int as u16;
    intconf.threshold_distance_high = 0 as libc::c_int as u16;
    return intconf;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_set_GPIO_distance_threshold(
    mut Dev: VL53LX_DEV,
    mut threshold_high: u16,
    mut threshold_low: u16,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    (*pdev).dyn_cfg.system__thresh_high = threshold_high;
    (*pdev).dyn_cfg.system__thresh_low = threshold_low;
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_set_GPIO_rate_threshold(
    mut Dev: VL53LX_DEV,
    mut threshold_high: u16,
    mut threshold_low: u16,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    (*pdev).gen_cfg.system__thresh_rate_high = threshold_high;
    (*pdev).gen_cfg.system__thresh_rate_low = threshold_low;
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_set_GPIO_thresholds_from_struct(
    mut Dev: VL53LX_DEV,
    mut pintconf: *mut VL53LX_GPIO_interrupt_config_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    status = VL53LX_set_GPIO_distance_threshold(
        Dev,
        (*pintconf).threshold_distance_high,
        (*pintconf).threshold_distance_low,
    );
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_set_GPIO_rate_threshold(
            Dev,
            (*pintconf).threshold_rate_high,
            (*pintconf).threshold_rate_low,
        );
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_set_ref_spad_char_config(
    mut Dev: VL53LX_DEV,
    mut vcsel_period_a: u8,
    mut phasecal_timeout_us: u32,
    mut total_rate_target_mcps: u16,
    mut max_count_rate_rtn_limit_mcps: u16,
    mut min_count_rate_rtn_limit_mcps: u16,
    mut fast_osc_frequency: u16,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    let mut buffer: [u8; 2] = [0; 2];
    let mut macro_period_us: u32 = 0 as libc::c_int as u32;
    let mut timeout_mclks: u32 = 0 as libc::c_int as u32;
    macro_period_us = VL53LX_calc_macro_period_us(fast_osc_frequency, vcsel_period_a);
    if macro_period_us == 0 as libc::c_int as libc::c_uint {
        macro_period_us = 1 as libc::c_int as u32;
    }
    timeout_mclks = phasecal_timeout_us << 12 as libc::c_int;
    timeout_mclks = timeout_mclks.wrapping_add(macro_period_us >> 1 as libc::c_int);
    timeout_mclks = timeout_mclks.wrapping_div(macro_period_us);
    if timeout_mclks > 0xff as libc::c_int as libc::c_uint {
        (*pdev).gen_cfg.phasecal_config__timeout_macrop = 0xff as libc::c_int as u8;
    } else {
        (*pdev).gen_cfg.phasecal_config__timeout_macrop = timeout_mclks as u8;
    }
    (*pdev).tim_cfg.range_config__vcsel_period_a = vcsel_period_a;
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_WrByte(
            Dev,
            0x4b as libc::c_int as u16,
            (*pdev).gen_cfg.phasecal_config__timeout_macrop,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_WrByte(
            Dev,
            0x60 as libc::c_int as u16,
            (*pdev).tim_cfg.range_config__vcsel_period_a,
        );
    }
    buffer[0 as libc::c_int as usize] = (*pdev).tim_cfg.range_config__vcsel_period_a;
    buffer[1 as libc::c_int as usize] = (*pdev).tim_cfg.range_config__vcsel_period_a;
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_WriteMulti(
            Dev,
            0x78 as libc::c_int as u16,
            buffer.as_mut_ptr(),
            2 as libc::c_int as u32,
        );
    }
    (*pdev).customer.ref_spad_char__total_rate_target_mcps = total_rate_target_mcps;
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_WrWord(
            Dev,
            0x1c as libc::c_int as u16,
            total_rate_target_mcps,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_WrWord(
            Dev,
            0x64 as libc::c_int as u16,
            max_count_rate_rtn_limit_mcps,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_WrWord(
            Dev,
            0x66 as libc::c_int as u16,
            min_count_rate_rtn_limit_mcps,
        );
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_set_ssc_config(
    mut Dev: VL53LX_DEV,
    mut pssc_cfg: *mut VL53LX_ssc_config_t,
    mut fast_osc_frequency: u16,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut buffer: [u8; 5] = [0; 5];
    let mut macro_period_us: u32 = 0 as libc::c_int as u32;
    let mut timeout_encoded: u16 = 0 as libc::c_int as u16;
    macro_period_us = VL53LX_calc_macro_period_us(
        fast_osc_frequency,
        (*pssc_cfg).VL53LX_p_005,
    );
    timeout_encoded = VL53LX_calc_encoded_timeout(
        (*pssc_cfg).timeout_us,
        macro_period_us,
    );
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_WrByte(
            Dev,
            0x47 as libc::c_int as u16,
            (*pssc_cfg).vcsel_start,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_WrByte(
            Dev,
            0x4a as libc::c_int as u16,
            (*pssc_cfg).vcsel_width,
        );
    }
    buffer[0 as libc::c_int
        as usize] = ((timeout_encoded as libc::c_int & 0xff00 as libc::c_int)
        >> 8 as libc::c_int) as u8;
    buffer[1 as libc::c_int
        as usize] = (timeout_encoded as libc::c_int & 0xff as libc::c_int) as u8;
    buffer[2 as libc::c_int as usize] = (*pssc_cfg).VL53LX_p_005;
    buffer[3 as libc::c_int
        as usize] = (((*pssc_cfg).rate_limit_mcps as libc::c_int & 0xff00 as libc::c_int)
        >> 8 as libc::c_int) as u8;
    buffer[4 as libc::c_int
        as usize] = ((*pssc_cfg).rate_limit_mcps as libc::c_int & 0xff as libc::c_int)
        as u8;
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_WriteMulti(
            Dev,
            0x61 as libc::c_int as u16,
            buffer.as_mut_ptr(),
            5 as libc::c_int as u32,
        );
    }
    buffer[0 as libc::c_int as usize] = (*pssc_cfg).VL53LX_p_005;
    buffer[1 as libc::c_int as usize] = (*pssc_cfg).VL53LX_p_005;
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_WriteMulti(
            Dev,
            0x78 as libc::c_int as u16,
            buffer.as_mut_ptr(),
            2 as libc::c_int as u32,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_WrByte(
            Dev,
            0x29 as libc::c_int as u16,
            (*pssc_cfg).array_select,
        );
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_get_spad_rate_data(
    mut Dev: VL53LX_DEV,
    mut pspad_rates: *mut VL53LX_spad_rate_data_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut i: libc::c_int = 0 as libc::c_int;
    let mut VL53LX_p_003: [u8; 512] = [0; 512];
    let mut pdata: *mut u8 = &mut *VL53LX_p_003
        .as_mut_ptr()
        .offset(0 as libc::c_int as isize) as *mut u8;
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_disable_firmware(Dev);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_ReadMulti(
            Dev,
            0xe00 as libc::c_int as u16,
            pdata,
            512 as libc::c_int as u32,
        );
    }
    pdata = &mut *VL53LX_p_003.as_mut_ptr().offset(0 as libc::c_int as isize)
        as *mut u8;
    i = 0 as libc::c_int;
    while i < 256 as libc::c_int {
        (*pspad_rates)
            .rate_data[i
            as usize] = VL53LX_decode_unsigned_integer(
            pdata,
            2 as libc::c_int as u8,
        ) as u16;
        pdata = pdata.offset(2 as libc::c_int as isize);
        i += 1;
    }
    (*pspad_rates).VL53LX_p_020 = 256 as libc::c_int as u16;
    (*pspad_rates).no_of_values = 256 as libc::c_int as u16;
    (*pspad_rates).fractional_bits = 15 as libc::c_int as u8;
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_enable_firmware(Dev);
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_dynamic_xtalk_correction_calc_required_samples(
    mut Dev: VL53LX_DEV,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    let mut pres: *mut VL53LX_LLDriverResults_t = &mut (*Dev).Data.llresults;
    let mut pconfig: *mut VL53LX_smudge_corrector_config_t = &mut (*pdev)
        .smudge_correct_config;
    let mut pint: *mut VL53LX_smudge_corrector_internals_t = &mut (*pdev)
        .smudge_corrector_internals;
    let mut presults: *mut VL53LX_range_results_t = &mut (*pres).range_results;
    let mut pxmonitor: *mut VL53LX_range_data_t = &mut (*presults).xmonitor;
    let mut peak_duration_us: u32 = (*pxmonitor).peak_duration_us;
    let mut temp64a: u64 = 0;
    let mut temp64z: u64 = 0;
    temp64a = ((*pxmonitor).VL53LX_p_017).wrapping_add((*pxmonitor).VL53LX_p_016)
        as u64;
    if peak_duration_us == 0 as libc::c_int as libc::c_uint {
        peak_duration_us = 1000 as libc::c_int as u32;
    }
    temp64a = temp64a
        .wrapping_mul(1000 as libc::c_int as libc::c_ulong)
        .wrapping_div(peak_duration_us as libc::c_ulong);
    temp64a = temp64a
        .wrapping_mul(1000 as libc::c_int as libc::c_ulong)
        .wrapping_div(peak_duration_us as libc::c_ulong);
    temp64z = ((*pconfig).noise_margin)
        .wrapping_mul((*pxmonitor).VL53LX_p_004 as libc::c_uint) as u64;
    if temp64z == 0 as libc::c_int as libc::c_ulong {
        temp64z = 1 as libc::c_int as u64;
    }
    temp64a = temp64a
        .wrapping_mul(1000 as libc::c_int as libc::c_ulong)
        .wrapping_mul(256 as libc::c_int as libc::c_ulong);
    temp64a = temp64a.wrapping_div(temp64z);
    temp64a = temp64a
        .wrapping_mul(1000 as libc::c_int as libc::c_ulong)
        .wrapping_mul(256 as libc::c_int as libc::c_ulong);
    temp64a = temp64a.wrapping_div(temp64z);
    (*pint).required_samples = temp64a as u32;
    if (*pint).required_samples < 2 as libc::c_int as libc::c_uint {
        (*pint).required_samples = 2 as libc::c_int as u32;
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_dynamic_xtalk_correction_calc_new_xtalk(
    mut Dev: VL53LX_DEV,
    mut xtalk_offset_out: u32,
    mut pconfig: *mut VL53LX_smudge_corrector_config_t,
    mut pout: *mut VL53LX_smudge_corrector_data_t,
    mut add_smudge: u8,
    mut soft_update: u8,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    let mut x_gradient_scaler: i16 = 0;
    let mut y_gradient_scaler: i16 = 0;
    let mut orig_xtalk_offset: u32 = 0;
    let mut orig_x_gradient: i16 = 0;
    let mut orig_y_gradient: i16 = 0;
    let mut histo_merge_nb: u8 = 0;
    let mut i: u8 = 0;
    let mut itemp32: i32 = 0;
    let mut SmudgeFactor: u32 = 0;
    let mut pX: *mut VL53LX_xtalk_config_t = &mut (*pdev).xtalk_cfg;
    let mut pC: *mut VL53LX_xtalk_calibration_results_t = &mut (*pdev).xtalk_cal;
    let mut pcpo: *mut u32 = 0 as *mut u32;
    let mut max: u32 = 0;
    let mut nXtalk: u32 = 0;
    let mut cXtalk: u32 = 0;
    let mut incXtalk: u32 = 0;
    let mut cval: u32 = 0;
    if add_smudge as libc::c_int == 1 as libc::c_int {
        (*pout)
            .algo__crosstalk_compensation_plane_offset_kcps = xtalk_offset_out
            .wrapping_add((*pconfig).smudge_margin as u32);
    } else {
        (*pout).algo__crosstalk_compensation_plane_offset_kcps = xtalk_offset_out;
    }
    orig_xtalk_offset = (*pX).nvm_default__crosstalk_compensation_plane_offset_kcps;
    orig_x_gradient = (*pX).nvm_default__crosstalk_compensation_x_plane_gradient_kcps;
    orig_y_gradient = (*pX).nvm_default__crosstalk_compensation_y_plane_gradient_kcps;
    if ((*pconfig).user_scaler_set as libc::c_int == 0 as libc::c_int
        || (*pconfig).scaler_calc_method as libc::c_int == 1 as libc::c_int)
        && (*pC).algo__crosstalk_compensation_plane_offset_kcps
            != 0 as libc::c_int as libc::c_uint
    {
        VL53LX_compute_histo_merge_nb(Dev, &mut histo_merge_nb);
        if histo_merge_nb as libc::c_int == 0 as libc::c_int {
            histo_merge_nb = 1 as libc::c_int as u8;
        }
        if (*pdev).tuning_parms.tp_hist_merge as libc::c_int != 1 as libc::c_int {
            orig_xtalk_offset = (*pC).algo__crosstalk_compensation_plane_offset_kcps;
        } else {
            orig_xtalk_offset = (*pC)
                .algo__xtalk_cpo_HistoMerge_kcps[(histo_merge_nb as libc::c_int
                - 1 as libc::c_int) as usize];
        }
        orig_x_gradient = (*pC).algo__crosstalk_compensation_x_plane_gradient_kcps;
        orig_y_gradient = (*pC).algo__crosstalk_compensation_y_plane_gradient_kcps;
    }
    if (*pconfig).user_scaler_set as libc::c_int == 0 as libc::c_int
        && orig_x_gradient as libc::c_int == 0 as libc::c_int
    {
        let ref mut fresh67 = (*pout).gradient_zero_flag;
        *fresh67 = (*fresh67 as libc::c_int | 0x1 as libc::c_int) as u8;
    }
    if (*pconfig).user_scaler_set as libc::c_int == 0 as libc::c_int
        && orig_y_gradient as libc::c_int == 0 as libc::c_int
    {
        let ref mut fresh68 = (*pout).gradient_zero_flag;
        *fresh68 = (*fresh68 as libc::c_int | 0x2 as libc::c_int) as u8;
    }
    if orig_xtalk_offset == 0 as libc::c_int as libc::c_uint {
        orig_xtalk_offset = 1 as libc::c_int as u32;
    }
    if (*pconfig).user_scaler_set as libc::c_int == 1 as libc::c_int {
        x_gradient_scaler = (*pconfig).x_gradient_scaler;
        y_gradient_scaler = (*pconfig).y_gradient_scaler;
    } else {
        x_gradient_scaler = (((orig_x_gradient as i32) << 6 as libc::c_int)
            as libc::c_uint)
            .wrapping_div(orig_xtalk_offset) as i16;
        (*pconfig).x_gradient_scaler = x_gradient_scaler;
        y_gradient_scaler = (((orig_y_gradient as i32) << 6 as libc::c_int)
            as libc::c_uint)
            .wrapping_div(orig_xtalk_offset) as i16;
        (*pconfig).y_gradient_scaler = y_gradient_scaler;
    }
    if (*pconfig).scaler_calc_method as libc::c_int == 0 as libc::c_int {
        itemp32 = ((*pout).algo__crosstalk_compensation_plane_offset_kcps)
            .wrapping_mul(x_gradient_scaler as libc::c_uint) as i32;
        itemp32 = itemp32 >> 6 as libc::c_int;
        if itemp32 > 0xffff as libc::c_int {
            itemp32 = 0xffff as libc::c_int;
        }
        (*pout).algo__crosstalk_compensation_x_plane_gradient_kcps = itemp32 as i16;
        itemp32 = ((*pout).algo__crosstalk_compensation_plane_offset_kcps)
            .wrapping_mul(y_gradient_scaler as libc::c_uint) as i32;
        itemp32 = itemp32 >> 6 as libc::c_int;
        if itemp32 > 0xffff as libc::c_int {
            itemp32 = 0xffff as libc::c_int;
        }
        (*pout).algo__crosstalk_compensation_y_plane_gradient_kcps = itemp32 as i16;
    } else if (*pconfig).scaler_calc_method as libc::c_int == 1 as libc::c_int {
        itemp32 = orig_xtalk_offset
            .wrapping_sub((*pout).algo__crosstalk_compensation_plane_offset_kcps)
            as i32;
        itemp32 = itemp32 / 16 as libc::c_int;
        itemp32 = itemp32 << 2 as libc::c_int;
        itemp32 = itemp32 + orig_x_gradient as i32;
        if itemp32 > 0xffff as libc::c_int {
            itemp32 = 0xffff as libc::c_int;
        }
        (*pout).algo__crosstalk_compensation_x_plane_gradient_kcps = itemp32 as i16;
        itemp32 = orig_xtalk_offset
            .wrapping_sub((*pout).algo__crosstalk_compensation_plane_offset_kcps)
            as i32;
        itemp32 = itemp32 / 80 as libc::c_int;
        itemp32 = itemp32 << 2 as libc::c_int;
        itemp32 = itemp32 + orig_y_gradient as i32;
        if itemp32 > 0xffff as libc::c_int {
            itemp32 = 0xffff as libc::c_int;
        }
        (*pout).algo__crosstalk_compensation_y_plane_gradient_kcps = itemp32 as i16;
    }
    if (*pconfig).smudge_corr_apply_enabled as libc::c_int == 1 as libc::c_int
        && soft_update as libc::c_int != 1 as libc::c_int
    {
        (*pout).new_xtalk_applied_flag = 1 as libc::c_int as u8;
        nXtalk = (*pout).algo__crosstalk_compensation_plane_offset_kcps;
        VL53LX_compute_histo_merge_nb(Dev, &mut histo_merge_nb);
        max = (*pdev).tuning_parms.tp_hist_merge_max_size as u32;
        pcpo = &mut *((*pC).algo__xtalk_cpo_HistoMerge_kcps)
            .as_mut_ptr()
            .offset(0 as libc::c_int as isize) as *mut u32;
        if histo_merge_nb as libc::c_int > 0 as libc::c_int
            && (*pdev).tuning_parms.tp_hist_merge as libc::c_int == 1 as libc::c_int
            && nXtalk != 0 as libc::c_int as libc::c_uint
        {
            cXtalk = (*pX).algo__crosstalk_compensation_plane_offset_kcps;
            SmudgeFactor = cXtalk
                .wrapping_mul(1000 as libc::c_int as libc::c_uint)
                .wrapping_div(nXtalk);
            if max == 0 as libc::c_int as libc::c_uint
                || SmudgeFactor >= (*pconfig).max_smudge_factor
            {
                (*pout).new_xtalk_applied_flag = 0 as libc::c_int as u8;
            } else {
                incXtalk = nXtalk.wrapping_div(max);
                cval = 0 as libc::c_int as u32;
                i = 0 as libc::c_int as u8;
                while (i as libc::c_uint)
                    < max.wrapping_sub(1 as libc::c_int as libc::c_uint)
                {
                    cval = (cval as libc::c_uint).wrapping_add(incXtalk) as u32
                        as u32;
                    *pcpo = cval;
                    pcpo = pcpo.offset(1);
                    i = i.wrapping_add(1);
                }
                *pcpo = nXtalk;
            }
        }
        if (*pout).new_xtalk_applied_flag != 0 {
            (*pX)
                .algo__crosstalk_compensation_plane_offset_kcps = (*pout)
                .algo__crosstalk_compensation_plane_offset_kcps;
            (*pX)
                .algo__crosstalk_compensation_x_plane_gradient_kcps = (*pout)
                .algo__crosstalk_compensation_x_plane_gradient_kcps;
            (*pX)
                .algo__crosstalk_compensation_y_plane_gradient_kcps = (*pout)
                .algo__crosstalk_compensation_y_plane_gradient_kcps;
            if (*pconfig).smudge_corr_single_apply as libc::c_int == 1 as libc::c_int {
                (*pconfig).smudge_corr_apply_enabled = 0 as libc::c_int as u8;
                (*pconfig).smudge_corr_single_apply = 0 as libc::c_int as u8;
            }
        }
    }
    if soft_update as libc::c_int != 1 as libc::c_int {
        (*pout).smudge_corr_valid = 1 as libc::c_int as u8;
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_dynamic_xtalk_correction_corrector(
    mut Dev: VL53LX_DEV,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    let mut pres: *mut VL53LX_LLDriverResults_t = &mut (*Dev).Data.llresults;
    let mut pconfig: *mut VL53LX_smudge_corrector_config_t = &mut (*pdev)
        .smudge_correct_config;
    let mut pint: *mut VL53LX_smudge_corrector_internals_t = &mut (*pdev)
        .smudge_corrector_internals;
    let mut pout: *mut VL53LX_smudge_corrector_data_t = &mut (*pres)
        .range_results
        .smudge_corrector_data;
    let mut pR: *mut VL53LX_range_results_t = &mut (*pres).range_results;
    let mut pX: *mut VL53LX_xtalk_config_t = &mut (*pdev).xtalk_cfg;
    let mut run_smudge_detection: u8 = 0 as libc::c_int as u8;
    let mut merging_complete: u8 = 0 as libc::c_int as u8;
    let mut run_nodetect: u8 = 0 as libc::c_int as u8;
    let mut ambient_check: u8 = 0 as libc::c_int as u8;
    let mut itemp32: i32 = 0 as libc::c_int;
    let mut utemp64: u64 = 0 as libc::c_int as u64;
    let mut continue_processing: u8 = 0 as libc::c_int as u8;
    let mut xtalk_offset_out: u32 = 0 as libc::c_int as u32;
    let mut xtalk_offset_in: u32 = 0 as libc::c_int as u32;
    let mut current_xtalk: u32 = 0 as libc::c_int as u32;
    let mut smudge_margin_adjusted: u32 = 0 as libc::c_int as u32;
    let mut i: u8 = 0 as libc::c_int as u8;
    let mut nodetect_index: u8 = 0 as libc::c_int as u8;
    let mut amr: u16 = 0;
    let mut cco: u32 = 0;
    let mut histo_merge_nb: u8 = 0;
    VL53LX_compute_histo_merge_nb(Dev, &mut histo_merge_nb);
    if histo_merge_nb as libc::c_int == 0 as libc::c_int
        || (*pdev).tuning_parms.tp_hist_merge as libc::c_int != 1 as libc::c_int
    {
        histo_merge_nb = 1 as libc::c_int as u8;
    }
    VL53LX_dynamic_xtalk_correction_output_init(pres);
    ambient_check = ((*pconfig).smudge_corr_ambient_threshold
        == 0 as libc::c_int as libc::c_uint
        || ((*pconfig).smudge_corr_ambient_threshold)
            .wrapping_mul(histo_merge_nb as libc::c_uint)
            > (*pR).xmonitor.ambient_count_rate_mcps as u32) as libc::c_int
        as u8;
    merging_complete = ((*pdev).tuning_parms.tp_hist_merge as libc::c_int
        != 1 as libc::c_int
        || histo_merge_nb as libc::c_int
            == (*pdev).tuning_parms.tp_hist_merge_max_size as libc::c_int) as libc::c_int
        as u8;
    run_smudge_detection = ((*pconfig).smudge_corr_enabled as libc::c_int
        == 1 as libc::c_int && ambient_check as libc::c_int != 0
        && (*pR).xmonitor.range_status as libc::c_int
            == 9 as libc::c_int as VL53LX_DeviceError as libc::c_int
        && merging_complete as libc::c_int != 0) as libc::c_int as u8;
    if (*pR).xmonitor.range_status as libc::c_int
        != 9 as libc::c_int as VL53LX_DeviceError as libc::c_int
        && (*pconfig).smudge_corr_enabled as libc::c_int == 1 as libc::c_int
    {
        run_nodetect = 2 as libc::c_int as u8;
        i = 0 as libc::c_int as u8;
        while (i as libc::c_int) < (*pR).active_results as libc::c_int {
            if (*pR).VL53LX_p_003[i as usize].range_status as libc::c_int
                == 9 as libc::c_int as VL53LX_DeviceError as libc::c_int
            {
                if (*pR).VL53LX_p_003[i as usize].median_range_mm as libc::c_int
                    <= (*pconfig).nodetect_min_range_mm as libc::c_int
                {
                    run_nodetect = 0 as libc::c_int as u8;
                } else if run_nodetect as libc::c_int == 2 as libc::c_int {
                    run_nodetect = 1 as libc::c_int as u8;
                    nodetect_index = i;
                }
            }
            i = i.wrapping_add(1);
        }
        if run_nodetect as libc::c_int == 2 as libc::c_int {
            run_nodetect = 0 as libc::c_int as u8;
        }
        amr = (*pR).VL53LX_p_003[nodetect_index as usize].ambient_count_rate_mcps;
        if run_nodetect as libc::c_int == 1 as libc::c_int {
            utemp64 = (1000 as libc::c_int as libc::c_ulong)
                .wrapping_mul(amr as u64);
            utemp64 = utemp64 << 9 as libc::c_int;
            if utemp64 < (*pconfig).nodetect_ambient_threshold as libc::c_ulong {
                run_nodetect = 1 as libc::c_int as u8;
            } else {
                run_nodetect = 0 as libc::c_int as u8;
            }
        }
    }
    if run_smudge_detection != 0 {
        (*pint).nodetect_counter = 0 as libc::c_int as u32;
        VL53LX_dynamic_xtalk_correction_calc_required_samples(Dev);
        xtalk_offset_in = (*pR).xmonitor.VL53LX_p_009;
        cco = (*pX).algo__crosstalk_compensation_plane_offset_kcps;
        current_xtalk = cco << 2 as libc::c_int;
        smudge_margin_adjusted = ((*pconfig).smudge_margin as u32)
            << 2 as libc::c_int;
        itemp32 = xtalk_offset_in
            .wrapping_sub(current_xtalk)
            .wrapping_add(smudge_margin_adjusted) as i32;
        if itemp32 < 0 as libc::c_int {
            itemp32 = itemp32 * -(1 as libc::c_int);
        }
        if itemp32 > (*pconfig).single_xtalk_delta as i32 {
            if xtalk_offset_in as i32
                > current_xtalk as i32 - smudge_margin_adjusted as i32
            {
                (*pout).single_xtalk_delta_flag = 1 as libc::c_int as u8;
            } else {
                (*pout).single_xtalk_delta_flag = 2 as libc::c_int as u8;
            }
        }
        (*pint)
            .current_samples = ((*pint).current_samples)
            .wrapping_add(1 as libc::c_int as libc::c_uint);
        if (*pint).current_samples > (*pconfig).sample_limit {
            (*pout).sample_limit_exceeded_flag = 1 as libc::c_int as u8;
            continue_processing = 2 as libc::c_int as u8;
        } else {
            (*pint)
                .accumulator = ((*pint).accumulator)
                .wrapping_add(xtalk_offset_in as libc::c_ulong);
        }
        if (*pint).current_samples < (*pint).required_samples {
            continue_processing = 1 as libc::c_int as u8;
        }
        xtalk_offset_out = ((*pint).accumulator)
            .wrapping_div((*pint).current_samples as libc::c_ulong) as u32;
        itemp32 = xtalk_offset_out
            .wrapping_sub(current_xtalk)
            .wrapping_add(smudge_margin_adjusted) as i32;
        if itemp32 < 0 as libc::c_int {
            itemp32 = itemp32 * -(1 as libc::c_int);
        }
        if continue_processing as libc::c_int == 0 as libc::c_int
            && itemp32 >= (*pconfig).averaged_xtalk_delta as i32
        {
            if xtalk_offset_out as i32
                > current_xtalk as i32 - smudge_margin_adjusted as i32
            {
                (*pout).averaged_xtalk_delta_flag = 1 as libc::c_int as u8;
            } else {
                (*pout).averaged_xtalk_delta_flag = 2 as libc::c_int as u8;
            }
        }
        if continue_processing as libc::c_int == 0 as libc::c_int
            && itemp32 < (*pconfig).averaged_xtalk_delta as i32
        {
            continue_processing = 2 as libc::c_int as u8;
        }
        (*pout).smudge_corr_clipped = 0 as libc::c_int as u8;
        if continue_processing as libc::c_int == 0 as libc::c_int
            && (*pconfig).smudge_corr_clip_limit != 0 as libc::c_int as libc::c_uint
        {
            if xtalk_offset_out
                > ((*pconfig).smudge_corr_clip_limit)
                    .wrapping_mul(histo_merge_nb as libc::c_uint)
            {
                (*pout).smudge_corr_clipped = 1 as libc::c_int as u8;
                continue_processing = 2 as libc::c_int as u8;
            }
        }
        if (*pconfig).user_xtalk_offset_limit_hi as libc::c_int != 0
            && xtalk_offset_out > (*pconfig).user_xtalk_offset_limit
        {
            xtalk_offset_out = (*pconfig).user_xtalk_offset_limit;
        }
        if (*pconfig).user_xtalk_offset_limit_hi as libc::c_int == 0 as libc::c_int
            && xtalk_offset_out < (*pconfig).user_xtalk_offset_limit
        {
            xtalk_offset_out = (*pconfig).user_xtalk_offset_limit;
        }
        xtalk_offset_out = xtalk_offset_out >> 2 as libc::c_int;
        if xtalk_offset_out > 0x3ffff as libc::c_int as libc::c_uint {
            xtalk_offset_out = 0x3ffff as libc::c_int as u32;
        }
        if continue_processing as libc::c_int == 0 as libc::c_int {
            VL53LX_dynamic_xtalk_correction_calc_new_xtalk(
                Dev,
                xtalk_offset_out,
                pconfig,
                pout,
                1 as libc::c_int as u8,
                0 as libc::c_int as u8,
            );
            continue_processing = 2 as libc::c_int as u8;
        } else {
            VL53LX_dynamic_xtalk_correction_calc_new_xtalk(
                Dev,
                xtalk_offset_out,
                pconfig,
                pout,
                1 as libc::c_int as u8,
                1 as libc::c_int as u8,
            );
        }
        if continue_processing as libc::c_int == 2 as libc::c_int {
            (*pint).accumulator = 0 as libc::c_int as u64;
            (*pint).current_samples = 0 as libc::c_int as u32;
            (*pint).nodetect_counter = 0 as libc::c_int as u32;
        }
    }
    continue_processing = 0 as libc::c_int as u8;
    if run_nodetect as libc::c_int == 1 as libc::c_int {
        let ref mut fresh69 = (*pint).nodetect_counter;
        *fresh69 = (*fresh69 as libc::c_uint)
            .wrapping_add(1 as libc::c_int as libc::c_uint) as u32 as u32;
        if (*pint).nodetect_counter < (*pconfig).nodetect_sample_limit {
            continue_processing = 1 as libc::c_int as u8;
        }
        xtalk_offset_out = (*pconfig).nodetect_xtalk_offset;
        if continue_processing as libc::c_int == 0 as libc::c_int {
            VL53LX_dynamic_xtalk_correction_calc_new_xtalk(
                Dev,
                xtalk_offset_out,
                pconfig,
                pout,
                0 as libc::c_int as u8,
                0 as libc::c_int as u8,
            );
            (*pout).smudge_corr_valid = 2 as libc::c_int as u8;
            continue_processing = 2 as libc::c_int as u8;
        } else {
            VL53LX_dynamic_xtalk_correction_calc_new_xtalk(
                Dev,
                xtalk_offset_out,
                pconfig,
                pout,
                0 as libc::c_int as u8,
                1 as libc::c_int as u8,
            );
        }
        if continue_processing as libc::c_int == 2 as libc::c_int {
            (*pint).accumulator = 0 as libc::c_int as u64;
            (*pint).current_samples = 0 as libc::c_int as u32;
            (*pint).nodetect_counter = 0 as libc::c_int as u32;
        }
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_dynamic_xtalk_correction_data_init(
    mut Dev: VL53LX_DEV,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    let mut pres: *mut VL53LX_LLDriverResults_t = &mut (*Dev).Data.llresults;
    (*pdev).smudge_correct_config.smudge_corr_enabled = 1 as libc::c_int as u8;
    (*pdev)
        .smudge_correct_config
        .smudge_corr_apply_enabled = 1 as libc::c_int as u8;
    (*pdev).smudge_correct_config.smudge_corr_single_apply = 0 as libc::c_int as u8;
    (*pdev).smudge_correct_config.smudge_margin = 0 as libc::c_int as u16;
    (*pdev).smudge_correct_config.noise_margin = 100 as libc::c_int as u32;
    (*pdev).smudge_correct_config.user_xtalk_offset_limit = 0 as libc::c_int as u32;
    (*pdev)
        .smudge_correct_config
        .user_xtalk_offset_limit_hi = 0 as libc::c_int as u8;
    (*pdev).smudge_correct_config.sample_limit = 200 as libc::c_int as u32;
    (*pdev).smudge_correct_config.single_xtalk_delta = 2048 as libc::c_int as u32;
    (*pdev).smudge_correct_config.averaged_xtalk_delta = 308 as libc::c_int as u32;
    (*pdev)
        .smudge_correct_config
        .smudge_corr_clip_limit = 10240 as libc::c_int as u32;
    (*pdev)
        .smudge_correct_config
        .smudge_corr_ambient_threshold = 128 as libc::c_int as u32;
    (*pdev).smudge_correct_config.scaler_calc_method = 0 as libc::c_int as u8;
    (*pdev).smudge_correct_config.x_gradient_scaler = 256 as libc::c_int as i16;
    (*pdev).smudge_correct_config.y_gradient_scaler = 256 as libc::c_int as i16;
    (*pdev).smudge_correct_config.user_scaler_set = 0 as libc::c_int as u8;
    (*pdev)
        .smudge_correct_config
        .nodetect_ambient_threshold = 57671680 as libc::c_int as u32;
    (*pdev).smudge_correct_config.nodetect_sample_limit = 40 as libc::c_int as u32;
    (*pdev).smudge_correct_config.nodetect_xtalk_offset = 410 as libc::c_int as u32;
    (*pdev).smudge_correct_config.nodetect_min_range_mm = 900 as libc::c_int as u16;
    (*pdev).smudge_correct_config.max_smudge_factor = 2000 as libc::c_int as u32;
    (*pdev).smudge_corrector_internals.current_samples = 0 as libc::c_int as u32;
    (*pdev).smudge_corrector_internals.required_samples = 0 as libc::c_int as u32;
    (*pdev).smudge_corrector_internals.accumulator = 0 as libc::c_int as u64;
    (*pdev).smudge_corrector_internals.nodetect_counter = 0 as libc::c_int as u32;
    VL53LX_dynamic_xtalk_correction_output_init(pres);
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_dynamic_xtalk_correction_output_init(
    mut pres: *mut VL53LX_LLDriverResults_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdata: *mut VL53LX_smudge_corrector_data_t = 0
        as *mut VL53LX_smudge_corrector_data_t;
    pdata = &mut (*pres).range_results.smudge_corrector_data;
    (*pdata).smudge_corr_valid = 0 as libc::c_int as u8;
    (*pdata).smudge_corr_clipped = 0 as libc::c_int as u8;
    (*pdata).single_xtalk_delta_flag = 0 as libc::c_int as u8;
    (*pdata).averaged_xtalk_delta_flag = 0 as libc::c_int as u8;
    (*pdata).sample_limit_exceeded_flag = 0 as libc::c_int as u8;
    (*pdata).gradient_zero_flag = 0 as libc::c_int as u8;
    (*pdata).new_xtalk_applied_flag = 0 as libc::c_int as u8;
    (*pdata)
        .algo__crosstalk_compensation_plane_offset_kcps = 0 as libc::c_int as u32;
    (*pdata)
        .algo__crosstalk_compensation_x_plane_gradient_kcps = 0 as libc::c_int
        as i16;
    (*pdata)
        .algo__crosstalk_compensation_y_plane_gradient_kcps = 0 as libc::c_int
        as i16;
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_xtalk_cal_data_init(
    mut Dev: VL53LX_DEV,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    (*pdev)
        .xtalk_cal
        .algo__crosstalk_compensation_plane_offset_kcps = 0 as libc::c_int as u32;
    (*pdev)
        .xtalk_cal
        .algo__crosstalk_compensation_x_plane_gradient_kcps = 0 as libc::c_int
        as i16;
    (*pdev)
        .xtalk_cal
        .algo__crosstalk_compensation_y_plane_gradient_kcps = 0 as libc::c_int
        as i16;
    memset(
        &mut *((*pdev).xtalk_cal.algo__xtalk_cpo_HistoMerge_kcps)
            .as_mut_ptr()
            .offset(0 as libc::c_int as isize) as *mut u32 as *mut libc::c_void,
        0 as libc::c_int,
        ::std::mem::size_of::<[u32; 6]>() as libc::c_ulong,
    );
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_low_power_auto_data_init(
    mut Dev: VL53LX_DEV,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    (*pdev).low_power_auto_data.vhv_loop_bound = 3 as libc::c_int as u8;
    (*pdev).low_power_auto_data.is_low_power_auto_mode = 0 as libc::c_int as u8;
    (*pdev).low_power_auto_data.low_power_auto_range_count = 0 as libc::c_int as u8;
    (*pdev).low_power_auto_data.saved_interrupt_config = 0 as libc::c_int as u8;
    (*pdev).low_power_auto_data.saved_vhv_init = 0 as libc::c_int as u8;
    (*pdev).low_power_auto_data.saved_vhv_timeout = 0 as libc::c_int as u8;
    (*pdev).low_power_auto_data.first_run_phasecal_result = 0 as libc::c_int as u8;
    (*pdev)
        .low_power_auto_data
        .dss__total_rate_per_spad_mcps = 0 as libc::c_int as u32;
    (*pdev).low_power_auto_data.dss__required_spads = 0 as libc::c_int as u16;
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_low_power_auto_data_stop_range(
    mut Dev: VL53LX_DEV,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    (*pdev)
        .low_power_auto_data
        .low_power_auto_range_count = 0xff as libc::c_int as u8;
    (*pdev).low_power_auto_data.first_run_phasecal_result = 0 as libc::c_int as u8;
    (*pdev)
        .low_power_auto_data
        .dss__total_rate_per_spad_mcps = 0 as libc::c_int as u32;
    (*pdev).low_power_auto_data.dss__required_spads = 0 as libc::c_int as u16;
    if (*pdev).low_power_auto_data.saved_vhv_init as libc::c_int != 0 as libc::c_int {
        (*pdev).stat_nvm.vhv_config__init = (*pdev).low_power_auto_data.saved_vhv_init;
    }
    if (*pdev).low_power_auto_data.saved_vhv_timeout as libc::c_int != 0 as libc::c_int {
        (*pdev)
            .stat_nvm
            .vhv_config__timeout_macrop_loop_bound = (*pdev)
            .low_power_auto_data
            .saved_vhv_timeout;
    }
    (*pdev).gen_cfg.phasecal_config__override = 0 as libc::c_int as u8;
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_config_low_power_auto_mode(
    mut pgeneral: *mut VL53LX_general_config_t,
    mut pdynamic: *mut VL53LX_dynamic_config_t,
    mut plpadata: *mut VL53LX_low_power_auto_data_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    (*plpadata).is_low_power_auto_mode = 1 as libc::c_int as u8;
    (*plpadata).low_power_auto_range_count = 0 as libc::c_int as u8;
    (*pdynamic)
        .system__sequence_config = (0x1 as libc::c_int | 0x2 as libc::c_int
        | 0x8 as libc::c_int | 0x80 as libc::c_int) as u8;
    (*pgeneral)
        .dss_config__manual_effective_spads_select = ((200 as libc::c_int)
        << 8 as libc::c_int) as u16;
    (*pgeneral).dss_config__roi_mode_control = 2 as libc::c_int as VL53LX_DeviceDssMode;
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_low_power_auto_setup_manual_calibration(
    mut Dev: VL53LX_DEV,
) -> VL53LX_Error {
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    (*pdev).low_power_auto_data.saved_vhv_init = (*pdev).stat_nvm.vhv_config__init;
    (*pdev)
        .low_power_auto_data
        .saved_vhv_timeout = (*pdev).stat_nvm.vhv_config__timeout_macrop_loop_bound;
    let ref mut fresh70 = (*pdev).stat_nvm.vhv_config__init;
    *fresh70 = (*fresh70 as libc::c_int & 0x7f as libc::c_int) as u8;
    (*pdev)
        .stat_nvm
        .vhv_config__timeout_macrop_loop_bound = (((*pdev)
        .stat_nvm
        .vhv_config__timeout_macrop_loop_bound as libc::c_int & 0x3 as libc::c_int)
        + (((*pdev).low_power_auto_data.vhv_loop_bound as libc::c_int)
            << 2 as libc::c_int)) as u8;
    (*pdev).gen_cfg.phasecal_config__override = 0x1 as libc::c_int as u8;
    (*pdev)
        .low_power_auto_data
        .first_run_phasecal_result = (*pdev).dbg_results.phasecal_result__vcsel_start;
    (*pdev)
        .gen_cfg
        .cal_config__vcsel_start = (*pdev).low_power_auto_data.first_run_phasecal_result;
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_low_power_auto_update_DSS(
    mut Dev: VL53LX_DEV,
) -> VL53LX_Error {
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    let mut pS: *mut VL53LX_system_results_t = &mut (*pdev).sys_results;
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut utemp32a: u32 = 0;
    utemp32a = ((*pS).result__peak_signal_count_rate_crosstalk_corrected_mcps_sd0
        as libc::c_int + (*pS).result__ambient_count_rate_mcps_sd0 as libc::c_int)
        as u32;
    if utemp32a > 0xffff as libc::c_int as libc::c_uint {
        utemp32a = 0xffff as libc::c_int as u32;
    }
    utemp32a = utemp32a << 16 as libc::c_int;
    if (*pdev).sys_results.result__dss_actual_effective_spads_sd0 as libc::c_int
        == 0 as libc::c_int
    {
        status = -(15 as libc::c_int) as VL53LX_Error;
    } else {
        utemp32a = utemp32a
            .wrapping_div(
                (*pdev).sys_results.result__dss_actual_effective_spads_sd0
                    as libc::c_uint,
            );
        (*pdev).low_power_auto_data.dss__total_rate_per_spad_mcps = utemp32a;
        utemp32a = (((*pdev).stat_cfg.dss_config__target_total_rate_mcps as libc::c_int)
            << 16 as libc::c_int) as u32;
        if (*pdev).low_power_auto_data.dss__total_rate_per_spad_mcps
            == 0 as libc::c_int as libc::c_uint
        {
            status = -(15 as libc::c_int) as VL53LX_Error;
        } else {
            utemp32a = utemp32a
                .wrapping_div((*pdev).low_power_auto_data.dss__total_rate_per_spad_mcps);
            if utemp32a > 0xffff as libc::c_int as libc::c_uint {
                utemp32a = 0xffff as libc::c_int as u32;
            }
            (*pdev).low_power_auto_data.dss__required_spads = utemp32a as u16;
            (*pdev)
                .gen_cfg
                .dss_config__manual_effective_spads_select = (*pdev)
                .low_power_auto_data
                .dss__required_spads;
            (*pdev)
                .gen_cfg
                .dss_config__roi_mode_control = 2 as libc::c_int as VL53LX_DeviceDssMode;
        }
    }
    if status as libc::c_int == -(15 as libc::c_int) as VL53LX_Error as libc::c_int {
        (*pdev)
            .low_power_auto_data
            .dss__required_spads = 0x8000 as libc::c_int as u16;
        (*pdev)
            .gen_cfg
            .dss_config__manual_effective_spads_select = (*pdev)
            .low_power_auto_data
            .dss__required_spads;
        (*pdev)
            .gen_cfg
            .dss_config__roi_mode_control = 2 as libc::c_int as VL53LX_DeviceDssMode;
        status = 0 as libc::c_int as VL53LX_Error;
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_compute_histo_merge_nb(
    mut Dev: VL53LX_DEV,
    mut histo_merge_nb: *mut u8,
) -> VL53LX_Error {
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut i: u8 = 0;
    let mut timing: u8 = 0;
    let mut sum: u8 = 0 as libc::c_int as u8;
    timing = (if (*pdev).hist_data.bin_seq[0 as libc::c_int as usize] as libc::c_int
        == 7 as libc::c_int
    {
        1 as libc::c_int
    } else {
        0 as libc::c_int
    }) as u8;
    i = 0 as libc::c_int as u8;
    while (i as libc::c_int) < 6 as libc::c_int {
        if (*pdev).multi_bins_rec[i as usize][timing as usize][7 as libc::c_int as usize]
            > 0 as libc::c_int
        {
            sum = sum.wrapping_add(1);
        }
        i = i.wrapping_add(1);
    }
    *histo_merge_nb = sum;
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_calc_pll_period_us(
    mut fast_osc_frequency: u16,
) -> u32 {
    let mut pll_period_us: u32 = 0 as libc::c_int as u32;
    if fast_osc_frequency as libc::c_int > 0 as libc::c_int {
        pll_period_us = (((0x1 as libc::c_int) << 30 as libc::c_int)
            / fast_osc_frequency as libc::c_int) as u32;
    }
    return pll_period_us;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_duration_maths(
    mut pll_period_us: u32,
    mut vcsel_parm_pclks: u32,
    mut window_vclks: u32,
    mut elapsed_mclks: u32,
) -> u32 {
    let mut tmp_long_int: u64 = 0 as libc::c_int as u64;
    let mut duration_us: u32 = 0 as libc::c_int as u32;
    duration_us = window_vclks.wrapping_mul(pll_period_us);
    duration_us = duration_us >> 12 as libc::c_int;
    tmp_long_int = duration_us as u64;
    duration_us = elapsed_mclks.wrapping_mul(vcsel_parm_pclks);
    duration_us = duration_us >> 4 as libc::c_int;
    tmp_long_int = tmp_long_int.wrapping_mul(duration_us as u64);
    tmp_long_int = tmp_long_int >> 12 as libc::c_int;
    if tmp_long_int > 0xffffffff as libc::c_uint as libc::c_ulong {
        tmp_long_int = 0xffffffff as libc::c_uint as u64;
    }
    duration_us = tmp_long_int as u32;
    return duration_us;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_events_per_spad_maths(
    mut VL53LX_p_010: i32,
    mut num_spads: u16,
    mut duration: u32,
) -> u32 {
    let mut total_hist_counts: u64 = 0 as libc::c_int as u64;
    let mut xtalk_per_spad: u64 = 0 as libc::c_int as u64;
    let mut rate_per_spad_kcps: u32 = 0 as libc::c_int as u32;
    let mut dividend: u64 = (VL53LX_p_010 as u64)
        .wrapping_mul(1000 as libc::c_int as libc::c_ulong)
        .wrapping_mul(256 as libc::c_int as libc::c_ulong);
    if num_spads as libc::c_int != 0 as libc::c_int {
        total_hist_counts = dividend.wrapping_div(num_spads as u64);
    }
    if duration > 0 as libc::c_int as libc::c_uint {
        let mut dividend_0: u64 = (total_hist_counts << 11 as libc::c_int)
            .wrapping_add(
                (duration as u64).wrapping_div(2 as libc::c_int as libc::c_ulong),
            );
        xtalk_per_spad = dividend_0.wrapping_div(duration as u64);
    } else {
        xtalk_per_spad = total_hist_counts << 11 as libc::c_int;
    }
    rate_per_spad_kcps = xtalk_per_spad as u32;
    return rate_per_spad_kcps;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_isqrt(mut num: u32) -> u32 {
    let mut res: u32 = 0 as libc::c_int as u32;
    let mut bit: u32 = ((1 as libc::c_int) << 30 as libc::c_int) as u32;
    while bit > num {
        bit >>= 2 as libc::c_int;
    }
    while bit != 0 as libc::c_int as libc::c_uint {
        if num >= res.wrapping_add(bit) {
            num = (num as libc::c_uint).wrapping_sub(res.wrapping_add(bit)) as u32
                as u32;
            res = (res >> 1 as libc::c_int).wrapping_add(bit);
        } else {
            res >>= 1 as libc::c_int;
        }
        bit >>= 2 as libc::c_int;
    }
    return res;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_hist_calc_zero_distance_phase(
    mut pdata: *mut VL53LX_histogram_bin_data_t,
) {
    let mut period: u32 = 0 as libc::c_int as u32;
    let mut VL53LX_p_014: u32 = 0 as libc::c_int as u32;
    period = (2048 as libc::c_int as libc::c_uint)
        .wrapping_mul(VL53LX_decode_vcsel_period((*pdata).VL53LX_p_005) as u32);
    VL53LX_p_014 = period;
    VL53LX_p_014 = (VL53LX_p_014 as libc::c_uint)
        .wrapping_add((*pdata).phasecal_result__reference_phase as u32) as u32
        as u32;
    VL53LX_p_014 = (VL53LX_p_014 as libc::c_uint)
        .wrapping_add(
            (2048 as libc::c_int as libc::c_uint)
                .wrapping_mul((*pdata).phasecal_result__vcsel_start as u32),
        ) as u32 as u32;
    VL53LX_p_014 = (VL53LX_p_014 as libc::c_uint)
        .wrapping_sub(
            (2048 as libc::c_int as libc::c_uint)
                .wrapping_mul((*pdata).cal_config__vcsel_start as u32),
        ) as u32 as u32;
    if period != 0 as libc::c_int as libc::c_uint {
        VL53LX_p_014 = VL53LX_p_014.wrapping_rem(period);
    } else {
        VL53LX_p_014 = 0 as libc::c_int as u32;
    }
    (*pdata).zero_distance_phase = VL53LX_p_014 as u16;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_hist_estimate_ambient_from_thresholded_bins(
    mut ambient_threshold_sigma: i32,
    mut pdata: *mut VL53LX_histogram_bin_data_t,
) {
    let mut bin: u8 = 0 as libc::c_int as u8;
    let mut VL53LX_p_031: i32 = 0 as libc::c_int;
    VL53LX_hist_find_min_max_bin_values(pdata);
    VL53LX_p_031 = VL53LX_isqrt((*pdata).min_bin_value as u32) as i32;
    VL53LX_p_031 *= ambient_threshold_sigma;
    VL53LX_p_031 += 0x7 as libc::c_int;
    VL53LX_p_031 = VL53LX_p_031 >> 4 as libc::c_int;
    VL53LX_p_031 += (*pdata).min_bin_value;
    (*pdata).number_of_ambient_samples = 0 as libc::c_int as u8;
    (*pdata).ambient_events_sum = 0 as libc::c_int;
    bin = 0 as libc::c_int as u8;
    while (bin as libc::c_int) < (*pdata).VL53LX_p_021 as libc::c_int {
        if (*pdata).bin_data[bin as usize] < VL53LX_p_031 {
            let ref mut fresh71 = (*pdata).ambient_events_sum;
            *fresh71 += (*pdata).bin_data[bin as usize];
            let ref mut fresh72 = (*pdata).number_of_ambient_samples;
            *fresh72 = (*fresh72).wrapping_add(1);
        }
        bin = bin.wrapping_add(1);
    }
    if (*pdata).number_of_ambient_samples as libc::c_int > 0 as libc::c_int {
        (*pdata).VL53LX_p_028 = (*pdata).ambient_events_sum;
        let ref mut fresh73 = (*pdata).VL53LX_p_028;
        *fresh73 += (*pdata).number_of_ambient_samples as i32 / 2 as libc::c_int;
        let ref mut fresh74 = (*pdata).VL53LX_p_028;
        *fresh74 /= (*pdata).number_of_ambient_samples as i32;
    }
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_hist_remove_ambient_bins(
    mut pdata: *mut VL53LX_histogram_bin_data_t,
) {
    let mut bin: u8 = 0 as libc::c_int as u8;
    let mut lc: u8 = 0 as libc::c_int as u8;
    let mut i: u8 = 0 as libc::c_int as u8;
    if (*pdata).bin_seq[0 as libc::c_int as usize] as libc::c_int & 0x7 as libc::c_int
        == 0x7 as libc::c_int
    {
        i = 0 as libc::c_int as u8;
        lc = 0 as libc::c_int as u8;
        while (lc as libc::c_int) < 6 as libc::c_int {
            if (*pdata).bin_seq[lc as usize] as libc::c_int & 0x7 as libc::c_int
                != 0x7 as libc::c_int
            {
                (*pdata).bin_seq[i as usize] = (*pdata).bin_seq[lc as usize];
                (*pdata).bin_rep[i as usize] = (*pdata).bin_rep[lc as usize];
                i = i.wrapping_add(1);
            }
            lc = lc.wrapping_add(1);
        }
        lc = i;
        while (lc as libc::c_int) < 6 as libc::c_int {
            (*pdata)
                .bin_seq[lc
                as usize] = (15 as libc::c_int + 1 as libc::c_int) as u8;
            (*pdata).bin_rep[lc as usize] = 0 as libc::c_int as u8;
            lc = lc.wrapping_add(1);
        }
    }
    if (*pdata).number_of_ambient_bins as libc::c_int > 0 as libc::c_int {
        bin = (*pdata).number_of_ambient_bins;
        while (bin as libc::c_int) < (*pdata).VL53LX_p_020 as libc::c_int {
            (*pdata)
                .bin_data[(bin as libc::c_int
                - (*pdata).number_of_ambient_bins as libc::c_int)
                as usize] = (*pdata).bin_data[bin as usize];
            bin = bin.wrapping_add(1);
        }
        (*pdata)
            .VL53LX_p_021 = ((*pdata).VL53LX_p_021 as libc::c_int
            - (*pdata).number_of_ambient_bins as libc::c_int) as u8;
        (*pdata).number_of_ambient_bins = 0 as libc::c_int as u8;
    }
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_calc_pll_period_mm(
    mut fast_osc_frequency: u16,
) -> u32 {
    let mut pll_period_us: u32 = 0 as libc::c_int as u32;
    let mut pll_period_mm: u32 = 0 as libc::c_int as u32;
    pll_period_us = VL53LX_calc_pll_period_us(fast_osc_frequency);
    pll_period_mm = ((299704 as libc::c_int >> 3 as libc::c_int) as libc::c_uint)
        .wrapping_mul(pll_period_us >> 2 as libc::c_int);
    pll_period_mm = pll_period_mm
        .wrapping_add(((0x1 as libc::c_int) << 15 as libc::c_int) as libc::c_uint)
        >> 16 as libc::c_int;
    return pll_period_mm;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_rate_maths(
    mut VL53LX_p_018: i32,
    mut time_us: u32,
) -> u16 {
    let mut tmp_int: u32 = 0 as libc::c_int as u32;
    let mut frac_bits: u32 = 7 as libc::c_int as u32;
    let mut rate_mcps: u16 = 0 as libc::c_int as u16;
    if VL53LX_p_018 > ((0x1 as libc::c_int) << 29 as libc::c_int) - 1 as libc::c_int {
        tmp_int = (((0x1 as libc::c_int) << 29 as libc::c_int) - 1 as libc::c_int)
            as u32;
    } else if VL53LX_p_018 > 0 as libc::c_int {
        tmp_int = VL53LX_p_018 as u32;
    }
    if VL53LX_p_018 > (0x1 as libc::c_int) << 24 as libc::c_int {
        frac_bits = 3 as libc::c_int as u32;
    } else {
        frac_bits = 7 as libc::c_int as u32;
    }
    if time_us > 0 as libc::c_int as libc::c_uint {
        tmp_int = (tmp_int << frac_bits)
            .wrapping_add(time_us.wrapping_div(2 as libc::c_int as libc::c_uint))
            .wrapping_div(time_us);
    }
    if VL53LX_p_018 > (0x1 as libc::c_int) << 24 as libc::c_int {
        tmp_int = tmp_int << 4 as libc::c_int;
    }
    if tmp_int > 0xffff as libc::c_int as libc::c_uint {
        tmp_int = 0xffff as libc::c_int as u32;
    }
    rate_mcps = tmp_int as u16;
    return rate_mcps;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_rate_per_spad_maths(
    mut frac_bits: u32,
    mut peak_count_rate: u32,
    mut num_spads: u16,
    mut max_output_value: u32,
) -> u16 {
    let mut tmp_int: u32 = 0 as libc::c_int as u32;
    let mut rate_per_spad: u16 = 0 as libc::c_int as u16;
    if num_spads as libc::c_int > 0 as libc::c_int {
        tmp_int = (peak_count_rate << 8 as libc::c_int) << frac_bits;
        tmp_int = tmp_int
            .wrapping_add(
                (num_spads as u32).wrapping_div(2 as libc::c_int as libc::c_uint),
            )
            .wrapping_div(num_spads as u32);
    } else {
        tmp_int = peak_count_rate << frac_bits;
    }
    if tmp_int > max_output_value {
        tmp_int = max_output_value;
    }
    rate_per_spad = tmp_int as u16;
    return rate_per_spad;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_range_maths(
    mut fast_osc_frequency: u16,
    mut VL53LX_p_014: u16,
    mut zero_distance_phase: u16,
    mut fractional_bits: u8,
    mut gain_factor: i32,
    mut range_offset_mm: i32,
) -> i32 {
    let mut pll_period_us: u32 = 0 as libc::c_int as u32;
    let mut tmp_long_int: i64 = 0 as libc::c_int as i64;
    let mut range_mm: i32 = 0 as libc::c_int;
    let mut range_mm_10: i32 = 0 as libc::c_int;
    pll_period_us = VL53LX_calc_pll_period_us(fast_osc_frequency);
    tmp_long_int = VL53LX_p_014 as i64 - zero_distance_phase as i64;
    tmp_long_int = tmp_long_int * pll_period_us as i64;
    tmp_long_int = tmp_long_int
        / ((0x1 as libc::c_int) << 9 as libc::c_int) as libc::c_long;
    tmp_long_int = tmp_long_int
        * (299704 as libc::c_int >> 3 as libc::c_int) as libc::c_long;
    tmp_long_int = tmp_long_int
        / ((0x1 as libc::c_int) << 22 as libc::c_int) as libc::c_long;
    range_mm = tmp_long_int as i32 + range_offset_mm;
    range_mm *= gain_factor;
    range_mm += 0x400 as libc::c_int;
    range_mm /= 0x800 as libc::c_int;
    if fractional_bits as libc::c_int == 0 as libc::c_int {
        range_mm_10 = range_mm * 10 as libc::c_int;
        range_mm_10 = range_mm_10 / ((0x1 as libc::c_int) << 2 as libc::c_int);
        if (range_mm_10 % 10 as libc::c_int) < 5 as libc::c_int {
            range_mm = (range_mm_10 / 10 as libc::c_int) as i16 as i32;
        } else {
            range_mm = (range_mm_10 / 10 as libc::c_int + 1 as libc::c_int) as i16
                as i32;
        }
    } else if fractional_bits as libc::c_int == 1 as libc::c_int {
        range_mm = range_mm / ((0x1 as libc::c_int) << 1 as libc::c_int);
    }
    return range_mm;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_decode_vcsel_period(
    mut vcsel_period_reg: u8,
) -> u8 {
    let mut VL53LX_p_030: u8 = 0 as libc::c_int as u8;
    VL53LX_p_030 = ((vcsel_period_reg as libc::c_int + 1 as libc::c_int)
        << 1 as libc::c_int) as u8;
    return VL53LX_p_030;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_copy_xtalk_bin_data_to_histogram_data_struct(
    mut pxtalk: *mut VL53LX_xtalk_histogram_shape_t,
    mut phist: *mut VL53LX_histogram_bin_data_t,
) {
    (*phist).cal_config__vcsel_start = (*pxtalk).cal_config__vcsel_start;
    (*phist).VL53LX_p_015 = (*pxtalk).VL53LX_p_015;
    (*phist).VL53LX_p_019 = (*pxtalk).VL53LX_p_019;
    (*phist)
        .phasecal_result__reference_phase = (*pxtalk).phasecal_result__reference_phase;
    (*phist).phasecal_result__vcsel_start = (*pxtalk).phasecal_result__vcsel_start;
    (*phist).vcsel_width = (*pxtalk).vcsel_width;
    (*phist).zero_distance_phase = (*pxtalk).zero_distance_phase;
    (*phist).zone_id = (*pxtalk).zone_id;
    (*phist).VL53LX_p_020 = (*pxtalk).VL53LX_p_020;
    (*phist).time_stamp = (*pxtalk).time_stamp;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_init_histogram_bin_data_struct(
    mut bin_value: i32,
    mut VL53LX_p_021: u16,
    mut pdata: *mut VL53LX_histogram_bin_data_t,
) {
    let mut i: u16 = 0 as libc::c_int as u16;
    (*pdata).cfg_device_state = 3 as libc::c_int as VL53LX_DeviceState;
    (*pdata).rd_device_state = 3 as libc::c_int as VL53LX_DeviceState;
    (*pdata).zone_id = 0 as libc::c_int as u8;
    (*pdata).time_stamp = 0 as libc::c_int as u32;
    (*pdata).VL53LX_p_019 = 0 as libc::c_int as u8;
    (*pdata).VL53LX_p_020 = 24 as libc::c_int as u8;
    (*pdata).VL53LX_p_021 = VL53LX_p_021 as u8;
    (*pdata).number_of_ambient_bins = 0 as libc::c_int as u8;
    (*pdata).result__interrupt_status = 0 as libc::c_int as u8;
    (*pdata).result__range_status = 0 as libc::c_int as u8;
    (*pdata).result__report_status = 0 as libc::c_int as u8;
    (*pdata).result__stream_count = 0 as libc::c_int as u8;
    (*pdata).result__dss_actual_effective_spads = 0 as libc::c_int as u16;
    (*pdata).phasecal_result__reference_phase = 0 as libc::c_int as u16;
    (*pdata).phasecal_result__vcsel_start = 0 as libc::c_int as u8;
    (*pdata).cal_config__vcsel_start = 0 as libc::c_int as u8;
    (*pdata).vcsel_width = 0 as libc::c_int as u16;
    (*pdata).VL53LX_p_005 = 0 as libc::c_int as u8;
    (*pdata).VL53LX_p_015 = 0 as libc::c_int as u16;
    (*pdata).total_periods_elapsed = 0 as libc::c_int as u32;
    (*pdata).min_bin_value = 0 as libc::c_int;
    (*pdata).max_bin_value = 0 as libc::c_int;
    (*pdata).zero_distance_phase = 0 as libc::c_int as u16;
    (*pdata).number_of_ambient_samples = 0 as libc::c_int as u8;
    (*pdata).ambient_events_sum = 0 as libc::c_int;
    (*pdata).VL53LX_p_028 = 0 as libc::c_int;
    i = 0 as libc::c_int as u16;
    while (i as libc::c_int) < 6 as libc::c_int {
        (*pdata).bin_seq[i as usize] = i as u8;
        i = i.wrapping_add(1);
    }
    i = 0 as libc::c_int as u16;
    while (i as libc::c_int) < 6 as libc::c_int {
        (*pdata).bin_rep[i as usize] = 1 as libc::c_int as u8;
        i = i.wrapping_add(1);
    }
    i = 0 as libc::c_int as u16;
    while (i as libc::c_int) < 24 as libc::c_int {
        if (i as libc::c_int) < VL53LX_p_021 as libc::c_int {
            (*pdata).bin_data[i as usize] = bin_value;
        } else {
            (*pdata).bin_data[i as usize] = 0 as libc::c_int;
        }
        i = i.wrapping_add(1);
    }
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_decode_row_col(
    mut spad_number: u8,
    mut prow: *mut u8,
    mut pcol: *mut u8,
) {
    if spad_number as libc::c_int > 127 as libc::c_int {
        *prow = (8 as libc::c_int
            + (255 as libc::c_int - spad_number as libc::c_int & 0x7 as libc::c_int))
            as u8;
        *pcol = (spad_number as libc::c_int - 128 as libc::c_int >> 3 as libc::c_int)
            as u8;
    } else {
        *prow = (spad_number as libc::c_int & 0x7 as libc::c_int) as u8;
        *pcol = (127 as libc::c_int - spad_number as libc::c_int >> 3 as libc::c_int)
            as u8;
    };
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_hist_find_min_max_bin_values(
    mut pdata: *mut VL53LX_histogram_bin_data_t,
) {
    let mut bin: u8 = 0 as libc::c_int as u8;
    bin = 0 as libc::c_int as u8;
    while (bin as libc::c_int) < (*pdata).VL53LX_p_021 as libc::c_int {
        if bin as libc::c_int == 0 as libc::c_int
            || (*pdata).min_bin_value >= (*pdata).bin_data[bin as usize]
        {
            (*pdata).min_bin_value = (*pdata).bin_data[bin as usize];
        }
        if bin as libc::c_int == 0 as libc::c_int
            || (*pdata).max_bin_value <= (*pdata).bin_data[bin as usize]
        {
            (*pdata).max_bin_value = (*pdata).bin_data[bin as usize];
        }
        bin = bin.wrapping_add(1);
    }
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_hist_estimate_ambient_from_ambient_bins(
    mut pdata: *mut VL53LX_histogram_bin_data_t,
) {
    let mut bin: u8 = 0 as libc::c_int as u8;
    if (*pdata).number_of_ambient_bins as libc::c_int > 0 as libc::c_int {
        (*pdata).number_of_ambient_samples = (*pdata).number_of_ambient_bins;
        (*pdata).ambient_events_sum = 0 as libc::c_int;
        bin = 0 as libc::c_int as u8;
        while (bin as libc::c_int) < (*pdata).number_of_ambient_bins as libc::c_int {
            let ref mut fresh75 = (*pdata).ambient_events_sum;
            *fresh75 += (*pdata).bin_data[bin as usize];
            bin = bin.wrapping_add(1);
        }
        (*pdata).VL53LX_p_028 = (*pdata).ambient_events_sum;
        let ref mut fresh76 = (*pdata).VL53LX_p_028;
        *fresh76 += (*pdata).number_of_ambient_bins as i32 / 2 as libc::c_int;
        let ref mut fresh77 = (*pdata).VL53LX_p_028;
        *fresh77 /= (*pdata).number_of_ambient_bins as i32;
    }
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_f_001(
    mut target_reflectance: u16,
    mut pcal: *mut VL53LX_dmax_calibration_data_t,
    mut pcfg: *mut VL53LX_hist_gen3_dmax_config_t,
    mut pbins: *mut VL53LX_histogram_bin_data_t,
    mut pdata: *mut VL53LX_hist_gen3_dmax_private_data_t,
    mut pambient_dmax_mm: *mut i16,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pll_period_us: u32 = 0 as libc::c_int as u32;
    let mut periods_elapsed: u32 = 0 as libc::c_int as u32;
    let mut tmp32: u32 = 0 as libc::c_int as u32;
    let mut tmp64: u64 = 0 as libc::c_int as u64;
    let mut amb_thres_delta: u32 = 0 as libc::c_int as u32;
    (*pdata).VL53LX_p_004 = 0 as libc::c_int as u16;
    (*pdata).VL53LX_p_033 = 0 as libc::c_int as u32;
    (*pdata).VL53LX_p_034 = 0 as libc::c_int as u16;
    (*pdata).VL53LX_p_009 = 0 as libc::c_int as u32;
    (*pdata).VL53LX_p_028 = 0 as libc::c_int as u32;
    (*pdata).VL53LX_p_035 = 0 as libc::c_int as u32;
    (*pdata).VL53LX_p_036 = 0 as libc::c_int as i16;
    (*pdata).VL53LX_p_022 = 0 as libc::c_int as i16;
    *pambient_dmax_mm = 0 as libc::c_int as i16;
    if (*pbins).VL53LX_p_015 as libc::c_int != 0 as libc::c_int
        && (*pbins).total_periods_elapsed != 0 as libc::c_int as libc::c_uint
    {
        pll_period_us = VL53LX_calc_pll_period_us((*pbins).VL53LX_p_015);
        periods_elapsed = ((*pbins).total_periods_elapsed)
            .wrapping_add(1 as libc::c_int as libc::c_uint);
        (*pdata)
            .VL53LX_p_037 = VL53LX_duration_maths(
            pll_period_us,
            ((1 as libc::c_int) << 4 as libc::c_int) as u32,
            2048 as libc::c_int as u32,
            periods_elapsed,
        );
        (*pdata)
            .VL53LX_p_034 = VL53LX_rate_maths(
            (*pbins).VL53LX_p_028,
            (*pdata).VL53LX_p_037,
        );
        (*pdata)
            .VL53LX_p_033 = VL53LX_events_per_spad_maths(
            (*pbins).VL53LX_p_028,
            (*pbins).result__dss_actual_effective_spads,
            (*pdata).VL53LX_p_037,
        );
        (*pdata).VL53LX_p_038 = (*pcfg).max_effective_spads;
        (*pdata).VL53LX_p_004 = (*pcfg).max_effective_spads;
        if (*pdata).VL53LX_p_033 > 0 as libc::c_int as libc::c_uint {
            tmp64 = (*pcfg).dss_config__target_total_rate_mcps as u64;
            tmp64 = (tmp64 as libc::c_ulong)
                .wrapping_mul(1000 as libc::c_int as libc::c_ulong) as u64
                as u64;
            tmp64 <<= 11 as libc::c_int + 1 as libc::c_int;
            tmp32 = ((*pdata).VL53LX_p_033)
                .wrapping_div(2 as libc::c_int as libc::c_uint);
            tmp64 = (tmp64 as libc::c_ulong).wrapping_add(tmp32 as u64) as u64
                as u64;
            tmp64 = tmp64.wrapping_div((*pdata).VL53LX_p_033 as u64);
            if tmp64 < (*pcfg).max_effective_spads as u64 {
                (*pdata).VL53LX_p_004 = tmp64 as u16;
            }
        }
    }
    if (*pcal).ref__actual_effective_spads as libc::c_int != 0 as libc::c_int
        && (*pbins).VL53LX_p_015 as libc::c_int != 0 as libc::c_int
        && (*pcal).ref_reflectance_pc as libc::c_int != 0 as libc::c_int
        && (*pbins).total_periods_elapsed != 0 as libc::c_int as libc::c_uint
    {
        tmp64 = (*pcal).ref__peak_signal_count_rate_mcps as u64;
        tmp64 = (tmp64 as libc::c_ulong)
            .wrapping_mul((1000 as libc::c_int * 256 as libc::c_int) as libc::c_ulong)
            as u64 as u64;
        tmp32 = ((*pcal).ref__actual_effective_spads as libc::c_int / 2 as libc::c_int)
            as u32;
        tmp64 = (tmp64 as libc::c_ulong).wrapping_add(tmp32 as u64) as u64
            as u64;
        tmp64 = tmp64.wrapping_div((*pcal).ref__actual_effective_spads as u64);
        (*pdata).VL53LX_p_009 = tmp64 as u32;
        (*pdata).VL53LX_p_009 <<= 4 as libc::c_int;
        tmp64 = (*pdata).VL53LX_p_037 as u64;
        tmp64 = (tmp64 as libc::c_ulong).wrapping_mul((*pdata).VL53LX_p_033 as u64)
            as u64 as u64;
        tmp64 = (tmp64 as libc::c_ulong).wrapping_mul((*pdata).VL53LX_p_004 as u64)
            as u64 as u64;
        tmp64 = (tmp64 as libc::c_ulong)
            .wrapping_add(
                ((1 as libc::c_int) << 11 as libc::c_int + 7 as libc::c_int)
                    as libc::c_ulong,
            ) as u64 as u64;
        tmp64 >>= 11 as libc::c_int + 8 as libc::c_int;
        tmp64 = (tmp64 as libc::c_ulong)
            .wrapping_add(500 as libc::c_int as libc::c_ulong) as u64 as u64;
        tmp64 = tmp64.wrapping_div(1000 as libc::c_int as libc::c_ulong);
        if tmp64 > 0xffffff as libc::c_int as libc::c_ulong {
            tmp64 = 0xffffff as libc::c_int as u64;
        }
        (*pdata).VL53LX_p_028 = tmp64 as u32;
        tmp64 = (*pdata).VL53LX_p_037 as u64;
        tmp64 = (tmp64 as libc::c_ulong).wrapping_mul((*pdata).VL53LX_p_009 as u64)
            as u64 as u64;
        tmp64 = (tmp64 as libc::c_ulong).wrapping_mul((*pdata).VL53LX_p_004 as u64)
            as u64 as u64;
        tmp64 = (tmp64 as libc::c_ulong)
            .wrapping_add(
                ((1 as libc::c_int) << 11 as libc::c_int + 7 as libc::c_int)
                    as libc::c_ulong,
            ) as u64 as u64;
        tmp64 >>= 11 as libc::c_int + 8 as libc::c_int;
        tmp64 = (tmp64 as libc::c_ulong)
            .wrapping_mul(
                (target_reflectance as u64)
                    .wrapping_mul((*pcal).coverglass_transmission as u64),
            ) as u64 as u64;
        tmp64 = (tmp64 as libc::c_ulong)
            .wrapping_add(
                ((*pcal).ref_reflectance_pc as u64)
                    .wrapping_mul(128 as libc::c_int as libc::c_ulong),
            ) as u64 as u64;
        tmp64 = tmp64
            .wrapping_div(
                ((*pcal).ref_reflectance_pc as u64)
                    .wrapping_mul(256 as libc::c_int as libc::c_ulong),
            );
        tmp64 = (tmp64 as libc::c_ulong)
            .wrapping_add(500 as libc::c_int as libc::c_ulong) as u64 as u64;
        tmp64 = tmp64.wrapping_div(1000 as libc::c_int as libc::c_ulong);
        if tmp64 > 0xffffff as libc::c_int as libc::c_ulong {
            tmp64 = 0xffffff as libc::c_int as u64;
        }
        (*pdata).VL53LX_p_035 = tmp64 as u32;
        tmp32 = VL53LX_isqrt((*pdata).VL53LX_p_028 << 8 as libc::c_int);
        tmp32 = (tmp32 as libc::c_uint)
            .wrapping_mul((*pcfg).ambient_thresh_sigma as u32) as u32
            as u32;
        if (*pdata).VL53LX_p_028 < (*pcfg).min_ambient_thresh_events as u32 {
            amb_thres_delta = ((*pcfg).min_ambient_thresh_events as libc::c_uint)
                .wrapping_sub((*pdata).VL53LX_p_028);
            amb_thres_delta <<= 8 as libc::c_int;
            if tmp32 < amb_thres_delta {
                tmp32 = amb_thres_delta;
            }
        }
        (*pdata)
            .VL53LX_p_022 = VL53LX_f_002(
            tmp32,
            (*pdata).VL53LX_p_035,
            (*pcal).ref__distance_mm as u32,
            (*pcfg).signal_thresh_sigma as u32,
        ) as i16;
        tmp32 = (*pdata).VL53LX_p_035;
        tmp32 = (tmp32 as libc::c_uint).wrapping_mul((*pbins).vcsel_width as u32)
            as u32 as u32;
        tmp32 = (tmp32 as libc::c_uint)
            .wrapping_add(((1 as libc::c_int) << 3 as libc::c_int) as libc::c_uint)
            as u32 as u32;
        tmp32 = (tmp32 as libc::c_uint)
            .wrapping_div(((1 as libc::c_int) << 4 as libc::c_int) as libc::c_uint)
            as u32 as u32;
        (*pdata)
            .VL53LX_p_036 = VL53LX_f_002(
            (256 as libc::c_int as libc::c_uint)
                .wrapping_mul((*pcfg).signal_total_events_limit as u32),
            tmp32,
            (*pcal).ref__distance_mm as u32,
            (*pcfg).signal_thresh_sigma as u32,
        ) as i16;
        if ((*pdata).VL53LX_p_036 as libc::c_int) < (*pdata).VL53LX_p_022 as libc::c_int
        {
            *pambient_dmax_mm = (*pdata).VL53LX_p_036;
        } else {
            *pambient_dmax_mm = (*pdata).VL53LX_p_022;
        }
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_f_002(
    mut events_threshold: u32,
    mut ref_signal_events: u32,
    mut ref_distance_mm: u32,
    mut signal_thresh_sigma: u32,
) -> u32 {
    let mut tmp32: u32 = 0 as libc::c_int as u32;
    let mut range_mm: u32 = 0 as libc::c_int as u32;
    tmp32 = (4 as libc::c_int as libc::c_uint).wrapping_mul(events_threshold);
    tmp32 = (tmp32 as libc::c_uint)
        .wrapping_add(signal_thresh_sigma.wrapping_mul(signal_thresh_sigma)) as u32
        as u32;
    tmp32 = VL53LX_isqrt(tmp32);
    tmp32 = (tmp32 as libc::c_uint).wrapping_add(signal_thresh_sigma) as u32
        as u32;
    range_mm = VL53LX_isqrt(ref_signal_events << 4 as libc::c_int);
    range_mm = (range_mm as libc::c_uint).wrapping_mul(ref_distance_mm) as u32
        as u32;
    if tmp32 != 0 as libc::c_int as libc::c_uint {
        range_mm = (range_mm as libc::c_uint).wrapping_add(tmp32) as u32
            as u32;
        range_mm = (range_mm as libc::c_uint)
            .wrapping_div((2 as libc::c_int as libc::c_uint).wrapping_mul(tmp32))
            as u32 as u32;
    }
    return range_mm;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_f_003(
    mut palgo: *mut VL53LX_hist_gen3_algo_private_data_t,
) {
    let mut lb: u8 = 0 as libc::c_int as u8;
    (*palgo).VL53LX_p_020 = 24 as libc::c_int as u8;
    (*palgo).VL53LX_p_019 = 0 as libc::c_int as u8;
    (*palgo).VL53LX_p_021 = 0 as libc::c_int as u8;
    (*palgo).VL53LX_p_039 = 0 as libc::c_int as u8;
    (*palgo).VL53LX_p_028 = 0 as libc::c_int;
    (*palgo).VL53LX_p_031 = 0 as libc::c_int;
    lb = (*palgo).VL53LX_p_019;
    while (lb as libc::c_int) < (*palgo).VL53LX_p_020 as libc::c_int {
        (*palgo).VL53LX_p_040[lb as usize] = 0 as libc::c_int as u8;
        (*palgo).VL53LX_p_041[lb as usize] = 0 as libc::c_int as u8;
        (*palgo).VL53LX_p_042[lb as usize] = 0 as libc::c_int as u8;
        (*palgo).VL53LX_p_043[lb as usize] = 0 as libc::c_int;
        (*palgo).VL53LX_p_018[lb as usize] = 0 as libc::c_int;
        lb = lb.wrapping_add(1);
    }
    (*palgo).VL53LX_p_044 = 0 as libc::c_int as u8;
    (*palgo).VL53LX_p_045 = 8 as libc::c_int as u8;
    (*palgo).VL53LX_p_046 = 0 as libc::c_int as u8;
    VL53LX_init_histogram_bin_data_struct(
        0 as libc::c_int,
        24 as libc::c_int as u16,
        &mut (*palgo).VL53LX_p_006,
    );
    VL53LX_init_histogram_bin_data_struct(
        0 as libc::c_int,
        24 as libc::c_int as u16,
        &mut (*palgo).VL53LX_p_047,
    );
    VL53LX_init_histogram_bin_data_struct(
        0 as libc::c_int,
        24 as libc::c_int as u16,
        &mut (*palgo).VL53LX_p_048,
    );
    VL53LX_init_histogram_bin_data_struct(
        0 as libc::c_int,
        24 as libc::c_int as u16,
        &mut (*palgo).VL53LX_p_049,
    );
    VL53LX_init_histogram_bin_data_struct(
        0 as libc::c_int,
        24 as libc::c_int as u16,
        &mut (*palgo).VL53LX_p_050,
    );
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_f_006(
    mut ambient_threshold_events_scaler: u16,
    mut ambient_threshold_sigma: i32,
    mut min_ambient_threshold_events: i32,
    mut algo__crosstalk_compensation_enable: u8,
    mut pbins: *mut VL53LX_histogram_bin_data_t,
    mut pxtalk: *mut VL53LX_histogram_bin_data_t,
    mut palgo: *mut VL53LX_hist_gen3_algo_private_data_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut lb: u8 = 0 as libc::c_int as u8;
    let mut VL53LX_p_001: u8 = 0 as libc::c_int as u8;
    let mut tmp: i64 = 0 as libc::c_int as i64;
    let mut amb_events: i32 = 0 as libc::c_int;
    let mut VL53LX_p_018: i32 = 0 as libc::c_int;
    let mut samples: i32 = 0 as libc::c_int;
    (*palgo).VL53LX_p_020 = (*pbins).VL53LX_p_020;
    (*palgo).VL53LX_p_019 = (*pbins).VL53LX_p_019;
    (*palgo).VL53LX_p_021 = (*pbins).VL53LX_p_021;
    (*palgo).VL53LX_p_028 = (*pbins).VL53LX_p_028;
    (*palgo).VL53LX_p_030 = VL53LX_decode_vcsel_period((*pbins).VL53LX_p_005);
    tmp = (*pbins).VL53LX_p_028 as i64;
    tmp *= ambient_threshold_events_scaler as i64;
    tmp += 2048 as libc::c_int as libc::c_long;
    tmp = tmp / 4096 as libc::c_int as libc::c_long;
    amb_events = tmp as i32;
    lb = 0 as libc::c_int as u8;
    while (lb as libc::c_int) < (*pbins).VL53LX_p_021 as libc::c_int {
        VL53LX_p_001 = (lb as libc::c_int >> 2 as libc::c_int) as u8;
        samples = (*pbins).bin_rep[VL53LX_p_001 as usize] as i32;
        if samples > 0 as libc::c_int {
            if (lb as libc::c_int) < (*pxtalk).VL53LX_p_021 as libc::c_int
                && algo__crosstalk_compensation_enable as libc::c_int > 0 as libc::c_int
            {
                VL53LX_p_018 = samples * (amb_events + (*pxtalk).bin_data[lb as usize]);
            } else {
                VL53LX_p_018 = samples * amb_events;
            }
            VL53LX_p_018 = VL53LX_isqrt(VL53LX_p_018 as u32) as i32;
            VL53LX_p_018 += samples / 2 as libc::c_int;
            VL53LX_p_018 /= samples;
            VL53LX_p_018 *= ambient_threshold_sigma;
            VL53LX_p_018 += 8 as libc::c_int;
            VL53LX_p_018 /= 16 as libc::c_int;
            VL53LX_p_018 += amb_events;
            if VL53LX_p_018 < min_ambient_threshold_events {
                VL53LX_p_018 = min_ambient_threshold_events;
            }
            (*palgo).VL53LX_p_052[lb as usize] = VL53LX_p_018;
            (*palgo).VL53LX_p_031 = VL53LX_p_018;
        }
        lb = lb.wrapping_add(1);
    }
    (*palgo).VL53LX_p_039 = 0 as libc::c_int as u8;
    lb = (*pbins).VL53LX_p_019;
    while (lb as libc::c_int) < (*pbins).VL53LX_p_021 as libc::c_int {
        if (*pbins).bin_data[lb as usize] > (*palgo).VL53LX_p_052[lb as usize] {
            (*palgo).VL53LX_p_040[lb as usize] = 1 as libc::c_int as u8;
            (*palgo).VL53LX_p_041[lb as usize] = 1 as libc::c_int as u8;
            let ref mut fresh78 = (*palgo).VL53LX_p_039;
            *fresh78 = (*fresh78).wrapping_add(1);
        } else {
            (*palgo).VL53LX_p_040[lb as usize] = 0 as libc::c_int as u8;
            (*palgo).VL53LX_p_041[lb as usize] = 0 as libc::c_int as u8;
        }
        lb = lb.wrapping_add(1);
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_f_007(
    mut palgo: *mut VL53LX_hist_gen3_algo_private_data_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut i: u8 = 0 as libc::c_int as u8;
    let mut j: u8 = 0 as libc::c_int as u8;
    let mut found: u8 = 0 as libc::c_int as u8;
    (*palgo).VL53LX_p_044 = 0 as libc::c_int as u8;
    i = 0 as libc::c_int as u8;
    while (i as libc::c_int) < (*palgo).VL53LX_p_030 as libc::c_int {
        j = ((i as libc::c_int + 1 as libc::c_int)
            % (*palgo).VL53LX_p_030 as libc::c_int) as u8;
        if (i as libc::c_int) < (*palgo).VL53LX_p_021 as libc::c_int
            && (j as libc::c_int) < (*palgo).VL53LX_p_021 as libc::c_int
        {
            if (*palgo).VL53LX_p_041[i as usize] as libc::c_int == 0 as libc::c_int
                && (*palgo).VL53LX_p_041[j as usize] as libc::c_int == 1 as libc::c_int
                && found as libc::c_int == 0 as libc::c_int
            {
                (*palgo).VL53LX_p_044 = i;
                found = 1 as libc::c_int as u8;
            }
        }
        i = i.wrapping_add(1);
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_f_008(
    mut palgo: *mut VL53LX_hist_gen3_algo_private_data_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut i: u8 = 0 as libc::c_int as u8;
    let mut j: u8 = 0 as libc::c_int as u8;
    let mut lb: u8 = 0 as libc::c_int as u8;
    lb = (*palgo).VL53LX_p_044;
    while (lb as libc::c_int)
        < (*palgo).VL53LX_p_044 as libc::c_int + (*palgo).VL53LX_p_030 as libc::c_int
    {
        i = (lb as libc::c_int % (*palgo).VL53LX_p_030 as libc::c_int) as u8;
        j = ((lb as libc::c_int + 1 as libc::c_int)
            % (*palgo).VL53LX_p_030 as libc::c_int) as u8;
        if (i as libc::c_int) < (*palgo).VL53LX_p_021 as libc::c_int
            && (j as libc::c_int) < (*palgo).VL53LX_p_021 as libc::c_int
        {
            if (*palgo).VL53LX_p_041[i as usize] as libc::c_int == 0 as libc::c_int
                && (*palgo).VL53LX_p_041[j as usize] as libc::c_int == 1 as libc::c_int
            {
                let ref mut fresh79 = (*palgo).VL53LX_p_046;
                *fresh79 = (*fresh79).wrapping_add(1);
            }
            if (*palgo).VL53LX_p_046 as libc::c_int
                > (*palgo).VL53LX_p_045 as libc::c_int
            {
                (*palgo).VL53LX_p_046 = (*palgo).VL53LX_p_045;
            }
            if (*palgo).VL53LX_p_041[i as usize] as libc::c_int > 0 as libc::c_int {
                (*palgo).VL53LX_p_042[i as usize] = (*palgo).VL53LX_p_046;
            } else {
                (*palgo).VL53LX_p_042[i as usize] = 0 as libc::c_int as u8;
            }
        }
        lb = lb.wrapping_add(1);
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_f_009(
    mut palgo: *mut VL53LX_hist_gen3_algo_private_data_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut i: u8 = 0 as libc::c_int as u8;
    let mut j: u8 = 0 as libc::c_int as u8;
    let mut blb: u8 = 0 as libc::c_int as u8;
    let mut pulse_no: u8 = 0 as libc::c_int as u8;
    let mut max_filter_half_width: u8 = 0 as libc::c_int as u8;
    let mut pdata: *mut VL53LX_hist_pulse_data_t = 0 as *mut VL53LX_hist_pulse_data_t;
    max_filter_half_width = ((*palgo).VL53LX_p_030 as libc::c_int - 1 as libc::c_int)
        as u8;
    max_filter_half_width = (max_filter_half_width as libc::c_int >> 1 as libc::c_int)
        as u8;
    blb = (*palgo).VL53LX_p_044;
    while (blb as libc::c_int)
        < (*palgo).VL53LX_p_044 as libc::c_int + (*palgo).VL53LX_p_030 as libc::c_int
    {
        i = (blb as libc::c_int % (*palgo).VL53LX_p_030 as libc::c_int) as u8;
        j = ((blb as libc::c_int + 1 as libc::c_int)
            % (*palgo).VL53LX_p_030 as libc::c_int) as u8;
        if (i as libc::c_int) < (*palgo).VL53LX_p_021 as libc::c_int
            && (j as libc::c_int) < (*palgo).VL53LX_p_021 as libc::c_int
        {
            if (*palgo).VL53LX_p_042[i as usize] as libc::c_int == 0 as libc::c_int
                && (*palgo).VL53LX_p_042[j as usize] as libc::c_int > 0 as libc::c_int
            {
                pulse_no = ((*palgo).VL53LX_p_042[j as usize] as libc::c_int
                    - 1 as libc::c_int) as u8;
                if (pulse_no as libc::c_int) < (*palgo).VL53LX_p_045 as libc::c_int {
                    pdata = &mut *((*palgo).VL53LX_p_003)
                        .as_mut_ptr()
                        .offset(pulse_no as isize) as *mut VL53LX_hist_pulse_data_t;
                    (*pdata).VL53LX_p_012 = blb;
                    (*pdata)
                        .VL53LX_p_019 = (blb as libc::c_int + 1 as libc::c_int)
                        as u8;
                    (*pdata).VL53LX_p_023 = 0xff as libc::c_int as u8;
                    (*pdata).VL53LX_p_024 = 0 as libc::c_int as u8;
                    (*pdata).VL53LX_p_013 = 0 as libc::c_int as u8;
                }
            }
            if (*palgo).VL53LX_p_042[i as usize] as libc::c_int > 0 as libc::c_int
                && (*palgo).VL53LX_p_042[j as usize] as libc::c_int == 0 as libc::c_int
            {
                pulse_no = ((*palgo).VL53LX_p_042[i as usize] as libc::c_int
                    - 1 as libc::c_int) as u8;
                if (pulse_no as libc::c_int) < (*palgo).VL53LX_p_045 as libc::c_int {
                    pdata = &mut *((*palgo).VL53LX_p_003)
                        .as_mut_ptr()
                        .offset(pulse_no as isize) as *mut VL53LX_hist_pulse_data_t;
                    (*pdata).VL53LX_p_024 = blb;
                    (*pdata)
                        .VL53LX_p_013 = (blb as libc::c_int + 1 as libc::c_int)
                        as u8;
                    (*pdata)
                        .VL53LX_p_025 = ((*pdata).VL53LX_p_024 as libc::c_int
                        + 1 as libc::c_int - (*pdata).VL53LX_p_019 as libc::c_int)
                        as u8;
                    (*pdata)
                        .VL53LX_p_051 = ((*pdata).VL53LX_p_013 as libc::c_int
                        + 1 as libc::c_int - (*pdata).VL53LX_p_012 as libc::c_int)
                        as u8;
                    if (*pdata).VL53LX_p_051 as libc::c_int
                        > max_filter_half_width as libc::c_int
                    {
                        (*pdata).VL53LX_p_051 = max_filter_half_width;
                    }
                }
            }
        }
        blb = blb.wrapping_add(1);
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_f_016(
    mut target_order: VL53LX_HistTargetOrder,
    mut palgo: *mut VL53LX_hist_gen3_algo_private_data_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut tmp: VL53LX_hist_pulse_data_t = VL53LX_hist_pulse_data_t {
        VL53LX_p_012: 0,
        VL53LX_p_019: 0,
        VL53LX_p_023: 0,
        VL53LX_p_024: 0,
        VL53LX_p_013: 0,
        VL53LX_p_025: 0,
        VL53LX_p_051: 0,
        VL53LX_p_016: 0,
        VL53LX_p_017: 0,
        VL53LX_p_010: 0,
        VL53LX_p_026: 0,
        VL53LX_p_011: 0,
        VL53LX_p_027: 0,
        VL53LX_p_002: 0,
    };
    let mut ptmp: *mut VL53LX_hist_pulse_data_t = &mut tmp;
    let mut p0: *mut VL53LX_hist_pulse_data_t = 0 as *mut VL53LX_hist_pulse_data_t;
    let mut p1: *mut VL53LX_hist_pulse_data_t = 0 as *mut VL53LX_hist_pulse_data_t;
    let mut i: u8 = 0 as libc::c_int as u8;
    let mut swapped: u8 = 1 as libc::c_int as u8;
    if (*palgo).VL53LX_p_046 as libc::c_int > 1 as libc::c_int {
        while swapped as libc::c_int > 0 as libc::c_int {
            swapped = 0 as libc::c_int as u8;
            i = 1 as libc::c_int as u8;
            while (i as libc::c_int) < (*palgo).VL53LX_p_046 as libc::c_int {
                p0 = &mut *((*palgo).VL53LX_p_003)
                    .as_mut_ptr()
                    .offset((i as libc::c_int - 1 as libc::c_int) as isize)
                    as *mut VL53LX_hist_pulse_data_t;
                p1 = &mut *((*palgo).VL53LX_p_003).as_mut_ptr().offset(i as isize)
                    as *mut VL53LX_hist_pulse_data_t;
                if target_order as libc::c_int
                    == 2 as libc::c_int as VL53LX_HistTargetOrder as libc::c_int
                {
                    if (*p0).VL53LX_p_010 < (*p1).VL53LX_p_010 {
                        memcpy(
                            ptmp as *mut libc::c_void,
                            p1 as *const libc::c_void,
                            ::std::mem::size_of::<VL53LX_hist_pulse_data_t>()
                                as libc::c_ulong,
                        );
                        memcpy(
                            p1 as *mut libc::c_void,
                            p0 as *const libc::c_void,
                            ::std::mem::size_of::<VL53LX_hist_pulse_data_t>()
                                as libc::c_ulong,
                        );
                        memcpy(
                            p0 as *mut libc::c_void,
                            ptmp as *const libc::c_void,
                            ::std::mem::size_of::<VL53LX_hist_pulse_data_t>()
                                as libc::c_ulong,
                        );
                        swapped = 1 as libc::c_int as u8;
                    }
                } else if (*p0).VL53LX_p_011 > (*p1).VL53LX_p_011 {
                    memcpy(
                        ptmp as *mut libc::c_void,
                        p1 as *const libc::c_void,
                        ::std::mem::size_of::<VL53LX_hist_pulse_data_t>()
                            as libc::c_ulong,
                    );
                    memcpy(
                        p1 as *mut libc::c_void,
                        p0 as *const libc::c_void,
                        ::std::mem::size_of::<VL53LX_hist_pulse_data_t>()
                            as libc::c_ulong,
                    );
                    memcpy(
                        p0 as *mut libc::c_void,
                        ptmp as *const libc::c_void,
                        ::std::mem::size_of::<VL53LX_hist_pulse_data_t>()
                            as libc::c_ulong,
                    );
                    swapped = 1 as libc::c_int as u8;
                }
                i = i.wrapping_add(1);
            }
        }
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_f_010(
    mut pulse_no: u8,
    mut pbins: *mut VL53LX_histogram_bin_data_t,
    mut palgo: *mut VL53LX_hist_gen3_algo_private_data_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut i: u8 = 0 as libc::c_int as u8;
    let mut lb: u8 = 0 as libc::c_int as u8;
    let mut pdata: *mut VL53LX_hist_pulse_data_t = &mut *((*palgo).VL53LX_p_003)
        .as_mut_ptr()
        .offset(pulse_no as isize) as *mut VL53LX_hist_pulse_data_t;
    (*pdata).VL53LX_p_017 = 0 as libc::c_int;
    (*pdata).VL53LX_p_016 = 0 as libc::c_int;
    lb = (*pdata).VL53LX_p_012;
    while lb as libc::c_int <= (*pdata).VL53LX_p_013 as libc::c_int {
        i = (lb as libc::c_int % (*palgo).VL53LX_p_030 as libc::c_int) as u8;
        let ref mut fresh80 = (*pdata).VL53LX_p_017;
        *fresh80 += (*pbins).bin_data[i as usize];
        let ref mut fresh81 = (*pdata).VL53LX_p_016;
        *fresh81 += (*palgo).VL53LX_p_028;
        lb = lb.wrapping_add(1);
    }
    (*pdata).VL53LX_p_010 = (*pdata).VL53LX_p_017 - (*pdata).VL53LX_p_016;
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_f_015(
    mut pulse_no: u8,
    mut clip_events: u8,
    mut pbins: *mut VL53LX_histogram_bin_data_t,
    mut palgo: *mut VL53LX_hist_gen3_algo_private_data_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut i: u8 = 0 as libc::c_int as u8;
    let mut VL53LX_p_012: i16 = 0 as libc::c_int as i16;
    let mut VL53LX_p_013: i16 = 0 as libc::c_int as i16;
    let mut window_width: i16 = 0 as libc::c_int as i16;
    let mut tmp_phase: u32 = 0 as libc::c_int as u32;
    let mut pdata: *mut VL53LX_hist_pulse_data_t = &mut *((*palgo).VL53LX_p_003)
        .as_mut_ptr()
        .offset(pulse_no as isize) as *mut VL53LX_hist_pulse_data_t;
    if (*pdata).VL53LX_p_023 as libc::c_int == 0xff as libc::c_int {
        (*pdata).VL53LX_p_023 = 1 as libc::c_int as u8;
    }
    i = ((*pdata).VL53LX_p_023 as libc::c_int % (*palgo).VL53LX_p_030 as libc::c_int)
        as u8;
    VL53LX_p_012 = i as i16;
    VL53LX_p_012 = (VL53LX_p_012 as libc::c_int
        + (*pdata).VL53LX_p_012 as i16 as libc::c_int) as i16;
    VL53LX_p_012 = (VL53LX_p_012 as libc::c_int
        - (*pdata).VL53LX_p_023 as i16 as libc::c_int) as i16;
    VL53LX_p_013 = i as i16;
    VL53LX_p_013 = (VL53LX_p_013 as libc::c_int
        + (*pdata).VL53LX_p_013 as i16 as libc::c_int) as i16;
    VL53LX_p_013 = (VL53LX_p_013 as libc::c_int
        - (*pdata).VL53LX_p_023 as i16 as libc::c_int) as i16;
    window_width = (VL53LX_p_013 as libc::c_int - VL53LX_p_012 as libc::c_int)
        as i16;
    if window_width as libc::c_int > 3 as libc::c_int {
        window_width = 3 as libc::c_int as i16;
    }
    status = VL53LX_f_020(
        VL53LX_p_012,
        (VL53LX_p_012 as libc::c_int + window_width as libc::c_int) as i16,
        (*palgo).VL53LX_p_030,
        clip_events,
        pbins,
        &mut (*pdata).VL53LX_p_026,
    );
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_f_020(
            (VL53LX_p_013 as libc::c_int - window_width as libc::c_int) as i16,
            VL53LX_p_013,
            (*palgo).VL53LX_p_030,
            clip_events,
            pbins,
            &mut (*pdata).VL53LX_p_027,
        );
    }
    if (*pdata).VL53LX_p_026 > (*pdata).VL53LX_p_027 {
        tmp_phase = (*pdata).VL53LX_p_026;
        (*pdata).VL53LX_p_026 = (*pdata).VL53LX_p_027;
        (*pdata).VL53LX_p_027 = tmp_phase;
    }
    if (*pdata).VL53LX_p_011 < (*pdata).VL53LX_p_026 {
        (*pdata).VL53LX_p_026 = (*pdata).VL53LX_p_011;
    }
    if (*pdata).VL53LX_p_011 > (*pdata).VL53LX_p_027 {
        (*pdata).VL53LX_p_027 = (*pdata).VL53LX_p_011;
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_f_020(
    mut VL53LX_p_019: i16,
    mut VL53LX_p_024: i16,
    mut VL53LX_p_030: u8,
    mut clip_events: u8,
    mut pbins: *mut VL53LX_histogram_bin_data_t,
    mut pphase: *mut u32,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut i: i16 = 0 as libc::c_int as i16;
    let mut lb: i16 = 0 as libc::c_int as i16;
    let mut VL53LX_p_018: i64 = 0 as libc::c_int as i64;
    let mut event_sum: i64 = 0 as libc::c_int as i64;
    let mut weighted_sum: i64 = 0 as libc::c_int as i64;
    *pphase = 0xffff as libc::c_int as u32;
    if VL53LX_p_030 as libc::c_int != 0 as libc::c_int {
        lb = VL53LX_p_019;
        while lb as libc::c_int <= VL53LX_p_024 as libc::c_int {
            if (lb as libc::c_int) < 0 as libc::c_int {
                i = (lb as libc::c_int + VL53LX_p_030 as i16 as libc::c_int)
                    as i16;
            } else {
                i = (lb as libc::c_int % VL53LX_p_030 as i16 as libc::c_int)
                    as i16;
            }
            if i as libc::c_int >= 0 as libc::c_int
                && (i as libc::c_int) < 24 as libc::c_int
            {
                VL53LX_p_018 = (*pbins).bin_data[i as usize] as i64
                    - (*pbins).VL53LX_p_028 as i64;
                if clip_events as libc::c_int > 0 as libc::c_int
                    && VL53LX_p_018 < 0 as libc::c_int as libc::c_long
                {
                    VL53LX_p_018 = 0 as libc::c_int as i64;
                }
                event_sum += VL53LX_p_018;
                weighted_sum
                    += VL53LX_p_018
                        * (1024 as libc::c_int as libc::c_long
                            + 2048 as libc::c_int as libc::c_long * lb as i64);
            }
            lb += 1;
        }
    }
    if event_sum > 0 as libc::c_int as libc::c_long {
        weighted_sum += event_sum / 2 as libc::c_int as libc::c_long;
        weighted_sum = weighted_sum / event_sum;
        if weighted_sum < 0 as libc::c_int as libc::c_long {
            weighted_sum = 0 as libc::c_int as i64;
        }
        *pphase = weighted_sum as u32;
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_f_011(
    mut pulse_no: u8,
    mut pbins: *mut VL53LX_histogram_bin_data_t,
    mut palgo: *mut VL53LX_hist_gen3_algo_private_data_t,
    mut pad_value: i32,
    mut ppulse: *mut VL53LX_histogram_bin_data_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut i: u8 = 0 as libc::c_int as u8;
    let mut lb: u8 = 0 as libc::c_int as u8;
    let mut pdata: *mut VL53LX_hist_pulse_data_t = &mut *((*palgo).VL53LX_p_003)
        .as_mut_ptr()
        .offset(pulse_no as isize) as *mut VL53LX_hist_pulse_data_t;
    memcpy(
        ppulse as *mut libc::c_void,
        pbins as *const libc::c_void,
        ::std::mem::size_of::<VL53LX_histogram_bin_data_t>() as libc::c_ulong,
    );
    lb = (*palgo).VL53LX_p_044;
    while (lb as libc::c_int)
        < (*palgo).VL53LX_p_044 as libc::c_int + (*palgo).VL53LX_p_030 as libc::c_int
    {
        if (lb as libc::c_int) < (*pdata).VL53LX_p_012 as libc::c_int
            || lb as libc::c_int > (*pdata).VL53LX_p_013 as libc::c_int
        {
            i = (lb as libc::c_int % (*palgo).VL53LX_p_030 as libc::c_int) as u8;
            if (i as libc::c_int) < (*ppulse).VL53LX_p_021 as libc::c_int {
                (*ppulse).bin_data[i as usize] = pad_value;
            }
        }
        lb = lb.wrapping_add(1);
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_f_014(
    mut bin: u8,
    mut sigma_estimator__sigma_ref_mm: u8,
    mut VL53LX_p_030: u8,
    mut VL53LX_p_051: u8,
    mut crosstalk_compensation_enable: u8,
    mut phist_data_ap: *mut VL53LX_histogram_bin_data_t,
    mut phist_data_zp: *mut VL53LX_histogram_bin_data_t,
    mut pxtalk_hist: *mut VL53LX_histogram_bin_data_t,
    mut psigma_est: *mut u16,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut func_status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut i: u8 = 0 as libc::c_int as u8;
    let mut VL53LX_p_007: i32 = 0 as libc::c_int;
    let mut VL53LX_p_032: i32 = 0 as libc::c_int;
    let mut VL53LX_p_001: i32 = 0 as libc::c_int;
    let mut a_zp: i32 = 0 as libc::c_int;
    let mut c_zp: i32 = 0 as libc::c_int;
    let mut ax: i32 = 0 as libc::c_int;
    let mut bx: i32 = 0 as libc::c_int;
    let mut cx: i32 = 0 as libc::c_int;
    if VL53LX_p_030 as libc::c_int == 0 as libc::c_int {
        *psigma_est = 0xffff as libc::c_int as u16;
        return -(15 as libc::c_int) as VL53LX_Error;
    }
    i = (bin as libc::c_int % VL53LX_p_030 as libc::c_int) as u8;
    VL53LX_f_022(
        i,
        VL53LX_p_051,
        phist_data_zp,
        &mut a_zp,
        &mut VL53LX_p_032,
        &mut c_zp,
    );
    VL53LX_f_022(
        i,
        VL53LX_p_051,
        phist_data_ap,
        &mut VL53LX_p_007,
        &mut VL53LX_p_032,
        &mut VL53LX_p_001,
    );
    if crosstalk_compensation_enable as libc::c_int > 0 as libc::c_int {
        VL53LX_f_022(i, VL53LX_p_051, pxtalk_hist, &mut ax, &mut bx, &mut cx);
    }
    func_status = VL53LX_f_023(
        sigma_estimator__sigma_ref_mm,
        VL53LX_p_007 as u32,
        VL53LX_p_032 as u32,
        VL53LX_p_001 as u32,
        a_zp as u32,
        c_zp as u32,
        bx as u32,
        ax as u32,
        cx as u32,
        (*phist_data_ap).VL53LX_p_028 as u32,
        (*phist_data_ap).VL53LX_p_015,
        psigma_est,
    );
    if func_status as libc::c_int == -(15 as libc::c_int) as VL53LX_Error as libc::c_int
    {
        *psigma_est = 0xffff as libc::c_int as u16;
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_f_017(
    mut range_id: u8,
    mut valid_phase_low: u8,
    mut valid_phase_high: u8,
    mut sigma_thres: u16,
    mut pbins: *mut VL53LX_histogram_bin_data_t,
    mut ppulse: *mut VL53LX_hist_pulse_data_t,
    mut pdata: *mut VL53LX_range_data_t,
) {
    let mut lower_phase_limit: u16 = 0 as libc::c_int as u16;
    let mut upper_phase_limit: u16 = 0 as libc::c_int as u16;
    (*pdata).range_id = range_id;
    (*pdata).time_stamp = 0 as libc::c_int as u32;
    (*pdata).VL53LX_p_012 = (*ppulse).VL53LX_p_012;
    (*pdata).VL53LX_p_019 = (*ppulse).VL53LX_p_019;
    (*pdata).VL53LX_p_023 = (*ppulse).VL53LX_p_023;
    (*pdata).VL53LX_p_024 = (*ppulse).VL53LX_p_024;
    (*pdata).VL53LX_p_013 = (*ppulse).VL53LX_p_013;
    (*pdata).VL53LX_p_025 = (*ppulse).VL53LX_p_025;
    (*pdata)
        .VL53LX_p_029 = ((*ppulse).VL53LX_p_013 as libc::c_int + 1 as libc::c_int
        - (*ppulse).VL53LX_p_012 as libc::c_int) as u8;
    (*pdata).zero_distance_phase = (*pbins).zero_distance_phase;
    (*pdata).VL53LX_p_002 = (*ppulse).VL53LX_p_002;
    (*pdata).VL53LX_p_026 = (*ppulse).VL53LX_p_026 as u16;
    (*pdata).VL53LX_p_011 = (*ppulse).VL53LX_p_011 as u16;
    (*pdata).VL53LX_p_027 = (*ppulse).VL53LX_p_027 as u16;
    (*pdata).VL53LX_p_017 = (*ppulse).VL53LX_p_017 as u32;
    (*pdata).VL53LX_p_010 = (*ppulse).VL53LX_p_010;
    (*pdata).VL53LX_p_016 = (*ppulse).VL53LX_p_016 as u32;
    (*pdata).total_periods_elapsed = (*pbins).total_periods_elapsed;
    (*pdata).range_status = 19 as libc::c_int as VL53LX_DeviceError;
    if sigma_thres as libc::c_int > 0 as libc::c_int
        && (*ppulse).VL53LX_p_002 as u32
            > (sigma_thres as u32) << 5 as libc::c_int
    {
        (*pdata).range_status = 6 as libc::c_int as VL53LX_DeviceError;
    }
    lower_phase_limit = ((valid_phase_low as libc::c_int) << 8 as libc::c_int)
        as u16;
    if (lower_phase_limit as libc::c_int) < (*pdata).zero_distance_phase as libc::c_int {
        lower_phase_limit = ((*pdata).zero_distance_phase as libc::c_int
            - lower_phase_limit as libc::c_int) as u16;
    } else {
        lower_phase_limit = 0 as libc::c_int as u16;
    }
    upper_phase_limit = ((valid_phase_high as libc::c_int) << 8 as libc::c_int)
        as u16;
    upper_phase_limit = (upper_phase_limit as libc::c_int
        + (*pbins).zero_distance_phase as libc::c_int) as u16;
    if ((*pdata).VL53LX_p_011 as libc::c_int) < lower_phase_limit as libc::c_int
        || (*pdata).VL53LX_p_011 as libc::c_int > upper_phase_limit as libc::c_int
    {
        (*pdata).range_status = 5 as libc::c_int as VL53LX_DeviceError;
    }
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_f_025(
    mut pdmax_cal: *mut VL53LX_dmax_calibration_data_t,
    mut pdmax_cfg: *mut VL53LX_hist_gen3_dmax_config_t,
    mut ppost_cfg: *mut VL53LX_hist_post_process_config_t,
    mut pbins_input: *mut VL53LX_histogram_bin_data_t,
    mut pxtalk: *mut VL53LX_histogram_bin_data_t,
    mut palgo3: *mut VL53LX_hist_gen3_algo_private_data_t,
    mut pfiltered: *mut VL53LX_hist_gen4_algo_filtered_data_t,
    mut pdmax_algo: *mut VL53LX_hist_gen3_dmax_private_data_t,
    mut presults: *mut VL53LX_range_results_t,
    mut histo_merge_nb: u8,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut ppulse_data: *mut VL53LX_hist_pulse_data_t = 0
        as *mut VL53LX_hist_pulse_data_t;
    let mut prange_data: *mut VL53LX_range_data_t = 0 as *mut VL53LX_range_data_t;
    let mut p: u8 = 0 as libc::c_int as u8;
    let mut pB: *mut VL53LX_histogram_bin_data_t = &mut (*palgo3).VL53LX_p_006;
    VL53LX_f_003(palgo3);
    memcpy(
        &mut (*palgo3).VL53LX_p_006 as *mut VL53LX_histogram_bin_data_t
            as *mut libc::c_void,
        pbins_input as *const libc::c_void,
        ::std::mem::size_of::<VL53LX_histogram_bin_data_t>() as libc::c_ulong,
    );
    (*presults).cfg_device_state = (*pbins_input).cfg_device_state;
    (*presults).rd_device_state = (*pbins_input).rd_device_state;
    (*presults).zone_id = (*pbins_input).zone_id;
    (*presults).stream_count = (*pbins_input).result__stream_count;
    (*presults).wrap_dmax_mm = 0 as libc::c_int as i16;
    (*presults).max_results = 4 as libc::c_int as u8;
    (*presults).active_results = 0 as libc::c_int as u8;
    p = 0 as libc::c_int as u8;
    while (p as libc::c_int) < 5 as libc::c_int {
        (*presults).VL53LX_p_022[p as usize] = 0 as libc::c_int as i16;
        p = p.wrapping_add(1);
    }
    VL53LX_hist_calc_zero_distance_phase(&mut (*palgo3).VL53LX_p_006);
    VL53LX_hist_estimate_ambient_from_thresholded_bins(
        (*ppost_cfg).ambient_thresh_sigma0 as i32,
        &mut (*palgo3).VL53LX_p_006,
    );
    VL53LX_hist_estimate_ambient_from_ambient_bins(&mut (*palgo3).VL53LX_p_006);
    VL53LX_hist_remove_ambient_bins(&mut (*palgo3).VL53LX_p_006);
    if (*ppost_cfg).algo__crosstalk_compensation_enable as libc::c_int > 0 as libc::c_int
    {
        VL53LX_f_005(pxtalk, &mut (*palgo3).VL53LX_p_006, &mut (*palgo3).VL53LX_p_047);
    }
    (*pdmax_cfg).ambient_thresh_sigma = (*ppost_cfg).ambient_thresh_sigma1;
    p = 0 as libc::c_int as u8;
    while (p as libc::c_int) < 5 as libc::c_int {
        if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
            status = VL53LX_f_001(
                (*pdmax_cfg).target_reflectance_for_dmax_calc[p as usize],
                pdmax_cal,
                pdmax_cfg,
                &mut (*palgo3).VL53LX_p_006,
                pdmax_algo,
                &mut *((*presults).VL53LX_p_022).as_mut_ptr().offset(p as isize),
            );
        }
        p = p.wrapping_add(1);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_f_006(
            (*ppost_cfg).ambient_thresh_events_scaler,
            (*pdmax_cfg).ambient_thresh_sigma as i32,
            (*ppost_cfg).min_ambient_thresh_events,
            (*ppost_cfg).algo__crosstalk_compensation_enable,
            &mut (*palgo3).VL53LX_p_006,
            &mut (*palgo3).VL53LX_p_047,
            palgo3,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_f_007(palgo3);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_f_008(palgo3);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_f_009(palgo3);
    }
    p = 0 as libc::c_int as u8;
    while (p as libc::c_int) < (*palgo3).VL53LX_p_046 as libc::c_int {
        ppulse_data = &mut *((*palgo3).VL53LX_p_003).as_mut_ptr().offset(p as isize)
            as *mut VL53LX_hist_pulse_data_t;
        if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
            status = VL53LX_f_010(p, &mut (*palgo3).VL53LX_p_006, palgo3);
        }
        if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
            status = VL53LX_f_011(
                p,
                &mut (*palgo3).VL53LX_p_006,
                palgo3,
                (*pB).VL53LX_p_028,
                &mut (*palgo3).VL53LX_p_048,
            );
        }
        if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
            status = VL53LX_f_011(
                p,
                &mut (*palgo3).VL53LX_p_006,
                palgo3,
                0 as libc::c_int,
                &mut (*palgo3).VL53LX_p_049,
            );
        }
        if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
            status = VL53LX_f_011(
                p,
                &mut (*palgo3).VL53LX_p_047,
                palgo3,
                0 as libc::c_int,
                &mut (*palgo3).VL53LX_p_050,
            );
        }
        if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
            status = VL53LX_f_026(p, &mut (*palgo3).VL53LX_p_048, palgo3, pfiltered);
        }
        if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
            status = VL53LX_f_027(p, (*ppost_cfg).noise_threshold, pfiltered, palgo3);
        }
        if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
            status = VL53LX_f_014(
                (*ppulse_data).VL53LX_p_023,
                (*ppost_cfg).sigma_estimator__sigma_ref_mm,
                (*palgo3).VL53LX_p_030,
                (*ppulse_data).VL53LX_p_051,
                (*ppost_cfg).algo__crosstalk_compensation_enable,
                &mut (*palgo3).VL53LX_p_048,
                &mut (*palgo3).VL53LX_p_049,
                &mut (*palgo3).VL53LX_p_050,
                &mut (*ppulse_data).VL53LX_p_002,
            );
        }
        if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
            status = VL53LX_f_015(
                p,
                1 as libc::c_int as u8,
                &mut (*palgo3).VL53LX_p_006,
                palgo3,
            );
        }
        p = p.wrapping_add(1);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_f_016((*ppost_cfg).hist_target_order, palgo3);
    }
    p = 0 as libc::c_int as u8;
    while (p as libc::c_int) < (*palgo3).VL53LX_p_046 as libc::c_int {
        ppulse_data = &mut *((*palgo3).VL53LX_p_003).as_mut_ptr().offset(p as isize)
            as *mut VL53LX_hist_pulse_data_t;
        if ((*presults).active_results as libc::c_int)
            < (*presults).max_results as libc::c_int
        {
            if (*ppulse_data).VL53LX_p_010 > (*ppost_cfg).signal_total_events_limit
                && ((*ppulse_data).VL53LX_p_023 as libc::c_int) < 0xff as libc::c_int
            {
                prange_data = &mut *((*presults).VL53LX_p_003)
                    .as_mut_ptr()
                    .offset((*presults).active_results as isize)
                    as *mut VL53LX_range_data_t;
                if status as libc::c_int
                    == 0 as libc::c_int as VL53LX_Error as libc::c_int
                {
                    VL53LX_f_017(
                        (*presults).active_results,
                        (*ppost_cfg).valid_phase_low,
                        (*ppost_cfg).valid_phase_high,
                        (*ppost_cfg).sigma_thresh,
                        &mut (*palgo3).VL53LX_p_006,
                        ppulse_data,
                        prange_data,
                    );
                }
                if status as libc::c_int
                    == 0 as libc::c_int as VL53LX_Error as libc::c_int
                {
                    status = VL53LX_f_018(
                        (*pB).vcsel_width,
                        (*pB).VL53LX_p_015,
                        (*pB).total_periods_elapsed,
                        (*pB).result__dss_actual_effective_spads,
                        prange_data,
                        histo_merge_nb,
                    );
                }
                if status as libc::c_int
                    == 0 as libc::c_int as VL53LX_Error as libc::c_int
                {
                    VL53LX_f_019(
                        (*ppost_cfg).gain_factor,
                        (*ppost_cfg).range_offset_mm,
                        prange_data,
                    );
                }
                let ref mut fresh82 = (*presults).active_results;
                *fresh82 = (*fresh82).wrapping_add(1);
            }
        }
        p = p.wrapping_add(1);
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_f_026(
    mut pulse_no: u8,
    mut ppulse: *mut VL53LX_histogram_bin_data_t,
    mut palgo3: *mut VL53LX_hist_gen3_algo_private_data_t,
    mut pfiltered: *mut VL53LX_hist_gen4_algo_filtered_data_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdata: *mut VL53LX_hist_pulse_data_t = &mut *((*palgo3).VL53LX_p_003)
        .as_mut_ptr()
        .offset(pulse_no as isize) as *mut VL53LX_hist_pulse_data_t;
    let mut lb: u8 = 0 as libc::c_int as u8;
    let mut i: u8 = 0 as libc::c_int as u8;
    let mut suma: i32 = 0 as libc::c_int;
    let mut sumb: i32 = 0 as libc::c_int;
    let mut sumc: i32 = 0 as libc::c_int;
    (*pfiltered).VL53LX_p_020 = (*palgo3).VL53LX_p_020;
    (*pfiltered).VL53LX_p_019 = (*palgo3).VL53LX_p_019;
    (*pfiltered).VL53LX_p_021 = (*palgo3).VL53LX_p_021;
    lb = (*pdata).VL53LX_p_012;
    while lb as libc::c_int <= (*pdata).VL53LX_p_013 as libc::c_int {
        i = (lb as libc::c_int % (*palgo3).VL53LX_p_030 as libc::c_int) as u8;
        VL53LX_f_022(i, (*pdata).VL53LX_p_051, ppulse, &mut suma, &mut sumb, &mut sumc);
        (*pfiltered).VL53LX_p_007[i as usize] = suma;
        (*pfiltered).VL53LX_p_032[i as usize] = sumb;
        (*pfiltered).VL53LX_p_001[i as usize] = sumc;
        (*pfiltered)
            .VL53LX_p_053[i as usize] = suma + sumb - (sumc + (*palgo3).VL53LX_p_028);
        (*pfiltered)
            .VL53LX_p_054[i as usize] = sumb + sumc - (suma + (*palgo3).VL53LX_p_028);
        lb = lb.wrapping_add(1);
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_f_027(
    mut pulse_no: u8,
    mut noise_threshold: u16,
    mut pfiltered: *mut VL53LX_hist_gen4_algo_filtered_data_t,
    mut palgo3: *mut VL53LX_hist_gen3_algo_private_data_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut func_status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdata: *mut VL53LX_hist_pulse_data_t = &mut *((*palgo3).VL53LX_p_003)
        .as_mut_ptr()
        .offset(pulse_no as isize) as *mut VL53LX_hist_pulse_data_t;
    let mut lb: u8 = 0 as libc::c_int as u8;
    let mut i: u8 = 0 as libc::c_int as u8;
    let mut j: u8 = 0 as libc::c_int as u8;
    lb = (*pdata).VL53LX_p_012;
    while (lb as libc::c_int) < (*pdata).VL53LX_p_013 as libc::c_int {
        i = (lb as libc::c_int % (*palgo3).VL53LX_p_030 as libc::c_int) as u8;
        j = ((lb as libc::c_int + 1 as libc::c_int)
            % (*palgo3).VL53LX_p_030 as libc::c_int) as u8;
        if (i as libc::c_int) < (*palgo3).VL53LX_p_021 as libc::c_int
            && (j as libc::c_int) < (*palgo3).VL53LX_p_021 as libc::c_int
        {
            if (*pfiltered).VL53LX_p_053[i as usize] == 0 as libc::c_int
                && (*pfiltered).VL53LX_p_054[i as usize] == 0 as libc::c_int
            {
                (*pfiltered).VL53LX_p_040[i as usize] = 0 as libc::c_int as u8;
            } else if (*pfiltered).VL53LX_p_053[i as usize] >= 0 as libc::c_int
                    && (*pfiltered).VL53LX_p_054[i as usize] >= 0 as libc::c_int
                {
                (*pfiltered).VL53LX_p_040[i as usize] = 1 as libc::c_int as u8;
            } else if (*pfiltered).VL53LX_p_053[i as usize] < 0 as libc::c_int
                    && (*pfiltered).VL53LX_p_054[i as usize] >= 0 as libc::c_int
                    && (*pfiltered).VL53LX_p_053[j as usize] >= 0 as libc::c_int
                    && (*pfiltered).VL53LX_p_054[j as usize] < 0 as libc::c_int
                {
                (*pfiltered).VL53LX_p_040[i as usize] = 1 as libc::c_int as u8;
            } else {
                (*pfiltered).VL53LX_p_040[i as usize] = 0 as libc::c_int as u8;
            }
            if (*pfiltered).VL53LX_p_040[i as usize] as libc::c_int > 0 as libc::c_int {
                (*pdata).VL53LX_p_023 = lb;
                func_status = VL53LX_f_028(
                    lb,
                    (*pfiltered).VL53LX_p_007[i as usize],
                    (*pfiltered).VL53LX_p_032[i as usize],
                    (*pfiltered).VL53LX_p_001[i as usize],
                    0 as libc::c_int,
                    0 as libc::c_int,
                    0 as libc::c_int,
                    (*palgo3).VL53LX_p_028,
                    (*palgo3).VL53LX_p_030,
                    &mut (*pdata).VL53LX_p_011,
                );
                if func_status as libc::c_int
                    == -(15 as libc::c_int) as VL53LX_Error as libc::c_int
                {
                    (*pfiltered).VL53LX_p_040[i as usize] = 0 as libc::c_int as u8;
                }
            }
        }
        lb = lb.wrapping_add(1);
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_f_028(
    mut bin: u8,
    mut VL53LX_p_007: i32,
    mut VL53LX_p_032: i32,
    mut VL53LX_p_001: i32,
    mut ax: i32,
    mut bx: i32,
    mut cx: i32,
    mut VL53LX_p_028: i32,
    mut VL53LX_p_030: u8,
    mut pmean_phase: *mut u32,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = -(15 as libc::c_int) as VL53LX_Error;
    let mut mean_phase: i64 = 0xffff as libc::c_int as i64;
    let mut mean_phase32: i32 = 0;
    let mut VL53LX_p_055: i64 = 0 as libc::c_int as i64;
    let mut half_b_minus_amb: i64 = 0 as libc::c_int as i64;
    VL53LX_p_055 = 4096 as libc::c_int as libc::c_long
        * (VL53LX_p_001 as i64 - cx as i64 - VL53LX_p_007 as i64
            - ax as i64);
    half_b_minus_amb = 4096 as libc::c_int as libc::c_long
        * (VL53LX_p_032 as i64 - bx as i64 - VL53LX_p_028 as i64);
    if half_b_minus_amb != 0 as libc::c_int as libc::c_long {
        mean_phase = 4096 as libc::c_int as libc::c_long * VL53LX_p_055
            + half_b_minus_amb;
        mean_phase = mean_phase / (half_b_minus_amb * 2 as libc::c_int as libc::c_long);
        mean_phase += 2048 as libc::c_int as libc::c_long;
        mean_phase += 4096 as libc::c_int as libc::c_long * bin as i64;
        mean_phase = (mean_phase + 1 as libc::c_int as libc::c_long)
            / 2 as libc::c_int as libc::c_long;
        if mean_phase < 0 as libc::c_int as libc::c_long {
            mean_phase = 0 as libc::c_int as i64;
        }
        if mean_phase > 0xffff as libc::c_int as libc::c_long {
            mean_phase = 0xffff as libc::c_int as i64;
        }
        mean_phase32 = mean_phase as i32;
        mean_phase32 = mean_phase32 % (VL53LX_p_030 as i32 * 2048 as libc::c_int);
        mean_phase = mean_phase32 as i64;
        status = 0 as libc::c_int as VL53LX_Error;
    }
    *pmean_phase = mean_phase as u32;
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_set_calib_config(
    mut Dev: VL53LX_DEV,
    mut vcsel_delay__a0: u8,
    mut calib_1: u8,
    mut calib_2: u8,
    mut calib_3: u8,
    mut calib_2__a0: u8,
    mut spad_readout: u8,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut comms_buffer: [u8; 3] = [0; 3];
    status = VL53LX_enable_powerforce(Dev);
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_disable_firmware(Dev);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_WrByte(Dev, 0xa1a as libc::c_int as u16, vcsel_delay__a0);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        comms_buffer[0 as libc::c_int as usize] = calib_1;
        comms_buffer[1 as libc::c_int as usize] = calib_2;
        comms_buffer[2 as libc::c_int as usize] = calib_3;
        status = VL53LX_WriteMulti(
            Dev,
            0x6c4 as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
            3 as libc::c_int as u32,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_WrByte(Dev, 0xa0a as libc::c_int as u16, calib_2__a0);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_WrByte(Dev, 0x6cf as libc::c_int as u16, spad_readout);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_enable_firmware(Dev);
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_set_hist_calib_pulse_delay(
    mut Dev: VL53LX_DEV,
    mut calib_delay: u8,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    status = VL53LX_set_calib_config(
        Dev,
        0x1 as libc::c_int as u8,
        calib_delay,
        0x4 as libc::c_int as u8,
        0x8 as libc::c_int as u8,
        0x14 as libc::c_int as u8,
        0xf5 as libc::c_int as u8,
    );
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_disable_calib_pulse_delay(
    mut Dev: VL53LX_DEV,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    status = VL53LX_set_calib_config(
        Dev,
        0 as libc::c_int as u8,
        0 as libc::c_int as u8,
        0 as libc::c_int as u8,
        0 as libc::c_int as u8,
        0 as libc::c_int as u8,
        0x45 as libc::c_int as u8,
    );
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_f_022(
    mut VL53LX_p_032: u8,
    mut filter_woi: u8,
    mut pbins: *mut VL53LX_histogram_bin_data_t,
    mut pa: *mut i32,
    mut pb: *mut i32,
    mut pc: *mut i32,
) {
    let mut w: u8 = 0 as libc::c_int as u8;
    let mut j: u8 = 0 as libc::c_int as u8;
    *pa = 0 as libc::c_int;
    *pb = (*pbins).bin_data[VL53LX_p_032 as usize];
    *pc = 0 as libc::c_int;
    w = 0 as libc::c_int as u8;
    while (w as libc::c_int)
        < ((filter_woi as libc::c_int) << 1 as libc::c_int) + 1 as libc::c_int
    {
        j = ((VL53LX_p_032 as libc::c_int + w as libc::c_int
            + (*pbins).VL53LX_p_021 as libc::c_int - filter_woi as libc::c_int)
            % (*pbins).VL53LX_p_021 as libc::c_int) as u8;
        if (w as libc::c_int) < filter_woi as libc::c_int {
            *pa += (*pbins).bin_data[j as usize];
        } else if w as libc::c_int > filter_woi as libc::c_int {
            *pc += (*pbins).bin_data[j as usize];
        }
        w = w.wrapping_add(1);
    }
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_f_018(
    mut vcsel_width: u16,
    mut fast_osc_frequency: u16,
    mut total_periods_elapsed: u32,
    mut VL53LX_p_004: u16,
    mut pdata: *mut VL53LX_range_data_t,
    mut histo_merge_nb: u8,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pll_period_us: u32 = 0 as libc::c_int as u32;
    let mut periods_elapsed: u32 = 0 as libc::c_int as u32;
    let mut count_rate_total: u32 = 0 as libc::c_int as u32;
    (*pdata).width = vcsel_width;
    (*pdata).fast_osc_frequency = fast_osc_frequency;
    (*pdata).total_periods_elapsed = total_periods_elapsed;
    (*pdata).VL53LX_p_004 = VL53LX_p_004;
    if (*pdata).fast_osc_frequency as libc::c_int == 0 as libc::c_int {
        status = -(15 as libc::c_int) as VL53LX_Error;
    }
    if (*pdata).total_periods_elapsed == 0 as libc::c_int as libc::c_uint {
        status = -(15 as libc::c_int) as VL53LX_Error;
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        pll_period_us = VL53LX_calc_pll_period_us((*pdata).fast_osc_frequency);
        periods_elapsed = ((*pdata).total_periods_elapsed)
            .wrapping_add(1 as libc::c_int as libc::c_uint);
        (*pdata)
            .peak_duration_us = VL53LX_duration_maths(
            pll_period_us,
            (*pdata).width as u32,
            2048 as libc::c_int as u32,
            periods_elapsed,
        );
        (*pdata)
            .woi_duration_us = VL53LX_duration_maths(
            pll_period_us,
            ((*pdata).VL53LX_p_029 as u32) << 4 as libc::c_int,
            2048 as libc::c_int as u32,
            periods_elapsed,
        );
        (*pdata)
            .peak_signal_count_rate_mcps = VL53LX_rate_maths(
            (*pdata).VL53LX_p_010,
            (*pdata).peak_duration_us,
        );
        (*pdata)
            .avg_signal_count_rate_mcps = VL53LX_rate_maths(
            (*pdata).VL53LX_p_010,
            (*pdata).woi_duration_us,
        );
        (*pdata)
            .ambient_count_rate_mcps = VL53LX_rate_maths(
            (*pdata).VL53LX_p_016 as i32,
            (*pdata).woi_duration_us,
        );
        count_rate_total = ((*pdata).peak_signal_count_rate_mcps as u32)
            .wrapping_add((*pdata).ambient_count_rate_mcps as u32);
        if histo_merge_nb as libc::c_int > 1 as libc::c_int {
            count_rate_total = (count_rate_total as libc::c_uint)
                .wrapping_div(histo_merge_nb as libc::c_uint) as u32 as u32;
        }
        (*pdata)
            .total_rate_per_spad_mcps = VL53LX_rate_per_spad_maths(
            0x6 as libc::c_int as u32,
            count_rate_total,
            (*pdata).VL53LX_p_004,
            0xffff as libc::c_int as u32,
        );
        (*pdata)
            .VL53LX_p_009 = VL53LX_events_per_spad_maths(
            (*pdata).VL53LX_p_010,
            (*pdata).VL53LX_p_004,
            (*pdata).peak_duration_us,
        );
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_f_019(
    mut gain_factor: u16,
    mut range_offset_mm: i16,
    mut pdata: *mut VL53LX_range_data_t,
) {
    (*pdata)
        .min_range_mm = VL53LX_range_maths(
        (*pdata).fast_osc_frequency,
        (*pdata).VL53LX_p_026,
        (*pdata).zero_distance_phase,
        0 as libc::c_int as u8,
        gain_factor as i32,
        range_offset_mm as i32,
    ) as i16;
    (*pdata)
        .median_range_mm = VL53LX_range_maths(
        (*pdata).fast_osc_frequency,
        (*pdata).VL53LX_p_011,
        (*pdata).zero_distance_phase,
        0 as libc::c_int as u8,
        gain_factor as i32,
        range_offset_mm as i32,
    ) as i16;
    (*pdata)
        .max_range_mm = VL53LX_range_maths(
        (*pdata).fast_osc_frequency,
        (*pdata).VL53LX_p_027,
        (*pdata).zero_distance_phase,
        0 as libc::c_int as u8,
        gain_factor as i32,
        range_offset_mm as i32,
    ) as i16;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_f_029(
    mut pdata: *mut VL53LX_histogram_bin_data_t,
    mut ambient_estimate_counts_per_bin: i32,
) {
    let mut i: u8 = 0 as libc::c_int as u8;
    i = 0 as libc::c_int as u8;
    while (i as libc::c_int) < (*pdata).VL53LX_p_021 as libc::c_int {
        (*pdata)
            .bin_data[i
            as usize] = (*pdata).bin_data[i as usize] - ambient_estimate_counts_per_bin;
        i = i.wrapping_add(1);
    }
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_f_005(
    mut pxtalk: *mut VL53LX_histogram_bin_data_t,
    mut pbins: *mut VL53LX_histogram_bin_data_t,
    mut pxtalk_realigned: *mut VL53LX_histogram_bin_data_t,
) {
    let mut i: u8 = 0 as libc::c_int as u8;
    let mut min_bins: u8 = 0 as libc::c_int as u8;
    let mut bin_offset: i8 = 0 as libc::c_int as i8;
    let mut bin_access: i8 = 0 as libc::c_int as i8;
    memcpy(
        pxtalk_realigned as *mut libc::c_void,
        pbins as *const libc::c_void,
        ::std::mem::size_of::<VL53LX_histogram_bin_data_t>() as libc::c_ulong,
    );
    i = 0 as libc::c_int as u8;
    while (i as libc::c_int) < (*pxtalk_realigned).VL53LX_p_020 as libc::c_int {
        (*pxtalk_realigned).bin_data[i as usize] = 0 as libc::c_int;
        i = i.wrapping_add(1);
    }
    bin_offset = VL53LX_f_030(pbins, pxtalk);
    if ((*pxtalk).VL53LX_p_021 as libc::c_int) < (*pbins).VL53LX_p_021 as libc::c_int {
        min_bins = (*pxtalk).VL53LX_p_021;
    } else {
        min_bins = (*pbins).VL53LX_p_021;
    }
    i = 0 as libc::c_int as u8;
    while (i as libc::c_int) < min_bins as libc::c_int {
        if bin_offset as libc::c_int >= 0 as libc::c_int {
            bin_access = ((i as i8 as libc::c_int + bin_offset as libc::c_int)
                % (*pbins).VL53LX_p_021 as i8 as libc::c_int) as i8;
        } else {
            bin_access = (((*pbins).VL53LX_p_021 as i8 as libc::c_int
                + (i as i8 as libc::c_int + bin_offset as libc::c_int))
                % (*pbins).VL53LX_p_021 as i8 as libc::c_int) as i8;
        }
        if (*pbins).bin_data[bin_access as u8 as usize]
            > (*pxtalk).bin_data[i as usize]
        {
            (*pbins)
                .bin_data[bin_access as u8
                as usize] = (*pbins).bin_data[bin_access as u8 as usize]
                - (*pxtalk).bin_data[i as usize];
        } else {
            (*pbins).bin_data[bin_access as u8 as usize] = 0 as libc::c_int;
        }
        (*pxtalk_realigned)
            .bin_data[bin_access as u8 as usize] = (*pxtalk).bin_data[i as usize];
        i = i.wrapping_add(1);
    }
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_f_030(
    mut pdata1: *mut VL53LX_histogram_bin_data_t,
    mut pdata2: *mut VL53LX_histogram_bin_data_t,
) -> i8 {
    let mut phase_delta: i32 = 0 as libc::c_int;
    let mut bin_offset: i8 = 0 as libc::c_int as i8;
    let mut period: u32 = 0 as libc::c_int as u32;
    let mut remapped_phase: u32 = 0 as libc::c_int as u32;
    period = (2048 as libc::c_int as libc::c_uint)
        .wrapping_mul(VL53LX_decode_vcsel_period((*pdata1).VL53LX_p_005) as u32);
    if period != 0 as libc::c_int as libc::c_uint {
        remapped_phase = ((*pdata2).zero_distance_phase as u32)
            .wrapping_rem(period);
    }
    phase_delta = (*pdata1).zero_distance_phase as i32 - remapped_phase as i32;
    if phase_delta > 0 as libc::c_int {
        bin_offset = ((phase_delta + 1024 as libc::c_int) / 2048 as libc::c_int)
            as i8;
    } else {
        bin_offset = ((phase_delta - 1024 as libc::c_int) / 2048 as libc::c_int)
            as i8;
    }
    return bin_offset;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_f_031(
    mut pidata: *mut VL53LX_histogram_bin_data_t,
    mut podata: *mut VL53LX_histogram_bin_data_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut bin_initial_index: [u8; 16] = [0; 16];
    let mut bin_repeat_count: [u8; 16] = [0; 16];
    let mut bin_cfg: u8 = 0 as libc::c_int as u8;
    let mut bin_seq_length: u8 = 0 as libc::c_int as u8;
    let mut repeat_count: i32 = 0 as libc::c_int;
    let mut VL53LX_p_032: u8 = 0 as libc::c_int as u8;
    let mut lc: u8 = 0 as libc::c_int as u8;
    let mut i: u8 = 0 as libc::c_int as u8;
    memcpy(
        podata as *mut libc::c_void,
        pidata as *const libc::c_void,
        ::std::mem::size_of::<VL53LX_histogram_bin_data_t>() as libc::c_ulong,
    );
    (*podata).VL53LX_p_021 = 0 as libc::c_int as u8;
    lc = 0 as libc::c_int as u8;
    while (lc as libc::c_int) < 6 as libc::c_int {
        (*podata)
            .bin_seq[lc as usize] = (15 as libc::c_int + 1 as libc::c_int) as u8;
        lc = lc.wrapping_add(1);
    }
    lc = 0 as libc::c_int as u8;
    while (lc as libc::c_int) < (*podata).VL53LX_p_020 as libc::c_int {
        (*podata).bin_data[lc as usize] = 0 as libc::c_int;
        lc = lc.wrapping_add(1);
    }
    lc = 0 as libc::c_int as u8;
    while lc as libc::c_int <= 15 as libc::c_int {
        bin_initial_index[lc as usize] = 0 as libc::c_int as u8;
        bin_repeat_count[lc as usize] = 0 as libc::c_int as u8;
        lc = lc.wrapping_add(1);
    }
    bin_seq_length = 0 as libc::c_int as u8;
    lc = 0 as libc::c_int as u8;
    while (lc as libc::c_int) < 6 as libc::c_int {
        bin_cfg = (*pidata).bin_seq[lc as usize];
        if bin_repeat_count[bin_cfg as usize] as libc::c_int == 0 as libc::c_int {
            bin_initial_index[bin_cfg
                as usize] = (bin_seq_length as libc::c_int * 4 as libc::c_int)
                as u8;
            (*podata).bin_seq[bin_seq_length as usize] = bin_cfg;
            bin_seq_length = bin_seq_length.wrapping_add(1);
        }
        bin_repeat_count[bin_cfg
            as usize] = (bin_repeat_count[bin_cfg as usize]).wrapping_add(1);
        VL53LX_p_032 = bin_initial_index[bin_cfg as usize];
        i = 0 as libc::c_int as u8;
        while (i as libc::c_int) < 4 as libc::c_int {
            let ref mut fresh83 = (*podata)
                .bin_data[(VL53LX_p_032 as libc::c_int + i as libc::c_int) as usize];
            *fresh83
                += (*pidata)
                    .bin_data[(lc as libc::c_int * 4 as libc::c_int + i as libc::c_int)
                    as usize];
            i = i.wrapping_add(1);
        }
        lc = lc.wrapping_add(1);
    }
    lc = 0 as libc::c_int as u8;
    while (lc as libc::c_int) < 6 as libc::c_int {
        bin_cfg = (*podata).bin_seq[lc as usize];
        if bin_cfg as libc::c_int <= 15 as libc::c_int {
            (*podata).bin_rep[lc as usize] = bin_repeat_count[bin_cfg as usize];
        } else {
            (*podata).bin_rep[lc as usize] = 0 as libc::c_int as u8;
        }
        lc = lc.wrapping_add(1);
    }
    (*podata)
        .VL53LX_p_021 = (bin_seq_length as libc::c_int * 4 as libc::c_int) as u8;
    lc = 0 as libc::c_int as u8;
    while lc as libc::c_int <= 15 as libc::c_int {
        repeat_count = bin_repeat_count[lc as usize] as i32;
        if repeat_count > 0 as libc::c_int {
            VL53LX_p_032 = bin_initial_index[lc as usize];
            i = 0 as libc::c_int as u8;
            while (i as libc::c_int) < 4 as libc::c_int {
                let ref mut fresh84 = (*podata)
                    .bin_data[(VL53LX_p_032 as libc::c_int + i as libc::c_int) as usize];
                *fresh84 += repeat_count / 2 as libc::c_int;
                let ref mut fresh85 = (*podata)
                    .bin_data[(VL53LX_p_032 as libc::c_int + i as libc::c_int) as usize];
                *fresh85 /= repeat_count;
                i = i.wrapping_add(1);
            }
        }
        lc = lc.wrapping_add(1);
    }
    (*podata).number_of_ambient_bins = 0 as libc::c_int as u8;
    if bin_repeat_count[7 as libc::c_int as usize] as libc::c_int > 0 as libc::c_int
        || bin_repeat_count[15 as libc::c_int as usize] as libc::c_int > 0 as libc::c_int
    {
        (*podata).number_of_ambient_bins = 4 as libc::c_int as u8;
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_hist_process_data(
    mut pdmax_cal: *mut VL53LX_dmax_calibration_data_t,
    mut pdmax_cfg: *mut VL53LX_hist_gen3_dmax_config_t,
    mut ppost_cfg: *mut VL53LX_hist_post_process_config_t,
    mut pbins_input: *mut VL53LX_histogram_bin_data_t,
    mut pxtalk_shape: *mut VL53LX_xtalk_histogram_data_t,
    mut pArea1: *mut u8,
    mut pArea2: *mut u8,
    mut presults: *mut VL53LX_range_results_t,
    mut HistMergeNumber: *mut u8,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut palgo_gen3: *mut VL53LX_hist_gen3_algo_private_data_t = pArea1
        as *mut VL53LX_hist_gen3_algo_private_data_t;
    let mut pfiltered4: *mut VL53LX_hist_gen4_algo_filtered_data_t = pArea2
        as *mut VL53LX_hist_gen4_algo_filtered_data_t;
    let mut dmax_algo_gen3: VL53LX_hist_gen3_dmax_private_data_t = VL53LX_hist_gen3_dmax_private_data_t {
        VL53LX_p_037: 0,
        VL53LX_p_063: 0,
        VL53LX_p_064: 0,
        VL53LX_p_065: 0,
        VL53LX_p_066: 0,
        VL53LX_p_067: 0,
        VL53LX_p_038: 0,
        VL53LX_p_009: 0,
        VL53LX_p_033: 0,
        VL53LX_p_034: 0,
        VL53LX_p_004: 0,
        VL53LX_p_028: 0,
        VL53LX_p_035: 0,
        VL53LX_p_036: 0,
        VL53LX_p_022: 0,
    };
    let mut pdmax_algo_gen3: *mut VL53LX_hist_gen3_dmax_private_data_t = &mut dmax_algo_gen3;
    let mut bins_averaged: VL53LX_histogram_bin_data_t = VL53LX_histogram_bin_data_t {
        cfg_device_state: 0,
        rd_device_state: 0,
        zone_id: 0,
        time_stamp: 0,
        VL53LX_p_019: 0,
        VL53LX_p_020: 0,
        VL53LX_p_021: 0,
        number_of_ambient_bins: 0,
        bin_seq: [0; 6],
        bin_rep: [0; 6],
        bin_data: [0; 24],
        result__interrupt_status: 0,
        result__range_status: 0,
        result__report_status: 0,
        result__stream_count: 0,
        result__dss_actual_effective_spads: 0,
        phasecal_result__reference_phase: 0,
        phasecal_result__vcsel_start: 0,
        cal_config__vcsel_start: 0,
        vcsel_width: 0,
        VL53LX_p_005: 0,
        VL53LX_p_015: 0,
        total_periods_elapsed: 0,
        peak_duration_us: 0,
        woi_duration_us: 0,
        min_bin_value: 0,
        max_bin_value: 0,
        zero_distance_phase: 0,
        number_of_ambient_samples: 0,
        ambient_events_sum: 0,
        VL53LX_p_028: 0,
        roi_config__user_roi_centre_spad: 0,
        roi_config__user_roi_requested_global_xy_size: 0,
    };
    let mut pbins_averaged: *mut VL53LX_histogram_bin_data_t = &mut bins_averaged;
    let mut pdata: *mut VL53LX_range_data_t = 0 as *mut VL53LX_range_data_t;
    let mut xtalk_rate_kcps: u32 = 0 as libc::c_int as u32;
    let mut max_xtalk_rate_per_spad_kcps: u32 = 0 as libc::c_int as u32;
    let mut xtalk_enable: u8 = 0 as libc::c_int as u8;
    let mut r: u8 = 0 as libc::c_int as u8;
    let mut t: u8 = 0 as libc::c_int as u8;
    let mut XtalkDetectMaxSigma: u32 = 0 as libc::c_int as u32;
    let mut delta_mm: i16 = 0 as libc::c_int as i16;
    VL53LX_f_031(pbins_input, pbins_averaged);
    VL53LX_init_histogram_bin_data_struct(
        0 as libc::c_int,
        (*pxtalk_shape).xtalk_shape.VL53LX_p_021 as u16,
        &mut (*pxtalk_shape).xtalk_hist_removed,
    );
    VL53LX_copy_xtalk_bin_data_to_histogram_data_struct(
        &mut (*pxtalk_shape).xtalk_shape,
        &mut (*pxtalk_shape).xtalk_hist_removed,
    );
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int
        && (*ppost_cfg).algo__crosstalk_compensation_enable as libc::c_int
            > 0 as libc::c_int
    {
        status = VL53LX_f_032(
            (*ppost_cfg).algo__crosstalk_compensation_plane_offset_kcps,
            (*ppost_cfg).algo__crosstalk_compensation_x_plane_gradient_kcps,
            (*ppost_cfg).algo__crosstalk_compensation_y_plane_gradient_kcps,
            0 as libc::c_int as i8,
            0 as libc::c_int as i8,
            (*pbins_input).result__dss_actual_effective_spads,
            (*pbins_input).roi_config__user_roi_centre_spad,
            (*pbins_input).roi_config__user_roi_requested_global_xy_size,
            &mut xtalk_rate_kcps,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int
        && (*ppost_cfg).algo__crosstalk_compensation_enable as libc::c_int
            > 0 as libc::c_int
    {
        status = VL53LX_f_033(
            pbins_averaged,
            &mut (*pxtalk_shape).xtalk_shape,
            xtalk_rate_kcps,
            &mut (*pxtalk_shape).xtalk_hist_removed,
        );
    }
    (*presults).xmonitor.total_periods_elapsed = (*pbins_averaged).total_periods_elapsed;
    (*presults)
        .xmonitor
        .VL53LX_p_004 = (*pbins_averaged).result__dss_actual_effective_spads;
    (*presults).xmonitor.peak_signal_count_rate_mcps = 0 as libc::c_int as u16;
    (*presults).xmonitor.VL53LX_p_009 = 0 as libc::c_int as u32;
    (*presults).xmonitor.range_id = 0 as libc::c_int as u8;
    (*presults).xmonitor.range_status = 0 as libc::c_int as VL53LX_DeviceError;
    xtalk_enable = 0 as libc::c_int as u8;
    if (*ppost_cfg).algo__crosstalk_compensation_enable as libc::c_int > 0 as libc::c_int
    {
        xtalk_enable = 1 as libc::c_int as u8;
    }
    r = 0 as libc::c_int as u8;
    while r as libc::c_int <= xtalk_enable as libc::c_int {
        (*ppost_cfg).algo__crosstalk_compensation_enable = r;
        status = VL53LX_f_025(
            pdmax_cal,
            pdmax_cfg,
            ppost_cfg,
            pbins_averaged,
            &mut (*pxtalk_shape).xtalk_hist_removed,
            palgo_gen3,
            pfiltered4,
            pdmax_algo_gen3,
            presults,
            *HistMergeNumber,
        );
        if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int
            && r as libc::c_int == 0 as libc::c_int
        {
            if (*presults).active_results as libc::c_int == 0 as libc::c_int {
                pdata = &mut *((*presults).VL53LX_p_003)
                    .as_mut_ptr()
                    .offset(0 as libc::c_int as isize) as *mut VL53LX_range_data_t;
                (*pdata).ambient_count_rate_mcps = (*pdmax_algo_gen3).VL53LX_p_034;
                (*pdata).VL53LX_p_004 = (*pdmax_algo_gen3).VL53LX_p_004;
            }
            max_xtalk_rate_per_spad_kcps = (*ppost_cfg)
                .algo__crosstalk_detect_max_valid_rate_kcps as u32;
            max_xtalk_rate_per_spad_kcps = (max_xtalk_rate_per_spad_kcps as libc::c_uint)
                .wrapping_mul(*HistMergeNumber as u32) as u32 as u32;
            max_xtalk_rate_per_spad_kcps <<= 4 as libc::c_int;
            t = 0 as libc::c_int as u8;
            while (t as libc::c_int) < (*presults).active_results as libc::c_int {
                pdata = &mut *((*presults).VL53LX_p_003).as_mut_ptr().offset(t as isize)
                    as *mut VL53LX_range_data_t;
                if (*pdata).max_range_mm as libc::c_int
                    > (*pdata).min_range_mm as libc::c_int
                {
                    delta_mm = ((*pdata).max_range_mm as libc::c_int
                        - (*pdata).min_range_mm as libc::c_int) as i16;
                } else {
                    delta_mm = ((*pdata).min_range_mm as libc::c_int
                        - (*pdata).max_range_mm as libc::c_int) as i16;
                }
                XtalkDetectMaxSigma = (*ppost_cfg).algo__crosstalk_detect_max_sigma_mm
                    as u32;
                XtalkDetectMaxSigma = (XtalkDetectMaxSigma as libc::c_uint)
                    .wrapping_mul(*HistMergeNumber as u32) as u32 as u32;
                XtalkDetectMaxSigma <<= 5 as libc::c_int;
                if (*pdata).median_range_mm as libc::c_int
                    > (*ppost_cfg).algo__crosstalk_detect_min_valid_range_mm
                        as libc::c_int
                    && ((*pdata).median_range_mm as libc::c_int)
                        < (*ppost_cfg).algo__crosstalk_detect_max_valid_range_mm
                            as libc::c_int
                    && (*pdata).VL53LX_p_009 < max_xtalk_rate_per_spad_kcps
                    && ((*pdata).VL53LX_p_002 as libc::c_uint) < XtalkDetectMaxSigma
                    && (delta_mm as libc::c_int)
                        < (*ppost_cfg).algo__crosstalk_detect_min_max_tolerance
                            as libc::c_int
                {
                    memcpy(
                        &mut (*presults).xmonitor as *mut VL53LX_range_data_t
                            as *mut libc::c_void,
                        pdata as *const libc::c_void,
                        ::std::mem::size_of::<VL53LX_range_data_t>() as libc::c_ulong,
                    );
                }
                t = t.wrapping_add(1);
            }
        }
        r = r.wrapping_add(1);
    }
    (*ppost_cfg).algo__crosstalk_compensation_enable = xtalk_enable;
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_hist_ambient_dmax(
    mut target_reflectance: u16,
    mut pdmax_cal: *mut VL53LX_dmax_calibration_data_t,
    mut pdmax_cfg: *mut VL53LX_hist_gen3_dmax_config_t,
    mut pbins: *mut VL53LX_histogram_bin_data_t,
    mut pambient_dmax_mm: *mut i16,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut dmax_algo: VL53LX_hist_gen3_dmax_private_data_t = VL53LX_hist_gen3_dmax_private_data_t {
        VL53LX_p_037: 0,
        VL53LX_p_063: 0,
        VL53LX_p_064: 0,
        VL53LX_p_065: 0,
        VL53LX_p_066: 0,
        VL53LX_p_067: 0,
        VL53LX_p_038: 0,
        VL53LX_p_009: 0,
        VL53LX_p_033: 0,
        VL53LX_p_034: 0,
        VL53LX_p_004: 0,
        VL53LX_p_028: 0,
        VL53LX_p_035: 0,
        VL53LX_p_036: 0,
        VL53LX_p_022: 0,
    };
    let mut pdmax_algo: *mut VL53LX_hist_gen3_dmax_private_data_t = &mut dmax_algo;
    status = VL53LX_f_001(
        target_reflectance,
        pdmax_cal,
        pdmax_cfg,
        pbins,
        pdmax_algo,
        pambient_dmax_mm,
    );
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_nvm_enable(
    mut Dev: VL53LX_DEV,
    mut nvm_ctrl_pulse_width: u16,
    mut nvm_power_up_delay_us: i32,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_disable_firmware(Dev);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_enable_powerforce(Dev);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_WaitUs(Dev, 250 as libc::c_int);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_WrByte(
            Dev,
            0x781 as libc::c_int as u16,
            0x1 as libc::c_int as u8,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_WrByte(
            Dev,
            0x683 as libc::c_int as u16,
            0x5 as libc::c_int as u8,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_WaitUs(Dev, nvm_power_up_delay_us);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_WrByte(
            Dev,
            0x780 as libc::c_int as u16,
            0x1 as libc::c_int as u8,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_WrWord(
            Dev,
            0x784 as libc::c_int as u16,
            nvm_ctrl_pulse_width,
        );
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_nvm_read(
    mut Dev: VL53LX_DEV,
    mut start_address: u8,
    mut count: u8,
    mut pdata: *mut u8,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut nvm_addr: u8 = 0 as libc::c_int as u8;
    nvm_addr = start_address;
    while (nvm_addr as libc::c_int) < start_address as libc::c_int + count as libc::c_int
    {
        if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
            status = VL53LX_WrByte(Dev, 0x794 as libc::c_int as u16, nvm_addr);
        }
        if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
            status = VL53LX_WrByte(
                Dev,
                0x783 as libc::c_int as u16,
                0 as libc::c_int as u8,
            );
        }
        if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
            status = VL53LX_WaitUs(Dev, 5 as libc::c_int);
        }
        if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
            status = VL53LX_WrByte(
                Dev,
                0x783 as libc::c_int as u16,
                0x1 as libc::c_int as u8,
            );
        }
        if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
            status = VL53LX_ReadMulti(
                Dev,
                0x790 as libc::c_int as u16,
                pdata,
                4 as libc::c_int as u32,
            );
        }
        pdata = pdata.offset(4 as libc::c_int as isize);
        nvm_addr = nvm_addr.wrapping_add(1);
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_nvm_disable(mut Dev: VL53LX_DEV) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_WrByte(
            Dev,
            0x783 as libc::c_int as u16,
            0x1 as libc::c_int as u8,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_WrByte(
            Dev,
            0x781 as libc::c_int as u16,
            0 as libc::c_int as u8,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_disable_powerforce(Dev);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_enable_firmware(Dev);
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_nvm_format_decode(
    mut buf_size: u16,
    mut pbuffer: *mut u8,
    mut pdata: *mut VL53LX_decoded_nvm_data_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut i: u8 = 0 as libc::c_int as u8;
    let mut ptmp: *mut u8 = 0 as *mut u8;
    let mut pptmp: [libc::c_int; 4] = [0; 4];
    if (buf_size as libc::c_int) < 512 as libc::c_int {
        return -(9 as libc::c_int) as VL53LX_Error;
    }
    (*pdata)
        .nvm__identification_model_id = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer.offset(0x8 as libc::c_int as isize),
        0xff as libc::c_int as u32,
        0 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u8;
    (*pdata)
        .nvm__identification_module_type = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer.offset(0xc as libc::c_int as isize),
        0xff as libc::c_int as u32,
        0 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u8;
    (*pdata)
        .nvm__identification_revision_id = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer.offset(0xd as libc::c_int as isize),
        0xf as libc::c_int as u32,
        0 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u8;
    (*pdata)
        .nvm__identification_module_id = VL53LX_i2c_decode_with_mask(
        2 as libc::c_int as u16,
        pbuffer.offset(0xe as libc::c_int as isize),
        0xffff as libc::c_int as u32,
        0 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u16;
    (*pdata)
        .nvm__i2c_valid = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer.offset(0x10 as libc::c_int as isize),
        0xff as libc::c_int as u32,
        0 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u8;
    (*pdata)
        .nvm__i2c_device_address_ews = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer.offset(0x11 as libc::c_int as isize),
        0xff as libc::c_int as u32,
        0 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u8;
    (*pdata)
        .nvm__ews__fast_osc_frequency = VL53LX_i2c_decode_with_mask(
        2 as libc::c_int as u16,
        pbuffer.offset(0x14 as libc::c_int as isize),
        0xffff as libc::c_int as u32,
        0 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u16;
    (*pdata)
        .nvm__ews__fast_osc_trim_max = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer.offset(0x16 as libc::c_int as isize),
        0x7f as libc::c_int as u32,
        0 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u8;
    (*pdata)
        .nvm__ews__fast_osc_freq_set = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer.offset(0x17 as libc::c_int as isize),
        0x7 as libc::c_int as u32,
        0 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u8;
    (*pdata)
        .nvm__ews__slow_osc_calibration = VL53LX_i2c_decode_with_mask(
        2 as libc::c_int as u16,
        pbuffer.offset(0x18 as libc::c_int as isize),
        0x3ff as libc::c_int as u32,
        0 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u16;
    (*pdata)
        .nvm__fmt__fast_osc_frequency = VL53LX_i2c_decode_with_mask(
        2 as libc::c_int as u16,
        pbuffer.offset(0x1c as libc::c_int as isize),
        0xffff as libc::c_int as u32,
        0 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u16;
    (*pdata)
        .nvm__fmt__fast_osc_trim_max = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer.offset(0x1e as libc::c_int as isize),
        0x7f as libc::c_int as u32,
        0 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u8;
    (*pdata)
        .nvm__fmt__fast_osc_freq_set = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer.offset(0x1f as libc::c_int as isize),
        0x7 as libc::c_int as u32,
        0 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u8;
    (*pdata)
        .nvm__fmt__slow_osc_calibration = VL53LX_i2c_decode_with_mask(
        2 as libc::c_int as u16,
        pbuffer.offset(0x20 as libc::c_int as isize),
        0x3ff as libc::c_int as u32,
        0 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u16;
    (*pdata)
        .nvm__vhv_config_unlock = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer.offset(0x28 as libc::c_int as isize),
        0xff as libc::c_int as u32,
        0 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u8;
    (*pdata)
        .nvm__ref_selvddpix = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer.offset(0x29 as libc::c_int as isize),
        0xf as libc::c_int as u32,
        0 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u8;
    (*pdata)
        .nvm__ref_selvquench = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer.offset(0x2a as libc::c_int as isize),
        0x78 as libc::c_int as u32,
        3 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u8;
    (*pdata)
        .nvm__regavdd1v2_sel = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer.offset(0x2b as libc::c_int as isize),
        0xc as libc::c_int as u32,
        2 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u8;
    (*pdata)
        .nvm__regdvdd1v2_sel = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer.offset(0x2b as libc::c_int as isize),
        0x3 as libc::c_int as u32,
        0 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u8;
    (*pdata)
        .nvm__vhv_timeout__macrop = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer.offset(0x2c as libc::c_int as isize),
        0x3 as libc::c_int as u32,
        0 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u8;
    (*pdata)
        .nvm__vhv_loop_bound = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer.offset(0x2c as libc::c_int as isize),
        0xfc as libc::c_int as u32,
        2 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u8;
    (*pdata)
        .nvm__vhv_count_threshold = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer.offset(0x2d as libc::c_int as isize),
        0xff as libc::c_int as u32,
        0 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u8;
    (*pdata)
        .nvm__vhv_offset = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer.offset(0x2e as libc::c_int as isize),
        0x3f as libc::c_int as u32,
        0 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u8;
    (*pdata)
        .nvm__vhv_init_enable = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer.offset(0x2f as libc::c_int as isize),
        0x80 as libc::c_int as u32,
        7 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u8;
    (*pdata)
        .nvm__vhv_init_value = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer.offset(0x2f as libc::c_int as isize),
        0x3f as libc::c_int as u32,
        0 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u8;
    (*pdata)
        .nvm__laser_safety_vcsel_trim_ll = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer.offset(0x30 as libc::c_int as isize),
        0x7 as libc::c_int as u32,
        0 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u8;
    (*pdata)
        .nvm__laser_safety_vcsel_selion_ll = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer.offset(0x31 as libc::c_int as isize),
        0x3f as libc::c_int as u32,
        0 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u8;
    (*pdata)
        .nvm__laser_safety_vcsel_selion_max_ll = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer.offset(0x32 as libc::c_int as isize),
        0x3f as libc::c_int as u32,
        0 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u8;
    (*pdata)
        .nvm__laser_safety_mult_ll = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer.offset(0x34 as libc::c_int as isize),
        0x3f as libc::c_int as u32,
        0 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u8;
    (*pdata)
        .nvm__laser_safety_clip_ll = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer.offset(0x35 as libc::c_int as isize),
        0x3f as libc::c_int as u32,
        0 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u8;
    (*pdata)
        .nvm__laser_safety_vcsel_trim_ld = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer.offset(0x38 as libc::c_int as isize),
        0x7 as libc::c_int as u32,
        0 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u8;
    (*pdata)
        .nvm__laser_safety_vcsel_selion_ld = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer.offset(0x39 as libc::c_int as isize),
        0x3f as libc::c_int as u32,
        0 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u8;
    (*pdata)
        .nvm__laser_safety_vcsel_selion_max_ld = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer.offset(0x3a as libc::c_int as isize),
        0x3f as libc::c_int as u32,
        0 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u8;
    (*pdata)
        .nvm__laser_safety_mult_ld = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer.offset(0x3c as libc::c_int as isize),
        0x3f as libc::c_int as u32,
        0 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u8;
    (*pdata)
        .nvm__laser_safety_clip_ld = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer.offset(0x3d as libc::c_int as isize),
        0x3f as libc::c_int as u32,
        0 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u8;
    (*pdata)
        .nvm__laser_safety_lock_byte = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer.offset(0x40 as libc::c_int as isize),
        0xff as libc::c_int as u32,
        0 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u8;
    (*pdata)
        .nvm__laser_safety_unlock_byte = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer.offset(0x44 as libc::c_int as isize),
        0xff as libc::c_int as u32,
        0 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u8;
    ptmp = pbuffer.offset(0x48 as libc::c_int as isize);
    i = 0 as libc::c_int as u8;
    while (i as libc::c_int) < 32 as libc::c_int {
        let fresh86 = ptmp;
        ptmp = ptmp.offset(1);
        (*pdata).nvm__ews__spad_enables_rtn[i as usize] = *fresh86;
        i = i.wrapping_add(1);
    }
    ptmp = pbuffer.offset(0x68 as libc::c_int as isize);
    i = 0 as libc::c_int as u8;
    while (i as libc::c_int) < 6 as libc::c_int {
        let fresh87 = ptmp;
        ptmp = ptmp.offset(1);
        (*pdata).nvm__ews__spad_enables_ref__loc1[i as usize] = *fresh87;
        i = i.wrapping_add(1);
    }
    ptmp = pbuffer.offset(0x70 as libc::c_int as isize);
    i = 0 as libc::c_int as u8;
    while (i as libc::c_int) < 6 as libc::c_int {
        let fresh88 = ptmp;
        ptmp = ptmp.offset(1);
        (*pdata).nvm__ews__spad_enables_ref__loc2[i as usize] = *fresh88;
        i = i.wrapping_add(1);
    }
    ptmp = pbuffer.offset(0x78 as libc::c_int as isize);
    i = 0 as libc::c_int as u8;
    while (i as libc::c_int) < 6 as libc::c_int {
        let fresh89 = ptmp;
        ptmp = ptmp.offset(1);
        (*pdata).nvm__ews__spad_enables_ref__loc3[i as usize] = *fresh89;
        i = i.wrapping_add(1);
    }
    ptmp = pbuffer.offset(0x80 as libc::c_int as isize);
    i = 0 as libc::c_int as u8;
    while (i as libc::c_int) < 32 as libc::c_int {
        let fresh90 = ptmp;
        ptmp = ptmp.offset(1);
        (*pdata).nvm__fmt__spad_enables_rtn[i as usize] = *fresh90;
        i = i.wrapping_add(1);
    }
    ptmp = pbuffer.offset(0xa0 as libc::c_int as isize);
    i = 0 as libc::c_int as u8;
    while (i as libc::c_int) < 6 as libc::c_int {
        let fresh91 = ptmp;
        ptmp = ptmp.offset(1);
        (*pdata).nvm__fmt__spad_enables_ref__loc1[i as usize] = *fresh91;
        i = i.wrapping_add(1);
    }
    ptmp = pbuffer.offset(0xa8 as libc::c_int as isize);
    i = 0 as libc::c_int as u8;
    while (i as libc::c_int) < 6 as libc::c_int {
        let fresh92 = ptmp;
        ptmp = ptmp.offset(1);
        (*pdata).nvm__fmt__spad_enables_ref__loc2[i as usize] = *fresh92;
        i = i.wrapping_add(1);
    }
    ptmp = pbuffer.offset(0xb0 as libc::c_int as isize);
    i = 0 as libc::c_int as u8;
    while (i as libc::c_int) < 6 as libc::c_int {
        let fresh93 = ptmp;
        ptmp = ptmp.offset(1);
        (*pdata).nvm__fmt__spad_enables_ref__loc3[i as usize] = *fresh93;
        i = i.wrapping_add(1);
    }
    (*pdata)
        .nvm__fmt__roi_config__mode_roi_centre_spad = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer.offset(0xb8 as libc::c_int as isize),
        0xff as libc::c_int as u32,
        0 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u8;
    (*pdata)
        .nvm__fmt__roi_config__mode_roi_x_size = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer.offset(0xb9 as libc::c_int as isize),
        0xf0 as libc::c_int as u32,
        4 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u8;
    (*pdata)
        .nvm__fmt__roi_config__mode_roi_y_size = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer.offset(0xb9 as libc::c_int as isize),
        0xf as libc::c_int as u32,
        0 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u8;
    (*pdata)
        .nvm__fmt__ref_spad_apply__num_requested_ref_spad = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer.offset(0xbc as libc::c_int as isize),
        0xff as libc::c_int as u32,
        0 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u8;
    (*pdata)
        .nvm__fmt__ref_spad_man__ref_location = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer.offset(0xbd as libc::c_int as isize),
        0x3 as libc::c_int as u32,
        0 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u8;
    (*pdata)
        .nvm__fmt__mm_config__inner_offset_mm = VL53LX_i2c_decode_with_mask(
        2 as libc::c_int as u16,
        pbuffer.offset(0xc0 as libc::c_int as isize),
        0xffff as libc::c_int as u32,
        0 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u16;
    (*pdata)
        .nvm__fmt__mm_config__outer_offset_mm = VL53LX_i2c_decode_with_mask(
        2 as libc::c_int as u16,
        pbuffer.offset(0xc2 as libc::c_int as isize),
        0xffff as libc::c_int as u32,
        0 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u16;
    (*pdata)
        .nvm__fmt__algo_part_to_part_range_offset_mm = VL53LX_i2c_decode_with_mask(
        2 as libc::c_int as u16,
        pbuffer.offset(0xc4 as libc::c_int as isize),
        0xfff as libc::c_int as u32,
        0 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u16;
    (*pdata)
        .nvm__fmt__algo__crosstalk_compensation_plane_offset_kcps = VL53LX_i2c_decode_with_mask(
        2 as libc::c_int as u16,
        pbuffer.offset(0xc8 as libc::c_int as isize),
        0xffff as libc::c_int as u32,
        0 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u16;
    (*pdata)
        .nvm__fmt__algo__crosstalk_compensation_x_plane_gradient_kcps = VL53LX_i2c_decode_with_mask(
        2 as libc::c_int as u16,
        pbuffer.offset(0xca as libc::c_int as isize),
        0xffff as libc::c_int as u32,
        0 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u16;
    (*pdata)
        .nvm__fmt__algo__crosstalk_compensation_y_plane_gradient_kcps = VL53LX_i2c_decode_with_mask(
        2 as libc::c_int as u16,
        pbuffer.offset(0xcc as libc::c_int as isize),
        0xffff as libc::c_int as u32,
        0 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u16;
    (*pdata)
        .nvm__fmt__spare__host_config__nvm_config_spare_0 = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer.offset(0xce as libc::c_int as isize),
        0xff as libc::c_int as u32,
        0 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u8;
    (*pdata)
        .nvm__fmt__spare__host_config__nvm_config_spare_1 = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer.offset(0xcf as libc::c_int as isize),
        0xff as libc::c_int as u32,
        0 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u8;
    (*pdata)
        .nvm__customer_space_programmed = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer.offset(0xe0 as libc::c_int as isize),
        0xff as libc::c_int as u32,
        0 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u8;
    (*pdata)
        .nvm__cust__i2c_device_address = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer.offset(0xe4 as libc::c_int as isize),
        0xff as libc::c_int as u32,
        0 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u8;
    (*pdata)
        .nvm__cust__ref_spad_apply__num_requested_ref_spad = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer.offset(0xe8 as libc::c_int as isize),
        0xff as libc::c_int as u32,
        0 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u8;
    (*pdata)
        .nvm__cust__ref_spad_man__ref_location = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer.offset(0xe9 as libc::c_int as isize),
        0x3 as libc::c_int as u32,
        0 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u8;
    (*pdata)
        .nvm__cust__mm_config__inner_offset_mm = VL53LX_i2c_decode_with_mask(
        2 as libc::c_int as u16,
        pbuffer.offset(0xec as libc::c_int as isize),
        0xffff as libc::c_int as u32,
        0 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u16;
    (*pdata)
        .nvm__cust__mm_config__outer_offset_mm = VL53LX_i2c_decode_with_mask(
        2 as libc::c_int as u16,
        pbuffer.offset(0xee as libc::c_int as isize),
        0xffff as libc::c_int as u32,
        0 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u16;
    (*pdata)
        .nvm__cust__algo_part_to_part_range_offset_mm = VL53LX_i2c_decode_with_mask(
        2 as libc::c_int as u16,
        pbuffer.offset(0xf0 as libc::c_int as isize),
        0xfff as libc::c_int as u32,
        0 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u16;
    (*pdata)
        .nvm__cust__algo__crosstalk_compensation_plane_offset_kcps = VL53LX_i2c_decode_with_mask(
        2 as libc::c_int as u16,
        pbuffer.offset(0xf4 as libc::c_int as isize),
        0xffff as libc::c_int as u32,
        0 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u16;
    (*pdata)
        .nvm__cust__algo__crosstalk_compensation_x_plane_gradient_kcps = VL53LX_i2c_decode_with_mask(
        2 as libc::c_int as u16,
        pbuffer.offset(0xf6 as libc::c_int as isize),
        0xffff as libc::c_int as u32,
        0 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u16;
    (*pdata)
        .nvm__cust__algo__crosstalk_compensation_y_plane_gradient_kcps = VL53LX_i2c_decode_with_mask(
        2 as libc::c_int as u16,
        pbuffer.offset(0xf8 as libc::c_int as isize),
        0xffff as libc::c_int as u32,
        0 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u16;
    (*pdata)
        .nvm__cust__spare__host_config__nvm_config_spare_0 = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer.offset(0xfa as libc::c_int as isize),
        0xff as libc::c_int as u32,
        0 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u8;
    (*pdata)
        .nvm__cust__spare__host_config__nvm_config_spare_1 = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer.offset(0xfb as libc::c_int as isize),
        0xff as libc::c_int as u32,
        0 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u8;
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_nvm_decode_optical_centre(
            buf_size,
            pbuffer.offset(0xb8 as libc::c_int as isize),
            &mut (*pdata).fmt_optical_centre,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_nvm_decode_cal_peak_rate_map(
            buf_size,
            pbuffer.offset(0x15c as libc::c_int as isize),
            &mut (*pdata).fmt_peak_rate_map,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_nvm_decode_additional_offset_cal_data(
            buf_size,
            pbuffer.offset(0x194 as libc::c_int as isize),
            &mut (*pdata).fmt_add_offset_data,
        );
    }
    pptmp[0 as libc::c_int as usize] = 0x19c as libc::c_int;
    pptmp[1 as libc::c_int as usize] = 0x1ac as libc::c_int;
    pptmp[2 as libc::c_int as usize] = 0x1bc as libc::c_int;
    pptmp[3 as libc::c_int as usize] = 0x1cc as libc::c_int;
    i = 0 as libc::c_int as u8;
    while (i as libc::c_int) < 4 as libc::c_int {
        if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
            status = VL53LX_nvm_decode_fmt_range_results_data(
                buf_size,
                pbuffer.offset(pptmp[i as usize] as isize),
                &mut *((*pdata).fmt_range_data).as_mut_ptr().offset(i as isize),
            );
        }
        i = i.wrapping_add(1);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_nvm_decode_fmt_info(buf_size, pbuffer, &mut (*pdata).fmt_info);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_nvm_decode_ews_info(buf_size, pbuffer, &mut (*pdata).ews_info);
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_nvm_decode_optical_centre(
    mut buf_size: u16,
    mut pbuffer: *mut u8,
    mut pdata: *mut VL53LX_optical_centre_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut tmp: u16 = 0 as libc::c_int as u16;
    if (buf_size as libc::c_int) < 4 as libc::c_int {
        return -(9 as libc::c_int) as VL53LX_Error;
    }
    tmp = 0x100 as libc::c_int as u16;
    tmp = (tmp as libc::c_int
        - *pbuffer.offset(2 as libc::c_int as isize) as u16 as libc::c_int)
        as u16;
    if tmp as libc::c_int > 0xff as libc::c_int {
        tmp = 0 as libc::c_int as u16;
    }
    (*pdata).x_centre = tmp as u8;
    (*pdata).y_centre = *pbuffer.offset(3 as libc::c_int as isize);
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_nvm_decode_cal_peak_rate_map(
    mut buf_size: u16,
    mut pbuffer: *mut u8,
    mut pdata: *mut VL53LX_cal_peak_rate_map_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut ptmp: *mut u8 = 0 as *mut u8;
    let mut i: u8 = 0 as libc::c_int as u8;
    if (buf_size as libc::c_int) < 56 as libc::c_int {
        return -(9 as libc::c_int) as VL53LX_Error;
    }
    (*pdata)
        .cal_distance_mm = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer,
    ) as i16;
    (*pdata)
        .cal_reflectance_pc = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(2 as libc::c_int as isize),
    );
    (*pdata)
        .cal_reflectance_pc = ((*pdata).cal_reflectance_pc as libc::c_int
        >> 6 as libc::c_int) as u16;
    (*pdata).max_samples = 25 as libc::c_int as u16;
    (*pdata).width = 5 as libc::c_int as u16;
    (*pdata).height = 5 as libc::c_int as u16;
    ptmp = pbuffer.offset(4 as libc::c_int as isize);
    i = 0 as libc::c_int as u8;
    while (i as libc::c_int) < 25 as libc::c_int {
        (*pdata)
            .peak_rate_mcps[i
            as usize] = VL53LX_i2c_decode_uint16_t(2 as libc::c_int as u16, ptmp);
        ptmp = ptmp.offset(2 as libc::c_int as isize);
        i = i.wrapping_add(1);
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_nvm_decode_additional_offset_cal_data(
    mut buf_size: u16,
    mut pbuffer: *mut u8,
    mut pdata: *mut VL53LX_additional_offset_cal_data_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    if (buf_size as libc::c_int) < 8 as libc::c_int {
        return -(9 as libc::c_int) as VL53LX_Error;
    }
    (*pdata)
        .result__mm_inner_actual_effective_spads = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer,
    );
    (*pdata)
        .result__mm_outer_actual_effective_spads = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(2 as libc::c_int as isize),
    );
    (*pdata)
        .result__mm_inner_peak_signal_count_rtn_mcps = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(4 as libc::c_int as isize),
    );
    (*pdata)
        .result__mm_outer_peak_signal_count_rtn_mcps = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(6 as libc::c_int as isize),
    );
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_nvm_decode_fmt_range_results_data(
    mut buf_size: u16,
    mut pbuffer: *mut u8,
    mut pdata: *mut VL53LX_decoded_nvm_fmt_range_data_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    if (buf_size as libc::c_int) < 16 as libc::c_int {
        return -(9 as libc::c_int) as VL53LX_Error;
    }
    (*pdata)
        .result__actual_effective_rtn_spads = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer,
    );
    (*pdata)
        .ref_spad_array__num_requested_ref_spads = *pbuffer
        .offset(2 as libc::c_int as isize);
    (*pdata).ref_spad_array__ref_location = *pbuffer.offset(3 as libc::c_int as isize);
    (*pdata)
        .result__peak_signal_count_rate_rtn_mcps = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(4 as libc::c_int as isize),
    );
    (*pdata)
        .result__ambient_count_rate_rtn_mcps = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(6 as libc::c_int as isize),
    );
    (*pdata)
        .result__peak_signal_count_rate_ref_mcps = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(8 as libc::c_int as isize),
    );
    (*pdata)
        .result__ambient_count_rate_ref_mcps = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(10 as libc::c_int as isize),
    );
    (*pdata)
        .measured_distance_mm = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(12 as libc::c_int as isize),
    );
    (*pdata)
        .measured_distance_stdev_mm = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(14 as libc::c_int as isize),
    );
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_nvm_decode_fmt_info(
    mut buf_size: u16,
    mut pbuffer: *mut u8,
    mut pdata: *mut VL53LX_decoded_nvm_fmt_info_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    if (buf_size as libc::c_int) < 512 as libc::c_int {
        return -(9 as libc::c_int) as VL53LX_Error;
    }
    (*pdata)
        .nvm__fmt__fgc[0 as libc::c_int
        as usize] = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer.offset(0x1dc as libc::c_int as isize),
        0xfe as libc::c_int as u32,
        1 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as libc::c_char;
    (*pdata)
        .nvm__fmt__fgc[1 as libc::c_int
        as usize] = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer.offset(0x1dd as libc::c_int as isize),
        0x1fc as libc::c_int as u32,
        2 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as libc::c_char;
    (*pdata)
        .nvm__fmt__fgc[2 as libc::c_int
        as usize] = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer
            .offset(0x1de as libc::c_int as isize)
            .offset(-(1 as libc::c_int as isize)),
        0x3f8 as libc::c_int as u32,
        3 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as libc::c_char;
    (*pdata)
        .nvm__fmt__fgc[3 as libc::c_int
        as usize] = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer
            .offset(0x1df as libc::c_int as isize)
            .offset(-(1 as libc::c_int as isize)),
        0x7f0 as libc::c_int as u32,
        4 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as libc::c_char;
    (*pdata)
        .nvm__fmt__fgc[4 as libc::c_int
        as usize] = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer
            .offset(0x1e0 as libc::c_int as isize)
            .offset(-(1 as libc::c_int as isize)),
        0xfe0 as libc::c_int as u32,
        5 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as libc::c_char;
    (*pdata)
        .nvm__fmt__fgc[5 as libc::c_int
        as usize] = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer
            .offset(0x1e1 as libc::c_int as isize)
            .offset(-(1 as libc::c_int as isize)),
        0x1fc0 as libc::c_int as u32,
        6 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as libc::c_char;
    (*pdata)
        .nvm__fmt__fgc[6 as libc::c_int
        as usize] = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer
            .offset(0x1e2 as libc::c_int as isize)
            .offset(-(1 as libc::c_int as isize)),
        0x3f80 as libc::c_int as u32,
        7 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as libc::c_char;
    (*pdata)
        .nvm__fmt__fgc[7 as libc::c_int
        as usize] = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer.offset(0x1e2 as libc::c_int as isize),
        0x7f as libc::c_int as u32,
        0 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as libc::c_char;
    (*pdata)
        .nvm__fmt__fgc[8 as libc::c_int
        as usize] = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer.offset(0x1e3 as libc::c_int as isize),
        0xfe as libc::c_int as u32,
        1 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as libc::c_char;
    (*pdata)
        .nvm__fmt__fgc[9 as libc::c_int
        as usize] = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer.offset(0x1e4 as libc::c_int as isize),
        0x1fc as libc::c_int as u32,
        2 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as libc::c_char;
    (*pdata)
        .nvm__fmt__fgc[10 as libc::c_int
        as usize] = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer
            .offset(0x1e5 as libc::c_int as isize)
            .offset(-(1 as libc::c_int as isize)),
        0x3f8 as libc::c_int as u32,
        3 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as libc::c_char;
    (*pdata)
        .nvm__fmt__fgc[11 as libc::c_int
        as usize] = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer
            .offset(0x1e6 as libc::c_int as isize)
            .offset(-(1 as libc::c_int as isize)),
        0x7f0 as libc::c_int as u32,
        4 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as libc::c_char;
    (*pdata)
        .nvm__fmt__fgc[12 as libc::c_int
        as usize] = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer
            .offset(0x1e7 as libc::c_int as isize)
            .offset(-(1 as libc::c_int as isize)),
        0xfe0 as libc::c_int as u32,
        5 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as libc::c_char;
    (*pdata)
        .nvm__fmt__fgc[13 as libc::c_int
        as usize] = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer
            .offset(0x1e8 as libc::c_int as isize)
            .offset(-(1 as libc::c_int as isize)),
        0x1fc0 as libc::c_int as u32,
        6 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as libc::c_char;
    (*pdata)
        .nvm__fmt__fgc[14 as libc::c_int
        as usize] = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer
            .offset(0x1e9 as libc::c_int as isize)
            .offset(-(1 as libc::c_int as isize)),
        0x3f80 as libc::c_int as u32,
        7 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as libc::c_char;
    (*pdata)
        .nvm__fmt__fgc[15 as libc::c_int
        as usize] = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer.offset(0x1e9 as libc::c_int as isize),
        0x7f as libc::c_int as u32,
        0 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as libc::c_char;
    (*pdata)
        .nvm__fmt__fgc[16 as libc::c_int
        as usize] = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer.offset(0x1ea as libc::c_int as isize),
        0xfe as libc::c_int as u32,
        1 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as libc::c_char;
    (*pdata)
        .nvm__fmt__fgc[17 as libc::c_int
        as usize] = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer.offset(0x1eb as libc::c_int as isize),
        0x1fc as libc::c_int as u32,
        2 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as libc::c_char;
    (*pdata)
        .nvm__fmt__fgc[18 as libc::c_int as usize] = 0 as libc::c_int as libc::c_char;
    (*pdata)
        .nvm__fmt__test_program_major = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer.offset(0x1ec as libc::c_int as isize),
        0xe0 as libc::c_int as u32,
        5 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u8;
    (*pdata)
        .nvm__fmt__test_program_minor = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer.offset(0x1ec as libc::c_int as isize),
        0x1f as libc::c_int as u32,
        0 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u8;
    (*pdata)
        .nvm__fmt__map_major = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer.offset(0x1ed as libc::c_int as isize),
        0xe0 as libc::c_int as u32,
        5 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u8;
    (*pdata)
        .nvm__fmt__map_minor = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer.offset(0x1ed as libc::c_int as isize),
        0x1f as libc::c_int as u32,
        0 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u8;
    (*pdata)
        .nvm__fmt__year = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer.offset(0x1ee as libc::c_int as isize),
        0xf0 as libc::c_int as u32,
        4 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u8;
    (*pdata)
        .nvm__fmt__month = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer.offset(0x1ee as libc::c_int as isize),
        0xf as libc::c_int as u32,
        0 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u8;
    (*pdata)
        .nvm__fmt__day = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer.offset(0x1ef as libc::c_int as isize),
        0xf8 as libc::c_int as u32,
        3 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u8;
    (*pdata)
        .nvm__fmt__module_date_phase = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer.offset(0x1ef as libc::c_int as isize),
        0x7 as libc::c_int as u32,
        0 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u8;
    (*pdata)
        .nvm__fmt__time = VL53LX_i2c_decode_with_mask(
        2 as libc::c_int as u16,
        pbuffer.offset(0x1f0 as libc::c_int as isize),
        0xffff as libc::c_int as u32,
        0 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u16;
    (*pdata)
        .nvm__fmt__tester_id = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer.offset(0x1f2 as libc::c_int as isize),
        0xff as libc::c_int as u32,
        0 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u8;
    (*pdata)
        .nvm__fmt__site_id = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer.offset(0x1f3 as libc::c_int as isize),
        0xff as libc::c_int as u32,
        0 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u8;
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_nvm_decode_ews_info(
    mut buf_size: u16,
    mut pbuffer: *mut u8,
    mut pdata: *mut VL53LX_decoded_nvm_ews_info_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    if (buf_size as libc::c_int) < 512 as libc::c_int {
        return -(9 as libc::c_int) as VL53LX_Error;
    }
    (*pdata)
        .nvm__ews__test_program_major = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer.offset(0x1f4 as libc::c_int as isize),
        0xe0 as libc::c_int as u32,
        5 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u8;
    (*pdata)
        .nvm__ews__test_program_minor = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer.offset(0x1f4 as libc::c_int as isize),
        0x1f as libc::c_int as u32,
        0 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u8;
    (*pdata)
        .nvm__ews__probe_card_major = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer.offset(0x1f5 as libc::c_int as isize),
        0xf0 as libc::c_int as u32,
        4 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u8;
    (*pdata)
        .nvm__ews__probe_card_minor = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer.offset(0x1f5 as libc::c_int as isize),
        0xf as libc::c_int as u32,
        0 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u8;
    (*pdata)
        .nvm__ews__tester_id = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer.offset(0x1f6 as libc::c_int as isize),
        0xff as libc::c_int as u32,
        0 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u8;
    (*pdata)
        .nvm__ews__lot[0 as libc::c_int
        as usize] = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer.offset(0x1f8 as libc::c_int as isize),
        0xfc as libc::c_int as u32,
        2 as libc::c_int as u32,
        32 as libc::c_int as u32,
    ) as libc::c_char;
    (*pdata)
        .nvm__ews__lot[1 as libc::c_int
        as usize] = VL53LX_i2c_decode_with_mask(
        2 as libc::c_int as u16,
        pbuffer
            .offset(0x1f9 as libc::c_int as isize)
            .offset(-(1 as libc::c_int as isize)),
        0x3f0 as libc::c_int as u32,
        4 as libc::c_int as u32,
        32 as libc::c_int as u32,
    ) as libc::c_char;
    (*pdata)
        .nvm__ews__lot[2 as libc::c_int
        as usize] = VL53LX_i2c_decode_with_mask(
        2 as libc::c_int as u16,
        pbuffer
            .offset(0x1fa as libc::c_int as isize)
            .offset(-(1 as libc::c_int as isize)),
        0xfc0 as libc::c_int as u32,
        6 as libc::c_int as u32,
        32 as libc::c_int as u32,
    ) as libc::c_char;
    (*pdata)
        .nvm__ews__lot[3 as libc::c_int
        as usize] = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer.offset(0x1fa as libc::c_int as isize),
        0x3f as libc::c_int as u32,
        0 as libc::c_int as u32,
        32 as libc::c_int as u32,
    ) as libc::c_char;
    (*pdata)
        .nvm__ews__lot[4 as libc::c_int
        as usize] = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer.offset(0x1fb as libc::c_int as isize),
        0xfc as libc::c_int as u32,
        2 as libc::c_int as u32,
        32 as libc::c_int as u32,
    ) as libc::c_char;
    (*pdata)
        .nvm__ews__lot[5 as libc::c_int
        as usize] = VL53LX_i2c_decode_with_mask(
        2 as libc::c_int as u16,
        pbuffer
            .offset(0x1fc as libc::c_int as isize)
            .offset(-(1 as libc::c_int as isize)),
        0x3f0 as libc::c_int as u32,
        4 as libc::c_int as u32,
        32 as libc::c_int as u32,
    ) as libc::c_char;
    (*pdata)
        .nvm__ews__lot[6 as libc::c_int
        as usize] = VL53LX_i2c_decode_with_mask(
        2 as libc::c_int as u16,
        pbuffer
            .offset(0x1fd as libc::c_int as isize)
            .offset(-(1 as libc::c_int as isize)),
        0xfc0 as libc::c_int as u32,
        6 as libc::c_int as u32,
        32 as libc::c_int as u32,
    ) as libc::c_char;
    (*pdata).nvm__ews__lot[7 as libc::c_int as usize] = 0 as libc::c_int as libc::c_char;
    (*pdata)
        .nvm__ews__wafer = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer.offset(0x1fd as libc::c_int as isize),
        0x1f as libc::c_int as u32,
        0 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u8;
    (*pdata)
        .nvm__ews__xcoord = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer.offset(0x1fe as libc::c_int as isize),
        0xff as libc::c_int as u32,
        0 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u8;
    (*pdata)
        .nvm__ews__ycoord = VL53LX_i2c_decode_with_mask(
        1 as libc::c_int as u16,
        pbuffer.offset(0x1ff as libc::c_int as isize),
        0xff as libc::c_int as u32,
        0 as libc::c_int as u32,
        0 as libc::c_int as u32,
    ) as u8;
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_nvm_format_encode(
    mut pnvm_info: *mut VL53LX_decoded_nvm_data_t,
    mut pnvm_data: *mut u8,
) {}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_read_nvm_raw_data(
    mut Dev: VL53LX_DEV,
    mut start_address: u8,
    mut count: u8,
    mut pnvm_raw_data: *mut u8,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_nvm_enable(
            Dev,
            0x4 as libc::c_int as u16,
            50 as libc::c_int,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_nvm_read(Dev, start_address, count, pnvm_raw_data);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_nvm_disable(Dev);
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_read_nvm(
    mut Dev: VL53LX_DEV,
    mut nvm_format: u8,
    mut pnvm_info: *mut VL53LX_decoded_nvm_data_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut nvm_data: [u8; 512] = [0; 512];
    status = VL53LX_read_nvm_raw_data(
        Dev,
        0 as libc::c_int as u8,
        (512 as libc::c_int >> 2 as libc::c_int) as u8,
        nvm_data.as_mut_ptr(),
    );
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_nvm_format_decode(
            512 as libc::c_int as u16,
            nvm_data.as_mut_ptr(),
            pnvm_info,
        );
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_read_nvm_optical_centre(
    mut Dev: VL53LX_DEV,
    mut pcentre: *mut VL53LX_optical_centre_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut nvm_data: [u8; 4] = [0; 4];
    status = VL53LX_read_nvm_raw_data(
        Dev,
        (0xb8 as libc::c_int >> 2 as libc::c_int) as u8,
        (4 as libc::c_int >> 2 as libc::c_int) as u8,
        nvm_data.as_mut_ptr(),
    );
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_nvm_decode_optical_centre(
            4 as libc::c_int as u16,
            nvm_data.as_mut_ptr(),
            pcentre,
        );
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_read_nvm_cal_peak_rate_map(
    mut Dev: VL53LX_DEV,
    mut pcal_data: *mut VL53LX_cal_peak_rate_map_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut nvm_data: [u8; 56] = [0; 56];
    status = VL53LX_read_nvm_raw_data(
        Dev,
        (0x15c as libc::c_int >> 2 as libc::c_int) as u8,
        (56 as libc::c_int >> 2 as libc::c_int) as u8,
        nvm_data.as_mut_ptr(),
    );
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_nvm_decode_cal_peak_rate_map(
            56 as libc::c_int as u16,
            nvm_data.as_mut_ptr(),
            pcal_data,
        );
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_read_nvm_additional_offset_cal_data(
    mut Dev: VL53LX_DEV,
    mut pcal_data: *mut VL53LX_additional_offset_cal_data_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut nvm_data: [u8; 8] = [0; 8];
    status = VL53LX_read_nvm_raw_data(
        Dev,
        (0x194 as libc::c_int >> 2 as libc::c_int) as u8,
        (8 as libc::c_int >> 2 as libc::c_int) as u8,
        nvm_data.as_mut_ptr(),
    );
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_nvm_decode_additional_offset_cal_data(
            8 as libc::c_int as u16,
            nvm_data.as_mut_ptr(),
            pcal_data,
        );
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_read_nvm_fmt_range_results_data(
    mut Dev: VL53LX_DEV,
    mut range_results_select: u16,
    mut prange_data: *mut VL53LX_decoded_nvm_fmt_range_data_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut nvm_data: [u8; 16] = [0; 16];
    status = VL53LX_read_nvm_raw_data(
        Dev,
        (range_results_select as libc::c_int >> 2 as libc::c_int) as u8,
        (16 as libc::c_int >> 2 as libc::c_int) as u8,
        nvm_data.as_mut_ptr(),
    );
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_nvm_decode_fmt_range_results_data(
            16 as libc::c_int as u16,
            nvm_data.as_mut_ptr(),
            prange_data,
        );
    }
    return status;
}
#[no_mangle]
pub static mut _power_board_in_use: u32 = 0 as libc::c_int as u32;
#[no_mangle]
pub static mut _power_board_extended: u32 = 0 as libc::c_int as u32;
#[no_mangle]
pub static mut global_comms_type: u8 = 0 as libc::c_int as u8;
#[no_mangle]
pub unsafe extern "C" fn VL53LX_CommsInitialise(
    mut pdev: *mut VL53LX_Dev_t,
    mut comms_type: u8,
    mut comms_speed_khz: u16,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut comms_error_string: [libc::c_char; 1024] = [0; 1024];
    global_comms_type = comms_type;
    status = -(13 as libc::c_int) as VL53LX_Error;
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_CommsClose(mut pdev: *mut VL53LX_Dev_t) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut comms_error_string: [libc::c_char; 1024] = [0; 1024];
    status = -(13 as libc::c_int) as VL53LX_Error;
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_WriteMulti(
    mut pdev: *mut VL53LX_Dev_t,
    mut index: u16,
    mut pdata: *mut u8,
    mut count: u32,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut position: u32 = 0 as libc::c_int as u32;
    let mut data_size: u32 = 0 as libc::c_int as u32;
    let mut comms_error_string: [libc::c_char; 1024] = [0; 1024];
    if global_comms_type as libc::c_int == 0x1 as libc::c_int {
        position = 0 as libc::c_int as u32;
        while position < count {
            if count > 56 as libc::c_int as libc::c_uint {
                if position.wrapping_add(56 as libc::c_int as libc::c_uint) > count {
                    data_size = count.wrapping_sub(position);
                } else {
                    data_size = 56 as libc::c_int as u32;
                }
            } else {
                data_size = count;
            }
            if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
                if unimplemented() != 0 as libc::c_int {
                    status = -(13 as libc::c_int) as VL53LX_Error;
                }
            }
            position = (position as libc::c_uint)
                .wrapping_add(56 as libc::c_int as libc::c_uint) as u32 as u32;
        }
        if status as libc::c_int != 0 as libc::c_int as VL53LX_Error as libc::c_int {
            unimplemented();
        }
    } else if global_comms_type as libc::c_int == 0 as libc::c_int {
        if unimplemented() != 0 as libc::c_int {
            status = -(13 as libc::c_int) as VL53LX_Error;
        }
        if status as libc::c_int != 0 as libc::c_int as VL53LX_Error as libc::c_int {
            unimplemented();
        }
    } else {
        status = -(13 as libc::c_int) as VL53LX_Error;
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_ReadMulti(
    mut pdev: *mut VL53LX_Dev_t,
    mut index: u16,
    mut pdata: *mut u8,
    mut count: u32,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut position: u32 = 0 as libc::c_int as u32;
    let mut data_size: u32 = 0 as libc::c_int as u32;
    let mut comms_error_string: [libc::c_char; 1024] = [0; 1024];
    if global_comms_type as libc::c_int == 0x1 as libc::c_int {
        position = 0 as libc::c_int as u32;
        while position < count {
            if count > 56 as libc::c_int as libc::c_uint {
                if position.wrapping_add(56 as libc::c_int as libc::c_uint) > count {
                    data_size = count.wrapping_sub(position);
                } else {
                    data_size = 56 as libc::c_int as u32;
                }
            } else {
                data_size = count;
            }
            if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
                if unimplemented() != 0 as libc::c_int {
                    status = -(13 as libc::c_int) as VL53LX_Error;
                }
            }
            position = (position as libc::c_uint)
                .wrapping_add(56 as libc::c_int as libc::c_uint) as u32 as u32;
        }
        if status as libc::c_int != 0 as libc::c_int as VL53LX_Error as libc::c_int {
            unimplemented();
        }
    } else if global_comms_type as libc::c_int == 0 as libc::c_int {
        if unimplemented() != 0 as libc::c_int {
            status = -(13 as libc::c_int) as VL53LX_Error;
        }
        if status as libc::c_int != 0 as libc::c_int as VL53LX_Error as libc::c_int {
            unimplemented();
        }
    } else {
        status = -(13 as libc::c_int) as VL53LX_Error;
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_WrByte(
    mut pdev: *mut VL53LX_Dev_t,
    mut index: u16,
    mut VL53LX_p_003: u8,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut buffer: [u8; 1] = [0; 1];
    buffer[0 as libc::c_int as usize] = VL53LX_p_003;
    status = VL53LX_WriteMulti(
        pdev,
        index,
        buffer.as_mut_ptr(),
        1 as libc::c_int as u32,
    );
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_WrWord(
    mut pdev: *mut VL53LX_Dev_t,
    mut index: u16,
    mut VL53LX_p_003: u16,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut buffer: [u8; 2] = [0; 2];
    buffer[0 as libc::c_int
        as usize] = (VL53LX_p_003 as libc::c_int >> 8 as libc::c_int) as u8;
    buffer[1 as libc::c_int
        as usize] = (VL53LX_p_003 as libc::c_int & 0xff as libc::c_int) as u8;
    status = VL53LX_WriteMulti(
        pdev,
        index,
        buffer.as_mut_ptr(),
        2 as libc::c_int as u32,
    );
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_WrDWord(
    mut pdev: *mut VL53LX_Dev_t,
    mut index: u16,
    mut VL53LX_p_003: u32,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut buffer: [u8; 4] = [0; 4];
    buffer[0 as libc::c_int as usize] = (VL53LX_p_003 >> 24 as libc::c_int) as u8;
    buffer[1 as libc::c_int
        as usize] = ((VL53LX_p_003 & 0xff0000 as libc::c_int as libc::c_uint)
        >> 16 as libc::c_int) as u8;
    buffer[2 as libc::c_int
        as usize] = ((VL53LX_p_003 & 0xff00 as libc::c_int as libc::c_uint)
        >> 8 as libc::c_int) as u8;
    buffer[3 as libc::c_int
        as usize] = (VL53LX_p_003 & 0xff as libc::c_int as libc::c_uint) as u8;
    status = VL53LX_WriteMulti(
        pdev,
        index,
        buffer.as_mut_ptr(),
        4 as libc::c_int as u32,
    );
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_RdByte(
    mut pdev: *mut VL53LX_Dev_t,
    mut index: u16,
    mut pdata: *mut u8,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut buffer: [u8; 1] = [0; 1];
    status = VL53LX_ReadMulti(
        pdev,
        index,
        buffer.as_mut_ptr(),
        1 as libc::c_int as u32,
    );
    *pdata = buffer[0 as libc::c_int as usize];
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_RdWord(
    mut pdev: *mut VL53LX_Dev_t,
    mut index: u16,
    mut pdata: *mut u16,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut buffer: [u8; 2] = [0; 2];
    status = VL53LX_ReadMulti(
        pdev,
        index,
        buffer.as_mut_ptr(),
        2 as libc::c_int as u32,
    );
    *pdata = (((buffer[0 as libc::c_int as usize] as u16 as libc::c_int)
        << 8 as libc::c_int)
        + buffer[1 as libc::c_int as usize] as u16 as libc::c_int) as u16;
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_RdDWord(
    mut pdev: *mut VL53LX_Dev_t,
    mut index: u16,
    mut pdata: *mut u32,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut buffer: [u8; 4] = [0; 4];
    status = VL53LX_ReadMulti(
        pdev,
        index,
        buffer.as_mut_ptr(),
        4 as libc::c_int as u32,
    );
    *pdata = ((buffer[0 as libc::c_int as usize] as u32) << 24 as libc::c_int)
        .wrapping_add(
            (buffer[1 as libc::c_int as usize] as u32) << 16 as libc::c_int,
        )
        .wrapping_add(
            (buffer[2 as libc::c_int as usize] as u32) << 8 as libc::c_int,
        )
        .wrapping_add(buffer[3 as libc::c_int as usize] as u32);
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_WaitUs(
    mut pdev: *mut VL53LX_Dev_t,
    mut wait_us: i32,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut wait_ms: libc::c_float = wait_us as libc::c_float / 1000.0f32;
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_WaitMs(
    mut pdev: *mut VL53LX_Dev_t,
    mut wait_ms: i32,
) -> VL53LX_Error {
    return VL53LX_WaitUs(pdev, wait_ms * 1000 as libc::c_int);
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_GetTimerFrequency(
    mut ptimer_freq_hz: *mut i32,
) -> VL53LX_Error {
    *ptimer_freq_hz = 0 as libc::c_int;
    return 0 as libc::c_int as VL53LX_Error;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_GetTimerValue(
    mut ptimer_count: *mut i32,
) -> VL53LX_Error {
    *ptimer_count = 0 as libc::c_int;
    return 0 as libc::c_int as VL53LX_Error;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_GpioSetMode(
    mut pin: u8,
    mut mode: u8,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_GpioSetValue(
    mut pin: u8,
    mut value: u8,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_GpioGetValue(
    mut pin: u8,
    mut pvalue: *mut u8,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut value: libc::c_int = 0 as libc::c_int;
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_GpioXshutdown(mut value: u8) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_GpioSetMode(
            3 as libc::c_int as u8,
            5 as libc::c_int as u8,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        if value != 0 {
            if unimplemented() != 0 as libc::c_int {
                status = -(13 as libc::c_int) as VL53LX_Error;
            }
        } else if unimplemented() != 0 as libc::c_int {
            status = -(13 as libc::c_int) as VL53LX_Error;
        }
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_GpioCommsSelect(mut value: u8) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_GpioSetMode(
            4 as libc::c_int as u8,
            5 as libc::c_int as u8,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        if value != 0 {
            if unimplemented() != 0 as libc::c_int {
                status = -(13 as libc::c_int) as VL53LX_Error;
            }
        } else if unimplemented() != 0 as libc::c_int {
            status = -(13 as libc::c_int) as VL53LX_Error;
        }
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_GpioPowerEnable(mut value: u8) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_GpioInterruptEnable(
    mut function: Option::<unsafe extern "C" fn() -> ()>,
    mut edge_type: u8,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_GpioInterruptDisable() -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_GetTickCount(
    mut pdev: *mut VL53LX_Dev_t,
    mut ptick_count_ms: *mut u32,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    *ptick_count_ms = unimplemented() as u32;
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_WaitValueMaskEx(
    mut pdev: *mut VL53LX_Dev_t,
    mut timeout_ms: u32,
    mut index: u16,
    mut value: u8,
    mut mask: u8,
    mut poll_delay_ms: u32,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut start_time_ms: u32 = 0 as libc::c_int as u32;
    let mut current_time_ms: u32 = 0 as libc::c_int as u32;
    let mut byte_value: u8 = 0 as libc::c_int as u8;
    let mut found: u8 = 0 as libc::c_int as u8;
    VL53LX_GetTickCount(pdev, &mut start_time_ms);
    (*pdev).new_data_ready_poll_duration_ms = 0 as libc::c_int as u32;
    while status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int
        && (*pdev).new_data_ready_poll_duration_ms < timeout_ms
        && found as libc::c_int == 0 as libc::c_int
    {
        status = VL53LX_RdByte(pdev, index, &mut byte_value);
        if byte_value as libc::c_int & mask as libc::c_int == value as libc::c_int {
            found = 1 as libc::c_int as u8;
        }
        VL53LX_GetTickCount(pdev, &mut current_time_ms);
        (*pdev)
            .new_data_ready_poll_duration_ms = current_time_ms
            .wrapping_sub(start_time_ms);
    }
    if found as libc::c_int == 0 as libc::c_int
        && status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int
    {
        status = -(7 as libc::c_int) as VL53LX_Error;
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_platform_init(
    mut pdev: *mut VL53LX_Dev_t,
    mut i2c_slave_address: u8,
    mut comms_type: u8,
    mut comms_speed_khz: u16,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    (*pdev).i2c_slave_address = i2c_slave_address;
    (*pdev).comms_type = comms_type;
    (*pdev).comms_speed_khz = comms_speed_khz;
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_CommsInitialise(
            pdev,
            (*pdev).comms_type,
            (*pdev).comms_speed_khz,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_GpioXshutdown(0 as libc::c_int as u8);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_GpioPowerEnable(0 as libc::c_int as u8);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_GpioCommsSelect(0 as libc::c_int as u8);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_WaitUs(pdev, 1000 as libc::c_int);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_GpioPowerEnable(1 as libc::c_int as u8);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_WaitUs(pdev, 1000 as libc::c_int);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_GpioXshutdown(1 as libc::c_int as u8);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_WaitUs(pdev, 100 as libc::c_int);
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_platform_terminate(
    mut pdev: *mut VL53LX_Dev_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_GpioXshutdown(0 as libc::c_int as u8);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_GpioPowerEnable(0 as libc::c_int as u8);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_CommsClose(pdev);
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_ipp_hist_process_data(
    mut Dev: VL53LX_DEV,
    mut pdmax_cal: *mut VL53LX_dmax_calibration_data_t,
    mut pdmax_cfg: *mut VL53LX_hist_gen3_dmax_config_t,
    mut ppost_cfg: *mut VL53LX_hist_post_process_config_t,
    mut pbins: *mut VL53LX_histogram_bin_data_t,
    mut pxtalk: *mut VL53LX_xtalk_histogram_data_t,
    mut pArea1: *mut u8,
    mut pArea2: *mut u8,
    mut phisto_merge_nb: *mut u8,
    mut presults: *mut VL53LX_range_results_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    status = VL53LX_hist_process_data(
        pdmax_cal,
        pdmax_cfg,
        ppost_cfg,
        pbins,
        pxtalk,
        pArea1,
        pArea2,
        presults,
        phisto_merge_nb,
    );
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_ipp_hist_ambient_dmax(
    mut Dev: VL53LX_DEV,
    mut target_reflectance: u16,
    mut pdmax_cal: *mut VL53LX_dmax_calibration_data_t,
    mut pdmax_cfg: *mut VL53LX_hist_gen3_dmax_config_t,
    mut pbins: *mut VL53LX_histogram_bin_data_t,
    mut pambient_dmax_mm: *mut i16,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    status = VL53LX_hist_ambient_dmax(
        target_reflectance,
        pdmax_cal,
        pdmax_cfg,
        pbins,
        pambient_dmax_mm,
    );
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_ipp_xtalk_calibration_process_data(
    mut Dev: VL53LX_DEV,
    mut pxtalk_ranges: *mut VL53LX_xtalk_range_results_t,
    mut pxtalk_shape: *mut VL53LX_xtalk_histogram_data_t,
    mut pxtalk_cal: *mut VL53LX_xtalk_calibration_results_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    status = VL53LX_xtalk_calibration_process_data(
        pxtalk_ranges,
        pxtalk_shape,
        pxtalk_cal,
    );
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_i2c_encode_static_nvm_managed(
    mut pdata: *mut VL53LX_static_nvm_managed_t,
    mut buf_size: u16,
    mut pbuffer: *mut u8,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    if (buf_size as libc::c_int) < 11 as libc::c_int {
        return -(10 as libc::c_int) as VL53LX_Error;
    }
    *pbuffer
        .offset(
            0 as libc::c_int as isize,
        ) = ((*pdata).i2c_slave__device_address as libc::c_int & 0x7f as libc::c_int)
        as u8;
    *pbuffer
        .offset(
            1 as libc::c_int as isize,
        ) = ((*pdata).ana_config__vhv_ref_sel_vddpix as libc::c_int & 0xf as libc::c_int)
        as u8;
    *pbuffer
        .offset(
            2 as libc::c_int as isize,
        ) = ((*pdata).ana_config__vhv_ref_sel_vquench as libc::c_int
        & 0x7f as libc::c_int) as u8;
    *pbuffer
        .offset(
            3 as libc::c_int as isize,
        ) = ((*pdata).ana_config__reg_avdd1v2_sel as libc::c_int & 0x3 as libc::c_int)
        as u8;
    *pbuffer
        .offset(
            4 as libc::c_int as isize,
        ) = ((*pdata).ana_config__fast_osc__trim as libc::c_int & 0x7f as libc::c_int)
        as u8;
    VL53LX_i2c_encode_uint16_t(
        (*pdata).osc_measured__fast_osc__frequency,
        2 as libc::c_int as u16,
        pbuffer.offset(5 as libc::c_int as isize),
    );
    *pbuffer
        .offset(
            7 as libc::c_int as isize,
        ) = (*pdata).vhv_config__timeout_macrop_loop_bound;
    *pbuffer.offset(8 as libc::c_int as isize) = (*pdata).vhv_config__count_thresh;
    *pbuffer
        .offset(
            9 as libc::c_int as isize,
        ) = ((*pdata).vhv_config__offset as libc::c_int & 0x3f as libc::c_int)
        as u8;
    *pbuffer.offset(10 as libc::c_int as isize) = (*pdata).vhv_config__init;
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_i2c_decode_static_nvm_managed(
    mut buf_size: u16,
    mut pbuffer: *mut u8,
    mut pdata: *mut VL53LX_static_nvm_managed_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    if (buf_size as libc::c_int) < 11 as libc::c_int {
        return -(10 as libc::c_int) as VL53LX_Error;
    }
    (*pdata)
        .i2c_slave__device_address = (*pbuffer.offset(0 as libc::c_int as isize)
        as libc::c_int & 0x7f as libc::c_int) as u8;
    (*pdata)
        .ana_config__vhv_ref_sel_vddpix = (*pbuffer.offset(1 as libc::c_int as isize)
        as libc::c_int & 0xf as libc::c_int) as u8;
    (*pdata)
        .ana_config__vhv_ref_sel_vquench = (*pbuffer.offset(2 as libc::c_int as isize)
        as libc::c_int & 0x7f as libc::c_int) as u8;
    (*pdata)
        .ana_config__reg_avdd1v2_sel = (*pbuffer.offset(3 as libc::c_int as isize)
        as libc::c_int & 0x3 as libc::c_int) as u8;
    (*pdata)
        .ana_config__fast_osc__trim = (*pbuffer.offset(4 as libc::c_int as isize)
        as libc::c_int & 0x7f as libc::c_int) as u8;
    (*pdata)
        .osc_measured__fast_osc__frequency = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(5 as libc::c_int as isize),
    );
    (*pdata)
        .vhv_config__timeout_macrop_loop_bound = *pbuffer
        .offset(7 as libc::c_int as isize);
    (*pdata).vhv_config__count_thresh = *pbuffer.offset(8 as libc::c_int as isize);
    (*pdata)
        .vhv_config__offset = (*pbuffer.offset(9 as libc::c_int as isize) as libc::c_int
        & 0x3f as libc::c_int) as u8;
    (*pdata).vhv_config__init = *pbuffer.offset(10 as libc::c_int as isize);
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_set_static_nvm_managed(
    mut Dev: VL53LX_DEV,
    mut pdata: *mut VL53LX_static_nvm_managed_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut comms_buffer: [u8; 11] = [0; 11];
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_i2c_encode_static_nvm_managed(
            pdata,
            11 as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_WriteMulti(
            Dev,
            0x1 as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
            11 as libc::c_int as u32,
        );
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_get_static_nvm_managed(
    mut Dev: VL53LX_DEV,
    mut pdata: *mut VL53LX_static_nvm_managed_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut comms_buffer: [u8; 11] = [0; 11];
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_ReadMulti(
            Dev,
            0x1 as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
            11 as libc::c_int as u32,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_i2c_decode_static_nvm_managed(
            11 as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
            pdata,
        );
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_i2c_encode_customer_nvm_managed(
    mut pdata: *mut VL53LX_customer_nvm_managed_t,
    mut buf_size: u16,
    mut pbuffer: *mut u8,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    if (buf_size as libc::c_int) < 23 as libc::c_int {
        return -(10 as libc::c_int) as VL53LX_Error;
    }
    *pbuffer
        .offset(0 as libc::c_int as isize) = (*pdata).global_config__spad_enables_ref_0;
    *pbuffer
        .offset(1 as libc::c_int as isize) = (*pdata).global_config__spad_enables_ref_1;
    *pbuffer
        .offset(2 as libc::c_int as isize) = (*pdata).global_config__spad_enables_ref_2;
    *pbuffer
        .offset(3 as libc::c_int as isize) = (*pdata).global_config__spad_enables_ref_3;
    *pbuffer
        .offset(4 as libc::c_int as isize) = (*pdata).global_config__spad_enables_ref_4;
    *pbuffer
        .offset(
            5 as libc::c_int as isize,
        ) = ((*pdata).global_config__spad_enables_ref_5 as libc::c_int
        & 0xf as libc::c_int) as u8;
    *pbuffer
        .offset(6 as libc::c_int as isize) = (*pdata).global_config__ref_en_start_select;
    *pbuffer
        .offset(
            7 as libc::c_int as isize,
        ) = ((*pdata).ref_spad_man__num_requested_ref_spads as libc::c_int
        & 0x3f as libc::c_int) as u8;
    *pbuffer
        .offset(
            8 as libc::c_int as isize,
        ) = ((*pdata).ref_spad_man__ref_location as libc::c_int & 0x3 as libc::c_int)
        as u8;
    VL53LX_i2c_encode_uint16_t(
        (*pdata).algo__crosstalk_compensation_plane_offset_kcps,
        2 as libc::c_int as u16,
        pbuffer.offset(9 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_int16_t(
        (*pdata).algo__crosstalk_compensation_x_plane_gradient_kcps,
        2 as libc::c_int as u16,
        pbuffer.offset(11 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_int16_t(
        (*pdata).algo__crosstalk_compensation_y_plane_gradient_kcps,
        2 as libc::c_int as u16,
        pbuffer.offset(13 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint16_t(
        (*pdata).ref_spad_char__total_rate_target_mcps,
        2 as libc::c_int as u16,
        pbuffer.offset(15 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_int16_t(
        ((*pdata).algo__part_to_part_range_offset_mm as libc::c_int
            & 0x1fff as libc::c_int) as i16,
        2 as libc::c_int as u16,
        pbuffer.offset(17 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_int16_t(
        (*pdata).mm_config__inner_offset_mm,
        2 as libc::c_int as u16,
        pbuffer.offset(19 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_int16_t(
        (*pdata).mm_config__outer_offset_mm,
        2 as libc::c_int as u16,
        pbuffer.offset(21 as libc::c_int as isize),
    );
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_i2c_decode_customer_nvm_managed(
    mut buf_size: u16,
    mut pbuffer: *mut u8,
    mut pdata: *mut VL53LX_customer_nvm_managed_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    if (buf_size as libc::c_int) < 23 as libc::c_int {
        return -(10 as libc::c_int) as VL53LX_Error;
    }
    (*pdata)
        .global_config__spad_enables_ref_0 = *pbuffer.offset(0 as libc::c_int as isize);
    (*pdata)
        .global_config__spad_enables_ref_1 = *pbuffer.offset(1 as libc::c_int as isize);
    (*pdata)
        .global_config__spad_enables_ref_2 = *pbuffer.offset(2 as libc::c_int as isize);
    (*pdata)
        .global_config__spad_enables_ref_3 = *pbuffer.offset(3 as libc::c_int as isize);
    (*pdata)
        .global_config__spad_enables_ref_4 = *pbuffer.offset(4 as libc::c_int as isize);
    (*pdata)
        .global_config__spad_enables_ref_5 = (*pbuffer.offset(5 as libc::c_int as isize)
        as libc::c_int & 0xf as libc::c_int) as u8;
    (*pdata)
        .global_config__ref_en_start_select = *pbuffer.offset(6 as libc::c_int as isize);
    (*pdata)
        .ref_spad_man__num_requested_ref_spads = (*pbuffer
        .offset(7 as libc::c_int as isize) as libc::c_int & 0x3f as libc::c_int)
        as u8;
    (*pdata)
        .ref_spad_man__ref_location = (*pbuffer.offset(8 as libc::c_int as isize)
        as libc::c_int & 0x3 as libc::c_int) as u8;
    (*pdata)
        .algo__crosstalk_compensation_plane_offset_kcps = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(9 as libc::c_int as isize),
    );
    (*pdata)
        .algo__crosstalk_compensation_x_plane_gradient_kcps = VL53LX_i2c_decode_int16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(11 as libc::c_int as isize),
    );
    (*pdata)
        .algo__crosstalk_compensation_y_plane_gradient_kcps = VL53LX_i2c_decode_int16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(13 as libc::c_int as isize),
    );
    (*pdata)
        .ref_spad_char__total_rate_target_mcps = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(15 as libc::c_int as isize),
    );
    (*pdata)
        .algo__part_to_part_range_offset_mm = (VL53LX_i2c_decode_int16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(17 as libc::c_int as isize),
    ) as libc::c_int & 0x1fff as libc::c_int) as i16;
    (*pdata)
        .mm_config__inner_offset_mm = VL53LX_i2c_decode_int16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(19 as libc::c_int as isize),
    );
    (*pdata)
        .mm_config__outer_offset_mm = VL53LX_i2c_decode_int16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(21 as libc::c_int as isize),
    );
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_set_customer_nvm_managed(
    mut Dev: VL53LX_DEV,
    mut pdata: *mut VL53LX_customer_nvm_managed_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut comms_buffer: [u8; 23] = [0; 23];
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_i2c_encode_customer_nvm_managed(
            pdata,
            23 as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_WriteMulti(
            Dev,
            0xd as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
            23 as libc::c_int as u32,
        );
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_get_customer_nvm_managed(
    mut Dev: VL53LX_DEV,
    mut pdata: *mut VL53LX_customer_nvm_managed_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut comms_buffer: [u8; 23] = [0; 23];
    let mut offset: i16 = 0;
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_ReadMulti(
            Dev,
            0xd as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
            23 as libc::c_int as u32,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_i2c_decode_customer_nvm_managed(
            23 as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
            pdata,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        offset = (*pdata).algo__part_to_part_range_offset_mm;
        offset = (offset as libc::c_int / 4 as libc::c_int) as i16;
        if offset as libc::c_int >= 1024 as libc::c_int {
            offset = (offset as libc::c_int - 2048 as libc::c_int) as i16;
        }
        (*pdata).algo__part_to_part_range_offset_mm = 0 as libc::c_int as i16;
        (*pdata).mm_config__inner_offset_mm = offset;
        (*pdata).mm_config__outer_offset_mm = offset;
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_i2c_encode_static_config(
    mut pdata: *mut VL53LX_static_config_t,
    mut buf_size: u16,
    mut pbuffer: *mut u8,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    if (buf_size as libc::c_int) < 32 as libc::c_int {
        return -(10 as libc::c_int) as VL53LX_Error;
    }
    VL53LX_i2c_encode_uint16_t(
        (*pdata).dss_config__target_total_rate_mcps,
        2 as libc::c_int as u16,
        pbuffer.offset(0 as libc::c_int as isize),
    );
    *pbuffer
        .offset(
            2 as libc::c_int as isize,
        ) = ((*pdata).debug__ctrl as libc::c_int & 0x1 as libc::c_int) as u8;
    *pbuffer
        .offset(
            3 as libc::c_int as isize,
        ) = ((*pdata).test_mode__ctrl as libc::c_int & 0xf as libc::c_int) as u8;
    *pbuffer
        .offset(
            4 as libc::c_int as isize,
        ) = ((*pdata).clk_gating__ctrl as libc::c_int & 0xf as libc::c_int) as u8;
    *pbuffer
        .offset(
            5 as libc::c_int as isize,
        ) = ((*pdata).nvm_bist__ctrl as libc::c_int & 0x1f as libc::c_int) as u8;
    *pbuffer
        .offset(
            6 as libc::c_int as isize,
        ) = ((*pdata).nvm_bist__num_nvm_words as libc::c_int & 0x7f as libc::c_int)
        as u8;
    *pbuffer
        .offset(
            7 as libc::c_int as isize,
        ) = ((*pdata).nvm_bist__start_address as libc::c_int & 0x7f as libc::c_int)
        as u8;
    *pbuffer
        .offset(
            8 as libc::c_int as isize,
        ) = ((*pdata).host_if__status as libc::c_int & 0x1 as libc::c_int) as u8;
    *pbuffer.offset(9 as libc::c_int as isize) = (*pdata).pad_i2c_hv__config;
    *pbuffer
        .offset(
            10 as libc::c_int as isize,
        ) = ((*pdata).pad_i2c_hv__extsup_config as libc::c_int & 0x1 as libc::c_int)
        as u8;
    *pbuffer
        .offset(
            11 as libc::c_int as isize,
        ) = ((*pdata).gpio_hv_pad__ctrl as libc::c_int & 0x3 as libc::c_int) as u8;
    *pbuffer
        .offset(
            12 as libc::c_int as isize,
        ) = ((*pdata).gpio_hv_mux__ctrl as libc::c_int & 0x1f as libc::c_int) as u8;
    *pbuffer
        .offset(
            13 as libc::c_int as isize,
        ) = ((*pdata).gpio__tio_hv_status as libc::c_int & 0x3 as libc::c_int)
        as u8;
    *pbuffer
        .offset(
            14 as libc::c_int as isize,
        ) = ((*pdata).gpio__fio_hv_status as libc::c_int & 0x3 as libc::c_int)
        as u8;
    *pbuffer
        .offset(
            15 as libc::c_int as isize,
        ) = ((*pdata).ana_config__spad_sel_pswidth as libc::c_int & 0x7 as libc::c_int)
        as u8;
    *pbuffer
        .offset(
            16 as libc::c_int as isize,
        ) = ((*pdata).ana_config__vcsel_pulse_width_offset as libc::c_int
        & 0x1f as libc::c_int) as u8;
    *pbuffer
        .offset(
            17 as libc::c_int as isize,
        ) = ((*pdata).ana_config__fast_osc__config_ctrl as libc::c_int
        & 0x1 as libc::c_int) as u8;
    *pbuffer
        .offset(
            18 as libc::c_int as isize,
        ) = (*pdata).sigma_estimator__effective_pulse_width_ns;
    *pbuffer
        .offset(
            19 as libc::c_int as isize,
        ) = (*pdata).sigma_estimator__effective_ambient_width_ns;
    *pbuffer.offset(20 as libc::c_int as isize) = (*pdata).sigma_estimator__sigma_ref_mm;
    *pbuffer
        .offset(
            21 as libc::c_int as isize,
        ) = (*pdata).algo__crosstalk_compensation_valid_height_mm;
    *pbuffer
        .offset(
            22 as libc::c_int as isize,
        ) = (*pdata).spare_host_config__static_config_spare_0;
    *pbuffer
        .offset(
            23 as libc::c_int as isize,
        ) = (*pdata).spare_host_config__static_config_spare_1;
    VL53LX_i2c_encode_uint16_t(
        (*pdata).algo__range_ignore_threshold_mcps,
        2 as libc::c_int as u16,
        pbuffer.offset(24 as libc::c_int as isize),
    );
    *pbuffer
        .offset(
            26 as libc::c_int as isize,
        ) = (*pdata).algo__range_ignore_valid_height_mm;
    *pbuffer.offset(27 as libc::c_int as isize) = (*pdata).algo__range_min_clip;
    *pbuffer
        .offset(
            28 as libc::c_int as isize,
        ) = ((*pdata).algo__consistency_check__tolerance as libc::c_int
        & 0xf as libc::c_int) as u8;
    *pbuffer
        .offset(
            29 as libc::c_int as isize,
        ) = (*pdata).spare_host_config__static_config_spare_2;
    *pbuffer
        .offset(
            30 as libc::c_int as isize,
        ) = ((*pdata).sd_config__reset_stages_msb as libc::c_int & 0xf as libc::c_int)
        as u8;
    *pbuffer.offset(31 as libc::c_int as isize) = (*pdata).sd_config__reset_stages_lsb;
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_i2c_decode_static_config(
    mut buf_size: u16,
    mut pbuffer: *mut u8,
    mut pdata: *mut VL53LX_static_config_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    if (buf_size as libc::c_int) < 32 as libc::c_int {
        return -(10 as libc::c_int) as VL53LX_Error;
    }
    (*pdata)
        .dss_config__target_total_rate_mcps = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(0 as libc::c_int as isize),
    );
    (*pdata)
        .debug__ctrl = (*pbuffer.offset(2 as libc::c_int as isize) as libc::c_int
        & 0x1 as libc::c_int) as u8;
    (*pdata)
        .test_mode__ctrl = (*pbuffer.offset(3 as libc::c_int as isize) as libc::c_int
        & 0xf as libc::c_int) as u8;
    (*pdata)
        .clk_gating__ctrl = (*pbuffer.offset(4 as libc::c_int as isize) as libc::c_int
        & 0xf as libc::c_int) as u8;
    (*pdata)
        .nvm_bist__ctrl = (*pbuffer.offset(5 as libc::c_int as isize) as libc::c_int
        & 0x1f as libc::c_int) as u8;
    (*pdata)
        .nvm_bist__num_nvm_words = (*pbuffer.offset(6 as libc::c_int as isize)
        as libc::c_int & 0x7f as libc::c_int) as u8;
    (*pdata)
        .nvm_bist__start_address = (*pbuffer.offset(7 as libc::c_int as isize)
        as libc::c_int & 0x7f as libc::c_int) as u8;
    (*pdata)
        .host_if__status = (*pbuffer.offset(8 as libc::c_int as isize) as libc::c_int
        & 0x1 as libc::c_int) as u8;
    (*pdata).pad_i2c_hv__config = *pbuffer.offset(9 as libc::c_int as isize);
    (*pdata)
        .pad_i2c_hv__extsup_config = (*pbuffer.offset(10 as libc::c_int as isize)
        as libc::c_int & 0x1 as libc::c_int) as u8;
    (*pdata)
        .gpio_hv_pad__ctrl = (*pbuffer.offset(11 as libc::c_int as isize) as libc::c_int
        & 0x3 as libc::c_int) as u8;
    (*pdata)
        .gpio_hv_mux__ctrl = (*pbuffer.offset(12 as libc::c_int as isize) as libc::c_int
        & 0x1f as libc::c_int) as u8;
    (*pdata)
        .gpio__tio_hv_status = (*pbuffer.offset(13 as libc::c_int as isize)
        as libc::c_int & 0x3 as libc::c_int) as u8;
    (*pdata)
        .gpio__fio_hv_status = (*pbuffer.offset(14 as libc::c_int as isize)
        as libc::c_int & 0x3 as libc::c_int) as u8;
    (*pdata)
        .ana_config__spad_sel_pswidth = (*pbuffer.offset(15 as libc::c_int as isize)
        as libc::c_int & 0x7 as libc::c_int) as u8;
    (*pdata)
        .ana_config__vcsel_pulse_width_offset = (*pbuffer
        .offset(16 as libc::c_int as isize) as libc::c_int & 0x1f as libc::c_int)
        as u8;
    (*pdata)
        .ana_config__fast_osc__config_ctrl = (*pbuffer.offset(17 as libc::c_int as isize)
        as libc::c_int & 0x1 as libc::c_int) as u8;
    (*pdata)
        .sigma_estimator__effective_pulse_width_ns = *pbuffer
        .offset(18 as libc::c_int as isize);
    (*pdata)
        .sigma_estimator__effective_ambient_width_ns = *pbuffer
        .offset(19 as libc::c_int as isize);
    (*pdata).sigma_estimator__sigma_ref_mm = *pbuffer.offset(20 as libc::c_int as isize);
    (*pdata)
        .algo__crosstalk_compensation_valid_height_mm = *pbuffer
        .offset(21 as libc::c_int as isize);
    (*pdata)
        .spare_host_config__static_config_spare_0 = *pbuffer
        .offset(22 as libc::c_int as isize);
    (*pdata)
        .spare_host_config__static_config_spare_1 = *pbuffer
        .offset(23 as libc::c_int as isize);
    (*pdata)
        .algo__range_ignore_threshold_mcps = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(24 as libc::c_int as isize),
    );
    (*pdata)
        .algo__range_ignore_valid_height_mm = *pbuffer
        .offset(26 as libc::c_int as isize);
    (*pdata).algo__range_min_clip = *pbuffer.offset(27 as libc::c_int as isize);
    (*pdata)
        .algo__consistency_check__tolerance = (*pbuffer
        .offset(28 as libc::c_int as isize) as libc::c_int & 0xf as libc::c_int)
        as u8;
    (*pdata)
        .spare_host_config__static_config_spare_2 = *pbuffer
        .offset(29 as libc::c_int as isize);
    (*pdata)
        .sd_config__reset_stages_msb = (*pbuffer.offset(30 as libc::c_int as isize)
        as libc::c_int & 0xf as libc::c_int) as u8;
    (*pdata).sd_config__reset_stages_lsb = *pbuffer.offset(31 as libc::c_int as isize);
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_set_static_config(
    mut Dev: VL53LX_DEV,
    mut pdata: *mut VL53LX_static_config_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut comms_buffer: [u8; 32] = [0; 32];
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_i2c_encode_static_config(
            pdata,
            32 as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_WriteMulti(
            Dev,
            0x24 as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
            32 as libc::c_int as u32,
        );
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_get_static_config(
    mut Dev: VL53LX_DEV,
    mut pdata: *mut VL53LX_static_config_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut comms_buffer: [u8; 32] = [0; 32];
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_ReadMulti(
            Dev,
            0x24 as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
            32 as libc::c_int as u32,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_i2c_decode_static_config(
            32 as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
            pdata,
        );
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_i2c_encode_general_config(
    mut pdata: *mut VL53LX_general_config_t,
    mut buf_size: u16,
    mut pbuffer: *mut u8,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    if (buf_size as libc::c_int) < 22 as libc::c_int {
        return -(10 as libc::c_int) as VL53LX_Error;
    }
    *pbuffer
        .offset(
            0 as libc::c_int as isize,
        ) = (*pdata).gph_config__stream_count_update_value;
    *pbuffer.offset(1 as libc::c_int as isize) = (*pdata).global_config__stream_divider;
    *pbuffer.offset(2 as libc::c_int as isize) = (*pdata).system__interrupt_config_gpio;
    *pbuffer
        .offset(
            3 as libc::c_int as isize,
        ) = ((*pdata).cal_config__vcsel_start as libc::c_int & 0x7f as libc::c_int)
        as u8;
    VL53LX_i2c_encode_uint16_t(
        ((*pdata).cal_config__repeat_rate as libc::c_int & 0xfff as libc::c_int)
            as u16,
        2 as libc::c_int as u16,
        pbuffer.offset(4 as libc::c_int as isize),
    );
    *pbuffer
        .offset(
            6 as libc::c_int as isize,
        ) = ((*pdata).global_config__vcsel_width as libc::c_int & 0x7f as libc::c_int)
        as u8;
    *pbuffer
        .offset(7 as libc::c_int as isize) = (*pdata).phasecal_config__timeout_macrop;
    *pbuffer.offset(8 as libc::c_int as isize) = (*pdata).phasecal_config__target;
    *pbuffer
        .offset(
            9 as libc::c_int as isize,
        ) = ((*pdata).phasecal_config__override as libc::c_int & 0x1 as libc::c_int)
        as u8;
    *pbuffer
        .offset(
            11 as libc::c_int as isize,
        ) = ((*pdata).dss_config__roi_mode_control as libc::c_int & 0x7 as libc::c_int)
        as u8;
    VL53LX_i2c_encode_uint16_t(
        (*pdata).system__thresh_rate_high,
        2 as libc::c_int as u16,
        pbuffer.offset(12 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint16_t(
        (*pdata).system__thresh_rate_low,
        2 as libc::c_int as u16,
        pbuffer.offset(14 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint16_t(
        (*pdata).dss_config__manual_effective_spads_select,
        2 as libc::c_int as u16,
        pbuffer.offset(16 as libc::c_int as isize),
    );
    *pbuffer
        .offset(18 as libc::c_int as isize) = (*pdata).dss_config__manual_block_select;
    *pbuffer
        .offset(19 as libc::c_int as isize) = (*pdata).dss_config__aperture_attenuation;
    *pbuffer.offset(20 as libc::c_int as isize) = (*pdata).dss_config__max_spads_limit;
    *pbuffer.offset(21 as libc::c_int as isize) = (*pdata).dss_config__min_spads_limit;
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_i2c_decode_general_config(
    mut buf_size: u16,
    mut pbuffer: *mut u8,
    mut pdata: *mut VL53LX_general_config_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    if (buf_size as libc::c_int) < 22 as libc::c_int {
        return -(10 as libc::c_int) as VL53LX_Error;
    }
    (*pdata)
        .gph_config__stream_count_update_value = *pbuffer
        .offset(0 as libc::c_int as isize);
    (*pdata).global_config__stream_divider = *pbuffer.offset(1 as libc::c_int as isize);
    (*pdata).system__interrupt_config_gpio = *pbuffer.offset(2 as libc::c_int as isize);
    (*pdata)
        .cal_config__vcsel_start = (*pbuffer.offset(3 as libc::c_int as isize)
        as libc::c_int & 0x7f as libc::c_int) as u8;
    (*pdata)
        .cal_config__repeat_rate = (VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(4 as libc::c_int as isize),
    ) as libc::c_int & 0xfff as libc::c_int) as u16;
    (*pdata)
        .global_config__vcsel_width = (*pbuffer.offset(6 as libc::c_int as isize)
        as libc::c_int & 0x7f as libc::c_int) as u8;
    (*pdata)
        .phasecal_config__timeout_macrop = *pbuffer.offset(7 as libc::c_int as isize);
    (*pdata).phasecal_config__target = *pbuffer.offset(8 as libc::c_int as isize);
    (*pdata)
        .phasecal_config__override = (*pbuffer.offset(9 as libc::c_int as isize)
        as libc::c_int & 0x1 as libc::c_int) as u8;
    (*pdata)
        .dss_config__roi_mode_control = (*pbuffer.offset(11 as libc::c_int as isize)
        as libc::c_int & 0x7 as libc::c_int) as u8;
    (*pdata)
        .system__thresh_rate_high = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(12 as libc::c_int as isize),
    );
    (*pdata)
        .system__thresh_rate_low = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(14 as libc::c_int as isize),
    );
    (*pdata)
        .dss_config__manual_effective_spads_select = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(16 as libc::c_int as isize),
    );
    (*pdata)
        .dss_config__manual_block_select = *pbuffer.offset(18 as libc::c_int as isize);
    (*pdata)
        .dss_config__aperture_attenuation = *pbuffer.offset(19 as libc::c_int as isize);
    (*pdata).dss_config__max_spads_limit = *pbuffer.offset(20 as libc::c_int as isize);
    (*pdata).dss_config__min_spads_limit = *pbuffer.offset(21 as libc::c_int as isize);
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_set_general_config(
    mut Dev: VL53LX_DEV,
    mut pdata: *mut VL53LX_general_config_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut comms_buffer: [u8; 22] = [0; 22];
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_i2c_encode_general_config(
            pdata,
            22 as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_WriteMulti(
            Dev,
            0x44 as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
            22 as libc::c_int as u32,
        );
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_get_general_config(
    mut Dev: VL53LX_DEV,
    mut pdata: *mut VL53LX_general_config_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut comms_buffer: [u8; 22] = [0; 22];
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_ReadMulti(
            Dev,
            0x44 as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
            22 as libc::c_int as u32,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_i2c_decode_general_config(
            22 as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
            pdata,
        );
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_i2c_encode_timing_config(
    mut pdata: *mut VL53LX_timing_config_t,
    mut buf_size: u16,
    mut pbuffer: *mut u8,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    if (buf_size as libc::c_int) < 23 as libc::c_int {
        return -(10 as libc::c_int) as VL53LX_Error;
    }
    *pbuffer
        .offset(
            0 as libc::c_int as isize,
        ) = ((*pdata).mm_config__timeout_macrop_a_hi as libc::c_int & 0xf as libc::c_int)
        as u8;
    *pbuffer.offset(1 as libc::c_int as isize) = (*pdata).mm_config__timeout_macrop_a_lo;
    *pbuffer
        .offset(
            2 as libc::c_int as isize,
        ) = ((*pdata).mm_config__timeout_macrop_b_hi as libc::c_int & 0xf as libc::c_int)
        as u8;
    *pbuffer.offset(3 as libc::c_int as isize) = (*pdata).mm_config__timeout_macrop_b_lo;
    *pbuffer
        .offset(
            4 as libc::c_int as isize,
        ) = ((*pdata).range_config__timeout_macrop_a_hi as libc::c_int
        & 0xf as libc::c_int) as u8;
    *pbuffer
        .offset(5 as libc::c_int as isize) = (*pdata).range_config__timeout_macrop_a_lo;
    *pbuffer
        .offset(
            6 as libc::c_int as isize,
        ) = ((*pdata).range_config__vcsel_period_a as libc::c_int & 0x3f as libc::c_int)
        as u8;
    *pbuffer
        .offset(
            7 as libc::c_int as isize,
        ) = ((*pdata).range_config__timeout_macrop_b_hi as libc::c_int
        & 0xf as libc::c_int) as u8;
    *pbuffer
        .offset(8 as libc::c_int as isize) = (*pdata).range_config__timeout_macrop_b_lo;
    *pbuffer
        .offset(
            9 as libc::c_int as isize,
        ) = ((*pdata).range_config__vcsel_period_b as libc::c_int & 0x3f as libc::c_int)
        as u8;
    VL53LX_i2c_encode_uint16_t(
        (*pdata).range_config__sigma_thresh,
        2 as libc::c_int as u16,
        pbuffer.offset(10 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint16_t(
        (*pdata).range_config__min_count_rate_rtn_limit_mcps,
        2 as libc::c_int as u16,
        pbuffer.offset(12 as libc::c_int as isize),
    );
    *pbuffer.offset(14 as libc::c_int as isize) = (*pdata).range_config__valid_phase_low;
    *pbuffer
        .offset(15 as libc::c_int as isize) = (*pdata).range_config__valid_phase_high;
    VL53LX_i2c_encode_uint32_t(
        (*pdata).system__intermeasurement_period,
        4 as libc::c_int as u16,
        pbuffer.offset(18 as libc::c_int as isize),
    );
    *pbuffer
        .offset(
            22 as libc::c_int as isize,
        ) = ((*pdata).system__fractional_enable as libc::c_int & 0x1 as libc::c_int)
        as u8;
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_i2c_decode_timing_config(
    mut buf_size: u16,
    mut pbuffer: *mut u8,
    mut pdata: *mut VL53LX_timing_config_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    if (buf_size as libc::c_int) < 23 as libc::c_int {
        return -(10 as libc::c_int) as VL53LX_Error;
    }
    (*pdata)
        .mm_config__timeout_macrop_a_hi = (*pbuffer.offset(0 as libc::c_int as isize)
        as libc::c_int & 0xf as libc::c_int) as u8;
    (*pdata).mm_config__timeout_macrop_a_lo = *pbuffer.offset(1 as libc::c_int as isize);
    (*pdata)
        .mm_config__timeout_macrop_b_hi = (*pbuffer.offset(2 as libc::c_int as isize)
        as libc::c_int & 0xf as libc::c_int) as u8;
    (*pdata).mm_config__timeout_macrop_b_lo = *pbuffer.offset(3 as libc::c_int as isize);
    (*pdata)
        .range_config__timeout_macrop_a_hi = (*pbuffer.offset(4 as libc::c_int as isize)
        as libc::c_int & 0xf as libc::c_int) as u8;
    (*pdata)
        .range_config__timeout_macrop_a_lo = *pbuffer.offset(5 as libc::c_int as isize);
    (*pdata)
        .range_config__vcsel_period_a = (*pbuffer.offset(6 as libc::c_int as isize)
        as libc::c_int & 0x3f as libc::c_int) as u8;
    (*pdata)
        .range_config__timeout_macrop_b_hi = (*pbuffer.offset(7 as libc::c_int as isize)
        as libc::c_int & 0xf as libc::c_int) as u8;
    (*pdata)
        .range_config__timeout_macrop_b_lo = *pbuffer.offset(8 as libc::c_int as isize);
    (*pdata)
        .range_config__vcsel_period_b = (*pbuffer.offset(9 as libc::c_int as isize)
        as libc::c_int & 0x3f as libc::c_int) as u8;
    (*pdata)
        .range_config__sigma_thresh = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(10 as libc::c_int as isize),
    );
    (*pdata)
        .range_config__min_count_rate_rtn_limit_mcps = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(12 as libc::c_int as isize),
    );
    (*pdata).range_config__valid_phase_low = *pbuffer.offset(14 as libc::c_int as isize);
    (*pdata)
        .range_config__valid_phase_high = *pbuffer.offset(15 as libc::c_int as isize);
    (*pdata)
        .system__intermeasurement_period = VL53LX_i2c_decode_uint32_t(
        4 as libc::c_int as u16,
        pbuffer.offset(18 as libc::c_int as isize),
    );
    (*pdata)
        .system__fractional_enable = (*pbuffer.offset(22 as libc::c_int as isize)
        as libc::c_int & 0x1 as libc::c_int) as u8;
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_set_timing_config(
    mut Dev: VL53LX_DEV,
    mut pdata: *mut VL53LX_timing_config_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut comms_buffer: [u8; 23] = [0; 23];
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_i2c_encode_timing_config(
            pdata,
            23 as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_WriteMulti(
            Dev,
            0x5a as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
            23 as libc::c_int as u32,
        );
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_get_timing_config(
    mut Dev: VL53LX_DEV,
    mut pdata: *mut VL53LX_timing_config_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut comms_buffer: [u8; 23] = [0; 23];
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_ReadMulti(
            Dev,
            0x5a as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
            23 as libc::c_int as u32,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_i2c_decode_timing_config(
            23 as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
            pdata,
        );
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_i2c_encode_dynamic_config(
    mut pdata: *mut VL53LX_dynamic_config_t,
    mut buf_size: u16,
    mut pbuffer: *mut u8,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    if (buf_size as libc::c_int) < 18 as libc::c_int {
        return -(10 as libc::c_int) as VL53LX_Error;
    }
    *pbuffer
        .offset(
            0 as libc::c_int as isize,
        ) = ((*pdata).system__grouped_parameter_hold_0 as libc::c_int
        & 0x3 as libc::c_int) as u8;
    VL53LX_i2c_encode_uint16_t(
        (*pdata).system__thresh_high,
        2 as libc::c_int as u16,
        pbuffer.offset(1 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint16_t(
        (*pdata).system__thresh_low,
        2 as libc::c_int as u16,
        pbuffer.offset(3 as libc::c_int as isize),
    );
    *pbuffer
        .offset(
            5 as libc::c_int as isize,
        ) = ((*pdata).system__enable_xtalk_per_quadrant as libc::c_int
        & 0x1 as libc::c_int) as u8;
    *pbuffer
        .offset(
            6 as libc::c_int as isize,
        ) = ((*pdata).system__seed_config as libc::c_int & 0x7 as libc::c_int)
        as u8;
    *pbuffer.offset(7 as libc::c_int as isize) = (*pdata).sd_config__woi_sd0;
    *pbuffer.offset(8 as libc::c_int as isize) = (*pdata).sd_config__woi_sd1;
    *pbuffer
        .offset(
            9 as libc::c_int as isize,
        ) = ((*pdata).sd_config__initial_phase_sd0 as libc::c_int & 0x7f as libc::c_int)
        as u8;
    *pbuffer
        .offset(
            10 as libc::c_int as isize,
        ) = ((*pdata).sd_config__initial_phase_sd1 as libc::c_int & 0x7f as libc::c_int)
        as u8;
    *pbuffer
        .offset(
            11 as libc::c_int as isize,
        ) = ((*pdata).system__grouped_parameter_hold_1 as libc::c_int
        & 0x3 as libc::c_int) as u8;
    *pbuffer
        .offset(
            12 as libc::c_int as isize,
        ) = ((*pdata).sd_config__first_order_select as libc::c_int & 0x3 as libc::c_int)
        as u8;
    *pbuffer
        .offset(
            13 as libc::c_int as isize,
        ) = ((*pdata).sd_config__quantifier as libc::c_int & 0xf as libc::c_int)
        as u8;
    *pbuffer
        .offset(14 as libc::c_int as isize) = (*pdata).roi_config__user_roi_centre_spad;
    *pbuffer
        .offset(
            15 as libc::c_int as isize,
        ) = (*pdata).roi_config__user_roi_requested_global_xy_size;
    *pbuffer.offset(16 as libc::c_int as isize) = (*pdata).system__sequence_config;
    *pbuffer
        .offset(
            17 as libc::c_int as isize,
        ) = ((*pdata).system__grouped_parameter_hold as libc::c_int & 0x3 as libc::c_int)
        as u8;
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_i2c_decode_dynamic_config(
    mut buf_size: u16,
    mut pbuffer: *mut u8,
    mut pdata: *mut VL53LX_dynamic_config_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    if (buf_size as libc::c_int) < 18 as libc::c_int {
        return -(10 as libc::c_int) as VL53LX_Error;
    }
    (*pdata)
        .system__grouped_parameter_hold_0 = (*pbuffer.offset(0 as libc::c_int as isize)
        as libc::c_int & 0x3 as libc::c_int) as u8;
    (*pdata)
        .system__thresh_high = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(1 as libc::c_int as isize),
    );
    (*pdata)
        .system__thresh_low = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(3 as libc::c_int as isize),
    );
    (*pdata)
        .system__enable_xtalk_per_quadrant = (*pbuffer.offset(5 as libc::c_int as isize)
        as libc::c_int & 0x1 as libc::c_int) as u8;
    (*pdata)
        .system__seed_config = (*pbuffer.offset(6 as libc::c_int as isize) as libc::c_int
        & 0x7 as libc::c_int) as u8;
    (*pdata).sd_config__woi_sd0 = *pbuffer.offset(7 as libc::c_int as isize);
    (*pdata).sd_config__woi_sd1 = *pbuffer.offset(8 as libc::c_int as isize);
    (*pdata)
        .sd_config__initial_phase_sd0 = (*pbuffer.offset(9 as libc::c_int as isize)
        as libc::c_int & 0x7f as libc::c_int) as u8;
    (*pdata)
        .sd_config__initial_phase_sd1 = (*pbuffer.offset(10 as libc::c_int as isize)
        as libc::c_int & 0x7f as libc::c_int) as u8;
    (*pdata)
        .system__grouped_parameter_hold_1 = (*pbuffer.offset(11 as libc::c_int as isize)
        as libc::c_int & 0x3 as libc::c_int) as u8;
    (*pdata)
        .sd_config__first_order_select = (*pbuffer.offset(12 as libc::c_int as isize)
        as libc::c_int & 0x3 as libc::c_int) as u8;
    (*pdata)
        .sd_config__quantifier = (*pbuffer.offset(13 as libc::c_int as isize)
        as libc::c_int & 0xf as libc::c_int) as u8;
    (*pdata)
        .roi_config__user_roi_centre_spad = *pbuffer.offset(14 as libc::c_int as isize);
    (*pdata)
        .roi_config__user_roi_requested_global_xy_size = *pbuffer
        .offset(15 as libc::c_int as isize);
    (*pdata).system__sequence_config = *pbuffer.offset(16 as libc::c_int as isize);
    (*pdata)
        .system__grouped_parameter_hold = (*pbuffer.offset(17 as libc::c_int as isize)
        as libc::c_int & 0x3 as libc::c_int) as u8;
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_set_dynamic_config(
    mut Dev: VL53LX_DEV,
    mut pdata: *mut VL53LX_dynamic_config_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut comms_buffer: [u8; 18] = [0; 18];
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_i2c_encode_dynamic_config(
            pdata,
            18 as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_WriteMulti(
            Dev,
            0x71 as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
            18 as libc::c_int as u32,
        );
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_get_dynamic_config(
    mut Dev: VL53LX_DEV,
    mut pdata: *mut VL53LX_dynamic_config_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut comms_buffer: [u8; 18] = [0; 18];
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_ReadMulti(
            Dev,
            0x71 as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
            18 as libc::c_int as u32,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_i2c_decode_dynamic_config(
            18 as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
            pdata,
        );
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_i2c_encode_system_control(
    mut pdata: *mut VL53LX_system_control_t,
    mut buf_size: u16,
    mut pbuffer: *mut u8,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    if (buf_size as libc::c_int) < 5 as libc::c_int {
        return -(10 as libc::c_int) as VL53LX_Error;
    }
    *pbuffer
        .offset(
            0 as libc::c_int as isize,
        ) = ((*pdata).power_management__go1_power_force as libc::c_int
        & 0x1 as libc::c_int) as u8;
    *pbuffer
        .offset(
            1 as libc::c_int as isize,
        ) = ((*pdata).system__stream_count_ctrl as libc::c_int & 0x1 as libc::c_int)
        as u8;
    *pbuffer
        .offset(
            2 as libc::c_int as isize,
        ) = ((*pdata).firmware__enable as libc::c_int & 0x1 as libc::c_int) as u8;
    *pbuffer
        .offset(
            3 as libc::c_int as isize,
        ) = ((*pdata).system__interrupt_clear as libc::c_int & 0x3 as libc::c_int)
        as u8;
    *pbuffer.offset(4 as libc::c_int as isize) = (*pdata).system__mode_start;
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_i2c_decode_system_control(
    mut buf_size: u16,
    mut pbuffer: *mut u8,
    mut pdata: *mut VL53LX_system_control_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    if (buf_size as libc::c_int) < 5 as libc::c_int {
        return -(10 as libc::c_int) as VL53LX_Error;
    }
    (*pdata)
        .power_management__go1_power_force = (*pbuffer.offset(0 as libc::c_int as isize)
        as libc::c_int & 0x1 as libc::c_int) as u8;
    (*pdata)
        .system__stream_count_ctrl = (*pbuffer.offset(1 as libc::c_int as isize)
        as libc::c_int & 0x1 as libc::c_int) as u8;
    (*pdata)
        .firmware__enable = (*pbuffer.offset(2 as libc::c_int as isize) as libc::c_int
        & 0x1 as libc::c_int) as u8;
    (*pdata)
        .system__interrupt_clear = (*pbuffer.offset(3 as libc::c_int as isize)
        as libc::c_int & 0x3 as libc::c_int) as u8;
    (*pdata).system__mode_start = *pbuffer.offset(4 as libc::c_int as isize);
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_set_system_control(
    mut Dev: VL53LX_DEV,
    mut pdata: *mut VL53LX_system_control_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut comms_buffer: [u8; 5] = [0; 5];
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_i2c_encode_system_control(
            pdata,
            5 as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_WriteMulti(
            Dev,
            0x83 as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
            5 as libc::c_int as u32,
        );
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_get_system_control(
    mut Dev: VL53LX_DEV,
    mut pdata: *mut VL53LX_system_control_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut comms_buffer: [u8; 5] = [0; 5];
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_ReadMulti(
            Dev,
            0x83 as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
            5 as libc::c_int as u32,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_i2c_decode_system_control(
            5 as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
            pdata,
        );
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_i2c_encode_system_results(
    mut pdata: *mut VL53LX_system_results_t,
    mut buf_size: u16,
    mut pbuffer: *mut u8,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    if (buf_size as libc::c_int) < 44 as libc::c_int {
        return -(10 as libc::c_int) as VL53LX_Error;
    }
    *pbuffer
        .offset(
            0 as libc::c_int as isize,
        ) = ((*pdata).result__interrupt_status as libc::c_int & 0x3f as libc::c_int)
        as u8;
    *pbuffer.offset(1 as libc::c_int as isize) = (*pdata).result__range_status;
    *pbuffer
        .offset(
            2 as libc::c_int as isize,
        ) = ((*pdata).result__report_status as libc::c_int & 0xf as libc::c_int)
        as u8;
    *pbuffer.offset(3 as libc::c_int as isize) = (*pdata).result__stream_count;
    VL53LX_i2c_encode_uint16_t(
        (*pdata).result__dss_actual_effective_spads_sd0,
        2 as libc::c_int as u16,
        pbuffer.offset(4 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint16_t(
        (*pdata).result__peak_signal_count_rate_mcps_sd0,
        2 as libc::c_int as u16,
        pbuffer.offset(6 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint16_t(
        (*pdata).result__ambient_count_rate_mcps_sd0,
        2 as libc::c_int as u16,
        pbuffer.offset(8 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint16_t(
        (*pdata).result__sigma_sd0,
        2 as libc::c_int as u16,
        pbuffer.offset(10 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint16_t(
        (*pdata).result__phase_sd0,
        2 as libc::c_int as u16,
        pbuffer.offset(12 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint16_t(
        (*pdata).result__final_crosstalk_corrected_range_mm_sd0,
        2 as libc::c_int as u16,
        pbuffer.offset(14 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint16_t(
        (*pdata).result__peak_signal_count_rate_crosstalk_corrected_mcps_sd0,
        2 as libc::c_int as u16,
        pbuffer.offset(16 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint16_t(
        (*pdata).result__mm_inner_actual_effective_spads_sd0,
        2 as libc::c_int as u16,
        pbuffer.offset(18 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint16_t(
        (*pdata).result__mm_outer_actual_effective_spads_sd0,
        2 as libc::c_int as u16,
        pbuffer.offset(20 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint16_t(
        (*pdata).result__avg_signal_count_rate_mcps_sd0,
        2 as libc::c_int as u16,
        pbuffer.offset(22 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint16_t(
        (*pdata).result__dss_actual_effective_spads_sd1,
        2 as libc::c_int as u16,
        pbuffer.offset(24 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint16_t(
        (*pdata).result__peak_signal_count_rate_mcps_sd1,
        2 as libc::c_int as u16,
        pbuffer.offset(26 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint16_t(
        (*pdata).result__ambient_count_rate_mcps_sd1,
        2 as libc::c_int as u16,
        pbuffer.offset(28 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint16_t(
        (*pdata).result__sigma_sd1,
        2 as libc::c_int as u16,
        pbuffer.offset(30 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint16_t(
        (*pdata).result__phase_sd1,
        2 as libc::c_int as u16,
        pbuffer.offset(32 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint16_t(
        (*pdata).result__final_crosstalk_corrected_range_mm_sd1,
        2 as libc::c_int as u16,
        pbuffer.offset(34 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint16_t(
        (*pdata).result__spare_0_sd1,
        2 as libc::c_int as u16,
        pbuffer.offset(36 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint16_t(
        (*pdata).result__spare_1_sd1,
        2 as libc::c_int as u16,
        pbuffer.offset(38 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint16_t(
        (*pdata).result__spare_2_sd1,
        2 as libc::c_int as u16,
        pbuffer.offset(40 as libc::c_int as isize),
    );
    *pbuffer.offset(42 as libc::c_int as isize) = (*pdata).result__spare_3_sd1;
    *pbuffer.offset(43 as libc::c_int as isize) = (*pdata).result__thresh_info;
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_i2c_decode_system_results(
    mut buf_size: u16,
    mut pbuffer: *mut u8,
    mut pdata: *mut VL53LX_system_results_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    if (buf_size as libc::c_int) < 44 as libc::c_int {
        return -(10 as libc::c_int) as VL53LX_Error;
    }
    (*pdata)
        .result__interrupt_status = (*pbuffer.offset(0 as libc::c_int as isize)
        as libc::c_int & 0x3f as libc::c_int) as u8;
    (*pdata).result__range_status = *pbuffer.offset(1 as libc::c_int as isize);
    (*pdata)
        .result__report_status = (*pbuffer.offset(2 as libc::c_int as isize)
        as libc::c_int & 0xf as libc::c_int) as u8;
    (*pdata).result__stream_count = *pbuffer.offset(3 as libc::c_int as isize);
    (*pdata)
        .result__dss_actual_effective_spads_sd0 = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(4 as libc::c_int as isize),
    );
    (*pdata)
        .result__peak_signal_count_rate_mcps_sd0 = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(6 as libc::c_int as isize),
    );
    (*pdata)
        .result__ambient_count_rate_mcps_sd0 = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(8 as libc::c_int as isize),
    );
    (*pdata)
        .result__sigma_sd0 = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(10 as libc::c_int as isize),
    );
    (*pdata)
        .result__phase_sd0 = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(12 as libc::c_int as isize),
    );
    (*pdata)
        .result__final_crosstalk_corrected_range_mm_sd0 = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(14 as libc::c_int as isize),
    );
    (*pdata)
        .result__peak_signal_count_rate_crosstalk_corrected_mcps_sd0 = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(16 as libc::c_int as isize),
    );
    (*pdata)
        .result__mm_inner_actual_effective_spads_sd0 = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(18 as libc::c_int as isize),
    );
    (*pdata)
        .result__mm_outer_actual_effective_spads_sd0 = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(20 as libc::c_int as isize),
    );
    (*pdata)
        .result__avg_signal_count_rate_mcps_sd0 = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(22 as libc::c_int as isize),
    );
    (*pdata)
        .result__dss_actual_effective_spads_sd1 = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(24 as libc::c_int as isize),
    );
    (*pdata)
        .result__peak_signal_count_rate_mcps_sd1 = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(26 as libc::c_int as isize),
    );
    (*pdata)
        .result__ambient_count_rate_mcps_sd1 = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(28 as libc::c_int as isize),
    );
    (*pdata)
        .result__sigma_sd1 = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(30 as libc::c_int as isize),
    );
    (*pdata)
        .result__phase_sd1 = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(32 as libc::c_int as isize),
    );
    (*pdata)
        .result__final_crosstalk_corrected_range_mm_sd1 = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(34 as libc::c_int as isize),
    );
    (*pdata)
        .result__spare_0_sd1 = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(36 as libc::c_int as isize),
    );
    (*pdata)
        .result__spare_1_sd1 = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(38 as libc::c_int as isize),
    );
    (*pdata)
        .result__spare_2_sd1 = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(40 as libc::c_int as isize),
    );
    (*pdata).result__spare_3_sd1 = *pbuffer.offset(42 as libc::c_int as isize);
    (*pdata).result__thresh_info = *pbuffer.offset(43 as libc::c_int as isize);
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_set_system_results(
    mut Dev: VL53LX_DEV,
    mut pdata: *mut VL53LX_system_results_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut comms_buffer: [u8; 44] = [0; 44];
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_i2c_encode_system_results(
            pdata,
            44 as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_WriteMulti(
            Dev,
            0x88 as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
            44 as libc::c_int as u32,
        );
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_get_system_results(
    mut Dev: VL53LX_DEV,
    mut pdata: *mut VL53LX_system_results_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut comms_buffer: [u8; 44] = [0; 44];
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_ReadMulti(
            Dev,
            0x88 as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
            44 as libc::c_int as u32,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_i2c_decode_system_results(
            44 as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
            pdata,
        );
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_i2c_encode_core_results(
    mut pdata: *mut VL53LX_core_results_t,
    mut buf_size: u16,
    mut pbuffer: *mut u8,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    if (buf_size as libc::c_int) < 33 as libc::c_int {
        return -(10 as libc::c_int) as VL53LX_Error;
    }
    VL53LX_i2c_encode_uint32_t(
        (*pdata).result_core__ambient_window_events_sd0,
        4 as libc::c_int as u16,
        pbuffer.offset(0 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint32_t(
        (*pdata).result_core__ranging_total_events_sd0,
        4 as libc::c_int as u16,
        pbuffer.offset(4 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_int32_t(
        (*pdata).result_core__signal_total_events_sd0,
        4 as libc::c_int as u16,
        pbuffer.offset(8 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint32_t(
        (*pdata).result_core__total_periods_elapsed_sd0,
        4 as libc::c_int as u16,
        pbuffer.offset(12 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint32_t(
        (*pdata).result_core__ambient_window_events_sd1,
        4 as libc::c_int as u16,
        pbuffer.offset(16 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint32_t(
        (*pdata).result_core__ranging_total_events_sd1,
        4 as libc::c_int as u16,
        pbuffer.offset(20 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_int32_t(
        (*pdata).result_core__signal_total_events_sd1,
        4 as libc::c_int as u16,
        pbuffer.offset(24 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint32_t(
        (*pdata).result_core__total_periods_elapsed_sd1,
        4 as libc::c_int as u16,
        pbuffer.offset(28 as libc::c_int as isize),
    );
    *pbuffer.offset(32 as libc::c_int as isize) = (*pdata).result_core__spare_0;
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_i2c_decode_core_results(
    mut buf_size: u16,
    mut pbuffer: *mut u8,
    mut pdata: *mut VL53LX_core_results_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    if (buf_size as libc::c_int) < 33 as libc::c_int {
        return -(10 as libc::c_int) as VL53LX_Error;
    }
    (*pdata)
        .result_core__ambient_window_events_sd0 = VL53LX_i2c_decode_uint32_t(
        4 as libc::c_int as u16,
        pbuffer.offset(0 as libc::c_int as isize),
    );
    (*pdata)
        .result_core__ranging_total_events_sd0 = VL53LX_i2c_decode_uint32_t(
        4 as libc::c_int as u16,
        pbuffer.offset(4 as libc::c_int as isize),
    );
    (*pdata)
        .result_core__signal_total_events_sd0 = VL53LX_i2c_decode_int32_t(
        4 as libc::c_int as u16,
        pbuffer.offset(8 as libc::c_int as isize),
    );
    (*pdata)
        .result_core__total_periods_elapsed_sd0 = VL53LX_i2c_decode_uint32_t(
        4 as libc::c_int as u16,
        pbuffer.offset(12 as libc::c_int as isize),
    );
    (*pdata)
        .result_core__ambient_window_events_sd1 = VL53LX_i2c_decode_uint32_t(
        4 as libc::c_int as u16,
        pbuffer.offset(16 as libc::c_int as isize),
    );
    (*pdata)
        .result_core__ranging_total_events_sd1 = VL53LX_i2c_decode_uint32_t(
        4 as libc::c_int as u16,
        pbuffer.offset(20 as libc::c_int as isize),
    );
    (*pdata)
        .result_core__signal_total_events_sd1 = VL53LX_i2c_decode_int32_t(
        4 as libc::c_int as u16,
        pbuffer.offset(24 as libc::c_int as isize),
    );
    (*pdata)
        .result_core__total_periods_elapsed_sd1 = VL53LX_i2c_decode_uint32_t(
        4 as libc::c_int as u16,
        pbuffer.offset(28 as libc::c_int as isize),
    );
    (*pdata).result_core__spare_0 = *pbuffer.offset(32 as libc::c_int as isize);
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_set_core_results(
    mut Dev: VL53LX_DEV,
    mut pdata: *mut VL53LX_core_results_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut comms_buffer: [u8; 33] = [0; 33];
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_i2c_encode_core_results(
            pdata,
            33 as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_disable_firmware(Dev);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_WriteMulti(
            Dev,
            0xb4 as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
            33 as libc::c_int as u32,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_enable_firmware(Dev);
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_get_core_results(
    mut Dev: VL53LX_DEV,
    mut pdata: *mut VL53LX_core_results_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut comms_buffer: [u8; 33] = [0; 33];
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_ReadMulti(
            Dev,
            0xb4 as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
            33 as libc::c_int as u32,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_i2c_decode_core_results(
            33 as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
            pdata,
        );
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_i2c_encode_debug_results(
    mut pdata: *mut VL53LX_debug_results_t,
    mut buf_size: u16,
    mut pbuffer: *mut u8,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    if (buf_size as libc::c_int) < 56 as libc::c_int {
        return -(10 as libc::c_int) as VL53LX_Error;
    }
    VL53LX_i2c_encode_uint16_t(
        (*pdata).phasecal_result__reference_phase,
        2 as libc::c_int as u16,
        pbuffer.offset(0 as libc::c_int as isize),
    );
    *pbuffer
        .offset(
            2 as libc::c_int as isize,
        ) = ((*pdata).phasecal_result__vcsel_start as libc::c_int & 0x7f as libc::c_int)
        as u8;
    *pbuffer
        .offset(
            3 as libc::c_int as isize,
        ) = ((*pdata).ref_spad_char_result__num_actual_ref_spads as libc::c_int
        & 0x3f as libc::c_int) as u8;
    *pbuffer
        .offset(
            4 as libc::c_int as isize,
        ) = ((*pdata).ref_spad_char_result__ref_location as libc::c_int
        & 0x3 as libc::c_int) as u8;
    *pbuffer
        .offset(
            5 as libc::c_int as isize,
        ) = ((*pdata).vhv_result__coldboot_status as libc::c_int & 0x1 as libc::c_int)
        as u8;
    *pbuffer
        .offset(
            6 as libc::c_int as isize,
        ) = ((*pdata).vhv_result__search_result as libc::c_int & 0x3f as libc::c_int)
        as u8;
    *pbuffer
        .offset(
            7 as libc::c_int as isize,
        ) = ((*pdata).vhv_result__latest_setting as libc::c_int & 0x3f as libc::c_int)
        as u8;
    VL53LX_i2c_encode_uint16_t(
        ((*pdata).result__osc_calibrate_val as libc::c_int & 0x3ff as libc::c_int)
            as u16,
        2 as libc::c_int as u16,
        pbuffer.offset(8 as libc::c_int as isize),
    );
    *pbuffer
        .offset(
            10 as libc::c_int as isize,
        ) = ((*pdata).ana_config__powerdown_go1 as libc::c_int & 0x3 as libc::c_int)
        as u8;
    *pbuffer
        .offset(
            11 as libc::c_int as isize,
        ) = ((*pdata).ana_config__ref_bg_ctrl as libc::c_int & 0x3 as libc::c_int)
        as u8;
    *pbuffer
        .offset(
            12 as libc::c_int as isize,
        ) = ((*pdata).ana_config__regdvdd1v2_ctrl as libc::c_int & 0xf as libc::c_int)
        as u8;
    *pbuffer
        .offset(
            13 as libc::c_int as isize,
        ) = ((*pdata).ana_config__osc_slow_ctrl as libc::c_int & 0x7 as libc::c_int)
        as u8;
    *pbuffer
        .offset(
            14 as libc::c_int as isize,
        ) = ((*pdata).test_mode__status as libc::c_int & 0x1 as libc::c_int) as u8;
    *pbuffer
        .offset(
            15 as libc::c_int as isize,
        ) = ((*pdata).firmware__system_status as libc::c_int & 0x3 as libc::c_int)
        as u8;
    *pbuffer.offset(16 as libc::c_int as isize) = (*pdata).firmware__mode_status;
    *pbuffer
        .offset(17 as libc::c_int as isize) = (*pdata).firmware__secondary_mode_status;
    VL53LX_i2c_encode_uint16_t(
        ((*pdata).firmware__cal_repeat_rate_counter as libc::c_int
            & 0xfff as libc::c_int) as u16,
        2 as libc::c_int as u16,
        pbuffer.offset(18 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint16_t(
        (*pdata).gph__system__thresh_high,
        2 as libc::c_int as u16,
        pbuffer.offset(22 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint16_t(
        (*pdata).gph__system__thresh_low,
        2 as libc::c_int as u16,
        pbuffer.offset(24 as libc::c_int as isize),
    );
    *pbuffer
        .offset(
            26 as libc::c_int as isize,
        ) = ((*pdata).gph__system__enable_xtalk_per_quadrant as libc::c_int
        & 0x1 as libc::c_int) as u8;
    *pbuffer
        .offset(
            27 as libc::c_int as isize,
        ) = ((*pdata).gph__spare_0 as libc::c_int & 0x7 as libc::c_int) as u8;
    *pbuffer.offset(28 as libc::c_int as isize) = (*pdata).gph__sd_config__woi_sd0;
    *pbuffer.offset(29 as libc::c_int as isize) = (*pdata).gph__sd_config__woi_sd1;
    *pbuffer
        .offset(
            30 as libc::c_int as isize,
        ) = ((*pdata).gph__sd_config__initial_phase_sd0 as libc::c_int
        & 0x7f as libc::c_int) as u8;
    *pbuffer
        .offset(
            31 as libc::c_int as isize,
        ) = ((*pdata).gph__sd_config__initial_phase_sd1 as libc::c_int
        & 0x7f as libc::c_int) as u8;
    *pbuffer
        .offset(
            32 as libc::c_int as isize,
        ) = ((*pdata).gph__sd_config__first_order_select as libc::c_int
        & 0x3 as libc::c_int) as u8;
    *pbuffer
        .offset(
            33 as libc::c_int as isize,
        ) = ((*pdata).gph__sd_config__quantifier as libc::c_int & 0xf as libc::c_int)
        as u8;
    *pbuffer
        .offset(
            34 as libc::c_int as isize,
        ) = (*pdata).gph__roi_config__user_roi_centre_spad;
    *pbuffer
        .offset(
            35 as libc::c_int as isize,
        ) = (*pdata).gph__roi_config__user_roi_requested_global_xy_size;
    *pbuffer.offset(36 as libc::c_int as isize) = (*pdata).gph__system__sequence_config;
    *pbuffer
        .offset(
            37 as libc::c_int as isize,
        ) = ((*pdata).gph__gph_id as libc::c_int & 0x1 as libc::c_int) as u8;
    *pbuffer
        .offset(
            38 as libc::c_int as isize,
        ) = ((*pdata).system__interrupt_set as libc::c_int & 0x3 as libc::c_int)
        as u8;
    *pbuffer
        .offset(
            39 as libc::c_int as isize,
        ) = ((*pdata).interrupt_manager__enables as libc::c_int & 0x1f as libc::c_int)
        as u8;
    *pbuffer
        .offset(
            40 as libc::c_int as isize,
        ) = ((*pdata).interrupt_manager__clear as libc::c_int & 0x1f as libc::c_int)
        as u8;
    *pbuffer
        .offset(
            41 as libc::c_int as isize,
        ) = ((*pdata).interrupt_manager__status as libc::c_int & 0x1f as libc::c_int)
        as u8;
    *pbuffer
        .offset(
            42 as libc::c_int as isize,
        ) = ((*pdata).mcu_to_host_bank__wr_access_en as libc::c_int & 0x1 as libc::c_int)
        as u8;
    *pbuffer
        .offset(
            43 as libc::c_int as isize,
        ) = ((*pdata).power_management__go1_reset_status as libc::c_int
        & 0x1 as libc::c_int) as u8;
    *pbuffer
        .offset(
            44 as libc::c_int as isize,
        ) = ((*pdata).pad_startup_mode__value_ro as libc::c_int & 0x3 as libc::c_int)
        as u8;
    *pbuffer
        .offset(
            45 as libc::c_int as isize,
        ) = ((*pdata).pad_startup_mode__value_ctrl as libc::c_int & 0x3f as libc::c_int)
        as u8;
    VL53LX_i2c_encode_uint32_t(
        (*pdata).pll_period_us & 0x3ffff as libc::c_int as libc::c_uint,
        4 as libc::c_int as u16,
        pbuffer.offset(46 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint32_t(
        (*pdata).interrupt_scheduler__data_out,
        4 as libc::c_int as u16,
        pbuffer.offset(50 as libc::c_int as isize),
    );
    *pbuffer
        .offset(
            54 as libc::c_int as isize,
        ) = ((*pdata).nvm_bist__complete as libc::c_int & 0x1 as libc::c_int) as u8;
    *pbuffer
        .offset(
            55 as libc::c_int as isize,
        ) = ((*pdata).nvm_bist__status as libc::c_int & 0x1 as libc::c_int) as u8;
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_i2c_decode_debug_results(
    mut buf_size: u16,
    mut pbuffer: *mut u8,
    mut pdata: *mut VL53LX_debug_results_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    if (buf_size as libc::c_int) < 56 as libc::c_int {
        return -(10 as libc::c_int) as VL53LX_Error;
    }
    (*pdata)
        .phasecal_result__reference_phase = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(0 as libc::c_int as isize),
    );
    (*pdata)
        .phasecal_result__vcsel_start = (*pbuffer.offset(2 as libc::c_int as isize)
        as libc::c_int & 0x7f as libc::c_int) as u8;
    (*pdata)
        .ref_spad_char_result__num_actual_ref_spads = (*pbuffer
        .offset(3 as libc::c_int as isize) as libc::c_int & 0x3f as libc::c_int)
        as u8;
    (*pdata)
        .ref_spad_char_result__ref_location = (*pbuffer.offset(4 as libc::c_int as isize)
        as libc::c_int & 0x3 as libc::c_int) as u8;
    (*pdata)
        .vhv_result__coldboot_status = (*pbuffer.offset(5 as libc::c_int as isize)
        as libc::c_int & 0x1 as libc::c_int) as u8;
    (*pdata)
        .vhv_result__search_result = (*pbuffer.offset(6 as libc::c_int as isize)
        as libc::c_int & 0x3f as libc::c_int) as u8;
    (*pdata)
        .vhv_result__latest_setting = (*pbuffer.offset(7 as libc::c_int as isize)
        as libc::c_int & 0x3f as libc::c_int) as u8;
    (*pdata)
        .result__osc_calibrate_val = (VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(8 as libc::c_int as isize),
    ) as libc::c_int & 0x3ff as libc::c_int) as u16;
    (*pdata)
        .ana_config__powerdown_go1 = (*pbuffer.offset(10 as libc::c_int as isize)
        as libc::c_int & 0x3 as libc::c_int) as u8;
    (*pdata)
        .ana_config__ref_bg_ctrl = (*pbuffer.offset(11 as libc::c_int as isize)
        as libc::c_int & 0x3 as libc::c_int) as u8;
    (*pdata)
        .ana_config__regdvdd1v2_ctrl = (*pbuffer.offset(12 as libc::c_int as isize)
        as libc::c_int & 0xf as libc::c_int) as u8;
    (*pdata)
        .ana_config__osc_slow_ctrl = (*pbuffer.offset(13 as libc::c_int as isize)
        as libc::c_int & 0x7 as libc::c_int) as u8;
    (*pdata)
        .test_mode__status = (*pbuffer.offset(14 as libc::c_int as isize) as libc::c_int
        & 0x1 as libc::c_int) as u8;
    (*pdata)
        .firmware__system_status = (*pbuffer.offset(15 as libc::c_int as isize)
        as libc::c_int & 0x3 as libc::c_int) as u8;
    (*pdata).firmware__mode_status = *pbuffer.offset(16 as libc::c_int as isize);
    (*pdata)
        .firmware__secondary_mode_status = *pbuffer.offset(17 as libc::c_int as isize);
    (*pdata)
        .firmware__cal_repeat_rate_counter = (VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(18 as libc::c_int as isize),
    ) as libc::c_int & 0xfff as libc::c_int) as u16;
    (*pdata)
        .gph__system__thresh_high = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(22 as libc::c_int as isize),
    );
    (*pdata)
        .gph__system__thresh_low = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(24 as libc::c_int as isize),
    );
    (*pdata)
        .gph__system__enable_xtalk_per_quadrant = (*pbuffer
        .offset(26 as libc::c_int as isize) as libc::c_int & 0x1 as libc::c_int)
        as u8;
    (*pdata)
        .gph__spare_0 = (*pbuffer.offset(27 as libc::c_int as isize) as libc::c_int
        & 0x7 as libc::c_int) as u8;
    (*pdata).gph__sd_config__woi_sd0 = *pbuffer.offset(28 as libc::c_int as isize);
    (*pdata).gph__sd_config__woi_sd1 = *pbuffer.offset(29 as libc::c_int as isize);
    (*pdata)
        .gph__sd_config__initial_phase_sd0 = (*pbuffer.offset(30 as libc::c_int as isize)
        as libc::c_int & 0x7f as libc::c_int) as u8;
    (*pdata)
        .gph__sd_config__initial_phase_sd1 = (*pbuffer.offset(31 as libc::c_int as isize)
        as libc::c_int & 0x7f as libc::c_int) as u8;
    (*pdata)
        .gph__sd_config__first_order_select = (*pbuffer
        .offset(32 as libc::c_int as isize) as libc::c_int & 0x3 as libc::c_int)
        as u8;
    (*pdata)
        .gph__sd_config__quantifier = (*pbuffer.offset(33 as libc::c_int as isize)
        as libc::c_int & 0xf as libc::c_int) as u8;
    (*pdata)
        .gph__roi_config__user_roi_centre_spad = *pbuffer
        .offset(34 as libc::c_int as isize);
    (*pdata)
        .gph__roi_config__user_roi_requested_global_xy_size = *pbuffer
        .offset(35 as libc::c_int as isize);
    (*pdata).gph__system__sequence_config = *pbuffer.offset(36 as libc::c_int as isize);
    (*pdata)
        .gph__gph_id = (*pbuffer.offset(37 as libc::c_int as isize) as libc::c_int
        & 0x1 as libc::c_int) as u8;
    (*pdata)
        .system__interrupt_set = (*pbuffer.offset(38 as libc::c_int as isize)
        as libc::c_int & 0x3 as libc::c_int) as u8;
    (*pdata)
        .interrupt_manager__enables = (*pbuffer.offset(39 as libc::c_int as isize)
        as libc::c_int & 0x1f as libc::c_int) as u8;
    (*pdata)
        .interrupt_manager__clear = (*pbuffer.offset(40 as libc::c_int as isize)
        as libc::c_int & 0x1f as libc::c_int) as u8;
    (*pdata)
        .interrupt_manager__status = (*pbuffer.offset(41 as libc::c_int as isize)
        as libc::c_int & 0x1f as libc::c_int) as u8;
    (*pdata)
        .mcu_to_host_bank__wr_access_en = (*pbuffer.offset(42 as libc::c_int as isize)
        as libc::c_int & 0x1 as libc::c_int) as u8;
    (*pdata)
        .power_management__go1_reset_status = (*pbuffer
        .offset(43 as libc::c_int as isize) as libc::c_int & 0x1 as libc::c_int)
        as u8;
    (*pdata)
        .pad_startup_mode__value_ro = (*pbuffer.offset(44 as libc::c_int as isize)
        as libc::c_int & 0x3 as libc::c_int) as u8;
    (*pdata)
        .pad_startup_mode__value_ctrl = (*pbuffer.offset(45 as libc::c_int as isize)
        as libc::c_int & 0x3f as libc::c_int) as u8;
    (*pdata)
        .pll_period_us = VL53LX_i2c_decode_uint32_t(
        4 as libc::c_int as u16,
        pbuffer.offset(46 as libc::c_int as isize),
    ) & 0x3ffff as libc::c_int as libc::c_uint;
    (*pdata)
        .interrupt_scheduler__data_out = VL53LX_i2c_decode_uint32_t(
        4 as libc::c_int as u16,
        pbuffer.offset(50 as libc::c_int as isize),
    );
    (*pdata)
        .nvm_bist__complete = (*pbuffer.offset(54 as libc::c_int as isize) as libc::c_int
        & 0x1 as libc::c_int) as u8;
    (*pdata)
        .nvm_bist__status = (*pbuffer.offset(55 as libc::c_int as isize) as libc::c_int
        & 0x1 as libc::c_int) as u8;
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_set_debug_results(
    mut Dev: VL53LX_DEV,
    mut pdata: *mut VL53LX_debug_results_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut comms_buffer: [u8; 56] = [0; 56];
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_i2c_encode_debug_results(
            pdata,
            56 as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_disable_firmware(Dev);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_WriteMulti(
            Dev,
            0xd6 as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
            56 as libc::c_int as u32,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_enable_firmware(Dev);
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_get_debug_results(
    mut Dev: VL53LX_DEV,
    mut pdata: *mut VL53LX_debug_results_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut comms_buffer: [u8; 56] = [0; 56];
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_ReadMulti(
            Dev,
            0xd6 as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
            56 as libc::c_int as u32,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_i2c_decode_debug_results(
            56 as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
            pdata,
        );
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_i2c_encode_nvm_copy_data(
    mut pdata: *mut VL53LX_nvm_copy_data_t,
    mut buf_size: u16,
    mut pbuffer: *mut u8,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    if (buf_size as libc::c_int) < 49 as libc::c_int {
        return -(10 as libc::c_int) as VL53LX_Error;
    }
    *pbuffer.offset(0 as libc::c_int as isize) = (*pdata).identification__model_id;
    *pbuffer.offset(1 as libc::c_int as isize) = (*pdata).identification__module_type;
    *pbuffer.offset(2 as libc::c_int as isize) = (*pdata).identification__revision_id;
    VL53LX_i2c_encode_uint16_t(
        (*pdata).identification__module_id,
        2 as libc::c_int as u16,
        pbuffer.offset(3 as libc::c_int as isize),
    );
    *pbuffer
        .offset(
            5 as libc::c_int as isize,
        ) = ((*pdata).ana_config__fast_osc__trim_max as libc::c_int
        & 0x7f as libc::c_int) as u8;
    *pbuffer
        .offset(
            6 as libc::c_int as isize,
        ) = ((*pdata).ana_config__fast_osc__freq_set as libc::c_int & 0x7 as libc::c_int)
        as u8;
    *pbuffer
        .offset(
            7 as libc::c_int as isize,
        ) = ((*pdata).ana_config__vcsel_trim as libc::c_int & 0x7 as libc::c_int)
        as u8;
    *pbuffer
        .offset(
            8 as libc::c_int as isize,
        ) = ((*pdata).ana_config__vcsel_selion as libc::c_int & 0x3f as libc::c_int)
        as u8;
    *pbuffer
        .offset(
            9 as libc::c_int as isize,
        ) = ((*pdata).ana_config__vcsel_selion_max as libc::c_int & 0x3f as libc::c_int)
        as u8;
    *pbuffer
        .offset(
            10 as libc::c_int as isize,
        ) = ((*pdata).protected_laser_safety__lock_bit as libc::c_int
        & 0x1 as libc::c_int) as u8;
    *pbuffer
        .offset(
            11 as libc::c_int as isize,
        ) = ((*pdata).laser_safety__key as libc::c_int & 0x7f as libc::c_int) as u8;
    *pbuffer
        .offset(
            12 as libc::c_int as isize,
        ) = ((*pdata).laser_safety__key_ro as libc::c_int & 0x1 as libc::c_int)
        as u8;
    *pbuffer
        .offset(
            13 as libc::c_int as isize,
        ) = ((*pdata).laser_safety__clip as libc::c_int & 0x3f as libc::c_int)
        as u8;
    *pbuffer
        .offset(
            14 as libc::c_int as isize,
        ) = ((*pdata).laser_safety__mult as libc::c_int & 0x3f as libc::c_int)
        as u8;
    *pbuffer
        .offset(15 as libc::c_int as isize) = (*pdata).global_config__spad_enables_rtn_0;
    *pbuffer
        .offset(16 as libc::c_int as isize) = (*pdata).global_config__spad_enables_rtn_1;
    *pbuffer
        .offset(17 as libc::c_int as isize) = (*pdata).global_config__spad_enables_rtn_2;
    *pbuffer
        .offset(18 as libc::c_int as isize) = (*pdata).global_config__spad_enables_rtn_3;
    *pbuffer
        .offset(19 as libc::c_int as isize) = (*pdata).global_config__spad_enables_rtn_4;
    *pbuffer
        .offset(20 as libc::c_int as isize) = (*pdata).global_config__spad_enables_rtn_5;
    *pbuffer
        .offset(21 as libc::c_int as isize) = (*pdata).global_config__spad_enables_rtn_6;
    *pbuffer
        .offset(22 as libc::c_int as isize) = (*pdata).global_config__spad_enables_rtn_7;
    *pbuffer
        .offset(23 as libc::c_int as isize) = (*pdata).global_config__spad_enables_rtn_8;
    *pbuffer
        .offset(24 as libc::c_int as isize) = (*pdata).global_config__spad_enables_rtn_9;
    *pbuffer
        .offset(
            25 as libc::c_int as isize,
        ) = (*pdata).global_config__spad_enables_rtn_10;
    *pbuffer
        .offset(
            26 as libc::c_int as isize,
        ) = (*pdata).global_config__spad_enables_rtn_11;
    *pbuffer
        .offset(
            27 as libc::c_int as isize,
        ) = (*pdata).global_config__spad_enables_rtn_12;
    *pbuffer
        .offset(
            28 as libc::c_int as isize,
        ) = (*pdata).global_config__spad_enables_rtn_13;
    *pbuffer
        .offset(
            29 as libc::c_int as isize,
        ) = (*pdata).global_config__spad_enables_rtn_14;
    *pbuffer
        .offset(
            30 as libc::c_int as isize,
        ) = (*pdata).global_config__spad_enables_rtn_15;
    *pbuffer
        .offset(
            31 as libc::c_int as isize,
        ) = (*pdata).global_config__spad_enables_rtn_16;
    *pbuffer
        .offset(
            32 as libc::c_int as isize,
        ) = (*pdata).global_config__spad_enables_rtn_17;
    *pbuffer
        .offset(
            33 as libc::c_int as isize,
        ) = (*pdata).global_config__spad_enables_rtn_18;
    *pbuffer
        .offset(
            34 as libc::c_int as isize,
        ) = (*pdata).global_config__spad_enables_rtn_19;
    *pbuffer
        .offset(
            35 as libc::c_int as isize,
        ) = (*pdata).global_config__spad_enables_rtn_20;
    *pbuffer
        .offset(
            36 as libc::c_int as isize,
        ) = (*pdata).global_config__spad_enables_rtn_21;
    *pbuffer
        .offset(
            37 as libc::c_int as isize,
        ) = (*pdata).global_config__spad_enables_rtn_22;
    *pbuffer
        .offset(
            38 as libc::c_int as isize,
        ) = (*pdata).global_config__spad_enables_rtn_23;
    *pbuffer
        .offset(
            39 as libc::c_int as isize,
        ) = (*pdata).global_config__spad_enables_rtn_24;
    *pbuffer
        .offset(
            40 as libc::c_int as isize,
        ) = (*pdata).global_config__spad_enables_rtn_25;
    *pbuffer
        .offset(
            41 as libc::c_int as isize,
        ) = (*pdata).global_config__spad_enables_rtn_26;
    *pbuffer
        .offset(
            42 as libc::c_int as isize,
        ) = (*pdata).global_config__spad_enables_rtn_27;
    *pbuffer
        .offset(
            43 as libc::c_int as isize,
        ) = (*pdata).global_config__spad_enables_rtn_28;
    *pbuffer
        .offset(
            44 as libc::c_int as isize,
        ) = (*pdata).global_config__spad_enables_rtn_29;
    *pbuffer
        .offset(
            45 as libc::c_int as isize,
        ) = (*pdata).global_config__spad_enables_rtn_30;
    *pbuffer
        .offset(
            46 as libc::c_int as isize,
        ) = (*pdata).global_config__spad_enables_rtn_31;
    *pbuffer
        .offset(47 as libc::c_int as isize) = (*pdata).roi_config__mode_roi_centre_spad;
    *pbuffer.offset(48 as libc::c_int as isize) = (*pdata).roi_config__mode_roi_xy_size;
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_i2c_decode_nvm_copy_data(
    mut buf_size: u16,
    mut pbuffer: *mut u8,
    mut pdata: *mut VL53LX_nvm_copy_data_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    if (buf_size as libc::c_int) < 49 as libc::c_int {
        return -(10 as libc::c_int) as VL53LX_Error;
    }
    (*pdata).identification__model_id = *pbuffer.offset(0 as libc::c_int as isize);
    (*pdata).identification__module_type = *pbuffer.offset(1 as libc::c_int as isize);
    (*pdata).identification__revision_id = *pbuffer.offset(2 as libc::c_int as isize);
    (*pdata)
        .identification__module_id = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(3 as libc::c_int as isize),
    );
    (*pdata)
        .ana_config__fast_osc__trim_max = (*pbuffer.offset(5 as libc::c_int as isize)
        as libc::c_int & 0x7f as libc::c_int) as u8;
    (*pdata)
        .ana_config__fast_osc__freq_set = (*pbuffer.offset(6 as libc::c_int as isize)
        as libc::c_int & 0x7 as libc::c_int) as u8;
    (*pdata)
        .ana_config__vcsel_trim = (*pbuffer.offset(7 as libc::c_int as isize)
        as libc::c_int & 0x7 as libc::c_int) as u8;
    (*pdata)
        .ana_config__vcsel_selion = (*pbuffer.offset(8 as libc::c_int as isize)
        as libc::c_int & 0x3f as libc::c_int) as u8;
    (*pdata)
        .ana_config__vcsel_selion_max = (*pbuffer.offset(9 as libc::c_int as isize)
        as libc::c_int & 0x3f as libc::c_int) as u8;
    (*pdata)
        .protected_laser_safety__lock_bit = (*pbuffer.offset(10 as libc::c_int as isize)
        as libc::c_int & 0x1 as libc::c_int) as u8;
    (*pdata)
        .laser_safety__key = (*pbuffer.offset(11 as libc::c_int as isize) as libc::c_int
        & 0x7f as libc::c_int) as u8;
    (*pdata)
        .laser_safety__key_ro = (*pbuffer.offset(12 as libc::c_int as isize)
        as libc::c_int & 0x1 as libc::c_int) as u8;
    (*pdata)
        .laser_safety__clip = (*pbuffer.offset(13 as libc::c_int as isize) as libc::c_int
        & 0x3f as libc::c_int) as u8;
    (*pdata)
        .laser_safety__mult = (*pbuffer.offset(14 as libc::c_int as isize) as libc::c_int
        & 0x3f as libc::c_int) as u8;
    (*pdata)
        .global_config__spad_enables_rtn_0 = *pbuffer.offset(15 as libc::c_int as isize);
    (*pdata)
        .global_config__spad_enables_rtn_1 = *pbuffer.offset(16 as libc::c_int as isize);
    (*pdata)
        .global_config__spad_enables_rtn_2 = *pbuffer.offset(17 as libc::c_int as isize);
    (*pdata)
        .global_config__spad_enables_rtn_3 = *pbuffer.offset(18 as libc::c_int as isize);
    (*pdata)
        .global_config__spad_enables_rtn_4 = *pbuffer.offset(19 as libc::c_int as isize);
    (*pdata)
        .global_config__spad_enables_rtn_5 = *pbuffer.offset(20 as libc::c_int as isize);
    (*pdata)
        .global_config__spad_enables_rtn_6 = *pbuffer.offset(21 as libc::c_int as isize);
    (*pdata)
        .global_config__spad_enables_rtn_7 = *pbuffer.offset(22 as libc::c_int as isize);
    (*pdata)
        .global_config__spad_enables_rtn_8 = *pbuffer.offset(23 as libc::c_int as isize);
    (*pdata)
        .global_config__spad_enables_rtn_9 = *pbuffer.offset(24 as libc::c_int as isize);
    (*pdata)
        .global_config__spad_enables_rtn_10 = *pbuffer
        .offset(25 as libc::c_int as isize);
    (*pdata)
        .global_config__spad_enables_rtn_11 = *pbuffer
        .offset(26 as libc::c_int as isize);
    (*pdata)
        .global_config__spad_enables_rtn_12 = *pbuffer
        .offset(27 as libc::c_int as isize);
    (*pdata)
        .global_config__spad_enables_rtn_13 = *pbuffer
        .offset(28 as libc::c_int as isize);
    (*pdata)
        .global_config__spad_enables_rtn_14 = *pbuffer
        .offset(29 as libc::c_int as isize);
    (*pdata)
        .global_config__spad_enables_rtn_15 = *pbuffer
        .offset(30 as libc::c_int as isize);
    (*pdata)
        .global_config__spad_enables_rtn_16 = *pbuffer
        .offset(31 as libc::c_int as isize);
    (*pdata)
        .global_config__spad_enables_rtn_17 = *pbuffer
        .offset(32 as libc::c_int as isize);
    (*pdata)
        .global_config__spad_enables_rtn_18 = *pbuffer
        .offset(33 as libc::c_int as isize);
    (*pdata)
        .global_config__spad_enables_rtn_19 = *pbuffer
        .offset(34 as libc::c_int as isize);
    (*pdata)
        .global_config__spad_enables_rtn_20 = *pbuffer
        .offset(35 as libc::c_int as isize);
    (*pdata)
        .global_config__spad_enables_rtn_21 = *pbuffer
        .offset(36 as libc::c_int as isize);
    (*pdata)
        .global_config__spad_enables_rtn_22 = *pbuffer
        .offset(37 as libc::c_int as isize);
    (*pdata)
        .global_config__spad_enables_rtn_23 = *pbuffer
        .offset(38 as libc::c_int as isize);
    (*pdata)
        .global_config__spad_enables_rtn_24 = *pbuffer
        .offset(39 as libc::c_int as isize);
    (*pdata)
        .global_config__spad_enables_rtn_25 = *pbuffer
        .offset(40 as libc::c_int as isize);
    (*pdata)
        .global_config__spad_enables_rtn_26 = *pbuffer
        .offset(41 as libc::c_int as isize);
    (*pdata)
        .global_config__spad_enables_rtn_27 = *pbuffer
        .offset(42 as libc::c_int as isize);
    (*pdata)
        .global_config__spad_enables_rtn_28 = *pbuffer
        .offset(43 as libc::c_int as isize);
    (*pdata)
        .global_config__spad_enables_rtn_29 = *pbuffer
        .offset(44 as libc::c_int as isize);
    (*pdata)
        .global_config__spad_enables_rtn_30 = *pbuffer
        .offset(45 as libc::c_int as isize);
    (*pdata)
        .global_config__spad_enables_rtn_31 = *pbuffer
        .offset(46 as libc::c_int as isize);
    (*pdata)
        .roi_config__mode_roi_centre_spad = *pbuffer.offset(47 as libc::c_int as isize);
    (*pdata).roi_config__mode_roi_xy_size = *pbuffer.offset(48 as libc::c_int as isize);
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_set_nvm_copy_data(
    mut Dev: VL53LX_DEV,
    mut pdata: *mut VL53LX_nvm_copy_data_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut comms_buffer: [u8; 49] = [0; 49];
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_i2c_encode_nvm_copy_data(
            pdata,
            49 as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_disable_firmware(Dev);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_WriteMulti(
            Dev,
            0x10f as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
            49 as libc::c_int as u32,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_enable_firmware(Dev);
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_get_nvm_copy_data(
    mut Dev: VL53LX_DEV,
    mut pdata: *mut VL53LX_nvm_copy_data_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut comms_buffer: [u8; 49] = [0; 49];
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_ReadMulti(
            Dev,
            0x10f as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
            49 as libc::c_int as u32,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_i2c_decode_nvm_copy_data(
            49 as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
            pdata,
        );
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_i2c_encode_prev_shadow_system_results(
    mut pdata: *mut VL53LX_prev_shadow_system_results_t,
    mut buf_size: u16,
    mut pbuffer: *mut u8,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    if (buf_size as libc::c_int) < 44 as libc::c_int {
        return -(10 as libc::c_int) as VL53LX_Error;
    }
    *pbuffer
        .offset(
            0 as libc::c_int as isize,
        ) = ((*pdata).prev_shadow_result__interrupt_status as libc::c_int
        & 0x3f as libc::c_int) as u8;
    *pbuffer
        .offset(1 as libc::c_int as isize) = (*pdata).prev_shadow_result__range_status;
    *pbuffer
        .offset(
            2 as libc::c_int as isize,
        ) = ((*pdata).prev_shadow_result__report_status as libc::c_int
        & 0xf as libc::c_int) as u8;
    *pbuffer
        .offset(3 as libc::c_int as isize) = (*pdata).prev_shadow_result__stream_count;
    VL53LX_i2c_encode_uint16_t(
        (*pdata).prev_shadow_result__dss_actual_effective_spads_sd0,
        2 as libc::c_int as u16,
        pbuffer.offset(4 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint16_t(
        (*pdata).prev_shadow_result__peak_signal_count_rate_mcps_sd0,
        2 as libc::c_int as u16,
        pbuffer.offset(6 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint16_t(
        (*pdata).prev_shadow_result__ambient_count_rate_mcps_sd0,
        2 as libc::c_int as u16,
        pbuffer.offset(8 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint16_t(
        (*pdata).prev_shadow_result__sigma_sd0,
        2 as libc::c_int as u16,
        pbuffer.offset(10 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint16_t(
        (*pdata).prev_shadow_result__phase_sd0,
        2 as libc::c_int as u16,
        pbuffer.offset(12 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint16_t(
        (*pdata).prev_shadow_result__final_crosstalk_corrected_range_mm_sd0,
        2 as libc::c_int as u16,
        pbuffer.offset(14 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint16_t(
        (*pdata).psr__peak_signal_count_rate_crosstalk_corrected_mcps_sd0,
        2 as libc::c_int as u16,
        pbuffer.offset(16 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint16_t(
        (*pdata).prev_shadow_result__mm_inner_actual_effective_spads_sd0,
        2 as libc::c_int as u16,
        pbuffer.offset(18 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint16_t(
        (*pdata).prev_shadow_result__mm_outer_actual_effective_spads_sd0,
        2 as libc::c_int as u16,
        pbuffer.offset(20 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint16_t(
        (*pdata).prev_shadow_result__avg_signal_count_rate_mcps_sd0,
        2 as libc::c_int as u16,
        pbuffer.offset(22 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint16_t(
        (*pdata).prev_shadow_result__dss_actual_effective_spads_sd1,
        2 as libc::c_int as u16,
        pbuffer.offset(24 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint16_t(
        (*pdata).prev_shadow_result__peak_signal_count_rate_mcps_sd1,
        2 as libc::c_int as u16,
        pbuffer.offset(26 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint16_t(
        (*pdata).prev_shadow_result__ambient_count_rate_mcps_sd1,
        2 as libc::c_int as u16,
        pbuffer.offset(28 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint16_t(
        (*pdata).prev_shadow_result__sigma_sd1,
        2 as libc::c_int as u16,
        pbuffer.offset(30 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint16_t(
        (*pdata).prev_shadow_result__phase_sd1,
        2 as libc::c_int as u16,
        pbuffer.offset(32 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint16_t(
        (*pdata).prev_shadow_result__final_crosstalk_corrected_range_mm_sd1,
        2 as libc::c_int as u16,
        pbuffer.offset(34 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint16_t(
        (*pdata).prev_shadow_result__spare_0_sd1,
        2 as libc::c_int as u16,
        pbuffer.offset(36 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint16_t(
        (*pdata).prev_shadow_result__spare_1_sd1,
        2 as libc::c_int as u16,
        pbuffer.offset(38 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint16_t(
        (*pdata).prev_shadow_result__spare_2_sd1,
        2 as libc::c_int as u16,
        pbuffer.offset(40 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint16_t(
        (*pdata).prev_shadow_result__spare_3_sd1,
        2 as libc::c_int as u16,
        pbuffer.offset(42 as libc::c_int as isize),
    );
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_i2c_decode_prev_shadow_system_results(
    mut buf_size: u16,
    mut pbuffer: *mut u8,
    mut pdata: *mut VL53LX_prev_shadow_system_results_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    if (buf_size as libc::c_int) < 44 as libc::c_int {
        return -(10 as libc::c_int) as VL53LX_Error;
    }
    (*pdata)
        .prev_shadow_result__interrupt_status = (*pbuffer
        .offset(0 as libc::c_int as isize) as libc::c_int & 0x3f as libc::c_int)
        as u8;
    (*pdata)
        .prev_shadow_result__range_status = *pbuffer.offset(1 as libc::c_int as isize);
    (*pdata)
        .prev_shadow_result__report_status = (*pbuffer.offset(2 as libc::c_int as isize)
        as libc::c_int & 0xf as libc::c_int) as u8;
    (*pdata)
        .prev_shadow_result__stream_count = *pbuffer.offset(3 as libc::c_int as isize);
    (*pdata)
        .prev_shadow_result__dss_actual_effective_spads_sd0 = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(4 as libc::c_int as isize),
    );
    (*pdata)
        .prev_shadow_result__peak_signal_count_rate_mcps_sd0 = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(6 as libc::c_int as isize),
    );
    (*pdata)
        .prev_shadow_result__ambient_count_rate_mcps_sd0 = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(8 as libc::c_int as isize),
    );
    (*pdata)
        .prev_shadow_result__sigma_sd0 = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(10 as libc::c_int as isize),
    );
    (*pdata)
        .prev_shadow_result__phase_sd0 = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(12 as libc::c_int as isize),
    );
    (*pdata)
        .prev_shadow_result__final_crosstalk_corrected_range_mm_sd0 = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(14 as libc::c_int as isize),
    );
    (*pdata)
        .psr__peak_signal_count_rate_crosstalk_corrected_mcps_sd0 = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(16 as libc::c_int as isize),
    );
    (*pdata)
        .prev_shadow_result__mm_inner_actual_effective_spads_sd0 = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(18 as libc::c_int as isize),
    );
    (*pdata)
        .prev_shadow_result__mm_outer_actual_effective_spads_sd0 = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(20 as libc::c_int as isize),
    );
    (*pdata)
        .prev_shadow_result__avg_signal_count_rate_mcps_sd0 = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(22 as libc::c_int as isize),
    );
    (*pdata)
        .prev_shadow_result__dss_actual_effective_spads_sd1 = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(24 as libc::c_int as isize),
    );
    (*pdata)
        .prev_shadow_result__peak_signal_count_rate_mcps_sd1 = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(26 as libc::c_int as isize),
    );
    (*pdata)
        .prev_shadow_result__ambient_count_rate_mcps_sd1 = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(28 as libc::c_int as isize),
    );
    (*pdata)
        .prev_shadow_result__sigma_sd1 = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(30 as libc::c_int as isize),
    );
    (*pdata)
        .prev_shadow_result__phase_sd1 = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(32 as libc::c_int as isize),
    );
    (*pdata)
        .prev_shadow_result__final_crosstalk_corrected_range_mm_sd1 = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(34 as libc::c_int as isize),
    );
    (*pdata)
        .prev_shadow_result__spare_0_sd1 = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(36 as libc::c_int as isize),
    );
    (*pdata)
        .prev_shadow_result__spare_1_sd1 = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(38 as libc::c_int as isize),
    );
    (*pdata)
        .prev_shadow_result__spare_2_sd1 = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(40 as libc::c_int as isize),
    );
    (*pdata)
        .prev_shadow_result__spare_3_sd1 = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(42 as libc::c_int as isize),
    );
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_set_prev_shadow_system_results(
    mut Dev: VL53LX_DEV,
    mut pdata: *mut VL53LX_prev_shadow_system_results_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut comms_buffer: [u8; 44] = [0; 44];
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_i2c_encode_prev_shadow_system_results(
            pdata,
            44 as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_disable_firmware(Dev);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_WriteMulti(
            Dev,
            0xed0 as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
            44 as libc::c_int as u32,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_enable_firmware(Dev);
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_get_prev_shadow_system_results(
    mut Dev: VL53LX_DEV,
    mut pdata: *mut VL53LX_prev_shadow_system_results_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut comms_buffer: [u8; 44] = [0; 44];
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_disable_firmware(Dev);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_ReadMulti(
            Dev,
            0xed0 as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
            44 as libc::c_int as u32,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_enable_firmware(Dev);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_i2c_decode_prev_shadow_system_results(
            44 as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
            pdata,
        );
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_i2c_encode_prev_shadow_core_results(
    mut pdata: *mut VL53LX_prev_shadow_core_results_t,
    mut buf_size: u16,
    mut pbuffer: *mut u8,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    if (buf_size as libc::c_int) < 33 as libc::c_int {
        return -(10 as libc::c_int) as VL53LX_Error;
    }
    VL53LX_i2c_encode_uint32_t(
        (*pdata).prev_shadow_result_core__ambient_window_events_sd0,
        4 as libc::c_int as u16,
        pbuffer.offset(0 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint32_t(
        (*pdata).prev_shadow_result_core__ranging_total_events_sd0,
        4 as libc::c_int as u16,
        pbuffer.offset(4 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_int32_t(
        (*pdata).prev_shadow_result_core__signal_total_events_sd0,
        4 as libc::c_int as u16,
        pbuffer.offset(8 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint32_t(
        (*pdata).prev_shadow_result_core__total_periods_elapsed_sd0,
        4 as libc::c_int as u16,
        pbuffer.offset(12 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint32_t(
        (*pdata).prev_shadow_result_core__ambient_window_events_sd1,
        4 as libc::c_int as u16,
        pbuffer.offset(16 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint32_t(
        (*pdata).prev_shadow_result_core__ranging_total_events_sd1,
        4 as libc::c_int as u16,
        pbuffer.offset(20 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_int32_t(
        (*pdata).prev_shadow_result_core__signal_total_events_sd1,
        4 as libc::c_int as u16,
        pbuffer.offset(24 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint32_t(
        (*pdata).prev_shadow_result_core__total_periods_elapsed_sd1,
        4 as libc::c_int as u16,
        pbuffer.offset(28 as libc::c_int as isize),
    );
    *pbuffer
        .offset(32 as libc::c_int as isize) = (*pdata).prev_shadow_result_core__spare_0;
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_i2c_decode_prev_shadow_core_results(
    mut buf_size: u16,
    mut pbuffer: *mut u8,
    mut pdata: *mut VL53LX_prev_shadow_core_results_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    if (buf_size as libc::c_int) < 33 as libc::c_int {
        return -(10 as libc::c_int) as VL53LX_Error;
    }
    (*pdata)
        .prev_shadow_result_core__ambient_window_events_sd0 = VL53LX_i2c_decode_uint32_t(
        4 as libc::c_int as u16,
        pbuffer.offset(0 as libc::c_int as isize),
    );
    (*pdata)
        .prev_shadow_result_core__ranging_total_events_sd0 = VL53LX_i2c_decode_uint32_t(
        4 as libc::c_int as u16,
        pbuffer.offset(4 as libc::c_int as isize),
    );
    (*pdata)
        .prev_shadow_result_core__signal_total_events_sd0 = VL53LX_i2c_decode_int32_t(
        4 as libc::c_int as u16,
        pbuffer.offset(8 as libc::c_int as isize),
    );
    (*pdata)
        .prev_shadow_result_core__total_periods_elapsed_sd0 = VL53LX_i2c_decode_uint32_t(
        4 as libc::c_int as u16,
        pbuffer.offset(12 as libc::c_int as isize),
    );
    (*pdata)
        .prev_shadow_result_core__ambient_window_events_sd1 = VL53LX_i2c_decode_uint32_t(
        4 as libc::c_int as u16,
        pbuffer.offset(16 as libc::c_int as isize),
    );
    (*pdata)
        .prev_shadow_result_core__ranging_total_events_sd1 = VL53LX_i2c_decode_uint32_t(
        4 as libc::c_int as u16,
        pbuffer.offset(20 as libc::c_int as isize),
    );
    (*pdata)
        .prev_shadow_result_core__signal_total_events_sd1 = VL53LX_i2c_decode_int32_t(
        4 as libc::c_int as u16,
        pbuffer.offset(24 as libc::c_int as isize),
    );
    (*pdata)
        .prev_shadow_result_core__total_periods_elapsed_sd1 = VL53LX_i2c_decode_uint32_t(
        4 as libc::c_int as u16,
        pbuffer.offset(28 as libc::c_int as isize),
    );
    (*pdata)
        .prev_shadow_result_core__spare_0 = *pbuffer.offset(32 as libc::c_int as isize);
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_set_prev_shadow_core_results(
    mut Dev: VL53LX_DEV,
    mut pdata: *mut VL53LX_prev_shadow_core_results_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut comms_buffer: [u8; 33] = [0; 33];
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_i2c_encode_prev_shadow_core_results(
            pdata,
            33 as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_disable_firmware(Dev);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_WriteMulti(
            Dev,
            0xefc as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
            33 as libc::c_int as u32,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_enable_firmware(Dev);
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_get_prev_shadow_core_results(
    mut Dev: VL53LX_DEV,
    mut pdata: *mut VL53LX_prev_shadow_core_results_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut comms_buffer: [u8; 33] = [0; 33];
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_disable_firmware(Dev);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_ReadMulti(
            Dev,
            0xefc as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
            33 as libc::c_int as u32,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_enable_firmware(Dev);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_i2c_decode_prev_shadow_core_results(
            33 as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
            pdata,
        );
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_i2c_encode_patch_debug(
    mut pdata: *mut VL53LX_patch_debug_t,
    mut buf_size: u16,
    mut pbuffer: *mut u8,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    if (buf_size as libc::c_int) < 2 as libc::c_int {
        return -(10 as libc::c_int) as VL53LX_Error;
    }
    *pbuffer.offset(0 as libc::c_int as isize) = (*pdata).result__debug_status;
    *pbuffer.offset(1 as libc::c_int as isize) = (*pdata).result__debug_stage;
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_i2c_decode_patch_debug(
    mut buf_size: u16,
    mut pbuffer: *mut u8,
    mut pdata: *mut VL53LX_patch_debug_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    if (buf_size as libc::c_int) < 2 as libc::c_int {
        return -(10 as libc::c_int) as VL53LX_Error;
    }
    (*pdata).result__debug_status = *pbuffer.offset(0 as libc::c_int as isize);
    (*pdata).result__debug_stage = *pbuffer.offset(1 as libc::c_int as isize);
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_set_patch_debug(
    mut Dev: VL53LX_DEV,
    mut pdata: *mut VL53LX_patch_debug_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut comms_buffer: [u8; 2] = [0; 2];
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_i2c_encode_patch_debug(
            pdata,
            2 as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_disable_firmware(Dev);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_WriteMulti(
            Dev,
            0xf20 as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
            2 as libc::c_int as u32,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_enable_firmware(Dev);
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_get_patch_debug(
    mut Dev: VL53LX_DEV,
    mut pdata: *mut VL53LX_patch_debug_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut comms_buffer: [u8; 2] = [0; 2];
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_disable_firmware(Dev);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_ReadMulti(
            Dev,
            0xf20 as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
            2 as libc::c_int as u32,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_enable_firmware(Dev);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_i2c_decode_patch_debug(
            2 as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
            pdata,
        );
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_i2c_encode_gph_general_config(
    mut pdata: *mut VL53LX_gph_general_config_t,
    mut buf_size: u16,
    mut pbuffer: *mut u8,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    if (buf_size as libc::c_int) < 5 as libc::c_int {
        return -(10 as libc::c_int) as VL53LX_Error;
    }
    VL53LX_i2c_encode_uint16_t(
        (*pdata).gph__system__thresh_rate_high,
        2 as libc::c_int as u16,
        pbuffer.offset(0 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint16_t(
        (*pdata).gph__system__thresh_rate_low,
        2 as libc::c_int as u16,
        pbuffer.offset(2 as libc::c_int as isize),
    );
    *pbuffer
        .offset(4 as libc::c_int as isize) = (*pdata).gph__system__interrupt_config_gpio;
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_i2c_decode_gph_general_config(
    mut buf_size: u16,
    mut pbuffer: *mut u8,
    mut pdata: *mut VL53LX_gph_general_config_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    if (buf_size as libc::c_int) < 5 as libc::c_int {
        return -(10 as libc::c_int) as VL53LX_Error;
    }
    (*pdata)
        .gph__system__thresh_rate_high = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(0 as libc::c_int as isize),
    );
    (*pdata)
        .gph__system__thresh_rate_low = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(2 as libc::c_int as isize),
    );
    (*pdata)
        .gph__system__interrupt_config_gpio = *pbuffer.offset(4 as libc::c_int as isize);
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_set_gph_general_config(
    mut Dev: VL53LX_DEV,
    mut pdata: *mut VL53LX_gph_general_config_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut comms_buffer: [u8; 5] = [0; 5];
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_i2c_encode_gph_general_config(
            pdata,
            5 as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_disable_firmware(Dev);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_WriteMulti(
            Dev,
            0xf24 as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
            5 as libc::c_int as u32,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_enable_firmware(Dev);
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_get_gph_general_config(
    mut Dev: VL53LX_DEV,
    mut pdata: *mut VL53LX_gph_general_config_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut comms_buffer: [u8; 5] = [0; 5];
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_disable_firmware(Dev);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_ReadMulti(
            Dev,
            0xf24 as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
            5 as libc::c_int as u32,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_enable_firmware(Dev);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_i2c_decode_gph_general_config(
            5 as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
            pdata,
        );
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_i2c_encode_gph_static_config(
    mut pdata: *mut VL53LX_gph_static_config_t,
    mut buf_size: u16,
    mut pbuffer: *mut u8,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    if (buf_size as libc::c_int) < 6 as libc::c_int {
        return -(10 as libc::c_int) as VL53LX_Error;
    }
    *pbuffer
        .offset(
            0 as libc::c_int as isize,
        ) = ((*pdata).gph__dss_config__roi_mode_control as libc::c_int
        & 0x7 as libc::c_int) as u8;
    VL53LX_i2c_encode_uint16_t(
        (*pdata).gph__dss_config__manual_effective_spads_select,
        2 as libc::c_int as u16,
        pbuffer.offset(1 as libc::c_int as isize),
    );
    *pbuffer
        .offset(
            3 as libc::c_int as isize,
        ) = (*pdata).gph__dss_config__manual_block_select;
    *pbuffer
        .offset(4 as libc::c_int as isize) = (*pdata).gph__dss_config__max_spads_limit;
    *pbuffer
        .offset(5 as libc::c_int as isize) = (*pdata).gph__dss_config__min_spads_limit;
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_i2c_decode_gph_static_config(
    mut buf_size: u16,
    mut pbuffer: *mut u8,
    mut pdata: *mut VL53LX_gph_static_config_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    if (buf_size as libc::c_int) < 6 as libc::c_int {
        return -(10 as libc::c_int) as VL53LX_Error;
    }
    (*pdata)
        .gph__dss_config__roi_mode_control = (*pbuffer.offset(0 as libc::c_int as isize)
        as libc::c_int & 0x7 as libc::c_int) as u8;
    (*pdata)
        .gph__dss_config__manual_effective_spads_select = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(1 as libc::c_int as isize),
    );
    (*pdata)
        .gph__dss_config__manual_block_select = *pbuffer
        .offset(3 as libc::c_int as isize);
    (*pdata)
        .gph__dss_config__max_spads_limit = *pbuffer.offset(4 as libc::c_int as isize);
    (*pdata)
        .gph__dss_config__min_spads_limit = *pbuffer.offset(5 as libc::c_int as isize);
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_set_gph_static_config(
    mut Dev: VL53LX_DEV,
    mut pdata: *mut VL53LX_gph_static_config_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut comms_buffer: [u8; 6] = [0; 6];
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_i2c_encode_gph_static_config(
            pdata,
            6 as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_disable_firmware(Dev);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_WriteMulti(
            Dev,
            0xf2f as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
            6 as libc::c_int as u32,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_enable_firmware(Dev);
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_get_gph_static_config(
    mut Dev: VL53LX_DEV,
    mut pdata: *mut VL53LX_gph_static_config_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut comms_buffer: [u8; 6] = [0; 6];
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_disable_firmware(Dev);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_ReadMulti(
            Dev,
            0xf2f as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
            6 as libc::c_int as u32,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_enable_firmware(Dev);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_i2c_decode_gph_static_config(
            6 as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
            pdata,
        );
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_i2c_encode_gph_timing_config(
    mut pdata: *mut VL53LX_gph_timing_config_t,
    mut buf_size: u16,
    mut pbuffer: *mut u8,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    if (buf_size as libc::c_int) < 16 as libc::c_int {
        return -(10 as libc::c_int) as VL53LX_Error;
    }
    *pbuffer
        .offset(
            0 as libc::c_int as isize,
        ) = ((*pdata).gph__mm_config__timeout_macrop_a_hi as libc::c_int
        & 0xf as libc::c_int) as u8;
    *pbuffer
        .offset(
            1 as libc::c_int as isize,
        ) = (*pdata).gph__mm_config__timeout_macrop_a_lo;
    *pbuffer
        .offset(
            2 as libc::c_int as isize,
        ) = ((*pdata).gph__mm_config__timeout_macrop_b_hi as libc::c_int
        & 0xf as libc::c_int) as u8;
    *pbuffer
        .offset(
            3 as libc::c_int as isize,
        ) = (*pdata).gph__mm_config__timeout_macrop_b_lo;
    *pbuffer
        .offset(
            4 as libc::c_int as isize,
        ) = ((*pdata).gph__range_config__timeout_macrop_a_hi as libc::c_int
        & 0xf as libc::c_int) as u8;
    *pbuffer
        .offset(
            5 as libc::c_int as isize,
        ) = (*pdata).gph__range_config__timeout_macrop_a_lo;
    *pbuffer
        .offset(
            6 as libc::c_int as isize,
        ) = ((*pdata).gph__range_config__vcsel_period_a as libc::c_int
        & 0x3f as libc::c_int) as u8;
    *pbuffer
        .offset(
            7 as libc::c_int as isize,
        ) = ((*pdata).gph__range_config__vcsel_period_b as libc::c_int
        & 0x3f as libc::c_int) as u8;
    *pbuffer
        .offset(
            8 as libc::c_int as isize,
        ) = ((*pdata).gph__range_config__timeout_macrop_b_hi as libc::c_int
        & 0xf as libc::c_int) as u8;
    *pbuffer
        .offset(
            9 as libc::c_int as isize,
        ) = (*pdata).gph__range_config__timeout_macrop_b_lo;
    VL53LX_i2c_encode_uint16_t(
        (*pdata).gph__range_config__sigma_thresh,
        2 as libc::c_int as u16,
        pbuffer.offset(10 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint16_t(
        (*pdata).gph__range_config__min_count_rate_rtn_limit_mcps,
        2 as libc::c_int as u16,
        pbuffer.offset(12 as libc::c_int as isize),
    );
    *pbuffer
        .offset(
            14 as libc::c_int as isize,
        ) = (*pdata).gph__range_config__valid_phase_low;
    *pbuffer
        .offset(
            15 as libc::c_int as isize,
        ) = (*pdata).gph__range_config__valid_phase_high;
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_i2c_decode_gph_timing_config(
    mut buf_size: u16,
    mut pbuffer: *mut u8,
    mut pdata: *mut VL53LX_gph_timing_config_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    if (buf_size as libc::c_int) < 16 as libc::c_int {
        return -(10 as libc::c_int) as VL53LX_Error;
    }
    (*pdata)
        .gph__mm_config__timeout_macrop_a_hi = (*pbuffer
        .offset(0 as libc::c_int as isize) as libc::c_int & 0xf as libc::c_int)
        as u8;
    (*pdata)
        .gph__mm_config__timeout_macrop_a_lo = *pbuffer
        .offset(1 as libc::c_int as isize);
    (*pdata)
        .gph__mm_config__timeout_macrop_b_hi = (*pbuffer
        .offset(2 as libc::c_int as isize) as libc::c_int & 0xf as libc::c_int)
        as u8;
    (*pdata)
        .gph__mm_config__timeout_macrop_b_lo = *pbuffer
        .offset(3 as libc::c_int as isize);
    (*pdata)
        .gph__range_config__timeout_macrop_a_hi = (*pbuffer
        .offset(4 as libc::c_int as isize) as libc::c_int & 0xf as libc::c_int)
        as u8;
    (*pdata)
        .gph__range_config__timeout_macrop_a_lo = *pbuffer
        .offset(5 as libc::c_int as isize);
    (*pdata)
        .gph__range_config__vcsel_period_a = (*pbuffer.offset(6 as libc::c_int as isize)
        as libc::c_int & 0x3f as libc::c_int) as u8;
    (*pdata)
        .gph__range_config__vcsel_period_b = (*pbuffer.offset(7 as libc::c_int as isize)
        as libc::c_int & 0x3f as libc::c_int) as u8;
    (*pdata)
        .gph__range_config__timeout_macrop_b_hi = (*pbuffer
        .offset(8 as libc::c_int as isize) as libc::c_int & 0xf as libc::c_int)
        as u8;
    (*pdata)
        .gph__range_config__timeout_macrop_b_lo = *pbuffer
        .offset(9 as libc::c_int as isize);
    (*pdata)
        .gph__range_config__sigma_thresh = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(10 as libc::c_int as isize),
    );
    (*pdata)
        .gph__range_config__min_count_rate_rtn_limit_mcps = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(12 as libc::c_int as isize),
    );
    (*pdata)
        .gph__range_config__valid_phase_low = *pbuffer
        .offset(14 as libc::c_int as isize);
    (*pdata)
        .gph__range_config__valid_phase_high = *pbuffer
        .offset(15 as libc::c_int as isize);
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_set_gph_timing_config(
    mut Dev: VL53LX_DEV,
    mut pdata: *mut VL53LX_gph_timing_config_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut comms_buffer: [u8; 16] = [0; 16];
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_i2c_encode_gph_timing_config(
            pdata,
            16 as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_disable_firmware(Dev);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_WriteMulti(
            Dev,
            0xf36 as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
            16 as libc::c_int as u32,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_enable_firmware(Dev);
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_get_gph_timing_config(
    mut Dev: VL53LX_DEV,
    mut pdata: *mut VL53LX_gph_timing_config_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut comms_buffer: [u8; 16] = [0; 16];
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_disable_firmware(Dev);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_ReadMulti(
            Dev,
            0xf36 as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
            16 as libc::c_int as u32,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_enable_firmware(Dev);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_i2c_decode_gph_timing_config(
            16 as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
            pdata,
        );
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_i2c_encode_fw_internal(
    mut pdata: *mut VL53LX_fw_internal_t,
    mut buf_size: u16,
    mut pbuffer: *mut u8,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    if (buf_size as libc::c_int) < 2 as libc::c_int {
        return -(10 as libc::c_int) as VL53LX_Error;
    }
    *pbuffer
        .offset(
            0 as libc::c_int as isize,
        ) = (*pdata).firmware__internal_stream_count_div;
    *pbuffer
        .offset(
            1 as libc::c_int as isize,
        ) = (*pdata).firmware__internal_stream_counter_val;
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_i2c_decode_fw_internal(
    mut buf_size: u16,
    mut pbuffer: *mut u8,
    mut pdata: *mut VL53LX_fw_internal_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    if (buf_size as libc::c_int) < 2 as libc::c_int {
        return -(10 as libc::c_int) as VL53LX_Error;
    }
    (*pdata)
        .firmware__internal_stream_count_div = *pbuffer
        .offset(0 as libc::c_int as isize);
    (*pdata)
        .firmware__internal_stream_counter_val = *pbuffer
        .offset(1 as libc::c_int as isize);
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_set_fw_internal(
    mut Dev: VL53LX_DEV,
    mut pdata: *mut VL53LX_fw_internal_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut comms_buffer: [u8; 2] = [0; 2];
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_i2c_encode_fw_internal(
            pdata,
            2 as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_disable_firmware(Dev);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_WriteMulti(
            Dev,
            0xf46 as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
            2 as libc::c_int as u32,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_enable_firmware(Dev);
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_get_fw_internal(
    mut Dev: VL53LX_DEV,
    mut pdata: *mut VL53LX_fw_internal_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut comms_buffer: [u8; 2] = [0; 2];
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_disable_firmware(Dev);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_ReadMulti(
            Dev,
            0xf46 as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
            2 as libc::c_int as u32,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_enable_firmware(Dev);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_i2c_decode_fw_internal(
            2 as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
            pdata,
        );
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_i2c_encode_patch_results(
    mut pdata: *mut VL53LX_patch_results_t,
    mut buf_size: u16,
    mut pbuffer: *mut u8,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    if (buf_size as libc::c_int) < 90 as libc::c_int {
        return -(10 as libc::c_int) as VL53LX_Error;
    }
    *pbuffer
        .offset(
            0 as libc::c_int as isize,
        ) = ((*pdata).dss_calc__roi_ctrl as libc::c_int & 0x3 as libc::c_int) as u8;
    *pbuffer.offset(1 as libc::c_int as isize) = (*pdata).dss_calc__spare_1;
    *pbuffer.offset(2 as libc::c_int as isize) = (*pdata).dss_calc__spare_2;
    *pbuffer.offset(3 as libc::c_int as isize) = (*pdata).dss_calc__spare_3;
    *pbuffer.offset(4 as libc::c_int as isize) = (*pdata).dss_calc__spare_4;
    *pbuffer.offset(5 as libc::c_int as isize) = (*pdata).dss_calc__spare_5;
    *pbuffer.offset(6 as libc::c_int as isize) = (*pdata).dss_calc__spare_6;
    *pbuffer.offset(7 as libc::c_int as isize) = (*pdata).dss_calc__spare_7;
    *pbuffer.offset(8 as libc::c_int as isize) = (*pdata).dss_calc__user_roi_spad_en_0;
    *pbuffer.offset(9 as libc::c_int as isize) = (*pdata).dss_calc__user_roi_spad_en_1;
    *pbuffer.offset(10 as libc::c_int as isize) = (*pdata).dss_calc__user_roi_spad_en_2;
    *pbuffer.offset(11 as libc::c_int as isize) = (*pdata).dss_calc__user_roi_spad_en_3;
    *pbuffer.offset(12 as libc::c_int as isize) = (*pdata).dss_calc__user_roi_spad_en_4;
    *pbuffer.offset(13 as libc::c_int as isize) = (*pdata).dss_calc__user_roi_spad_en_5;
    *pbuffer.offset(14 as libc::c_int as isize) = (*pdata).dss_calc__user_roi_spad_en_6;
    *pbuffer.offset(15 as libc::c_int as isize) = (*pdata).dss_calc__user_roi_spad_en_7;
    *pbuffer.offset(16 as libc::c_int as isize) = (*pdata).dss_calc__user_roi_spad_en_8;
    *pbuffer.offset(17 as libc::c_int as isize) = (*pdata).dss_calc__user_roi_spad_en_9;
    *pbuffer.offset(18 as libc::c_int as isize) = (*pdata).dss_calc__user_roi_spad_en_10;
    *pbuffer.offset(19 as libc::c_int as isize) = (*pdata).dss_calc__user_roi_spad_en_11;
    *pbuffer.offset(20 as libc::c_int as isize) = (*pdata).dss_calc__user_roi_spad_en_12;
    *pbuffer.offset(21 as libc::c_int as isize) = (*pdata).dss_calc__user_roi_spad_en_13;
    *pbuffer.offset(22 as libc::c_int as isize) = (*pdata).dss_calc__user_roi_spad_en_14;
    *pbuffer.offset(23 as libc::c_int as isize) = (*pdata).dss_calc__user_roi_spad_en_15;
    *pbuffer.offset(24 as libc::c_int as isize) = (*pdata).dss_calc__user_roi_spad_en_16;
    *pbuffer.offset(25 as libc::c_int as isize) = (*pdata).dss_calc__user_roi_spad_en_17;
    *pbuffer.offset(26 as libc::c_int as isize) = (*pdata).dss_calc__user_roi_spad_en_18;
    *pbuffer.offset(27 as libc::c_int as isize) = (*pdata).dss_calc__user_roi_spad_en_19;
    *pbuffer.offset(28 as libc::c_int as isize) = (*pdata).dss_calc__user_roi_spad_en_20;
    *pbuffer.offset(29 as libc::c_int as isize) = (*pdata).dss_calc__user_roi_spad_en_21;
    *pbuffer.offset(30 as libc::c_int as isize) = (*pdata).dss_calc__user_roi_spad_en_22;
    *pbuffer.offset(31 as libc::c_int as isize) = (*pdata).dss_calc__user_roi_spad_en_23;
    *pbuffer.offset(32 as libc::c_int as isize) = (*pdata).dss_calc__user_roi_spad_en_24;
    *pbuffer.offset(33 as libc::c_int as isize) = (*pdata).dss_calc__user_roi_spad_en_25;
    *pbuffer.offset(34 as libc::c_int as isize) = (*pdata).dss_calc__user_roi_spad_en_26;
    *pbuffer.offset(35 as libc::c_int as isize) = (*pdata).dss_calc__user_roi_spad_en_27;
    *pbuffer.offset(36 as libc::c_int as isize) = (*pdata).dss_calc__user_roi_spad_en_28;
    *pbuffer.offset(37 as libc::c_int as isize) = (*pdata).dss_calc__user_roi_spad_en_29;
    *pbuffer.offset(38 as libc::c_int as isize) = (*pdata).dss_calc__user_roi_spad_en_30;
    *pbuffer.offset(39 as libc::c_int as isize) = (*pdata).dss_calc__user_roi_spad_en_31;
    *pbuffer.offset(40 as libc::c_int as isize) = (*pdata).dss_calc__user_roi_0;
    *pbuffer.offset(41 as libc::c_int as isize) = (*pdata).dss_calc__user_roi_1;
    *pbuffer.offset(42 as libc::c_int as isize) = (*pdata).dss_calc__mode_roi_0;
    *pbuffer.offset(43 as libc::c_int as isize) = (*pdata).dss_calc__mode_roi_1;
    *pbuffer.offset(44 as libc::c_int as isize) = (*pdata).sigma_estimator_calc__spare_0;
    VL53LX_i2c_encode_uint16_t(
        (*pdata).vhv_result__peak_signal_rate_mcps,
        2 as libc::c_int as u16,
        pbuffer.offset(46 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint32_t(
        (*pdata).vhv_result__signal_total_events_ref,
        4 as libc::c_int as u16,
        pbuffer.offset(48 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint16_t(
        (*pdata).phasecal_result__phase_output_ref,
        2 as libc::c_int as u16,
        pbuffer.offset(52 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint16_t(
        (*pdata).dss_result__total_rate_per_spad,
        2 as libc::c_int as u16,
        pbuffer.offset(54 as libc::c_int as isize),
    );
    *pbuffer.offset(56 as libc::c_int as isize) = (*pdata).dss_result__enabled_blocks;
    VL53LX_i2c_encode_uint16_t(
        (*pdata).dss_result__num_requested_spads,
        2 as libc::c_int as u16,
        pbuffer.offset(58 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint16_t(
        (*pdata).mm_result__inner_intersection_rate,
        2 as libc::c_int as u16,
        pbuffer.offset(62 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint16_t(
        (*pdata).mm_result__outer_complement_rate,
        2 as libc::c_int as u16,
        pbuffer.offset(64 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint16_t(
        (*pdata).mm_result__total_offset,
        2 as libc::c_int as u16,
        pbuffer.offset(66 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint32_t(
        (*pdata).xtalk_calc__xtalk_for_enabled_spads
            & 0xffffff as libc::c_int as libc::c_uint,
        4 as libc::c_int as u16,
        pbuffer.offset(68 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint32_t(
        (*pdata).xtalk_result__avg_xtalk_user_roi_kcps
            & 0xffffff as libc::c_int as libc::c_uint,
        4 as libc::c_int as u16,
        pbuffer.offset(72 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint32_t(
        (*pdata).xtalk_result__avg_xtalk_mm_inner_roi_kcps
            & 0xffffff as libc::c_int as libc::c_uint,
        4 as libc::c_int as u16,
        pbuffer.offset(76 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint32_t(
        (*pdata).xtalk_result__avg_xtalk_mm_outer_roi_kcps
            & 0xffffff as libc::c_int as libc::c_uint,
        4 as libc::c_int as u16,
        pbuffer.offset(80 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint32_t(
        (*pdata).range_result__accum_phase,
        4 as libc::c_int as u16,
        pbuffer.offset(84 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint16_t(
        (*pdata).range_result__offset_corrected_range,
        2 as libc::c_int as u16,
        pbuffer.offset(88 as libc::c_int as isize),
    );
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_i2c_decode_patch_results(
    mut buf_size: u16,
    mut pbuffer: *mut u8,
    mut pdata: *mut VL53LX_patch_results_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    if (buf_size as libc::c_int) < 90 as libc::c_int {
        return -(10 as libc::c_int) as VL53LX_Error;
    }
    (*pdata)
        .dss_calc__roi_ctrl = (*pbuffer.offset(0 as libc::c_int as isize) as libc::c_int
        & 0x3 as libc::c_int) as u8;
    (*pdata).dss_calc__spare_1 = *pbuffer.offset(1 as libc::c_int as isize);
    (*pdata).dss_calc__spare_2 = *pbuffer.offset(2 as libc::c_int as isize);
    (*pdata).dss_calc__spare_3 = *pbuffer.offset(3 as libc::c_int as isize);
    (*pdata).dss_calc__spare_4 = *pbuffer.offset(4 as libc::c_int as isize);
    (*pdata).dss_calc__spare_5 = *pbuffer.offset(5 as libc::c_int as isize);
    (*pdata).dss_calc__spare_6 = *pbuffer.offset(6 as libc::c_int as isize);
    (*pdata).dss_calc__spare_7 = *pbuffer.offset(7 as libc::c_int as isize);
    (*pdata).dss_calc__user_roi_spad_en_0 = *pbuffer.offset(8 as libc::c_int as isize);
    (*pdata).dss_calc__user_roi_spad_en_1 = *pbuffer.offset(9 as libc::c_int as isize);
    (*pdata).dss_calc__user_roi_spad_en_2 = *pbuffer.offset(10 as libc::c_int as isize);
    (*pdata).dss_calc__user_roi_spad_en_3 = *pbuffer.offset(11 as libc::c_int as isize);
    (*pdata).dss_calc__user_roi_spad_en_4 = *pbuffer.offset(12 as libc::c_int as isize);
    (*pdata).dss_calc__user_roi_spad_en_5 = *pbuffer.offset(13 as libc::c_int as isize);
    (*pdata).dss_calc__user_roi_spad_en_6 = *pbuffer.offset(14 as libc::c_int as isize);
    (*pdata).dss_calc__user_roi_spad_en_7 = *pbuffer.offset(15 as libc::c_int as isize);
    (*pdata).dss_calc__user_roi_spad_en_8 = *pbuffer.offset(16 as libc::c_int as isize);
    (*pdata).dss_calc__user_roi_spad_en_9 = *pbuffer.offset(17 as libc::c_int as isize);
    (*pdata).dss_calc__user_roi_spad_en_10 = *pbuffer.offset(18 as libc::c_int as isize);
    (*pdata).dss_calc__user_roi_spad_en_11 = *pbuffer.offset(19 as libc::c_int as isize);
    (*pdata).dss_calc__user_roi_spad_en_12 = *pbuffer.offset(20 as libc::c_int as isize);
    (*pdata).dss_calc__user_roi_spad_en_13 = *pbuffer.offset(21 as libc::c_int as isize);
    (*pdata).dss_calc__user_roi_spad_en_14 = *pbuffer.offset(22 as libc::c_int as isize);
    (*pdata).dss_calc__user_roi_spad_en_15 = *pbuffer.offset(23 as libc::c_int as isize);
    (*pdata).dss_calc__user_roi_spad_en_16 = *pbuffer.offset(24 as libc::c_int as isize);
    (*pdata).dss_calc__user_roi_spad_en_17 = *pbuffer.offset(25 as libc::c_int as isize);
    (*pdata).dss_calc__user_roi_spad_en_18 = *pbuffer.offset(26 as libc::c_int as isize);
    (*pdata).dss_calc__user_roi_spad_en_19 = *pbuffer.offset(27 as libc::c_int as isize);
    (*pdata).dss_calc__user_roi_spad_en_20 = *pbuffer.offset(28 as libc::c_int as isize);
    (*pdata).dss_calc__user_roi_spad_en_21 = *pbuffer.offset(29 as libc::c_int as isize);
    (*pdata).dss_calc__user_roi_spad_en_22 = *pbuffer.offset(30 as libc::c_int as isize);
    (*pdata).dss_calc__user_roi_spad_en_23 = *pbuffer.offset(31 as libc::c_int as isize);
    (*pdata).dss_calc__user_roi_spad_en_24 = *pbuffer.offset(32 as libc::c_int as isize);
    (*pdata).dss_calc__user_roi_spad_en_25 = *pbuffer.offset(33 as libc::c_int as isize);
    (*pdata).dss_calc__user_roi_spad_en_26 = *pbuffer.offset(34 as libc::c_int as isize);
    (*pdata).dss_calc__user_roi_spad_en_27 = *pbuffer.offset(35 as libc::c_int as isize);
    (*pdata).dss_calc__user_roi_spad_en_28 = *pbuffer.offset(36 as libc::c_int as isize);
    (*pdata).dss_calc__user_roi_spad_en_29 = *pbuffer.offset(37 as libc::c_int as isize);
    (*pdata).dss_calc__user_roi_spad_en_30 = *pbuffer.offset(38 as libc::c_int as isize);
    (*pdata).dss_calc__user_roi_spad_en_31 = *pbuffer.offset(39 as libc::c_int as isize);
    (*pdata).dss_calc__user_roi_0 = *pbuffer.offset(40 as libc::c_int as isize);
    (*pdata).dss_calc__user_roi_1 = *pbuffer.offset(41 as libc::c_int as isize);
    (*pdata).dss_calc__mode_roi_0 = *pbuffer.offset(42 as libc::c_int as isize);
    (*pdata).dss_calc__mode_roi_1 = *pbuffer.offset(43 as libc::c_int as isize);
    (*pdata).sigma_estimator_calc__spare_0 = *pbuffer.offset(44 as libc::c_int as isize);
    (*pdata)
        .vhv_result__peak_signal_rate_mcps = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(46 as libc::c_int as isize),
    );
    (*pdata)
        .vhv_result__signal_total_events_ref = VL53LX_i2c_decode_uint32_t(
        4 as libc::c_int as u16,
        pbuffer.offset(48 as libc::c_int as isize),
    );
    (*pdata)
        .phasecal_result__phase_output_ref = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(52 as libc::c_int as isize),
    );
    (*pdata)
        .dss_result__total_rate_per_spad = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(54 as libc::c_int as isize),
    );
    (*pdata).dss_result__enabled_blocks = *pbuffer.offset(56 as libc::c_int as isize);
    (*pdata)
        .dss_result__num_requested_spads = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(58 as libc::c_int as isize),
    );
    (*pdata)
        .mm_result__inner_intersection_rate = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(62 as libc::c_int as isize),
    );
    (*pdata)
        .mm_result__outer_complement_rate = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(64 as libc::c_int as isize),
    );
    (*pdata)
        .mm_result__total_offset = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(66 as libc::c_int as isize),
    );
    (*pdata)
        .xtalk_calc__xtalk_for_enabled_spads = VL53LX_i2c_decode_uint32_t(
        4 as libc::c_int as u16,
        pbuffer.offset(68 as libc::c_int as isize),
    ) & 0xffffff as libc::c_int as libc::c_uint;
    (*pdata)
        .xtalk_result__avg_xtalk_user_roi_kcps = VL53LX_i2c_decode_uint32_t(
        4 as libc::c_int as u16,
        pbuffer.offset(72 as libc::c_int as isize),
    ) & 0xffffff as libc::c_int as libc::c_uint;
    (*pdata)
        .xtalk_result__avg_xtalk_mm_inner_roi_kcps = VL53LX_i2c_decode_uint32_t(
        4 as libc::c_int as u16,
        pbuffer.offset(76 as libc::c_int as isize),
    ) & 0xffffff as libc::c_int as libc::c_uint;
    (*pdata)
        .xtalk_result__avg_xtalk_mm_outer_roi_kcps = VL53LX_i2c_decode_uint32_t(
        4 as libc::c_int as u16,
        pbuffer.offset(80 as libc::c_int as isize),
    ) & 0xffffff as libc::c_int as libc::c_uint;
    (*pdata)
        .range_result__accum_phase = VL53LX_i2c_decode_uint32_t(
        4 as libc::c_int as u16,
        pbuffer.offset(84 as libc::c_int as isize),
    );
    (*pdata)
        .range_result__offset_corrected_range = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(88 as libc::c_int as isize),
    );
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_set_patch_results(
    mut Dev: VL53LX_DEV,
    mut pdata: *mut VL53LX_patch_results_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut comms_buffer: [u8; 90] = [0; 90];
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_i2c_encode_patch_results(
            pdata,
            90 as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_disable_firmware(Dev);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_WriteMulti(
            Dev,
            0xf54 as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
            90 as libc::c_int as u32,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_enable_firmware(Dev);
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_get_patch_results(
    mut Dev: VL53LX_DEV,
    mut pdata: *mut VL53LX_patch_results_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut comms_buffer: [u8; 90] = [0; 90];
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_disable_firmware(Dev);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_ReadMulti(
            Dev,
            0xf54 as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
            90 as libc::c_int as u32,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_enable_firmware(Dev);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_i2c_decode_patch_results(
            90 as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
            pdata,
        );
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_i2c_encode_shadow_system_results(
    mut pdata: *mut VL53LX_shadow_system_results_t,
    mut buf_size: u16,
    mut pbuffer: *mut u8,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    if (buf_size as libc::c_int) < 82 as libc::c_int {
        return -(10 as libc::c_int) as VL53LX_Error;
    }
    *pbuffer
        .offset(
            0 as libc::c_int as isize,
        ) = (*pdata).shadow_phasecal_result__vcsel_start;
    *pbuffer
        .offset(
            2 as libc::c_int as isize,
        ) = ((*pdata).shadow_result__interrupt_status as libc::c_int
        & 0x3f as libc::c_int) as u8;
    *pbuffer.offset(3 as libc::c_int as isize) = (*pdata).shadow_result__range_status;
    *pbuffer
        .offset(
            4 as libc::c_int as isize,
        ) = ((*pdata).shadow_result__report_status as libc::c_int & 0xf as libc::c_int)
        as u8;
    *pbuffer.offset(5 as libc::c_int as isize) = (*pdata).shadow_result__stream_count;
    VL53LX_i2c_encode_uint16_t(
        (*pdata).shadow_result__dss_actual_effective_spads_sd0,
        2 as libc::c_int as u16,
        pbuffer.offset(6 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint16_t(
        (*pdata).shadow_result__peak_signal_count_rate_mcps_sd0,
        2 as libc::c_int as u16,
        pbuffer.offset(8 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint16_t(
        (*pdata).shadow_result__ambient_count_rate_mcps_sd0,
        2 as libc::c_int as u16,
        pbuffer.offset(10 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint16_t(
        (*pdata).shadow_result__sigma_sd0,
        2 as libc::c_int as u16,
        pbuffer.offset(12 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint16_t(
        (*pdata).shadow_result__phase_sd0,
        2 as libc::c_int as u16,
        pbuffer.offset(14 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint16_t(
        (*pdata).shadow_result__final_crosstalk_corrected_range_mm_sd0,
        2 as libc::c_int as u16,
        pbuffer.offset(16 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint16_t(
        (*pdata).shr__peak_signal_count_rate_crosstalk_corrected_mcps_sd0,
        2 as libc::c_int as u16,
        pbuffer.offset(18 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint16_t(
        (*pdata).shadow_result__mm_inner_actual_effective_spads_sd0,
        2 as libc::c_int as u16,
        pbuffer.offset(20 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint16_t(
        (*pdata).shadow_result__mm_outer_actual_effective_spads_sd0,
        2 as libc::c_int as u16,
        pbuffer.offset(22 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint16_t(
        (*pdata).shadow_result__avg_signal_count_rate_mcps_sd0,
        2 as libc::c_int as u16,
        pbuffer.offset(24 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint16_t(
        (*pdata).shadow_result__dss_actual_effective_spads_sd1,
        2 as libc::c_int as u16,
        pbuffer.offset(26 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint16_t(
        (*pdata).shadow_result__peak_signal_count_rate_mcps_sd1,
        2 as libc::c_int as u16,
        pbuffer.offset(28 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint16_t(
        (*pdata).shadow_result__ambient_count_rate_mcps_sd1,
        2 as libc::c_int as u16,
        pbuffer.offset(30 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint16_t(
        (*pdata).shadow_result__sigma_sd1,
        2 as libc::c_int as u16,
        pbuffer.offset(32 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint16_t(
        (*pdata).shadow_result__phase_sd1,
        2 as libc::c_int as u16,
        pbuffer.offset(34 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint16_t(
        (*pdata).shadow_result__final_crosstalk_corrected_range_mm_sd1,
        2 as libc::c_int as u16,
        pbuffer.offset(36 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint16_t(
        (*pdata).shadow_result__spare_0_sd1,
        2 as libc::c_int as u16,
        pbuffer.offset(38 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint16_t(
        (*pdata).shadow_result__spare_1_sd1,
        2 as libc::c_int as u16,
        pbuffer.offset(40 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint16_t(
        (*pdata).shadow_result__spare_2_sd1,
        2 as libc::c_int as u16,
        pbuffer.offset(42 as libc::c_int as isize),
    );
    *pbuffer.offset(44 as libc::c_int as isize) = (*pdata).shadow_result__spare_3_sd1;
    *pbuffer.offset(45 as libc::c_int as isize) = (*pdata).shadow_result__thresh_info;
    *pbuffer
        .offset(
            80 as libc::c_int as isize,
        ) = (*pdata).shadow_phasecal_result__reference_phase_hi;
    *pbuffer
        .offset(
            81 as libc::c_int as isize,
        ) = (*pdata).shadow_phasecal_result__reference_phase_lo;
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_i2c_decode_shadow_system_results(
    mut buf_size: u16,
    mut pbuffer: *mut u8,
    mut pdata: *mut VL53LX_shadow_system_results_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    if (buf_size as libc::c_int) < 82 as libc::c_int {
        return -(10 as libc::c_int) as VL53LX_Error;
    }
    (*pdata)
        .shadow_phasecal_result__vcsel_start = *pbuffer
        .offset(0 as libc::c_int as isize);
    (*pdata)
        .shadow_result__interrupt_status = (*pbuffer.offset(2 as libc::c_int as isize)
        as libc::c_int & 0x3f as libc::c_int) as u8;
    (*pdata).shadow_result__range_status = *pbuffer.offset(3 as libc::c_int as isize);
    (*pdata)
        .shadow_result__report_status = (*pbuffer.offset(4 as libc::c_int as isize)
        as libc::c_int & 0xf as libc::c_int) as u8;
    (*pdata).shadow_result__stream_count = *pbuffer.offset(5 as libc::c_int as isize);
    (*pdata)
        .shadow_result__dss_actual_effective_spads_sd0 = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(6 as libc::c_int as isize),
    );
    (*pdata)
        .shadow_result__peak_signal_count_rate_mcps_sd0 = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(8 as libc::c_int as isize),
    );
    (*pdata)
        .shadow_result__ambient_count_rate_mcps_sd0 = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(10 as libc::c_int as isize),
    );
    (*pdata)
        .shadow_result__sigma_sd0 = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(12 as libc::c_int as isize),
    );
    (*pdata)
        .shadow_result__phase_sd0 = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(14 as libc::c_int as isize),
    );
    (*pdata)
        .shadow_result__final_crosstalk_corrected_range_mm_sd0 = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(16 as libc::c_int as isize),
    );
    (*pdata)
        .shr__peak_signal_count_rate_crosstalk_corrected_mcps_sd0 = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(18 as libc::c_int as isize),
    );
    (*pdata)
        .shadow_result__mm_inner_actual_effective_spads_sd0 = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(20 as libc::c_int as isize),
    );
    (*pdata)
        .shadow_result__mm_outer_actual_effective_spads_sd0 = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(22 as libc::c_int as isize),
    );
    (*pdata)
        .shadow_result__avg_signal_count_rate_mcps_sd0 = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(24 as libc::c_int as isize),
    );
    (*pdata)
        .shadow_result__dss_actual_effective_spads_sd1 = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(26 as libc::c_int as isize),
    );
    (*pdata)
        .shadow_result__peak_signal_count_rate_mcps_sd1 = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(28 as libc::c_int as isize),
    );
    (*pdata)
        .shadow_result__ambient_count_rate_mcps_sd1 = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(30 as libc::c_int as isize),
    );
    (*pdata)
        .shadow_result__sigma_sd1 = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(32 as libc::c_int as isize),
    );
    (*pdata)
        .shadow_result__phase_sd1 = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(34 as libc::c_int as isize),
    );
    (*pdata)
        .shadow_result__final_crosstalk_corrected_range_mm_sd1 = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(36 as libc::c_int as isize),
    );
    (*pdata)
        .shadow_result__spare_0_sd1 = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(38 as libc::c_int as isize),
    );
    (*pdata)
        .shadow_result__spare_1_sd1 = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(40 as libc::c_int as isize),
    );
    (*pdata)
        .shadow_result__spare_2_sd1 = VL53LX_i2c_decode_uint16_t(
        2 as libc::c_int as u16,
        pbuffer.offset(42 as libc::c_int as isize),
    );
    (*pdata).shadow_result__spare_3_sd1 = *pbuffer.offset(44 as libc::c_int as isize);
    (*pdata).shadow_result__thresh_info = *pbuffer.offset(45 as libc::c_int as isize);
    (*pdata)
        .shadow_phasecal_result__reference_phase_hi = *pbuffer
        .offset(80 as libc::c_int as isize);
    (*pdata)
        .shadow_phasecal_result__reference_phase_lo = *pbuffer
        .offset(81 as libc::c_int as isize);
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_set_shadow_system_results(
    mut Dev: VL53LX_DEV,
    mut pdata: *mut VL53LX_shadow_system_results_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut comms_buffer: [u8; 82] = [0; 82];
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_i2c_encode_shadow_system_results(
            pdata,
            82 as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_disable_firmware(Dev);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_WriteMulti(
            Dev,
            0xfae as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
            82 as libc::c_int as u32,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_enable_firmware(Dev);
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_get_shadow_system_results(
    mut Dev: VL53LX_DEV,
    mut pdata: *mut VL53LX_shadow_system_results_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut comms_buffer: [u8; 82] = [0; 82];
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_disable_firmware(Dev);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_ReadMulti(
            Dev,
            0xfae as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
            82 as libc::c_int as u32,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_enable_firmware(Dev);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_i2c_decode_shadow_system_results(
            82 as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
            pdata,
        );
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_i2c_encode_shadow_core_results(
    mut pdata: *mut VL53LX_shadow_core_results_t,
    mut buf_size: u16,
    mut pbuffer: *mut u8,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    if (buf_size as libc::c_int) < 33 as libc::c_int {
        return -(10 as libc::c_int) as VL53LX_Error;
    }
    VL53LX_i2c_encode_uint32_t(
        (*pdata).shadow_result_core__ambient_window_events_sd0,
        4 as libc::c_int as u16,
        pbuffer.offset(0 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint32_t(
        (*pdata).shadow_result_core__ranging_total_events_sd0,
        4 as libc::c_int as u16,
        pbuffer.offset(4 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_int32_t(
        (*pdata).shadow_result_core__signal_total_events_sd0,
        4 as libc::c_int as u16,
        pbuffer.offset(8 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint32_t(
        (*pdata).shadow_result_core__total_periods_elapsed_sd0,
        4 as libc::c_int as u16,
        pbuffer.offset(12 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint32_t(
        (*pdata).shadow_result_core__ambient_window_events_sd1,
        4 as libc::c_int as u16,
        pbuffer.offset(16 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint32_t(
        (*pdata).shadow_result_core__ranging_total_events_sd1,
        4 as libc::c_int as u16,
        pbuffer.offset(20 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_int32_t(
        (*pdata).shadow_result_core__signal_total_events_sd1,
        4 as libc::c_int as u16,
        pbuffer.offset(24 as libc::c_int as isize),
    );
    VL53LX_i2c_encode_uint32_t(
        (*pdata).shadow_result_core__total_periods_elapsed_sd1,
        4 as libc::c_int as u16,
        pbuffer.offset(28 as libc::c_int as isize),
    );
    *pbuffer.offset(32 as libc::c_int as isize) = (*pdata).shadow_result_core__spare_0;
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_i2c_decode_shadow_core_results(
    mut buf_size: u16,
    mut pbuffer: *mut u8,
    mut pdata: *mut VL53LX_shadow_core_results_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    if (buf_size as libc::c_int) < 33 as libc::c_int {
        return -(10 as libc::c_int) as VL53LX_Error;
    }
    (*pdata)
        .shadow_result_core__ambient_window_events_sd0 = VL53LX_i2c_decode_uint32_t(
        4 as libc::c_int as u16,
        pbuffer.offset(0 as libc::c_int as isize),
    );
    (*pdata)
        .shadow_result_core__ranging_total_events_sd0 = VL53LX_i2c_decode_uint32_t(
        4 as libc::c_int as u16,
        pbuffer.offset(4 as libc::c_int as isize),
    );
    (*pdata)
        .shadow_result_core__signal_total_events_sd0 = VL53LX_i2c_decode_int32_t(
        4 as libc::c_int as u16,
        pbuffer.offset(8 as libc::c_int as isize),
    );
    (*pdata)
        .shadow_result_core__total_periods_elapsed_sd0 = VL53LX_i2c_decode_uint32_t(
        4 as libc::c_int as u16,
        pbuffer.offset(12 as libc::c_int as isize),
    );
    (*pdata)
        .shadow_result_core__ambient_window_events_sd1 = VL53LX_i2c_decode_uint32_t(
        4 as libc::c_int as u16,
        pbuffer.offset(16 as libc::c_int as isize),
    );
    (*pdata)
        .shadow_result_core__ranging_total_events_sd1 = VL53LX_i2c_decode_uint32_t(
        4 as libc::c_int as u16,
        pbuffer.offset(20 as libc::c_int as isize),
    );
    (*pdata)
        .shadow_result_core__signal_total_events_sd1 = VL53LX_i2c_decode_int32_t(
        4 as libc::c_int as u16,
        pbuffer.offset(24 as libc::c_int as isize),
    );
    (*pdata)
        .shadow_result_core__total_periods_elapsed_sd1 = VL53LX_i2c_decode_uint32_t(
        4 as libc::c_int as u16,
        pbuffer.offset(28 as libc::c_int as isize),
    );
    (*pdata).shadow_result_core__spare_0 = *pbuffer.offset(32 as libc::c_int as isize);
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_set_shadow_core_results(
    mut Dev: VL53LX_DEV,
    mut pdata: *mut VL53LX_shadow_core_results_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut comms_buffer: [u8; 33] = [0; 33];
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_i2c_encode_shadow_core_results(
            pdata,
            33 as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_disable_firmware(Dev);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_WriteMulti(
            Dev,
            0xfdc as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
            33 as libc::c_int as u32,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_enable_firmware(Dev);
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_get_shadow_core_results(
    mut Dev: VL53LX_DEV,
    mut pdata: *mut VL53LX_shadow_core_results_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut comms_buffer: [u8; 33] = [0; 33];
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_disable_firmware(Dev);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_ReadMulti(
            Dev,
            0xfdc as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
            33 as libc::c_int as u32,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_enable_firmware(Dev);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_i2c_decode_shadow_core_results(
            33 as libc::c_int as u16,
            comms_buffer.as_mut_ptr(),
            pdata,
        );
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_f_023(
    mut sigma_estimator__sigma_ref_mm: u8,
    mut VL53LX_p_007: u32,
    mut VL53LX_p_032: u32,
    mut VL53LX_p_001: u32,
    mut a_zp: u32,
    mut c_zp: u32,
    mut bx: u32,
    mut ax_zp: u32,
    mut cx_zp: u32,
    mut VL53LX_p_028: u32,
    mut fast_osc_frequency: u16,
    mut psigma_est: *mut u16,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = -(15 as libc::c_int) as VL53LX_Error;
    let mut sigma_int: u32 = 0xffff as libc::c_int as u32;
    let mut pll_period_mm: u32 = 0 as libc::c_int as u32;
    let mut tmp0: u64 = 0 as libc::c_int as u64;
    let mut tmp1: u64 = 0 as libc::c_int as u64;
    let mut b_minus_amb: u64 = 0 as libc::c_int as u64;
    let mut VL53LX_p_055: u64 = 0 as libc::c_int as u64;
    *psigma_est = 0xffff as libc::c_int as u16;
    if fast_osc_frequency as libc::c_int != 0 as libc::c_int {
        pll_period_mm = VL53LX_calc_pll_period_mm(fast_osc_frequency);
        if VL53LX_p_028 > VL53LX_p_032 {
            b_minus_amb = (VL53LX_p_028 as u64)
                .wrapping_sub(VL53LX_p_032 as u64);
        } else {
            b_minus_amb = (VL53LX_p_032 as u64)
                .wrapping_sub(VL53LX_p_028 as u64);
        }
        if VL53LX_p_007 > VL53LX_p_001 {
            VL53LX_p_055 = (VL53LX_p_007 as u64)
                .wrapping_sub(VL53LX_p_001 as u64);
        } else {
            VL53LX_p_055 = (VL53LX_p_001 as u64)
                .wrapping_sub(VL53LX_p_007 as u64);
        }
        if b_minus_amb != 0 as libc::c_int as libc::c_ulong {
            tmp0 = (VL53LX_p_032 as u64)
                .wrapping_add(bx as u64)
                .wrapping_add(VL53LX_p_028 as u64);
            if tmp0 > 0xffffff as libc::c_int as libc::c_ulong {
                tmp0 = 0xffffff as libc::c_int as u64;
            }
            tmp1 = VL53LX_p_055.wrapping_mul(VL53LX_p_055);
            tmp1 = tmp1 << 8 as libc::c_int;
            if tmp1 > 0xffffffffffffff as libc::c_long as libc::c_ulong {
                tmp1 = 0xffffffffffffff as libc::c_long as u64;
            }
            tmp1 = tmp1.wrapping_div(b_minus_amb);
            tmp1 = tmp1.wrapping_div(b_minus_amb);
            if tmp1 > 0x7fffffffff as libc::c_long as u64 {
                tmp1 = 0x7fffffffff as libc::c_long as u64;
            }
            tmp0 = tmp1.wrapping_mul(tmp0);
            tmp1 = (c_zp as u64)
                .wrapping_add(cx_zp as u64)
                .wrapping_add(a_zp as u64)
                .wrapping_add(ax_zp as u64);
            if tmp1 > 0xffffff as libc::c_int as u64 {
                tmp1 = 0xffffff as libc::c_int as u64;
            }
            tmp1 = tmp1 << 8 as libc::c_int;
            tmp0 = tmp1.wrapping_add(tmp0);
            if tmp0 > 0x7fffffffffffffff as libc::c_long as u64 {
                tmp0 = 0x7fffffffffffffff as libc::c_long as u64;
            }
            if tmp0 > 0xffffffff as libc::c_uint as u64 {
                tmp0 = tmp0.wrapping_div(b_minus_amb);
                tmp0 = tmp0.wrapping_mul(pll_period_mm as libc::c_ulong);
            } else {
                tmp0 = tmp0.wrapping_mul(pll_period_mm as libc::c_ulong);
                tmp0 = tmp0.wrapping_div(b_minus_amb);
            }
            if tmp0 > 0x7fffffffffffffff as libc::c_long as u64 {
                tmp0 = 0x7fffffffffffffff as libc::c_long as u64;
            }
            if tmp0 > 0xffffffff as libc::c_uint as u64 {
                tmp0 = tmp0.wrapping_div(b_minus_amb);
                tmp0 = tmp0.wrapping_div(4 as libc::c_int as libc::c_ulong);
                tmp0 = tmp0.wrapping_mul(pll_period_mm as libc::c_ulong);
            } else {
                tmp0 = tmp0.wrapping_mul(pll_period_mm as libc::c_ulong);
                tmp0 = tmp0.wrapping_div(b_minus_amb);
                tmp0 = tmp0.wrapping_div(4 as libc::c_int as libc::c_ulong);
            }
            if tmp0 > 0x7fffffffffffffff as libc::c_long as u64 {
                tmp0 = 0x7fffffffffffffff as libc::c_long as u64;
            }
            tmp0 = tmp0 >> 2 as libc::c_int;
            if tmp0 > 0xffffffff as libc::c_uint as u64 {
                tmp0 = 0xffffffff as libc::c_uint as u64;
            }
            tmp1 = (sigma_estimator__sigma_ref_mm as u64) << 7 as libc::c_int;
            tmp1 = tmp1.wrapping_mul(tmp1);
            tmp0 = tmp0.wrapping_add(tmp1);
            if tmp0 > 0xffffffff as libc::c_uint as u64 {
                tmp0 = 0xffffffff as libc::c_uint as u64;
            }
            sigma_int = VL53LX_isqrt(tmp0 as u32);
            *psigma_est = sigma_int as u16;
            status = 0 as libc::c_int as VL53LX_Error;
        }
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_is_firmware_ready_silicon(
    mut Dev: VL53LX_DEV,
    mut pready: *mut u8,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    let mut comms_buffer: [u8; 5] = [0; 5];
    status = VL53LX_ReadMulti(
        Dev,
        0xfd as libc::c_int as u16,
        comms_buffer.as_mut_ptr(),
        5 as libc::c_int as u32,
    );
    if !(status as libc::c_int != 0 as libc::c_int as VL53LX_Error as libc::c_int) {
        (*pdev)
            .dbg_results
            .interrupt_manager__enables = comms_buffer[0 as libc::c_int as usize];
        (*pdev)
            .dbg_results
            .interrupt_manager__clear = comms_buffer[1 as libc::c_int as usize];
        (*pdev)
            .dbg_results
            .interrupt_manager__status = comms_buffer[2 as libc::c_int as usize];
        (*pdev)
            .dbg_results
            .mcu_to_host_bank__wr_access_en = comms_buffer[3 as libc::c_int as usize];
        (*pdev)
            .dbg_results
            .power_management__go1_reset_status = comms_buffer[4 as libc::c_int
            as usize];
        if (*pdev).sys_ctrl.power_management__go1_power_force as libc::c_int
            & 0x1 as libc::c_int == 0x1 as libc::c_int
        {
            if (*pdev).dbg_results.interrupt_manager__enables as libc::c_int
                & 0x1f as libc::c_int == 0x1f as libc::c_int
                && (*pdev).dbg_results.interrupt_manager__clear as libc::c_int
                    & 0x1f as libc::c_int == 0x1f as libc::c_int
            {
                *pready = 0x1 as libc::c_int as u8;
            } else {
                *pready = 0 as libc::c_int as u8;
            }
        } else if (*pdev).dbg_results.power_management__go1_reset_status as libc::c_int
                & 0x1 as libc::c_int == 0 as libc::c_int
            {
            *pready = 0x1 as libc::c_int as u8;
        } else {
            *pready = 0 as libc::c_int as u8;
        }
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_wait_for_boot_completion(
    mut Dev: VL53LX_DEV,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    let mut fw_ready: u8 = 0 as libc::c_int as u8;
    if (*pdev).wait_method as libc::c_int
        == 0 as libc::c_int as VL53LX_WaitMethod as libc::c_int
    {
        status = VL53LX_poll_for_boot_completion(Dev, 500 as libc::c_int as u32);
    } else {
        fw_ready = 0 as libc::c_int as u8;
        while fw_ready as libc::c_int == 0 as libc::c_int
            && status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int
        {
            status = VL53LX_is_boot_complete(Dev, &mut fw_ready);
            if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
                status = VL53LX_WaitMs(Dev, 1 as libc::c_int);
            }
        }
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_wait_for_range_completion(
    mut Dev: VL53LX_DEV,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    let mut data_ready: u8 = 0 as libc::c_int as u8;
    if (*pdev).wait_method as libc::c_int
        == 0 as libc::c_int as VL53LX_WaitMethod as libc::c_int
    {
        status = VL53LX_poll_for_range_completion(Dev, 2000 as libc::c_int as u32);
    } else {
        data_ready = 0 as libc::c_int as u8;
        while data_ready as libc::c_int == 0 as libc::c_int
            && status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int
        {
            status = VL53LX_is_new_data_ready(Dev, &mut data_ready);
            if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
                status = VL53LX_WaitMs(Dev, 1 as libc::c_int);
            }
        }
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_wait_for_test_completion(
    mut Dev: VL53LX_DEV,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    let mut data_ready: u8 = 0 as libc::c_int as u8;
    if (*pdev).wait_method as libc::c_int
        == 0 as libc::c_int as VL53LX_WaitMethod as libc::c_int
    {
        status = VL53LX_poll_for_range_completion(Dev, 60000 as libc::c_int as u32);
    } else {
        data_ready = 0 as libc::c_int as u8;
        while data_ready as libc::c_int == 0 as libc::c_int
            && status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int
        {
            status = VL53LX_is_new_data_ready(Dev, &mut data_ready);
            if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
                status = VL53LX_WaitMs(Dev, 1 as libc::c_int);
            }
        }
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_is_boot_complete(
    mut Dev: VL53LX_DEV,
    mut pready: *mut u8,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut firmware__system_status: u8 = 0 as libc::c_int as u8;
    status = VL53LX_RdByte(
        Dev,
        0xe5 as libc::c_int as u16,
        &mut firmware__system_status,
    );
    if firmware__system_status as libc::c_int & 0x1 as libc::c_int == 0x1 as libc::c_int
    {
        *pready = 0x1 as libc::c_int as u8;
        VL53LX_init_ll_driver_state(Dev, 3 as libc::c_int as VL53LX_DeviceState);
    } else {
        *pready = 0 as libc::c_int as u8;
        VL53LX_init_ll_driver_state(Dev, 2 as libc::c_int as VL53LX_DeviceState);
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_is_firmware_ready(
    mut Dev: VL53LX_DEV,
    mut pready: *mut u8,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    status = VL53LX_is_firmware_ready_silicon(Dev, pready);
    (*pdev).fw_ready = *pready;
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_is_new_data_ready(
    mut Dev: VL53LX_DEV,
    mut pready: *mut u8,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    let mut gpio__mux_active_high_hv: u8 = 0 as libc::c_int as u8;
    let mut gpio__tio_hv_status: u8 = 0 as libc::c_int as u8;
    let mut interrupt_ready: u8 = 0 as libc::c_int as u8;
    gpio__mux_active_high_hv = ((*pdev).stat_cfg.gpio_hv_mux__ctrl as libc::c_int
        & 0x10 as libc::c_int) as u8;
    if gpio__mux_active_high_hv as libc::c_int == 0 as libc::c_int {
        interrupt_ready = 0x1 as libc::c_int as u8;
    } else {
        interrupt_ready = 0 as libc::c_int as u8;
    }
    status = VL53LX_RdByte(
        Dev,
        0x31 as libc::c_int as u16,
        &mut gpio__tio_hv_status,
    );
    if gpio__tio_hv_status as libc::c_int & 0x1 as libc::c_int
        == interrupt_ready as libc::c_int
    {
        *pready = 0x1 as libc::c_int as u8;
    } else {
        *pready = 0 as libc::c_int as u8;
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_poll_for_boot_completion(
    mut Dev: VL53LX_DEV,
    mut timeout_ms: u32,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    status = VL53LX_WaitUs(Dev, 1200 as libc::c_int);
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_WaitValueMaskEx(
            Dev,
            timeout_ms,
            0xe5 as libc::c_int as u16,
            0x1 as libc::c_int as u8,
            0x1 as libc::c_int as u8,
            1 as libc::c_int as u32,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        VL53LX_init_ll_driver_state(Dev, 3 as libc::c_int as VL53LX_DeviceState);
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_poll_for_firmware_ready(
    mut Dev: VL53LX_DEV,
    mut timeout_ms: u32,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    let mut start_time_ms: u32 = 0 as libc::c_int as u32;
    let mut current_time_ms: u32 = 0 as libc::c_int as u32;
    let mut poll_delay_ms: i32 = 1 as libc::c_int;
    let mut fw_ready: u8 = 0 as libc::c_int as u8;
    VL53LX_GetTickCount(Dev, &mut start_time_ms);
    (*pdev).fw_ready_poll_duration_ms = 0 as libc::c_int as u32;
    while status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int
        && (*pdev).fw_ready_poll_duration_ms < timeout_ms
        && fw_ready as libc::c_int == 0 as libc::c_int
    {
        status = VL53LX_is_firmware_ready(Dev, &mut fw_ready);
        if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int
            && fw_ready as libc::c_int == 0 as libc::c_int
            && poll_delay_ms > 0 as libc::c_int
        {
            status = VL53LX_WaitMs(Dev, poll_delay_ms);
        }
        VL53LX_GetTickCount(Dev, &mut current_time_ms);
        (*pdev).fw_ready_poll_duration_ms = current_time_ms.wrapping_sub(start_time_ms);
    }
    if fw_ready as libc::c_int == 0 as libc::c_int
        && status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int
    {
        status = -(7 as libc::c_int) as VL53LX_Error;
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_poll_for_range_completion(
    mut Dev: VL53LX_DEV,
    mut timeout_ms: u32,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut pdev: *mut VL53LX_LLDriverData_t = &mut (*Dev).Data.LLData;
    let mut gpio__mux_active_high_hv: u8 = 0 as libc::c_int as u8;
    let mut interrupt_ready: u8 = 0 as libc::c_int as u8;
    gpio__mux_active_high_hv = ((*pdev).stat_cfg.gpio_hv_mux__ctrl as libc::c_int
        & 0x10 as libc::c_int) as u8;
    if gpio__mux_active_high_hv as libc::c_int == 0 as libc::c_int {
        interrupt_ready = 0x1 as libc::c_int as u8;
    } else {
        interrupt_ready = 0 as libc::c_int as u8;
    }
    status = VL53LX_WaitValueMaskEx(
        Dev,
        timeout_ms,
        0x31 as libc::c_int as u16,
        interrupt_ready,
        0x1 as libc::c_int as u8,
        1 as libc::c_int as u32,
    );
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_xtalk_calibration_process_data(
    mut pxtalk_results: *mut VL53LX_xtalk_range_results_t,
    mut pxtalk_shape: *mut VL53LX_xtalk_histogram_data_t,
    mut pxtalk_cal: *mut VL53LX_xtalk_calibration_results_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut xtalk_debug: VL53LX_xtalk_algo_data_t = VL53LX_xtalk_algo_data_t {
        VL53LX_p_061: [0; 4],
        VL53LX_p_059: 0,
        VL53LX_p_060: 0,
        VL53LX_p_056: VL53LX_histogram_bin_data_t {
            cfg_device_state: 0,
            rd_device_state: 0,
            zone_id: 0,
            time_stamp: 0,
            VL53LX_p_019: 0,
            VL53LX_p_020: 0,
            VL53LX_p_021: 0,
            number_of_ambient_bins: 0,
            bin_seq: [0; 6],
            bin_rep: [0; 6],
            bin_data: [0; 24],
            result__interrupt_status: 0,
            result__range_status: 0,
            result__report_status: 0,
            result__stream_count: 0,
            result__dss_actual_effective_spads: 0,
            phasecal_result__reference_phase: 0,
            phasecal_result__vcsel_start: 0,
            cal_config__vcsel_start: 0,
            vcsel_width: 0,
            VL53LX_p_005: 0,
            VL53LX_p_015: 0,
            total_periods_elapsed: 0,
            peak_duration_us: 0,
            woi_duration_us: 0,
            min_bin_value: 0,
            max_bin_value: 0,
            zero_distance_phase: 0,
            number_of_ambient_samples: 0,
            ambient_events_sum: 0,
            VL53LX_p_028: 0,
            roi_config__user_roi_centre_spad: 0,
            roi_config__user_roi_requested_global_xy_size: 0,
        },
        VL53LX_p_057: VL53LX_histogram_bin_data_t {
            cfg_device_state: 0,
            rd_device_state: 0,
            zone_id: 0,
            time_stamp: 0,
            VL53LX_p_019: 0,
            VL53LX_p_020: 0,
            VL53LX_p_021: 0,
            number_of_ambient_bins: 0,
            bin_seq: [0; 6],
            bin_rep: [0; 6],
            bin_data: [0; 24],
            result__interrupt_status: 0,
            result__range_status: 0,
            result__report_status: 0,
            result__stream_count: 0,
            result__dss_actual_effective_spads: 0,
            phasecal_result__reference_phase: 0,
            phasecal_result__vcsel_start: 0,
            cal_config__vcsel_start: 0,
            vcsel_width: 0,
            VL53LX_p_005: 0,
            VL53LX_p_015: 0,
            total_periods_elapsed: 0,
            peak_duration_us: 0,
            woi_duration_us: 0,
            min_bin_value: 0,
            max_bin_value: 0,
            zero_distance_phase: 0,
            number_of_ambient_samples: 0,
            ambient_events_sum: 0,
            VL53LX_p_028: 0,
            roi_config__user_roi_centre_spad: 0,
            roi_config__user_roi_requested_global_xy_size: 0,
        },
        VL53LX_p_058: 0,
        VL53LX_p_062: [0; 12],
    };
    let mut pdebug: *mut VL53LX_xtalk_algo_data_t = &mut xtalk_debug;
    let mut pxtalk_data: *mut VL53LX_xtalk_range_data_t = 0
        as *mut VL53LX_xtalk_range_data_t;
    let mut avg_bins: VL53LX_histogram_bin_data_t = VL53LX_histogram_bin_data_t {
        cfg_device_state: 0,
        rd_device_state: 0,
        zone_id: 0,
        time_stamp: 0,
        VL53LX_p_019: 0,
        VL53LX_p_020: 0,
        VL53LX_p_021: 0,
        number_of_ambient_bins: 0,
        bin_seq: [0; 6],
        bin_rep: [0; 6],
        bin_data: [0; 24],
        result__interrupt_status: 0,
        result__range_status: 0,
        result__report_status: 0,
        result__stream_count: 0,
        result__dss_actual_effective_spads: 0,
        phasecal_result__reference_phase: 0,
        phasecal_result__vcsel_start: 0,
        cal_config__vcsel_start: 0,
        vcsel_width: 0,
        VL53LX_p_005: 0,
        VL53LX_p_015: 0,
        total_periods_elapsed: 0,
        peak_duration_us: 0,
        woi_duration_us: 0,
        min_bin_value: 0,
        max_bin_value: 0,
        zero_distance_phase: 0,
        number_of_ambient_samples: 0,
        ambient_events_sum: 0,
        VL53LX_p_028: 0,
        roi_config__user_roi_centre_spad: 0,
        roi_config__user_roi_requested_global_xy_size: 0,
    };
    let mut pavg_bins: *mut VL53LX_histogram_bin_data_t = &mut avg_bins;
    memcpy(
        pavg_bins as *mut libc::c_void,
        &mut (*pxtalk_results).central_histogram_avg as *mut VL53LX_histogram_bin_data_t
            as *const libc::c_void,
        ::std::mem::size_of::<VL53LX_histogram_bin_data_t>() as libc::c_ulong,
    );
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        VL53LX_init_histogram_bin_data_struct(
            0 as libc::c_int,
            0 as libc::c_int as u16,
            &mut (*pdebug).VL53LX_p_056,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        VL53LX_init_histogram_bin_data_struct(
            0 as libc::c_int,
            0 as libc::c_int as u16,
            &mut (*pdebug).VL53LX_p_057,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_f_039(
            pxtalk_results,
            pdebug,
            &mut (*pxtalk_cal).algo__crosstalk_compensation_x_plane_gradient_kcps,
            &mut (*pxtalk_cal).algo__crosstalk_compensation_y_plane_gradient_kcps,
        );
    }
    if !(status as libc::c_int != 0 as libc::c_int as VL53LX_Error as libc::c_int) {
        pxtalk_data = &mut *((*pxtalk_results).VL53LX_p_003)
            .as_mut_ptr()
            .offset(4 as libc::c_int as isize) as *mut VL53LX_xtalk_range_data_t;
        if (*pxtalk_data).no_of_samples as libc::c_int > 0 as libc::c_int {
            if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
                memcpy(
                    &mut (*pdebug).VL53LX_p_056 as *mut VL53LX_histogram_bin_data_t
                        as *mut libc::c_void,
                    pavg_bins as *const libc::c_void,
                    ::std::mem::size_of::<VL53LX_histogram_bin_data_t>() as libc::c_ulong,
                );
            }
            status = VL53LX_f_040(
                pxtalk_data,
                pdebug,
                &mut (*pxtalk_cal).algo__crosstalk_compensation_plane_offset_kcps,
            );
            if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
                status = VL53LX_f_041(
                    pavg_bins,
                    pdebug,
                    pxtalk_data,
                    (*pxtalk_results).central_histogram__window_start,
                    (*pxtalk_results).central_histogram__window_end,
                    &mut (*pxtalk_shape).xtalk_shape,
                );
            }
        } else {
            (*pxtalk_cal)
                .algo__crosstalk_compensation_plane_offset_kcps = 0 as libc::c_int
                as u32;
            (*pdebug).VL53LX_p_058 = 0 as libc::c_int as u32;
        }
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_f_041(
    mut pavg_bins: *mut VL53LX_histogram_bin_data_t,
    mut pdebug: *mut VL53LX_xtalk_algo_data_t,
    mut pxtalk_data: *mut VL53LX_xtalk_range_data_t,
    mut histogram__window_start: u8,
    mut histogram__window_end: u8,
    mut pxtalk_shape: *mut VL53LX_xtalk_histogram_shape_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut ambient_thresh: u32 = 0 as libc::c_int as u32;
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        VL53LX_f_029(pavg_bins, (*pavg_bins).VL53LX_p_028);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        VL53LX_f_043(
            6 as libc::c_int as u8,
            (*pavg_bins).VL53LX_p_028,
            &mut ambient_thresh,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_f_044(
            pavg_bins,
            ambient_thresh,
            histogram__window_start,
            histogram__window_end,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_f_045(pavg_bins, pxtalk_data, pdebug, pxtalk_shape);
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_f_039(
    mut pxtalk_results: *mut VL53LX_xtalk_range_results_t,
    mut pdebug: *mut VL53LX_xtalk_algo_data_t,
    mut xgradient: *mut i16,
    mut ygradient: *mut i16,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut presults_int: *mut VL53LX_xtalk_range_data_t = 0
        as *mut VL53LX_xtalk_range_data_t;
    let mut i: libc::c_int = 0 as libc::c_int;
    let mut xtalk_per_spad: [u32; 4] = [0; 4];
    let mut VL53LX_p_059: i32 = 0 as libc::c_int;
    let mut VL53LX_p_060: i32 = 0 as libc::c_int;
    let mut result_invalid: u8 = 0 as libc::c_int as u8;
    *xgradient = 0 as libc::c_int as i16;
    *ygradient = 0 as libc::c_int as i16;
    i = 0 as libc::c_int;
    while i < 4 as libc::c_int {
        xtalk_per_spad[i as usize] = 0 as libc::c_int as u32;
        i += 1;
    }
    i = 0 as libc::c_int;
    while i < 4 as libc::c_int {
        if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
            presults_int = &mut *((*pxtalk_results).VL53LX_p_003)
                .as_mut_ptr()
                .offset(i as isize) as *mut VL53LX_xtalk_range_data_t;
            if (*presults_int).no_of_samples as libc::c_int == 0 as libc::c_int {
                result_invalid = 1 as libc::c_int as u8;
                (*pdebug).VL53LX_p_061[i as usize] = 0 as libc::c_int as u32;
            } else {
                xtalk_per_spad[i as usize] = (*presults_int).rate_per_spad_kcps_avg;
                (*pdebug).VL53LX_p_061[i as usize] = xtalk_per_spad[i as usize];
            }
        }
        i += 1;
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int
        && result_invalid as libc::c_int == 0 as libc::c_int
    {
        if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
            VL53LX_p_059 = (xtalk_per_spad[1 as libc::c_int as usize] as i32
                - xtalk_per_spad[0 as libc::c_int as usize] as i32)
                / 8 as libc::c_int;
            VL53LX_p_060 = (xtalk_per_spad[3 as libc::c_int as usize] as i32
                - xtalk_per_spad[2 as libc::c_int as usize] as i32)
                / 8 as libc::c_int;
        }
        if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
            if VL53LX_p_059 < -(32767 as libc::c_int) {
                VL53LX_p_059 = -(32767 as libc::c_int);
            } else if VL53LX_p_059 > 32767 as libc::c_int {
                VL53LX_p_059 = 32767 as libc::c_int;
            }
            if VL53LX_p_060 < -(32767 as libc::c_int) {
                VL53LX_p_060 = -(32767 as libc::c_int);
            } else if VL53LX_p_060 > 32767 as libc::c_int {
                VL53LX_p_060 = 32767 as libc::c_int;
            }
            (*pdebug).VL53LX_p_059 = VL53LX_p_059 as i16;
            (*pdebug).VL53LX_p_060 = VL53LX_p_060 as i16;
        }
    } else {
        VL53LX_p_059 = 0 as libc::c_int;
        VL53LX_p_060 = 0 as libc::c_int;
        (*pdebug).VL53LX_p_059 = 0 as libc::c_int as i16;
        (*pdebug).VL53LX_p_060 = 0 as libc::c_int as i16;
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        *xgradient = VL53LX_p_059 as i16;
        *ygradient = VL53LX_p_060 as i16;
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_f_040(
    mut pxtalk_data: *mut VL53LX_xtalk_range_data_t,
    mut pdebug: *mut VL53LX_xtalk_algo_data_t,
    mut xtalk_mean_offset_kcps: *mut u32,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut xtalk_per_spad: u32 = 0 as libc::c_int as u32;
    let mut result_invalid: u8 = 0 as libc::c_int as u8;
    *xtalk_mean_offset_kcps = 0 as libc::c_int as u32;
    if (*pxtalk_data).no_of_samples as libc::c_int == 0 as libc::c_int {
        result_invalid = 1 as libc::c_int as u8;
        (*pdebug).VL53LX_p_058 = 0 as libc::c_int as u32;
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int
        && result_invalid as libc::c_int == 0 as libc::c_int
    {
        xtalk_per_spad = (*pxtalk_data).rate_per_spad_kcps_avg >> 2 as libc::c_int;
        (*pdebug).VL53LX_p_058 = xtalk_per_spad;
        if xtalk_per_spad < 0x3ffff as libc::c_int as libc::c_uint {
            *xtalk_mean_offset_kcps = xtalk_per_spad;
        } else {
            *xtalk_mean_offset_kcps = 0x3ffff as libc::c_int as u32;
        }
    } else {
        *xtalk_mean_offset_kcps = 0 as libc::c_int as u32;
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_f_045(
    mut phist_data: *mut VL53LX_histogram_bin_data_t,
    mut pxtalk_data: *mut VL53LX_xtalk_range_data_t,
    mut pdebug: *mut VL53LX_xtalk_algo_data_t,
    mut pxtalk_histo: *mut VL53LX_xtalk_histogram_shape_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut idx: u8 = 0;
    let mut tmpi32: i32 = 0;
    let mut i: u8 = 0 as libc::c_int as u8;
    let mut bin_data: [u64; 12] = [0; 12];
    (*pxtalk_histo).VL53LX_p_020 = (*phist_data).VL53LX_p_020;
    (*pxtalk_histo).cal_config__vcsel_start = (*phist_data).cal_config__vcsel_start;
    (*pxtalk_histo).VL53LX_p_015 = (*phist_data).VL53LX_p_015;
    (*pxtalk_histo).VL53LX_p_019 = (*phist_data).VL53LX_p_019;
    (*pxtalk_histo).time_stamp = (*phist_data).time_stamp;
    (*pxtalk_histo).vcsel_width = (*phist_data).vcsel_width;
    (*pxtalk_histo).zero_distance_phase = (*phist_data).zero_distance_phase;
    (*pxtalk_histo).zone_id = (*phist_data).zone_id;
    (*pxtalk_histo).VL53LX_p_021 = 12 as libc::c_int as u8;
    (*pxtalk_histo)
        .phasecal_result__reference_phase = (*phist_data)
        .phasecal_result__reference_phase;
    (*pxtalk_histo)
        .phasecal_result__vcsel_start = (*phist_data).phasecal_result__vcsel_start;
    memcpy(
        &mut (*pdebug).VL53LX_p_057 as *mut VL53LX_histogram_bin_data_t
            as *mut libc::c_void,
        phist_data as *const libc::c_void,
        ::std::mem::size_of::<VL53LX_histogram_bin_data_t>() as libc::c_ulong,
    );
    if (*pxtalk_data).signal_total_events_avg == 0 as libc::c_int {
        i = 0 as libc::c_int as u8;
        while (i as libc::c_int) < (*pxtalk_histo).VL53LX_p_021 as libc::c_int {
            bin_data[i as usize] = 0 as libc::c_int as u64;
            i = i.wrapping_add(1);
        }
    } else {
        i = 0 as libc::c_int as u8;
        while (i as libc::c_int) < (*pxtalk_histo).VL53LX_p_021 as libc::c_int {
            idx = (i as libc::c_int
                + (*phist_data).number_of_ambient_bins as libc::c_int) as u8;
            if (*phist_data).bin_data[idx as usize] > 0 as libc::c_int {
                bin_data[i
                    as usize] = ((*phist_data).bin_data[idx as usize] as u64)
                    << 10 as libc::c_int;
                tmpi32 = (*pxtalk_data).signal_total_events_avg / 2 as libc::c_int;
                bin_data[i
                    as usize] = (bin_data[i as usize]).wrapping_add(tmpi32 as u64);
                bin_data[i
                    as usize] = (bin_data[i as usize])
                    .wrapping_div((*pxtalk_data).signal_total_events_avg as u64);
            } else {
                bin_data[i as usize] = 0 as libc::c_int as u64;
            }
            i = i.wrapping_add(1);
        }
    }
    i = 0 as libc::c_int as u8;
    while (i as libc::c_int) < 12 as libc::c_int {
        (*pxtalk_histo).bin_data[i as usize] = bin_data[i as usize] as u32;
        i = i.wrapping_add(1);
    }
    i = 0 as libc::c_int as u8;
    while (i as libc::c_int) < (*pxtalk_histo).VL53LX_p_021 as libc::c_int {
        (*pdebug).VL53LX_p_062[i as usize] = (*pxtalk_histo).bin_data[i as usize];
        i = i.wrapping_add(1);
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_f_046(
    mut pcustomer: *mut VL53LX_customer_nvm_managed_t,
    mut pdyn_cfg: *mut VL53LX_dynamic_config_t,
    mut pxtalk_shape: *mut VL53LX_xtalk_histogram_data_t,
    mut pip_hist_data: *mut VL53LX_histogram_bin_data_t,
    mut pop_hist_data: *mut VL53LX_histogram_bin_data_t,
    mut pxtalk_count_data: *mut VL53LX_histogram_bin_data_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut xtalk_rate_kcps: u32 = 0 as libc::c_int as u32;
    memcpy(
        pop_hist_data as *mut libc::c_void,
        pip_hist_data as *const libc::c_void,
        ::std::mem::size_of::<VL53LX_histogram_bin_data_t>() as libc::c_ulong,
    );
    status = VL53LX_f_032(
        (*pcustomer).algo__crosstalk_compensation_plane_offset_kcps as u32,
        (*pcustomer).algo__crosstalk_compensation_x_plane_gradient_kcps,
        (*pcustomer).algo__crosstalk_compensation_y_plane_gradient_kcps,
        0 as libc::c_int as i8,
        0 as libc::c_int as i8,
        (*pip_hist_data).result__dss_actual_effective_spads,
        (*pdyn_cfg).roi_config__user_roi_centre_spad,
        (*pdyn_cfg).roi_config__user_roi_requested_global_xy_size,
        &mut xtalk_rate_kcps,
    );
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_f_033(
            pip_hist_data,
            &mut (*pxtalk_shape).xtalk_shape,
            xtalk_rate_kcps,
            pxtalk_count_data,
        );
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        status = VL53LX_f_047(
            pop_hist_data,
            pxtalk_count_data,
            (*pip_hist_data).number_of_ambient_bins,
        );
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_f_032(
    mut mean_offset: u32,
    mut xgradient: i16,
    mut ygradient: i16,
    mut centre_offset_x: i8,
    mut centre_offset_y: i8,
    mut roi_effective_spads: u16,
    mut roi_centre_spad: u8,
    mut roi_xy_size: u8,
    mut xtalk_rate_kcps: *mut u32,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut row: u8 = 0 as libc::c_int as u8;
    let mut col: u8 = 0 as libc::c_int as u8;
    let mut bound_l_x: i16 = 0 as libc::c_int as i16;
    let mut bound_r_x: i16 = 0 as libc::c_int as i16;
    let mut bound_u_y: i16 = 0 as libc::c_int as i16;
    let mut bound_d_y: i16 = 0 as libc::c_int as i16;
    let mut xtalk_rate_ll: i64 = 0 as libc::c_int as i64;
    let mut xtalk_rate_ur: i64 = 0 as libc::c_int as i64;
    let mut xtalk_avg: i64 = 0 as libc::c_int as i64;
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        VL53LX_decode_row_col(roi_centre_spad, &mut row, &mut col);
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        if roi_xy_size as i16 as libc::c_int / 16 as libc::c_int & 0x1 as libc::c_int
            == 1 as libc::c_int
        {
            bound_l_x = (col as i16 as libc::c_int
                - (roi_xy_size as i16 as libc::c_int / 32 as libc::c_int
                    + 1 as libc::c_int)) as i16;
        } else {
            bound_l_x = (col as i16 as libc::c_int
                - roi_xy_size as i16 as libc::c_int / 32 as libc::c_int) as i16;
        }
        bound_r_x = (col as i16 as libc::c_int
            + roi_xy_size as i16 as libc::c_int / 32 as libc::c_int) as i16;
        if roi_xy_size as i16 as libc::c_int & 0x1 as libc::c_int == 1 as libc::c_int
        {
            bound_d_y = (row as i16 as libc::c_int
                - ((roi_xy_size as i16 as libc::c_int & 0xf as libc::c_int)
                    / 2 as libc::c_int + 1 as libc::c_int)) as i16;
        } else {
            bound_d_y = (row as i16 as libc::c_int
                - (roi_xy_size as i16 as libc::c_int & 0xf as libc::c_int)
                    / 2 as libc::c_int) as i16;
        }
        bound_u_y = (row as i16 as libc::c_int
            + (roi_xy_size as i16 as libc::c_int & 0xf as libc::c_int)
                / 2 as libc::c_int) as i16;
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        bound_l_x = (2 as libc::c_int * bound_l_x as libc::c_int - 15 as libc::c_int
            + 2 as libc::c_int * centre_offset_x as i16 as libc::c_int) as i16;
        bound_r_x = (2 as libc::c_int * bound_r_x as libc::c_int - 15 as libc::c_int
            + 2 as libc::c_int * centre_offset_x as i16 as libc::c_int) as i16;
        bound_u_y = (2 as libc::c_int * bound_u_y as libc::c_int - 15 as libc::c_int
            + 2 as libc::c_int * centre_offset_y as i16 as libc::c_int) as i16;
        bound_d_y = (2 as libc::c_int * bound_d_y as libc::c_int - 15 as libc::c_int
            + 2 as libc::c_int * centre_offset_y as i16 as libc::c_int) as i16;
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        xtalk_rate_ll = bound_l_x as i64 * xgradient as i64
            + bound_d_y as i64 * ygradient as i64;
        xtalk_rate_ll = (xtalk_rate_ll + 1 as libc::c_int as libc::c_long)
            / 2 as libc::c_int as libc::c_long;
        xtalk_rate_ll += mean_offset as i64 * 4 as libc::c_int as libc::c_long;
        xtalk_rate_ur = bound_r_x as i64 * xgradient as i64
            + bound_u_y as i64 * ygradient as i64;
        xtalk_rate_ur = (xtalk_rate_ur + 1 as libc::c_int as libc::c_long)
            / 2 as libc::c_int as libc::c_long;
        xtalk_rate_ur += mean_offset as i64 * 4 as libc::c_int as libc::c_long;
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        xtalk_avg = (xtalk_rate_ll + xtalk_rate_ur + 1 as libc::c_int as libc::c_long)
            / 2 as libc::c_int as libc::c_long;
    }
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        if xtalk_avg < 0 as libc::c_int as libc::c_long {
            xtalk_avg = 0 as libc::c_int as i64;
        }
    }
    *xtalk_rate_kcps = xtalk_avg as u32;
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_f_033(
    mut phist_data: *mut VL53LX_histogram_bin_data_t,
    mut pxtalk_data: *mut VL53LX_xtalk_histogram_shape_t,
    mut xtalk_rate_kcps: u32,
    mut pxtalkcount_data: *mut VL53LX_histogram_bin_data_t,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut xtalk_events_per_spad: u64 = 0 as libc::c_int as u64;
    let mut xtalk_total_events: u64 = 0 as libc::c_int as u64;
    let mut xtalk_temp_bin: u64 = 0 as libc::c_int as u64;
    let mut i: u8 = 0 as libc::c_int as u8;
    xtalk_events_per_spad = (xtalk_rate_kcps as u64)
        .wrapping_mul((*phist_data).peak_duration_us as u64)
        .wrapping_add(500 as libc::c_int as libc::c_ulong)
        .wrapping_div(1000 as libc::c_int as libc::c_ulong);
    xtalk_total_events = xtalk_events_per_spad
        .wrapping_mul((*phist_data).result__dss_actual_effective_spads as u64);
    xtalk_total_events = xtalk_total_events
        .wrapping_div(256 as libc::c_int as libc::c_ulong);
    xtalk_total_events = xtalk_total_events
        .wrapping_add(1024 as libc::c_int as libc::c_ulong)
        .wrapping_div(2048 as libc::c_int as libc::c_ulong);
    if xtalk_total_events > 0xffffffff as libc::c_uint as libc::c_ulong {
        xtalk_total_events = 0xffffffff as libc::c_uint as u64;
    }
    i = 0 as libc::c_int as u8;
    while (i as libc::c_int) < (*pxtalk_data).VL53LX_p_021 as libc::c_int {
        xtalk_temp_bin = ((*pxtalk_data).bin_data[i as usize] as u64)
            .wrapping_mul(xtalk_total_events);
        xtalk_temp_bin = xtalk_temp_bin
            .wrapping_add(512 as libc::c_int as libc::c_ulong)
            .wrapping_div(1024 as libc::c_int as libc::c_ulong);
        (*pxtalkcount_data).bin_data[i as usize] = xtalk_temp_bin as u32 as i32;
        i = i.wrapping_add(1);
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_f_047(
    mut phist_data: *mut VL53LX_histogram_bin_data_t,
    mut pxtalk_data: *mut VL53LX_histogram_bin_data_t,
    mut xtalk_bin_offset: u8,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut i: u8 = 0 as libc::c_int as u8;
    let mut temp_bin: i32 = 0;
    if status as libc::c_int == 0 as libc::c_int as VL53LX_Error as libc::c_int {
        i = xtalk_bin_offset;
        while (i as libc::c_int) < (*pxtalk_data).VL53LX_p_021 as libc::c_int {
            temp_bin = (*phist_data).bin_data[i as usize]
                - (*pxtalk_data)
                    .bin_data[(i as libc::c_int - xtalk_bin_offset as libc::c_int)
                    as usize];
            if temp_bin < 0 as libc::c_int {
                temp_bin = 0 as libc::c_int;
            }
            (*phist_data).bin_data[i as usize] = temp_bin as u32 as i32;
            i = i.wrapping_add(1);
        }
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_f_044(
    mut pxtalk_data: *mut VL53LX_histogram_bin_data_t,
    mut amb_threshold: u32,
    mut VL53LX_p_019: u8,
    mut VL53LX_p_024: u8,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut i: u8 = 0 as libc::c_int as u8;
    let mut first_bin_int: u8 = 0 as libc::c_int as u8;
    let mut first_bin_inc: u8 = 0 as libc::c_int as u8;
    let mut last_bin_int: u8 = 0 as libc::c_int as u8;
    let mut realign_bin: u8 = 0 as libc::c_int as u8;
    let mut realign_index: u8 = 0 as libc::c_int as u8;
    let mut realign_bin_data: [i32; 24] = [0; 24];
    i = 0 as libc::c_int as u8;
    while (i as libc::c_int) < 24 as libc::c_int {
        realign_bin_data[i as usize] = 0 as libc::c_int;
        i = i.wrapping_add(1);
    }
    first_bin_int = VL53LX_p_019;
    last_bin_int = VL53LX_p_024;
    VL53LX_hist_remove_ambient_bins(pxtalk_data);
    first_bin_int = (first_bin_int as libc::c_int
        % (*pxtalk_data).VL53LX_p_021 as libc::c_int) as u8;
    last_bin_int = (last_bin_int as libc::c_int
        % (*pxtalk_data).VL53LX_p_021 as libc::c_int) as u8;
    first_bin_inc = ((first_bin_int as libc::c_int + 1 as libc::c_int)
        % (*pxtalk_data).VL53LX_p_021 as libc::c_int) as u8;
    if first_bin_inc as libc::c_int > last_bin_int as libc::c_int {
        realign_bin = ((*pxtalk_data).VL53LX_p_021 as libc::c_int
            - first_bin_inc as libc::c_int) as u8;
        first_bin_int = ((first_bin_int as libc::c_int + realign_bin as libc::c_int)
            % (*pxtalk_data).VL53LX_p_021 as libc::c_int) as u8;
        last_bin_int = ((last_bin_int as libc::c_int + realign_bin as libc::c_int)
            % (*pxtalk_data).VL53LX_p_021 as libc::c_int) as u8;
        (*pxtalk_data)
            .zero_distance_phase = ((*pxtalk_data).zero_distance_phase as libc::c_int
            + realign_bin as u16 as libc::c_int * 2048 as libc::c_int) as u16;
    }
    if realign_bin as libc::c_int > 0 as libc::c_int {
        i = 0 as libc::c_int as u8;
        while (i as libc::c_int) < (*pxtalk_data).VL53LX_p_021 as libc::c_int {
            realign_bin_data[i as usize] = (*pxtalk_data).bin_data[i as usize];
            i = i.wrapping_add(1);
        }
        i = 0 as libc::c_int as u8;
        while (i as libc::c_int) < (*pxtalk_data).VL53LX_p_021 as libc::c_int {
            realign_index = (((*pxtalk_data).VL53LX_p_021 as libc::c_int
                - realign_bin as libc::c_int + i as libc::c_int)
                % (*pxtalk_data).VL53LX_p_021 as libc::c_int) as u8;
            (*pxtalk_data)
                .bin_data[i as usize] = realign_bin_data[realign_index as usize];
            i = i.wrapping_add(1);
        }
    }
    i = 0 as libc::c_int as u8;
    while (i as libc::c_int) < (*pxtalk_data).VL53LX_p_021 as libc::c_int {
        if first_bin_int as libc::c_int <= last_bin_int as libc::c_int {
            if i as libc::c_int >= first_bin_int as libc::c_int
                && i as libc::c_int <= last_bin_int as libc::c_int
            {
                if (*pxtalk_data).bin_data[i as usize] < amb_threshold as i32 {
                    (*pxtalk_data).bin_data[i as usize] = 0 as libc::c_int;
                }
            } else {
                (*pxtalk_data).bin_data[i as usize] = 0 as libc::c_int;
            }
        } else if i as libc::c_int >= first_bin_int as libc::c_int
                || i as libc::c_int <= last_bin_int as libc::c_int
            {
            if (*pxtalk_data).bin_data[i as usize] < amb_threshold as i32 {
                (*pxtalk_data).bin_data[i as usize] = 0 as libc::c_int;
            }
        } else {
            (*pxtalk_data).bin_data[i as usize] = 0 as libc::c_int;
        }
        i = i.wrapping_add(1);
    }
    return status;
}
#[no_mangle]
pub unsafe extern "C" fn VL53LX_f_043(
    mut sigma_mult: u8,
    mut VL53LX_p_028: i32,
    mut ambient_noise: *mut u32,
) -> VL53LX_Error {
    let mut status: VL53LX_Error = 0 as libc::c_int as VL53LX_Error;
    let mut ambient_events_per_bin_int: u32 = 0 as libc::c_int as u32;
    if VL53LX_p_028 <= 0 as libc::c_int {
        ambient_events_per_bin_int = 1 as libc::c_int as u32;
    } else {
        ambient_events_per_bin_int = VL53LX_p_028 as u32;
    }
    *ambient_noise = VL53LX_isqrt(ambient_events_per_bin_int);
    *ambient_noise = (*ambient_noise).wrapping_mul(sigma_mult as u32);
    return status;
}
