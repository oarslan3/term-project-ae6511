/* Include files */

#include "blascompat32.h"
#include "car_model_friction_sfun.h"
#include "c2_car_model_friction.h"
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "car_model_friction_sfun_debug_macros.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */
static const char *c2_debug_family_names[50] = { "g", "m", "Iz", "l_F", "l_R",
  "B", "C", "mu", "f_Fz", "f_Rz", "kmh2ms", "ms2kmh", "deg2rad", "rad2deg",
  "f_Fx_min", "f_Fx_max", "f_Rx_min", "f_Rx_max", "Vx", "Vy", "r", "psi", "xi",
  "yi", "f_Fx", "f_Rx", "delta", "Vxw", "Vyw", "V", "beta", "f_Fy_max", "s_Fy",
  "f_Fy", "f_Ry_max", "s_Ry", "f_Ry", "dVx", "dVy", "dr", "dpsi", "dxi", "dyi",
  "nargin", "nargout", "X", "u", "dX", "f_F", "s" };

/* Function Declarations */
static void initialize_c2_car_model_friction
  (SFc2_car_model_frictionInstanceStruct *chartInstance);
static void initialize_params_c2_car_model_friction
  (SFc2_car_model_frictionInstanceStruct *chartInstance);
static void enable_c2_car_model_friction(SFc2_car_model_frictionInstanceStruct
  *chartInstance);
static void disable_c2_car_model_friction(SFc2_car_model_frictionInstanceStruct *
  chartInstance);
static void c2_update_debugger_state_c2_car_model_friction
  (SFc2_car_model_frictionInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c2_car_model_friction
  (SFc2_car_model_frictionInstanceStruct *chartInstance);
static void set_sim_state_c2_car_model_friction
  (SFc2_car_model_frictionInstanceStruct *chartInstance, const mxArray *c2_st);
static void finalize_c2_car_model_friction(SFc2_car_model_frictionInstanceStruct
  *chartInstance);
static void sf_c2_car_model_friction(SFc2_car_model_frictionInstanceStruct
  *chartInstance);
static void c2_c2_car_model_friction(SFc2_car_model_frictionInstanceStruct
  *chartInstance);
static void init_script_number_translation(uint32_T c2_machineNumber, uint32_T
  c2_chartNumber);
static real_T c2_sign(SFc2_car_model_frictionInstanceStruct *chartInstance,
                      real_T c2_x);
static real_T c2_power(SFc2_car_model_frictionInstanceStruct *chartInstance,
  real_T c2_a, real_T c2_b);
static void c2_eml_scalar_eg(SFc2_car_model_frictionInstanceStruct
  *chartInstance);
static void c2_eml_error(SFc2_car_model_frictionInstanceStruct *chartInstance);
static real_T c2_sqrt(SFc2_car_model_frictionInstanceStruct *chartInstance,
                      real_T c2_x);
static void c2_b_eml_error(SFc2_car_model_frictionInstanceStruct *chartInstance);
static real_T c2_atan2(SFc2_car_model_frictionInstanceStruct *chartInstance,
  real_T c2_y, real_T c2_x);
static const mxArray *c2_sf_marshall(void *chartInstanceVoid, void *c2_u);
static const mxArray *c2_b_sf_marshall(void *chartInstanceVoid, void *c2_u);
static const mxArray *c2_c_sf_marshall(void *chartInstanceVoid, void *c2_u);
static const mxArray *c2_d_sf_marshall(void *chartInstanceVoid, void *c2_u);
static void c2_info_helper(c2_ResolvedFunctionInfo c2_info[52]);
static const mxArray *c2_e_sf_marshall(void *chartInstanceVoid, void *c2_u);
static void c2_emlrt_marshallIn(SFc2_car_model_frictionInstanceStruct
  *chartInstance, const mxArray *c2_dX, const char_T *c2_name, real_T c2_y[6]);
static void c2_b_emlrt_marshallIn(SFc2_car_model_frictionInstanceStruct
  *chartInstance, const mxArray *c2_f_F, const char_T *c2_name, real_T c2_y[2]);
static uint8_T c2_c_emlrt_marshallIn(SFc2_car_model_frictionInstanceStruct
  *chartInstance, const mxArray *c2_b_is_active_c2_car_model_friction, const
  char_T *c2_name);
static void init_dsm_address_info(SFc2_car_model_frictionInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c2_car_model_friction
  (SFc2_car_model_frictionInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  chartInstance->c2_is_active_c2_car_model_friction = 0U;
}

static void initialize_params_c2_car_model_friction
  (SFc2_car_model_frictionInstanceStruct *chartInstance)
{
}

static void enable_c2_car_model_friction(SFc2_car_model_frictionInstanceStruct
  *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void disable_c2_car_model_friction(SFc2_car_model_frictionInstanceStruct *
  chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void c2_update_debugger_state_c2_car_model_friction
  (SFc2_car_model_frictionInstanceStruct *chartInstance)
{
}

static const mxArray *get_sim_state_c2_car_model_friction
  (SFc2_car_model_frictionInstanceStruct *chartInstance)
{
  const mxArray *c2_st = NULL;
  const mxArray *c2_y = NULL;
  int32_T c2_i0;
  real_T c2_hoistedGlobal[6];
  int32_T c2_i1;
  real_T c2_u[6];
  const mxArray *c2_b_y = NULL;
  int32_T c2_i2;
  real_T c2_b_hoistedGlobal[2];
  int32_T c2_i3;
  real_T c2_b_u[2];
  const mxArray *c2_c_y = NULL;
  int32_T c2_i4;
  real_T c2_c_hoistedGlobal[2];
  int32_T c2_i5;
  real_T c2_c_u[2];
  const mxArray *c2_d_y = NULL;
  uint8_T c2_d_hoistedGlobal;
  uint8_T c2_d_u;
  const mxArray *c2_e_y = NULL;
  real_T (*c2_s)[2];
  real_T (*c2_f_F)[2];
  real_T (*c2_dX)[6];
  c2_s = (real_T (*)[2])ssGetOutputPortSignal(chartInstance->S, 3);
  c2_f_F = (real_T (*)[2])ssGetOutputPortSignal(chartInstance->S, 2);
  c2_dX = (real_T (*)[6])ssGetOutputPortSignal(chartInstance->S, 1);
  c2_st = NULL;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_createcellarray(4));
  for (c2_i0 = 0; c2_i0 < 6; c2_i0 = c2_i0 + 1) {
    c2_hoistedGlobal[c2_i0] = (*c2_dX)[c2_i0];
  }

  for (c2_i1 = 0; c2_i1 < 6; c2_i1 = c2_i1 + 1) {
    c2_u[c2_i1] = c2_hoistedGlobal[c2_i1];
  }

  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", &c2_u, 0, 0U, 1U, 0U, 1, 6));
  sf_mex_setcell(c2_y, 0, c2_b_y);
  for (c2_i2 = 0; c2_i2 < 2; c2_i2 = c2_i2 + 1) {
    c2_b_hoistedGlobal[c2_i2] = (*c2_f_F)[c2_i2];
  }

  for (c2_i3 = 0; c2_i3 < 2; c2_i3 = c2_i3 + 1) {
    c2_b_u[c2_i3] = c2_b_hoistedGlobal[c2_i3];
  }

  c2_c_y = NULL;
  sf_mex_assign(&c2_c_y, sf_mex_create("y", &c2_b_u, 0, 0U, 1U, 0U, 1, 2));
  sf_mex_setcell(c2_y, 1, c2_c_y);
  for (c2_i4 = 0; c2_i4 < 2; c2_i4 = c2_i4 + 1) {
    c2_c_hoistedGlobal[c2_i4] = (*c2_s)[c2_i4];
  }

  for (c2_i5 = 0; c2_i5 < 2; c2_i5 = c2_i5 + 1) {
    c2_c_u[c2_i5] = c2_c_hoistedGlobal[c2_i5];
  }

  c2_d_y = NULL;
  sf_mex_assign(&c2_d_y, sf_mex_create("y", &c2_c_u, 0, 0U, 1U, 0U, 1, 2));
  sf_mex_setcell(c2_y, 2, c2_d_y);
  c2_d_hoistedGlobal = chartInstance->c2_is_active_c2_car_model_friction;
  c2_d_u = c2_d_hoistedGlobal;
  c2_e_y = NULL;
  sf_mex_assign(&c2_e_y, sf_mex_create("y", &c2_d_u, 3, 0U, 0U, 0U, 0));
  sf_mex_setcell(c2_y, 3, c2_e_y);
  sf_mex_assign(&c2_st, c2_y);
  return c2_st;
}

static void set_sim_state_c2_car_model_friction
  (SFc2_car_model_frictionInstanceStruct *chartInstance, const mxArray *c2_st)
{
  const mxArray *c2_u;
  real_T c2_dv0[6];
  int32_T c2_i6;
  real_T c2_dv1[2];
  int32_T c2_i7;
  real_T c2_dv2[2];
  int32_T c2_i8;
  real_T (*c2_dX)[6];
  real_T (*c2_f_F)[2];
  real_T (*c2_s)[2];
  c2_s = (real_T (*)[2])ssGetOutputPortSignal(chartInstance->S, 3);
  c2_f_F = (real_T (*)[2])ssGetOutputPortSignal(chartInstance->S, 2);
  c2_dX = (real_T (*)[6])ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c2_doneDoubleBufferReInit = TRUE;
  c2_u = sf_mex_dup(c2_st);
  c2_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 0)), "dX",
                      c2_dv0);
  for (c2_i6 = 0; c2_i6 < 6; c2_i6 = c2_i6 + 1) {
    (*c2_dX)[c2_i6] = c2_dv0[c2_i6];
  }

  c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 1)),
                        "f_F", c2_dv1);
  for (c2_i7 = 0; c2_i7 < 2; c2_i7 = c2_i7 + 1) {
    (*c2_f_F)[c2_i7] = c2_dv1[c2_i7];
  }

  c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 2)), "s",
                        c2_dv2);
  for (c2_i8 = 0; c2_i8 < 2; c2_i8 = c2_i8 + 1) {
    (*c2_s)[c2_i8] = c2_dv2[c2_i8];
  }

  chartInstance->c2_is_active_c2_car_model_friction = c2_c_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 3)),
     "is_active_c2_car_model_friction");
  sf_mex_destroy(&c2_u);
  c2_update_debugger_state_c2_car_model_friction(chartInstance);
  sf_mex_destroy(&c2_st);
}

static void finalize_c2_car_model_friction(SFc2_car_model_frictionInstanceStruct
  *chartInstance)
{
}

static void sf_c2_car_model_friction(SFc2_car_model_frictionInstanceStruct
  *chartInstance)
{
  int32_T c2_i9;
  int32_T c2_i10;
  int32_T c2_i11;
  int32_T c2_i12;
  int32_T c2_i13;
  int32_T c2_previousEvent;
  real_T (*c2_s)[2];
  real_T (*c2_f_F)[2];
  real_T (*c2_dX)[6];
  real_T (*c2_u)[3];
  real_T (*c2_X)[6];
  c2_s = (real_T (*)[2])ssGetOutputPortSignal(chartInstance->S, 3);
  c2_f_F = (real_T (*)[2])ssGetOutputPortSignal(chartInstance->S, 2);
  c2_dX = (real_T (*)[6])ssGetOutputPortSignal(chartInstance->S, 1);
  c2_u = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 1);
  c2_X = (real_T (*)[6])ssGetInputPortSignal(chartInstance->S, 0);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG,1);
  for (c2_i9 = 0; c2_i9 < 6; c2_i9 = c2_i9 + 1) {
    _SFD_DATA_RANGE_CHECK((*c2_X)[c2_i9], 0U);
  }

  for (c2_i10 = 0; c2_i10 < 3; c2_i10 = c2_i10 + 1) {
    _SFD_DATA_RANGE_CHECK((*c2_u)[c2_i10], 1U);
  }

  for (c2_i11 = 0; c2_i11 < 6; c2_i11 = c2_i11 + 1) {
    _SFD_DATA_RANGE_CHECK((*c2_dX)[c2_i11], 2U);
  }

  for (c2_i12 = 0; c2_i12 < 2; c2_i12 = c2_i12 + 1) {
    _SFD_DATA_RANGE_CHECK((*c2_f_F)[c2_i12], 3U);
  }

  for (c2_i13 = 0; c2_i13 < 2; c2_i13 = c2_i13 + 1) {
    _SFD_DATA_RANGE_CHECK((*c2_s)[c2_i13], 4U);
  }

  c2_previousEvent = _sfEvent_;
  _sfEvent_ = CALL_EVENT;
  c2_c2_car_model_friction(chartInstance);
  _sfEvent_ = c2_previousEvent;
  sf_debug_check_for_state_inconsistency(_car_model_frictionMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void c2_c2_car_model_friction(SFc2_car_model_frictionInstanceStruct
  *chartInstance)
{
  int32_T c2_i14;
  real_T c2_hoistedGlobal[6];
  int32_T c2_i15;
  real_T c2_b_hoistedGlobal[3];
  int32_T c2_i16;
  real_T c2_X[6];
  int32_T c2_i17;
  real_T c2_u[3];
  uint32_T c2_debug_family_var_map[50];
  real_T c2_g;
  real_T c2_m;
  real_T c2_Iz;
  real_T c2_l_F;
  real_T c2_l_R;
  real_T c2_B;
  real_T c2_C;
  real_T c2_mu;
  real_T c2_f_Fz;
  real_T c2_f_Rz;
  real_T c2_kmh2ms;
  real_T c2_ms2kmh;
  real_T c2_deg2rad;
  real_T c2_rad2deg;
  real_T c2_f_Fx_min;
  real_T c2_f_Fx_max;
  real_T c2_f_Rx_min;
  real_T c2_f_Rx_max;
  real_T c2_Vx;
  real_T c2_Vy;
  real_T c2_r;
  real_T c2_psi;
  real_T c2_xi;
  real_T c2_yi;
  real_T c2_f_Fx;
  real_T c2_f_Rx;
  real_T c2_delta;
  real_T c2_Vxw;
  real_T c2_Vyw;
  real_T c2_V;
  real_T c2_beta;
  real_T c2_f_Fy_max;
  real_T c2_s_Fy;
  real_T c2_f_Fy;
  real_T c2_f_Ry_max;
  real_T c2_s_Ry;
  real_T c2_f_Ry;
  real_T c2_dVx;
  real_T c2_dVy;
  real_T c2_dr;
  real_T c2_dpsi;
  real_T c2_dxi;
  real_T c2_dyi;
  real_T c2_nargin = 2.0;
  real_T c2_nargout = 3.0;
  real_T c2_dX[6];
  real_T c2_f_F[2];
  real_T c2_s[2];
  real_T c2_x;
  real_T c2_b_x;
  real_T c2_c_x;
  real_T c2_d_x;
  real_T c2_e_x;
  real_T c2_f_x;
  real_T c2_g_x;
  real_T c2_h_x;
  real_T c2_i_x;
  real_T c2_j_x;
  real_T c2_k_x;
  real_T c2_l_x;
  real_T c2_a;
  real_T c2_b;
  real_T c2_b_a;
  real_T c2_b_b;
  real_T c2_m_x;
  real_T c2_n_x;
  real_T c2_o_x;
  real_T c2_p_x;
  real_T c2_q_x;
  real_T c2_c_a;
  real_T c2_y;
  real_T c2_r_x;
  real_T c2_s_x;
  real_T c2_t_x;
  real_T c2_u_x;
  real_T c2_v_x;
  real_T c2_w_x;
  real_T c2_d_a;
  real_T c2_b_y;
  real_T c2_x_x;
  real_T c2_y_x;
  real_T c2_ab_x;
  real_T c2_bb_x;
  real_T c2_c_y;
  real_T c2_cb_x;
  real_T c2_d_y;
  real_T c2_db_x;
  real_T c2_e_y;
  real_T c2_c_b;
  real_T c2_f_y;
  real_T c2_eb_x;
  real_T c2_fb_x;
  real_T c2_gb_x;
  real_T c2_d_b;
  real_T c2_g_y;
  real_T c2_hb_x;
  real_T c2_ib_x;
  real_T c2_jb_x;
  real_T c2_kb_x;
  real_T c2_lb_x;
  real_T c2_mb_x;
  real_T c2_nb_x;
  real_T c2_ob_x;
  real_T c2_e_a;
  real_T c2_h_y;
  real_T c2_pb_x;
  real_T c2_qb_x;
  real_T c2_rb_x;
  real_T c2_sb_x;
  real_T c2_i_y;
  real_T c2_tb_x;
  real_T c2_j_y;
  real_T c2_ub_x;
  real_T c2_k_y;
  real_T c2_e_b;
  real_T c2_l_y;
  real_T c2_vb_x;
  real_T c2_wb_x;
  real_T c2_xb_x;
  real_T c2_f_b;
  real_T c2_m_y;
  real_T c2_yb_x;
  real_T c2_ac_x;
  real_T c2_bc_x;
  real_T c2_cc_x;
  real_T c2_dc_x;
  real_T c2_ec_x;
  real_T c2_fc_x;
  real_T c2_gc_x;
  real_T c2_hc_x;
  real_T c2_ic_x;
  real_T c2_jc_x;
  real_T c2_kc_x;
  real_T c2_lc_x;
  real_T c2_mc_x;
  real_T c2_nc_x;
  real_T c2_oc_x;
  real_T c2_pc_x;
  real_T c2_qc_x;
  real_T c2_rc_x;
  real_T c2_sc_x;
  real_T c2_tc_x;
  real_T c2_uc_x;
  real_T c2_vc_x;
  real_T c2_wc_x;
  real_T c2_xc_x;
  real_T c2_yc_x;
  real_T c2_ad_x;
  real_T c2_bd_x;
  real_T c2_cd_x;
  real_T c2_dd_x;
  real_T c2_ed_x;
  real_T c2_fd_x;
  real_T c2_gd_x;
  real_T c2_b_dVx[6];
  int32_T c2_i18;
  real_T c2_b_s_Fy[2];
  int32_T c2_i19;
  real_T c2_b_f_Fy[2];
  int32_T c2_i20;
  int32_T c2_i21;
  int32_T c2_i22;
  int32_T c2_i23;
  real_T (*c2_b_dX)[6];
  real_T (*c2_b_f_F)[2];
  real_T (*c2_b_s)[2];
  real_T (*c2_b_u)[3];
  real_T (*c2_b_X)[6];
  c2_b_s = (real_T (*)[2])ssGetOutputPortSignal(chartInstance->S, 3);
  c2_b_f_F = (real_T (*)[2])ssGetOutputPortSignal(chartInstance->S, 2);
  c2_b_dX = (real_T (*)[6])ssGetOutputPortSignal(chartInstance->S, 1);
  c2_b_u = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 1);
  c2_b_X = (real_T (*)[6])ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG,1);
  for (c2_i14 = 0; c2_i14 < 6; c2_i14 = c2_i14 + 1) {
    c2_hoistedGlobal[c2_i14] = (*c2_b_X)[c2_i14];
  }

  for (c2_i15 = 0; c2_i15 < 3; c2_i15 = c2_i15 + 1) {
    c2_b_hoistedGlobal[c2_i15] = (*c2_b_u)[c2_i15];
  }

  for (c2_i16 = 0; c2_i16 < 6; c2_i16 = c2_i16 + 1) {
    c2_X[c2_i16] = c2_hoistedGlobal[c2_i16];
  }

  for (c2_i17 = 0; c2_i17 < 3; c2_i17 = c2_i17 + 1) {
    c2_u[c2_i17] = c2_b_hoistedGlobal[c2_i17];
  }

  sf_debug_symbol_scope_push_eml(0U, 50U, 50U, c2_debug_family_names,
    c2_debug_family_var_map);
  sf_debug_symbol_scope_add_eml(&c2_g, c2_d_sf_marshall, 0U);
  sf_debug_symbol_scope_add_eml(&c2_m, c2_d_sf_marshall, 1U);
  sf_debug_symbol_scope_add_eml(&c2_Iz, c2_d_sf_marshall, 2U);
  sf_debug_symbol_scope_add_eml(&c2_l_F, c2_d_sf_marshall, 3U);
  sf_debug_symbol_scope_add_eml(&c2_l_R, c2_d_sf_marshall, 4U);
  sf_debug_symbol_scope_add_eml(&c2_B, c2_d_sf_marshall, 5U);
  sf_debug_symbol_scope_add_eml(&c2_C, c2_d_sf_marshall, 6U);
  sf_debug_symbol_scope_add_eml(&c2_mu, c2_d_sf_marshall, 7U);
  sf_debug_symbol_scope_add_eml(&c2_f_Fz, c2_d_sf_marshall, 8U);
  sf_debug_symbol_scope_add_eml(&c2_f_Rz, c2_d_sf_marshall, 9U);
  sf_debug_symbol_scope_add_eml(&c2_kmh2ms, c2_d_sf_marshall, 10U);
  sf_debug_symbol_scope_add_eml(&c2_ms2kmh, c2_d_sf_marshall, 11U);
  sf_debug_symbol_scope_add_eml(&c2_deg2rad, c2_d_sf_marshall, 12U);
  sf_debug_symbol_scope_add_eml(&c2_rad2deg, c2_d_sf_marshall, 13U);
  sf_debug_symbol_scope_add_eml(&c2_f_Fx_min, c2_d_sf_marshall, 14U);
  sf_debug_symbol_scope_add_eml(&c2_f_Fx_max, c2_d_sf_marshall, 15U);
  sf_debug_symbol_scope_add_eml(&c2_f_Rx_min, c2_d_sf_marshall, 16U);
  sf_debug_symbol_scope_add_eml(&c2_f_Rx_max, c2_d_sf_marshall, 17U);
  sf_debug_symbol_scope_add_eml(&c2_Vx, c2_d_sf_marshall, 18U);
  sf_debug_symbol_scope_add_eml(&c2_Vy, c2_d_sf_marshall, 19U);
  sf_debug_symbol_scope_add_eml(&c2_r, c2_d_sf_marshall, 20U);
  sf_debug_symbol_scope_add_eml(&c2_psi, c2_d_sf_marshall, 21U);
  sf_debug_symbol_scope_add_eml(&c2_xi, c2_d_sf_marshall, 22U);
  sf_debug_symbol_scope_add_eml(&c2_yi, c2_d_sf_marshall, 23U);
  sf_debug_symbol_scope_add_eml(&c2_f_Fx, c2_d_sf_marshall, 24U);
  sf_debug_symbol_scope_add_eml(&c2_f_Rx, c2_d_sf_marshall, 25U);
  sf_debug_symbol_scope_add_eml(&c2_delta, c2_d_sf_marshall, 26U);
  sf_debug_symbol_scope_add_eml(&c2_Vxw, c2_d_sf_marshall, 27U);
  sf_debug_symbol_scope_add_eml(&c2_Vyw, c2_d_sf_marshall, 28U);
  sf_debug_symbol_scope_add_eml(&c2_V, c2_d_sf_marshall, 29U);
  sf_debug_symbol_scope_add_eml(&c2_beta, c2_d_sf_marshall, 30U);
  sf_debug_symbol_scope_add_eml(&c2_f_Fy_max, c2_d_sf_marshall, 31U);
  sf_debug_symbol_scope_add_eml(&c2_s_Fy, c2_d_sf_marshall, 32U);
  sf_debug_symbol_scope_add_eml(&c2_f_Fy, c2_d_sf_marshall, 33U);
  sf_debug_symbol_scope_add_eml(&c2_f_Ry_max, c2_d_sf_marshall, 34U);
  sf_debug_symbol_scope_add_eml(&c2_s_Ry, c2_d_sf_marshall, 35U);
  sf_debug_symbol_scope_add_eml(&c2_f_Ry, c2_d_sf_marshall, 36U);
  sf_debug_symbol_scope_add_eml(&c2_dVx, c2_d_sf_marshall, 37U);
  sf_debug_symbol_scope_add_eml(&c2_dVy, c2_d_sf_marshall, 38U);
  sf_debug_symbol_scope_add_eml(&c2_dr, c2_d_sf_marshall, 39U);
  sf_debug_symbol_scope_add_eml(&c2_dpsi, c2_d_sf_marshall, 40U);
  sf_debug_symbol_scope_add_eml(&c2_dxi, c2_d_sf_marshall, 41U);
  sf_debug_symbol_scope_add_eml(&c2_dyi, c2_d_sf_marshall, 42U);
  sf_debug_symbol_scope_add_eml(&c2_nargin, c2_d_sf_marshall, 43U);
  sf_debug_symbol_scope_add_eml(&c2_nargout, c2_d_sf_marshall, 44U);
  sf_debug_symbol_scope_add_eml(&c2_X, c2_b_sf_marshall, 45U);
  sf_debug_symbol_scope_add_eml(&c2_u, c2_c_sf_marshall, 46U);
  sf_debug_symbol_scope_add_eml(&c2_dX, c2_b_sf_marshall, 47U);
  sf_debug_symbol_scope_add_eml(&c2_f_F, c2_sf_marshall, 48U);
  sf_debug_symbol_scope_add_eml(&c2_s, c2_sf_marshall, 49U);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0,4);
  c2_g = 9.82;
  _SFD_EML_CALL(0,5);
  c2_m = 1450.0;
  _SFD_EML_CALL(0,6);
  c2_Iz = 2740.0;
  _SFD_EML_CALL(0,7);
  c2_l_F = 1.1;
  _SFD_EML_CALL(0,8);
  c2_l_R = 1.6;
  _SFD_EML_CALL(0,9);
  c2_B = 7.0;
  _SFD_EML_CALL(0,10);
  c2_C = 1.4;
  _SFD_EML_CALL(0,11);
  c2_mu = 0.5;
  _SFD_EML_CALL(0,12);
  c2_f_Fz = 6.1512480000000010E+004;
  _SFD_EML_CALL(0,13);
  c2_f_Rz = 4.2289830000000009E+004;
  _SFD_EML_CALL(0,15);
  c2_kmh2ms = 2.7777777777777779E-001;
  _SFD_EML_CALL(0,16);
  c2_ms2kmh = 3.6;
  _SFD_EML_CALL(0,17);
  c2_deg2rad = 1.7453292519943295E-002;
  _SFD_EML_CALL(0,18);
  c2_rad2deg = 5.7295779513082323E+001;

  /*  Minimum and maximum values of the controls */
  _SFD_EML_CALL(0,22);
  c2_f_Fx_min = -2.8756240000000005E+004;
  _SFD_EML_CALL(0,23);
  c2_f_Fx_max = 0.0;
  _SFD_EML_CALL(0,25);
  c2_f_Rx_min = -1.9144915000000005E+004;
  _SFD_EML_CALL(0,26);
  c2_f_Rx_max = 1.9144915000000005E+004;
  _SFD_EML_CALL(0,28);
  c2_Vx = c2_X[0];
  _SFD_EML_CALL(0,29);
  c2_Vy = c2_X[1];
  _SFD_EML_CALL(0,30);
  c2_r = c2_X[2];
  _SFD_EML_CALL(0,31);
  c2_psi = c2_X[3];
  _SFD_EML_CALL(0,32);
  c2_xi = c2_X[4];
  _SFD_EML_CALL(0,33);
  c2_yi = c2_X[5];
  _SFD_EML_CALL(0,36);
  c2_f_Fx = c2_u[0];
  _SFD_EML_CALL(0,37);
  c2_f_Rx = c2_u[1];
  _SFD_EML_CALL(0,38);
  c2_delta = c2_u[2];
  _SFD_EML_CALL(0,40);
  c2_mu = 0.5;
  _SFD_EML_CALL(0,42);
  c2_x = c2_delta;
  c2_b_x = c2_x;
  c2_c_x = c2_b_x;
  c2_b_x = c2_c_x;
  c2_b_x = muDoubleScalarCos(c2_b_x);
  c2_d_x = c2_delta;
  c2_e_x = c2_d_x;
  c2_f_x = c2_e_x;
  c2_e_x = c2_f_x;
  c2_e_x = muDoubleScalarSin(c2_e_x);
  c2_Vxw = c2_b_x * c2_Vx + c2_e_x * c2_Vy;
  _SFD_EML_CALL(0,43);
  c2_g_x = c2_delta;
  c2_h_x = c2_g_x;
  c2_i_x = c2_h_x;
  c2_h_x = c2_i_x;
  c2_h_x = muDoubleScalarSin(c2_h_x);
  c2_j_x = c2_delta;
  c2_k_x = c2_j_x;
  c2_l_x = c2_k_x;
  c2_k_x = c2_l_x;
  c2_k_x = muDoubleScalarCos(c2_k_x);
  c2_Vyw = (-c2_h_x) * c2_Vx + c2_k_x * c2_Vy;
  _SFD_EML_CALL(0,45);
  if (CV_EML_IF(0, 0, c2_f_Fx >= 0.0)) {
    _SFD_EML_CALL(0,46);
    c2_f_Fx = 0.0;
  } else {
    _SFD_EML_CALL(0,47);
    if (CV_EML_IF(0, 1, c2_f_Fx <= -3.0756240000000005E+004)) {
      _SFD_EML_CALL(0,48);
      c2_f_Fx = -3.0756240000000005E+004;
    }
  }

  _SFD_EML_CALL(0,51);
  c2_a = c2_f_Fx;
  c2_b = c2_sign(chartInstance, c2_Vxw);
  c2_f_Fx = c2_a * c2_b;
  _SFD_EML_CALL(0,53);
  if (CV_EML_IF(0, 2, c2_f_Rx >= 2.1144915000000005E+004)) {
    _SFD_EML_CALL(0,54);
    c2_f_Rx = 2.1144915000000005E+004;
  } else {
    _SFD_EML_CALL(0,55);
    if (CV_EML_IF(0, 3, c2_f_Rx <= -2.1144915000000005E+004)) {
      _SFD_EML_CALL(0,56);
      c2_f_Rx = -2.1144915000000005E+004;
    }
  }

  _SFD_EML_CALL(0,59);
  if (CV_EML_IF(0, 4, c2_f_Rx < 0.0)) {
    /*  braking request */
    _SFD_EML_CALL(0,60);
    c2_b_a = c2_sign(chartInstance, c2_Vx);
    c2_b_b = c2_f_Rx;
    c2_f_Rx = c2_b_a * c2_b_b;
  }

  _SFD_EML_CALL(0,63);
  if (CV_EML_IF(0, 5, c2_delta >= 7.8539816339744828E-001)) {
    _SFD_EML_CALL(0,64);
    c2_delta = 7.8539816339744828E-001;
  } else {
    _SFD_EML_CALL(0,65);
    if (CV_EML_IF(0, 6, c2_delta <= -7.8539816339744828E-001)) {
      _SFD_EML_CALL(0,66);
      c2_delta = -7.8539816339744828E-001;
    }
  }

  /* Auxiliar variables */
  _SFD_EML_CALL(0,70);
  c2_V = c2_sqrt(chartInstance, c2_power(chartInstance, c2_Vx, 2.0) + c2_power
                 (chartInstance, c2_Vy, 2.0));
  _SFD_EML_CALL(0,71);
  c2_beta = c2_atan2(chartInstance, c2_Vy, c2_Vx);
  _SFD_EML_CALL(0,73);
  c2_m_x = c2_power(chartInstance, 3.0756240000000005E+004, 2.0) - c2_power
    (chartInstance, c2_f_Fx, 2.0);
  c2_f_Fy_max = c2_m_x;
  if (c2_f_Fy_max < 0.0) {
    c2_b_eml_error(chartInstance);
  }

  c2_n_x = c2_f_Fy_max;
  c2_f_Fy_max = c2_n_x;
  c2_f_Fy_max = muDoubleScalarSqrt(c2_f_Fy_max);
  _SFD_EML_CALL(0,74);
  c2_o_x = c2_beta - c2_delta;
  c2_p_x = c2_o_x;
  c2_q_x = c2_p_x;
  c2_p_x = c2_q_x;
  c2_p_x = muDoubleScalarSin(c2_p_x);
  c2_c_a = c2_r;
  c2_y = c2_c_a * 1.1;
  c2_r_x = c2_delta;
  c2_s_x = c2_r_x;
  c2_t_x = c2_s_x;
  c2_s_x = c2_t_x;
  c2_s_x = muDoubleScalarCos(c2_s_x);
  c2_u_x = c2_beta - c2_delta;
  c2_v_x = c2_u_x;
  c2_w_x = c2_v_x;
  c2_v_x = c2_w_x;
  c2_v_x = muDoubleScalarCos(c2_v_x);
  c2_d_a = c2_r;
  c2_b_y = c2_d_a * 1.1;
  c2_x_x = c2_delta;
  c2_y_x = c2_x_x;
  c2_ab_x = c2_y_x;
  c2_y_x = c2_ab_x;
  c2_y_x = muDoubleScalarSin(c2_y_x);
  c2_bb_x = c2_V * c2_p_x + c2_y * c2_s_x;
  c2_c_y = c2_V * c2_v_x + c2_b_y * c2_y_x;
  c2_cb_x = c2_bb_x;
  c2_d_y = c2_c_y;
  c2_db_x = c2_cb_x;
  c2_e_y = c2_d_y;
  c2_s_Fy = c2_db_x / c2_e_y;
  _SFD_EML_CALL(0,76);
  if (CV_EML_IF(0, 7, c2_V < 0.8)) {
    _SFD_EML_CALL(0,77);
    c2_s_Fy = 0.0;
  }

  _SFD_EML_CALL(0,80);
  c2_c_b = c2_s_Fy;
  c2_f_y = 7.0 * c2_c_b;
  c2_eb_x = c2_f_y;
  c2_fb_x = c2_eb_x;
  c2_gb_x = c2_fb_x;
  c2_fb_x = c2_gb_x;
  c2_fb_x = muDoubleScalarAtan(c2_fb_x);
  c2_d_b = c2_fb_x;
  c2_g_y = 1.4 * c2_d_b;
  c2_hb_x = c2_g_y;
  c2_ib_x = c2_hb_x;
  c2_jb_x = c2_ib_x;
  c2_ib_x = c2_jb_x;
  c2_ib_x = muDoubleScalarSin(c2_ib_x);
  c2_f_Fy = (-c2_f_Fy_max) * c2_ib_x;
  _SFD_EML_CALL(0,83);
  c2_kb_x = c2_power(chartInstance, 2.1144915000000005E+004, 2.0) - c2_power
    (chartInstance, c2_f_Rx, 2.0);
  c2_f_Ry_max = c2_kb_x;
  if (c2_f_Ry_max < 0.0) {
    c2_b_eml_error(chartInstance);
  }

  c2_lb_x = c2_f_Ry_max;
  c2_f_Ry_max = c2_lb_x;
  c2_f_Ry_max = muDoubleScalarSqrt(c2_f_Ry_max);
  _SFD_EML_CALL(0,84);
  c2_mb_x = c2_beta;
  c2_nb_x = c2_mb_x;
  c2_ob_x = c2_nb_x;
  c2_nb_x = c2_ob_x;
  c2_nb_x = muDoubleScalarSin(c2_nb_x);
  c2_e_a = c2_r;
  c2_h_y = c2_e_a * 1.6;
  c2_pb_x = c2_beta;
  c2_qb_x = c2_pb_x;
  c2_rb_x = c2_qb_x;
  c2_qb_x = c2_rb_x;
  c2_qb_x = muDoubleScalarCos(c2_qb_x);
  c2_sb_x = c2_V * c2_nb_x - c2_h_y;
  c2_i_y = c2_V * c2_qb_x;
  c2_tb_x = c2_sb_x;
  c2_j_y = c2_i_y;
  c2_ub_x = c2_tb_x;
  c2_k_y = c2_j_y;
  c2_s_Ry = c2_ub_x / c2_k_y;
  _SFD_EML_CALL(0,86);
  if (CV_EML_IF(0, 8, c2_V < 0.8)) {
    _SFD_EML_CALL(0,87);
    c2_s_Ry = 0.0;
  }

  _SFD_EML_CALL(0,90);
  c2_e_b = c2_s_Ry;
  c2_l_y = 7.0 * c2_e_b;
  c2_vb_x = c2_l_y;
  c2_wb_x = c2_vb_x;
  c2_xb_x = c2_wb_x;
  c2_wb_x = c2_xb_x;
  c2_wb_x = muDoubleScalarAtan(c2_wb_x);
  c2_f_b = c2_wb_x;
  c2_m_y = 1.4 * c2_f_b;
  c2_yb_x = c2_m_y;
  c2_ac_x = c2_yb_x;
  c2_bc_x = c2_ac_x;
  c2_ac_x = c2_bc_x;
  c2_ac_x = muDoubleScalarSin(c2_ac_x);
  c2_f_Ry = (-c2_f_Ry_max) * c2_ac_x;
  _SFD_EML_CALL(0,93);
  c2_cc_x = c2_delta;
  c2_dc_x = c2_cc_x;
  c2_ec_x = c2_dc_x;
  c2_dc_x = c2_ec_x;
  c2_dc_x = muDoubleScalarCos(c2_dc_x);
  c2_fc_x = c2_delta;
  c2_gc_x = c2_fc_x;
  c2_hc_x = c2_gc_x;
  c2_gc_x = c2_hc_x;
  c2_gc_x = muDoubleScalarSin(c2_gc_x);
  c2_dVx = 6.8965517241379305E-004 * ((c2_f_Fx * c2_dc_x - c2_f_Fy * c2_gc_x) +
    c2_f_Rx) + c2_Vy * c2_r;
  _SFD_EML_CALL(0,94);
  c2_ic_x = c2_delta;
  c2_jc_x = c2_ic_x;
  c2_kc_x = c2_jc_x;
  c2_jc_x = c2_kc_x;
  c2_jc_x = muDoubleScalarSin(c2_jc_x);
  c2_lc_x = c2_delta;
  c2_mc_x = c2_lc_x;
  c2_nc_x = c2_mc_x;
  c2_mc_x = c2_nc_x;
  c2_mc_x = muDoubleScalarCos(c2_mc_x);
  c2_dVy = 6.8965517241379305E-004 * ((c2_f_Fx * c2_jc_x + c2_f_Fy * c2_mc_x) +
    c2_f_Ry) - c2_Vx * c2_r;
  _SFD_EML_CALL(0,95);
  c2_oc_x = c2_delta;
  c2_pc_x = c2_oc_x;
  c2_qc_x = c2_pc_x;
  c2_pc_x = c2_qc_x;
  c2_pc_x = muDoubleScalarCos(c2_pc_x);
  c2_rc_x = c2_delta;
  c2_sc_x = c2_rc_x;
  c2_tc_x = c2_sc_x;
  c2_sc_x = c2_tc_x;
  c2_sc_x = muDoubleScalarSin(c2_sc_x);
  c2_dr = 3.6496350364963501E-004 * ((c2_f_Fy * c2_pc_x + c2_f_Fx * c2_sc_x) *
    c2_l_F - c2_f_Ry * c2_l_R);
  _SFD_EML_CALL(0,96);
  c2_dpsi = c2_r;
  _SFD_EML_CALL(0,97);
  c2_uc_x = c2_psi;
  c2_vc_x = c2_uc_x;
  c2_wc_x = c2_vc_x;
  c2_vc_x = c2_wc_x;
  c2_vc_x = muDoubleScalarCos(c2_vc_x);
  c2_xc_x = c2_psi;
  c2_yc_x = c2_xc_x;
  c2_ad_x = c2_yc_x;
  c2_yc_x = c2_ad_x;
  c2_yc_x = muDoubleScalarSin(c2_yc_x);
  c2_dxi = c2_vc_x * c2_Vx - c2_yc_x * c2_Vy;
  _SFD_EML_CALL(0,98);
  c2_bd_x = c2_psi;
  c2_cd_x = c2_bd_x;
  c2_dd_x = c2_cd_x;
  c2_cd_x = c2_dd_x;
  c2_cd_x = muDoubleScalarSin(c2_cd_x);
  c2_ed_x = c2_psi;
  c2_fd_x = c2_ed_x;
  c2_gd_x = c2_fd_x;
  c2_fd_x = c2_gd_x;
  c2_fd_x = muDoubleScalarCos(c2_fd_x);
  c2_dyi = c2_cd_x * c2_Vx + c2_fd_x * c2_Vy;
  _SFD_EML_CALL(0,100);
  c2_b_dVx[0] = c2_dVx;
  c2_b_dVx[1] = c2_dVy;
  c2_b_dVx[2] = c2_dr;
  c2_b_dVx[3] = c2_dpsi;
  c2_b_dVx[4] = c2_dxi;
  c2_b_dVx[5] = c2_dyi;
  for (c2_i18 = 0; c2_i18 < 6; c2_i18 = c2_i18 + 1) {
    c2_dX[c2_i18] = c2_b_dVx[c2_i18];
  }

  _SFD_EML_CALL(0,102);
  c2_b_s_Fy[0] = c2_s_Fy;
  c2_b_s_Fy[1] = c2_s_Ry;
  for (c2_i19 = 0; c2_i19 < 2; c2_i19 = c2_i19 + 1) {
    c2_s[c2_i19] = c2_b_s_Fy[c2_i19];
  }

  _SFD_EML_CALL(0,104);
  c2_b_f_Fy[0] = c2_f_Fy;
  c2_b_f_Fy[1] = c2_f_Ry;
  for (c2_i20 = 0; c2_i20 < 2; c2_i20 = c2_i20 + 1) {
    c2_f_F[c2_i20] = c2_b_f_Fy[c2_i20];
  }

  _SFD_EML_CALL(0,-104);
  sf_debug_symbol_scope_pop();
  for (c2_i21 = 0; c2_i21 < 6; c2_i21 = c2_i21 + 1) {
    (*c2_b_dX)[c2_i21] = c2_dX[c2_i21];
  }

  for (c2_i22 = 0; c2_i22 < 2; c2_i22 = c2_i22 + 1) {
    (*c2_b_f_F)[c2_i22] = c2_f_F[c2_i22];
  }

  for (c2_i23 = 0; c2_i23 < 2; c2_i23 = c2_i23 + 1) {
    (*c2_b_s)[c2_i23] = c2_s[c2_i23];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG,1);
}

static void init_script_number_translation(uint32_T c2_machineNumber, uint32_T
  c2_chartNumber)
{
}

static real_T c2_sign(SFc2_car_model_frictionInstanceStruct *chartInstance,
                      real_T c2_x)
{
  real_T c2_b_x;
  real_T c2_c_x;
  real_T c2_d_x;
  boolean_T c2_b;
  c2_b_x = c2_x;
  c2_c_x = c2_b_x;
  c2_b_x = c2_c_x;
  c2_d_x = c2_b_x;
  c2_b = muDoubleScalarIsNaN(c2_d_x);
  if (c2_b) {
    return rtNaN;
  } else if (c2_b_x > 0.0) {
    return 1.0;
  } else if (c2_b_x < 0.0) {
    return -1.0;
  } else {
    return 0.0;
  }
}

static real_T c2_power(SFc2_car_model_frictionInstanceStruct *chartInstance,
  real_T c2_a, real_T c2_b)
{
  real_T c2_ak;
  real_T c2_bk;
  real_T c2_x;
  real_T c2_b_x;
  c2_eml_scalar_eg(chartInstance);
  c2_ak = c2_a;
  c2_bk = c2_b;
  if (c2_ak < 0.0) {
    c2_x = c2_bk;
    c2_b_x = c2_x;
    c2_b_x = muDoubleScalarFloor(c2_b_x);
    if (c2_b_x != c2_bk) {
      c2_eml_error(chartInstance);
      goto label_1;
    }
  }

 label_1:
  ;
  return muDoubleScalarPower(c2_ak, c2_bk);
}

static void c2_eml_scalar_eg(SFc2_car_model_frictionInstanceStruct
  *chartInstance)
{
}

static void c2_eml_error(SFc2_car_model_frictionInstanceStruct *chartInstance)
{
  int32_T c2_i24;
  static char_T c2_cv0[32] = { 'E', 'm', 'b', 'e', 'd', 'd', 'e', 'd', 'M', 'A',
    'T', 'L', 'A', 'B', ':', 'p', 'o', 'w', 'e', 'r', ':'
    , 'd', 'o', 'm', 'a', 'i', 'n', 'E', 'r', 'r', 'o', 'r' };

  char_T c2_u[32];
  const mxArray *c2_y = NULL;
  int32_T c2_i25;
  static char_T c2_cv1[102] = { 'D', 'o', 'm', 'a', 'i', 'n', ' ', 'e', 'r', 'r',
    'o', 'r', '.', ' ', 'T', 'o', ' ', 'c', 'o', 'm',
    'p', 'u', 't', 'e', ' ', 'c', 'o', 'm', 'p', 'l', 'e', 'x', ' ', 'r', 'e',
    's', 'u', 'l', 't', 's',
    ',', ' ', 'm', 'a', 'k', 'e', ' ', 'a', 't', ' ', 'l', 'e', 'a', 's', 't',
    ' ', 'o', 'n', 'e', ' ',
    'i', 'n', 'p', 'u', 't', ' ', 'c', 'o', 'm', 'p', 'l', 'e', 'x', ',', ' ',
    'e', '.', 'g', '.', ' ',
    '\'', 'p', 'o', 'w', 'e', 'r', '(', 'c', 'o', 'm', 'p', 'l', 'e', 'x', '(',
    'a', ')', ',', 'b', ')',
    '\'', '.' };

  char_T c2_b_u[102];
  const mxArray *c2_b_y = NULL;
  for (c2_i24 = 0; c2_i24 < 32; c2_i24 = c2_i24 + 1) {
    c2_u[c2_i24] = c2_cv0[c2_i24];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 10, 0U, 1U, 0U, 2, 1, 32));
  for (c2_i25 = 0; c2_i25 < 102; c2_i25 = c2_i25 + 1) {
    c2_b_u[c2_i25] = c2_cv1[c2_i25];
  }

  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", &c2_b_u, 10, 0U, 1U, 0U, 2, 1, 102));
  sf_mex_call_debug("error", 0U, 2U, 14, c2_y, 14, c2_b_y);
}

static real_T c2_sqrt(SFc2_car_model_frictionInstanceStruct *chartInstance,
                      real_T c2_x)
{
  real_T c2_b_x;
  real_T c2_c_x;
  c2_b_x = c2_x;
  if (c2_b_x < 0.0) {
    c2_b_eml_error(chartInstance);
  }

  c2_c_x = c2_b_x;
  c2_b_x = c2_c_x;
  return muDoubleScalarSqrt(c2_b_x);
}

static void c2_b_eml_error(SFc2_car_model_frictionInstanceStruct *chartInstance)
{
  int32_T c2_i26;
  static char_T c2_cv2[31] = { 'E', 'm', 'b', 'e', 'd', 'd', 'e', 'd', 'M', 'A',
    'T', 'L', 'A', 'B', ':', 's', 'q', 'r', 't', ':', 'd'
    , 'o', 'm', 'a', 'i', 'n', 'E', 'r', 'r', 'o', 'r' };

  char_T c2_u[31];
  const mxArray *c2_y = NULL;
  int32_T c2_i27;
  static char_T c2_cv3[77] = { 'D', 'o', 'm', 'a', 'i', 'n', ' ', 'e', 'r', 'r',
    'o', 'r', '.', ' ', 'T', 'o', ' ', 'c', 'o', 'm', 'p'
    , 'u', 't', 'e', ' ', 'c', 'o', 'm', 'p', 'l', 'e', 'x', ' ', 'r', 'e', 's',
    'u', 'l', 't', 's', ' ',
    'f', 'r', 'o', 'm', ' ', 'r', 'e', 'a', 'l', ' ', 'x', ',', ' ', 'u', 's',
    'e', ' ', '\'', 's', 'q',
    'r', 't', '(', 'c', 'o', 'm', 'p', 'l', 'e', 'x', '(', 'x', ')', ')', '\'',
    '.' };

  char_T c2_b_u[77];
  const mxArray *c2_b_y = NULL;
  for (c2_i26 = 0; c2_i26 < 31; c2_i26 = c2_i26 + 1) {
    c2_u[c2_i26] = c2_cv2[c2_i26];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 10, 0U, 1U, 0U, 2, 1, 31));
  for (c2_i27 = 0; c2_i27 < 77; c2_i27 = c2_i27 + 1) {
    c2_b_u[c2_i27] = c2_cv3[c2_i27];
  }

  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", &c2_b_u, 10, 0U, 1U, 0U, 2, 1, 77));
  sf_mex_call_debug("error", 0U, 2U, 14, c2_y, 14, c2_b_y);
}

static real_T c2_atan2(SFc2_car_model_frictionInstanceStruct *chartInstance,
  real_T c2_y, real_T c2_x)
{
  real_T c2_b_y;
  real_T c2_b_x;
  c2_eml_scalar_eg(chartInstance);
  c2_b_y = c2_y;
  c2_b_x = c2_x;
  return muDoubleScalarAtan2(c2_b_y, c2_b_x);
}

static const mxArray *c2_sf_marshall(void *chartInstanceVoid, void *c2_u)
{
  const mxArray *c2_y = NULL;
  int32_T c2_i28;
  real_T c2_b_u[2];
  const mxArray *c2_b_y = NULL;
  SFc2_car_model_frictionInstanceStruct *chartInstance;
  chartInstance = (SFc2_car_model_frictionInstanceStruct *)chartInstanceVoid;
  c2_y = NULL;
  for (c2_i28 = 0; c2_i28 < 2; c2_i28 = c2_i28 + 1) {
    c2_b_u[c2_i28] = (*((real_T (*)[2])c2_u))[c2_i28];
  }

  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", &c2_b_u, 0, 0U, 1U, 0U, 1, 2));
  sf_mex_assign(&c2_y, c2_b_y);
  return c2_y;
}

static const mxArray *c2_b_sf_marshall(void *chartInstanceVoid, void *c2_u)
{
  const mxArray *c2_y = NULL;
  int32_T c2_i29;
  real_T c2_b_u[6];
  const mxArray *c2_b_y = NULL;
  SFc2_car_model_frictionInstanceStruct *chartInstance;
  chartInstance = (SFc2_car_model_frictionInstanceStruct *)chartInstanceVoid;
  c2_y = NULL;
  for (c2_i29 = 0; c2_i29 < 6; c2_i29 = c2_i29 + 1) {
    c2_b_u[c2_i29] = (*((real_T (*)[6])c2_u))[c2_i29];
  }

  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", &c2_b_u, 0, 0U, 1U, 0U, 1, 6));
  sf_mex_assign(&c2_y, c2_b_y);
  return c2_y;
}

static const mxArray *c2_c_sf_marshall(void *chartInstanceVoid, void *c2_u)
{
  const mxArray *c2_y = NULL;
  int32_T c2_i30;
  real_T c2_b_u[3];
  const mxArray *c2_b_y = NULL;
  SFc2_car_model_frictionInstanceStruct *chartInstance;
  chartInstance = (SFc2_car_model_frictionInstanceStruct *)chartInstanceVoid;
  c2_y = NULL;
  for (c2_i30 = 0; c2_i30 < 3; c2_i30 = c2_i30 + 1) {
    c2_b_u[c2_i30] = (*((real_T (*)[3])c2_u))[c2_i30];
  }

  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", &c2_b_u, 0, 0U, 1U, 0U, 1, 3));
  sf_mex_assign(&c2_y, c2_b_y);
  return c2_y;
}

static const mxArray *c2_d_sf_marshall(void *chartInstanceVoid, void *c2_u)
{
  const mxArray *c2_y = NULL;
  real_T c2_b_u;
  const mxArray *c2_b_y = NULL;
  SFc2_car_model_frictionInstanceStruct *chartInstance;
  chartInstance = (SFc2_car_model_frictionInstanceStruct *)chartInstanceVoid;
  c2_y = NULL;
  c2_b_u = *((real_T *)c2_u);
  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", &c2_b_u, 0, 0U, 0U, 0U, 0));
  sf_mex_assign(&c2_y, c2_b_y);
  return c2_y;
}

const mxArray *sf_c2_car_model_friction_get_eml_resolved_functions_info(void)
{
  const mxArray *c2_nameCaptureInfo = NULL;
  c2_ResolvedFunctionInfo c2_info[52];
  const mxArray *c2_m0 = NULL;
  int32_T c2_i31;
  c2_ResolvedFunctionInfo *c2_r0;
  c2_nameCaptureInfo = NULL;
  c2_info_helper(c2_info);
  sf_mex_assign(&c2_m0, sf_mex_createstruct("nameCaptureInfo", 1, 52));
  for (c2_i31 = 0; c2_i31 < 52; c2_i31 = c2_i31 + 1) {
    c2_r0 = &c2_info[c2_i31];
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", c2_r0->context, 15,
      0U, 0U, 0U, 2, 1, strlen(c2_r0->context)), "context",
                    "nameCaptureInfo", c2_i31);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", c2_r0->name, 15, 0U,
      0U, 0U, 2, 1, strlen(c2_r0->name)), "name",
                    "nameCaptureInfo", c2_i31);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", c2_r0->dominantType,
      15, 0U, 0U, 0U, 2, 1, strlen(c2_r0->dominantType)),
                    "dominantType", "nameCaptureInfo", c2_i31);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", c2_r0->resolved, 15,
      0U, 0U, 0U, 2, 1, strlen(c2_r0->resolved)), "resolved"
                    , "nameCaptureInfo", c2_i31);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", &c2_r0->fileLength,
      7, 0U, 0U, 0U, 0), "fileLength", "nameCaptureInfo",
                    c2_i31);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", &c2_r0->fileTime1, 7,
      0U, 0U, 0U, 0), "fileTime1", "nameCaptureInfo", c2_i31
                    );
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", &c2_r0->fileTime2, 7,
      0U, 0U, 0U, 0), "fileTime2", "nameCaptureInfo", c2_i31
                    );
  }

  sf_mex_assign(&c2_nameCaptureInfo, c2_m0);
  return c2_nameCaptureInfo;
}

static void c2_info_helper(c2_ResolvedFunctionInfo c2_info[52])
{
  c2_info[0].context = "";
  c2_info[0].name = "mtimes";
  c2_info[0].dominantType = "double";
  c2_info[0].resolved = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[0].fileLength = 3425U;
  c2_info[0].fileTime1 = 1250694366U;
  c2_info[0].fileTime2 = 0U;
  c2_info[1].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[1].name = "nargin";
  c2_info[1].dominantType = "";
  c2_info[1].resolved = "[B]nargin";
  c2_info[1].fileLength = 0U;
  c2_info[1].fileTime1 = 0U;
  c2_info[1].fileTime2 = 0U;
  c2_info[2].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[2].name = "gt";
  c2_info[2].dominantType = "double";
  c2_info[2].resolved = "[B]gt";
  c2_info[2].fileLength = 0U;
  c2_info[2].fileTime1 = 0U;
  c2_info[2].fileTime2 = 0U;
  c2_info[3].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[3].name = "isa";
  c2_info[3].dominantType = "double";
  c2_info[3].resolved = "[B]isa";
  c2_info[3].fileLength = 0U;
  c2_info[3].fileTime1 = 0U;
  c2_info[3].fileTime2 = 0U;
  c2_info[4].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[4].name = "isinteger";
  c2_info[4].dominantType = "double";
  c2_info[4].resolved = "[B]isinteger";
  c2_info[4].fileLength = 0U;
  c2_info[4].fileTime1 = 0U;
  c2_info[4].fileTime2 = 0U;
  c2_info[5].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[5].name = "isscalar";
  c2_info[5].dominantType = "double";
  c2_info[5].resolved = "[B]isscalar";
  c2_info[5].fileLength = 0U;
  c2_info[5].fileTime1 = 0U;
  c2_info[5].fileTime2 = 0U;
  c2_info[6].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[6].name = "strcmp";
  c2_info[6].dominantType = "char";
  c2_info[6].resolved = "[B]strcmp";
  c2_info[6].fileLength = 0U;
  c2_info[6].fileTime1 = 0U;
  c2_info[6].fileTime2 = 0U;
  c2_info[7].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[7].name = "size";
  c2_info[7].dominantType = "double";
  c2_info[7].resolved = "[B]size";
  c2_info[7].fileLength = 0U;
  c2_info[7].fileTime1 = 0U;
  c2_info[7].fileTime2 = 0U;
  c2_info[8].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[8].name = "eq";
  c2_info[8].dominantType = "double";
  c2_info[8].resolved = "[B]eq";
  c2_info[8].fileLength = 0U;
  c2_info[8].fileTime1 = 0U;
  c2_info[8].fileTime2 = 0U;
  c2_info[9].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[9].name = "class";
  c2_info[9].dominantType = "double";
  c2_info[9].resolved = "[B]class";
  c2_info[9].fileLength = 0U;
  c2_info[9].fileTime1 = 0U;
  c2_info[9].fileTime2 = 0U;
  c2_info[10].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[10].name = "not";
  c2_info[10].dominantType = "logical";
  c2_info[10].resolved = "[B]not";
  c2_info[10].fileLength = 0U;
  c2_info[10].fileTime1 = 0U;
  c2_info[10].fileTime2 = 0U;
  c2_info[11].context = "";
  c2_info[11].name = "plus";
  c2_info[11].dominantType = "double";
  c2_info[11].resolved = "[B]plus";
  c2_info[11].fileLength = 0U;
  c2_info[11].fileTime1 = 0U;
  c2_info[11].fileTime2 = 0U;
  c2_info[12].context = "";
  c2_info[12].name = "mrdivide";
  c2_info[12].dominantType = "double";
  c2_info[12].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.m";
  c2_info[12].fileLength = 800U;
  c2_info[12].fileTime1 = 1238455890U;
  c2_info[12].fileTime2 = 0U;
  c2_info[13].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.m";
  c2_info[13].name = "ge";
  c2_info[13].dominantType = "double";
  c2_info[13].resolved = "[B]ge";
  c2_info[13].fileLength = 0U;
  c2_info[13].fileTime1 = 0U;
  c2_info[13].fileTime2 = 0U;
  c2_info[14].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.m";
  c2_info[14].name = "rdivide";
  c2_info[14].dominantType = "double";
  c2_info[14].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c2_info[14].fileLength = 403U;
  c2_info[14].fileTime1 = 1244757152U;
  c2_info[14].fileTime2 = 0U;
  c2_info[15].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c2_info[15].name = "eml_div";
  c2_info[15].dominantType = "double";
  c2_info[15].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c2_info[15].fileLength = 4269U;
  c2_info[15].fileTime1 = 1228115426U;
  c2_info[15].fileTime2 = 0U;
  c2_info[16].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m/eml_fldiv";
  c2_info[16].name = "isreal";
  c2_info[16].dominantType = "double";
  c2_info[16].resolved = "[B]isreal";
  c2_info[16].fileLength = 0U;
  c2_info[16].fileTime1 = 0U;
  c2_info[16].fileTime2 = 0U;
  c2_info[17].context = "";
  c2_info[17].name = "pi";
  c2_info[17].dominantType = "";
  c2_info[17].resolved = "[B]pi";
  c2_info[17].fileLength = 0U;
  c2_info[17].fileTime1 = 0U;
  c2_info[17].fileTime2 = 0U;
  c2_info[18].context = "";
  c2_info[18].name = "uminus";
  c2_info[18].dominantType = "double";
  c2_info[18].resolved = "[B]uminus";
  c2_info[18].fileLength = 0U;
  c2_info[18].fileTime1 = 0U;
  c2_info[18].fileTime2 = 0U;
  c2_info[19].context = "";
  c2_info[19].name = "minus";
  c2_info[19].dominantType = "double";
  c2_info[19].resolved = "[B]minus";
  c2_info[19].fileLength = 0U;
  c2_info[19].fileTime1 = 0U;
  c2_info[19].fileTime2 = 0U;
  c2_info[20].context = "";
  c2_info[20].name = "cos";
  c2_info[20].dominantType = "double";
  c2_info[20].resolved = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/cos.m";
  c2_info[20].fileLength = 324U;
  c2_info[20].fileTime1 = 1203469550U;
  c2_info[20].fileTime2 = 0U;
  c2_info[21].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/cos.m";
  c2_info[21].name = "eml_scalar_cos";
  c2_info[21].dominantType = "double";
  c2_info[21].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_cos.m";
  c2_info[21].fileLength = 602U;
  c2_info[21].fileTime1 = 1209352386U;
  c2_info[21].fileTime2 = 0U;
  c2_info[22].context = "";
  c2_info[22].name = "times";
  c2_info[22].dominantType = "double";
  c2_info[22].resolved = "[B]times";
  c2_info[22].fileLength = 0U;
  c2_info[22].fileTime1 = 0U;
  c2_info[22].fileTime2 = 0U;
  c2_info[23].context = "";
  c2_info[23].name = "sin";
  c2_info[23].dominantType = "double";
  c2_info[23].resolved = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/sin.m";
  c2_info[23].fileLength = 324U;
  c2_info[23].fileTime1 = 1203469642U;
  c2_info[23].fileTime2 = 0U;
  c2_info[24].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/sin.m";
  c2_info[24].name = "eml_scalar_sin";
  c2_info[24].dominantType = "double";
  c2_info[24].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_sin.m";
  c2_info[24].fileLength = 601U;
  c2_info[24].fileTime1 = 1209352390U;
  c2_info[24].fileTime2 = 0U;
  c2_info[25].context = "";
  c2_info[25].name = "le";
  c2_info[25].dominantType = "double";
  c2_info[25].resolved = "[B]le";
  c2_info[25].fileLength = 0U;
  c2_info[25].fileTime1 = 0U;
  c2_info[25].fileTime2 = 0U;
  c2_info[26].context = "";
  c2_info[26].name = "sign";
  c2_info[26].dominantType = "double";
  c2_info[26].resolved = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/sign.m";
  c2_info[26].fileLength = 408U;
  c2_info[26].fileTime1 = 1203469640U;
  c2_info[26].fileTime2 = 0U;
  c2_info[27].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/sign.m";
  c2_info[27].name = "eml_scalar_sign";
  c2_info[27].dominantType = "double";
  c2_info[27].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_sign.m";
  c2_info[27].fileLength = 543U;
  c2_info[27].fileTime1 = 1203469612U;
  c2_info[27].fileTime2 = 0U;
  c2_info[28].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_sign.m";
  c2_info[28].name = "isnan";
  c2_info[28].dominantType = "double";
  c2_info[28].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m";
  c2_info[28].fileLength = 506U;
  c2_info[28].fileTime1 = 1228115410U;
  c2_info[28].fileTime2 = 0U;
  c2_info[29].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_sign.m";
  c2_info[29].name = "eml_guarded_nan";
  c2_info[29].dominantType = "char";
  c2_info[29].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_guarded_nan.m";
  c2_info[29].fileLength = 485U;
  c2_info[29].fileTime1 = 1192488380U;
  c2_info[29].fileTime2 = 0U;
  c2_info[30].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_guarded_nan.m";
  c2_info[30].name = "eml_is_float_class";
  c2_info[30].dominantType = "char";
  c2_info[30].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_is_float_class.m";
  c2_info[30].fileLength = 226U;
  c2_info[30].fileTime1 = 1197872040U;
  c2_info[30].fileTime2 = 0U;
  c2_info[31].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_is_float_class.m";
  c2_info[31].name = "ischar";
  c2_info[31].dominantType = "char";
  c2_info[31].resolved = "[B]ischar";
  c2_info[31].fileLength = 0U;
  c2_info[31].fileTime1 = 0U;
  c2_info[31].fileTime2 = 0U;
  c2_info[32].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_guarded_nan.m";
  c2_info[32].name = "nan";
  c2_info[32].dominantType = "char";
  c2_info[32].resolved = "[B]nan";
  c2_info[32].fileLength = 0U;
  c2_info[32].fileTime1 = 0U;
  c2_info[32].fileTime2 = 0U;
  c2_info[33].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_sign.m";
  c2_info[33].name = "ones";
  c2_info[33].dominantType = "char";
  c2_info[33].resolved = "[B]ones";
  c2_info[33].fileLength = 0U;
  c2_info[33].fileTime1 = 0U;
  c2_info[33].fileTime2 = 0U;
  c2_info[34].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_sign.m";
  c2_info[34].name = "lt";
  c2_info[34].dominantType = "double";
  c2_info[34].resolved = "[B]lt";
  c2_info[34].fileLength = 0U;
  c2_info[34].fileTime1 = 0U;
  c2_info[34].fileTime2 = 0U;
  c2_info[35].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_sign.m";
  c2_info[35].name = "zeros";
  c2_info[35].dominantType = "char";
  c2_info[35].resolved = "[B]zeros";
  c2_info[35].fileLength = 0U;
  c2_info[35].fileTime1 = 0U;
  c2_info[35].fileTime2 = 0U;
  c2_info[36].context = "";
  c2_info[36].name = "power";
  c2_info[36].dominantType = "double";
  c2_info[36].resolved = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m";
  c2_info[36].fileLength = 5380U;
  c2_info[36].fileTime1 = 1228115498U;
  c2_info[36].fileTime2 = 0U;
  c2_info[37].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m";
  c2_info[37].name = "eml_scalar_eg";
  c2_info[37].dominantType = "double";
  c2_info[37].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[37].fileLength = 3068U;
  c2_info[37].fileTime1 = 1240283610U;
  c2_info[37].fileTime2 = 0U;
  c2_info[38].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m/any_enums";
  c2_info[38].name = "false";
  c2_info[38].dominantType = "";
  c2_info[38].resolved = "[B]false";
  c2_info[38].fileLength = 0U;
  c2_info[38].fileTime1 = 0U;
  c2_info[38].fileTime2 = 0U;
  c2_info[39].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[39].name = "isstruct";
  c2_info[39].dominantType = "double";
  c2_info[39].resolved = "[B]isstruct";
  c2_info[39].fileLength = 0U;
  c2_info[39].fileTime1 = 0U;
  c2_info[39].fileTime2 = 0U;
  c2_info[40].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m/zerosum";
  c2_info[40].name = "cast";
  c2_info[40].dominantType = "double";
  c2_info[40].resolved = "[B]cast";
  c2_info[40].fileLength = 0U;
  c2_info[40].fileTime1 = 0U;
  c2_info[40].fileTime2 = 0U;
  c2_info[41].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m";
  c2_info[41].name = "eml_scalexp_alloc";
  c2_info[41].dominantType = "double";
  c2_info[41].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c2_info[41].fileLength = 808U;
  c2_info[41].fileTime1 = 1230516298U;
  c2_info[41].fileTime2 = 0U;
  c2_info[42].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m";
  c2_info[42].name = "eml_scalar_floor";
  c2_info[42].dominantType = "double";
  c2_info[42].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m";
  c2_info[42].fileLength = 260U;
  c2_info[42].fileTime1 = 1209352390U;
  c2_info[42].fileTime2 = 0U;
  c2_info[43].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m";
  c2_info[43].name = "ne";
  c2_info[43].dominantType = "double";
  c2_info[43].resolved = "[B]ne";
  c2_info[43].fileLength = 0U;
  c2_info[43].fileTime1 = 0U;
  c2_info[43].fileTime2 = 0U;
  c2_info[44].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m";
  c2_info[44].name = "eml_error";
  c2_info[44].dominantType = "char";
  c2_info[44].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m";
  c2_info[44].fileLength = 315U;
  c2_info[44].fileTime1 = 1213948344U;
  c2_info[44].fileTime2 = 0U;
  c2_info[45].context = "";
  c2_info[45].name = "sqrt";
  c2_info[45].dominantType = "double";
  c2_info[45].resolved = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m";
  c2_info[45].fileLength = 572U;
  c2_info[45].fileTime1 = 1203469644U;
  c2_info[45].fileTime2 = 0U;
  c2_info[46].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m";
  c2_info[46].name = "eml_scalar_sqrt";
  c2_info[46].dominantType = "double";
  c2_info[46].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_sqrt.m";
  c2_info[46].fileLength = 664U;
  c2_info[46].fileTime1 = 1209352394U;
  c2_info[46].fileTime2 = 0U;
  c2_info[47].context = "";
  c2_info[47].name = "atan2";
  c2_info[47].dominantType = "double";
  c2_info[47].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/atan2.m";
  c2_info[47].fileLength = 605U;
  c2_info[47].fileTime1 = 1236278854U;
  c2_info[47].fileTime2 = 0U;
  c2_info[48].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/atan2.m";
  c2_info[48].name = "eml_scalar_atan2";
  c2_info[48].dominantType = "double";
  c2_info[48].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_atan2.m";
  c2_info[48].fileLength = 964U;
  c2_info[48].fileTime1 = 1209352386U;
  c2_info[48].fileTime2 = 0U;
  c2_info[49].context = "";
  c2_info[49].name = "atan";
  c2_info[49].dominantType = "double";
  c2_info[49].resolved = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/atan.m";
  c2_info[49].fileLength = 536U;
  c2_info[49].fileTime1 = 1203469546U;
  c2_info[49].fileTime2 = 0U;
  c2_info[50].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/atan.m";
  c2_info[50].name = "eml_scalar_atan";
  c2_info[50].dominantType = "double";
  c2_info[50].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_atan.m";
  c2_info[50].fileLength = 281U;
  c2_info[50].fileTime1 = 1203469576U;
  c2_info[50].fileTime2 = 0U;
  c2_info[51].context = "";
  c2_info[51].name = "ctranspose";
  c2_info[51].dominantType = "double";
  c2_info[51].resolved = "[B]ctranspose";
  c2_info[51].fileLength = 0U;
  c2_info[51].fileTime1 = 0U;
  c2_info[51].fileTime2 = 0U;
}

static const mxArray *c2_e_sf_marshall(void *chartInstanceVoid, void *c2_u)
{
  const mxArray *c2_y = NULL;
  boolean_T c2_b_u;
  const mxArray *c2_b_y = NULL;
  SFc2_car_model_frictionInstanceStruct *chartInstance;
  chartInstance = (SFc2_car_model_frictionInstanceStruct *)chartInstanceVoid;
  c2_y = NULL;
  c2_b_u = *((boolean_T *)c2_u);
  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", &c2_b_u, 11, 0U, 0U, 0U, 0));
  sf_mex_assign(&c2_y, c2_b_y);
  return c2_y;
}

static void c2_emlrt_marshallIn(SFc2_car_model_frictionInstanceStruct
  *chartInstance, const mxArray *c2_dX, const char_T *c2_name,
  real_T c2_y[6])
{
  real_T c2_dv3[6];
  int32_T c2_i32;
  sf_mex_import(c2_name, sf_mex_dup(c2_dX), &c2_dv3, 1, 0, 0U, 1, 0U, 1, 6);
  for (c2_i32 = 0; c2_i32 < 6; c2_i32 = c2_i32 + 1) {
    c2_y[c2_i32] = c2_dv3[c2_i32];
  }

  sf_mex_destroy(&c2_dX);
}

static void c2_b_emlrt_marshallIn(SFc2_car_model_frictionInstanceStruct
  *chartInstance, const mxArray *c2_f_F, const char_T *c2_name
  , real_T c2_y[2])
{
  real_T c2_dv4[2];
  int32_T c2_i33;
  sf_mex_import(c2_name, sf_mex_dup(c2_f_F), &c2_dv4, 1, 0, 0U, 1, 0U, 1, 2);
  for (c2_i33 = 0; c2_i33 < 2; c2_i33 = c2_i33 + 1) {
    c2_y[c2_i33] = c2_dv4[c2_i33];
  }

  sf_mex_destroy(&c2_f_F);
}

static uint8_T c2_c_emlrt_marshallIn(SFc2_car_model_frictionInstanceStruct
  *chartInstance, const mxArray *
  c2_b_is_active_c2_car_model_friction, const char_T *c2_name)
{
  uint8_T c2_y;
  uint8_T c2_u0;
  sf_mex_import(c2_name, sf_mex_dup(c2_b_is_active_c2_car_model_friction),
                &c2_u0, 1, 3, 0U, 0, 0U, 0);
  c2_y = c2_u0;
  sf_mex_destroy(&c2_b_is_active_c2_car_model_friction);
  return c2_y;
}

static void init_dsm_address_info(SFc2_car_model_frictionInstanceStruct
  *chartInstance)
{
}

/* SFunction Glue Code */
void sf_c2_car_model_friction_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(940013752U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(2887641681U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3016343194U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(2319942422U);
}

mxArray *sf_c2_car_model_friction_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,4,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateDoubleMatrix(4,1,mxREAL);
    double *pr = mxGetPr(mxChecksum);
    pr[0] = (double)(51512434U);
    pr[1] = (double)(1958250689U);
    pr[2] = (double)(2846098021U);
    pr[3] = (double)(4254971078U);
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,2,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(6);
      pr[1] = (double)(1);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(3);
      pr[1] = (double)(1);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxCreateDoubleMatrix(0,0,
                mxREAL));
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,3,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(6);
      pr[1] = (double)(1);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(2);
      pr[1] = (double)(1);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(2);
      pr[1] = (double)(1);
      mxSetField(mxData,2,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,2,"type",mxType);
    }

    mxSetField(mxData,2,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  return(mxAutoinheritanceInfo);
}

static mxArray *sf_get_sim_state_info_c2_car_model_friction(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x4'type','srcId','name','auxInfo'{{M[1],M[5],T\"dX\",},{M[1],M[7],T\"f_F\",},{M[1],M[14],T\"s\",},{M[8],M[0],T\"is_active_c2_car_model_friction\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 4, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c2_car_model_friction_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc2_car_model_frictionInstanceStruct *chartInstance;
    chartInstance = (SFc2_car_model_frictionInstanceStruct *) ((ChartInfoStruct *)
      (ssGetUserData(S)))->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (_car_model_frictionMachineNumber_,
           2,
           1,
           1,
           5,
           0,
           0,
           0,
           0,
           0,
           &(chartInstance->chartNumber),
           &(chartInstance->instanceNumber),
           ssGetPath(S),
           (void *)S);
        if (chartAlreadyPresent==0) {
          /* this is the first instance */
          init_script_number_translation(_car_model_frictionMachineNumber_,
            chartInstance->chartNumber);
          sf_debug_set_chart_disable_implicit_casting
            (_car_model_frictionMachineNumber_,chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds(_car_model_frictionMachineNumber_,
            chartInstance->chartNumber,
            0,
            0,
            0);

          {
            unsigned int dimVector[1];
            dimVector[0]= 6;
            _SFD_SET_DATA_PROPS(0,1,1,0,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
                                1.0,0,"X",0,(MexFcnForType)c2_b_sf_marshall);
          }

          {
            unsigned int dimVector[1];
            dimVector[0]= 3;
            _SFD_SET_DATA_PROPS(1,1,1,0,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
                                1.0,0,"u",0,(MexFcnForType)c2_c_sf_marshall);
          }

          {
            unsigned int dimVector[1];
            dimVector[0]= 6;
            _SFD_SET_DATA_PROPS(2,2,0,1,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
                                1.0,0,"dX",0,(MexFcnForType)c2_b_sf_marshall);
          }

          {
            unsigned int dimVector[1];
            dimVector[0]= 2;
            _SFD_SET_DATA_PROPS(3,2,0,1,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
                                1.0,0,"f_F",0,(MexFcnForType)c2_sf_marshall);
          }

          {
            unsigned int dimVector[1];
            dimVector[0]= 2;
            _SFD_SET_DATA_PROPS(4,2,0,1,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
                                1.0,0,"s",0,(MexFcnForType)c2_sf_marshall);
          }

          _SFD_STATE_INFO(0,0,2);
          _SFD_CH_SUBSTATE_COUNT(0);
          _SFD_CH_SUBSTATE_DECOMP(0);
        }

        _SFD_CV_INIT_CHART(0,0,0,0);

        {
          _SFD_CV_INIT_STATE(0,0,0,0,0,0,NULL,NULL);
        }

        _SFD_CV_INIT_TRANS(0,0,NULL,NULL,0,NULL);

        /* Initialization of EML Model Coverage */
        _SFD_CV_INIT_EML(0,1,9,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,2076);
        _SFD_CV_INIT_EML_IF(0,0,737,752,775,800);
        _SFD_CV_INIT_EML_IF(0,1,775,800,-1,800);
        _SFD_CV_INIT_EML_IF(0,2,867,888,918,943);
        _SFD_CV_INIT_EML_IF(0,3,918,943,-1,943);
        _SFD_CV_INIT_EML_IF(0,4,982,995,-1,1053);
        _SFD_CV_INIT_EML_IF(0,5,1059,1083,1115,1143);
        _SFD_CV_INIT_EML_IF(0,6,1115,1143,-1,1143);
        _SFD_CV_INIT_EML_IF(0,7,1401,1413,-1,1439);
        _SFD_CV_INIT_EML_IF(0,8,1586,1598,-1,1624);
        _SFD_TRANS_COV_WTS(0,0,0,1,0);
        if (chartAlreadyPresent==0) {
          _SFD_TRANS_COV_MAPS(0,
                              0,NULL,NULL,
                              0,NULL,NULL,
                              1,NULL,NULL,
                              0,NULL,NULL);
        }

        {
          real_T (*c2_X)[6];
          real_T (*c2_u)[3];
          real_T (*c2_dX)[6];
          real_T (*c2_f_F)[2];
          real_T (*c2_s)[2];
          c2_s = (real_T (*)[2])ssGetOutputPortSignal(chartInstance->S, 3);
          c2_f_F = (real_T (*)[2])ssGetOutputPortSignal(chartInstance->S, 2);
          c2_dX = (real_T (*)[6])ssGetOutputPortSignal(chartInstance->S, 1);
          c2_u = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 1);
          c2_X = (real_T (*)[6])ssGetInputPortSignal(chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, c2_X);
          _SFD_SET_DATA_VALUE_PTR(1U, c2_u);
          _SFD_SET_DATA_VALUE_PTR(2U, c2_dX);
          _SFD_SET_DATA_VALUE_PTR(3U, c2_f_F);
          _SFD_SET_DATA_VALUE_PTR(4U, c2_s);
        }
      }
    } else {
      sf_debug_reset_current_state_configuration
        (_car_model_frictionMachineNumber_,chartInstance->chartNumber,
         chartInstance->instanceNumber);
    }
  }
}

static void sf_opaque_initialize_c2_car_model_friction(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc2_car_model_frictionInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c2_car_model_friction((SFc2_car_model_frictionInstanceStruct*)
    chartInstanceVar);
  initialize_c2_car_model_friction((SFc2_car_model_frictionInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c2_car_model_friction(void *chartInstanceVar)
{
  enable_c2_car_model_friction((SFc2_car_model_frictionInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c2_car_model_friction(void *chartInstanceVar)
{
  disable_c2_car_model_friction((SFc2_car_model_frictionInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c2_car_model_friction(void *chartInstanceVar)
{
  sf_c2_car_model_friction((SFc2_car_model_frictionInstanceStruct*)
    chartInstanceVar);
}

static mxArray* sf_internal_get_sim_state_c2_car_model_friction(SimStruct* S)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c2_car_model_friction
    ((SFc2_car_model_frictionInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = sf_get_sim_state_info_c2_car_model_friction();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_raw2high'.\n");
  }

  return plhs[0];
}

static void sf_internal_set_sim_state_c2_car_model_friction(SimStruct* S, const
  mxArray *st)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = mxDuplicateArray(st);      /* high level simctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c2_car_model_friction();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c2_car_model_friction((SFc2_car_model_frictionInstanceStruct*)
    chartInfo->chartInstance, mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static mxArray* sf_opaque_get_sim_state_c2_car_model_friction(SimStruct* S)
{
  return sf_internal_get_sim_state_c2_car_model_friction(S);
}

static void sf_opaque_set_sim_state_c2_car_model_friction(SimStruct* S, const
  mxArray *st)
{
  sf_internal_set_sim_state_c2_car_model_friction(S, st);
}

static void sf_opaque_terminate_c2_car_model_friction(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc2_car_model_frictionInstanceStruct*) chartInstanceVar)
      ->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
    }

    finalize_c2_car_model_friction((SFc2_car_model_frictionInstanceStruct*)
      chartInstanceVar);
    free((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c2_car_model_friction(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c2_car_model_friction
      ((SFc2_car_model_frictionInstanceStruct*)(((ChartInfoStruct *)
         ssGetUserData(S))->chartInstance));
  }
}

static void mdlSetWorkWidths_c2_car_model_friction(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable("car_model_friction","car_model_friction",2);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop("car_model_friction","car_model_friction",
                2,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop("car_model_friction",
      "car_model_friction",2,"gatewayCannotBeInlinedMultipleTimes"));
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,"car_model_friction",
        "car_model_friction",2,2);
      sf_mark_chart_reusable_outputs(S,"car_model_friction","car_model_friction",
        2,3);
    }

    sf_set_rtw_dwork_info(S,"car_model_friction","car_model_friction",2);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
    ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  }

  ssSetChecksum0(S,(1090093505U));
  ssSetChecksum1(S,(4035758515U));
  ssSetChecksum2(S,(367719970U));
  ssSetChecksum3(S,(3890773984U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
}

static void mdlRTW_c2_car_model_friction(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    sf_write_symbol_mapping(S, "car_model_friction", "car_model_friction",2);
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c2_car_model_friction(SimStruct *S)
{
  SFc2_car_model_frictionInstanceStruct *chartInstance;
  chartInstance = (SFc2_car_model_frictionInstanceStruct *)malloc(sizeof
    (SFc2_car_model_frictionInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc2_car_model_frictionInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c2_car_model_friction;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c2_car_model_friction;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c2_car_model_friction;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c2_car_model_friction;
  chartInstance->chartInfo.disableChart =
    sf_opaque_disable_c2_car_model_friction;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c2_car_model_friction;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c2_car_model_friction;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c2_car_model_friction;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c2_car_model_friction;
  chartInstance->chartInfo.mdlStart = mdlStart_c2_car_model_friction;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c2_car_model_friction;
  chartInstance->chartInfo.extModeExec = NULL;
  chartInstance->chartInfo.restoreLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.restoreBeforeLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.storeCurrentConfiguration = NULL;
  chartInstance->S = S;
  ssSetUserData(S,(void *)(&(chartInstance->chartInfo)));/* register the chart instance with simstruct */
  if (!sim_mode_is_rtw_gen(S)) {
    init_dsm_address_info(chartInstance);
  }

  chart_debug_initialization(S,1);
}

void c2_car_model_friction_method_dispatcher(SimStruct *S, int_T method, void
  *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c2_car_model_friction(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c2_car_model_friction(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c2_car_model_friction(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c2_car_model_friction_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
