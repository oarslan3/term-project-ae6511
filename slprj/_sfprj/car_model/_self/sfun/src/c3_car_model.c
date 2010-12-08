/* Include files */

#include "blascompat32.h"
#include "car_model_sfun.h"
#include "c3_car_model.h"
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "car_model_sfun_debug_macros.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */
static const char *c3_debug_family_names[22] = { "l_F", "l_R", "Vx", "Vy", "r",
  "psi", "f_Fx", "f_Rx", "delta", "beta", "V", "s_Fy_num", "s_Fy_denum", "s_Fy",
  "s_Ry_num", "s_Ry_denum", "s_Ry", "nargin", "nargout", "X", "u", "s" };

/* Function Declarations */
static void initialize_c3_car_model(SFc3_car_modelInstanceStruct *chartInstance);
static void initialize_params_c3_car_model(SFc3_car_modelInstanceStruct
  *chartInstance);
static void enable_c3_car_model(SFc3_car_modelInstanceStruct *chartInstance);
static void disable_c3_car_model(SFc3_car_modelInstanceStruct *chartInstance);
static void c3_update_debugger_state_c3_car_model(SFc3_car_modelInstanceStruct
  *chartInstance);
static const mxArray *get_sim_state_c3_car_model(SFc3_car_modelInstanceStruct
  *chartInstance);
static void set_sim_state_c3_car_model(SFc3_car_modelInstanceStruct
  *chartInstance, const mxArray *c3_st);
static void finalize_c3_car_model(SFc3_car_modelInstanceStruct *chartInstance);
static void sf_c3_car_model(SFc3_car_modelInstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T c3_machineNumber, uint32_T
  c3_chartNumber);
static real_T c3_mpower(SFc3_car_modelInstanceStruct *chartInstance, real_T c3_a);
static void c3_eml_error(SFc3_car_modelInstanceStruct *chartInstance);
static const mxArray *c3_sf_marshall(void *chartInstanceVoid, void *c3_u);
static const mxArray *c3_b_sf_marshall(void *chartInstanceVoid, void *c3_u);
static const mxArray *c3_c_sf_marshall(void *chartInstanceVoid, void *c3_u);
static const mxArray *c3_d_sf_marshall(void *chartInstanceVoid, void *c3_u);
static void c3_info_helper(c3_ResolvedFunctionInfo c3_info[38]);
static const mxArray *c3_e_sf_marshall(void *chartInstanceVoid, void *c3_u);
static void c3_emlrt_marshallIn(SFc3_car_modelInstanceStruct *chartInstance,
  const mxArray *c3_s, const char_T *c3_name, real_T c3_y[2]);
static uint8_T c3_b_emlrt_marshallIn(SFc3_car_modelInstanceStruct *chartInstance,
  const mxArray *c3_b_is_active_c3_car_model, const char_T *c3_name);
static void init_dsm_address_info(SFc3_car_modelInstanceStruct *chartInstance);

/* Function Definitions */
static void initialize_c3_car_model(SFc3_car_modelInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  chartInstance->c3_is_active_c3_car_model = 0U;
}

static void initialize_params_c3_car_model(SFc3_car_modelInstanceStruct
  *chartInstance)
{
}

static void enable_c3_car_model(SFc3_car_modelInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void disable_c3_car_model(SFc3_car_modelInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void c3_update_debugger_state_c3_car_model(SFc3_car_modelInstanceStruct
  *chartInstance)
{
}

static const mxArray *get_sim_state_c3_car_model(SFc3_car_modelInstanceStruct
  *chartInstance)
{
  const mxArray *c3_st = NULL;
  const mxArray *c3_y = NULL;
  int32_T c3_i0;
  real_T c3_hoistedGlobal[2];
  int32_T c3_i1;
  real_T c3_u[2];
  const mxArray *c3_b_y = NULL;
  uint8_T c3_b_hoistedGlobal;
  uint8_T c3_b_u;
  const mxArray *c3_c_y = NULL;
  real_T (*c3_s)[2];
  c3_s = (real_T (*)[2])ssGetOutputPortSignal(chartInstance->S, 1);
  c3_st = NULL;
  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_createcellarray(2));
  for (c3_i0 = 0; c3_i0 < 2; c3_i0 = c3_i0 + 1) {
    c3_hoistedGlobal[c3_i0] = (*c3_s)[c3_i0];
  }

  for (c3_i1 = 0; c3_i1 < 2; c3_i1 = c3_i1 + 1) {
    c3_u[c3_i1] = c3_hoistedGlobal[c3_i1];
  }

  c3_b_y = NULL;
  sf_mex_assign(&c3_b_y, sf_mex_create("y", &c3_u, 0, 0U, 1U, 0U, 1, 2));
  sf_mex_setcell(c3_y, 0, c3_b_y);
  c3_b_hoistedGlobal = chartInstance->c3_is_active_c3_car_model;
  c3_b_u = c3_b_hoistedGlobal;
  c3_c_y = NULL;
  sf_mex_assign(&c3_c_y, sf_mex_create("y", &c3_b_u, 3, 0U, 0U, 0U, 0));
  sf_mex_setcell(c3_y, 1, c3_c_y);
  sf_mex_assign(&c3_st, c3_y);
  return c3_st;
}

static void set_sim_state_c3_car_model(SFc3_car_modelInstanceStruct
  *chartInstance, const mxArray *c3_st)
{
  const mxArray *c3_u;
  real_T c3_dv0[2];
  int32_T c3_i2;
  real_T (*c3_s)[2];
  c3_s = (real_T (*)[2])ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c3_doneDoubleBufferReInit = TRUE;
  c3_u = sf_mex_dup(c3_st);
  c3_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c3_u, 0)), "s",
                      c3_dv0);
  for (c3_i2 = 0; c3_i2 < 2; c3_i2 = c3_i2 + 1) {
    (*c3_s)[c3_i2] = c3_dv0[c3_i2];
  }

  chartInstance->c3_is_active_c3_car_model = c3_b_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getcell(c3_u, 1)),
    "is_active_c3_car_model");
  sf_mex_destroy(&c3_u);
  c3_update_debugger_state_c3_car_model(chartInstance);
  sf_mex_destroy(&c3_st);
}

static void finalize_c3_car_model(SFc3_car_modelInstanceStruct *chartInstance)
{
}

static void sf_c3_car_model(SFc3_car_modelInstanceStruct *chartInstance)
{
  int32_T c3_i3;
  int32_T c3_i4;
  int32_T c3_i5;
  int32_T c3_previousEvent;
  int32_T c3_i6;
  real_T c3_hoistedGlobal[4];
  int32_T c3_i7;
  real_T c3_b_hoistedGlobal[3];
  int32_T c3_i8;
  real_T c3_X[4];
  int32_T c3_i9;
  real_T c3_u[3];
  uint32_T c3_debug_family_var_map[22];
  real_T c3_l_F;
  real_T c3_l_R;
  real_T c3_Vx;
  real_T c3_Vy;
  real_T c3_r;
  real_T c3_psi;
  real_T c3_f_Fx;
  real_T c3_f_Rx;
  real_T c3_delta;
  real_T c3_beta;
  real_T c3_V;
  real_T c3_s_Fy_num;
  real_T c3_s_Fy_denum;
  real_T c3_s_Fy;
  real_T c3_s_Ry_num;
  real_T c3_s_Ry_denum;
  real_T c3_s_Ry;
  real_T c3_nargin = 2.0;
  real_T c3_nargout = 1.0;
  real_T c3_s[2];
  real_T c3_A;
  real_T c3_B;
  real_T c3_x;
  real_T c3_y;
  real_T c3_b_x;
  real_T c3_b_y;
  real_T c3_c_x;
  real_T c3_c_y;
  real_T c3_d_y;
  real_T c3_d_x;
  real_T c3_e_x;
  real_T c3_f_x;
  real_T c3_g_x;
  real_T c3_h_x;
  real_T c3_i_x;
  real_T c3_j_x;
  real_T c3_a;
  real_T c3_b;
  real_T c3_e_y;
  real_T c3_b_a;
  real_T c3_f_y;
  real_T c3_k_x;
  real_T c3_l_x;
  real_T c3_m_x;
  real_T c3_c_a;
  real_T c3_b_b;
  real_T c3_g_y;
  real_T c3_n_x;
  real_T c3_o_x;
  real_T c3_p_x;
  real_T c3_d_a;
  real_T c3_c_b;
  real_T c3_h_y;
  real_T c3_e_a;
  real_T c3_i_y;
  real_T c3_q_x;
  real_T c3_r_x;
  real_T c3_s_x;
  real_T c3_f_a;
  real_T c3_d_b;
  real_T c3_j_y;
  real_T c3_b_A;
  real_T c3_b_B;
  real_T c3_t_x;
  real_T c3_k_y;
  real_T c3_u_x;
  real_T c3_l_y;
  real_T c3_v_x;
  real_T c3_m_y;
  real_T c3_w_x;
  real_T c3_x_x;
  real_T c3_y_x;
  real_T c3_g_a;
  real_T c3_e_b;
  real_T c3_n_y;
  real_T c3_h_a;
  real_T c3_o_y;
  real_T c3_ab_x;
  real_T c3_bb_x;
  real_T c3_cb_x;
  real_T c3_i_a;
  real_T c3_f_b;
  real_T c3_c_A;
  real_T c3_c_B;
  real_T c3_db_x;
  real_T c3_p_y;
  real_T c3_eb_x;
  real_T c3_q_y;
  real_T c3_fb_x;
  real_T c3_r_y;
  int32_T c3_i10;
  real_T (*c3_b_s)[2];
  real_T (*c3_b_u)[3];
  real_T (*c3_b_X)[4];
  c3_b_u = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 1);
  c3_b_s = (real_T (*)[2])ssGetOutputPortSignal(chartInstance->S, 1);
  c3_b_X = (real_T (*)[4])ssGetInputPortSignal(chartInstance->S, 0);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG,2);
  for (c3_i3 = 0; c3_i3 < 4; c3_i3 = c3_i3 + 1) {
    _SFD_DATA_RANGE_CHECK((*c3_b_X)[c3_i3], 0U);
  }

  for (c3_i4 = 0; c3_i4 < 2; c3_i4 = c3_i4 + 1) {
    _SFD_DATA_RANGE_CHECK((*c3_b_s)[c3_i4], 1U);
  }

  for (c3_i5 = 0; c3_i5 < 3; c3_i5 = c3_i5 + 1) {
    _SFD_DATA_RANGE_CHECK((*c3_b_u)[c3_i5], 2U);
  }

  c3_previousEvent = _sfEvent_;
  _sfEvent_ = CALL_EVENT;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG,2);
  for (c3_i6 = 0; c3_i6 < 4; c3_i6 = c3_i6 + 1) {
    c3_hoistedGlobal[c3_i6] = (*c3_b_X)[c3_i6];
  }

  for (c3_i7 = 0; c3_i7 < 3; c3_i7 = c3_i7 + 1) {
    c3_b_hoistedGlobal[c3_i7] = (*c3_b_u)[c3_i7];
  }

  for (c3_i8 = 0; c3_i8 < 4; c3_i8 = c3_i8 + 1) {
    c3_X[c3_i8] = c3_hoistedGlobal[c3_i8];
  }

  for (c3_i9 = 0; c3_i9 < 3; c3_i9 = c3_i9 + 1) {
    c3_u[c3_i9] = c3_b_hoistedGlobal[c3_i9];
  }

  sf_debug_symbol_scope_push_eml(0U, 22U, 22U, c3_debug_family_names,
    c3_debug_family_var_map);
  sf_debug_symbol_scope_add_eml(&c3_l_F, c3_d_sf_marshall, 0U);
  sf_debug_symbol_scope_add_eml(&c3_l_R, c3_d_sf_marshall, 1U);
  sf_debug_symbol_scope_add_eml(&c3_Vx, c3_d_sf_marshall, 2U);
  sf_debug_symbol_scope_add_eml(&c3_Vy, c3_d_sf_marshall, 3U);
  sf_debug_symbol_scope_add_eml(&c3_r, c3_d_sf_marshall, 4U);
  sf_debug_symbol_scope_add_eml(&c3_psi, c3_d_sf_marshall, 5U);
  sf_debug_symbol_scope_add_eml(&c3_f_Fx, c3_d_sf_marshall, 6U);
  sf_debug_symbol_scope_add_eml(&c3_f_Rx, c3_d_sf_marshall, 7U);
  sf_debug_symbol_scope_add_eml(&c3_delta, c3_d_sf_marshall, 8U);
  sf_debug_symbol_scope_add_eml(&c3_beta, c3_d_sf_marshall, 9U);
  sf_debug_symbol_scope_add_eml(&c3_V, c3_d_sf_marshall, 10U);
  sf_debug_symbol_scope_add_eml(&c3_s_Fy_num, c3_d_sf_marshall, 11U);
  sf_debug_symbol_scope_add_eml(&c3_s_Fy_denum, c3_d_sf_marshall, 12U);
  sf_debug_symbol_scope_add_eml(&c3_s_Fy, c3_d_sf_marshall, 13U);
  sf_debug_symbol_scope_add_eml(&c3_s_Ry_num, c3_d_sf_marshall, 14U);
  sf_debug_symbol_scope_add_eml(&c3_s_Ry_denum, c3_d_sf_marshall, 15U);
  sf_debug_symbol_scope_add_eml(&c3_s_Ry, c3_d_sf_marshall, 16U);
  sf_debug_symbol_scope_add_eml(&c3_nargin, c3_d_sf_marshall, 17U);
  sf_debug_symbol_scope_add_eml(&c3_nargout, c3_d_sf_marshall, 18U);
  sf_debug_symbol_scope_add_eml(&c3_X, c3_c_sf_marshall, 19U);
  sf_debug_symbol_scope_add_eml(&c3_u, c3_b_sf_marshall, 20U);
  sf_debug_symbol_scope_add_eml(&c3_s, c3_sf_marshall, 21U);
  CV_EML_FCN(0, 0);

  /*  l_F = param.l_F; */
  /*  l_R = param.l_R; */
  _SFD_EML_CALL(0,7);
  c3_l_F = 1.1;
  _SFD_EML_CALL(0,8);
  c3_l_R = 1.6;
  _SFD_EML_CALL(0,10);
  c3_Vx = c3_X[0];
  _SFD_EML_CALL(0,11);
  c3_Vy = c3_X[1];
  _SFD_EML_CALL(0,12);
  c3_r = c3_X[2];
  _SFD_EML_CALL(0,13);
  c3_psi = c3_X[3];
  _SFD_EML_CALL(0,15);
  c3_f_Fx = c3_u[0];
  _SFD_EML_CALL(0,16);
  c3_f_Rx = c3_u[1];
  _SFD_EML_CALL(0,17);
  c3_delta = c3_u[2];
  _SFD_EML_CALL(0,20);
  if (CV_EML_COND(0, 0, c3_Vx == 0.0)) {
    if (CV_EML_COND(0, 1, c3_Vy == 0.0)) {
      CV_EML_MCDC(0, 0, TRUE);
      CV_EML_IF(0, 0, TRUE);
      _SFD_EML_CALL(0,21);
      c3_beta = 0.0;
      goto label_1;
    }
  }

  CV_EML_MCDC(0, 0, FALSE);
  CV_EML_IF(0, 0, FALSE);
  _SFD_EML_CALL(0,23);
  c3_A = c3_Vy;
  c3_B = c3_Vx;
  c3_x = c3_A;
  c3_y = c3_B;
  c3_b_x = c3_x;
  c3_b_y = c3_y;
  c3_c_x = c3_b_x;
  c3_c_y = c3_b_y;
  c3_d_y = c3_c_x / c3_c_y;
  c3_d_x = c3_d_y;
  c3_beta = c3_d_x;
  c3_e_x = c3_beta;
  c3_beta = c3_e_x;
  c3_beta = muDoubleScalarAtan(c3_beta);
 label_1:
  ;
  _SFD_EML_CALL(0,26);
  c3_f_x = c3_mpower(chartInstance, c3_Vx) + c3_mpower(chartInstance, c3_Vy);
  c3_V = c3_f_x;
  if (c3_V < 0.0) {
    c3_eml_error(chartInstance);
  }

  c3_g_x = c3_V;
  c3_V = c3_g_x;
  c3_V = muDoubleScalarSqrt(c3_V);
  _SFD_EML_CALL(0,28);
  c3_h_x = c3_beta - c3_delta;
  c3_i_x = c3_h_x;
  c3_j_x = c3_i_x;
  c3_i_x = c3_j_x;
  c3_i_x = muDoubleScalarSin(c3_i_x);
  c3_a = c3_V;
  c3_b = c3_i_x;
  c3_e_y = c3_a * c3_b;
  c3_b_a = c3_r;
  c3_f_y = c3_b_a * 1.1;
  c3_k_x = c3_delta;
  c3_l_x = c3_k_x;
  c3_m_x = c3_l_x;
  c3_l_x = c3_m_x;
  c3_l_x = muDoubleScalarCos(c3_l_x);
  c3_c_a = c3_f_y;
  c3_b_b = c3_l_x;
  c3_g_y = c3_c_a * c3_b_b;
  c3_s_Fy_num = c3_e_y + c3_g_y;
  _SFD_EML_CALL(0,29);
  c3_n_x = c3_beta - c3_delta;
  c3_o_x = c3_n_x;
  c3_p_x = c3_o_x;
  c3_o_x = c3_p_x;
  c3_o_x = muDoubleScalarCos(c3_o_x);
  c3_d_a = c3_V;
  c3_c_b = c3_o_x;
  c3_h_y = c3_d_a * c3_c_b;
  c3_e_a = c3_r;
  c3_i_y = c3_e_a * 1.1;
  c3_q_x = c3_delta;
  c3_r_x = c3_q_x;
  c3_s_x = c3_r_x;
  c3_r_x = c3_s_x;
  c3_r_x = muDoubleScalarSin(c3_r_x);
  c3_f_a = c3_i_y;
  c3_d_b = c3_r_x;
  c3_j_y = c3_f_a * c3_d_b;
  c3_s_Fy_denum = c3_h_y + c3_j_y;
  _SFD_EML_CALL(0,31);
  if (CV_EML_COND(0, 2, c3_s_Fy_num == 0.0)) {
    if (CV_EML_COND(0, 3, c3_s_Fy_denum == 0.0)) {
      CV_EML_MCDC(0, 1, TRUE);
      CV_EML_IF(0, 1, TRUE);
      _SFD_EML_CALL(0,32);
      c3_s_Fy = 0.0;
      goto label_2;
    }
  }

  CV_EML_MCDC(0, 1, FALSE);
  CV_EML_IF(0, 1, FALSE);
  _SFD_EML_CALL(0,34);
  c3_b_A = c3_s_Fy_num;
  c3_b_B = c3_s_Fy_denum;
  c3_t_x = c3_b_A;
  c3_k_y = c3_b_B;
  c3_u_x = c3_t_x;
  c3_l_y = c3_k_y;
  c3_v_x = c3_u_x;
  c3_m_y = c3_l_y;
  c3_s_Fy = c3_v_x / c3_m_y;
 label_2:
  ;
  _SFD_EML_CALL(0,37);
  c3_w_x = c3_beta;
  c3_x_x = c3_w_x;
  c3_y_x = c3_x_x;
  c3_x_x = c3_y_x;
  c3_x_x = muDoubleScalarSin(c3_x_x);
  c3_g_a = c3_V;
  c3_e_b = c3_x_x;
  c3_n_y = c3_g_a * c3_e_b;
  c3_h_a = c3_r;
  c3_o_y = c3_h_a * 1.6;
  c3_s_Ry_num = c3_n_y - c3_o_y;
  _SFD_EML_CALL(0,38);
  c3_ab_x = c3_beta;
  c3_bb_x = c3_ab_x;
  c3_cb_x = c3_bb_x;
  c3_bb_x = c3_cb_x;
  c3_bb_x = muDoubleScalarCos(c3_bb_x);
  c3_i_a = c3_V;
  c3_f_b = c3_bb_x;
  c3_s_Ry_denum = c3_i_a * c3_f_b;
  _SFD_EML_CALL(0,40);
  if (CV_EML_COND(0, 4, c3_s_Ry_num == 0.0)) {
    if (CV_EML_COND(0, 5, c3_s_Ry_denum == 0.0)) {
      CV_EML_MCDC(0, 2, TRUE);
      CV_EML_IF(0, 2, TRUE);
      _SFD_EML_CALL(0,41);
      c3_s_Ry = 0.0;
      goto label_3;
    }
  }

  CV_EML_MCDC(0, 2, FALSE);
  CV_EML_IF(0, 2, FALSE);
  _SFD_EML_CALL(0,43);
  c3_c_A = c3_s_Ry_num;
  c3_c_B = c3_s_Ry_denum;
  c3_db_x = c3_c_A;
  c3_p_y = c3_c_B;
  c3_eb_x = c3_db_x;
  c3_q_y = c3_p_y;
  c3_fb_x = c3_eb_x;
  c3_r_y = c3_q_y;
  c3_s_Ry = c3_fb_x / c3_r_y;
 label_3:
  ;
  _SFD_EML_CALL(0,46);
  c3_s[0] = c3_s_Fy;
  c3_s[1] = c3_s_Ry;
  _SFD_EML_CALL(0,-46);
  sf_debug_symbol_scope_pop();
  for (c3_i10 = 0; c3_i10 < 2; c3_i10 = c3_i10 + 1) {
    (*c3_b_s)[c3_i10] = c3_s[c3_i10];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG,2);
  _sfEvent_ = c3_previousEvent;
  sf_debug_check_for_state_inconsistency(_car_modelMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void init_script_number_translation(uint32_T c3_machineNumber, uint32_T
  c3_chartNumber)
{
}

static real_T c3_mpower(SFc3_car_modelInstanceStruct *chartInstance, real_T c3_a)
{
  real_T c3_b_a;
  real_T c3_ak;
  c3_b_a = c3_a;
  c3_ak = c3_b_a;
  return muDoubleScalarPower(c3_ak, 2.0);
}

static void c3_eml_error(SFc3_car_modelInstanceStruct *chartInstance)
{
  int32_T c3_i11;
  static char_T c3_cv0[31] = { 'E', 'm', 'b', 'e', 'd', 'd', 'e', 'd', 'M', 'A',
    'T', 'L', 'A', 'B', ':', 's', 'q', 'r', 't', ':', 'd'
    , 'o', 'm', 'a', 'i', 'n', 'E', 'r', 'r', 'o', 'r' };

  char_T c3_u[31];
  const mxArray *c3_y = NULL;
  int32_T c3_i12;
  static char_T c3_cv1[77] = { 'D', 'o', 'm', 'a', 'i', 'n', ' ', 'e', 'r', 'r',
    'o', 'r', '.', ' ', 'T', 'o', ' ', 'c', 'o', 'm', 'p'
    , 'u', 't', 'e', ' ', 'c', 'o', 'm', 'p', 'l', 'e', 'x', ' ', 'r', 'e', 's',
    'u', 'l', 't', 's', ' ',
    'f', 'r', 'o', 'm', ' ', 'r', 'e', 'a', 'l', ' ', 'x', ',', ' ', 'u', 's',
    'e', ' ', '\'', 's', 'q',
    'r', 't', '(', 'c', 'o', 'm', 'p', 'l', 'e', 'x', '(', 'x', ')', ')', '\'',
    '.' };

  char_T c3_b_u[77];
  const mxArray *c3_b_y = NULL;
  for (c3_i11 = 0; c3_i11 < 31; c3_i11 = c3_i11 + 1) {
    c3_u[c3_i11] = c3_cv0[c3_i11];
  }

  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", &c3_u, 10, 0U, 1U, 0U, 2, 1, 31));
  for (c3_i12 = 0; c3_i12 < 77; c3_i12 = c3_i12 + 1) {
    c3_b_u[c3_i12] = c3_cv1[c3_i12];
  }

  c3_b_y = NULL;
  sf_mex_assign(&c3_b_y, sf_mex_create("y", &c3_b_u, 10, 0U, 1U, 0U, 2, 1, 77));
  sf_mex_call_debug("error", 0U, 2U, 14, c3_y, 14, c3_b_y);
}

static const mxArray *c3_sf_marshall(void *chartInstanceVoid, void *c3_u)
{
  const mxArray *c3_y = NULL;
  int32_T c3_i13;
  real_T c3_b_u[2];
  const mxArray *c3_b_y = NULL;
  SFc3_car_modelInstanceStruct *chartInstance;
  chartInstance = (SFc3_car_modelInstanceStruct *)chartInstanceVoid;
  c3_y = NULL;
  for (c3_i13 = 0; c3_i13 < 2; c3_i13 = c3_i13 + 1) {
    c3_b_u[c3_i13] = (*((real_T (*)[2])c3_u))[c3_i13];
  }

  c3_b_y = NULL;
  sf_mex_assign(&c3_b_y, sf_mex_create("y", &c3_b_u, 0, 0U, 1U, 0U, 1, 2));
  sf_mex_assign(&c3_y, c3_b_y);
  return c3_y;
}

static const mxArray *c3_b_sf_marshall(void *chartInstanceVoid, void *c3_u)
{
  const mxArray *c3_y = NULL;
  int32_T c3_i14;
  real_T c3_b_u[3];
  const mxArray *c3_b_y = NULL;
  SFc3_car_modelInstanceStruct *chartInstance;
  chartInstance = (SFc3_car_modelInstanceStruct *)chartInstanceVoid;
  c3_y = NULL;
  for (c3_i14 = 0; c3_i14 < 3; c3_i14 = c3_i14 + 1) {
    c3_b_u[c3_i14] = (*((real_T (*)[3])c3_u))[c3_i14];
  }

  c3_b_y = NULL;
  sf_mex_assign(&c3_b_y, sf_mex_create("y", &c3_b_u, 0, 0U, 1U, 0U, 1, 3));
  sf_mex_assign(&c3_y, c3_b_y);
  return c3_y;
}

static const mxArray *c3_c_sf_marshall(void *chartInstanceVoid, void *c3_u)
{
  const mxArray *c3_y = NULL;
  int32_T c3_i15;
  real_T c3_b_u[4];
  const mxArray *c3_b_y = NULL;
  SFc3_car_modelInstanceStruct *chartInstance;
  chartInstance = (SFc3_car_modelInstanceStruct *)chartInstanceVoid;
  c3_y = NULL;
  for (c3_i15 = 0; c3_i15 < 4; c3_i15 = c3_i15 + 1) {
    c3_b_u[c3_i15] = (*((real_T (*)[4])c3_u))[c3_i15];
  }

  c3_b_y = NULL;
  sf_mex_assign(&c3_b_y, sf_mex_create("y", &c3_b_u, 0, 0U, 1U, 0U, 1, 4));
  sf_mex_assign(&c3_y, c3_b_y);
  return c3_y;
}

static const mxArray *c3_d_sf_marshall(void *chartInstanceVoid, void *c3_u)
{
  const mxArray *c3_y = NULL;
  real_T c3_b_u;
  const mxArray *c3_b_y = NULL;
  SFc3_car_modelInstanceStruct *chartInstance;
  chartInstance = (SFc3_car_modelInstanceStruct *)chartInstanceVoid;
  c3_y = NULL;
  c3_b_u = *((real_T *)c3_u);
  c3_b_y = NULL;
  sf_mex_assign(&c3_b_y, sf_mex_create("y", &c3_b_u, 0, 0U, 0U, 0U, 0));
  sf_mex_assign(&c3_y, c3_b_y);
  return c3_y;
}

const mxArray *sf_c3_car_model_get_eml_resolved_functions_info(void)
{
  const mxArray *c3_nameCaptureInfo = NULL;
  c3_ResolvedFunctionInfo c3_info[38];
  const mxArray *c3_m0 = NULL;
  int32_T c3_i16;
  c3_ResolvedFunctionInfo *c3_r0;
  c3_nameCaptureInfo = NULL;
  c3_info_helper(c3_info);
  sf_mex_assign(&c3_m0, sf_mex_createstruct("nameCaptureInfo", 1, 38));
  for (c3_i16 = 0; c3_i16 < 38; c3_i16 = c3_i16 + 1) {
    c3_r0 = &c3_info[c3_i16];
    sf_mex_addfield(c3_m0, sf_mex_create("nameCaptureInfo", c3_r0->context, 15,
      0U, 0U, 0U, 2, 1, strlen(c3_r0->context)), "context",
                    "nameCaptureInfo", c3_i16);
    sf_mex_addfield(c3_m0, sf_mex_create("nameCaptureInfo", c3_r0->name, 15, 0U,
      0U, 0U, 2, 1, strlen(c3_r0->name)), "name",
                    "nameCaptureInfo", c3_i16);
    sf_mex_addfield(c3_m0, sf_mex_create("nameCaptureInfo", c3_r0->dominantType,
      15, 0U, 0U, 0U, 2, 1, strlen(c3_r0->dominantType)),
                    "dominantType", "nameCaptureInfo", c3_i16);
    sf_mex_addfield(c3_m0, sf_mex_create("nameCaptureInfo", c3_r0->resolved, 15,
      0U, 0U, 0U, 2, 1, strlen(c3_r0->resolved)), "resolved"
                    , "nameCaptureInfo", c3_i16);
    sf_mex_addfield(c3_m0, sf_mex_create("nameCaptureInfo", &c3_r0->fileLength,
      7, 0U, 0U, 0U, 0), "fileLength", "nameCaptureInfo",
                    c3_i16);
    sf_mex_addfield(c3_m0, sf_mex_create("nameCaptureInfo", &c3_r0->fileTime1, 7,
      0U, 0U, 0U, 0), "fileTime1", "nameCaptureInfo", c3_i16
                    );
    sf_mex_addfield(c3_m0, sf_mex_create("nameCaptureInfo", &c3_r0->fileTime2, 7,
      0U, 0U, 0U, 0), "fileTime2", "nameCaptureInfo", c3_i16
                    );
  }

  sf_mex_assign(&c3_nameCaptureInfo, c3_m0);
  return c3_nameCaptureInfo;
}

static void c3_info_helper(c3_ResolvedFunctionInfo c3_info[38])
{
  c3_info[0].context = "";
  c3_info[0].name = "eq";
  c3_info[0].dominantType = "double";
  c3_info[0].resolved = "[B]eq";
  c3_info[0].fileLength = 0U;
  c3_info[0].fileTime1 = 0U;
  c3_info[0].fileTime2 = 0U;
  c3_info[1].context = "";
  c3_info[1].name = "mrdivide";
  c3_info[1].dominantType = "double";
  c3_info[1].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.m";
  c3_info[1].fileLength = 800U;
  c3_info[1].fileTime1 = 1238455890U;
  c3_info[1].fileTime2 = 0U;
  c3_info[2].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.m";
  c3_info[2].name = "nargin";
  c3_info[2].dominantType = "";
  c3_info[2].resolved = "[B]nargin";
  c3_info[2].fileLength = 0U;
  c3_info[2].fileTime1 = 0U;
  c3_info[2].fileTime2 = 0U;
  c3_info[3].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.m";
  c3_info[3].name = "ge";
  c3_info[3].dominantType = "double";
  c3_info[3].resolved = "[B]ge";
  c3_info[3].fileLength = 0U;
  c3_info[3].fileTime1 = 0U;
  c3_info[3].fileTime2 = 0U;
  c3_info[4].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.m";
  c3_info[4].name = "isscalar";
  c3_info[4].dominantType = "double";
  c3_info[4].resolved = "[B]isscalar";
  c3_info[4].fileLength = 0U;
  c3_info[4].fileTime1 = 0U;
  c3_info[4].fileTime2 = 0U;
  c3_info[5].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.m";
  c3_info[5].name = "rdivide";
  c3_info[5].dominantType = "double";
  c3_info[5].resolved = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c3_info[5].fileLength = 403U;
  c3_info[5].fileTime1 = 1244757152U;
  c3_info[5].fileTime2 = 0U;
  c3_info[6].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c3_info[6].name = "gt";
  c3_info[6].dominantType = "double";
  c3_info[6].resolved = "[B]gt";
  c3_info[6].fileLength = 0U;
  c3_info[6].fileTime1 = 0U;
  c3_info[6].fileTime2 = 0U;
  c3_info[7].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c3_info[7].name = "isa";
  c3_info[7].dominantType = "double";
  c3_info[7].resolved = "[B]isa";
  c3_info[7].fileLength = 0U;
  c3_info[7].fileTime1 = 0U;
  c3_info[7].fileTime2 = 0U;
  c3_info[8].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c3_info[8].name = "eml_div";
  c3_info[8].dominantType = "double";
  c3_info[8].resolved = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c3_info[8].fileLength = 4269U;
  c3_info[8].fileTime1 = 1228115426U;
  c3_info[8].fileTime2 = 0U;
  c3_info[9].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c3_info[9].name = "isinteger";
  c3_info[9].dominantType = "double";
  c3_info[9].resolved = "[B]isinteger";
  c3_info[9].fileLength = 0U;
  c3_info[9].fileTime1 = 0U;
  c3_info[9].fileTime2 = 0U;
  c3_info[10].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m/eml_fldiv";
  c3_info[10].name = "isreal";
  c3_info[10].dominantType = "double";
  c3_info[10].resolved = "[B]isreal";
  c3_info[10].fileLength = 0U;
  c3_info[10].fileTime1 = 0U;
  c3_info[10].fileTime2 = 0U;
  c3_info[11].context = "";
  c3_info[11].name = "atan";
  c3_info[11].dominantType = "double";
  c3_info[11].resolved = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/atan.m";
  c3_info[11].fileLength = 536U;
  c3_info[11].fileTime1 = 1203469546U;
  c3_info[11].fileTime2 = 0U;
  c3_info[12].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/atan.m";
  c3_info[12].name = "not";
  c3_info[12].dominantType = "logical";
  c3_info[12].resolved = "[B]not";
  c3_info[12].fileLength = 0U;
  c3_info[12].fileTime1 = 0U;
  c3_info[12].fileTime2 = 0U;
  c3_info[13].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/atan.m";
  c3_info[13].name = "eml_scalar_atan";
  c3_info[13].dominantType = "double";
  c3_info[13].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_atan.m";
  c3_info[13].fileLength = 281U;
  c3_info[13].fileTime1 = 1203469576U;
  c3_info[13].fileTime2 = 0U;
  c3_info[14].context = "";
  c3_info[14].name = "mpower";
  c3_info[14].dominantType = "double";
  c3_info[14].resolved = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m";
  c3_info[14].fileLength = 3710U;
  c3_info[14].fileTime1 = 1238455888U;
  c3_info[14].fileTime2 = 0U;
  c3_info[15].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m";
  c3_info[15].name = "ndims";
  c3_info[15].dominantType = "double";
  c3_info[15].resolved = "[B]ndims";
  c3_info[15].fileLength = 0U;
  c3_info[15].fileTime1 = 0U;
  c3_info[15].fileTime2 = 0U;
  c3_info[16].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m";
  c3_info[16].name = "size";
  c3_info[16].dominantType = "double";
  c3_info[16].resolved = "[B]size";
  c3_info[16].fileLength = 0U;
  c3_info[16].fileTime1 = 0U;
  c3_info[16].fileTime2 = 0U;
  c3_info[17].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m";
  c3_info[17].name = "power";
  c3_info[17].dominantType = "double";
  c3_info[17].resolved = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m";
  c3_info[17].fileLength = 5380U;
  c3_info[17].fileTime1 = 1228115498U;
  c3_info[17].fileTime2 = 0U;
  c3_info[18].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m";
  c3_info[18].name = "eml_scalar_eg";
  c3_info[18].dominantType = "double";
  c3_info[18].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c3_info[18].fileLength = 3068U;
  c3_info[18].fileTime1 = 1240283610U;
  c3_info[18].fileTime2 = 0U;
  c3_info[19].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m/any_enums";
  c3_info[19].name = "false";
  c3_info[19].dominantType = "";
  c3_info[19].resolved = "[B]false";
  c3_info[19].fileLength = 0U;
  c3_info[19].fileTime1 = 0U;
  c3_info[19].fileTime2 = 0U;
  c3_info[20].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c3_info[20].name = "isstruct";
  c3_info[20].dominantType = "double";
  c3_info[20].resolved = "[B]isstruct";
  c3_info[20].fileLength = 0U;
  c3_info[20].fileTime1 = 0U;
  c3_info[20].fileTime2 = 0U;
  c3_info[21].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m/zerosum";
  c3_info[21].name = "class";
  c3_info[21].dominantType = "double";
  c3_info[21].resolved = "[B]class";
  c3_info[21].fileLength = 0U;
  c3_info[21].fileTime1 = 0U;
  c3_info[21].fileTime2 = 0U;
  c3_info[22].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m/zerosum";
  c3_info[22].name = "cast";
  c3_info[22].dominantType = "double";
  c3_info[22].resolved = "[B]cast";
  c3_info[22].fileLength = 0U;
  c3_info[22].fileTime1 = 0U;
  c3_info[22].fileTime2 = 0U;
  c3_info[23].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m/zerosum";
  c3_info[23].name = "plus";
  c3_info[23].dominantType = "double";
  c3_info[23].resolved = "[B]plus";
  c3_info[23].fileLength = 0U;
  c3_info[23].fileTime1 = 0U;
  c3_info[23].fileTime2 = 0U;
  c3_info[24].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m";
  c3_info[24].name = "eml_scalexp_alloc";
  c3_info[24].dominantType = "double";
  c3_info[24].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c3_info[24].fileLength = 808U;
  c3_info[24].fileTime1 = 1230516298U;
  c3_info[24].fileTime2 = 0U;
  c3_info[25].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c3_info[25].name = "minus";
  c3_info[25].dominantType = "double";
  c3_info[25].resolved = "[B]minus";
  c3_info[25].fileLength = 0U;
  c3_info[25].fileTime1 = 0U;
  c3_info[25].fileTime2 = 0U;
  c3_info[26].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m";
  c3_info[26].name = "lt";
  c3_info[26].dominantType = "double";
  c3_info[26].resolved = "[B]lt";
  c3_info[26].fileLength = 0U;
  c3_info[26].fileTime1 = 0U;
  c3_info[26].fileTime2 = 0U;
  c3_info[27].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m";
  c3_info[27].name = "eml_scalar_floor";
  c3_info[27].dominantType = "double";
  c3_info[27].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m";
  c3_info[27].fileLength = 260U;
  c3_info[27].fileTime1 = 1209352390U;
  c3_info[27].fileTime2 = 0U;
  c3_info[28].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m";
  c3_info[28].name = "ne";
  c3_info[28].dominantType = "double";
  c3_info[28].resolved = "[B]ne";
  c3_info[28].fileLength = 0U;
  c3_info[28].fileTime1 = 0U;
  c3_info[28].fileTime2 = 0U;
  c3_info[29].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m";
  c3_info[29].name = "eml_error";
  c3_info[29].dominantType = "char";
  c3_info[29].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m";
  c3_info[29].fileLength = 315U;
  c3_info[29].fileTime1 = 1213948344U;
  c3_info[29].fileTime2 = 0U;
  c3_info[30].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m";
  c3_info[30].name = "strcmp";
  c3_info[30].dominantType = "char";
  c3_info[30].resolved = "[B]strcmp";
  c3_info[30].fileLength = 0U;
  c3_info[30].fileTime1 = 0U;
  c3_info[30].fileTime2 = 0U;
  c3_info[31].context = "";
  c3_info[31].name = "sqrt";
  c3_info[31].dominantType = "double";
  c3_info[31].resolved = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m";
  c3_info[31].fileLength = 572U;
  c3_info[31].fileTime1 = 1203469644U;
  c3_info[31].fileTime2 = 0U;
  c3_info[32].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m";
  c3_info[32].name = "eml_scalar_sqrt";
  c3_info[32].dominantType = "double";
  c3_info[32].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_sqrt.m";
  c3_info[32].fileLength = 664U;
  c3_info[32].fileTime1 = 1209352394U;
  c3_info[32].fileTime2 = 0U;
  c3_info[33].context = "";
  c3_info[33].name = "sin";
  c3_info[33].dominantType = "double";
  c3_info[33].resolved = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/sin.m";
  c3_info[33].fileLength = 324U;
  c3_info[33].fileTime1 = 1203469642U;
  c3_info[33].fileTime2 = 0U;
  c3_info[34].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/sin.m";
  c3_info[34].name = "eml_scalar_sin";
  c3_info[34].dominantType = "double";
  c3_info[34].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_sin.m";
  c3_info[34].fileLength = 601U;
  c3_info[34].fileTime1 = 1209352390U;
  c3_info[34].fileTime2 = 0U;
  c3_info[35].context = "";
  c3_info[35].name = "mtimes";
  c3_info[35].dominantType = "double";
  c3_info[35].resolved = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c3_info[35].fileLength = 3425U;
  c3_info[35].fileTime1 = 1250694366U;
  c3_info[35].fileTime2 = 0U;
  c3_info[36].context = "";
  c3_info[36].name = "cos";
  c3_info[36].dominantType = "double";
  c3_info[36].resolved = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/cos.m";
  c3_info[36].fileLength = 324U;
  c3_info[36].fileTime1 = 1203469550U;
  c3_info[36].fileTime2 = 0U;
  c3_info[37].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/cos.m";
  c3_info[37].name = "eml_scalar_cos";
  c3_info[37].dominantType = "double";
  c3_info[37].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_cos.m";
  c3_info[37].fileLength = 602U;
  c3_info[37].fileTime1 = 1209352386U;
  c3_info[37].fileTime2 = 0U;
}

static const mxArray *c3_e_sf_marshall(void *chartInstanceVoid, void *c3_u)
{
  const mxArray *c3_y = NULL;
  boolean_T c3_b_u;
  const mxArray *c3_b_y = NULL;
  SFc3_car_modelInstanceStruct *chartInstance;
  chartInstance = (SFc3_car_modelInstanceStruct *)chartInstanceVoid;
  c3_y = NULL;
  c3_b_u = *((boolean_T *)c3_u);
  c3_b_y = NULL;
  sf_mex_assign(&c3_b_y, sf_mex_create("y", &c3_b_u, 11, 0U, 0U, 0U, 0));
  sf_mex_assign(&c3_y, c3_b_y);
  return c3_y;
}

static void c3_emlrt_marshallIn(SFc3_car_modelInstanceStruct *chartInstance,
  const mxArray *c3_s, const char_T *c3_name, real_T c3_y
  [2])
{
  real_T c3_dv1[2];
  int32_T c3_i17;
  sf_mex_import(c3_name, sf_mex_dup(c3_s), &c3_dv1, 1, 0, 0U, 1, 0U, 1, 2);
  for (c3_i17 = 0; c3_i17 < 2; c3_i17 = c3_i17 + 1) {
    c3_y[c3_i17] = c3_dv1[c3_i17];
  }

  sf_mex_destroy(&c3_s);
}

static uint8_T c3_b_emlrt_marshallIn(SFc3_car_modelInstanceStruct *chartInstance,
  const mxArray *c3_b_is_active_c3_car_model, const
  char_T *c3_name)
{
  uint8_T c3_y;
  uint8_T c3_u0;
  sf_mex_import(c3_name, sf_mex_dup(c3_b_is_active_c3_car_model), &c3_u0, 1, 3,
                0U, 0, 0U, 0);
  c3_y = c3_u0;
  sf_mex_destroy(&c3_b_is_active_c3_car_model);
  return c3_y;
}

static void init_dsm_address_info(SFc3_car_modelInstanceStruct *chartInstance)
{
}

/* SFunction Glue Code */
void sf_c3_car_model_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(3965561185U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(1856121540U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3850389951U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(4180257981U);
}

mxArray *sf_c3_car_model_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,4,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateDoubleMatrix(4,1,mxREAL);
    double *pr = mxGetPr(mxChecksum);
    pr[0] = (double)(21589190U);
    pr[1] = (double)(1873670222U);
    pr[2] = (double)(3542488969U);
    pr[3] = (double)(3186658690U);
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,2,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(4);
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

    mxArray *mxData = mxCreateStructMatrix(1,1,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(2);
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
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  return(mxAutoinheritanceInfo);
}

static mxArray *sf_get_sim_state_info_c3_car_model(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x2'type','srcId','name','auxInfo'{{M[1],M[5],T\"s\",},{M[8],M[0],T\"is_active_c3_car_model\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 2, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c3_car_model_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc3_car_modelInstanceStruct *chartInstance;
    chartInstance = (SFc3_car_modelInstanceStruct *) ((ChartInfoStruct *)
      (ssGetUserData(S)))->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart(_car_modelMachineNumber_,
          3,
          1,
          1,
          3,
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
          init_script_number_translation(_car_modelMachineNumber_,
            chartInstance->chartNumber);
          sf_debug_set_chart_disable_implicit_casting(_car_modelMachineNumber_,
            chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds(_car_modelMachineNumber_,
            chartInstance->chartNumber,
            0,
            0,
            0);

          {
            unsigned int dimVector[1];
            dimVector[0]= 4;
            _SFD_SET_DATA_PROPS(0,1,1,0,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
                                1.0,0,"X",0,(MexFcnForType)c3_c_sf_marshall);
          }

          {
            unsigned int dimVector[1];
            dimVector[0]= 2;
            _SFD_SET_DATA_PROPS(1,2,0,1,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
                                1.0,0,"s",0,(MexFcnForType)c3_sf_marshall);
          }

          {
            unsigned int dimVector[1];
            dimVector[0]= 3;
            _SFD_SET_DATA_PROPS(2,1,1,0,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
                                1.0,0,"u",0,(MexFcnForType)c3_b_sf_marshall);
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
        _SFD_CV_INIT_EML(0,1,3,0,0,0,0,6,3);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,665);
        _SFD_CV_INIT_EML_IF(0,0,197,219,234,266);
        _SFD_CV_INIT_EML_IF(0,1,397,434,449,489);
        _SFD_CV_INIT_EML_IF(0,2,554,591,606,646);

        {
          static int condStart[] = { 201, 212 };

          static int condEnd[] = { 208, 218 };

          static int pfixExpr[] = { 0, 1, -3 };

          _SFD_CV_INIT_EML_MCDC(0,0,201,218,2,0,&(condStart[0]),&(condEnd[0]),3,
                                &(pfixExpr[0]));
        }

        {
          static int condStart[] = { 401, 418 };

          static int condEnd[] = { 414, 433 };

          static int pfixExpr[] = { 0, 1, -3 };

          _SFD_CV_INIT_EML_MCDC(0,1,401,433,2,2,&(condStart[0]),&(condEnd[0]),3,
                                &(pfixExpr[0]));
        }

        {
          static int condStart[] = { 558, 575 };

          static int condEnd[] = { 571, 590 };

          static int pfixExpr[] = { 0, 1, -3 };

          _SFD_CV_INIT_EML_MCDC(0,2,558,590,2,4,&(condStart[0]),&(condEnd[0]),3,
                                &(pfixExpr[0]));
        }

        _SFD_TRANS_COV_WTS(0,0,0,1,0);
        if (chartAlreadyPresent==0) {
          _SFD_TRANS_COV_MAPS(0,
                              0,NULL,NULL,
                              0,NULL,NULL,
                              1,NULL,NULL,
                              0,NULL,NULL);
        }

        {
          real_T (*c3_X)[4];
          real_T (*c3_s)[2];
          real_T (*c3_u)[3];
          c3_u = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 1);
          c3_s = (real_T (*)[2])ssGetOutputPortSignal(chartInstance->S, 1);
          c3_X = (real_T (*)[4])ssGetInputPortSignal(chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, c3_X);
          _SFD_SET_DATA_VALUE_PTR(1U, c3_s);
          _SFD_SET_DATA_VALUE_PTR(2U, c3_u);
        }
      }
    } else {
      sf_debug_reset_current_state_configuration(_car_modelMachineNumber_,
        chartInstance->chartNumber,chartInstance->instanceNumber);
    }
  }
}

static void sf_opaque_initialize_c3_car_model(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc3_car_modelInstanceStruct*) chartInstanceVar
    )->S,0);
  initialize_params_c3_car_model((SFc3_car_modelInstanceStruct*)
    chartInstanceVar);
  initialize_c3_car_model((SFc3_car_modelInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_enable_c3_car_model(void *chartInstanceVar)
{
  enable_c3_car_model((SFc3_car_modelInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_disable_c3_car_model(void *chartInstanceVar)
{
  disable_c3_car_model((SFc3_car_modelInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_gateway_c3_car_model(void *chartInstanceVar)
{
  sf_c3_car_model((SFc3_car_modelInstanceStruct*) chartInstanceVar);
}

static mxArray* sf_internal_get_sim_state_c3_car_model(SimStruct* S)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c3_car_model((SFc3_car_modelInstanceStruct*)
    chartInfo->chartInstance);         /* raw sim ctx */
  prhs[3] = sf_get_sim_state_info_c3_car_model();/* state var info */
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

static void sf_internal_set_sim_state_c3_car_model(SimStruct* S, const mxArray
  *st)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = mxDuplicateArray(st);      /* high level simctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c3_car_model();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c3_car_model((SFc3_car_modelInstanceStruct*)
    chartInfo->chartInstance, mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static mxArray* sf_opaque_get_sim_state_c3_car_model(SimStruct* S)
{
  return sf_internal_get_sim_state_c3_car_model(S);
}

static void sf_opaque_set_sim_state_c3_car_model(SimStruct* S, const mxArray *st)
{
  sf_internal_set_sim_state_c3_car_model(S, st);
}

static void sf_opaque_terminate_c3_car_model(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc3_car_modelInstanceStruct*) chartInstanceVar)->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
    }

    finalize_c3_car_model((SFc3_car_modelInstanceStruct*) chartInstanceVar);
    free((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c3_car_model(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c3_car_model((SFc3_car_modelInstanceStruct*)
      (((ChartInfoStruct *)ssGetUserData(S))->chartInstance));
  }
}

static void mdlSetWorkWidths_c3_car_model(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable("car_model","car_model",3);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop("car_model","car_model",3,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop("car_model","car_model",3,
      "gatewayCannotBeInlinedMultipleTimes"));
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,"car_model","car_model",3,2);
      sf_mark_chart_reusable_outputs(S,"car_model","car_model",3,1);
    }

    sf_set_rtw_dwork_info(S,"car_model","car_model",3);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
    ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  }

  ssSetChecksum0(S,(1243352367U));
  ssSetChecksum1(S,(2652406795U));
  ssSetChecksum2(S,(3769041910U));
  ssSetChecksum3(S,(1275700U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
}

static void mdlRTW_c3_car_model(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    sf_write_symbol_mapping(S, "car_model", "car_model",3);
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c3_car_model(SimStruct *S)
{
  SFc3_car_modelInstanceStruct *chartInstance;
  chartInstance = (SFc3_car_modelInstanceStruct *)malloc(sizeof
    (SFc3_car_modelInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc3_car_modelInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway = sf_opaque_gateway_c3_car_model;
  chartInstance->chartInfo.initializeChart = sf_opaque_initialize_c3_car_model;
  chartInstance->chartInfo.terminateChart = sf_opaque_terminate_c3_car_model;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c3_car_model;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c3_car_model;
  chartInstance->chartInfo.getSimState = sf_opaque_get_sim_state_c3_car_model;
  chartInstance->chartInfo.setSimState = sf_opaque_set_sim_state_c3_car_model;
  chartInstance->chartInfo.getSimStateInfo = sf_get_sim_state_info_c3_car_model;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c3_car_model;
  chartInstance->chartInfo.mdlStart = mdlStart_c3_car_model;
  chartInstance->chartInfo.mdlSetWorkWidths = mdlSetWorkWidths_c3_car_model;
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

void c3_car_model_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c3_car_model(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c3_car_model(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c3_car_model(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c3_car_model_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
