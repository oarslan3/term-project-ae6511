/* Include files */

#include "blascompat32.h"
#include "car_model_sfun.h"
#include "c2_car_model.h"
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "car_model_sfun_debug_macros.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */
static const char *c2_debug_family_names[26] = { "m", "Iz", "l_F", "l_R", "Vx",
  "Vy", "r", "psi", "f_Fx", "f_Rx", "delta", "f_Fy", "f_Fz", "f_Ry", "f_Rz",
  "dVx", "dVy", "dr", "dpsi", "nargin", "nargout", "X", "u", "f_F", "f_R", "dX"
};

/* Function Declarations */
static void initialize_c2_car_model(SFc2_car_modelInstanceStruct *chartInstance);
static void initialize_params_c2_car_model(SFc2_car_modelInstanceStruct
  *chartInstance);
static void enable_c2_car_model(SFc2_car_modelInstanceStruct *chartInstance);
static void disable_c2_car_model(SFc2_car_modelInstanceStruct *chartInstance);
static void c2_update_debugger_state_c2_car_model(SFc2_car_modelInstanceStruct
  *chartInstance);
static const mxArray *get_sim_state_c2_car_model(SFc2_car_modelInstanceStruct
  *chartInstance);
static void set_sim_state_c2_car_model(SFc2_car_modelInstanceStruct
  *chartInstance, const mxArray *c2_st);
static void finalize_c2_car_model(SFc2_car_modelInstanceStruct *chartInstance);
static void sf_c2_car_model(SFc2_car_modelInstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T c2_machineNumber, uint32_T
  c2_chartNumber);
static const mxArray *c2_sf_marshall(void *chartInstanceVoid, void *c2_u);
static const mxArray *c2_b_sf_marshall(void *chartInstanceVoid, void *c2_u);
static const mxArray *c2_c_sf_marshall(void *chartInstanceVoid, void *c2_u);
static const mxArray *c2_d_sf_marshall(void *chartInstanceVoid, void *c2_u);
static void c2_info_helper(c2_ResolvedFunctionInfo c2_info[23]);
static const mxArray *c2_e_sf_marshall(void *chartInstanceVoid, void *c2_u);
static void c2_emlrt_marshallIn(SFc2_car_modelInstanceStruct *chartInstance,
  const mxArray *c2_dX, const char_T *c2_name, real_T c2_y[4]);
static uint8_T c2_b_emlrt_marshallIn(SFc2_car_modelInstanceStruct *chartInstance,
  const mxArray *c2_b_is_active_c2_car_model, const char_T *c2_name);
static void init_dsm_address_info(SFc2_car_modelInstanceStruct *chartInstance);

/* Function Definitions */
static void initialize_c2_car_model(SFc2_car_modelInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  chartInstance->c2_is_active_c2_car_model = 0U;
}

static void initialize_params_c2_car_model(SFc2_car_modelInstanceStruct
  *chartInstance)
{
}

static void enable_c2_car_model(SFc2_car_modelInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void disable_c2_car_model(SFc2_car_modelInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void c2_update_debugger_state_c2_car_model(SFc2_car_modelInstanceStruct
  *chartInstance)
{
}

static const mxArray *get_sim_state_c2_car_model(SFc2_car_modelInstanceStruct
  *chartInstance)
{
  const mxArray *c2_st = NULL;
  const mxArray *c2_y = NULL;
  int32_T c2_i0;
  real_T c2_hoistedGlobal[4];
  int32_T c2_i1;
  real_T c2_u[4];
  const mxArray *c2_b_y = NULL;
  uint8_T c2_b_hoistedGlobal;
  uint8_T c2_b_u;
  const mxArray *c2_c_y = NULL;
  real_T (*c2_dX)[4];
  c2_dX = (real_T (*)[4])ssGetOutputPortSignal(chartInstance->S, 1);
  c2_st = NULL;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_createcellarray(2));
  for (c2_i0 = 0; c2_i0 < 4; c2_i0 = c2_i0 + 1) {
    c2_hoistedGlobal[c2_i0] = (*c2_dX)[c2_i0];
  }

  for (c2_i1 = 0; c2_i1 < 4; c2_i1 = c2_i1 + 1) {
    c2_u[c2_i1] = c2_hoistedGlobal[c2_i1];
  }

  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", &c2_u, 0, 0U, 1U, 0U, 1, 4));
  sf_mex_setcell(c2_y, 0, c2_b_y);
  c2_b_hoistedGlobal = chartInstance->c2_is_active_c2_car_model;
  c2_b_u = c2_b_hoistedGlobal;
  c2_c_y = NULL;
  sf_mex_assign(&c2_c_y, sf_mex_create("y", &c2_b_u, 3, 0U, 0U, 0U, 0));
  sf_mex_setcell(c2_y, 1, c2_c_y);
  sf_mex_assign(&c2_st, c2_y);
  return c2_st;
}

static void set_sim_state_c2_car_model(SFc2_car_modelInstanceStruct
  *chartInstance, const mxArray *c2_st)
{
  const mxArray *c2_u;
  real_T c2_dv0[4];
  int32_T c2_i2;
  real_T (*c2_dX)[4];
  c2_dX = (real_T (*)[4])ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c2_doneDoubleBufferReInit = TRUE;
  c2_u = sf_mex_dup(c2_st);
  c2_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 0)), "dX",
                      c2_dv0);
  for (c2_i2 = 0; c2_i2 < 4; c2_i2 = c2_i2 + 1) {
    (*c2_dX)[c2_i2] = c2_dv0[c2_i2];
  }

  chartInstance->c2_is_active_c2_car_model = c2_b_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getcell(c2_u, 1)),
    "is_active_c2_car_model");
  sf_mex_destroy(&c2_u);
  c2_update_debugger_state_c2_car_model(chartInstance);
  sf_mex_destroy(&c2_st);
}

static void finalize_c2_car_model(SFc2_car_modelInstanceStruct *chartInstance)
{
}

static void sf_c2_car_model(SFc2_car_modelInstanceStruct *chartInstance)
{
  int32_T c2_i3;
  int32_T c2_i4;
  int32_T c2_i5;
  int32_T c2_i6;
  int32_T c2_i7;
  int32_T c2_previousEvent;
  int32_T c2_i8;
  real_T c2_hoistedGlobal[4];
  int32_T c2_i9;
  real_T c2_b_hoistedGlobal[3];
  int32_T c2_i10;
  real_T c2_c_hoistedGlobal[2];
  int32_T c2_i11;
  real_T c2_d_hoistedGlobal[2];
  int32_T c2_i12;
  real_T c2_X[4];
  int32_T c2_i13;
  real_T c2_u[3];
  int32_T c2_i14;
  real_T c2_f_F[2];
  int32_T c2_i15;
  real_T c2_f_R[2];
  uint32_T c2_debug_family_var_map[26];
  real_T c2_m;
  real_T c2_Iz;
  real_T c2_l_F;
  real_T c2_l_R;
  real_T c2_Vx;
  real_T c2_Vy;
  real_T c2_r;
  real_T c2_psi;
  real_T c2_f_Fx;
  real_T c2_f_Rx;
  real_T c2_delta;
  real_T c2_f_Fy;
  real_T c2_f_Fz;
  real_T c2_f_Ry;
  real_T c2_f_Rz;
  real_T c2_dVx;
  real_T c2_dVy;
  real_T c2_dr;
  real_T c2_dpsi;
  real_T c2_nargin = 4.0;
  real_T c2_nargout = 1.0;
  real_T c2_dX[4];
  real_T c2_x;
  real_T c2_b_x;
  real_T c2_c_x;
  real_T c2_a;
  real_T c2_b;
  real_T c2_y;
  real_T c2_d_x;
  real_T c2_e_x;
  real_T c2_f_x;
  real_T c2_b_a;
  real_T c2_b_b;
  real_T c2_b_y;
  real_T c2_c_b;
  real_T c2_c_y;
  real_T c2_c_a;
  real_T c2_d_b;
  real_T c2_d_y;
  real_T c2_g_x;
  real_T c2_h_x;
  real_T c2_i_x;
  real_T c2_d_a;
  real_T c2_e_b;
  real_T c2_e_y;
  real_T c2_j_x;
  real_T c2_k_x;
  real_T c2_l_x;
  real_T c2_e_a;
  real_T c2_f_b;
  real_T c2_f_y;
  real_T c2_g_b;
  real_T c2_g_y;
  real_T c2_f_a;
  real_T c2_h_b;
  real_T c2_h_y;
  real_T c2_m_x;
  real_T c2_n_x;
  real_T c2_o_x;
  real_T c2_g_a;
  real_T c2_i_b;
  real_T c2_i_y;
  real_T c2_p_x;
  real_T c2_q_x;
  real_T c2_r_x;
  real_T c2_h_a;
  real_T c2_j_b;
  real_T c2_j_y;
  real_T c2_i_a;
  real_T c2_k_y;
  real_T c2_j_a;
  real_T c2_l_y;
  real_T c2_k_b;
  real_T c2_b_dVx[4];
  int32_T c2_i16;
  int32_T c2_i17;
  real_T (*c2_b_dX)[4];
  real_T (*c2_b_f_R)[2];
  real_T (*c2_b_f_F)[2];
  real_T (*c2_b_u)[3];
  real_T (*c2_b_X)[4];
  c2_b_dX = (real_T (*)[4])ssGetOutputPortSignal(chartInstance->S, 1);
  c2_b_f_R = (real_T (*)[2])ssGetInputPortSignal(chartInstance->S, 3);
  c2_b_f_F = (real_T (*)[2])ssGetInputPortSignal(chartInstance->S, 2);
  c2_b_u = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 1);
  c2_b_X = (real_T (*)[4])ssGetInputPortSignal(chartInstance->S, 0);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG,1);
  for (c2_i3 = 0; c2_i3 < 4; c2_i3 = c2_i3 + 1) {
    _SFD_DATA_RANGE_CHECK((*c2_b_X)[c2_i3], 0U);
  }

  for (c2_i4 = 0; c2_i4 < 3; c2_i4 = c2_i4 + 1) {
    _SFD_DATA_RANGE_CHECK((*c2_b_u)[c2_i4], 1U);
  }

  for (c2_i5 = 0; c2_i5 < 2; c2_i5 = c2_i5 + 1) {
    _SFD_DATA_RANGE_CHECK((*c2_b_f_F)[c2_i5], 2U);
  }

  for (c2_i6 = 0; c2_i6 < 2; c2_i6 = c2_i6 + 1) {
    _SFD_DATA_RANGE_CHECK((*c2_b_f_R)[c2_i6], 3U);
  }

  for (c2_i7 = 0; c2_i7 < 4; c2_i7 = c2_i7 + 1) {
    _SFD_DATA_RANGE_CHECK((*c2_b_dX)[c2_i7], 4U);
  }

  c2_previousEvent = _sfEvent_;
  _sfEvent_ = CALL_EVENT;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG,1);
  for (c2_i8 = 0; c2_i8 < 4; c2_i8 = c2_i8 + 1) {
    c2_hoistedGlobal[c2_i8] = (*c2_b_X)[c2_i8];
  }

  for (c2_i9 = 0; c2_i9 < 3; c2_i9 = c2_i9 + 1) {
    c2_b_hoistedGlobal[c2_i9] = (*c2_b_u)[c2_i9];
  }

  for (c2_i10 = 0; c2_i10 < 2; c2_i10 = c2_i10 + 1) {
    c2_c_hoistedGlobal[c2_i10] = (*c2_b_f_F)[c2_i10];
  }

  for (c2_i11 = 0; c2_i11 < 2; c2_i11 = c2_i11 + 1) {
    c2_d_hoistedGlobal[c2_i11] = (*c2_b_f_R)[c2_i11];
  }

  for (c2_i12 = 0; c2_i12 < 4; c2_i12 = c2_i12 + 1) {
    c2_X[c2_i12] = c2_hoistedGlobal[c2_i12];
  }

  for (c2_i13 = 0; c2_i13 < 3; c2_i13 = c2_i13 + 1) {
    c2_u[c2_i13] = c2_b_hoistedGlobal[c2_i13];
  }

  for (c2_i14 = 0; c2_i14 < 2; c2_i14 = c2_i14 + 1) {
    c2_f_F[c2_i14] = c2_c_hoistedGlobal[c2_i14];
  }

  for (c2_i15 = 0; c2_i15 < 2; c2_i15 = c2_i15 + 1) {
    c2_f_R[c2_i15] = c2_d_hoistedGlobal[c2_i15];
  }

  sf_debug_symbol_scope_push_eml(0U, 26U, 26U, c2_debug_family_names,
    c2_debug_family_var_map);
  sf_debug_symbol_scope_add_eml(&c2_m, c2_d_sf_marshall, 0U);
  sf_debug_symbol_scope_add_eml(&c2_Iz, c2_d_sf_marshall, 1U);
  sf_debug_symbol_scope_add_eml(&c2_l_F, c2_d_sf_marshall, 2U);
  sf_debug_symbol_scope_add_eml(&c2_l_R, c2_d_sf_marshall, 3U);
  sf_debug_symbol_scope_add_eml(&c2_Vx, c2_d_sf_marshall, 4U);
  sf_debug_symbol_scope_add_eml(&c2_Vy, c2_d_sf_marshall, 5U);
  sf_debug_symbol_scope_add_eml(&c2_r, c2_d_sf_marshall, 6U);
  sf_debug_symbol_scope_add_eml(&c2_psi, c2_d_sf_marshall, 7U);
  sf_debug_symbol_scope_add_eml(&c2_f_Fx, c2_d_sf_marshall, 8U);
  sf_debug_symbol_scope_add_eml(&c2_f_Rx, c2_d_sf_marshall, 9U);
  sf_debug_symbol_scope_add_eml(&c2_delta, c2_d_sf_marshall, 10U);
  sf_debug_symbol_scope_add_eml(&c2_f_Fy, c2_d_sf_marshall, 11U);
  sf_debug_symbol_scope_add_eml(&c2_f_Fz, c2_d_sf_marshall, 12U);
  sf_debug_symbol_scope_add_eml(&c2_f_Ry, c2_d_sf_marshall, 13U);
  sf_debug_symbol_scope_add_eml(&c2_f_Rz, c2_d_sf_marshall, 14U);
  sf_debug_symbol_scope_add_eml(&c2_dVx, c2_d_sf_marshall, 15U);
  sf_debug_symbol_scope_add_eml(&c2_dVy, c2_d_sf_marshall, 16U);
  sf_debug_symbol_scope_add_eml(&c2_dr, c2_d_sf_marshall, 17U);
  sf_debug_symbol_scope_add_eml(&c2_dpsi, c2_d_sf_marshall, 18U);
  sf_debug_symbol_scope_add_eml(&c2_nargin, c2_d_sf_marshall, 19U);
  sf_debug_symbol_scope_add_eml(&c2_nargout, c2_d_sf_marshall, 20U);
  sf_debug_symbol_scope_add_eml(&c2_X, c2_sf_marshall, 21U);
  sf_debug_symbol_scope_add_eml(&c2_u, c2_c_sf_marshall, 22U);
  sf_debug_symbol_scope_add_eml(&c2_f_F, c2_b_sf_marshall, 23U);
  sf_debug_symbol_scope_add_eml(&c2_f_R, c2_b_sf_marshall, 24U);
  sf_debug_symbol_scope_add_eml(&c2_dX, c2_sf_marshall, 25U);
  CV_EML_FCN(0, 0);

  /*  m = param.m; */
  /*  Iz = param.Iz; */
  /*  l_F = param.l_F; */
  /*  l_R = param.l_R; */
  /*  B = param.B; */
  /*  C = param.C; */
  /*  mu = param.mu; */
  /*  delta_max = param.delta_max; */
  _SFD_EML_CALL(0,13);
  c2_m = 1450.0;
  _SFD_EML_CALL(0,14);
  c2_Iz = 2740.0;
  _SFD_EML_CALL(0,15);
  c2_l_F = 1.1;
  _SFD_EML_CALL(0,16);
  c2_l_R = 1.6;
  _SFD_EML_CALL(0,18);
  c2_Vx = c2_X[0];
  _SFD_EML_CALL(0,19);
  c2_Vy = c2_X[1];
  _SFD_EML_CALL(0,20);
  c2_r = c2_X[2];
  _SFD_EML_CALL(0,21);
  c2_psi = c2_X[3];
  _SFD_EML_CALL(0,23);
  c2_f_Fx = c2_u[0];
  _SFD_EML_CALL(0,24);
  c2_f_Rx = c2_u[1];
  _SFD_EML_CALL(0,25);
  c2_delta = c2_u[2];
  _SFD_EML_CALL(0,27);
  c2_f_Fy = c2_f_F[0];
  _SFD_EML_CALL(0,28);
  c2_f_Fz = c2_f_F[1];
  _SFD_EML_CALL(0,30);
  c2_f_Ry = c2_f_R[0];
  _SFD_EML_CALL(0,31);
  c2_f_Rz = c2_f_R[1];
  _SFD_EML_CALL(0,34);
  c2_x = c2_delta;
  c2_b_x = c2_x;
  c2_c_x = c2_b_x;
  c2_b_x = c2_c_x;
  c2_b_x = muDoubleScalarCos(c2_b_x);
  c2_a = c2_f_Fx;
  c2_b = c2_b_x;
  c2_y = c2_a * c2_b;
  c2_d_x = c2_delta;
  c2_e_x = c2_d_x;
  c2_f_x = c2_e_x;
  c2_e_x = c2_f_x;
  c2_e_x = muDoubleScalarSin(c2_e_x);
  c2_b_a = c2_f_Fy;
  c2_b_b = c2_e_x;
  c2_b_y = c2_b_a * c2_b_b;
  c2_c_b = (c2_y - c2_b_y) + c2_f_Rx;
  c2_c_y = 6.8965517241379305E-004 * c2_c_b;
  c2_c_a = c2_Vy;
  c2_d_b = c2_r;
  c2_d_y = c2_c_a * c2_d_b;
  c2_dVx = c2_c_y + c2_d_y;
  _SFD_EML_CALL(0,35);
  c2_g_x = c2_delta;
  c2_h_x = c2_g_x;
  c2_i_x = c2_h_x;
  c2_h_x = c2_i_x;
  c2_h_x = muDoubleScalarSin(c2_h_x);
  c2_d_a = c2_f_Fx;
  c2_e_b = c2_h_x;
  c2_e_y = c2_d_a * c2_e_b;
  c2_j_x = c2_delta;
  c2_k_x = c2_j_x;
  c2_l_x = c2_k_x;
  c2_k_x = c2_l_x;
  c2_k_x = muDoubleScalarCos(c2_k_x);
  c2_e_a = c2_f_Fy;
  c2_f_b = c2_k_x;
  c2_f_y = c2_e_a * c2_f_b;
  c2_g_b = (c2_e_y + c2_f_y) + c2_f_Ry;
  c2_g_y = 6.8965517241379305E-004 * c2_g_b;
  c2_f_a = c2_Vx;
  c2_h_b = c2_r;
  c2_h_y = c2_f_a * c2_h_b;
  c2_dVy = c2_g_y - c2_h_y;
  _SFD_EML_CALL(0,36);
  c2_m_x = c2_delta;
  c2_n_x = c2_m_x;
  c2_o_x = c2_n_x;
  c2_n_x = c2_o_x;
  c2_n_x = muDoubleScalarCos(c2_n_x);
  c2_g_a = c2_f_Fy;
  c2_i_b = c2_n_x;
  c2_i_y = c2_g_a * c2_i_b;
  c2_p_x = c2_delta;
  c2_q_x = c2_p_x;
  c2_r_x = c2_q_x;
  c2_q_x = c2_r_x;
  c2_q_x = muDoubleScalarSin(c2_q_x);
  c2_h_a = c2_f_Fx;
  c2_j_b = c2_q_x;
  c2_j_y = c2_h_a * c2_j_b;
  c2_i_a = c2_i_y + c2_j_y;
  c2_k_y = c2_i_a * 1.1;
  c2_j_a = c2_f_Ry;
  c2_l_y = c2_j_a * 1.6;
  c2_k_b = c2_k_y - c2_l_y;
  c2_dr = 3.6496350364963501E-004 * c2_k_b;
  _SFD_EML_CALL(0,37);
  c2_dpsi = c2_r;
  _SFD_EML_CALL(0,39);
  c2_b_dVx[0] = c2_dVx;
  c2_b_dVx[1] = c2_dVy;
  c2_b_dVx[2] = c2_dr;
  c2_b_dVx[3] = c2_dpsi;
  for (c2_i16 = 0; c2_i16 < 4; c2_i16 = c2_i16 + 1) {
    c2_dX[c2_i16] = c2_b_dVx[c2_i16];
  }

  _SFD_EML_CALL(0,-39);
  sf_debug_symbol_scope_pop();
  for (c2_i17 = 0; c2_i17 < 4; c2_i17 = c2_i17 + 1) {
    (*c2_b_dX)[c2_i17] = c2_dX[c2_i17];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG,1);
  _sfEvent_ = c2_previousEvent;
  sf_debug_check_for_state_inconsistency(_car_modelMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void init_script_number_translation(uint32_T c2_machineNumber, uint32_T
  c2_chartNumber)
{
}

static const mxArray *c2_sf_marshall(void *chartInstanceVoid, void *c2_u)
{
  const mxArray *c2_y = NULL;
  int32_T c2_i18;
  real_T c2_b_u[4];
  const mxArray *c2_b_y = NULL;
  SFc2_car_modelInstanceStruct *chartInstance;
  chartInstance = (SFc2_car_modelInstanceStruct *)chartInstanceVoid;
  c2_y = NULL;
  for (c2_i18 = 0; c2_i18 < 4; c2_i18 = c2_i18 + 1) {
    c2_b_u[c2_i18] = (*((real_T (*)[4])c2_u))[c2_i18];
  }

  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", &c2_b_u, 0, 0U, 1U, 0U, 1, 4));
  sf_mex_assign(&c2_y, c2_b_y);
  return c2_y;
}

static const mxArray *c2_b_sf_marshall(void *chartInstanceVoid, void *c2_u)
{
  const mxArray *c2_y = NULL;
  int32_T c2_i19;
  real_T c2_b_u[2];
  const mxArray *c2_b_y = NULL;
  SFc2_car_modelInstanceStruct *chartInstance;
  chartInstance = (SFc2_car_modelInstanceStruct *)chartInstanceVoid;
  c2_y = NULL;
  for (c2_i19 = 0; c2_i19 < 2; c2_i19 = c2_i19 + 1) {
    c2_b_u[c2_i19] = (*((real_T (*)[2])c2_u))[c2_i19];
  }

  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", &c2_b_u, 0, 0U, 1U, 0U, 1, 2));
  sf_mex_assign(&c2_y, c2_b_y);
  return c2_y;
}

static const mxArray *c2_c_sf_marshall(void *chartInstanceVoid, void *c2_u)
{
  const mxArray *c2_y = NULL;
  int32_T c2_i20;
  real_T c2_b_u[3];
  const mxArray *c2_b_y = NULL;
  SFc2_car_modelInstanceStruct *chartInstance;
  chartInstance = (SFc2_car_modelInstanceStruct *)chartInstanceVoid;
  c2_y = NULL;
  for (c2_i20 = 0; c2_i20 < 3; c2_i20 = c2_i20 + 1) {
    c2_b_u[c2_i20] = (*((real_T (*)[3])c2_u))[c2_i20];
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
  SFc2_car_modelInstanceStruct *chartInstance;
  chartInstance = (SFc2_car_modelInstanceStruct *)chartInstanceVoid;
  c2_y = NULL;
  c2_b_u = *((real_T *)c2_u);
  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", &c2_b_u, 0, 0U, 0U, 0U, 0));
  sf_mex_assign(&c2_y, c2_b_y);
  return c2_y;
}

const mxArray *sf_c2_car_model_get_eml_resolved_functions_info(void)
{
  const mxArray *c2_nameCaptureInfo = NULL;
  c2_ResolvedFunctionInfo c2_info[23];
  const mxArray *c2_m0 = NULL;
  int32_T c2_i21;
  c2_ResolvedFunctionInfo *c2_r0;
  c2_nameCaptureInfo = NULL;
  c2_info_helper(c2_info);
  sf_mex_assign(&c2_m0, sf_mex_createstruct("nameCaptureInfo", 1, 23));
  for (c2_i21 = 0; c2_i21 < 23; c2_i21 = c2_i21 + 1) {
    c2_r0 = &c2_info[c2_i21];
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", c2_r0->context, 15,
      0U, 0U, 0U, 2, 1, strlen(c2_r0->context)), "context",
                    "nameCaptureInfo", c2_i21);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", c2_r0->name, 15, 0U,
      0U, 0U, 2, 1, strlen(c2_r0->name)), "name",
                    "nameCaptureInfo", c2_i21);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", c2_r0->dominantType,
      15, 0U, 0U, 0U, 2, 1, strlen(c2_r0->dominantType)),
                    "dominantType", "nameCaptureInfo", c2_i21);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", c2_r0->resolved, 15,
      0U, 0U, 0U, 2, 1, strlen(c2_r0->resolved)), "resolved"
                    , "nameCaptureInfo", c2_i21);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", &c2_r0->fileLength,
      7, 0U, 0U, 0U, 0), "fileLength", "nameCaptureInfo",
                    c2_i21);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", &c2_r0->fileTime1, 7,
      0U, 0U, 0U, 0), "fileTime1", "nameCaptureInfo", c2_i21
                    );
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", &c2_r0->fileTime2, 7,
      0U, 0U, 0U, 0), "fileTime2", "nameCaptureInfo", c2_i21
                    );
  }

  sf_mex_assign(&c2_nameCaptureInfo, c2_m0);
  return c2_nameCaptureInfo;
}

static void c2_info_helper(c2_ResolvedFunctionInfo c2_info[23])
{
  c2_info[0].context = "";
  c2_info[0].name = "mrdivide";
  c2_info[0].dominantType = "double";
  c2_info[0].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.m";
  c2_info[0].fileLength = 800U;
  c2_info[0].fileTime1 = 1238455890U;
  c2_info[0].fileTime2 = 0U;
  c2_info[1].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.m";
  c2_info[1].name = "nargin";
  c2_info[1].dominantType = "";
  c2_info[1].resolved = "[B]nargin";
  c2_info[1].fileLength = 0U;
  c2_info[1].fileTime1 = 0U;
  c2_info[1].fileTime2 = 0U;
  c2_info[2].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.m";
  c2_info[2].name = "ge";
  c2_info[2].dominantType = "double";
  c2_info[2].resolved = "[B]ge";
  c2_info[2].fileLength = 0U;
  c2_info[2].fileTime1 = 0U;
  c2_info[2].fileTime2 = 0U;
  c2_info[3].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.m";
  c2_info[3].name = "isscalar";
  c2_info[3].dominantType = "double";
  c2_info[3].resolved = "[B]isscalar";
  c2_info[3].fileLength = 0U;
  c2_info[3].fileTime1 = 0U;
  c2_info[3].fileTime2 = 0U;
  c2_info[4].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.m";
  c2_info[4].name = "rdivide";
  c2_info[4].dominantType = "double";
  c2_info[4].resolved = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c2_info[4].fileLength = 403U;
  c2_info[4].fileTime1 = 1244757152U;
  c2_info[4].fileTime2 = 0U;
  c2_info[5].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c2_info[5].name = "gt";
  c2_info[5].dominantType = "double";
  c2_info[5].resolved = "[B]gt";
  c2_info[5].fileLength = 0U;
  c2_info[5].fileTime1 = 0U;
  c2_info[5].fileTime2 = 0U;
  c2_info[6].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c2_info[6].name = "isa";
  c2_info[6].dominantType = "double";
  c2_info[6].resolved = "[B]isa";
  c2_info[6].fileLength = 0U;
  c2_info[6].fileTime1 = 0U;
  c2_info[6].fileTime2 = 0U;
  c2_info[7].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c2_info[7].name = "eml_div";
  c2_info[7].dominantType = "double";
  c2_info[7].resolved = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c2_info[7].fileLength = 4269U;
  c2_info[7].fileTime1 = 1228115426U;
  c2_info[7].fileTime2 = 0U;
  c2_info[8].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c2_info[8].name = "isinteger";
  c2_info[8].dominantType = "double";
  c2_info[8].resolved = "[B]isinteger";
  c2_info[8].fileLength = 0U;
  c2_info[8].fileTime1 = 0U;
  c2_info[8].fileTime2 = 0U;
  c2_info[9].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m/eml_fldiv";
  c2_info[9].name = "isreal";
  c2_info[9].dominantType = "double";
  c2_info[9].resolved = "[B]isreal";
  c2_info[9].fileLength = 0U;
  c2_info[9].fileTime1 = 0U;
  c2_info[9].fileTime2 = 0U;
  c2_info[10].context = "";
  c2_info[10].name = "cos";
  c2_info[10].dominantType = "double";
  c2_info[10].resolved = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/cos.m";
  c2_info[10].fileLength = 324U;
  c2_info[10].fileTime1 = 1203469550U;
  c2_info[10].fileTime2 = 0U;
  c2_info[11].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/cos.m";
  c2_info[11].name = "eml_scalar_cos";
  c2_info[11].dominantType = "double";
  c2_info[11].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_cos.m";
  c2_info[11].fileLength = 602U;
  c2_info[11].fileTime1 = 1209352386U;
  c2_info[11].fileTime2 = 0U;
  c2_info[12].context = "";
  c2_info[12].name = "mtimes";
  c2_info[12].dominantType = "double";
  c2_info[12].resolved = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[12].fileLength = 3425U;
  c2_info[12].fileTime1 = 1250694366U;
  c2_info[12].fileTime2 = 0U;
  c2_info[13].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[13].name = "strcmp";
  c2_info[13].dominantType = "char";
  c2_info[13].resolved = "[B]strcmp";
  c2_info[13].fileLength = 0U;
  c2_info[13].fileTime1 = 0U;
  c2_info[13].fileTime2 = 0U;
  c2_info[14].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[14].name = "size";
  c2_info[14].dominantType = "double";
  c2_info[14].resolved = "[B]size";
  c2_info[14].fileLength = 0U;
  c2_info[14].fileTime1 = 0U;
  c2_info[14].fileTime2 = 0U;
  c2_info[15].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[15].name = "eq";
  c2_info[15].dominantType = "double";
  c2_info[15].resolved = "[B]eq";
  c2_info[15].fileLength = 0U;
  c2_info[15].fileTime1 = 0U;
  c2_info[15].fileTime2 = 0U;
  c2_info[16].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[16].name = "class";
  c2_info[16].dominantType = "double";
  c2_info[16].resolved = "[B]class";
  c2_info[16].fileLength = 0U;
  c2_info[16].fileTime1 = 0U;
  c2_info[16].fileTime2 = 0U;
  c2_info[17].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[17].name = "not";
  c2_info[17].dominantType = "logical";
  c2_info[17].resolved = "[B]not";
  c2_info[17].fileLength = 0U;
  c2_info[17].fileTime1 = 0U;
  c2_info[17].fileTime2 = 0U;
  c2_info[18].context = "";
  c2_info[18].name = "sin";
  c2_info[18].dominantType = "double";
  c2_info[18].resolved = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/sin.m";
  c2_info[18].fileLength = 324U;
  c2_info[18].fileTime1 = 1203469642U;
  c2_info[18].fileTime2 = 0U;
  c2_info[19].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/sin.m";
  c2_info[19].name = "eml_scalar_sin";
  c2_info[19].dominantType = "double";
  c2_info[19].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_sin.m";
  c2_info[19].fileLength = 601U;
  c2_info[19].fileTime1 = 1209352390U;
  c2_info[19].fileTime2 = 0U;
  c2_info[20].context = "";
  c2_info[20].name = "minus";
  c2_info[20].dominantType = "double";
  c2_info[20].resolved = "[B]minus";
  c2_info[20].fileLength = 0U;
  c2_info[20].fileTime1 = 0U;
  c2_info[20].fileTime2 = 0U;
  c2_info[21].context = "";
  c2_info[21].name = "plus";
  c2_info[21].dominantType = "double";
  c2_info[21].resolved = "[B]plus";
  c2_info[21].fileLength = 0U;
  c2_info[21].fileTime1 = 0U;
  c2_info[21].fileTime2 = 0U;
  c2_info[22].context = "";
  c2_info[22].name = "ctranspose";
  c2_info[22].dominantType = "double";
  c2_info[22].resolved = "[B]ctranspose";
  c2_info[22].fileLength = 0U;
  c2_info[22].fileTime1 = 0U;
  c2_info[22].fileTime2 = 0U;
}

static const mxArray *c2_e_sf_marshall(void *chartInstanceVoid, void *c2_u)
{
  const mxArray *c2_y = NULL;
  boolean_T c2_b_u;
  const mxArray *c2_b_y = NULL;
  SFc2_car_modelInstanceStruct *chartInstance;
  chartInstance = (SFc2_car_modelInstanceStruct *)chartInstanceVoid;
  c2_y = NULL;
  c2_b_u = *((boolean_T *)c2_u);
  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", &c2_b_u, 11, 0U, 0U, 0U, 0));
  sf_mex_assign(&c2_y, c2_b_y);
  return c2_y;
}

static void c2_emlrt_marshallIn(SFc2_car_modelInstanceStruct *chartInstance,
  const mxArray *c2_dX, const char_T *c2_name, real_T
  c2_y[4])
{
  real_T c2_dv1[4];
  int32_T c2_i22;
  sf_mex_import(c2_name, sf_mex_dup(c2_dX), &c2_dv1, 1, 0, 0U, 1, 0U, 1, 4);
  for (c2_i22 = 0; c2_i22 < 4; c2_i22 = c2_i22 + 1) {
    c2_y[c2_i22] = c2_dv1[c2_i22];
  }

  sf_mex_destroy(&c2_dX);
}

static uint8_T c2_b_emlrt_marshallIn(SFc2_car_modelInstanceStruct *chartInstance,
  const mxArray *c2_b_is_active_c2_car_model, const
  char_T *c2_name)
{
  uint8_T c2_y;
  uint8_T c2_u0;
  sf_mex_import(c2_name, sf_mex_dup(c2_b_is_active_c2_car_model), &c2_u0, 1, 3,
                0U, 0, 0U, 0);
  c2_y = c2_u0;
  sf_mex_destroy(&c2_b_is_active_c2_car_model);
  return c2_y;
}

static void init_dsm_address_info(SFc2_car_modelInstanceStruct *chartInstance)
{
}

/* SFunction Glue Code */
void sf_c2_car_model_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(466414238U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(145437001U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3231199459U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(1825690484U);
}

mxArray *sf_c2_car_model_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,4,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateDoubleMatrix(4,1,mxREAL);
    double *pr = mxGetPr(mxChecksum);
    pr[0] = (double)(2049816183U);
    pr[1] = (double)(1355668588U);
    pr[2] = (double)(1372235997U);
    pr[3] = (double)(1468908850U);
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,4,3,dataFields);

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

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(2);
      pr[1] = (double)(1);
      mxSetField(mxData,3,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,3,"type",mxType);
    }

    mxSetField(mxData,3,"complexity",mxCreateDoubleScalar(0));
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
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  return(mxAutoinheritanceInfo);
}

static mxArray *sf_get_sim_state_info_c2_car_model(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x2'type','srcId','name','auxInfo'{{M[1],M[5],T\"dX\",},{M[8],M[0],T\"is_active_c2_car_model\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 2, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c2_car_model_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc2_car_modelInstanceStruct *chartInstance;
    chartInstance = (SFc2_car_modelInstanceStruct *) ((ChartInfoStruct *)
      (ssGetUserData(S)))->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart(_car_modelMachineNumber_,
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
                                1.0,0,"X",0,(MexFcnForType)c2_sf_marshall);
          }

          {
            unsigned int dimVector[1];
            dimVector[0]= 3;
            _SFD_SET_DATA_PROPS(1,1,1,0,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
                                1.0,0,"u",0,(MexFcnForType)c2_c_sf_marshall);
          }

          {
            unsigned int dimVector[1];
            dimVector[0]= 2;
            _SFD_SET_DATA_PROPS(2,1,1,0,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
                                1.0,0,"f_F",0,(MexFcnForType)c2_b_sf_marshall);
          }

          {
            unsigned int dimVector[1];
            dimVector[0]= 2;
            _SFD_SET_DATA_PROPS(3,1,1,0,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
                                1.0,0,"f_R",0,(MexFcnForType)c2_b_sf_marshall);
          }

          {
            unsigned int dimVector[1];
            dimVector[0]= 4;
            _SFD_SET_DATA_PROPS(4,2,0,1,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
                                1.0,0,"dX",0,(MexFcnForType)c2_sf_marshall);
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
        _SFD_CV_INIT_EML(0,1,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,616);
        _SFD_TRANS_COV_WTS(0,0,0,1,0);
        if (chartAlreadyPresent==0) {
          _SFD_TRANS_COV_MAPS(0,
                              0,NULL,NULL,
                              0,NULL,NULL,
                              1,NULL,NULL,
                              0,NULL,NULL);
        }

        {
          real_T (*c2_X)[4];
          real_T (*c2_u)[3];
          real_T (*c2_f_F)[2];
          real_T (*c2_f_R)[2];
          real_T (*c2_dX)[4];
          c2_dX = (real_T (*)[4])ssGetOutputPortSignal(chartInstance->S, 1);
          c2_f_R = (real_T (*)[2])ssGetInputPortSignal(chartInstance->S, 3);
          c2_f_F = (real_T (*)[2])ssGetInputPortSignal(chartInstance->S, 2);
          c2_u = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 1);
          c2_X = (real_T (*)[4])ssGetInputPortSignal(chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, c2_X);
          _SFD_SET_DATA_VALUE_PTR(1U, c2_u);
          _SFD_SET_DATA_VALUE_PTR(2U, c2_f_F);
          _SFD_SET_DATA_VALUE_PTR(3U, c2_f_R);
          _SFD_SET_DATA_VALUE_PTR(4U, c2_dX);
        }
      }
    } else {
      sf_debug_reset_current_state_configuration(_car_modelMachineNumber_,
        chartInstance->chartNumber,chartInstance->instanceNumber);
    }
  }
}

static void sf_opaque_initialize_c2_car_model(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc2_car_modelInstanceStruct*) chartInstanceVar
    )->S,0);
  initialize_params_c2_car_model((SFc2_car_modelInstanceStruct*)
    chartInstanceVar);
  initialize_c2_car_model((SFc2_car_modelInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_enable_c2_car_model(void *chartInstanceVar)
{
  enable_c2_car_model((SFc2_car_modelInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_disable_c2_car_model(void *chartInstanceVar)
{
  disable_c2_car_model((SFc2_car_modelInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_gateway_c2_car_model(void *chartInstanceVar)
{
  sf_c2_car_model((SFc2_car_modelInstanceStruct*) chartInstanceVar);
}

static mxArray* sf_internal_get_sim_state_c2_car_model(SimStruct* S)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c2_car_model((SFc2_car_modelInstanceStruct*)
    chartInfo->chartInstance);         /* raw sim ctx */
  prhs[3] = sf_get_sim_state_info_c2_car_model();/* state var info */
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

static void sf_internal_set_sim_state_c2_car_model(SimStruct* S, const mxArray
  *st)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = mxDuplicateArray(st);      /* high level simctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c2_car_model();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c2_car_model((SFc2_car_modelInstanceStruct*)
    chartInfo->chartInstance, mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static mxArray* sf_opaque_get_sim_state_c2_car_model(SimStruct* S)
{
  return sf_internal_get_sim_state_c2_car_model(S);
}

static void sf_opaque_set_sim_state_c2_car_model(SimStruct* S, const mxArray *st)
{
  sf_internal_set_sim_state_c2_car_model(S, st);
}

static void sf_opaque_terminate_c2_car_model(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc2_car_modelInstanceStruct*) chartInstanceVar)->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
    }

    finalize_c2_car_model((SFc2_car_modelInstanceStruct*) chartInstanceVar);
    free((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c2_car_model(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c2_car_model((SFc2_car_modelInstanceStruct*)
      (((ChartInfoStruct *)ssGetUserData(S))->chartInstance));
  }
}

static void mdlSetWorkWidths_c2_car_model(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable("car_model","car_model",2);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop("car_model","car_model",2,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop("car_model","car_model",2,
      "gatewayCannotBeInlinedMultipleTimes"));
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,"car_model","car_model",2,4);
      sf_mark_chart_reusable_outputs(S,"car_model","car_model",2,1);
    }

    sf_set_rtw_dwork_info(S,"car_model","car_model",2);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
    ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  }

  ssSetChecksum0(S,(1114507231U));
  ssSetChecksum1(S,(3829893209U));
  ssSetChecksum2(S,(1743350458U));
  ssSetChecksum3(S,(1542285703U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
}

static void mdlRTW_c2_car_model(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    sf_write_symbol_mapping(S, "car_model", "car_model",2);
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c2_car_model(SimStruct *S)
{
  SFc2_car_modelInstanceStruct *chartInstance;
  chartInstance = (SFc2_car_modelInstanceStruct *)malloc(sizeof
    (SFc2_car_modelInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc2_car_modelInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway = sf_opaque_gateway_c2_car_model;
  chartInstance->chartInfo.initializeChart = sf_opaque_initialize_c2_car_model;
  chartInstance->chartInfo.terminateChart = sf_opaque_terminate_c2_car_model;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c2_car_model;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c2_car_model;
  chartInstance->chartInfo.getSimState = sf_opaque_get_sim_state_c2_car_model;
  chartInstance->chartInfo.setSimState = sf_opaque_set_sim_state_c2_car_model;
  chartInstance->chartInfo.getSimStateInfo = sf_get_sim_state_info_c2_car_model;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c2_car_model;
  chartInstance->chartInfo.mdlStart = mdlStart_c2_car_model;
  chartInstance->chartInfo.mdlSetWorkWidths = mdlSetWorkWidths_c2_car_model;
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

void c2_car_model_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c2_car_model(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c2_car_model(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c2_car_model(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c2_car_model_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
