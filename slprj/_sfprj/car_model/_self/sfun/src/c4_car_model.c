/* Include files */

#include "blascompat32.h"
#include "car_model_sfun.h"
#include "c4_car_model.h"
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "car_model_sfun_debug_macros.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */
static const char *c4_debug_family_names[35] = { "m", "l_F", "l_R", "mu", "C",
  "B", "g", "Vx", "Vy", "r", "psi", "f_Fx", "f_Rx", "delta", "f_Fy", "f_Fz",
  "f_Ry", "f_Rz", "s_Fy", "s_Ry", "f_Fz_out", "f_Rz_out", "f_Fy_max", "f_Ry_max",
  "f_Fy_out", "f_Ry_out", "nargin", "nargout", "X", "u", "f_F", "f_R", "s",
  "f_F_out", "f_R_out" };

/* Function Declarations */
static void initialize_c4_car_model(SFc4_car_modelInstanceStruct *chartInstance);
static void initialize_params_c4_car_model(SFc4_car_modelInstanceStruct
  *chartInstance);
static void enable_c4_car_model(SFc4_car_modelInstanceStruct *chartInstance);
static void disable_c4_car_model(SFc4_car_modelInstanceStruct *chartInstance);
static void c4_update_debugger_state_c4_car_model(SFc4_car_modelInstanceStruct
  *chartInstance);
static const mxArray *get_sim_state_c4_car_model(SFc4_car_modelInstanceStruct
  *chartInstance);
static void set_sim_state_c4_car_model(SFc4_car_modelInstanceStruct
  *chartInstance, const mxArray *c4_st);
static void finalize_c4_car_model(SFc4_car_modelInstanceStruct *chartInstance);
static void sf_c4_car_model(SFc4_car_modelInstanceStruct *chartInstance);
static void c4_c4_car_model(SFc4_car_modelInstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T c4_machineNumber, uint32_T
  c4_chartNumber);
static real_T c4_mpower(SFc4_car_modelInstanceStruct *chartInstance, real_T c4_a);
static void c4_eml_error(SFc4_car_modelInstanceStruct *chartInstance);
static const mxArray *c4_sf_marshall(void *chartInstanceVoid, void *c4_u);
static const mxArray *c4_b_sf_marshall(void *chartInstanceVoid, void *c4_u);
static const mxArray *c4_c_sf_marshall(void *chartInstanceVoid, void *c4_u);
static const mxArray *c4_d_sf_marshall(void *chartInstanceVoid, void *c4_u);
static void c4_info_helper(c4_ResolvedFunctionInfo c4_info[33]);
static const mxArray *c4_e_sf_marshall(void *chartInstanceVoid, void *c4_u);
static void c4_emlrt_marshallIn(SFc4_car_modelInstanceStruct *chartInstance,
  const mxArray *c4_f_F_out, const char_T *c4_name, real_T c4_y[2]);
static uint8_T c4_b_emlrt_marshallIn(SFc4_car_modelInstanceStruct *chartInstance,
  const mxArray *c4_b_is_active_c4_car_model, const char_T *c4_name);
static void init_dsm_address_info(SFc4_car_modelInstanceStruct *chartInstance);

/* Function Definitions */
static void initialize_c4_car_model(SFc4_car_modelInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  chartInstance->c4_is_active_c4_car_model = 0U;
}

static void initialize_params_c4_car_model(SFc4_car_modelInstanceStruct
  *chartInstance)
{
}

static void enable_c4_car_model(SFc4_car_modelInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void disable_c4_car_model(SFc4_car_modelInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void c4_update_debugger_state_c4_car_model(SFc4_car_modelInstanceStruct
  *chartInstance)
{
}

static const mxArray *get_sim_state_c4_car_model(SFc4_car_modelInstanceStruct
  *chartInstance)
{
  const mxArray *c4_st = NULL;
  const mxArray *c4_y = NULL;
  int32_T c4_i0;
  real_T c4_hoistedGlobal[2];
  int32_T c4_i1;
  real_T c4_u[2];
  const mxArray *c4_b_y = NULL;
  int32_T c4_i2;
  real_T c4_b_hoistedGlobal[2];
  int32_T c4_i3;
  real_T c4_b_u[2];
  const mxArray *c4_c_y = NULL;
  uint8_T c4_c_hoistedGlobal;
  uint8_T c4_c_u;
  const mxArray *c4_d_y = NULL;
  real_T (*c4_f_R_out)[2];
  real_T (*c4_f_F_out)[2];
  c4_f_R_out = (real_T (*)[2])ssGetOutputPortSignal(chartInstance->S, 2);
  c4_f_F_out = (real_T (*)[2])ssGetOutputPortSignal(chartInstance->S, 1);
  c4_st = NULL;
  c4_y = NULL;
  sf_mex_assign(&c4_y, sf_mex_createcellarray(3));
  for (c4_i0 = 0; c4_i0 < 2; c4_i0 = c4_i0 + 1) {
    c4_hoistedGlobal[c4_i0] = (*c4_f_F_out)[c4_i0];
  }

  for (c4_i1 = 0; c4_i1 < 2; c4_i1 = c4_i1 + 1) {
    c4_u[c4_i1] = c4_hoistedGlobal[c4_i1];
  }

  c4_b_y = NULL;
  sf_mex_assign(&c4_b_y, sf_mex_create("y", &c4_u, 0, 0U, 1U, 0U, 1, 2));
  sf_mex_setcell(c4_y, 0, c4_b_y);
  for (c4_i2 = 0; c4_i2 < 2; c4_i2 = c4_i2 + 1) {
    c4_b_hoistedGlobal[c4_i2] = (*c4_f_R_out)[c4_i2];
  }

  for (c4_i3 = 0; c4_i3 < 2; c4_i3 = c4_i3 + 1) {
    c4_b_u[c4_i3] = c4_b_hoistedGlobal[c4_i3];
  }

  c4_c_y = NULL;
  sf_mex_assign(&c4_c_y, sf_mex_create("y", &c4_b_u, 0, 0U, 1U, 0U, 1, 2));
  sf_mex_setcell(c4_y, 1, c4_c_y);
  c4_c_hoistedGlobal = chartInstance->c4_is_active_c4_car_model;
  c4_c_u = c4_c_hoistedGlobal;
  c4_d_y = NULL;
  sf_mex_assign(&c4_d_y, sf_mex_create("y", &c4_c_u, 3, 0U, 0U, 0U, 0));
  sf_mex_setcell(c4_y, 2, c4_d_y);
  sf_mex_assign(&c4_st, c4_y);
  return c4_st;
}

static void set_sim_state_c4_car_model(SFc4_car_modelInstanceStruct
  *chartInstance, const mxArray *c4_st)
{
  const mxArray *c4_u;
  real_T c4_dv0[2];
  int32_T c4_i4;
  real_T c4_dv1[2];
  int32_T c4_i5;
  real_T (*c4_f_F_out)[2];
  real_T (*c4_f_R_out)[2];
  c4_f_R_out = (real_T (*)[2])ssGetOutputPortSignal(chartInstance->S, 2);
  c4_f_F_out = (real_T (*)[2])ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c4_doneDoubleBufferReInit = TRUE;
  c4_u = sf_mex_dup(c4_st);
  c4_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c4_u, 0)),
                      "f_F_out", c4_dv0);
  for (c4_i4 = 0; c4_i4 < 2; c4_i4 = c4_i4 + 1) {
    (*c4_f_F_out)[c4_i4] = c4_dv0[c4_i4];
  }

  c4_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c4_u, 1)),
                      "f_R_out", c4_dv1);
  for (c4_i5 = 0; c4_i5 < 2; c4_i5 = c4_i5 + 1) {
    (*c4_f_R_out)[c4_i5] = c4_dv1[c4_i5];
  }

  chartInstance->c4_is_active_c4_car_model = c4_b_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getcell(c4_u, 2)),
    "is_active_c4_car_model");
  sf_mex_destroy(&c4_u);
  c4_update_debugger_state_c4_car_model(chartInstance);
  sf_mex_destroy(&c4_st);
}

static void finalize_c4_car_model(SFc4_car_modelInstanceStruct *chartInstance)
{
}

static void sf_c4_car_model(SFc4_car_modelInstanceStruct *chartInstance)
{
  int32_T c4_i6;
  int32_T c4_i7;
  int32_T c4_i8;
  int32_T c4_i9;
  int32_T c4_i10;
  int32_T c4_i11;
  int32_T c4_i12;
  int32_T c4_previousEvent;
  real_T (*c4_s)[2];
  real_T (*c4_f_R)[2];
  real_T (*c4_f_F)[2];
  real_T (*c4_u)[3];
  real_T (*c4_X)[4];
  real_T (*c4_f_R_out)[2];
  real_T (*c4_f_F_out)[2];
  c4_s = (real_T (*)[2])ssGetInputPortSignal(chartInstance->S, 4);
  c4_f_R = (real_T (*)[2])ssGetInputPortSignal(chartInstance->S, 3);
  c4_f_F = (real_T (*)[2])ssGetInputPortSignal(chartInstance->S, 2);
  c4_u = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 1);
  c4_X = (real_T (*)[4])ssGetInputPortSignal(chartInstance->S, 0);
  c4_f_R_out = (real_T (*)[2])ssGetOutputPortSignal(chartInstance->S, 2);
  c4_f_F_out = (real_T (*)[2])ssGetOutputPortSignal(chartInstance->S, 1);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG,3);
  for (c4_i6 = 0; c4_i6 < 2; c4_i6 = c4_i6 + 1) {
    _SFD_DATA_RANGE_CHECK((*c4_f_F_out)[c4_i6], 0U);
  }

  for (c4_i7 = 0; c4_i7 < 2; c4_i7 = c4_i7 + 1) {
    _SFD_DATA_RANGE_CHECK((*c4_f_R_out)[c4_i7], 1U);
  }

  for (c4_i8 = 0; c4_i8 < 4; c4_i8 = c4_i8 + 1) {
    _SFD_DATA_RANGE_CHECK((*c4_X)[c4_i8], 2U);
  }

  for (c4_i9 = 0; c4_i9 < 3; c4_i9 = c4_i9 + 1) {
    _SFD_DATA_RANGE_CHECK((*c4_u)[c4_i9], 3U);
  }

  for (c4_i10 = 0; c4_i10 < 2; c4_i10 = c4_i10 + 1) {
    _SFD_DATA_RANGE_CHECK((*c4_f_F)[c4_i10], 4U);
  }

  for (c4_i11 = 0; c4_i11 < 2; c4_i11 = c4_i11 + 1) {
    _SFD_DATA_RANGE_CHECK((*c4_f_R)[c4_i11], 5U);
  }

  for (c4_i12 = 0; c4_i12 < 2; c4_i12 = c4_i12 + 1) {
    _SFD_DATA_RANGE_CHECK((*c4_s)[c4_i12], 6U);
  }

  c4_previousEvent = _sfEvent_;
  _sfEvent_ = CALL_EVENT;
  c4_c4_car_model(chartInstance);
  _sfEvent_ = c4_previousEvent;
  sf_debug_check_for_state_inconsistency(_car_modelMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void c4_c4_car_model(SFc4_car_modelInstanceStruct *chartInstance)
{
  int32_T c4_i13;
  real_T c4_hoistedGlobal[4];
  int32_T c4_i14;
  real_T c4_b_hoistedGlobal[3];
  int32_T c4_i15;
  real_T c4_c_hoistedGlobal[2];
  int32_T c4_i16;
  real_T c4_d_hoistedGlobal[2];
  int32_T c4_i17;
  real_T c4_e_hoistedGlobal[2];
  int32_T c4_i18;
  real_T c4_X[4];
  int32_T c4_i19;
  real_T c4_u[3];
  int32_T c4_i20;
  real_T c4_f_F[2];
  int32_T c4_i21;
  real_T c4_f_R[2];
  int32_T c4_i22;
  real_T c4_s[2];
  uint32_T c4_debug_family_var_map[35];
  real_T c4_m;
  real_T c4_l_F;
  real_T c4_l_R;
  real_T c4_mu;
  real_T c4_C;
  real_T c4_B;
  real_T c4_g;
  real_T c4_Vx;
  real_T c4_Vy;
  real_T c4_r;
  real_T c4_psi;
  real_T c4_f_Fx;
  real_T c4_f_Rx;
  real_T c4_delta;
  real_T c4_f_Fy;
  real_T c4_f_Fz;
  real_T c4_f_Ry;
  real_T c4_f_Rz;
  real_T c4_s_Fy;
  real_T c4_s_Ry;
  real_T c4_f_Fz_out;
  real_T c4_f_Rz_out;
  real_T c4_f_Fy_max;
  real_T c4_f_Ry_max;
  real_T c4_f_Fy_out;
  real_T c4_f_Ry_out;
  real_T c4_nargin = 5.0;
  real_T c4_nargout = 2.0;
  real_T c4_f_F_out[2];
  real_T c4_f_R_out[2];
  real_T c4_b;
  real_T c4_y;
  real_T c4_x;
  real_T c4_b_x;
  real_T c4_b_b;
  real_T c4_b_y;
  real_T c4_c_x;
  real_T c4_d_x;
  real_T c4_c_b;
  real_T c4_c_y;
  real_T c4_e_x;
  real_T c4_f_x;
  real_T c4_g_x;
  real_T c4_d_b;
  real_T c4_d_y;
  real_T c4_h_x;
  real_T c4_i_x;
  real_T c4_j_x;
  real_T c4_a;
  real_T c4_e_b;
  real_T c4_f_b;
  real_T c4_e_y;
  real_T c4_k_x;
  real_T c4_l_x;
  real_T c4_m_x;
  real_T c4_g_b;
  real_T c4_f_y;
  real_T c4_n_x;
  real_T c4_o_x;
  real_T c4_p_x;
  real_T c4_b_a;
  real_T c4_h_b;
  int32_T c4_i23;
  int32_T c4_i24;
  real_T (*c4_b_f_F_out)[2];
  real_T (*c4_b_f_R_out)[2];
  real_T (*c4_b_s)[2];
  real_T (*c4_b_f_R)[2];
  real_T (*c4_b_f_F)[2];
  real_T (*c4_b_u)[3];
  real_T (*c4_b_X)[4];
  c4_b_s = (real_T (*)[2])ssGetInputPortSignal(chartInstance->S, 4);
  c4_b_f_R = (real_T (*)[2])ssGetInputPortSignal(chartInstance->S, 3);
  c4_b_f_F = (real_T (*)[2])ssGetInputPortSignal(chartInstance->S, 2);
  c4_b_u = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 1);
  c4_b_X = (real_T (*)[4])ssGetInputPortSignal(chartInstance->S, 0);
  c4_b_f_R_out = (real_T (*)[2])ssGetOutputPortSignal(chartInstance->S, 2);
  c4_b_f_F_out = (real_T (*)[2])ssGetOutputPortSignal(chartInstance->S, 1);
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG,3);
  for (c4_i13 = 0; c4_i13 < 4; c4_i13 = c4_i13 + 1) {
    c4_hoistedGlobal[c4_i13] = (*c4_b_X)[c4_i13];
  }

  for (c4_i14 = 0; c4_i14 < 3; c4_i14 = c4_i14 + 1) {
    c4_b_hoistedGlobal[c4_i14] = (*c4_b_u)[c4_i14];
  }

  for (c4_i15 = 0; c4_i15 < 2; c4_i15 = c4_i15 + 1) {
    c4_c_hoistedGlobal[c4_i15] = (*c4_b_f_F)[c4_i15];
  }

  for (c4_i16 = 0; c4_i16 < 2; c4_i16 = c4_i16 + 1) {
    c4_d_hoistedGlobal[c4_i16] = (*c4_b_f_R)[c4_i16];
  }

  for (c4_i17 = 0; c4_i17 < 2; c4_i17 = c4_i17 + 1) {
    c4_e_hoistedGlobal[c4_i17] = (*c4_b_s)[c4_i17];
  }

  for (c4_i18 = 0; c4_i18 < 4; c4_i18 = c4_i18 + 1) {
    c4_X[c4_i18] = c4_hoistedGlobal[c4_i18];
  }

  for (c4_i19 = 0; c4_i19 < 3; c4_i19 = c4_i19 + 1) {
    c4_u[c4_i19] = c4_b_hoistedGlobal[c4_i19];
  }

  for (c4_i20 = 0; c4_i20 < 2; c4_i20 = c4_i20 + 1) {
    c4_f_F[c4_i20] = c4_c_hoistedGlobal[c4_i20];
  }

  for (c4_i21 = 0; c4_i21 < 2; c4_i21 = c4_i21 + 1) {
    c4_f_R[c4_i21] = c4_d_hoistedGlobal[c4_i21];
  }

  for (c4_i22 = 0; c4_i22 < 2; c4_i22 = c4_i22 + 1) {
    c4_s[c4_i22] = c4_e_hoistedGlobal[c4_i22];
  }

  sf_debug_symbol_scope_push_eml(0U, 35U, 35U, c4_debug_family_names,
    c4_debug_family_var_map);
  sf_debug_symbol_scope_add_eml(&c4_m, c4_d_sf_marshall, 0U);
  sf_debug_symbol_scope_add_eml(&c4_l_F, c4_d_sf_marshall, 1U);
  sf_debug_symbol_scope_add_eml(&c4_l_R, c4_d_sf_marshall, 2U);
  sf_debug_symbol_scope_add_eml(&c4_mu, c4_d_sf_marshall, 3U);
  sf_debug_symbol_scope_add_eml(&c4_C, c4_d_sf_marshall, 4U);
  sf_debug_symbol_scope_add_eml(&c4_B, c4_d_sf_marshall, 5U);
  sf_debug_symbol_scope_add_eml(&c4_g, c4_d_sf_marshall, 6U);
  sf_debug_symbol_scope_add_eml(&c4_Vx, c4_d_sf_marshall, 7U);
  sf_debug_symbol_scope_add_eml(&c4_Vy, c4_d_sf_marshall, 8U);
  sf_debug_symbol_scope_add_eml(&c4_r, c4_d_sf_marshall, 9U);
  sf_debug_symbol_scope_add_eml(&c4_psi, c4_d_sf_marshall, 10U);
  sf_debug_symbol_scope_add_eml(&c4_f_Fx, c4_d_sf_marshall, 11U);
  sf_debug_symbol_scope_add_eml(&c4_f_Rx, c4_d_sf_marshall, 12U);
  sf_debug_symbol_scope_add_eml(&c4_delta, c4_d_sf_marshall, 13U);
  sf_debug_symbol_scope_add_eml(&c4_f_Fy, c4_d_sf_marshall, 14U);
  sf_debug_symbol_scope_add_eml(&c4_f_Fz, c4_d_sf_marshall, 15U);
  sf_debug_symbol_scope_add_eml(&c4_f_Ry, c4_d_sf_marshall, 16U);
  sf_debug_symbol_scope_add_eml(&c4_f_Rz, c4_d_sf_marshall, 17U);
  sf_debug_symbol_scope_add_eml(&c4_s_Fy, c4_d_sf_marshall, 18U);
  sf_debug_symbol_scope_add_eml(&c4_s_Ry, c4_d_sf_marshall, 19U);
  sf_debug_symbol_scope_add_eml(&c4_f_Fz_out, c4_d_sf_marshall, 20U);
  sf_debug_symbol_scope_add_eml(&c4_f_Rz_out, c4_d_sf_marshall, 21U);
  sf_debug_symbol_scope_add_eml(&c4_f_Fy_max, c4_d_sf_marshall, 22U);
  sf_debug_symbol_scope_add_eml(&c4_f_Ry_max, c4_d_sf_marshall, 23U);
  sf_debug_symbol_scope_add_eml(&c4_f_Fy_out, c4_d_sf_marshall, 24U);
  sf_debug_symbol_scope_add_eml(&c4_f_Ry_out, c4_d_sf_marshall, 25U);
  sf_debug_symbol_scope_add_eml(&c4_nargin, c4_d_sf_marshall, 26U);
  sf_debug_symbol_scope_add_eml(&c4_nargout, c4_d_sf_marshall, 27U);
  sf_debug_symbol_scope_add_eml(&c4_X, c4_c_sf_marshall, 28U);
  sf_debug_symbol_scope_add_eml(&c4_u, c4_b_sf_marshall, 29U);
  sf_debug_symbol_scope_add_eml(&c4_f_F, c4_sf_marshall, 30U);
  sf_debug_symbol_scope_add_eml(&c4_f_R, c4_sf_marshall, 31U);
  sf_debug_symbol_scope_add_eml(&c4_s, c4_sf_marshall, 32U);
  sf_debug_symbol_scope_add_eml(&c4_f_F_out, c4_sf_marshall, 33U);
  sf_debug_symbol_scope_add_eml(&c4_f_R_out, c4_sf_marshall, 34U);
  CV_EML_FCN(0, 0);

  /*  m = param.m; */
  /*  l_F = param.l_F; */
  /*  l_R = param.l_R; */
  /*  mu = param.mu; */
  _SFD_EML_CALL(0,8);
  c4_m = 1450.0;
  _SFD_EML_CALL(0,9);
  c4_l_F = 1.1;
  _SFD_EML_CALL(0,10);
  c4_l_R = 1.6;
  _SFD_EML_CALL(0,11);
  c4_mu = 0.5;
  _SFD_EML_CALL(0,12);
  c4_C = 1.4;
  _SFD_EML_CALL(0,13);
  c4_B = 7.0;
  _SFD_EML_CALL(0,15);
  c4_g = 9.82;
  _SFD_EML_CALL(0,18);
  c4_Vx = c4_X[0];
  _SFD_EML_CALL(0,19);
  c4_Vy = c4_X[1];
  _SFD_EML_CALL(0,20);
  c4_r = c4_X[2];
  _SFD_EML_CALL(0,21);
  c4_psi = c4_X[3];
  _SFD_EML_CALL(0,23);
  c4_f_Fx = c4_u[0];
  _SFD_EML_CALL(0,24);
  c4_f_Rx = c4_u[1];
  _SFD_EML_CALL(0,25);
  c4_delta = c4_u[2];
  _SFD_EML_CALL(0,27);
  c4_f_Fy = c4_f_F[0];
  _SFD_EML_CALL(0,28);
  c4_f_Fz = c4_f_F[1];
  _SFD_EML_CALL(0,30);
  c4_f_Ry = c4_f_R[0];
  _SFD_EML_CALL(0,31);
  c4_f_Rz = c4_f_R[1];
  _SFD_EML_CALL(0,33);
  c4_s_Fy = c4_s[0];
  _SFD_EML_CALL(0,34);
  c4_s_Ry = c4_s[1];
  _SFD_EML_CALL(0,37);
  c4_f_Fz_out = 6.1512480000000010E+004;
  _SFD_EML_CALL(0,38);
  c4_f_Rz_out = 4.2289830000000009E+004;
  _SFD_EML_CALL(0,40);
  c4_b = c4_f_Fz;
  c4_y = 0.5 * c4_b;
  c4_x = c4_mpower(chartInstance, c4_y) - c4_mpower(chartInstance, c4_f_Fx);
  c4_f_Fy_max = c4_x;
  if (c4_f_Fy_max < 0.0) {
    c4_eml_error(chartInstance);
  }

  c4_b_x = c4_f_Fy_max;
  c4_f_Fy_max = c4_b_x;
  c4_f_Fy_max = muDoubleScalarSqrt(c4_f_Fy_max);
  _SFD_EML_CALL(0,41);
  c4_b_b = c4_f_Rz;
  c4_b_y = 0.5 * c4_b_b;
  c4_c_x = c4_mpower(chartInstance, c4_b_y) - c4_mpower(chartInstance, c4_f_Rx);
  c4_f_Ry_max = c4_c_x;
  if (c4_f_Ry_max < 0.0) {
    c4_eml_error(chartInstance);
  }

  c4_d_x = c4_f_Ry_max;
  c4_f_Ry_max = c4_d_x;
  c4_f_Ry_max = muDoubleScalarSqrt(c4_f_Ry_max);
  _SFD_EML_CALL(0,43);
  c4_c_b = c4_s_Fy;
  c4_c_y = 7.0 * c4_c_b;
  c4_e_x = c4_c_y;
  c4_f_x = c4_e_x;
  c4_g_x = c4_f_x;
  c4_f_x = c4_g_x;
  c4_f_x = muDoubleScalarAtan(c4_f_x);
  c4_d_b = c4_f_x;
  c4_d_y = 1.4 * c4_d_b;
  c4_h_x = c4_d_y;
  c4_i_x = c4_h_x;
  c4_j_x = c4_i_x;
  c4_i_x = c4_j_x;
  c4_i_x = muDoubleScalarSin(c4_i_x);
  c4_a = -c4_f_Fy_max;
  c4_e_b = c4_i_x;
  c4_f_Fy_out = c4_a * c4_e_b;
  _SFD_EML_CALL(0,44);
  c4_f_b = c4_s_Ry;
  c4_e_y = 7.0 * c4_f_b;
  c4_k_x = c4_e_y;
  c4_l_x = c4_k_x;
  c4_m_x = c4_l_x;
  c4_l_x = c4_m_x;
  c4_l_x = muDoubleScalarAtan(c4_l_x);
  c4_g_b = c4_l_x;
  c4_f_y = 1.4 * c4_g_b;
  c4_n_x = c4_f_y;
  c4_o_x = c4_n_x;
  c4_p_x = c4_o_x;
  c4_o_x = c4_p_x;
  c4_o_x = muDoubleScalarSin(c4_o_x);
  c4_b_a = -c4_f_Ry_max;
  c4_h_b = c4_o_x;
  c4_f_Ry_out = c4_b_a * c4_h_b;
  _SFD_EML_CALL(0,46);
  c4_f_F_out[0] = c4_f_Fy_out;
  c4_f_F_out[1] = c4_f_Fz_out;
  _SFD_EML_CALL(0,47);
  c4_f_R_out[0] = c4_f_Ry_out;
  c4_f_R_out[1] = c4_f_Rz_out;
  _SFD_EML_CALL(0,-47);
  sf_debug_symbol_scope_pop();
  for (c4_i23 = 0; c4_i23 < 2; c4_i23 = c4_i23 + 1) {
    (*c4_b_f_F_out)[c4_i23] = c4_f_F_out[c4_i23];
  }

  for (c4_i24 = 0; c4_i24 < 2; c4_i24 = c4_i24 + 1) {
    (*c4_b_f_R_out)[c4_i24] = c4_f_R_out[c4_i24];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG,3);
}

static void init_script_number_translation(uint32_T c4_machineNumber, uint32_T
  c4_chartNumber)
{
}

static real_T c4_mpower(SFc4_car_modelInstanceStruct *chartInstance, real_T c4_a)
{
  real_T c4_b_a;
  real_T c4_ak;
  c4_b_a = c4_a;
  c4_ak = c4_b_a;
  return muDoubleScalarPower(c4_ak, 2.0);
}

static void c4_eml_error(SFc4_car_modelInstanceStruct *chartInstance)
{
  int32_T c4_i25;
  static char_T c4_cv0[31] = { 'E', 'm', 'b', 'e', 'd', 'd', 'e', 'd', 'M', 'A',
    'T', 'L', 'A', 'B', ':', 's', 'q', 'r', 't', ':', 'd'
    , 'o', 'm', 'a', 'i', 'n', 'E', 'r', 'r', 'o', 'r' };

  char_T c4_u[31];
  const mxArray *c4_y = NULL;
  int32_T c4_i26;
  static char_T c4_cv1[77] = { 'D', 'o', 'm', 'a', 'i', 'n', ' ', 'e', 'r', 'r',
    'o', 'r', '.', ' ', 'T', 'o', ' ', 'c', 'o', 'm', 'p'
    , 'u', 't', 'e', ' ', 'c', 'o', 'm', 'p', 'l', 'e', 'x', ' ', 'r', 'e', 's',
    'u', 'l', 't', 's', ' ',
    'f', 'r', 'o', 'm', ' ', 'r', 'e', 'a', 'l', ' ', 'x', ',', ' ', 'u', 's',
    'e', ' ', '\'', 's', 'q',
    'r', 't', '(', 'c', 'o', 'm', 'p', 'l', 'e', 'x', '(', 'x', ')', ')', '\'',
    '.' };

  char_T c4_b_u[77];
  const mxArray *c4_b_y = NULL;
  for (c4_i25 = 0; c4_i25 < 31; c4_i25 = c4_i25 + 1) {
    c4_u[c4_i25] = c4_cv0[c4_i25];
  }

  c4_y = NULL;
  sf_mex_assign(&c4_y, sf_mex_create("y", &c4_u, 10, 0U, 1U, 0U, 2, 1, 31));
  for (c4_i26 = 0; c4_i26 < 77; c4_i26 = c4_i26 + 1) {
    c4_b_u[c4_i26] = c4_cv1[c4_i26];
  }

  c4_b_y = NULL;
  sf_mex_assign(&c4_b_y, sf_mex_create("y", &c4_b_u, 10, 0U, 1U, 0U, 2, 1, 77));
  sf_mex_call_debug("error", 0U, 2U, 14, c4_y, 14, c4_b_y);
}

static const mxArray *c4_sf_marshall(void *chartInstanceVoid, void *c4_u)
{
  const mxArray *c4_y = NULL;
  int32_T c4_i27;
  real_T c4_b_u[2];
  const mxArray *c4_b_y = NULL;
  SFc4_car_modelInstanceStruct *chartInstance;
  chartInstance = (SFc4_car_modelInstanceStruct *)chartInstanceVoid;
  c4_y = NULL;
  for (c4_i27 = 0; c4_i27 < 2; c4_i27 = c4_i27 + 1) {
    c4_b_u[c4_i27] = (*((real_T (*)[2])c4_u))[c4_i27];
  }

  c4_b_y = NULL;
  sf_mex_assign(&c4_b_y, sf_mex_create("y", &c4_b_u, 0, 0U, 1U, 0U, 1, 2));
  sf_mex_assign(&c4_y, c4_b_y);
  return c4_y;
}

static const mxArray *c4_b_sf_marshall(void *chartInstanceVoid, void *c4_u)
{
  const mxArray *c4_y = NULL;
  int32_T c4_i28;
  real_T c4_b_u[3];
  const mxArray *c4_b_y = NULL;
  SFc4_car_modelInstanceStruct *chartInstance;
  chartInstance = (SFc4_car_modelInstanceStruct *)chartInstanceVoid;
  c4_y = NULL;
  for (c4_i28 = 0; c4_i28 < 3; c4_i28 = c4_i28 + 1) {
    c4_b_u[c4_i28] = (*((real_T (*)[3])c4_u))[c4_i28];
  }

  c4_b_y = NULL;
  sf_mex_assign(&c4_b_y, sf_mex_create("y", &c4_b_u, 0, 0U, 1U, 0U, 1, 3));
  sf_mex_assign(&c4_y, c4_b_y);
  return c4_y;
}

static const mxArray *c4_c_sf_marshall(void *chartInstanceVoid, void *c4_u)
{
  const mxArray *c4_y = NULL;
  int32_T c4_i29;
  real_T c4_b_u[4];
  const mxArray *c4_b_y = NULL;
  SFc4_car_modelInstanceStruct *chartInstance;
  chartInstance = (SFc4_car_modelInstanceStruct *)chartInstanceVoid;
  c4_y = NULL;
  for (c4_i29 = 0; c4_i29 < 4; c4_i29 = c4_i29 + 1) {
    c4_b_u[c4_i29] = (*((real_T (*)[4])c4_u))[c4_i29];
  }

  c4_b_y = NULL;
  sf_mex_assign(&c4_b_y, sf_mex_create("y", &c4_b_u, 0, 0U, 1U, 0U, 1, 4));
  sf_mex_assign(&c4_y, c4_b_y);
  return c4_y;
}

static const mxArray *c4_d_sf_marshall(void *chartInstanceVoid, void *c4_u)
{
  const mxArray *c4_y = NULL;
  real_T c4_b_u;
  const mxArray *c4_b_y = NULL;
  SFc4_car_modelInstanceStruct *chartInstance;
  chartInstance = (SFc4_car_modelInstanceStruct *)chartInstanceVoid;
  c4_y = NULL;
  c4_b_u = *((real_T *)c4_u);
  c4_b_y = NULL;
  sf_mex_assign(&c4_b_y, sf_mex_create("y", &c4_b_u, 0, 0U, 0U, 0U, 0));
  sf_mex_assign(&c4_y, c4_b_y);
  return c4_y;
}

const mxArray *sf_c4_car_model_get_eml_resolved_functions_info(void)
{
  const mxArray *c4_nameCaptureInfo = NULL;
  c4_ResolvedFunctionInfo c4_info[33];
  const mxArray *c4_m0 = NULL;
  int32_T c4_i30;
  c4_ResolvedFunctionInfo *c4_r0;
  c4_nameCaptureInfo = NULL;
  c4_info_helper(c4_info);
  sf_mex_assign(&c4_m0, sf_mex_createstruct("nameCaptureInfo", 1, 33));
  for (c4_i30 = 0; c4_i30 < 33; c4_i30 = c4_i30 + 1) {
    c4_r0 = &c4_info[c4_i30];
    sf_mex_addfield(c4_m0, sf_mex_create("nameCaptureInfo", c4_r0->context, 15,
      0U, 0U, 0U, 2, 1, strlen(c4_r0->context)), "context",
                    "nameCaptureInfo", c4_i30);
    sf_mex_addfield(c4_m0, sf_mex_create("nameCaptureInfo", c4_r0->name, 15, 0U,
      0U, 0U, 2, 1, strlen(c4_r0->name)), "name",
                    "nameCaptureInfo", c4_i30);
    sf_mex_addfield(c4_m0, sf_mex_create("nameCaptureInfo", c4_r0->dominantType,
      15, 0U, 0U, 0U, 2, 1, strlen(c4_r0->dominantType)),
                    "dominantType", "nameCaptureInfo", c4_i30);
    sf_mex_addfield(c4_m0, sf_mex_create("nameCaptureInfo", c4_r0->resolved, 15,
      0U, 0U, 0U, 2, 1, strlen(c4_r0->resolved)), "resolved"
                    , "nameCaptureInfo", c4_i30);
    sf_mex_addfield(c4_m0, sf_mex_create("nameCaptureInfo", &c4_r0->fileLength,
      7, 0U, 0U, 0U, 0), "fileLength", "nameCaptureInfo",
                    c4_i30);
    sf_mex_addfield(c4_m0, sf_mex_create("nameCaptureInfo", &c4_r0->fileTime1, 7,
      0U, 0U, 0U, 0), "fileTime1", "nameCaptureInfo", c4_i30
                    );
    sf_mex_addfield(c4_m0, sf_mex_create("nameCaptureInfo", &c4_r0->fileTime2, 7,
      0U, 0U, 0U, 0), "fileTime2", "nameCaptureInfo", c4_i30
                    );
  }

  sf_mex_assign(&c4_nameCaptureInfo, c4_m0);
  return c4_nameCaptureInfo;
}

static void c4_info_helper(c4_ResolvedFunctionInfo c4_info[33])
{
  c4_info[0].context = "";
  c4_info[0].name = "mtimes";
  c4_info[0].dominantType = "double";
  c4_info[0].resolved = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c4_info[0].fileLength = 3425U;
  c4_info[0].fileTime1 = 1250694366U;
  c4_info[0].fileTime2 = 0U;
  c4_info[1].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c4_info[1].name = "nargin";
  c4_info[1].dominantType = "";
  c4_info[1].resolved = "[B]nargin";
  c4_info[1].fileLength = 0U;
  c4_info[1].fileTime1 = 0U;
  c4_info[1].fileTime2 = 0U;
  c4_info[2].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c4_info[2].name = "gt";
  c4_info[2].dominantType = "double";
  c4_info[2].resolved = "[B]gt";
  c4_info[2].fileLength = 0U;
  c4_info[2].fileTime1 = 0U;
  c4_info[2].fileTime2 = 0U;
  c4_info[3].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c4_info[3].name = "isa";
  c4_info[3].dominantType = "double";
  c4_info[3].resolved = "[B]isa";
  c4_info[3].fileLength = 0U;
  c4_info[3].fileTime1 = 0U;
  c4_info[3].fileTime2 = 0U;
  c4_info[4].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c4_info[4].name = "isinteger";
  c4_info[4].dominantType = "double";
  c4_info[4].resolved = "[B]isinteger";
  c4_info[4].fileLength = 0U;
  c4_info[4].fileTime1 = 0U;
  c4_info[4].fileTime2 = 0U;
  c4_info[5].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c4_info[5].name = "isscalar";
  c4_info[5].dominantType = "double";
  c4_info[5].resolved = "[B]isscalar";
  c4_info[5].fileLength = 0U;
  c4_info[5].fileTime1 = 0U;
  c4_info[5].fileTime2 = 0U;
  c4_info[6].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c4_info[6].name = "strcmp";
  c4_info[6].dominantType = "char";
  c4_info[6].resolved = "[B]strcmp";
  c4_info[6].fileLength = 0U;
  c4_info[6].fileTime1 = 0U;
  c4_info[6].fileTime2 = 0U;
  c4_info[7].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c4_info[7].name = "size";
  c4_info[7].dominantType = "double";
  c4_info[7].resolved = "[B]size";
  c4_info[7].fileLength = 0U;
  c4_info[7].fileTime1 = 0U;
  c4_info[7].fileTime2 = 0U;
  c4_info[8].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c4_info[8].name = "eq";
  c4_info[8].dominantType = "double";
  c4_info[8].resolved = "[B]eq";
  c4_info[8].fileLength = 0U;
  c4_info[8].fileTime1 = 0U;
  c4_info[8].fileTime2 = 0U;
  c4_info[9].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c4_info[9].name = "class";
  c4_info[9].dominantType = "double";
  c4_info[9].resolved = "[B]class";
  c4_info[9].fileLength = 0U;
  c4_info[9].fileTime1 = 0U;
  c4_info[9].fileTime2 = 0U;
  c4_info[10].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c4_info[10].name = "not";
  c4_info[10].dominantType = "logical";
  c4_info[10].resolved = "[B]not";
  c4_info[10].fileLength = 0U;
  c4_info[10].fileTime1 = 0U;
  c4_info[10].fileTime2 = 0U;
  c4_info[11].context = "";
  c4_info[11].name = "plus";
  c4_info[11].dominantType = "double";
  c4_info[11].resolved = "[B]plus";
  c4_info[11].fileLength = 0U;
  c4_info[11].fileTime1 = 0U;
  c4_info[11].fileTime2 = 0U;
  c4_info[12].context = "";
  c4_info[12].name = "mpower";
  c4_info[12].dominantType = "double";
  c4_info[12].resolved = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m";
  c4_info[12].fileLength = 3710U;
  c4_info[12].fileTime1 = 1238455888U;
  c4_info[12].fileTime2 = 0U;
  c4_info[13].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m";
  c4_info[13].name = "ndims";
  c4_info[13].dominantType = "double";
  c4_info[13].resolved = "[B]ndims";
  c4_info[13].fileLength = 0U;
  c4_info[13].fileTime1 = 0U;
  c4_info[13].fileTime2 = 0U;
  c4_info[14].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m";
  c4_info[14].name = "power";
  c4_info[14].dominantType = "double";
  c4_info[14].resolved = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m";
  c4_info[14].fileLength = 5380U;
  c4_info[14].fileTime1 = 1228115498U;
  c4_info[14].fileTime2 = 0U;
  c4_info[15].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m";
  c4_info[15].name = "eml_scalar_eg";
  c4_info[15].dominantType = "double";
  c4_info[15].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c4_info[15].fileLength = 3068U;
  c4_info[15].fileTime1 = 1240283610U;
  c4_info[15].fileTime2 = 0U;
  c4_info[16].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m/any_enums";
  c4_info[16].name = "false";
  c4_info[16].dominantType = "";
  c4_info[16].resolved = "[B]false";
  c4_info[16].fileLength = 0U;
  c4_info[16].fileTime1 = 0U;
  c4_info[16].fileTime2 = 0U;
  c4_info[17].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c4_info[17].name = "isstruct";
  c4_info[17].dominantType = "double";
  c4_info[17].resolved = "[B]isstruct";
  c4_info[17].fileLength = 0U;
  c4_info[17].fileTime1 = 0U;
  c4_info[17].fileTime2 = 0U;
  c4_info[18].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m/zerosum";
  c4_info[18].name = "cast";
  c4_info[18].dominantType = "double";
  c4_info[18].resolved = "[B]cast";
  c4_info[18].fileLength = 0U;
  c4_info[18].fileTime1 = 0U;
  c4_info[18].fileTime2 = 0U;
  c4_info[19].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m/allreal";
  c4_info[19].name = "isreal";
  c4_info[19].dominantType = "double";
  c4_info[19].resolved = "[B]isreal";
  c4_info[19].fileLength = 0U;
  c4_info[19].fileTime1 = 0U;
  c4_info[19].fileTime2 = 0U;
  c4_info[20].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m";
  c4_info[20].name = "eml_scalexp_alloc";
  c4_info[20].dominantType = "double";
  c4_info[20].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c4_info[20].fileLength = 808U;
  c4_info[20].fileTime1 = 1230516298U;
  c4_info[20].fileTime2 = 0U;
  c4_info[21].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c4_info[21].name = "minus";
  c4_info[21].dominantType = "double";
  c4_info[21].resolved = "[B]minus";
  c4_info[21].fileLength = 0U;
  c4_info[21].fileTime1 = 0U;
  c4_info[21].fileTime2 = 0U;
  c4_info[22].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m";
  c4_info[22].name = "lt";
  c4_info[22].dominantType = "double";
  c4_info[22].resolved = "[B]lt";
  c4_info[22].fileLength = 0U;
  c4_info[22].fileTime1 = 0U;
  c4_info[22].fileTime2 = 0U;
  c4_info[23].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m";
  c4_info[23].name = "eml_scalar_floor";
  c4_info[23].dominantType = "double";
  c4_info[23].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m";
  c4_info[23].fileLength = 260U;
  c4_info[23].fileTime1 = 1209352390U;
  c4_info[23].fileTime2 = 0U;
  c4_info[24].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m";
  c4_info[24].name = "ne";
  c4_info[24].dominantType = "double";
  c4_info[24].resolved = "[B]ne";
  c4_info[24].fileLength = 0U;
  c4_info[24].fileTime1 = 0U;
  c4_info[24].fileTime2 = 0U;
  c4_info[25].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m";
  c4_info[25].name = "eml_error";
  c4_info[25].dominantType = "char";
  c4_info[25].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m";
  c4_info[25].fileLength = 315U;
  c4_info[25].fileTime1 = 1213948344U;
  c4_info[25].fileTime2 = 0U;
  c4_info[26].context = "";
  c4_info[26].name = "sqrt";
  c4_info[26].dominantType = "double";
  c4_info[26].resolved = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m";
  c4_info[26].fileLength = 572U;
  c4_info[26].fileTime1 = 1203469644U;
  c4_info[26].fileTime2 = 0U;
  c4_info[27].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m";
  c4_info[27].name = "eml_scalar_sqrt";
  c4_info[27].dominantType = "double";
  c4_info[27].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_sqrt.m";
  c4_info[27].fileLength = 664U;
  c4_info[27].fileTime1 = 1209352394U;
  c4_info[27].fileTime2 = 0U;
  c4_info[28].context = "";
  c4_info[28].name = "uminus";
  c4_info[28].dominantType = "double";
  c4_info[28].resolved = "[B]uminus";
  c4_info[28].fileLength = 0U;
  c4_info[28].fileTime1 = 0U;
  c4_info[28].fileTime2 = 0U;
  c4_info[29].context = "";
  c4_info[29].name = "atan";
  c4_info[29].dominantType = "double";
  c4_info[29].resolved = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/atan.m";
  c4_info[29].fileLength = 536U;
  c4_info[29].fileTime1 = 1203469546U;
  c4_info[29].fileTime2 = 0U;
  c4_info[30].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/atan.m";
  c4_info[30].name = "eml_scalar_atan";
  c4_info[30].dominantType = "double";
  c4_info[30].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_atan.m";
  c4_info[30].fileLength = 281U;
  c4_info[30].fileTime1 = 1203469576U;
  c4_info[30].fileTime2 = 0U;
  c4_info[31].context = "";
  c4_info[31].name = "sin";
  c4_info[31].dominantType = "double";
  c4_info[31].resolved = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/sin.m";
  c4_info[31].fileLength = 324U;
  c4_info[31].fileTime1 = 1203469642U;
  c4_info[31].fileTime2 = 0U;
  c4_info[32].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/sin.m";
  c4_info[32].name = "eml_scalar_sin";
  c4_info[32].dominantType = "double";
  c4_info[32].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_sin.m";
  c4_info[32].fileLength = 601U;
  c4_info[32].fileTime1 = 1209352390U;
  c4_info[32].fileTime2 = 0U;
}

static const mxArray *c4_e_sf_marshall(void *chartInstanceVoid, void *c4_u)
{
  const mxArray *c4_y = NULL;
  boolean_T c4_b_u;
  const mxArray *c4_b_y = NULL;
  SFc4_car_modelInstanceStruct *chartInstance;
  chartInstance = (SFc4_car_modelInstanceStruct *)chartInstanceVoid;
  c4_y = NULL;
  c4_b_u = *((boolean_T *)c4_u);
  c4_b_y = NULL;
  sf_mex_assign(&c4_b_y, sf_mex_create("y", &c4_b_u, 11, 0U, 0U, 0U, 0));
  sf_mex_assign(&c4_y, c4_b_y);
  return c4_y;
}

static void c4_emlrt_marshallIn(SFc4_car_modelInstanceStruct *chartInstance,
  const mxArray *c4_f_F_out, const char_T *c4_name,
  real_T c4_y[2])
{
  real_T c4_dv2[2];
  int32_T c4_i31;
  sf_mex_import(c4_name, sf_mex_dup(c4_f_F_out), &c4_dv2, 1, 0, 0U, 1, 0U, 1, 2);
  for (c4_i31 = 0; c4_i31 < 2; c4_i31 = c4_i31 + 1) {
    c4_y[c4_i31] = c4_dv2[c4_i31];
  }

  sf_mex_destroy(&c4_f_F_out);
}

static uint8_T c4_b_emlrt_marshallIn(SFc4_car_modelInstanceStruct *chartInstance,
  const mxArray *c4_b_is_active_c4_car_model, const
  char_T *c4_name)
{
  uint8_T c4_y;
  uint8_T c4_u0;
  sf_mex_import(c4_name, sf_mex_dup(c4_b_is_active_c4_car_model), &c4_u0, 1, 3,
                0U, 0, 0U, 0);
  c4_y = c4_u0;
  sf_mex_destroy(&c4_b_is_active_c4_car_model);
  return c4_y;
}

static void init_dsm_address_info(SFc4_car_modelInstanceStruct *chartInstance)
{
}

/* SFunction Glue Code */
void sf_c4_car_model_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(1344128994U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(3185285043U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1681516482U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(768590515U);
}

mxArray *sf_c4_car_model_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,4,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateDoubleMatrix(4,1,mxREAL);
    double *pr = mxGetPr(mxChecksum);
    pr[0] = (double)(2607301397U);
    pr[1] = (double)(2238032426U);
    pr[2] = (double)(2340413001U);
    pr[3] = (double)(1942693168U);
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,5,3,dataFields);

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

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(2);
      pr[1] = (double)(1);
      mxSetField(mxData,4,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,4,"type",mxType);
    }

    mxSetField(mxData,4,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxCreateDoubleMatrix(0,0,
                mxREAL));
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,2,3,dataFields);

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
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  return(mxAutoinheritanceInfo);
}

static mxArray *sf_get_sim_state_info_c4_car_model(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x3'type','srcId','name','auxInfo'{{M[1],M[25],T\"f_F_out\",},{M[1],M[26],T\"f_R_out\",},{M[8],M[0],T\"is_active_c4_car_model\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 3, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c4_car_model_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc4_car_modelInstanceStruct *chartInstance;
    chartInstance = (SFc4_car_modelInstanceStruct *) ((ChartInfoStruct *)
      (ssGetUserData(S)))->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart(_car_modelMachineNumber_,
          4,
          1,
          1,
          7,
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
            dimVector[0]= 2;
            _SFD_SET_DATA_PROPS(0,2,0,1,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
                                1.0,0,"f_F_out",0,(MexFcnForType)c4_sf_marshall);
          }

          {
            unsigned int dimVector[1];
            dimVector[0]= 2;
            _SFD_SET_DATA_PROPS(1,2,0,1,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
                                1.0,0,"f_R_out",0,(MexFcnForType)c4_sf_marshall);
          }

          {
            unsigned int dimVector[1];
            dimVector[0]= 4;
            _SFD_SET_DATA_PROPS(2,1,1,0,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
                                1.0,0,"X",0,(MexFcnForType)c4_c_sf_marshall);
          }

          {
            unsigned int dimVector[1];
            dimVector[0]= 3;
            _SFD_SET_DATA_PROPS(3,1,1,0,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
                                1.0,0,"u",0,(MexFcnForType)c4_b_sf_marshall);
          }

          {
            unsigned int dimVector[1];
            dimVector[0]= 2;
            _SFD_SET_DATA_PROPS(4,1,1,0,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
                                1.0,0,"f_F",0,(MexFcnForType)c4_sf_marshall);
          }

          {
            unsigned int dimVector[1];
            dimVector[0]= 2;
            _SFD_SET_DATA_PROPS(5,1,1,0,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
                                1.0,0,"f_R",0,(MexFcnForType)c4_sf_marshall);
          }

          {
            unsigned int dimVector[1];
            dimVector[0]= 2;
            _SFD_SET_DATA_PROPS(6,1,1,0,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
                                1.0,0,"s",0,(MexFcnForType)c4_sf_marshall);
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
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,681);
        _SFD_TRANS_COV_WTS(0,0,0,1,0);
        if (chartAlreadyPresent==0) {
          _SFD_TRANS_COV_MAPS(0,
                              0,NULL,NULL,
                              0,NULL,NULL,
                              1,NULL,NULL,
                              0,NULL,NULL);
        }

        {
          real_T (*c4_f_F_out)[2];
          real_T (*c4_f_R_out)[2];
          real_T (*c4_X)[4];
          real_T (*c4_u)[3];
          real_T (*c4_f_F)[2];
          real_T (*c4_f_R)[2];
          real_T (*c4_s)[2];
          c4_s = (real_T (*)[2])ssGetInputPortSignal(chartInstance->S, 4);
          c4_f_R = (real_T (*)[2])ssGetInputPortSignal(chartInstance->S, 3);
          c4_f_F = (real_T (*)[2])ssGetInputPortSignal(chartInstance->S, 2);
          c4_u = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 1);
          c4_X = (real_T (*)[4])ssGetInputPortSignal(chartInstance->S, 0);
          c4_f_R_out = (real_T (*)[2])ssGetOutputPortSignal(chartInstance->S, 2);
          c4_f_F_out = (real_T (*)[2])ssGetOutputPortSignal(chartInstance->S, 1);
          _SFD_SET_DATA_VALUE_PTR(0U, c4_f_F_out);
          _SFD_SET_DATA_VALUE_PTR(1U, c4_f_R_out);
          _SFD_SET_DATA_VALUE_PTR(2U, c4_X);
          _SFD_SET_DATA_VALUE_PTR(3U, c4_u);
          _SFD_SET_DATA_VALUE_PTR(4U, c4_f_F);
          _SFD_SET_DATA_VALUE_PTR(5U, c4_f_R);
          _SFD_SET_DATA_VALUE_PTR(6U, c4_s);
        }
      }
    } else {
      sf_debug_reset_current_state_configuration(_car_modelMachineNumber_,
        chartInstance->chartNumber,chartInstance->instanceNumber);
    }
  }
}

static void sf_opaque_initialize_c4_car_model(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc4_car_modelInstanceStruct*) chartInstanceVar
    )->S,0);
  initialize_params_c4_car_model((SFc4_car_modelInstanceStruct*)
    chartInstanceVar);
  initialize_c4_car_model((SFc4_car_modelInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_enable_c4_car_model(void *chartInstanceVar)
{
  enable_c4_car_model((SFc4_car_modelInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_disable_c4_car_model(void *chartInstanceVar)
{
  disable_c4_car_model((SFc4_car_modelInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_gateway_c4_car_model(void *chartInstanceVar)
{
  sf_c4_car_model((SFc4_car_modelInstanceStruct*) chartInstanceVar);
}

static mxArray* sf_internal_get_sim_state_c4_car_model(SimStruct* S)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c4_car_model((SFc4_car_modelInstanceStruct*)
    chartInfo->chartInstance);         /* raw sim ctx */
  prhs[3] = sf_get_sim_state_info_c4_car_model();/* state var info */
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

static void sf_internal_set_sim_state_c4_car_model(SimStruct* S, const mxArray
  *st)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = mxDuplicateArray(st);      /* high level simctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c4_car_model();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c4_car_model((SFc4_car_modelInstanceStruct*)
    chartInfo->chartInstance, mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static mxArray* sf_opaque_get_sim_state_c4_car_model(SimStruct* S)
{
  return sf_internal_get_sim_state_c4_car_model(S);
}

static void sf_opaque_set_sim_state_c4_car_model(SimStruct* S, const mxArray *st)
{
  sf_internal_set_sim_state_c4_car_model(S, st);
}

static void sf_opaque_terminate_c4_car_model(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc4_car_modelInstanceStruct*) chartInstanceVar)->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
    }

    finalize_c4_car_model((SFc4_car_modelInstanceStruct*) chartInstanceVar);
    free((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c4_car_model(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c4_car_model((SFc4_car_modelInstanceStruct*)
      (((ChartInfoStruct *)ssGetUserData(S))->chartInstance));
  }
}

static void mdlSetWorkWidths_c4_car_model(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable("car_model","car_model",4);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop("car_model","car_model",4,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop("car_model","car_model",4,
      "gatewayCannotBeInlinedMultipleTimes"));
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 4, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,"car_model","car_model",4,5);
      sf_mark_chart_reusable_outputs(S,"car_model","car_model",4,2);
    }

    sf_set_rtw_dwork_info(S,"car_model","car_model",4);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
    ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  }

  ssSetChecksum0(S,(1146879499U));
  ssSetChecksum1(S,(337417185U));
  ssSetChecksum2(S,(249344821U));
  ssSetChecksum3(S,(2087599758U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
}

static void mdlRTW_c4_car_model(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    sf_write_symbol_mapping(S, "car_model", "car_model",4);
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c4_car_model(SimStruct *S)
{
  SFc4_car_modelInstanceStruct *chartInstance;
  chartInstance = (SFc4_car_modelInstanceStruct *)malloc(sizeof
    (SFc4_car_modelInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc4_car_modelInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway = sf_opaque_gateway_c4_car_model;
  chartInstance->chartInfo.initializeChart = sf_opaque_initialize_c4_car_model;
  chartInstance->chartInfo.terminateChart = sf_opaque_terminate_c4_car_model;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c4_car_model;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c4_car_model;
  chartInstance->chartInfo.getSimState = sf_opaque_get_sim_state_c4_car_model;
  chartInstance->chartInfo.setSimState = sf_opaque_set_sim_state_c4_car_model;
  chartInstance->chartInfo.getSimStateInfo = sf_get_sim_state_info_c4_car_model;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c4_car_model;
  chartInstance->chartInfo.mdlStart = mdlStart_c4_car_model;
  chartInstance->chartInfo.mdlSetWorkWidths = mdlSetWorkWidths_c4_car_model;
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

void c4_car_model_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c4_car_model(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c4_car_model(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c4_car_model(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c4_car_model_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
