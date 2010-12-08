/* Include files */

#include "blascompat32.h"
#include "car_model_sfun.h"
#include "c5_car_model.h"
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "car_model_sfun_debug_macros.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */
static const char *c5_debug_family_names[9] = { "Vx", "Vy", "r", "psi", "T",
  "nargin", "nargout", "X", "Vi" };

/* Function Declarations */
static void initialize_c5_car_model(SFc5_car_modelInstanceStruct *chartInstance);
static void initialize_params_c5_car_model(SFc5_car_modelInstanceStruct
  *chartInstance);
static void enable_c5_car_model(SFc5_car_modelInstanceStruct *chartInstance);
static void disable_c5_car_model(SFc5_car_modelInstanceStruct *chartInstance);
static void c5_update_debugger_state_c5_car_model(SFc5_car_modelInstanceStruct
  *chartInstance);
static const mxArray *get_sim_state_c5_car_model(SFc5_car_modelInstanceStruct
  *chartInstance);
static void set_sim_state_c5_car_model(SFc5_car_modelInstanceStruct
  *chartInstance, const mxArray *c5_st);
static void finalize_c5_car_model(SFc5_car_modelInstanceStruct *chartInstance);
static void sf_c5_car_model(SFc5_car_modelInstanceStruct *chartInstance);
static void c5_c5_car_model(SFc5_car_modelInstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T c5_machineNumber, uint32_T
  c5_chartNumber);
static void c5_eml_scalar_eg(SFc5_car_modelInstanceStruct *chartInstance);
static const mxArray *c5_sf_marshall(void *chartInstanceVoid, void *c5_u);
static const mxArray *c5_b_sf_marshall(void *chartInstanceVoid, void *c5_u);
static const mxArray *c5_c_sf_marshall(void *chartInstanceVoid, void *c5_u);
static const mxArray *c5_d_sf_marshall(void *chartInstanceVoid, void *c5_u);
static void c5_info_helper(c5_ResolvedFunctionInfo c5_info[43]);
static const mxArray *c5_e_sf_marshall(void *chartInstanceVoid, void *c5_u);
static void c5_emlrt_marshallIn(SFc5_car_modelInstanceStruct *chartInstance,
  const mxArray *c5_Vi, const char_T *c5_name, real_T c5_y[2]);
static uint8_T c5_b_emlrt_marshallIn(SFc5_car_modelInstanceStruct *chartInstance,
  const mxArray *c5_b_is_active_c5_car_model, const char_T *c5_name);
static void init_dsm_address_info(SFc5_car_modelInstanceStruct *chartInstance);

/* Function Definitions */
static void initialize_c5_car_model(SFc5_car_modelInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  chartInstance->c5_is_active_c5_car_model = 0U;
}

static void initialize_params_c5_car_model(SFc5_car_modelInstanceStruct
  *chartInstance)
{
}

static void enable_c5_car_model(SFc5_car_modelInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void disable_c5_car_model(SFc5_car_modelInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void c5_update_debugger_state_c5_car_model(SFc5_car_modelInstanceStruct
  *chartInstance)
{
}

static const mxArray *get_sim_state_c5_car_model(SFc5_car_modelInstanceStruct
  *chartInstance)
{
  const mxArray *c5_st = NULL;
  const mxArray *c5_y = NULL;
  int32_T c5_i0;
  real_T c5_hoistedGlobal[2];
  int32_T c5_i1;
  real_T c5_u[2];
  const mxArray *c5_b_y = NULL;
  uint8_T c5_b_hoistedGlobal;
  uint8_T c5_b_u;
  const mxArray *c5_c_y = NULL;
  real_T (*c5_Vi)[2];
  c5_Vi = (real_T (*)[2])ssGetOutputPortSignal(chartInstance->S, 1);
  c5_st = NULL;
  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_createcellarray(2));
  for (c5_i0 = 0; c5_i0 < 2; c5_i0 = c5_i0 + 1) {
    c5_hoistedGlobal[c5_i0] = (*c5_Vi)[c5_i0];
  }

  for (c5_i1 = 0; c5_i1 < 2; c5_i1 = c5_i1 + 1) {
    c5_u[c5_i1] = c5_hoistedGlobal[c5_i1];
  }

  c5_b_y = NULL;
  sf_mex_assign(&c5_b_y, sf_mex_create("y", &c5_u, 0, 0U, 1U, 0U, 1, 2));
  sf_mex_setcell(c5_y, 0, c5_b_y);
  c5_b_hoistedGlobal = chartInstance->c5_is_active_c5_car_model;
  c5_b_u = c5_b_hoistedGlobal;
  c5_c_y = NULL;
  sf_mex_assign(&c5_c_y, sf_mex_create("y", &c5_b_u, 3, 0U, 0U, 0U, 0));
  sf_mex_setcell(c5_y, 1, c5_c_y);
  sf_mex_assign(&c5_st, c5_y);
  return c5_st;
}

static void set_sim_state_c5_car_model(SFc5_car_modelInstanceStruct
  *chartInstance, const mxArray *c5_st)
{
  const mxArray *c5_u;
  real_T c5_dv0[2];
  int32_T c5_i2;
  real_T (*c5_Vi)[2];
  c5_Vi = (real_T (*)[2])ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c5_doneDoubleBufferReInit = TRUE;
  c5_u = sf_mex_dup(c5_st);
  c5_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c5_u, 0)), "Vi",
                      c5_dv0);
  for (c5_i2 = 0; c5_i2 < 2; c5_i2 = c5_i2 + 1) {
    (*c5_Vi)[c5_i2] = c5_dv0[c5_i2];
  }

  chartInstance->c5_is_active_c5_car_model = c5_b_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getcell(c5_u, 1)),
    "is_active_c5_car_model");
  sf_mex_destroy(&c5_u);
  c5_update_debugger_state_c5_car_model(chartInstance);
  sf_mex_destroy(&c5_st);
}

static void finalize_c5_car_model(SFc5_car_modelInstanceStruct *chartInstance)
{
}

static void sf_c5_car_model(SFc5_car_modelInstanceStruct *chartInstance)
{
  int32_T c5_i3;
  int32_T c5_i4;
  int32_T c5_previousEvent;
  real_T (*c5_Vi)[2];
  real_T (*c5_X)[4];
  c5_Vi = (real_T (*)[2])ssGetOutputPortSignal(chartInstance->S, 1);
  c5_X = (real_T (*)[4])ssGetInputPortSignal(chartInstance->S, 0);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG,4);
  for (c5_i3 = 0; c5_i3 < 4; c5_i3 = c5_i3 + 1) {
    _SFD_DATA_RANGE_CHECK((*c5_X)[c5_i3], 0U);
  }

  for (c5_i4 = 0; c5_i4 < 2; c5_i4 = c5_i4 + 1) {
    _SFD_DATA_RANGE_CHECK((*c5_Vi)[c5_i4], 1U);
  }

  c5_previousEvent = _sfEvent_;
  _sfEvent_ = CALL_EVENT;
  c5_c5_car_model(chartInstance);
  _sfEvent_ = c5_previousEvent;
  sf_debug_check_for_state_inconsistency(_car_modelMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void c5_c5_car_model(SFc5_car_modelInstanceStruct *chartInstance)
{
  int32_T c5_i5;
  real_T c5_hoistedGlobal[4];
  int32_T c5_i6;
  real_T c5_X[4];
  uint32_T c5_debug_family_var_map[9];
  real_T c5_Vx;
  real_T c5_Vy;
  real_T c5_r;
  real_T c5_psi;
  real_T c5_T[4];
  real_T c5_nargin = 1.0;
  real_T c5_nargout = 1.0;
  real_T c5_Vi[2];
  real_T c5_x;
  real_T c5_b_x;
  real_T c5_c_x;
  real_T c5_d_x;
  real_T c5_e_x;
  real_T c5_f_x;
  real_T c5_g_x;
  real_T c5_h_x;
  real_T c5_i_x;
  real_T c5_j_x;
  real_T c5_k_x;
  real_T c5_l_x;
  int32_T c5_i7;
  real_T c5_a[4];
  real_T c5_b[2];
  int32_T c5_i8;
  real_T c5_A[4];
  int32_T c5_i9;
  real_T c5_B[2];
  int32_T c5_i10;
  int32_T c5_i11;
  real_T c5_b_A[4];
  int32_T c5_i12;
  real_T c5_b_B[2];
  int32_T c5_i13;
  real_T c5_c_A[4];
  int32_T c5_i14;
  real_T c5_c_B[2];
  int32_T c5_i15;
  int32_T c5_i16;
  int32_T c5_i17;
  int32_T c5_i18;
  real_T (*c5_b_Vi)[2];
  real_T (*c5_b_X)[4];
  c5_b_Vi = (real_T (*)[2])ssGetOutputPortSignal(chartInstance->S, 1);
  c5_b_X = (real_T (*)[4])ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG,4);
  for (c5_i5 = 0; c5_i5 < 4; c5_i5 = c5_i5 + 1) {
    c5_hoistedGlobal[c5_i5] = (*c5_b_X)[c5_i5];
  }

  for (c5_i6 = 0; c5_i6 < 4; c5_i6 = c5_i6 + 1) {
    c5_X[c5_i6] = c5_hoistedGlobal[c5_i6];
  }

  sf_debug_symbol_scope_push_eml(0U, 9U, 9U, c5_debug_family_names,
    c5_debug_family_var_map);
  sf_debug_symbol_scope_add_eml(&c5_Vx, c5_c_sf_marshall, 0U);
  sf_debug_symbol_scope_add_eml(&c5_Vy, c5_c_sf_marshall, 1U);
  sf_debug_symbol_scope_add_eml(&c5_r, c5_c_sf_marshall, 2U);
  sf_debug_symbol_scope_add_eml(&c5_psi, c5_c_sf_marshall, 3U);
  sf_debug_symbol_scope_add_eml(&c5_T, c5_d_sf_marshall, 4U);
  sf_debug_symbol_scope_add_eml(&c5_nargin, c5_c_sf_marshall, 5U);
  sf_debug_symbol_scope_add_eml(&c5_nargout, c5_c_sf_marshall, 6U);
  sf_debug_symbol_scope_add_eml(&c5_X, c5_b_sf_marshall, 7U);
  sf_debug_symbol_scope_add_eml(&c5_Vi, c5_sf_marshall, 8U);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0,4);
  c5_Vx = c5_X[0];
  _SFD_EML_CALL(0,5);
  c5_Vy = c5_X[1];
  _SFD_EML_CALL(0,6);
  c5_r = c5_X[2];
  _SFD_EML_CALL(0,7);
  c5_psi = c5_X[3];
  _SFD_EML_CALL(0,9);
  c5_x = c5_psi;
  c5_b_x = c5_x;
  c5_c_x = c5_b_x;
  c5_b_x = c5_c_x;
  c5_b_x = muDoubleScalarCos(c5_b_x);
  c5_d_x = c5_psi;
  c5_e_x = c5_d_x;
  c5_f_x = c5_e_x;
  c5_e_x = c5_f_x;
  c5_e_x = muDoubleScalarSin(c5_e_x);
  c5_g_x = c5_psi;
  c5_h_x = c5_g_x;
  c5_i_x = c5_h_x;
  c5_h_x = c5_i_x;
  c5_h_x = muDoubleScalarSin(c5_h_x);
  c5_j_x = c5_psi;
  c5_k_x = c5_j_x;
  c5_l_x = c5_k_x;
  c5_k_x = c5_l_x;
  c5_k_x = muDoubleScalarCos(c5_k_x);
  c5_T[0] = c5_b_x;
  c5_T[2] = -c5_e_x;
  c5_T[1] = c5_h_x;
  c5_T[3] = c5_k_x;
  _SFD_EML_CALL(0,11);
  for (c5_i7 = 0; c5_i7 < 4; c5_i7 = c5_i7 + 1) {
    c5_a[c5_i7] = c5_T[c5_i7];
  }

  c5_b[0] = c5_Vx;
  c5_b[1] = c5_Vy;
  c5_eml_scalar_eg(chartInstance);
  c5_eml_scalar_eg(chartInstance);
  for (c5_i8 = 0; c5_i8 < 4; c5_i8 = c5_i8 + 1) {
    c5_A[c5_i8] = c5_a[c5_i8];
  }

  for (c5_i9 = 0; c5_i9 < 2; c5_i9 = c5_i9 + 1) {
    c5_B[c5_i9] = c5_b[c5_i9];
  }

  for (c5_i10 = 0; c5_i10 < 2; c5_i10 = c5_i10 + 1) {
    c5_Vi[c5_i10] = 0.0;
  }

  for (c5_i11 = 0; c5_i11 < 4; c5_i11 = c5_i11 + 1) {
    c5_b_A[c5_i11] = c5_A[c5_i11];
  }

  for (c5_i12 = 0; c5_i12 < 2; c5_i12 = c5_i12 + 1) {
    c5_b_B[c5_i12] = c5_B[c5_i12];
  }

  for (c5_i13 = 0; c5_i13 < 4; c5_i13 = c5_i13 + 1) {
    c5_c_A[c5_i13] = c5_b_A[c5_i13];
  }

  for (c5_i14 = 0; c5_i14 < 2; c5_i14 = c5_i14 + 1) {
    c5_c_B[c5_i14] = c5_b_B[c5_i14];
  }

  for (c5_i15 = 0; c5_i15 < 2; c5_i15 = c5_i15 + 1) {
    c5_Vi[c5_i15] = 0.0;
    c5_i16 = 0;
    for (c5_i17 = 0; c5_i17 < 2; c5_i17 = c5_i17 + 1) {
      c5_Vi[c5_i15] = c5_Vi[c5_i15] + c5_c_A[c5_i16 + c5_i15] * c5_c_B[c5_i17];
      c5_i16 = c5_i16 + 2;
    }
  }

  _SFD_EML_CALL(0,-11);
  sf_debug_symbol_scope_pop();
  for (c5_i18 = 0; c5_i18 < 2; c5_i18 = c5_i18 + 1) {
    (*c5_b_Vi)[c5_i18] = c5_Vi[c5_i18];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG,4);
}

static void init_script_number_translation(uint32_T c5_machineNumber, uint32_T
  c5_chartNumber)
{
}

static void c5_eml_scalar_eg(SFc5_car_modelInstanceStruct *chartInstance)
{
}

static const mxArray *c5_sf_marshall(void *chartInstanceVoid, void *c5_u)
{
  const mxArray *c5_y = NULL;
  int32_T c5_i19;
  real_T c5_b_u[2];
  const mxArray *c5_b_y = NULL;
  SFc5_car_modelInstanceStruct *chartInstance;
  chartInstance = (SFc5_car_modelInstanceStruct *)chartInstanceVoid;
  c5_y = NULL;
  for (c5_i19 = 0; c5_i19 < 2; c5_i19 = c5_i19 + 1) {
    c5_b_u[c5_i19] = (*((real_T (*)[2])c5_u))[c5_i19];
  }

  c5_b_y = NULL;
  sf_mex_assign(&c5_b_y, sf_mex_create("y", &c5_b_u, 0, 0U, 1U, 0U, 1, 2));
  sf_mex_assign(&c5_y, c5_b_y);
  return c5_y;
}

static const mxArray *c5_b_sf_marshall(void *chartInstanceVoid, void *c5_u)
{
  const mxArray *c5_y = NULL;
  int32_T c5_i20;
  real_T c5_b_u[4];
  const mxArray *c5_b_y = NULL;
  SFc5_car_modelInstanceStruct *chartInstance;
  chartInstance = (SFc5_car_modelInstanceStruct *)chartInstanceVoid;
  c5_y = NULL;
  for (c5_i20 = 0; c5_i20 < 4; c5_i20 = c5_i20 + 1) {
    c5_b_u[c5_i20] = (*((real_T (*)[4])c5_u))[c5_i20];
  }

  c5_b_y = NULL;
  sf_mex_assign(&c5_b_y, sf_mex_create("y", &c5_b_u, 0, 0U, 1U, 0U, 1, 4));
  sf_mex_assign(&c5_y, c5_b_y);
  return c5_y;
}

static const mxArray *c5_c_sf_marshall(void *chartInstanceVoid, void *c5_u)
{
  const mxArray *c5_y = NULL;
  real_T c5_b_u;
  const mxArray *c5_b_y = NULL;
  SFc5_car_modelInstanceStruct *chartInstance;
  chartInstance = (SFc5_car_modelInstanceStruct *)chartInstanceVoid;
  c5_y = NULL;
  c5_b_u = *((real_T *)c5_u);
  c5_b_y = NULL;
  sf_mex_assign(&c5_b_y, sf_mex_create("y", &c5_b_u, 0, 0U, 0U, 0U, 0));
  sf_mex_assign(&c5_y, c5_b_y);
  return c5_y;
}

static const mxArray *c5_d_sf_marshall(void *chartInstanceVoid, void *c5_u)
{
  const mxArray *c5_y = NULL;
  int32_T c5_i21;
  int32_T c5_i22;
  int32_T c5_i23;
  real_T c5_b_u[4];
  const mxArray *c5_b_y = NULL;
  SFc5_car_modelInstanceStruct *chartInstance;
  chartInstance = (SFc5_car_modelInstanceStruct *)chartInstanceVoid;
  c5_y = NULL;
  c5_i21 = 0;
  for (c5_i22 = 0; c5_i22 < 2; c5_i22 = c5_i22 + 1) {
    for (c5_i23 = 0; c5_i23 < 2; c5_i23 = c5_i23 + 1) {
      c5_b_u[c5_i23 + c5_i21] = (*((real_T (*)[4])c5_u))[c5_i23 + c5_i21];
    }

    c5_i21 = c5_i21 + 2;
  }

  c5_b_y = NULL;
  sf_mex_assign(&c5_b_y, sf_mex_create("y", &c5_b_u, 0, 0U, 1U, 0U, 2, 2, 2));
  sf_mex_assign(&c5_y, c5_b_y);
  return c5_y;
}

const mxArray *sf_c5_car_model_get_eml_resolved_functions_info(void)
{
  const mxArray *c5_nameCaptureInfo = NULL;
  c5_ResolvedFunctionInfo c5_info[43];
  const mxArray *c5_m0 = NULL;
  int32_T c5_i24;
  c5_ResolvedFunctionInfo *c5_r0;
  c5_nameCaptureInfo = NULL;
  c5_info_helper(c5_info);
  sf_mex_assign(&c5_m0, sf_mex_createstruct("nameCaptureInfo", 1, 43));
  for (c5_i24 = 0; c5_i24 < 43; c5_i24 = c5_i24 + 1) {
    c5_r0 = &c5_info[c5_i24];
    sf_mex_addfield(c5_m0, sf_mex_create("nameCaptureInfo", c5_r0->context, 15,
      0U, 0U, 0U, 2, 1, strlen(c5_r0->context)), "context",
                    "nameCaptureInfo", c5_i24);
    sf_mex_addfield(c5_m0, sf_mex_create("nameCaptureInfo", c5_r0->name, 15, 0U,
      0U, 0U, 2, 1, strlen(c5_r0->name)), "name",
                    "nameCaptureInfo", c5_i24);
    sf_mex_addfield(c5_m0, sf_mex_create("nameCaptureInfo", c5_r0->dominantType,
      15, 0U, 0U, 0U, 2, 1, strlen(c5_r0->dominantType)),
                    "dominantType", "nameCaptureInfo", c5_i24);
    sf_mex_addfield(c5_m0, sf_mex_create("nameCaptureInfo", c5_r0->resolved, 15,
      0U, 0U, 0U, 2, 1, strlen(c5_r0->resolved)), "resolved"
                    , "nameCaptureInfo", c5_i24);
    sf_mex_addfield(c5_m0, sf_mex_create("nameCaptureInfo", &c5_r0->fileLength,
      7, 0U, 0U, 0U, 0), "fileLength", "nameCaptureInfo",
                    c5_i24);
    sf_mex_addfield(c5_m0, sf_mex_create("nameCaptureInfo", &c5_r0->fileTime1, 7,
      0U, 0U, 0U, 0), "fileTime1", "nameCaptureInfo", c5_i24
                    );
    sf_mex_addfield(c5_m0, sf_mex_create("nameCaptureInfo", &c5_r0->fileTime2, 7,
      0U, 0U, 0U, 0), "fileTime2", "nameCaptureInfo", c5_i24
                    );
  }

  sf_mex_assign(&c5_nameCaptureInfo, c5_m0);
  return c5_nameCaptureInfo;
}

static void c5_info_helper(c5_ResolvedFunctionInfo c5_info[43])
{
  c5_info[0].context = "";
  c5_info[0].name = "cos";
  c5_info[0].dominantType = "double";
  c5_info[0].resolved = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/cos.m";
  c5_info[0].fileLength = 324U;
  c5_info[0].fileTime1 = 1203469550U;
  c5_info[0].fileTime2 = 0U;
  c5_info[1].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/cos.m";
  c5_info[1].name = "nargin";
  c5_info[1].dominantType = "";
  c5_info[1].resolved = "[B]nargin";
  c5_info[1].fileLength = 0U;
  c5_info[1].fileTime1 = 0U;
  c5_info[1].fileTime2 = 0U;
  c5_info[2].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/cos.m";
  c5_info[2].name = "gt";
  c5_info[2].dominantType = "double";
  c5_info[2].resolved = "[B]gt";
  c5_info[2].fileLength = 0U;
  c5_info[2].fileTime1 = 0U;
  c5_info[2].fileTime2 = 0U;
  c5_info[3].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/cos.m";
  c5_info[3].name = "isa";
  c5_info[3].dominantType = "double";
  c5_info[3].resolved = "[B]isa";
  c5_info[3].fileLength = 0U;
  c5_info[3].fileTime1 = 0U;
  c5_info[3].fileTime2 = 0U;
  c5_info[4].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/cos.m";
  c5_info[4].name = "eml_scalar_cos";
  c5_info[4].dominantType = "double";
  c5_info[4].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_cos.m";
  c5_info[4].fileLength = 602U;
  c5_info[4].fileTime1 = 1209352386U;
  c5_info[4].fileTime2 = 0U;
  c5_info[5].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_cos.m";
  c5_info[5].name = "isreal";
  c5_info[5].dominantType = "double";
  c5_info[5].resolved = "[B]isreal";
  c5_info[5].fileLength = 0U;
  c5_info[5].fileTime1 = 0U;
  c5_info[5].fileTime2 = 0U;
  c5_info[6].context = "";
  c5_info[6].name = "sin";
  c5_info[6].dominantType = "double";
  c5_info[6].resolved = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/sin.m";
  c5_info[6].fileLength = 324U;
  c5_info[6].fileTime1 = 1203469642U;
  c5_info[6].fileTime2 = 0U;
  c5_info[7].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/sin.m";
  c5_info[7].name = "eml_scalar_sin";
  c5_info[7].dominantType = "double";
  c5_info[7].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_sin.m";
  c5_info[7].fileLength = 601U;
  c5_info[7].fileTime1 = 1209352390U;
  c5_info[7].fileTime2 = 0U;
  c5_info[8].context = "";
  c5_info[8].name = "uminus";
  c5_info[8].dominantType = "double";
  c5_info[8].resolved = "[B]uminus";
  c5_info[8].fileLength = 0U;
  c5_info[8].fileTime1 = 0U;
  c5_info[8].fileTime2 = 0U;
  c5_info[9].context = "";
  c5_info[9].name = "mtimes";
  c5_info[9].dominantType = "double";
  c5_info[9].resolved = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c5_info[9].fileLength = 3425U;
  c5_info[9].fileTime1 = 1250694366U;
  c5_info[9].fileTime2 = 0U;
  c5_info[10].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c5_info[10].name = "isinteger";
  c5_info[10].dominantType = "double";
  c5_info[10].resolved = "[B]isinteger";
  c5_info[10].fileLength = 0U;
  c5_info[10].fileTime1 = 0U;
  c5_info[10].fileTime2 = 0U;
  c5_info[11].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c5_info[11].name = "isscalar";
  c5_info[11].dominantType = "double";
  c5_info[11].resolved = "[B]isscalar";
  c5_info[11].fileLength = 0U;
  c5_info[11].fileTime1 = 0U;
  c5_info[11].fileTime2 = 0U;
  c5_info[12].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c5_info[12].name = "ndims";
  c5_info[12].dominantType = "double";
  c5_info[12].resolved = "[B]ndims";
  c5_info[12].fileLength = 0U;
  c5_info[12].fileTime1 = 0U;
  c5_info[12].fileTime2 = 0U;
  c5_info[13].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c5_info[13].name = "le";
  c5_info[13].dominantType = "double";
  c5_info[13].resolved = "[B]le";
  c5_info[13].fileLength = 0U;
  c5_info[13].fileTime1 = 0U;
  c5_info[13].fileTime2 = 0U;
  c5_info[14].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c5_info[14].name = "size";
  c5_info[14].dominantType = "double";
  c5_info[14].resolved = "[B]size";
  c5_info[14].fileLength = 0U;
  c5_info[14].fileTime1 = 0U;
  c5_info[14].fileTime2 = 0U;
  c5_info[15].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c5_info[15].name = "eq";
  c5_info[15].dominantType = "double";
  c5_info[15].resolved = "[B]eq";
  c5_info[15].fileLength = 0U;
  c5_info[15].fileTime1 = 0U;
  c5_info[15].fileTime2 = 0U;
  c5_info[16].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c5_info[16].name = "not";
  c5_info[16].dominantType = "logical";
  c5_info[16].resolved = "[B]not";
  c5_info[16].fileLength = 0U;
  c5_info[16].fileTime1 = 0U;
  c5_info[16].fileTime2 = 0U;
  c5_info[17].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c5_info[17].name = "strcmp";
  c5_info[17].dominantType = "char";
  c5_info[17].resolved = "[B]strcmp";
  c5_info[17].fileLength = 0U;
  c5_info[17].fileTime1 = 0U;
  c5_info[17].fileTime2 = 0U;
  c5_info[18].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c5_info[18].name = "class";
  c5_info[18].dominantType = "double";
  c5_info[18].resolved = "[B]class";
  c5_info[18].fileLength = 0U;
  c5_info[18].fileTime1 = 0U;
  c5_info[18].fileTime2 = 0U;
  c5_info[19].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c5_info[19].name = "ne";
  c5_info[19].dominantType = "logical";
  c5_info[19].resolved = "[B]ne";
  c5_info[19].fileLength = 0U;
  c5_info[19].fileTime1 = 0U;
  c5_info[19].fileTime2 = 0U;
  c5_info[20].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c5_info[20].name = "eml_index_class";
  c5_info[20].dominantType = "";
  c5_info[20].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c5_info[20].fileLength = 909U;
  c5_info[20].fileTime1 = 1192488382U;
  c5_info[20].fileTime2 = 0U;
  c5_info[21].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c5_info[21].name = "ones";
  c5_info[21].dominantType = "char";
  c5_info[21].resolved = "[B]ones";
  c5_info[21].fileLength = 0U;
  c5_info[21].fileTime1 = 0U;
  c5_info[21].fileTime2 = 0U;
  c5_info[22].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c5_info[22].name = "cast";
  c5_info[22].dominantType = "double";
  c5_info[22].resolved = "[B]cast";
  c5_info[22].fileLength = 0U;
  c5_info[22].fileTime1 = 0U;
  c5_info[22].fileTime2 = 0U;
  c5_info[23].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c5_info[23].name = "eml_scalar_eg";
  c5_info[23].dominantType = "double";
  c5_info[23].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c5_info[23].fileLength = 3068U;
  c5_info[23].fileTime1 = 1240283610U;
  c5_info[23].fileTime2 = 0U;
  c5_info[24].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m/any_enums";
  c5_info[24].name = "false";
  c5_info[24].dominantType = "";
  c5_info[24].resolved = "[B]false";
  c5_info[24].fileLength = 0U;
  c5_info[24].fileTime1 = 0U;
  c5_info[24].fileTime2 = 0U;
  c5_info[25].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c5_info[25].name = "isstruct";
  c5_info[25].dominantType = "double";
  c5_info[25].resolved = "[B]isstruct";
  c5_info[25].fileLength = 0U;
  c5_info[25].fileTime1 = 0U;
  c5_info[25].fileTime2 = 0U;
  c5_info[26].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m/zerosum";
  c5_info[26].name = "plus";
  c5_info[26].dominantType = "double";
  c5_info[26].resolved = "[B]plus";
  c5_info[26].fileLength = 0U;
  c5_info[26].fileTime1 = 0U;
  c5_info[26].fileTime2 = 0U;
  c5_info[27].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c5_info[27].name = "isempty";
  c5_info[27].dominantType = "double";
  c5_info[27].resolved = "[B]isempty";
  c5_info[27].fileLength = 0U;
  c5_info[27].fileTime1 = 0U;
  c5_info[27].fileTime2 = 0U;
  c5_info[28].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c5_info[28].name = "eml_xgemm";
  c5_info[28].dominantType = "int32";
  c5_info[28].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m";
  c5_info[28].fileLength = 3184U;
  c5_info[28].fileTime1 = 1209352452U;
  c5_info[28].fileTime2 = 0U;
  c5_info[29].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c5_info[29].name = "lt";
  c5_info[29].dominantType = "int32";
  c5_info[29].resolved = "[B]lt";
  c5_info[29].fileLength = 0U;
  c5_info[29].fileTime1 = 0U;
  c5_info[29].fileTime2 = 0U;
  c5_info[30].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m/itcount";
  c5_info[30].name = "length";
  c5_info[30].dominantType = "double";
  c5_info[30].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m";
  c5_info[30].fileLength = 326U;
  c5_info[30].fileTime1 = 1226598874U;
  c5_info[30].fileTime2 = 0U;
  c5_info[31].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m/itcount";
  c5_info[31].name = "min";
  c5_info[31].dominantType = "double";
  c5_info[31].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m";
  c5_info[31].fileLength = 362U;
  c5_info[31].fileTime1 = 1244757152U;
  c5_info[31].fileTime2 = 0U;
  c5_info[32].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m";
  c5_info[32].name = "nargout";
  c5_info[32].dominantType = "";
  c5_info[32].resolved = "[B]nargout";
  c5_info[32].fileLength = 0U;
  c5_info[32].fileTime1 = 0U;
  c5_info[32].fileTime2 = 0U;
  c5_info[33].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m";
  c5_info[33].name = "eml_min_or_max";
  c5_info[33].dominantType = "char";
  c5_info[33].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m";
  c5_info[33].fileLength = 9969U;
  c5_info[33].fileTime1 = 1240283606U;
  c5_info[33].fileTime2 = 0U;
  c5_info[34].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m";
  c5_info[34].name = "ischar";
  c5_info[34].dominantType = "char";
  c5_info[34].resolved = "[B]ischar";
  c5_info[34].fileLength = 0U;
  c5_info[34].fileTime1 = 0U;
  c5_info[34].fileTime2 = 0U;
  c5_info[35].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m";
  c5_info[35].name = "isnumeric";
  c5_info[35].dominantType = "double";
  c5_info[35].resolved = "[B]isnumeric";
  c5_info[35].fileLength = 0U;
  c5_info[35].fileTime1 = 0U;
  c5_info[35].fileTime2 = 0U;
  c5_info[36].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m/eml_bin_extremum";
  c5_info[36].name = "islogical";
  c5_info[36].dominantType = "double";
  c5_info[36].resolved = "[B]islogical";
  c5_info[36].fileLength = 0U;
  c5_info[36].fileTime1 = 0U;
  c5_info[36].fileTime2 = 0U;
  c5_info[37].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m/eml_bin_extremum";
  c5_info[37].name = "eml_scalexp_alloc";
  c5_info[37].dominantType = "double";
  c5_info[37].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c5_info[37].fileLength = 808U;
  c5_info[37].fileTime1 = 1230516298U;
  c5_info[37].fileTime2 = 0U;
  c5_info[38].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c5_info[38].name = "minus";
  c5_info[38].dominantType = "double";
  c5_info[38].resolved = "[B]minus";
  c5_info[38].fileLength = 0U;
  c5_info[38].fileTime1 = 0U;
  c5_info[38].fileTime2 = 0U;
  c5_info[39].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c5_info[39].name = "eml_refblas_xgemm";
  c5_info[39].dominantType = "int32";
  c5_info[39].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemm.m";
  c5_info[39].fileLength = 5748U;
  c5_info[39].fileTime1 = 1228115472U;
  c5_info[39].fileTime2 = 0U;
  c5_info[40].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemm.m";
  c5_info[40].name = "eml_index_minus";
  c5_info[40].dominantType = "int32";
  c5_info[40].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c5_info[40].fileLength = 277U;
  c5_info[40].fileTime1 = 1192488384U;
  c5_info[40].fileTime2 = 0U;
  c5_info[41].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemm.m";
  c5_info[41].name = "eml_index_times";
  c5_info[41].dominantType = "int32";
  c5_info[41].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c5_info[41].fileLength = 280U;
  c5_info[41].fileTime1 = 1192488386U;
  c5_info[41].fileTime2 = 0U;
  c5_info[42].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemm.m";
  c5_info[42].name = "eml_index_plus";
  c5_info[42].dominantType = "int32";
  c5_info[42].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c5_info[42].fileLength = 272U;
  c5_info[42].fileTime1 = 1192488384U;
  c5_info[42].fileTime2 = 0U;
}

static const mxArray *c5_e_sf_marshall(void *chartInstanceVoid, void *c5_u)
{
  const mxArray *c5_y = NULL;
  boolean_T c5_b_u;
  const mxArray *c5_b_y = NULL;
  SFc5_car_modelInstanceStruct *chartInstance;
  chartInstance = (SFc5_car_modelInstanceStruct *)chartInstanceVoid;
  c5_y = NULL;
  c5_b_u = *((boolean_T *)c5_u);
  c5_b_y = NULL;
  sf_mex_assign(&c5_b_y, sf_mex_create("y", &c5_b_u, 11, 0U, 0U, 0U, 0));
  sf_mex_assign(&c5_y, c5_b_y);
  return c5_y;
}

static void c5_emlrt_marshallIn(SFc5_car_modelInstanceStruct *chartInstance,
  const mxArray *c5_Vi, const char_T *c5_name, real_T
  c5_y[2])
{
  real_T c5_dv1[2];
  int32_T c5_i25;
  sf_mex_import(c5_name, sf_mex_dup(c5_Vi), &c5_dv1, 1, 0, 0U, 1, 0U, 1, 2);
  for (c5_i25 = 0; c5_i25 < 2; c5_i25 = c5_i25 + 1) {
    c5_y[c5_i25] = c5_dv1[c5_i25];
  }

  sf_mex_destroy(&c5_Vi);
}

static uint8_T c5_b_emlrt_marshallIn(SFc5_car_modelInstanceStruct *chartInstance,
  const mxArray *c5_b_is_active_c5_car_model, const
  char_T *c5_name)
{
  uint8_T c5_y;
  uint8_T c5_u0;
  sf_mex_import(c5_name, sf_mex_dup(c5_b_is_active_c5_car_model), &c5_u0, 1, 3,
                0U, 0, 0U, 0);
  c5_y = c5_u0;
  sf_mex_destroy(&c5_b_is_active_c5_car_model);
  return c5_y;
}

static void init_dsm_address_info(SFc5_car_modelInstanceStruct *chartInstance)
{
}

/* SFunction Glue Code */
void sf_c5_car_model_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(4158254553U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(3103705841U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(2405777386U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(2920293494U);
}

mxArray *sf_c5_car_model_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,4,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateDoubleMatrix(4,1,mxREAL);
    double *pr = mxGetPr(mxChecksum);
    pr[0] = (double)(4158796161U);
    pr[1] = (double)(3969647935U);
    pr[2] = (double)(948014481U);
    pr[3] = (double)(3105398620U);
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
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

static mxArray *sf_get_sim_state_info_c5_car_model(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x2'type','srcId','name','auxInfo'{{M[1],M[5],T\"Vi\",},{M[8],M[0],T\"is_active_c5_car_model\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 2, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c5_car_model_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc5_car_modelInstanceStruct *chartInstance;
    chartInstance = (SFc5_car_modelInstanceStruct *) ((ChartInfoStruct *)
      (ssGetUserData(S)))->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart(_car_modelMachineNumber_,
          5,
          1,
          1,
          2,
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
                                1.0,0,"X",0,(MexFcnForType)c5_b_sf_marshall);
          }

          {
            unsigned int dimVector[1];
            dimVector[0]= 2;
            _SFD_SET_DATA_PROPS(1,2,0,1,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
                                1.0,0,"Vi",0,(MexFcnForType)c5_sf_marshall);
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
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,148);
        _SFD_TRANS_COV_WTS(0,0,0,1,0);
        if (chartAlreadyPresent==0) {
          _SFD_TRANS_COV_MAPS(0,
                              0,NULL,NULL,
                              0,NULL,NULL,
                              1,NULL,NULL,
                              0,NULL,NULL);
        }

        {
          real_T (*c5_X)[4];
          real_T (*c5_Vi)[2];
          c5_Vi = (real_T (*)[2])ssGetOutputPortSignal(chartInstance->S, 1);
          c5_X = (real_T (*)[4])ssGetInputPortSignal(chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, c5_X);
          _SFD_SET_DATA_VALUE_PTR(1U, c5_Vi);
        }
      }
    } else {
      sf_debug_reset_current_state_configuration(_car_modelMachineNumber_,
        chartInstance->chartNumber,chartInstance->instanceNumber);
    }
  }
}

static void sf_opaque_initialize_c5_car_model(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc5_car_modelInstanceStruct*) chartInstanceVar
    )->S,0);
  initialize_params_c5_car_model((SFc5_car_modelInstanceStruct*)
    chartInstanceVar);
  initialize_c5_car_model((SFc5_car_modelInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_enable_c5_car_model(void *chartInstanceVar)
{
  enable_c5_car_model((SFc5_car_modelInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_disable_c5_car_model(void *chartInstanceVar)
{
  disable_c5_car_model((SFc5_car_modelInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_gateway_c5_car_model(void *chartInstanceVar)
{
  sf_c5_car_model((SFc5_car_modelInstanceStruct*) chartInstanceVar);
}

static mxArray* sf_internal_get_sim_state_c5_car_model(SimStruct* S)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c5_car_model((SFc5_car_modelInstanceStruct*)
    chartInfo->chartInstance);         /* raw sim ctx */
  prhs[3] = sf_get_sim_state_info_c5_car_model();/* state var info */
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

static void sf_internal_set_sim_state_c5_car_model(SimStruct* S, const mxArray
  *st)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = mxDuplicateArray(st);      /* high level simctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c5_car_model();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c5_car_model((SFc5_car_modelInstanceStruct*)
    chartInfo->chartInstance, mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static mxArray* sf_opaque_get_sim_state_c5_car_model(SimStruct* S)
{
  return sf_internal_get_sim_state_c5_car_model(S);
}

static void sf_opaque_set_sim_state_c5_car_model(SimStruct* S, const mxArray *st)
{
  sf_internal_set_sim_state_c5_car_model(S, st);
}

static void sf_opaque_terminate_c5_car_model(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc5_car_modelInstanceStruct*) chartInstanceVar)->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
    }

    finalize_c5_car_model((SFc5_car_modelInstanceStruct*) chartInstanceVar);
    free((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c5_car_model(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c5_car_model((SFc5_car_modelInstanceStruct*)
      (((ChartInfoStruct *)ssGetUserData(S))->chartInstance));
  }
}

static void mdlSetWorkWidths_c5_car_model(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable("car_model","car_model",5);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop("car_model","car_model",5,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop("car_model","car_model",5,
      "gatewayCannotBeInlinedMultipleTimes"));
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,"car_model","car_model",5,1);
      sf_mark_chart_reusable_outputs(S,"car_model","car_model",5,1);
    }

    sf_set_rtw_dwork_info(S,"car_model","car_model",5);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
    ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  }

  ssSetChecksum0(S,(3605814388U));
  ssSetChecksum1(S,(4166520733U));
  ssSetChecksum2(S,(2613154650U));
  ssSetChecksum3(S,(4127610077U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
}

static void mdlRTW_c5_car_model(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    sf_write_symbol_mapping(S, "car_model", "car_model",5);
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c5_car_model(SimStruct *S)
{
  SFc5_car_modelInstanceStruct *chartInstance;
  chartInstance = (SFc5_car_modelInstanceStruct *)malloc(sizeof
    (SFc5_car_modelInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc5_car_modelInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway = sf_opaque_gateway_c5_car_model;
  chartInstance->chartInfo.initializeChart = sf_opaque_initialize_c5_car_model;
  chartInstance->chartInfo.terminateChart = sf_opaque_terminate_c5_car_model;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c5_car_model;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c5_car_model;
  chartInstance->chartInfo.getSimState = sf_opaque_get_sim_state_c5_car_model;
  chartInstance->chartInfo.setSimState = sf_opaque_set_sim_state_c5_car_model;
  chartInstance->chartInfo.getSimStateInfo = sf_get_sim_state_info_c5_car_model;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c5_car_model;
  chartInstance->chartInfo.mdlStart = mdlStart_c5_car_model;
  chartInstance->chartInfo.mdlSetWorkWidths = mdlSetWorkWidths_c5_car_model;
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

void c5_car_model_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c5_car_model(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c5_car_model(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c5_car_model(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c5_car_model_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
