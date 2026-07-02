/* Include files */

#include "updated_target_driver_2026_withVariants_sfun.h"
#include "c5_updated_target_driver_2026_withVariants.h"
#define _SF_MEX_LISTEN_FOR_CTRL_C(S)   sf_mex_listen_for_ctrl_c(S);
#ifdef utFree
#undef utFree
#endif

#ifdef utMalloc
#undef utMalloc
#endif

#ifdef __cplusplus

extern "C" void *utMalloc(size_t size);
extern "C" void utFree(void*);

#else

extern void *utMalloc(size_t size);
extern void utFree(void*);

#endif

/* Forward Declarations */

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static real_T _sfTime_;
static emlrtRSInfo c5_emlrtRSI = { 4,  /* lineNo */
  "Variant Subsystem/Simulation/MATLAB Function",/* fcnName */
  "#updated_target_driver_2026_withVariants:1321"/* pathName */
};

/* Function Declarations */
static void initialize_c5_updated_target_driver_2026_withVariants
  (SFc5_updated_target_driver_2026_withVariantsInstanceStruct *chartInstance);
static void initialize_params_c5_updated_target_driver_2026_withVariants
  (SFc5_updated_target_driver_2026_withVariantsInstanceStruct *chartInstance);
static void mdl_start_c5_updated_target_driver_2026_withVariants
  (SFc5_updated_target_driver_2026_withVariantsInstanceStruct *chartInstance);
static void mdl_terminate_c5_updated_target_driver_2026_withVariants
  (SFc5_updated_target_driver_2026_withVariantsInstanceStruct *chartInstance);
static void
  mdl_setup_runtime_resources_c5_updated_target_driver_2026_withVariants
  (SFc5_updated_target_driver_2026_withVariantsInstanceStruct *chartInstance);
static void
  mdl_cleanup_runtime_resources_c5_updated_target_driver_2026_withVariants
  (SFc5_updated_target_driver_2026_withVariantsInstanceStruct *chartInstance);
static void enable_c5_updated_target_driver_2026_withVariants
  (SFc5_updated_target_driver_2026_withVariantsInstanceStruct *chartInstance);
static void disable_c5_updated_target_driver_2026_withVariants
  (SFc5_updated_target_driver_2026_withVariantsInstanceStruct *chartInstance);
static void sf_gateway_c5_updated_target_driver_2026_withVariants
  (SFc5_updated_target_driver_2026_withVariantsInstanceStruct *chartInstance);
static void ext_mode_exec_c5_updated_target_driver_2026_withVariants
  (SFc5_updated_target_driver_2026_withVariantsInstanceStruct *chartInstance);
static void c5_update_jit_animation_c5_updated_target_driver_2026_withVarian
  (SFc5_updated_target_driver_2026_withVariantsInstanceStruct *chartInstance);
static void c5_do_animation_call_c5_updated_target_driver_2026_withVariants
  (SFc5_updated_target_driver_2026_withVariantsInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c5_updated_target_driver_2026_withVariants
  (SFc5_updated_target_driver_2026_withVariantsInstanceStruct *chartInstance);
static void set_sim_state_c5_updated_target_driver_2026_withVariants
  (SFc5_updated_target_driver_2026_withVariantsInstanceStruct *chartInstance,
   const mxArray *c5_st);
static real_T c5_emlrt_marshallIn
  (SFc5_updated_target_driver_2026_withVariantsInstanceStruct *chartInstance,
   const mxArray *c5_nullptr, const char_T *c5_identifier);
static real_T c5_b_emlrt_marshallIn
  (SFc5_updated_target_driver_2026_withVariantsInstanceStruct *chartInstance,
   const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId);
static void init_dsm_address_info
  (SFc5_updated_target_driver_2026_withVariantsInstanceStruct *chartInstance);
static void init_simulink_io_address
  (SFc5_updated_target_driver_2026_withVariantsInstanceStruct *chartInstance);

/* Function Definitions */
static void initialize_c5_updated_target_driver_2026_withVariants
  (SFc5_updated_target_driver_2026_withVariantsInstanceStruct *chartInstance)
{
  sim_mode_is_external(chartInstance->S);
  chartInstance->c5_doneDoubleBufferReInit = false;
  chartInstance->c5_sfEvent = CALL_EVENT;
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void initialize_params_c5_updated_target_driver_2026_withVariants
  (SFc5_updated_target_driver_2026_withVariantsInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void mdl_start_c5_updated_target_driver_2026_withVariants
  (SFc5_updated_target_driver_2026_withVariantsInstanceStruct *chartInstance)
{
  sim_mode_is_external(chartInstance->S);
}

static void mdl_terminate_c5_updated_target_driver_2026_withVariants
  (SFc5_updated_target_driver_2026_withVariantsInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void
  mdl_setup_runtime_resources_c5_updated_target_driver_2026_withVariants
  (SFc5_updated_target_driver_2026_withVariantsInstanceStruct *chartInstance)
{
  static const uint32_T c5_decisionTxtEndIdx = 0U;
  static const uint32_T c5_decisionTxtStartIdx = 0U;
  sfSetAnimationVectors(chartInstance->S, &chartInstance->c5_JITStateAnimation[0],
                        &chartInstance->c5_JITTransitionAnimation[0]);
  covrtCreateStateflowInstanceData(chartInstance->c5_covrtInstance, 1U, 0U, 1U,
    186U);
  covrtChartInitFcn(chartInstance->c5_covrtInstance, 0U, false, false, false);
  covrtStateInitFcn(chartInstance->c5_covrtInstance, 0U, 0U, false, false, false,
                    0U, &c5_decisionTxtStartIdx, &c5_decisionTxtEndIdx);
  covrtTransInitFcn(chartInstance->c5_covrtInstance, 0U, 0, NULL, NULL, 0U, NULL);
  covrtEmlInitFcn(chartInstance->c5_covrtInstance, "", 4U, 0U, 1U, 0U, 0U, 0U,
                  0U, 0U, 0U, 0U, 0U, 0U);
  covrtEmlFcnInitFcn(chartInstance->c5_covrtInstance, 4U, 0U, 0U,
                     "c5_updated_target_driver_2026_withVariants", 0, -1, 363);
  covrtEmlInitFcn(chartInstance->c5_covrtInstance,
                  "/Users/franklinho/robobee_simulator/simulink/robobee_tcp_step_codegen.m",
                  14U, 0U, 1U, 0U, 2U, 0U, 0U, 0U, 0U, 0U, 0U, 0U);
  covrtEmlFcnInitFcn(chartInstance->c5_covrtInstance, 14U, 0U, 0U,
                     "robobee_tcp_step_codegen", 0, -1, 1262);
  covrtEmlIfInitFcn(chartInstance->c5_covrtInstance, 14U, 0U, 0U, 480, 505, 668,
                    1258, false);
  covrtEmlIfInitFcn(chartInstance->c5_covrtInstance, 14U, 0U, 1U, 1175, 1189, -1,
                    1254, false);
  covrtEmlRelationalInitFcn(chartInstance->c5_covrtInstance, 14U, 0U, 0U, 1178,
    1189, -1, 1U);
}

static void
  mdl_cleanup_runtime_resources_c5_updated_target_driver_2026_withVariants
  (SFc5_updated_target_driver_2026_withVariantsInstanceStruct *chartInstance)
{
  covrtDeleteStateflowInstanceData(chartInstance->c5_covrtInstance);
}

static void enable_c5_updated_target_driver_2026_withVariants
  (SFc5_updated_target_driver_2026_withVariantsInstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c5_updated_target_driver_2026_withVariants
  (SFc5_updated_target_driver_2026_withVariantsInstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void sf_gateway_c5_updated_target_driver_2026_withVariants
  (SFc5_updated_target_driver_2026_withVariantsInstanceStruct *chartInstance)
{
  real_T c5_pose[7];
  real_T c5_b_alpha_rad;
  real_T c5_b_beta_rad;
  real_T c5_b_bias_voltage_v;
  real_T c5_b_dt_s;
  real_T c5_b_gamma_rad;
  real_T c5_b_left_voltage_v;
  real_T c5_b_right_voltage_v;
  real_T c5_b_time;
  real_T c5_b_x;
  real_T c5_b_y;
  real_T c5_b_z;
  real_T c5_c_bias_voltage_v;
  real_T c5_c_dt_s;
  real_T c5_c_left_voltage_v;
  real_T c5_c_right_voltage_v;
  int32_T c5_i;
  int32_T c5_status;
  covrtSigUpdateFcn(chartInstance->c5_covrtInstance, 3U,
                    *chartInstance->c5_bias_voltage_v);
  covrtSigUpdateFcn(chartInstance->c5_covrtInstance, 2U,
                    *chartInstance->c5_right_voltage_v);
  covrtSigUpdateFcn(chartInstance->c5_covrtInstance, 1U,
                    *chartInstance->c5_left_voltage_v);
  covrtSigUpdateFcn(chartInstance->c5_covrtInstance, 0U, *chartInstance->c5_dt_s);
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c5_JITTransitionAnimation[0] = 0U;
  chartInstance->c5_sfEvent = CALL_EVENT;
  c5_b_dt_s = *chartInstance->c5_dt_s;
  c5_b_left_voltage_v = *chartInstance->c5_left_voltage_v;
  c5_b_right_voltage_v = *chartInstance->c5_right_voltage_v;
  c5_b_bias_voltage_v = *chartInstance->c5_bias_voltage_v;
  covrtEmlFcnEval(chartInstance->c5_covrtInstance, 4U, 0, 0);
  c5_c_dt_s = c5_b_dt_s;
  c5_c_left_voltage_v = c5_b_left_voltage_v;
  c5_c_right_voltage_v = c5_b_right_voltage_v;
  c5_c_bias_voltage_v = c5_b_bias_voltage_v;
  covrtEmlFcnEval(chartInstance->c5_covrtInstance, 14U, 0, 0);
  covrtEmlIfEval(chartInstance->c5_covrtInstance, 14U, 0, 0, false);
  c5_status = robobee_tcp_step_c(c5_c_dt_s, c5_c_left_voltage_v,
    c5_c_right_voltage_v, c5_c_bias_voltage_v, &c5_pose[0]);
  if (covrtEmlIfEval(chartInstance->c5_covrtInstance, 14U, 0, 1,
                     covrtRelationalopUpdateFcn(chartInstance->c5_covrtInstance,
        14U, 0U, 0U, (real_T)c5_status, 0.0, -1, 1U, c5_status != 0))) {
    for (c5_i = 0; c5_i < 7; c5_i++) {
      c5_pose[c5_i] = rtNaN;
    }

    c5_pose[0] = (real_T)c5_status;
  }

  c5_b_time = c5_pose[0];
  c5_b_x = c5_pose[1];
  c5_b_y = c5_pose[2];
  c5_b_z = c5_pose[3];
  c5_b_alpha_rad = c5_pose[4];
  c5_b_beta_rad = c5_pose[5];
  c5_b_gamma_rad = c5_pose[6];
  *chartInstance->c5_time = c5_b_time;
  *chartInstance->c5_x = c5_b_x;
  *chartInstance->c5_y = c5_b_y;
  *chartInstance->c5_z = c5_b_z;
  *chartInstance->c5_alpha_rad = c5_b_alpha_rad;
  *chartInstance->c5_beta_rad = c5_b_beta_rad;
  *chartInstance->c5_gamma_rad = c5_b_gamma_rad;
  covrtSigUpdateFcn(chartInstance->c5_covrtInstance, 4U, *chartInstance->c5_time);
  covrtSigUpdateFcn(chartInstance->c5_covrtInstance, 5U, *chartInstance->c5_x);
  covrtSigUpdateFcn(chartInstance->c5_covrtInstance, 6U, *chartInstance->c5_y);
  covrtSigUpdateFcn(chartInstance->c5_covrtInstance, 7U, *chartInstance->c5_z);
  covrtSigUpdateFcn(chartInstance->c5_covrtInstance, 8U,
                    *chartInstance->c5_alpha_rad);
  covrtSigUpdateFcn(chartInstance->c5_covrtInstance, 9U,
                    *chartInstance->c5_beta_rad);
  covrtSigUpdateFcn(chartInstance->c5_covrtInstance, 10U,
                    *chartInstance->c5_gamma_rad);
}

static void ext_mode_exec_c5_updated_target_driver_2026_withVariants
  (SFc5_updated_target_driver_2026_withVariantsInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c5_update_jit_animation_c5_updated_target_driver_2026_withVarian
  (SFc5_updated_target_driver_2026_withVariantsInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c5_do_animation_call_c5_updated_target_driver_2026_withVariants
  (SFc5_updated_target_driver_2026_withVariantsInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static const mxArray *get_sim_state_c5_updated_target_driver_2026_withVariants
  (SFc5_updated_target_driver_2026_withVariantsInstanceStruct *chartInstance)
{
  const mxArray *c5_b_y = NULL;
  const mxArray *c5_c_y = NULL;
  const mxArray *c5_d_y = NULL;
  const mxArray *c5_e_y = NULL;
  const mxArray *c5_f_y = NULL;
  const mxArray *c5_g_y = NULL;
  const mxArray *c5_h_y = NULL;
  const mxArray *c5_i_y = NULL;
  const mxArray *c5_st;
  c5_st = NULL;
  c5_st = NULL;
  c5_b_y = NULL;
  sf_mex_assign(&c5_b_y, sf_mex_createcellmatrix(7, 1), false);
  c5_c_y = NULL;
  sf_mex_assign(&c5_c_y, sf_mex_create("y", chartInstance->c5_alpha_rad, 0, 0U,
    0, 0U, 0), false);
  sf_mex_setcell(c5_b_y, 0, c5_c_y);
  c5_d_y = NULL;
  sf_mex_assign(&c5_d_y, sf_mex_create("y", chartInstance->c5_beta_rad, 0, 0U, 0,
    0U, 0), false);
  sf_mex_setcell(c5_b_y, 1, c5_d_y);
  c5_e_y = NULL;
  sf_mex_assign(&c5_e_y, sf_mex_create("y", chartInstance->c5_gamma_rad, 0, 0U,
    0, 0U, 0), false);
  sf_mex_setcell(c5_b_y, 2, c5_e_y);
  c5_f_y = NULL;
  sf_mex_assign(&c5_f_y, sf_mex_create("y", chartInstance->c5_time, 0, 0U, 0, 0U,
    0), false);
  sf_mex_setcell(c5_b_y, 3, c5_f_y);
  c5_g_y = NULL;
  sf_mex_assign(&c5_g_y, sf_mex_create("y", chartInstance->c5_x, 0, 0U, 0, 0U, 0),
                false);
  sf_mex_setcell(c5_b_y, 4, c5_g_y);
  c5_h_y = NULL;
  sf_mex_assign(&c5_h_y, sf_mex_create("y", chartInstance->c5_y, 0, 0U, 0, 0U, 0),
                false);
  sf_mex_setcell(c5_b_y, 5, c5_h_y);
  c5_i_y = NULL;
  sf_mex_assign(&c5_i_y, sf_mex_create("y", chartInstance->c5_z, 0, 0U, 0, 0U, 0),
                false);
  sf_mex_setcell(c5_b_y, 6, c5_i_y);
  sf_mex_assign(&c5_st, c5_b_y, false);
  return c5_st;
}

static void set_sim_state_c5_updated_target_driver_2026_withVariants
  (SFc5_updated_target_driver_2026_withVariantsInstanceStruct *chartInstance,
   const mxArray *c5_st)
{
  const mxArray *c5_u;
  chartInstance->c5_doneDoubleBufferReInit = true;
  c5_u = sf_mex_dup(c5_st);
  *chartInstance->c5_alpha_rad = c5_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c5_u, 0)), "alpha_rad");
  *chartInstance->c5_beta_rad = c5_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c5_u, 1)), "beta_rad");
  *chartInstance->c5_gamma_rad = c5_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c5_u, 2)), "gamma_rad");
  *chartInstance->c5_time = c5_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c5_u, 3)), "time");
  *chartInstance->c5_x = c5_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c5_u, 4)), "x");
  *chartInstance->c5_y = c5_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c5_u, 5)), "y");
  *chartInstance->c5_z = c5_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c5_u, 6)), "z");
  sf_mex_destroy(&c5_u);
  sf_mex_destroy(&c5_st);
}

const mxArray
  *sf_c5_updated_target_driver_2026_withVariants_get_eml_resolved_functions_info
  (void)
{
  const mxArray *c5_nameCaptureInfo = NULL;
  const char_T *c5_data[4] = {
    "789cc553cb4ac340149d482d6eaa5d892b3fc108a2c5a568a50a16a1352022318f5b33263313261369bfa2bfe2fff8232e258fc90b8688c17a37670e67e69e7b"
    "0f0cd26eee3484d02ecaeab89fe120e7c31cb750bd9abaa64059dba8577b27f5758e0ea302962223d42250bc7419c1d4a262be0a01718858f00e6eaa2c700073",
    "4c605625d38491eb8a5490444ace971e38fe2c26887b5139615025451e2f8a7d7b2d7934ab9947f39ef45bfed24ff63f68f1933a6736b3014ce184662420341d"
    "e6c22bd0ae7b4bdfbe728e4c71596c0750eefdd1d1ef56e957d79fc6cffa43043cd217dca27e80a9c774194684491c5882713d3d61eaebaa9c8e48d6af2dafbd",
    "1fceaffa3703b493a231be7adba4dfd7e7e1f926fd64fd975fd77fb7aff01b36f41377ea4e8c33cbe6a7c628c68fab11151793728efb169fb6399082ff75ff6f"
    "8d036314", "" };

  c5_nameCaptureInfo = NULL;
  emlrtNameCaptureMxArrayR2016a(&c5_data[0], 1608U, &c5_nameCaptureInfo);
  return c5_nameCaptureInfo;
}

static real_T c5_emlrt_marshallIn
  (SFc5_updated_target_driver_2026_withVariantsInstanceStruct *chartInstance,
   const mxArray *c5_nullptr, const char_T *c5_identifier)
{
  emlrtMsgIdentifier c5_thisId;
  real_T c5_b_y;
  c5_thisId.fIdentifier = (const char_T *)c5_identifier;
  c5_thisId.fParent = NULL;
  c5_thisId.bParentIsCell = false;
  c5_b_y = c5_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c5_nullptr),
    &c5_thisId);
  sf_mex_destroy(&c5_nullptr);
  return c5_b_y;
}

static real_T c5_b_emlrt_marshallIn
  (SFc5_updated_target_driver_2026_withVariantsInstanceStruct *chartInstance,
   const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId)
{
  real_T c5_b_y;
  real_T c5_d;
  (void)chartInstance;
  sf_mex_import(c5_parentId, sf_mex_dup(c5_u), &c5_d, 1, 0, 0U, 0, 0U, 0);
  c5_b_y = c5_d;
  sf_mex_destroy(&c5_u);
  return c5_b_y;
}

static void init_dsm_address_info
  (SFc5_updated_target_driver_2026_withVariantsInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void init_simulink_io_address
  (SFc5_updated_target_driver_2026_withVariantsInstanceStruct *chartInstance)
{
  chartInstance->c5_covrtInstance = (CovrtStateflowInstance *)
    sfrtGetCovrtInstance(chartInstance->S);
  chartInstance->c5_fEmlrtCtx = (void *)sfrtGetEmlrtCtx(chartInstance->S);
  chartInstance->c5_time = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 1);
  chartInstance->c5_dt_s = (real_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 0);
  chartInstance->c5_left_voltage_v = (real_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 1);
  chartInstance->c5_right_voltage_v = (real_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 2);
  chartInstance->c5_bias_voltage_v = (real_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 3);
  chartInstance->c5_x = (real_T *)ssGetOutputPortSignal_wrapper(chartInstance->S,
    2);
  chartInstance->c5_y = (real_T *)ssGetOutputPortSignal_wrapper(chartInstance->S,
    3);
  chartInstance->c5_z = (real_T *)ssGetOutputPortSignal_wrapper(chartInstance->S,
    4);
  chartInstance->c5_alpha_rad = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 5);
  chartInstance->c5_beta_rad = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 6);
  chartInstance->c5_gamma_rad = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 7);
}

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* SFunction Glue Code */
void sf_c5_updated_target_driver_2026_withVariants_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(478270686U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(3643793626U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3549134544U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(1257854415U);
}

mxArray *sf_c5_updated_target_driver_2026_withVariants_third_party_uses_info
  (void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray *sf_c5_updated_target_driver_2026_withVariants_jit_fallback_info(void)
{
  const char *infoFields[] = { "fallbackType", "fallbackReason",
    "hiddenFallbackType", "hiddenFallbackReason", "incompatibleSymbol" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 5, infoFields);
  mxArray *fallbackType = mxCreateString("late");
  mxArray *fallbackReason = mxCreateString("ir_function_calls");
  mxArray *hiddenFallbackType = mxCreateString("");
  mxArray *hiddenFallbackReason = mxCreateString("");
  mxArray *incompatibleSymbol = mxCreateString("robobee_tcp_step_c");
  mxSetField(mxInfo, 0, infoFields[0], fallbackType);
  mxSetField(mxInfo, 0, infoFields[1], fallbackReason);
  mxSetField(mxInfo, 0, infoFields[2], hiddenFallbackType);
  mxSetField(mxInfo, 0, infoFields[3], hiddenFallbackReason);
  mxSetField(mxInfo, 0, infoFields[4], incompatibleSymbol);
  return mxInfo;
}

mxArray *sf_c5_updated_target_driver_2026_withVariants_updateBuildInfo_args_info
  (void)
{
  mxArray *mxBIArgs = sf_mex_decode(
    "eNpjYPT0ZQACPiCewcTAwAakOYCYkQECWJH4TEjiIPUcjKSpd2BAqGfBop4fSb0AlJ+YkuKZl5x"
    "TmpIakFiSUQw2ZwIDfnsZ0exNIGCvDpq9IL5+aHFqUbF+WlFiXnZOZl5Gvn5RflJ+UmpqfHFmbm"
    "lOYkl+kT6YlZmXDbMngsrhwYfmLj5IeATnlxYlp7pl5qQWQ8QeMJBmrwcBe0XQ7AXxYb4vSS6IT"
    "87JTM0r0UumZ/gCAATfPjE="
    );
  return mxBIArgs;
}

static const mxArray
  *sf_get_sim_state_info_c5_updated_target_driver_2026_withVariants(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  mxArray *mxVarInfo = sf_mex_decode(
    "eNpjYPT0ZQACPiA+wMrAwAakOYCYiQECWKF8RiBmh9IQcRa4uAIQl1QWpILEi4uSPVOAdF5iLpi"
    "fWFrhmZeWDzbfggFhPhsW8xmRzOeEikPAB3vK9Js4gPQ7IOlnwaKfE0m/AJSfmFOQkRhflJgCFe"
    "dDogfOP6YO6Pqx+YcDzT8gflJqCdg7g8MfZmTHS3pibu7gixc1sH8MCPiHBcU/LAwlmbmpg8P9h"
    "kS5nxHF/YwMFYMm/I3Icn/loHG/MVnur0JyPwCpXS1p"
    );
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c5_updated_target_driver_2026_withVariants_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static const char* sf_get_instance_specialization(void)
{
  return "sgKu9p9ZZ6akBpbtsPiTOID";
}

static void sf_opaque_initialize_c5_updated_target_driver_2026_withVariants(void
  *chartInstanceVar)
{
  initialize_params_c5_updated_target_driver_2026_withVariants
    ((SFc5_updated_target_driver_2026_withVariantsInstanceStruct*)
     chartInstanceVar);
  initialize_c5_updated_target_driver_2026_withVariants
    ((SFc5_updated_target_driver_2026_withVariantsInstanceStruct*)
     chartInstanceVar);
}

static void sf_opaque_enable_c5_updated_target_driver_2026_withVariants(void
  *chartInstanceVar)
{
  enable_c5_updated_target_driver_2026_withVariants
    ((SFc5_updated_target_driver_2026_withVariantsInstanceStruct*)
     chartInstanceVar);
}

static void sf_opaque_disable_c5_updated_target_driver_2026_withVariants(void
  *chartInstanceVar)
{
  disable_c5_updated_target_driver_2026_withVariants
    ((SFc5_updated_target_driver_2026_withVariantsInstanceStruct*)
     chartInstanceVar);
}

static void sf_opaque_gateway_c5_updated_target_driver_2026_withVariants(void
  *chartInstanceVar)
{
  sf_gateway_c5_updated_target_driver_2026_withVariants
    ((SFc5_updated_target_driver_2026_withVariantsInstanceStruct*)
     chartInstanceVar);
}

static const mxArray*
  sf_opaque_get_sim_state_c5_updated_target_driver_2026_withVariants(SimStruct*
  S)
{
  return get_sim_state_c5_updated_target_driver_2026_withVariants
    ((SFc5_updated_target_driver_2026_withVariantsInstanceStruct *)
     sf_get_chart_instance_ptr(S));    /* raw sim ctx */
}

static void sf_opaque_set_sim_state_c5_updated_target_driver_2026_withVariants
  (SimStruct* S, const mxArray *st)
{
  set_sim_state_c5_updated_target_driver_2026_withVariants
    ((SFc5_updated_target_driver_2026_withVariantsInstanceStruct*)
     sf_get_chart_instance_ptr(S), st);
}

static void
  sf_opaque_cleanup_runtime_resources_c5_updated_target_driver_2026_withVariants
  (void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc5_updated_target_driver_2026_withVariantsInstanceStruct*)
                    chartInstanceVar)->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_updated_target_driver_2026_withVariants_optimization_info();
    }

    mdl_cleanup_runtime_resources_c5_updated_target_driver_2026_withVariants
      ((SFc5_updated_target_driver_2026_withVariantsInstanceStruct*)
       chartInstanceVar);
    utFree(chartInstanceVar);
    if (ssGetUserData(S)!= NULL) {
      sf_free_ChartRunTimeInfo(S);
    }

    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_mdl_start_c5_updated_target_driver_2026_withVariants(void *
  chartInstanceVar)
{
  mdl_start_c5_updated_target_driver_2026_withVariants
    ((SFc5_updated_target_driver_2026_withVariantsInstanceStruct*)
     chartInstanceVar);
  if (chartInstanceVar) {
    sf_reset_warnings_ChartRunTimeInfo
      (((SFc5_updated_target_driver_2026_withVariantsInstanceStruct*)
        chartInstanceVar)->S);
  }
}

static void sf_opaque_mdl_terminate_c5_updated_target_driver_2026_withVariants
  (void *chartInstanceVar)
{
  mdl_terminate_c5_updated_target_driver_2026_withVariants
    ((SFc5_updated_target_driver_2026_withVariantsInstanceStruct*)
     chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c5_updated_target_driver_2026_withVariants
  (SimStruct *S)
{
  mdlProcessParamsCommon(S);
  if (sf_machine_global_initializer_called()) {
    initialize_params_c5_updated_target_driver_2026_withVariants
      ((SFc5_updated_target_driver_2026_withVariantsInstanceStruct*)
       sf_get_chart_instance_ptr(S));
  }
}

const char* sf_c5_updated_target_driver_2026_withVariants_get_post_codegen_info
  (void)
{
  int i;
  const char* encStrCodegen [26] = {
    "eNrdWUtv41QUdkqamaLpABuYBWJYIYQEHVramUoI2uZBA31EJC2iIxRu7JP4Evvacx9pMit+ANJ",
    "s2LBA4o8gsWSNhITE/AJWsJsl5zpO6nEzie1oaMSV3ObY+e55P65j5KqHBq6beP102zAK+P86Xk",
    "vGcC2HdC5yDe/njQ9DuvaiYZg24bLK2p6RfpmeBR1gddVu035KLFNujXDiigx8GXHhcxCeoyT1W",
    "DrhKWsDB2biBr7HZSq+grrKoaxbUczUnMUXNjXtuu0px9rDDYl1zJzBs/j6StaQY4lyMGUFwJI2",
    "91THrjikM90KXJ4XbTC7QrmpbSVA1pWvVRWHypHUd6DcB7PKhCRoBTFD37okEoqynzpCqKiP0J7",
    "rO5Sw5La2iaiDj9Eh4cS38O+xkmi9hHyRX4syIj1OiVN2naKO8ITYmoNyHmJYO6ntLF1ZgpbqdC",
    "jraOty5QJD/TFOEtiqXfR6wEkHjlnKHNTalfuBg8dxmTwHqzosM+WgcodeEZmwAd9yDy0kMvKtm",
    "KxIHEekwzY8/wB64AT8S0SSDNgh/xRgIajV8E4J11mQMpMUow8UhNiixyya3MO9GCoouEdYPBPA",
    "qatDCiw081j08UazYlIJ6blFTKPSwUFCfpexVSaBt4kJiWsfJ1QAChzEVUq+FhWk5Wg0WkkGWib",
    "eAVhmqCHaipXOPd5FG6ctshe20pmQDg1WB0ogISgYZYzuU+KohDK7AiucDo8TgRUrHV/E6vzJBD",
    "aJaYOl+wl14BCE3kAkrs/YR3ZR2x6VgxIIk1M/aSYpARY2Em2lxsCHE9Zl3jmrcM+thxPBlLgCw",
    "KpBOMO2sIdtiQ8qKHwyqTk8aOjIyjKeuUQ6pKVj4xNg2Fm0rroTEhOzqsxwdEOB5sHW6UNs7UxQ",
    "IXGcGpSDHLCCefRWLt08+kpIb4xzqSoaHD1FsA0znJR2W8GwBw3qQnCjTnCmGJLh0nzvGBd8V5a",
    "m813CT7mMOGNO3NcRXH6Cfd6J4F4OaXOzqYJZyGpKwjsgmxZaCHhz/c76VvOcSjss0mKyPW4k8M",
    "MI9+sL6fy3GtKPgjMBlu2+vBjULzLBczH+mdQphKGNHu1hvOjVpkNf1qPEkabcSuTRmNCP9OegM",
    "dSxMXA7klBOlLhkh7i9Rytq7xFuf4afbsVwmuZey2sBNKXpNzEz/GZ4SNL73ZuxXyG2XyGwmsJ8",
    "GMrz7Qz8pzG8pu+Xv1o7EcDFWpsT1sViZXtrIyGDA42elNdGR5u1Z8n/nnthl6geL82Ij9Xw/mm",
    "59M08+Cd/vrE9Dz4aD/PiZ8XFqzE/aHrDOrL2T7dIi2+e3lX0y8FdJnf3h/u9HdkvN2G/aHxm+X",
    "5W3LT6lr9U3/JGDpfG7URwq8Z0XCG00W9Lf/3+c8H84ejxj/989/j7R/PUc7+Qrn7dDOnXR+ep8",
    "cTZuzSUJfH/azH/a1p0PlPb/vbZ2Rbp7vktKWq0cVwtBfv9sjxd3msxeUf338RL6mqq9+dm1YqU",
    "XKKGZ/Z4vBdm2GPlqXj/++P58B/sxONhkr1WYvbSNHF8mzQ5sSbE8dXps7mTpI5fj+mj6RbIQJ3",
    "F0GMrs186xHUXzy9v7czq87lI7RnqkzckzhKLIf/7ieTPPSV/zugvjP3XM8k/WBj5NzLJ/zAm/6",
    "KfP67qvHOVciY59+Qz4q7Nec76r3Dz6ve859NF+37W8+Ki6WFMqU83JuiVi+27qHr9YaSb72+H9",
    "Efjd/FFmzrWhLeB4eMDIO1JT/8n8f0kpf1G55mytl/4Y+LZxi4jzkDQ4cuW0e0a179jjR9xIGLy",
    "O9ar6EOT5oxJfX45lt+aPmlU3r03Rz/7F2mBh40=",
    ""
  };

  static char newstr [1841] = "";
  newstr[0] = '\0';
  for (i = 0; i < 26; i++) {
    strcat(newstr, encStrCodegen[i]);
  }

  return newstr;
}

static void mdlSetWorkWidths_c5_updated_target_driver_2026_withVariants
  (SimStruct *S)
{
  const char* newstr =
    sf_c5_updated_target_driver_2026_withVariants_get_post_codegen_info();
  sf_set_work_widths(S, newstr);
  ssSetChecksum0(S,(3588883154U));
  ssSetChecksum1(S,(2573403837U));
  ssSetChecksum2(S,(4053654606U));
  ssSetChecksum3(S,(2408897676U));
}

static void mdlRTW_c5_updated_target_driver_2026_withVariants(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlSetupRuntimeResources_c5_updated_target_driver_2026_withVariants
  (SimStruct *S)
{
  SFc5_updated_target_driver_2026_withVariantsInstanceStruct *chartInstance;
  chartInstance = (SFc5_updated_target_driver_2026_withVariantsInstanceStruct *)
    utMalloc(sizeof(SFc5_updated_target_driver_2026_withVariantsInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  memset(chartInstance, 0, sizeof
         (SFc5_updated_target_driver_2026_withVariantsInstanceStruct));
  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c5_updated_target_driver_2026_withVariants;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c5_updated_target_driver_2026_withVariants;
  chartInstance->chartInfo.mdlStart =
    sf_opaque_mdl_start_c5_updated_target_driver_2026_withVariants;
  chartInstance->chartInfo.mdlTerminate =
    sf_opaque_mdl_terminate_c5_updated_target_driver_2026_withVariants;
  chartInstance->chartInfo.mdlCleanupRuntimeResources =
    sf_opaque_cleanup_runtime_resources_c5_updated_target_driver_2026_withVariants;
  chartInstance->chartInfo.enableChart =
    sf_opaque_enable_c5_updated_target_driver_2026_withVariants;
  chartInstance->chartInfo.disableChart =
    sf_opaque_disable_c5_updated_target_driver_2026_withVariants;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c5_updated_target_driver_2026_withVariants;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c5_updated_target_driver_2026_withVariants;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c5_updated_target_driver_2026_withVariants;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW =
    mdlRTW_c5_updated_target_driver_2026_withVariants;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c5_updated_target_driver_2026_withVariants;
  chartInstance->chartInfo.extModeExec = NULL;
  chartInstance->chartInfo.restoreLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.restoreBeforeLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.storeCurrentConfiguration = NULL;
  chartInstance->chartInfo.callAtomicSubchartUserFcn = NULL;
  chartInstance->chartInfo.callAtomicSubchartAutoFcn = NULL;
  chartInstance->chartInfo.callAtomicSubchartEventFcn = NULL;
  chartInstance->S = S;
  chartInstance->chartInfo.dispatchToExportedFcn = NULL;
  sf_init_ChartRunTimeInfo(S, &(chartInstance->chartInfo), false, 0);
  init_dsm_address_info(chartInstance);
  init_simulink_io_address(chartInstance);
  if (!sim_mode_is_rtw_gen(S)) {
  }

  mdl_setup_runtime_resources_c5_updated_target_driver_2026_withVariants
    (chartInstance);
}

void c5_updated_target_driver_2026_withVariants_method_dispatcher(SimStruct *S,
  int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_SETUP_RUNTIME_RESOURCES:
    mdlSetupRuntimeResources_c5_updated_target_driver_2026_withVariants(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c5_updated_target_driver_2026_withVariants(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c5_updated_target_driver_2026_withVariants(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c5_updated_target_driver_2026_withVariants_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
