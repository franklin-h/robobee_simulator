/* Include files */

#include "updated_target_driver_2026_withVariants_MPC_andwlqp2_sfun.h"
#include "c5_updated_target_driver_2026_withVariants_MPC_andwlqp2.h"
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
  "#updated_target_driver_2026_withVariants_MPC_andwlqp2:1321"/* pathName */
};

/* Function Declarations */
static void initialize_c5_updated_target_driver_2026_withVariants_MPC_andwlqp2
  (SFc5_updated_target_driver_2026_withVariants_MPC_andwlqp2InstanceStruct
   *chartInstance);
static void
  initialize_params_c5_updated_target_driver_2026_withVariants_MPC_andwlqp2
  (SFc5_updated_target_driver_2026_withVariants_MPC_andwlqp2InstanceStruct
   *chartInstance);
static void mdl_start_c5_updated_target_driver_2026_withVariants_MPC_andwlqp2
  (SFc5_updated_target_driver_2026_withVariants_MPC_andwlqp2InstanceStruct
   *chartInstance);
static void
  mdl_terminate_c5_updated_target_driver_2026_withVariants_MPC_andwlqp2
  (SFc5_updated_target_driver_2026_withVariants_MPC_andwlqp2InstanceStruct
   *chartInstance);
static void
  mdl_setup_runtime_resources_c5_updated_target_driver_2026_withVariants_MPC_andwlqp2
  (SFc5_updated_target_driver_2026_withVariants_MPC_andwlqp2InstanceStruct
   *chartInstance);
static void
  mdl_cleanup_runtime_resources_c5_updated_target_driver_2026_withVariants_MPC_andwlqp2
  (SFc5_updated_target_driver_2026_withVariants_MPC_andwlqp2InstanceStruct
   *chartInstance);
static void enable_c5_updated_target_driver_2026_withVariants_MPC_andwlqp2
  (SFc5_updated_target_driver_2026_withVariants_MPC_andwlqp2InstanceStruct
   *chartInstance);
static void disable_c5_updated_target_driver_2026_withVariants_MPC_andwlqp2
  (SFc5_updated_target_driver_2026_withVariants_MPC_andwlqp2InstanceStruct
   *chartInstance);
static void sf_gateway_c5_updated_target_driver_2026_withVariants_MPC_andwlqp2
  (SFc5_updated_target_driver_2026_withVariants_MPC_andwlqp2InstanceStruct
   *chartInstance);
static void
  ext_mode_exec_c5_updated_target_driver_2026_withVariants_MPC_andwlqp2
  (SFc5_updated_target_driver_2026_withVariants_MPC_andwlqp2InstanceStruct
   *chartInstance);
static void c5_update_jit_animation_c5_updated_target_driver_2026_withVarian
  (SFc5_updated_target_driver_2026_withVariants_MPC_andwlqp2InstanceStruct
   *chartInstance);
static void c5_do_animation_call_c5_updated_target_driver_2026_withVariants_
  (SFc5_updated_target_driver_2026_withVariants_MPC_andwlqp2InstanceStruct
   *chartInstance);
static const mxArray
  *get_sim_state_c5_updated_target_driver_2026_withVariants_MPC_andwlqp2
  (SFc5_updated_target_driver_2026_withVariants_MPC_andwlqp2InstanceStruct
   *chartInstance);
static void
  set_sim_state_c5_updated_target_driver_2026_withVariants_MPC_andwlqp2
  (SFc5_updated_target_driver_2026_withVariants_MPC_andwlqp2InstanceStruct
   *chartInstance, const mxArray *c5_st);
static real_T c5_emlrt_marshallIn
  (SFc5_updated_target_driver_2026_withVariants_MPC_andwlqp2InstanceStruct
   *chartInstance, const mxArray *c5_nullptr, const char_T *c5_identifier);
static real_T c5_b_emlrt_marshallIn
  (SFc5_updated_target_driver_2026_withVariants_MPC_andwlqp2InstanceStruct
   *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId);
static void init_dsm_address_info
  (SFc5_updated_target_driver_2026_withVariants_MPC_andwlqp2InstanceStruct
   *chartInstance);
static void init_simulink_io_address
  (SFc5_updated_target_driver_2026_withVariants_MPC_andwlqp2InstanceStruct
   *chartInstance);

/* Function Definitions */
static void initialize_c5_updated_target_driver_2026_withVariants_MPC_andwlqp2
  (SFc5_updated_target_driver_2026_withVariants_MPC_andwlqp2InstanceStruct
   *chartInstance)
{
  sim_mode_is_external(chartInstance->S);
  chartInstance->c5_doneDoubleBufferReInit = false;
  chartInstance->c5_sfEvent = CALL_EVENT;
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void
  initialize_params_c5_updated_target_driver_2026_withVariants_MPC_andwlqp2
  (SFc5_updated_target_driver_2026_withVariants_MPC_andwlqp2InstanceStruct
   *chartInstance)
{
  (void)chartInstance;
}

static void mdl_start_c5_updated_target_driver_2026_withVariants_MPC_andwlqp2
  (SFc5_updated_target_driver_2026_withVariants_MPC_andwlqp2InstanceStruct
   *chartInstance)
{
  sim_mode_is_external(chartInstance->S);
}

static void
  mdl_terminate_c5_updated_target_driver_2026_withVariants_MPC_andwlqp2
  (SFc5_updated_target_driver_2026_withVariants_MPC_andwlqp2InstanceStruct
   *chartInstance)
{
  (void)chartInstance;
}

static void
  mdl_setup_runtime_resources_c5_updated_target_driver_2026_withVariants_MPC_andwlqp2
  (SFc5_updated_target_driver_2026_withVariants_MPC_andwlqp2InstanceStruct
   *chartInstance)
{
  static const uint32_T c5_decisionTxtEndIdx = 0U;
  static const uint32_T c5_decisionTxtStartIdx = 0U;
  sfSetAnimationVectors(chartInstance->S, &chartInstance->c5_JITStateAnimation[0],
                        &chartInstance->c5_JITTransitionAnimation[0]);
  covrtCreateStateflowInstanceData(chartInstance->c5_covrtInstance, 1U, 0U, 1U,
    243U);
  covrtChartInitFcn(chartInstance->c5_covrtInstance, 0U, false, false, false);
  covrtStateInitFcn(chartInstance->c5_covrtInstance, 0U, 0U, false, false, false,
                    0U, &c5_decisionTxtStartIdx, &c5_decisionTxtEndIdx);
  covrtTransInitFcn(chartInstance->c5_covrtInstance, 0U, 0, NULL, NULL, 0U, NULL);
  covrtEmlInitFcn(chartInstance->c5_covrtInstance, "", 4U, 0U, 1U, 0U, 0U, 0U,
                  0U, 0U, 0U, 0U, 0U, 0U);
  covrtEmlFcnInitFcn(chartInstance->c5_covrtInstance, 4U, 0U, 0U,
                     "c5_updated_target_driver_2026_withVariants_MPC_andwlqp2",
                     0, -1, 363);
  covrtEmlInitFcn(chartInstance->c5_covrtInstance,
                  "/Users/franklinho/robobee_simulator/simulink/robobee_tcp_step_codegen.m",
                  14U, 0U, 1U, 0U, 2U, 0U, 0U, 0U, 0U, 0U, 0U, 0U);
  covrtEmlFcnInitFcn(chartInstance->c5_covrtInstance, 14U, 0U, 0U,
                     "robobee_tcp_step_codegen", 0, -1, 1470);
  covrtEmlIfInitFcn(chartInstance->c5_covrtInstance, 14U, 0U, 0U, 688, 713, 876,
                    1466, false);
  covrtEmlIfInitFcn(chartInstance->c5_covrtInstance, 14U, 0U, 1U, 1383, 1397, -1,
                    1462, false);
  covrtEmlRelationalInitFcn(chartInstance->c5_covrtInstance, 14U, 0U, 0U, 1386,
    1397, -1, 1U);
}

static void
  mdl_cleanup_runtime_resources_c5_updated_target_driver_2026_withVariants_MPC_andwlqp2
  (SFc5_updated_target_driver_2026_withVariants_MPC_andwlqp2InstanceStruct
   *chartInstance)
{
  covrtDeleteStateflowInstanceData(chartInstance->c5_covrtInstance);
}

static void enable_c5_updated_target_driver_2026_withVariants_MPC_andwlqp2
  (SFc5_updated_target_driver_2026_withVariants_MPC_andwlqp2InstanceStruct
   *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c5_updated_target_driver_2026_withVariants_MPC_andwlqp2
  (SFc5_updated_target_driver_2026_withVariants_MPC_andwlqp2InstanceStruct
   *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void sf_gateway_c5_updated_target_driver_2026_withVariants_MPC_andwlqp2
  (SFc5_updated_target_driver_2026_withVariants_MPC_andwlqp2InstanceStruct
   *chartInstance)
{
  real_T c5_pose[11];
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
    for (c5_i = 0; c5_i < 11; c5_i++) {
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

static void
  ext_mode_exec_c5_updated_target_driver_2026_withVariants_MPC_andwlqp2
  (SFc5_updated_target_driver_2026_withVariants_MPC_andwlqp2InstanceStruct
   *chartInstance)
{
  (void)chartInstance;
}

static void c5_update_jit_animation_c5_updated_target_driver_2026_withVarian
  (SFc5_updated_target_driver_2026_withVariants_MPC_andwlqp2InstanceStruct
   *chartInstance)
{
  (void)chartInstance;
}

static void c5_do_animation_call_c5_updated_target_driver_2026_withVariants_
  (SFc5_updated_target_driver_2026_withVariants_MPC_andwlqp2InstanceStruct
   *chartInstance)
{
  (void)chartInstance;
}

static const mxArray
  *get_sim_state_c5_updated_target_driver_2026_withVariants_MPC_andwlqp2
  (SFc5_updated_target_driver_2026_withVariants_MPC_andwlqp2InstanceStruct
   *chartInstance)
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

static void
  set_sim_state_c5_updated_target_driver_2026_withVariants_MPC_andwlqp2
  (SFc5_updated_target_driver_2026_withVariants_MPC_andwlqp2InstanceStruct
   *chartInstance, const mxArray *c5_st)
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
  *sf_c5_updated_target_driver_2026_withVariants_MPC_andwlqp2_get_eml_resolved_functions_info
  (void)
{
  const mxArray *c5_nameCaptureInfo = NULL;
  const char_T *c5_data[4] = {
    "789cc5534b4ec330147450a9d814ba429ca28115eb0a5a040254d10f1208857c5e89496c4771529a5e802557e1465c05e5e3fc242b8888f236e3d1d86fde1bc9"
    "48b9bc511042fb28ade36e8abd8cf733dc41d5aaeb8a0445eda24ee59dd03f3234190d601da484ea04f297162398ea3498451e201f3873576025ca12bb30c304",
    "a665721b33322e493989a5f87c6683e94c43827c9b1713ba6592e7f12cd9b7d39047bdea79d4ef09bff52ffd44ffa3063fa1fbcc60068016989ec603f0349359"
    "f002b4eddec2b72b9d23552c161a2e147b7fb6f4bb92fa55f5c7d1933ae7e07375e9ebd47131b5992ac2e09884ae1e305f4d4e983aaa2ca70149fb35e575f0c3",
    "f965ffa687f6121cdebfbf6ed30f7f4d06dbf413f55f7e6dffdda1c4af5fd3f9707472771a2d162b6b730ed1c3985fcfdf2e8a39260d3e4d732009ffebfedfa1"
    "31634a", "" };

  c5_nameCaptureInfo = NULL;
  emlrtNameCaptureMxArrayR2016a(&c5_data[0], 1608U, &c5_nameCaptureInfo);
  return c5_nameCaptureInfo;
}

static real_T c5_emlrt_marshallIn
  (SFc5_updated_target_driver_2026_withVariants_MPC_andwlqp2InstanceStruct
   *chartInstance, const mxArray *c5_nullptr, const char_T *c5_identifier)
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
  (SFc5_updated_target_driver_2026_withVariants_MPC_andwlqp2InstanceStruct
   *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId)
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
  (SFc5_updated_target_driver_2026_withVariants_MPC_andwlqp2InstanceStruct
   *chartInstance)
{
  (void)chartInstance;
}

static void init_simulink_io_address
  (SFc5_updated_target_driver_2026_withVariants_MPC_andwlqp2InstanceStruct
   *chartInstance)
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
void sf_c5_updated_target_driver_2026_withVariants_MPC_andwlqp2_get_check_sum
  (mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(12528594U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(727609648U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(2464157020U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(1643694753U);
}

mxArray
  *sf_c5_updated_target_driver_2026_withVariants_MPC_andwlqp2_third_party_uses_info
  (void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray
  *sf_c5_updated_target_driver_2026_withVariants_MPC_andwlqp2_jit_fallback_info
  (void)
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

mxArray
  *sf_c5_updated_target_driver_2026_withVariants_MPC_andwlqp2_updateBuildInfo_args_info
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
  *sf_get_sim_state_info_c5_updated_target_driver_2026_withVariants_MPC_andwlqp2
  (void)
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
  sf_c5_updated_target_driver_2026_withVariants_MPC_andwlqp2_get_check_sum
    (&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static const char* sf_get_instance_specialization(void)
{
  return "s4AheeGNHG4hsDIkE1ZmcOC";
}

static void
  sf_opaque_initialize_c5_updated_target_driver_2026_withVariants_MPC_andwlqp2
  (void *chartInstanceVar)
{
  initialize_params_c5_updated_target_driver_2026_withVariants_MPC_andwlqp2
    ((SFc5_updated_target_driver_2026_withVariants_MPC_andwlqp2InstanceStruct*)
     chartInstanceVar);
  initialize_c5_updated_target_driver_2026_withVariants_MPC_andwlqp2
    ((SFc5_updated_target_driver_2026_withVariants_MPC_andwlqp2InstanceStruct*)
     chartInstanceVar);
}

static void
  sf_opaque_enable_c5_updated_target_driver_2026_withVariants_MPC_andwlqp2(void *
  chartInstanceVar)
{
  enable_c5_updated_target_driver_2026_withVariants_MPC_andwlqp2
    ((SFc5_updated_target_driver_2026_withVariants_MPC_andwlqp2InstanceStruct*)
     chartInstanceVar);
}

static void
  sf_opaque_disable_c5_updated_target_driver_2026_withVariants_MPC_andwlqp2(void
  *chartInstanceVar)
{
  disable_c5_updated_target_driver_2026_withVariants_MPC_andwlqp2
    ((SFc5_updated_target_driver_2026_withVariants_MPC_andwlqp2InstanceStruct*)
     chartInstanceVar);
}

static void
  sf_opaque_gateway_c5_updated_target_driver_2026_withVariants_MPC_andwlqp2(void
  *chartInstanceVar)
{
  sf_gateway_c5_updated_target_driver_2026_withVariants_MPC_andwlqp2
    ((SFc5_updated_target_driver_2026_withVariants_MPC_andwlqp2InstanceStruct*)
     chartInstanceVar);
}

static const mxArray*
  sf_opaque_get_sim_state_c5_updated_target_driver_2026_withVariants_MPC_andwlqp2
  (SimStruct* S)
{
  return get_sim_state_c5_updated_target_driver_2026_withVariants_MPC_andwlqp2
    ((SFc5_updated_target_driver_2026_withVariants_MPC_andwlqp2InstanceStruct *)
     sf_get_chart_instance_ptr(S));    /* raw sim ctx */
}

static void
  sf_opaque_set_sim_state_c5_updated_target_driver_2026_withVariants_MPC_andwlqp2
  (SimStruct* S, const mxArray *st)
{
  set_sim_state_c5_updated_target_driver_2026_withVariants_MPC_andwlqp2
    ((SFc5_updated_target_driver_2026_withVariants_MPC_andwlqp2InstanceStruct*)
     sf_get_chart_instance_ptr(S), st);
}

static void
  sf_opaque_cleanup_runtime_resources_c5_updated_target_driver_2026_withVariants_MPC_andwlqp2
  (void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S =
      ((SFc5_updated_target_driver_2026_withVariants_MPC_andwlqp2InstanceStruct*)
       chartInstanceVar)->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_updated_target_driver_2026_withVariants_MPC_andwlqp2_optimization_info
        ();
    }

    mdl_cleanup_runtime_resources_c5_updated_target_driver_2026_withVariants_MPC_andwlqp2
      ((SFc5_updated_target_driver_2026_withVariants_MPC_andwlqp2InstanceStruct*)
       chartInstanceVar);
    utFree(chartInstanceVar);
    if (ssGetUserData(S)!= NULL) {
      sf_free_ChartRunTimeInfo(S);
    }

    ssSetUserData(S,NULL);
  }
}

static void
  sf_opaque_mdl_start_c5_updated_target_driver_2026_withVariants_MPC_andwlqp2
  (void *chartInstanceVar)
{
  mdl_start_c5_updated_target_driver_2026_withVariants_MPC_andwlqp2
    ((SFc5_updated_target_driver_2026_withVariants_MPC_andwlqp2InstanceStruct*)
     chartInstanceVar);
  if (chartInstanceVar) {
    sf_reset_warnings_ChartRunTimeInfo
      (((SFc5_updated_target_driver_2026_withVariants_MPC_andwlqp2InstanceStruct*)
        chartInstanceVar)->S);
  }
}

static void
  sf_opaque_mdl_terminate_c5_updated_target_driver_2026_withVariants_MPC_andwlqp2
  (void *chartInstanceVar)
{
  mdl_terminate_c5_updated_target_driver_2026_withVariants_MPC_andwlqp2
    ((SFc5_updated_target_driver_2026_withVariants_MPC_andwlqp2InstanceStruct*)
     chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void
  mdlProcessParameters_c5_updated_target_driver_2026_withVariants_MPC_andwlqp2
  (SimStruct *S)
{
  mdlProcessParamsCommon(S);
  if (sf_machine_global_initializer_called()) {
    initialize_params_c5_updated_target_driver_2026_withVariants_MPC_andwlqp2
      ((SFc5_updated_target_driver_2026_withVariants_MPC_andwlqp2InstanceStruct*)
       sf_get_chart_instance_ptr(S));
  }
}

const char
  * sf_c5_updated_target_driver_2026_withVariants_MPC_andwlqp2_get_post_codegen_info
  (void)
{
  int i;
  const char* encStrCodegen [26] = {
    "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",
    ""
  };

  static char newstr [1853] = "";
  newstr[0] = '\0';
  for (i = 0; i < 26; i++) {
    strcat(newstr, encStrCodegen[i]);
  }

  return newstr;
}

static void
  mdlSetWorkWidths_c5_updated_target_driver_2026_withVariants_MPC_andwlqp2
  (SimStruct *S)
{
  const char* newstr =
    sf_c5_updated_target_driver_2026_withVariants_MPC_andwlqp2_get_post_codegen_info
    ();
  sf_set_work_widths(S, newstr);
  ssSetChecksum0(S,(2801961351U));
  ssSetChecksum1(S,(4259575322U));
  ssSetChecksum2(S,(2151388162U));
  ssSetChecksum3(S,(864075677U));
}

static void mdlRTW_c5_updated_target_driver_2026_withVariants_MPC_andwlqp2
  (SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void
  mdlSetupRuntimeResources_c5_updated_target_driver_2026_withVariants_MPC_andwlqp2
  (SimStruct *S)
{
  SFc5_updated_target_driver_2026_withVariants_MPC_andwlqp2InstanceStruct
    *chartInstance;
  chartInstance =
    (SFc5_updated_target_driver_2026_withVariants_MPC_andwlqp2InstanceStruct *)
    utMalloc(sizeof
             (SFc5_updated_target_driver_2026_withVariants_MPC_andwlqp2InstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  memset(chartInstance, 0, sizeof
         (SFc5_updated_target_driver_2026_withVariants_MPC_andwlqp2InstanceStruct));
  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c5_updated_target_driver_2026_withVariants_MPC_andwlqp2;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c5_updated_target_driver_2026_withVariants_MPC_andwlqp2;
  chartInstance->chartInfo.mdlStart =
    sf_opaque_mdl_start_c5_updated_target_driver_2026_withVariants_MPC_andwlqp2;
  chartInstance->chartInfo.mdlTerminate =
    sf_opaque_mdl_terminate_c5_updated_target_driver_2026_withVariants_MPC_andwlqp2;
  chartInstance->chartInfo.mdlCleanupRuntimeResources =
    sf_opaque_cleanup_runtime_resources_c5_updated_target_driver_2026_withVariants_MPC_andwlqp2;
  chartInstance->chartInfo.enableChart =
    sf_opaque_enable_c5_updated_target_driver_2026_withVariants_MPC_andwlqp2;
  chartInstance->chartInfo.disableChart =
    sf_opaque_disable_c5_updated_target_driver_2026_withVariants_MPC_andwlqp2;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c5_updated_target_driver_2026_withVariants_MPC_andwlqp2;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c5_updated_target_driver_2026_withVariants_MPC_andwlqp2;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c5_updated_target_driver_2026_withVariants_MPC_andwlqp2;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW =
    mdlRTW_c5_updated_target_driver_2026_withVariants_MPC_andwlqp2;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c5_updated_target_driver_2026_withVariants_MPC_andwlqp2;
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

  mdl_setup_runtime_resources_c5_updated_target_driver_2026_withVariants_MPC_andwlqp2
    (chartInstance);
}

void c5_updated_target_driver_2026_withVariants_MPC_andwlqp2_method_dispatcher
  (SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_SETUP_RUNTIME_RESOURCES:
    mdlSetupRuntimeResources_c5_updated_target_driver_2026_withVariants_MPC_andwlqp2
      (S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c5_updated_target_driver_2026_withVariants_MPC_andwlqp2(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c5_updated_target_driver_2026_withVariants_MPC_andwlqp2
      (S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c5_updated_target_driver_2026_withVariants_MPC_andwlqp2_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
