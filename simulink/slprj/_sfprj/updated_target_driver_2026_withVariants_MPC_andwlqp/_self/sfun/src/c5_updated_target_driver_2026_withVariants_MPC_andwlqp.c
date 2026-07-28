/* Include files */

#include "updated_target_driver_2026_withVariants_MPC_andwlqp_sfun.h"
#include "c5_updated_target_driver_2026_withVariants_MPC_andwlqp.h"
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
  "#updated_target_driver_2026_withVariants_MPC_andwlqp:1321"/* pathName */
};

/* Function Declarations */
static void initialize_c5_updated_target_driver_2026_withVariants_MPC_andwlqp
  (SFc5_updated_target_driver_2026_withVariants_MPC_andwlqpInstanceStruct
   *chartInstance);
static void
  initialize_params_c5_updated_target_driver_2026_withVariants_MPC_andwlqp
  (SFc5_updated_target_driver_2026_withVariants_MPC_andwlqpInstanceStruct
   *chartInstance);
static void mdl_start_c5_updated_target_driver_2026_withVariants_MPC_andwlqp
  (SFc5_updated_target_driver_2026_withVariants_MPC_andwlqpInstanceStruct
   *chartInstance);
static void mdl_terminate_c5_updated_target_driver_2026_withVariants_MPC_andwlqp
  (SFc5_updated_target_driver_2026_withVariants_MPC_andwlqpInstanceStruct
   *chartInstance);
static void
  mdl_setup_runtime_resources_c5_updated_target_driver_2026_withVariants_MPC_andwlqp
  (SFc5_updated_target_driver_2026_withVariants_MPC_andwlqpInstanceStruct
   *chartInstance);
static void
  mdl_cleanup_runtime_resources_c5_updated_target_driver_2026_withVariants_MPC_andwlqp
  (SFc5_updated_target_driver_2026_withVariants_MPC_andwlqpInstanceStruct
   *chartInstance);
static void enable_c5_updated_target_driver_2026_withVariants_MPC_andwlqp
  (SFc5_updated_target_driver_2026_withVariants_MPC_andwlqpInstanceStruct
   *chartInstance);
static void disable_c5_updated_target_driver_2026_withVariants_MPC_andwlqp
  (SFc5_updated_target_driver_2026_withVariants_MPC_andwlqpInstanceStruct
   *chartInstance);
static void sf_gateway_c5_updated_target_driver_2026_withVariants_MPC_andwlqp
  (SFc5_updated_target_driver_2026_withVariants_MPC_andwlqpInstanceStruct
   *chartInstance);
static void ext_mode_exec_c5_updated_target_driver_2026_withVariants_MPC_andwlqp
  (SFc5_updated_target_driver_2026_withVariants_MPC_andwlqpInstanceStruct
   *chartInstance);
static void c5_update_jit_animation_c5_updated_target_driver_2026_withVarian
  (SFc5_updated_target_driver_2026_withVariants_MPC_andwlqpInstanceStruct
   *chartInstance);
static void c5_do_animation_call_c5_updated_target_driver_2026_withVariants_
  (SFc5_updated_target_driver_2026_withVariants_MPC_andwlqpInstanceStruct
   *chartInstance);
static const mxArray
  *get_sim_state_c5_updated_target_driver_2026_withVariants_MPC_andwlqp
  (SFc5_updated_target_driver_2026_withVariants_MPC_andwlqpInstanceStruct
   *chartInstance);
static void set_sim_state_c5_updated_target_driver_2026_withVariants_MPC_andwlqp
  (SFc5_updated_target_driver_2026_withVariants_MPC_andwlqpInstanceStruct
   *chartInstance, const mxArray *c5_st);
static real_T c5_emlrt_marshallIn
  (SFc5_updated_target_driver_2026_withVariants_MPC_andwlqpInstanceStruct
   *chartInstance, const mxArray *c5_nullptr, const char_T *c5_identifier);
static real_T c5_b_emlrt_marshallIn
  (SFc5_updated_target_driver_2026_withVariants_MPC_andwlqpInstanceStruct
   *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId);
static void init_dsm_address_info
  (SFc5_updated_target_driver_2026_withVariants_MPC_andwlqpInstanceStruct
   *chartInstance);
static void init_simulink_io_address
  (SFc5_updated_target_driver_2026_withVariants_MPC_andwlqpInstanceStruct
   *chartInstance);

/* Function Definitions */
static void initialize_c5_updated_target_driver_2026_withVariants_MPC_andwlqp
  (SFc5_updated_target_driver_2026_withVariants_MPC_andwlqpInstanceStruct
   *chartInstance)
{
  sim_mode_is_external(chartInstance->S);
  chartInstance->c5_doneDoubleBufferReInit = false;
  chartInstance->c5_sfEvent = CALL_EVENT;
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void
  initialize_params_c5_updated_target_driver_2026_withVariants_MPC_andwlqp
  (SFc5_updated_target_driver_2026_withVariants_MPC_andwlqpInstanceStruct
   *chartInstance)
{
  (void)chartInstance;
}

static void mdl_start_c5_updated_target_driver_2026_withVariants_MPC_andwlqp
  (SFc5_updated_target_driver_2026_withVariants_MPC_andwlqpInstanceStruct
   *chartInstance)
{
  sim_mode_is_external(chartInstance->S);
}

static void mdl_terminate_c5_updated_target_driver_2026_withVariants_MPC_andwlqp
  (SFc5_updated_target_driver_2026_withVariants_MPC_andwlqpInstanceStruct
   *chartInstance)
{
  (void)chartInstance;
}

static void
  mdl_setup_runtime_resources_c5_updated_target_driver_2026_withVariants_MPC_andwlqp
  (SFc5_updated_target_driver_2026_withVariants_MPC_andwlqpInstanceStruct
   *chartInstance)
{
  static const uint32_T c5_decisionTxtEndIdx = 0U;
  static const uint32_T c5_decisionTxtStartIdx = 0U;
  sfSetAnimationVectors(chartInstance->S, &chartInstance->c5_JITStateAnimation[0],
                        &chartInstance->c5_JITTransitionAnimation[0]);
  covrtCreateStateflowInstanceData(chartInstance->c5_covrtInstance, 1U, 0U, 1U,
    280U);
  covrtChartInitFcn(chartInstance->c5_covrtInstance, 0U, false, false, false);
  covrtStateInitFcn(chartInstance->c5_covrtInstance, 0U, 0U, false, false, false,
                    0U, &c5_decisionTxtStartIdx, &c5_decisionTxtEndIdx);
  covrtTransInitFcn(chartInstance->c5_covrtInstance, 0U, 0, NULL, NULL, 0U, NULL);
  covrtEmlInitFcn(chartInstance->c5_covrtInstance, "", 4U, 0U, 1U, 0U, 0U, 0U,
                  0U, 0U, 0U, 0U, 0U, 0U);
  covrtEmlFcnInitFcn(chartInstance->c5_covrtInstance, 4U, 0U, 0U,
                     "c5_updated_target_driver_2026_withVariants_MPC_andwlqp", 0,
                     -1, 363);
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
  mdl_cleanup_runtime_resources_c5_updated_target_driver_2026_withVariants_MPC_andwlqp
  (SFc5_updated_target_driver_2026_withVariants_MPC_andwlqpInstanceStruct
   *chartInstance)
{
  covrtDeleteStateflowInstanceData(chartInstance->c5_covrtInstance);
}

static void enable_c5_updated_target_driver_2026_withVariants_MPC_andwlqp
  (SFc5_updated_target_driver_2026_withVariants_MPC_andwlqpInstanceStruct
   *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c5_updated_target_driver_2026_withVariants_MPC_andwlqp
  (SFc5_updated_target_driver_2026_withVariants_MPC_andwlqpInstanceStruct
   *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void sf_gateway_c5_updated_target_driver_2026_withVariants_MPC_andwlqp
  (SFc5_updated_target_driver_2026_withVariants_MPC_andwlqpInstanceStruct
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

static void ext_mode_exec_c5_updated_target_driver_2026_withVariants_MPC_andwlqp
  (SFc5_updated_target_driver_2026_withVariants_MPC_andwlqpInstanceStruct
   *chartInstance)
{
  (void)chartInstance;
}

static void c5_update_jit_animation_c5_updated_target_driver_2026_withVarian
  (SFc5_updated_target_driver_2026_withVariants_MPC_andwlqpInstanceStruct
   *chartInstance)
{
  (void)chartInstance;
}

static void c5_do_animation_call_c5_updated_target_driver_2026_withVariants_
  (SFc5_updated_target_driver_2026_withVariants_MPC_andwlqpInstanceStruct
   *chartInstance)
{
  (void)chartInstance;
}

static const mxArray
  *get_sim_state_c5_updated_target_driver_2026_withVariants_MPC_andwlqp
  (SFc5_updated_target_driver_2026_withVariants_MPC_andwlqpInstanceStruct
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

static void set_sim_state_c5_updated_target_driver_2026_withVariants_MPC_andwlqp
  (SFc5_updated_target_driver_2026_withVariants_MPC_andwlqpInstanceStruct
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
  *sf_c5_updated_target_driver_2026_withVariants_MPC_andwlqp_get_eml_resolved_functions_info
  (void)
{
  const mxArray *c5_nameCaptureInfo = NULL;
  const char_T *c5_data[4] = {
    "789cc553cb4ac340149d482d6eaa5d895bc1b5d1956bd15614df7d2c148979dc9a9879844c529bfe83b8f62ffc44c963f2822162b0decd99c399b9e7de038394"
    "f32b0521b489d23ae8a6d8cb783fc33554adbaae4850d43aea54de09fd234393d10016414aa84e207f6931e2509d06e3c803e40367780e56a2cc1c0c6387c0a8",
    "4cae634686252927b1149f4f6c30dd5148906ff362425c26791ecf927d3b0d79d4ab9e47fd9ef05bfcd24ff4df69f013bacf0c66006881e9693c004f3399052f"
    "40dbee2d7cbbd23952c562a181a1d8fbaba5df85d4afaa3f0e9ed409079fab335fa72e76a8cd541106774888f580f96a7272a8abca72da2769bfa6bcb67e38bf",
    "ecdff4d046829f3777afabf47b5feeeeadd24fd47ff9b5fd77db12bf7e4de7c783c3fba3683a9d5bcb53881e86fc72f27656cc71dbe0d3340792f0bfeeff0ddb"
    "d762ea", "" };

  c5_nameCaptureInfo = NULL;
  emlrtNameCaptureMxArrayR2016a(&c5_data[0], 1608U, &c5_nameCaptureInfo);
  return c5_nameCaptureInfo;
}

static real_T c5_emlrt_marshallIn
  (SFc5_updated_target_driver_2026_withVariants_MPC_andwlqpInstanceStruct
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
  (SFc5_updated_target_driver_2026_withVariants_MPC_andwlqpInstanceStruct
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
  (SFc5_updated_target_driver_2026_withVariants_MPC_andwlqpInstanceStruct
   *chartInstance)
{
  (void)chartInstance;
}

static void init_simulink_io_address
  (SFc5_updated_target_driver_2026_withVariants_MPC_andwlqpInstanceStruct
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
void sf_c5_updated_target_driver_2026_withVariants_MPC_andwlqp_get_check_sum
  (mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(12528594U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(727609648U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(2464157020U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(1643694753U);
}

mxArray
  *sf_c5_updated_target_driver_2026_withVariants_MPC_andwlqp_third_party_uses_info
  (void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray
  *sf_c5_updated_target_driver_2026_withVariants_MPC_andwlqp_jit_fallback_info
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
  *sf_c5_updated_target_driver_2026_withVariants_MPC_andwlqp_updateBuildInfo_args_info
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
  *sf_get_sim_state_info_c5_updated_target_driver_2026_withVariants_MPC_andwlqp
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
  sf_c5_updated_target_driver_2026_withVariants_MPC_andwlqp_get_check_sum
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
  sf_opaque_initialize_c5_updated_target_driver_2026_withVariants_MPC_andwlqp
  (void *chartInstanceVar)
{
  initialize_params_c5_updated_target_driver_2026_withVariants_MPC_andwlqp
    ((SFc5_updated_target_driver_2026_withVariants_MPC_andwlqpInstanceStruct*)
     chartInstanceVar);
  initialize_c5_updated_target_driver_2026_withVariants_MPC_andwlqp
    ((SFc5_updated_target_driver_2026_withVariants_MPC_andwlqpInstanceStruct*)
     chartInstanceVar);
}

static void
  sf_opaque_enable_c5_updated_target_driver_2026_withVariants_MPC_andwlqp(void
  *chartInstanceVar)
{
  enable_c5_updated_target_driver_2026_withVariants_MPC_andwlqp
    ((SFc5_updated_target_driver_2026_withVariants_MPC_andwlqpInstanceStruct*)
     chartInstanceVar);
}

static void
  sf_opaque_disable_c5_updated_target_driver_2026_withVariants_MPC_andwlqp(void *
  chartInstanceVar)
{
  disable_c5_updated_target_driver_2026_withVariants_MPC_andwlqp
    ((SFc5_updated_target_driver_2026_withVariants_MPC_andwlqpInstanceStruct*)
     chartInstanceVar);
}

static void
  sf_opaque_gateway_c5_updated_target_driver_2026_withVariants_MPC_andwlqp(void *
  chartInstanceVar)
{
  sf_gateway_c5_updated_target_driver_2026_withVariants_MPC_andwlqp
    ((SFc5_updated_target_driver_2026_withVariants_MPC_andwlqpInstanceStruct*)
     chartInstanceVar);
}

static const mxArray*
  sf_opaque_get_sim_state_c5_updated_target_driver_2026_withVariants_MPC_andwlqp
  (SimStruct* S)
{
  return get_sim_state_c5_updated_target_driver_2026_withVariants_MPC_andwlqp
    ((SFc5_updated_target_driver_2026_withVariants_MPC_andwlqpInstanceStruct *)
     sf_get_chart_instance_ptr(S));    /* raw sim ctx */
}

static void
  sf_opaque_set_sim_state_c5_updated_target_driver_2026_withVariants_MPC_andwlqp
  (SimStruct* S, const mxArray *st)
{
  set_sim_state_c5_updated_target_driver_2026_withVariants_MPC_andwlqp
    ((SFc5_updated_target_driver_2026_withVariants_MPC_andwlqpInstanceStruct*)
     sf_get_chart_instance_ptr(S), st);
}

static void
  sf_opaque_cleanup_runtime_resources_c5_updated_target_driver_2026_withVariants_MPC_andwlqp
  (void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S =
      ((SFc5_updated_target_driver_2026_withVariants_MPC_andwlqpInstanceStruct*)
       chartInstanceVar)->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_updated_target_driver_2026_withVariants_MPC_andwlqp_optimization_info
        ();
    }

    mdl_cleanup_runtime_resources_c5_updated_target_driver_2026_withVariants_MPC_andwlqp
      ((SFc5_updated_target_driver_2026_withVariants_MPC_andwlqpInstanceStruct*)
       chartInstanceVar);
    utFree(chartInstanceVar);
    if (ssGetUserData(S)!= NULL) {
      sf_free_ChartRunTimeInfo(S);
    }

    ssSetUserData(S,NULL);
  }
}

static void
  sf_opaque_mdl_start_c5_updated_target_driver_2026_withVariants_MPC_andwlqp
  (void *chartInstanceVar)
{
  mdl_start_c5_updated_target_driver_2026_withVariants_MPC_andwlqp
    ((SFc5_updated_target_driver_2026_withVariants_MPC_andwlqpInstanceStruct*)
     chartInstanceVar);
  if (chartInstanceVar) {
    sf_reset_warnings_ChartRunTimeInfo
      (((SFc5_updated_target_driver_2026_withVariants_MPC_andwlqpInstanceStruct*)
        chartInstanceVar)->S);
  }
}

static void
  sf_opaque_mdl_terminate_c5_updated_target_driver_2026_withVariants_MPC_andwlqp
  (void *chartInstanceVar)
{
  mdl_terminate_c5_updated_target_driver_2026_withVariants_MPC_andwlqp
    ((SFc5_updated_target_driver_2026_withVariants_MPC_andwlqpInstanceStruct*)
     chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void
  mdlProcessParameters_c5_updated_target_driver_2026_withVariants_MPC_andwlqp
  (SimStruct *S)
{
  mdlProcessParamsCommon(S);
  if (sf_machine_global_initializer_called()) {
    initialize_params_c5_updated_target_driver_2026_withVariants_MPC_andwlqp
      ((SFc5_updated_target_driver_2026_withVariants_MPC_andwlqpInstanceStruct*)
       sf_get_chart_instance_ptr(S));
  }
}

const char
  * sf_c5_updated_target_driver_2026_withVariants_MPC_andwlqp_get_post_codegen_info
  (void)
{
  int i;
  const char* encStrCodegen [26] = {
    "eNrdWUtv41QUdkqmM0XTATYwQkgDEkJsoNPHlJFA0E4enaC+mCQdqQhFN/ZJfIl97d5HHl11h1g",
    "gNmxYsIZ/AUt+Anv+ACvEBolzHSf1uJnEdgSNuJLbHDvfPe/HdYxc5cDAdQevn+4ZxjL+v4XXkj",
    "FcN0I6F7mG9/PGRyF9/KJhmDbhssJanpF+mZ4FbWBV1WrRfkosU+4x4cQVGfgy4sITEJ6jJPVYO",
    "uEpawEHZuIGvsdlKr6CusqhrFNWzNScxVObmnbV9pRjPcINiXXEnMHz+PpKHiPHIuVgyjKAJW3u",
    "qbZddkh7uhW47BVsMDtCualtJUBWla9VFQfKkdR3oNQHs8KEJGgFMUPfqiQSCrKfOkKoqI7Qnus",
    "7lLDktraJqIKP0SGh7lv490hJtF5CvsivSRmRHqfEKblOQUd4Quyxg3IeYFg7qe0sXVmEpmq3KW",
    "tr63LlAkP9MU4S2KpV8LrASRuOWMoc1NqV+oGDx3GZPAcrOiwz5aByh14RmbAB31IXLSQy8i2br",
    "EAcR6TD1jx/H7rgBPyLRJIM2CH/FGAhqFXzTgjXWZAykxSjZwpCbMFjFk3u4W4MFRTcQyyeCeDU",
    "1SEFFpp5LPp4o1kxqYT03AKmUXF/PyG/q9gKk8BbxITEtY8TKgAFDuIqJV+LCtJ0NBqtJAMtE+8",
    "ALDPUEC3Fij2Pd9DGaYvspa10JqRDg9WGIkgICkYJo/uEOCqhzK7ACqfDoy6wYqXji1idP5nAJj",
    "FtsHQ/oQ4cgNAbiMT1GfvILmrbpXJQBGFy6ifNJCXAwkairVQb+FBnHeb1WJl7bjWcCKbEFQBWD",
    "cIZtoVH2Jb4oIzCJ5Oaw1lNR1aW8cwl0iFNHRt7wLCzaF11JyQmZlWJ4eiGAs2DrdJzbO1MUCFx",
    "nBqUghywgnn0bi7dPPpKSG+Oc6kiahw9RbANM5yUdpvBsAc16kJwo0pwphiS4dJ87xuXfFeWpvN",
    "dwk+5jDhjTpwdweUn2Gc7gns5pM0HDRXMQlZDEt4G2bDQQsAbG/c3ths9Ku2wSIvGwXGhQZjVc8",
    "78q3LeTuCPkZy/vpDOj6sh/W1wNsDy3ZeXA/tlRngu5gGTOpUwxNGzXYwbvVp06NNqlDjUlFuOP",
    "BoT+pH+HDSIKjYIbkcSy4kSV+wQt/toRe0+wj2e4a+7MZymudf0mgANafoNzBC/ER6W9H4PZ+y3",
    "HNtvObCawrwYynMxA/9pDK/pz0tfrNUFcLHW4oR1sGjZ3tpIyOBgoyfmtdERZ+158r/vXtolqsd",
    "LM+JjNbz//dFnX86D/+b8rbfnwUfjYV78rLh4NeYHTYvd0vqTDwYnJ13rvAiD07LYr/f2hvu9G9",
    "kvN2G/aHxm+X5W3LQ6l79S5/JGDpfG7URwq8Z03HJoo6++Xvrx9T9//3vpuw8vfvj5YnOeuu4vp",
    "6tfd0L6jdG5ajx5dq8MZ0n8/1rM/5oWW7s2wN7h470tWxQrndL6qWseFYL9frkxXd6bMXlH99/E",
    "S+pqqvfnZsWKlFyihmf3eLwvz7DHyjPx/scn8+G3duLxMMleKzF7aZo4vk0anFgT4vj69Hmwk6S",
    "O34rpo+kmyECdxdBjO7Nf2sR1F88v7+zM6vO5SO0Z6pM3JM4SiyH/eiL5c8/InzP6C2P/jUzyDx",
    "ZG/s1M8p/H5F/0c8h1nXuuU84k5558RtxNY75z1n+Fm1e/f3s+XbTvZz0vLpoexpT6dHuCXrnYv",
    "ouq129Guvn+Xkh/PH4nX7CpY014Kxg+3gfSmvT0fxLff6W03+g8U9L2C39UPN3cZcQZCDp82TK6",
    "fcz171njRxyImPyu9Tr60KQ5Y1KfvxHLb03Xa+X3Hs7Rz/4BHs6J9Q==",
    ""
  };

  static char newstr [1857] = "";
  newstr[0] = '\0';
  for (i = 0; i < 26; i++) {
    strcat(newstr, encStrCodegen[i]);
  }

  return newstr;
}

static void
  mdlSetWorkWidths_c5_updated_target_driver_2026_withVariants_MPC_andwlqp
  (SimStruct *S)
{
  const char* newstr =
    sf_c5_updated_target_driver_2026_withVariants_MPC_andwlqp_get_post_codegen_info
    ();
  sf_set_work_widths(S, newstr);
  ssSetChecksum0(S,(2801961351U));
  ssSetChecksum1(S,(4259575322U));
  ssSetChecksum2(S,(2151388162U));
  ssSetChecksum3(S,(864075677U));
}

static void mdlRTW_c5_updated_target_driver_2026_withVariants_MPC_andwlqp
  (SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void
  mdlSetupRuntimeResources_c5_updated_target_driver_2026_withVariants_MPC_andwlqp
  (SimStruct *S)
{
  SFc5_updated_target_driver_2026_withVariants_MPC_andwlqpInstanceStruct
    *chartInstance;
  chartInstance =
    (SFc5_updated_target_driver_2026_withVariants_MPC_andwlqpInstanceStruct *)
    utMalloc(sizeof
             (SFc5_updated_target_driver_2026_withVariants_MPC_andwlqpInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  memset(chartInstance, 0, sizeof
         (SFc5_updated_target_driver_2026_withVariants_MPC_andwlqpInstanceStruct));
  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c5_updated_target_driver_2026_withVariants_MPC_andwlqp;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c5_updated_target_driver_2026_withVariants_MPC_andwlqp;
  chartInstance->chartInfo.mdlStart =
    sf_opaque_mdl_start_c5_updated_target_driver_2026_withVariants_MPC_andwlqp;
  chartInstance->chartInfo.mdlTerminate =
    sf_opaque_mdl_terminate_c5_updated_target_driver_2026_withVariants_MPC_andwlqp;
  chartInstance->chartInfo.mdlCleanupRuntimeResources =
    sf_opaque_cleanup_runtime_resources_c5_updated_target_driver_2026_withVariants_MPC_andwlqp;
  chartInstance->chartInfo.enableChart =
    sf_opaque_enable_c5_updated_target_driver_2026_withVariants_MPC_andwlqp;
  chartInstance->chartInfo.disableChart =
    sf_opaque_disable_c5_updated_target_driver_2026_withVariants_MPC_andwlqp;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c5_updated_target_driver_2026_withVariants_MPC_andwlqp;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c5_updated_target_driver_2026_withVariants_MPC_andwlqp;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c5_updated_target_driver_2026_withVariants_MPC_andwlqp;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW =
    mdlRTW_c5_updated_target_driver_2026_withVariants_MPC_andwlqp;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c5_updated_target_driver_2026_withVariants_MPC_andwlqp;
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

  mdl_setup_runtime_resources_c5_updated_target_driver_2026_withVariants_MPC_andwlqp
    (chartInstance);
}

void c5_updated_target_driver_2026_withVariants_MPC_andwlqp_method_dispatcher
  (SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_SETUP_RUNTIME_RESOURCES:
    mdlSetupRuntimeResources_c5_updated_target_driver_2026_withVariants_MPC_andwlqp
      (S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c5_updated_target_driver_2026_withVariants_MPC_andwlqp(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c5_updated_target_driver_2026_withVariants_MPC_andwlqp
      (S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c5_updated_target_driver_2026_withVariants_MPC_andwlqp_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
