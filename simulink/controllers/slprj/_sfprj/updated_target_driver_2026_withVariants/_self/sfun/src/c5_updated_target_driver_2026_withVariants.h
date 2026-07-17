#ifndef __c5_updated_target_driver_2026_withVariants_h__
#define __c5_updated_target_driver_2026_withVariants_h__

/* Forward Declarations */

/* Type Definitions */
#ifndef typedef_SFc5_updated_target_driver_2026_withVariantsInstanceStruct
#define typedef_SFc5_updated_target_driver_2026_withVariantsInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  int32_T c5_sfEvent;
  boolean_T c5_doneDoubleBufferReInit;
  uint8_T c5_JITStateAnimation[1];
  uint8_T c5_JITTransitionAnimation[1];
  CovrtStateflowInstance *c5_covrtInstance;
  void *c5_fEmlrtCtx;
  real_T *c5_time;
  real_T *c5_dt_s;
  real_T *c5_left_voltage_v;
  real_T *c5_right_voltage_v;
  real_T *c5_bias_voltage_v;
  real_T *c5_x;
  real_T *c5_y;
  real_T *c5_z;
  real_T *c5_alpha_rad;
  real_T *c5_beta_rad;
  real_T *c5_gamma_rad;
} SFc5_updated_target_driver_2026_withVariantsInstanceStruct;

#endif                                 /* typedef_SFc5_updated_target_driver_2026_withVariantsInstanceStruct */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
extern const mxArray
  *sf_c5_updated_target_driver_2026_withVariants_get_eml_resolved_functions_info
  (void);

/* Function Definitions */
extern void sf_c5_updated_target_driver_2026_withVariants_get_check_sum(mxArray *
  plhs[]);
extern void c5_updated_target_driver_2026_withVariants_method_dispatcher
  (SimStruct *S, int_T method, void *data);

#endif
