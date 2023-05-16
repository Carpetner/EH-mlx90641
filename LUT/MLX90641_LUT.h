/**
 * @copyright (C) 2017 Melexis N.V.
 *
 */
#ifndef _MLX641_LUT_H_
#define _MLX641_LUT_H_

    int MLX90641_GenerateLUTs(float tMin, float tMax, float tStep, paramsMLX90641 *params, float *lut1, float *lut2);
    void MLX90641_LookUpTo(uint16_t *frameData, const paramsMLX90641 *params, float emissivity, float tr, float tMin, float tStep, uint16_t lutLines, float *lut1, float *lut2, float *result);
    
#endif
