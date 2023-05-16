/**
 * @copyright (C) 2017 Melexis N.V.
 *
 */
#include <MLX90641_I2C_Driver.h>
#include <MLX90641_API.h>
#include <MLX90641_LUT.h>
#include <math.h>
 
void UpdateLUT(float ta, float tr, float ksta, float emissivity, float tMin, float tStep, uint16_t lutLines, float *lut1, float *lut2, float *lut);
  
//------------------------------------------------------------------------------

void MLX90641_LookUpTo(uint16_t *frameData, const paramsMLX90641 *params, float emissivity, float tr, float tMin, float tStep, uint16_t lutLines, float *lut1, float *lut2, float *result)
{
    float vdd;
    float ta;
    float gain;
    float irDataCP;
    float irData;
    float alphaCompensated;
    float To;
    uint16_t subPage;
    float lut[lutLines];
    int itn;
    int idxH;
    int idxL;
    int idxM;
    float k1;
    float ktaScale;
    float kvScale;
    float alphaScale;
    float kta;
    float kv;
        
    subPage = frameData[241];
    vdd = MLX90641_GetVdd(frameData, params);
    ta = MLX90641_GetTa(frameData, params);    
   
    ktaScale = pow(2,(double)params->ktaScale);
    kvScale = pow(2,(double)params->kvScale);
    alphaScale = pow(2,(double)params->alphaScale);
    
    UpdateLUT(ta, tr, params->KsTa, emissivity, tMin, tStep, lutLines, lut1, lut2, lut);
        
//------------------------- Gain calculation -----------------------------------    
    gain = frameData[202];
    if(gain > 32767)
    {
        gain = gain - 65536;
    }
    
    gain = params->gainEE / gain; 
  
//------------------------- To calculation -------------------------------------    
    irDataCP = frameData[200];  
    if(irDataCP > 32767)
    {
        irDataCP = irDataCP - 65536;
    }
    irDataCP = irDataCP * gain;

    irDataCP = irDataCP - params->cpOffset * (1 + params->cpKta * (ta - 25)) * (1 + params->cpKv * (vdd - 3.3));

    for( int pixelNumber = 0; pixelNumber < 192; pixelNumber++)
    {      
        irData = frameData[pixelNumber];
        if(irData > 32767)
        {
            irData = irData - 65536;
        }
        irData = irData * gain;
        
        kta = (float)params->kta[pixelNumber]/ktaScale;
        kv = (float)params->kv[pixelNumber]/kvScale;
            
        irData = irData - params->offset[subPage][pixelNumber]*(1 + kta*(ta - 25))*(1 + kv*(vdd - 3.3));                
    
        irData = irData - params->tgc * irDataCP;
            
        alphaCompensated = params->alpha[pixelNumber] / alphaScale;
        alphaCompensated = irData*alphaCompensated;
                
        itn = 0;
        idxL = 0;
        idxH = lutLines - 1;
            
        if(alphaCompensated >= lut[idxH])
        {
            result[pixelNumber] = tMin+idxH*tStep;
        }
        else if(alphaCompensated <= lut[idxL])
        {
            result[pixelNumber] = tMin;
        }           
        else       
        {
            while(itn < 20 && (idxH - idxL)>1)
            {
                idxM = (idxH+idxL)/2;
                if (alphaCompensated <= lut[idxM])
                {
                    idxH =  idxM;
                }
                else if(alphaCompensated >= lut[idxM])
                {
                    idxL = idxM;
                }   
                
                itn = itn + 1;     
            }    
            
            To = tMin + idxL * tStep; 
            k1 = (lut[idxH] - lut[idxL])/tStep;
            To = To + (alphaCompensated - lut[idxL])/k1;
    
            result[pixelNumber] = To;
 
        }    
    }
}

//------------------------------------------------------------------------------

int MLX90641_GenerateLUTs(float tMin, float tMax, float tStep, paramsMLX90641 *params, float *lut1, float *lut2)
{
    int lNum = 0;
    double temp = 0;
    double tempK = 0;
    double k[8];
    double lutKsTo;
    float ird;
    double ta4Temp;
    double sx;
    float To;
    int range;
        
    temp = (tMax-tMin)/tStep;
    lNum = ceil(temp) + 1;
    
    if (lNum < 1)
    {
        lNum = -9;
    }
    else
    {                  
        k[1] = 1 / (1 + params->ksTo[1] * 20);
        k[0] = k[1] / (1 + params->ksTo[0] * 20);
        k[2] = 1 ;
        k[3] = (1 + params->ksTo[2] * params->ct[3]);
        k[4] = k[3] * (1 + params->ksTo[3] * (params->ct[4] - params->ct[3]));
        k[5] = k[4] * (1 + params->ksTo[4] * (params->ct[5] - params->ct[4]));
        k[6] = k[5] * (1 + params->ksTo[5] * (params->ct[6] - params->ct[5]));
        k[7] = k[6] * (1 + params->ksTo[6] * (params->ct[7] - params->ct[6]));
                
        ta4Temp = 298.15 * 298.15 * 298.15 * 298.15; 
        
        temp = tMin;
        
        if(temp < params->ct[1])
        {
            lutKsTo = params->ksTo[0];            
        }
        else if(temp < params->ct[2])   
        {
            lutKsTo = params->ksTo[1];            
        }   
        else if(temp < params->ct[3])
        {
            lutKsTo = params->ksTo[2];           
        }
        else if(temp < params->ct[4])
        {
            lutKsTo = params->ksTo[3];            
        } 
        else if(temp < params->ct[5])
        {
            lutKsTo = params->ksTo[4];            
        } 
        else if(temp < params->ct[6])
        {
            lutKsTo = params->ksTo[5];            
        }
        else if(temp < params->ct[7])
        {
            lutKsTo = params->ksTo[6];            
        }  
        else
        {
            lutKsTo = params->ksTo[7];            
        }                  
            
        tempK = temp + 273.15;
        
        for(int i=0; i<lNum; i++)
        {
            lut1[i] = tempK*tempK*tempK*tempK; 
            lut1[i] = SCALEALPHA*lut1[i];        
            
            if(temp > params->ct[7])
            {
                lut2[i] = k[7]*(1 + params->ksTo[7]*(temp - params->ct[7]));
            }       
            else if(temp == 25.0 || temp == 0)
            {
                if(i == 0)
                {    
                    lut2[i] = 1;
                }
                else
                {
                    lut2[i] = lut2[i-1];
                }        
                    
            }    
            else
            {                
                lutKsTo = 1 + lutKsTo*temp; 
                
                ird = SCALEALPHA * lutKsTo * (tempK*tempK*tempK*tempK - ta4Temp);
                
                sx = SCALEALPHA * SCALEALPHA * SCALEALPHA * (ird + SCALEALPHA * ta4Temp);
                sx = sqrt(sqrt(sx)) * params->ksTo[2];            
                    
                To = sqrt(sqrt(ird/(SCALEALPHA * (1 - params->ksTo[2] * 273.15) + sx) + ta4Temp)) - 273.15;    
                          
                if(To < params->ct[1])
                {
                    range = 0;
                }
                else if(To < params->ct[2])   
                {
                    range = 1;            
                }   
                else if(To < params->ct[3])
                {
                    range = 2;            
                }
                else if(To < params->ct[4])
                {
                    range = 3;            
                }
                else if(To < params->ct[5])
                {
                    range = 4;            
                }
                else if(To < params->ct[6])
                {
                    range = 5;            
                }
                else
                {
                    range = 6;            
                }      
                
                To = sqrt(sqrt(ird / (SCALEALPHA * k[range] * (1 + params->ksTo[range] * (To - params->ct[range]))) + ta4Temp));
                  
                
                lutKsTo = (ird/(SCALEALPHA * (To*To*To*To-ta4Temp))-1)/(To-273.15);
                k[7] = 1 + lutKsTo * params->ct[7];
                
                lut2[i] = 1 + lutKsTo*temp;
            }
    
            temp = temp + tStep; 
            tempK = tempK + tStep;

        }                          
        
    }
    
    return lNum;              
}

//------------------------------------------------------------------------------

void UpdateLUT(float ta, float tr, float ksta, float emissivity, float tMin, float tStep, uint16_t lutLines, float *lut1, float *lut2, float *lut)
{
    float k1 = 0;
    float ta4;
    float tr4;
    float taTr;
    
    ta4 = ta + 273.15;
    ta4 = ta4*ta4;
    ta4 = ta4*ta4;
    tr4 = tr + 273.15;
    tr4 = tr4*tr4;
    tr4 = tr4*tr4;
    taTr = tr4 - (tr4-ta4)/emissivity;
    k1 = SCALEALPHA*taTr;
        
    for (int i=0; i<lutLines; i++)
    {
        lut[i] = lut1[i] - k1;
        lut[i] = lut[i]*(1 + ksta*(ta-25.0f))*lut2[i];
        lut[i] = lut[i]*emissivity;
    }
    
}    
