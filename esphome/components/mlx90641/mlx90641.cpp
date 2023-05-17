
#include "mlx90641.h"

#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace mlx90641 {

float emmisivity_ = 0.95;
float ta_shift_ = 5;
uint8_t refresh_rate_ = 0x00;  //0x00..5Hz|0x01..1Hz|0x02..2Hz|0x03..4Hz|0x04..8Hz|0x05..16Hz|0x06..32Hz|0x07..64Hz
bool absolute_mode_ = false;

static const uint8_t I2C_BUFFER_SIZE = 254;    //set to even number, smaller than i2c buffer length

static const uint8_t MLX_ROWS = 12;
static const uint8_t MLX_COLS = 16;
static const uint16_t MLX_IMAGE_SIZE = 192;
static const uint16_t MLX_FRAME_SIZE = 242;
static const uint16_t MLX_EEBUF_SIZE = 832;

static const uint16_t MLX_STATUS_REGISTER = 0x8000;
static const uint16_t MLX_STATUS_INIT_VAL = 0x0010;
static const uint16_t MLX_SUBPAGE_NR_MASK = 0x0001;
static const uint16_t MLX_DATA_READY_MASK = 0x0008;

static const uint16_t MLX_CTRL_REGISTER_1 = 0x800D;
static const uint16_t MLX_CTRL_REGISTER_2 = 0x800E;
static const uint16_t MLX_FREQ_SHIFT_MASK = 0x0007;
static const uint16_t MLX_RFRSH_RATE_MASK = 0xFC7F;

static const uint16_t MLX_I2C_CF_REGISTER = 0x800F;
static const uint16_t MLX_ADRESS_REGISTER = 0x8010;
static const uint16_t MLX_EPROM_0_ADDRESS = 0x2400;
static const uint16_t MLX_RAM_NUL_ADDRESS = 0x0400;
static const uint16_t MLX_RAM_ADDRESS_INC = 0x0040;
static const uint16_t MLX_RAM_AUX_ADDRESS = 0x0580;
static const uint16_t MLX_AUX_DATA_LENGTH = 0x0030;

static const float SCALEALPHA = 0.000001;

MLX90641Component::MlxDeviceParams MlxParams;

static const char *const TAG = "mlx90641";

void MLX90641Component::setup() 
{
//dump EEPROM & decode Hamming
  uint16_t eebuf[MLX_EEBUF_SIZE];
  ESP_LOGCONFIG(TAG, "Setting up device...");
  if (this->read_1616_(MLX_EPROM_0_ADDRESS, MLX_EEBUF_SIZE, eebuf) != 0) 
  {
    ESP_LOGE(TAG, "EEprom read failed");
    this->mark_failed();  return;
  }
  int8_t err = HammingDecode(eebuf);
  if (!err)  
    ESP_LOGI(TAG, "EEprom read succesful, no errors");
  if (err = -9)  
    ESP_LOGI(TAG, "EEprom read errors, most probably corrected...");
  if (err = -10) 
  {
    ESP_LOGW(TAG, "EEprom read errors, data may be corrupted...");
    this->status_set_warning();
  }
//extract parameters from EEPROM
  if (MLX90641_ExtractParameters(eebuf, &MlxParams) != 0) 
  {
    ESP_LOGE(TAG, "EEprom data validation failed");
    this->mark_failed();  return;
  }
  ESP_LOGI(TAG, "EEprom data valid, extracting parameters.................[OK]");
//set refresh rate
  uint16_t control_reg;
  float rr;
  uint16_t rate = ((uint16_t)refresh_rate_ & MLX_FREQ_SHIFT_MASK) << MLX_FREQ_SHIFT_MASK;
  if (this->read_1616_(MLX_CTRL_REGISTER_1, 1, &control_reg) != 0) 
  {
    this->status_set_warning();
    ESP_LOGW(TAG, "Error reading current refresh rate, device default will be used");
  }
  else 
  {
    rate = (control_reg & MLX_RFRSH_RATE_MASK) | rate;
    if (this->write_1616_(MLX_CTRL_REGISTER_1, rate) != 0) 
    {
      this->status_set_warning();
      ESP_LOGW(TAG, "Error setting refresh rate, device default should work");
    }
    else 
    {
      switch (refresh_rate_) 
      { 
        case 0x00:
          rr = 0.5;  break;                                           
        case 0x01:
          rr = 1;    break;                                                
        case 0x02:
          rr = 2;    break;                                                
        case 0x03:
          rr = 4;    break;                                                          
        case 0x04:
          rr = 8;    break;                                                
        case 0x05:
          rr = 16;   break;                                                
        case 0x06:
          rr = 32;   break;                                            
        case 0x07:
          rr = 64;   break;        
      }
      ESP_LOGI(TAG, "Refresh rate set to %.1f Hz", rr);
    }
  }
  ESP_LOGI(TAG, "Device ready");
}

inline uint16_t i2ctohs(uint16_t i2cshort) { return convert_big_endian(i2cshort); }

//read $len 16bit words starting at 16bit adress - byte-swap on little endian arch
int MLX90641Component::read_1616_(uint16_t address, uint16_t len, uint16_t *data)
{
  uint8_t chunks = 2 * len / I2C_BUFFER_SIZE;
  uint16_t remain = 2 * len - I2C_BUFFER_SIZE * chunks; 
  uint8_t buf[2] = {0};
  for (uint8_t j = 0; j < chunks; j++) {
    buf[0] = (address + j * I2C_BUFFER_SIZE / 2) >> 8; //msb
    buf[1] = (address + j * I2C_BUFFER_SIZE / 2) & 0x00FF; //lsb
    if (this->write(buf, 2, false) != i2c::ERROR_OK) {
      ESP_LOGE(TAG, "write failed");
      return -1;
    }
    if (this->read(reinterpret_cast<uint8_t *>(data + I2C_BUFFER_SIZE / 2 * j), I2C_BUFFER_SIZE) != i2c::ERROR_OK)
      return -1;
  }
  if (remain != 0) {
    buf[0] = (address + chunks * I2C_BUFFER_SIZE / 2) >> 8; //msb
    buf[1] = (address + chunks * I2C_BUFFER_SIZE / 2) & 0x00FF; //lsb
    if (this->write(buf, 2, false) != i2c::ERROR_OK)
      return -1;
    if (this->read(reinterpret_cast<uint8_t *>(data + I2C_BUFFER_SIZE / 2 * chunks), remain) != i2c::ERROR_OK)
      return -1;
  }
  for (uint16_t i = 0; i < len; i++)
    data[i] = i2ctohs(data[i]);
  return 0;
}
/*
//write-->read $length 16bit words starting at 16bit adress
int MLX90641Component::read_1616_(uint16_t address, uint16_t length, uint16_t *data)
{
  uint8_t addr[2] = {0};
  uint16_t *d = data;
  while (length >= I2C_BUFFER_SIZE / 2)                 //read in chunks of i2c buffer length
  {
    addr[0] = address >> 8;                             //msb
    addr[1] = address & 0x00FF;                         //lsb
    if (this->write(addr, 2, false) != i2c::ERROR_OK)               //don't drop the line 
      return -1;
    if (this->read(reinterpret_cast<uint8_t *>(d), I2C_BUFFER_SIZE) != i2c::ERROR_OK)
      return -1;
    address += I2C_BUFFER_SIZE / 2;
    d += I2C_BUFFER_SIZE / 2;
    length -= I2C_BUFFER_SIZE / 2;
  }
  if (length > 0)                                       //read remaining
  {
    addr[0] = address >> 8;                             
    addr[1] = address & 0x00FF;                         
    if (this->write(addr, 2, false) != i2c::ERROR_OK)   
      return -1;
    if (this->read(reinterpret_cast<uint8_t *>(d), length * 2) != i2c::ERROR_OK)       //cast 16-8
      return -1;    
  }
  for (uint16_t i = 0; i < length; i++)                    //byte-swap on little endian arch
    data[i] = i2ctohs(data[i]);
  return 0;
}
*/
//write 16bits of data to a 16bit address, proof-read
int MLX90641Component::write_1616_(uint16_t address, uint16_t data) 
{
  static uint16_t OK;
  uint8_t buf[4] = {0};
  buf[0] = address >> 8;
  buf[1] = address & 0x00FF;
  buf[2] = data >> 8;
  buf[3] = data & 0x00FF;
  if (this->write(buf, 4) != i2c::ERROR_OK)
    return -1;
  if (!this->read_1616_(address, 1, &OK))
    return -2;
  if (data != OK)  
    return -2;
  return 0;
}

int MLX90641Component::HammingDecode(uint16_t *eeData)
{
    int error = 0;
    int16_t parity[5];
    int8_t D[16];
    int16_t check;
    uint16_t data;
    uint16_t mask;
    
    for (int addr=16; addr<832; addr++) {   
        parity[0] = -1;
        parity[1] = -1;
        parity[2] = -1;
        parity[3] = -1;
        parity[4] = -1;        
        data = eeData[addr];        
        mask = 1;
        for (int i = 0; i < 16; i++) {          
            D[i] = (data & mask) >> i;
            mask = mask << 1;
        }        
        parity[0] = D[0]^D[1]^D[3]^D[4]^D[6]^D[8]^D[10]^D[11];
        parity[1] = D[0]^D[2]^D[3]^D[5]^D[6]^D[9]^D[10]^D[12];
        parity[2] = D[1]^D[2]^D[3]^D[7]^D[8]^D[9]^D[10]^D[13];
        parity[3] = D[4]^D[5]^D[6]^D[7]^D[8]^D[9]^D[10]^D[14];
        parity[4] = D[0]^D[1]^D[2]^D[3]^D[4]^D[5]^D[6]^D[7]^D[8]^D[9]^D[10]^D[11]^D[12]^D[13]^D[14]^D[15];
        
        if ((parity[0]!=0) || (parity[1]!=0) || (parity[2]!=0) || (parity[3]!=0) || (parity[4]!=0))
        {        
            check = (parity[0]<<0) + (parity[1]<<1) + (parity[2]<<2) + (parity[3]<<3) + (parity[4]<<4);
    
            if ((check > 15)&&(check < 32)) {
                switch (check) {    
                    case 16:
                        D[15] = 1 - D[15];  break;                                            
                    case 24:
                        D[14] = 1 - D[14];  break;                                                
                    case 20:
                        D[13] = 1 - D[13];  break;                                                
                    case 18:
                        D[12] = 1 - D[12];  break;                                                          
                    case 17:
                        D[11] = 1 - D[11];  break;                                                
                    case 31:
                        D[10] = 1 - D[10];  break;                                                
                    case 30:
                        D[9] = 1 - D[9];  break;                                            
                    case 29:
                        D[8] = 1 - D[8];  break;                                                            
                    case 28:
                        D[7] = 1 - D[7];  break;                                                
                    case 27:
                        D[6] = 1 - D[6];  break;                                                    
                    case 26:
                        D[5] = 1 - D[5];  break;                                                    
                    case 25:
                        D[4] = 1 - D[4];  break;                                                     
                    case 23:
                        D[3] = 1 - D[3];  break;                                                 
                    case 22:
                        D[2] = 1 - D[2];  break;                                                     
                    case 21:
                        D[1] = 1 - D[1];  break;                                                 
                    case 19:
                        D[0] = 1 - D[0];  break;                                     
                }  
                error = -9;
                data = 0;
                mask = 1;
                for (int i = 0; i < 16; i++) {                    
                    data = data + D[i]*mask;
                    mask = mask << 1;
                }
            }
            else  error = -10;                
        }
        eeData[addr] = data & 0x07FF;
    }
    return error;
}

int MLX90641Component::ValidateFrameData(uint16_t *frameData)
{
    uint8_t line = 0;
    
    for(int i=0; i<192; i+=16)
    {
        if(frameData[i] == 0x7FFF) return -8;
        line = line + 1;
    }    
        
    return 0;    
}

int MLX90641Component::ValidateAuxData(uint16_t *auxData) 
{
    
    if(auxData[0] == 0x7FFF) return -8;    
    
    for(int i=8; i<19; i++)
    {
        if(auxData[i] == 0x7FFF) return -8;
    }
    
    for(int i=20; i<23; i++)
    {
        if(auxData[i] == 0x7FFF) return -8;
    }
    
    for(int i=24; i<33; i++)
    {
        if(auxData[i] == 0x7FFF) return -8;
    }
    
    for(int i=40; i<48; i++)
    {
        if(auxData[i] == 0x7FFF) return -8;
    }
    
    return 0;
    
}

int MLX90641Component::MLX90641_ExtractParameters(uint16_t *eeData, MlxDeviceParams *mlx90641)
{
    int error = CheckEEPROMValid(eeData);
    
    if(error == 0)
    {
        ExtractVDDParameters(eeData, mlx90641);
        ExtractPTATParameters(eeData, mlx90641);
        ExtractGainParameters(eeData, mlx90641);
        ExtractTgcParameters(eeData, mlx90641);
        ExtractEmissivityParameters(eeData, mlx90641);
        ExtractResolutionParameters(eeData, mlx90641);
        ExtractKsTaParameters(eeData, mlx90641);
        ExtractKsToParameters(eeData, mlx90641);
        ExtractCPParameters(eeData, mlx90641);
        ExtractAlphaParameters(eeData, mlx90641);
        ExtractOffsetParameters(eeData, mlx90641);
        ExtractKtaPixelParameters(eeData, mlx90641);
        ExtractKvPixelParameters(eeData, mlx90641);        
        error = ExtractDeviatingPixels(eeData, mlx90641);  
    }
    
    return error;

}

void MLX90641Component::MLX90641_CalculateTo(uint16_t *frameData, const MlxDeviceParams *params, float emissivity, float tr, float *result)
{
    float vdd;
    float ta;
    float ta4;
    float tr4;
    float taTr;
    float gain;
    float irDataCP;
    float irData;
    float alphaCompensated;
    float Sx;
    float To;
    float alphaCorrR[8];
    int8_t range;
    uint16_t subPage;
    float ktaScale;
    float kvScale;
    float alphaScale;
    float kta;
    float kv;
    
    subPage = frameData[241];
    vdd = MLX90641_GetVdd(frameData, params);
    ta = MLX90641_GetTa(frameData, params);    
    ta4 = (ta + 273.15);
    ta4 = ta4 * ta4;
    ta4 = ta4 * ta4;
    tr4 = (tr + 273.15);
    tr4 = tr4 * tr4;
    tr4 = tr4 * tr4;
    
    taTr = tr4 - (tr4-ta4)/emissivity;
    
    ktaScale = pow(2,(double)params->ktaScale);
    kvScale = pow(2,(double)params->kvScale);
    alphaScale = pow(2,(double)params->alphaScale);
    
    alphaCorrR[1] = 1 / (1 + params->ksTo[1] * 20);
    alphaCorrR[0] = alphaCorrR[1] / (1 + params->ksTo[0] * 20);
    alphaCorrR[2] = 1 ;
    alphaCorrR[3] = (1 + params->ksTo[2] * params->ct[3]);
    alphaCorrR[4] = alphaCorrR[3] * (1 + params->ksTo[3] * (params->ct[4] - params->ct[3]));
    alphaCorrR[5] = alphaCorrR[4] * (1 + params->ksTo[4] * (params->ct[5] - params->ct[4]));
    alphaCorrR[6] = alphaCorrR[5] * (1 + params->ksTo[5] * (params->ct[6] - params->ct[5]));
    alphaCorrR[7] = alphaCorrR[6] * (1 + params->ksTo[6] * (params->ct[7] - params->ct[6]));
    
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
        
        irData = irData / emissivity;
        
        alphaCompensated = SCALEALPHA*alphaScale/params->alpha[pixelNumber];
        alphaCompensated = alphaCompensated*(1 + params->KsTa * (ta - 25));
        
        Sx = alphaCompensated * alphaCompensated * alphaCompensated * (irData + alphaCompensated * taTr);
        Sx = sqrt(sqrt(Sx)) * params->ksTo[2];
        
        To = sqrt(sqrt(irData/(alphaCompensated * (1 - params->ksTo[2] * 273.15) + Sx) + taTr)) - 273.15;
                
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
        else if(To < params->ct[7])
        {
            range = 6;            
        }
        else
        {
            range = 7;            
        }      
        
        To = sqrt(sqrt(irData / (alphaCompensated * alphaCorrR[range] * (1 + params->ksTo[range] * (To - params->ct[range]))) + taTr)) - 273.15;
        
        result[pixelNumber] = To;
    }
}

float MLX90641Component::MLX90641_GetVdd(uint16_t *frameData, const MlxDeviceParams *params)
{
    float vdd;
    float resolutionCorrection;
    
    int resolutionRAM;    
    
    vdd = frameData[234];
    if(vdd > 32767)
    {
        vdd = vdd - 65536;
    }
    resolutionRAM = (frameData[240] & 0x0C00) >> 10;
    resolutionCorrection = pow(2, (double)params->resolutionEE) / pow(2, (double)resolutionRAM);
    vdd = (resolutionCorrection * vdd - params->vdd25) / params->kVdd + 3.3;
    
    return vdd;
}

float MLX90641Component::MLX90641_GetTa(uint16_t *frameData, const MlxDeviceParams *params)
{
    float ptat;
    float ptatArt;
    float vdd;
    float ta;
    
    vdd = MLX90641_GetVdd(frameData, params);
    
    ptat = frameData[224];
    if(ptat > 32767)
    {
        ptat = ptat - 65536;
    }
    
    ptatArt = frameData[192];
    if(ptatArt > 32767)
    {
        ptatArt = ptatArt - 65536;
    }
    ptatArt = (ptat / (ptat * params->alphaPTAT + ptatArt)) * pow(2, (double)18);
    
    ta = (ptatArt / (1 + params->KvPTAT * (vdd - 3.3)) - params->vPTAT25);
    ta = ta / params->KtPTAT + 25;
    
    return ta;
}

void MLX90641Component::ExtractVDDParameters(uint16_t *eeData, MlxDeviceParams *mlx90641)
{
    int16_t kVdd;
    int16_t vdd25;
    
    kVdd = eeData[39];
    if(kVdd > 1023)
    {
        kVdd = kVdd - 2048;
    }
    kVdd = 32 * kVdd;
    
    vdd25 = eeData[38];
    if(vdd25 > 1023)
    {
        vdd25 = vdd25 - 2048;
    }
    vdd25 = 32 * vdd25;
    
    mlx90641->kVdd = kVdd;
    mlx90641->vdd25 = vdd25; 
}

void MLX90641Component::ExtractPTATParameters(uint16_t *eeData, MlxDeviceParams *mlx90641)
{
    float KvPTAT;
    float KtPTAT;
    int16_t vPTAT25;
    float alphaPTAT;
    
    KvPTAT = eeData[43];
    if(KvPTAT > 1023)
    {
        KvPTAT = KvPTAT - 2048;
    }
    KvPTAT = KvPTAT/4096;
    
    KtPTAT = eeData[42];
    if(KtPTAT > 1023)
    {
        KtPTAT = KtPTAT - 2048;
    }
    KtPTAT = KtPTAT/8;
    
    vPTAT25 = 32 * eeData[40] + eeData[41];
    
    alphaPTAT = eeData[44] / 128.0f;
    
    mlx90641->KvPTAT = KvPTAT;
    mlx90641->KtPTAT = KtPTAT;    
    mlx90641->vPTAT25 = vPTAT25;
    mlx90641->alphaPTAT = alphaPTAT;   
}

void MLX90641Component::ExtractGainParameters(uint16_t *eeData, MlxDeviceParams *mlx90641)
{
    int16_t gainEE;
    
    gainEE = 32 * eeData[36] + eeData[37];

    mlx90641->gainEE = gainEE;    
}

void MLX90641Component::ExtractTgcParameters(uint16_t *eeData, MlxDeviceParams *mlx90641)
{
    float tgc;
    tgc = eeData[51] & 0x01FF;
    if(tgc > 255)
    {
        tgc = tgc - 512;
    }
    tgc = tgc / 64.0f;
    
    mlx90641->tgc = tgc;        
}

void MLX90641Component::ExtractEmissivityParameters(uint16_t *eeData, MlxDeviceParams *mlx90641)
{
    float emissivity;
    emissivity = eeData[35];
       
    if(emissivity > 1023)
    {
        emissivity = emissivity - 2048;
    }
    emissivity = emissivity/512;
    
    mlx90641->emissivityEE = emissivity;
}

void MLX90641Component::ExtractResolutionParameters(uint16_t *eeData, MlxDeviceParams *mlx90641)
{
    uint8_t resolutionEE;
    resolutionEE = (eeData[51] & 0x0600) >> 9;    
    
    mlx90641->resolutionEE = resolutionEE;
}

void MLX90641Component::ExtractKsTaParameters(uint16_t *eeData, MlxDeviceParams *mlx90641)
{
    float KsTa;
    KsTa = eeData[34];
    if(KsTa > 1023)
    {
        KsTa = KsTa - 2048;
    }
    KsTa = KsTa / 32768.0f;
    
    mlx90641->KsTa = KsTa;
}

void MLX90641Component::ExtractKsToParameters(uint16_t *eeData, MlxDeviceParams *mlx90641)
{
    int KsToScale;
    
    mlx90641->ct[0] = -40;
    mlx90641->ct[1] = -20;
    mlx90641->ct[2] = 0;
    mlx90641->ct[3] = 80;
    mlx90641->ct[4] = 120;
    mlx90641->ct[5] = eeData[58];
    mlx90641->ct[6] = eeData[60];
    mlx90641->ct[7] = eeData[62];
     
    KsToScale = eeData[52];
    KsToScale = 1 << KsToScale;
    
    mlx90641->ksTo[0] = eeData[53];
    mlx90641->ksTo[1] = eeData[54];
    mlx90641->ksTo[2] = eeData[55];
    mlx90641->ksTo[3] = eeData[56];
    mlx90641->ksTo[4] = eeData[57];
    mlx90641->ksTo[5] = eeData[59];
    mlx90641->ksTo[6] = eeData[61];
    mlx90641->ksTo[7] = eeData[63];
    
    
    for(int i = 0; i < 8; i++)
    {
        if(mlx90641->ksTo[i] > 1023)
        {
            mlx90641->ksTo[i] = mlx90641->ksTo[i] - 2048;
        }
        mlx90641->ksTo[i] = mlx90641->ksTo[i] / KsToScale;
    } 
}

void MLX90641Component::ExtractAlphaParameters(uint16_t *eeData, MlxDeviceParams *mlx90641)
{
    float rowMaxAlphaNorm[6];
    uint16_t scaleRowAlpha[6];
    uint8_t alphaScale;
    float alphaTemp[192];
    float temp;
    int p = 0;

    scaleRowAlpha[0] = (eeData[25] >> 5) + 20;
    scaleRowAlpha[1] = (eeData[25] & 0x001F) + 20;
    scaleRowAlpha[2] = (eeData[26] >> 5) + 20;
    scaleRowAlpha[3] = (eeData[26] & 0x001F) + 20;
    scaleRowAlpha[4] = (eeData[27] >> 5) + 20;
    scaleRowAlpha[5] = (eeData[27] & 0x001F) + 20;

    
    for(int i = 0; i < 6; i++)
    {
        rowMaxAlphaNorm[i] = eeData[28 + i] / pow(2,(double)scaleRowAlpha[i]);
        rowMaxAlphaNorm[i] = rowMaxAlphaNorm[i] / 2047.0f;
    }

    for(int i = 0; i < 6; i++)
    {
        for(int j = 0; j < 32; j ++)
        {
            p = 32 * i +j;
            alphaTemp[p] = eeData[256 + p] * rowMaxAlphaNorm[i]; 
            alphaTemp[p] = alphaTemp[p] - mlx90641->tgc * mlx90641->cpAlpha;
            alphaTemp[p] = SCALEALPHA/alphaTemp[p];
        }
    }
    
    temp = alphaTemp[0];
    for(int i = 1; i < 192; i++)
    {
        if (alphaTemp[i] > temp)
        {
            temp = alphaTemp[i];
        }
    }
    
    alphaScale = 0;
    while(temp < 32768)
    {
        temp = temp*2;
        alphaScale = alphaScale + 1;
    } 
    
    for(int i = 0; i < 192; i++)
    {
        temp = alphaTemp[i] * pow(2,(double)alphaScale);        
        mlx90641->alpha[i] = (temp + 0.5);        
        
    } 
    
    mlx90641->alphaScale = alphaScale;      
}

void MLX90641Component::ExtractOffsetParameters(uint16_t *eeData, MlxDeviceParams *mlx90641)
{
    int scaleOffset;
    int16_t offsetRef;
    int16_t tempOffset; 
    
    scaleOffset = eeData[16] >> 5;
    scaleOffset = 1 << scaleOffset;

    offsetRef = 32 * eeData[17] + eeData[18];
    if (offsetRef > 32767)
    {
        offsetRef = offsetRef - 65536;
    }

    for(int i = 0; i < 192; i++)
    {
        tempOffset = eeData[64 + i];
        if(tempOffset > 1023)
        {
           tempOffset = eeData[64 + i] - 2048; 
        }
        mlx90641->offset[0][i] = tempOffset * scaleOffset + offsetRef;
        
        tempOffset = eeData[640 + i];
        if(tempOffset > 1023)
        {
           tempOffset = eeData[640 + i] - 2048; 
        }
        mlx90641->offset[1][i] = tempOffset * scaleOffset + offsetRef;
    }
}

void MLX90641Component::ExtractKtaPixelParameters(uint16_t *eeData, MlxDeviceParams *mlx90641)
{
    uint8_t ktaScale1;
    uint8_t ktaScale2;
    int16_t ktaAvg;
    int16_t tempKta;
    float ktaTemp[192];
    float temp;

    ktaAvg = eeData[21];
    if (ktaAvg > 1023)
    {
        ktaAvg = ktaAvg - 2048;
    }
  
    ktaScale1 = eeData[22] >> 5;
    ktaScale2 = eeData[22] & 0x001F;

    for(int i = 0; i < 192; i++)
    {
        tempKta = (eeData[448 + i] >> 5);
        if (tempKta > 31)
        {
            tempKta = tempKta - 64;
        }

        ktaTemp[i] = tempKta * pow(2,(double)ktaScale2);
        ktaTemp[i] = ktaTemp[i] + ktaAvg;
        ktaTemp[i] = ktaTemp[i] / pow(2,(double)ktaScale1);
    }
    
    temp = fabs(ktaTemp[0]);
    for(int i = 1; i < 192; i++)
    {
        if (fabs(ktaTemp[i]) > temp)
        {
            temp = fabs(ktaTemp[i]);
        }
    }
    
    ktaScale1 = 0;
    while(temp < 64)
    {
        temp = temp*2;
        ktaScale1 = ktaScale1 + 1;
    }    
     
    for(int i = 0; i < 192; i++)
    {
        temp = ktaTemp[i] * pow(2,(double)ktaScale1);
        if (temp < 0)
        {
            mlx90641->kta[i] = (temp - 0.5);
        }
        else
        {
            mlx90641->kta[i] = (temp + 0.5);
        }        
        
    } 
    
    mlx90641->ktaScale = ktaScale1;
}

void MLX90641Component::ExtractKvPixelParameters(uint16_t *eeData, MlxDeviceParams *mlx90641)
{
    uint8_t kvScale1;
    uint8_t kvScale2;
    int16_t kvAvg;
    int16_t tempKv;
    float kvTemp[192];
    float temp;

    kvAvg = eeData[23];
    if (kvAvg > 1023)
    {
        kvAvg = kvAvg - 2048;
    }
  
    kvScale1 = eeData[24] >> 5;
    kvScale2 = eeData[24] & 0x001F;

    for(int i = 0; i < 192; i++)
    {
        tempKv = (eeData[448 + i] & 0x001F);
        if (tempKv > 15)
        {
            tempKv = tempKv - 32;
        }

        kvTemp[i] = tempKv * pow(2,(double)kvScale2);
        kvTemp[i] = kvTemp[i] + kvAvg;
        kvTemp[i] = kvTemp[i] / pow(2,(double)kvScale1);
    }
    
    temp = fabs(kvTemp[0]);
    for(int i = 1; i < 192; i++)
    {
        if (fabs(kvTemp[i]) > temp)
        {
            temp = fabs(kvTemp[i]);
        }
    }
    
    kvScale1 = 0;
    while(temp < 64)
    {
        temp = temp*2;
        kvScale1 = kvScale1 + 1;
    }    
     
    for(int i = 0; i < 192; i++)
    {
        temp = kvTemp[i] * pow(2,(double)kvScale1);
        if (temp < 0)
        {
            mlx90641->kv[i] = (temp - 0.5);
        }
        else
        {
            mlx90641->kv[i] = (temp + 0.5);
        }        
        
    } 
    
    mlx90641->kvScale = kvScale1;        
}

void MLX90641Component::ExtractCPParameters(uint16_t *eeData, MlxDeviceParams *mlx90641)
{
    float alphaCP;
    int16_t offsetCP;
    float cpKv;
    float cpKta;
    uint8_t alphaScale;
    uint8_t ktaScale1;
    uint8_t kvScale;

    alphaScale = eeData[46];
    
    offsetCP = 32 * eeData[47] + eeData[48];
    if (offsetCP > 32767)
    {
        offsetCP = offsetCP - 65536;
    }
       
    alphaCP = eeData[45];
    if (alphaCP > 1023)
    {
        alphaCP = alphaCP - 2048;
    }
    
    alphaCP = alphaCP /  pow(2,(double)alphaScale);
    
    
    cpKta = eeData[49] & 0x003F;
    if (cpKta > 31)
    {
        cpKta = cpKta - 64;
    }
    ktaScale1 = eeData[49] >> 6;    
    mlx90641->cpKta = cpKta / pow(2,(double)ktaScale1);
    
    cpKv = eeData[50] & 0x003F;
    if (cpKv > 31)
    {
        cpKv = cpKv - 64;
    }
    kvScale = eeData[50] >> 6;
    mlx90641->cpKv = cpKv / pow(2,(double)kvScale);
       
    mlx90641->cpAlpha = alphaCP;
    mlx90641->cpOffset = offsetCP;
}

int MLX90641Component::CheckEEPROMValid(uint16_t *eeData)  
{
    int deviceSelect;
    deviceSelect = eeData[10] & 0x0040;
    if(deviceSelect != 0)
    {
        return 0;
    }
    
    return -7;    
}

int MLX90641Component::ExtractDeviatingPixels(uint16_t *eeData, MlxDeviceParams *mlx90641)
{
    uint16_t pixCnt = 0;
    uint16_t brokenPixCnt = 0;

    int warn = 0;
    
    mlx90641->brokenPixel = 0xFFFF;
        
    pixCnt = 0;    
    while (pixCnt < 192 && brokenPixCnt < 2)
    {
        if((eeData[pixCnt+64] == 0) && (eeData[pixCnt+256] == 0) && (eeData[pixCnt+448] == 0) && (eeData[pixCnt+640] == 0))
        {
            mlx90641->brokenPixel = pixCnt;
            brokenPixCnt = brokenPixCnt + 1;
        }    
        
        pixCnt = pixCnt + 1;
    } 
    
    if(brokenPixCnt > 1)  
    {
        warn = -3;
    }         
    
    return warn;
       
} 

void MLX90641Component::dump_config() 
{
  ESP_LOGCONFIG(TAG, "Sensors:");
  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Device can't be enabled");
  }
  LOG_UPDATE_INTERVAL(this);
  LOG_SENSOR("  ", "Ambient", this->ambient_sensor_);
  LOG_SENSOR("  ", "Object", this->object_sensor_);
//  LOG_TEXT_SENSOR("  ", "Frame", this->frame_sensor_);
}

float MLX90641Component::get_setup_priority() const { return setup_priority::DATA; }

void MLX90641Component::update() 
{
  float calc_time;
  float read_time;
  float wait_time;
  float start_time = micros();

  uint16_t frame_buf [MLX_FRAME_SIZE];
  float image_buf [MLX_IMAGE_SIZE];         //this is the image
  uint8_t abs = (absolute_mode_) ? 2 : 1;
  float Ta;

  for (abs ; abs > 0; abs--)    //for correct absolute values we need to read both sub-pages
  {
    uint16_t status_reg;
    uint16_t data_ready = 0;
    uint16_t ram_addr;
    uint16_t aux_data[48];
    uint16_t control_reg;
    uint8_t sub_page = 0;
//wait for data ready - change this!!
    while (!data_ready) 
    {
      if (this->read_1616_(MLX_STATUS_REGISTER, 1, &status_reg) != 0) 
      {
        this->status_set_warning();  return; 
      }
      data_ready = status_reg & MLX_DATA_READY_MASK;
    }
//reset data ready
    sub_page = status_reg & MLX_SUBPAGE_NR_MASK;
    if (this->write_1616_(MLX_STATUS_REGISTER, MLX_STATUS_INIT_VAL) == -1) 
    {
      this->status_set_warning();  return; 
    }

    wait_time = micros();
//read subpage 0 or 1 from ram
    if (sub_page = 0) 
      ram_addr = MLX_RAM_NUL_ADDRESS;
    else 
      ram_addr = MLX_RAM_NUL_ADDRESS + MLX_COLS * 2;
    for (uint8_t j = 0; j < MLX_ROWS / 2; j++) 
    {
      if (this->read_1616_(ram_addr + MLX_RAM_ADDRESS_INC * j, MLX_COLS * 2, frame_buf + MLX_COLS * 2 * j) != 0) 
      {
        this->status_set_warning();  return; 
      }
    }
//read aux data
    if (this->read_1616_(MLX_RAM_AUX_ADDRESS, MLX_AUX_DATA_LENGTH, aux_data) != 0) 
    {
      this->status_set_warning();  return; 
    }
    if (this->read_1616_(MLX_CTRL_REGISTER_1, 1, &control_reg) != 0) 
    {
      this->status_set_warning();  return; 
    }

    read_time = micros();
//add aux data to frame
    frame_buf[MLX_FRAME_SIZE -2] = control_reg;
    frame_buf[MLX_FRAME_SIZE -1] = sub_page;
    if (ValidateAuxData(aux_data) == 0) 
    {
      for (uint8_t i = 0; i < MLX_AUX_DATA_LENGTH; i++)
        frame_buf[MLX_IMAGE_SIZE + i] = aux_data[i];
    }
    if (ValidateFrameData(frame_buf)) 
    { 
      this->status_set_warning();  return; 
    }
//calculate temperature values to image buffer
    Ta = MLX90641_GetTa(frame_buf, &MlxParams);
    float tr = Ta - ta_shift_;
    MLX90641_CalculateTo(frame_buf, &MlxParams, emmisivity_, tr, image_buf);
  }

    calc_time = micros();
//lets see
  float ambient = Ta;
  float object = image_buf[69]; //peek at pixel #69
  float center_avg = 0;
  for (int y = 5; y < 9; y++)
  {
    for(int x = 7; x < 11; x++)
    {
      center_avg += image_buf[ (y - 1) * 16 + x ]; 
    }
  }
  center_avg /= 16;

  if (this->ambient_sensor_ != nullptr && !std::isnan(ambient))
    this->ambient_sensor_->publish_state(ambient);
  if (this->object_sensor_ != nullptr && !std::isnan(object))
    this->object_sensor_->publish_state(object);
//  this->frame_sensor_->publish_state("test test");

  this->status_clear_warning();

  ESP_LOGI(TAG, " ");
  ESP_LOGD(TAG, "   ambient: %.1f°C  center_avg: %.1f°C  pixel: %.1f°C", ambient, center_avg, object);
//make some art
  ESP_LOGI(TAG, "   -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --");
  for (int y = 0 ; y < 12 ; y++) {
    char sline[50] = {0};                          
    for (int x = 0 ; x < 16 ; x++) 
    {
      float pixel = image_buf[y * 16 + x];

      if (pixel < 21) strcat( sline, " , ");
      else if (pixel < 23) strcat( sline, " $ ");
      else if (pixel < 25) strcat( sline, " Y ");
      else if (pixel < 27) strcat( sline, " + ");
      else if (pixel < 29) strcat( sline, " P ");
      else if (pixel < 31) strcat( sline, " * ");
      else if (pixel < 33) strcat( sline, " % ");
      else if (pixel < 35) strcat( sline, " D ");
      else if (pixel < 37) strcat( sline, " J ");
      else strcat( sline, " @ ");                
    }

    ESP_LOGI(TAG, "   |%s|", sline);
  }
  ESP_LOGI(TAG, "   -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --");
  ESP_LOGD(TAG, "   waiting: %.2fms  reading: %.1fms  grinding: %.2fms", (wait_time - start_time) / 1000, (read_time - wait_time) / 1000, (calc_time - read_time) / 1000);
  ESP_LOGI(TAG, " ");
}

}  // namespace mlx90641
}  // namespace esphome

