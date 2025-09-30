/*
 * Copyright 2025 Core Devices LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "audio_definitions.h"
#include "kernel/pbl_malloc.h"
#include "system/passert.h"
#include "system/logging.h"
#include "util/misc.h"
#include "services/common/system_task.h"

//AVDD 3V3 for obelix
#define BSP_AVDD_V18_ENABLE     0
#if BSP_AVDD_V18_ENABLE
    #define SINC_GAIN           0xa0
#else
    #define SINC_GAIN           0x14D
#endif

static const AUDCODE_DAC_CLK_CONFIG_TYPE codec_dac_clk_config[9] =
{
#if ALL_CLK_USING_PLL
    {48000, 1, 1, 0, SINC_GAIN, 1,  5, 4, 2, 20, 20, 0},
    {32000, 1, 1, 1, SINC_GAIN, 1,  5, 4, 2, 20, 20, 0},
    {24000, 1, 1, 5, SINC_GAIN, 1, 10, 2, 2, 10, 10, 1},
    {16000, 1, 1, 4, SINC_GAIN, 1,  5, 4, 2, 20, 20, 0},
    {12000, 1, 1, 7, SINC_GAIN, 1, 20, 2, 1,  5,  5, 1},
    { 8000, 1, 1, 8, SINC_GAIN, 1, 10, 2, 2, 10, 10, 1},
#else
    {48000, 0, 1, 0, SINC_GAIN, 0,  5, 4, 2, 20, 20, 0},
    {32000, 0, 1, 1, SINC_GAIN, 0,  5, 4, 2, 20, 20, 0},
    {24000, 0, 1, 5, SINC_GAIN, 0, 10, 2, 2, 10, 10, 1},
    {16000, 0, 1, 4, SINC_GAIN, 0,  5, 4, 2, 20, 20, 0},
    {12000, 0, 1, 7, SINC_GAIN, 0, 20, 2, 1,  5,  5, 1},
    { 8000, 0, 1, 8, SINC_GAIN, 0, 10, 2, 2, 10, 10, 1},
#endif
    {44100, 1, 1, 0, SINC_GAIN, 1,  5, 4, 2, 20, 20, 0},
    {22050, 1, 1, 5, SINC_GAIN, 1, 10, 2, 2, 10, 10, 1},
    {11025, 1, 1, 7, SINC_GAIN, 1, 20, 2, 1,  5,  5, 1},
};
#define DAC_CLK_CONFIG_SIZE (sizeof(codec_dac_clk_config)/sizeof(AUDCODE_DAC_CLK_CONFIG_TYPE))

typedef struct pll_vco
{
    uint32_t freq;
    uint32_t vco_value;
    uint32_t target_cnt;
} pll_vco_t;

static pll_vco_t g_pll_vco_tab[2] =
{
    {48, 0, 2001},
    {44, 0, 1834},
};

static void prv_bf0_disable_pll(AudioDeviceState* state)
{
    HAL_TURN_OFF_PLL();
    state->pll_state = AUDIO_PLL_CLOSED;
}

static int prv_bf0_pll_calibration()
{
    uint32_t pll_cnt;
    uint32_t xtal_cnt;
    uint32_t fc_vco;
    uint32_t fc_vco_min;
    uint32_t fc_vco_max;
    uint32_t delta_cnt = 0;
    uint32_t delta_cnt_min = 0;
    uint32_t delta_cnt_max = 0;
    uint32_t delta_fc_vco;
    uint32_t target_cnt;

    HAL_PMU_EnableAudio(1);
    HAL_RCC_EnableModule(RCC_MOD_AUDCODEC_HP);
    HAL_RCC_EnableModule(RCC_MOD_AUDCODEC_LP);

    HAL_TURN_ON_PLL();

    // VCO freq calibration
    hwp_audcodec->PLL_CFG0 |= AUDCODEC_PLL_CFG0_OPEN;
    hwp_audcodec->PLL_CFG2 |= AUDCODEC_PLL_CFG2_EN_LF_VCIN;
    hwp_audcodec->PLL_CAL_CFG = (0    << AUDCODEC_PLL_CAL_CFG_EN_Pos) |
                                (2000 << AUDCODEC_PLL_CAL_CFG_LEN_Pos);
    for (uint8_t i = 0; i < sizeof(g_pll_vco_tab) / sizeof(g_pll_vco_tab[0]); i++)
    {
        target_cnt = g_pll_vco_tab[i].target_cnt;
        fc_vco = 16;
        delta_fc_vco = 8;
        /* setup calibration and run
        ** target pll_cnt = ceil(46MHz/48MHz*2000)+1 = 1918
        ** target difference between pll_cnt and xtal_cnt should be less than 1 */
        while (delta_fc_vco != 0)
        {
            hwp_audcodec->PLL_CFG0 &= ~AUDCODEC_PLL_CFG0_FC_VCO;
            hwp_audcodec->PLL_CFG0 |= (fc_vco << AUDCODEC_PLL_CFG0_FC_VCO_Pos);
            hwp_audcodec->PLL_CAL_CFG |= AUDCODEC_PLL_CAL_CFG_EN;
            while (!(hwp_audcodec->PLL_CAL_CFG & AUDCODEC_PLL_CAL_CFG_DONE_Msk));
            pll_cnt = (hwp_audcodec->PLL_CAL_RESULT >> AUDCODEC_PLL_CAL_RESULT_PLL_CNT_Pos);
            xtal_cnt = (hwp_audcodec->PLL_CAL_RESULT & AUDCODEC_PLL_CAL_RESULT_XTAL_CNT_Msk);
            hwp_audcodec->PLL_CAL_CFG &= ~AUDCODEC_PLL_CAL_CFG_EN;
            if (pll_cnt < target_cnt)
            {
                fc_vco = fc_vco + delta_fc_vco;
                delta_cnt = target_cnt - pll_cnt;
            }
            else if (pll_cnt > target_cnt)
            {
                fc_vco = fc_vco - delta_fc_vco;
                delta_cnt = pll_cnt - target_cnt;
            }
            delta_fc_vco = delta_fc_vco >> 1;
        }

        if (fc_vco == 0)
        {
            fc_vco_min = 0;
        }
        else
        {
            fc_vco_min = fc_vco - 1;
        }
        if (fc_vco == 31)
        {
            fc_vco_max = fc_vco;
        }
        else
        {
            fc_vco_max = fc_vco + 1;
        }

        hwp_audcodec->PLL_CFG0 &= ~AUDCODEC_PLL_CFG0_FC_VCO;
        hwp_audcodec->PLL_CFG0 |= (fc_vco_min << AUDCODEC_PLL_CFG0_FC_VCO_Pos);
        hwp_audcodec->PLL_CAL_CFG |= AUDCODEC_PLL_CAL_CFG_EN;

        while (!(hwp_audcodec->PLL_CAL_CFG & AUDCODEC_PLL_CAL_CFG_DONE_Msk));
        pll_cnt = (hwp_audcodec->PLL_CAL_RESULT >> AUDCODEC_PLL_CAL_RESULT_PLL_CNT_Pos);
        hwp_audcodec->PLL_CAL_CFG &= ~AUDCODEC_PLL_CAL_CFG_EN;
        if (pll_cnt < target_cnt)
        {
            delta_cnt_min = target_cnt - pll_cnt;
        }
        else if (pll_cnt > target_cnt)
        {
            delta_cnt_min = pll_cnt - target_cnt;
        }

        hwp_audcodec->PLL_CFG0 &= ~AUDCODEC_PLL_CFG0_FC_VCO;
        hwp_audcodec->PLL_CFG0 |= (fc_vco_max << AUDCODEC_PLL_CFG0_FC_VCO_Pos);
        hwp_audcodec->PLL_CAL_CFG |= AUDCODEC_PLL_CAL_CFG_EN;
        while (!(hwp_audcodec->PLL_CAL_CFG & AUDCODEC_PLL_CAL_CFG_DONE_Msk));
        pll_cnt = (hwp_audcodec->PLL_CAL_RESULT >> AUDCODEC_PLL_CAL_RESULT_PLL_CNT_Pos);
        hwp_audcodec->PLL_CAL_CFG &= ~AUDCODEC_PLL_CAL_CFG_EN;
        if (pll_cnt < target_cnt)
        {
            delta_cnt_max = target_cnt - pll_cnt;
        }
        else if (pll_cnt > target_cnt)
        {
            delta_cnt_max = pll_cnt - target_cnt;
        }

        if (delta_cnt_min <= delta_cnt && delta_cnt_min <= delta_cnt_max)
        {
            g_pll_vco_tab[i].vco_value = fc_vco_min;
        }
        else if (delta_cnt_max <= delta_cnt && delta_cnt_max <= delta_cnt_min)
        {
            g_pll_vco_tab[i].vco_value = fc_vco_max;
        }
        else
        {
            g_pll_vco_tab[i].vco_value = fc_vco;
        }
    }
    hwp_audcodec->PLL_CFG2 &= ~AUDCODEC_PLL_CFG2_EN_LF_VCIN;
    hwp_audcodec->PLL_CFG0 &= ~AUDCODEC_PLL_CFG0_OPEN;

    HAL_TURN_OFF_PLL();

    return 0;
}

static int prv_bf0_enable_pll(uint32_t freq, uint8_t type)
{
    uint8_t freq_type;
    uint8_t test_result = -1;
    uint8_t vco_index = 0;

    freq_type = type << 1;
    if ((freq == 44100) || (freq == 22050) || (freq == 11025))
    {
        vco_index = 1;
        freq_type += 1;
    }

    HAL_TURN_ON_PLL();

    hwp_audcodec->PLL_CFG0 &= ~AUDCODEC_PLL_CFG0_FC_VCO;
    hwp_audcodec->PLL_CFG0 |= (g_pll_vco_tab[vco_index].vco_value << AUDCODEC_PLL_CFG0_FC_VCO_Pos);

    do
    {
        test_result = updata_pll_freq(freq_type);
    }
    while (test_result != 0);

    return test_result;
}

static int prv_bf0_update_pll(uint32_t freq, uint8_t type)
{
    uint8_t freq_type;
    uint8_t test_result = -1;
    uint8_t vco_index = 0;

    freq_type = type << 1;
    if ((freq == 44100) || (freq == 22050) || (freq == 11025))
    {
        vco_index = 1;
        freq_type += 1;
    }

    hwp_audcodec->PLL_CFG0 &= ~AUDCODEC_PLL_CFG0_FC_VCO;
    hwp_audcodec->PLL_CFG0 |= (g_pll_vco_tab[vco_index].vco_value << AUDCODEC_PLL_CFG0_FC_VCO_Pos);

    do
    {
        test_result = updata_pll_freq(freq_type);
    }
    while (test_result != 0);

    return test_result;
}

static void prv_bf0_audio_pll_config(AudioDevice* audio_device, const AUDCODE_DAC_CLK_CONFIG_TYPE *dac_cfg) {
    AudioDeviceState* state = audio_device->state;
    if (dac_cfg->clk_src_sel) //pll
    {
        if (state->pll_state == AUDIO_PLL_CLOSED)
        {
            prv_bf0_enable_pll(dac_cfg->samplerate, 1);
            state->pll_state = AUDIO_PLL_ENABLE;
            state->pll_samplerate = dac_cfg->samplerate;
        }
        else
        {
            prv_bf0_update_pll(dac_cfg->samplerate, 1);
            state->pll_state = AUDIO_PLL_ENABLE;
            state->pll_samplerate = dac_cfg->samplerate;
        }
    }
    else //xtal
    {
        if (state->pll_state == AUDIO_PLL_CLOSED)
        {
            HAL_TURN_ON_PLL();
            state->pll_state = AUDIO_PLL_OPEN;
        }
    }
}

static bool prv_allocate_buffers(AudioDeviceState *state) {
    // Allocate circular buffer storage
    state->circ_buffer_storage = kernel_malloc(CIRCULAR_BUF_SIZE_BYTES);
    if (!state->circ_buffer_storage) {
        PBL_LOG(LOG_LEVEL_ERROR, "Failed to allocate circular buffer storage");
        return false;
    }

    // Initialize circular buffer with allocated storage
    circular_buffer_init(&state->circ_buffer, state->circ_buffer_storage, CIRCULAR_BUF_SIZE_BYTES);

    return true;
}

static void prv_free_buffers(AudioDeviceState *state) {
    // Free circular buffer storage
    if (state->circ_buffer_storage) {
        kernel_free(state->circ_buffer_storage);
        state->circ_buffer_storage = NULL;
    }
}

bool audec_init(AudioDevice* audio_device) {
    AudioDeviceState* state = audio_device->state;
    AUDCODEC_HandleTypeDef *haudcodec = &state->audcodec;
    haudcodec->Instance = hwp_audcodec;
    haudcodec->hdma[HAL_AUDCODEC_DAC_CH0] = malloc(sizeof(DMA_HandleTypeDef));

    PBL_ASSERT(haudcodec->hdma[HAL_AUDCODEC_DAC_CH0], "allocated mem error");
    memset(haudcodec->hdma[HAL_AUDCODEC_DAC_CH0], 0, sizeof(DMA_HandleTypeDef));

    haudcodec->hdma[HAL_AUDCODEC_DAC_CH0]->Instance = audio_device->audec_dma_channel;
    haudcodec->hdma[HAL_AUDCODEC_DAC_CH0]->Init.Request = audio_device->audec_dma_request;
    haudcodec->hdma[HAL_AUDCODEC_DAC_CH0]->Init.IrqPrio = audio_device->irq_priority;

    haudcodec->Init.en_dly_sel = 0;
    haudcodec->Init.dac_cfg.opmode = 1;
    haudcodec->Init.adc_cfg.opmode = 1;
    haudcodec->bufSize = CFG_AUDIO_PLAYBACK_PIPE_SIZE * 2;
    state->audec_queue_buf[HAL_AUDCODEC_DAC_CH0] = NULL;

    HAL_PMU_EnableAudio(1);
    HAL_RCC_EnableModule(RCC_MOD_AUDCODEC_HP);
    HAL_RCC_EnableModule(RCC_MOD_AUDCODEC_LP);
    HAL_AUDCODEC_Init(haudcodec);

    for (uint8_t i = 0; i < DAC_CLK_CONFIG_SIZE; i++)
    {
        if (audio_device->samplerate == codec_dac_clk_config[i].samplerate)
        {
            haudcodec->Init.samplerate_index = i;
            haudcodec->Init.dac_cfg.dac_clk = (AUDCODE_DAC_CLK_CONFIG_TYPE *)&codec_dac_clk_config[i];
            break;
        }
    }

    if (haudcodec->buf[HAL_AUDCODEC_DAC_CH0] == NULL)
    {
        haudcodec->buf[HAL_AUDCODEC_DAC_CH0] = kernel_malloc(haudcodec->bufSize);
        memset(haudcodec->buf[HAL_AUDCODEC_DAC_CH0], 0, haudcodec->bufSize);
        PBL_ASSERT(haudcodec->buf[HAL_AUDCODEC_DAC_CH0], "allocated mem error");

    }
    state->queue_buf[HAL_AUDCODEC_DAC_CH0] = haudcodec->buf[HAL_AUDCODEC_DAC_CH0];
    HAL_AUDCODEC_Config_TChanel(haudcodec, 0, &haudcodec->Init.dac_cfg);
    state->tx_instanc = HAL_AUDCODEC_DAC_CH0;

    prv_bf0_pll_calibration();

    return true;
}

void audec_start(AudioDevice* audio_device, AudioTransCB cb) {
    AudioDeviceState* state = audio_device->state;
    AUDCODEC_HandleTypeDef *haudcodec = &state->audcodec;
    state->trans_cb = cb;

    prv_allocate_buffers(state);

    prv_bf0_audio_pll_config(audio_device, &codec_dac_clk_config[haudcodec->Init.samplerate_index]);
    HAL_NVIC_SetPriority(audio_device->audec_dma_irq, audio_device->irq_priority, 0);
    HAL_AUDCODEC_Transmit_DMA(haudcodec, haudcodec->buf[HAL_AUDCODEC_DAC_CH0], haudcodec->bufSize, HAL_AUDCODEC_DAC_CH0);
    HAL_NVIC_EnableIRQ(audio_device->audec_dma_irq);
    state->tx_instanc = HAL_AUDCODEC_DAC_CH0;

    /* enable AUDCODEC at last*/
    __HAL_AUDCODEC_DAC_ENABLE(haudcodec);

    HAL_AUDCODEC_Config_DACPath(haudcodec, 1);
    HAL_AUDCODEC_Config_Analog_DACPath(haudcodec->Init.dac_cfg.dac_clk);
    HAL_AUDCODEC_Config_DACPath(haudcodec, 0);
}

uint32_t audec_write(AudioDevice* audio_device, void *writeBuf, uint32_t size) {
    AudioDeviceState* state = audio_device->state;
    if (state->circ_buffer_storage) {
        uint32_t free_size = circular_buffer_get_write_space_remaining(&state->circ_buffer);
        PBL_ASSERT(size<=free_size, "circ buf over");
        circular_buffer_write(&state->circ_buffer, writeBuf, size);
        return circular_buffer_get_write_space_remaining(&state->circ_buffer);
    }

    return 0;
}

void audec_set_vol(AudioDevice* audio_device, int volume) {
    AUDCODEC_HandleTypeDef *haudcodec = &audio_device->state->audcodec;
    #define MIN_VOLUME          0
    #define MAX_VOLUME          100
    if (volume > MAX_VOLUME)
        volume = MAX_VOLUME;
    if (volume < MIN_VOLUME)
        volume = MIN_VOLUME;

    if (volume == 0) {
        HAL_AUDCODEC_Mute_DACPath(haudcodec, 1);
    } else {
        HAL_AUDCODEC_Mute_DACPath(haudcodec, 0);
        //convert to HAL decoder range (-36~54)*2
        volume -= 36;
        volume *= 2;
        HAL_AUDCODEC_Config_DACPath_Volume(haudcodec, 0, volume);
        HAL_AUDCODEC_Config_DACPath_Volume(haudcodec, 1, volume);
    }
}

void audec_stop(AudioDevice* audio_device) {
    AudioDeviceState* state = audio_device->state;
    AUDCODEC_HandleTypeDef *haudcodec = &state->audcodec;

    prv_bf0_disable_pll(state);

    HAL_NVIC_DisableIRQ(audio_device->audec_dma_irq);
    HAL_AUDCODEC_DMAStop(haudcodec, HAL_AUDCODEC_DAC_CH0);
    haudcodec->channel_ref &= ~(1 << HAL_AUDCODEC_DAC_CH0);
    haudcodec->State[HAL_AUDCODEC_DAC_CH0] = HAL_AUDCODEC_STATE_READY;

    HAL_AUDCODEC_Config_DACPath(haudcodec, 1);
    HAL_AUDCODEC_Close_Analog_DACPath();
    __HAL_AUDCODEC_DAC_DISABLE(haudcodec);
    HAL_AUDCODEC_Clear_All_Channel(haudcodec, 1);

    prv_free_buffers(state);
    memset(haudcodec->buf[HAL_AUDCODEC_DAC_CH0], 0, haudcodec->bufSize);
}

void audec_dac0_dma_irq_handler(AudioDevice* audio_device)
{
    HAL_DMA_IRQHandler(audio_device->state->audcodec.hdma[0]);
}

static void prv_audio_trans_bg(void* data) {
    AudioDeviceState* state  = (AudioDeviceState*) data;
    uint32_t free_size = circular_buffer_get_write_space_remaining(&state->circ_buffer);
    state->trans_cb(&free_size);
}

static void prv_dma_request_processing(AudioDeviceState* state) {
    if (!state->circ_buffer_storage) return;

    uint32_t available_data = circular_buffer_get_read_space_remaining(&state->circ_buffer);
    uint32_t trans_size = CFG_AUDIO_PLAYBACK_PIPE_SIZE;
    if (available_data < CFG_AUDIO_PLAYBACK_PIPE_SIZE) {
        PBL_LOG(LOG_LEVEL_DEBUG, "audio data not enough remain:%" PRIu32 "", available_data);
        memset(state->queue_buf[HAL_AUDCODEC_DAC_CH0], 0, CFG_AUDIO_PLAYBACK_PIPE_SIZE);
        trans_size = available_data;
    }
    if (trans_size > 0) {
        uint16_t bytes_copied = circular_buffer_copy(&state->circ_buffer,
            state->queue_buf[HAL_AUDCODEC_DAC_CH0], trans_size);
        PBL_ASSERT(bytes_copied == trans_size, "circ buffer read err");
        circular_buffer_consume(&state->circ_buffer, bytes_copied);
    }
    uint32_t free_size = circular_buffer_get_write_space_remaining(&state->circ_buffer);
    if(state->trans_cb && free_size >= CFG_AUDIO_PLAYBACK_PIPE_SIZE) {
        bool system_task_switch_context = false;
        system_task_add_callback_from_isr(prv_audio_trans_bg, (void*)state,
            &system_task_switch_context);
    }
}

void HAL_AUDCODEC_TxCpltCallback(AUDCODEC_HandleTypeDef *haprc, int cid)
{
    AudioDeviceState* state = container_of(haprc, struct AudioState, audcodec);

    state->queue_buf[HAL_AUDCODEC_DAC_CH0] = haprc->buf[HAL_AUDCODEC_DAC_CH0] + haprc->bufSize/2;
    prv_dma_request_processing(state);
}

void HAL_AUDCODEC_TxHalfCpltCallback(AUDCODEC_HandleTypeDef *haprc, int cid)
{
    AudioDeviceState* state = container_of(haprc, struct AudioState, audcodec);

    state->queue_buf[HAL_AUDCODEC_DAC_CH0] = haprc->buf[HAL_AUDCODEC_DAC_CH0];
    prv_dma_request_processing(state);
}