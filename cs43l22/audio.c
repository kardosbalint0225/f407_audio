#include "audio.h"

cs43l22_desc_t hcs43l22;

/**
  * @brief  
  * @param  
  * @retval 
  */
uint32_t audio_init(audio_opt_t *opt)
{
	audio_io_init();
	cs43l22_init(&hcs43l22);
}

/**
  * @brief  
  * @param  
  * @retval 
  */
uint32_t audio_set_format(audio_opt_t *opt, const uint32_t *format)
{
	uint32_t retv = cs43l22_set_format(&hcs43l22, *format);
	return retv;
}

/**
  * @brief  
  * @param  
  * @retval 
  */
uint32_t audio_set_rate_near(audio_opt_t *opt, uint32_t *rate)
{
	uint32_t retv = cs43l22_set_rate_near(&hcs43l22, rate);
	return retv;
}

/**
  * @brief  
  * @param  
  * @retval 
  */
uint32_t audio_set_rate(audio_opt_t *opt, uint32_t *rate)
{
	uint32_t retv = cs43l22_set_rate_near(&hcs43l22, rate);
	return retv;
}

/**
  * @brief  
  * @param  
  * @retval 
  */
uint32_t audio_set_channels(audio_opt_t *opt, const uint32_t *channels)
{
	uint32_t retv = cs43l22_set_channels(&hcs43l22, *channels);
	return retv;
}

/**
  * @brief  
  * @param  
  * @retval 
  */
uint32_t audio_set_access(audio_opt_t *opt, const audio_access_t access)
{
	
}

/**
  * @brief  
  * @param  
  * @retval 
  */
uint32_t audio_set_period_size(audio_opt_t *opt, const uint32_t period_size)
{
	
}

/**
  * @brief  
  * @param  
  * @retval 
  */
uint32_t audio_set_buffer_size(audio_opt_t *opt, const uint32_t buffer_size)
{
	
}

/**
  * @brief  
  * @param  
  * @retval 
  */
uint32_t audio_set_periods(audio_opt_t *opt, const uint32_t periods)
{
	
}

