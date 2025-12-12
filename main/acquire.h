#ifndef ACQUIRE_H
#define ACQUIRE_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize data acquisition system
 * 
 * @return 0 on success, negative on error
 */
int acquire_init(void);

/**
 * @brief Start data acquisition
 * 
 * @return 0 on success, negative on error
 */
int acquire_start(void);

/**
 * @brief Stop data acquisition
 */
void acquire_stop(void);

#ifdef __cplusplus
}
#endif

#endif // ACQUIRE_H
