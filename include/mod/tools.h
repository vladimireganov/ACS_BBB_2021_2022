#include <rc/time.h> // for nanos
#include <inttypes.h> // for PRIu64
#include <math.h>
#include <stdio.h> //for fscanf

/**
 * @brief		finds time elapsed in seconds
 *
 * @return		time in seconds
 */
double finddt_s(uint64_t ti);