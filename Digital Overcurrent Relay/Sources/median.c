/*! @file median.c
 *
 *  @brief Median filter.
 *
 *  This contains the functions for performing a median filter on byte-sized data.
 *
 *  @author Phong 10692820 & Emily 12016681
 *  @date 2017-05-17
 */

/*!
 *  @addtogroup median_module Median module documentation
 *  @{
 */

#include "median.h"

uint8_t Median_Filter3(const uint8_t n1, const uint8_t n2, const uint8_t n3)
{
  if (n1 > n2)
    {
      if (n2 > n3)
	return n2;	// n1>n2,n2>n3	n1>n2>n3
      else
	{
	  if (n1 > n3)
	    return n3;	// n1>n2,n3>n2,n1>n3	n1>n3>n2
	  else
	    return n1;	// n1>n2,n3>n2,n3>n1	n3>n2>n1
	}
    }
  else
    {
      if (n3 > n2)
	return n2;	// n2>n1,n3>n2	n3>n2>n1
      else
	{
	  if (n1 > n3)
	    return n1;	// n2>n1,n2>n3,n1>n3	n2>n1>n3
	  else
	    return n3;	// n2>n1,n2>n3,n3>n1	n2>n3>n1
	}
    }
}

/*!
 * @}
*/
