#ifndef LIN_SYS5_SOLVER_H
#define LIN_SYS5_SOLVER_H

#include <stdint.h>
#include "defines.h"

uint8_t lin_sys5_solve(int64_t _matrix5[5][5], int64_t* _known_terms, float* _solutions);

//helper
void    sort_rows(float _matrix5[5][5], float* _known_terms, uint8_t* _indexes);
void    swap_rows(float* _m_row1, float* _m_row2, float* _known_term1, float* _known_term2, uint8_t* _index1, uint8_t* _index2);
void    gauss_reduction(float _matrix5[5][5], float* _known_terms, uint8_t* _indexes);
uint8_t back_substitution(float _matrix5[5][5], float* _known_terms, float* _solutions, uint8_t* _indexes);
#endif //LIN_SYS5_SOLVER_H