#include "lin_sys5_solver.h"
#include <stdint.h>
#include <stdlib.h>

uint8_t lin_sys5_solve(int64_t _matrix5[5][5], int64_t* _known_terms, float* _solutions)
{
    uint8_t retVal = 0;
    uint8_t ii,jj,cnt;

    float matrix5_f[5][5];
    float known_terms_f[5];
    uint8_t indexes[5] = {0,1,2,3,4};

    //check non NULL pointers
    if(NULL != _matrix5 && NULL != _known_terms && NULL != _solutions)
    {
        for(ii = 0; ii < 5; ii++)
        {
            for(jj = 0; jj < 5; jj++)
            {
                matrix5_f[ii][jj] = (float)(_matrix5[ii][jj]);
            }
            known_terms_f[ii] = (float)(_known_terms[ii]);
        }

        //check known terms to avoid null solution
        cnt = 0;
        for(ii = 0; ii < 5; ii++)
        {
            if(0 != _known_terms[ii]) {cnt++;}
        }

        if(cnt != 0)
        {
            sort_rows(matrix5_f, known_terms_f, indexes);
            gauss_reduction(matrix5_f, known_terms_f, indexes);
            retVal = back_substitution(matrix5_f, known_terms_f, _solutions, indexes);
        }
    }

    return retVal;
}

void sort_rows(float _matrix5[5][5], float* _known_terms, uint8_t* _indexes)
{
    uint8_t ii, jj;
    uint8_t index_max;
    
    
    for(ii = 0; ii < 5; ii++)
    {
        index_max = ii;
        for(jj = ii; jj < 5; jj++)
        {
            if(ABS(_matrix5[jj][ii]) > ABS(_matrix5[index_max][ii]))
            {
                index_max = jj;
            }
        }

        //swap row ii with row index_max
        swap_rows(_matrix5[ii], _matrix5[index_max], &_known_terms[ii], &_known_terms[index_max], &_indexes[ii], &_indexes[index_max]);
    }

    return;
}

void swap_rows(float* _m_row1, float* _m_row2, float* _known_term1, float* _known_term2, uint8_t* _index1, uint8_t* _index2)
{
    //we assume that rows are 5 values long
    const uint8_t N = 5;
    uint8_t ii;
    float tmp;

    for(ii = 0; ii < N; ii++)
    {
        tmp = _m_row1[ii];
        _m_row1[ii] = _m_row2[ii];
        _m_row2[ii] = tmp;
    }

    tmp = *_known_term1;
    *_known_term1 = *_known_term2;
    *_known_term2 = tmp;

    ii = *_index1;
    *_index1 = *_index2;
    *_index2 = ii;

    return;
}

void gauss_reduction(float _matrix5[5][5], float* _known_terms, uint8_t* _indexes)
{
    uint8_t ii,jj;
    uint8_t kk;
    float coeff;

    for(jj = 0; jj < 4; jj++)
    {
        for(ii = jj + 1; ii < 5; ii++)
        {
            //calculate coefficient
            coeff = _matrix5[ii][jj] / _matrix5[jj][jj];

            for(kk = 0; kk < 5; kk++)
            {
                _matrix5[ii][kk] -= coeff * _matrix5[jj][kk];
            }

            _known_terms[ii] -= coeff * _known_terms[jj];
        }
    }
    return;
}

uint8_t back_substitution(float _matrix5[5][5], float* _known_terms, float* _solutions, uint8_t* _indexes)
{
    int8_t ii,jj;
    uint8_t retVal = 1;

    for(ii = 5-1; ii >= 0; ii--)
    {
        _solutions[ii] = _known_terms[ii];
        for(jj = 5-1; jj > ii; jj--)
        {
            _solutions[ii] -= _solutions[jj] * _matrix5[ii][jj];
        }

        if(0.0f != _matrix5[ii][ii])
        {
            _solutions[ii] /= _matrix5[ii][ii];
        }
        else 
        {
            //matrice non invertibile
            retVal = 0;
        }
    }
    return retVal;
}