////////////////////////////////////////////////////////////////////////////////
//  Matrix Multiply
//  Multiply matrix A times matrix B, matrix A dimension m x n, matrix B dimension n x p
//  Result placed in matrix C, dimension m x p
//
//  Call as: matrixMultiply(m, n, p, C, A, B)
////////////////////////////////////////////////////////////////////////////////

void matrixMultiply(int aRows, int aCols_bRows, int bCols, float matrixC[], float matrixA[], float matrixB[])
{
  int i, j, k;

  for (i = 0; i < aRows * bCols; i++)
  {
    matrixC[i] = 0.0;
  }

  for (i = 0; i < aRows; i++)
  {
    for(j = 0; j < aCols_bRows; j++)
    {
      for(k = 0;  k < bCols; k++)
      {
       matrixC[i * bCols + k] += matrixA[i * aCols_bRows + j] * matrixB[j * bCols + k];
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
//  Matrix Addition
//  Add matrix A to matrix B, dimensions m x n
//  Result placed in matrix C, dimension m x n
//
//  Call as: matrixAdd(m, n, C, A, B)
////////////////////////////////////////////////////////////////////////////////

void matrixAdd(int rows, int cols, float matrixC[], float matrixA[], float matrixB[])
{
  int i;

  for (i = 0; i < rows * cols; i++)
  {
    matrixC[i] = matrixA[i] + matrixB[i];
  }
}

////////////////////////////////////////////////////////////////////////////////
//  Matrix Subtraction
//  Subtract matrix A from matrix B, dimensions m x n
//  Result placed in matrix C, dimension m x n
//
//  Call as: matrixSubtract(m, n, C, A, B)
////////////////////////////////////////////////////////////////////////////////

void matrixSubtract(int rows, int cols, float matrixC[], float matrixA[], float matrixB[])
{
  int i;

  for (i = 0; i < rows * cols; i++)
  {
    matrixC[i] = matrixA[i] - matrixB[i];
  }
}


////////////////////////////////////////////////////////////////////////////////
//  Matrix Scaling
//  Scale matrix A, dimensions m x n, by a scaler, S
//  Result placed in matrix C, dimension m x n
//
//  Call as: matrixScale(m, n, C, S, B)
////////////////////////////////////////////////////////////////////////////////

void matrixScale(int rows, int cols, float matrixC[], float scaler, float matrixA[])
{
  int i;

  for (i = 0; i < rows * cols; i++)
  {
    matrixC[i] = scaler * matrixA[i];
  }
}

////////////////////////////////////////////////////////////////////////////////
//  3 x 3 Matrix Transpose
//  Compute 3 x 3 Transpose of A
//  Result placed in matrix C, dimension 3 x 3
//
//  Call as: Transpose3x3(C, A)
////////////////////////////////////////////////////////////////////////////////

void matrixTranspose3x3(float matrixC[9], float matrixA[9])
{

  matrixC[0] = matrixA[0];
  matrixC[1] = matrixA[3];
  matrixC[2] = matrixA[6];
  matrixC[3] = matrixA[1];
  matrixC[4] = matrixA[4];
  matrixC[5] = matrixA[7];
  matrixC[6] = matrixA[2];
  matrixC[7] = matrixA[5];
  matrixC[8] = matrixA[8];
}

////////////////////////////////////////////////////////////////////////////////
//  3 x 3 Matrix Inverse
//  Compute 3 x 3 Inverse of A
//  Result placed in matrix C, dimension 3 x 3
//
//  Call as: Inverse3x3(C, A)
////////////////////////////////////////////////////////////////////////////////

void matrixInverse3x3(float matrixC[9], float matrixA[9])
{

  float det;
  float transposeA[9];
  float minors[9];
  float transposeMinors[9];

  det = matrixA[0] * (matrixA[4] * matrixA[8] - matrixA[5] * matrixA[7]) -
        matrixA[1] * (matrixA[3] * matrixA[8] - matrixA[5] * matrixA[6]) +
        matrixA[2] * (matrixA[3] * matrixA[7] - matrixA[4] * matrixA[6]);

  matrixTranspose3x3(transposeA, matrixA);

  minors[0] = matrixA[4] * matrixA[8] - matrixA[5] * matrixA[7];
  minors[1] = matrixA[5] * matrixA[6] - matrixA[3] * matrixA[8];
  minors[2] = matrixA[3] * matrixA[7] - matrixA[4] * matrixA[6];
  minors[3] = matrixA[2] * matrixA[7] - matrixA[1] * matrixA[8];
  minors[4] = matrixA[0] * matrixA[8] - matrixA[2] * matrixA[6];
  minors[5] = matrixA[1] * matrixA[6] - matrixA[0] * matrixA[7];
  minors[6] = matrixA[1] * matrixA[5] - matrixA[2] * matrixA[4];
  minors[7] = matrixA[2] * matrixA[3] - matrixA[0] * matrixA[5];
  minors[8] = matrixA[0] * matrixA[4] - matrixA[1] * matrixA[3];

  matrixTranspose3x3(transposeMinors, minors);

  det = 1/det;

  matrixScale(3,3, matrixC, det, transposeMinors);
}

