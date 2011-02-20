////////////////////////////////////////////////////////////////////////////////
//  Vector Dot Product
//  Return the Dot product of vectors a and b with length m
//
//  Call as: vectorDotProduct(m, a, b)
////////////////////////////////////////////////////////////////////////////////

float vectorDotProduct(int length, float vector1[], float vector2[])
{
  float dotProduct = 0;
  int   i;

  for (i = 0; i < length; i++)
  {
  dotProduct += vector1[i] * vector2[i];
  }

  return dotProduct;
}

////////////////////////////////////////////////////////////////////////////////
//  Vector Cross Product
//  Compute the cross product of vectors a and b with length 3
//  Place result in vector C
//
//  Call as: vectorDotProduct(c, a, b)
////////////////////////////////////////////////////////////////////////////////

void vectorCrossProduct(float vectorC[3], float vectorA[3], float vectorB[3])
{
  vectorC[0] = (vectorA[1] * vectorB[2]) - (vectorA[2] * vectorB[1]);
  vectorC[1] = (vectorA[2] * vectorB[0]) - (vectorA[0] * vectorB[2]);
  vectorC[2] = (vectorA[0] * vectorB[1]) - (vectorA[1] * vectorB[0]);
}

////////////////////////////////////////////////////////////////////////////////
//  Multiply a vector by a scalar
//  Mulitply vector a with length m by a scalar
//  Place result in vector b
//
//  Call as: vectorScale(m, b, a, scalar)
////////////////////////////////////////////////////////////////////////////////

void vectorScale(int length, float scaledVector[], float inputVector[], float scalar)
{
  int i;

  for (i = 0; i < length; i++)
  {
   scaledVector[i] = inputVector[i] * scalar;
  }
}

////////////////////////////////////////////////////////////////////////////////
//  Compute sum of 2 vectors
//  Add vector a to vector b, both of length m
//  Place result in vector c
//
//  Call as: vectorAdd(m, c, b, a)
////////////////////////////////////////////////////////////////////////////////

void vectorAdd(int length, float vectorC[], float vectorA[], float vectorB[])
{
  int i;

  for(i = 0; i < length; i++)
  {
     vectorC[i] = vectorA[i] + vectorB[i];
  }
}

////////////////////////////////////////////////////////////////////////////////
//  Compute difference of 2 vectors
//  Subtract vector a from vector b, both of length m
//  Place result in vector c
//
//  Call as: vectorSubtract(m, c, b, a)
////////////////////////////////////////////////////////////////////////////////

void vectorSubtract(int length, float vectorC[], float vectorA[], float vectorB[])
{
  int i;

  for(i = 0; i < length; i++)
  {
     vectorC[i] = vectorA[i] - vectorB[i];
  }
}


