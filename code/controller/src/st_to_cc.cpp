#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <math.h>

#include <st_to_cc.hpp>

/******************************************************************************/

int i4_max(int i1, int i2)

/******************************************************************************/
/*
  Purpose:

    I4_MAX returns the maximum of two I4's.

  Licensing:

    This code is distributed under the GNU LGPL license.

  Modified:

    29 August 2006

  Author:

    John Burkardt

  Parameters:

    Input, int I1, I2, are two integers to be compared.

    Output, int I4_MAX, the larger of I1 and I2.
*/
{
  int value;

  if (i2 < i1) {
    value = i1;
  } else {
    value = i2;
  }
  return value;
}
/******************************************************************************/

int i4_min(int i1, int i2)

/******************************************************************************/
/*
  Purpose:

    I4_MIN returns the smaller of two I4's.

  Licensing:

    This code is distributed under the GNU LGPL license.

  Modified:

    29 August 2006

  Author:

    John Burkardt

  Parameters:

    Input, int I1, I2, two integers to be compared.

    Output, int I4_MIN, the smaller of I1 and I2.
*/
{
  int value;

  if (i1 < i2) {
    value = i1;
  } else {
    value = i2;
  }
  return value;
}
/******************************************************************************/

int *i4vec_copy_new(int n, int a1[])

/******************************************************************************/
/*
  Purpose:

    I4VEC_COPY_NEW copies an I4VEC.

  Discussion:

    An I4VEC is a vector of I4's.

  Licensing:

    This code is distributed under the GNU LGPL license.

  Modified:

    04 July 2008

  Author:

    John Burkardt

  Parameters:

    Input, int N, the number of entries in the vectors.

    Input, int A1[N], the vector to be copied.

    Output, int I4VEC_COPY_NEW[N], the copy of A1.
*/
{
  int *a2;
  int i;

  a2 = (int *)malloc(n * sizeof(int));

  for (i = 0; i < n; i++) {
    a2[i] = a1[i];
  }
  return a2;
}
/******************************************************************************/

void i4vec_dec(int n, int a[])

/******************************************************************************/
/*
  Purpose:

    I4VEC_DEC decrements an I4VEC.

  Discussion:

    An I4VEC is a vector of I4's.

  Licensing:

    This code is distributed under the GNU LGPL license.

  Modified:

    15 July 2014

  Author:

    John Burkardt

  Parameters:

    Input, int N, the number of entries in the vectors.

    Input/output, int A[N], the vector to be decremented.
*/
{
  int i;

  for (i = 0; i < n; i++) {
    a[i] = a[i] - 1;
  }
  return;
}
/******************************************************************************/

void i4vec_inc(int n, int a[])

/******************************************************************************/
/*
  Purpose:

    I4VEC_INC increments an I4VEC.

  Discussion:

    An I4VEC is a vector of I4's.

  Licensing:

    This code is distributed under the GNU LGPL license.

  Modified:

    15 July 2014

  Author:

    John Burkardt

  Parameters:

    Input, int N, the number of entries in the vectors.

    Input/output, int A[N], the vector to be incremented.
*/
{
  int i;

  for (i = 0; i < n; i++) {
    a[i] = a[i] + 1;
  }
  return;
}
/******************************************************************************/

int i4vec_max(int n, int a[])

/******************************************************************************/
/*
  Purpose:

    I4VEC_MAX returns the value of the maximum element in an I4VEC.

  Discussion:

    An I4VEC is a vector of I4's.

  Licensing:

    This code is distributed under the GNU LGPL license.

  Modified:

    17 May 2003

  Author:

    John Burkardt

  Parameters:

    Input, int N, the number of entries in the array.

    Input, int A[N], the array to be checked.

    Output, int IVEC_MAX, the value of the maximum element.  This
    is set to 0 if N <= 0.
*/
{
  int i;
  int value;

  if (n <= 0) {
    return 0;
  }

  value = a[0];

  for (i = 1; i < n; i++) {
    if (value < a[i]) {
      value = a[i];
    }
  }

  return value;
}
/******************************************************************************/

int i4vec_min(int n, int a[])

/******************************************************************************/
/*
  Purpose:

    I4VEC_MIN returns the minimum element in an I4VEC.

  Discussion:

    An I4VEC is a vector of I4's.

  Licensing:

    This code is distributed under the GNU LGPL license.

  Modified:

    17 May 2003

  Author:

    John Burkardt

  Parameters:

    Input, int N, the number of entries in the array.

    Input, int A[N], the array to be checked.

    Output, int I4VEC_MIN, the value of the minimum element.  This
    is set to 0 if N <= 0.
*/
{
  int i;
  int value;

  if (n <= 0) {
    return 0;
  }

  value = a[0];

  for (i = 1; i < n; i++) {
    if (a[i] < value) {
      value = a[i];
    }
  }
  return value;
}

/******************************************************************************/

int i4vec2_compare(int n, int a1[], int a2[], int i, int j)

/******************************************************************************/
/*
  Purpose:

    I4VEC2_COMPARE compares pairs of integers stored in two vectors.

  Licensing:

    This code is distributed under the GNU LGPL license.

  Modified:

    30 June 2009

  Author:

    John Burkardt

  Parameters:

    Input, int N, the number of data items.

    Input, int A1[N], A2[N], contain the two components of each item.

    Input, int I, J, the items to be compared.  These values will be
    1-based indices for the arrays A1 and A2.

    Output, int I4VEC2_COMPARE, the results of the comparison:
    -1, item I < item J,
     0, item I = item J,
    +1, item J < item I.
*/
{
  int isgn;

  isgn = 0;

  if (a1[i - 1] < a1[j - 1]) {
    isgn = -1;
  } else if (a1[i - 1] == a1[j - 1]) {
    if (a2[i - 1] < a2[j - 1]) {
      isgn = -1;
    } else if (a2[i - 1] < a2[j - 1]) {
      isgn = 0;
    } else if (a2[j - 1] < a2[i - 1]) {
      isgn = +1;
    }
  } else if (a1[j - 1] < a1[i - 1]) {
    isgn = +1;
  }

  return isgn;
}
/******************************************************************************/

void i4vec2_sort_a(int n, int a1[], int a2[])

/******************************************************************************/
/*
  Purpose:

    I4VEC2_SORT_A ascending sorts an I4VEC2.

  Discussion:

    Each item to be sorted is a pair of integers (I,J), with the I
    and J values stored in separate vectors A1 and A2.

  Licensing:

    This code is distributed under the GNU LGPL license.

  Modified:

    30 June 2009

  Author:

    John Burkardt

  Parameters:

    Input, int N, the number of items of data.

    Input/output, int A1[N], A2[N], the data to be sorted..
*/
{
  int i;
  int indx;
  int isgn;
  int j;
  int temp;
  /*
    Initialize.
  */
  i = 0;
  indx = 0;
  isgn = 0;
  j = 0;
  /*
    Call the external heap sorter.
  */
  for (;;) {
    sort_heap_external(n, &indx, &i, &j, isgn);
    /*
      Interchange the I and J objects.
    */
    if (0 < indx) {
      temp = a1[i - 1];
      a1[i - 1] = a1[j - 1];
      a1[j - 1] = temp;

      temp = a2[i - 1];
      a2[i - 1] = a2[j - 1];
      a2[j - 1] = temp;
    }
    /*
      Compare the I and J objects.
    */
    else if (indx < 0) {
      isgn = i4vec2_compare(n, a1, a2, i, j);
    } else if (indx == 0) {
      break;
    }
  }
  return;
}
/******************************************************************************/

int i4vec2_sorted_unique_count(int n, int a1[], int a2[])

/******************************************************************************/
/*
  Purpose:

    I4VEC2_SORTED_UNIQUE_COUNT counts unique elements in an I4VEC2.

  Discussion:

    Item I is stored as the pair A1(I), A2(I).

    The items must have been sorted, or at least it must be the
    case that equal items are stored in adjacent vector locations.

  Licensing:

    This code is distributed under the GNU LGPL license.

  Modified:

    12 July 2014

  Author:

    John Burkardt

  Parameters:

    Input, int N, the number of items.

    Input, int A1[N], A2[N], the array of N items.

    Output, int I4VEC2_SORTED_UNIQUE_NUM, the number of unique items.
*/
{
  int i;
  int iu;
  int unique_num;

  unique_num = 0;

  if (n <= 0) {
    return unique_num;
  }

  iu = 0;
  unique_num = 1;

  for (i = 1; i < n; i++) {
    if (a1[i] != a1[iu] || a2[i] != a2[iu]) {
      iu = i;
      unique_num = unique_num + 1;
    }
  }

  return unique_num;
}
/******************************************************************************/

void i4vec2_sorted_uniquely(int n1, int a1[], int b1[], int n2, int a2[], int b2[])

/******************************************************************************/
/*
  Purpose:

    I4VEC2_SORTED_UNIQUELY keeps the unique elements in an I4VEC2.

  Discussion:

    Item I is stored as the pair A1(I), A2(I).

    The items must have been sorted, or at least it must be the
    case that equal items are stored in adjacent vector locations.

    If the items were not sorted, then this routine will only
    replace a string of equal values by a single representative.

  Licensing:

    This code is distributed under the GNU LGPL license.

  Modified:

    14 July 2014

  Author:

    John Burkardt

  Parameters:

    Input, int N1, the number of items.

    Input, int A1[N1], B1[N1], the input array.

    Input, int N2, the number of unique items.

    Input, int A2[N2], B2[N2], the output array of unique items.
*/
{
  int i1;
  int i2;

  i1 = 0;
  i2 = 0;

  if (n1 <= 0) {
    return;
  }

  a2[i2] = a1[i1];
  b2[i2] = b1[i1];

  for (i1 = 1; i1 < n1; i1++) {
    if (a1[i1] != a2[i2] || b1[i1] != b2[i2]) {
      i2 = i2 + 1;
      a2[i2] = a1[i1];
      b2[i2] = b1[i1];
    }
  }

  return;
}


/******************************************************************************/

void sort_heap_external(int n, int *indx, int *i, int *j, int isgn)

/******************************************************************************/
/*
  Purpose:

    SORT_HEAP_EXTERNAL externally sorts a list of items into ascending order.

  Discussion:

    The actual list is not passed to the routine.  Hence it may
    consist of integers, reals, numbers, names, etc.  The user,
    after each return from the routine, will be asked to compare or
    interchange two items.

    The current version of this code mimics the FORTRAN version,
    so the values of I and J, in particular, are FORTRAN indices.

  Licensing:

    This code is distributed under the GNU LGPL license.

  Modified:

    05 February 2004

  Author:

    Original FORTRAN77 version by Albert Nijenhuis, Herbert Wilf.
    C version by John Burkardt.

  Reference:

    Albert Nijenhuis, Herbert Wilf,
    Combinatorial Algorithms,
    Academic Press, 1978, second edition,
    ISBN 0-12-519260-6.

  Parameters:

    Input, int N, the length of the input list.

    Input/output, int *INDX.
    The user must set INDX to 0 before the first call.
    On return,
      if INDX is greater than 0, the user must interchange
      items I and J and recall the routine.
      If INDX is less than 0, the user is to compare items I
      and J and return in ISGN a negative value if I is to
      precede J, and a positive value otherwise.
      If INDX is 0, the sorting is done.

    Output, int *I, *J.  On return with INDX positive,
    elements I and J of the user's list should be
    interchanged.  On return with INDX negative, elements I
    and J are to be compared by the user.

    Input, int ISGN. On return with INDX negative, the
    user should compare elements I and J of the list.  If
    item I is to precede item J, set ISGN negative,
    otherwise set ISGN positive.
*/
{
  static int i_save = 0;
  static int j_save = 0;
  static int k = 0;
  static int k1 = 0;
  static int n1 = 0;
  /*
    INDX = 0: This is the first call.
  */
  if (*indx == 0) {
    i_save = 0;
    j_save = 0;
    k = n / 2;
    k1 = k;
    n1 = n;
  }
  /*
    INDX < 0: The user is returning the results of a comparison.
  */
  else if (*indx < 0) {
    if (*indx == -2) {
      if (isgn < 0) {
        i_save = i_save + 1;
      }
      j_save = k1;
      k1 = i_save;
      *indx = -1;
      *i = i_save;
      *j = j_save;
      return;
    }

    if (0 < isgn) {
      *indx = 2;
      *i = i_save;
      *j = j_save;
      return;
    }

    if (k <= 1) {
      if (n1 == 1) {
        i_save = 0;
        j_save = 0;
        *indx = 0;
      } else {
        i_save = n1;
        j_save = 1;
        n1 = n1 - 1;
        *indx = 1;
      }
      *i = i_save;
      *j = j_save;
      return;
    }
    k = k - 1;
    k1 = k;
  }
  /*
    0 < INDX: the user was asked to make an interchange.
  */
  else if (*indx == 1) {
    k1 = k;
  }

  for (;;) {
    i_save = 2 * k1;

    if (i_save == n1) {
      j_save = k1;
      k1 = i_save;
      *indx = -1;
      *i = i_save;
      *j = j_save;
      return;
    } else if (i_save <= n1) {
      j_save = i_save + 1;
      *indx = -2;
      *i = i_save;
      *j = j_save;
      return;
    }

    if (k <= 1) {
      break;
    }

    k = k - 1;
    k1 = k;
  }

  if (n1 == 1) {
    i_save = 0;
    j_save = 0;
    *indx = 0;
    *i = i_save;
    *j = j_save;
  } else {
    i_save = n1;
    j_save = 1;
    n1 = n1 - 1;
    *indx = 1;
    *i = i_save;
    *j = j_save;
  }

  return;
}



/******************************************************************************/

int st_to_cc_size(int nst, int ist[], int jst[])

/******************************************************************************/
/*
  Purpose:

    ST_TO_CC_SIZE sizes CC indexes based on ST data.

  Licensing:

    This code is distributed under the GNU LGPL license.

  Modified:

    15 July 2014

  Author:

    John Burkardt

  Parameters:

    Input, int NST, the number of ST elements.

    Input, int IST[NST], JST[NST], the ST rows and columns.

    Output, int ST_TO_CC_SIZE, the number of CC elements.
*/
{
  int *ist2;
  int *jst2;
  int ncc;
  /*
    Make copies so the sorting doesn't confuse the user.
  */
  ist2 = i4vec_copy_new(nst, ist);
  jst2 = i4vec_copy_new(nst, jst);
  /*
    Sort by column first, then row.
  */
  i4vec2_sort_a(nst, jst2, ist2);
  /*
    Count the unique pairs.
  */
  ncc = i4vec2_sorted_unique_count(nst, jst2, ist2);

  free(ist2);
  free(jst2);

  return ncc;
}
/******************************************************************************/

void st_to_cc_index(int nst, int ist[], int jst[], int ncc, int n, int icc[], int ccc[])

/******************************************************************************/
/*
  Purpose:

    ST_TO_CC_INDEX creates CC indices from ST data.

  Licensing:

    This code is distributed under the GNU LGPL license.

  Modified:

    15 July 2014

  Author:

    John Burkardt

  Parameters:

    Input, int NST, the number of ST elements.

    Input, int IST[NST], JST[NST], the ST rows and columns.

    Input, int NCC, the number of CC elements.

    Input, int N, the number of columns in the matrix.

    Output, int ICC[NCC], the CC rows.

    Output, int CCC[N+1], the compressed CC columns.
*/
{
  int *ist2;
  int j;
  int *jcc;
  int jhi;
  int jlo;
  int *jst2;
  int k;
  /*
    Make copies so the sorting doesn't confuse the user.
  */
  ist2 = i4vec_copy_new(nst, ist);
  jst2 = i4vec_copy_new(nst, jst);
  /*
    Sort the elements.
  */
  i4vec2_sort_a(nst, jst2, ist2);
  /*
    Get the unique elements.
  */
  jcc = (int *)malloc(ncc * sizeof(int));
  i4vec2_sorted_uniquely(nst, jst2, ist2, ncc, jcc, icc);
  /*
    Compress the column index.
  */
  ccc[0] = 0;
  jlo = 0;
  for (k = 0; k < ncc; k++) {
    jhi = jcc[k];
    if (jhi != jlo) {
      for (j = jlo + 1; j <= jhi; j++) {
        ccc[j] = k;
      }
      jlo = jhi;
    }
  }
  jhi = n;
  for (j = jlo + 1; j <= jhi; j++) {
    ccc[j] = ncc;
  }

  free(ist2);
  free(jcc);
  free(jst2);

  return;
}
/******************************************************************************/

double *st_to_cc_values(int nst, int ist[], int jst[], double ast[], int ncc, int n, int icc[], int ccc[])

/******************************************************************************/
/*
  Purpose:

    ST_TO_CC_VALUES creates CC values from ST data.

  Licensing:

    This code is distributed under the GNU LGPL license.

  Modified:

    15 July 2014

  Author:

    John Burkardt

  Parameters:

    Input, int NST, the number of ST elements.

    Input, int IST[NST], JST[NST], the ST rows and columns.

    Input, double AST[NST], the ST values.

    Input, int NCC, the number of CC elements.

    Input, int N, the number of columns.

    Input, int ICC[NCC], the CC rows.

    Input, int CCC[N+1], the CC compressed columns.

    Output, double ST_TO_CC_VALUES[NCC], the CC values.
*/
{
  double *acc;
  int chi;
  int clo;
  int fail;
  int i;
  int j;
  int kcc;
  int kst;

  acc = (double *)malloc(ncc * sizeof(double));

  for (i = 0; i < ncc; i++) {
    acc[i] = 0.0;
  }

  for (kst = 0; kst < nst; kst++) {
    i = ist[kst];
    j = jst[kst];

    clo = ccc[j];
    chi = ccc[j + 1];

    fail = 1;

    for (kcc = clo; kcc < chi; kcc++) {
      if (icc[kcc] == i) {
        acc[kcc] = acc[kcc] + ast[kst];
        fail = 0;
        break;
      }
    }

    if (fail) {
      fprintf(stderr, "\n");
      fprintf(stderr, "ST_TO_CC_VALUES - Fatal error!\n");
      fprintf(stderr, "  ST entry cannot be located in CC array.\n");
      fprintf(stderr, "  ST index KST    = %d\n", kst);
      fprintf(stderr, "  ST row IST(KST) = %d\n", ist[kst]);
      fprintf(stderr, "  ST col JST(KST) = %d\n", jst[kst]);
      fprintf(stderr, "  ST val AST(KST) = %g\n", ast[kst]);
      exit(1);
    }
  }

  return acc;
}

