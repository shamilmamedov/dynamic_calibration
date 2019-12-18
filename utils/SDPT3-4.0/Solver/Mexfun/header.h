
#ifndef MWSIZE_MAX
    #define  mwIndex        int
    #define  mwSignedIndex  int
    #define  mwSize         int
#endif

#if !defined(_WIN32)
#define dgemv dgemv_
#endif

#define mwIndex ptrdiff_t
#define mwSize  ptrdiff_t