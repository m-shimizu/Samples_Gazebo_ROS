/********************************************** Math and Unix and Graphics **
* /  This header is for sources of Math and Unix and Graphics Library.    / *
 * /              Created by Masaru SHIMIZU (9/26/1997).                 / *
  * /              Edited by Masaru SHIMIZU (3/6/2011).                 / *
   **********************************************************************/ 

#ifndef M_DF
#define	M_DF

typedef unsigned int	word;
typedef	unsigned char	byte;
typedef	unsigned short	sword;
typedef	unsigned long	lword;
typedef	sword			u_short;
typedef	lword			longword;
#if !defined( _WINDOWS )
//typedef	sword			BYTE; // WORD?
#if !defined(BYTE)
typedef	byte			BYTE;
#endif
#if !defined(WORD)
typedef	sword			WORD;
#endif
#if !defined(DWORD)
typedef	lword			DWORD;
#endif
#endif

#ifndef FILENAME_MAX
#define	FILENAME_MAX	256
#endif
//#define	INT_MAX		0x7FFFFFFF
#define	M_VERYSMALL	(1.0e-12)
#define	NEARYZERO(X)	((X)<M_VERYSMALL&&(X)>-M_VERYSMALL)
#define	forever		for(;;)
//#define	_ABSOF(X)	(((X)<(typeof(X))0)?(-(X)):(X))
#define	_ABSOF(X)	(((X)<0)?(-(X)):(X))
#define	_MAX(X,Y)	(((X)>(Y))?(X):(Y))
#define	_MIN(X,Y)	(((X)<(Y))?(X):(Y))
#define	_MAXEQU(X,Y)	(X = _MAX(X,Y))
#define	_MINEQU(X,Y)	(X = _MIN(X,Y))
#define	mul_self(S)	((S)*(S))
#define	numof(A)	(sizeof(A)/sizeof(A[0]))
#define	__PL		printf("%s : Line %d\n", __FILE__, __LINE__);
#define	__PL2		SCI1_PRINTF("%s : Line %d\n", __FILE__, __LINE__);
#define	MAXSTRCPY(D,S)	strncpy(D,S,sizeof(D)-1)
#define	MAXSTRCAT(D,S)	strncat(D,S,sizeof(D)-strlen(D)-1)
#define	STRNCPY(D,S,N)	memcpy(D,S,N);D[N]=0

enum
{
	_GOOD,
	_BAD,
	_FALSE
};

#ifdef shram
#define	W_RAM_DIV_ROM	2
#else
#define	W_RAM_DIV_ROM	1
#endif

#ifdef Sample_____
#define	TRACE(X)	SCI1_PRINTF(X)
#define	TRACE2(X,Y)	SCI1_PRINTF(X,Y)
#else
#define	TRACE(X)	
#define	TRACE2(X,Y)	
#endif

#endif
