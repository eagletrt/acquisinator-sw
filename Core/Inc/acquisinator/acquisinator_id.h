#ifndef ACQUISINATOR_ID_H
#define ACQUISINATOR_ID_H

#define ACQUISINATOR_ID_1  (1u) // default ID
#define ACQUISINATOR_ID_2  (2u)
#define ACQUISINATOR_ID_3  (3u)
#define ACQUISINATOR_ID_4  (4u)

//override default ID using build flag -DACQUISINATOR_ID=x
#ifndef ACQUISINATOR_ID
  #define ACQUISINATOR_ID (ACQUISINATOR_ID_1)
#endif

#if ((ACQUISINATOR_ID) < ACQUISINATOR_ID_1) || ((ACQUISINATOR_ID) > ACQUISINATOR_ID_4)
  #error "INVALID ACQUISINATOR_ID: must be 1, 2, 3 or 4"
#endif

#endif /* ACQUISINATOR_ID_H */