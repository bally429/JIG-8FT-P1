#ifndef __QUEUE_1_0_H__
#define __QUEUE_1_0_H__


// Size must be 8*N, N >= 1
#define QUEUE_CREATE(Name,Size) \
unsigned char Name##_Mem[Size] = {0}; \
unsigned int  Name##_Front=0;\
unsigned int  Name##_Rear=0;\
const unsigned int Name##_Max = Size


#define QUEUE_REFERENCE(Name,Size)\
extern unsigned char Name##_Mem[Size]; \
extern unsigned int  Name##_Front;\
extern unsigned int  Name##_Rear;\
extern const unsigned int Name##_Max


#define QUEUE_U64_CREATE(Name,Size) \
uint64_t  Name##_Mem[Size] = {0}; \
unsigned int  Name##_Front=0;\
unsigned int  Name##_Rear=0;\
const unsigned int Name##_Max = Size


#define QUEUE_U64_REFERENCE(Name,Size)\
extern uint64_t Name##_Mem[Size]; \
extern unsigned int  Name##_Front;\
extern unsigned int  Name##_Rear;\
extern const unsigned int Name##_Max


#define QUEUE_EMPTY(Name) \
((Name##_Front == Name##_Rear) ? 1 : 0)

#define QUEUE_FULL(Name) \
((Name##_Front == ((Name##_Rear + 1)%(Name##_Max - 1))) ? 1 : 0)
	
#define QUEUE_CLEAR(Name)\
Name##_Front = Name##_Rear



#define QUEUE_PULL(Name)\
Name##_Mem[Name##_Front];\
Name##_Front = (Name##_Front + 1)%(Name##_Max - 1)


#define QUEUE_PUT(Name,Data)\
Name##_Mem[Name##_Rear] = Data;\
Name##_Rear = (Name##_Rear + 1)%(Name##_Max - 1)


#define QUEUE_ALARM(Name,Keep)\
((Name##_Front == ((Name##_Rear + Keep)%(Name##_Max - 1))) ? 1 : 0)

#endif
