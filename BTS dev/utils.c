#include <stdio.h>
#include <string.h>

#define WIDTH 640
#define HEIGHT 480
#define STRUCT_SIZE 4
#define SEQUENCE_SIZE 5
#define MIN_VALID_SEQUENCE 3
const char sequence[] = { 5,5,5,5 };


/*
	ord("g"): Color.GREEN, 0
    ord("m"): Color.MAGENTA, 1
    ord("b"): Color.BLUE, 2
    ord("f"): Color.ORANGE, 3
    ord("w"): Color.WHITE, 4
    ord("d"): Color.BLACK, 5
    ord("o"): Color.OTHER, 6
*/

//in - image, out - balls [X,Y,size,exists]

void processBorders(unsigned char *in, size_t in_size, unsigned int *out, size_t out_size)
{	
	int cur_indx = 0;
	int cnt;
	for(int i=0; i<out_size / STRUCT_SIZE; i++)
	{	
		cur_indx = 0;
		for(int j=HEIGHT; j > out[i*STRUCT_SIZE + 1] && j >SEQUENCE_SIZE && cur_indx<(int)sizeof(sequence); j--)
		{
			cnt = 0;
			for(int k=0; k<SEQUENCE_SIZE; k++)
			{
				
				if(in[(j+k)*WIDTH + out[i*STRUCT_SIZE + 0]] == sequence[cur_indx])
				{
					//printf("%cnt++  \n",cur_indx);
					cnt++;
				}
			}
			
			if(cnt >= MIN_VALID_SEQUENCE)
			{
				cur_indx++;
		    }
		}
		if(cur_indx==(int)sizeof(sequence))
		{
			out[i*STRUCT_SIZE + 3] = 0;
		}
	}
	//printf("\n");
	
}
