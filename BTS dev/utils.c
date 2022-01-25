#include <stdio.h>
#include <string.h>

#define WIDTH 640
#define HEIGHT 480
#define STRUCT_SIZE 4
#define SEQUENCE_SIZE 5
#define MIN_VALID_SEQUENCE 3
const char sequence[] = { 4,5,6,4 }; 


/*
	ord("g"): Color.GREEN, 1
    ord("m"): Color.MAGENTA, 2
    ord("b"): Color.BLUE, 3
    ord("f"): Color.ORANGE, 4
    ord("w"): Color.WHITE, 5
    ord("d"): Color.BLACK, 6
    ord("o"): Color.OTHER, 
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
					cnt++;
				}
			}
			
			if(cnt >= MIN_VALID_SEQUENCE)
			{
				//printf("%cnt++  \n",cur_indx);
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

int isObstacle(unsigned char *in, size_t in_size)
{	
	int cnt = 0;
	for(int i=0; i<200; i+=2) //0,1,4,5 -> 2,3,4
	{		
		if(in[(480-20-i/2)*WIDTH+(320-(200-i))]!=1 && in[(480-20-i/2)*WIDTH+(320-(200-i))]!=0 && in[(480-20-i/2)*WIDTH+(320-(200-i))]!=4 && in[(480-20-i/2)*WIDTH+(320-(200-i))]!=5)
		{
			cnt = 0;
			if(in[(480-20-i/2+2)*WIDTH+(320-(200-i+2))]!=1 && in[(480-20-i/2+2)*WIDTH+(320-(200-i+2))]!=0 && in[(480-20-i/2+2)*WIDTH+(320-(200-i+2))]!=4 && in[(480-20-i/2+2)*WIDTH+(320-(200-i+2))]!=5)
				cnt++;
			if(in[(480-20-i/2-2)*WIDTH+(320-(200-i+2))]!=1 && in[(480-20-i/2-2)*WIDTH+(320-(200-i+2))]!=0 && in[(480-20-i/2-2)*WIDTH+(320-(200-i+2))]!=4 && in[(480-20-i/2-2)*WIDTH+(320-(200-i+2))]!=5)
				cnt++;
			if(in[(480-20-i/2+2)*WIDTH+(320-(200-i-2))]!=1 && in[(480-20-i/2+2)*WIDTH+(320-(200-i-2))]!=0 && in[(480-20-i/2+2)*WIDTH+(320-(200-i-2))]!=4 && in[(480-20-i/2+2)*WIDTH+(320-(200-i-2))]!=5)
				cnt++;
			if(in[(480-20-i/2-2)*WIDTH+(320-(200-i-2))]!=1 && in[(480-20-i/2-2)*WIDTH+(320-(200-i-2))]!=0 && in[(480-20-i/2-2)*WIDTH+(320-(200-i-2))]!=4 && in[(480-20-i/2-2)*WIDTH+(320-(200-i-2))]!=5)
				cnt++;
			if(cnt>2)
			{
				//printf("%d, %d \n",(480-20-i/2), (320-(200-i)));
				//printf("%d\n",in[(480-20-i/2)*WIDTH+(320-(200-i))]);
				//printf("%d\n",in[(480-20-i/2+20)*WIDTH+(320-(200-i))]);
				//printf("%d\n",in[(480-20-i/2)*WIDTH+(320-(200-i))]);
				return -1;
			}
		}	
	}
	for(int i=0; i<200; i+=2)
	{		
		if(in[(480-20-i/2)*WIDTH+(320+(200-i))] !=1 && in[(480-20-i/2)*WIDTH+(320+(200-i))]!=0 && in[(480-20-i/2+2)*WIDTH+(320+(200-i))]!=4 && in[(480-20-i/2+2)*WIDTH+(320+(200-i))]!=5)
		{
			cnt = 0;
			if(in[(480-20-i/2+2)*WIDTH+(320+(200-i+2))]!=1 && in[(480-20-i/2+2)*WIDTH+(320+(200-i+2))]!=0 && in[(480-20-i/2+2)*WIDTH+(320+(200-i+2))]!=4 && in[(480-20-i/2+2)*WIDTH+(320+(200-i+2))]!=5)
				cnt++;
			if(in[(480-20-i/2-2)*WIDTH+(320+(200-i+2))]!=1 && in[(480-20-i/2-2)*WIDTH+(320+(200-i+2))]!=0 && in[(480-20-i/2-2)*WIDTH+(320+(200-i+2))]!=4 && in[(480-20-i/2-2)*WIDTH+(320+(200-i+2))]!=5)
				cnt++;
			if(in[(480-20-i/2+2)*WIDTH+(320+(200-i-2))]!=1 && in[(480-20-i/2+2)*WIDTH+(320+(200-i-2))]!=0 && in[(480-20-i/2+2)*WIDTH+(320+(200-i-2))]!=4 && in[(480-20-i/2+2)*WIDTH+(320+(200-i-2))]!=5)
				cnt++;
			if(in[(480-20-i/2-2)*WIDTH+(320+(200-i-2))]!=1 && in[(480-20-i/2-2)*WIDTH+(320+(200-i-2))]!=0 && in[(480-20-i/2-2)*WIDTH+(320+(200-i-2))]!=4 && in[(480-20-i/2-2)*WIDTH+(320+(200-i-2))]!=5)
				cnt++;
			if(cnt>2)
			{
				//printf("%d\n",in[(480-20-i/2)*WIDTH+(320-(200-i))]);
				return 1;
			}
		}
	}
	return 0;
	//printf("\n");
	
}
