#include <stdio.h>
#include <string.h>

int myFunction(int num)
{
    if (num == 0)
 
        // if number is 0, do not perform any operation.
        return 0;
    else
        // if number is power of 2, return 1 else return 0
          return ((num & (num - 1)) == 0 ? 1 : 0) ;
 
}

void cfun(const int *indatav, size_t size, int *outdatav) 
{
    size_t i;
    for (i = 0; i < size; ++i)
        outdatav[i] = indatav[i] * 2;
}

void recolor(int *in, size_t size, int *out) //RGB
{
	int v1,v2,v3;
	for(int i=1; i<479; i++)
	{
		v1 = 640*3*i;
		for(int j=1; j<639; j++)
		{
			v2 = 3*j;
			for(int k=0; k<3; k++)
			{
				out[v1+v2+k] = in[v1+v2+k]*(-8)+in[v1-640*3+v2-3+k]+in[v1- 640*3+v2+k]+in[v1- 640*3+v2+3+k]+in[v1+v2-3+k]+in[v1+v2+3+k]+in[v1+640*3+v2-3+k]+in[v1+640*3+v2+k]+in[v1+640*3+v2+3+k];
				if(out[v1+v2+k]>48)
				{
					break;
				}
			}
			if(out[v1+v2]>48 || out[v1+v2+1]>48 || out[v1+v2+2]>48)
			{
				//out[v1+v2]=255;
				//out[v1+v2+1]=255;
				//out[v1+v2+2]=255;
				//out[v1+v2-3]=255;
				//out[v1+v2-2]=255;
				//out[v1+v2-1]=255;
				//out[v1+v2-640*3]=255;
				//out[v1+v2-640*3+1]=255;
				//out[v1+v2-640*3+2]=255;
			}
			else
			{
				out[v1+v2]=0;
				out[v1+v2+1]=0;
				out[v1+v2+2]=0;
			}
		}
	}
	memcpy(in,out,size);
	for(int i=1; i<479; i++)
	{
		v1 = 640*3*i;
		for(int j=1; j<639; j++)
		{
			v2 = 3*j;
			if(out[v1+v2]>48)
			{
			for(int k=0; k<3; k++)
			{
				out[v1+v2+k] = in[v1+v2+k]+in[v1-640*3+v2-3+k]+in[v1- 640*3+v2+k]+in[v1- 640*3+v2+3+k]+in[v1+v2-3+k]+in[v1+v2+3+k]+in[v1+640*3+v2-3+k]+in[v1+640*3+v2+k]+in[v1+640*3+v2+3+k];
				if(out[v1+v2+k]>1024)
				{
					break;
				}
			}
			if(out[v1+v2]>1024 || out[v1+v2+1]>1024 || out[v1+v2+2]>1024)
			{
				out[v1+v2]=255;
				out[v1+v2+1]=255;
				out[v1+v2+2]=255;
				//out[v1+v2-3]=255;
				//out[v1+v2-2]=255;
				//out[v1+v2-1]=255;
				//out[v1+v2-640*3]=255;
				//out[v1+v2-640*3+1]=255;
				//out[v1+v2-640*3+2]=255;
			}
			else
			{
				out[v1+v2]=0;
				out[v1+v2+1]=0;
				out[v1+v2+2]=0;
			}
			}
		}
	}
	
}

void recolorDummy(int *in, size_t size, int *out) //RGB
{
	for(int i=0; i<size; i+=3)
	{
			if(in[i+0]>100 && in[i+1]>100 && in[i+2]>70) //White
			{
				out[i] = 255;
				out[i+1] = 255;
				out[i+2] = 255;
			}
			else if(in[i+1]>50 && in[i+1]>in[i+0] && in[i+1]>in[i+2]) //Green
			{
				out[i] = 0;
				out[i+1] = 255;
				out[i+2] = 0;
			}
			else if(in[i+0]<=80 && in[i+1]<=80 && in[i+2]<=80) //Black
			{
				out[i] = 0;
				out[i+1] = 0;
				out[i+2] = 0;
			}
			else if(in[i+0]>in[i+1] && in[i+2]>in[i+1]) //Magenta
			{
				out[i] = 255;
				out[i+1] = 0;
				out[i+2] = 255;
			}	
			else if(in[i+2]>in[i+0]) //Orange
			{
				out[i+0] = 0;
				out[i+1] = 165;
				out[i+2] = 255;
			}
			else if(in[i+0]>in[i+2] && in[i+0]>in[i+1] && in[i+2]<20) //Blue 
			{
				out[i] = 255;
				out[i+1] = 0;
				out[i+2] = 0;
			}	
			else if(in[i+0]>80 && in[i+1]>80 && in[i+2]>80)
			{
				out[i] = 255;
				out[i+1] = 255;
				out[i+2] = 255;
			}	
	}
}
